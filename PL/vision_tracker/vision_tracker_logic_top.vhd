--------------------------------------------------------------------------------
-- vision_tracker_logic_top.vhd
--
-- Vision half of the PYNQ-Z2 hand-tracker PL core (IP block 1 of 2; the servo
-- half is two_axis_servo_logic_top). NOT the AXI-Lite top: all control/config
-- inputs and tracking outputs are plain ports, to be wired to the AXI-Lite
-- register layer later. Everything here is pure, vendor-IP-free,
-- synthesizable logic, ready to be packaged as one custom Vivado IP.
--
-- Pipeline (all on one 100 MHz clock):
--
--   pixel stream -> rgb_to_ycrcb -> roi_skin_segmenter -> mask_cleaner_3x3
--                -> hand_measurement -> roi_tracker_update (feeds ROI back
--                   to the segmenter)
--
-- Two flavours of error outputs:
--   * error_x / error_y            — SNAPSHOT values for the register layer
--                                    (frame-atomic, see snapshot bank below).
--   * error_x_live / error_y_live  — LIVE values, qualified by metrics_valid,
--                                    for two_axis_servo_logic_top. The servo
--                                    PD loop consumes the live strobe, never
--                                    the snapshot.
--
-- Read-side snapshot bank: all tracking outputs (cx/cy/bbox/count/current
-- ROI/errors) are latched in ONE clock at the end of each measured frame,
-- so the future AXI-Lite layer exposes a frame-consistent packet even though
-- the PS reads its registers one by one. The live signals keep feeding the
-- internal tracking loop unchanged.
--
-- Stream notes:
--   * s_pixel_ready is constant '1': this pipeline never back-pressures.
--     That is a system requirement — the axis_broadcaster upstream stalls
--     the HDMI display path too if the tracker ever stops draining.
--   * s_pixel_last  = end-of-line (AXI4-Stream video TLAST).
--     s_frame_start = start-of-frame (TUSER/SOF); optional, defaults '0'
--     (the pixel counter self-wraps, SOF just makes resync immediate).
--   * Channel mapping assumes R = bits 23:16, G = 15:8, B = 7:0. The PS
--     writes the framebuffer as BGR — if the bytes arrive B-first after the
--     axis_subset_converter, swap the slices HERE (single place to fix).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity vision_tracker_logic_top is
    generic (
        FRAME_WIDTH  : integer := 640;
        FRAME_HEIGHT : integer := 480
    );
    port (
        clk                 : in  std_logic;
        resetn              : in  std_logic;  -- synchronous, active low

        -- RGB pixel stream in (from DMA / video pipeline)
        s_pixel_valid       : in  std_logic;
        s_pixel_ready       : out std_logic;
        s_pixel_data_rgb    : in  std_logic_vector(23 downto 0);
        s_pixel_last        : in  std_logic;               -- end of line
        s_frame_start       : in  std_logic := '0';        -- start of frame (optional)

        -- Control/config from the future register layer
        track_enable        : in  std_logic;
        roi_seed_x          : in  std_logic_vector(9 downto 0);
        roi_seed_y          : in  std_logic_vector(9 downto 0);
        roi_seed_w          : in  std_logic_vector(9 downto 0);
        roi_seed_h          : in  std_logic_vector(9 downto 0);
        cr_min              : in  std_logic_vector(7 downto 0);
        cr_max              : in  std_logic_vector(7 downto 0);
        cb_min              : in  std_logic_vector(7 downto 0);
        cb_max              : in  std_logic_vector(7 downto 0);

        -- Tracking outputs to the future register layer (snapshot bank)
        cx                  : out std_logic_vector(9 downto 0);
        cy                  : out std_logic_vector(9 downto 0);
        bbox_w              : out std_logic_vector(9 downto 0);
        bbox_h              : out std_logic_vector(9 downto 0);
        skin_pixel_count    : out std_logic_vector(18 downto 0);
        current_roi_x       : out std_logic_vector(9 downto 0);
        current_roi_y       : out std_logic_vector(9 downto 0);
        current_roi_w       : out std_logic_vector(9 downto 0);
        current_roi_h       : out std_logic_vector(9 downto 0);
        error_x             : out std_logic_vector(10 downto 0);  -- signed
        error_y             : out std_logic_vector(10 downto 0);  -- signed

        -- Live PD-loop feed to two_axis_servo_logic_top (NOT snapshot)
        error_x_live        : out std_logic_vector(10 downto 0);  -- signed
        error_y_live        : out std_logic_vector(10 downto 0);  -- signed
        metrics_valid       : out std_logic
    );
end entity vision_tracker_logic_top;

architecture rtl of vision_tracker_logic_top is

    -- Converter -> segmenter
    signal conv_valid, conv_ll, conv_fs      : std_logic;
    signal conv_cr, conv_cb                  : std_logic_vector(7 downto 0);

    -- Segmenter -> cleaner
    signal seg_x, seg_y                      : std_logic_vector(9 downto 0);
    signal seg_mask, seg_valid, seg_fe       : std_logic;

    -- Cleaner -> measurement
    signal cln_x, cln_y                      : std_logic_vector(9 downto 0);
    signal cln_mask, cln_valid, cln_fe       : std_logic;

    -- Measurement outputs (live — feed the tracking loop)
    signal m_cx, m_cy, m_bw, m_bh            : std_logic_vector(9 downto 0);
    signal m_cnt                             : std_logic_vector(18 downto 0);
    signal m_ex, m_ey                        : std_logic_vector(10 downto 0);
    signal m_valid, m_report                 : std_logic;

    -- Current tracking ROI (live — fed back to the segmenter)
    signal roi_x, roi_y, roi_w, roi_h        : std_logic_vector(9 downto 0);

    -- Read-side snapshot bank (what the register layer exposes).
    -- roi_tracker_update is pipelined (ROI_LATENCY clks), so the snapshot
    -- strobe is delayed by the same amount to capture THIS frame's ROI.
    constant ROI_LATENCY                     : integer := 5;
    signal report_dly                        : std_logic_vector(ROI_LATENCY downto 1);
    signal snap_cx, snap_cy                  : std_logic_vector(9 downto 0);
    signal snap_bw, snap_bh                  : std_logic_vector(9 downto 0);
    signal snap_cnt                          : std_logic_vector(18 downto 0);
    signal snap_rx, snap_ry                  : std_logic_vector(9 downto 0);
    signal snap_rw, snap_rh                  : std_logic_vector(9 downto 0);
    signal snap_ex, snap_ey                  : std_logic_vector(10 downto 0);

begin

    -- Never back-pressure the video pipeline (see header note).
    s_pixel_ready <= '1';

    u_ycrcb : entity work.rgb_to_ycrcb
        port map (
            clk             => clk,
            resetn          => resetn,
            in_valid        => s_pixel_valid,
            in_r            => s_pixel_data_rgb(23 downto 16),
            in_g            => s_pixel_data_rgb(15 downto 8),
            in_b            => s_pixel_data_rgb(7 downto 0),
            in_line_last    => s_pixel_last,
            in_frame_start  => s_frame_start,
            out_valid       => conv_valid,
            out_y           => open,   -- luma unused: classification is chroma-only
            out_cr          => conv_cr,
            out_cb          => conv_cb,
            out_line_last   => conv_ll,
            out_frame_start => conv_fs
        );

    u_segment : entity work.roi_skin_segmenter
        generic map (
            FRAME_WIDTH  => FRAME_WIDTH,
            FRAME_HEIGHT => FRAME_HEIGHT
        )
        port map (
            clk           => clk,
            resetn        => resetn,
            pixel_valid   => conv_valid,
            pixel_cr      => conv_cr,
            pixel_cb      => conv_cb,
            line_last     => conv_ll,
            frame_start   => conv_fs,
            current_roi_x => roi_x,
            current_roi_y => roi_y,
            current_roi_w => roi_w,
            current_roi_h => roi_h,
            cr_min        => cr_min,
            cr_max        => cr_max,
            cb_min        => cb_min,
            cb_max        => cb_max,
            pixel_x_out   => seg_x,
            pixel_y_out   => seg_y,
            skin_mask     => seg_mask,
            mask_valid    => seg_valid,
            frame_end_out => seg_fe
        );

    u_cleaner : entity work.mask_cleaner_3x3
        generic map (
            FRAME_WIDTH => FRAME_WIDTH
        )
        port map (
            clk             => clk,
            resetn          => resetn,
            skin_mask_in    => seg_mask,
            pixel_x         => seg_x,
            pixel_y         => seg_y,
            mask_valid_in   => seg_valid,
            frame_end_in    => seg_fe,
            clean_skin_mask => cln_mask,
            pixel_x_out     => cln_x,
            pixel_y_out     => cln_y,
            mask_valid_out  => cln_valid,
            frame_end_out   => cln_fe
        );

    u_measure : entity work.hand_measurement
        generic map (
            FRAME_WIDTH  => FRAME_WIDTH,
            FRAME_HEIGHT => FRAME_HEIGHT
        )
        port map (
            clk              => clk,
            resetn           => resetn,
            clean_skin_mask  => cln_mask,
            pixel_x          => cln_x,
            pixel_y          => cln_y,
            valid            => cln_valid,
            frame_end        => cln_fe,
            track_enable     => track_enable,
            cx               => m_cx,
            cy               => m_cy,
            bbox_w           => m_bw,
            bbox_h           => m_bh,
            skin_pixel_count => m_cnt,
            error_x          => m_ex,
            error_y          => m_ey,
            metrics_valid    => m_valid,
            report_valid     => m_report
        );

    u_roi : entity work.roi_tracker_update
        generic map (
            FRAME_WIDTH  => FRAME_WIDTH,
            FRAME_HEIGHT => FRAME_HEIGHT
        )
        port map (
            clk              => clk,
            resetn           => resetn,
            track_enable     => track_enable,
            roi_seed_x       => roi_seed_x,
            roi_seed_y       => roi_seed_y,
            roi_seed_w       => roi_seed_w,
            roi_seed_h       => roi_seed_h,
            cx               => m_cx,
            cy               => m_cy,
            bbox_w           => m_bw,
            bbox_h           => m_bh,
            skin_pixel_count => m_cnt,
            metrics_valid    => m_valid,
            report_valid     => m_report,
            current_roi_x    => roi_x,
            current_roi_y    => roi_y,
            current_roi_w    => roi_w,
            current_roi_h    => roi_h
        );

    -- Live PD-loop feed (servo IP consumes these, qualified by metrics_valid).
    error_x_live  <= m_ex;
    error_y_live  <= m_ey;
    metrics_valid <= m_valid;

    -- ── Read-side snapshot bank ───────────────────────────────────────────
    -- Latched ROI_LATENCY cycles after report_valid: by then roi_tracker_update
    -- has pushed the ROI derived from this frame's metrics through its pipeline
    -- and committed it, so the packet pairs the geometry with the ROI it
    -- produced. The other latched fields (cx/cy/bbox/count/errors) are ready
    -- one cycle after report_valid and simply hold, so the extra delay is
    -- harmless for them. While TRACK_ENABLE = '0' the bank is transparent
    -- instead, so the PS sees the seed echoed back immediately at handoff
    -- (first validation crop = seed ROI, not stale values). All eleven values
    -- update on a single clock edge and then hold for a whole frame — a
    -- sequential PS read burst sees one frame's packet.
    snapshot : process(clk)
    begin
        if rising_edge(clk) then
            if resetn = '0' then
                report_dly <= (others => '0');
                snap_cx   <= (others => '0');
                snap_cy   <= (others => '0');
                snap_bw   <= (others => '0');
                snap_bh   <= (others => '0');
                snap_cnt  <= (others => '0');
                snap_rx   <= (others => '0');
                snap_ry   <= (others => '0');
                snap_rw   <= (others => '0');
                snap_rh   <= (others => '0');
                snap_ex   <= (others => '0');
                snap_ey   <= (others => '0');
            else
                report_dly <= report_dly(ROI_LATENCY - 1 downto 1) & m_report;
                if (track_enable = '0') or (report_dly(ROI_LATENCY) = '1') then
                    snap_cx  <= m_cx;
                    snap_cy  <= m_cy;
                    snap_bw  <= m_bw;
                    snap_bh  <= m_bh;
                    snap_cnt <= m_cnt;
                    snap_rx  <= roi_x;
                    snap_ry  <= roi_y;
                    snap_rw  <= roi_w;
                    snap_rh  <= roi_h;
                    snap_ex  <= m_ex;
                    snap_ey  <= m_ey;
                end if;
            end if;
        end if;
    end process;

    -- Register-layer outputs — snapshot values only, never the live signals.
    cx               <= snap_cx;
    cy               <= snap_cy;
    bbox_w           <= snap_bw;
    bbox_h           <= snap_bh;
    skin_pixel_count <= snap_cnt;
    current_roi_x    <= snap_rx;
    current_roi_y    <= snap_ry;
    current_roi_w    <= snap_rw;
    current_roi_h    <= snap_rh;
    error_x          <= snap_ex;
    error_y          <= snap_ey;

end architecture rtl;
