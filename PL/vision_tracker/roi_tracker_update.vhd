--------------------------------------------------------------------------------
-- roi_tracker_update.vhd
--
-- Owns the PL's internal tracking ROI. Velocity-PREDICTIVE while locked, with an
-- ADAPTIVE EXPAND-ON-LOSS search instead of a blind acquisition pad, and an
-- ADAPTIVE accept threshold that scales with the tracked object's size.
--
--   * track_enable = '0': ROI follows the PS seed; velocity history cleared.
--   * ACCEPTED frame (metrics_valid and count >= adaptive threshold): re-centre
--     on the PREDICTED next centroid (cx + smoothed velocity), size bbox +
--     2*tight speed-scaled pad, clamped to the frame. Resets the loss search.
--   * LOST frame (a frame completed, report_valid, but count below the accept
--     threshold): GROW the ROI to search — per axis by an amount that scales
--     with the last velocity magnitude and the number of consecutive lost
--     frames, larger along the last velocity, smaller the opposite way, bounded
--     by [LOSS_MIN, LOSS_MAX] per step and MAX_LOSS_W/H overall. With no
--     velocity yet the growth is symmetric and ramps up gradually.
--
-- Adaptive accept threshold (why): during the large-ROI search a fixed low
-- count threshold could re-lock onto a small distractor. Instead the threshold
-- tracks the object: min_count = max(COUNT_FLOOR, last_bbox_area >> COUNT_SHIFT)
-- so re-acquisition needs a blob comparable to the tracked object. COUNT_SHIFT
-- is deliberately generous (small fraction) so a spread/!thin pose is not
-- rejected; COUNT_FLOOR keeps small objects trackable.
--
-- Adaptive expansion (why): the forward step >= |velocity| guarantees the
-- search edge outruns a target that kept going; the lost-frame ramp widens the
-- search the longer the target is missing; the smaller reverse step still
-- covers a reversal, at lower distractor cost. All bounded by LOSS_MIN/MAX.
--
-- Velocity is smoothed (average with the previous estimate) and clamped to
-- +/-VEL_MAX, so a single noisy centroid cannot fling the ROI across the frame.
--
-- PIPELINED. A new ROI is produced at most once per frame, so the arithmetic is
-- spread across register stages with millions of cycles of slack:
--   Accept path (latency 5): S1 velocity, S2 smoothing, S3 predicted centre +
--     speed-scaled pad, S4 size (clamped), S5 top-left (clamped) -> commit.
--   Loss path   (latency 4): L1 per-axis forward step (|v| + ramp, bounded),
--     L2 per-side growth from the velocity sign (reverse = forward/2),
--     L3 grown size (capped) + raw top-left, L4 position clamp -> commit.
-- The read-side snapshot bank in vision_tracker_logic_top delays its latch by
-- ROI_LATENCY (>= max path latency) so the packet captures the frame's ROI.
--
-- No LOST decision here — the PL only predicts, searches or holds; declaring the
-- target permanently lost and re-seeding is the PS validator's job.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity roi_tracker_update is
    generic (
        FRAME_WIDTH  : integer := 640;
        FRAME_HEIGHT : integer := 480;
        -- ponytail: PL-internal by design (the PS deliberately cannot write
        -- them — see the register-map contract). Retune here if needed.
        PAD_BASE     : integer := 24;   -- base pixels added on each side of the bbox
        PAD_MAX      : integer := 96;   -- cap on the (base + speed) per-side padding
        VEL_MAX      : integer := 120;  -- clamp on the per-frame velocity estimate
        -- Adaptive accept threshold: min_count = max(COUNT_FLOOR, area>>SHIFT).
        COUNT_FLOOR  : integer := 150;  -- floor (~12x12 blob), also the cold-start value
        COUNT_SHIFT  : integer := 3;    -- area fraction = 1/2**SHIFT (3 -> 1/8 fill)
        -- Adaptive expand-on-loss search.
        LOSS_BASE    : integer := 24;   -- base growth added to |velocity|
        LOSS_RAMP    : integer := 24;   -- extra growth per consecutive lost frame
        LOSS_MIN     : integer := 16;   -- min per-side step
        LOSS_MAX     : integer := 200;  -- max per-side step
        MAX_LOSS_W   : integer := 520;  -- max searched ROI width  (<= FRAME_WIDTH)
        MAX_LOSS_H   : integer := 400   -- max searched ROI height (<= FRAME_HEIGHT)
    );
    port (
        clk              : in  std_logic;
        resetn           : in  std_logic;  -- synchronous, active low

        track_enable     : in  std_logic;

        roi_seed_x       : in  std_logic_vector(9 downto 0);
        roi_seed_y       : in  std_logic_vector(9 downto 0);
        roi_seed_w       : in  std_logic_vector(9 downto 0);
        roi_seed_h       : in  std_logic_vector(9 downto 0);

        cx               : in  std_logic_vector(9 downto 0);
        cy               : in  std_logic_vector(9 downto 0);
        bbox_w           : in  std_logic_vector(9 downto 0);
        bbox_h           : in  std_logic_vector(9 downto 0);
        skin_pixel_count : in  std_logic_vector(18 downto 0);
        metrics_valid    : in  std_logic;
        report_valid     : in  std_logic;  -- 1-clk pulse per completed frame (incl. count=0)

        current_roi_x    : out std_logic_vector(9 downto 0);
        current_roi_y    : out std_logic_vector(9 downto 0);
        current_roi_w    : out std_logic_vector(9 downto 0);
        current_roi_h    : out std_logic_vector(9 downto 0)
    );
end entity roi_tracker_update;

architecture rtl of roi_tracker_update is

    constant MC_DIV : integer := 2 ** COUNT_SHIFT;

    signal roi_x_r, roi_y_r : unsigned(9 downto 0);
    signal roi_w_r, roi_h_r : unsigned(9 downto 0);

    -- Prediction / adaptation state, carried between accepted frames.
    signal cx_prev, cy_prev : integer range 0 to 2047;
    signal vx_f, vy_f       : integer range -2048 to 2047;  -- smoothed px/frame
    signal have_prev        : std_logic;
    signal min_count_reg    : integer range 0 to 524287;    -- adaptive accept threshold
    signal grow_accum       : integer range 0 to 2047;      -- loss-ramp accumulator

    -- Accept-path pipeline registers (enable bit + payload per stage).
    signal s1_en                : std_logic;
    signal s1_cx, s1_cy         : integer range 0 to 2047;
    signal s1_bw, s1_bh         : integer range 0 to 2047;
    signal s1_vxr, s1_vyr       : integer range -2048 to 2047;

    signal s2_en                : std_logic;
    signal s2_cx, s2_cy         : integer range 0 to 2047;
    signal s2_bw, s2_bh         : integer range 0 to 2047;
    signal s2_vxn, s2_vyn       : integer range -2048 to 2047;

    signal s3_en                : std_logic;
    signal s3_cxp, s3_cyp       : integer range -4096 to 4095;
    signal s3_addx, s3_addy     : integer range 0 to 2047;
    signal s3_bw, s3_bh         : integer range 0 to 2047;

    signal s4_en                : std_logic;
    signal s4_cxp, s4_cyp       : integer range -4096 to 4095;
    signal s4_w, s4_h           : integer range 0 to 2047;

    -- Loss-path pipeline registers.
    signal l1_en                : std_logic;
    signal l1_fwx, l1_fwy       : integer range 0 to 2047;  -- forward step magnitude
    signal l1_sx, l1_sy         : integer range -1 to 1;    -- velocity sign per axis

    signal l2_en                : std_logic;
    signal l2_gl, l2_gr         : integer range 0 to 2047;  -- grow left/right (x)
    signal l2_gu, l2_gd         : integer range 0 to 2047;  -- grow up/down   (y)

    signal l3_en                : std_logic;
    signal l3_w, l3_h           : integer range 0 to 2047;
    signal l3_xraw, l3_yraw     : integer range -4096 to 4095;

begin

    process(clk)
        variable cxi, cyi   : integer range 0 to 2047;
        variable vxr, vyr   : integer range -2048 to 2047;
        variable vxn, vyn   : integer range -2048 to 2047;
        variable ax, ay     : integer range 0 to 2047;
        variable wv, hv     : integer range 0 to 2047;
        variable xv, yv     : integer range -4096 to 4095;
        variable accepted   : boolean;
        variable area       : integer range 0 to 524287;
        variable mc         : integer range 0 to 524287;
        variable ramp       : integer range 0 to 4095;
        variable fwx, fwy   : integer range 0 to 4095;
        variable rvx, rvy   : integer range 0 to 2047;
    begin
        if rising_edge(clk) then
            if resetn = '0' then
                roi_x_r       <= (others => '0');
                roi_y_r       <= (others => '0');
                roi_w_r       <= to_unsigned(FRAME_WIDTH, 10);
                roi_h_r       <= to_unsigned(FRAME_HEIGHT, 10);
                cx_prev       <= 0;
                cy_prev       <= 0;
                vx_f          <= 0;
                vy_f          <= 0;
                have_prev     <= '0';
                min_count_reg <= COUNT_FLOOR;
                grow_accum    <= 0;
                s1_en         <= '0';
                s2_en         <= '0';
                s3_en         <= '0';
                s4_en         <= '0';
                l1_en         <= '0';
                l2_en         <= '0';
                l3_en         <= '0';
            else
                -- Stage enables default low; each stage re-asserts the next.
                s1_en <= '0';
                s2_en <= '0';
                s3_en <= '0';
                s4_en <= '0';
                l1_en <= '0';
                l2_en <= '0';
                l3_en <= '0';

                if track_enable = '0' then
                    -- Follow the PS seed; full frame until a real seed is
                    -- written. Clear velocity/loss state so a new lock is fresh.
                    if (unsigned(roi_seed_w) = 0) or (unsigned(roi_seed_h) = 0) then
                        roi_x_r <= (others => '0');
                        roi_y_r <= (others => '0');
                        roi_w_r <= to_unsigned(FRAME_WIDTH, 10);
                        roi_h_r <= to_unsigned(FRAME_HEIGHT, 10);
                    else
                        roi_x_r <= unsigned(roi_seed_x);
                        roi_y_r <= unsigned(roi_seed_y);
                        roi_w_r <= unsigned(roi_seed_w);
                        roi_h_r <= unsigned(roi_seed_h);
                    end if;
                    vx_f          <= 0;
                    vy_f          <= 0;
                    have_prev     <= '0';
                    min_count_reg <= COUNT_FLOOR;
                    grow_accum    <= 0;
                else
                    accepted := (metrics_valid = '1')
                            and (to_integer(unsigned(skin_pixel_count)) >= min_count_reg);

                    -- ══ ACCEPT PATH ═══════════════════════════════════════
                    -- ── S1: clamped per-frame velocity ─────────────────────
                    if accepted then
                        cxi := to_integer(unsigned(cx));
                        cyi := to_integer(unsigned(cy));
                        if have_prev = '1' then
                            vxr := cxi - cx_prev;
                            if    vxr >  VEL_MAX then vxr :=  VEL_MAX;
                            elsif vxr < -VEL_MAX then vxr := -VEL_MAX; end if;
                            vyr := cyi - cy_prev;
                            if    vyr >  VEL_MAX then vyr :=  VEL_MAX;
                            elsif vyr < -VEL_MAX then vyr := -VEL_MAX; end if;
                        else
                            vxr := 0;
                            vyr := 0;
                        end if;

                        s1_cx  <= cxi;
                        s1_cy  <= cyi;
                        s1_bw  <= to_integer(unsigned(bbox_w));
                        s1_bh  <= to_integer(unsigned(bbox_h));
                        s1_vxr <= vxr;
                        s1_vyr <= vyr;
                        s1_en  <= '1';

                        -- Commit centroid history for the next frame.
                        cx_prev   <= cxi;
                        cy_prev   <= cyi;
                        have_prev <= '1';

                        -- Adaptive accept threshold from this frame's bbox
                        -- area, and reset the loss-search ramp (re-acquired).
                        area := to_integer(unsigned(bbox_w))
                              * to_integer(unsigned(bbox_h));
                        mc := area / MC_DIV;
                        if mc < COUNT_FLOOR then mc := COUNT_FLOOR; end if;
                        min_count_reg <= mc;
                        grow_accum    <= 0;
                    end if;

                    -- ── S2: velocity smoothing ─────────────────────────────
                    if s1_en = '1' then
                        vxn := (s1_vxr + vx_f) / 2;
                        vyn := (s1_vyr + vy_f) / 2;
                        s2_cx  <= s1_cx;
                        s2_cy  <= s1_cy;
                        s2_bw  <= s1_bw;
                        s2_bh  <= s1_bh;
                        s2_vxn <= vxn;
                        s2_vyn <= vyn;
                        s2_en  <= '1';
                        vx_f <= vxn;
                        vy_f <= vyn;
                    end if;

                    -- ── S3: predicted centre + speed-scaled pad ────────────
                    if s2_en = '1' then
                        s3_cxp <= s2_cx + s2_vxn;
                        s3_cyp <= s2_cy + s2_vyn;
                        ax := PAD_BASE + abs(s2_vxn) / 2;
                        if ax > PAD_MAX then ax := PAD_MAX; end if;
                        ay := PAD_BASE + abs(s2_vyn) / 2;
                        if ay > PAD_MAX then ay := PAD_MAX; end if;
                        s3_addx <= ax;
                        s3_addy <= ay;
                        s3_bw   <= s2_bw;
                        s3_bh   <= s2_bh;
                        s3_en   <= '1';
                    end if;

                    -- ── S4: ROI size, clamped to the frame ─────────────────
                    if s3_en = '1' then
                        wv := s3_bw + 2 * s3_addx;
                        if wv > FRAME_WIDTH then wv := FRAME_WIDTH; end if;
                        hv := s3_bh + 2 * s3_addy;
                        if hv > FRAME_HEIGHT then hv := FRAME_HEIGHT; end if;
                        s4_w   <= wv;
                        s4_h   <= hv;
                        s4_cxp <= s3_cxp;
                        s4_cyp <= s3_cyp;
                        s4_en  <= '1';
                    end if;

                    -- ── S5: top-left from predicted centre → commit ────────
                    if s4_en = '1' then
                        xv := s4_cxp - s4_w / 2;
                        if xv < 0 then xv := 0; end if;
                        if xv + s4_w > FRAME_WIDTH then xv := FRAME_WIDTH - s4_w; end if;
                        yv := s4_cyp - s4_h / 2;
                        if yv < 0 then yv := 0; end if;
                        if yv + s4_h > FRAME_HEIGHT then yv := FRAME_HEIGHT - s4_h; end if;

                        roi_x_r <= to_unsigned(xv, 10);
                        roi_y_r <= to_unsigned(yv, 10);
                        roi_w_r <= to_unsigned(s4_w, 10);
                        roi_h_r <= to_unsigned(s4_h, 10);
                    end if;

                    -- ══ LOSS PATH (adaptive search) ═══════════════════════
                    -- ── L1: per-axis forward step = |velocity| + ramp ──────
                    if (report_valid = '1') and (not accepted) and (have_prev = '1')
                       and ((to_integer(roi_w_r) < MAX_LOSS_W)
                         or (to_integer(roi_h_r) < MAX_LOSS_H)) then
                        ramp := LOSS_BASE + grow_accum;

                        fwx := abs(vx_f) + ramp;
                        if    fwx > LOSS_MAX then fwx := LOSS_MAX;
                        elsif fwx < LOSS_MIN then fwx := LOSS_MIN; end if;
                        fwy := abs(vy_f) + ramp;
                        if    fwy > LOSS_MAX then fwy := LOSS_MAX;
                        elsif fwy < LOSS_MIN then fwy := LOSS_MIN; end if;

                        l1_fwx <= fwx;
                        l1_fwy <= fwy;
                        if    vx_f > 0 then l1_sx <=  1;
                        elsif vx_f < 0 then l1_sx <= -1;
                        else                l1_sx <=  0; end if;
                        if    vy_f > 0 then l1_sy <=  1;
                        elsif vy_f < 0 then l1_sy <= -1;
                        else                l1_sy <=  0; end if;
                        l1_en <= '1';

                        -- Widen the search the longer the target is missing.
                        if grow_accum < LOSS_MAX then
                            grow_accum <= grow_accum + LOSS_RAMP;
                        end if;
                    end if;

                    -- ── L2: per-side growth (reverse = forward/2), by sign ─
                    if l1_en = '1' then
                        rvx := l1_fwx / 2;
                        if rvx < LOSS_MIN then rvx := LOSS_MIN; end if;
                        rvy := l1_fwy / 2;
                        if rvy < LOSS_MIN then rvy := LOSS_MIN; end if;

                        if    l1_sx > 0 then l2_gl <= rvx;    l2_gr <= l1_fwx;
                        elsif l1_sx < 0 then l2_gl <= l1_fwx; l2_gr <= rvx;
                        else                 l2_gl <= l1_fwx; l2_gr <= l1_fwx; end if;
                        if    l1_sy > 0 then l2_gu <= rvy;    l2_gd <= l1_fwy;
                        elsif l1_sy < 0 then l2_gu <= l1_fwy; l2_gd <= rvy;
                        else                 l2_gu <= l1_fwy; l2_gd <= l1_fwy; end if;
                        l2_en <= '1';
                    end if;

                    -- ── L3: grown size (capped) + raw top-left ─────────────
                    if l2_en = '1' then
                        wv := to_integer(roi_w_r) + l2_gl + l2_gr;
                        if wv > MAX_LOSS_W then wv := MAX_LOSS_W; end if;
                        hv := to_integer(roi_h_r) + l2_gu + l2_gd;
                        if hv > MAX_LOSS_H then hv := MAX_LOSS_H; end if;
                        l3_w    <= wv;
                        l3_h    <= hv;
                        l3_xraw <= to_integer(roi_x_r) - l2_gl;
                        l3_yraw <= to_integer(roi_y_r) - l2_gu;
                        l3_en   <= '1';
                    end if;

                    -- ── L4: clamp position into the frame → commit ─────────
                    if l3_en = '1' then
                        xv := l3_xraw;
                        if xv < 0 then xv := 0; end if;
                        if xv + l3_w > FRAME_WIDTH then xv := FRAME_WIDTH - l3_w; end if;
                        yv := l3_yraw;
                        if yv < 0 then yv := 0; end if;
                        if yv + l3_h > FRAME_HEIGHT then yv := FRAME_HEIGHT - l3_h; end if;

                        roi_x_r <= to_unsigned(xv, 10);
                        roi_w_r <= to_unsigned(l3_w, 10);
                        roi_y_r <= to_unsigned(yv, 10);
                        roi_h_r <= to_unsigned(l3_h, 10);
                    end if;
                end if;
            end if;
        end if;
    end process;

    current_roi_x <= std_logic_vector(roi_x_r);
    current_roi_y <= std_logic_vector(roi_y_r);
    current_roi_w <= std_logic_vector(roi_w_r);
    current_roi_h <= std_logic_vector(roi_h_r);

end architecture rtl;
