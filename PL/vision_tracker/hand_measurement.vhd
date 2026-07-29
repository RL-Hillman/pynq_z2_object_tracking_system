--------------------------------------------------------------------------------
-- hand_measurement.vhd
--
-- Per-frame skin-blob geometry: centroid, bounding box, skin-pixel count and
-- signed error from the frame centre. Geometry only — no LOCKED/LOST logic
-- (deciding "is this still a hand?" belongs to the PS validator).
--
-- Operation:
--   * During the frame: accumulate sum_x / sum_y / count and track
--     min/max x/y over all '1' mask pixels.
--   * One clk after frame_end (so the frame's last pixel is included), the
--     accumulators are snapshotted and cleared; the next frame accumulates
--     immediately while a sequential restoring divider computes
--     cx = sum_x / count and cy = sum_y / count in the background
--     (~60 clk, finished long before the next frame ends).
--   * When count > 0: outputs update and metrics_valid pulses for 1 clk.
--   * When count = 0: skin_pixel_count / bbox update to 0 but cx/cy/error
--     HOLD their last values and metrics_valid does NOT pulse, so the ROI
--     tracker and the servo hold instead of chasing garbage.
--   * report_valid pulses for 1 clk on EVERY frame whose outputs were
--     updated (count > 0 AND count = 0 frames). The read-side snapshot bank
--     in the top keys on it; metrics_valid remains the "valid geometry"
--     strobe for the ROI tracker and the servos.
--   * When track_enable = '0': outputs freeze entirely (the register layer
--     reports stale values while tracking is disabled, per the PS contract).
--
-- Widths: sum_x <= 639*307200 < 2**28, count <= 307200 < 2**19.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity hand_measurement is
    generic (
        FRAME_WIDTH  : integer := 640;
        FRAME_HEIGHT : integer := 480
    );
    port (
        clk              : in  std_logic;
        resetn           : in  std_logic;  -- synchronous, active low

        clean_skin_mask  : in  std_logic;
        pixel_x          : in  std_logic_vector(9 downto 0);
        pixel_y          : in  std_logic_vector(9 downto 0);
        valid            : in  std_logic;
        frame_end        : in  std_logic;  -- pulse aligned with the frame's last valid pixel
        track_enable     : in  std_logic;

        cx               : out std_logic_vector(9 downto 0);
        cy               : out std_logic_vector(9 downto 0);
        bbox_w           : out std_logic_vector(9 downto 0);
        bbox_h           : out std_logic_vector(9 downto 0);
        skin_pixel_count : out std_logic_vector(18 downto 0);
        error_x          : out std_logic_vector(10 downto 0);  -- signed: CENTER_X - cx
        error_y          : out std_logic_vector(10 downto 0);  -- signed: CENTER_Y - cy
        metrics_valid    : out std_logic;  -- 1-clk pulse per frame with count > 0
        report_valid     : out std_logic   -- 1-clk pulse per frame whose outputs updated
    );
end entity hand_measurement;

architecture rtl of hand_measurement is

    constant CENTER_X : integer := FRAME_WIDTH / 2;
    constant CENTER_Y : integer := FRAME_HEIGHT / 2;

    -- Frame accumulators.
    signal sum_x, sum_y : unsigned(27 downto 0);
    signal cnt          : unsigned(18 downto 0);
    signal min_x, min_y : unsigned(9 downto 0);
    signal max_x, max_y : unsigned(9 downto 0);

    signal frame_end_d1 : std_logic;

    -- Snapshot taken at frame end (divider works on these while the next
    -- frame accumulates).
    signal s_sum_y                       : unsigned(27 downto 0);
    signal s_cnt                         : unsigned(18 downto 0);
    signal s_min_x, s_min_y              : unsigned(9 downto 0);
    signal s_max_x, s_max_y              : unsigned(9 downto 0);

    -- Sequential restoring divider (28-bit dividend / 19-bit divisor).
    type div_state_t is (IDLE, DIV_X, SAVE_X, DIV_Y, LATCH);
    signal state   : div_state_t;
    signal num     : unsigned(27 downto 0);
    signal divisor : unsigned(18 downto 0);
    signal remain  : unsigned(19 downto 0);
    signal quo     : unsigned(27 downto 0);
    signal idx     : integer range 0 to 27;
    signal cx_val  : unsigned(9 downto 0);

begin

    process(clk)
        variable rnext : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            if resetn = '0' then
                sum_x <= (others => '0');
                sum_y <= (others => '0');
                cnt   <= (others => '0');
                min_x <= (others => '1');
                min_y <= (others => '1');
                max_x <= (others => '0');
                max_y <= (others => '0');
                frame_end_d1     <= '0';
                state            <= IDLE;
                metrics_valid    <= '0';
                report_valid     <= '0';
                cx               <= (others => '0');
                cy               <= (others => '0');
                bbox_w           <= (others => '0');
                bbox_h           <= (others => '0');
                skin_pixel_count <= (others => '0');
                error_x          <= (others => '0');
                error_y          <= (others => '0');
            else
                metrics_valid <= '0';
                report_valid  <= '0';
                frame_end_d1  <= frame_end;

                -- ── Per-pixel accumulation ─────────────────────────────────
                if (valid = '1') and (clean_skin_mask = '1') then
                    sum_x <= sum_x + resize(unsigned(pixel_x), 28);
                    sum_y <= sum_y + resize(unsigned(pixel_y), 28);
                    cnt   <= cnt + 1;
                    if unsigned(pixel_x) < min_x then min_x <= unsigned(pixel_x); end if;
                    if unsigned(pixel_x) > max_x then max_x <= unsigned(pixel_x); end if;
                    if unsigned(pixel_y) < min_y then min_y <= unsigned(pixel_y); end if;
                    if unsigned(pixel_y) > max_y then max_y <= unsigned(pixel_y); end if;
                end if;

                -- ── Frame end: snapshot + clear (1 clk after the last pixel,
                --    so the accumulators already include it) ────────────────
                if frame_end_d1 = '1' then
                    -- Clear for the next frame in all cases.
                    sum_x <= (others => '0');
                    sum_y <= (others => '0');
                    cnt   <= (others => '0');
                    min_x <= (others => '1');
                    min_y <= (others => '1');
                    max_x <= (others => '0');
                    max_y <= (others => '0');

                    if track_enable = '1' and state = IDLE then
                        if cnt = 0 then
                            -- No skin this frame: report it, hold cx/cy/error,
                            -- no metrics pulse (downstream holds position/ROI).
                            skin_pixel_count <= (others => '0');
                            bbox_w           <= (others => '0');
                            bbox_h           <= (others => '0');
                            report_valid     <= '1';
                        else
                            s_sum_y <= sum_y;
                            s_cnt   <= cnt;
                            s_min_x <= min_x;
                            s_max_x <= max_x;
                            s_min_y <= min_y;
                            s_max_y <= max_y;
                            num     <= sum_x;
                            divisor <= cnt;
                            remain  <= (others => '0');
                            quo     <= (others => '0');
                            idx     <= 27;
                            state   <= DIV_X;
                        end if;
                    end if;
                end if;

                -- ── Divider FSM (runs while the next frame accumulates) ────
                case state is

                    when DIV_X | DIV_Y =>
                        rnext := remain(18 downto 0) & num(idx);
                        if rnext >= resize(divisor, 20) then
                            remain   <= rnext - resize(divisor, 20);
                            quo(idx) <= '1';
                        else
                            remain   <= rnext;
                            quo(idx) <= '0';
                        end if;
                        if idx = 0 then
                            if state = DIV_X then
                                state <= SAVE_X;
                            else
                                state <= LATCH;
                            end if;
                        else
                            idx <= idx - 1;
                        end if;

                    when SAVE_X =>
                        -- Quotient <= 639 by construction, low 10 bits suffice.
                        cx_val <= quo(9 downto 0);
                        num    <= s_sum_y;
                        remain <= (others => '0');
                        quo    <= (others => '0');
                        idx    <= 27;
                        state  <= DIV_Y;

                    when LATCH =>
                        cx               <= std_logic_vector(cx_val);
                        cy               <= std_logic_vector(quo(9 downto 0));
                        bbox_w           <= std_logic_vector(s_max_x - s_min_x + 1);
                        bbox_h           <= std_logic_vector(s_max_y - s_min_y + 1);
                        skin_pixel_count <= std_logic_vector(s_cnt);
                        error_x <= std_logic_vector(
                            to_signed(CENTER_X, 11) - signed(resize(cx_val, 11)));
                        error_y <= std_logic_vector(
                            to_signed(CENTER_Y, 11) - signed(resize(quo(9 downto 0), 11)));
                        metrics_valid <= '1';
                        report_valid  <= '1';
                        state         <= IDLE;

                    when IDLE =>
                        null;

                end case;
            end if;
        end if;
    end process;

end architecture rtl;
