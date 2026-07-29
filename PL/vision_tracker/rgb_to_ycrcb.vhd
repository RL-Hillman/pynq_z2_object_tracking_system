--------------------------------------------------------------------------------
-- rgb_to_ycrcb.vhd
--
-- Fixed-point RGB -> YCrCb converter (BT.601 full range — the same matrix
-- OpenCV uses on the PS side, so PL classification agrees with the Python
-- calibration to within a few counts of rounding; that residual is absorbed
-- by SKIN_THR_PADDING on the PS side).
--
--   Y  = ( 77*R + 150*G +  29*B) >> 8
--   Cb = ((-43*R -  85*G + 128*B) >> 8) + 128
--   Cr = ((128*R - 107*G -  21*B) >> 8) + 128
--
-- Two-stage pipeline: stage 1 registers the weighted sums, stage 2 registers
-- the shifted/offset/clamped result. Total latency = 2 clk. valid,
-- end-of-line and start-of-frame flags are delayed by the same 2 clk so the
-- downstream pixel counter stays aligned with the data.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity rgb_to_ycrcb is
    port (
        clk             : in  std_logic;
        resetn          : in  std_logic;  -- synchronous, active low

        in_valid        : in  std_logic;
        in_r            : in  std_logic_vector(7 downto 0);
        in_g            : in  std_logic_vector(7 downto 0);
        in_b            : in  std_logic_vector(7 downto 0);
        in_line_last    : in  std_logic;  -- end-of-line flag, aligned with in_valid
        in_frame_start  : in  std_logic;  -- start-of-frame flag, aligned with in_valid

        out_valid       : out std_logic;
        out_y           : out std_logic_vector(7 downto 0);
        out_cr          : out std_logic_vector(7 downto 0);
        out_cb          : out std_logic_vector(7 downto 0);
        out_line_last   : out std_logic;
        out_frame_start : out std_logic
    );
end entity rgb_to_ycrcb;

architecture rtl of rgb_to_ycrcb is

    -- Matrix coefficients as 9-bit signed constants (range -256..255).
    constant C_Y_R  : signed(8 downto 0) := to_signed(  77, 9);
    constant C_Y_G  : signed(8 downto 0) := to_signed( 150, 9);
    constant C_Y_B  : signed(8 downto 0) := to_signed(  29, 9);
    constant C_CB_R : signed(8 downto 0) := to_signed( -43, 9);
    constant C_CB_G : signed(8 downto 0) := to_signed( -85, 9);
    constant C_CB_B : signed(8 downto 0) := to_signed( 128, 9);
    constant C_CR_R : signed(8 downto 0) := to_signed( 128, 9);
    constant C_CR_G : signed(8 downto 0) := to_signed(-107, 9);
    constant C_CR_B : signed(8 downto 0) := to_signed( -21, 9);

    -- Channels zero-extended to 9-bit signed (always non-negative).
    signal r_s, g_s, b_s : signed(8 downto 0);

    -- Stage-1 registers: full-precision weighted sums (max |value| < 2**19).
    signal y_acc, cb_acc, cr_acc : signed(19 downto 0);
    signal v_d1, ll_d1, fs_d1    : std_logic;

    -- Stage-2 (output) registers.
    signal v_d2, ll_d2, fs_d2    : std_logic;
    signal y_q, cr_q, cb_q       : std_logic_vector(7 downto 0);

    -- Clamp a post-shift result to the 0..255 pixel range.
    function clamp8(v : signed(19 downto 0)) return std_logic_vector is
    begin
        if v < 0 then
            return x"00";
        elsif v > 255 then
            return x"FF";
        else
            return std_logic_vector(v(7 downto 0));
        end if;
    end function;

begin

    r_s <= signed('0' & in_r);
    g_s <= signed('0' & in_g);
    b_s <= signed('0' & in_b);

    process(clk)
        variable y_v, cb_v, cr_v : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            if resetn = '0' then
                v_d1 <= '0';  v_d2 <= '0';
                ll_d1 <= '0'; ll_d2 <= '0';
                fs_d1 <= '0'; fs_d2 <= '0';
            else
                -- Stage 1: 9x9 products (18 bit) summed into 20-bit accumulators.
                y_acc  <= resize(r_s * C_Y_R,  20) + resize(g_s * C_Y_G,  20)
                        + resize(b_s * C_Y_B,  20);
                cb_acc <= resize(r_s * C_CB_R, 20) + resize(g_s * C_CB_G, 20)
                        + resize(b_s * C_CB_B, 20);
                cr_acc <= resize(r_s * C_CR_R, 20) + resize(g_s * C_CR_G, 20)
                        + resize(b_s * C_CR_B, 20);
                v_d1  <= in_valid;
                ll_d1 <= in_line_last;
                fs_d1 <= in_frame_start;

                -- Stage 2: arithmetic >>8 (floor), chroma offset, clamp.
                y_v  := shift_right(y_acc, 8);
                cb_v := shift_right(cb_acc, 8) + 128;
                cr_v := shift_right(cr_acc, 8) + 128;
                y_q  <= clamp8(y_v);
                cb_q <= clamp8(cb_v);
                cr_q <= clamp8(cr_v);
                v_d2  <= v_d1;
                ll_d2 <= ll_d1;
                fs_d2 <= fs_d1;
            end if;
        end if;
    end process;

    out_valid       <= v_d2;
    out_y           <= y_q;
    out_cr          <= cr_q;
    out_cb          <= cb_q;
    out_line_last   <= ll_d2;
    out_frame_start <= fs_d2;

end architecture rtl;
