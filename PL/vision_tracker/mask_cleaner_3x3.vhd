--------------------------------------------------------------------------------
-- mask_cleaner_3x3.vhd
--
-- 3x3 majority filter on the binary skin mask: output '1' when 5 or more of
-- the 9 window pixels are '1'. Because the previous stage forces pixels
-- outside the ROI to '0', the filter effectively operates only inside the
-- ROI (with mild erosion at the ROI border, which is intended).
--
-- Implementation: causal window. Two single-bit line buffers hold the two
-- previous rows; 3-tap column shift registers form the window. When the
-- input pixel (x, y) arrives, the full window centred on (x-1, y-1) is
-- available, so the output for centre (x-1, y-1) is emitted on that same
-- clock (registered, 1 clk latency).
--
-- Border policy: centres on the outermost 1-pixel frame ring (x = 0,
-- x = W-1, y = 0, y = H-1) have no complete window and are NOT emitted.
-- The effective analysis region is the frame interior, which is irrelevant
-- for hand tracking (the ROI never hugs the frame border).
--
-- This scheme also guarantees no cross-frame spill: the last output of a
-- frame (centre (W-2, H-2)) is produced exactly when the frame's last input
-- pixel (W-1, H-1) arrives, so frame_end_in passes through with the same
-- 1-clk delay and stays aligned with the final output of the frame.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity mask_cleaner_3x3 is
    generic (
        FRAME_WIDTH : integer := 640
    );
    port (
        clk             : in  std_logic;
        resetn          : in  std_logic;  -- synchronous, active low

        skin_mask_in    : in  std_logic;
        pixel_x         : in  std_logic_vector(9 downto 0);
        pixel_y         : in  std_logic_vector(9 downto 0);
        mask_valid_in   : in  std_logic;
        frame_end_in    : in  std_logic;  -- pulse with the frame's last input pixel

        clean_skin_mask : out std_logic;
        pixel_x_out     : out std_logic_vector(9 downto 0);  -- window centre x
        pixel_y_out     : out std_logic_vector(9 downto 0);  -- window centre y
        mask_valid_out  : out std_logic;
        frame_end_out   : out std_logic   -- aligned with the frame's last output
    );
end entity mask_cleaner_3x3;

architecture rtl of mask_cleaner_3x3 is

    -- Single-bit line buffers (async read + sync write -> distributed RAM).
    type line_buf_t is array (0 to FRAME_WIDTH - 1) of std_logic;
    signal row_m1 : line_buf_t := (others => '0');  -- row y-1
    signal row_m2 : line_buf_t := (others => '0');  -- row y-2

    -- Column shift registers: (0) = col x-1, (1) = col x-2 (before update).
    signal sr_top, sr_mid, sr_bot : std_logic_vector(1 downto 0) := (others => '0');

    function count_ones(v : std_logic_vector(8 downto 0)) return integer is
        variable n : integer range 0 to 9 := 0;
    begin
        for i in v'range loop
            if v(i) = '1' then
                n := n + 1;
            end if;
        end loop;
        return n;
    end function;

begin

    process(clk)
        variable x_i            : integer range 0 to FRAME_WIDTH - 1;
        variable top_rd, mid_rd : std_logic;
        variable win            : std_logic_vector(8 downto 0);
    begin
        if rising_edge(clk) then
            if resetn = '0' then
                mask_valid_out  <= '0';
                frame_end_out   <= '0';
                clean_skin_mask <= '0';
            else
                mask_valid_out <= '0';
                -- Same 1-clk register delay as the data path -> stays aligned
                -- with the last emitted window of the frame.
                frame_end_out  <= frame_end_in;

                if mask_valid_in = '1' then
                    x_i    := to_integer(unsigned(pixel_x));
                    top_rd := row_m2(x_i);   -- row y-2, col x (read-before-write)
                    mid_rd := row_m1(x_i);   -- row y-1, col x

                    -- 3x3 window: rows y-2..y, columns x-2..x.
                    win := (sr_top(1) & sr_top(0) & top_rd)
                         & (sr_mid(1) & sr_mid(0) & mid_rd)
                         & (sr_bot(1) & sr_bot(0) & skin_mask_in);

                    -- Emit centre (x-1, y-1) when the window is fully inside
                    -- the frame.
                    if (unsigned(pixel_x) >= 2) and (unsigned(pixel_y) >= 2) then
                        mask_valid_out <= '1';
                        pixel_x_out    <= std_logic_vector(unsigned(pixel_x) - 1);
                        pixel_y_out    <= std_logic_vector(unsigned(pixel_y) - 1);
                        if count_ones(win) >= 5 then
                            clean_skin_mask <= '1';
                        else
                            clean_skin_mask <= '0';
                        end if;
                    end if;

                    -- Update line buffers and column taps for the next pixel.
                    row_m2(x_i) <= mid_rd;
                    row_m1(x_i) <= skin_mask_in;
                    sr_top <= sr_top(0) & top_rd;
                    sr_mid <= sr_mid(0) & mid_rd;
                    sr_bot <= sr_bot(0) & skin_mask_in;
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
