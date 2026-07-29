--------------------------------------------------------------------------------
-- roi_skin_segmenter.vhd
--
-- Pixel X/Y counter + ROI gate + Cr/Cb skin classifier, in one stage.
--
-- Counts the (x, y) position of each incoming pixel and classifies it:
--   skin = inside current ROI  AND  cr_min <= Cr <= cr_max
--                              AND  cb_min <= Cb <= cb_max
-- Pixels outside the ROI are always 0.
--
-- Frame synchronisation:
--   * line_last   = end-of-line flag (AXI4-Stream video TLAST): resets x,
--                   increments y. The counter also self-wraps at
--                   FRAME_WIDTH-1 so a missing flag cannot desync a line.
--   * frame_start = start-of-frame flag (TUSER/SOF): forces the current
--                   pixel to (0,0), so any drift recovers at each frame.
--   * frame_end_out pulses together with the LAST pixel of the frame
--     (x = FRAME_WIDTH-1, y = FRAME_HEIGHT-1) for downstream latching.
--
-- Latency: 1 clk. Outputs are registered and aligned with mask_valid.
-- Classification is chroma-only (Cr/Cb) — luma is not an input by design.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity roi_skin_segmenter is
    generic (
        FRAME_WIDTH  : integer := 640;
        FRAME_HEIGHT : integer := 480
    );
    port (
        clk           : in  std_logic;
        resetn        : in  std_logic;  -- synchronous, active low

        pixel_valid   : in  std_logic;
        pixel_cr      : in  std_logic_vector(7 downto 0);
        pixel_cb      : in  std_logic_vector(7 downto 0);
        line_last     : in  std_logic;
        frame_start   : in  std_logic;

        current_roi_x : in  std_logic_vector(9 downto 0);
        current_roi_y : in  std_logic_vector(9 downto 0);
        current_roi_w : in  std_logic_vector(9 downto 0);
        current_roi_h : in  std_logic_vector(9 downto 0);
        cr_min        : in  std_logic_vector(7 downto 0);
        cr_max        : in  std_logic_vector(7 downto 0);
        cb_min        : in  std_logic_vector(7 downto 0);
        cb_max        : in  std_logic_vector(7 downto 0);

        pixel_x_out   : out std_logic_vector(9 downto 0);
        pixel_y_out   : out std_logic_vector(9 downto 0);
        skin_mask     : out std_logic;
        mask_valid    : out std_logic;
        frame_end_out : out std_logic   -- 1-clk pulse with the frame's last pixel
    );
end entity roi_skin_segmenter;

architecture rtl of roi_skin_segmenter is

    -- Position of the NEXT pixel to arrive.
    signal x_cnt : unsigned(9 downto 0) := (others => '0');
    signal y_cnt : unsigned(9 downto 0) := (others => '0');

begin

    process(clk)
        variable cur_x, cur_y         : unsigned(9 downto 0);
        variable roi_x_end, roi_y_end : unsigned(10 downto 0);  -- exclusive bounds, <= 1280
        variable in_roi, cr_ok, cb_ok : boolean;
    begin
        if rising_edge(clk) then
            if resetn = '0' then
                x_cnt         <= (others => '0');
                y_cnt         <= (others => '0');
                skin_mask     <= '0';
                mask_valid    <= '0';
                frame_end_out <= '0';
            else
                mask_valid    <= '0';
                frame_end_out <= '0';

                if pixel_valid = '1' then
                    -- Coordinate of THIS pixel (frame_start overrides to (0,0)).
                    if frame_start = '1' then
                        cur_x := (others => '0');
                        cur_y := (others => '0');
                    else
                        cur_x := x_cnt;
                        cur_y := y_cnt;
                    end if;

                    -- ROI window test (right/bottom bounds are exclusive).
                    roi_x_end := resize(unsigned(current_roi_x), 11)
                               + resize(unsigned(current_roi_w), 11);
                    roi_y_end := resize(unsigned(current_roi_y), 11)
                               + resize(unsigned(current_roi_h), 11);
                    in_roi := (cur_x >= unsigned(current_roi_x))
                          and (resize(cur_x, 11) < roi_x_end)
                          and (cur_y >= unsigned(current_roi_y))
                          and (resize(cur_y, 11) < roi_y_end);

                    -- Chroma thresholds (inclusive).
                    cr_ok := (unsigned(pixel_cr) >= unsigned(cr_min))
                         and (unsigned(pixel_cr) <= unsigned(cr_max));
                    cb_ok := (unsigned(pixel_cb) >= unsigned(cb_min))
                         and (unsigned(pixel_cb) <= unsigned(cb_max));

                    if in_roi and cr_ok and cb_ok then
                        skin_mask <= '1';
                    else
                        skin_mask <= '0';
                    end if;

                    pixel_x_out <= std_logic_vector(cur_x);
                    pixel_y_out <= std_logic_vector(cur_y);
                    mask_valid  <= '1';

                    if (cur_x = FRAME_WIDTH - 1) and (cur_y = FRAME_HEIGHT - 1) then
                        frame_end_out <= '1';
                    end if;

                    -- Advance to the next pixel position.
                    if (line_last = '1') or (cur_x = FRAME_WIDTH - 1) then
                        x_cnt <= (others => '0');
                        if cur_y = FRAME_HEIGHT - 1 then
                            y_cnt <= (others => '0');
                        else
                            y_cnt <= cur_y + 1;
                        end if;
                    else
                        x_cnt <= cur_x + 1;
                        y_cnt <= cur_y;   -- keep override from frame_start
                    end if;
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
