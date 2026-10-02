--------------------------------------------------------------------------------
-- rgb_to_ycrcb.vhd
--
-- RGB to YCrCb color conversion.
--
--   Y  = ( 77*R + 150*G +  29*B) >> 8
--   Cb = ((-43*R -  85*G + 128*B) >> 8) + 128
--   Cr = ((128*R - 107*G -  21*B) >> 8) + 128
--
-- All conversion arithmetic is completed in one clock cycle.
-- The final Y, Cr and Cb values are registered at the output.
--
-- Total latency = 1 clock.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity rgb_to_ycrcb is
    port (
        clk      : in  std_logic;
        reset_n  : in  std_logic;
        enable : in std_logic;
        
        pixel_valid_in : in std_logic;
        new_frame_in   : in std_logic;
        last_line_in   : in std_logic;
        
        R_in     : in  std_logic_vector(7 downto 0);
        G_in     : in  std_logic_vector(7 downto 0);
        B_in     : in  std_logic_vector(7 downto 0);

        y_out    : out std_logic_vector(7 downto 0);
        cr_out   : out std_logic_vector(7 downto 0);
        cb_out   : out std_logic_vector(7 downto 0);
        
        pixel_valid_out : out std_logic;
        new_frame_out   : out std_logic;
        last_line_out   : out std_logic
        
    );
end entity rgb_to_ycrcb;


architecture rtl of rgb_to_ycrcb is

    ---------------------------------------------------------------------------
    -- RGB to YCrCb conversion coefficients
    ---------------------------------------------------------------------------

    constant R_TO_Y  : signed(8 downto 0) := to_signed(  77, 9);
    constant G_TO_Y  : signed(8 downto 0) := to_signed( 150, 9);
    constant B_TO_Y  : signed(8 downto 0) := to_signed(  29, 9);

    constant R_TO_CB : signed(8 downto 0) := to_signed( -43, 9);
    constant G_TO_CB : signed(8 downto 0) := to_signed( -85, 9);
    constant B_TO_CB : signed(8 downto 0) := to_signed( 128, 9);

    constant R_TO_CR : signed(8 downto 0) := to_signed( 128, 9);
    constant G_TO_CR : signed(8 downto 0) := to_signed(-107, 9);
    constant B_TO_CR : signed(8 downto 0) := to_signed( -21, 9);


    ---------------------------------------------------------------------------
    -- RGB inputs converted to positive 9-bit signed values
    ---------------------------------------------------------------------------

    signal R_signed : signed(8 downto 0);
    signal G_signed : signed(8 downto 0);
    signal B_signed : signed(8 downto 0);


begin

    ---------------------------------------------------------------------------
    -- Convert unsigned RGB values to positive signed values
    ---------------------------------------------------------------------------

    R_signed <= signed('0' & R_in);
    G_signed <= signed('0' & G_in);
    B_signed <= signed('0' & B_in);


    ---------------------------------------------------------------------------
    -- RGB to YCrCb conversion
    ---------------------------------------------------------------------------

    process(clk)

        variable y_temp  : signed(16 downto 0);
        variable cb_temp : signed(16 downto 0);
        variable cr_temp : signed(16 downto 0);

    begin

        if rising_edge(clk) then

            if reset_n = '0' then

                y_out  <= (others => '0');
                cr_out <= (others => '0');
                cb_out <= (others => '0');
                
                pixel_valid_out <= '0';
                new_frame_out   <= '0';
                last_line_out   <= '0';


            elsif enable = '1' then
    
                ---------------------------------------------------------------
                -- Delay pixel timing together with the color conversion.
                ---------------------------------------------------------------
                pixel_valid_out <= pixel_valid_in;
                new_frame_out   <= new_frame_in;
                last_line_out   <= last_line_in;
                
                
                ---------------------------------------------------------------
                -- Calculate weighted RGB sums
                ---------------------------------------------------------------
                if pixel_valid_in = '1' then

                y_temp :=
                    resize(R_signed * R_TO_Y, 17) +
                    resize(G_signed * G_TO_Y, 17) +
                    resize(B_signed * B_TO_Y, 17);

                cb_temp :=
                    resize(R_signed * R_TO_CB, 17) +
                    resize(G_signed * G_TO_CB, 17) +
                    resize(B_signed * B_TO_CB, 17);

                cr_temp :=
                    resize(R_signed * R_TO_CR, 17) +
                    resize(G_signed * G_TO_CR, 17) +
                    resize(B_signed * B_TO_CR, 17);


                ---------------------------------------------------------------
                -- Divide by 256
                -- Add 128 to Cr and Cb to move them into the 0..255 range
                ---------------------------------------------------------------

                y_temp :=
                    shift_right(y_temp, 8);

                cb_temp :=
                    shift_right(cb_temp, 8) +
                    to_signed(128, 17);

                cr_temp :=
                    shift_right(cr_temp, 8) +
                    to_signed(128, 17);


                ---------------------------------------------------------------
                -- Register final 8-bit YCrCb outputs
                ---------------------------------------------------------------

                y_out  <= std_logic_vector(y_temp(7 downto 0));
                cb_out <= std_logic_vector(cb_temp(7 downto 0));
                cr_out <= std_logic_vector(cr_temp(7 downto 0));
                end if;
            end if;

        end if;

    end process;


end architecture rtl;