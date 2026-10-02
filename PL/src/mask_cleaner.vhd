--------------------------------------------------------------------------------
-- mask_cleaner.vhd
--
-- Applies a generic KERNEL_SIZE x KERNEL_SIZE majority filter to a binary mask.
--
-- KERNEL_SIZE must be odd and >= 3.
--
-- The output is a complete, aligned FRAME_WIDTH x FRAME_HEIGHT raster.
--
-- For every output pixel:
--
--   stream_pixel_valid_out = '1'
--       A new output pixel is available.
--
--   pixel_x_out / pixel_y_out
--       The actual coordinates of that output pixel.
--
--   filtered_mask_valid_out = '1'
--       A complete KERNEL_SIZE x KERNEL_SIZE window existed for this pixel.
--
--
-- This means all downstream blocks receive a normal aligned raster and do not
-- need to know anything about the filter offset.
--
-- Examples:
--
--   KERNEL_SIZE = 3
--      FILTER_OUTPUT_OFFSET = 1
--      majority threshold = 5
--
--   KERNEL_SIZE = 5
--      FILTER_OUTPUT_OFFSET = 2
--      majority threshold = 13
--
-- Throughput after initial alignment:
--
--      1 output pixel per valid input pixel
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.vision_tracker_pkg.all;

entity mask_cleaner is

    generic (
        KERNEL_SIZE  : integer := 3
    );

    port (

        clk     : in std_logic;
        reset_n : in std_logic;
        enable  : in std_logic;


        mask_in : in std_logic;

        pixel_x : in std_logic_vector(FRAME_X_BITS - 1 downto 0);
        pixel_y : in std_logic_vector(FRAME_Y_BITS - 1 downto 0);

        stream_pixel_valid_in : in std_logic;

        frame_end_in : in std_logic;


        filtered_mask : out std_logic;

        pixel_x_out : out std_logic_vector(FRAME_X_BITS - 1 downto 0);
        pixel_y_out : out std_logic_vector(FRAME_Y_BITS - 1 downto 0);

        stream_pixel_valid_out  : out std_logic;
        filtered_mask_valid_out : out std_logic;
        
        filtered_frame_end_out : out std_logic;
        frame_end_out : out std_logic

    );

end entity mask_cleaner;


architecture rtl of mask_cleaner is


    ---------------------------------------------------------------------------
    -- Derived constants.
    ---------------------------------------------------------------------------

    constant FILTER_OUTPUT_OFFSET : integer :=
        KERNEL_SIZE / 2;


    constant WINDOW_PIXELS : integer :=
        KERNEL_SIZE * KERNEL_SIZE;


    constant MAJORITY_THRESHOLD : integer :=
        WINDOW_PIXELS / 2 + 1;


    ---------------------------------------------------------------------------
    -- Number of valid input pixels between the source pixel and the output
    -- pixel represented by the filtered result.
    --
    -- Example:
    --
    --   3x3, FRAME_WIDTH = 640
    --
    --   1 * 640 + 1 = 641 valid pixels
    --
    -- After this initial delay the cleaner produces one aligned output pixel
    -- for every valid input pixel.
    ---------------------------------------------------------------------------

    constant OUTPUT_ALIGNMENT_DELAY : integer :=
        FILTER_OUTPUT_OFFSET * FRAME_WIDTH
        + FILTER_OUTPUT_OFFSET;


    ---------------------------------------------------------------------------
    -- Line buffers.
    --
    -- For a KERNEL_SIZE x KERNEL_SIZE window we need KERNEL_SIZE-1 previous
    -- rows.
    ---------------------------------------------------------------------------

    type line_buffer_t is
        array (
            0 to KERNEL_SIZE - 2,
            0 to FRAME_WIDTH - 1
        ) of std_logic;


    signal line_buffer : line_buffer_t;


    ---------------------------------------------------------------------------
    -- Complete KERNEL_SIZE x KERNEL_SIZE window.
    --
    -- First index  = row inside the window.
    -- Second index = column inside the window.
    ---------------------------------------------------------------------------

    type window_t is
        array (
            0 to KERNEL_SIZE - 1,
            0 to KERNEL_SIZE - 1
        ) of std_logic;


    signal window_reg : window_t :=
        (others => (others => '0'));


    ---------------------------------------------------------------------------
    -- Number of '1' pixels currently stored in window_reg.
    --
    -- Updated incrementally instead of recounting the complete window.
    ---------------------------------------------------------------------------

    signal window_ones_count :
        integer range 0 to WINDOW_PIXELS := 0;


    ---------------------------------------------------------------------------
    -- One complete vertical column of the window.
    ---------------------------------------------------------------------------

    type column_t is
        array (0 to KERNEL_SIZE - 1) of std_logic;


    ---------------------------------------------------------------------------
    -- Output raster alignment.
    --
    -- No output is produced during the initial filter offset.
    -- After that, output_x_reg/output_y_reg simply count through the complete
    -- output frame.
    ---------------------------------------------------------------------------

    signal output_started : std_logic := '0';


    signal alignment_count :
        integer range 0 to OUTPUT_ALIGNMENT_DELAY - 1 := 0;


    signal output_x_reg :
        integer range 0 to FRAME_WIDTH - 1 := 0;


    signal output_y_reg :
        integer range 0 to FRAME_HEIGHT - 1 := 0;


begin


    ---------------------------------------------------------------------------
    -- Main filtering process.
    ---------------------------------------------------------------------------

    cleaner_proc : process(clk)

        variable x_index :
            integer range 0 to FRAME_WIDTH - 1;


        -----------------------------------------------------------------------
        -- New rightmost column of the current window.
        -----------------------------------------------------------------------

        variable right_col : column_t;


        -----------------------------------------------------------------------
        -- Window after shifting left and inserting right_col.
        -----------------------------------------------------------------------

        variable window_next : window_t;


        -----------------------------------------------------------------------
        -- Incremental majority-count update.
        -----------------------------------------------------------------------

        variable outgoing_ones :
            integer range 0 to KERNEL_SIZE;

        variable incoming_ones :
            integer range 0 to KERNEL_SIZE;

        variable next_ones_count :
            integer range 0 to WINDOW_PIXELS;


    begin

        if rising_edge(clk) then


            if reset_n = '0' then

                filtered_mask <= '0';

                pixel_x_out <= (others => '0');
                pixel_y_out <= (others => '0');

                stream_pixel_valid_out  <= '0';
                filtered_mask_valid_out <= '0';

                filtered_frame_end_out <= '0';
                frame_end_out <= '0';


                window_reg <=
                    (others => (others => '0'));

                window_ones_count <= 0;
          
                output_started <= '0';

                alignment_count <= 0;

                output_x_reg <= 0;
                output_y_reg <= 0;


            elsif enable = '1' then


                ----------------------------------------------------------------
                -- Default output values.
                --
                -- These signals become '1' only when the current clock really
                -- produces the corresponding event.
                ----------------------------------------------------------------

                filtered_mask <= '0';

                stream_pixel_valid_out  <= '0';
                filtered_mask_valid_out <= '0';

                filtered_frame_end_out <= '0';
                frame_end_out <= '0';

                if stream_pixel_valid_in = '1' then

                    ----------------------------------------------------------------
                    -- End of the valid filtered region for the current source frame.
                    --
                    -- When the final source pixel arrives, the majority filter also
                    -- produces the final pixel for which a complete KERNEL_SIZE x
                    -- KERNEL_SIZE window exists.
                    --
                    -- This pulse is used by object_measurement to close the current
                    -- measurement without waiting for the invalid border pixels of
                    -- the aligned output raster.
                    ----------------------------------------------------------------
                    if
                        (frame_end_in = '1')
                        and
                        (output_started = '1')
                    then
                    
                        filtered_frame_end_out <= '1';
                    
                    end if;


                    x_index :=
                        to_integer(unsigned(pixel_x));


                    ----------------------------------------------------------------
                    -- Build the newest/rightmost vertical column.
                    ----------------------------------------------------------------

                    for row_index in 0 to KERNEL_SIZE - 2 loop

                        right_col(row_index) :=
                            line_buffer(row_index, x_index);

                    end loop;


                    right_col(KERNEL_SIZE - 1) :=
                        mask_in;


                    ----------------------------------------------------------------
                    -- Incrementally update the number of '1' pixels.
                    --
                    -- The leftmost column leaves the window and right_col enters.
                    ----------------------------------------------------------------

                    outgoing_ones := 0;
                    incoming_ones := 0;

                    for row_index in 0 to KERNEL_SIZE - 1 loop

                        if window_reg(row_index, 0) = '1' then

                            outgoing_ones :=
                                outgoing_ones + 1;

                        end if;


                        if right_col(row_index) = '1' then

                            incoming_ones :=
                                incoming_ones + 1;

                        end if;

                    end loop;


                    next_ones_count :=
                        window_ones_count - outgoing_ones + incoming_ones;


                    ----------------------------------------------------------------
                    -- Shift the complete window one column to the left.
                    ----------------------------------------------------------------

                    for row_index in 0 to KERNEL_SIZE - 1 loop

                        for column_index in 0 to KERNEL_SIZE - 2 loop

                            window_next(row_index, column_index) :=
                                window_reg( row_index, column_index + 1
                                );

                        end loop;


                        ----------------------------------------------------------------
                        -- Insert the new column at the right side.
                        ----------------------------------------------------------------

                        window_next(
                            row_index,
                            KERNEL_SIZE - 1 ) := right_col(row_index);

                    end loop;


                    ----------------------------------------------------------------
                    -- Save the new window and its number of '1' pixels for the
                    -- next input pixel.
                    ----------------------------------------------------------------

                    window_reg <= window_next;

                    window_ones_count <= next_ones_count;


                    ----------------------------------------------------------------
                    -- Initial output alignment.
                    --
                    -- The filter result is spatially behind the current source
                    -- position by:
                    --
                    --   FILTER_OUTPUT_OFFSET rows
                    --   +
                    --   FILTER_OUTPUT_OFFSET columns
                    --
                    -- We therefore wait this fixed number of valid input pixels
                    -- once after reset.
                    ----------------------------------------------------------------

                    if output_started = '0' then

                        if
                            alignment_count =
                                OUTPUT_ALIGNMENT_DELAY - 1
                        then

                            output_started <= '1';

                            output_x_reg <= 0;
                            output_y_reg <= 0;

                        else

                            alignment_count <=
                                alignment_count + 1;

                        end if;


                    else


                        ----------------------------------------------------------------
                        -- One complete aligned output pixel is available.
                        ----------------------------------------------------------------

                        stream_pixel_valid_out <= '1';


                        pixel_x_out <= std_logic_vector(
                            to_unsigned(
                                output_x_reg, pixel_x_out'length
                            )
                        );


                        pixel_y_out <= std_logic_vector(
                            to_unsigned(
                                output_y_reg, pixel_y_out'length
                            )
                        );


                        ----------------------------------------------------------------
                        -- A complete filter window exists only when the aligned
                        -- output position is far enough from the top and left borders.
                        --
                        -- If no complete window exists:
                        --
                        --   filtered_mask_valid_out = 0
                        --   filtered_mask           = 0
                        ----------------------------------------------------------------

                        if
                            (output_x_reg >= FILTER_OUTPUT_OFFSET)
                            and
                            (output_y_reg >= FILTER_OUTPUT_OFFSET)
                        then
                        
                            filtered_mask_valid_out <= '1';


                            ----------------------------------------------------
                            -- Majority decision.
                            ----------------------------------------------------

                            if
                                next_ones_count >= MAJORITY_THRESHOLD
                            then

                                filtered_mask <= '1';

                            else

                                filtered_mask <= '0';

                            end if;

                        end if;


                        ----------------------------------------------------------------
                        -- The output raster itself determines frame end.
                        --
                        -- This is important because the final output pixels may
                        -- have filtered_mask_valid_out = 0, but they are still
                        -- real pixels in the complete output frame.
                        ----------------------------------------------------------------

                        if
                            (output_x_reg = FRAME_WIDTH - 1)
                            and
                            (output_y_reg = FRAME_HEIGHT - 1)
                        then

                            frame_end_out <= '1';

                        end if;


                        ----------------------------------------------------------------
                        -- Advance the aligned output raster.
                        ----------------------------------------------------------------

                        if output_x_reg = FRAME_WIDTH - 1 then

                            output_x_reg <= 0;


                            if output_y_reg = FRAME_HEIGHT - 1 then

                                output_y_reg <= 0;

                            else

                                output_y_reg <= output_y_reg + 1;

                            end if;


                        else

                            output_x_reg <= output_x_reg + 1;

                        end if;

                    end if;


                    ----------------------------------------------------------------
                    -- Update line buffers.
                    --
                    -- Every stored row moves one step farther into the past.
                    -- The current input pixel becomes the newest stored row.
                    ----------------------------------------------------------------

                    for row_index in 0 to KERNEL_SIZE - 3 loop

                        line_buffer(row_index, x_index) <=
                            line_buffer(
                                row_index + 1, x_index
                            );

                    end loop;


                    line_buffer(
                        KERNEL_SIZE - 2, x_index) <= mask_in;


                end if;

            end if;

        end if;

    end process cleaner_proc;


end architecture rtl;