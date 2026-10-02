--------------------------------------------------------------------------------
-- roi_mask_generator.vhd
--
-- Tracks the X/Y position of each incoming pixel and generates a binary mask
-- according to the current ROI and Cr/Cb thresholds.
--
-- A pixel is classified as part of the target when:
--
--   inside ROI
--   AND cr_min <= Cr <= cr_max
--   AND cb_min <= Cb <= cb_max
--
-- Pixels outside the ROI are always classified as 0.
--
-- The architecture is divided into two processes:
--
--   1. position_tracking_process
--        Tracks the X/Y position of the incoming pixel stream.
--
--   2. pixel_classification_process
--        Checks ROI and Cr/Cb thresholds and generates the binary mask.
--
-- Stream synchronization:
--   * new_frame = first pixel of a new frame
--   * last_line = last pixel of the current line
--
-- frame_end_out pulses for one clock together with the final valid pixel
-- of the frame.
--
-- Latency = 1 clock.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.vision_tracker_pkg.all;

entity roi_mask_generator is


    port (

        clk     : in std_logic;
        reset_n : in std_logic;
        enable  : in std_logic;


        ------------------------------------------------------------------------
        -- Incoming pixel
        ------------------------------------------------------------------------

        pixel_valid : in std_logic;

        pixel_cr : in std_logic_vector(7 downto 0);
        pixel_cb : in std_logic_vector(7 downto 0);


        ------------------------------------------------------------------------
        -- Video synchronization
        ------------------------------------------------------------------------

        last_line : in std_logic;
        new_frame : in std_logic;


        ------------------------------------------------------------------------
        -- Current tracking ROI
        ------------------------------------------------------------------------

        current_roi_x : in std_logic_vector(FRAME_X_BITS - 1 downto 0);
        current_roi_y : in std_logic_vector(FRAME_Y_BITS - 1 downto 0);

        current_roi_w : in std_logic_vector(FRAME_X_BITS - 1 downto 0);
        current_roi_h : in std_logic_vector(FRAME_Y_BITS - 1 downto 0);


        ------------------------------------------------------------------------
        -- Chroma thresholds
        ------------------------------------------------------------------------

        cr_min : in std_logic_vector(7 downto 0);
        cr_max : in std_logic_vector(7 downto 0);

        cb_min : in std_logic_vector(7 downto 0);
        cb_max : in std_logic_vector(7 downto 0);


        ------------------------------------------------------------------------
        -- Classified pixel output
        ------------------------------------------------------------------------

        pixel_x_out : out std_logic_vector(FRAME_X_BITS - 1 downto 0);
        pixel_y_out : out std_logic_vector(FRAME_Y_BITS - 1 downto 0);

        pixel_match  : out std_logic;
        result_valid : out std_logic;


        ------------------------------------------------------------------------
        -- End-of-frame pulse
        ------------------------------------------------------------------------

        frame_end_out : out std_logic

    );

end entity roi_mask_generator;



architecture rtl of roi_mask_generator is


    ---------------------------------------------------------------------------
    -- Pixel position counters
    --
    -- These registers hold the expected position of the next valid pixel.
    ---------------------------------------------------------------------------
    signal x_cnt : unsigned(FRAME_X_BITS - 1 downto 0) := (others => '0');
    signal y_cnt : unsigned(FRAME_Y_BITS - 1 downto 0) := (others => '0');


    begin


    ---------------------------------------------------------------------------
    -- PROCESS 1: PIXEL POSITION TRACKING
    --
    -- Responsible for maintaining the X/Y position of the incoming
    -- pixel stream.
    ---------------------------------------------------------------------------

    position_tracking_process : process(clk)

    begin

        if rising_edge(clk) then

            if reset_n = '0' then

                x_cnt <= (others => '0');
                y_cnt <= (others => '0');

           elsif enable = '1' then

                if pixel_valid = '1' then
    
    
                    if new_frame = '1' then
    
                        x_cnt <= to_unsigned(1, x_cnt'length);
                        y_cnt <= (others => '0');
    
    
                    elsif last_line = '1' then
    
                        -- The next line starts at X = 0.
                        x_cnt <= (others => '0');
    
                        -----------------------------------------------------------
                        -- Last line of the frame
                        -----------------------------------------------------------
                        if y_cnt =
                           to_unsigned(FRAME_HEIGHT - 1, y_cnt'length) then
    
                            -- Prepare for the next frame.
                            y_cnt <= (others => '0');
    
                        else
                            y_cnt <= y_cnt + 1;
    
                        end if;
    
    
                    else
                        -- y_cnt stays unchanged.
                        x_cnt <= x_cnt + 1;
    
                    end if;
    end if;
                end if;
        end if;

    end process position_tracking_process;



    ---------------------------------------------------------------------------
    -- PROCESS 2: PIXEL CLASSIFICATION
    --
    -- Responsible for determining whether the current pixel belongs
    -- to the tracked object.
    --
    -- The pixel must:
    --
    --   1. Be inside the current ROI
    --   2. Have Cr inside the configured range
    --   3. Have Cb inside the configured range
    ---------------------------------------------------------------------------

    pixel_classification_process : process(clk)


        -----------------------------------------------------------------------
        -- Position of the pixel being processed in the current clock
        --
        -- Normally the position comes from x_cnt/y_cnt.
        --
        -- When new_frame = '1', the incoming pixel is explicitly known to
        -- be the first pixel of the frame, so its position is (0,0).
        -----------------------------------------------------------------------

        variable current_x : unsigned(FRAME_X_BITS - 1 downto 0);
        variable current_y : unsigned(FRAME_Y_BITS - 1 downto 0);


        -----------------------------------------------------------------------
        -- Exclusive ROI boundaries
        -----------------------------------------------------------------------

        variable roi_right  : unsigned(FRAME_X_BITS - 1 downto 0);
        variable roi_bottom : unsigned(FRAME_Y_BITS - 1 downto 0);


        -----------------------------------------------------------------------
        -- Classification conditions
        -----------------------------------------------------------------------

        variable in_roi      : boolean;
        variable cr_in_range : boolean;
        variable cb_in_range : boolean;
        
        
        
            
        ---------------------------------------------------------------------------
        -- Frame-stable tracking ROI
        --
        -- roi_tracker_update may produce a new ROI while the current frame is
        -- already being processed.
        --
        -- Therefore, the tracking ROI is captured when a new frame begins and the
        -- captured values are used for the rest of that frame. This guarantees that
        -- every pixel in one frame is classified using the same ROI.
        ---------------------------------------------------------------------------
        variable frame_roi_x : unsigned(FRAME_X_BITS - 1 downto 0);
        variable frame_roi_y : unsigned(FRAME_Y_BITS - 1 downto 0);
        variable frame_roi_w : unsigned(FRAME_X_BITS - 1 downto 0);
        variable frame_roi_h : unsigned(FRAME_Y_BITS - 1 downto 0);

    begin

        if rising_edge(clk) then


            if reset_n = '0' then

                pixel_x_out <= (others => '0');
                pixel_y_out <= (others => '0');

                pixel_match <= '0';

                result_valid  <= '0';
                frame_end_out <= '0';
                ---------------------------------------------------------------
                -- Clear the stored frame ROI.
                ---------------------------------------------------------------
                frame_roi_x := (others => '0');
                frame_roi_y := (others => '0');
                frame_roi_w := (others => '0');
                frame_roi_h := (others => '0');
                
            elsif enable = '1' then



                ---------------------------------------------------------------
                -- Default values for one-clock pulse outputs
                ---------------------------------------------------------------
                result_valid  <= '0';
                frame_end_out <= '0';


                if pixel_valid = '1' then

                    -----------------------------------------------------------
                    -- Determine current pixel position
                    -----------------------------------------------------------
                    if new_frame = '1' then

                        current_x := (others => '0');
                        current_y := (others => '0');
                        
                        ---------------------------------------------------------------
                        -- Capture the tracking ROI at the beginning of the frame.
                        --
                        -- roi_tracker_update may change current_roi_* later, but those
                        -- changes will only be used starting from the next frame.
                        ---------------------------------------------------------------
                        frame_roi_x := unsigned(current_roi_x);
                        frame_roi_y := unsigned(current_roi_y);
                        frame_roi_w := unsigned(current_roi_w);
                        frame_roi_h := unsigned(current_roi_h);
                        
                    else

                        current_x := x_cnt;
                        current_y := y_cnt;

                    end if;


                    -----------------------------------------------------------
                    -- Output current pixel position
                    -----------------------------------------------------------

                    pixel_x_out <= std_logic_vector(current_x);
                    pixel_y_out <= std_logic_vector(current_y);

                    result_valid <= '1';


                    -----------------------------------------------------------
                    -- Calculate exclusive ROI boundaries
                    --
                    -- Example:
                    --
                    --   frame_roi_x = 100
                    --   frame_roi_w = 200
                    --
                    --   roi_right = 300
                    --
                    -- Valid X coordinates are therefore 100..299.
                    -----------------------------------------------------------
                    
                    roi_right :=
                        resize(frame_roi_x, roi_right'length)
                        +
                        resize(frame_roi_w, roi_right'length);
                    
                    roi_bottom :=
                        resize(frame_roi_y, roi_bottom'length)
                        +
                        resize(frame_roi_h, roi_bottom'length);


                    -----------------------------------------------------------
                    -- Check whether the current pixel is inside the ROI
                    -----------------------------------------------------------

                    in_roi :=
                    
                        (current_x >= frame_roi_x)
                    
                        and
                    
                        (resize(current_x, roi_right'length) < roi_right)
                    
                        and
                    
                        (current_y >= frame_roi_y)
                    
                        and
                    
                        (resize(current_y, roi_bottom'length) < roi_bottom);


                    -----------------------------------------------------------
                    -- Check whether Cr is inside its allowed range
                    -----------------------------------------------------------

                    cr_in_range :=

                        (unsigned(pixel_cr) >= unsigned(cr_min))

                        and

                        (unsigned(pixel_cr) <= unsigned(cr_max));


                    -----------------------------------------------------------
                    -- Check whether Cb is inside its allowed range
                    -----------------------------------------------------------

                    cb_in_range :=

                        (unsigned(pixel_cb) >= unsigned(cb_min))

                        and

                        (unsigned(pixel_cb) <= unsigned(cb_max));


                    -----------------------------------------------------------
                    -- Classify current pixel
                    -----------------------------------------------------------

                    if in_roi and cr_in_range and cb_in_range then

                        pixel_match <= '1';

                    else

                        pixel_match <= '0';

                    end if;


                    -----------------------------------------------------------
                    -- Detect final pixel of the frame
                    -----------------------------------------------------------

                    if
                        (current_x =
                         to_unsigned(FRAME_WIDTH - 1, current_x'length))

                        and

                        (current_y =
                         to_unsigned(FRAME_HEIGHT - 1, current_y'length))
                    then

                        frame_end_out <= '1';

                    end if;

                end if;

            end if;

        end if;

    end process pixel_classification_process;


end architecture rtl;