--------------------------------------------------------------------------------
-- object_display.vhd
--
-- Creates the debug image sent to the HDMI video path.
--
--
-- Display:
--   * black background
--   * cleaned mask in white
--   * tracking ROI border in green
--   * measured centre as a red '+'
--   * predicted centre as a blue 'X'
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.vision_tracker_pkg.all;

entity object_display is

    generic (
        ROI_BORDER_THICKNESS : positive := 2;
        CENTER_RADIUS        : natural := 4
    );

    port (

        clk     : in std_logic;
        reset_n : in std_logic;
        enable  : in std_logic;


        ------------------------------------------------------------------------
        -- Aligned raster from mask_cleaner.
        ------------------------------------------------------------------------
        pixel_x        : in std_logic_vector(FRAME_X_BITS - 1 downto 0);
        pixel_y        : in std_logic_vector(FRAME_Y_BITS - 1 downto 0);
        pixel_valid_in : in std_logic;

        filtered_mask : in std_logic;


        ------------------------------------------------------------------------
        -- Current tracking ROI from roi_tracker_update.
        ------------------------------------------------------------------------
        tracking_roi_x      : in std_logic_vector(FRAME_X_BITS - 1 downto 0);
        tracking_roi_y      : in std_logic_vector(FRAME_Y_BITS - 1 downto 0);
        tracking_roi_width  : in std_logic_vector(FRAME_X_BITS - 1 downto 0);
        tracking_roi_height : in std_logic_vector(FRAME_Y_BITS - 1 downto 0);


        ------------------------------------------------------------------------
        -- Measured centre from object_measurement.
        ------------------------------------------------------------------------
        measured_center_x : in std_logic_vector(FRAME_X_BITS - 1 downto 0);
        measured_center_y : in std_logic_vector(FRAME_Y_BITS - 1 downto 0);


        ------------------------------------------------------------------------
        -- Predicted centre from roi_tracker_update.
        ------------------------------------------------------------------------
        predicted_center_x     : in signed(PREDICTED_CENTER_X_BITS - 1 downto 0);
        predicted_center_y     : in signed(PREDICTED_CENTER_Y_BITS - 1 downto 0);

        tracking_enable : in std_logic;
        tracking_lost : in std_logic;

        ------------------------------------------------------------------------
        -- Debug-video output.
        ------------------------------------------------------------------------
        rgb_out         : out std_logic_vector(23 downto 0);
        pixel_valid_out : out std_logic;
        line_end_out    : out std_logic;
        frame_start_out : out std_logic

    );

end entity object_display;


architecture rtl of object_display is


    ---------------------------------------------------------------------------
    -- Measured centre marker: '+'.
    --
    -- The distances were already calculated and registered in Stage 1.
    ---------------------------------------------------------------------------
    function is_plus_marker_from_distances(
        dx     : integer;
        dy     : integer;
        radius : integer
    ) return boolean is
    begin

        return
            ((dx = 0) and (dy <= radius))
            or
            ((dy = 0) and (dx <= radius));

    end function;


    ---------------------------------------------------------------------------
    -- Predicted centre marker: 'X'.
    --
    -- The pixel belongs to the X when it is close enough to the centre and
    -- has the same horizontal and vertical distance from it.
    ---------------------------------------------------------------------------
    function is_x_marker_from_distances(
        dx     : integer;
        dy     : integer;
        radius : integer
    ) return boolean is
    begin

        return
            (dx <= radius)
            and
            (dy <= radius)
            and
            (dx = dy);

    end function;


    ---------------------------------------------------------------------------
    -- Tracking ROI border.
    --
    -- A pixel is inside the ROI when all four edge distances are non-negative.
    -- It belongs to the border when at least one distance is smaller than the
    -- configured border thickness.
    ---------------------------------------------------------------------------
    function is_roi_border_from_distances(
        left_distance   : integer;
        right_distance  : integer;
        top_distance    : integer;
        bottom_distance : integer;
        thickness       : integer
    ) return boolean is
    begin

        return
            (left_distance   >= 0)
            and
            (right_distance  >= 0)
            and
            (top_distance    >= 0)
            and
            (bottom_distance >= 0)
            and
            (
                (left_distance   < thickness)
                or
                (right_distance  < thickness)
                or
                (top_distance    < thickness)
                or
                (bottom_distance < thickness)
            );

    end function;

    ---------------------------------------------------------------------------
    -- Display colours.
    ---------------------------------------------------------------------------
    constant COLOR_BACKGROUND       : std_logic_vector(23 downto 0) := x"000000";
    constant COLOR_MASK             : std_logic_vector(23 downto 0) := x"FFFFFF";
    constant COLOR_ROI              : std_logic_vector(23 downto 0) := x"00FF00";
    constant COLOR_MEASURED_CENTER  : std_logic_vector(23 downto 0) := x"FF0000";
    constant COLOR_PREDICTED_CENTER : std_logic_vector(23 downto 0) := x"0000FF";

    ---------------------------------------------------------------------------
    -- Display pipeline - delayed pixel information.
    --
    -- These registers delay the original pixel-related information by one
    -- clock so that it remains aligned with the geometry values calculated
    -- for the same pixel.
    ---------------------------------------------------------------------------

    signal pixel_valid_reg : std_logic;

    signal filtered_mask_reg : std_logic;

    signal frame_start_reg : std_logic;
    signal line_end_reg    : std_logic;

    signal tracking_enable_reg : std_logic;
    signal tracking_lost_reg   : std_logic;


    ---------------------------------------------------------------------------
    -- Display pipeline - registered geometry results.
    --
    -- Stage 1 calculates the coordinate distances for the current pixel.
    -- These results are registered and used by Stage 2 on the following clock.
    ---------------------------------------------------------------------------

    ---------------------------------------------------------------------------
    -- Distance from the current pixel to each ROI edge.
    --
    -- Negative values mean that the pixel is outside that ROI edge.
    ---------------------------------------------------------------------------
    signal geom_roi_left_distance_reg :
        integer range -FRAME_WIDTH to FRAME_WIDTH;

    signal geom_roi_right_distance_reg :
        integer range -FRAME_WIDTH to FRAME_WIDTH;

    signal geom_roi_top_distance_reg :
        integer range -FRAME_HEIGHT to FRAME_HEIGHT;

    signal geom_roi_bottom_distance_reg :
        integer range -FRAME_HEIGHT to FRAME_HEIGHT;


    ---------------------------------------------------------------------------
    -- Absolute distance from the current pixel to the measured object centre.
    ---------------------------------------------------------------------------
    signal geom_measured_dx_reg :
        integer range 0 to FRAME_WIDTH - 1;

    signal geom_measured_dy_reg :
        integer range 0 to FRAME_HEIGHT - 1;


    ---------------------------------------------------------------------------
    -- Absolute distance from the current pixel to the predicted object centre.
    --
    -- The predicted centre may temporarily be outside the frame by up to
    -- VEL_MAX pixels.
    ---------------------------------------------------------------------------
    signal geom_predicted_dx_reg :
        integer range 0 to FRAME_WIDTH - 1 + VEL_MAX;

    signal geom_predicted_dy_reg :
        integer range 0 to FRAME_HEIGHT - 1 + VEL_MAX;

begin

    ---------------------------------------------------------------------------
    -- STAGE 1 - GEOMETRY
    --
    -- For every valid input pixel:
    --   - capture a new tracking snapshot at the beginning of a frame,
    --   - calculate the distances from the ROI and object centres,
    --   - register the geometry results,
    --   - delay the remaining pixel information by one clock.
    --
    -- The registered results are consumed by display_stage_proc.
    ---------------------------------------------------------------------------
    geometry_stage_proc : process(clk)

        -----------------------------------------------------------------------
        -- Current input pixel position.
        -----------------------------------------------------------------------
        variable pixel_x_int :
            integer range 0 to FRAME_WIDTH - 1;

        variable pixel_y_int :
            integer range 0 to FRAME_HEIGHT - 1;


        -----------------------------------------------------------------------
        -- Tracking snapshot for the current display frame.
        --
        -- These variables are updated only at pixel (0,0) and retain their
        -- values for the complete frame.
        -----------------------------------------------------------------------
        variable frame_roi_left :
            integer range 0 to FRAME_WIDTH - 1;

        variable frame_roi_right :
            integer range -1 to FRAME_WIDTH - 1;

        variable frame_roi_top :
            integer range 0 to FRAME_HEIGHT - 1;

        variable frame_roi_bottom :
            integer range -1 to FRAME_HEIGHT - 1;


        variable frame_measured_x :
            integer range 0 to FRAME_WIDTH - 1;

        variable frame_measured_y :
            integer range 0 to FRAME_HEIGHT - 1;


        variable frame_predicted_x :
            integer range -VEL_MAX to FRAME_WIDTH - 1 + VEL_MAX;

        variable frame_predicted_y :
            integer range -VEL_MAX to FRAME_HEIGHT - 1 + VEL_MAX;


        variable frame_tracking_enable : std_logic;
        variable frame_tracking_lost   : std_logic;

    begin

        if rising_edge(clk) then

            -------------------------------------------------------------------
            -- Reset.
            -------------------------------------------------------------------
            if reset_n = '0' then

                ---------------------------------------------------------------
                -- Delayed pixel information.
                ---------------------------------------------------------------
                pixel_valid_reg   <= '0';
                filtered_mask_reg <= '0';

                frame_start_reg <= '0';
                line_end_reg    <= '0';

                tracking_enable_reg <= '0';
                tracking_lost_reg   <= '0';


                ---------------------------------------------------------------
                -- Registered geometry results.
                ---------------------------------------------------------------
                geom_roi_left_distance_reg   <= 0;
                geom_roi_right_distance_reg  <= 0;
                geom_roi_top_distance_reg    <= 0;
                geom_roi_bottom_distance_reg <= 0;

                geom_measured_dx_reg <= 0;
                geom_measured_dy_reg <= 0;

                geom_predicted_dx_reg <= 0;
                geom_predicted_dy_reg <= 0;


                ---------------------------------------------------------------
                -- Current-frame tracking snapshot.
                ---------------------------------------------------------------
                frame_roi_left   := 0;
                frame_roi_right  := 0;
                frame_roi_top    := 0;
                frame_roi_bottom := 0;

                frame_measured_x := 0;
                frame_measured_y := 0;

                frame_predicted_x := 0;
                frame_predicted_y := 0;

                frame_tracking_enable := '0';
                frame_tracking_lost   := '0';


            -------------------------------------------------------------------
            -- Global pipeline enable.
            --
            -- When enable = '0', Stage 1 holds its complete state.
            -------------------------------------------------------------------
            elsif enable = '1' then

                ---------------------------------------------------------------
                -- Default values for the one-clock control pulses.
                --
                -- They are asserted below only for the corresponding valid
                -- pixel.
                ---------------------------------------------------------------
                frame_start_reg <= '0';
                line_end_reg    <= '0';


                ---------------------------------------------------------------
                -- Process the current input pixel.
                ---------------------------------------------------------------
                if pixel_valid_in = '1' then

                    -----------------------------------------------------------
                    -- Current pixel position.
                    -----------------------------------------------------------
                    pixel_x_int :=
                        to_integer(unsigned(pixel_x));

                    pixel_y_int :=
                        to_integer(unsigned(pixel_y));


                    -----------------------------------------------------------
                    -- Start of a new display frame.
                    --
                    -- Capture one coherent tracking snapshot for the complete
                    -- frame.
                    --
                    -- Since these are variables, the new snapshot values are
                    -- immediately available to the geometry calculations below.
                    -- Therefore pixel (0,0) already uses the new snapshot.
                    -----------------------------------------------------------
                    if
                        (pixel_x_int = 0)
                        and
                        (pixel_y_int = 0)
                    then

                        frame_start_reg <= '1';

                        -------------------------------------------------------
                        -- ROI snapshot.
                        --
                        -- Right and bottom are calculated once per frame rather
                        -- than once for every pixel.
                        -------------------------------------------------------
                        frame_roi_left :=
                            to_integer(unsigned(tracking_roi_x));

                        frame_roi_top :=
                            to_integer(unsigned(tracking_roi_y));

                        frame_roi_right :=
                            frame_roi_left
                            + to_integer(unsigned(tracking_roi_width))
                            - 1;

                        frame_roi_bottom :=
                            frame_roi_top
                            + to_integer(unsigned(tracking_roi_height))
                            - 1;


                        -------------------------------------------------------
                        -- Measured-centre snapshot.
                        -------------------------------------------------------
                        frame_measured_x :=
                            to_integer(unsigned(measured_center_x));

                        frame_measured_y :=
                            to_integer(unsigned(measured_center_y));


                        -------------------------------------------------------
                        -- Predicted-centre snapshot.
                        -------------------------------------------------------
                        frame_predicted_x :=
                            to_integer(predicted_center_x);

                        frame_predicted_y :=
                            to_integer(predicted_center_y);


                        -------------------------------------------------------
                        -- Tracking-state snapshot.
                        -------------------------------------------------------
                        frame_tracking_enable :=
                            tracking_enable;

                        frame_tracking_lost :=
                            tracking_lost;

                    end if;


                    -----------------------------------------------------------
                    -- End of line.
                    -----------------------------------------------------------
                    if pixel_x_int = FRAME_WIDTH - 1 then

                        line_end_reg <= '1';

                    end if;

                    -----------------------------------------------------------
                    -- ROI geometry.
                    --
                    -- Calculate the distance from the current pixel to each
                    -- ROI edge.
                    --
                    -- All four calculations execute in parallel.
                    -----------------------------------------------------------
                    geom_roi_left_distance_reg <=
                        pixel_x_int - frame_roi_left;

                    geom_roi_right_distance_reg <=
                        frame_roi_right - pixel_x_int;

                    geom_roi_top_distance_reg <=
                        pixel_y_int - frame_roi_top;

                    geom_roi_bottom_distance_reg <=
                        frame_roi_bottom - pixel_y_int;


                    -----------------------------------------------------------
                    -- Absolute distance from the measured centre.
                    -----------------------------------------------------------
                    geom_measured_dx_reg <=
                        abs(
                            pixel_x_int - frame_measured_x
                        );

                    geom_measured_dy_reg <=
                        abs(
                            pixel_y_int - frame_measured_y
                        );


                    -----------------------------------------------------------
                    -- Absolute distance from the predicted centre.
                    -----------------------------------------------------------
                    geom_predicted_dx_reg <=
                        abs(
                            pixel_x_int - frame_predicted_x
                        );

                    geom_predicted_dy_reg <=
                        abs(
                            pixel_y_int - frame_predicted_y
                        );

                end if;

                ---------------------------------------------------------------
                -- Delay the remaining information belonging to the current
                -- pixel by one clock.
                --
                -- These registers keep the original pixel information aligned
                -- with the geometry results registered above.
                --
                -- The tracking-state assignments are intentionally placed
                -- after the frame snapshot so that pixel (0,0) receives the
                -- tracking state of the new frame.
                --
                -- When pixel_valid_reg = '0', the other registered values are
                -- ignored by Stage 2.
                ---------------------------------------------------------------
                pixel_valid_reg <= pixel_valid_in;

                filtered_mask_reg <= filtered_mask;

                tracking_enable_reg <= frame_tracking_enable;

                tracking_lost_reg <= frame_tracking_lost;

            end if;

        end if;

    end process geometry_stage_proc;


    ---------------------------------------------------------------------------
    -- STAGE 2 - DISPLAY DECISION
    --
    -- Use the registered geometry and pixel information from Stage 1 to:
    --   - detect the ROI border,
    --   - detect the measured-centre '+' marker,
    --   - detect the predicted-centre 'X' marker,
    --   - select the final output colour.
    --
    -- While Stage 1 processes the current input pixel, this process operates
    -- on the previous pipeline pixel.
    ---------------------------------------------------------------------------
    display_stage_proc : process(clk)

        -----------------------------------------------------------------------
        -- Marker decisions for the registered pixel.
        -----------------------------------------------------------------------
        variable on_roi_edge      : boolean;
        variable measured_marker  : boolean;
        variable predicted_marker : boolean;

    begin

        if rising_edge(clk) then

            -------------------------------------------------------------------
            -- Reset.
            -------------------------------------------------------------------
            if reset_n = '0' then

                rgb_out <=
                    COLOR_BACKGROUND;

                pixel_valid_out <= '0';
                frame_start_out <= '0';
                line_end_out    <= '0';


            -------------------------------------------------------------------
            -- Global pipeline enable.
            --
            -- When enable = '0', Stage 2 holds its complete state together
            -- with Stage 1.
            -------------------------------------------------------------------
            elsif enable = '1' then

                ---------------------------------------------------------------
                -- Pass the registered pixel timing to the output.
                ---------------------------------------------------------------
                pixel_valid_out <= pixel_valid_reg;

                frame_start_out <= frame_start_reg;

                line_end_out <= line_end_reg;


                ---------------------------------------------------------------
                -- Default marker decisions.
                ---------------------------------------------------------------
                on_roi_edge      := false;
                measured_marker  := false;
                predicted_marker := false;


                ---------------------------------------------------------------
                -- Calculate display markers only for a valid pixel while
                -- tracking is enabled.
                ---------------------------------------------------------------
                if
                    (pixel_valid_reg = '1')
                    and
                    (tracking_enable_reg = '1')
                then

                    -----------------------------------------------------------
                    -- ROI border.
                    -----------------------------------------------------------
                    on_roi_edge :=
                        is_roi_border_from_distances(
                            geom_roi_left_distance_reg,
                            geom_roi_right_distance_reg,
                            geom_roi_top_distance_reg,
                            geom_roi_bottom_distance_reg,
                            ROI_BORDER_THICKNESS
                        );


                    -----------------------------------------------------------
                    -- Measured centre: red '+'.
                    -----------------------------------------------------------
                    measured_marker :=
                        is_plus_marker_from_distances(
                            geom_measured_dx_reg,
                            geom_measured_dy_reg,
                            CENTER_RADIUS
                        );


                    -----------------------------------------------------------
                    -- Predicted centre: blue 'X'.
                    --
                    -- The predicted marker is hidden while tracking is lost.
                    -----------------------------------------------------------
                    if tracking_lost_reg = '0' then

                        predicted_marker :=
                            is_x_marker_from_distances(
                                geom_predicted_dx_reg,
                                geom_predicted_dy_reg,
                                CENTER_RADIUS
                            );

                    end if;

                end if;


                ---------------------------------------------------------------
                -- Final colour selection.
                --
                -- Priority:
                --
                --   predicted centre
                --          >
                --   measured centre
                --          >
                --   ROI border
                --          >
                --   cleaned mask
                --          >
                --   background
                ---------------------------------------------------------------
                if pixel_valid_reg = '1' then

                    if predicted_marker then

                        rgb_out <= COLOR_PREDICTED_CENTER;


                    elsif measured_marker then

                        rgb_out <= COLOR_MEASURED_CENTER;


                    elsif on_roi_edge then

                        rgb_out <= COLOR_ROI;


                    elsif filtered_mask_reg = '1' then

                        rgb_out <= COLOR_MASK;

                    else

                        rgb_out <= COLOR_BACKGROUND;

                    end if;

                end if;

            end if;

        end if;

    end process display_stage_proc;
    
end architecture rtl;