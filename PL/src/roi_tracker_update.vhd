--------------------------------------------------------------------------------
-- roi_tracker_update.vhd
--
-- Maintains the tracking ROI inside the PL.
--
-- A completed frame causes one of two actions:
--
--   1. ACCEPT
--      Enough matching pixels were found.
--      The module:
--        - calculates the object's raw velocity,
--        - smooths the velocity,
--        - predicts the next object centre,
--        - publishes the predicted centre,
--        - builds a tight ROI around the predicted position.
--
--   2. LOSS
--      Not enough matching pixels were found.
--      The current ROI is already positioned around the last prediction, so
--      the module does not perform another full prediction without a new
--      measurement. Instead, it expands the current ROI, with more growth in
--      the last known direction of motion.
--
-- ACCEPT and LOSS processing are pipelined across multiple clock cycles.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.vision_tracker_pkg.all;

entity roi_tracker_update is
    generic (

        -- ROI padding during normal tracking.
        PAD_BASE : integer := 24;
        PAD_MAX  : integer := 96;
        
        -- Maximum ROI size during normal tracking.
        MAX_TRACK_ROI_W : positive := 260;
        MAX_TRACK_ROI_H : positive := 180;

        -- Minimum matching-pixel requirement:
        -- max(COUNT_FLOOR, previous_object_area / 2**COUNT_SHIFT)
        COUNT_FLOOR : integer := 150;
        COUNT_SHIFT : integer := 3;

        -- ROI expansion after a missed frame.
        LOSS_BASE  : integer := 10;
        LOSS_RAMP  : integer := 8;
        LOSS_MIN   : integer := 8;
        LOSS_MAX   : integer := 40;
        MAX_LOSS_W : integer := 300;
        MAX_LOSS_H : integer := 200
    );
    port (
        clk     : in std_logic;
        reset_n : in std_logic;  -- synchronous, active low
        enable : in std_logic;
        
        tracking_enable : in std_logic;

        -- Initial ROI supplied by the PS.
        initial_roi_x      : in std_logic_vector(FRAME_X_BITS - 1 downto 0);
        initial_roi_y      : in std_logic_vector(FRAME_Y_BITS - 1 downto 0);
        initial_roi_width  : in std_logic_vector(FRAME_X_BITS - 1 downto 0);
        initial_roi_height : in std_logic_vector(FRAME_Y_BITS - 1 downto 0);

        -- Measurements produced for the completed frame.
        object_center_x    : in std_logic_vector(FRAME_X_BITS - 1 downto 0);
        object_center_y    : in std_logic_vector(FRAME_Y_BITS - 1 downto 0);
        object_width       : in std_logic_vector(FRAME_X_BITS - 1 downto 0);
        object_height      : in std_logic_vector(FRAME_Y_BITS - 1 downto 0);
        object_pixel_count : in std_logic_vector(PIXEL_COUNT_BITS - 1 downto 0);

        -- One-clock pulse from object_measurement.
        frame_measurement_done : in std_logic;
        
        -- One enabled-clock pulse when the tracker has finished processing
        -- the completed-frame measurement and all tracking outputs are updated. 
        tracker_update_done : out std_logic;

        -- Predicted object centre.
        -- Valid for one clock when predicted_center_valid = '1'.
        -- Signed because the prediction may temporarily fall outside the frame.
        predicted_center_x     : out signed(PREDICTED_CENTER_X_BITS - 1 downto 0);
        predicted_center_y     : out signed(PREDICTED_CENTER_Y_BITS - 1 downto 0);
        predicted_center_valid : out std_logic;

        -- Current tracking ROI.
        tracking_roi_x      : out std_logic_vector(FRAME_X_BITS - 1 downto 0);
        tracking_roi_y      : out std_logic_vector(FRAME_Y_BITS - 1 downto 0);
        tracking_roi_width  : out std_logic_vector(FRAME_X_BITS - 1 downto 0);
        tracking_roi_height : out std_logic_vector(FRAME_Y_BITS - 1 downto 0);
        
        tracking_lost : out std_logic
    );
end entity roi_tracker_update;

architecture rtl of roi_tracker_update is

    ---------------------------------------------------------------------------
    -- Helper: clamp an integer to a legal range.
    ---------------------------------------------------------------------------
    function clamp_integer(
        value       : integer;
        lower_bound : integer;
        upper_bound : integer
    ) return integer is
    begin
        if value < lower_bound then
            return lower_bound;
        elsif value > upper_bound then
            return upper_bound;
        else
            return value;
        end if;
    end function;

   

    ---------------------------------------------------------------------------
    -- Calculate the minimum number of matching pixels required for the next
    -- frame to be accepted.
    --
    -- threshold = max(
    --     COUNT_FLOOR,
    --     object_area / 2**COUNT_SHIFT
    -- )
    ---------------------------------------------------------------------------
    function calculate_min_accepted_pixels(
        measured_width  : integer;
        measured_height : integer
    ) return integer is
        variable object_area : integer;
        variable threshold   : integer;

    begin
        object_area := measured_width * measured_height;

        threshold :=
            object_area / (2 ** COUNT_SHIFT);

        if threshold < COUNT_FLOOR then
            return COUNT_FLOOR;
        else
            return threshold;
        end if;
    end function;


    ---------------------------------------------------------------------------
    -- Calculate the requested ROI expansion for one axis during LOSS.
    --
    -- This stage depends only on:
    --   - the last known smoothed velocity,
    --   - the accumulated LOSS ramp.
    --
    -- It does NOT consider the current ROI size or MAX_LOSS yet.
    -- That limitation is performed later in LOSS_EXPAND.
    --
    -- negative side:
    --   X -> left
    --   Y -> up
    --
    -- positive side:
    --   X -> right
    --   Y -> down
    ---------------------------------------------------------------------------
    procedure calculate_requested_loss_axis(
        previous_smoothed_velocity : in integer;
        loss_ramp_pro                  : in integer;

        signal requested_negative : out integer;
        signal requested_positive : out integer
    ) is
        variable main_growth      : integer;
        variable secondary_growth : integer;

    begin

        -----------------------------------------------------------------------
        -- Motion toward the positive side:
        -- X -> right
        -- Y -> down
        -----------------------------------------------------------------------
        if previous_smoothed_velocity > 0 then

            main_growth := clamp_integer(
                (abs(previous_smoothed_velocity) / 4)
                    + LOSS_BASE
                    + loss_ramp_pro,
                LOSS_MIN,
                LOSS_MAX
            );

            secondary_growth := clamp_integer(
                main_growth / 2,
                LOSS_MIN,
                LOSS_MAX
            );

            requested_negative <= secondary_growth;
            requested_positive <= main_growth;

        -----------------------------------------------------------------------
        -- Motion toward the negative side:
        -- X -> left
        -- Y -> up
        -----------------------------------------------------------------------
        elsif previous_smoothed_velocity < 0 then

            main_growth := clamp_integer(
                (abs(previous_smoothed_velocity) / 4)
                    + LOSS_BASE
                    + loss_ramp_pro,
                LOSS_MIN,
                LOSS_MAX
            );

            secondary_growth := clamp_integer(
                main_growth / 2,
                LOSS_MIN,
                LOSS_MAX
            );

            requested_negative <= main_growth;
            requested_positive <= secondary_growth;

        -----------------------------------------------------------------------
        -- No known direction.
        -- Request equal expansion on both sides.
        -----------------------------------------------------------------------
        else

            main_growth := clamp_integer(
                LOSS_BASE + loss_ramp_pro,
                LOSS_MIN,
                LOSS_MAX
            );

            requested_negative <= main_growth;
            requested_positive <= main_growth;

        end if;

    end procedure;

    ---------------------------------------------------------------------------
    -- Current ROI
    ---------------------------------------------------------------------------
    signal roi_x_reg      : integer range 0 to FRAME_WIDTH - 1;
    signal roi_y_reg      : integer range 0 to FRAME_HEIGHT - 1;
    signal roi_width_reg  : integer range 1 to FRAME_WIDTH;
    signal roi_height_reg : integer range 1 to FRAME_HEIGHT;

    ---------------------------------------------------------------------------
    -- Tracking history
    ---------------------------------------------------------------------------
    signal previous_center_x : integer range 0 to FRAME_WIDTH - 1;
    signal previous_center_y : integer range 0 to FRAME_HEIGHT - 1;

    signal previous_smoothed_velocity_x :
        integer range -VEL_MAX to VEL_MAX;

    signal previous_smoothed_velocity_y :
        integer range -VEL_MAX to VEL_MAX;

    signal previous_center_valid : std_logic;


    ---------------------------------------------------------------------------
    -- Skip the first completed measurement after tracking is enabled.
    --
    -- The first measurement may belong to a frame that was already being
    -- processed while the PS updated the ROI / object thresholds.
    -- Starting from the next measurement, the tracker uses only measurements
    -- produced after the handoff frame has been discarded.
    ---------------------------------------------------------------------------
    signal skip_first_measurement : std_logic;

    ---------------------------------------------------------------------------
    -- Adaptive state
    ---------------------------------------------------------------------------
    signal min_accepted_pixels :
        integer range 0 to FRAME_WIDTH * FRAME_HEIGHT;

    signal loss_ramp_pixels :
        integer range 0 to LOSS_MAX;
        
        
    ---------------------------------------------------------------------------
    -- Tracker state machine
    ---------------------------------------------------------------------------
    type tracker_state_t is (
        TRACKER_IDLE,
        
        ACCEPT_VELOCITY,
        ACCEPT_PREDICT,
        ACCEPT_ROI,
        ACCEPT_COMMIT,
        
        LOSS_EXPAND,
        LOSS_COMMIT
    );

    signal tracker_state : tracker_state_t;

    ---------------------------------------------------------------------------
    -- ACCEPT pipeline registers.
    ---------------------------------------------------------------------------
    signal accept_center_x :
        integer range 0 to FRAME_WIDTH - 1;

    signal accept_center_y :
        integer range 0 to FRAME_HEIGHT - 1;

    signal accept_width :
        integer range 1 to FRAME_WIDTH;

    signal accept_height :
        integer range 1 to FRAME_HEIGHT;

    ---------------------------------------------------------------------------
    -- Smoothed velocity.
    ---------------------------------------------------------------------------
    signal accept_smoothed_velocity_x :
        integer range -VEL_MAX to VEL_MAX;

    signal accept_smoothed_velocity_y :
        integer range -VEL_MAX to VEL_MAX;

    ---------------------------------------------------------------------------
    -- Predicted centre.
    ---------------------------------------------------------------------------
    signal accept_predicted_x :
        integer range -VEL_MAX to FRAME_WIDTH - 1 + VEL_MAX;

    signal accept_predicted_y :
        integer range -VEL_MAX to FRAME_HEIGHT - 1 + VEL_MAX;

    ---------------------------------------------------------------------------
    -- New ROI size.
    ---------------------------------------------------------------------------
    signal accept_roi_width :
        integer range 1 to FRAME_WIDTH;

    signal accept_roi_height :
        integer range 1 to FRAME_HEIGHT;
        
    signal accept_padding_x :
        integer range 0 to PAD_MAX;

    signal accept_padding_y :
        integer range 0 to PAD_MAX;    
    
    ---------------------------------------------------------------------------
    -- LOSS pipeline - requested expansion.
    --
    -- These values describe how much the tracker would LIKE to expand in each
    -- direction according to velocity and loss ramp.
    --
    -- They are calculated in TRACKER_IDLE and registered before LOSS_EXPAND.
    ---------------------------------------------------------------------------
    signal loss_requested_left :
        integer range 0 to LOSS_MAX;

    signal loss_requested_right :
        integer range 0 to LOSS_MAX;

    signal loss_requested_up :
        integer range 0 to LOSS_MAX;

    signal loss_requested_down :
        integer range 0 to LOSS_MAX;


    ---------------------------------------------------------------------------
    -- LOSS pipeline - remaining expansion room.
    --
    -- Store how much additional width / height is still allowed before
    -- reaching MAX_LOSS_W / MAX_LOSS_H.
    --
    -- These values are calculated in TRACKER_IDLE and registered before
    -- LOSS_EXPAND, breaking the previous long combinational LOSS path.
    ---------------------------------------------------------------------------
    signal loss_remaining_width :
        integer range 0 to MAX_LOSS_W;

    signal loss_remaining_height :
        integer range 0 to MAX_LOSS_H;


    ---------------------------------------------------------------------------
    -- LOSS pipeline - final expansion.
    --
    -- LOSS_EXPAND converts the requested expansion into the actual expansion
    -- that fits inside MAX_LOSS_W / MAX_LOSS_H.
    --
    -- LOSS_COMMIT only needs:
    --   - how far the ROI moves left/up,
    --   - the final ROI width/height.
    ---------------------------------------------------------------------------
    signal loss_grow_left :
        integer range 0 to LOSS_MAX;

    signal loss_grow_up :
        integer range 0 to LOSS_MAX;

    signal loss_expanded_width :
        integer range 1 to FRAME_WIDTH;

    signal loss_expanded_height :
        integer range 1 to FRAME_HEIGHT;
        
        
begin

    ---------------------------------------------------------------------------
    -- Main tracker process
    ---------------------------------------------------------------------------
    tracker_proc : process(clk)

        -----------------------------------------------------------------------
        -- ACCEPT stage-local variables
        -----------------------------------------------------------------------
        variable raw_velocity_x :
            integer range -VEL_MAX to VEL_MAX;

        variable raw_velocity_y :
            integer range -VEL_MAX to VEL_MAX;

        -----------------------------------------------------------------------
        -- ROI position calculation variables
        -----------------------------------------------------------------------
        variable new_roi_x : integer;
        variable new_roi_y : integer;

        -----------------------------------------------------------------------
        -- Initial ROI variables
        -----------------------------------------------------------------------
        variable initial_roi_width_int  : integer range 1 to FRAME_WIDTH;
        variable initial_roi_height_int : integer range 1 to FRAME_HEIGHT;

    begin
        if rising_edge(clk) then

            -------------------------------------------------------------------
            -- Reset
            -------------------------------------------------------------------
            if reset_n = '0' then

                roi_x_reg      <= 0;
                roi_y_reg      <= 0;
                roi_width_reg  <= FRAME_WIDTH;
                roi_height_reg <= FRAME_HEIGHT;

                previous_center_x <= 0;
                previous_center_y <= 0;

                previous_smoothed_velocity_x <= 0;
                previous_smoothed_velocity_y <= 0;

                previous_center_valid <= '0';

                min_accepted_pixels <= COUNT_FLOOR;
                loss_ramp_pixels    <= 0;

                predicted_center_x <= (others => '0');
                predicted_center_y <= (others => '0');
                
                tracker_state <= TRACKER_IDLE;

                accept_center_x <= 0;
                accept_center_y <= 0;
                
                accept_width  <= 1;
                accept_height <= 1;
                
                accept_smoothed_velocity_x <= 0;
                accept_smoothed_velocity_y <= 0;
                
                accept_predicted_x <= 0;
                accept_predicted_y <= 0;
                
                accept_roi_width  <= 1;
                accept_roi_height <= 1;


                accept_padding_x <= 0;
                accept_padding_y <= 0;
                
                ----------------------------------------------------------------
                -- LOSS pipeline registers.
                ----------------------------------------------------------------

                -- Requested expansion.
                loss_requested_left  <= 0;
                loss_requested_right <= 0;
                loss_requested_up    <= 0;
                loss_requested_down  <= 0;

                -- Remaining expansion room stored between LOSS stages.
                loss_remaining_width  <= 0;
                loss_remaining_height <= 0;

                -- Final expansion used by LOSS_COMMIT.
                loss_grow_left <= 0;
                loss_grow_up   <= 0;

                loss_expanded_width  <= 1;
                loss_expanded_height <= 1;           
                tracking_lost <= '0';
                
                predicted_center_valid <= '0';
                tracker_update_done    <= '0';
                
                -- The first measurement after tracking starts must be ignored.
                skip_first_measurement <= '1';
              
              
             elsif enable = '0' then
    
            -- Global pipeline pause.
            -- Hold all tracker state and output-valid pulses unchanged.
            null;
              
            else  
                -------------------------------------------------------------------
                -- Default pulses while the tracker is allowed to advance.
                -------------------------------------------------------------------
                predicted_center_valid <= '0';
                tracker_update_done    <= '0';
                -------------------------------------------------------------------
                -- Tracking disabled: load the initial ROI from the PS and clear motion history.
                -------------------------------------------------------------------
                if tracking_enable = '0' then
    
                    tracker_state <= TRACKER_IDLE;
                                
                    -- The next completed measurement may belong to the handoff frame,
                    -- so it will be ignored when tracking starts.
                    skip_first_measurement <= '1';
                    
                    tracking_lost <= '0';
                    
                    previous_center_x <= 0;
                    previous_center_y <= 0;
    
                    previous_smoothed_velocity_x <= 0;
                    previous_smoothed_velocity_y <= 0;
    
                    previous_center_valid <= '0';
    
                    min_accepted_pixels <= COUNT_FLOOR;
                    loss_ramp_pixels    <= 0;
    
                    predicted_center_x <= (others => '0');
                    predicted_center_y <= (others => '0');
    
                    if (unsigned(initial_roi_width) = 0) or (unsigned(initial_roi_height) = 0) then
    
                        roi_x_reg      <= 0;
                        roi_y_reg      <= 0;
                        roi_width_reg  <= FRAME_WIDTH;
                        roi_height_reg <= FRAME_HEIGHT;
    
                    else
                        initial_roi_width_int := clamp_integer(
                            to_integer(unsigned(initial_roi_width)),
                            1,
                            FRAME_WIDTH
                        );
    
                        initial_roi_height_int := clamp_integer(
                            to_integer(unsigned(initial_roi_height)),
                            1,
                            FRAME_HEIGHT
                        );
    
                        roi_x_reg <= clamp_integer(
                            to_integer(unsigned(initial_roi_x)),
                            0,
                            FRAME_WIDTH - initial_roi_width_int
                        );
    
                        roi_y_reg <= clamp_integer(
                            to_integer(unsigned(initial_roi_y)),
                            0,
                            FRAME_HEIGHT - initial_roi_height_int
                        );
    
                        roi_width_reg  <= initial_roi_width_int;
                        roi_height_reg <= initial_roi_height_int;
                    end if;
                    
                -------------------------------------------------------------------
                -- Tracking active
                -------------------------------------------------------------------
                else
    
                    case tracker_state is
    
                        -----------------------------------------------------------
                        -- IDLE
                        --
                        -- Wait for a completed-frame measurement.
                        -----------------------------------------------------------
                        when TRACKER_IDLE =>
    
                            if frame_measurement_done = '1' then
    
                                ---------------------------------------------------
                                -- Ignore the first measurement after tracking
                                -- becomes enabled.
                                ---------------------------------------------------
                                if skip_first_measurement = '1' then
    
                                    skip_first_measurement <= '0';
    
                                    -- The measurement was intentionally discarded,
                                    -- but processing of this frame is complete.
                                    tracker_update_done <= '1';
    
                                ---------------------------------------------------
                                -- ACCEPT
                                ---------------------------------------------------
                                elsif
                                    to_integer(unsigned(object_pixel_count))>= min_accepted_pixels
                                then
        
                                    ------------------------------------------------
                                    -- Store the accepted measurement.
                                    --
                                    -- These registers hold the measurement while
                                    -- the ACCEPT calculation progresses through
                                    -- the following clock cycles.
                                    ------------------------------------------------
                                    accept_center_x <= clamp_integer(
                                        to_integer(unsigned(object_center_x)),
                                        0,
                                        FRAME_WIDTH - 1
                                    );
    
                                    accept_center_y <= clamp_integer(
                                        to_integer(unsigned(object_center_y)),
                                        0,
                                        FRAME_HEIGHT - 1
                                    );
    
                                    accept_width <= clamp_integer(
                                        to_integer(unsigned(object_width)),
                                        1,
                                        FRAME_WIDTH
                                    );
    
                                    accept_height <= clamp_integer(
                                        to_integer(unsigned(object_height)),
                                        1,
                                        FRAME_HEIGHT
                                    );
    
                                    tracker_state <= ACCEPT_VELOCITY;
    
                                ---------------------------------------------------
                                -- LOSS
                                ---------------------------------------------------
                                else
    
                                    tracking_lost <= '1';
    
                                    if previous_center_valid = '1' and
                                        (
                                            (roi_width_reg < MAX_LOSS_W)
                                            or
                                            (roi_height_reg < MAX_LOSS_H)
                                        )
                                    then
                                        ------------------------------------------------
                                        -- LOSS stage 1
                                        --
                                        -- Calculate how much we would LIKE to expand
                                        -- according to the last valid velocity and the
                                        -- accumulated loss ramp.
                                        --
                                        -- The MAX_LOSS limitation is intentionally
                                        -- performed later in LOSS_EXPAND.
                                        ------------------------------------------------

                                        ------------------------------------------------
                                        -- X axis requested expansion.
                                        --
                                        -- negative = left
                                        -- positive = right
                                        ------------------------------------------------
                                        calculate_requested_loss_axis(
                                        
                                            previous_smoothed_velocity =>
                                                previous_smoothed_velocity_x,

                                            loss_ramp_pro =>
                                                loss_ramp_pixels,

                                            requested_negative =>
                                                loss_requested_left,

                                            requested_positive =>
                                                loss_requested_right
                                        );


                                        ------------------------------------------------
                                        -- Y axis requested expansion.
                                        --
                                        -- negative = up
                                        -- positive = down
                                        ------------------------------------------------
                                        calculate_requested_loss_axis(
                                        
                                            previous_smoothed_velocity =>
                                                previous_smoothed_velocity_y,

                                            loss_ramp_pro =>
                                                loss_ramp_pixels,

                                            requested_negative =>
                                                 loss_requested_up,

                                            requested_positive =>
                                                loss_requested_down
                                        );


                                        ------------------------------------------------
                                        -- Store how much additional growth is still
                                        -- allowed before reaching MAX_LOSS.
                                        --
                                        -- One axis may already be at its limit while
                                        -- the other axis can still expand.
                                        ------------------------------------------------
                                        if roi_width_reg < MAX_LOSS_W then

                                            loss_remaining_width <=
                                                MAX_LOSS_W - roi_width_reg;

                                        else

                                            loss_remaining_width <= 0;

                                        end if;


                                        if roi_height_reg < MAX_LOSS_H then

                                            loss_remaining_height <=
                                                MAX_LOSS_H - roi_height_reg;

                                        else

                                            loss_remaining_height <= 0;

                                        end if;


                                        ------------------------------------------------
                                        -- Continue LOSS processing on the next clock.
                                        ------------------------------------------------
                                        tracker_state <= LOSS_EXPAND;   
                                
                                    else
                                
                                        ------------------------------------------------
                                        -- ROI cannot be expanded any further.
                                        ------------------------------------------------
                                        tracker_update_done <= '1';
    
                                    end if;
                                    
                                end if;
    
                            end if;
    
    
                        -----------------------------------------------------------
                        -- VELOCITY
                        --
                        -- Calculate raw movement and smooth it with the previous
                        -- accepted velocity.
                        -----------------------------------------------------------
                        when ACCEPT_VELOCITY =>
    
                            if previous_center_valid = '1' then
    
                                raw_velocity_x := clamp_integer(
                                    accept_center_x - previous_center_x,
                                    -VEL_MAX,
                                    VEL_MAX
                                );
    
                                raw_velocity_y := clamp_integer(
                                    accept_center_y - previous_center_y,
                                    -VEL_MAX,
                                    VEL_MAX
                                );
    
                            else
    
                                raw_velocity_x := 0;
                                raw_velocity_y := 0;
    
                            end if;
    
                            accept_smoothed_velocity_x <=
                                (raw_velocity_x + previous_smoothed_velocity_x)/ 2;
    
                            accept_smoothed_velocity_y <=
                                (raw_velocity_y + previous_smoothed_velocity_y)/ 2;
    
                            tracker_state <= ACCEPT_PREDICT;
    
    
                        -----------------------------------------------------------
                        -- PREDICT
                        --
                        -- Predict the object centre for the next frame.
                        -----------------------------------------------------------
                        when ACCEPT_PREDICT =>
    
                            accept_predicted_x <=
                                accept_center_x + accept_smoothed_velocity_x;
    
                            accept_predicted_y <=
                                accept_center_y + accept_smoothed_velocity_y;
    
                            accept_padding_x <= clamp_integer(
                                PAD_BASE + abs(accept_smoothed_velocity_x) / 2,
                                0,
                                PAD_MAX
                            );
    
                            accept_padding_y <= clamp_integer(
                                PAD_BASE + abs(accept_smoothed_velocity_y) / 2,
                                0,
                                PAD_MAX
                            );
                            
                            tracker_state <= ACCEPT_ROI;
    
    
                        -----------------------------------------------------------
                        -- ROI
                        --
                        -- Calculate speed-dependent padding and the new ROI size.
                        -----------------------------------------------------------
                        when ACCEPT_ROI =>
    
                            accept_roi_width <= clamp_integer(
                                accept_width + 2 * accept_padding_x,
                                1,
                                MAX_TRACK_ROI_W
                            );
    
                            accept_roi_height <= clamp_integer(
                                accept_height + 2 * accept_padding_y,
                                1,
                                MAX_TRACK_ROI_H
                            );
    
                            tracker_state <= ACCEPT_COMMIT;
    
    
                        -----------------------------------------------------------
                        -- ACCEPT_COMMIT
                        --
                        -- Position the ROI around the prediction and publish all
                        -- completed tracking state atomically.
                        -----------------------------------------------------------
                        when ACCEPT_COMMIT =>
                        
                            tracking_lost <= '0';

                            new_roi_x := clamp_integer(
                                accept_predicted_x - accept_roi_width / 2,
                                0,
                                FRAME_WIDTH - accept_roi_width
                            );
    
                            new_roi_y := clamp_integer(
                                accept_predicted_y - accept_roi_height / 2,
                                0,
                                FRAME_HEIGHT - accept_roi_height
                            );
    
                            -------------------------------------------------------
                            -- Store the new ROI.
                            -------------------------------------------------------
                            roi_x_reg      <= new_roi_x;
                            roi_y_reg      <= new_roi_y;
                            roi_width_reg  <= accept_roi_width;
                            roi_height_reg <= accept_roi_height;
    
                            -------------------------------------------------------
                            -- Publish the predicted centre.
                            -------------------------------------------------------
                            predicted_center_x <= to_signed(
                                accept_predicted_x,
                                predicted_center_x'length
                            );
    
                            predicted_center_y <= to_signed(
                                accept_predicted_y,
                                predicted_center_y'length
                            );
    
                            predicted_center_valid <= '1';
    
                            -------------------------------------------------------
                            -- Save accepted tracking history.
                            -------------------------------------------------------
                            previous_center_x <= accept_center_x;
                            previous_center_y <= accept_center_y;
    
                            previous_smoothed_velocity_x <=
                                accept_smoothed_velocity_x;
    
                            previous_smoothed_velocity_y <=
                                accept_smoothed_velocity_y;
    
                            previous_center_valid <= '1';
    
                            -------------------------------------------------------
                            -- Threshold for the next completed frame.
                            -------------------------------------------------------
                            min_accepted_pixels <=
                                calculate_min_accepted_pixels(
                                    accept_width,
                                    accept_height
                                );
    
                            loss_ramp_pixels <= 0;
    
                            -------------------------------------------------------
                            -- ACCEPT is now completely finished.
                            -------------------------------------------------------
                            tracker_update_done <= '1';
                            tracker_state        <= TRACKER_IDLE;
                              
                        -----------------------------------------------------------
                        -- LOSS_EXPAND
                        --
                        -- Limit the requested ROI expansion according to the
                        -- remaining room before MAX_LOSS_W / MAX_LOSS_H.
                        --
                        -- The requested growth and remaining room were registered
                        -- in TRACKER_IDLE, so this calculation is performed in a
                        -- separate clock cycle.
                        -----------------------------------------------------------
                        when LOSS_EXPAND =>

                            -------------------------------------------------------
                            -- X AXIS - final ROI width.
                            --
                            -- If the full requested expansion fits, use all of it.
                            -- Otherwise, expand only by the remaining room up to
                            -- MAX_LOSS_W.
                            -------------------------------------------------------
                            loss_expanded_width <=
                                roi_width_reg + clamp_integer(
                                    loss_requested_left + loss_requested_right,
                                    0,
                                    loss_remaining_width
                                );


                            -------------------------------------------------------
                            -- X AXIS - actual LEFT expansion.
                            -------------------------------------------------------

                            -------------------------------------------------------
                            -- Moving right:
                            -- RIGHT receives priority.
                            --
                            -- LEFT may use only the room that remains after the
                            -- requested RIGHT expansion.
                            -------------------------------------------------------
                            if previous_smoothed_velocity_x > 0 then

                                loss_grow_left <=
                                    clamp_integer(
                                        loss_remaining_width - loss_requested_right,
                                        0,
                                        loss_requested_left
                                    );


                            -------------------------------------------------------
                            -- Moving left:
                            -- LEFT receives priority.
                            -------------------------------------------------------
                            elsif previous_smoothed_velocity_x < 0 then

                                loss_grow_left <=
                                    clamp_integer(
                                        loss_requested_left,
                                        0,
                                        loss_remaining_width
                                    );


                            -------------------------------------------------------
                            -- No known horizontal direction.
                            --
                            -- Expand approximately symmetrically.
                            -- LEFT may use at most half of the remaining room.
                            -------------------------------------------------------
                            else

                                loss_grow_left <=
                                    clamp_integer(
                                        loss_requested_left,
                                        0,
                                        loss_remaining_width / 2
                                    );

                            end if;


                            -------------------------------------------------------
                            -- Y AXIS - final ROI height.
                            -------------------------------------------------------
                            loss_expanded_height <=
                                roi_height_reg + clamp_integer(
                                    loss_requested_up + loss_requested_down,
                                    0,
                                    loss_remaining_height
                                );


                            -------------------------------------------------------
                            -- Y AXIS - actual UP expansion.
                            -------------------------------------------------------

                            -------------------------------------------------------
                            -- Moving down:
                            -- DOWN receives priority.
                            --
                            -- UP may use only the room that remains after the
                            -- requested DOWN expansion.
                            -------------------------------------------------------
                            if previous_smoothed_velocity_y > 0 then

                                loss_grow_up <=
                                    clamp_integer(
                                        loss_remaining_height - loss_requested_down,
                                        0,
                                        loss_requested_up
                                    );


                            -------------------------------------------------------
                            -- Moving up:
                            -- UP receives priority.
                            -------------------------------------------------------
                            elsif previous_smoothed_velocity_y < 0 then

                                loss_grow_up <=
                                    clamp_integer(
                                        loss_requested_up,
                                        0,
                                        loss_remaining_height
                                    );


                            -------------------------------------------------------
                            -- No known vertical direction.
                            --
                            -- Expand approximately symmetrically.
                            -- UP may use at most half of the remaining room.
                            -------------------------------------------------------
                            else

                                loss_grow_up <=
                                    clamp_integer(
                                        loss_requested_up,
                                        0,
                                        loss_remaining_height / 2
                                    );

                            end if;


                            -------------------------------------------------------
                            -- The final expansion values are now registered.
                            -- LOSS_COMMIT will apply them to the ROI on the next
                            -- enabled clock.
                            -------------------------------------------------------
                            tracker_state <= LOSS_COMMIT;      
                        -----------------------------------------------------------
                        -- LOSS_COMMIT
                        --
                        -- Use the registered LOSS expansion values to calculate
                        -- and publish the new ROI.
                        -----------------------------------------------------------
                        when LOSS_COMMIT =>
                        
                            new_roi_x :=
                                roi_x_reg - loss_grow_left;
                        
                            new_roi_y :=
                                roi_y_reg - loss_grow_up;
                        
                            roi_x_reg <= clamp_integer(
                                new_roi_x,
                                0,
                                FRAME_WIDTH - loss_expanded_width
                            );
                        
                            roi_y_reg <= clamp_integer(
                                new_roi_y,
                                0,
                                FRAME_HEIGHT - loss_expanded_height
                            );
                        
                            roi_width_reg  <= loss_expanded_width;
                            roi_height_reg <= loss_expanded_height;
                        
                            loss_ramp_pixels <= clamp_integer(
                                loss_ramp_pixels + LOSS_RAMP,
                                0,
                                LOSS_MAX
                            );
                        
                            -------------------------------------------------------
                            -- LOSS processing is now complete.
                            -------------------------------------------------------
                            tracker_update_done <= '1';
                            tracker_state        <= TRACKER_IDLE;  
                    end case;
    
                end if;
                
            end if;
            
        end if;    
    end process tracker_proc;

    ---------------------------------------------------------------------------
    -- tracking_roi Outputs
    ---------------------------------------------------------------------------
    tracking_roi_x <= std_logic_vector(
        to_unsigned(roi_x_reg, tracking_roi_x'length)
    );

    tracking_roi_y <= std_logic_vector(
        to_unsigned(roi_y_reg, tracking_roi_y'length)
    );

    tracking_roi_width <= std_logic_vector(
        to_unsigned(roi_width_reg, tracking_roi_width'length)
    );

    tracking_roi_height <= std_logic_vector(
        to_unsigned(roi_height_reg, tracking_roi_height'length)
    );

end architecture rtl;
