--------------------------------------------------------------------------------
-- vision_tracker_logic_top.vhd
--
-- Structural top level for the vision-tracking logic.
--
-- Processing pipeline:
--
--   RGB input
--      |
--      v
--   rgb_to_ycrcb
--      |
--      v
--   roi_mask_generator
--      |
--      v
--   mask_cleaner
--      |-------------------------> object_display
--      |
--      v
--   object_measurement
--      |
--      v
--   roi_tracker_update
--      |-------------------------> object_display
--      |
--      v
--   PS result snapshot
--
-- All PS-facing tracking outputs are captured together after roi_tracker_update
-- has processed the completed-frame measurement.
--
-- tracking_snapshot_counter increments whenever a new coherent result snapshot
-- is published, allowing the PS to detect new tracking data.
--
-- Frame dimensions and all resolution-dependent interface widths come from
-- vision_tracker_pkg. Algorithm-specific tuning parameters are explicitly
-- mapped at the relevant component instantiations below.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.vision_tracker_pkg.all;


entity vision_tracker_logic_top is

    port (

        ------------------------------------------------------------------------
        -- Clock / reset / global processing enable
        ------------------------------------------------------------------------
        clk     : in std_logic;
        reset_n : in std_logic;
        enable  : in std_logic;


        ------------------------------------------------------------------------
        -- Incoming RGB video stream
        ------------------------------------------------------------------------
        pixel_valid_in : in std_logic;
        new_frame_in   : in std_logic;
        last_line_in   : in std_logic;

        r_in : in std_logic_vector(7 downto 0);
        g_in : in std_logic_vector(7 downto 0);
        b_in : in std_logic_vector(7 downto 0);


        ------------------------------------------------------------------------
        -- Tracking control from the PS
        ------------------------------------------------------------------------
        tracking_enable : in std_logic;


        ------------------------------------------------------------------------
        -- Initial ROI supplied by the PS
        ------------------------------------------------------------------------
        initial_roi_x      : in std_logic_vector(FRAME_X_BITS - 1 downto 0);
        initial_roi_y      : in std_logic_vector(FRAME_Y_BITS - 1 downto 0);
        initial_roi_width  : in std_logic_vector(FRAME_X_BITS - 1 downto 0);
        initial_roi_height : in std_logic_vector(FRAME_Y_BITS - 1 downto 0);


        ------------------------------------------------------------------------
        -- Cr/Cb classification thresholds supplied by the PS
        ------------------------------------------------------------------------
        cr_min : in std_logic_vector(7 downto 0);
        cr_max : in std_logic_vector(7 downto 0);
        cb_min : in std_logic_vector(7 downto 0);
        cb_max : in std_logic_vector(7 downto 0);


        ------------------------------------------------------------------------
        -- Coherent tracking result snapshot for the PS
        --
        -- All outputs in this section are updated together.
        -- tracking_snapshot_counter increments whenever a new snapshot is available.
        ------------------------------------------------------------------------
        object_center_x    : out std_logic_vector(FRAME_X_BITS - 1 downto 0);
        object_center_y    : out std_logic_vector(FRAME_Y_BITS - 1 downto 0);
        object_width       : out std_logic_vector(FRAME_X_BITS - 1 downto 0);
        object_height      : out std_logic_vector(FRAME_Y_BITS - 1 downto 0);
        object_pixel_count : out std_logic_vector(PIXEL_COUNT_BITS - 1 downto 0);

        tracking_roi_x      : out std_logic_vector(FRAME_X_BITS - 1 downto 0);
        tracking_roi_y      : out std_logic_vector(FRAME_Y_BITS - 1 downto 0);
        tracking_roi_width  : out std_logic_vector(FRAME_X_BITS - 1 downto 0);
        tracking_roi_height : out std_logic_vector(FRAME_Y_BITS - 1 downto 0);

        tracking_lost : out std_logic;

        predicted_center_x     : out signed(PREDICTED_CENTER_X_BITS - 1 downto 0);
        predicted_center_y     : out signed(PREDICTED_CENTER_Y_BITS - 1 downto 0);

        ------------------------------------------------------------------------
        -- Tracking snapshot counter.
        --
        -- Incremented whenever a new coherent tracking snapshot is published.
        -- The PS can detect new tracking data by checking whether this value changed.
        ------------------------------------------------------------------------
        tracking_snapshot_counter : out std_logic_vector(31 downto 0);


        ------------------------------------------------------------------------
        -- Debug video stream for the HDMI path
        ------------------------------------------------------------------------
        debug_rgb_out         : out std_logic_vector(23 downto 0);
        debug_pixel_valid_out : out std_logic;
        debug_line_end_out    : out std_logic;
        debug_frame_start_out : out std_logic;

        pixel_ready_out : out std_logic
    );

end entity vision_tracker_logic_top;


architecture structural of vision_tracker_logic_top is


    ---------------------------------------------------------------------------
    -- rgb_to_ycrcb -> roi_mask_generator
    ---------------------------------------------------------------------------
    signal ycrcb_cr : std_logic_vector(7 downto 0);
    signal ycrcb_cb : std_logic_vector(7 downto 0);

    signal ycrcb_pixel_valid : std_logic;
    signal ycrcb_new_frame   : std_logic;
    signal ycrcb_last_line   : std_logic;


    ---------------------------------------------------------------------------
    -- roi_mask_generator -> mask_cleaner
    ---------------------------------------------------------------------------
    signal raw_mask : std_logic;

    signal raw_mask_x : std_logic_vector(FRAME_X_BITS - 1 downto 0);
    signal raw_mask_y : std_logic_vector(FRAME_Y_BITS - 1 downto 0);

    signal raw_mask_valid : std_logic;
    signal raw_frame_end  : std_logic;


    ---------------------------------------------------------------------------
    -- mask_cleaner -> object_measurement / object_display
    ---------------------------------------------------------------------------
    signal cleaned_mask : std_logic;

    signal cleaned_pixel_x : std_logic_vector(FRAME_X_BITS - 1 downto 0);
    signal cleaned_pixel_y : std_logic_vector(FRAME_Y_BITS - 1 downto 0);

    signal cleaned_stream_valid : std_logic;
    signal cleaned_mask_valid   : std_logic;
    signal cleaned_filtered_frame_end : std_logic;

    ---------------------------------------------------------------------------
    -- object_measurement -> roi_tracker_update / result snapshot / display
    ---------------------------------------------------------------------------
    signal measured_center_x : std_logic_vector(FRAME_X_BITS - 1 downto 0);
    signal measured_center_y : std_logic_vector(FRAME_Y_BITS - 1 downto 0);

    signal measured_width  : std_logic_vector(FRAME_X_BITS - 1 downto 0);
    signal measured_height : std_logic_vector(FRAME_Y_BITS - 1 downto 0);

    signal measured_pixel_count :
        std_logic_vector(PIXEL_COUNT_BITS - 1 downto 0);

    signal measurement_done : std_logic;


    ---------------------------------------------------------------------------
    -- roi_tracker_update -> result snapshot / object_display
    ---------------------------------------------------------------------------
    signal current_roi_x      : std_logic_vector(FRAME_X_BITS - 1 downto 0);
    signal current_roi_y      : std_logic_vector(FRAME_Y_BITS - 1 downto 0);
    signal current_roi_width  : std_logic_vector(FRAME_X_BITS - 1 downto 0);
    signal current_roi_height : std_logic_vector(FRAME_Y_BITS - 1 downto 0);

    signal predicted_x :
        signed(PREDICTED_CENTER_X_BITS - 1 downto 0);

    signal predicted_y :
        signed(PREDICTED_CENTER_Y_BITS - 1 downto 0);

    signal tracker_lost     : std_logic;

    signal tracker_update_done : std_logic;

    signal snapshot_counter_reg :
        unsigned(31 downto 0) := (others => '0');



    ---------------------------------------------------------------------------
    -- Input frame synchronization
    --
    -- result_ready remembers whether the previous frame tracking result is ready.
    -- tmp_pixel_ready_out blocks only the start of a new frame when needed.
    -- gated_pixel_valid prevents blocked pixels from entering the pipeline.
    ---------------------------------------------------------------------------
    signal result_ready : std_logic := '1';
    signal compute_enable    : std_logic;
    signal tmp_pixel_ready_out : std_logic;
    signal gated_pixel_valid : std_logic;
begin

    ---------------------------------------------------------------------------
    -- Tracking completion enable
    --
    -- Normally the tracking logic follows the video-pipeline enable.
    -- However, once a frame result is pending, the logic required to finish
    -- that result must continue running even if downstream video backpressure
    -- temporarily pauses the pixel pipeline.
    ---------------------------------------------------------------------------
    compute_enable <=
        '1' when (result_ready = '0')
        else enable;
    
    ---------------------------------------------------------------------------
    -- Track whether the previous frame result is ready.
    ---------------------------------------------------------------------------
    result_ready_proc : process(clk)
    begin
        if rising_edge(clk) then

            if reset_n = '0' then

                result_ready <= '1';
            elsif compute_enable = '0' then
            
                -- Hold only when there is no pending tracking result.
                null;
                
            elsif tracking_enable = '0' then

                result_ready <= '1';

            -------------------------------------------------------------------
            -- A new frame was accepted, so wait for its tracking result.
            -------------------------------------------------------------------
            elsif (gated_pixel_valid = '1') and (new_frame_in = '1') then

                result_ready <= '0';
            -------------------------------------------------------------------
            -- Tracking result is ready, so the next frame may start.
            -------------------------------------------------------------------
            elsif tracker_update_done = '1' then

                result_ready <= '1';

            end if;

        end if;
    end process result_ready_proc;

    ---------------------------------------------------------------------------
    -- Block a new frame while waiting for the previous tracking result.
    ---------------------------------------------------------------------------
    tmp_pixel_ready_out <=
         '0'
         when
         (enable = '0')
         or
         (
        (tracking_enable = '1')
         and
         (new_frame_in = '1')
         and
         (result_ready = '0')
        )
         else
                           '1';


    pixel_ready_out<=tmp_pixel_ready_out;
    gated_pixel_valid <= pixel_valid_in and tmp_pixel_ready_out;



    ---------------------------------------------------------------------------
    -- RGB -> YCrCb
    ---------------------------------------------------------------------------
    rgb_to_ycrcb_inst : entity work.rgb_to_ycrcb
        port map (
            clk     => clk,
            reset_n => reset_n,
            enable  => enable,

            -------------------------------------------------------------------
            -- Only accept the input pixel when the upstream source is valid
            -- and the frame-synchronization logic allows it to enter.
            -------------------------------------------------------------------
            pixel_valid_in =>  gated_pixel_valid,


            new_frame_in   => new_frame_in,
            last_line_in   => last_line_in,

            R_in => r_in,
            G_in => g_in,
            B_in => b_in,

            y_out  => open,
            cr_out => ycrcb_cr,
            cb_out => ycrcb_cb,

            pixel_valid_out => ycrcb_pixel_valid,
            new_frame_out   => ycrcb_new_frame,
            last_line_out   => ycrcb_last_line
        );


    ---------------------------------------------------------------------------
    -- ROI + Cr/Cb classification
    ---------------------------------------------------------------------------
    roi_mask_generator_inst : entity work.roi_mask_generator
        port map (
            clk     => clk,
            reset_n => reset_n,
            enable  => enable,

            pixel_valid => ycrcb_pixel_valid,

            pixel_cr => ycrcb_cr,
            pixel_cb => ycrcb_cb,

            last_line => ycrcb_last_line,
            new_frame => ycrcb_new_frame,

            current_roi_x => current_roi_x,
            current_roi_y => current_roi_y,
            current_roi_w => current_roi_width,
            current_roi_h => current_roi_height,

            cr_min => cr_min,
            cr_max => cr_max,
            cb_min => cb_min,
            cb_max => cb_max,

            pixel_x_out => raw_mask_x,
            pixel_y_out => raw_mask_y,

            pixel_match  => raw_mask,
            result_valid => raw_mask_valid,

            frame_end_out => raw_frame_end
        );


    ---------------------------------------------------------------------------
    -- Majority-filter mask cleaning and raster alignment
    ---------------------------------------------------------------------------
    mask_cleaner_inst : entity work.mask_cleaner
        generic map (
            KERNEL_SIZE => 5
       )
        port map (
            clk     => clk,
            reset_n => reset_n,
            enable  => enable,

            mask_in => raw_mask,

            pixel_x => raw_mask_x,
            pixel_y => raw_mask_y,

            stream_pixel_valid_in => raw_mask_valid,

            frame_end_in => raw_frame_end,

            filtered_mask => cleaned_mask,

            pixel_x_out => cleaned_pixel_x,
            pixel_y_out => cleaned_pixel_y,

            stream_pixel_valid_out  => cleaned_stream_valid,
            filtered_mask_valid_out => cleaned_mask_valid,

            frame_end_out => open,
            filtered_frame_end_out => cleaned_filtered_frame_end

        );


    ---------------------------------------------------------------------------
    -- Measure the cleaned object mask once per aligned output frame
    ---------------------------------------------------------------------------
    object_measurement_inst : entity work.object_measurement
        port map (
            clk     => clk,
            reset_n => reset_n,
            enable  => enable,

            mask_in => cleaned_mask,

            pixel_x => cleaned_pixel_x,
            pixel_y => cleaned_pixel_y,

            filtered_mask_valid_in => cleaned_mask_valid,
            frame_end_in            => cleaned_filtered_frame_end,

            centroid_x => measured_center_x,
            centroid_y => measured_center_y,

            bbox_width  => measured_width,
            bbox_height => measured_height,

            object_pixel_count => measured_pixel_count,

            frame_measurement_done => measurement_done
        );


    ---------------------------------------------------------------------------
    -- Update the tracking ROI and predict the next centre
    ---------------------------------------------------------------------------
    roi_tracker_update_inst : entity work.roi_tracker_update
        generic map (
        
            -- ROI padding during normal tracking.
            PAD_BASE    => 8,
            PAD_MAX     => 40,

            -- Maximum ROI size during normal tracking.
            MAX_TRACK_ROI_W => 200,
            MAX_TRACK_ROI_H => 200,
        
            -- Minimum matching-pixel requirement:
            -- max(COUNT_FLOOR, previous_object_area / 2**COUNT_SHIFT)
            COUNT_FLOOR => 150,
            COUNT_SHIFT => 3,

            -- ROI expansion after a missed frame.
            LOSS_BASE   => 10,
            LOSS_RAMP   => 5,
            LOSS_MIN    => 5,
            LOSS_MAX    => 40,

            -- Maximum ROI size while tracking is lost.
            -- These values must not exceed FRAME_WIDTH / FRAME_HEIGHT
            -- and should be selected according to the application.
            MAX_LOSS_W  => 240,
            MAX_LOSS_H  => 240
        )
        port map (
            clk     => clk,
            reset_n => reset_n,
            enable=> compute_enable,
            
            tracking_enable => tracking_enable,

            initial_roi_x      => initial_roi_x,
            initial_roi_y      => initial_roi_y,
            initial_roi_width  => initial_roi_width,
            initial_roi_height => initial_roi_height,

            object_center_x    => measured_center_x,
            object_center_y    => measured_center_y,
            object_width       => measured_width,
            object_height      => measured_height,
            object_pixel_count => measured_pixel_count,

            frame_measurement_done => measurement_done,
            tracker_update_done    => tracker_update_done,

            predicted_center_x     => predicted_x,
            predicted_center_y     => predicted_y,
            predicted_center_valid => open,

            tracking_roi_x      => current_roi_x,
            tracking_roi_y      => current_roi_y,
            tracking_roi_width  => current_roi_width,
            tracking_roi_height => current_roi_height,

            tracking_lost => tracker_lost
        );





    ---------------------------------------------------------------------------
    -- Debug display
    --
    -- object_display receives the live tracking results directly and captures
    -- one internal snapshot at the beginning of each displayed frame.
    --
    -- Therefore, the ROI and centre markers remain constant throughout the
    -- complete display frame even if the tracker updates during that frame.
    ---------------------------------------------------------------------------
    object_display_inst : entity work.object_display
        generic map (
        ROI_BORDER_THICKNESS => 2,
        CENTER_RADIUS        => 4
         )
        port map (
            clk     => clk,
            reset_n => reset_n,
            enable  => enable,

            pixel_x        => cleaned_pixel_x,
            pixel_y        => cleaned_pixel_y,
            pixel_valid_in => cleaned_stream_valid,

            filtered_mask => cleaned_mask,

            tracking_roi_x      => current_roi_x,
            tracking_roi_y      => current_roi_y,
            tracking_roi_width  => current_roi_width,
            tracking_roi_height => current_roi_height,

            measured_center_x => measured_center_x,
            measured_center_y => measured_center_y,

            predicted_center_x => predicted_x,
            predicted_center_y => predicted_y,

            tracking_enable => tracking_enable,
            tracking_lost   => tracker_lost,

            rgb_out         => debug_rgb_out,
            pixel_valid_out => debug_pixel_valid_out,
            line_end_out    => debug_line_end_out,
            frame_start_out => debug_frame_start_out
        );


    ---------------------------------------------------------------------------
    -- PS result snapshot
    --
    -- All PS-facing tracking outputs are updated on the same clock.
    ---------------------------------------------------------------------------
    result_snapshot_proc : process(clk)
    begin
        if rising_edge(clk) then

            if reset_n = '0' then

                snapshot_counter_reg <= (others => '0');

                object_center_x    <= (others => '0');
                object_center_y    <= (others => '0');
                object_width       <= (others => '0');
                object_height      <= (others => '0');
                object_pixel_count <= (others => '0');

                tracking_roi_x      <= (others => '0');
                tracking_roi_y      <= (others => '0');
                tracking_roi_width  <= (others => '0');
                tracking_roi_height <= (others => '0');

                tracking_lost <= '0';

                predicted_center_x     <= (others => '0');
                predicted_center_y     <= (others => '0');

           elsif compute_enable = '1' then

                ----------------------------------------------------------------
                -- Capture one coherent result only after roi_tracker_update
                -- has completely processed the completed-frame measurement.
                ----------------------------------------------------------------
                if tracker_update_done = '1' then


                    object_center_x    <= measured_center_x;
                    object_center_y    <= measured_center_y;
                    object_width       <= measured_width;
                    object_height      <= measured_height;
                    object_pixel_count <= measured_pixel_count;

                    tracking_roi_x      <= current_roi_x;
                    tracking_roi_y      <= current_roi_y;
                    tracking_roi_width  <= current_roi_width;
                    tracking_roi_height <= current_roi_height;

                    tracking_lost <= tracker_lost;

                    predicted_center_x     <= predicted_x;
                    predicted_center_y     <= predicted_y;

                    snapshot_counter_reg <= snapshot_counter_reg + 1;

                end if;

            end if;

        end if;
    end process result_snapshot_proc;

    tracking_snapshot_counter <=
        std_logic_vector(snapshot_counter_reg);

end architecture structural;
