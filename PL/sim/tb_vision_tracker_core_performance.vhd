--------------------------------------------------------------------------------
-- tb_vision_tracker_core_performance.vhd
--
-- Performance testbench for vision_tracker_logic_top.
--
-- Measures:
--   1. Datapath throughput over one complete input frame.
--   2. Video-path latency: accepted input SOF -> corresponding output SOF.
--   3. Tracking-result latency: last accepted input pixel -> new snapshot.
--   4. Frame-to-frame interval and internal boundary overhead.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;
use std.textio.all;

use work.vision_tracker_pkg.all;


entity tb_vision_tracker_core_performance is
end entity;


architecture sim of tb_vision_tracker_core_performance is

    ---------------------------------------------------------------------------
    -- Clock
    ---------------------------------------------------------------------------
    constant CLK_PERIOD : time := 10 ns;

    -- Derived from CLK_PERIOD.
    constant CLK_FREQ_HZ : real :=
        1.0e9 / real(CLK_PERIOD / 1 ns);

    ---------------------------------------------------------------------------
    -- DUT inputs
    ---------------------------------------------------------------------------
    signal clk     : std_logic := '0';
    signal reset_n : std_logic := '0';
    signal enable  : std_logic := '1';

    signal pixel_valid_in : std_logic := '0';
    signal new_frame_in   : std_logic := '0';
    signal last_line_in   : std_logic := '0';

    signal r_in : std_logic_vector(7 downto 0) := x"60";
    signal g_in : std_logic_vector(7 downto 0) := x"A0";
    signal b_in : std_logic_vector(7 downto 0) := x"40";

    signal tracking_enable : std_logic := '0';

    signal initial_roi_x :
        std_logic_vector(FRAME_X_BITS - 1 downto 0) :=
            (others => '0');

    signal initial_roi_y :
        std_logic_vector(FRAME_Y_BITS - 1 downto 0) :=
            (others => '0');

    signal initial_roi_width :
        std_logic_vector(FRAME_X_BITS - 1 downto 0) :=
            std_logic_vector(to_unsigned(FRAME_WIDTH, FRAME_X_BITS));

    signal initial_roi_height :
        std_logic_vector(FRAME_Y_BITS - 1 downto 0) :=
            std_logic_vector(to_unsigned(FRAME_HEIGHT, FRAME_Y_BITS));

    -- Full Cr/Cb range so every in-ROI pixel passes color classification.
    signal cr_min : std_logic_vector(7 downto 0) := x"00";
    signal cr_max : std_logic_vector(7 downto 0) := x"FF";
    signal cb_min : std_logic_vector(7 downto 0) := x"00";
    signal cb_max : std_logic_vector(7 downto 0) := x"FF";


    ---------------------------------------------------------------------------
    -- DUT tracking outputs
    ---------------------------------------------------------------------------
    signal object_center_x :
        std_logic_vector(FRAME_X_BITS - 1 downto 0);

    signal object_center_y :
        std_logic_vector(FRAME_Y_BITS - 1 downto 0);

    signal object_width :
        std_logic_vector(FRAME_X_BITS - 1 downto 0);

    signal object_height :
        std_logic_vector(FRAME_Y_BITS - 1 downto 0);

    signal object_pixel_count :
        std_logic_vector(PIXEL_COUNT_BITS - 1 downto 0);

    signal tracking_roi_x :
        std_logic_vector(FRAME_X_BITS - 1 downto 0);

    signal tracking_roi_y :
        std_logic_vector(FRAME_Y_BITS - 1 downto 0);

    signal tracking_roi_width :
        std_logic_vector(FRAME_X_BITS - 1 downto 0);

    signal tracking_roi_height :
        std_logic_vector(FRAME_Y_BITS - 1 downto 0);

    signal tracking_lost : std_logic;

    signal predicted_center_x :
        signed(PREDICTED_CENTER_X_BITS - 1 downto 0);

    signal predicted_center_y :
        signed(PREDICTED_CENTER_Y_BITS - 1 downto 0);

    signal tracking_snapshot_counter :
        std_logic_vector(31 downto 0);


    ---------------------------------------------------------------------------
    -- DUT video outputs
    ---------------------------------------------------------------------------
    signal debug_rgb_out :
        std_logic_vector(23 downto 0);

    signal debug_pixel_valid_out : std_logic;
    signal debug_line_end_out    : std_logic;
    signal debug_frame_start_out : std_logic;

    signal pixel_ready_out : std_logic;


    ---------------------------------------------------------------------------
    -- Measured values
    ---------------------------------------------------------------------------
    signal perf_input_pixels                 : integer := -1;
    signal perf_input_active_cycles          : integer := -1;

    signal perf_video_latency_cycles         : integer := -1;
    signal perf_tracking_latency_cycles      : integer := -1;
    signal perf_frame_start_interval_cycles  : integer := -1;


    ---------------------------------------------------------------------------
    -- Formatting helper
    -- Input is in milli-units: 325521 -> "325.521"
    ---------------------------------------------------------------------------
    function fixed3(milli_value : integer) return string is

        variable whole_part : integer;
        variable frac_part  : integer;

    begin

        whole_part := milli_value / 1000;
        frac_part  := abs(milli_value rem 1000);

        if frac_part < 10 then
            return integer'image(whole_part)
                   & ".00"
                   & integer'image(frac_part);

        elsif frac_part < 100 then
            return integer'image(whole_part)
                   & ".0"
                   & integer'image(frac_part);

        else
            return integer'image(whole_part)
                   & "."
                   & integer'image(frac_part);
        end if;

    end function;


begin

    ---------------------------------------------------------------------------
    -- Clock
    ---------------------------------------------------------------------------
    clk <= not clk after CLK_PERIOD / 2;


    ---------------------------------------------------------------------------
    -- DUT
    ---------------------------------------------------------------------------
    DUT : entity work.vision_tracker_logic_top
        port map (

            clk     => clk,
            reset_n => reset_n,
            enable  => enable,

            pixel_valid_in => pixel_valid_in,
            new_frame_in   => new_frame_in,
            last_line_in   => last_line_in,

            r_in => r_in,
            g_in => g_in,
            b_in => b_in,

            tracking_enable => tracking_enable,

            initial_roi_x      => initial_roi_x,
            initial_roi_y      => initial_roi_y,
            initial_roi_width  => initial_roi_width,
            initial_roi_height => initial_roi_height,

            cr_min => cr_min,
            cr_max => cr_max,
            cb_min => cb_min,
            cb_max => cb_max,

            object_center_x    => object_center_x,
            object_center_y    => object_center_y,
            object_width       => object_width,
            object_height      => object_height,
            object_pixel_count => object_pixel_count,

            tracking_roi_x      => tracking_roi_x,
            tracking_roi_y      => tracking_roi_y,
            tracking_roi_width  => tracking_roi_width,
            tracking_roi_height => tracking_roi_height,

            tracking_lost => tracking_lost,

            predicted_center_x => predicted_center_x,
            predicted_center_y => predicted_center_y,

            tracking_snapshot_counter =>
                tracking_snapshot_counter,

            debug_rgb_out         => debug_rgb_out,
            debug_pixel_valid_out => debug_pixel_valid_out,
            debug_line_end_out    => debug_line_end_out,
            debug_frame_start_out => debug_frame_start_out,

            pixel_ready_out => pixel_ready_out
        );


    ---------------------------------------------------------------------------
    -- Stimulus
    ---------------------------------------------------------------------------
    stimulus_proc : process

        procedure send_one_frame is
        begin

            for y in 0 to FRAME_HEIGHT - 1 loop

                for x in 0 to FRAME_WIDTH - 1 loop

                    pixel_valid_in <= '1';

                    if (x = 0) and (y = 0) then
                        new_frame_in <= '1';
                    else
                        new_frame_in <= '0';
                    end if;

                    if x = FRAME_WIDTH - 1 then
                        last_line_in <= '1';
                    else
                        last_line_in <= '0';
                    end if;

                    -- Hold the current pixel until it is accepted.
                    loop
                        wait until rising_edge(clk);
                        exit when pixel_ready_out = '1';
                    end loop;

                end loop;

            end loop;

            pixel_valid_in <= '0';
            new_frame_in   <= '0';
            last_line_in   <= '0';

        end procedure;


        procedure send_next_frame_sof_only is
        begin

            pixel_valid_in <= '1';
            new_frame_in   <= '1';
            last_line_in   <= '0';

            -- Hold the next-frame SOF until accepted.
            loop
                wait until rising_edge(clk);
                exit when pixel_ready_out = '1';
            end loop;

            pixel_valid_in <= '0';
            new_frame_in   <= '0';
            last_line_in   <= '0';

        end procedure;


        -----------------------------------------------------------------------
        -- Report variables
        -----------------------------------------------------------------------
        variable L : line;

        variable clock_mhz_milli :
            integer;

        variable input_pixels_per_clock_milli :
            integer;

        variable input_mpixel_per_second_milli :
            integer;

        variable frame_equivalent_rate_milli :
            integer;

        variable sustained_frame_rate_milli :
            integer;

        variable efficiency_milli :
            integer;

        variable boundary_overhead :
            integer;

        variable video_latency_us_milli :
            integer;

        variable tracking_latency_us_milli :
            integer;


        procedure print_line(constant s : in string) is
        begin
            write(L, s);
            writeline(output, L);
        end procedure;


    begin

        -----------------------------------------------------------------------
        -- Reset
        -----------------------------------------------------------------------
        reset_n <= '0';
        tracking_enable <= '0';

        pixel_valid_in <= '0';
        new_frame_in   <= '0';
        last_line_in   <= '0';

        wait for 10 * CLK_PERIOD;

        wait until falling_edge(clk);
        reset_n <= '1';

        wait for 4 * CLK_PERIOD;

        wait until falling_edge(clk);
        tracking_enable <= '1';

        wait for 4 * CLK_PERIOD;


        report "Performance simulation running..." severity note;


        -----------------------------------------------------------------------
        -- Complete measured frame
        -----------------------------------------------------------------------
        send_one_frame;


        -----------------------------------------------------------------------
        -- Only the SOF of the next frame is needed for frame-interval timing.
        -----------------------------------------------------------------------
        send_next_frame_sof_only;


        -----------------------------------------------------------------------
        -- Allow the final tracking result and monitor updates to complete
        -- before printing the report.   
     -----------------------------------------------------------------------

        wait for 1 us;


        -----------------------------------------------------------------------
        -- Calculate all displayed performance values from measured values.
        -----------------------------------------------------------------------
        clock_mhz_milli :=
            integer(
                (CLK_FREQ_HZ / 1.0e6) * 1000.0
            );


        if
            (perf_input_pixels > 0)
            and
            (perf_input_active_cycles > 0)
        then

            input_pixels_per_clock_milli :=
                integer(
                    (
                        real(perf_input_pixels)
                        / real(perf_input_active_cycles)
                    )
                    * 1000.0
                );

            input_mpixel_per_second_milli :=
                integer(
                    (
                        real(perf_input_pixels)
                        / real(perf_input_active_cycles)
                    )
                    * (CLK_FREQ_HZ / 1.0e6)
                    * 1000.0
                );

            frame_equivalent_rate_milli :=
                integer(
                    (
                        (
                            real(perf_input_pixels)
                            / real(perf_input_active_cycles)
                        )
                        * CLK_FREQ_HZ
                        / real(FRAME_PIXEL_COUNT)
                    )
                    * 1000.0
                );

        else

            input_pixels_per_clock_milli := -1;
            input_mpixel_per_second_milli := -1;
            frame_equivalent_rate_milli := -1;

        end if;


        if perf_video_latency_cycles >= 0 then

            -- fixed3 expects milli-microseconds.
            -- 1 milli-us = 1 ns = 1000 ps.
            video_latency_us_milli :=
                (
                    perf_video_latency_cycles
                    * integer(CLK_PERIOD / 1 ps)
                )
                / 1000;

        else

            video_latency_us_milli := -1;

        end if;


        if perf_tracking_latency_cycles >= 0 then

            tracking_latency_us_milli :=
                (
                    perf_tracking_latency_cycles
                    * integer(CLK_PERIOD / 1 ps)
                )
                / 1000;

        else

            tracking_latency_us_milli := -1;

        end if;


        if
            (perf_frame_start_interval_cycles > 0)
            and
            (perf_input_active_cycles > 0)
        then

            boundary_overhead :=
                perf_frame_start_interval_cycles
                - perf_input_active_cycles;

            sustained_frame_rate_milli :=
                integer(
                    (
                        CLK_FREQ_HZ
                        / real(perf_frame_start_interval_cycles)
                    )
                    * 1000.0
                );

            efficiency_milli :=
                integer(
                    (
                        real(perf_input_active_cycles)
                        / real(perf_frame_start_interval_cycles)
                    )
                    * 100000.0
                );

        else

            boundary_overhead := -1;
            sustained_frame_rate_milli := -1;
            efficiency_milli := -1;

        end if;


        -----------------------------------------------------------------------
        -- Final report
        -----------------------------------------------------------------------
        print_line("");
        print_line("======================================================================");
        print_line("             VISION TRACKER IP - PERFORMANCE RESULTS");
        print_line("======================================================================");
        print_line("");

        print_line("CONFIGURATION");

        print_line(
            "  Clock frequency                      : "
            & fixed3(clock_mhz_milli)
            & " MHz"
        );

        print_line(
            "  Frame size                           : "
            & integer'image(FRAME_WIDTH)
            & " x "
            & integer'image(FRAME_HEIGHT)
        );

        print_line(
            "  Pixels per frame                     : "
            & integer'image(FRAME_PIXEL_COUNT)
        );

        print_line("");


        print_line("THROUGHPUT");

        if
            (perf_input_pixels = FRAME_PIXEL_COUNT)
            and
            (perf_input_active_cycles = perf_input_pixels)
        then

            print_line(
                "  Measured datapath rate               : "
                & fixed3(input_pixels_per_clock_milli)
                & " pixel/clock  [PASS]"
            );

        else

            print_line(
                "  Measured input rate                  : FAILED"
            );

        end if;


        if input_mpixel_per_second_milli >= 0 then

            print_line(
                "  Pixel throughput                     : "
                & fixed3(input_mpixel_per_second_milli)
                & " MPixel/s"
            );

        end if;


        if frame_equivalent_rate_milli >= 0 then

            print_line(
                "  Frame-equivalent rate                : "
                & fixed3(frame_equivalent_rate_milli)
                & " FPS"
            );

        end if;

        print_line("");


        print_line("LATENCY");

        if perf_video_latency_cycles >= 0 then

            print_line(
                "  Video path                           : "
                & integer'image(perf_video_latency_cycles)
                & " clocks = "
                & fixed3(video_latency_us_milli)
                & " us"
            );

        else

            print_line(
                "  Video path                           : NOT MEASURED"
            );

        end if;


        if perf_tracking_latency_cycles >= 0 then

            print_line(
                "  Tracking result                      : "
                & integer'image(perf_tracking_latency_cycles)
                & " clocks = "
                & fixed3(tracking_latency_us_milli)
                & " us"
            );

        else

            print_line(
                "  Tracking result                      : NOT MEASURED"
            );

        end if;

        print_line("");


        print_line("FRAME-TO-FRAME PERFORMANCE");

        if perf_frame_start_interval_cycles > 0 then

            print_line(
                "  Frame-start interval                 : "
                & integer'image(perf_frame_start_interval_cycles)
                & " clocks"
            );

            print_line(
                "  Internal frame-boundary overhead     : "
                & integer'image(boundary_overhead)
                & " clocks"
            );

            print_line(
                "  Max core frame rate                  : "
                & fixed3(sustained_frame_rate_milli)
                & " FPS"
            );

            print_line(
                "  Datapath efficiency                  : "
                & fixed3(efficiency_milli)
                & " %"
            );

        else

            print_line(
                "  Frame-to-frame measurement           : NOT MEASURED"
            );

        end if;

        print_line("");
        print_line("======================================================================");
        print_line("");


        stop;
        wait;

    end process stimulus_proc;


    ---------------------------------------------------------------------------
    -- Performance monitor
    ---------------------------------------------------------------------------
    monitor_proc : process

        variable cycle_now :
            integer;

        variable frame_number :
            integer := 0;

        variable accepted_pixels_this_frame :
            integer := 0;

        variable first_frame_sof_cycle :
            integer := -1;

        variable first_frame_last_pixel_cycle :
            integer := -1;

        variable second_frame_sof_cycle :
            integer := -1;


        variable first_output_sof_seen :
            boolean := false;

        variable last_snapshot :
            unsigned(31 downto 0) := (others => '0');

        variable first_snapshot_seen :
            boolean := false;


        variable active_cycles :
            integer;

        variable video_latency_cycles :
            integer;

        variable tracking_latency_cycles :
            integer;

        variable frame_start_interval_cycles :
            integer;

    begin

        loop

            -------------------------------------------------------------------
            -- Sample input handshake exactly at the active clock edge.
            -------------------------------------------------------------------
            wait until rising_edge(clk);

            cycle_now :=
                integer(now / CLK_PERIOD);


            if reset_n = '0' then

                frame_number := 0;
                accepted_pixels_this_frame := 0;

                first_frame_sof_cycle := -1;
                first_frame_last_pixel_cycle := -1;
                second_frame_sof_cycle := -1;

                first_output_sof_seen := false;

                last_snapshot := (others => '0');
                first_snapshot_seen := false;


            else

                ----------------------------------------------------------------
                -- For this throughput test, any backpressure inside an active frame
                -- is treated as a failure.
                -- A blocked SOF is allowed while the DUT completes frame-boundary processing.
                ----------------------------------------------------------------
                if
                    (pixel_valid_in = '1')
                    and
                    (pixel_ready_out = '0')
                    and
                    (new_frame_in = '0')
                then

                    assert false
                        report
                            "FAIL: unexpected backpressure inside active frame."
                        severity failure;

                end if;


                ----------------------------------------------------------------
                -- Accepted input pixel
                ----------------------------------------------------------------
                if
                    (pixel_valid_in = '1')
                    and
                    (pixel_ready_out = '1')
                then

                    ------------------------------------------------------------
                    -- Start of accepted frame
                    ------------------------------------------------------------
                    if new_frame_in = '1' then

                        frame_number :=
                            frame_number + 1;

                        accepted_pixels_this_frame := 0;


                        if frame_number = 1 then

                            first_frame_sof_cycle :=
                                cycle_now;


                        elsif frame_number = 2 then

                            second_frame_sof_cycle :=
                                cycle_now;

                            frame_start_interval_cycles :=
                                second_frame_sof_cycle
                                - first_frame_sof_cycle;

                            perf_frame_start_interval_cycles <=
                                frame_start_interval_cycles;

                        end if;

                    end if;


                    accepted_pixels_this_frame :=
                        accepted_pixels_this_frame + 1;


                    ------------------------------------------------------------
                    -- End of complete Frame 1
                    ------------------------------------------------------------
                    if
                        (frame_number = 1)
                        and
                        (accepted_pixels_this_frame = FRAME_PIXEL_COUNT)
                    then

                        first_frame_last_pixel_cycle :=
                            cycle_now;

                        active_cycles :=
                            first_frame_last_pixel_cycle
                            - first_frame_sof_cycle
                            + 1;

                        perf_input_pixels <=
                            accepted_pixels_this_frame;

                        perf_input_active_cycles <=
                            active_cycles;


                        assert
                            accepted_pixels_this_frame =
                                FRAME_PIXEL_COUNT
                        report
                            "FAIL: input frame pixel count is incorrect."
                        severity failure;


                        assert
                            active_cycles =
                                accepted_pixels_this_frame
                        report
                            "FAIL: input did not sustain one accepted pixel per clock."
                        severity failure;

                    end if;

                end if;


                ----------------------------------------------------------------
                -- Let registered DUT outputs produced by this edge settle.
                ----------------------------------------------------------------
                wait for 1 ps;


                ----------------------------------------------------------------
                -- First output SOF -> video-path latency.
                ----------------------------------------------------------------
                if
                    (not first_output_sof_seen)
                    and
                    (debug_pixel_valid_out = '1')
                    and
                    (debug_frame_start_out = '1')
                    and
                    (first_frame_sof_cycle >= 0)
                then

                    first_output_sof_seen :=
                        true;

                    video_latency_cycles :=
                        cycle_now
                        - first_frame_sof_cycle;

                    perf_video_latency_cycles <=
                        video_latency_cycles;

                end if;


                ----------------------------------------------------------------
                -- First tracking snapshot update observed after Frame 1.
                ----------------------------------------------------------------
                if
                    unsigned(tracking_snapshot_counter)
                    /= last_snapshot
                then

                    if
                        (not first_snapshot_seen)
                        and
                        (first_frame_last_pixel_cycle >= 0)
                    then

                        first_snapshot_seen :=
                            true;

                        tracking_latency_cycles :=
                            cycle_now
                            - first_frame_last_pixel_cycle;

                        perf_tracking_latency_cycles <=
                            tracking_latency_cycles;

                    end if;


                    last_snapshot :=
                        unsigned(tracking_snapshot_counter);

                end if;

            end if;

        end loop;

    end process monitor_proc;


end architecture sim;
