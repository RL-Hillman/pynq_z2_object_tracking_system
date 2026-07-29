--------------------------------------------------------------------------------
-- two_axis_servo_logic_top.vhd
--
-- Servo half of the PYNQ-Z2 hand-tracker PL core (IP block 2 of 2; the vision
-- half is vision_tracker_logic_top). NOT the AXI-Lite top: all control inputs
-- are plain ports, to be wired to the AXI-Lite register layer later. Pure,
-- vendor-IP-free, synthesizable logic, ready to be packaged as one custom
-- Vivado IP — and reusable as a generic two-axis servo driver.
--
--   error_x -> pan servo_controller  -> pwm -> servo_pwm_pan
--   error_y -> tilt servo_controller -> pwm -> servo_pwm_tilt
--     (the search sweep is pan-only: the tilt controller has its sweep input
--      tied '0', so during SEARCH the tilt axis re-centres and stays level)
--
-- error_x / error_y are the LIVE measurement errors (signed, pixels),
-- qualified by metrics_valid — wire them to vision_tracker_logic_top's
-- error_x_live / error_y_live / metrics_valid, NOT to its snapshot outputs.
--
-- Clock and PWM frequency are generics (CLK_FREQ_HZ / PWM_FREQ_HZ); every
-- tick value below — servo_controller limits and PWM period/limits — is
-- DERIVED from them (1.0 / 1.5 / 2.0 ms), so changing the clock cannot
-- desynchronise the two halves. The PS deliberately cannot write any of
-- this (register-map contract). Defaults reproduce the original
-- 100 MHz / 50 Hz behaviour exactly.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity two_axis_servo_logic_top is
    generic (
        CLK_FREQ_HZ : integer := 100_000_000;
        PWM_FREQ_HZ : integer := 50
    );
    port (
        clk                 : in  std_logic;
        resetn              : in  std_logic;  -- synchronous, active low

        -- Control from the future register layer (CTRL bits)
        servo_enable        : in  std_logic;
        servo_hold          : in  std_logic;
        track_enable        : in  std_logic;
        search_sweep_enable : in  std_logic;

        -- Live PD-loop feed from vision_tracker_logic_top
        error_x             : in  std_logic_vector(10 downto 0);  -- signed
        error_y             : in  std_logic_vector(10 downto 0);  -- signed
        metrics_valid       : in  std_logic;

        -- Servo outputs
        servo_pwm_pan       : out std_logic;   -- X axis, follows error_x
        servo_pwm_tilt      : out std_logic    -- Y axis, follows error_y
    );
end entity two_axis_servo_logic_top;

architecture rtl of two_axis_servo_logic_top is

    -- Servo commands
    signal ticks_pan, ticks_tilt : std_logic_vector(20 downto 0);

begin

    -- Pan (X) axis: follows error_x, owns the SEARCH sweep.
    u_servo_pan : entity work.servo_controller
        generic map (
            CLK_HZ       => CLK_FREQ_HZ,
            MIN_TICKS    => CLK_FREQ_HZ / 1000,        -- 1.0 ms
            CENTER_TICKS => (3 * CLK_FREQ_HZ) / 2000,  -- 1.5 ms
            MAX_TICKS    => CLK_FREQ_HZ / 500,         -- 2.0 ms
            SERVO_DIR    => 1
        )
        port map (
            clk                 => clk,
            resetn              => resetn,
            servo_enable        => servo_enable,
            servo_hold          => servo_hold,
            track_enable        => track_enable,
            search_sweep_enable => search_sweep_enable,
            error_in            => error_x,
            metrics_valid       => metrics_valid,
            pulse_width_ticks   => ticks_pan
        );

    u_pwm_pan : entity work.pwm_servo_generator
        generic map (
            CLK_FREQ_HZ => CLK_FREQ_HZ,
            PWM_FREQ_HZ => PWM_FREQ_HZ
        )
        port map (
            clk               => clk,
            resetn            => resetn,
            servo_enable      => servo_enable,
            pulse_width_ticks => ticks_pan,
            servo_pwm         => servo_pwm_pan
        );

    -- Tilt (Y) axis: follows error_y. The sweep input is tied '0' — a search
    -- sweep scans the horizon with pan only, so during SEARCH this axis takes
    -- priority 4 and re-centres.
    u_servo_tilt : entity work.servo_controller
        generic map (
            CLK_HZ       => CLK_FREQ_HZ,
            MIN_TICKS    => CLK_FREQ_HZ / 1000,        -- 1.0 ms
            CENTER_TICKS => (3 * CLK_FREQ_HZ) / 2000,  -- 1.5 ms
            MAX_TICKS    => CLK_FREQ_HZ / 500,         -- 2.0 ms
            SERVO_DIR    => 1
        )
        port map (
            clk                 => clk,
            resetn              => resetn,
            servo_enable        => servo_enable,
            servo_hold          => servo_hold,
            track_enable        => track_enable,
            search_sweep_enable => '0',
            error_in            => error_y,
            metrics_valid       => metrics_valid,
            pulse_width_ticks   => ticks_tilt
        );

    u_pwm_tilt : entity work.pwm_servo_generator
        generic map (
            CLK_FREQ_HZ => CLK_FREQ_HZ,
            PWM_FREQ_HZ => PWM_FREQ_HZ
        )
        port map (
            clk               => clk,
            resetn            => resetn,
            servo_enable      => servo_enable,
            pulse_width_ticks => ticks_tilt,
            servo_pwm         => servo_pwm_tilt
        );

end architecture rtl;
