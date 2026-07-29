--------------------------------------------------------------------------------
-- pwm_servo_generator.vhd
--
-- RC-servo PWM generator. Frequency-configurable: ALL timing is derived
-- from the two generics. PERIOD = CLK_FREQ_HZ / PWM_FREQ_HZ; the pulse
-- limits are fixed TIMES (1.0 ms / 2.0 ms — an RC-servo property that does
-- not depend on the refresh rate), so they derive from CLK_FREQ_HZ alone.
-- Defaults (100 MHz, 50 Hz): period = 2,000,000 ticks, 1.0 ms = 100,000,
-- 1.5 ms = 150,000, 2.0 ms = 200,000 — identical to the old hardcoded values.
--
-- The commanded pulse width is latched only at the period boundary, so a
-- mid-period command change can never produce a runt or glitched pulse.
-- The latched value is defensively clamped to [MIN_TICKS, MAX_TICKS].
-- servo_enable is likewise armed only at the period boundary (en_lat): the
-- PS sets it over AXI at a random counter phase, and an enable landing
-- inside the pulse window would otherwise start a partial (runt) pulse.
-- Disable is NOT delayed — it cuts the output combinationally, see below.
--
-- servo_enable = '0' -> output '0' (no pulses at all). Chosen over forcing a
-- centre pulse: with no pulses a standard RC servo de-energises and simply
-- holds its last position by its own electronics — exactly the behaviour the
-- PS-side contract documents ("PWM output disabled; servo holds last
-- position by hardware default"). Forcing centre would instead slam the
-- mechanism to mid-travel on every disable.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity pwm_servo_generator is
    generic (
        CLK_FREQ_HZ : integer := 100_000_000;
        PWM_FREQ_HZ : integer := 50
    );
    port (
        clk               : in  std_logic;
        resetn            : in  std_logic;  -- synchronous, active low

        servo_enable      : in  std_logic;
        pulse_width_ticks : in  std_logic_vector(20 downto 0);

        servo_pwm         : out std_logic
    );
end entity pwm_servo_generator;

architecture rtl of pwm_servo_generator is

    -- Everything below is computed from the generics (see header).
    constant PERIOD    : integer := CLK_FREQ_HZ / PWM_FREQ_HZ;  -- 2,000,000 default
    constant MIN_TICKS : integer := CLK_FREQ_HZ / 1000;         -- 1.0 ms pulse
    constant MAX_TICKS : integer := CLK_FREQ_HZ / 500;          -- 2.0 ms pulse

    signal cnt       : integer range 0 to PERIOD - 1 := 0;
    signal pulse_lat : integer range 0 to MAX_TICKS  := MIN_TICKS;
    signal en_lat    : std_logic := '0';
    signal pwm_r     : std_logic := '0';

begin

    process(clk)
        variable req : integer;
    begin
        if rising_edge(clk) then
            if resetn = '0' then
                cnt       <= 0;
                pulse_lat <= MIN_TICKS;
                en_lat    <= '0';
                pwm_r     <= '0';
            else
                -- Period counter; latch (and clamp) the command at rollover.
                if cnt = PERIOD - 1 then
                    cnt <= 0;
                    req := to_integer(unsigned(pulse_width_ticks));
                    if req < MIN_TICKS then
                        req := MIN_TICKS;
                    elsif req > MAX_TICKS then
                        req := MAX_TICKS;
                    end if;
                    pulse_lat <= req;
                else
                    cnt <= cnt + 1;
                end if;

                -- Arm the output only at a period boundary so a mid-period
                -- enable can never start a partial (runt) pulse. A disable
                -- disarms until the next boundary (and cuts pwm_r below).
                if cnt = PERIOD - 1 then
                    en_lat <= servo_enable;
                elsif servo_enable = '0' then
                    en_lat <= '0';
                end if;

                -- Registered output: high for pulse_lat ticks of each period.
                -- servo_enable appears directly too, so disable is immediate.
                if (servo_enable = '1') and (en_lat = '1') and (cnt < pulse_lat) then
                    pwm_r <= '1';
                else
                    pwm_r <= '0';
                end if;
            end if;
        end if;
    end process;

    servo_pwm <= pwm_r;

end architecture rtl;
