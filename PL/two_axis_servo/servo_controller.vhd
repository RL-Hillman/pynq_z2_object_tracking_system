--------------------------------------------------------------------------------
-- servo_controller.vhd
--
-- SINGLE-AXIS servo controller — instantiate once per axis (pan: error_x,
-- tilt: error_y). Computes the pulse width (in clk ticks) from the tracking
-- error or the search sweep, per the PS/PL CTRL contract, in descending
-- priority:
--
--   1. servo_hold = '1'          -> freeze the last pulse width
--   2. track_enable = '1'        -> PD control on error_in (one step per
--                                   metrics_valid pulse, i.e. per frame)
--   3. search_sweep_enable = '1' -> slow left-right sweep
--   4. otherwise                 -> slew gently back to centre (a hard jump
--                                   to centre would slam the mechanism)
--
-- For an axis that must not sweep (tilt), tie search_sweep_enable to '0':
-- during SEARCH that axis then takes priority 4 and re-centres.
--
-- KP/KD/deadband/step-limit are PL-internal constants by design — the PS
-- deliberately cannot write them (register-map contract). Retune here and
-- rebuild the bitstream. SERVO_DIR is a generic so each axis can flip its
-- direction independently to match the physical mounting.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity servo_controller is
    generic (
        CLK_HZ       : integer := 100_000_000;
        MIN_TICKS    : integer := 100_000;   -- 1.0 ms
        CENTER_TICKS : integer := 150_000;   -- 1.5 ms
        MAX_TICKS    : integer := 200_000;   -- 2.0 ms
        -- ponytail: flip to -1 per instance if the servo moves away from the
        -- hand on real hardware — mounting direction can't be known from code.
        SERVO_DIR    : integer := 1
    );
    port (
        clk                 : in  std_logic;
        resetn              : in  std_logic;  -- synchronous, active low

        servo_enable        : in  std_logic;
        servo_hold          : in  std_logic;
        track_enable        : in  std_logic;
        search_sweep_enable : in  std_logic;

        error_in            : in  std_logic_vector(10 downto 0);  -- signed axis error
        metrics_valid       : in  std_logic;

        pulse_width_ticks   : out std_logic_vector(20 downto 0)
    );
end entity servo_controller;

architecture rtl of servo_controller is

    -- PD gains and limits (ticks per pixel of error).
    constant KP         : integer := 8;
    constant KD         : integer := 4;
    constant DEADBAND   : integer := 5;      -- pixels: no correction near centre
    constant STEP_LIMIT : integer := 1500;   -- max ticks per frame update

    -- Slow timebase for sweep / return-to-centre: 1 kHz.
    constant SLOW_DIV    : integer := CLK_HZ / 1000;
    constant SWEEP_STEP  : integer := 25;    -- ticks per ms -> full range in ~4 s
    constant CENTER_STEP : integer := 25;    -- return-to-centre slew rate

    signal pos         : integer range MIN_TICKS to MAX_TICKS := CENTER_TICKS;
    signal prev_err    : integer range -1024 to 1023 := 0;
    signal sweep_right : boolean := true;
    signal presc       : integer range 0 to SLOW_DIV - 1 := 0;

    -- PD pipeline stage: the full chain (err-prev_err -> KP/KD multiply-add ->
    -- step clamp -> pos add -> rail clamp) misses 100 MHz as one cycle
    -- (18 logic levels, WNS -2.2 ns OOC). Split in two: cycle 1 computes and
    -- clamps the step, cycle 2 applies it to pos. One clk extra latency on a
    -- ~30 Hz update is invisible (PWM latches pos at 20 ms boundaries).
    signal step_r        : integer range -STEP_LIMIT to STEP_LIMIT := 0;
    signal step_pending  : std_logic := '0';

begin

    process(clk)
        variable tick  : boolean;
        variable err   : integer range -1024 to 1023;
        variable step  : integer;
        variable pos_v : integer;
    begin
        if rising_edge(clk) then
            if resetn = '0' then
                pos          <= CENTER_TICKS;
                prev_err     <= 0;
                sweep_right  <= true;
                presc        <= 0;
                step_r       <= 0;
                step_pending <= '0';
            else
                -- 1 kHz tick for the slow movements.
                tick := (presc = SLOW_DIV - 1);
                if tick then
                    presc <= 0;
                else
                    presc <= presc + 1;
                end if;

                -- A queued PD step lives exactly one cycle: re-armed below on
                -- metrics_valid, applied (or dropped by a higher priority —
                -- hold wins means freeze wins) on the following cycle.
                step_pending <= '0';

                if servo_enable = '0' then
                    -- Output stage is disabled (PWM gen emits no pulses);
                    -- freeze so nothing drifts invisibly while off.
                    null;

                elsif servo_hold = '1' then
                    -- Priority 1: freeze at the current position.
                    null;

                elsif track_enable = '1' then
                    -- Priority 2: PD step once per frame measurement,
                    -- pipelined over two cycles (see step_r declaration).

                    -- Stage 2: apply last cycle's clamped step to pos.
                    if step_pending = '1' then
                        pos_v := pos + SERVO_DIR * step_r;
                        if pos_v > MAX_TICKS then
                            pos_v := MAX_TICKS;
                        elsif pos_v < MIN_TICKS then
                            pos_v := MIN_TICKS;
                        end if;
                        pos <= pos_v;
                    end if;

                    -- Stage 1: compute and clamp the step, queue for stage 2.
                    if metrics_valid = '1' then
                        err := to_integer(signed(error_in));
                        -- |err| > DEADBAND, written without abs() for synthesis
                        -- portability (some synth engines reject integer abs).
                        if (err > DEADBAND) or (err < -DEADBAND) then
                            step := KP * err + KD * (err - prev_err);
                            if step > STEP_LIMIT then
                                step := STEP_LIMIT;
                            elsif step < -STEP_LIMIT then
                                step := -STEP_LIMIT;
                            end if;
                            step_r       <= step;
                            step_pending <= '1';
                        end if;
                        prev_err <= err;   -- update even inside the deadband
                    end if;

                elsif search_sweep_enable = '1' then
                    -- Priority 3: slow left-right sweep, reverse at the rails.
                    if tick then
                        if sweep_right then
                            if pos + SWEEP_STEP >= MAX_TICKS then
                                pos         <= MAX_TICKS;
                                sweep_right <= false;
                            else
                                pos <= pos + SWEEP_STEP;
                            end if;
                        else
                            if pos - SWEEP_STEP <= MIN_TICKS then
                                pos         <= MIN_TICKS;
                                sweep_right <= true;
                            else
                                pos <= pos - SWEEP_STEP;
                            end if;
                        end if;
                    end if;

                else
                    -- Priority 4: slew gently back to centre and stay there.
                    if tick then
                        if pos > CENTER_TICKS + CENTER_STEP then
                            pos <= pos - CENTER_STEP;
                        elsif pos < CENTER_TICKS - CENTER_STEP then
                            pos <= pos + CENTER_STEP;
                        else
                            pos <= CENTER_TICKS;
                        end if;
                    end if;
                end if;
            end if;
        end if;
    end process;

    pulse_width_ticks <= std_logic_vector(to_unsigned(pos, 21));

end architecture rtl;
