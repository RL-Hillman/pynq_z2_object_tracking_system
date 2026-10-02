--------------------------------------------------------------------------------
-- object_measurement.vhd
--
-- Calculates object measurements from a binary mask.
--
-- The design is split into two independent clocked processes:
--
--   1. frame_accumulator_process
--      Collects pixel statistics for the current frame and stores a stable
--      snapshot when the frame ends.
--
--   2. centroid_divider_process
--      Uses the completed-frame snapshot to calculate centroid X and Y in
--      parallel with two small sequential dividers, then updates the outputs.
--
-- While the dividers process the completed frame, the accumulator process can
-- already collect pixels from the next frame.
---------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.vision_tracker_pkg.all;

entity object_measurement is

    port (

        clk     : in std_logic;
        reset_n : in std_logic;
        enable  : in std_logic;
        
        mask_in : in std_logic;

        pixel_x : in std_logic_vector(FRAME_X_BITS - 1 downto 0);
        pixel_y : in std_logic_vector(FRAME_Y_BITS - 1 downto 0);

        filtered_mask_valid_in : in std_logic;
        frame_end_in    : in std_logic;

        centroid_x : out std_logic_vector(FRAME_X_BITS - 1 downto 0);
        centroid_y : out std_logic_vector(FRAME_Y_BITS - 1 downto 0);

        bbox_width  : out std_logic_vector(FRAME_X_BITS - 1 downto 0);
        bbox_height : out std_logic_vector(FRAME_Y_BITS - 1 downto 0);

        object_pixel_count : out std_logic_vector(PIXEL_COUNT_BITS - 1 downto 0);

        frame_measurement_done : out std_logic

    );

end entity object_measurement;


architecture rtl of object_measurement is


    ---------------------------------------------------------------------------
    -- Current-frame accumulators
    ---------------------------------------------------------------------------

    signal sum_x_acc :
        unsigned(X_SUM_BITS - 1 downto 0) := (others => '0');

    signal sum_y_acc :
        unsigned(Y_SUM_BITS - 1 downto 0) := (others => '0');

    signal pixel_count_acc :
        unsigned(PIXEL_COUNT_BITS - 1 downto 0) := (others => '0');

    signal min_x_acc :
        unsigned(FRAME_X_BITS - 1 downto 0) := (others => '1');

    signal max_x_acc :
        unsigned(FRAME_X_BITS - 1 downto 0) := (others => '0');

    signal min_y_acc :
        unsigned(FRAME_Y_BITS - 1 downto 0) := (others => '1');

    signal max_y_acc :
        unsigned(FRAME_Y_BITS - 1 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Completed-frame snapshot
    --
    -- These values remain stable while the divider process works.
    ---------------------------------------------------------------------------
    signal frame_sum_x :
        unsigned(X_SUM_BITS - 1 downto 0) := (others => '0');

    signal frame_sum_y :
        unsigned(Y_SUM_BITS - 1 downto 0) := (others => '0');

    signal frame_pixel_count :
        unsigned(PIXEL_COUNT_BITS - 1 downto 0) := (others => '0');

    signal frame_min_x :
        unsigned(FRAME_X_BITS - 1 downto 0) := (others => '0');

    signal frame_max_x :
        unsigned(FRAME_X_BITS - 1 downto 0) := (others => '0');

    signal frame_min_y :
        unsigned(FRAME_Y_BITS - 1 downto 0) := (others => '0');

    signal frame_max_y :
        unsigned(FRAME_Y_BITS - 1 downto 0) := (others => '0');

    -- One-clock internal pulse: a new completed-frame snapshot is available.
    signal frame_snapshot_ready : std_logic := '0';


    ---------------------------------------------------------------------------
    -- Parallel sequential-divider state machine
    ---------------------------------------------------------------------------

    type divider_state_t is (
        IDLE,
        DIVIDE_XY,
        UPDATE_OUTPUTS
    );

    signal divider_state : divider_state_t := IDLE;


    ---------------------------------------------------------------------------
    -- Parallel sequential-divider registers
    ---------------------------------------------------------------------------

    signal remainder_x :
        unsigned(PIXEL_COUNT_BITS - 1 downto 0) := (others => '0');
    
    signal remainder_y :
        unsigned(PIXEL_COUNT_BITS - 1 downto 0) := (others => '0');

    signal centroid_x_tmp :
        unsigned(FRAME_X_BITS - 1 downto 0) := (others => '0');
    
    signal centroid_y_tmp :
        unsigned(FRAME_Y_BITS - 1 downto 0) := (others => '0');

    signal bit_index_x :
    integer range 0 to FRAME_X_BITS - 1 := FRAME_X_BITS - 1;

    signal bit_index_y :
        integer range 0 to FRAME_Y_BITS - 1 := FRAME_Y_BITS - 1;


begin


    ---------------------------------------------------------------------------
    -- PROCESS 1: Frame accumulator and snapshot
    --
    -- This process deals with the incoming pixel stream.
    -- It collects sum/count/bounding-box data and, at frame end, copies the
    -- completed values into the frame_* snapshot registers.
    ---------------------------------------------------------------------------

    frame_accumulator_process : process(clk)

        -- Values after processing the current input pixel.
        -- Variables update immediately inside the process, which ensures that
        -- the final pixel is included when frame_end_in arrives on the same
        -- clock as that pixel.
        variable next_sum_x :
            unsigned(X_SUM_BITS - 1 downto 0);
        
        variable next_sum_y :
            unsigned(Y_SUM_BITS - 1 downto 0);
        
        variable next_pixel_count :
            unsigned(PIXEL_COUNT_BITS - 1 downto 0);
        
        variable next_min_x :
            unsigned(FRAME_X_BITS - 1 downto 0);
        
        variable next_max_x :
            unsigned(FRAME_X_BITS - 1 downto 0);
        
        variable next_min_y :
            unsigned(FRAME_Y_BITS - 1 downto 0);
        
        variable next_max_y :
            unsigned(FRAME_Y_BITS - 1 downto 0);

    begin

        if rising_edge(clk) then

            if reset_n = '0' then

                -- Current-frame accumulators
                sum_x_acc <= (others => '0');
                sum_y_acc <= (others => '0');

                pixel_count_acc <= (others => '0');

                min_x_acc <= (others => '1');
                max_x_acc <= (others => '0');

                min_y_acc <= (others => '1');
                max_y_acc <= (others => '0');

                -- Completed-frame snapshot
                frame_sum_x <= (others => '0');
                frame_sum_y <= (others => '0');

                frame_pixel_count <= (others => '0');

                frame_min_x <= (others => '0');
                frame_max_x <= (others => '0');

                frame_min_y <= (others => '0');
                frame_max_y <= (others => '0');

                frame_snapshot_ready <= '0';

            else

                frame_snapshot_ready <= '0';
                
                if enable = '1' then
                
                    -- At the start of each pixel, copy the current accumulator state into
                    -- the next_* variables. This is especially important after a frame ends:
                    -- the accumulators are reset for the new frame, while process variables
                    -- may still contain values from the previous frame.
                    next_sum_x := sum_x_acc;
                    next_sum_y := sum_y_acc;
    
                    next_pixel_count := pixel_count_acc;
    
                    next_min_x := min_x_acc;
                    next_max_x := max_x_acc;
    
                    next_min_y := min_y_acc;
                    next_max_y := max_y_acc;
    
    
                    ----------------------------------------------------------------
                    -- Process the current object pixel
                    ----------------------------------------------------------------
    
                    if (filtered_mask_valid_in = '1') and
                       (mask_in = '1') then
    
                        next_sum_x :=
                            sum_x_acc
                            +
                            resize(unsigned(pixel_x), next_sum_x'length);
    
                        next_sum_y :=
                            sum_y_acc
                            +
                            resize(unsigned(pixel_y), next_sum_y'length);
    
                        next_pixel_count := pixel_count_acc + 1;
    
                        if unsigned(pixel_x) < min_x_acc then
                            next_min_x := unsigned(pixel_x);
                        end if;
    
                        if unsigned(pixel_x) > max_x_acc then
                            next_max_x := unsigned(pixel_x);
                        end if;
    
                        if unsigned(pixel_y) < min_y_acc then
                            next_min_y := unsigned(pixel_y);
                        end if;
    
                        if unsigned(pixel_y) > max_y_acc then
                            next_max_y := unsigned(pixel_y);
                        end if;
    
                    end if;
    
    
                    ----------------------------------------------------------------
                    -- End of frame
                    ----------------------------------------------------------------
    
                    if (frame_end_in = '1') then
    
                        -- A new snapshot is only accepted while the divider is free.
                        -- Under normal video timing the divider finishes long before
                        -- the next frame ends.
                        if divider_state = IDLE then
    
                            frame_sum_x <= next_sum_x;
                            frame_sum_y <= next_sum_y;
    
                            frame_pixel_count <= next_pixel_count;
    
                            frame_min_x <= next_min_x;
                            frame_max_x <= next_max_x;
    
                            frame_min_y <= next_min_y;
                            frame_max_y <= next_max_y;
    
                            frame_snapshot_ready <= '1';
    
                        end if;
    
                        -- The current frame is finished. Always clear the live
                        -- accumulators so the next frame starts cleanly.
                        sum_x_acc <= (others => '0');
                        sum_y_acc <= (others => '0');
    
                        pixel_count_acc <= (others => '0');
    
                        min_x_acc <= (others => '1');
                        max_x_acc <= (others => '0');
    
                        min_y_acc <= (others => '1');
                        max_y_acc <= (others => '0');
    
                    else
    
                        -- Frame continues: store the updated accumulator values for
                        -- the next clock.
                        sum_x_acc <= next_sum_x;
                        sum_y_acc <= next_sum_y;
    
                        pixel_count_acc <= next_pixel_count;
    
                        min_x_acc <= next_min_x;
                        max_x_acc <= next_max_x;
    
                        min_y_acc <= next_min_y;
                        max_y_acc <= next_max_y;
    
                    end if;

                 end if;
            
            end if;
            
        end if;

    end process frame_accumulator_process;


    ---------------------------------------------------------------------------
    -- PROCESS 2: Parallel centroid dividers and output update
    --
    -- This process never touches the live pixel accumulators. It only consumes
    -- the stable frame_* snapshot produced by frame_accumulator_process.
    ---------------------------------------------------------------------------

    centroid_divider_process : process(clk)

    variable shifted_remainder_x :
        unsigned(PIXEL_COUNT_BITS downto 0);
    
    variable shifted_remainder_y :
        unsigned(PIXEL_COUNT_BITS downto 0);

    variable done_x : std_logic;
    variable done_y : std_logic;

    begin

        if rising_edge(clk) then

            if reset_n = '0' then

                divider_state <= IDLE;

                remainder_x <= (others => '0');
                remainder_y <= (others => '0');

                centroid_x_tmp <= (others => '0');
                centroid_y_tmp <= (others => '0');

                bit_index_x <= FRAME_X_BITS - 1;
                bit_index_y <= FRAME_Y_BITS - 1;
                
                centroid_x <= (others => '0');
                centroid_y <= (others => '0');

                bbox_width  <= (others => '0');
                bbox_height <= (others => '0');

                object_pixel_count <= (others => '0');

                frame_measurement_done <= '0';
                done_y:= '0';
                done_x:= '0';
            else

                --frame_measurement_done is a one-clock event pulses.
                frame_measurement_done <= '0';


                case divider_state is

                    when IDLE =>

                        if frame_snapshot_ready = '1' then
                    
                            if frame_pixel_count = 0 then
                    
                                centroid_x_tmp <= (others => '0');
                                centroid_y_tmp <= (others => '0');
                                       
                                divider_state <= UPDATE_OUTPUTS;
                    
                            else
                    
                                -- IMPORTANT:
                                -- The centroid result requires only FRAME_X_BITS / FRAME_Y_BITS.
                                --
                                -- Therefore, quotient bits above these widths are always zero.
                                -- The corresponding upper frame_sum bits can be loaded directly
                                -- into the remainder, skipping division steps that cannot produce
                                -- a '1' in the centroid result.
                                --
                                -- The divider then processes only the remaining centroid bits
                                -- sequentially.
                                remainder_x <=
                                    resize(
                                        frame_sum_x(
                                            X_SUM_BITS - 1 downto FRAME_X_BITS
                                        ),
                                        remainder_x'length
                                    );
                    
                                remainder_y <=
                                    resize(
                                        frame_sum_y(
                                            Y_SUM_BITS - 1 downto FRAME_Y_BITS
                                        ),
                                        remainder_y'length
                                    );
                    
                                centroid_x_tmp <= (others => '0');
                                centroid_y_tmp <= (others => '0');
                    
                                bit_index_x <= FRAME_X_BITS - 1;
                                bit_index_y <= FRAME_Y_BITS - 1;
                    
                                done_x := '0';
                                done_y := '0';
                    
                                divider_state <= DIVIDE_XY;
                    
                            end if;
                    
                        end if;
                    

                    ----------------------------------------------------------------
                    -- Calculate the centroid coordinates:
                    --   centroid_x = frame_sum_x / frame_pixel_count
                    --   centroid_y = frame_sum_y / frame_pixel_count
                    ----------------------------------------------------------------
                    when DIVIDE_XY =>

                     if done_x='0' then
                        if bit_index_x=0 then
                            done_x:='1';
                        end if;
                        -- X division step
                        shifted_remainder_x :=remainder_x & frame_sum_x(bit_index_x);
                        if shifted_remainder_x >= resize(frame_pixel_count, shifted_remainder_x'length) then

                            remainder_x <=
                                resize(shifted_remainder_x - resize(frame_pixel_count, shifted_remainder_x'length), remainder_x'length);

                            centroid_x_tmp(bit_index_x) <= '1';


                        else

                            remainder_x <= resize(shifted_remainder_x,remainder_x'length);

                       end if;
                   end if; 

        
                    if done_y='0' then
                        if bit_index_y=0 then
                            done_y:='1';
                        end if;
                        -- Y division step -- runs in parallel with X
                         shifted_remainder_y :=
                            remainder_y & frame_sum_y(bit_index_y);
                            
                        if shifted_remainder_y >=
                           resize(frame_pixel_count, shifted_remainder_y'length) then
                        
                            remainder_y <=
                                resize(
                                    shifted_remainder_y -
                                    resize(frame_pixel_count, shifted_remainder_y'length),
                                    remainder_y'length
                                );
    
                            centroid_y_tmp(bit_index_y) <= '1';
    
                        else
    
                            remainder_y <=resize(shifted_remainder_y,remainder_y'length);
    
                        end if;
                    end if;

                        if (done_x = '1') and (done_y = '1') then
                            divider_state <= UPDATE_OUTPUTS;
                        else
                            if bit_index_x > 0 then
                                bit_index_x <= bit_index_x - 1;
                            end if;
                        
                            if bit_index_y > 0 then
                                bit_index_y <= bit_index_y - 1;
                            end if;
                        end if;
    
                    ----------------------------------------------------------------
                    -- Publish the completed frame measurement
                    ----------------------------------------------------------------

                    when UPDATE_OUTPUTS =>

                        centroid_x <= std_logic_vector(centroid_x_tmp);
                        centroid_y <= std_logic_vector(centroid_y_tmp);
                        
                        if frame_pixel_count = 0 then

                            bbox_width  <= (others => '0');
                            bbox_height <= (others => '0');

                        else
                        
                        bbox_width <=
                            std_logic_vector(frame_max_x - frame_min_x + 1);

                        bbox_height <= 
                        std_logic_vector(frame_max_y - frame_min_y + 1);
                        
                        end if;
                        
                        object_pixel_count <=
                            std_logic_vector(frame_pixel_count);

                        frame_measurement_done <= '1';

                        divider_state <= IDLE;

                end case;

            end if;

        end if;

    end process centroid_divider_process;


end architecture rtl;
