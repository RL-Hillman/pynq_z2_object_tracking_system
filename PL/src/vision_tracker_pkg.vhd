--------------------------------------------------------------------------------
-- vision_tracker_pkg.vhd
--
-- Global compile-time configuration for the vision tracker.
--
-- To change the video resolution, change only FRAME_WIDTH and FRAME_HEIGHT.
-- All coordinate, size, pixel-count, accumulator and prediction widths are
-- derived automatically from those constants.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


--------------------------------------------------------------------------------
-- Small elaboration-time helper package.
--------------------------------------------------------------------------------
package vision_tracker_math_pkg is


    function clog2(value : positive) return positive;
    
    function coordinate_sum_bits(
    frame_pixel_count : positive;
    axis_size         : positive
) return positive;

end package vision_tracker_math_pkg;


package body vision_tracker_math_pkg is

 
    function clog2(value : positive) return positive is
    
        variable remaining : natural := value - 1;
        variable bits      : natural := 0;
    begin
        while remaining > 0 loop
            remaining := remaining / 2;
            bits := bits + 1;
        end loop;

        if bits = 0 then
            return 1;
        else
            return bits;
        end if;

    end function;



    ---------------------------------------------------------------------------
    -- Returns the number of bits required to store the maximum possible
    -- accumulated coordinate sum for one frame axis.
    --
    -- The calculation is performed with unsigned arithmetic instead of
    -- integer arithmetic so large frame resolutions do not cause
    -- elaboration-time integer overflow.
    ---------------------------------------------------------------------------
    function coordinate_sum_bits(
        frame_pixel_count : positive;
        axis_size         : positive
    ) return positive is
    
        constant PIXEL_BITS : positive :=
            clog2(frame_pixel_count + 1);
    
        constant AXIS_BITS : positive :=
            clog2(axis_size);
    
        variable pixel_count_u :
            unsigned(PIXEL_BITS - 1 downto 0);
    
        variable axis_max_u :
            unsigned(AXIS_BITS - 1 downto 0);
    
        variable max_sum :
            unsigned(PIXEL_BITS + AXIS_BITS - 1 downto 0);
    
    begin
    
        pixel_count_u :=
            to_unsigned(frame_pixel_count, PIXEL_BITS);
    
        axis_max_u :=
            to_unsigned(axis_size - 1, AXIS_BITS);
    
        -- Maximum possible coordinate sum:
        --
        --     frame_pixel_count * (axis_size - 1) / 2
        --
        -- The multiplication is performed as unsigned arithmetic instead of
        -- integer arithmetic, preventing elaboration-time integer overflow for
        -- large frame resolutions.
        max_sum :=
            shift_right(
                pixel_count_u * axis_max_u,
                1
            );
    
        -- Find the highest '1' bit.
        for i in max_sum'high downto 0 loop
    
            if max_sum(i) = '1' then
                return i + 1;
            end if;
    
        end loop;
    
        return 1;
    
    end function;



end package body vision_tracker_math_pkg;


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.vision_tracker_math_pkg.all;


package vision_tracker_pkg is

    ---------------------------------------------------------------------------
    -- Global frame configuration.
    --
    -- Change these two constants to rebuild the complete PL for another
    -- resolution.
    ---------------------------------------------------------------------------
    constant FRAME_WIDTH  : positive := 640;
    constant FRAME_HEIGHT : positive := 480;


    ---------------------------------------------------------------------------
    -- Tracker-wide value that affects interface width.
    --
    -- Algorithm-only tuning parameters remain local to roi_tracker_update.
    ---------------------------------------------------------------------------
    constant VEL_MAX : positive := 120;


    ---------------------------------------------------------------------------
    -- Width used for X/Y coordinates and ROI width/height.
    --
    -- The +1 is intentional: a size signal must be able to represent the full
    -- frame dimension itself. For example, a 1024-pixel-wide frame needs to
    -- represent ROI width = 1024, not only coordinate 1023.
    ---------------------------------------------------------------------------
    constant FRAME_X_BITS : positive :=
        clog2(FRAME_WIDTH + 1);

    constant FRAME_Y_BITS : positive :=
        clog2(FRAME_HEIGHT + 1);


    constant FRAME_PIXEL_COUNT : positive :=
        FRAME_WIDTH * FRAME_HEIGHT;

    ---------------------------------------------------------------------------
    -- Width required to count every pixel in a frame, including the value
    -- FRAME_PIXEL_COUNT itself.
    ---------------------------------------------------------------------------
    constant PIXEL_COUNT_BITS : positive :=
        clog2(FRAME_PIXEL_COUNT + 1);


    ---------------------------------------------------------------------------
    -- Bit widths required for the accumulated X and Y coordinate sums.
    --
    -- object_measurement adds the coordinate of every detected object pixel:
    --
    --     sum_x = x1 + x2 + x3 + ...
    --     sum_y = y1 + y2 + y3 + ...
    --
    -- The maximum possible sum occurs when every pixel in the frame belongs
    -- to the object.
    --
    -- In that case, the average X coordinate of all frame pixels is:
    --
    --     (FRAME_WIDTH - 1) / 2
    --
    -- and the average Y coordinate is:
    --
    --     (FRAME_HEIGHT - 1) / 2
    --
    -- Therefore:
    --
    --     max sum_x = FRAME_PIXEL_COUNT * (FRAME_WIDTH  - 1) / 2
    --     max sum_y = FRAME_PIXEL_COUNT * (FRAME_HEIGHT - 1) / 2
    --
    -- coordinate_sum_bits calculates the required width using unsigned
    -- arithmetic, avoiding integer overflow for large frame resolutions.
    ---------------------------------------------------------------------------
    constant X_SUM_BITS : positive :=
        coordinate_sum_bits(FRAME_PIXEL_COUNT, FRAME_WIDTH);

    constant Y_SUM_BITS : positive :=
        coordinate_sum_bits(FRAME_PIXEL_COUNT, FRAME_HEIGHT);


    ---------------------------------------------------------------------------
    -- Signed widths required for the predicted X and Y center coordinates.
    --
    -- The predicted center may temporarily move outside the frame by up to
    -- VEL_MAX pixels.
    --
    -- For example, with FRAME_WIDTH = 640 and VEL_MAX = 120:
    --
    --     predicted X range = -120 ... 759
    --
    -- The width includes enough magnitude bits for the largest possible
    -- prediction and one additional bit for the sign.
    ---------------------------------------------------------------------------
    constant PREDICTED_CENTER_X_BITS : positive :=
        clog2(FRAME_WIDTH + VEL_MAX + 1) + 1;

    constant PREDICTED_CENTER_Y_BITS : positive :=
        clog2(FRAME_HEIGHT + VEL_MAX + 1) + 1;


end package vision_tracker_pkg;
