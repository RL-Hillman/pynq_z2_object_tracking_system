library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.vision_tracker_pkg.all;

entity vision_tracker_ip is
	generic (
		-- Users to add parameters here

		-- User parameters ends
		-- Do not modify the parameters beyond this line


		-- Parameters of Axi Slave Bus Interface S_AXI_TRACK
		C_S_AXI_TRACK_DATA_WIDTH	: integer	:= 32;
		C_S_AXI_TRACK_ADDR_WIDTH	: integer	:= 7
	);
	port (
	
		-- Users to add ports here
		
		
        ------------------------------------------------------------------------
        -- AXI4-Stream video input                                            RL
        ------------------------------------------------------------------------
        s_axis_video_tdata  : in  std_logic_vector(23 downto 0); -- RGB pixel data
        s_axis_video_tvalid : in  std_logic;                     -- New valid pixel available
        s_axis_video_tuser  : in  std_logic;                     -- Start of frame
        s_axis_video_tlast  : in  std_logic;                     -- End of line

        s_axis_video_tready : out std_logic;                  -- Tracker ready to accept pixel
        ------------------------------------------------------------------------
        -- Processed video output
        ------------------------------------------------------------------------
        video_out_tdata  : out std_logic_vector(23 downto 0); -- Processed RGB pixel
        video_out_tvalid : out std_logic;                     -- Valid output pixel
        video_out_tuser  : out std_logic;                     -- Start of frame
        video_out_tlast  : out std_logic;                     -- End of line
        video_out_tready : in std_logic;


		-- User ports ends
		-- Do not modify the ports beyond this line


		-- Ports of Axi Slave Bus Interface S_AXI_TRACK
		s_axi_track_aclk	: in std_logic;
		s_axi_track_aresetn	: in std_logic;
		s_axi_track_awaddr	: in std_logic_vector(C_S_AXI_TRACK_ADDR_WIDTH-1 downto 0);
		s_axi_track_awprot	: in std_logic_vector(2 downto 0);
		s_axi_track_awvalid	: in std_logic;
		s_axi_track_awready	: out std_logic;
		s_axi_track_wdata	: in std_logic_vector(C_S_AXI_TRACK_DATA_WIDTH-1 downto 0);
		s_axi_track_wstrb	: in std_logic_vector((C_S_AXI_TRACK_DATA_WIDTH/8)-1 downto 0);
		s_axi_track_wvalid	: in std_logic;
		s_axi_track_wready	: out std_logic;
		s_axi_track_bresp	: out std_logic_vector(1 downto 0);
		s_axi_track_bvalid	: out std_logic;
		s_axi_track_bready	: in std_logic;
		s_axi_track_araddr	: in std_logic_vector(C_S_AXI_TRACK_ADDR_WIDTH-1 downto 0);
		s_axi_track_arprot	: in std_logic_vector(2 downto 0);
		s_axi_track_arvalid	: in std_logic;
		s_axi_track_arready	: out std_logic;
		s_axi_track_rdata	: out std_logic_vector(C_S_AXI_TRACK_DATA_WIDTH-1 downto 0);
		s_axi_track_rresp	: out std_logic_vector(1 downto 0);
		s_axi_track_rvalid	: out std_logic;
		s_axi_track_rready	: in std_logic
	);
end vision_tracker_ip;

architecture arch_imp of vision_tracker_ip is


    ------------------------------------------------------------------------
    -- AXI configuration -> tracking core                                 RL  
    ------------------------------------------------------------------------
    signal tracking_enable_s :
        std_logic;

    signal initial_roi_x_s :
        std_logic_vector(C_S_AXI_TRACK_DATA_WIDTH-1 downto 0);

    signal initial_roi_y_s :
        std_logic_vector(C_S_AXI_TRACK_DATA_WIDTH-1 downto 0);

    signal initial_roi_width_s :
        std_logic_vector(C_S_AXI_TRACK_DATA_WIDTH-1 downto 0);

    signal initial_roi_height_s :
        std_logic_vector(C_S_AXI_TRACK_DATA_WIDTH-1 downto 0);

    signal object_thresholds_s :
        std_logic_vector(C_S_AXI_TRACK_DATA_WIDTH-1 downto 0);


    ------------------------------------------------------------------------
    -- Tracking core -> AXI                                               RL
    ------------------------------------------------------------------------
    signal object_center_x_s :
        std_logic_vector(C_S_AXI_TRACK_DATA_WIDTH-1 downto 0);

    signal object_center_y_s :
        std_logic_vector(C_S_AXI_TRACK_DATA_WIDTH-1 downto 0);

    signal object_width_s :
        std_logic_vector(C_S_AXI_TRACK_DATA_WIDTH-1 downto 0);

    signal object_height_s :
        std_logic_vector(C_S_AXI_TRACK_DATA_WIDTH-1 downto 0);

    signal object_pixel_count_s :
        std_logic_vector(C_S_AXI_TRACK_DATA_WIDTH-1 downto 0);

    signal tracking_roi_x_s :
        std_logic_vector(C_S_AXI_TRACK_DATA_WIDTH-1 downto 0);

    signal tracking_roi_y_s :
        std_logic_vector(C_S_AXI_TRACK_DATA_WIDTH-1 downto 0);

    signal tracking_roi_width_s :
        std_logic_vector(C_S_AXI_TRACK_DATA_WIDTH-1 downto 0);

    signal tracking_roi_height_s :
        std_logic_vector(C_S_AXI_TRACK_DATA_WIDTH-1 downto 0);

    signal tracking_lost_s :
        std_logic;

    signal predicted_center_x_s :
        signed(C_S_AXI_TRACK_DATA_WIDTH-1 downto 0);

    signal predicted_center_y_s :
        signed(C_S_AXI_TRACK_DATA_WIDTH-1 downto 0);

    signal snapshot_counter_s :
        std_logic_vector(31 downto 0);
    
    -------------------------------------------
    -- Allow the video pipeline to advance   RL
    -------------------------------------------
    signal video_pipeline_enable_s : std_logic;
    signal video_out_tvalid_s : std_logic;


	-- component declaration
	component vision_tracker_ip_slave_lite_v1_0_S_AXI_TRACK is
		generic (
		C_S_AXI_DATA_WIDTH	: integer	:= 32;
		C_S_AXI_ADDR_WIDTH	: integer	:= 7
		);
		port (
		
		------------------------------------------------------------------------
        -- Configuration outputs to the tracking core                         RL
        ------------------------------------------------------------------------
        tracking_enable_out     : out std_logic;

        initial_roi_x_out       : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        initial_roi_y_out       : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        initial_roi_width_out   : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        initial_roi_height_out  : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);

        object_thresholds_out     : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);


        ------------------------------------------------------------------------
        -- Tracking results from the tracking core                            RL
        ------------------------------------------------------------------------
        tracking_lost_in        : in std_logic;

        snapshot_counter_in     : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);

        frame_width_in          : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        frame_height_in         : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);

        object_center_x_in      : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        object_center_y_in      : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        object_width_in         : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        object_height_in        : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        object_pixel_count_in   : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);

        tracking_roi_x_in       : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        tracking_roi_y_in       : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        tracking_roi_width_in   : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        tracking_roi_height_in  : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);

        predicted_center_x_in   : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        predicted_center_y_in   : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
		------------------------------------------------------------------------------
		
	
		S_AXI_ACLK	: in std_logic;
		S_AXI_ARESETN	: in std_logic;
		S_AXI_AWADDR	: in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
		S_AXI_AWPROT	: in std_logic_vector(2 downto 0);
		S_AXI_AWVALID	: in std_logic;
		S_AXI_AWREADY	: out std_logic;
		S_AXI_WDATA	: in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
		S_AXI_WSTRB	: in std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
		S_AXI_WVALID	: in std_logic;
		S_AXI_WREADY	: out std_logic;
		S_AXI_BRESP	: out std_logic_vector(1 downto 0);
		S_AXI_BVALID	: out std_logic;
		S_AXI_BREADY	: in std_logic;
		S_AXI_ARADDR	: in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
		S_AXI_ARPROT	: in std_logic_vector(2 downto 0);
		S_AXI_ARVALID	: in std_logic;
		S_AXI_ARREADY	: out std_logic;
		S_AXI_RDATA	: out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
		S_AXI_RRESP	: out std_logic_vector(1 downto 0);
		S_AXI_RVALID	: out std_logic;
		S_AXI_RREADY	: in std_logic
		);
	end component vision_tracker_ip_slave_lite_v1_0_S_AXI_TRACK;

begin

-- Instantiation of Axi Bus Interface S_AXI_TRACK
vision_tracker_ip_slave_lite_v1_0_S_AXI_TRACK_inst : vision_tracker_ip_slave_lite_v1_0_S_AXI_TRACK
	generic map (
		C_S_AXI_DATA_WIDTH	=> C_S_AXI_TRACK_DATA_WIDTH,
		C_S_AXI_ADDR_WIDTH	=> C_S_AXI_TRACK_ADDR_WIDTH
	)
	port map (
	    ------------------------------------------------------------------------
        -- PS -> tracking core                                                RL
        ------------------------------------------------------------------------
        tracking_enable_out    => tracking_enable_s,

        initial_roi_x_out      => initial_roi_x_s,
        initial_roi_y_out      => initial_roi_y_s,
        initial_roi_width_out  => initial_roi_width_s,
        initial_roi_height_out => initial_roi_height_s,

        object_thresholds_out => object_thresholds_s,

        ------------------------------------------------------------------------
        -- Tracking core -> PS                                                RL
        ------------------------------------------------------------------------
        tracking_lost_in       => tracking_lost_s,
        snapshot_counter_in    => snapshot_counter_s,

        frame_width_in =>
            std_logic_vector(
                to_unsigned(FRAME_WIDTH, C_S_AXI_TRACK_DATA_WIDTH)
            ),

        frame_height_in =>
            std_logic_vector(
                to_unsigned(FRAME_HEIGHT, C_S_AXI_TRACK_DATA_WIDTH)
            ),

        object_center_x_in     => object_center_x_s,
        object_center_y_in     => object_center_y_s,
        object_width_in        => object_width_s,
        object_height_in       => object_height_s,
        object_pixel_count_in  => object_pixel_count_s,

        tracking_roi_x_in      => tracking_roi_x_s,
        tracking_roi_y_in      => tracking_roi_y_s,
        tracking_roi_width_in  => tracking_roi_width_s,
        tracking_roi_height_in => tracking_roi_height_s,

        predicted_center_x_in  =>
            std_logic_vector(predicted_center_x_s),

        predicted_center_y_in  =>
            std_logic_vector(predicted_center_y_s),
            
            
            
		S_AXI_ACLK	=> s_axi_track_aclk,
		S_AXI_ARESETN	=> s_axi_track_aresetn,
		S_AXI_AWADDR	=> s_axi_track_awaddr,
		S_AXI_AWPROT	=> s_axi_track_awprot,
		S_AXI_AWVALID	=> s_axi_track_awvalid,
		S_AXI_AWREADY	=> s_axi_track_awready,
		S_AXI_WDATA	=> s_axi_track_wdata,
		S_AXI_WSTRB	=> s_axi_track_wstrb,
		S_AXI_WVALID	=> s_axi_track_wvalid,
		S_AXI_WREADY	=> s_axi_track_wready,
		S_AXI_BRESP	=> s_axi_track_bresp,
		S_AXI_BVALID	=> s_axi_track_bvalid,
		S_AXI_BREADY	=> s_axi_track_bready,
		S_AXI_ARADDR	=> s_axi_track_araddr,
		S_AXI_ARPROT	=> s_axi_track_arprot,
		S_AXI_ARVALID	=> s_axi_track_arvalid,
		S_AXI_ARREADY	=> s_axi_track_arready,
		S_AXI_RDATA	=> s_axi_track_rdata,
		S_AXI_RRESP	=> s_axi_track_rresp,
		S_AXI_RVALID	=> s_axi_track_rvalid,
		S_AXI_RREADY	=> s_axi_track_rready
	);

	-- Add user logic here


    ------------------------------------------------------------------------
    -- Vision tracking core                                               RL
    ------------------------------------------------------------------------
    TRACKER_CORE_U : entity work.vision_tracker_logic_top

        port map (

            ----------------------------------------------------------------
            -- Clock / reset
            ----------------------------------------------------------------
            clk     => s_axi_track_aclk,
            reset_n => s_axi_track_aresetn,
            enable  => video_pipeline_enable_s,

            ----------------------------------------------------------------
            -- Video input
            ----------------------------------------------------------------
            pixel_valid_in => s_axis_video_tvalid,
            new_frame_in   => s_axis_video_tuser,
            last_line_in   => s_axis_video_tlast,

            r_in => s_axis_video_tdata(23 downto 16),
            g_in => s_axis_video_tdata(15 downto 8),
            b_in => s_axis_video_tdata(7 downto 0),

            ----------------------------------------------------------------
            -- Configuration from PS
            ----------------------------------------------------------------
            tracking_enable => tracking_enable_s,

            initial_roi_x =>
                initial_roi_x_s(FRAME_X_BITS-1 downto 0),

            initial_roi_y =>
                initial_roi_y_s(FRAME_Y_BITS-1 downto 0),

            initial_roi_width =>
                initial_roi_width_s(FRAME_X_BITS-1 downto 0),

            initial_roi_height =>
                initial_roi_height_s(FRAME_Y_BITS-1 downto 0),

            cr_min => object_thresholds_s(7 downto 0),
            cr_max => object_thresholds_s(15 downto 8),
            cb_min => object_thresholds_s(23 downto 16),
            cb_max => object_thresholds_s(31 downto 24),

            ----------------------------------------------------------------
            -- Tracking results
            ----------------------------------------------------------------
            object_center_x =>
                object_center_x_s(FRAME_X_BITS-1 downto 0),

            object_center_y =>
                object_center_y_s(FRAME_Y_BITS-1 downto 0),

            object_width =>
                object_width_s(FRAME_X_BITS-1 downto 0),

            object_height =>
                object_height_s(FRAME_Y_BITS-1 downto 0),

            object_pixel_count =>
                object_pixel_count_s(PIXEL_COUNT_BITS-1 downto 0),

            tracking_roi_x =>
                tracking_roi_x_s(FRAME_X_BITS-1 downto 0),

            tracking_roi_y =>
                tracking_roi_y_s(FRAME_Y_BITS-1 downto 0),

            tracking_roi_width =>
                tracking_roi_width_s(FRAME_X_BITS-1 downto 0),

            tracking_roi_height =>
                tracking_roi_height_s(FRAME_Y_BITS-1 downto 0),

            tracking_lost =>
                tracking_lost_s,

            predicted_center_x =>
                predicted_center_x_s(
                    PREDICTED_CENTER_X_BITS-1 downto 0
                ),

            predicted_center_y =>
                predicted_center_y_s(
                    PREDICTED_CENTER_Y_BITS-1 downto 0
                ),

            tracking_snapshot_counter =>
                snapshot_counter_s,

            ----------------------------------------------------------------
            -- Processed video output
            ----------------------------------------------------------------
            debug_rgb_out         => video_out_tdata,
            debug_pixel_valid_out => video_out_tvalid_s,
            debug_line_end_out    => video_out_tlast,
            debug_frame_start_out => video_out_tuser,

            ----------------------------------------------------------------
            -- AXI-Stream input backpressure
            ----------------------------------------------------------------
            pixel_ready_out =>
                s_axis_video_tready

        );

    ------------------------------------------------------------------------
    -- Extend tracking results to the 32-bit AXI register width.          RL
    --
    -- Unsigned results are zero-extended.
    -- Signed predicted centers are sign-extended to preserve negative values.
    ------------------------------------------------------------------------
    object_center_x_s(
        C_S_AXI_TRACK_DATA_WIDTH-1 downto FRAME_X_BITS
    ) <= (others => '0');

    object_center_y_s(
        C_S_AXI_TRACK_DATA_WIDTH-1 downto FRAME_Y_BITS
    ) <= (others => '0');

    object_width_s(
        C_S_AXI_TRACK_DATA_WIDTH-1 downto FRAME_X_BITS
    ) <= (others => '0');

    object_height_s(
        C_S_AXI_TRACK_DATA_WIDTH-1 downto FRAME_Y_BITS
    ) <= (others => '0');

    object_pixel_count_s(
        C_S_AXI_TRACK_DATA_WIDTH-1 downto PIXEL_COUNT_BITS
    ) <= (others => '0');


    tracking_roi_x_s(
        C_S_AXI_TRACK_DATA_WIDTH-1 downto FRAME_X_BITS
    ) <= (others => '0');

    tracking_roi_y_s(
        C_S_AXI_TRACK_DATA_WIDTH-1 downto FRAME_Y_BITS
    ) <= (others => '0');

    tracking_roi_width_s(
        C_S_AXI_TRACK_DATA_WIDTH-1 downto FRAME_X_BITS
    ) <= (others => '0');

    tracking_roi_height_s(
        C_S_AXI_TRACK_DATA_WIDTH-1 downto FRAME_Y_BITS
    ) <= (others => '0');


    predicted_center_x_s(
        C_S_AXI_TRACK_DATA_WIDTH-1 downto PREDICTED_CENTER_X_BITS
    ) <= (
        others =>
            predicted_center_x_s(PREDICTED_CENTER_X_BITS-1)
    );

    predicted_center_y_s(
        C_S_AXI_TRACK_DATA_WIDTH-1 downto PREDICTED_CENTER_Y_BITS
    ) <= (
        others =>
            predicted_center_y_s(PREDICTED_CENTER_Y_BITS-1)
    );

    ------------------------------------------------------------------------
    -- Allow the video pipeline to advance when:
    --   * no valid output pixel is currently waiting, or
    --   * the downstream block is ready to accept the current pixel.
    --
    -- If a valid output pixel is waiting and the downstream block is not
    -- ready, the entire video pipeline is paused.
    ------------------------------------------------------------------------
    video_pipeline_enable_s <=
        (not video_out_tvalid_s) or video_out_tready;
        
    video_out_tvalid <= video_out_tvalid_s;

	-- User logic ends

end arch_imp;
