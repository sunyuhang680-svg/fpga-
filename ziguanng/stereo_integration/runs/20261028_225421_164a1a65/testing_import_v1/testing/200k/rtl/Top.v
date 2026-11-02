`timescale 1ns / 1ps
// 每个工程目录各有一个 build_cfg.vh（在 include 路径里）：
//   prj/build_cfg.vh      直通（默认值）
//   prj_ref/build_cfg.vh  原工程 ISP 通路 + 串口抓图上传，用作颜色校准基准
`include "build_cfg.vh"
`ifndef CFG_DIRECT_VIDEO
`define CFG_DIRECT_VIDEO 1
`endif
`ifndef CFG_UART_UPLOAD
`define CFG_UART_UPLOAD 0
`endif
module Top#(
    parameter MEM_ROW_ADDR_WIDTH = 15,
    parameter MEM_COL_ADDR_WIDTH = 10,
    parameter MEM_BADDR_WIDTH    = 3,
    parameter MEM_DQ_WIDTH       = 32,
    parameter MEM_DQS_WIDTH      = 32/8,

    // 0：把摄像头0完整送入左右两个显示通道
    // 1：把摄像头1完整送入左右两个显示通道
    parameter TEST_CAMERA        = 1,

    // 1：向DDR写入固定红色，用来测试时序
    // 0：向DDR写入真实摄像头图像
    parameter TEST_SOLID_RED     = 0,
    parameter TOUCH_CLK_HZ = 27000000,
    parameter TOUCH_SPI_HZ = 100000,
    parameter TOUCH_SPI_PROBE_ONLY = 0,
    parameter TOUCH_RCVBC_SCOPE_TEST = 4, // A5: one guarded first-descriptor recovery, then full enumeration
    parameter TOUCH_DEBUG_DISPLAY = 0, // normal screen: cross marks and coordinates at each touch
    parameter TOUCH_MOSI_METER_TEST = 0,
    parameter TOUCH_MOSI_TEST_HZ = 1000,
    parameter MENU_PIXEL_CLK_HZ = 74250000,
    parameter IMU_CLK_HZ = 27000000,
    parameter IMU_SPI_HZ = 2250000,
    parameter IMU_DEBUG_DISPLAY = 1,
    // ---------------- 无DDR直通（RAW8 720p60） ----------------
    // 1：摄像头 RAW8 1280x720@60 → FPGA去马赛克 → 行缓存 → HDMI，DDR不参与显示
    // 0：恢复原来的 RGB565 + DDR 帧缓存通路
    parameter DIRECT_VIDEO = `CFG_DIRECT_VIDEO,
    parameter DIRECT_CAMERA = 0,          // 直通显示哪一路摄像头：0 或 1
    parameter BAYER_PATTERN = 0,          // 去马赛克上电默认排列：0 BGGR 1 GBRG 2 GRBG 3 RGGB（10-28 上升沿采样后抓图确认为 0）
    parameter SENSOR_BAYER_SEQ = 0,       // 写入 OV5640 0x4300[3:0]；上板确认用 0（输出 BGGR）
    // 1：保留原串口抓图上传；0：屏蔽上传，uart_tx 改为每秒打印帧率状态
    parameter ENABLE_UART_UPLOAD = `CFG_UART_UPLOAD,
    parameter CAM0_SAMPLE_NEGEDGE = 0,    // 原工程通路摄像头0采样沿：0 上升沿（10-28 抓图确认下降沿采样会错）
    parameter STATUS_UART_BAUD = 115200,
    parameter UART_CAPTURE_CLK_HZ = 74250000,
    parameter UART_CAPTURE_BAUD = 921600,
    parameter UART_EXPORT_NATIVE = 1, // 1: native 1280x720; 0: existing 640x720
    parameter UART_CAPTURE_CAMERAS = 1, // 抓图上传等几路摄像头：1 只有摄像头0（10-28 板上只接了一路）；2 双目
    // Upper functions consume a read-only tap AFTER the supplied color pipeline.
    parameter ENABLE_DDR_VIEW_RENDER = 1,
    parameter ENABLE_DDR_TEST_BARS = 0,
    parameter ENABLE_VIEW_GESTURES = 1,
    parameter ENABLE_ALGORITHM = 1,
    parameter ENABLE_AUX_READ = 1,
    parameter ENABLE_CMP_CONTROLS = 1,
    parameter ENABLE_MEASUREMENT = 1,
    parameter ENABLE_LIVE_STREAM = 0,
    parameter CAM_RATE_OVERLAY = 1,
    parameter DISPLAY_STARTUP_DELAY_MS = 1000,
    // 1 only after RAW8 mono lens/crop/ray constants are bound for both cameras.
    parameter RAW8_CALIBRATION_VALID = 0,
    // Separate synchronized-pair stereo R/T qualification; independent mono captures cannot authorize measurement.
    parameter RAW8_STEREO_CALIBRATION_VALID = 0,
    parameter IMU_SCOPE_TEST = 0 // 1: continuous pin test; 0: normal IMU driver
)(
    // Existing Mini Base USER_BUTTON1/2 upper UI controls.
    input wire key_mode_n,
    input wire key_menu_n,
    input wire uart_rx,
    output wire uart_tx,
	// ICM42688: J26.12 SCLK, .4 MOSI, .5 MISO, .10 CS; L7 at 3.3 V.
	input wire imu_miso,
	output wire imu_sclk, imu_mosi, imu_cs_n,
	input wire touch_miso,
    input wire touch_int_n,
    output wire touch_sclk, touch_mosi, touch_cs_n,
    output wire touch_reset_n, touch_vbus_en,
    input   wire                                sys_clk        ,//27Mhz
    input   wire                                clk_p          ,
    input   wire                                clk_n          ,
    input   wire                                rst_in         ,
    //DDR
    output  wire                                mem_rst_n      ,
    output  wire                                mem_ck         ,
    output  wire                                mem_ck_n       ,
    output  wire                                mem_cke        ,
    output  wire                                mem_cs_n       ,
    output  wire                                mem_ras_n      ,
    output  wire                                mem_cas_n      ,
    output  wire                                mem_we_n       ,
    output  wire                                mem_odt        ,
    output  wire    [MEM_ROW_ADDR_WIDTH-1:0]    mem_a          ,
    output  wire    [MEM_BADDR_WIDTH-1:0]       mem_ba         ,
    inout   wire    [MEM_DQ_WIDTH/8-1:0]        mem_dqs        ,
    inout   wire    [MEM_DQ_WIDTH/8-1:0]        mem_dqs_n      ,
    inout   wire    [MEM_DQ_WIDTH-1:0]          mem_dq         ,
    output  wire    [MEM_DQ_WIDTH/8-1:0]        mem_dm         ,
    output  reg                                 heart_beat_led ,
    //cam
    input   wire                                camera_clk_0   ,
    input   wire    [7:0]                       camera_data_0  ,
    input   wire                                camera_href_0  ,
    input   wire                                camera_vsync_0 ,
    output  wire                                SCL_0          ,
    output  wire                                SDA_0          ,
    output  wire                                cam_rst_0      ,
    //cam   
    input   wire                                camera_clk_1   ,
    input   wire    [7:0]                       camera_data_1  ,
    input   wire                                camera_href_1  ,
    input   wire                                camera_vsync_1 ,
    output  wire                                SCL_1          ,
    output  wire                                SDA_1          ,
    output  wire                                cam_rst_1      ,
    //hdmi                                                            
    output  wire    [2:0]                       tmds_data_p    ,
    output  wire    [2:0]                       tmds_data_n    ,
    output  wire                                tmds_clk_p     ,
    output  wire                                tmds_clk_n     ,

    output  reg                                 cam_led0       ,
    output  reg                                 cam_led1       
);
//=====================================================================================
//reg define
//=====================================================================================
// ENABLE_DDR
parameter CTRL_ADDR_WIDTH = MEM_ROW_ADDR_WIDTH + MEM_BADDR_WIDTH + MEM_COL_ADDR_WIDTH;//28
parameter TH_1S = 27'd33000000;
wire  ddr_init_done;
assign  LCD_EN   = 1'b1;
//=====================================================================================
//clk
//=====================================================================================
wire    pclk;
wire    pclkx5;
wire    locked0;
wire    rst_n;
wire            cam_vsync_0;
wire            cam_href_0;
wire            cam_write_en_0;
wire    [15:0]  cam_data_0;

// Snapshot control: native frames remain in DDR while UART drains them.
wire upload_busy,upload_freeze;
wire frozen_0,frozen_1;
wire [31:0] write_frame_id_0,write_frame_id_1,frozen_frame_id_0,frozen_frame_id_1;
wire [31:0] write_commit_id_0,write_commit_id_1;
wire [1:0] write_commit_index_0,write_commit_index_1;
wire write_finish_0,write_finish_1;
reg [31:0] completed_id_0,completed_id_1;
reg completed_valid_0,completed_valid_1;
wire cap_read_req_0,cap_read_req_1,cap_read_en_0,cap_read_en_1;
wire [1:0] capture_index_0,capture_index_1;
reg cam0_pixel_select;

always @(posedge camera_clk_0 or negedge rst_n) begin
    if (!rst_n)
        cam0_pixel_select <= 1'b0;
    else if (cam_vsync_0)
        cam0_pixel_select <= 1'b0;
    else if (!cam_href_0)
        cam0_pixel_select <= 1'b0;
    else if (cam_write_en_0)
        cam0_pixel_select <= ~cam0_pixel_select;
end

wire cam0_half_write_en;

assign cam0_half_write_en =
       cam_write_en_0 &&
       cam_href_0 &&
       !cam0_pixel_select;
//===================================写入申明区域=========================================

clk_hdmi clk_hdmi_inst (
  .clkout0(pclk),    // output
  .clkout1(pclkx5 ),    // output
  .lock(locked0),          // output
  .clkin1(sys_clk)       // input
);

assign rst_n = locked0 & rst_in;

reg [15:0] cam0_data_negedge;
reg        cam0_write_en_negedge;

always @(negedge camera_clk_0 or negedge rst_n) begin
    if (!rst_n) begin
        cam0_data_negedge     <= 16'd0;
        cam0_write_en_negedge <= 1'b0;
    end
    else begin
        cam0_data_negedge     <= cam_data_0;
        cam0_write_en_negedge <= cam0_half_write_en;
    end
end

//===========================================================================
// camera
//===========================================================================

wire            cam_vsync_1;
wire            cam_href_1;
wire            cam_write_en_1;
wire    [15:0]  cam_data_1;

wire            cam_init_done_0;
wire            cam_init_done_1;
//===========================================================================
// 摄像头0输入时序测试
//
// 摄像头原始DATA/HREF/VSYNC先在PCLK下降沿锁存，
// camera_data模块再在下一个PCLK上升沿使用这些稳定信号。
//===========================================================================
reg [7:0] camera_data_0_neg;
reg       camera_href_0_neg;
reg       camera_vsync_0_neg;

generate if (CAM0_SAMPLE_NEGEDGE != 0) begin : g_cam0_negedge
always @(negedge camera_clk_0 or negedge rst_n) begin
    if (!rst_n) begin
        camera_data_0_neg  <= 8'd0;
        camera_href_0_neg  <= 1'b0;
        camera_vsync_0_neg <= 1'b0;
    end
    else begin
        camera_data_0_neg  <= camera_data_0;
        camera_href_0_neg  <= camera_href_0;
        camera_vsync_0_neg <= camera_vsync_0;
    end
end
end else begin : g_cam0_posedge
// 上升沿采样（名字沿用 _neg，免得改动下游）
always @(posedge camera_clk_0 or negedge rst_n) begin
    if (!rst_n) begin
        camera_data_0_neg  <= 8'd0;
        camera_href_0_neg  <= 1'b0;
        camera_vsync_0_neg <= 1'b0;
    end
    else begin
        camera_data_0_neg  <= camera_data_0;
        camera_href_0_neg  <= camera_href_0;
        camera_vsync_0_neg <= camera_vsync_0;
    end
end
end endgenerate

camera #(
    .RAW8_60FPS     ( DIRECT_VIDEO  ),
    .RAW8_BAYER_SEQ ( SENSOR_BAYER_SEQ )
) camera_inst0(
    /*input   wire            */.clk           ( sys_clk             )  ,
    /*input   wire            */.rst_n         ( rst_n               )  ,
    /*output  wire            */.SCL           ( SCL_0               )  ,  
    /*output  wire            */.SDA           ( SDA_0               )  ,  
    /*input   wire            */.camera_clk    ( camera_clk_0        )  ,
    /*input   wire    [7:0]   */.camera_data   ( camera_data_0_neg       )  ,
    /*input   wire            */.camera_herf   ( camera_href_0_neg      )  ,
    /*input   wire            */.camera_vsync  ( camera_vsync_0_neg    )  ,
    /*input   wire            */.ddr_init      ( ddr_init_done       )  ,
    /*output  wire            */.camera_rstn   ( cam_rst_0           )  ,
    /*output  wire            */.camera_pwnd   ()  ,
    /*output  wire            */.init_done     ( cam_init_done_0     )  ,
    /*output  wire    [15:0]  */.wf_wr_data    ( cam_data_0          )  ,   //RGB565
    /*output  wire            */.wf_wr_en      ( cam_write_en_0      )  ,
    /*output  wire            */.vs            ( cam_vsync_0         )  ,
    /*output  wire            */.hs            ( cam_href_0          )  ,
    /*output  wire            */.sop           ()  ,
    /*output  wire            */.eop           ()  
);

camera #(
    .RAW8_60FPS     ( DIRECT_VIDEO  ),
    .RAW8_BAYER_SEQ ( SENSOR_BAYER_SEQ )
) camera_inst1(
    /*input   wire            */.clk           ( sys_clk             )  ,
    /*input   wire            */.rst_n         ( rst_n  )  ,
    /*output  wire            */.SCL           ( SCL_1               )  ,  
    /*output  wire            */.SDA           ( SDA_1               )  ,  
    /*input   wire            */.camera_clk    ( camera_clk_1        )  ,
    /*input   wire    [7:0]   */.camera_data   ( camera_data_1       )  ,
    /*input   wire            */.camera_herf   ( camera_href_1       )  ,
    /*input   wire            */.camera_vsync  ( camera_vsync_1      )  ,
    /*input   wire            */.ddr_init      ( ddr_init_done       )  ,
    /*output  wire            */.camera_rstn   ( cam_rst_1           )  ,
    /*output  wire            */.camera_pwnd   ()  ,
    /*output  wire            */.init_done     ( cam_init_done_1     )  ,
    /*output  wire    [15:0]  */.wf_wr_data    ( cam_data_1          )  ,   //RGB565
    /*output  wire            */.wf_wr_en      ( cam_write_en_1      )  ,
    /*output  wire            */.vs            ( cam_vsync_1         )  ,
    /*output  wire            */.hs            ( cam_href_1          )  ,
    /*output  wire            */.sop           ()  ,
    /*output  wire            */.eop           ()  
);


//camera clk signal
reg  [27:0]     cam_cnt_0;
reg             cam_cnt_flag_0;

reg  [27:0]     cam_cnt_1;
reg             cam_cnt_flag_1;

always @(posedge camera_clk_0 or negedge rst_n) begin
    if (!rst_n) begin
        cam_cnt_0 <= 28'd0;
        cam_cnt_flag_0 <= 1'b0;
    end
    else if(cam_cnt_0 == 28'd24_000_000) begin
        cam_cnt_0 <= 28'd0;
        cam_cnt_flag_0 <= 1'b1;
    end
    else begin
        cam_cnt_0 <= cam_cnt_0 + 28'd1;
        cam_cnt_flag_0 <= 1'b0;
    end
end

always @(posedge camera_clk_1 or negedge rst_n) begin
    if (!rst_n) begin
        cam_cnt_1 <= 28'd0;
        cam_cnt_flag_1 <= 1'b0;
    end
    else if(cam_cnt_1 == 28'd24_000_000) begin
        cam_cnt_1 <= 28'd0;
        cam_cnt_flag_1 <= 1'b1;
    end
    else begin
        cam_cnt_1 <= cam_cnt_1 + 28'd1;
        cam_cnt_flag_1 <= 1'b0;
    end
end

always @(posedge camera_clk_0 or negedge rst_n) begin
    if (!rst_n)
        cam_led0 <= 1'b1;
    else if(cam_cnt_flag_0)
        cam_led0 <= ~cam_led0;
end

always @(posedge camera_clk_1 or negedge rst_n) begin
    if (!rst_n)
        cam_led1 <= 1'b1;
    else if(cam_cnt_flag_1)
        cam_led1 <= ~cam_led1;
end

wire       write_req_0;
wire       write_req_ack_0;
wire [1:0] write_addr_index_0;
wire       write_req_1;
wire       write_req_ack_1;
wire [1:0] write_addr_index_1;
wire core_clk;
wire        dv_hs, dv_vs, dv_de;
// Supplied direct preview + independent, post-CCM stereo snapshot taps.
wire cp0_valid,cp1_valid;
wire [10:0] cp0_x,cp0_y,cp1_x,cp1_y;
wire [23:0] cp0_rgb,cp1_rgb;
wire capture_vsync_0,capture_vsync_1;
wire capture_start_0,capture_start_1,capture_complete_0,capture_complete_1;
wire capture_abort_0,capture_abort_1,capture_fault_0,capture_fault_1;
wire capture_ddr_finish_0,capture_ddr_finish_1;
wire [3:0] capture_valid_core_0,capture_valid_core_1;
wire [3:0] capture_valid_camera_0,capture_valid_camera_1;
(* ASYNC_REG="TRUE" *) reg [1:0] testing60_capture_reset_0,testing60_capture_reset_1;
wire testing60_capture_reset_n=rst_n && ddr_init_done;
always @(posedge camera_clk_0 or negedge testing60_capture_reset_n)
 if(!testing60_capture_reset_n)testing60_capture_reset_0<=0;
 else testing60_capture_reset_0<={testing60_capture_reset_0[0],1'b1};
always @(posedge camera_clk_1 or negedge testing60_capture_reset_n)
 if(!testing60_capture_reset_n)testing60_capture_reset_1<=0;
 else testing60_capture_reset_1<={testing60_capture_reset_1[0],1'b1};

//=====================================================================================
//video_scale
//=====================================================================================
//===========================================================================
// both processing chains share the selected camera
//===========================================================================
wire [15:0] write_data_0;
wire        write_en_0;

wire [15:0] write_data_1;
wire        write_en_1;


testing60_rgb_capture u_testing60_capture_0 (
 .clk(camera_clk_0),.rst_n(testing60_capture_reset_0[1]),.capture_enable(1'b1),
 .tap_cp_valid(cp0_valid),.tap_cp_x(cp0_x),.tap_cp_y(cp0_y),.tap_cp_rgb(cp0_rgb),
 .write_req(write_req_0),.write_req_ack(write_req_ack_0),
 .frame_vsync(capture_vsync_0),.write_en(write_en_0),.write_data(write_data_0),
 .frame_start(capture_start_0),.frame_complete(capture_complete_0),.frame_abort(capture_abort_0),
 .capture_fault(capture_fault_0),.fault_code(),.busy());
testing60_frame_quality u_testing60_quality_0 (
 .camera_clk(camera_clk_0),.core_clk(core_clk),.rst_n(rst_n && ddr_init_done),
 .frame_boundary(capture_vsync_0),.frame_start(capture_start_0),.frame_complete(capture_complete_0),.frame_abort(capture_abort_0),
 .write_index(write_addr_index_0),.write_req_ack(write_req_ack_0),.ddr_write_finish(capture_ddr_finish_0),
 .valid_core(capture_valid_core_0),.valid_camera(capture_valid_camera_0));

testing60_rgb_capture u_testing60_capture_1 (
 .clk(camera_clk_1),.rst_n(testing60_capture_reset_1[1]),.capture_enable(1'b1),
 .tap_cp_valid(cp1_valid),.tap_cp_x(cp1_x),.tap_cp_y(cp1_y),.tap_cp_rgb(cp1_rgb),
 .write_req(write_req_1),.write_req_ack(write_req_ack_1),
 .frame_vsync(capture_vsync_1),.write_en(write_en_1),.write_data(write_data_1),
 .frame_start(capture_start_1),.frame_complete(capture_complete_1),.frame_abort(capture_abort_1),
 .capture_fault(capture_fault_1),.fault_code(),.busy());
testing60_frame_quality u_testing60_quality_1 (
 .camera_clk(camera_clk_1),.core_clk(core_clk),.rst_n(rst_n && ddr_init_done),
 .frame_boundary(capture_vsync_1),.frame_start(capture_start_1),.frame_complete(capture_complete_1),.frame_abort(capture_abort_1),
 .write_index(write_addr_index_1),.write_req_ack(write_req_ack_1),.ddr_write_finish(capture_ddr_finish_1),
 .valid_core(capture_valid_core_1),.valid_camera(capture_valid_camera_1));

wire [1:0] read_addr_index_0;

wire [1:0] read_addr_index_1;

// UI state crosses from sys_clk to pixel/camera domains through synchronizers.
wire ui_image_mode,ui_menu_visible,ui_state_event,ui_pixel_event;
wire measurement_block_touch,measurement_active,measurement_force_static,measurement_allow_left,measurement_allow_right;
wire [23:0] measurement_video_rgb;
wire measurement_video_de,measurement_video_hs,measurement_video_vs;
wire [1:0] ui_layout,ui_feature,ui_zoom_source;
wire ui_zoom_active,ui_interaction_active;
wire [16:0] ui_left_step,ui_right_step;
wire [1:0] ui_left_rotation,ui_right_rotation;
wire signed [27:0] ui_left_origin_x,ui_left_origin_y,ui_right_origin_x,ui_right_origin_y;
wire [28:0] ui_pixel_state;
wire vs_out_sync;
wire left_render_front,right_render_front;
wire left_render_ready,right_render_ready;
wire ui_pixel_image_mode=ui_pixel_state[28];
wire ui_pixel_menu_visible=ui_pixel_state[27];
wire [1:0] ui_pixel_layout=ui_pixel_state[26:25];
wire [1:0] ui_pixel_feature=ui_pixel_state[24:23];
wire ui_pixel_zoom_active=ui_pixel_state[22];
wire [1:0] ui_pixel_zoom_source=ui_pixel_state[21:20];
wire ui_pixel_post_mode=ui_pixel_state[19];
wire [2:0] ui_pixel_post_tool=ui_pixel_state[18:16];
wire ui_pixel_post_page=ui_pixel_state[15];
wire [2:0] ui_pixel_press_code=ui_pixel_state[14:12];
wire ui_pixel_press_toggle=ui_pixel_state[11];
wire [2:0] ui_pixel_sub_color=ui_pixel_state[10:8];
wire [7:0] ui_pixel_knob_value=ui_pixel_state[7:0];
wire ui_post_mode,ui_post_page,ui_press_toggle;
wire [2:0] ui_post_tool,ui_press_code,ui_sub_color;
wire [7:0] ui_knob_value;
wire [127:0] ui_knob_values;
// CMP switches, bit 0 = LEFT CMP page (camera 0), bit 1 = RIGHT CMP page.
wire [1:0] ui_cmp_undistort,ui_cmp_binary;
wire [1:0] ui_pixel_cmp_undistort,ui_pixel_cmp_binary;
wire cmp_binary_pixel;
wire ui_cmp_layout=(ENABLE_CMP_CONTROLS!=0) && (ui_layout==2'd1 || ui_layout==2'd2);
// Transfer layout and both views as ONE acknowledged mailbox transaction.
// Geometry is computed in core_clk; each renderer snapshots it in S_LATCH
// and keeps it fixed until its entire next display frame has been written.
wire [281:0] ui_render_state;
// Display readers remain sequential. All spatial mapping is performed by
// the off-screen renderer, never by stretching/pausing the display FIFO.
wire [1:0] left_view_scale=0,right_view_scale=0;
wire [11:0] left_source_x=0,left_source_y=0,left_source_w=640,left_source_h=720;
wire [11:0] right_source_x=0,right_source_y=0,right_source_w=640,right_source_h=720;
wire [11:0] left_destination_x=0,left_destination_y=0,right_destination_x=0,right_destination_y=0;
// The view renderer publishes only complete DDR frames.  Synchronize its
// front-buffer selection into the pixel domain before the sequential readers
// latch a new display frame.
(* ASYNC_REG="TRUE" *) reg [1:0] left_render_front_pclk,right_render_front_pclk;
(* ASYNC_REG="TRUE" *) reg [1:0] left_render_ready_pclk,right_render_ready_pclk;
always @(posedge pclk or negedge rst_n) begin
    if(!rst_n) begin
        left_render_front_pclk<=0;
        right_render_front_pclk<=0;
        left_render_ready_pclk<=0;
        right_render_ready_pclk<=0;
    end else begin
        left_render_front_pclk<={left_render_front_pclk[0],left_render_front};
        right_render_front_pclk<={right_render_front_pclk[0],right_render_front};
        left_render_ready_pclk<={left_render_ready_pclk[0],left_render_ready};
        right_render_ready_pclk<={right_render_ready_pclk[0],right_render_ready};
    end
end
wire photo_hold,photo_memory_ready,photo_held0,photo_held1;
wire [1:0] photo_bank0,photo_bank1;
wire ui_hold_active0,ui_hold_active1;
(* ASYNC_REG="TRUE" *) reg [1:0] photo_pixel_reset;
always @(posedge pclk or negedge rst_n)
 if(!rst_n)photo_pixel_reset<=0;else photo_pixel_reset<={photo_pixel_reset[0],1'b1};
wire photo_pixel_rst_n=photo_pixel_reset[1];
(* ASYNC_REG="TRUE" *) reg [1:0] photo_ddr_ready;
always @(posedge pclk or negedge photo_pixel_rst_n)
 if(!photo_pixel_rst_n)photo_ddr_ready<=0;else photo_ddr_ready<={photo_ddr_ready[0],ddr_init_done};
assign photo_memory_ready=photo_ddr_ready[1];
testing60_photo_hold_mailbox photo_hold0(.camera_clk(camera_clk_0),.pixel_clk(pclk),.rst_n(rst_n),
 .ui_hold_active(ui_hold_active0),.completed_valid(capture_valid_camera_0),.completed_index(read_addr_index_0),
 .held_valid(photo_held0),.held_index(photo_bank0));
testing60_photo_hold_mailbox photo_hold1(.camera_clk(camera_clk_1),.pixel_clk(pclk),.rst_n(rst_n),
 .ui_hold_active(ui_hold_active1),.completed_valid(capture_valid_camera_1),.completed_index(read_addr_index_1),
 .held_valid(photo_held1),.held_index(photo_bank1));
(* ASYNC_REG="TRUE" *) reg [1:0] freeze_cam0_sync,freeze_cam1_sync;
wire algorithm_freeze_cam0,algorithm_freeze_cam1;
wire left_snapshot_request,right_snapshot_request,left_snapshot_grant,right_snapshot_grant;
wire left_snapshot_camera,right_snapshot_camera;
wire [24:0]left_snapshot_base,right_snapshot_base;
wire [31:0]left_snapshot_tag,right_snapshot_tag;
always @(posedge camera_clk_0 or negedge rst_n) begin
    if(!rst_n) freeze_cam0_sync<=0;
    else freeze_cam0_sync<={freeze_cam0_sync[0],photo_hold};
end
always @(posedge camera_clk_1 or negedge rst_n) begin
    if(!rst_n) freeze_cam1_sync<=0;
    else freeze_cam1_sync<={freeze_cam1_sync[0],photo_hold};
end

// Keep only complete frozen images; enter outside the capture publication edge.
wire testing60_freeze_cam0,testing60_freeze_cam1;
testing60_freeze_guard u_testing60_freeze_guard_0 (
    .camera_clk(camera_clk_0),.rst_n(rst_n),
    .ui_image_mode_sync(freeze_cam0_sync[1]),.camera_vsync(capture_vsync_0),
    .completed_index(read_addr_index_0),.completed_valid(capture_valid_camera_0),
    .algorithm_freeze(algorithm_freeze_cam0),.combined_freeze(testing60_freeze_cam0),.ui_hold_active(ui_hold_active0));
testing60_freeze_guard u_testing60_freeze_guard_1 (
    .camera_clk(camera_clk_1),.rst_n(rst_n),
    .ui_image_mode_sync(freeze_cam1_sync[1]),.camera_vsync(capture_vsync_1),
    .completed_index(read_addr_index_1),.completed_valid(capture_valid_camera_1),
    .algorithm_freeze(algorithm_freeze_cam1),.combined_freeze(testing60_freeze_cam1),.ui_hold_active(ui_hold_active1));

cmos_write_req_gen cmos_write_req_gen_inst0 (
    .rst             (~rst_n),
    .pclk            (camera_clk_0),
    .cmos_vsync      (capture_vsync_0),
    .freeze_req      (testing60_freeze_cam0),
    .write_req       (write_req_0),
    .write_addr_index(write_addr_index_0),
    .read_addr_index (read_addr_index_0),
    .write_req_ack   (write_req_ack_0)
);

cmos_write_req_gen cmos_write_req_gen_inst1 (
    .rst             (~rst_n),
    .pclk            (camera_clk_1),
    .cmos_vsync      (capture_vsync_1),
    .freeze_req      (testing60_freeze_cam1),
    .write_req       (write_req_1),
    .write_addr_index(write_addr_index_1),
    .read_addr_index (read_addr_index_1),
    .write_req_ack   (write_req_ack_1)
);

//=====================================================================================
//???visa??? 
//=====================================================================================
// 640x720 RGB565, 16 pixels per 256-bit DDR word
// 640 ? 720 ? 16 = 28800
localparam [24:0] FRAME_WORDS = 25'd28800;
// Slots 0/1/2 are the raw camera ring.  The original slot-3 address region is
// much larger than the current 640x720 RGB565 frame (28,800 256-bit words),
// so it safely contains two adjacent processed frames.  They are used as a
// front/back pair: the renderer writes one while HDMI reads the other.
localparam [24:0] LEFT_VIEW_BASE0  = 25'd6220800;
localparam [24:0] LEFT_VIEW_BASE1  = 25'd6249600;
localparam [24:0] RIGHT_VIEW_BASE0 = 25'd14515200;
localparam [24:0] RIGHT_VIEW_BASE1 = 25'd14544000;

parameter X_WIDTH   = 4'd12;
parameter Y_WIDTH   = 4'd12; 
localparam  X_BITS   = 4'd13   ;
localparam  Y_BITS   = 4'd13   ;
localparam  V_TOTAL  = 12'd750 ;
localparam  V_FP     = 12'd5   ;
localparam  V_BP     = 12'd20  ;
localparam  V_SYNC   = 12'd5   ;
localparam  V_ACT    = 12'd720 ;
localparam  H_TOTAL  = 12'd1650;
localparam  H_FP     = 12'd110 ;
localparam  H_BP     = 12'd220 ;
localparam  H_SYNC   = 12'd40  ;
localparam  H_ACT    = 12'd1280;
// Live stream: clocks the DDR reader chain output is delayed to line up with
// the live pipeline (both run on the same timing); checked by tb_live_select.
localparam  LIVE_READER_DELAY = 10;

wire            read_req_0;
wire            read_req_ack_0;
wire            read_en_0;
wire    [15:0]  read_data_0;
wire            read_empty_0;
wire    [12:0]  read_level_0;
    
wire            read_req_1;
wire            read_req_ack_1;
wire            read_en_1;
wire    [15:0]  read_data_1;
wire            read_empty_1;
wire    [12:0]  read_level_1;

wire            hs_out_sync;
wire            de_out_sync;

wire            vs_out_0;
wire            hs_out_0;
wire            de_out_0;

wire            vs_out_1;
wire            hs_out_1;
wire            de_out_1;

wire    [15:0]  vout_data_0;
wire    [15:0]  vout_data_1;

assign hs_out_sync=dv_hs;
assign vs_out_sync=dv_vs;
assign de_out_sync=dv_de;

wire [11:0] measurement_left_display_x,measurement_left_display_y;
wire [11:0] measurement_right_display_x,measurement_right_display_y;
video_rect_read_data video_rect_read_data_inst0(
	.video_clk                  ( pclk           ),
	.rst                        ( ~rst_n         ),
	.video_left_offset          ( 0              ),
	.video_top_offset           ( 0              ),
	.video_width                ( 640            ),
	.video_height	            ( 720            ),
	.view_enable                ( 1'b0             ),
	.view_scale_shift           ( left_view_scale ),
	.view_dst_left              ( left_destination_x ),
	.view_dst_top               ( left_destination_y ),
	.view_source_left           ( left_source_x   ),
	.view_source_width          ( left_source_w   ),
	.view_source_height         ( left_source_h   ),
	.read_req                   ( read_req_0     ),
	.read_req_ack               ( read_req_ack_0 ),
	.read_en                    ( read_en_0      ),
	.read_data                  ( read_data_0    ),
	.read_empty                 ( read_empty_0   ),
	.read_level                 ( read_level_0   ),
	.timing_hs                  ( hs_out_sync    ),
	.timing_vs                  ( vs_out_sync    ),
	.timing_de                  ( de_out_sync    ),
	.timing_data 	            ( 0              ),
	.hs                         ( hs_out_0       ),
	.vs                         ( vs_out_0       ),
	.de                         ( de_out_0       ),
    .x_cnt(measurement_left_display_x),.y_cnt(measurement_left_display_y),
	.vout_data                  ( vout_data_0    )
);

video_rect_read_data video_rect_read_data_inst1(
	.video_clk                  ( pclk           ),
	.rst                        ( ~rst_n         ),
	.video_left_offset          ( 640            ),
	.video_top_offset           ( 0              ),
	.video_width                ( 640            ),
	.video_height	            ( 720            ),
	.view_enable                ( 1'b0              ),
	.view_scale_shift           ( right_view_scale ),
	.view_dst_left              ( right_destination_x ),
	.view_dst_top               ( right_destination_y ),
	.view_source_left           ( right_source_x  ),
	.view_source_width          ( right_source_w  ),
	.view_source_height         ( right_source_h  ),
	.read_req                   ( read_req_1     ),
	.read_req_ack               ( read_req_ack_1 ),
	.read_en                    ( read_en_1      ),
	.read_data                  ( read_data_1    ),
	.read_empty                 ( read_empty_1   ),
	.read_level                 ( read_level_1   ),
	.timing_hs                  ( hs_out_0       ),
	.timing_vs                  ( vs_out_0       ),
	.timing_de                  ( de_out_0       ),
	.timing_data 	            ( vout_data_0    ),
	.hs                         ( hs_out_1       ),
	.vs                         ( vs_out_1       ),
	.de                         ( de_out_1       ),
    .x_cnt(measurement_right_display_x),.y_cnt(measurement_right_display_y),
	.vout_data                  ( vout_data_1    )
);

//===========================================================================
// video output
//===========================================================================
wire [23:0] dv_rgb;
wire [23:0] dv_right_rgb;
wire dv_right_hs,dv_right_vs,dv_right_de;
wire        dv_locked, dv_cam_present, dv_hdmi_frame_toggle, dv_cam_frame_toggle;
wire [10:0] dv_meas_width, dv_meas_height;
wire [15:0] dv_adj_gray, dv_uf_gray, dv_of_gray;
wire        dv_cam_pclk;
wire        status_uart_tx;

generate if (DIRECT_VIDEO != 0) begin : g_direct_video
    assign dv_cam_pclk = (DIRECT_CAMERA != 0) ? camera_clk_1 : camera_clk_0;

    // 串口命令：Bayer 排列、白平衡、采样沿、颜色矩阵/gamma、抓图（见 video_ctrl_uart）
    wire [1:0]   dv_bayer_sel;
    wire         dv_awb_en;
    wire [11:0]  dv_gain_r, dv_gain_b;
    wire         dv_gain_toggle;
    wire         dv_snap_req, dv_snap_tx, dv_snap_busy, dv_status_tx;
    wire [1:0]   dv_snap_mode;
    wire         dv_edge_sel, dv_awb_onepush;
    wire         dv_ccm_en, dv_ccm_tog, dv_lut_we;
    wire [107:0] dv_ccm_coef;
    wire [2:0]   dv_lut_ch;
    wire [7:0]   dv_lut_addr, dv_lut_data;
    wire [7:0]   dv_tap_raw;
    wire         dv_tap_href, dv_tap_vsync, dv_tap_dm_valid, dv_tap_cp_valid;
    wire [10:0]  dv_tap_dm_x, dv_tap_dm_y, dv_tap_cp_x, dv_tap_cp_y;
    wire [23:0]  dv_tap_dm_rgb, dv_tap_cp_rgb;
    video_ctrl_uart #(
        .CLK_HZ(27000000), .BAUD(STATUS_UART_BAUD),
        .DEFAULT_BAYER(BAYER_PATTERN), .DEFAULT_AWB(1'b0)   // 上电用标定好的固定白平衡
    ) u_ctrl_uart (
        .clk(sys_clk), .rst_n(rst_n), .rx(uart_rx),
        .bayer(dv_bayer_sel), .awb_en(dv_awb_en), .edge_sel(dv_edge_sel), .awb_onepush(dv_awb_onepush),
        .snap_req(dv_snap_req), .snap_mode(dv_snap_mode),
        .ccm_en(dv_ccm_en), .ccm_coef(dv_ccm_coef), .ccm_tog(dv_ccm_tog),
        .lut_we(dv_lut_we), .lut_ch(dv_lut_ch), .lut_addr(dv_lut_addr), .lut_data(dv_lut_data)
    );

    // 串口抓图：'r' 原始 Bayer，'p' 颜色处理前，'q' 颜色处理后；发送期间 uart_tx 切给它
    video_snapshot #(
        .WIDTH(1280), .HEIGHT(720), .CLK_HZ(27000000), .BAUD(STATUS_UART_BAUD)
    ) u_snapshot (
        .clk(sys_clk), .rst_n(rst_n), .req(dv_snap_req), .mode(dv_snap_mode),
        .cam_clk(dv_cam_pclk), .raw(dv_tap_raw), .href(dv_tap_href), .vsync(dv_tap_vsync),
        .dm_valid(dv_tap_dm_valid), .dm_x(dv_tap_dm_x), .dm_y(dv_tap_dm_y), .dm_rgb(dv_tap_dm_rgb),
        .cp_valid(dv_tap_cp_valid), .cp_x(dv_tap_cp_x), .cp_y(dv_tap_cp_y), .cp_rgb(dv_tap_cp_rgb),
        .tx(dv_snap_tx), .busy(dv_snap_busy)
    );
    assign status_uart_tx = dv_snap_busy ? dv_snap_tx : dv_status_tx;

    direct_camera_hdmi #(
        .WIDTH(1280), .HEIGHT(720),
        .H_TOTAL(H_TOTAL), .H_SYNC(H_SYNC), .H_BP(H_BP), .H_ACT(H_ACT),
        .V_TOTAL(V_TOTAL), .V_SYNC(V_SYNC), .V_BP(V_BP), .V_ACT(V_ACT)
    ) u_direct_video (
        .rst_n            ( rst_n ),
        .cam_pclk         ( dv_cam_pclk ),
        .cam_data         ( (DIRECT_CAMERA != 0) ? camera_data_1  : camera_data_0  ),
        .cam_href         ( (DIRECT_CAMERA != 0) ? camera_href_1  : camera_href_0  ),
        .cam_vsync        ( (DIRECT_CAMERA != 0) ? camera_vsync_1 : camera_vsync_0 ),
        .cam_cfg_done     ( (DIRECT_CAMERA != 0) ? cam_init_done_1 : cam_init_done_0 ),
        .bayer_sel        ( dv_bayer_sel ),
        .awb_en           ( dv_awb_en ),
        .edge_sel         ( dv_edge_sel ),
        .awb_onepush      ( dv_awb_onepush ),
        .ccm_en(dv_ccm_en), .ccm_coef(dv_ccm_coef), .ccm_tog(dv_ccm_tog),
        .lut_clk(sys_clk), .lut_we(dv_lut_we), .lut_ch(dv_lut_ch),
        .lut_addr(dv_lut_addr), .lut_data(dv_lut_data),
        .pclk             ( pclk ),
        .hs(dv_hs), .vs(dv_vs), .de(dv_de), .rgb(dv_rgb),
        .locked           ( dv_locked ),
        .cam_present      ( dv_cam_present ),
        .hdmi_frame_toggle( dv_hdmi_frame_toggle ),
        .cam_frame_toggle ( dv_cam_frame_toggle ),
        .meas_width       ( dv_meas_width ),
        .meas_height      ( dv_meas_height ),
        .adj_gray(dv_adj_gray), .uf_gray(dv_uf_gray), .of_gray(dv_of_gray),
        .gain_r(dv_gain_r), .gain_b(dv_gain_b), .gain_toggle(dv_gain_toggle),
        .tap_raw(dv_tap_raw), .tap_href(dv_tap_href), .tap_vsync(dv_tap_vsync),
        .tap_dm_valid(dv_tap_dm_valid), .tap_dm_x(dv_tap_dm_x), .tap_dm_y(dv_tap_dm_y),
        .tap_dm_rgb(dv_tap_dm_rgb),
        .tap_cp_valid(dv_tap_cp_valid), .tap_cp_x(dv_tap_cp_x), .tap_cp_y(dv_tap_cp_y),
        .tap_cp_rgb(dv_tap_cp_rgb)
    );

    assign cp0_valid=dv_tap_cp_valid;
    assign cp0_x=dv_tap_cp_x;assign cp0_y=dv_tap_cp_y;assign cp0_rgb=dv_tap_cp_rgb;
    // Identical supplied processing for C1; no feedback to C0 or its HDMI clock.
    direct_camera_hdmi #(
        .WIDTH(1280), .HEIGHT(720),
        .H_TOTAL(H_TOTAL), .H_SYNC(H_SYNC), .H_BP(H_BP), .H_ACT(H_ACT),
        .V_TOTAL(V_TOTAL), .V_SYNC(V_SYNC), .V_BP(V_BP), .V_ACT(V_ACT)
    ) u_direct_video_right (
        .rst_n            ( rst_n ),
        .cam_pclk         ( camera_clk_1 ),
        .cam_data         ( camera_data_1 ),
        .cam_href         ( camera_href_1 ),
        .cam_vsync        ( camera_vsync_1 ),
        .cam_cfg_done     ( cam_init_done_1 ),
        .bayer_sel        ( dv_bayer_sel ),
        .awb_en           ( dv_awb_en ),
        .edge_sel         ( dv_edge_sel ),
        .awb_onepush      ( dv_awb_onepush ),
        .ccm_en(dv_ccm_en), .ccm_coef(dv_ccm_coef), .ccm_tog(dv_ccm_tog),
        .lut_clk(sys_clk), .lut_we(dv_lut_we), .lut_ch(dv_lut_ch),
        .lut_addr(dv_lut_addr), .lut_data(dv_lut_data),
        .pclk             ( pclk ),
        .hs(dv_right_hs), .vs(dv_right_vs), .de(dv_right_de), .rgb(dv_right_rgb),
        .locked(),
        .cam_present(),
        .hdmi_frame_toggle(),
        .cam_frame_toggle(),
        .meas_width(),
        .meas_height(),
        .adj_gray(), .uf_gray(), .of_gray(),
        .gain_r(), .gain_b(), .gain_toggle(),
        .tap_raw(), .tap_href(), .tap_vsync(),
        .tap_dm_valid(), .tap_dm_x(), .tap_dm_y(),
        .tap_dm_rgb(),
        .tap_cp_valid(cp1_valid), .tap_cp_x(cp1_x), .tap_cp_y(cp1_y),
        .tap_cp_rgb(cp1_rgb)
    );

    // 每秒从 uart_tx 打印一行状态（115200 8N1）
    video_status_uart #(
        .CLK_HZ(27000000), .BAUD(STATUS_UART_BAUD)
    ) u_status_uart (
        .clk(sys_clk), .rst_n(rst_n), .cam_clk(dv_cam_pclk),
        .cam_frame_toggle(dv_cam_frame_toggle), .hdmi_frame_toggle(dv_hdmi_frame_toggle),
        .cam_width(dv_meas_width), .cam_height(dv_meas_height),
        .locked(dv_locked), .cam_present(dv_cam_present),
        .adj_gray(dv_adj_gray), .uf_gray(dv_uf_gray), .of_gray(dv_of_gray),
        .bayer(dv_bayer_sel), .awb_en(dv_awb_en), .edge_sel(dv_edge_sel), .ccm_en(dv_ccm_en),
        .gain_r(dv_gain_r), .gain_b(dv_gain_b), .gain_toggle(dv_gain_toggle),
        .tx(dv_status_tx)
    );
end else begin : g_no_direct_video
    assign cp0_valid=0;assign cp1_valid=0;assign cp0_x=0;assign cp0_y=0;assign cp1_x=0;assign cp1_y=0;assign cp0_rgb=0;assign cp1_rgb=0;
    assign dv_right_hs=0;assign dv_right_vs=0;assign dv_right_de=0;assign dv_right_rgb=0;
    assign dv_cam_pclk = 1'b0;
    assign dv_hs = 1'b0; assign dv_vs = 1'b0; assign dv_de = 1'b0; assign dv_rgb = 24'd0;
    assign dv_locked = 1'b0; assign dv_cam_present = 1'b0;
    assign dv_hdmi_frame_toggle = 1'b0; assign dv_cam_frame_toggle = 1'b0;
    assign dv_meas_width = 11'd0; assign dv_meas_height = 11'd0;
    assign dv_adj_gray = 16'd0; assign dv_uf_gray = 16'd0; assign dv_of_gray = 16'd0;
    assign status_uart_tx = 1'b1;
end endgenerate

generate if(DIRECT_VIDEO!=1 || DIRECT_CAMERA!=0 || CAM0_SAMPLE_NEGEDGE!=0 || ENABLE_LIVE_STREAM!=0 || ENABLE_UART_UPLOAD!=0) begin: invalid_testing60_mode
 TESTING60_REQUIRES_SUPPLIED_DIRECT_MODE invalid_configuration();
end endgenerate
assign uart_tx=status_uart_tx;

wire            hs_out   ;
wire            vs_out   ;
wire            de_out   ;
wire    [23:0]  vout_data;

wire base_video_hs,base_video_vs,base_video_de,base_frozen_active;
wire [23:0] base_video_rgb;
wire photo_legacy,photo_failed,photo_busy,photo_release,photo_configuration_event;
wire [2:0] photo_status;
wire photo_reader_req0,photo_reader_req1,photo_accepted_ok0,photo_accepted_ok1;
wire [27:0] photo_selected0,photo_selected1,photo_accepted0,photo_accepted1;
wire [27:0] photo_dynamic0,photo_dynamic1;
testing60_photo_initial_select #(.ID_BITS(28),.READER_DELAY(10)) u_testing60_photo_initial (
    .clk(pclk),.rst_n(photo_pixel_rst_n),.photo_request(ui_pixel_image_mode),.retry(1'b0),
    .memory_ready(photo_memory_ready),.configuration_change(photo_configuration_event),
    .pair_valid(photo_held0&&photo_held1),.pair_left(photo_dynamic0),.pair_right(photo_dynamic1),
    .consumer_req_left(read_req_0),.consumer_req_right(read_req_1),
    .reader_req_left(photo_reader_req0),.reader_req_right(photo_reader_req1),
    .reader_ack_left(read_req_ack_0&&photo_accepted_ok0),.reader_ack_right(read_req_ack_1&&photo_accepted_ok1),
    .accepted_left(photo_accepted0),.accepted_right(photo_accepted1),
    .level_left(read_level_0),.level_right(read_level_1),.pop_left(read_en_0),.pop_right(read_en_1),
    .empty_left(read_empty_0),.empty_right(read_empty_1),.selected_left(photo_selected0),.selected_right(photo_selected1),
    .photo_hold(photo_hold),.legacy_passthrough(photo_legacy),.lease_release(photo_release),
    .failed(photo_failed),.busy(photo_busy),.status(photo_status),
    .direct_hs(dv_hs),.direct_vs(dv_vs),.direct_de(dv_de),.direct_rgb(dv_rgb),
    .frozen_hs(hs_out_1),.frozen_vs(vs_out_1),.frozen_de(de_out_1),.frozen_rgb565(vout_data_1),
    .hs(base_video_hs),.vs(base_video_vs),.de(base_video_de),.rgb(base_video_rgb),.frozen_active(base_frozen_active));
// Live RIGHT CMP selects C1, STEREO/LEFT CMP select C0 (top menu keys only
// change layout; post_page changes only inside EDIT).
// DDR consumers remain clocked by the original C0 timing. Frozen stereo always
// returns to that aligned path. Camera clocks and both color pipelines are untouched.
testing60_live_camera_select #(.READER_DELAY(10)) u_testing60_live_select (
    .clk(pclk),.rst_n(rst_n),.page_right(ui_pixel_layout==2'd2),.frozen_active(base_frozen_active),
    .c0_hs(base_video_hs),.c0_vs(base_video_vs),.c0_de(base_video_de),.c0_rgb(base_video_rgb),
    .c1_hs(dv_right_hs),.c1_vs(dv_right_vs),.c1_de(dv_right_de),.c1_rgb(dv_right_rgb),
    .hs(hs_out),.vs(vs_out),.de(de_out),.rgb(vout_data),.selected_right(),.visible());

// ICM42688 reader; all six signed raw outputs belong to sys_clk.
(* ASYNC_REG="TRUE" *) reg [1:0] imu_reset_sync;
always @(posedge sys_clk or negedge rst_n) begin
    if(!rst_n) imu_reset_sync<=2'b00;
    else imu_reset_sync<={imu_reset_sync[0],1'b1};
end
wire signed [15:0] imu_gyro_x_raw,imu_gyro_y_raw,imu_gyro_z_raw;
wire signed [15:0] imu_accel_x_raw,imu_accel_y_raw,imu_accel_z_raw;
wire imu_data_valid,imu_init_done,imu_error;
wire [3:0] imu_error_code;
wire [7:0] imu_who_am_i,imu_last_int_status;
wire [31:0] imu_sample_count,imu_sample_timestamp;
wire imu_driver_sclk, imu_driver_mosi, imu_driver_cs_n;
wire imu_test_sclk, imu_test_mosi, imu_test_cs_n;
icm42688_pin_test #(.CLK_HZ(IMU_CLK_HZ),.BIT_HZ(2000)) u_imu_pin_test (
    .clk(sys_clk),.rst_n(imu_reset_sync[1]),
    .sclk(imu_test_sclk),.mosi(imu_test_mosi),.cs_n(imu_test_cs_n)
);
assign imu_sclk = IMU_SCOPE_TEST ? imu_test_sclk : imu_driver_sclk;
assign imu_mosi = IMU_SCOPE_TEST ? imu_test_mosi : imu_driver_mosi;
assign imu_cs_n = IMU_SCOPE_TEST ? imu_test_cs_n : imu_driver_cs_n;
icm42688 #(.CLK_HZ(IMU_CLK_HZ),.SPI_HZ(IMU_SPI_HZ)) u_imu (
    .clk(sys_clk),.rst_n(imu_reset_sync[1] && (IMU_SCOPE_TEST == 0)),
    .spi_miso(imu_miso),.spi_sclk(imu_driver_sclk),
    .spi_mosi(imu_driver_mosi),.spi_cs_n(imu_driver_cs_n),
    .gyro_x_raw(imu_gyro_x_raw),.gyro_y_raw(imu_gyro_y_raw),.gyro_z_raw(imu_gyro_z_raw),
    .accel_x_raw(imu_accel_x_raw),.accel_y_raw(imu_accel_y_raw),.accel_z_raw(imu_accel_z_raw),
    .data_valid(imu_data_valid),.init_done(imu_init_done),.error(imu_error),
    .error_code(imu_error_code),.who_am_i(imu_who_am_i),.last_int_status(imu_last_int_status),
    .sample_count(imu_sample_count),.sample_timestamp(imu_sample_timestamp)
);

// Startup attitude alignment and gyro bias, then pure quaternion integration.
wire imu_attitude_ready,imu_angle_valid,imu_attitude_busy,imu_attitude_timing_error;
wire [15:0] imu_calibration_count;
wire signed [31:0] imu_bias_x_q8,imu_bias_y_q8,imu_bias_z_q8;
wire signed [31:0] imu_q0,imu_q1,imu_q2,imu_q3;
wire signed [31:0] imu_roll_q16,imu_pitch_q16,imu_yaw_q16;
wire signed [15:0] imu_roll_cdeg,imu_pitch_cdeg,imu_yaw_cdeg;
icm42688_attitude #(.CLK_HZ(IMU_CLK_HZ)) u_imu_attitude (
    .clk(sys_clk),.rst_n(imu_reset_sync[1] && imu_init_done && !imu_error),
    .data_valid(imu_data_valid),.sample_timestamp(imu_sample_timestamp),
    .gyro_x_raw(imu_gyro_x_raw),.gyro_y_raw(imu_gyro_y_raw),.gyro_z_raw(imu_gyro_z_raw),
    .accel_x_raw(imu_accel_x_raw),.accel_y_raw(imu_accel_y_raw),.accel_z_raw(imu_accel_z_raw),
    .ready(imu_attitude_ready),.angle_valid(imu_angle_valid),.busy(imu_attitude_busy),
    .timing_error(imu_attitude_timing_error),.calibration_count(imu_calibration_count),
    .gyro_bias_x_q8(imu_bias_x_q8),.gyro_bias_y_q8(imu_bias_y_q8),.gyro_bias_z_q8(imu_bias_z_q8),
    .q0(imu_q0),.q1(imu_q1),.q2(imu_q2),.q3(imu_q3),
    .roll_q16(imu_roll_q16),.pitch_q16(imu_pitch_q16),.yaw_q16(imu_yaw_q16),
    .roll_cdeg(imu_roll_cdeg),.pitch_cdeg(imu_pitch_cdeg),.yaw_cdeg(imu_yaw_cdeg)
);

// Added touch subsystem and pixel overlay. Camera/DDR pipeline above is unchanged.
wire touch_event, point0_valid, point1_valid;
wire touch_usb_ready;
wire touch_diagnostic_event;
wire [511:0] touch_diagnostic_data;
wire touch_chip_identified;
wire touch_probe_done,touch_probe_pass,touch_probe_event;
wire [7:0] touch_revision,touch_probe_a,touch_probe_b;
wire [7:0] touch_error_code;
wire [15:0] point0_x,point0_y,point1_x,point1_y;
wire host_sclk,host_mosi,host_cs_n,host_reset_n,host_vbus_en;
wire meter_mosi;
max3421e_mosi_meter #(.CLK_HZ(TOUCH_CLK_HZ),.OUTPUT_HZ(TOUCH_MOSI_TEST_HZ)) u_mosi_meter (
    .clk(sys_clk),.rst_n(rst_n),.mosi_level(meter_mosi)
);
// Meter mode exclusively owns the pins. Keep the USB/SPI host in reset.
assign touch_sclk = TOUCH_MOSI_METER_TEST ? meter_mosi : host_sclk;
assign touch_mosi = TOUCH_MOSI_METER_TEST ? meter_mosi : host_mosi;
assign touch_cs_n = TOUCH_MOSI_METER_TEST ? meter_mosi : host_cs_n;
assign touch_reset_n = TOUCH_MOSI_METER_TEST ? rst_n : host_reset_n;
assign touch_vbus_en = TOUCH_MOSI_METER_TEST ? rst_n : host_vbus_en;
max3421e_touch #(.CLK_HZ(TOUCH_CLK_HZ),.SPI_HZ(TOUCH_SPI_HZ),
    .SPI_PROBE_ONLY(TOUCH_SPI_PROBE_ONLY),
    .RCVBC_SCOPE_TEST(TOUCH_RCVBC_SCOPE_TEST)) u_touch (
    .clk(sys_clk),.rst_n(rst_n && !TOUCH_MOSI_METER_TEST),.spi_miso(touch_miso),.max_irq_n(touch_int_n),
    .spi_sclk(host_sclk),.spi_mosi(host_mosi),.spi_cs_n(host_cs_n),
    .max_reset_n(host_reset_n),.vbus_en(host_vbus_en),
    .touch_event(touch_event),.point0_valid(point0_valid),.point0_id(),
    .point0_x(point0_x),.point0_y(point0_y),
    .point1_valid(point1_valid),.point1_id(),.point1_x(point1_x),.point1_y(point1_y),
    .touch_count(),.touch_overflow(),.usb_connected(),.usb_ready(touch_usb_ready),
    .chip_identified(touch_chip_identified),
    .spi_probe_done(touch_probe_done),.spi_probe_pass(touch_probe_pass),
    .spi_probe_event(touch_probe_event),.revision_raw(touch_revision),
    .probe_read_a(touch_probe_a),.probe_read_b(touch_probe_b),
    .diagnostic_event(touch_diagnostic_event),.diagnostic_data(touch_diagnostic_data),
    .error_code(touch_error_code),.debug_state(),.last_hrsl(),.report_error()
);
stereo_ui_controller #(.CLK_HZ(TOUCH_CLK_HZ),.ENABLE_CMP_CONTROLS(ENABLE_CMP_CONTROLS)) u_ui_controller (
    .clk(sys_clk),.rst_n(rst_n),.key_mode_n(key_mode_n),.key_menu_n(key_menu_n),
    .touch_event(touch_event && !measurement_block_touch),.point0_valid(point0_valid),.point1_valid(point1_valid),
    .point0_x(point0_x),.point0_y(point0_y),.point1_x(point1_x),.point1_y(point1_y),
    .image_mode(ui_image_mode),.menu_visible(ui_menu_visible),.state_event(ui_state_event),
    .layout(ui_layout),.feature(ui_feature),
    .cmp_undistort(ui_cmp_undistort),.cmp_binary(ui_cmp_binary),
    .zoom_active(ui_zoom_active),.zoom_source(ui_zoom_source),
    .interaction_active(ui_interaction_active),
    .post_mode(ui_post_mode),.post_tool(ui_post_tool),.post_page(ui_post_page),
    .press_code(ui_press_code),.press_toggle(ui_press_toggle),
    .sub_color(ui_sub_color),.knob_value(ui_knob_value),.knob_values(ui_knob_values),
    .left_step_q16(ui_left_step),.right_step_q16(ui_right_step),
    .left_rotation(ui_left_rotation),.right_rotation(ui_right_rotation),
    .left_origin_x(ui_left_origin_x),.left_origin_y(ui_left_origin_y),
    .right_origin_x(ui_right_origin_x),.right_origin_y(ui_right_origin_y)
);
reg doodle_press_toggle_d;
always @(posedge sys_clk or negedge rst_n) begin
    if(!rst_n) doodle_press_toggle_d<=1'b0;
    else doodle_press_toggle_d<=ui_press_toggle;
end
wire doodle_clear_pulse=(ui_press_toggle^doodle_press_toggle_d) && (ui_press_code==3'd6);
wire [16:0] left_doodle_address,right_doodle_address;
wire [3:0] left_doodle_index,right_doodle_index;
testing60_doodle_canvas u_left_doodle (
    .sys_clk(sys_clk),.mem_clk(core_clk),.rst_n(rst_n && ddr_init_done),
    .touch_event(touch_event && !measurement_block_touch),.point_valid(point0_valid),.point_x(point0_x),.point_y(point0_y),
    .enable_draw(ui_menu_visible&&ui_post_mode&&!ui_post_page&&(ui_post_tool==3'd0)&&!ui_cmp_layout),
    .enable_erase(ui_menu_visible&&ui_post_mode&&!ui_post_page&&(ui_post_tool==3'd1)&&!ui_cmp_layout),
    .side_right(1'b0),.color(ui_sub_color),
    .clear_event(doodle_clear_pulse&&!ui_post_page),
    .view_step_q16(ui_left_step),.view_origin_x(ui_left_origin_x),.view_origin_y(ui_left_origin_y),
    .read_address(left_doodle_address),.read_data(left_doodle_index)
);
testing60_doodle_canvas u_right_doodle (
    .sys_clk(sys_clk),.mem_clk(core_clk),.rst_n(rst_n && ddr_init_done),
    .touch_event(touch_event && !measurement_block_touch),.point_valid(point0_valid),.point_x(point0_x),.point_y(point0_y),
    .enable_draw(ui_menu_visible&&ui_post_mode&&ui_post_page&&(ui_post_tool==3'd0)&&!ui_cmp_layout),
    .enable_erase(ui_menu_visible&&ui_post_mode&&ui_post_page&&(ui_post_tool==3'd1)&&!ui_cmp_layout),
    .side_right(1'b1),.color(ui_sub_color),
    .clear_event(doodle_clear_pulse&&ui_post_page),
    .view_step_q16(ui_right_step),.view_origin_x(ui_right_origin_x),.view_origin_y(ui_right_origin_y),
    .read_address(right_doodle_address),.read_data(right_doodle_index)
);
// Latest-state mailbox keeps the multi-bit UI state coherent in pixel clock.
touch_event_cdc #(.WIDTH(29)) u_ui_state_cdc (
    .src_clk(sys_clk),.dst_clk(pclk),.rst_n(rst_n),.src_event(ui_state_event),
    .src_data({ui_image_mode,ui_menu_visible,ui_layout,ui_feature,ui_zoom_active,
        ui_zoom_source,ui_post_mode,ui_post_tool,ui_post_page,ui_press_code,
        ui_press_toggle,ui_sub_color,ui_knob_value}),
    .dst_event(ui_pixel_event),.dst_data(ui_pixel_state)
);
generate if(ENABLE_CMP_CONTROLS!=0) begin: cmp_label_state
    // Switch labels only; the pixel-path binary flag comes from the pair gate.
    touch_event_cdc #(.WIDTH(4)) u_cmp_label_cdc (
        .src_clk(sys_clk),.dst_clk(pclk),.rst_n(rst_n),.src_event(ui_state_event),
        .src_data({ui_cmp_undistort,ui_cmp_binary}),
        .dst_event(),.dst_data({ui_pixel_cmp_undistort,ui_pixel_cmp_binary})
    );
end else begin: no_cmp_label_state
    assign ui_pixel_cmp_undistort=2'b00;assign ui_pixel_cmp_binary=2'b00;
end endgenerate
wire [23:0] display_video_rgb;
wire display_video_de,display_video_hs,display_video_vs;
wire [23:0] rate_video_rgb;
wire rate_video_de,rate_video_hs,rate_video_vs;
generate if(CAM_RATE_OVERLAY!=0) begin: cam_rate
    cam_rate_overlay #(.CLK_HZ(MENU_PIXEL_CLK_HZ),.X0(16),.Y0(664)) u_cam_rate_overlay (
        .clk(pclk),.rst_n(rst_n),
        .cam0_clk(camera_clk_0),.cam0_vsync(camera_vsync_0),.cam0_href(camera_href_0),
        .cam1_clk(camera_clk_1),.cam1_vsync(camera_vsync_1),.cam1_href(camera_href_1),
        .hs_in(hs_out),.vs_in(vs_out),.de_in(de_out),.rgb_in(vout_data),
        .hs_out(rate_video_hs),.vs_out(rate_video_vs),.de_out(rate_video_de),.rgb_out(rate_video_rgb));
end else begin: no_cam_rate
    assign rate_video_rgb=vout_data;assign rate_video_de=de_out;assign rate_video_hs=hs_out;assign rate_video_vs=vs_out;
end endgenerate
// Keep IMU acquisition available to later measurement functions, but remove
// its diagnostic overlay completely from the user-facing HDMI path.
stereo_ui_layer #(.PIXEL_CLK_HZ(MENU_PIXEL_CLK_HZ),.ENABLE_CMP_CONTROLS(ENABLE_CMP_CONTROLS)) u_ui_layer (
    .pixel_clk(pclk),.rst_n(rst_n),
    .camera_rgb(rate_video_rgb),.camera_de(rate_video_de),
    .camera_hs(rate_video_hs),.camera_vs(rate_video_vs),
    .image_mode(ui_pixel_image_mode),.menu_visible(ui_pixel_menu_visible),.layout(ui_pixel_layout),
    .feature(ui_pixel_feature),.zoom_active(ui_pixel_zoom_active),.zoom_source(ui_pixel_zoom_source),
    .cmp_undistort(ui_pixel_cmp_undistort),.cmp_binary(ui_pixel_cmp_binary),.cmp_binary_active(cmp_binary_pixel),
    .post_mode(ui_pixel_post_mode),.post_tool(ui_pixel_post_tool),.post_page(ui_pixel_post_page),
    .press_code(ui_pixel_press_code),.press_toggle(ui_pixel_press_toggle),
    .sub_color(ui_pixel_sub_color),.knob_value(ui_pixel_knob_value),
    .video_rgb(display_video_rgb),.video_de(display_video_de),
    .video_hs(display_video_hs),.video_vs(display_video_vs)
);

// The pose cube is enabled only while the menu is open and camera motion is
// above threshold. It sits below touch feedback so the ripple stays topmost.
wire [23:0] pose_video_rgb;
wire pose_video_de,pose_video_hs,pose_video_vs;
testing60_imu_pose_overlay #(.SYS_CLK_HZ(IMU_CLK_HZ),
    .MOTION_THRESHOLD_RAW(250),.MOTION_HOLD_MS(500),
    .ENABLE(IMU_DEBUG_DISPLAY)) u_imu_pose_overlay (
    .sys_clk(sys_clk),.pixel_clk(pclk),.rst_n(rst_n),
    .menu_visible_sys(ui_menu_visible),
    .menu_visible_pixel(ui_pixel_menu_visible),
    .imu_data_valid(imu_data_valid),
    .imu_attitude_ready(imu_attitude_ready),
    .imu_angle_valid(imu_angle_valid),
    .gyro_x_raw(imu_gyro_x_raw),.gyro_y_raw(imu_gyro_y_raw),
    .gyro_z_raw(imu_gyro_z_raw),
    .q0(imu_q0),.q1(imu_q1),.q2(imu_q2),.q3(imu_q3),
    .camera_rgb(display_video_rgb),.camera_de(display_video_de),
    .camera_hs(display_video_hs),.camera_vs(display_video_vs),
    .video_rgb(pose_video_rgb),.video_de(pose_video_de),
    .video_hs(pose_video_hs),.video_vs(pose_video_vs)
);

// Touch feedback is the final compositor before TMDS, so the ripple remains
// visible above camera pixels, the pose cube and all menu controls.
wire [23:0] ui_video_rgb;
wire ui_video_de,ui_video_hs,ui_video_vs;
touch_video_overlay #(.PIXEL_CLK_HZ(MENU_PIXEL_CLK_HZ),
    .SHOW_DEBUG(TOUCH_DEBUG_DISPLAY)) u_overlay (
    .sys_clk(sys_clk),.pixel_clk(pclk),.rst_n(rst_n),
    .diagnostic_event(touch_diagnostic_event),.diagnostic_data(touch_diagnostic_data),
    .chip_identified(touch_chip_identified),
    .spi_probe_event(touch_probe_event),
    .spi_probe_data({touch_probe_pass,touch_revision,touch_probe_a,touch_probe_b}),
    .touch_event(touch_event && !measurement_block_touch),.point0_valid(point0_valid),.point1_valid(point1_valid),
    .point0_x(point0_x),.point0_y(point0_y),.point1_x(point1_x),.point1_y(point1_y),
    .camera_rgb(pose_video_rgb),.camera_de(pose_video_de),
    .camera_hs(pose_video_hs),.camera_vs(pose_video_vs),
    .video_rgb(ui_video_rgb),.video_de(ui_video_de),
    .video_hs(ui_video_hs),.video_vs(ui_video_vs)
);

// DDR becomes readable before either camera has filled a complete frame.
// Wait for all initializers, add a guard interval, then unblank strictly at a
// frame edge. This avoids exposing unwritten DDR contents during power-up.
(* ASYNC_REG="TRUE" *) reg [1:0] ddr_ready_sys;
always @(posedge sys_clk or negedge rst_n) begin
    if(!rst_n) begin
        ddr_ready_sys<=2'b00;
    end else begin
        ddr_ready_sys<={ddr_ready_sys[0],ddr_init_done};
    end
end
// Do not let an optional post-processing path blank the whole UI.  The display
// starts once DDR and both cameras are initialized; each half switches from
// its proven camera buffer to the rendered buffer only after that renderer has
// published a complete frame.
wire video_prerequisites_ready=cam_init_done_0;
wire [23:0] guarded_video_rgb;
wire guarded_video_de,guarded_video_hs,guarded_video_vs;
wire display_startup_ready;
video_startup_guard #(.SYS_CLK_HZ(TOUCH_CLK_HZ),
    .DELAY_MS(DISPLAY_STARTUP_DELAY_MS)) u_video_startup_guard (
    .sys_clk(sys_clk),.pixel_clk(pclk),.rst_n(rst_n),
    .prerequisites_ready(video_prerequisites_ready),
    .input_rgb(measurement_video_rgb),.input_de(measurement_video_de),
    .input_hs(measurement_video_hs),.input_vs(measurement_video_vs),
    .output_rgb(guarded_video_rgb),.output_de(guarded_video_de),
    .output_hs(guarded_video_hs),.output_vs(guarded_video_vs),
    .display_ready(display_startup_ready)
);

wire [23:0] testing60_final_rgb;
wire testing60_final_de,testing60_final_hs,testing60_final_vs;
wire [23:0] photo_notice_rgb;
wire photo_notice_de,photo_notice_hs,photo_notice_vs;
testing60_calibration_notice #(.MEAS_ONLY(RAW8_CALIBRATION_VALID && !RAW8_STEREO_CALIBRATION_VALID)) u_testing60_calibration_notice(
 .clk(pclk),.rst_n(rst_n),.show((!RAW8_CALIBRATION_VALID || !RAW8_STEREO_CALIBRATION_VALID) && ui_pixel_image_mode && ui_pixel_menu_visible),
 .rgb_in(guarded_video_rgb),.de_in(guarded_video_de),.hs_in(guarded_video_hs),.vs_in(guarded_video_vs),
 .rgb_out(photo_notice_rgb),.de_out(photo_notice_de),.hs_out(photo_notice_hs),.vs_out(photo_notice_vs));
testing60_photo_notice u_testing60_photo_notice(.clk(pclk),.rst_n(photo_pixel_rst_n),.show(ui_pixel_image_mode),.status(photo_status),
 .rgb_in(photo_notice_rgb),.de_in(photo_notice_de),.hs_in(photo_notice_hs),.vs_in(photo_notice_vs),
 .rgb_out(testing60_final_rgb),.de_out(testing60_final_de),.hs_out(testing60_final_hs),.vs_out(testing60_final_vs));

rgb2tmds rgb2tmds_inst (
    /*output  wire            */.tmds_clk_p  ( tmds_clk_p   ) ,
    /*output  wire            */.tmds_clk_n  ( tmds_clk_n   ) ,
    /*output  wire    [2:0]   */.tmds_data_p ( tmds_data_p  ) ,
    /*output  wire    [2:0]   */.tmds_data_n ( tmds_data_n  ) ,

    /*input   wire            */.rstn        ( rst_n        ) ,

    /*input   wire    [23:0]  */.vid_pdata   ( testing60_final_rgb ) ,
    /*input   wire            */.vid_pvde    ( testing60_final_de ) ,
    /*input   wire            */.vid_phsync  ( testing60_final_hs ) ,
    /*input   wire            */.vid_pvsync  ( testing60_final_vs ) ,
 
    /*input   wire            */.pixelclk    ( pclk         ) ,
    /*input   wire            */.serialclk   ( pclkx5       ) 
);

//=====================================================================================
// ddr write path
//=====================================================================================
parameter MEM_DATA_BITS          = 256;             //external memory user interface data width
parameter ADDR_BITS              = 25;             //external memory user interface address width
parameter BUSRT_BITS             = 10;             //external memory user interface burst width

wire                            wr_burst_req;
wire[BUSRT_BITS - 1:0]          wr_burst_len;
wire[ADDR_BITS - 1:0]           wr_burst_addr;
wire                            wr_burst_data_req;
wire[MEM_DATA_BITS - 1 : 0]     wr_burst_data;
wire                            wr_burst_finish;

wire                            rd_burst_req;
wire[BUSRT_BITS - 1:0]          rd_burst_len;
wire[ADDR_BITS - 1:0]           rd_burst_addr;
wire                            rd_burst_data_valid;
wire[MEM_DATA_BITS - 1 : 0]     rd_burst_data;
wire                            rd_burst_finish;

// New optional service boundary; existing four-channel arbiter stays unchanged.
wire legacy_rd_req,legacy_rd_valid,legacy_rd_done,ddr_rd_ready;
wire [BUSRT_BITS-1:0] legacy_rd_len;
wire [ADDR_BITS-1:0] legacy_rd_addr;
wire [255:0] legacy_rd_data;
wire patch_req_valid,patch_req_ready,patch_frame_locked,patch_pix_ready;
wire [24:0] patch_base_word;
wire [31:0] patch_request_tag,patch_current_tag;
wire [11:0] patch_x,patch_y,patch_width,patch_height;
wire patch_pix_valid,patch_pix_last,patch_done,patch_error,patch_busy,patch_ddr_fault;
wire [15:0] patch_rgb565;
wire [11:0] patch_pixel_x,patch_pixel_y;
wire [31:0] patch_pixel_tag;
wire [2:0] patch_error_code;
// Reserved internal request bus: the future snapshot/matching client replaces
// these tie-offs. No physical I/O ports, camera settings or UI behaviour change.
generate if(ENABLE_MEASUREMENT==0) begin:measurement_patch_tieoff
assign patch_req_valid=1'b0;
assign patch_frame_locked=1'b0;
assign patch_base_word=25'd0;
assign patch_request_tag=32'd0;
assign patch_current_tag=32'd0;
assign patch_x=12'd0;assign patch_y=12'd0;
assign patch_width=12'd1;assign patch_height=12'd1;
assign patch_pix_ready=1'b1;
end endgenerate


wire                            wr_burst_req_0;
wire[BUSRT_BITS - 1:0]          wr_burst_len_0;
wire[ADDR_BITS - 1:0]           wr_burst_addr_0;
wire                            wr_burst_data_req_0;
wire[MEM_DATA_BITS - 1 : 0]     wr_burst_data_0;
wire                            wr_burst_finish_0;

wire                            rd_burst_req_0;
wire[BUSRT_BITS - 1:0]          rd_burst_len_0;
wire[ADDR_BITS - 1:0]           rd_burst_addr_0;
wire                            rd_burst_data_valid_0;
wire[MEM_DATA_BITS - 1 : 0]     rd_burst_data_0;
wire                            rd_burst_finish_0;

wire                            wr_burst_req_1;
wire[BUSRT_BITS - 1:0]          wr_burst_len_1;
wire[ADDR_BITS - 1:0]           wr_burst_addr_1;
wire                            wr_burst_data_req_1;
wire[MEM_DATA_BITS - 1 : 0]     wr_burst_data_1;
wire                            wr_burst_finish_1;

wire                            rd_burst_req_1;
wire[BUSRT_BITS - 1:0]          rd_burst_len_1;
wire[ADDR_BITS - 1:0]           rd_burst_addr_1;
wire                            rd_burst_data_valid_1;
wire[MEM_DATA_BITS - 1 : 0]     rd_burst_data_1;
wire                            rd_burst_finish_1;
wire[1:0]                      display_read_index_latched_0;
wire[1:0]                      display_read_index_latched_1;
wire                           display_read_tag_latched_0;
wire                           display_read_tag_latched_1;

// Post-algorithm frame builders use the two previously unused DDR channels.
wire                            wr_burst_req_2,wr_burst_req_3;
wire[BUSRT_BITS - 1:0]          wr_burst_len_2,wr_burst_len_3;
wire[ADDR_BITS - 1:0]           wr_burst_addr_2,wr_burst_addr_3;
wire                            wr_burst_data_req_2,wr_burst_data_req_3;
wire[MEM_DATA_BITS - 1:0]       wr_burst_data_2,wr_burst_data_3;
wire                            wr_burst_finish_2,wr_burst_finish_3;
wire                            rd_burst_req_2,rd_burst_req_3;
wire[BUSRT_BITS - 1:0]          rd_burst_len_2,rd_burst_len_3;
wire[ADDR_BITS - 1:0]           rd_burst_addr_2,rd_burst_addr_3;
wire                            rd_burst_data_valid_2,rd_burst_data_valid_3;
wire[MEM_DATA_BITS - 1:0]       rd_burst_data_2,rd_burst_data_3;
wire                            rd_burst_finish_2,rd_burst_finish_3;


// Safe display fallback.  Before a renderer publishes its first complete
// frame, keep reading the original camera ring buffer.  This prevents a fault
// in either post-processing channel from turning the entire HDMI image black.
wire view_render_enabled=(ENABLE_DDR_VIEW_RENDER!=0);
wire [1:0] algorithm_left_publication_pixel,algorithm_right_publication_pixel;
// CMP pair gate publication (pixel domain); unused when ENABLE_CMP_CONTROLS=0.
wire [1:0] cmp_left_pub_pixel,cmp_right_pub_pixel;
wire left_display_rendered=view_render_enabled && ((ENABLE_ALGORITHM!=0)?algorithm_left_publication_pixel[1]:left_render_ready_pclk[1]);
wire right_display_rendered=view_render_enabled && ((ENABLE_ALGORITHM!=0)?algorithm_right_publication_pixel[1]:right_render_ready_pclk[1]);
wire left_fallback_cam1=(ui_pixel_layout==2'd2);
wire right_fallback_cam0=(ui_pixel_layout==2'd1);
wire [24:0] left_display_base0=left_display_rendered?LEFT_VIEW_BASE0:
    (left_fallback_cam1?25'd8294400:25'd0);
wire [24:0] left_display_base1=left_display_rendered?LEFT_VIEW_BASE1:
    (left_fallback_cam1?25'd10368000:25'd2073600);
wire [24:0] left_display_base2=left_display_rendered?LEFT_VIEW_BASE0:
    (left_fallback_cam1?25'd12441600:25'd4147200);
wire [24:0] left_display_base3=left_display_rendered?LEFT_VIEW_BASE1:
    (left_fallback_cam1?25'd14515200:25'd6220800);
wire [1:0] left_display_index=left_display_rendered?{1'b0,((ENABLE_ALGORITHM!=0)?algorithm_left_publication_pixel[0]:left_render_front_pclk[1])}:
    (left_fallback_cam1?read_addr_index_1:read_addr_index_0);
wire [24:0] right_display_base0=right_display_rendered?RIGHT_VIEW_BASE0:
    (right_fallback_cam0?25'd0:25'd8294400);
wire [24:0] right_display_base1=right_display_rendered?RIGHT_VIEW_BASE1:
    (right_fallback_cam0?25'd2073600:25'd10368000);
wire [24:0] right_display_base2=right_display_rendered?RIGHT_VIEW_BASE0:
    (right_fallback_cam0?25'd4147200:25'd12441600);
wire [24:0] right_display_base3=right_display_rendered?RIGHT_VIEW_BASE1:
    (right_fallback_cam0?25'd6220800:25'd14515200);
wire [1:0] right_display_index=right_display_rendered?{1'b0,((ENABLE_ALGORITHM!=0)?algorithm_right_publication_pixel[0]:right_render_front_pclk[1])}:
    (right_fallback_cam0?read_addr_index_0:read_addr_index_1);

wire [1:0] photo_initial_index0=left_display_rendered?left_display_index:(left_fallback_cam1?photo_bank1:photo_bank0);
wire [1:0] photo_initial_index1=right_display_rendered?right_display_index:(right_fallback_cam0?photo_bank0:photo_bank1);
wire [24:0] photo_initial_base0=(photo_initial_index0==0)?left_display_base0:(photo_initial_index0==1)?left_display_base1:(photo_initial_index0==2)?left_display_base2:left_display_base3;
wire [24:0] photo_initial_base1=(photo_initial_index1==0)?right_display_base0:(photo_initial_index1==1)?right_display_base1:(photo_initial_index1==2)?right_display_base2:right_display_base3;
// Initial qualification reads the HELD RAW bank only. The renderer keeps
// republishing its front/back buffer while frozen; qualifying on that index
// restarted PROBE at every publication and never reached SHOW. After SHOW the
// legacy descriptor flow switches to the rendered (undistorted/edited) frames.
wire [1:0] photo_raw_index0=left_fallback_cam1?photo_bank1:photo_bank0;
wire [1:0] photo_raw_index1=right_fallback_cam0?photo_bank0:photo_bank1;
wire [24:0] photo_raw_base0=(left_fallback_cam1?25'd8294400:25'd0)+photo_raw_index0*25'd2073600;
wire [24:0] photo_raw_base1=(right_fallback_cam0?25'd0:25'd8294400)+photo_raw_index1*25'd2073600;
assign photo_dynamic0={1'b0,photo_raw_index0,photo_raw_base0};
assign photo_dynamic1={1'b0,photo_raw_index1,photo_raw_base1};
// New enabled path transfers one complete descriptor with the frame request.
// The original memory reader and its FIFO remain unchanged.
wire left_display_mem_req,left_display_mem_ack,left_display_mem_tag;
wire right_display_mem_req,right_display_mem_ack,right_display_mem_tag;
wire [1:0] left_display_mem_index,right_display_mem_index;
wire [24:0] left_display_mem_base0,left_display_mem_base1,left_display_mem_base2,left_display_mem_base3;
wire [24:0] right_display_mem_base0,right_display_mem_base1,right_display_mem_base2,right_display_mem_base3;
generate if(ENABLE_ALGORITHM!=0)begin:display_descriptors
    reg [1:0] previous_left,previous_right;
    wire [1:0] publication_left={left_render_ready,left_render_front};
    wire [1:0] publication_right={right_render_ready,right_render_front};
    always @(posedge core_clk or negedge ddr_init_done)begin
        if(!ddr_init_done)begin previous_left<=0;previous_right<=0;end
        else begin previous_left<=publication_left;previous_right<=publication_right;end
    end
    wire [1:0] independent_left_pixel,independent_right_pixel;
    stereo_event_cdc #(.WIDTH(2),.INITIAL_PUBLISH(1)) left_publication(.src_clk(core_clk),.dst_clk(pclk),.rst_n(ddr_init_done),
        .src_event(publication_left!=previous_left),.src_data(publication_left),
        .dst_event(),.dst_data(independent_left_pixel));
    stereo_event_cdc #(.WIDTH(2),.INITIAL_PUBLISH(1)) right_publication(.src_clk(core_clk),.dst_clk(pclk),.rst_n(ddr_init_done),
        .src_event(publication_right!=previous_right),.src_data(publication_right),
        .dst_event(),.dst_data(independent_right_pixel));
    // CMP1: both halves cross in ONE mailbox inside cmp_pair_gate instead.
    assign algorithm_left_publication_pixel=(ENABLE_CMP_CONTROLS!=0)?cmp_left_pub_pixel:independent_left_pixel;
    assign algorithm_right_publication_pixel=(ENABLE_CMP_CONTROLS!=0)?cmp_right_pub_pixel:independent_right_pixel;
    wire [24:0] left_selected_base=(left_display_index==0)?left_display_base0:
        (left_display_index==1)?left_display_base1:(left_display_index==2)?left_display_base2:left_display_base3;
    wire [24:0] right_selected_base=(right_display_index==0)?right_display_base0:
        (right_display_index==1)?right_display_base1:(right_display_index==2)?right_display_base2:right_display_base3;
    // Frozen STEREO shows the two held raw camera banks directly (C0 left,
    // C1 right): capture -> DDR -> display, no renderer in the path. CMP
    // layouts keep the rendered (undistort/binary/edit) frames.
    wire photo_rendered_view=photo_legacy && (ui_pixel_layout!=2'd0);
    testing60_photo_descriptor_bridge left_descriptor(.pixel_clk(pclk),.mem_clk(core_clk),.rst_n(ddr_init_done),
        .pixel_req(photo_reader_req0),.pixel_ack(read_req_ack_0),.pixel_base(photo_rendered_view?left_selected_base:photo_selected0[24:0]),.pixel_index(photo_rendered_view?left_display_index:photo_selected0[26:25]),.pixel_tag(photo_rendered_view?left_display_rendered:photo_selected0[27]),
        .native_index(display_read_index_latched_0),.native_tag(display_read_tag_latched_0),.accepted_descriptor(photo_accepted0),.accepted_matches_request(photo_accepted_ok0),
        .mem_req(left_display_mem_req),.mem_ack(left_display_mem_ack),.mem_base(left_display_mem_base0),.mem_index(left_display_mem_index),.mem_tag(left_display_mem_tag));
    testing60_photo_descriptor_bridge right_descriptor(.pixel_clk(pclk),.mem_clk(core_clk),.rst_n(ddr_init_done),
        .pixel_req(photo_reader_req1),.pixel_ack(read_req_ack_1),.pixel_base(photo_rendered_view?right_selected_base:photo_selected1[24:0]),.pixel_index(photo_rendered_view?right_display_index:photo_selected1[26:25]),.pixel_tag(photo_rendered_view?right_display_rendered:photo_selected1[27]),
        .native_index(display_read_index_latched_1),.native_tag(display_read_tag_latched_1),.accepted_descriptor(photo_accepted1),.accepted_matches_request(photo_accepted_ok1),
        .mem_req(right_display_mem_req),.mem_ack(right_display_mem_ack),.mem_base(right_display_mem_base0),.mem_index(right_display_mem_index),.mem_tag(right_display_mem_tag));
    assign left_display_mem_base1=left_display_mem_base0;
    assign left_display_mem_base2=left_display_mem_base0;
    assign left_display_mem_base3=left_display_mem_base0;
    assign right_display_mem_base1=right_display_mem_base0;
    assign right_display_mem_base2=right_display_mem_base0;
    assign right_display_mem_base3=right_display_mem_base0;
end else begin:original_display_descriptors
    assign algorithm_left_publication_pixel=2'b00;assign algorithm_right_publication_pixel=2'b00;
    assign photo_accepted0=0;assign photo_accepted1=0;assign photo_accepted_ok0=0;assign photo_accepted_ok1=0;
    assign left_display_mem_req=read_req_0;assign right_display_mem_req=read_req_1;
    assign read_req_ack_0=left_display_mem_ack;assign read_req_ack_1=right_display_mem_ack;
    assign left_display_mem_tag=left_display_rendered;assign right_display_mem_tag=right_display_rendered;
    assign left_display_mem_index=left_display_index;assign right_display_mem_index=right_display_index;
    assign left_display_mem_base0=left_display_base0;assign left_display_mem_base1=left_display_base1;
    assign left_display_mem_base2=left_display_base2;assign left_display_mem_base3=left_display_base3;
    assign right_display_mem_base0=right_display_base0;assign right_display_mem_base1=right_display_base1;
    assign right_display_mem_base2=right_display_base2;assign right_display_mem_base3=right_display_base3;
end endgenerate

testing60_frame_read_write #(
	.MEM_DATA_BITS              (256                      ),
	.READ_DATA_BITS             (16                       ),
	.WRITE_DATA_BITS            (16                       ),
	.ADDR_BITS                  (25                       ),
	.BUSRT_BITS                 (10                       ),
	.BURST_SIZE                 (16                       ) //?
) frame_read_write_inst0(
	.rst                        (~ddr_init_done        ),
	.mem_clk                    (core_clk              ),
	.rd_burst_req               (rd_burst_req_0        ),
	.rd_burst_len               (rd_burst_len_0        ),
	.rd_burst_addr              (rd_burst_addr_0       ),
	.rd_burst_data_valid        (rd_burst_data_valid_0 ),
	.rd_burst_data              (rd_burst_data_0       ),
	.rd_burst_finish            (rd_burst_finish_0     ),
	.read_clk                   (pclk                  ),
	.read_req                   (left_display_mem_req  ),
	.read_req_ack               (left_display_mem_ack  ),
	.read_finish                (                      ),
	.read_buffer_tag            (left_display_mem_tag  ),
	.read_index_latched         (display_read_index_latched_0),
	.read_tag_latched           (display_read_tag_latched_0),
	.read_addr_0                (left_display_mem_base0),
	.read_addr_1                (left_display_mem_base1),
	.read_addr_2                (left_display_mem_base2),
	.read_addr_3                (left_display_mem_base3),
	.read_addr_index            (left_display_mem_index),
	.read_len(FRAME_WORDS),//frame size 
	.window_enable              (1'b0                  ),
	.window_scale_shift         (left_view_scale       ),
	.window_source_left         (left_source_x         ),
	.window_source_top          (left_source_y         ),
	.window_source_width        (left_source_w         ),
	.window_source_height       (left_source_h         ),
	.read_en                    (read_en_0             ),
	.read_data                  (read_data_0           ),
	.read_empty                 (read_empty_0          ),
	.read_level                 (read_level_0          ),

	.wr_burst_req               (wr_burst_req_0        ),
	.wr_burst_len               (wr_burst_len_0        ),
	.wr_burst_addr              (wr_burst_addr_0       ),
	.wr_burst_data_req          (wr_burst_data_req_0   ),
	.wr_burst_data              (wr_burst_data_0       ),
	.wr_burst_finish            (wr_burst_finish_0     ),
	.write_clk                  (camera_clk_0),
	.write_req                  (write_req_0           ),
	.write_req_ack              (write_req_ack_0       ),
	.write_finish               (capture_ddr_finish_0),
	.write_addr_0               (25'd0                 ),
	.write_addr_1               (25'd2073600           ),
	.write_addr_2               (25'd4147200           ),
	.write_addr_3               (25'd6220800           ),
	.write_addr_index           (write_addr_index_0    ),
	.write_len(FRAME_WORDS), //frame size  
	.write_en   (write_en_0),
	.write_data (write_data_0)
);
testing60_frame_read_write #(
	.MEM_DATA_BITS              (256                   ),
	.READ_DATA_BITS             (16                    ),
	.WRITE_DATA_BITS            (16                    ),
	.ADDR_BITS                  (25                    ),
	.BUSRT_BITS                 (10                    ),
	.BURST_SIZE                 (16                    ) //?
) frame_read_write_inst1(
	.rst                        (~ddr_init_done        ),
	.mem_clk                    (core_clk              ),
	.rd_burst_req               (rd_burst_req_1        ),
	.rd_burst_len               (rd_burst_len_1        ),
	.rd_burst_addr              (rd_burst_addr_1       ),
	.rd_burst_data_valid        (rd_burst_data_valid_1 ),
	.rd_burst_data              (rd_burst_data_1       ),
	.rd_burst_finish            (rd_burst_finish_1     ),
	.read_clk                   (pclk                  ),
	.read_req                   (right_display_mem_req ),
	.read_req_ack               (right_display_mem_ack ),
	.read_finish                (                      ),
	.read_buffer_tag            (right_display_mem_tag ),
	.read_index_latched         (display_read_index_latched_1),
	.read_tag_latched           (display_read_tag_latched_1),
	.read_addr_0                (right_display_mem_base0),
	.read_addr_1                (right_display_mem_base1),
	.read_addr_2                (right_display_mem_base2),
	.read_addr_3                (right_display_mem_base3),
	.read_addr_index            (right_display_mem_index),
	.read_len(FRAME_WORDS),//frame size 
	.window_enable              (1'b0                  ),
	.window_scale_shift         (right_view_scale      ),
	.window_source_left         (right_source_x        ),
	.window_source_top          (right_source_y        ),
	.window_source_width        (right_source_w        ),
	.window_source_height       (right_source_h        ),
	.read_en                    (read_en_1             ),
	.read_data                  (read_data_1           ),
	.read_empty                 (read_empty_1          ),
	.read_level                 (read_level_1          ),

	.wr_burst_req               (wr_burst_req_1        ),
	.wr_burst_len               (wr_burst_len_1        ),
	.wr_burst_addr              (wr_burst_addr_1       ),
	.wr_burst_data_req          (wr_burst_data_req_1   ),
	.wr_burst_data              (wr_burst_data_1    ),
	.wr_burst_finish            (wr_burst_finish_1     ),
	.write_clk(camera_clk_1),
	.write_req                  (write_req_1           ),
	.write_req_ack              (write_req_ack_1       ),
	.write_finish               (capture_ddr_finish_1),
	.write_addr_0               (25'd8294400           ),
	.write_addr_1               (25'd10368000          ),
	.write_addr_2               (25'd12441600          ),
	.write_addr_3               (25'd14515200          ),
	.write_addr_index           (write_addr_index_1    ),
	.write_len(FRAME_WORDS), //frame size  
	.write_en                   (write_en_1            ),
	.write_data                 (write_data_1)
);    

//===========================================================================
// Post-algorithm spatial processing into dedicated DDR display buffers
//===========================================================================
reg [1:0] cam0_read_index_meta,cam0_read_index_core;
reg [1:0] cam1_read_index_meta,cam1_read_index_core;
wire [1:0] render_layout_core=ui_render_state[151:150];
reg [1:0] render_source_ready_core;

wire render_control_reset_n=rst_n && ddr_init_done;
reg render_initial_publish;
always @(posedge sys_clk or negedge render_control_reset_n) begin
    if(!render_control_reset_n) render_initial_publish<=1'b1;
    else render_initial_publish<=1'b0;
end
// CMP switches travel in the SAME mailbox as layout/geometry/knobs, so the
// core never sees a new layout with stale switches (or vice versa).
localparam integer RENDER_STATE_BITS=(ENABLE_CMP_CONTROLS!=0)?286:282;
wire [3:0] render_cmp_core; // {undistort[1:0],binary[1:0]}
wire [RENDER_STATE_BITS-1:0] render_state_source,render_state_core;
generate if(ENABLE_CMP_CONTROLS!=0)begin:cmp_render_state
    assign render_state_source={ui_cmp_undistort,ui_cmp_binary,
        ui_feature,ui_knob_values,ui_layout,ui_left_rotation,ui_right_rotation,
        ui_left_step,ui_left_origin_x,ui_left_origin_y,
        ui_right_step,ui_right_origin_x,ui_right_origin_y};
    assign render_cmp_core=render_state_core[285:282];
end else begin:original_render_state
    assign render_state_source={ui_feature,ui_knob_values,ui_layout,ui_left_rotation,ui_right_rotation,
        ui_left_step,ui_left_origin_x,ui_left_origin_y,
        ui_right_step,ui_right_origin_x,ui_right_origin_y};
    assign render_cmp_core=4'd0;
end endgenerate
assign ui_render_state=render_state_core[281:0];
stereo_event_cdc #(.WIDTH(1),.INITIAL_PUBLISH(1)) photo_configuration_cdc(
 .src_clk(sys_clk),.dst_clk(pclk),.rst_n(rst_n),.src_event(ui_state_event),
 .src_data(1'b0),.src_ready(),.dst_event(photo_configuration_event),.dst_data());
stereo_event_cdc #(.WIDTH(RENDER_STATE_BITS),.SYNC_RESET(ENABLE_ALGORITHM),.INITIAL_PUBLISH(ENABLE_ALGORITHM)) u_render_state_cdc (
    .src_clk(sys_clk),.dst_clk(core_clk),.rst_n(render_control_reset_n),
    .src_event(ui_state_event || render_initial_publish),
    .src_data(render_state_source),
    .dst_event(),.dst_data(render_state_core)
);

wire [128:0] algorithm_imu_state;
stereo_event_cdc #(.WIDTH(129),.SYNC_RESET(ENABLE_ALGORITHM),.INITIAL_PUBLISH(ENABLE_ALGORITHM)) u_algorithm_imu_cdc (
    .src_clk(sys_clk),.dst_clk(core_clk),.rst_n(render_control_reset_n),
    .src_event(imu_angle_valid),
    .src_data({imu_attitude_ready,imu_q3,imu_q2,imu_q1,imu_q0}),
    .dst_event(),.dst_data(algorithm_imu_state)
);
(* mark_debug="true" *) wire left_algorithm_fault,right_algorithm_fault;
(* mark_debug="true" *) wire [31:0] left_algorithm_cycles,right_algorithm_cycles;
generate if(ENABLE_ALGORITHM!=0)begin:algorithm_snapshots
    wire hold0,hold1,valid0,valid1;
    wire[1:0]index0,index1;
    wire[31:0]tag0,tag1;
    testing60_camera_snapshot camera0(.camera_clk(camera_clk_0),.core_clk(core_clk),.rst_n(render_control_reset_n),
        .request_hold(hold0),.camera_vsync(capture_vsync_0),.completed_index(read_addr_index_0),.completed_valid(capture_valid_camera_0),
        .freeze_ring(algorithm_freeze_cam0),.snapshot_valid(valid0),.snapshot_index(index0),.snapshot_tag(tag0));
    testing60_camera_snapshot camera1(.camera_clk(camera_clk_1),.core_clk(core_clk),.rst_n(render_control_reset_n),
        .request_hold(hold1),.camera_vsync(capture_vsync_1),.completed_index(read_addr_index_1),.completed_valid(capture_valid_camera_1),
        .freeze_ring(algorithm_freeze_cam1),.snapshot_valid(valid1),.snapshot_index(index1),.snapshot_tag(tag1));
    stereo_frame_leases leases(.clk(core_clk),.rst_n(render_control_reset_n),
        .left_request(left_snapshot_request),.right_request(right_snapshot_request),
        .left_camera(measurement_force_static ? 1'b0 : (render_layout_core==2'd2)),.right_camera(measurement_force_static ? 1'b1 : (render_layout_core!=2'd1)),
        .cam0_valid(valid0 && capture_valid_core_0[index0]),.cam1_valid(valid1 && capture_valid_core_1[index1]),.cam0_index(index0),.cam1_index(index1),.cam0_tag(tag0),.cam1_tag(tag1),
        .cam0_hold(hold0),.cam1_hold(hold1),.left_grant(left_snapshot_grant),.right_grant(right_snapshot_grant),
        .left_granted_camera(left_snapshot_camera),.right_granted_camera(right_snapshot_camera),
        .left_base(left_snapshot_base),.right_base(right_snapshot_base),.left_tag(left_snapshot_tag),.right_tag(right_snapshot_tag));
end else begin:no_algorithm_snapshots
    assign algorithm_freeze_cam0=1'b0;assign algorithm_freeze_cam1=1'b0;
    assign left_snapshot_grant=1'b0;assign right_snapshot_grant=1'b0;
    assign left_snapshot_camera=1'b0;assign right_snapshot_camera=1'b0;
    assign left_snapshot_base=25'd0;assign right_snapshot_base=25'd0;
    assign left_snapshot_tag=32'd0;assign right_snapshot_tag=32'd0;
end endgenerate

always @(posedge core_clk or negedge ddr_init_done) begin
    if(!ddr_init_done) begin
        cam0_read_index_meta<=0;cam0_read_index_core<=0;
        cam1_read_index_meta<=0;cam1_read_index_core<=0;
        render_source_ready_core<=0;
    end else begin
        cam0_read_index_meta<=read_addr_index_0;
        cam0_read_index_core<=cam0_read_index_meta;
        cam1_read_index_meta<=read_addr_index_1;
        cam1_read_index_core<=cam1_read_index_meta;
        render_source_ready_core<={render_source_ready_core[0],
            (cam_init_done_0 && cam_init_done_1)};
    end
end

wire [24:0] cam0_render_base=(cam0_read_index_core==2'd0)?25'd0:
    (cam0_read_index_core==2'd1)?25'd2073600:
    (cam0_read_index_core==2'd2)?25'd4147200:25'd6220800;
wire [24:0] cam1_render_base=(cam1_read_index_core==2'd0)?25'd8294400:
    (cam1_read_index_core==2'd1)?25'd10368000:
    (cam1_read_index_core==2'd2)?25'd12441600:25'd14515200;
wire [24:0] left_render_source_base=(render_layout_core==2'd2)?
    cam1_render_base:cam0_render_base;
wire [24:0] right_render_source_base=(render_layout_core==2'd1)?
    cam0_render_base:cam1_render_base;

wire [72:0] full_frame_geometry={17'd65536,28'd0,28'd0};
wire [72:0] left_render_geometry=(ENABLE_VIEW_GESTURES!=0)?
    ui_render_state[145:73]:full_frame_geometry;
wire [72:0] right_render_geometry=(ENABLE_VIEW_GESTURES!=0)?
    ui_render_state[72:0]:full_frame_geometry;
wire [1:0] left_render_rotation=(ENABLE_VIEW_GESTURES!=0)?ui_render_state[149:148]:2'd0;
wire [1:0] right_render_rotation=(ENABLE_VIEW_GESTURES!=0)?ui_render_state[147:146]:2'd0;
wire [127:0] render_knobs=ui_render_state[279:152];
wire [47:0] left_adjust_values={render_knobs[119:112],render_knobs[103:96],
    render_knobs[87:80],render_knobs[71:64],render_knobs[55:48],render_knobs[39:32]};
wire [47:0] right_adjust_values={render_knobs[127:120],render_knobs[111:104],
    render_knobs[95:88],render_knobs[79:72],render_knobs[63:56],render_knobs[47:40]};

// Per-view configuration.  ENABLE_CMP_CONTROLS=0 keeps the original
// expressions.  In CMP (=1) both views show the selected page's camera:
//   binary OFF -> fullscreen: 640 source columns over 1280 display columns,
//                 left view = source x 0..319.5, right view = 320..639.5;
//   binary ON  -> left and right views identical (right binarized in pixel
//                 domain, after all adjustments);
//   undistort selects correction or raw for BOTH views.
// The pair gate freezes this configuration per paired job.
wire render_source_valid_base=view_render_enabled && ((ENABLE_DDR_TEST_BARS!=0) || render_source_ready_core[1]);
wire view_source_valid;
wire left_view_idle,right_view_idle;
wire left_cfg_bypass,right_cfg_bypass,left_cfg_doodle,right_cfg_doodle;
wire [16:0] left_cfg_step,left_cfg_step_x,right_cfg_step,right_cfg_step_x;
wire signed [27:0] left_cfg_origin_x,left_cfg_origin_y,right_cfg_origin_x,right_cfg_origin_y;
wire [1:0] left_cfg_rotation,right_cfg_rotation,left_cfg_feature,right_cfg_feature;
wire [47:0] left_cfg_adjust,right_cfg_adjust;
generate if(ENABLE_CMP_CONTROLS!=0 && ENABLE_ALGORITHM!=0)begin:cmp_pairing
    wire cmp_mode=!measurement_force_static && (render_layout_core==2'd1 || render_layout_core==2'd2);
    wire cmp_camera=(render_layout_core==2'd2);
    wire cmp_undistort=cmp_camera?render_cmp_core[3]:render_cmp_core[2];
    wire cmp_binary=cmp_camera?render_cmp_core[1]:render_cmp_core[0];
    wire [72:0] cmp_geometry=cmp_camera?right_render_geometry:left_render_geometry;
    wire [16:0] cmp_step=cmp_geometry[72:56];
    wire signed [27:0] cmp_origin_x=cmp_geometry[55:28];
    wire signed [27:0] cmp_origin_y=cmp_geometry[27:0];
    wire [16:0] cmp_step_x=cmp_binary?cmp_step:{1'b0,cmp_step[16:1]};
    wire signed [27:0] cmp_right_origin_x=cmp_binary?cmp_origin_x:
        (cmp_origin_x+$signed({11'd0,cmp_step})*28'sd320);
    wire [47:0] cmp_adjust=cmp_camera?right_adjust_values:left_adjust_values;
    // {bypass,step,step_x,origin_x,origin_y,rotation,adjust,feature,doodle}
    wire [143:0] left_live=cmp_mode?
        {!cmp_undistort,cmp_step,cmp_step_x,cmp_origin_x,cmp_origin_y,2'd0,cmp_adjust,2'd0,1'b0}:
        {(render_layout_core!=2'd0),left_render_geometry[72:56],left_render_geometry[72:56],
         left_render_geometry[55:28],left_render_geometry[27:0],left_render_rotation,
         left_adjust_values,ui_render_state[281:280],1'b1};
    wire [143:0] right_live=cmp_mode?
        {!cmp_undistort,cmp_step,cmp_step_x,cmp_right_origin_x,cmp_origin_y,2'd0,cmp_adjust,2'd0,1'b0}:
        {1'b0,right_render_geometry[72:56],right_render_geometry[72:56],
         right_render_geometry[55:28],right_render_geometry[27:0],right_render_rotation,
         right_adjust_values,ui_render_state[281:280],1'b1};
    wire [287:0] cfg_frozen;
    cmp_pair_gate #(.CFG_BITS(288)) u_cmp_pair_gate(
        .mem_clk(core_clk),.mem_rst(~ddr_init_done),.cmp_mode(cmp_mode),.cmp_key(cmp_camera),
        .source_valid_in(render_source_valid_base),.source_valid_out(view_source_valid),
        .left_idle(left_view_idle),.right_idle(right_view_idle),
        .left_pub({left_render_ready,left_render_front}),.right_pub({right_render_ready,right_render_front}),
        .cfg_live({left_live,right_live}),.binary_live(cmp_mode && cmp_binary),.cfg_out(cfg_frozen),.pair_running(),
        .pix_clk(pclk),.pix_rst_n(ddr_init_done),.left_capture(read_req_0),.right_capture(read_req_1),
        .left_pub_pixel(cmp_left_pub_pixel),.right_pub_pixel(cmp_right_pub_pixel),.binary_pixel(cmp_binary_pixel));
    assign {left_cfg_bypass,left_cfg_step,left_cfg_step_x,left_cfg_origin_x,left_cfg_origin_y,
        left_cfg_rotation,left_cfg_adjust,left_cfg_feature,left_cfg_doodle}=cfg_frozen[287:144];
    assign {right_cfg_bypass,right_cfg_step,right_cfg_step_x,right_cfg_origin_x,right_cfg_origin_y,
        right_cfg_rotation,right_cfg_adjust,right_cfg_feature,right_cfg_doodle}=cfg_frozen[143:0];
end else begin:original_view_config
    assign view_source_valid=render_source_valid_base;
    assign left_cfg_bypass=(render_layout_core!=2'd0);assign right_cfg_bypass=1'b0;
    assign left_cfg_step=left_render_geometry[72:56];assign left_cfg_step_x=left_render_geometry[72:56];
    assign right_cfg_step=right_render_geometry[72:56];assign right_cfg_step_x=right_render_geometry[72:56];
    assign left_cfg_origin_x=left_render_geometry[55:28];assign left_cfg_origin_y=left_render_geometry[27:0];
    assign right_cfg_origin_x=right_render_geometry[55:28];assign right_cfg_origin_y=right_render_geometry[27:0];
    assign left_cfg_rotation=left_render_rotation;assign right_cfg_rotation=right_render_rotation;
    assign left_cfg_adjust=left_adjust_values;assign right_cfg_adjust=right_adjust_values;
    assign left_cfg_feature=ui_render_state[281:280];assign right_cfg_feature=ui_render_state[281:280];
    assign left_cfg_doodle=1'b1;assign right_cfg_doodle=1'b1;
    assign cmp_left_pub_pixel=2'b00;assign cmp_right_pub_pixel=2'b00;assign cmp_binary_pixel=1'b0;
end endgenerate

stereo_algorithm_view #(.ADDR_BITS(ADDR_BITS),.BUSRT_BITS(BUSRT_BITS),.ENABLED(ENABLE_ALGORITHM),
	.OUTPUT_BASE0(LEFT_VIEW_BASE0),.OUTPUT_BASE1(LEFT_VIEW_BASE1),
	.TEST_PATTERN(ENABLE_DDR_TEST_BARS),.USE_AFFINE(1),.MIN_STEP_X((ENABLE_CMP_CONTROLS!=0)?8192:16384))
u_left_frame_view_render (
    .mem_clk(core_clk),.rst(~ddr_init_done),
    .source_camera(measurement_force_static ? 1'b0 : (render_layout_core==2'd2)),.bypass_correction(!RAW8_CALIBRATION_VALID || (measurement_force_static ? 1'b0 : left_cfg_bypass)),
    .feature(measurement_force_static ? 2'd0 : left_cfg_feature),.imu_ready(algorithm_imu_state[128]),.imu_q(algorithm_imu_state[127:0]),
    .algorithm_fault(left_algorithm_fault),.algorithm_cycles(left_algorithm_cycles),.job_idle(left_view_idle),
    .snapshot_request(left_snapshot_request),.snapshot_grant(left_snapshot_grant),.snapshot_camera(left_snapshot_camera),
    .snapshot_base(left_snapshot_base),.snapshot_tag(left_snapshot_tag),
	.source_valid(view_source_valid && measurement_allow_left),
	.source_base(left_render_source_base),
    .scale_shift(2'd0),.source_left(12'd0),.source_top(12'd0),
    .source_width(12'd640),.source_height(12'd720),.destination_left(12'd0),.destination_top(12'd0),
    .view_step_q16(measurement_force_static ? 17'd65536 : left_cfg_step),.view_step_x_q16(measurement_force_static ? 17'd65536 : left_cfg_step_x),
    .view_rotation(measurement_force_static ? 2'd0 : left_cfg_rotation),
    .view_origin_x(measurement_force_static ? 28'sd0 : left_cfg_origin_x),.view_origin_y(measurement_force_static ? 28'sd0 : left_cfg_origin_y),
	.apply_binary(1'b0),
	.adjust_values(measurement_force_static ? 48'h808080808080 : left_cfg_adjust),.doodle_index(measurement_force_static ? 4'd0 : (left_cfg_doodle?left_doodle_index:4'd0)),
	.doodle_address(left_doodle_address),
	.display_index_async(display_read_index_latched_0[0]),
	.display_accept_valid_async(display_read_tag_latched_0),
    .front_index(left_render_front),.frame_ready(left_render_ready),
    .rd_burst_req(rd_burst_req_2),.rd_burst_len(rd_burst_len_2),
    .rd_burst_addr(rd_burst_addr_2),.rd_burst_data_valid(rd_burst_data_valid_2),
    .rd_burst_data(rd_burst_data_2),.rd_burst_finish(rd_burst_finish_2),
    .wr_burst_req(wr_burst_req_2),.wr_burst_len(wr_burst_len_2),
    .wr_burst_addr(wr_burst_addr_2),.wr_burst_data_req(wr_burst_data_req_2),
    .wr_burst_data(wr_burst_data_2),.wr_burst_finish(wr_burst_finish_2)
);

stereo_algorithm_view #(.ADDR_BITS(ADDR_BITS),.BUSRT_BITS(BUSRT_BITS),.ENABLED(ENABLE_ALGORITHM),
	.OUTPUT_BASE0(RIGHT_VIEW_BASE0),.OUTPUT_BASE1(RIGHT_VIEW_BASE1),
	.TEST_PATTERN(ENABLE_DDR_TEST_BARS),.USE_AFFINE(1),.MIN_STEP_X((ENABLE_CMP_CONTROLS!=0)?8192:16384))
u_right_frame_view_render (
    .mem_clk(core_clk),.rst(~ddr_init_done),
    .source_camera(measurement_force_static ? 1'b1 : (render_layout_core!=2'd1)),.bypass_correction(!RAW8_CALIBRATION_VALID || (measurement_force_static ? 1'b0 : right_cfg_bypass)),
    .feature(measurement_force_static ? 2'd0 : right_cfg_feature),.imu_ready(algorithm_imu_state[128]),.imu_q(algorithm_imu_state[127:0]),
    .algorithm_fault(right_algorithm_fault),.algorithm_cycles(right_algorithm_cycles),.job_idle(right_view_idle),
    .snapshot_request(right_snapshot_request),.snapshot_grant(right_snapshot_grant),.snapshot_camera(right_snapshot_camera),
    .snapshot_base(right_snapshot_base),.snapshot_tag(right_snapshot_tag),
	.source_valid(view_source_valid && measurement_allow_right),
	.source_base(right_render_source_base),
    .scale_shift(2'd0),.source_left(12'd0),.source_top(12'd0),
    .source_width(12'd640),.source_height(12'd720),.destination_left(12'd0),.destination_top(12'd0),
    .view_step_q16(measurement_force_static ? 17'd65536 : right_cfg_step),.view_step_x_q16(measurement_force_static ? 17'd65536 : right_cfg_step_x),
    .view_rotation(measurement_force_static ? 2'd0 : right_cfg_rotation),
    .view_origin_x(measurement_force_static ? 28'sd0 : right_cfg_origin_x),.view_origin_y(measurement_force_static ? 28'sd0 : right_cfg_origin_y),
	.apply_binary(measurement_force_static ? 1'b0 : (render_layout_core!=2'd0)),
	.adjust_values(measurement_force_static ? 48'h808080808080 : right_cfg_adjust),.doodle_index(measurement_force_static ? 4'd0 : (right_cfg_doodle?right_doodle_index:4'd0)),
	.doodle_address(right_doodle_address),
	.display_index_async(display_read_index_latched_1[0]),
	.display_accept_valid_async(display_read_tag_latched_1),
    .front_index(right_render_front),.frame_ready(right_render_ready),
    .rd_burst_req(rd_burst_req_3),.rd_burst_len(rd_burst_len_3),
    .rd_burst_addr(rd_burst_addr_3),.rd_burst_data_valid(rd_burst_data_valid_3),
    .rd_burst_data(rd_burst_data_3),.rd_burst_finish(rd_burst_finish_3),
    .wr_burst_req(wr_burst_req_3),.wr_burst_len(wr_burst_len_3),
    .wr_burst_addr(wr_burst_addr_3),.wr_burst_data_req(wr_burst_data_req_3),
    .wr_burst_data(wr_burst_data_3),.wr_burst_finish(wr_burst_finish_3)
);


// Isolated measurement candidate. Invalid feature combinations deliberately
// fail elaboration; disabled configuration is the original direct path.
generate if(ENABLE_MEASUREMENT!=0 && ENABLE_ALGORITHM!=0 && ENABLE_AUX_READ!=0 &&
    ENABLE_DDR_VIEW_RENDER!=0 && ENABLE_LIVE_STREAM==0 && ENABLE_DDR_TEST_BARS==0) begin:measurement_enabled
 wire display_drop;
 measurement_display_drop display_guard(.clk(pclk),.rst_n(rst_n),
    .left_x(measurement_left_display_x),.left_y(measurement_left_display_y),
    .right_x(measurement_right_display_x),.right_y(measurement_right_display_y),
    .left_de(de_out_0),.right_de(de_out_1),.left_empty(read_empty_0),.right_empty(read_empty_1),.drop(display_drop));
 measurement_bridge #(.LEFT_BASE0(LEFT_VIEW_BASE0),.LEFT_BASE1(LEFT_VIEW_BASE1),
    .RIGHT_BASE0(RIGHT_VIEW_BASE0),.RIGHT_BASE1(RIGHT_VIEW_BASE1)) measurement (
    .sys_clk(sys_clk),.core_clk(core_clk),.pixel_clk(pclk),.rst_n(rst_n && ddr_init_done),
    .image_mode(ui_image_mode),.menu_visible(ui_menu_visible),.entry_allowed(RAW8_CALIBRATION_VALID && RAW8_STEREO_CALIBRATION_VALID && ui_layout==2'd0),
    .touch_event(touch_event),.point0_valid(point0_valid),.point1_valid(point1_valid),
    .point0_x(point0_x),.point0_y(point0_y),.point1_x(point1_x),.point1_y(point1_y),
    .block_touch(measurement_block_touch),.active_sys(measurement_active),
    .source_available(render_source_valid_base),.left_idle(left_view_idle),.right_idle(right_view_idle),
    .left_pub({left_render_ready,left_render_front}),.right_pub({right_render_ready,right_render_front}),
    .left_fault(left_algorithm_fault),.right_fault(right_algorithm_fault),
    .left_desc_tag(display_read_tag_latched_0),.left_desc_index(display_read_index_latched_0[0]),
    .right_desc_tag(display_read_tag_latched_1),.right_desc_index(display_read_index_latched_1[0]),
    .left_pub_pixel(algorithm_left_publication_pixel),.right_pub_pixel(algorithm_right_publication_pixel),
    .force_static(measurement_force_static),.allow_left(measurement_allow_left),.allow_right(measurement_allow_right),
    .raw_rgb(vout_data),.raw_de(de_out),.raw_hs(hs_out),.raw_vs(vs_out),
    .display_drop(display_drop),
    .legacy_rgb(ui_video_rgb),.legacy_de(ui_video_de),.legacy_hs(ui_video_hs),.legacy_vs(ui_video_vs),
    .video_rgb(measurement_video_rgb),.video_de(measurement_video_de),.video_hs(measurement_video_hs),.video_vs(measurement_video_vs),
    .patch_req_valid(patch_req_valid),.patch_req_ready(patch_req_ready),.patch_req_base_word(patch_base_word),
    .patch_req_frame_tag(patch_request_tag),.patch_frame_locked(patch_frame_locked),.patch_current_frame_tag(patch_current_tag),
    .patch_req_x(patch_x),.patch_req_y(patch_y),.patch_req_width(patch_width),.patch_req_height(patch_height),
    .patch_pix_valid(patch_pix_valid),.patch_pix_ready(patch_pix_ready),.patch_pix_rgb565(patch_rgb565),
    .patch_pix_x(patch_pixel_x),.patch_pix_y(patch_pixel_y),.patch_pix_frame_tag(patch_pixel_tag),.patch_pix_last(patch_pix_last),
    .patch_done(patch_done),.patch_error(patch_error),.patch_error_code(patch_error_code),.patch_busy(patch_busy));
end else if(ENABLE_MEASUREMENT==0) begin:measurement_disabled
 assign measurement_block_touch=1'b0;assign measurement_active=1'b0;assign measurement_force_static=1'b0;
 assign measurement_allow_left=1'b1;assign measurement_allow_right=1'b1;
 assign measurement_video_rgb=ui_video_rgb;assign measurement_video_de=ui_video_de;
 assign measurement_video_hs=ui_video_hs;assign measurement_video_vs=ui_video_vs;
end else begin:measurement_invalid_parameters
 MEASUREMENT_REQUIRES_ALGORITHM_AUX_RENDER_AND_NO_LIVE_OR_BARS invalid_configuration();
end endgenerate

/************************************************************
// channel arbitration
************************************************************/ 
// read arbitration
mem_read_arbi #(
	.MEM_DATA_BITS               (MEM_DATA_BITS),
	.ADDR_BITS                   (ADDR_BITS    ),
	.BUSRT_BITS                  (BUSRT_BITS   )
)mem_read_arbi_inst(
	.rst_n                        (ddr_init_done),
	.mem_clk                      (core_clk),
	.ch0_rd_burst_req             (rd_burst_req_0),
	.ch0_rd_burst_len             (rd_burst_len_0),
	.ch0_rd_burst_addr            (rd_burst_addr_0),
	.ch0_rd_burst_data_valid      (rd_burst_data_valid_0),
	.ch0_rd_burst_data            (rd_burst_data_0),
	.ch0_rd_burst_finish          (rd_burst_finish_0),
	
	.ch1_rd_burst_req             (rd_burst_req_1),
	.ch1_rd_burst_len             (rd_burst_len_1),
	.ch1_rd_burst_addr            (rd_burst_addr_1),
	.ch1_rd_burst_data_valid      (rd_burst_data_valid_1),
	.ch1_rd_burst_data            (rd_burst_data_1),
	.ch1_rd_burst_finish          (rd_burst_finish_1),

	.ch2_rd_burst_req             (rd_burst_req_2),
	.ch2_rd_burst_len             (rd_burst_len_2),
	.ch2_rd_burst_addr            (rd_burst_addr_2),
	.ch2_rd_burst_data_valid      (rd_burst_data_valid_2),
	.ch2_rd_burst_data            (rd_burst_data_2),
	.ch2_rd_burst_finish          (rd_burst_finish_2),

	.ch3_rd_burst_req             (rd_burst_req_3),
	.ch3_rd_burst_len             (rd_burst_len_3),
	.ch3_rd_burst_addr            (rd_burst_addr_3),
	.ch3_rd_burst_data_valid      (rd_burst_data_valid_3),
	.ch3_rd_burst_data            (rd_burst_data_3),
	.ch3_rd_burst_finish          (rd_burst_finish_3),
	
	.rd_burst_req                 (legacy_rd_req),
	.rd_burst_len                 (legacy_rd_len),
	.rd_burst_addr                (legacy_rd_addr),
	.rd_burst_data_valid          (legacy_rd_valid),
	.rd_burst_data                (legacy_rd_data),
	.rd_burst_finish              (legacy_rd_done)
);

generate if(ENABLE_AUX_READ!=0) begin: auxiliary_read_service
    // Reduce multi-bit FIFO levels to a single urgency flag in their own domain.
    reg display_urgent_pixel;
    (* ASYNC_REG="TRUE" *) reg [1:0] display_urgent_core;
    always @(posedge pclk or negedge ddr_init_done) begin
        if(!ddr_init_done) display_urgent_pixel<=1'b1;
        else display_urgent_pixel<=read_empty_0 || read_empty_1 ||
            read_level_0<13'd1024 || read_level_1<13'd1024;
    end
    always @(posedge core_clk or negedge ddr_init_done) begin
        if(!ddr_init_done) display_urgent_core<=2'b11;
        else display_urgent_core<={display_urgent_core[0],display_urgent_pixel};
    end
    wire aux_valid,aux_ready,aux_data_valid,aux_done;
    wire [24:0] aux_addr;
    wire [9:0] aux_len;
    wire [255:0] aux_data;
    ddr_patch_reader u_patch_reader(
        .clk(core_clk),.rst_n(ddr_init_done),.req_valid(patch_req_valid),.req_ready(patch_req_ready),
        .req_base_word(patch_base_word),.req_frame_tag(patch_request_tag),
        .frame_locked(patch_frame_locked),.current_frame_tag(patch_current_tag),
        .req_x(patch_x),.req_y(patch_y),.req_width(patch_width),.req_height(patch_height),
        .pix_valid(patch_pix_valid),.pix_ready(patch_pix_ready),.pix_rgb565(patch_rgb565),
        .pix_x(patch_pixel_x),.pix_y(patch_pixel_y),.pix_frame_tag(patch_pixel_tag),.pix_last(patch_pix_last),
        .done(patch_done),.error(patch_error),.error_code(patch_error_code),.busy(patch_busy),
        .rd_valid(aux_valid),.rd_ready(aux_ready),.rd_addr(aux_addr),.rd_len(aux_len),
        .rd_data_valid(aux_data_valid),.rd_data(aux_data),.rd_done(aux_done),.rd_fault(patch_ddr_fault));
    ddr_aux_read_mux u_aux_mux(
        .clk(core_clk),.rst_n(ddr_init_done),.legacy_req(legacy_rd_req),.legacy_addr(legacy_rd_addr),
        .legacy_len(legacy_rd_len),.legacy_urgent(display_urgent_core[1]),
        .legacy_data_valid(legacy_rd_valid),.legacy_data(legacy_rd_data),.legacy_done(legacy_rd_done),
        .aux_valid(aux_valid),.aux_ready(aux_ready),.aux_addr(aux_addr),.aux_len(aux_len),
        .aux_data_valid(aux_data_valid),.aux_data(aux_data),.aux_done(aux_done),.fault(patch_ddr_fault),
        .mem_req(rd_burst_req),.mem_addr(rd_burst_addr),.mem_len(rd_burst_len),.mem_ready(ddr_rd_ready),
        .mem_data_valid(rd_burst_data_valid),.mem_data(rd_burst_data),.mem_done(rd_burst_finish));
end else begin: original_read_path
    assign rd_burst_req=legacy_rd_req;
    assign rd_burst_addr=legacy_rd_addr;
    assign rd_burst_len=legacy_rd_len;
    assign legacy_rd_valid=rd_burst_data_valid;
    assign legacy_rd_data=rd_burst_data;
    assign legacy_rd_done=rd_burst_finish;
    assign patch_req_ready=1'b0;assign patch_pix_valid=1'b0;assign patch_pix_last=1'b0;
    assign patch_rgb565=16'd0;assign patch_pixel_x=12'd0;assign patch_pixel_y=12'd0;
    assign patch_pixel_tag=32'd0;assign patch_done=1'b0;assign patch_error=1'b0;
    assign patch_error_code=3'd0;assign patch_busy=1'b0;assign patch_ddr_fault=1'b0;
end endgenerate
// write arbitration
mem_write_arbi #(
	.MEM_DATA_BITS               (MEM_DATA_BITS),
	.ADDR_BITS                   (ADDR_BITS    ),
	.BUSRT_BITS                  (BUSRT_BITS   )
)mem_write_arbi_inst(
	.rst_n                       (ddr_init_done),
	.mem_clk                     (core_clk),
	
	.ch0_wr_burst_req            (wr_burst_req_0),
	.ch0_wr_burst_len            (wr_burst_len_0),
	.ch0_wr_burst_addr           (wr_burst_addr_0),
	.ch0_wr_burst_data_req       (wr_burst_data_req_0),
	.ch0_wr_burst_data           (wr_burst_data_0),
	.ch0_wr_burst_finish         (wr_burst_finish_0),
	
	.ch1_wr_burst_req            (wr_burst_req_1),
	.ch1_wr_burst_len            (wr_burst_len_1),
	.ch1_wr_burst_addr           (wr_burst_addr_1),
	.ch1_wr_burst_data_req       (wr_burst_data_req_1),
	.ch1_wr_burst_data           (wr_burst_data_1),
	.ch1_wr_burst_finish         (wr_burst_finish_1),

	.ch2_wr_burst_req            (wr_burst_req_2),
	.ch2_wr_burst_len            (wr_burst_len_2),
	.ch2_wr_burst_addr           (wr_burst_addr_2),
	.ch2_wr_burst_data_req       (wr_burst_data_req_2),
	.ch2_wr_burst_data           (wr_burst_data_2),
	.ch2_wr_burst_finish         (wr_burst_finish_2),

	.ch3_wr_burst_req            (wr_burst_req_3),
	.ch3_wr_burst_len            (wr_burst_len_3),
	.ch3_wr_burst_addr           (wr_burst_addr_3),
	.ch3_wr_burst_data_req       (wr_burst_data_req_3),
	.ch3_wr_burst_data           (wr_burst_data_3),
	.ch3_wr_burst_finish         (wr_burst_finish_3),

	.wr_burst_req(wr_burst_req),
	.wr_burst_len(wr_burst_len),
	.wr_burst_addr(wr_burst_addr),
	.wr_burst_data_req(wr_burst_data_req),
	.wr_burst_data(wr_burst_data),
	.wr_burst_finish(wr_burst_finish)	
);


wire [CTRL_ADDR_WIDTH-1:0]  axi_awaddr      ;
wire [3:0]                  axi_awid        ;
wire [3:0]                  axi_awlen       ;
wire                        axi_awready     ;
wire                        axi_awvalid     ;

wire [MEM_DQ_WIDTH*8-1:0]   axi_wdata       ;
wire [MEM_DQ_WIDTH*8/8-1:0] axi_wstrb       ;
wire                        axi_wready      ;

wire [CTRL_ADDR_WIDTH-1:0]  axi_araddr      ;
wire [3:0]                  axi_arid        ;
wire [3:0]                  axi_arlen       ;
wire                        axi_arready     ;
wire                        axi_arvalid     ;

wire [MEM_DQ_WIDTH*8-1:0]   axi_rdata       ;
wire                        axi_rid         ;
wire [3:0]                  axi_rlast       ;
wire                        axi_rvalid      ;

wire axi_bvalid;
assign axi_bvalid =1'b1; 
aq_axi_master_256	u_aq_axi_master (
	  .ARESETN                     (ddr_init_done         ),
	  .ACLK                        (core_clk              ),
	  .M_AXI_AWID                  (axi_awid          ),
	  .M_AXI_AWADDR                (axi_awaddr        ),
	  .M_AXI_AWLEN                 (axi_awlen         ),
	  .M_AXI_AWSIZE                (),
	  .M_AXI_AWBURST               (),
	  .M_AXI_AWLOCK                (),
	  .M_AXI_AWCACHE               (),
	  .M_AXI_AWPROT                (),
	  .M_AXI_AWQOS                 (),
	  .M_AXI_AWUSER                (),
	  .M_AXI_AWVALID               (axi_awvalid       ),
	  .M_AXI_AWREADY               (axi_awready       ),

	  .M_AXI_WDATA                 (axi_wdata         ),
	  .M_AXI_WSTRB                 (axi_wstrb         ),
	  .M_AXI_WLAST                 (),
	  .M_AXI_WUSER                 (),
	  .M_AXI_WVALID                (),
	  .M_AXI_WREADY                (axi_wready        ),

	  .M_AXI_BID                   (),
	  .M_AXI_BRESP                 (),
	  .M_AXI_BUSER                 (),
      .M_AXI_BVALID                (axi_bvalid        ),
	  .M_AXI_BREADY                (),

	  .M_AXI_ARID                  (axi_arid          ),
	  .M_AXI_ARADDR                (axi_araddr        ),
	  .M_AXI_ARLEN                 (axi_arlen         ),
	  .M_AXI_ARSIZE                (),
	  .M_AXI_ARBURST               (),
	  .M_AXI_ARLOCK                (),
	  .M_AXI_ARCACHE               (),
	  .M_AXI_ARPROT                (),
	  .M_AXI_ARQOS                 (),
	  .M_AXI_ARUSER                (),
	  .M_AXI_ARVALID               (axi_arvalid       ),
	  .M_AXI_ARREADY               (axi_arready       ),

	  .M_AXI_RID                   (axi_rid           ),
	  .M_AXI_RDATA                 (axi_rdata         ),
	  .M_AXI_RRESP                 (),
	  .M_AXI_RLAST                 (axi_rlast         ),
	  .M_AXI_RUSER                 (),
	  .M_AXI_RVALID                (axi_rvalid        ),
	  .M_AXI_RREADY                (),

	  .MASTER_RST                  (1'b0                  ),
	  .WR_START                    (wr_burst_req          ),
	  // Pango DDR addresses count 32-bit memory locations, not bytes:
	  // one 256-bit UI word spans eight locations. Length remains in bytes
	  // because aq_axi_master_256 uses LEN[...:5] to calculate burst beats.
	  .WR_ADRS                     ({4'b0,wr_burst_addr,3'd0}  ),
	  .WR_LEN                      ({17'd0,wr_burst_len,5'd0}  ),
	  .WR_READY                    (                      ),
	  .WR_FIFO_RE                  (wr_burst_data_req     ),
	  .WR_FIFO_EMPTY               (1'b0                  ),
	  .WR_FIFO_AEMPTY              (1'b0                  ),
	  .WR_FIFO_DATA                (wr_burst_data         ),
	  .WR_DONE                     (wr_burst_finish       ),
	  .RD_START                    (rd_burst_req          ),
	  .RD_ADRS                     ({4'b0,rd_burst_addr,3'd0}  ),
	  .RD_LEN                      ({17'd0,rd_burst_len,5'd0}   ),
	  .RD_READY                    (ddr_rd_ready          ),
	  .RD_FIFO_WE                  (rd_burst_data_valid   ),
	  .RD_FIFO_FULL                (1'b0                  ),
	  .RD_FIFO_AFULL               (1'b0                  ),
	  .RD_FIFO_DATA                (rd_burst_data         ),
	  .RD_DONE                     (rd_burst_finish       ),
	  .DEBUG                       (                      )
);
//=====================================================================================
//ddr
//=====================================================================================
wire clk_125Mhz ;

GTP_INBUFGDS #(
    .IOSTANDARD("DEFAULT"),
    .TERM_DIFF("ON")
) u_gtp (
    .O(clk_125Mhz), // OUTPUT  
    .I(clk_p), // INPUT  
    .IB(clk_n) // INPUT  
);

//ddr    
DDR_200H DDR_200H_inst (
    .ref_clk                   (clk_125Mhz            ),
    .resetn                    (rst_n               ),// input
    .ddr_init_done             (ddr_init_done      ),// outpu
    .pll_lock                  (pll_lock           ),// outpu
    .core_clk                  (core_clk),                                  // outpu
    .phy_pll_lock              (phy_pll_lock),                          // output
    .gpll_lock                 (gpll_lock),                                // output
    .rst_gpll_lock             (rst_gpll_lock),                        // output
    .ddrphy_cpd_lock           (ddrphy_cpd_lock),                    // output
    //.ddr_init_done             (ddr_init_done),                        // outpu
    .axi_awaddr                (axi_awaddr         ),// input [27:0]
    .axi_awuser_ap             (1'b0               ),// input
    .axi_awuser_id             (axi_awid           ),// input [3:0]
    .axi_awlen                 (axi_awlen          ),// input [3:0]
    .axi_awready               (axi_awready        ),// output
    .axi_awvalid               (axi_awvalid        ),// input

    .axi_wdata                 (axi_wdata          ),
    .axi_wstrb                 (axi_wstrb          ),// input [31:0]
    .axi_wready                (axi_wready         ),// output
    .axi_wusero_id             (                   ),// output [3:0]
    .axi_wusero_last           (),// output

    .axi_araddr                (axi_araddr         ),// input [27:0]
    .axi_aruser_ap             (1'b0               ),// input
    .axi_aruser_id             (axi_arid           ),// input [3:0]
    .axi_arlen                 (axi_arlen          ),// input [3:0]
    .axi_arready               (axi_arready        ),// output
    .axi_arvalid               (axi_arvalid        ),// input

    .axi_rdata                 (axi_rdata          ),// output [255:0]
    .axi_rid                   (axi_rid            ),// output [3:0]
    .axi_rlast                 (axi_rlast          ),// output
    .axi_rvalid                (axi_rvalid         ),// outpu

    .apb_clk                   (1'b0               ),// input
    .apb_rst_n                 (1'b1               ),// input
    .apb_sel                   (1'b0               ),// input
    .apb_enable                (1'b0               ),// input
    .apb_addr                  (8'b0               ),// input [7:0]
    .apb_write                 (1'b0               ),// input
    .apb_ready                 (                   ), // output
    .apb_wdata                 (16'b0              ),// input [15:0]
    .apb_rdata                 (                   ),// output [15:0]
//          .apb_int                   (                   ),// outpu
    .mem_rst_n                 (mem_rst_n          ),// output
    .mem_ck                    (mem_ck             ),// output
    .mem_ck_n                  (mem_ck_n           ),// output
    .mem_cke                   (mem_cke            ),// output
    .mem_cs_n                  (mem_cs_n           ),// output
    .mem_ras_n                 (mem_ras_n          ),// output
    .mem_cas_n                 (mem_cas_n          ),// output
    .mem_we_n                  (mem_we_n           ),// output
    .mem_odt                   (mem_odt            ),// output
    .mem_a                     (mem_a              ),// output [14:0]
    .mem_ba                    (mem_ba             ),// output [2:0]
    .mem_dqs                   (mem_dqs            ),// inout [3:0]
    .mem_dqs_n                 (mem_dqs_n          ),// inout [3:0]
    .mem_dq                    (mem_dq             ),// inout [31:0]
    .mem_dm                    (mem_dm             ),// output [3:0]
    //debug

    .dbg_gate_start(1'b0),                      // input
    .dbg_cpd_start(1'b0),                        // input
    .dbg_ddrphy_rst_n(1'b1),                  // input
    .dbg_gpll_scan_rst(1'b0),                // input
    .samp_position_dyn_adj(1'b0),        // input
    .init_samp_position_even(32'd0),    // input [31:0]
    .init_samp_position_odd(32'd0),      // input [31:0]
    .wrcal_position_dyn_adj(1'b0),      // input
    .init_wrcal_position(32'd0),            // input [31:0]
    .force_read_clk_ctrl(1'b0),            // input
    .init_slip_step(16'd0),                      // input [15:0]
    .init_read_clk_ctrl(12'd0),              // input [11:0]
    .debug_calib_ctrl(),                  // output [33:0]
    .dbg_slice_status(),                  // output [67:0]
    .dbg_slice_state(),                    // output [87:0]
    .debug_data(),                              // output [275:0]
    .dbg_dll_upd_state(),                // output [1:0]
    .debug_gpll_dps_phase(),          // output [8:0]
    .dbg_rst_dps_state(),                // output [2:0]
    .dbg_tran_err_rst_cnt(),          // output [5:0]
    .dbg_ddrphy_init_fail(),          // output
    .debug_cpd_offset_adj(1'b0),          // input
    .debug_cpd_offset_dir(1'b0),          // input
    .debug_cpd_offset(10'd0),                  // input [9:0]
    .debug_dps_cnt_dir0(),              // output [9:0]
    .debug_dps_cnt_dir1(),              // output [9:0]
    .ck_dly_en(1'b0),                                // input
    .init_ck_dly_step(8'h0),                  // input [7:0]
    .ck_dly_set_bin(),                      // output [7:0]
    .align_error(),                            // output
    .debug_rst_state(),                    // output [3:0]
    .debug_cpd_state()                     // output [3:0]
);

//=====================================================================================
// heartbeat counter
//=====================================================================================
reg [26:0] cnt;
always@(posedge core_clk) begin
    if (!ddr_init_done)
        cnt <= 27'd0;
    else if ( cnt >= TH_1S )
        cnt <= 27'd0;
    else
        cnt <= cnt + 27'd1;
end

always @(posedge core_clk) begin
    if (!ddr_init_done)
        heart_beat_led <= 1'd1;
    else if ( cnt >= TH_1S )
        heart_beat_led <= ~heart_beat_led;
end

endmodule

// Keep a stable black HDMI raster while DDR and both cameras initialize.
// This helper is intentionally kept in Top.v: Pango's GUI regenerates the
// project source list and otherwise drops a newly-added standalone RTL file.
module video_startup_guard #(
    parameter integer SYS_CLK_HZ = 27000000,
    parameter integer DELAY_MS   = 1000
)(
    input  wire        sys_clk,
    input  wire        pixel_clk,
    input  wire        rst_n,
    input  wire        prerequisites_ready,
    input  wire [23:0] input_rgb,
    input  wire        input_de,
    input  wire        input_hs,
    input  wire        input_vs,
    output wire [23:0] output_rgb,
    output wire        output_de,
    output wire        output_hs,
    output wire        output_vs,
    output wire        display_ready
);
    localparam integer DELAY_CYCLES_RAW = (SYS_CLK_HZ / 1000) * DELAY_MS;
    localparam integer DELAY_CYCLES = (DELAY_CYCLES_RAW < 1) ? 1 : DELAY_CYCLES_RAW;

    reg [31:0] delay_count;
    reg        release_sys;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n || !prerequisites_ready) begin
            delay_count <= 32'd0;
            release_sys <= 1'b0;
        end else if (!release_sys) begin
            if (delay_count >= DELAY_CYCLES - 1)
                release_sys <= 1'b1;
            else
                delay_count <= delay_count + 1'b1;
        end
    end

    (* ASYNC_REG="TRUE" *) reg release_meta;
    (* ASYNC_REG="TRUE" *) reg release_sync;
    reg vs_previous;
    reg release_frame;
    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            release_meta  <= 1'b0;
            release_sync  <= 1'b0;
            vs_previous   <= 1'b0;
            release_frame <= 1'b0;
        end else begin
            release_meta <= release_sys;
            release_sync <= release_meta;
            vs_previous  <= input_vs;
            if (!release_sync)
                release_frame <= 1'b0;
            else if (input_vs && !vs_previous)
                release_frame <= 1'b1;
        end
    end

    assign output_rgb   = release_frame ? input_rgb : 24'h000000;
    // Preserve valid HDMI timing during startup; blank only the pixel color.
    assign output_de    = input_de;
    assign output_hs    = input_hs;
    assign output_vs    = input_vs;
    assign display_ready = release_frame;
endmodule

