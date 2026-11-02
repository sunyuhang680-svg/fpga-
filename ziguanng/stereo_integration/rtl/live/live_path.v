`timescale 1ns/1ps
// Live (frame-bufferless) video path: two camera row rings, camera-locked
// display timing and the streaming undistortion.  Outputs a 1280x720 RGB565
// stream with its own hs/vs/de in the pixel clock domain.
// cam_sel picks the camera, bypass shows it raw; both are taken at the start
// of a frame.  Timing follows the selected camera.
module live_path #(
    parameter integer RING_LOG2=6,
    parameter integer LAG_ROWS=20,
    parameter integer MAX_WAIT=1023
)(
    input wire cam0_clk,cam0_rst_n,cam0_vsync,cam0_href,cam0_valid,input wire[15:0]cam0_pixel,
    input wire cam1_clk,cam1_rst_n,cam1_vsync,cam1_href,cam1_valid,input wire[15:0]cam1_pixel,
    input wire pix_clk,pix_rst_n,
    input wire cam_sel,bypass,
    output wire hs,vs,de,
    output wire[15:0]pixel,
    // request-side timing (leads hs/vs/de); the DDR readers can use it
    output wire timing_hs,timing_vs,timing_de,
    // measurements
    output wire[11:0]h_total,v_total_last,wait_last,
    output wire locked,
    output wire[31:0]row0_to_start,
    output reg[15:0]row_miss_count,       // pixels that needed a missing row, previous frame
    output wire frame_start
);
    localparam integer W=1280,H=720;
    localparam integer AW=$clog2(((1<<RING_LOG2)/2)*(W/2));
    wire[15:0]rows0,base0,rows1,base1;wire ev0,ev1;
    wire[AW-1:0]a00,a01,a10,a11;
    wire[15:0]d0_00,d0_01,d0_10,d0_11,d1_00,d1_01,d1_10,d1_11;
    live_line_ring #(.WIDTH(W),.RING_LOG2(RING_LOG2)) ring0(
        .cam_clk(cam0_clk),.cam_rst_n(cam0_rst_n),.cam_vsync(cam0_vsync),.cam_href(cam0_href),.cam_valid(cam0_valid),.cam_pixel(cam0_pixel),
        .pix_clk(pix_clk),.pix_rst_n(pix_rst_n),.rows_total_pix(rows0),.frame_base_pix(base0),.frame_event_pix(ev0),
        .rd_addr00(a00),.rd_addr01(a01),.rd_addr10(a10),.rd_addr11(a11),.rd_data00(d0_00),.rd_data01(d0_01),.rd_data10(d0_10),.rd_data11(d0_11));
    live_line_ring #(.WIDTH(W),.RING_LOG2(RING_LOG2)) ring1(
        .cam_clk(cam1_clk),.cam_rst_n(cam1_rst_n),.cam_vsync(cam1_vsync),.cam_href(cam1_href),.cam_valid(cam1_valid),.cam_pixel(cam1_pixel),
        .pix_clk(pix_clk),.pix_rst_n(pix_rst_n),.rows_total_pix(rows1),.frame_base_pix(base1),.frame_event_pix(ev1),
        .rd_addr00(a00),.rd_addr01(a01),.rd_addr10(a10),.rd_addr11(a11),.rd_data00(d1_00),.rd_data01(d1_01),.rd_data10(d1_10),.rd_data11(d1_11));

    // the timing follows the camera chosen for the next frame
    reg sel_t;
    always @(posedge pix_clk or negedge pix_rst_n)if(!pix_rst_n)sel_t<=0;else sel_t<=cam_sel;
    wire[11:0]tx,ty;
    live_genlock_timing #(.LAG_ROWS(LAG_ROWS),.MAX_WAIT(MAX_WAIT)) timing(
        .clk(pix_clk),.rst_n(pix_rst_n),
        .rows_total(sel_t?rows1:rows0),.frame_base(sel_t?base1:base0),.frame_event(sel_t?ev1:ev0),
        .hs(timing_hs),.vs(timing_vs),.de(timing_de),.x(tx),.y(ty),.frame_start(frame_start),
        .h_total(h_total),.v_total_last(v_total_last),.wait_last(wait_last),.locked(locked),.row0_to_start(row0_to_start));

    wire[383:0]coeff0,coeff1;
    live_calibration_constants constants(.left_coeff(coeff0),.right_coeff(coeff1));
    wire miss;wire[2:0]otag;
    live_undistort_stream #(.WIDTH(W),.HEIGHT(H),.RING_LOG2(RING_LOG2),.TAG_W(3)) stream(
        .clk(pix_clk),.rst_n(pix_rst_n),.frame_start(frame_start),.cam_sel_in(sel_t),.bypass_in(bypass),
        .coeff0(coeff0),.coeff1(coeff1),.base0(base0),.base1(base1),.rows0(rows0),.rows1(rows1),
        .req_valid(timing_de),.req_x(tx),.req_y(ty),.req_tag({timing_hs,timing_vs,timing_de}),
        .addr00(a00),.addr01(a01),.addr10(a10),.addr11(a11),
        .ram0_00(d0_00),.ram0_01(d0_01),.ram0_10(d0_10),.ram0_11(d0_11),
        .ram1_00(d1_00),.ram1_01(d1_01),.ram1_10(d1_10),.ram1_11(d1_11),
        .out_valid(),.out_pixel(pixel),.out_tag(otag),.row_miss(miss),
        .dbg_valid(),.dbg_us(),.dbg_vs());
    assign {hs,vs,de}=otag;
    reg[15:0]miss_run;
    always @(posedge pix_clk or negedge pix_rst_n)begin
        if(!pix_rst_n)begin miss_run<=0;row_miss_count<=0;end
        else if(frame_start)begin row_miss_count<=miss_run;miss_run<=0;end
        else if(miss && miss_run!=16'hffff)miss_run<=miss_run+16'd1;
    end
endmodule
