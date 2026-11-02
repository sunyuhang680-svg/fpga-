// SPDX-License-Identifier: MIT
// RGB insertion point before HDMI encoding. Positive-polarity VS assumed.
module touch_video_overlay #(
    parameter integer PIXEL_CLK_HZ=74250000,
    parameter integer SCREEN_W=1280, SCREEN_H=720,
    parameter integer CROSS_RADIUS=12,
    parameter integer SHOW_DEBUG=0
)(
    input wire sys_clk, pixel_clk, rst_n,
    input wire chip_identified,
    input wire spi_probe_event,
    input wire [24:0] spi_probe_data,
    input wire diagnostic_event,
    input wire [511:0] diagnostic_data,
    input wire touch_event, point0_valid, point1_valid,
    input wire [15:0] point0_x,point0_y,point1_x,point1_y,
    input wire [23:0] camera_rgb,
    input wire camera_de,camera_hs,camera_vs,
    output wire [23:0] video_rgb,
    output wire video_de,video_hs,video_vs
);
    wire event_pixel;
    wire diagnostic_pixel_event;
    wire [511:0] diagnostic_pixel_data;
    wire probe_pixel_event;
    wire [24:0] probe_pixel_data;
    wire chip_sync;
    // Normal display carries only touch points across clocks. The debug
    // mailboxes/synchronizers are not instantiated when SHOW_DEBUG is zero.
    generate if(SHOW_DEBUG!=0)begin:g_debug
    touch_event_cdc #(.WIDTH(512)) u_diagnostic_cdc (
        .src_clk(sys_clk),.dst_clk(pixel_clk),.rst_n(rst_n),
        .src_event(diagnostic_event),.src_data(diagnostic_data),
        .dst_event(diagnostic_pixel_event),.dst_data(diagnostic_pixel_data)
    );
    touch_event_cdc #(.WIDTH(25)) u_probe_cdc (
        .src_clk(sys_clk),.dst_clk(pixel_clk),.rst_n(rst_n),
        .src_event(spi_probe_event),.src_data(spi_probe_data),
        .dst_event(probe_pixel_event),.dst_data(probe_pixel_data)
    );
    (* ASYNC_REG="TRUE" *) reg chip_meta,chip_sync_reg;
    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin chip_meta<=0;chip_sync_reg<=0;end
        else begin chip_meta<=chip_identified;chip_sync_reg<=chip_meta;end
    end
    assign chip_sync=chip_sync_reg;
    end else begin:g_normal
        assign diagnostic_pixel_event=1'b0;
        assign diagnostic_pixel_data=512'd0;
        assign probe_pixel_event=1'b0;
        assign probe_pixel_data=25'd0;
        assign chip_sync=1'b0;
    end endgenerate
    wire [65:0] points;
    touch_event_cdc #(.WIDTH(66)) u_touch_cdc (
        .src_clk(sys_clk),.dst_clk(pixel_clk),.rst_n(rst_n),
        .src_event(touch_event),
        .src_data({point1_valid,point0_valid,point1_y,point1_x,point0_y,point0_x}),
        .dst_event(event_pixel),.dst_data(points)
    );
    reg [15:0] pixel_x,pixel_y;
    reg de_previous,vs_previous;
    wire frame_start=camera_vs && !vs_previous;
    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin pixel_x<=0;pixel_y<=0;de_previous<=0;vs_previous<=0;end
        else begin
            de_previous<=camera_de;vs_previous<=camera_vs;
            if (frame_start) begin pixel_x<=0;pixel_y<=0;end
            else begin
                if (camera_de) pixel_x<=pixel_x+1'b1;
                else pixel_x<=0;
                if (!camera_de && de_previous) pixel_y<=pixel_y+1'b1;
            end
        end
    end
    wire menu_valid;
    wire [23:0] menu_rgb;
    touch_menu_layer #(.PIXEL_CLK_HZ(PIXEL_CLK_HZ),.SCREEN_W(SCREEN_W),
        .SCREEN_H(SCREEN_H),.CROSS_RADIUS(CROSS_RADIUS),.SHOW_DEBUG(SHOW_DEBUG)) u_menu (
        .clk(pixel_clk),.rst_n(rst_n),.frame_start(frame_start),
        .chip_identified(chip_sync),
        .spi_probe_event(probe_pixel_event),.spi_probe_data(probe_pixel_data),
        .diagnostic_event(diagnostic_pixel_event),.diagnostic_data(diagnostic_pixel_data),
        .pixel_x(pixel_x),.pixel_y(pixel_y),.pixel_de(camera_de),
        .touch_event(event_pixel),.point0_valid(points[64]),.point1_valid(points[65]),
        .point0_x(points[15:0]),.point0_y(points[31:16]),
        .point1_x(points[47:32]),.point1_y(points[63:48]),
        .menu_valid(menu_valid),.menu_rgb(menu_rgb),.clear_tick()
    );
    video_layer_map u_map (
        .clk(pixel_clk),.rst_n(rst_n),.camera_rgb(camera_rgb),.menu_rgb(menu_rgb),
        .camera_de(camera_de),.camera_hs(camera_hs),.camera_vs(camera_vs),
        .menu_valid(menu_valid),.video_rgb(video_rgb),.video_de(video_de),
        .video_hs(video_hs),.video_vs(video_vs)
    );
endmodule
