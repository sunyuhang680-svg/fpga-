module testing60_imu_pose_overlay #(
    parameter integer SYS_CLK_HZ = 27000000,
    parameter integer MOTION_THRESHOLD_RAW = 250,
    parameter integer MOTION_HOLD_MS = 500,
    parameter integer ENABLE = 1
)(
    input  wire sys_clk,
    input  wire pixel_clk,
    input  wire rst_n,
    input  wire menu_visible_sys,
    input  wire menu_visible_pixel,
    input  wire imu_data_valid,
    input  wire imu_attitude_ready,
    input  wire imu_angle_valid,
    input  wire signed [15:0] gyro_x_raw,
    input  wire signed [15:0] gyro_y_raw,
    input  wire signed [15:0] gyro_z_raw,
    input  wire signed [31:0] q0,
    input  wire signed [31:0] q1,
    input  wire signed [31:0] q2,
    input  wire signed [31:0] q3,
    input  wire [23:0] camera_rgb,
    input  wire camera_de,
    input  wire camera_hs,
    input  wire camera_vs,
    output reg  [23:0] video_rgb,
    output reg  video_de,
    output reg  video_hs,
    output reg  video_vs
);
    localparam integer HOLD_CYCLES_RAW = (SYS_CLK_HZ/1000)*MOTION_HOLD_MS;
    localparam integer HOLD_CYCLES = (HOLD_CYCLES_RAW < 1) ? 1 : HOLD_CYCLES_RAW;
    localparam signed [63:0] ONE_Q30 = 64'sd1073741824;
    localparam signed [31:0] CUBE_HALF = 32'sd30;
    localparam signed [31:0] DEPTH_X = 32'sd15;
    localparam signed [31:0] DEPTH_Y = 32'sd10;
    localparam signed [15:0] CENTER_X = 16'sd105;
    localparam signed [15:0] CENTER_Y = 16'sd590;

    function [15:0] abs16;
        input signed [15:0] value;
        begin
            abs16 = value[15] ? (~value + 16'd1) : value;
        end
    endfunction

    wire moving_sample = (abs16(gyro_x_raw) >= MOTION_THRESHOLD_RAW) ||
                         (abs16(gyro_y_raw) >= MOTION_THRESHOLD_RAW) ||
                         (abs16(gyro_z_raw) >= MOTION_THRESHOLD_RAW);
    reg [31:0] motion_hold_count;
    reg motion_active_sys;
    reg motion_previous;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n || !menu_visible_sys || !ENABLE) begin
            motion_hold_count <= 32'd0;
            motion_active_sys <= 1'b0;
            motion_previous <= 1'b0;
        end else begin
            motion_previous <= motion_active_sys;
            if (imu_data_valid && imu_attitude_ready && moving_sample) begin
                motion_hold_count <= HOLD_CYCLES;
                motion_active_sys <= 1'b1;
            end else if (motion_active_sys) begin
                if (motion_hold_count == 0)
                    motion_active_sys <= 1'b0;
                else
                    motion_hold_count <= motion_hold_count - 1'b1;
            end
        end
    end

    // q0 is scalar (w), q1/q2/q3 are x/y/z. Products are Q60 and the
    // resulting rotation-matrix entries are reduced back to Q30.
    wire signed [63:0] q1q1 = q1*q1;
    wire signed [63:0] q2q2 = q2*q2;
    wire signed [63:0] q3q3 = q3*q3;
    wire signed [63:0] q0q1 = q0*q1;
    wire signed [63:0] q0q2 = q0*q2;
    wire signed [63:0] q0q3 = q0*q3;
    wire signed [63:0] q1q2 = q1*q2;
    wire signed [63:0] q1q3 = q1*q3;
    wire signed [63:0] q2q3 = q2*q3;
    wire signed [63:0] r00_w = ONE_Q30 - ((q2q2 + q3q3) >>> 29);
    wire signed [63:0] r01_w = (q1q2 - q0q3) >>> 29;
    wire signed [63:0] r02_w = (q1q3 + q0q2) >>> 29;
    wire signed [63:0] r10_w = (q1q2 + q0q3) >>> 29;
    wire signed [63:0] r11_w = ONE_Q30 - ((q1q1 + q3q3) >>> 29);
    wire signed [63:0] r12_w = (q2q3 - q0q1) >>> 29;
    wire signed [63:0] r20_w = (q1q3 - q0q2) >>> 29;
    wire signed [63:0] r21_w = (q2q3 + q0q1) >>> 29;
    wire signed [63:0] r22_w = ONE_Q30 - ((q1q1 + q2q2) >>> 29);
    wire signed [31:0] r00 = r00_w[31:0];
    wire signed [31:0] r01 = r01_w[31:0];
    wire signed [31:0] r02 = r02_w[31:0];
    wire signed [31:0] r10 = r10_w[31:0];
    wire signed [31:0] r11 = r11_w[31:0];
    wire signed [31:0] r12 = r12_w[31:0];
    wire signed [31:0] r20 = r20_w[31:0];
    wire signed [31:0] r21 = r21_w[31:0];
    wire signed [31:0] r22 = r22_w[31:0];

    // Camera/body mounting transform (right-handed):
    //   display X = IMU Z  (camera optical/forward direction)
    //   display Y = IMU Y
    //   display Z = -IMU X (physical up direction)
    // World Y is drawn to screen-right, world Z to screen-up, while world X
    // receives a small diagonal depth offset. This makes the calibrated pose
    // look like the real camera instead of the upside-down sensor package.
    wire signed [63:0] bx_x_w = r12*CUBE_HALF + r02*DEPTH_X;
    wire signed [63:0] bx_y_w = -r22*CUBE_HALF + r02*DEPTH_Y;
    wire signed [63:0] by_x_w = r11*CUBE_HALF + r01*DEPTH_X;
    wire signed [63:0] by_y_w = -r21*CUBE_HALF + r01*DEPTH_Y;
    wire signed [63:0] bz_x_w = -r10*CUBE_HALF - r00*DEPTH_X;
    wire signed [63:0] bz_y_w = r20*CUBE_HALF - r00*DEPTH_Y;
    wire signed [15:0] bx_x_sys = bx_x_w >>> 30;
    wire signed [15:0] bx_y_sys = bx_y_w >>> 30;
    wire signed [15:0] by_x_sys = by_x_w >>> 30;
    wire signed [15:0] by_y_sys = by_y_w >>> 30;
    wire signed [15:0] bz_x_sys = bz_x_w >>> 30;
    wire signed [15:0] bz_y_sys = bz_y_w >>> 30;

    wire pose_source_event = imu_angle_valid ||
                             (motion_active_sys != motion_previous);
    wire pose_pixel_event;
    wire [96:0] pose_pixel_data;
    touch_event_cdc #(.WIDTH(97)) u_pose_cdc (
        .src_clk(sys_clk),.dst_clk(pixel_clk),.rst_n(rst_n),
        .src_event(pose_source_event),
        .src_data({motion_active_sys,bx_x_sys,bx_y_sys,
                   by_x_sys,by_y_sys,bz_x_sys,bz_y_sys}),
        .dst_event(pose_pixel_event),.dst_data(pose_pixel_data)
    );

    reg [96:0] pose_pending;
    reg [96:0] pose_shown;
    reg [11:0] pixel_x,pixel_y;
    reg de_previous,vs_previous;
    wire frame_start = camera_vs && !vs_previous;
    wire line_end = de_previous && !camera_de;
    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            pose_pending <= 97'd0;
            pose_shown <= 97'd0;
            pixel_x <= 12'd0;
            pixel_y <= 12'd0;
            de_previous <= 1'b0;
            vs_previous <= 1'b0;
        end else begin
            de_previous <= camera_de;
            vs_previous <= camera_vs;
            if (pose_pixel_event)
                pose_pending <= pose_pixel_data;
            if (frame_start) begin
                pixel_y <= 12'd0;
                pose_shown <= pose_pending;
            end else if (line_end)
                pixel_y <= pixel_y + 1'b1;
            if (camera_de)
                pixel_x <= pixel_x + 1'b1;
            else
                pixel_x <= 12'd0;
        end
    end

    wire signed [15:0] bx_x = pose_shown[95:80];
    wire signed [15:0] bx_y = pose_shown[79:64];
    wire signed [15:0] by_x = pose_shown[63:48];
    wire signed [15:0] by_y = pose_shown[47:32];
    wire signed [15:0] bz_x = pose_shown[31:16];
    wire signed [15:0] bz_y = pose_shown[15:0];
    wire signed [15:0] px = $signed({4'b0000,pixel_x});
    wire signed [15:0] py = $signed({4'b0000,pixel_y});

    wire signed [15:0] v0x=CENTER_X-bx_x-by_x-bz_x;
    wire signed [15:0] v0y=CENTER_Y-bx_y-by_y-bz_y;
    wire signed [15:0] v1x=CENTER_X+bx_x-by_x-bz_x;
    wire signed [15:0] v1y=CENTER_Y+bx_y-by_y-bz_y;
    wire signed [15:0] v2x=CENTER_X+bx_x+by_x-bz_x;
    wire signed [15:0] v2y=CENTER_Y+bx_y+by_y-bz_y;
    wire signed [15:0] v3x=CENTER_X-bx_x+by_x-bz_x;
    wire signed [15:0] v3y=CENTER_Y-bx_y+by_y-bz_y;
    wire signed [15:0] v4x=CENTER_X-bx_x-by_x+bz_x;
    wire signed [15:0] v4y=CENTER_Y-bx_y-by_y+bz_y;
    wire signed [15:0] v5x=CENTER_X+bx_x-by_x+bz_x;
    wire signed [15:0] v5y=CENTER_Y+bx_y-by_y+bz_y;
    wire signed [15:0] v6x=CENTER_X+bx_x+by_x+bz_x;
    wire signed [15:0] v6y=CENTER_Y+bx_y+by_y+bz_y;
    wire signed [15:0] v7x=CENTER_X-bx_x+by_x+bz_x;
    wire signed [15:0] v7y=CENTER_Y-bx_y+by_y+bz_y;

    function line_pixel;
        input signed [15:0] test_x,test_y,x0,y0,x1,y1;
        input [2:0] thickness;
        reg signed [16:0] dx,dy,rx,ry,min_x,max_x,min_y,max_y,margin;
        reg signed [33:0] cross_value;
        reg [33:0] cross_abs;
        reg [16:0] dx_abs,dy_abs;
        reg [20:0] tolerance;
        begin
            dx=x1-x0; dy=y1-y0; rx=test_x-x0; ry=test_y-y0;
            min_x=(x0<x1)?x0:x1; max_x=(x0>x1)?x0:x1;
            min_y=(y0<y1)?y0:y1; max_y=(y0>y1)?y0:y1;
            margin={14'd0,thickness};
            dx_abs=dx[16]?-dx:dx; dy_abs=dy[16]?-dy:dy;
            cross_value=rx*dy-ry*dx;
            cross_abs=cross_value[33]?-cross_value:cross_value;
            tolerance=(dx_abs+dy_abs+1'b1)*thickness;
            line_pixel=(test_x>=min_x-margin && test_x<=max_x+margin &&
                        test_y>=min_y-margin && test_y<=max_y+margin &&
                        cross_abs<=tolerance);
        end
    endfunction

    function label_pixel;
        input signed [15:0] test_x,test_y,label_left,label_top;
        input [34:0] glyph_bits;
        reg signed [16:0] glyph_x,glyph_y;
        integer glyph_index;
        begin
            glyph_x=test_x-label_left;
            glyph_y=test_y-label_top;
            glyph_index=34-glyph_y*5-glyph_x;
            if(glyph_x>=0 && glyph_x<5 && glyph_y>=0 && glyph_y<7)
                label_pixel=glyph_bits[glyph_index];
            else
                label_pixel=1'b0;
        end
    endfunction

    localparam [34:0] GLYPH_X={5'b10001,5'b10001,5'b01010,5'b00100,
                               5'b01010,5'b10001,5'b10001};
    localparam [34:0] GLYPH_Y={5'b10001,5'b10001,5'b01010,5'b00100,
                               5'b00100,5'b00100,5'b00100};
    localparam [34:0] GLYPH_Z={5'b11111,5'b00001,5'b00010,5'b00100,
                               5'b01000,5'b10000,5'b11111};

    wire [20:0] line_hit;
    reg [2:0] label_pipe [0:3];
    wire cube_hit =
        line_hit[0] ||
        line_hit[1] ||
        line_hit[2] ||
        line_hit[3] ||
        line_hit[4] ||
        line_hit[5] ||
        line_hit[6] ||
        line_hit[7] ||
        line_hit[8] ||
        line_hit[9] ||
        line_hit[10] ||
        line_hit[11];
    wire signed [15:0] x_end_x=CENTER_X+(bx_x<<<1);
    wire signed [15:0] x_end_y=CENTER_Y+(bx_y<<<1);
    wire signed [15:0] y_end_x=CENTER_X+(by_x<<<1);
    wire signed [15:0] y_end_y=CENTER_Y+(by_y<<<1);
    wire signed [15:0] z_end_x=CENTER_X+(bz_x<<<1);
    wire signed [15:0] z_end_y=CENTER_Y+(bz_y<<<1);
    // Two short swept-back segments form an arrowhead at every axis end.
    wire signed [15:0] x_arrow1_x=x_end_x-(bx_x>>>1)-(bx_y>>>2);
    wire signed [15:0] x_arrow1_y=x_end_y-(bx_y>>>1)+(bx_x>>>2);
    wire signed [15:0] x_arrow2_x=x_end_x-(bx_x>>>1)+(bx_y>>>2);
    wire signed [15:0] x_arrow2_y=x_end_y-(bx_y>>>1)-(bx_x>>>2);
    wire signed [15:0] y_arrow1_x=y_end_x-(by_x>>>1)-(by_y>>>2);
    wire signed [15:0] y_arrow1_y=y_end_y-(by_y>>>1)+(by_x>>>2);
    wire signed [15:0] y_arrow2_x=y_end_x-(by_x>>>1)+(by_y>>>2);
    wire signed [15:0] y_arrow2_y=y_end_y-(by_y>>>1)-(by_x>>>2);
    wire signed [15:0] z_arrow1_x=z_end_x-(bz_x>>>1)-(bz_y>>>2);
    wire signed [15:0] z_arrow1_y=z_end_y-(bz_y>>>1)+(bz_x>>>2);
    wire signed [15:0] z_arrow2_x=z_end_x-(bz_x>>>1)+(bz_y>>>2);
    wire signed [15:0] z_arrow2_y=z_end_y-(bz_y>>>1)-(bz_x>>>2);
    wire signed [15:0] x_label_x=x_end_x+(bx_x[15]?-16'sd10:16'sd4);
    wire signed [15:0] x_label_y=x_end_y+(bx_y[15]?-16'sd10:16'sd4);
    wire signed [15:0] y_label_x=y_end_x+(by_x[15]?-16'sd10:16'sd4);
    wire signed [15:0] y_label_y=y_end_y+(by_y[15]?-16'sd10:16'sd4);
    wire signed [15:0] z_label_x=z_end_x+(bz_x[15]?-16'sd10:16'sd4);
    wire signed [15:0] z_label_y=z_end_y+(bz_y[15]?-16'sd10:16'sd4);
    wire x_axis_hit=line_hit[12] ||
                    line_hit[13] ||
                    line_hit[14] ||
                    label_pipe[3][0];
    wire y_axis_hit=line_hit[15] ||
                    line_hit[16] ||
                    line_hit[17] ||
                    label_pipe[3][1];
    wire z_axis_hit=line_hit[18] ||
                    line_hit[19] ||
                    line_hit[20] ||
                    label_pipe[3][2];
    wire pose_visible = ENABLE && menu_visible_pixel && pose_shown[96];


    // Pixel pipeline: geometry, differences, products, cross, hits, RGB.
    // Fixed total latency: 6 pixel clocks (reference latency: 1).
    reg signed [15:0] g_v0x;
    reg signed [15:0] g_v0y;
    reg signed [15:0] g_v1x;
    reg signed [15:0] g_v1y;
    reg signed [15:0] g_v2x;
    reg signed [15:0] g_v2y;
    reg signed [15:0] g_v3x;
    reg signed [15:0] g_v3y;
    reg signed [15:0] g_v4x;
    reg signed [15:0] g_v4y;
    reg signed [15:0] g_v5x;
    reg signed [15:0] g_v5y;
    reg signed [15:0] g_v6x;
    reg signed [15:0] g_v6y;
    reg signed [15:0] g_v7x;
    reg signed [15:0] g_v7y;
    reg signed [15:0] g_x_end_x;
    reg signed [15:0] g_x_end_y;
    reg signed [15:0] g_y_end_x;
    reg signed [15:0] g_y_end_y;
    reg signed [15:0] g_z_end_x;
    reg signed [15:0] g_z_end_y;
    reg signed [15:0] g_x_arrow1_x;
    reg signed [15:0] g_x_arrow1_y;
    reg signed [15:0] g_x_arrow2_x;
    reg signed [15:0] g_x_arrow2_y;
    reg signed [15:0] g_y_arrow1_x;
    reg signed [15:0] g_y_arrow1_y;
    reg signed [15:0] g_y_arrow2_x;
    reg signed [15:0] g_y_arrow2_y;
    reg signed [15:0] g_z_arrow1_x;
    reg signed [15:0] g_z_arrow1_y;
    reg signed [15:0] g_z_arrow2_x;
    reg signed [15:0] g_z_arrow2_y;
    reg signed [15:0] g_x_label_x;
    reg signed [15:0] g_x_label_y;
    reg signed [15:0] g_y_label_x;
    reg signed [15:0] g_y_label_y;
    reg signed [15:0] g_z_label_x;
    reg signed [15:0] g_z_label_y;
    reg signed [15:0] g_px;
    reg signed [15:0] g_py;
    reg [23:0] rgb_pipe [0:4];
    reg [3:0] ctl_pipe [0:4];
    integer pipe_i;
    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            g_v0x <= 16'sd0;
            g_v0y <= 16'sd0;
            g_v1x <= 16'sd0;
            g_v1y <= 16'sd0;
            g_v2x <= 16'sd0;
            g_v2y <= 16'sd0;
            g_v3x <= 16'sd0;
            g_v3y <= 16'sd0;
            g_v4x <= 16'sd0;
            g_v4y <= 16'sd0;
            g_v5x <= 16'sd0;
            g_v5y <= 16'sd0;
            g_v6x <= 16'sd0;
            g_v6y <= 16'sd0;
            g_v7x <= 16'sd0;
            g_v7y <= 16'sd0;
            g_x_end_x <= 16'sd0;
            g_x_end_y <= 16'sd0;
            g_y_end_x <= 16'sd0;
            g_y_end_y <= 16'sd0;
            g_z_end_x <= 16'sd0;
            g_z_end_y <= 16'sd0;
            g_x_arrow1_x <= 16'sd0;
            g_x_arrow1_y <= 16'sd0;
            g_x_arrow2_x <= 16'sd0;
            g_x_arrow2_y <= 16'sd0;
            g_y_arrow1_x <= 16'sd0;
            g_y_arrow1_y <= 16'sd0;
            g_y_arrow2_x <= 16'sd0;
            g_y_arrow2_y <= 16'sd0;
            g_z_arrow1_x <= 16'sd0;
            g_z_arrow1_y <= 16'sd0;
            g_z_arrow2_x <= 16'sd0;
            g_z_arrow2_y <= 16'sd0;
            g_x_label_x <= 16'sd0;
            g_x_label_y <= 16'sd0;
            g_y_label_x <= 16'sd0;
            g_y_label_y <= 16'sd0;
            g_z_label_x <= 16'sd0;
            g_z_label_y <= 16'sd0;
            g_px <= 16'sd0;
            g_py <= 16'sd0;
            for (pipe_i=0;pipe_i<5;pipe_i=pipe_i+1) begin rgb_pipe[pipe_i]<=0;ctl_pipe[pipe_i]<=0;end
        end else begin
            g_v0x <= v0x;
            g_v0y <= v0y;
            g_v1x <= v1x;
            g_v1y <= v1y;
            g_v2x <= v2x;
            g_v2y <= v2y;
            g_v3x <= v3x;
            g_v3y <= v3y;
            g_v4x <= v4x;
            g_v4y <= v4y;
            g_v5x <= v5x;
            g_v5y <= v5y;
            g_v6x <= v6x;
            g_v6y <= v6y;
            g_v7x <= v7x;
            g_v7y <= v7y;
            g_x_end_x <= x_end_x;
            g_x_end_y <= x_end_y;
            g_y_end_x <= y_end_x;
            g_y_end_y <= y_end_y;
            g_z_end_x <= z_end_x;
            g_z_end_y <= z_end_y;
            g_x_arrow1_x <= x_arrow1_x;
            g_x_arrow1_y <= x_arrow1_y;
            g_x_arrow2_x <= x_arrow2_x;
            g_x_arrow2_y <= x_arrow2_y;
            g_y_arrow1_x <= y_arrow1_x;
            g_y_arrow1_y <= y_arrow1_y;
            g_y_arrow2_x <= y_arrow2_x;
            g_y_arrow2_y <= y_arrow2_y;
            g_z_arrow1_x <= z_arrow1_x;
            g_z_arrow1_y <= z_arrow1_y;
            g_z_arrow2_x <= z_arrow2_x;
            g_z_arrow2_y <= z_arrow2_y;
            g_x_label_x <= x_label_x;
            g_x_label_y <= x_label_y;
            g_y_label_x <= y_label_x;
            g_y_label_y <= y_label_y;
            g_z_label_x <= z_label_x;
            g_z_label_y <= z_label_y;
            g_px <= px;
            g_py <= py;
            rgb_pipe[0]<=camera_rgb;
            ctl_pipe[0]<={pose_visible,camera_de,camera_hs,camera_vs};
            for (pipe_i=1;pipe_i<5;pipe_i=pipe_i+1) begin
                rgb_pipe[pipe_i]<=rgb_pipe[pipe_i-1];ctl_pipe[pipe_i]<=ctl_pipe[pipe_i-1];
            end
        end
    end
    testing60_pose_line_pipe u_line_0(pixel_clk,rst_n,g_px,g_py,g_v0x,g_v0y,g_v1x,g_v1y,3'd2,line_hit[0]);
    testing60_pose_line_pipe u_line_1(pixel_clk,rst_n,g_px,g_py,g_v1x,g_v1y,g_v2x,g_v2y,3'd2,line_hit[1]);
    testing60_pose_line_pipe u_line_2(pixel_clk,rst_n,g_px,g_py,g_v2x,g_v2y,g_v3x,g_v3y,3'd2,line_hit[2]);
    testing60_pose_line_pipe u_line_3(pixel_clk,rst_n,g_px,g_py,g_v3x,g_v3y,g_v0x,g_v0y,3'd2,line_hit[3]);
    testing60_pose_line_pipe u_line_4(pixel_clk,rst_n,g_px,g_py,g_v4x,g_v4y,g_v5x,g_v5y,3'd2,line_hit[4]);
    testing60_pose_line_pipe u_line_5(pixel_clk,rst_n,g_px,g_py,g_v5x,g_v5y,g_v6x,g_v6y,3'd2,line_hit[5]);
    testing60_pose_line_pipe u_line_6(pixel_clk,rst_n,g_px,g_py,g_v6x,g_v6y,g_v7x,g_v7y,3'd2,line_hit[6]);
    testing60_pose_line_pipe u_line_7(pixel_clk,rst_n,g_px,g_py,g_v7x,g_v7y,g_v4x,g_v4y,3'd2,line_hit[7]);
    testing60_pose_line_pipe u_line_8(pixel_clk,rst_n,g_px,g_py,g_v0x,g_v0y,g_v4x,g_v4y,3'd2,line_hit[8]);
    testing60_pose_line_pipe u_line_9(pixel_clk,rst_n,g_px,g_py,g_v1x,g_v1y,g_v5x,g_v5y,3'd2,line_hit[9]);
    testing60_pose_line_pipe u_line_10(pixel_clk,rst_n,g_px,g_py,g_v2x,g_v2y,g_v6x,g_v6y,3'd2,line_hit[10]);
    testing60_pose_line_pipe u_line_11(pixel_clk,rst_n,g_px,g_py,g_v3x,g_v3y,g_v7x,g_v7y,3'd2,line_hit[11]);
    testing60_pose_line_pipe u_line_12(pixel_clk,rst_n,g_px,g_py,CENTER_X,CENTER_Y,g_x_end_x,g_x_end_y,3'd3,line_hit[12]);
    testing60_pose_line_pipe u_line_13(pixel_clk,rst_n,g_px,g_py,g_x_end_x,g_x_end_y,g_x_arrow1_x,g_x_arrow1_y,3'd2,line_hit[13]);
    testing60_pose_line_pipe u_line_14(pixel_clk,rst_n,g_px,g_py,g_x_end_x,g_x_end_y,g_x_arrow2_x,g_x_arrow2_y,3'd2,line_hit[14]);
    testing60_pose_line_pipe u_line_15(pixel_clk,rst_n,g_px,g_py,CENTER_X,CENTER_Y,g_y_end_x,g_y_end_y,3'd3,line_hit[15]);
    testing60_pose_line_pipe u_line_16(pixel_clk,rst_n,g_px,g_py,g_y_end_x,g_y_end_y,g_y_arrow1_x,g_y_arrow1_y,3'd2,line_hit[16]);
    testing60_pose_line_pipe u_line_17(pixel_clk,rst_n,g_px,g_py,g_y_end_x,g_y_end_y,g_y_arrow2_x,g_y_arrow2_y,3'd2,line_hit[17]);
    testing60_pose_line_pipe u_line_18(pixel_clk,rst_n,g_px,g_py,CENTER_X,CENTER_Y,g_z_end_x,g_z_end_y,3'd3,line_hit[18]);
    testing60_pose_line_pipe u_line_19(pixel_clk,rst_n,g_px,g_py,g_z_end_x,g_z_end_y,g_z_arrow1_x,g_z_arrow1_y,3'd2,line_hit[19]);
    testing60_pose_line_pipe u_line_20(pixel_clk,rst_n,g_px,g_py,g_z_end_x,g_z_end_y,g_z_arrow2_x,g_z_arrow2_y,3'd2,line_hit[20]);
    integer label_i;
    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            for (label_i=0;label_i<4;label_i=label_i+1) label_pipe[label_i]<=0;
        end else begin
            label_pipe[0]<={label_pixel(g_px,g_py,g_z_label_x,g_z_label_y,GLYPH_Z),label_pixel(g_px,g_py,g_y_label_x,g_y_label_y,GLYPH_Y),label_pixel(g_px,g_py,g_x_label_x,g_x_label_y,GLYPH_X)};
            for (label_i=1;label_i<4;label_i=label_i+1) label_pipe[label_i]<=label_pipe[label_i-1];
        end
    end

    // 75% camera + 25% lavender gives a colored yet see-through wireframe.
    wire [9:0] cube_mix_r={2'b0,rgb_pipe[4][23:16]}+
                          ({2'b0,rgb_pipe[4][23:16]}<<1)+10'd190;
    wire [9:0] cube_mix_g={2'b0,rgb_pipe[4][15:8]}+
                          ({2'b0,rgb_pipe[4][15:8]}<<1)+10'd150;
    wire [9:0] cube_mix_b={2'b0,rgb_pipe[4][7:0]}+
                          ({2'b0,rgb_pipe[4][7:0]}<<1)+10'd255;
    wire [23:0] cube_blend={cube_mix_r[9:2],cube_mix_g[9:2],cube_mix_b[9:2]};

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            video_rgb<=24'd0; video_de<=1'b0;
            video_hs<=1'b0; video_vs<=1'b0;
        end else begin
            video_de<=ctl_pipe[4][2]; video_hs<=ctl_pipe[4][1]; video_vs<=ctl_pipe[4][0];
            if (!ctl_pipe[4][2]) video_rgb<=24'd0;
            else if (ctl_pipe[4][3] && x_axis_hit) video_rgb<=24'h42f56c;
            else if (ctl_pipe[4][3] && y_axis_hit) video_rgb<=24'hffe45c;
            else if (ctl_pipe[4][3] && z_axis_hit) video_rgb<=24'h45a3ff;
            else if (ctl_pipe[4][3] && cube_hit) video_rgb<=cube_blend;
            else video_rgb<=rgb_pipe[4];
        end
    end
endmodule

// All arithmetic widths and signed comparisons reproduce line_pixel exactly.
// Differences are 17 bits, products and their difference wrap at 34 bits.
module testing60_pose_line_pipe(
    input wire clk,rst_n,
    input wire signed [15:0] test_x,test_y,x0,y0,x1,y1,
    input wire [2:0] thickness,
    output reg hit
);
    wire signed [16:0] dx_w=$signed(x1)-$signed(x0);
    wire signed [16:0] dy_w=$signed(y1)-$signed(y0);
    wire signed [16:0] min_x=(x0<x1)?x0:x1,max_x=(x0>x1)?x0:x1;
    wire signed [16:0] min_y=(y0<y1)?y0:y1,max_y=(y0>y1)?y0:y1;
    wire signed [16:0] margin={14'd0,thickness};
    reg signed [16:0] dx,dy,rx,ry;
    reg [2:0] thickness_d;
    reg [2:0] box_pipe;
    wire [16:0] dx_abs=dx[16]?-dx:dx,dy_abs=dy[16]?-dy:dy;
    reg signed [33:0] prod_a,prod_b,cross_value;
    reg [20:0] tolerance,tolerance_d;
    wire [33:0] cross_abs=cross_value[33]?-cross_value:cross_value;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            dx<=0;dy<=0;rx<=0;ry<=0;thickness_d<=0;box_pipe<=0;
            prod_a<=0;prod_b<=0;cross_value<=0;tolerance<=0;tolerance_d<=0;hit<=0;
        end else begin
            dx<=dx_w;dy<=dy_w;rx<=$signed(test_x)-$signed(x0);ry<=$signed(test_y)-$signed(y0);
            thickness_d<=thickness;
            box_pipe[0]<=(test_x>=min_x-margin && test_x<=max_x+margin &&
                          test_y>=min_y-margin && test_y<=max_y+margin);
            box_pipe[2:1]<=box_pipe[1:0];
            prod_a<=rx*dy;prod_b<=ry*dx;
            tolerance<=(dx_abs+dy_abs+1'b1)*thickness_d;
            tolerance_d<=tolerance;
            cross_value<=prod_a-prod_b;
            hit<=box_pipe[2] && cross_abs<=tolerance_d;
        end
    end
endmodule
