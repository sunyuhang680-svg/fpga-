`timescale 1ns / 1ps
// ============================================================================
// claude merged_v1：直通视频 + 实时去畸变（无黑边）
//
// direct_camera_hdmi_ud = direct_camera_hdmi（direct_video.v）的拷贝，只把输出级
// direct_video_out（8 行 RGB888 环）换成 direct_video_out_ud：
//   - 2^RING_LOG2 行 RGB666 环（四个 bank：行奇偶 x 列奇偶，一拍读出 2x2 邻域）
//   - 每个输出像素按 Brown 模型（live_undistort_stream_ud）算源坐标，双线性插值
//   - HDMI 第 0 行比相机第 0 行晚 LOCK_LINES 行开始，保证需要的下方源行已写完；
//     帧锚点先延迟 LOCK_LINES-3 行再按原逻辑对齐（对齐仍发生在场消隐里）
// 其余（白平衡、去马赛克、颜色处理、行长固定 H_TOTAL 的跟随时序）完全沿用原代码。
// ============================================================================

module direct_camera_hdmi_ud #(
    parameter WIDTH         = 1280,
    parameter HEIGHT        = 720,
    parameter SKIP_FRAMES   = 2,     // 配置完成后丢掉的帧数
    parameter RING_LOG2     = 7,     // 去畸变行环 2^RING_LOG2 行（RGB666）
    parameter LOCK_LINES    = 48,    // HDMI 第 0 行比相机第 0 行开始晚的行数（要大于去畸变需要的下方行数）
    parameter AWB_MIN_GRAY  = 4096,  // 灰点少于这么多时白平衡退回灰度世界
    parameter AWB_DEF_R     = 259,   // 上电白平衡增益（256 = 1.0）
    parameter AWB_DEF_B     = 210,
    parameter H_TOTAL = 1650, parameter H_SYNC = 40, parameter H_BP = 220, parameter H_ACT = 1280,
    parameter V_TOTAL = 750,  parameter V_SYNC = 5,  parameter V_BP = 20,  parameter V_ACT = 720
)(
    input  wire         rst_n,          // 异步复位，低有效
    // 相机 DVP（直接接引脚）
    input  wire         cam_pclk,
    input  wire [7:0]   cam_data,
    input  wire         cam_href,
    input  wire         cam_vsync,
    input  wire         cam_cfg_done,   // 摄像头寄存器配置完成（任意时钟域）
    // 运行时设置（任意时钟域，下一帧生效）
    input  wire [1:0]   bayer_sel,      // 0 BGGR 1 GBRG 2 GRBG 3 RGGB
    input  wire         awb_en,         // 1：FPGA 自动白平衡；0：保持当前增益
    input  wire         edge_sel,       // DVP 采样沿：1 上升沿（默认）  0 下降沿（原工程做法）
    input  wire         awb_onepush,    // 翻转一次 = 一键白平衡（画面中央当白色）
    // 颜色处理（任意时钟域；ccm_tog 翻转表示 ccm_coef 已更新并保持稳定）
    input  wire         ccm_en,
    input  wire [107:0] ccm_coef,
    input  wire         ccm_tog,
    input  wire         lut_clk,
    input  wire         lut_we,
    input  wire [2:0]   lut_ch,
    input  wire [7:0]   lut_addr,
    input  wire [7:0]   lut_data,
    // HDMI 像素时钟域
    input  wire         pclk,
    input  wire         ud_bypass,      // 1：不去畸变（原图），帧头锁存；任意时钟域，准静态
    input  wire [383:0] ud_coeff,       // live_calibration_constants 格式
    output wire [15:0]  ud_miss_gray,   // 去畸变缺行像素计数（格雷码，诊断）
    output wire         hs,
    output wire         vs,
    output wire         de,
    output wire [23:0]  rgb,
    // 状态
    output wire         locked,
    output wire         cam_present,
    output wire         hdmi_frame_toggle,
    output wire         cam_frame_toggle,
    output wire [10:0]  meas_width,
    output wire [10:0]  meas_height,
    output wire [15:0]  adj_gray,
    output wire [15:0]  uf_gray,
    output wire [15:0]  of_gray,
    // 白平衡增益（相机时钟域，256 = 1.0；gain_toggle 翻转表示刚更新）
    output wire [11:0]  gain_r,
    output wire [11:0]  gain_b,
    output wire         gain_toggle,
    // 抓图用的观测点（相机时钟域）
    output wire [7:0]   tap_raw,        // 白平衡之前的 RAW
    output wire         tap_href,
    output wire         tap_vsync,
    output wire         tap_dm_valid,   // 去马赛克输出（颜色处理之前）
    output wire [10:0]  tap_dm_x,
    output wire [10:0]  tap_dm_y,
    output wire [23:0]  tap_dm_rgb,
    output wire         tap_cp_valid,   // 颜色处理之后
    output wire [10:0]  tap_cp_x,
    output wire [10:0]  tap_cp_y,
    output wire [23:0]  tap_cp_rgb
);
    // 复位同步释放
    reg [1:0] cam_rst_sync, pix_rst_sync;
    always @(posedge cam_pclk or negedge rst_n)
        if (!rst_n) cam_rst_sync <= 2'b00; else cam_rst_sync <= {cam_rst_sync[0], 1'b1};
    always @(posedge pclk or negedge rst_n)
        if (!rst_n) pix_rst_sync <= 2'b00; else pix_rst_sync <= {pix_rst_sync[0], 1'b1};
    wire cam_rst_n = cam_rst_sync[1];
    wire pix_rst_n = pix_rst_sync[1];

    // DVP 采样：下降沿和上升沿各采一份，按 edge_c 选用。
    // 10-28 上板抓图发现下降沿采样时数据位大量出错（像相邻像素按位与），
    // 说明 OV5640 在下降沿附近切换数据，所以默认改用上升沿。
    reg [7:0] data_n, data_r, data_p;
    reg       href_n, href_r, href_p, vsync_n, vsync_r, vsync_p;
    reg       edge_c;
    always @(negedge cam_pclk) begin
        data_n  <= cam_data;
        href_n  <= cam_href;
        vsync_n <= cam_vsync;
    end
    always @(posedge cam_pclk) begin
        data_r  <= cam_data;
        href_r  <= cam_href;
        vsync_r <= cam_vsync;
    end
    always @(posedge cam_pclk or negedge cam_rst_n) begin
        if (!cam_rst_n) begin
            data_p <= 8'd0; href_p <= 1'b0; vsync_p <= 1'b0;
        end else if (edge_c) begin
            data_p <= data_r; href_p <= href_r; vsync_p <= vsync_r;
        end else begin
            data_p <= data_n; href_p <= href_n; vsync_p <= vsync_n;
        end
    end

    (* ASYNC_REG = "TRUE" *) reg [1:0] cfg_sync;
    always @(posedge cam_pclk or negedge cam_rst_n)
        if (!cam_rst_n) cfg_sync <= 2'b00; else cfg_sync <= {cfg_sync[0], cam_cfg_done};

    // 运行时设置同步到相机时钟（连续两拍相同才采用）
    (* ASYNC_REG = "TRUE" *) reg [5:0] set_s1, set_s2;
    reg [5:0] set_s3;
    reg [1:0] bayer_c;
    reg       awb_c, ccm_en_c, push_c;
    always @(posedge cam_pclk or negedge cam_rst_n) begin
        if (!cam_rst_n) begin
            set_s1 <= 6'd0; set_s2 <= 6'd0; set_s3 <= 6'd0; bayer_c <= 2'd0; awb_c <= 1'b0; push_c <= 1'b0;
            edge_c <= 1'b1; ccm_en_c <= 1'b0;
        end else begin
            set_s1 <= {awb_onepush, ccm_en, edge_sel, awb_en, bayer_sel};
            set_s2 <= set_s1;
            set_s3 <= set_s2;
            if (set_s2 == set_s3) begin
                bayer_c  <= set_s3[1:0];
                awb_c    <= set_s3[2];
                edge_c   <= set_s3[3];
                ccm_en_c <= set_s3[4];
                push_c   <= set_s3[5];
            end
        end
    end

    // 颜色矩阵系数：ccm_tog 同步后再取（系数早已稳定）
    (* ASYNC_REG = "TRUE" *) reg [2:0] ccm_tog_s;
    reg [107:0] ccm_c;
    always @(posedge cam_pclk or negedge cam_rst_n) begin
        if (!cam_rst_n) begin
            ccm_tog_s <= 3'd0;
            ccm_c     <= {12'd256, 12'd0, 12'd0, 12'd0, 12'd256, 12'd0, 12'd0, 12'd0, 12'd256};
        end else begin
            ccm_tog_s <= {ccm_tog_s[1:0], ccm_tog};
            if (ccm_tog_s[2] ^ ccm_tog_s[1]) ccm_c <= ccm_coef;
        end
    end

    wire        dm_valid;
    wire [10:0] dm_x, dm_y;
    wire [23:0] dm_rgb;

    // 白平衡（RAW 域，按像素颜色乘增益；灰点统计用去马赛克输出）
    wire [7:0] wb_raw;
    wire       wb_href, wb_vsync;
    wire [1:0] frame_pattern;
    bayer_awb #(.WIDTH(WIDTH), .HEIGHT(HEIGHT), .MIN_GRAY(AWB_MIN_GRAY),
                .DEF_GAIN_R(AWB_DEF_R), .DEF_GAIN_B(AWB_DEF_B)) u_awb (
        .clk(cam_pclk), .rst_n(cam_rst_n),
        .pattern_in(bayer_c), .awb_en_in(awb_c), .onepush_tog(push_c),
        .vsync(vsync_p), .href(href_p), .raw(data_p),
        .st_valid(dm_valid), .st_x(dm_x), .st_y(dm_y), .st_rgb(dm_rgb),
        .vsync_o(wb_vsync), .href_o(wb_href), .raw_o(wb_raw),
        .pattern(frame_pattern),
        .gain_r(gain_r), .gain_b(gain_b), .gain_toggle(gain_toggle)
    );

    bayer_demosaic_stream #(
        .WIDTH(WIDTH), .HEIGHT(HEIGHT), .SKIP_FRAMES(SKIP_FRAMES)
    ) u_demosaic (
        .clk(cam_pclk), .rst_n(cam_rst_n), .enable(cfg_sync[1]),
        .pattern(frame_pattern),
        .vsync(wb_vsync), .href(wb_href), .raw(wb_raw),
        .out_valid(dm_valid), .out_x(dm_x), .out_y(dm_y), .out_rgb(dm_rgb),
        .meas_width(meas_width), .meas_height(meas_height),
        .frame_toggle(cam_frame_toggle)
    );

    // 颜色矩阵 + gamma
    wire        cp_valid;
    wire [10:0] cp_x, cp_y;
    wire [23:0] cp_rgb;
    color_proc u_color (
        .clk(cam_pclk), .rst_n(cam_rst_n), .enable(ccm_en_c), .coef(ccm_c),
        .in_valid(dm_valid), .in_x(dm_x), .in_y(dm_y), .in_rgb(dm_rgb),
        .out_valid(cp_valid), .out_x(cp_x), .out_y(cp_y), .out_rgb(cp_rgb),
        .lut_clk(lut_clk), .lut_we(lut_we), .lut_ch(lut_ch),
        .lut_addr(lut_addr), .lut_data(lut_data)
    );

    assign tap_raw      = data_p;
    assign tap_href     = href_p;
    assign tap_vsync    = vsync_p;
    assign tap_dm_valid = dm_valid;
    assign tap_dm_x     = dm_x;
    assign tap_dm_y     = dm_y;
    assign tap_dm_rgb   = dm_rgb;
    assign tap_cp_valid = cp_valid;
    assign tap_cp_x     = cp_x;
    assign tap_cp_y     = cp_y;
    assign tap_cp_rgb   = cp_rgb;

    direct_video_out_ud #(
        .WIDTH(WIDTH), .HEIGHT(HEIGHT), .RING_LOG2(RING_LOG2), .LOCK_LINES(LOCK_LINES),
        .H_TOTAL(H_TOTAL), .H_SYNC(H_SYNC), .H_BP(H_BP), .H_ACT(H_ACT),
        .V_TOTAL(V_TOTAL), .V_SYNC(V_SYNC), .V_BP(V_BP), .V_ACT(V_ACT)
    ) u_out (
        .wr_clk(cam_pclk), .wr_rst_n(cam_rst_n),
        .wr_valid(cp_valid), .wr_x(cp_x), .wr_y(cp_y), .wr_rgb(cp_rgb),
        .pclk(pclk), .rst_n(pix_rst_n),
        .ud_bypass(ud_bypass), .ud_coeff(ud_coeff), .ud_miss_gray(ud_miss_gray),
        .hs(hs), .vs(vs), .de(de), .rgb(rgb),
        .locked(locked), .cam_present(cam_present), .frame_toggle(hdmi_frame_toggle),
        .adj_gray(adj_gray), .uf_gray(uf_gray), .of_gray(of_gray)
    );
endmodule

// ----------------------------------------------------------------------------
// 去畸变行环 + 跟随相机的 HDMI 时序（由 direct_video_out 改写）
// ----------------------------------------------------------------------------
module direct_video_out_ud #(
    parameter WIDTH       = 1280,
    parameter HEIGHT      = 720,
    parameter RING_LOG2   = 7,
    parameter LOCK_LINES  = 48,
    parameter LOST_FRAMES = 3,
    parameter MAX_WAIT_LINES = 64,
    parameter H_TOTAL = 1650, parameter H_SYNC = 40, parameter H_BP = 220, parameter H_ACT = 1280,
    parameter V_TOTAL = 750,  parameter V_SYNC = 5,  parameter V_BP = 20,  parameter V_ACT = 720,
    parameter [23:0] NO_SIGNAL_RGB = 24'h000060
)(
    input  wire        wr_clk,
    input  wire        wr_rst_n,
    input  wire        wr_valid,
    input  wire [10:0] wr_x,
    input  wire [10:0] wr_y,
    input  wire [23:0] wr_rgb,
    input  wire        pclk,
    input  wire        rst_n,
    input  wire        ud_bypass,
    input  wire [383:0] ud_coeff,
    output reg  [15:0] ud_miss_gray,
    output reg         hs,
    output reg         vs,
    output reg         de,
    output reg  [23:0] rgb,
    output reg         locked,
    output reg         cam_present,
    output reg         frame_toggle,
    output reg  [15:0] adj_gray,
    output reg  [15:0] uf_gray,
    output reg  [15:0] of_gray
);
    function [15:0] gray2bin;
        input [15:0] gv;
        integer k;
        begin
            gray2bin[15] = gv[15];
            for (k = 14; k >= 0; k = k - 1)
                gray2bin[k] = gray2bin[k+1] ^ gv[k];
        end
    endfunction

    localparam HALF_W    = WIDTH / 2;
    localparam BANK_ROWS = (1 << RING_LOG2) / 2;
    localparam BANK_DEPTH= BANK_ROWS * HALF_W;
    localparam BANK_AW   = $clog2(BANK_DEPTH);
    localparam HS0    = H_SYNC + H_BP;
    localparam VS0    = V_SYNC + V_BP;
    localparam BASE_LOCK = 3;                      // 原设计的对齐提前量（在场消隐内）
    localparam V_LOCK = VS0 - BASE_LOCK;
    localparam ANCHOR_DELAY = LOCK_LINES - BASE_LOCK;   // 锚点延迟的行数

    // ---------------- 写侧（相机时钟）：RGB666 四 bank 行环 ----------------
    reg [15:0] rows_total, rows_gray, anchor_base;
    reg        anchor_tog;
    wire [RING_LOG2-1:0] slot = rows_total[RING_LOG2-1:0];
    wire [BANK_AW-1:0] wr_addr = slot[RING_LOG2-1:1] * HALF_W + wr_x[10:1];
    wire [17:0] wr_px = {wr_rgb[23:18], wr_rgb[15:10], wr_rgb[7:2]};
    wire we = wr_valid && (wr_x < WIDTH);
    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            rows_total <= 16'd0; rows_gray <= 16'd0;
            anchor_base <= 16'd0; anchor_tog <= 1'b0;
        end else begin
            if (wr_valid && wr_x == WIDTH - 1)
                rows_total <= rows_total + 16'd1;
            rows_gray <= rows_total ^ (rows_total >> 1);
            if (wr_valid && wr_x == 11'd0 && wr_y == 11'd0) begin
                anchor_tog  <= ~anchor_tog;
                anchor_base <= rows_total;
            end
        end
    end

    wire [BANK_AW-1:0] ra00, ra01, ra10, ra11;
    wire [17:0] rd00, rd01, rd10, rd11;
    ud_ring_bank #(.AW(BANK_AW), .DEPTH(BANK_DEPTH)) b00(.wclk(wr_clk), .we(we && !slot[0] && !wr_x[0]), .waddr(wr_addr), .wdata(wr_px), .rclk(pclk), .raddr(ra00), .rdata(rd00));
    ud_ring_bank #(.AW(BANK_AW), .DEPTH(BANK_DEPTH)) b01(.wclk(wr_clk), .we(we && !slot[0] &&  wr_x[0]), .waddr(wr_addr), .wdata(wr_px), .rclk(pclk), .raddr(ra01), .rdata(rd01));
    ud_ring_bank #(.AW(BANK_AW), .DEPTH(BANK_DEPTH)) b10(.wclk(wr_clk), .we(we &&  slot[0] && !wr_x[0]), .waddr(wr_addr), .wdata(wr_px), .rclk(pclk), .raddr(ra10), .rdata(rd10));
    ud_ring_bank #(.AW(BANK_AW), .DEPTH(BANK_DEPTH)) b11(.wclk(wr_clk), .we(we &&  slot[0] &&  wr_x[0]), .waddr(wr_addr), .wdata(wr_px), .rclk(pclk), .raddr(ra11), .rdata(rd11));

    // ---------------- 跨到像素时钟 ----------------
    (* ASYNC_REG = "TRUE" *) reg [2:0]  anc_s;
    (* ASYNC_REG = "TRUE" *) reg [15:0] rg_s1, rg_s2;
    (* ASYNC_REG = "TRUE" *) reg [1:0]  byp_s;
    reg [15:0] base_s;
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            anc_s <= 3'd0; rg_s1 <= 16'd0; rg_s2 <= 16'd0; base_s <= 16'd0; byp_s <= 2'b11;
        end else begin
            anc_s  <= {anc_s[1:0], anchor_tog};
            rg_s1  <= rows_gray;
            rg_s2  <= rg_s1;
            base_s <= anchor_base;   // 锚点延迟 ANCHOR_DELAY 行后才取用，早已稳定
            byp_s  <= {byp_s[0], ud_bypass};
        end
    end
    wire        anchor_edge = anc_s[2] ^ anc_s[1];
    wire [15:0] rows_now    = gray2bin(rg_s2);

    // ---------------- 时序发生器 ----------------
    reg [11:0] h_cnt, v_cnt;
    reg        anchor_pend;
    reg        dly_active;
    reg [7:0]  dly_lines;
    reg [15:0] frame_base;
    reg [2:0]  lost_cnt;
    reg [1:0]  good_cnt;
    reg [7:0]  wait_cnt;
    reg [15:0] adj_cnt;

    wire        line_end = (h_cnt == H_TOTAL - 1);
    wire [11:0] v_nat    = (v_cnt == V_TOTAL - 1) ? 12'd0 : v_cnt + 12'd1;
    // 延迟后的锚点：相机第 0 行开始后再过 ANCHOR_DELAY 个 HDMI 行
    wire        dly_fire = line_end && dly_active && (dly_lines <= 8'd1);
    wire        pend_now = anchor_pend | dly_fire;
    wire signed [13:0] v_err = $signed({2'b00, v_nat}) - $signed(V_LOCK);

    wire        h_act  = (h_cnt >= HS0) && (h_cnt < HS0 + H_ACT);
    wire        v_act  = (v_cnt >= VS0) && (v_cnt < VS0 + V_ACT);
    wire [11:0] x_act  = h_cnt - HS0;
    wire [11:0] y_act  = v_cnt - VS0;

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            h_cnt <= 12'd0; v_cnt <= 12'd0;
            anchor_pend <= 1'b0; frame_base <= 16'd0;
            dly_active <= 1'b0; dly_lines <= 8'd0;
            lost_cnt <= 3'd7; good_cnt <= 2'd0; wait_cnt <= 8'd0;
            adj_cnt <= 16'd0;
            locked <= 1'b0; cam_present <= 1'b0; frame_toggle <= 1'b0;
            adj_gray <= 16'd0;
        end else begin
            if (anchor_edge) begin
                dly_active <= 1'b1;
                dly_lines  <= ANCHOR_DELAY;
            end else if (line_end && dly_active) begin
                if (dly_lines <= 8'd1) dly_active <= 1'b0;
                dly_lines <= dly_lines - 8'd1;
            end
            if (dly_fire && !line_end) anchor_pend <= 1'b1;

            if (line_end) begin
                h_cnt <= 12'd0;
                if (pend_now) begin
                    anchor_pend <= 1'b0;
                    v_cnt       <= V_LOCK;
                    frame_base  <= base_s;
                    lost_cnt    <= 3'd0;
                    wait_cnt    <= 8'd0;
                    if (v_nat != V_LOCK || wait_cnt != 8'd0) adj_cnt <= adj_cnt + 16'd1;
                    if (v_err >= -2 && v_err <= 2 && wait_cnt <= 8'd2) begin
                        if (good_cnt != 2'd3) good_cnt <= good_cnt + 2'd1;
                    end else
                        good_cnt <= 2'd0;
                end else if (v_nat == V_LOCK && cam_present && wait_cnt < MAX_WAIT_LINES) begin
                    wait_cnt <= wait_cnt + 8'd1;
                end else begin
                    wait_cnt <= 8'd0;
                    v_cnt <= v_nat;
                    if (v_nat == 12'd0) begin
                        frame_toggle <= ~frame_toggle;
                        if (lost_cnt != 3'd7) lost_cnt <= lost_cnt + 3'd1;
                    end
                end
            end else begin
                h_cnt <= h_cnt + 12'd1;
            end

            cam_present <= (lost_cnt < LOST_FRAMES);
            locked      <= (lost_cnt < LOST_FRAMES) && (good_cnt >= 2'd2);
            adj_gray <= adj_cnt ^ (adj_cnt >> 1);
        end
    end

    // ---------------- 去畸变读出 ----------------
    // 帧参数（行基址、是否去畸变）在有效行开始前一行锁存：此时本帧锚点已生效。
    reg ud_frame_start;
    reg hs_a, vs_a, de_a, req_v;
    reg [11:0] req_x, req_y;
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            ud_frame_start <= 1'b0; hs_a <= 1'b0; vs_a <= 1'b0; de_a <= 1'b0; req_v <= 1'b0;
            req_x <= 12'd0; req_y <= 12'd0;
        end else begin
            ud_frame_start <= (h_cnt == 12'd0) && (v_cnt == VS0 - 1);
            hs_a  <= (h_cnt < H_SYNC);
            vs_a  <= (v_cnt < V_SYNC);
            de_a  <= h_act && v_act;
            req_v <= h_act && v_act;
            req_x <= x_act;
            req_y <= y_act;
        end
    end
    wire [2:0]  ud_tag;
    wire [17:0] ud_px;
    wire        ud_miss;
    live_undistort_stream_ud #(.WIDTH(WIDTH), .HEIGHT(HEIGHT), .RING_LOG2(RING_LOG2), .TAG_W(3), .DW(18)) u_stream (
        .clk(pclk), .rst_n(rst_n), .frame_start(ud_frame_start), .bypass_in(byp_s[1]),
        .coeff_in(ud_coeff), .base_in(frame_base), .rows(rows_now),
        .req_valid(req_v), .req_x(req_x), .req_y(req_y), .req_tag({hs_a, vs_a, de_a}),
        .addr00(ra00), .addr01(ra01), .addr10(ra10), .addr11(ra11),
        .ram_00(rd00), .ram_01(rd01), .ram_10(rd10), .ram_11(rd11),
        .out_valid(), .out_pixel(ud_px), .out_tag(ud_tag), .row_miss(ud_miss),
        .dbg_valid(), .dbg_us(), .dbg_vs());

    reg [15:0] miss_cnt;
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            hs <= 1'b0; vs <= 1'b0; de <= 1'b0; rgb <= 24'd0;
            miss_cnt <= 16'd0; ud_miss_gray <= 16'd0; uf_gray <= 16'd0; of_gray <= 16'd0;
        end else begin
            hs  <= ud_tag[2];
            vs  <= ud_tag[1];
            de  <= ud_tag[0];
            rgb <= !ud_tag[0] ? 24'd0 :
                   !cam_present ? NO_SIGNAL_RGB :
                   {ud_px[17:12], ud_px[17:16], ud_px[11:6], ud_px[11:10], ud_px[5:0], ud_px[5:4]};
            if (ud_miss) miss_cnt <= miss_cnt + 16'd1;
            ud_miss_gray <= miss_cnt ^ (miss_cnt >> 1);
            uf_gray      <= miss_cnt ^ (miss_cnt >> 1);
            of_gray      <= 16'd0;
        end
    end
endmodule

// 双口 RAM：写读独立时钟，读出寄存一拍
module ud_ring_bank #(parameter integer AW=16, parameter integer DEPTH=40960)(
    input wire wclk, we, input wire [AW-1:0] waddr, input wire [17:0] wdata,
    input wire rclk, input wire [AW-1:0] raddr, output reg [17:0] rdata
);
    reg [17:0] mem [0:DEPTH-1];
    always @(posedge wclk) if (we) mem[waddr] <= wdata;
    always @(posedge rclk) rdata <= mem[raddr];
endmodule
