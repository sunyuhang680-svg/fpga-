`timescale 1ns / 1ps
// ============================================================================
// 无 DDR 直通：OV5640 RAW8 → 白平衡 → 去马赛克 → 颜色处理 → 8 行环形缓存 → HDMI
//
//   direct_camera_hdmi      顶层封装（相机输入采样、复位同步）
//   bayer_awb               RAW 域自动白平衡（灰度世界）
//   bayer_demosaic_stream   RAW8 Bayer 双线性去马赛克（4 行原始缓存）
//   color_proc              3x3 颜色矩阵 + gamma 查找表
//   direct_video_out        8 行 RGB888 环形缓存 + 跟随相机帧头的 HDMI 时序
//   video_status_uart       每秒从串口打印帧率/像素时钟/锁定状态/白平衡增益
//   video_ctrl_uart         串口命令：Bayer 排列、白平衡、采样沿、颜色处理、抓图
//   video_snapshot          串口抓图（缩略图）
//
// HDMI 时序每帧在相机第 0 行开始处理时重新对齐（只在行边界调整行号，
// 每一行长度保持 H_TOTAL 不变），所以不需要整帧缓存，延迟只有几行。
// ============================================================================

// ----------------------------------------------------------------------------
// 顶层封装
// ----------------------------------------------------------------------------
module direct_camera_hdmi #(
    parameter WIDTH         = 1280,
    parameter HEIGHT        = 720,
    parameter SKIP_FRAMES   = 2,     // 配置完成后丢掉的帧数
    parameter SLOT_BITS     = 3,     // 环形缓存 2^SLOT_BITS 行
    parameter LOCK_LINES    = 3,     // HDMI 第 0 行比相机第 0 行处理开始晚的行数
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

    direct_video_out #(
        .WIDTH(WIDTH), .HEIGHT(HEIGHT), .SLOT_BITS(SLOT_BITS), .LOCK_LINES(LOCK_LINES),
        .H_TOTAL(H_TOTAL), .H_SYNC(H_SYNC), .H_BP(H_BP), .H_ACT(H_ACT),
        .V_TOTAL(V_TOTAL), .V_SYNC(V_SYNC), .V_BP(V_BP), .V_ACT(V_ACT)
    ) u_out (
        .wr_clk(cam_pclk), .wr_rst_n(cam_rst_n),
        .wr_valid(cp_valid), .wr_x(cp_x), .wr_y(cp_y), .wr_rgb(cp_rgb),
        .pclk(pclk), .rst_n(pix_rst_n),
        .hs(hs), .vs(vs), .de(de), .rgb(rgb),
        .locked(locked), .cam_present(cam_present), .frame_toggle(hdmi_frame_toggle),
        .adj_gray(adj_gray), .uf_gray(uf_gray), .of_gray(of_gray)
    );
endmodule

// ----------------------------------------------------------------------------
// RAW 域自动白平衡（灰点优先，灰度世界兜底）
//
// 灰点：在去马赛克输出（已乘当前增益）上，只统计接近灰色的像素
//   max(R,G,B) - min(R,G,B) <= G*3/8，且 G >= 24、max < 235。
//   修正量 = G和/R和、G和/B和，新增益 = 当前增益 × 修正量（闭环迭代）。
//   大块红色等饱和颜色不参与，不会把白色带偏（10-28 上板：红盒子占半个画面时
//   灰度世界把红色压到 0.74，白墙发青）。
// 兜底：灰点少于 MIN_GRAY 个时，用 RAW 三种像素的均值做灰度世界。
// 场消隐期间用一个串行除法器算，每帧向目标走 1/4，下一帧生效。
// 像素通路：raw * gain >> 8，超过 255 饱和。
//
// 工作方式（10-28 上板：OV5640 RAW 输出已带传感器自己的白平衡，但白色偏蓝 B/G≈1.22）：
//   awb_en_in=0：增益保持不变。上电为 DEF_GAIN_R/B（修正传感器的固定偏色）
//   awb_en_in=1：自动（灰点优先、灰度世界兜底）
//   onepush_tog 翻转：一键白平衡——下一次计算用画面中央 1/2 x 1/2 区域当白色，
//                     一步算出增益并保持（之后按 awb_en_in 决定是否继续自动）
// ----------------------------------------------------------------------------
module bayer_awb #(
    parameter WIDTH  = 1280,
    parameter HEIGHT = 720,
    parameter [7:0]  SAT_LEVEL = 8'd240,     // 统计时忽略 ≥ 此值的像素
    parameter [11:0] GAIN_MIN  = 12'd128,    // 0.5
    parameter [11:0] GAIN_MAX  = 12'd1023,   // 约 4.0
    parameter [15:0] MIN_AVG_G = 16'd1024,   // G 平均值低于 4.0（Q8.8）时太暗，不更新
    parameter [19:0] MIN_GRAY  = 20'd4096,   // 灰点少于这么多时退回灰度世界
    parameter [11:0] DEF_GAIN_R = 12'd259,   // 上电增益：按 10-28 白墙标定（R/G 0.99、B/G 1.22）
    parameter [11:0] DEF_GAIN_B = 12'd210
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [1:0]  pattern_in,   // 下一帧使用的 Bayer 排列
    input  wire        awb_en_in,
    input  wire        onepush_tog,  // 翻转一次 = 一键白平衡
    input  wire        vsync,
    input  wire        href,
    input  wire [7:0]  raw,
    input  wire        st_valid,     // 去马赛克输出（统计灰点、一键白平衡用）
    input  wire [10:0] st_x,
    input  wire [10:0] st_y,
    input  wire [23:0] st_rgb,
    output reg         vsync_o,      // 比输入晚 2 拍
    output reg         href_o,
    output reg  [7:0]  raw_o,
    output reg  [1:0]  pattern,      // 本帧的 Bayer 排列（场同步上升沿锁存）
    output reg  [11:0] gain_r,       // Q4.8
    output reg  [11:0] gain_b,
    output reg         gain_toggle
);
    reg        vsync_d, href_d, awb_on;
    reg [10:0] xcnt, ycnt;

    wire vsync_rise = vsync & ~vsync_d;
    wire xf  = (pattern == 2'd0) || (pattern == 2'd2);
    wire yf  = (pattern == 2'd0) || (pattern == 2'd1);
    wire cxp = xcnt[0] ^ xf;          // 统一成 RGGB 后的坐标奇偶
    wire cyp = ycnt[0] ^ yf;
    wire is_r = ~cxp & ~cyp;
    wire is_b =  cxp &  cyp;
    wire in_win = href && (xcnt < WIDTH) && (ycnt < HEIGHT);

    // ---------------- 像素通路 ----------------
    wire [11:0] g_sel = is_r ? gain_r : is_b ? gain_b : 12'd256;
    reg  [19:0] prod;
    reg         v1, h1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prod <= 20'd0; v1 <= 1'b0; h1 <= 1'b0;
            raw_o <= 8'd0; vsync_o <= 1'b0; href_o <= 1'b0;
        end else begin
            prod    <= raw * g_sel + 20'd128;
            v1      <= vsync;
            h1      <= href;
            raw_o   <= (prod[19:16] != 4'd0) ? 8'hFF : prod[15:8];
            vsync_o <= v1;
            href_o  <= h1;
        end
    end

    // ---------------- 统计 + 增益计算 ----------------
    localparam S_ACC = 3'd0, S_LOAD = 3'd1, S_RUN = 3'd2, S_STORE = 3'd3, S_UPD = 3'd4;
    reg [2:0]  st, step;
    reg [3:0]  it;
    reg [27:0] sum_r, sum_g, sum_b;
    reg [19:0] cnt_r, cnt_g, cnt_b;
    reg [15:0] avg_r, avg_g, avg_b, t_r, t_b;   // Q8.8
    reg [27:0] ng_r, ng_g, ng_b;                // 灰点和（已乘增益的 RGB）
    reg [19:0] ng_cnt;
    reg [15:0] c_r, c_b;                        // 修正量 G/R、G/B（Q8.8）
    reg [27:0] cb_r, cb_g, cb_b;                // 中央区域和（一键白平衡）
    reg        push_seen, push_now;
    reg [35:0] dn;
    reg [43:0] ds;
    reg [15:0] dq;

    reg [35:0] op_n;
    reg [27:0] op_d;
    always @(*) begin
        case (step)
            3'd0:    begin op_n = {sum_r, 8'd0};          op_d = {8'd0, cnt_r};  end
            3'd1:    begin op_n = {sum_g, 8'd0};          op_d = {8'd0, cnt_g};  end
            3'd2:    begin op_n = {sum_b, 8'd0};          op_d = {8'd0, cnt_b};  end
            3'd3:    begin op_n = {12'd0, avg_g, 8'd0};   op_d = {12'd0, avg_r}; end
            3'd4:    begin op_n = {12'd0, avg_g, 8'd0};   op_d = {12'd0, avg_b}; end
            3'd5:    begin op_n = push_now ? {cb_g, 8'd0} : {ng_g, 8'd0};  op_d = push_now ? cb_r : ng_r; end
            default: begin op_n = push_now ? {cb_g, 8'd0} : {ng_g, 8'd0};  op_d = push_now ? cb_b : ng_b; end
        endcase
    end

    // 一键白平衡统计区：画面中央 1/2 x 1/2
    wire in_center = st_valid && (st_x >= WIDTH / 4) && (st_x < (WIDTH * 3) / 4) &&
                     (st_y >= HEIGHT / 4) && (st_y < (HEIGHT * 3) / 4);

    // 灰点判断（去马赛克输出）
    wire [7:0] s_r = st_rgb[23:16], s_g = st_rgb[15:8], s_b = st_rgb[7:0];
    wire [7:0] s_mx = (s_r > s_g) ? ((s_r > s_b) ? s_r : s_b) : ((s_g > s_b) ? s_g : s_b);
    wire [7:0] s_mn = (s_r < s_g) ? ((s_r < s_b) ? s_r : s_b) : ((s_g < s_b) ? s_g : s_b);
    wire [8:0] s_tol = {3'd0, s_g[7:2]} + {4'd0, s_g[7:3]};     // G * 3/8
    wire       is_gray = st_valid && (s_g >= 8'd24) && (s_mx < 8'd235) &&
                         ({1'b0, s_mx - s_mn} <= s_tol);

    // 灰点目标增益 = 当前增益 × 修正量（Q4.8 × Q8.8 → Q4.8），超过 16 位饱和
    wire [27:0] ng_tr_full = gain_r * c_r;
    wire [27:0] ng_tb_full = gain_b * c_b;
    wire [15:0] ng_tr = (ng_tr_full[27:24] != 4'd0) ? 16'hFFFF : ng_tr_full[23:8];
    wire [15:0] ng_tb = (ng_tb_full[27:24] != 4'd0) ? 16'hFFFF : ng_tb_full[23:8];
    wire        use_gray = (ng_cnt >= MIN_GRAY);

    function [11:0] clamp_gain;
        input [15:0] t;
        begin
            if (t < GAIN_MIN)      clamp_gain = GAIN_MIN;
            else if (t > GAIN_MAX) clamp_gain = GAIN_MAX;
            else                   clamp_gain = t[11:0];
        end
    endfunction

    // 每帧向目标走 1/4（四舍五入，最终误差 ±1）
    function [11:0] smooth;
        input [11:0] cur;
        input [11:0] tgt;
        reg signed [13:0] diff, delta;
        begin
            diff   = $signed({2'b00, tgt}) - $signed({2'b00, cur}) + 14'sd2;
            delta  = diff >>> 2;              // 单独做有符号移位
            smooth = cur + delta[11:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_d <= 1'b0; href_d <= 1'b0; awb_on <= 1'b0;
            xcnt <= 11'd0; ycnt <= 11'd0; pattern <= 2'd0;
            gain_r <= DEF_GAIN_R; gain_b <= DEF_GAIN_B; gain_toggle <= 1'b0;
            cb_r <= 28'd0; cb_g <= 28'd0; cb_b <= 28'd0; push_seen <= 1'b0; push_now <= 1'b0;
            st <= S_ACC; step <= 3'd0; it <= 4'd0;
            sum_r <= 28'd0; sum_g <= 28'd0; sum_b <= 28'd0;
            cnt_r <= 20'd0; cnt_g <= 20'd0; cnt_b <= 20'd0;
            avg_r <= 16'd0; avg_g <= 16'd0; avg_b <= 16'd0; t_r <= 16'd0; t_b <= 16'd0;
            ng_r <= 28'd0; ng_g <= 28'd0; ng_b <= 28'd0; ng_cnt <= 20'd0;
            c_r <= 16'd256; c_b <= 16'd256;
            dn <= 36'd0; ds <= 44'd0; dq <= 16'd0;
        end else begin
            vsync_d <= vsync;
            href_d  <= href;
            if (href) xcnt <= xcnt + 11'd1; else xcnt <= 11'd0;
            if (vsync_rise)          ycnt <= 11'd0;
            else if (href_d & ~href) ycnt <= ycnt + 11'd1;

            case (st)
                S_ACC: begin
                    if (vsync_rise) begin
                        pattern   <= pattern_in;
                        awb_on    <= awb_en_in;
                        push_now  <= (onepush_tog != push_seen);
                        push_seen <= onepush_tog;
                        step      <= 3'd0;
                        st        <= S_LOAD;
                    end else begin
                        if (in_center) begin
                            cb_r <= cb_r + s_r; cb_g <= cb_g + s_g; cb_b <= cb_b + s_b;
                        end
                        if (in_win && raw < SAT_LEVEL) begin
                            if (is_r)      begin sum_r <= sum_r + raw; cnt_r <= cnt_r + 20'd1; end
                            else if (is_b) begin sum_b <= sum_b + raw; cnt_b <= cnt_b + 20'd1; end
                            else           begin sum_g <= sum_g + raw; cnt_g <= cnt_g + 20'd1; end
                        end
                        if (is_gray && ng_cnt != 20'hFFFFF) begin
                            ng_r <= ng_r + s_r; ng_g <= ng_g + s_g; ng_b <= ng_b + s_b;
                            ng_cnt <= ng_cnt + 20'd1;
                        end
                    end
                end
                S_LOAD: begin
                    if ({op_d, 16'd0} <= {8'd0, op_n}) begin   // 商超过 16 位（含除数为 0）：饱和
                        dq <= 16'hFFFF;
                        st <= S_STORE;
                    end else begin
                        dn <= op_n;
                        ds <= {1'b0, op_d, 15'd0};
                        dq <= 16'd0;
                        it <= 4'd0;
                        st <= S_RUN;
                    end
                end
                S_RUN: begin
                    if ({8'd0, dn} >= ds) begin
                        dn <= dn - ds[35:0];
                        dq <= {dq[14:0], 1'b1};
                    end else
                        dq <= {dq[14:0], 1'b0};
                    ds <= ds >> 1;
                    it <= it + 4'd1;
                    if (it == 4'd15) st <= S_STORE;
                end
                S_STORE: begin
                    case (step)
                        3'd0: avg_r <= dq;
                        3'd1: avg_g <= dq;
                        3'd2: avg_b <= dq;
                        3'd3: t_r   <= dq;
                        3'd4: t_b   <= dq;
                        3'd5: c_r   <= dq;
                        default: c_b <= dq;
                    endcase
                    if (step == 3'd6) st <= S_UPD;
                    else begin step <= step + 3'd1; st <= S_LOAD; end
                end
                default: begin   // S_UPD
                    if (push_now) begin
                        // 一键白平衡：一步到位
                        if (cb_r != 28'd0 && cb_b != 28'd0 && cb_g != 28'd0) begin
                            gain_r      <= clamp_gain(ng_tr);
                            gain_b      <= clamp_gain(ng_tb);
                        end
                    end else if (!awb_on) begin
                        // 自动关闭：保持
                    end else if (use_gray) begin
                        gain_r      <= smooth(gain_r, clamp_gain(ng_tr));
                        gain_b      <= smooth(gain_b, clamp_gain(ng_tb));
                    end else if (cnt_r != 20'd0 && cnt_g != 20'd0 && cnt_b != 20'd0 && avg_g >= MIN_AVG_G) begin
                        gain_r      <= smooth(gain_r, clamp_gain(t_r));
                        gain_b      <= smooth(gain_b, clamp_gain(t_b));
                    end
                    gain_toggle <= ~gain_toggle;   // 每帧都通知一次（手动模式下状态行也显示实际增益）
                    sum_r <= 28'd0; sum_g <= 28'd0; sum_b <= 28'd0;
                    cnt_r <= 20'd0; cnt_g <= 20'd0; cnt_b <= 20'd0;
                    ng_r <= 28'd0; ng_g <= 28'd0; ng_b <= 28'd0; ng_cnt <= 20'd0;
                    cb_r <= 28'd0; cb_g <= 28'd0; cb_b <= 28'd0;
                    push_now <= 1'b0;
                    st <= S_ACC;
                end
            endcase
        end
    end
endmodule

// ----------------------------------------------------------------------------
// RAW8 Bayer 流式双线性去马赛克
//
// 原始数据按行写入 4 个行存储体（行号 mod 4）。第 r+1 行收齐后，
// 引擎用第 r-1、r、r+1 行算出第 r 行的 RGB；边界按 2 像素镜像，
// 保持 Bayer 颜色相位不变。最后一行在整帧收完后补算。
// 只用加法和移位，不用乘法器。
// ----------------------------------------------------------------------------
module bayer_demosaic_stream #(
    parameter WIDTH         = 1280,
    parameter HEIGHT        = 720,
    parameter SKIP_FRAMES   = 2
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire [1:0]  pattern,      // 0 BGGR 1 GBRG 2 GRBG 3 RGGB，只在场消隐期间变化
    input  wire        vsync,
    input  wire        href,
    input  wire [7:0]  raw,
    output reg         out_valid,
    output reg  [10:0] out_x,
    output reg  [10:0] out_y,
    output reg  [23:0] out_rgb,
    output reg  [10:0] meas_width,
    output reg  [10:0] meas_height,
    output reg         frame_toggle
);
    // 把 4 种排列统一成 RGGB：(0,0)=R (1,0)=Gr (0,1)=Gb (1,1)=B
    reg XFLIP, YFLIP;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin XFLIP <= 1'b1; YFLIP <= 1'b1; end
        else begin
            XFLIP <= (pattern == 2'd0) || (pattern == 2'd2);
            YFLIP <= (pattern == 2'd0) || (pattern == 2'd1);
        end

    // ---------------- 输入侧 ----------------
    reg        vsync_d, href_d;
    reg [10:0] in_x, in_row, line_cnt;
    reg [3:0]  skip_cnt;
    reg        frame_ok, reset_pending;

    wire vsync_rise = vsync & ~vsync_d;
    wire href_rise  = href  & ~href_d;
    wire href_fall  = ~href & href_d;
    wire wr_en      = href & frame_ok & (in_row < HEIGHT) & (in_x < WIDTH);

    // ---------------- 引擎控制 ----------------
    reg        busy;
    reg [10:0] next_row, job_row, j;
    reg [1:0]  bank_t, bank_m, bank_b;

    wire last_ready      = (in_row == HEIGHT) && (next_row == HEIGHT - 1);
    wire engine_eligible = frame_ok && (next_row < HEIGHT) &&
                           ((in_row >= next_row + 11'd2) || last_ready);
    // 帧复位等引擎把上一帧最后一行做完再执行；新一行开始则强制执行
    wire apply_reset     = reset_pending && ((!busy && !engine_eligible) || href_rise);
    wire start_job       = !apply_reset && !busy && engine_eligible;

    wire [10:0] rt_w = (next_row == 0) ? 11'd1 : next_row - 11'd1;
    wire [10:0] rb_w = (next_row == HEIGHT - 1) ? (HEIGHT - 2) : next_row + 11'd1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_d <= 1'b0; href_d <= 1'b0;
            in_x <= 11'd0; in_row <= 11'd0; line_cnt <= 11'd0;
            skip_cnt <= 4'd0; frame_ok <= 1'b0; reset_pending <= 1'b0;
            meas_width <= 11'd0; meas_height <= 11'd0; frame_toggle <= 1'b0;
            busy <= 1'b0; next_row <= 11'd0; job_row <= 11'd0; j <= 11'd0;
            bank_t <= 2'd0; bank_m <= 2'd0; bank_b <= 2'd0;
        end else begin
            vsync_d <= vsync;
            href_d  <= href;

            // 行内像素计数
            if (href) in_x <= in_x + 11'd1;
            else      in_x <= 11'd0;

            if (href_fall) begin
                meas_width <= in_x;
                line_cnt   <= line_cnt + 11'd1;
            end

            if (vsync_rise) begin
                reset_pending <= 1'b1;
                frame_toggle  <= ~frame_toggle;
                meas_height   <= line_cnt;
                line_cnt      <= 11'd0;
                if (!enable)
                    skip_cnt <= 4'd0;
                else if (skip_cnt < SKIP_FRAMES)
                    skip_cnt <= skip_cnt + 4'd1;
            end

            if (!enable)
                frame_ok <= 1'b0;

            if (apply_reset) begin
                reset_pending <= 1'b0;
                in_row        <= 11'd0;
                next_row      <= 11'd0;
                frame_ok      <= enable && (skip_cnt >= SKIP_FRAMES);
            end else if (href_fall && frame_ok && in_row < HEIGHT) begin
                in_row <= in_row + 11'd1;
            end

            if (start_job) begin
                busy     <= 1'b1;
                j        <= 11'd0;
                job_row  <= next_row;
                next_row <= next_row + 11'd1;
                bank_t   <= rt_w[1:0];
                bank_m   <= next_row[1:0];
                bank_b   <= rb_w[1:0];
            end else if (busy) begin
                if (j == WIDTH + 1) busy <= 1'b0;
                j <= j + 11'd1;
            end
        end
    end

    // ---------------- 4 个原始行存储体 ----------------
    // 读列顺序：1, 0, 1, 2, ..., W-1, W-2（左右各镜像 1 列）
    wire [10:0] rd_col = (j == 11'd0)  ? 11'd1 :
                         (j <= WIDTH)  ? (j - 11'd1) : (WIDTH - 2);
    wire [31:0] bank_q;

    genvar g;
    generate
        for (g = 0; g < 4; g = g + 1) begin : g_bank
            reg [7:0] mem [0:WIDTH-1];
            reg [7:0] q;
            always @(posedge clk) begin
                if (wr_en && (in_row[1:0] == g))
                    mem[in_x] <= raw;
                q <= mem[rd_col];
            end
            assign bank_q[g*8 +: 8] = q;
        end
    endgenerate

    // ---------------- 3x3 窗口流水线 ----------------
    reg        s1_v, s2_v;
    reg [10:0] s1_j, s2_j, s1_row, s2_row;
    reg [1:0]  s1_bt, s1_bm, s1_bb;
    reg [7:0]  t0, t1, t2, m0, m1, m2, b0, b1, b2;

    wire [7:0] q_top = bank_q[s1_bt*8 +: 8];
    wire [7:0] q_mid = bank_q[s1_bm*8 +: 8];
    wire [7:0] q_bot = bank_q[s1_bb*8 +: 8];

    wire [9:0] sum_x4 = t1 + b1 + m0 + m2;   // 上下左右
    wire [9:0] diag  = t0 + t2 + b0 + b2;   // 四个对角
    wire [8:0] hsum  = m0 + m2;             // 左右
    wire [8:0] vsum  = t1 + b1;             // 上下
    wire       xp    = s2_j[0] ^ XFLIP;     // 中心 x = s2_j-2，奇偶与 s2_j 相同
    wire       yp    = s2_row[0] ^ YFLIP;

    reg [23:0] rgb_c;
    always @(*) begin
        case ({yp, xp})
            2'b00:   rgb_c = {m1,         sum_x4[9:2], diag[9:2]};   // R 点
            2'b01:   rgb_c = {hsum[8:1],  m1,         vsum[8:1]};   // R 行的 G
            2'b10:   rgb_c = {vsum[8:1],  m1,         hsum[8:1]};   // B 行的 G
            default: rgb_c = {diag[9:2],  sum_x4[9:2], m1       };   // B 点
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_v <= 1'b0; s2_v <= 1'b0; s1_j <= 11'd0; s2_j <= 11'd0;
            s1_row <= 11'd0; s2_row <= 11'd0;
            s1_bt <= 2'd0; s1_bm <= 2'd0; s1_bb <= 2'd0;
            t0 <= 8'd0; t1 <= 8'd0; t2 <= 8'd0;
            m0 <= 8'd0; m1 <= 8'd0; m2 <= 8'd0;
            b0 <= 8'd0; b1 <= 8'd0; b2 <= 8'd0;
            out_valid <= 1'b0; out_x <= 11'd0; out_y <= 11'd0; out_rgb <= 24'd0;
        end else begin
            // 第 1 级：存储体输出对应 rd_col(j)
            s1_v   <= busy;
            s1_j   <= j;
            s1_row <= job_row;
            s1_bt  <= bank_t; s1_bm <= bank_m; s1_bb <= bank_b;
            // 第 2 级：窗口左移一列
            s2_v   <= s1_v;
            s2_j   <= s1_j;
            s2_row <= s1_row;
            if (s1_v) begin
                t0 <= t1; t1 <= t2; t2 <= q_top;
                m0 <= m1; m1 <= m2; m2 <= q_mid;
                b0 <= b1; b1 <= b2; b2 <= q_bot;
            end
            // 第 3 级：输出
            out_valid <= s2_v && (s2_j >= 11'd2);
            out_x     <= s2_j - 11'd2;
            out_y     <= s2_row;
            out_rgb   <= rgb_c;
        end
    end
endmodule

// ----------------------------------------------------------------------------
// 颜色处理：3x3 颜色矩阵（饱和度/串色校正）+ 每通道 256 项查找表（gamma）
//   out = LUT[ clamp( (M * rgb + 128) >> 8 ) ]，M 为 Q8 有符号系数（256 = 1.0）
//   enable=0 时直接输出输入（延迟相同），方便对比
//   系数在相机时钟域，查找表写口在 lut_clk 域
//   9 个乘法器，3 个 256x8 查找表；延迟 4 拍
// ----------------------------------------------------------------------------
module color_proc (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         enable,
    input  wire [107:0] coef,          // 9 x 12 位：{m00,m01,m02,m10,...,m22}，m00 在最高位
    input  wire         in_valid,
    input  wire [10:0]  in_x,
    input  wire [10:0]  in_y,
    input  wire [23:0]  in_rgb,
    output reg          out_valid,
    output reg  [10:0]  out_x,
    output reg  [10:0]  out_y,
    output reg  [23:0]  out_rgb,
    input  wire         lut_clk,
    input  wire         lut_we,
    input  wire [2:0]   lut_ch,        // 写哪些通道：bit2 R  bit1 G  bit0 B
    input  wire [7:0]   lut_addr,
    input  wire [7:0]   lut_data
);
    // 系数拆开
    wire signed [11:0] c0 = coef[107:96], c1 = coef[95:84], c2 = coef[83:72];
    wire signed [11:0] c3 = coef[71:60],  c4 = coef[59:48], c5 = coef[47:36];
    wire signed [11:0] c6 = coef[35:24],  c7 = coef[23:12], c8 = coef[11:0];

    // 第 1 拍：9 个乘积
    wire signed [8:0] r_s = {1'b0, in_rgb[23:16]};
    wire signed [8:0] g_s = {1'b0, in_rgb[15:8]};
    wire signed [8:0] b_s = {1'b0, in_rgb[7:0]};
    reg  signed [20:0] p0, p1, p2, p3, p4, p5, p6, p7, p8;
    reg  [23:0] rgb1;
    reg         v1;
    reg  [10:0] x1, y1;
    always @(posedge clk) begin
        p0 <= r_s * c0; p1 <= g_s * c1; p2 <= b_s * c2;
        p3 <= r_s * c3; p4 <= g_s * c4; p5 <= b_s * c5;
        p6 <= r_s * c6; p7 <= g_s * c7; p8 <= b_s * c8;
        rgb1 <= in_rgb; v1 <= in_valid; x1 <= in_x; y1 <= in_y;
    end

    // 第 2 拍：求和、舍入、饱和
    function [7:0] sat8;
        input signed [22:0] s;
        reg signed [22:0] t;
        begin
            t = (s + 23'sd128) >>> 8;
            if (t < 0)        sat8 = 8'd0;
            else if (t > 255) sat8 = 8'd255;
            else              sat8 = t[7:0];
        end
    endfunction
    wire signed [22:0] sr = p0 + p1 + p2;
    wire signed [22:0] sg = p3 + p4 + p5;
    wire signed [22:0] sb = p6 + p7 + p8;
    reg  [7:0]  mr, mg, mb;
    reg  [23:0] rgb2;
    reg         v2;
    reg  [10:0] x2, y2;
    always @(posedge clk) begin
        mr <= sat8(sr); mg <= sat8(sg); mb <= sat8(sb);
        rgb2 <= rgb1; v2 <= v1; x2 <= x1; y2 <= y1;
    end

    // 第 3 拍：查找表
    reg [7:0] lut_r [0:255];
    reg [7:0] lut_g [0:255];
    reg [7:0] lut_b [0:255];
    always @(posedge lut_clk) begin
        if (lut_we && lut_ch[2]) lut_r[lut_addr] <= lut_data;
        if (lut_we && lut_ch[1]) lut_g[lut_addr] <= lut_data;
        if (lut_we && lut_ch[0]) lut_b[lut_addr] <= lut_data;
    end
    reg [7:0]  qr, qg, qb;
    reg [23:0] rgb3;
    reg        v3;
    reg [10:0] x3, y3;
    always @(posedge clk) begin
        qr <= lut_r[mr]; qg <= lut_g[mg]; qb <= lut_b[mb];
        rgb3 <= rgb2; v3 <= v2; x3 <= x2; y3 <= y2;
    end

    // 第 4 拍：输出
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0; out_x <= 11'd0; out_y <= 11'd0; out_rgb <= 24'd0;
        end else begin
            out_valid <= v3; out_x <= x3; out_y <= y3;
            out_rgb   <= enable ? {qr, qg, qb} : rgb3;
        end
    end
endmodule

// ----------------------------------------------------------------------------
// 环形行缓存 + 跟随相机的 HDMI 时序
// ----------------------------------------------------------------------------
module direct_video_out #(
    parameter WIDTH       = 1280,
    parameter HEIGHT      = 720,
    parameter SLOT_BITS   = 3,
    parameter LOCK_LINES  = 3,
    parameter LOST_FRAMES = 3,
    parameter MAX_WAIT_LINES = 64,                // 相机比 HDMI 慢时，最多在场消隐里多等这么多行
    parameter H_TOTAL = 1650, parameter H_SYNC = 40, parameter H_BP = 220, parameter H_ACT = 1280,
    parameter V_TOTAL = 750,  parameter V_SYNC = 5,  parameter V_BP = 20,  parameter V_ACT = 720,
    parameter [23:0] NO_SIGNAL_RGB = 24'h000060   // 没有相机时显示深蓝色
)(
    input  wire        wr_clk,
    input  wire        wr_rst_n,
    input  wire        wr_valid,
    input  wire [10:0] wr_x,
    input  wire [10:0] wr_y,
    input  wire [23:0] wr_rgb,
    input  wire        pclk,
    input  wire        rst_n,
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
    function integer clog2;
        input integer value;
        integer v;
        begin
            v = value - 1;
            clog2 = 0;
            while (v > 0) begin
                v = v >> 1;
                clog2 = clog2 + 1;
            end
        end
    endfunction

    function [15:0] gray2bin;
        input [15:0] gv;
        integer k;
        begin
            gray2bin[15] = gv[15];
            for (k = 14; k >= 0; k = k - 1)
                gray2bin[k] = gray2bin[k+1] ^ gv[k];
        end
    endfunction

    localparam NSLOT  = 1 << SLOT_BITS;
    localparam DEPTH  = NSLOT * WIDTH;
    localparam AW     = clog2(DEPTH);
    localparam HS0    = H_SYNC + H_BP;
    localparam VS0    = V_SYNC + V_BP;
    localparam V_LOCK = VS0 - LOCK_LINES;

    // ---------------- 写侧（相机时钟） ----------------
    reg [23:0] mem [0:DEPTH-1];
    wire [AW-1:0] wr_addr = wr_y[SLOT_BITS-1:0] * WIDTH + wr_x;
    always @(posedge wr_clk)
        if (wr_valid) mem[wr_addr] <= wr_rgb;

    reg [15:0] rows_total, rows_gray, anchor_base;
    reg        anchor_tog;
    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            rows_total <= 16'd0; rows_gray <= 16'd0;
            anchor_base <= 16'd0; anchor_tog <= 1'b0;
        end else begin
            if (wr_valid && wr_x == WIDTH - 1)
                rows_total <= rows_total + 16'd1;
            rows_gray <= rows_total ^ (rows_total >> 1);
            if (wr_valid && wr_x == 11'd0 && wr_y == 11'd0) begin
                anchor_tog  <= ~anchor_tog;       // 第 0 行开始输出：帧锚点
                anchor_base <= rows_total;
            end
        end
    end

    // ---------------- 跨到像素时钟 ----------------
    (* ASYNC_REG = "TRUE" *) reg [2:0]  anc_s;
    (* ASYNC_REG = "TRUE" *) reg [15:0] rg_s1, rg_s2;
    reg [15:0] base_s;
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            anc_s <= 3'd0; rg_s1 <= 16'd0; rg_s2 <= 16'd0; base_s <= 16'd0;
        end else begin
            anc_s  <= {anc_s[1:0], anchor_tog};
            rg_s1  <= rows_gray;
            rg_s2  <= rg_s1;
            base_s <= anchor_base;   // 在锚点翻转前已稳定一整帧
        end
    end
    wire        anchor_edge = anc_s[2] ^ anc_s[1];
    wire [15:0] rows_now    = gray2bin(rg_s2);

    // ---------------- 时序发生器 ----------------
    reg [11:0] h_cnt, v_cnt;
    reg        anchor_pend;
    reg [15:0] frame_base;
    reg [2:0]  lost_cnt;
    reg [1:0]  good_cnt;
    reg [7:0]  wait_cnt;
    reg [15:0] adj_cnt, uf_cnt, of_cnt;

    wire        line_end = (h_cnt == H_TOTAL - 1);
    wire [11:0] v_nat    = (v_cnt == V_TOTAL - 1) ? 12'd0 : v_cnt + 12'd1;
    wire        pend_now = anchor_pend | anchor_edge;
    wire signed [13:0] v_err = $signed({2'b00, v_nat}) - $signed(V_LOCK);

    wire        h_act  = (h_cnt >= HS0) && (h_cnt < HS0 + H_ACT);
    wire        v_act  = (v_cnt >= VS0) && (v_cnt < VS0 + V_ACT);
    wire [11:0] x_act  = h_cnt - HS0;
    wire [11:0] y_act  = v_cnt - VS0;
    wire        in_img = h_act && v_act && (x_act < WIDTH) && (y_act < HEIGHT);
    wire [AW-1:0] rd_addr = y_act[SLOT_BITS-1:0] * WIDTH + x_act;
    wire [15:0] frows  = rows_now - frame_base;      // 本帧已完成的行数

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            h_cnt <= 12'd0; v_cnt <= 12'd0;
            anchor_pend <= 1'b0; frame_base <= 16'd0;
            lost_cnt <= 3'd7; good_cnt <= 2'd0; wait_cnt <= 8'd0;
            adj_cnt <= 16'd0; uf_cnt <= 16'd0; of_cnt <= 16'd0;
            locked <= 1'b0; cam_present <= 1'b0; frame_toggle <= 1'b0;
            adj_gray <= 16'd0; uf_gray <= 16'd0; of_gray <= 16'd0;
        end else begin
            if (anchor_edge) anchor_pend <= 1'b1;

            if (line_end) begin
                h_cnt <= 12'd0;
                if (pend_now) begin
                    // 在行边界把行号对到 V_LOCK，下一帧第 0 行在 LOCK_LINES 行后开始
                    anchor_pend <= 1'b0;
                    v_cnt       <= V_LOCK;
                    frame_base  <= base_s;
                    lost_cnt    <= 3'd0;
                    wait_cnt    <= 8'd0;
                    if (v_nat != V_LOCK || wait_cnt != 8'd0) adj_cnt <= adj_cnt + 16'd1;
                    // 每帧修正不超过 2 行视为锁定
                    if (v_err >= -2 && v_err <= 2 && wait_cnt <= 8'd2) begin
                        if (good_cnt != 2'd3) good_cnt <= good_cnt + 2'd1;
                    end else
                        good_cnt <= 2'd0;
                end else if (v_nat == V_LOCK && cam_present && wait_cnt < MAX_WAIT_LINES) begin
                    // 相机帧头还没到：停在 V_LOCK-1 多走一行消隐等它
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

            // 监视：开始读第 y 行时它必须已完成；读完时第 y+N 行不能已开始写
            if (cam_present && v_act && (y_act < HEIGHT)) begin
                if (h_cnt == HS0 && frows <= y_act)
                    uf_cnt <= uf_cnt + 16'd1;
                if (h_cnt == HS0 + WIDTH && frows >= y_act + NSLOT)
                    of_cnt <= of_cnt + 16'd1;
            end

            adj_gray <= adj_cnt ^ (adj_cnt >> 1);
            uf_gray  <= uf_cnt  ^ (uf_cnt  >> 1);
            of_gray  <= of_cnt  ^ (of_cnt  >> 1);
        end
    end

    // ---------------- 读出流水线（3 拍） ----------------
    reg          hs_a, vs_a, de_a, img_a;
    reg [AW-1:0] addr_a;
    reg          hs_b, vs_b, de_b, img_b, show_b;
    reg [23:0]   q_b;
    always @(posedge pclk) begin
        hs_a   <= (h_cnt < H_SYNC);
        vs_a   <= (v_cnt < V_SYNC);
        de_a   <= h_act && v_act;
        img_a  <= in_img;
        addr_a <= rd_addr;

        q_b    <= mem[addr_a];
        hs_b   <= hs_a;
        vs_b   <= vs_a;
        de_b   <= de_a;
        img_b  <= img_a;
        show_b <= cam_present;

        hs  <= hs_b;
        vs  <= vs_b;
        de  <= de_b;
        rgb <= !de_b ? 24'd0 :
               !show_b ? NO_SIGNAL_RGB :
               img_b ? q_b : 24'd0;
    end
endmodule

// ----------------------------------------------------------------------------
// 状态串口：每秒打印一行，例如
// cam_fps=60 hdmi_fps=60 cam_pclk_hz=84000000 w=1280 h=720 lock=1 adj=8 uf=0 of=0 bayer=0 awb=1 rgain=185 bgain=120 edge=1 ccm=1
//   adj/uf/of 为本秒内的次数：帧对齐修正、读空、覆盖
//   rgain/bgain 为白平衡增益的百分比（185 = 1.85 倍）
// ----------------------------------------------------------------------------
module video_status_uart #(
    parameter CLK_HZ    = 27000000,
    parameter BAUD      = 115200,
    parameter REPORT_HZ = 1
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        cam_clk,
    input  wire        cam_frame_toggle,
    input  wire        hdmi_frame_toggle,
    input  wire [10:0] cam_width,
    input  wire [10:0] cam_height,
    input  wire        locked,
    input  wire        cam_present,
    input  wire [15:0] adj_gray,
    input  wire [15:0] uf_gray,
    input  wire [15:0] of_gray,
    input  wire [1:0]  bayer,          // clk 时钟域
    input  wire        awb_en,         // clk 时钟域
    input  wire        edge_sel,       // clk 时钟域
    input  wire        ccm_en,         // clk 时钟域
    input  wire [11:0] gain_r,         // 相机时钟域，gain_toggle 翻转后稳定一整帧
    input  wire [11:0] gain_b,
    input  wire        gain_toggle,
    output wire        tx
);
    function [27:0] g2b28;
        input [27:0] gv;
        integer k;
        begin
            g2b28[27] = gv[27];
            for (k = 26; k >= 0; k = k - 1)
                g2b28[k] = g2b28[k+1] ^ gv[k];
        end
    endfunction

    function [15:0] g2b16;
        input [15:0] gv;
        integer k;
        begin
            g2b16[15] = gv[15];
            for (k = 14; k >= 0; k = k - 1)
                g2b16[k] = g2b16[k+1] ^ gv[k];
        end
    endfunction

    function [27:0] pow10;
        input [3:0] i;
        begin
            case (i)
                4'd0: pow10 = 28'd1;
                4'd1: pow10 = 28'd10;
                4'd2: pow10 = 28'd100;
                4'd3: pow10 = 28'd1000;
                4'd4: pow10 = 28'd10000;
                4'd5: pow10 = 28'd100000;
                4'd6: pow10 = 28'd1000000;
                4'd7: pow10 = 28'd10000000;
                default: pow10 = 28'd100000000;
            endcase
        end
    endfunction

    // ---------------- 相机像素时钟计数（相机时钟域） ----------------
    reg [1:0]  cam_rst_sync;
    always @(posedge cam_clk or negedge rst_n)
        if (!rst_n) cam_rst_sync <= 2'b00; else cam_rst_sync <= {cam_rst_sync[0], 1'b1};
    reg [27:0] cc_bin, cc_gray;
    always @(posedge cam_clk or negedge cam_rst_sync[1]) begin
        if (!cam_rst_sync[1]) begin
            cc_bin <= 28'd0; cc_gray <= 28'd0;
        end else begin
            cc_bin  <= cc_bin + 28'd1;
            cc_gray <= cc_bin ^ (cc_bin >> 1);
        end
    end

    // ---------------- 同步到 clk ----------------
    (* ASYNC_REG = "TRUE" *) reg [27:0] cc_s1, cc_s2;
    (* ASYNC_REG = "TRUE" *) reg [15:0] adj_s1, adj_s2, uf_s1, uf_s2, of_s1, of_s2;
    (* ASYNC_REG = "TRUE" *) reg [10:0] w_s1, w_s2, h_s1, h_s2;
    (* ASYNC_REG = "TRUE" *) reg [2:0]  camf_s, hdmif_s;
    (* ASYNC_REG = "TRUE" *) reg [1:0]  lock_s, pres_s;

    localparam integer TICK_MAX = CLK_HZ / REPORT_HZ - 1;
    localparam integer BAUD_DIV = CLK_HZ / BAUD;

    reg [31:0] tick_cnt;
    reg [15:0] cam_frames, hdmi_frames;
    reg [27:0] cc_prev;
    reg [15:0] adj_prev, uf_prev, of_prev;
    // 快照
    reg [27:0] v_camfps, v_hdmifps, v_hz, v_w, v_h, v_lock, v_adj, v_uf, v_of;
    reg [27:0] v_bayer, v_awb, v_rg, v_bg, v_edge, v_ccm;
    reg        report_go;

    // 白平衡增益跨时钟：gain_toggle 同步后再取数据（数据已稳定）
    (* ASYNC_REG = "TRUE" *) reg [2:0] gt_s;
    reg [11:0] gr_c, gb_c;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gt_s <= 3'd0; gr_c <= 12'd256; gb_c <= 12'd256;
        end else begin
            gt_s <= {gt_s[1:0], gain_toggle};
            if (gt_s[2] ^ gt_s[1]) begin
                gr_c <= gain_r;
                gb_c <= gain_b;
            end
        end
    end
    // 打印成百分比：gain * 100 / 256 = gain * 25 / 64
    wire [16:0] gr_x25 = {gr_c, 4'd0} + {gr_c, 3'd0} + gr_c;
    wire [16:0] gb_x25 = {gb_c, 4'd0} + {gb_c, 3'd0} + gb_c;

    wire [27:0] cc_now  = g2b28(cc_s2);
    wire [15:0] adj_now = g2b16(adj_s2);
    wire [15:0] uf_now  = g2b16(uf_s2);
    wire [15:0] of_now  = g2b16(of_s2);
    wire        tick    = (tick_cnt == TICK_MAX);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cc_s1 <= 0; cc_s2 <= 0; adj_s1 <= 0; adj_s2 <= 0; uf_s1 <= 0; uf_s2 <= 0;
            of_s1 <= 0; of_s2 <= 0; w_s1 <= 0; w_s2 <= 0; h_s1 <= 0; h_s2 <= 0;
            camf_s <= 0; hdmif_s <= 0; lock_s <= 0; pres_s <= 0;
            tick_cnt <= 0; cam_frames <= 0; hdmi_frames <= 0;
            cc_prev <= 0; adj_prev <= 0; uf_prev <= 0; of_prev <= 0;
            v_camfps <= 0; v_hdmifps <= 0; v_hz <= 0; v_w <= 0; v_h <= 0;
            v_lock <= 0; v_adj <= 0; v_uf <= 0; v_of <= 0;
            v_bayer <= 0; v_awb <= 0; v_rg <= 0; v_bg <= 0; v_edge <= 0; v_ccm <= 0;
            report_go <= 1'b0;
        end else begin
            cc_s1 <= cc_gray;  cc_s2 <= cc_s1;
            adj_s1 <= adj_gray; adj_s2 <= adj_s1;
            uf_s1 <= uf_gray;  uf_s2 <= uf_s1;
            of_s1 <= of_gray;  of_s2 <= of_s1;
            w_s1 <= cam_width;  w_s2 <= w_s1;
            h_s1 <= cam_height; h_s2 <= h_s1;
            camf_s  <= {camf_s[1:0],  cam_frame_toggle};
            hdmif_s <= {hdmif_s[1:0], hdmi_frame_toggle};
            lock_s  <= {lock_s[0], locked};
            pres_s  <= {pres_s[0], cam_present};

            report_go <= 1'b0;
            if (tick) begin
                tick_cnt    <= 0;
                v_camfps    <= cam_frames  + (camf_s[2]  ^ camf_s[1]);
                v_hdmifps   <= hdmi_frames + (hdmif_s[2] ^ hdmif_s[1]);
                cam_frames  <= 0;
                hdmi_frames <= 0;
                v_hz        <= cc_now - cc_prev;
                cc_prev     <= cc_now;
                v_w         <= w_s2;
                v_h         <= h_s2;
                v_lock      <= lock_s[1];
                v_adj       <= adj_now - adj_prev;  adj_prev <= adj_now;
                v_uf        <= uf_now  - uf_prev;   uf_prev  <= uf_now;
                v_of        <= of_now  - of_prev;   of_prev  <= of_now;
                v_bayer     <= bayer;
                v_awb       <= awb_en;
                v_edge      <= edge_sel;
                v_ccm       <= ccm_en;
                v_rg        <= gr_x25[16:6];
                v_bg        <= gb_x25[16:6];
                report_go   <= 1'b1;
            end else begin
                tick_cnt <= tick_cnt + 1;
                if (camf_s[2]  ^ camf_s[1])  cam_frames  <= cam_frames  + 16'd1;
                if (hdmif_s[2] ^ hdmif_s[1]) hdmi_frames <= hdmi_frames + 16'd1;
            end
        end
    end

    // ---------------- 报文 ----------------
    // \001~\011、\016~\023 是数值字段占位符（避开 \012 \015），\015\012 为回车换行
    localparam integer MSG_LEN = 110;
    localparam [8*MSG_LEN-1:0] MSG =
        "cam_fps=\001 hdmi_fps=\002 cam_pclk_hz=\003 w=\004 h=\005 lock=\006 adj=\007 uf=\010 of=\011 bayer=\016 awb=\017 rgain=\020 bgain=\021 edge=\022 ccm=\023\015\012";

    reg [6:0]  idx;
    reg [2:0]  state;
    reg [27:0] work;
    reg [3:0]  pidx, digit;
    reg        started;
    reg        tx_load;
    reg [7:0]  tx_data;

    wire [7:0] ch = MSG[8*(MSG_LEN-1-idx) +: 8];

    reg [27:0] field_val;
    always @(*) begin
        case (ch)
            8'd1: field_val = v_camfps;
            8'd2: field_val = v_hdmifps;
            8'd3: field_val = v_hz;
            8'd4: field_val = v_w;
            8'd5: field_val = v_h;
            8'd6: field_val = v_lock;
            8'd7: field_val = v_adj;
            8'd8: field_val = v_uf;
            8'd9: field_val = v_of;
            8'd14: field_val = v_bayer;
            8'd15: field_val = v_awb;
            8'd16: field_val = v_rg;
            8'd17: field_val = v_bg;
            8'd18: field_val = v_edge;
            default: field_val = v_ccm;
        endcase
    end
    wire is_field = (ch >= 8'd1 && ch <= 8'd9) || (ch >= 8'd14 && ch <= 8'd19);

    localparam S_IDLE = 3'd0, S_CHAR = 3'd1, S_DIG = 3'd2,
               S_LOAD = 3'd3, S_WAIT = 3'd4;
    reg        sending_digit;

    // 串口发送器
    reg [9:0]  sh;
    reg [3:0]  nbits;
    reg [15:0] bcnt;
    wire       tx_idle = (nbits == 4'd0);
    assign tx = tx_idle ? 1'b1 : sh[0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sh <= 10'h3FF; nbits <= 4'd0; bcnt <= 16'd0;
        end else if (tx_load) begin
            sh <= {1'b1, tx_data, 1'b0}; nbits <= 4'd10; bcnt <= 16'd0;
        end else if (nbits != 4'd0) begin
            if (bcnt == BAUD_DIV - 1) begin
                bcnt  <= 16'd0;
                sh    <= {1'b1, sh[9:1]};
                nbits <= nbits - 4'd1;
            end else
                bcnt <= bcnt + 16'd1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            idx <= 0; state <= S_IDLE; work <= 0; pidx <= 0; digit <= 0;
            started <= 1'b0; tx_load <= 1'b0; tx_data <= 8'd0; sending_digit <= 1'b0;
        end else begin
            tx_load <= 1'b0;
            case (state)
                S_IDLE: if (report_go) begin idx <= 0; state <= S_CHAR; end
                S_CHAR: begin
                    if (idx == MSG_LEN)
                        state <= S_IDLE;
                    else if (is_field) begin
                        work <= field_val; pidx <= 4'd8; digit <= 4'd0; started <= 1'b0;
                        state <= S_DIG;
                    end else begin
                        tx_data <= ch; tx_load <= 1'b1; sending_digit <= 1'b0;
                        state <= S_LOAD;
                    end
                end
                S_DIG: begin
                    if (work >= pow10(pidx)) begin
                        work  <= work - pow10(pidx);
                        digit <= digit + 4'd1;
                    end else if (digit != 0 || started || pidx == 0) begin
                        tx_data <= 8'h30 + digit; tx_load <= 1'b1;
                        started <= 1'b1; sending_digit <= 1'b1;
                        state <= S_LOAD;
                    end else begin
                        pidx <= pidx - 4'd1;
                    end
                end
                S_LOAD: state <= S_WAIT;           // 等发送器装入
                S_WAIT: if (tx_idle) begin
                    if (!sending_digit)
                        begin idx <= idx + 1; state <= S_CHAR; end
                    else if (pidx == 0)
                        begin idx <= idx + 1; state <= S_CHAR; end
                    else
                        begin pidx <= pidx - 4'd1; digit <= 4'd0; state <= S_DIG; end
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

// ----------------------------------------------------------------------------
// 串口命令（uart_rx，8N1）：板上实时调整，不用重新编译
//   '0'~'3'  Bayer 排列 0 BGGR 1 GBRG 2 GRBG 3 RGGB
//   'a'      打开自动白平衡（灰点）  'n'  关闭（保持当前增益，上电默认）
//   'w'      一键白平衡：画面中央当白色，算一次增益并保持
//   'g'      DVP 上升沿采样（默认）  'f'  下降沿采样
//   'c'      打开颜色矩阵+gamma（默认）  'x'  关闭（输出线性画面）
//   'r'      抓原始 Bayer 缩略图  'p'  抓颜色处理前 RGB  'q'  抓颜色处理后 RGB
//   'M'+18 字节   颜色矩阵 9 个 int16（小端，Q8，256=1.0），行优先 m00 m01 m02 m10 ...
//   'L'+768 字节  查找表：R 的 256 项、G 的 256 项、B 的 256 项
// 上电后自动把默认色调曲线 y=255*(x/255)^0.89 写进查找表（函数名沿用 gamma22），矩阵为 DEFAULT_CCM。
// ----------------------------------------------------------------------------
module video_ctrl_uart #(
    parameter CLK_HZ = 27000000,
    parameter BAUD   = 115200,
    parameter [1:0] DEFAULT_BAYER = 2'd0,
    parameter       DEFAULT_AWB   = 1'b1,
    // 饱和度 1.75：[ 390 -112 -22 ; -58 336 -22 ; -58 -112 426 ]，每行之和 256（补码写成十六进制）
    // 10-28 用 OV5640 自带 ISP 画面校准得到（tools/fit_sat.py）
    parameter [107:0] DEFAULT_CCM = {12'h186, 12'hF90, 12'hFEA,
                                     12'hFC6, 12'h150, 12'hFEA,
                                     12'hFC6, 12'hF90, 12'h1AA}
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         rx,
    output reg  [1:0]   bayer,
    output reg          awb_en,
    output reg          edge_sel,      // 'g' 上升沿采样（默认）  'f' 下降沿
    output reg          awb_onepush,   // 'w'：翻转一次
    output reg          snap_req,      // 'p' / 'r' / 'q'：单拍脉冲
    output reg  [1:0]   snap_mode,     // 0 处理前 RGB  1 原始 Bayer  2 处理后 RGB
    output reg          ccm_en,
    output reg  [107:0] ccm_coef,      // 稳定后 ccm_tog 翻转
    output reg          ccm_tog,
    output reg          lut_we,
    output reg  [2:0]   lut_ch,
    output reg  [7:0]   lut_addr,
    output reg  [7:0]   lut_data
);
    localparam integer DIV = CLK_HZ / BAUD;

    function [7:0] gamma22;
        input [7:0] i;
        begin
            case (i)
8'd0: gamma22 = 8'd0;
                8'd1: gamma22 = 8'd2;
                8'd2: gamma22 = 8'd3;
                8'd3: gamma22 = 8'd5;
                8'd4: gamma22 = 8'd6;
                8'd5: gamma22 = 8'd8;
                8'd6: gamma22 = 8'd9;
                8'd7: gamma22 = 8'd10;
                8'd8: gamma22 = 8'd12;
                8'd9: gamma22 = 8'd13;
                8'd10: gamma22 = 8'd14;
                8'd11: gamma22 = 8'd16;
                8'd12: gamma22 = 8'd17;
                8'd13: gamma22 = 8'd18;
                8'd14: gamma22 = 8'd19;
                8'd15: gamma22 = 8'd20;
                8'd16: gamma22 = 8'd22;
                8'd17: gamma22 = 8'd23;
                8'd18: gamma22 = 8'd24;
                8'd19: gamma22 = 8'd25;
                8'd20: gamma22 = 8'd26;
                8'd21: gamma22 = 8'd28;
                8'd22: gamma22 = 8'd29;
                8'd23: gamma22 = 8'd30;
                8'd24: gamma22 = 8'd31;
                8'd25: gamma22 = 8'd32;
                8'd26: gamma22 = 8'd33;
                8'd27: gamma22 = 8'd35;
                8'd28: gamma22 = 8'd36;
                8'd29: gamma22 = 8'd37;
                8'd30: gamma22 = 8'd38;
                8'd31: gamma22 = 8'd39;
                8'd32: gamma22 = 8'd40;
                8'd33: gamma22 = 8'd41;
                8'd34: gamma22 = 8'd42;
                8'd35: gamma22 = 8'd44;
                8'd36: gamma22 = 8'd45;
                8'd37: gamma22 = 8'd46;
                8'd38: gamma22 = 8'd47;
                8'd39: gamma22 = 8'd48;
                8'd40: gamma22 = 8'd49;
                8'd41: gamma22 = 8'd50;
                8'd42: gamma22 = 8'd51;
                8'd43: gamma22 = 8'd52;
                8'd44: gamma22 = 8'd53;
                8'd45: gamma22 = 8'd54;
                8'd46: gamma22 = 8'd56;
                8'd47: gamma22 = 8'd57;
                8'd48: gamma22 = 8'd58;
                8'd49: gamma22 = 8'd59;
                8'd50: gamma22 = 8'd60;
                8'd51: gamma22 = 8'd61;
                8'd52: gamma22 = 8'd62;
                8'd53: gamma22 = 8'd63;
                8'd54: gamma22 = 8'd64;
                8'd55: gamma22 = 8'd65;
                8'd56: gamma22 = 8'd66;
                8'd57: gamma22 = 8'd67;
                8'd58: gamma22 = 8'd68;
                8'd59: gamma22 = 8'd69;
                8'd60: gamma22 = 8'd70;
                8'd61: gamma22 = 8'd71;
                8'd62: gamma22 = 8'd72;
                8'd63: gamma22 = 8'd73;
                8'd64: gamma22 = 8'd75;
                8'd65: gamma22 = 8'd76;
                8'd66: gamma22 = 8'd77;
                8'd67: gamma22 = 8'd78;
                8'd68: gamma22 = 8'd79;
                8'd69: gamma22 = 8'd80;
                8'd70: gamma22 = 8'd81;
                8'd71: gamma22 = 8'd82;
                8'd72: gamma22 = 8'd83;
                8'd73: gamma22 = 8'd84;
                8'd74: gamma22 = 8'd85;
                8'd75: gamma22 = 8'd86;
                8'd76: gamma22 = 8'd87;
                8'd77: gamma22 = 8'd88;
                8'd78: gamma22 = 8'd89;
                8'd79: gamma22 = 8'd90;
                8'd80: gamma22 = 8'd91;
                8'd81: gamma22 = 8'd92;
                8'd82: gamma22 = 8'd93;
                8'd83: gamma22 = 8'd94;
                8'd84: gamma22 = 8'd95;
                8'd85: gamma22 = 8'd96;
                8'd86: gamma22 = 8'd97;
                8'd87: gamma22 = 8'd98;
                8'd88: gamma22 = 8'd99;
                8'd89: gamma22 = 8'd100;
                8'd90: gamma22 = 8'd101;
                8'd91: gamma22 = 8'd102;
                8'd92: gamma22 = 8'd103;
                8'd93: gamma22 = 8'd104;
                8'd94: gamma22 = 8'd105;
                8'd95: gamma22 = 8'd106;
                8'd96: gamma22 = 8'd107;
                8'd97: gamma22 = 8'd108;
                8'd98: gamma22 = 8'd109;
                8'd99: gamma22 = 8'd110;
                8'd100: gamma22 = 8'd111;
                8'd101: gamma22 = 8'd112;
                8'd102: gamma22 = 8'd113;
                8'd103: gamma22 = 8'd114;
                8'd104: gamma22 = 8'd115;
                8'd105: gamma22 = 8'd116;
                8'd106: gamma22 = 8'd117;
                8'd107: gamma22 = 8'd118;
                8'd108: gamma22 = 8'd119;
                8'd109: gamma22 = 8'd120;
                8'd110: gamma22 = 8'd121;
                8'd111: gamma22 = 8'd122;
                8'd112: gamma22 = 8'd123;
                8'd113: gamma22 = 8'd124;
                8'd114: gamma22 = 8'd125;
                8'd115: gamma22 = 8'd126;
                8'd116: gamma22 = 8'd126;
                8'd117: gamma22 = 8'd127;
                8'd118: gamma22 = 8'd128;
                8'd119: gamma22 = 8'd129;
                8'd120: gamma22 = 8'd130;
                8'd121: gamma22 = 8'd131;
                8'd122: gamma22 = 8'd132;
                8'd123: gamma22 = 8'd133;
                8'd124: gamma22 = 8'd134;
                8'd125: gamma22 = 8'd135;
                8'd126: gamma22 = 8'd136;
                8'd127: gamma22 = 8'd137;
                8'd128: gamma22 = 8'd138;
                8'd129: gamma22 = 8'd139;
                8'd130: gamma22 = 8'd140;
                8'd131: gamma22 = 8'd141;
                8'd132: gamma22 = 8'd142;
                8'd133: gamma22 = 8'd143;
                8'd134: gamma22 = 8'd144;
                8'd135: gamma22 = 8'd145;
                8'd136: gamma22 = 8'd146;
                8'd137: gamma22 = 8'd147;
                8'd138: gamma22 = 8'd148;
                8'd139: gamma22 = 8'd149;
                8'd140: gamma22 = 8'd150;
                8'd141: gamma22 = 8'd150;
                8'd142: gamma22 = 8'd151;
                8'd143: gamma22 = 8'd152;
                8'd144: gamma22 = 8'd153;
                8'd145: gamma22 = 8'd154;
                8'd146: gamma22 = 8'd155;
                8'd147: gamma22 = 8'd156;
                8'd148: gamma22 = 8'd157;
                8'd149: gamma22 = 8'd158;
                8'd150: gamma22 = 8'd159;
                8'd151: gamma22 = 8'd160;
                8'd152: gamma22 = 8'd161;
                8'd153: gamma22 = 8'd162;
                8'd154: gamma22 = 8'd163;
                8'd155: gamma22 = 8'd164;
                8'd156: gamma22 = 8'd165;
                8'd157: gamma22 = 8'd166;
                8'd158: gamma22 = 8'd167;
                8'd159: gamma22 = 8'd167;
                8'd160: gamma22 = 8'd168;
                8'd161: gamma22 = 8'd169;
                8'd162: gamma22 = 8'd170;
                8'd163: gamma22 = 8'd171;
                8'd164: gamma22 = 8'd172;
                8'd165: gamma22 = 8'd173;
                8'd166: gamma22 = 8'd174;
                8'd167: gamma22 = 8'd175;
                8'd168: gamma22 = 8'd176;
                8'd169: gamma22 = 8'd177;
                8'd170: gamma22 = 8'd178;
                8'd171: gamma22 = 8'd179;
                8'd172: gamma22 = 8'd180;
                8'd173: gamma22 = 8'd181;
                8'd174: gamma22 = 8'd181;
                8'd175: gamma22 = 8'd182;
                8'd176: gamma22 = 8'd183;
                8'd177: gamma22 = 8'd184;
                8'd178: gamma22 = 8'd185;
                8'd179: gamma22 = 8'd186;
                8'd180: gamma22 = 8'd187;
                8'd181: gamma22 = 8'd188;
                8'd182: gamma22 = 8'd189;
                8'd183: gamma22 = 8'd190;
                8'd184: gamma22 = 8'd191;
                8'd185: gamma22 = 8'd192;
                8'd186: gamma22 = 8'd193;
                8'd187: gamma22 = 8'd193;
                8'd188: gamma22 = 8'd194;
                8'd189: gamma22 = 8'd195;
                8'd190: gamma22 = 8'd196;
                8'd191: gamma22 = 8'd197;
                8'd192: gamma22 = 8'd198;
                8'd193: gamma22 = 8'd199;
                8'd194: gamma22 = 8'd200;
                8'd195: gamma22 = 8'd201;
                8'd196: gamma22 = 8'd202;
                8'd197: gamma22 = 8'd203;
                8'd198: gamma22 = 8'd204;
                8'd199: gamma22 = 8'd205;
                8'd200: gamma22 = 8'd205;
                8'd201: gamma22 = 8'd206;
                8'd202: gamma22 = 8'd207;
                8'd203: gamma22 = 8'd208;
                8'd204: gamma22 = 8'd209;
                8'd205: gamma22 = 8'd210;
                8'd206: gamma22 = 8'd211;
                8'd207: gamma22 = 8'd212;
                8'd208: gamma22 = 8'd213;
                8'd209: gamma22 = 8'd214;
                8'd210: gamma22 = 8'd215;
                8'd211: gamma22 = 8'd215;
                8'd212: gamma22 = 8'd216;
                8'd213: gamma22 = 8'd217;
                8'd214: gamma22 = 8'd218;
                8'd215: gamma22 = 8'd219;
                8'd216: gamma22 = 8'd220;
                8'd217: gamma22 = 8'd221;
                8'd218: gamma22 = 8'd222;
                8'd219: gamma22 = 8'd223;
                8'd220: gamma22 = 8'd224;
                8'd221: gamma22 = 8'd225;
                8'd222: gamma22 = 8'd225;
                8'd223: gamma22 = 8'd226;
                8'd224: gamma22 = 8'd227;
                8'd225: gamma22 = 8'd228;
                8'd226: gamma22 = 8'd229;
                8'd227: gamma22 = 8'd230;
                8'd228: gamma22 = 8'd231;
                8'd229: gamma22 = 8'd232;
                8'd230: gamma22 = 8'd233;
                8'd231: gamma22 = 8'd234;
                8'd232: gamma22 = 8'd234;
                8'd233: gamma22 = 8'd235;
                8'd234: gamma22 = 8'd236;
                8'd235: gamma22 = 8'd237;
                8'd236: gamma22 = 8'd238;
                8'd237: gamma22 = 8'd239;
                8'd238: gamma22 = 8'd240;
                8'd239: gamma22 = 8'd241;
                8'd240: gamma22 = 8'd242;
                8'd241: gamma22 = 8'd243;
                8'd242: gamma22 = 8'd243;
                8'd243: gamma22 = 8'd244;
                8'd244: gamma22 = 8'd245;
                8'd245: gamma22 = 8'd246;
                8'd246: gamma22 = 8'd247;
                8'd247: gamma22 = 8'd248;
                8'd248: gamma22 = 8'd249;
                8'd249: gamma22 = 8'd250;
                8'd250: gamma22 = 8'd251;
                8'd251: gamma22 = 8'd251;
                8'd252: gamma22 = 8'd252;
                8'd253: gamma22 = 8'd253;
                8'd254: gamma22 = 8'd254;
                8'd255: gamma22 = 8'd255;
                default: gamma22 = 8'd255;
            endcase
        end
    endfunction

    // ---------------- 接收 ----------------
    (* ASYNC_REG = "TRUE" *) reg [1:0] rx_s;
    reg [15:0] cnt;
    reg [3:0]  bitn;
    reg [7:0]  shreg;
    reg        busy;
    reg        rx_stb;
    reg [7:0]  rx_byte;
    wire       rxb = rx_s[1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_s <= 2'b11; cnt <= 16'd0; bitn <= 4'd0; shreg <= 8'd0; busy <= 1'b0;
            rx_stb <= 1'b0; rx_byte <= 8'd0;
        end else begin
            rx_s   <= {rx_s[0], rx};
            rx_stb <= 1'b0;
            if (!busy) begin
                if (!rxb) begin                    // 起始位下降沿
                    busy <= 1'b1;
                    cnt  <= DIV / 2;
                    bitn <= 4'd0;
                end
            end else if (cnt != 16'd0) begin
                cnt <= cnt - 16'd1;
            end else begin
                cnt <= DIV - 1;
                if (bitn == 4'd0) begin
                    if (rxb) busy <= 1'b0;         // 毛刺，不是起始位
                    else     bitn <= 4'd1;
                end else if (bitn <= 4'd8) begin
                    shreg <= {rxb, shreg[7:1]};    // 低位先发
                    bitn  <= bitn + 4'd1;
                end else begin
                    busy <= 1'b0;
                    if (rxb) begin                 // 停止位正确才算收到
                        rx_stb  <= 1'b1;
                        rx_byte <= shreg;
                    end
                end
            end
        end
    end

    // ---------------- 解释命令 ----------------
    localparam P_NONE = 2'd0, P_CCM = 2'd1, P_LUT = 2'd2;
    reg [1:0]   pmode;
    reg [9:0]   pcnt;          // 已收到的负载字节数
    reg [143:0] cbuf;          // 9 x int16
    reg         init_busy;
    reg [8:0]   init_addr;

    // int16 Q8 → 12 位（超出 ±8.0 时饱和）
    function [11:0] to12;
        input [15:0] v;
        begin
            if ($signed(v) > 16'sd2047)       to12 = 12'h7FF;
            else if ($signed(v) < -16'sd2048) to12 = 12'h800;
            else                              to12 = v[11:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bayer <= DEFAULT_BAYER; awb_en <= DEFAULT_AWB; edge_sel <= 1'b1; awb_onepush <= 1'b0;
            snap_req <= 1'b0; snap_mode <= 2'd0;
            ccm_en <= 1'b1; ccm_coef <= DEFAULT_CCM; ccm_tog <= 1'b0;
            lut_we <= 1'b0; lut_ch <= 3'b000; lut_addr <= 8'd0; lut_data <= 8'd0;
            pmode <= P_NONE; pcnt <= 10'd0; cbuf <= 144'd0;
            init_busy <= 1'b1; init_addr <= 9'd0;
        end else begin
            snap_req <= 1'b0;
            lut_we   <= 1'b0;
            if (init_busy) begin
                // 上电写入默认色调曲线（三个通道同时写）
                lut_we   <= 1'b1;
                lut_ch   <= 3'b111;
                lut_addr <= init_addr[7:0];
                lut_data <= gamma22(init_addr[7:0]);
                if (init_addr == 9'd255) begin
                    init_busy <= 1'b0;
                    ccm_tog   <= ~ccm_tog;           // 让相机侧装入默认矩阵
                end
                init_addr <= init_addr + 9'd1;
            end else if (rx_stb) begin
                case (pmode)
                    P_CCM: begin
                        cbuf <= {cbuf[135:0], rx_byte};
                        pcnt <= pcnt + 10'd1;
                        if (pcnt == 10'd17) begin
                            // 第 k 个字节在 cbuf[(16-k)*8 +: 8]，m_i = {b(2i+1), b(2i)}
                            ccm_coef <= {to12({cbuf[127:120], cbuf[135:128]}),
                                         to12({cbuf[111:104], cbuf[119:112]}),
                                         to12({cbuf[95:88],   cbuf[103:96]}),
                                         to12({cbuf[79:72],   cbuf[87:80]}),
                                         to12({cbuf[63:56],   cbuf[71:64]}),
                                         to12({cbuf[47:40],   cbuf[55:48]}),
                                         to12({cbuf[31:24],   cbuf[39:32]}),
                                         to12({cbuf[15:8],    cbuf[23:16]}),
                                         to12({rx_byte,       cbuf[7:0]})};
                            ccm_tog <= ~ccm_tog;
                            pmode   <= P_NONE;
                        end
                    end
                    P_LUT: begin
                        lut_we   <= 1'b1;
                        lut_ch   <= (pcnt < 10'd256) ? 3'b100 : (pcnt < 10'd512) ? 3'b010 : 3'b001;
                        lut_addr <= pcnt[7:0];
                        lut_data <= rx_byte;
                        pcnt     <= pcnt + 10'd1;
                        if (pcnt == 10'd767) pmode <= P_NONE;
                    end
                    default: begin
                        case (rx_byte)
                            "0", "1", "2", "3": bayer  <= rx_byte[1:0];
                            "a", "A":           awb_en <= 1'b1;
                            "n", "N":           awb_en <= 1'b0;
                            "w", "W":           awb_onepush <= ~awb_onepush;
                            "g", "G":           edge_sel <= 1'b1;
                            "f", "F":           edge_sel <= 1'b0;
                            "c":                ccm_en <= 1'b1;
                            "x", "X":           ccm_en <= 1'b0;
                            "p":                begin snap_req <= 1'b1; snap_mode <= 2'd0; end
                            "r":                begin snap_req <= 1'b1; snap_mode <= 2'd1; end
                            "q", "Q":           begin snap_req <= 1'b1; snap_mode <= 2'd2; end
                            "M":                begin pmode <= P_CCM; pcnt <= 10'd0; end
                            "L":                begin pmode <= P_LUT; pcnt <= 10'd0; end
                            default: ;
                        endcase
                    end
                endcase
            end
        end
    end
endmodule

// ----------------------------------------------------------------------------
// 串口抓图：收到请求后抓下一整帧的缩略数据，从 tx 发回电脑
//   mode 1（'r'）：白平衡之前的原始 Bayer。每 8x8 取左上角 2x2 小块，
//                  4 字节顺序为 (x0,y0) (x1,y0) (x0,y1) (x1,y1)
//   mode 0（'p'）：去马赛克输出 RGB（白平衡之后、颜色处理之前），每 8x8 取左上角 1 点，4 字节为 R G B 0
//   mode 2（'q'）：颜色处理之后的 RGB（送 HDMI 的画面，叠加文字之前），格式同上
// 报文："@IMG" + 'R'/'P'/'Q' + (W/8)*(H/8)*4 字节（按行）+ "\r\n"
// 1280x720 时为 160x90x4 = 57600 字节，115200 波特率约 5 秒。
// 发送期间 busy=1，顶层用它把 uart_tx 切到这里。
// ----------------------------------------------------------------------------
module video_snapshot #(
    parameter WIDTH  = 1280,
    parameter HEIGHT = 720,
    parameter CLK_HZ = 27000000,
    parameter BAUD   = 115200
)(
    input  wire        clk,            // 发送时钟（sys_clk）
    input  wire        rst_n,
    input  wire        req,            // clk 域单拍脉冲
    input  wire [1:0]  mode,
    input  wire        cam_clk,
    input  wire [7:0]  raw,
    input  wire        href,
    input  wire        vsync,
    input  wire        dm_valid,
    input  wire [10:0] dm_x,
    input  wire [10:0] dm_y,
    input  wire [23:0] dm_rgb,
    input  wire        cp_valid,
    input  wire [10:0] cp_x,
    input  wire [10:0] cp_y,
    input  wire [23:0] cp_rgb,
    output wire        tx,
    output reg         busy
);
    localparam SW = WIDTH / 8;
    localparam SH = HEIGHT / 8;
    localparam N  = SW * SH;
    localparam integer BAUD_DIV = CLK_HZ / BAUD;

    // ---------------- 请求跨到相机时钟 ----------------
    reg       req_tog;
    reg [1:0] mode_r;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin req_tog <= 1'b0; mode_r <= 2'd0; end
        else if (req && !busy) begin req_tog <= ~req_tog; mode_r <= mode; end

    reg [1:0] cam_rst_sync;
    always @(posedge cam_clk or negedge rst_n)
        if (!rst_n) cam_rst_sync <= 2'b00; else cam_rst_sync <= {cam_rst_sync[0], 1'b1};
    wire cam_rst_n = cam_rst_sync[1];

    // 选择 RGB 来源：mode 2 用颜色处理后，否则用去马赛克输出
    wire        s_valid = (mode_r == 2'd2) ? cp_valid : dm_valid;
    wire [10:0] s_x     = (mode_r == 2'd2) ? cp_x     : dm_x;
    wire [10:0] s_y     = (mode_r == 2'd2) ? cp_y     : dm_y;
    wire [23:0] s_rgb   = (mode_r == 2'd2) ? cp_rgb   : dm_rgb;

    (* ASYNC_REG = "TRUE" *) reg [2:0] rq_s;
    reg        armed, capturing, cmode, done_tog;   // cmode：1 原始 Bayer
    reg        vsync_d, href_d;
    reg [10:0] xc, yc;
    wire vsync_rise = vsync & ~vsync_d;
    wire rgb_first  = s_valid && s_x == 11'd0 && s_y == 11'd0;
    wire rgb_last   = s_valid && s_x == WIDTH - 1 && s_y == HEIGHT - 1;

    always @(posedge cam_clk or negedge cam_rst_n) begin
        if (!cam_rst_n) begin
            rq_s <= 3'd0; armed <= 1'b0; capturing <= 1'b0; cmode <= 1'b0; done_tog <= 1'b0;
            vsync_d <= 1'b0; href_d <= 1'b0; xc <= 11'd0; yc <= 11'd0;
        end else begin
            rq_s    <= {rq_s[1:0], req_tog};
            vsync_d <= vsync;
            href_d  <= href;
            if (href) xc <= xc + 11'd1; else xc <= 11'd0;
            if (vsync_rise)          yc <= 11'd0;
            else if (href_d & ~href) yc <= yc + 11'd1;

            if (rq_s[2] ^ rq_s[1]) begin
                armed <= 1'b1;
                cmode <= (mode_r == 2'd1);             // mode_r 早已稳定
            end else if (armed && !capturing) begin
                if (cmode ? vsync_rise : rgb_first) begin
                    capturing <= 1'b1;
                    armed     <= 1'b0;
                end
            end else if (capturing) begin
                if (cmode ? vsync_rise : rgb_last) begin
                    capturing <= 1'b0;
                    done_tog  <= ~done_tog;
                end
            end
        end
    end

    // ---------------- 写入 4 个字节 RAM ----------------
    wire        rgb_cap = !cmode && s_valid && (capturing || (armed && rgb_first)) &&
                          s_x[2:0] == 3'd0 && s_y[2:0] == 3'd0;
    wire        raw_cap = cmode && capturing && href && xc < WIDTH && yc < HEIGHT &&
                          xc[2:1] == 2'd0 && yc[2:1] == 2'd0;
    wire [7:0]  ri = cmode ? xc[10:3] : s_x[10:3];
    wire [7:0]  rj = cmode ? yc[10:3] : s_y[10:3];
    wire [15:0] waddr = rj * SW + ri;
    wire [1:0]  lane  = {yc[0], xc[0]};

    reg [7:0] m0 [0:N-1];
    reg [7:0] m1 [0:N-1];
    reg [7:0] m2 [0:N-1];
    reg [7:0] m3 [0:N-1];
    always @(posedge cam_clk) begin
        if (rgb_cap) begin
            m0[waddr] <= s_rgb[23:16];
            m1[waddr] <= s_rgb[15:8];
            m2[waddr] <= s_rgb[7:0];
            m3[waddr] <= 8'd0;
        end else if (raw_cap) begin
            case (lane)
                2'd0: m0[waddr] <= raw;
                2'd1: m1[waddr] <= raw;
                2'd2: m2[waddr] <= raw;
                default: m3[waddr] <= raw;
            endcase
        end
    end

    // ---------------- 发送（clk 域） ----------------
    reg [15:0] raddr;
    reg [7:0]  q0, q1, q2, q3;
    always @(posedge clk) begin
        q0 <= m0[raddr]; q1 <= m1[raddr]; q2 <= m2[raddr]; q3 <= m3[raddr];
    end

    (* ASYNC_REG = "TRUE" *) reg [2:0] dn_s;
    reg [9:0]  sh;
    reg [3:0]  nbits;
    reg [15:0] bcnt;
    reg        tx_load;
    reg [7:0]  tx_data;
    wire       tx_idle = (nbits == 4'd0);
    assign tx = tx_idle ? 1'b1 : sh[0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sh <= 10'h3FF; nbits <= 4'd0; bcnt <= 16'd0;
        end else if (tx_load) begin
            sh <= {1'b1, tx_data, 1'b0}; nbits <= 4'd10; bcnt <= 16'd0;
        end else if (nbits != 4'd0) begin
            if (bcnt == BAUD_DIV - 1) begin
                bcnt <= 16'd0; sh <= {1'b1, sh[9:1]}; nbits <= nbits - 4'd1;
            end else
                bcnt <= bcnt + 16'd1;
        end
    end

    localparam T_IDLE = 3'd0, T_PICK = 3'd1, T_LOAD = 3'd2, T_WAIT = 3'd3, T_RD = 3'd4;
    reg [2:0]  tst;
    reg [2:0]  phase;     // 0 头部 1 数据 2 结尾
    reg [2:0]  hidx;
    reg [1:0]  bsel;

    function [7:0] head_byte;
        input [2:0] i;
        input [1:0] m;
        begin
            case (i)
                3'd0: head_byte = "@";
                3'd1: head_byte = "I";
                3'd2: head_byte = "M";
                3'd3: head_byte = "G";
                default: head_byte = (m == 2'd1) ? "R" : (m == 2'd2) ? "Q" : "P";
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dn_s <= 3'd0; busy <= 1'b0; tst <= T_IDLE; phase <= 3'd0; hidx <= 3'd0;
            bsel <= 2'd0; raddr <= 16'd0; tx_load <= 1'b0; tx_data <= 8'd0;
        end else begin
            dn_s    <= {dn_s[1:0], done_tog};
            tx_load <= 1'b0;
            case (tst)
                T_IDLE: if (dn_s[2] ^ dn_s[1]) begin
                    busy <= 1'b1; phase <= 3'd0; hidx <= 3'd0; bsel <= 2'd0; raddr <= 16'd0;
                    tst <= T_PICK;
                end
                T_RD: tst <= T_PICK;    // 等 RAM 按新地址读出
                T_PICK: begin
                    tst <= T_LOAD;
                    tx_load <= 1'b1;
                    case (phase)
                        3'd0: tx_data <= head_byte(hidx, mode_r);
                        3'd1: tx_data <= (bsel == 2'd0) ? q0 : (bsel == 2'd1) ? q1 :
                                         (bsel == 2'd2) ? q2 : q3;
                        default: tx_data <= (hidx == 3'd0) ? 8'h0D : 8'h0A;
                    endcase
                end
                T_LOAD: tst <= T_WAIT;
                default: if (tx_idle) begin   // T_WAIT：本字节发完，决定下一个
                    tst <= T_RD;
                    case (phase)
                        3'd0: if (hidx == 3'd4) begin phase <= 3'd1; hidx <= 3'd0; end
                              else hidx <= hidx + 3'd1;
                        3'd1: if (bsel == 2'd3) begin
                                  bsel <= 2'd0;
                                  if (raddr == N - 1) phase <= 3'd2;
                                  else raddr <= raddr + 16'd1;
                              end else bsel <= bsel + 2'd1;
                        default: if (hidx == 3'd1) begin busy <= 1'b0; tst <= T_IDLE; end
                                 else hidx <= hidx + 3'd1;
                    endcase
                end
            endcase
        end
    end
endmodule
