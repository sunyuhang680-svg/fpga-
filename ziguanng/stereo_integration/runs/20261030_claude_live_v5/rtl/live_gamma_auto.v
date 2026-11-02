`timescale 1ns / 1ps
// ============================================================================
// claude live_gamma：实时视频自动 gamma 控制器（sys_clk 域）
//
// 每收到一帧直方图（live_luma_hist 发布）：
//   S(g) = sum_i h[i] * T(i,g)，T(i,g) = round(255 * (i/255)^(g/256))，g 为 Q8，128..512（0.5..2.0）
//   目标 D = N * 128（映射后平均亮度为中灰），判据与渲染器 LIGHT（gamma_controller_search_v2）相同：
//   S 随 g 单调下降，二分找 S(g) <= D 的最小 g，取它和前一个 g 中误差较小者；
//   当前 g 的误差不大于它时保持当前值；然后按 (|t-cur|+4)>>3 平滑一步。
//   与 v2 的差别：误差并列时不再额外往 1.0 收拢；T 用 log2/exp2 表计算，与 v2 ROM 最多差 1。
// 算完把 T(i,next) 写进 color_proc 的三张查找表。关闭时写回恒等表一次。
// 乘法全部用移位相加的串行实现（APM 已用满）。每 bin 约 23 拍，每帧约 12 次求和，
// 50 MHz 下约 1.4 ms，远小于一帧。
// ============================================================================
module live_gamma_auto (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,          // 1：自动 gamma（LIGHT）
    // 直方图（相机时钟域发布）
    input  wire        pub_toggle,
    input  wire        pub_bank,
    input  wire [19:0] pub_count,
    output wire [8:0]  hist_addr,
    input  wire [19:0] hist_data,       // hist_addr 后一拍有效
    // 查找表写口（接 color_proc 的 lut_*，clk 即 lut_clk）
    output reg         lut_we,
    output reg  [7:0]  lut_addr,
    output reg  [7:0]  lut_data,
    output reg         lut_active,      // 1：查找表里是自动 gamma（应强制打开 color_proc）
    // 状态
    output reg  [9:0]  applied_gamma_q8,
    output reg  [9:0]  target_gamma_q8,
    output reg         frame_tick       // 每更新一次翻转
);
    // ---------------- 表 ----------------
    // 常数表在 live_gamma_tables.v，读出寄存一拍
    wire [15:0] log2_q;
    wire [16:0] exp2_q;

    // ---------------- 发布同步 ----------------
    (* ASYNC_REG = "TRUE" *) reg [2:0] tog_s;
    reg        job_pend, job_bank;
    reg [19:0] job_count;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) tog_s <= 3'd0;
        else        tog_s <= {tog_s[1:0], pub_toggle};
    end
    wire pub_edge = tog_s[2] ^ tog_s[1];

    // ---------------- 状态机 ----------------
    localparam IDLE=0, BIN_RD=1, BIN_WAIT=2, MUL_GL=3, EXP_RD=4, EXP_WAIT=5, T_CALC=6,
               MUL_HT=7, ACC=8, EVAL_DONE=9, SMOOTH=10, LUT_GEN=11, LUT_WR=12, DONE=13;
    localparam P_CURRENT=0, P_SEARCH=1, P_RIGHT=2, P_LEFT=3;

    reg [3:0]  state;
    reg        writing_lut;         // 1：本轮逐 bin 是给查找表算值
    reg [1:0]  phase;
    reg [7:0]  bin;
    reg [9:0]  g;                   // 当前求值的 gamma
    reg [3:0]  cnt;
    reg [19:0] h;
    reg [15:0] lval;
    reg [25:0] gl_acc;              // g * L
    reg [16:0] e_val;
    reg [7:0]  t_val;
    reg [27:0] ht_acc;              // h * T
    reg [27:0] sum;
    reg [9:0]  cur, lo, hi, cross_g, best_g;
    reg [27:0] cur_err, right_err, best_err, hi_err;
    reg [19:0] count;
    reg        bank;

    // 二分中点，11 位防溢出
    wire [10:0] mid_lo_w = {1'b0, lo} + {1'b0, g};
    wire [10:0] mid_hi_w = {1'b0, g} + 11'd1 + {1'b0, hi};
    wire [9:0]  mid_lo = mid_lo_w[10:1];
    wire [9:0]  mid_hi = mid_hi_w[10:1];
    wire [27:0] desired = {1'b0, count, 7'b0};
    wire [27:0] err_abs = (sum > desired) ? sum - desired : desired - sum;

    assign hist_addr = {bank, bin};

    // T 计算：x = E * 255，T = round(x >> (16 + k))
    wire [16:0] p_q12  = gl_acc[24:8];
    wire [4:0]  k      = p_q12[16:12];
    wire [9:0]  f      = p_q12[11:2];
    wire [24:0] x255   = {e_val, 8'd0} - {8'd0, e_val};
    wire [5:0]  sh     = 6'd16 + {1'b0, k};
    wire [25:0] x_rnd  = {1'b0, x255} + (26'd1 << (sh - 6'd1));
    wire [25:0] t_full = (sh > 6'd25) ? 26'd0 : (x_rnd >> sh);
    wire [7:0]  t_sat  = (t_full > 26'd255) ? 8'd255 : t_full[7:0];
    live_gamma_log2_rom u_log2(.clk(clk), .addr(bin), .data(log2_q));
    live_gamma_exp2_rom u_exp2(.clk(clk), .addr(f),   .data(exp2_q));

    reg en_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; writing_lut <= 1'b0; phase <= P_CURRENT; bin <= 8'd0; g <= 10'd256; cnt <= 4'd0;
            h <= 20'd0; lval <= 16'd0; gl_acc <= 26'd0; e_val <= 17'd0; t_val <= 8'd0;
            ht_acc <= 28'd0; sum <= 28'd0;
            cur <= 10'd256; lo <= 10'd128; hi <= 10'd513; cross_g <= 10'd128; best_g <= 10'd256;
            cur_err <= 28'd0; right_err <= 28'd0; best_err <= 28'd0; hi_err <= 28'hFFFFFFF; count <= 20'd0; bank <= 1'b0;
            job_pend <= 1'b0; job_bank <= 1'b0; job_count <= 20'd0;
            lut_we <= 1'b0; lut_addr <= 8'd0; lut_data <= 8'd0; lut_active <= 1'b0;
            applied_gamma_q8 <= 10'd256; target_gamma_q8 <= 10'd256; frame_tick <= 1'b0; en_d <= 1'b0;
        end else begin
            lut_we <= 1'b0;
            if (pub_edge) begin
                job_pend <= 1'b1; job_bank <= pub_bank; job_count <= pub_count;
            end
            case (state)
            IDLE: begin
                en_d <= enable;
                if (!enable && !pub_edge) job_pend <= 1'b0;   // 关闭期间的旧直方图不用
                if (!enable && (en_d || lut_active)) begin
                    // 关闭：写回恒等表
                    cur <= 10'd256; g <= 10'd256; bin <= 8'd0; writing_lut <= 1'b1;
                    applied_gamma_q8 <= 10'd256; target_gamma_q8 <= 10'd256;
                    state <= LUT_GEN;
                end else if (enable && job_pend && !pub_edge) begin
                    job_pend <= 1'b0; bank <= job_bank; count <= job_count;
                    if (job_count == 20'd0) state <= IDLE;
                    else begin
                        writing_lut <= 1'b0; phase <= P_CURRENT; g <= cur; bin <= 8'd0; sum <= 28'd0;
                        state <= BIN_RD;
                    end
                end
            end
            // ---- 每个 bin：读 h、L ----
            BIN_RD:   state <= BIN_WAIT;
            BIN_WAIT: begin
                h <= hist_data; lval <= log2_q; gl_acc <= 26'd0; cnt <= 4'd0;
                state <= MUL_GL;
            end
            MUL_GL: begin   // g * L，10 拍
                if (g[cnt]) gl_acc <= gl_acc + ({10'd0, lval} << cnt);
                if (cnt == 4'd9) state <= EXP_RD; else cnt <= cnt + 4'd1;
            end
            EXP_RD:   state <= EXP_WAIT;                           // f 已稳定，表在本拍末读出
            EXP_WAIT: begin e_val <= exp2_q; state <= T_CALC; end
            T_CALC: begin
                t_val  <= (g == 10'd256) ? bin : (bin == 8'd0) ? 8'd0 : t_sat;
                ht_acc <= 28'd0; cnt <= 4'd0;
                state  <= writing_lut ? LUT_WR : MUL_HT;
            end
            MUL_HT: begin   // h * T，8 拍
                if (t_val[cnt[2:0]]) ht_acc <= ht_acc + ({8'd0, h} << cnt[2:0]);
                if (cnt == 4'd7) state <= ACC; else cnt <= cnt + 4'd1;
            end
            ACC: begin
                sum <= sum + ht_acc;
                if (bin == 8'd255) state <= EVAL_DONE;
                else begin bin <= bin + 8'd1; state <= BIN_RD; end
            end
            // ---- 一次 S(g) 求完 ----
            EVAL_DONE: begin
                bin <= 8'd0;
                case (phase)
                P_CURRENT: begin
                    cur_err <= err_abs; lo <= 10'd128; hi <= 10'd513; hi_err <= 28'hFFFFFFF; g <= 10'd320;
                    phase <= P_SEARCH; sum <= 28'd0; state <= BIN_RD;
                end
                P_SEARCH: begin
                    // S(g) <= D：答案在 [lo, g]；否则在 [g+1, hi]
                    if (sum <= desired) begin
                        if (lo < g) begin hi <= g; hi_err <= err_abs; g <= mid_lo; sum <= 28'd0; state <= BIN_RD; end
                        else begin cross_g <= g; phase <= P_RIGHT; right_err <= err_abs; state <= SMOOTH; end
                    end else begin
                        if (g + 10'd1 < hi) begin lo <= g + 10'd1; g <= mid_hi; sum <= 28'd0; state <= BIN_RD; end
                        else begin cross_g <= g + 10'd1; phase <= P_RIGHT; right_err <= hi_err; state <= SMOOTH; end   // hi 已求过（513 时为最大值）
                    end
                end
                P_LEFT: begin
                    // 左邻 cross_g-1 的误差
                    if (err_abs < best_err) begin best_err <= err_abs; best_g <= g; end
                    phase <= P_CURRENT; state <= SMOOTH;
                end
                default: state <= IDLE;
                endcase
            end
            // SMOOTH 分两步：先补齐左邻求值，再平滑
            SMOOTH: begin
                if (phase == P_RIGHT) begin
                    // 右侧候选 = cross_g（若 <=512），先记为最佳，再算左邻 cross_g-1
                    if (cross_g <= 10'd512) begin best_g <= cross_g; best_err <= right_err; end
                    else begin best_g <= 10'd512; best_err <= 28'hFFFFFFF; end
                    if (cross_g > 10'd128) begin
                        g <= (cross_g > 10'd512) ? 10'd512 : cross_g - 10'd1;
                        phase <= P_LEFT; sum <= 28'd0; state <= BIN_RD;
                    end else phase <= P_CURRENT;
                end else begin
                    // 选目标并平滑一步
                    begin : smooth_step
                        reg [9:0] tgt, mag, step, nxt;
                        tgt  = (cur_err <= best_err) ? cur : best_g;
                        mag  = (tgt > cur) ? tgt - cur : cur - tgt;
                        step = (mag + 10'd4) >> 3;
                        nxt  = (tgt > cur) ? cur + step : cur - step;
                        target_gamma_q8 <= tgt;
                        cur <= nxt; g <= nxt;
                    end
                    bin <= 8'd0; writing_lut <= 1'b1; state <= LUT_GEN;
                end
            end
            // ---- 写查找表：对每个 bin 算 T(bin, g) ----
            LUT_GEN: state <= BIN_RD;
            LUT_WR: begin
                lut_we <= 1'b1; lut_addr <= bin; lut_data <= t_val;
                if (bin == 8'd255) state <= DONE;
                else begin bin <= bin + 8'd1; state <= BIN_RD; end
            end
            DONE: begin
                writing_lut <= 1'b0;
                applied_gamma_q8 <= g;
                lut_active <= (g != 10'd256) || en_d;
                frame_tick <= ~frame_tick;
                state <= IDLE;
            end
            default: state <= IDLE;
            endcase
        end
    end
endmodule
