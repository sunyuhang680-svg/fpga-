`timescale 1ns / 1ps
// ============================================================================
// claude live_gamma：实时视频亮度直方图（相机时钟域）
//
// - 输入去马赛克后的 RGB888（颜色矩阵/查找表之前），Y = (77R + 150G + 29B + 128) >> 8，
//   与渲染器 LIGHT（luminance_histogram_pipelined_v2）同一公式，乘常数全部用移位相加，不占 APM
// - 只统计 x、y 都为偶数且 y >= 2 的像素（1/4 抽样）；第 0、1 行用来清零本帧的 bank
// - 两个 bank 乒乓：相机第 (0,0) 像素到来时，上一帧的 bank 发布给 sys_clk 侧（toggle），
//   新帧的 bank 在第 0 行里清零。发布的 bank 要到再下一帧开始才会被重用（约一帧时间可读）
// - 读改写流水与 luminance_histogram_pipelined_v2 相同：前一次写的同 bin 结果前递
// ============================================================================
module live_luma_hist #(
    parameter WIDTH  = 1280,
    parameter HEIGHT = 720
)(
    input  wire        clk,
    input  wire        rst_n,       // 异步复位，模块内同步释放
    input  wire        in_valid,
    input  wire [10:0] in_x,
    input  wire [10:0] in_y,
    input  wire [23:0] in_rgb,
    // 发布（相机时钟产生，toggle 翻转时 pub_bank/pub_count 已稳定）
    output reg         pub_toggle,
    output reg         pub_bank,
    output reg  [19:0] pub_count,
    // 读口（读侧时钟，读出寄存一拍）
    input  wire        rd_clk,
    input  wire [8:0]  rd_addr,     // {bank, bin}
    output reg  [19:0] rd_data
);
    reg [1:0] rst_sync;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) rst_sync <= 2'b00; else rst_sync <= {rst_sync[0], 1'b1};
    wire srst_n = rst_sync[1];

    // 亮度：77 = 64+8+4+1，150 = 128+16+4+2，29 = 16+8+4+1
    wire [7:0] r = in_rgb[23:16], g = in_rgb[15:8], b = in_rgb[7:0];
    reg  [15:0] yr0, yg0, yb0;
    reg  [7:0]  bin1, bin2;
    reg  [2:0]  v;
    always @(posedge clk) begin
        yr0  <= {2'b0, r, 6'b0} + {5'b0, r, 3'b0} + {6'b0, r, 2'b0} + {8'b0, r};
        yg0  <= {1'b0, g, 7'b0} + {4'b0, g, 4'b0} + {6'b0, g, 2'b0} + {7'b0, g, 1'b0};
        yb0  <= {4'b0, b, 4'b0} + {5'b0, b, 3'b0} + {6'b0, b, 2'b0} + {8'b0, b} + 16'd128;
        bin1 <= (yr0 + yg0 + yb0) >> 8;
        bin2 <= bin1;
    end

    wire sample    = in_valid && !in_x[0] && !in_y[0] && (in_y >= 11'd2) && (in_x < WIDTH) && (in_y < HEIGHT);
    wire frame_beg = in_valid && (in_x == 11'd0) && (in_y == 11'd0);

    (* ram_style="block" *) reg [19:0] cnt_mem [0:511];   // 读改写用
    (* ram_style="block" *) reg [19:0] pub_mem [0:511];   // 同样的写，给读侧
    reg        bank;
    reg [8:0]  clr_idx;           // clr_idx[8]=1 表示正在清零
    reg [19:0] rv2, fwd_val, live_count;
    reg [7:0]  fwd_bin;
    reg        fwd_v, started;

    wire        writing = v[2];
    wire [19:0] inc     = ((fwd_v && fwd_bin == bin2) ? fwd_val : rv2) + 20'd1;
    wire        clearing= clr_idx[8];
    wire        mem_we  = clearing || writing;
    wire [8:0]  mem_wa  = clearing ? {bank, clr_idx[7:0]} : {bank, bin2};
    wire [19:0] mem_wd  = clearing ? 20'd0 : inc;

    always @(posedge clk) begin
        if (mem_we) begin cnt_mem[mem_wa] <= mem_wd; pub_mem[mem_wa] <= mem_wd; end
        rv2 <= cnt_mem[{bank, bin1}];
    end
    always @(posedge rd_clk) rd_data <= pub_mem[rd_addr];

    always @(posedge clk or negedge srst_n) begin
        if (!srst_n) begin
            v <= 3'd0; bank <= 1'b0; clr_idx <= 9'h100; live_count <= 20'd0; started <= 1'b0;
            fwd_v <= 1'b0; fwd_bin <= 8'd0; fwd_val <= 20'd0;
            pub_toggle <= 1'b0; pub_bank <= 1'b0; pub_count <= 20'd0;
        end else begin
            v <= {v[1:0], sample && !clearing};
            fwd_v <= writing;
            if (writing) begin fwd_bin <= bin2; fwd_val <= inc; end
            if (writing) live_count <= live_count + 20'd1;
            if (clearing) clr_idx <= (clr_idx[7:0] == 8'hFF) ? 9'd0 : clr_idx + 9'd1;
            if (frame_beg) begin
                // 上一帧完整收集过（从第 0 行开始）才发布
                if (started) begin
                    pub_bank   <= bank;
                    pub_count  <= live_count;
                    pub_toggle <= ~pub_toggle;
                end
                started    <= 1'b1;
                bank       <= ~bank;
                clr_idx    <= 9'h100;
                live_count <= 20'd0;
            end
        end
    end
endmodule
