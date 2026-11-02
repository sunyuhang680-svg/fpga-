module camera_data (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        vsync,
    input  wire        href,
    input  wire [7:0]  din,
    input  wire        cfg_done,
    input  wire        ddr_init,

    output reg  [15:0] pixel_data,
    output wire        wf_wr_en,
    output reg         sop,
    output reg         eop,
    output reg         vs,
    output reg         hs
);

//---------------------------------------------------------------------------
// 同步边沿检测
//---------------------------------------------------------------------------
reg [2:0] vsync_r;
reg       href_d;

wire vsync_negedge;
wire href_falling;

assign vsync_negedge = ~vsync_r[1] & vsync_r[2];
assign href_falling  = href_d & ~href;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        vsync_r <= 3'b111;
        href_d  <= 1'b0;
    end
    else begin
        vsync_r <= {vsync_r[1:0], vsync};
        href_d  <= href;
    end
end

//---------------------------------------------------------------------------
// 跳过摄像头启动后的前10帧
//---------------------------------------------------------------------------
reg [4:0] frame_skip_cnt;

wire pic_valid;

assign pic_valid = (frame_skip_cnt >= 5'd10);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        frame_skip_cnt <= 5'd0;
    end
    else if (!cfg_done) begin
        frame_skip_cnt <= 5'd0;
    end
    else if (vsync_negedge && frame_skip_cnt < 5'd10) begin
        frame_skip_cnt <= frame_skip_cnt + 1'b1;
    end
end

//---------------------------------------------------------------------------
// 行计数和像素计数
//---------------------------------------------------------------------------
reg [9:0]  line_count;
reg [10:0] pixel_count;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        line_count <= 10'd0;
    end
    else if (vsync_negedge) begin
        line_count <= 10'd0;
    end
    else if (href_falling) begin
        if (line_count < 10'd719)
            line_count <= line_count + 1'b1;
        else
            line_count <= 10'd0;
    end
end

//---------------------------------------------------------------------------
// RGB565两个8位字节拼接
//
// 每次HREF无效时清除byte_phase。
// 下一行一定重新从第一个字节开始，不继承上一行错误。
//---------------------------------------------------------------------------
reg       byte_phase;
reg [7:0] first_byte;
reg       pixel_data_vld;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        byte_phase     <= 1'b0;
        first_byte     <= 8'd0;
        pixel_data     <= 16'd0;
        pixel_data_vld <= 1'b0;
        pixel_count    <= 11'd0;
        sop            <= 1'b0;
        eop            <= 1'b0;
    end
    else begin
        // 默认不输出有效像素，只有第二个字节到来时置1
        pixel_data_vld <= 1'b0;
        sop            <= 1'b0;
        eop            <= 1'b0;

        if (!pic_valid || !cfg_done) begin
            byte_phase  <= 1'b0;
            pixel_count <= 11'd0;
        end
        else if (!href) begin
            // 每一行结束后重新对齐字节相位
            byte_phase  <= 1'b0;
            pixel_count <= 11'd0;
        end
        else begin
            if (!byte_phase) begin
                // RGB565第一个字节
                first_byte <= din;
                byte_phase <= 1'b1;
            end
            else begin
                // RGB565第二个字节，组成完整像素
                pixel_data     <= {first_byte, din};
                pixel_data_vld <= 1'b1;
                byte_phase     <= 1'b0;

                if (line_count == 10'd0 &&
                    pixel_count == 11'd0)
                    sop <= 1'b1;

                if (line_count == 10'd719 &&
                    pixel_count == 11'd1279)
                    eop <= 1'b1;

                if (pixel_count < 11'd1279)
                    pixel_count <= pixel_count + 1'b1;
                else
                    pixel_count <= 11'd0;
            end
        end
    end
end

//---------------------------------------------------------------------------
// 输出同步信号
//---------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        vs <= 1'b0;
        hs <= 1'b0;
    end
    else begin
        vs <= pic_valid ? vsync : 1'b0;
        hs <= pic_valid ? href  : 1'b0;
    end
end

assign wf_wr_en =
       pic_valid &&
       cfg_done &&
       ddr_init &&
       pixel_data_vld;

endmodule