`timescale 1ns/1ps
// ============================================================================
// claude live_v5c：在 v5 基础上加“实时渲染已上屏”绿色角标。
// claude live_rotate：testing60_photo_initial_select 的拷贝（原文件不改），
// 只多一个“实时渲染画面”模式，用于实时 ROTATE（陀螺仪）：
//   live_render=1（STEREO、选 ROTATE、未冻结）时，不冻结也向 DDR 读口发请求，
//   读的是渲染器最新发布的帧（Top 里 photo_rendered_view 同时看 live_active），
//   HDMI 时序不变（仍是 C0 的 720p60），画面内容按渲染器速度更新。
//   第 1 个场边界：开始发读请求（live_active）；第 2 个场边界：读口已预填，才切画面（live_show），
//   避免切换时闪一帧黑。退出时在场边界同时撤掉。
//   冻结流程（testing60_photo_initial）完全不变；冻结请求优先，live 立刻让出。
// ============================================================================
module testing60_photo_initial_select_live #(
 parameter integer ID_BITS=28,READER_DELAY=10,PIXELS_PER_EYE=460800,TIMEOUT_FRAMES=12
)(
 input wire clk,rst_n,photo_request,retry,memory_ready,configuration_change,
 input wire live_render,live_ready,
 input wire direct_hs,direct_vs,direct_de,input wire [23:0] direct_rgb,
 input wire frozen_hs,frozen_vs,frozen_de,input wire [15:0] frozen_rgb565,
 input wire pair_valid,input wire [ID_BITS-1:0] pair_left,pair_right,
 input wire consumer_req_left,consumer_req_right,
 output wire reader_req_left,reader_req_right,
 input wire reader_ack_left,reader_ack_right,
 input wire [ID_BITS-1:0] accepted_left,accepted_right,
 input wire [12:0] level_left,level_right,
 input wire pop_left,pop_right,empty_left,empty_right,
 output wire [ID_BITS-1:0] selected_left,selected_right,
 output wire hs,vs,de,output wire [23:0] rgb,
 output wire frozen_active,lease_release,failed,busy,photo_hold,legacy_passthrough,output wire [2:0] status,
 output reg live_active,live_show
);
 reg [READER_DELAY-1:0] dhs,dvs,dde;
 reg [23:0] colors[0:READER_DELAY-1];
 reg prev_vs;integer i;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   dhs<=0;dvs<=0;dde<=0;prev_vs<=0;
   for(i=0;i<READER_DELAY;i=i+1)colors[i]<=0;
  end else begin
   dhs<={dhs[READER_DELAY-2:0],direct_hs};
   dvs<={dvs[READER_DELAY-2:0],direct_vs};
   dde<={dde[READER_DELAY-2:0],direct_de};
   colors[0]<=direct_rgb;
   for(i=1;i<READER_DELAY;i=i+1)colors[i]<=colors[i-1];
   prev_vs<=dvs[READER_DELAY-1];
  end
 end
 assign hs=dhs[READER_DELAY-1];assign vs=dvs[READER_DELAY-1];assign de=dde[READER_DELAY-1];
 wire boundary=vs&&!prev_vs;
 reg frame_de_seen;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)frame_de_seen<=0;
  else if(boundary)frame_de_seen<=0;
  else if(dde[READER_DELAY-2])frame_de_seen<=1;
 end
 wire active_video_start=dde[READER_DELAY-2]&&!frame_de_seen;
 wire control_req_left,control_req_right;
 wire fifo_qualified_left,fifo_qualified_right;
 testing60_photo_initial_reader_ready left_ready(.clk(clk),.rst_n(rst_n),.request(control_req_left),
 .ack(reader_ack_left),.identity_ok(accepted_left==selected_left),.empty(empty_left),.level(level_left),.qualified(fifo_qualified_left));
 testing60_photo_initial_reader_ready right_ready(.clk(clk),.rst_n(rst_n),.request(control_req_right),
 .ack(reader_ack_right),.identity_ok(accepted_right==selected_right),.empty(empty_right),.level(level_right),.qualified(fifo_qualified_right));
 testing60_photo_initial #(.ID_BITS(ID_BITS),.PIXELS_PER_EYE(PIXELS_PER_EYE),.TIMEOUT_FRAMES(TIMEOUT_FRAMES)) control(
 .clk(clk),.rst_n(rst_n),.photo_request(photo_request),.retry(retry),.memory_ready(memory_ready),.configuration_change(configuration_change),.frame_start(boundary),.frame_end(boundary),.active_video_start(active_video_start),
 .lease_valid(pair_valid),.lease_left(pair_left),.lease_right(pair_right),
 .consumer_req_left(consumer_req_left),.consumer_req_right(consumer_req_right),
 .reader_req_left(control_req_left),.reader_req_right(control_req_right),
 .reader_ack_left(reader_ack_left),.reader_ack_right(reader_ack_right),
 .accepted_left(accepted_left),.accepted_right(accepted_right),
 .fifo_qualified_left(fifo_qualified_left),.fifo_qualified_right(fifo_qualified_right),
 .pop_left(pop_left),.pop_right(pop_right),.empty_left(empty_left),.empty_right(empty_right),
 .selected_left(selected_left),.selected_right(selected_right),.frozen_active(frozen_active),
 .lease_release(lease_release),.failed(failed),.busy(busy),.photo_hold(photo_hold),.legacy_passthrough(legacy_passthrough),.status(status));

 // 实时渲染画面：只在冻结流程完全空闲时使用
 wire live_allowed=live_render && live_ready && memory_ready && !photo_request && !frozen_active && !legacy_passthrough && !busy;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin live_active<=0;live_show<=0;end
  else if(boundary)begin
   // 只在场边界改变，不打断进行中的读请求握手；冻结或换功能时下一场边界让出
   live_show<=live_allowed && live_active;
   live_active<=live_allowed;
  end
 end
 assign reader_req_left=control_req_left || (live_active && consumer_req_left);
 assign reader_req_right=control_req_right || (live_active && consumer_req_right);

 wire aligned=(frozen_hs==hs)&&(frozen_vs==vs)&&(frozen_de==de);
 wire [23:0] frozen_rgb={frozen_rgb565[15:11],3'b0,frozen_rgb565[10:5],2'b0,frozen_rgb565[4:0],3'b0};
 // claude live_v5c：实时渲染画面已上屏时，右上角画 24x24 绿色方块（x 1240..1263, y 16..39）
 reg [11:0] mark_x,mark_y;reg mark_de_prev;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin mark_x<=0;mark_y<=0;mark_de_prev<=0;end
  else begin
   mark_de_prev<=de;
   if(boundary)begin mark_x<=0;mark_y<=0;end
   else begin
    if(de)mark_x<=mark_x+1'b1;
    else mark_x<=0;
    if(mark_de_prev && !de)mark_y<=mark_y+1'b1;
   end
  end
 end
 wire live_mark=live_show && de && mark_x>=12'd1240 && mark_x<12'd1264 && mark_y>=12'd16 && mark_y<12'd40;
 wire [23:0] base_rgb=(frozen_active || live_show)?((aligned&&frozen_de)?frozen_rgb:24'd0):colors[READER_DELAY-1];
 assign rgb=live_mark?24'h00FF00:base_rgb;
endmodule
