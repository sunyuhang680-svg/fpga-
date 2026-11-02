`timescale 1ns/1ps
// Single-buffer snapshot magnifier. Supply the un-overlaid, frozen,
// undistorted identity-rendered stream; output adds one pixel-clock latency.
module measurement_magnifier(
 input wire pixel_clk,rst_n,
 input wire [23:0] camera_rgb,input wire camera_de,camera_hs,camera_vs,
 input wire magnifier_valid,magnifier_right,input wire [9:0] magnifier_x,magnifier_y,
 output wire magnifier_visible,
 output reg [23:0] video_rgb,output reg video_de,video_hs,video_vs
);
 reg [11:0] x,y;reg prev_de,prev_vs;
 wire vs_fall=prev_vs && !camera_vs;
 wire line_end=prev_de && !camera_de;
 reg cfg_valid,cfg_right;reg [9:0] cfg_x,cfg_y,origin_x,origin_y;
 reg capturing,pending,ready;reg [10:0] write_count;
 reg [23:0] tile [0:1023];
 wire legal=magnifier_valid && magnifier_x<640 && magnifier_y<720;
 wire same=cfg_valid && legal && cfg_right==magnifier_right && cfg_x==magnifier_x && cfg_y==magnifier_y;
 assign magnifier_visible=ready && same;
 wire [9:0] requested_origin_x=(magnifier_x<16)?10'd0:(magnifier_x>623)?10'd608:magnifier_x-10'd16;
 wire [9:0] requested_origin_y=(magnifier_y<16)?10'd0:(magnifier_y>703)?10'd688:magnifier_y-10'd16;
 wire [11:0] source_x={2'b00,origin_x}+(cfg_right?12'd640:12'd0);
 wire in_sample=camera_de && capturing && same && x>=source_x && x<source_x+32 && y>=origin_y && y<origin_y+32;
 wire [4:0] sample_dx=x-source_x, sample_dy=y-origin_y;
 wire [9:0] write_addr={sample_dy,sample_dx};
 wire in_window=camera_de && x>=1152 && x<1280 && y>=160 && y<288;
 wire [6:0] window_dx=x-1152,window_dy=y-160;
 wire [9:0] read_addr={window_dy[6:2],window_dx[6:2]};
 wire [6:0] cross_x=(cfg_x-origin_x)*4+2,cross_y=(cfg_y-origin_y)*4+2;
 wire [7:0] dx=(window_dx>cross_x)?window_dx-cross_x:cross_x-window_dx;
 wire [7:0] dy=(window_dy>cross_y)?window_dy-cross_y:cross_y-window_dy;
 wire cross_hit=(dx<=1 && dy<=7) || (dy<=1 && dx<=7);
 always @(posedge pixel_clk or negedge rst_n) begin
  if(!rst_n) begin
   x<=0;y<=0;prev_de<=0;prev_vs<=0;video_rgb<=0;video_de<=0;video_hs<=0;video_vs<=0;
   cfg_valid<=0;cfg_right<=0;cfg_x<=0;cfg_y<=0;origin_x<=0;origin_y<=0;
   capturing<=0;pending<=0;ready<=0;write_count<=0;
  end else begin
   prev_de<=camera_de;prev_vs<=camera_vs;
   if(vs_fall) begin x<=0;y<=0;end
   else if(line_end) begin x<=0;y<=y+1'b1;end
   else if(camera_de) x<=x+1'b1;
   video_de<=camera_de;video_hs<=camera_hs;video_vs<=camera_vs;
   if(magnifier_visible && in_window) video_rgb<=cross_hit?24'hff3030:tile[read_addr];
   else video_rgb<=camera_rgb;
   if(!legal) begin
    cfg_valid<=0;capturing<=0;pending<=0;ready<=0;write_count<=0;
   end else if(!same) begin
    cfg_valid<=1;cfg_right<=magnifier_right;cfg_x<=magnifier_x;cfg_y<=magnifier_y;
    origin_x<=requested_origin_x;origin_y<=requested_origin_y;
    capturing<=vs_fall;pending<=0;ready<=0;write_count<=0;
   end else begin
    if(vs_fall) begin
     if(pending) begin ready<=1;pending<=0;capturing<=0;end
     else if(!ready) begin capturing<=1;write_count<=0;end
    end
    if(in_sample) begin
     tile[write_addr]<=camera_rgb;
     if(write_count==1023) begin capturing<=0;pending<=1;end
     else write_count<=write_count+1'b1;
    end
   end
  end
 end
endmodule
