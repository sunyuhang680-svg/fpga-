`timescale 1ns/1ps
// C0 is the existing aligned live/frozen display output. C1 is raw direct-video
// output. Both inputs are synchronous to clk, including their timing signals.
module testing60_live_camera_select #(parameter integer READER_DELAY=10)(
 input wire clk, rst_n, page_right, frozen_active,
 input wire c0_hs,c0_vs,c0_de,input wire [23:0] c0_rgb,
 input wire c1_hs,c1_vs,c1_de,input wire [23:0] c1_rgb,
 output wire hs,vs,de,output wire [23:0] rgb,
 output reg selected_right, output reg visible
);
 reg [READER_DELAY-1:0] rhs,rvs,rde;
 reg [23:0] rr[0:READER_DELAY-1];
 reg prev0,prev1; integer i;
 wire r_hs=rhs[READER_DELAY-1], r_vs=rvs[READER_DELAY-1], r_de=rde[READER_DELAY-1];
 wire [23:0] r_rgb=rr[READER_DELAY-1];
 wire rise0=c0_vs && !prev0, rise1=r_vs && !prev1;
 wire desired=page_right && !frozen_active;
 wire current_rise=selected_right ? rise1 : rise0;
 wire desired_rise=desired ? rise1 : rise0;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   rhs<=0;rvs<=0;rde<=0;prev0<=0;prev1<=0;selected_right<=0;visible<=0;
   for(i=0;i<READER_DELAY;i=i+1) rr[i]<=0;
  end else begin
   rhs<={rhs[READER_DELAY-2:0],c1_hs};
   rvs<={rvs[READER_DELAY-2:0],c1_vs};
   rde<={rde[READER_DELAY-2:0],c1_de};
   rr[0]<=c1_rgb;
   for(i=1;i<READER_DELAY;i=i+1) rr[i]<=rr[i-1];
   prev0<=c0_vs;prev1<=r_vs;
   if(visible) begin
    if(desired!=selected_right && current_rise) visible<=0;
   end else if(desired_rise) begin
    selected_right<=desired;
    visible<=1;
   end
  end
 end
 // Retain one complete source timing while waiting, but paint it black.
 assign hs=selected_right ? r_hs : c0_hs;
 assign vs=selected_right ? r_vs : c0_vs;
 assign de=selected_right ? r_de : c0_de;
 assign rgb=visible ? (selected_right ? r_rgb : c0_rgb) : 24'h000000;
endmodule
