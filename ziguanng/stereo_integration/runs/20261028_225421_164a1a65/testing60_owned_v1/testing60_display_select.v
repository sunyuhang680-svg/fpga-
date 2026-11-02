`timescale 1ns/1ps
// The two existing DDR display consumers add five clocks each. Keep every
// RGB888 direct pixel intact; only the frozen path has RGB565 quantization.
// Both sources use the supplied direct-video timing, which never stalls.
module testing60_display_select #(parameter integer READER_DELAY=10)(
 input wire clk,rst_n,freeze_request,
 input wire direct_hs,direct_vs,direct_de,input wire [23:0] direct_rgb,
 input wire frozen_hs,frozen_vs,frozen_de,input wire [15:0] frozen_rgb565,
 output wire hs,vs,de,output wire [23:0] rgb,output reg frozen_active
);
 reg [READER_DELAY-1:0] dhs,dvs,dde;
 reg [23:0] colors[0:READER_DELAY-1];
 reg prev_vs;integer i;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin
   dhs<=0;dvs<=0;dde<=0;prev_vs<=0;frozen_active<=0;
   for(i=0;i<READER_DELAY;i=i+1)colors[i]<=0;
  end else begin
   dhs<={dhs[READER_DELAY-2:0],direct_hs};
   dvs<={dvs[READER_DELAY-2:0],direct_vs};
   dde<={dde[READER_DELAY-2:0],direct_de};
   colors[0]<=direct_rgb;
   for(i=1;i<READER_DELAY;i=i+1)colors[i]<=colors[i-1];
   prev_vs<=dvs[READER_DELAY-1];
   if(dvs[READER_DELAY-1] && !prev_vs)frozen_active<=freeze_request;
  end
 end
 assign hs=dhs[READER_DELAY-1];
 assign vs=dvs[READER_DELAY-1];
 assign de=dde[READER_DELAY-1];
 wire aligned=(frozen_hs==hs)&&(frozen_vs==vs)&&(frozen_de==de);
 assign rgb=!frozen_active ? colors[READER_DELAY-1] :
   (aligned && frozen_de ? {frozen_rgb565[15:11],3'b0,frozen_rgb565[10:5],2'b0,frozen_rgb565[4:0],3'b0} : 24'd0);
endmodule
