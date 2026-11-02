`timescale 1ns/1ps
// Explain the unavailable geometric functions only in the frozen-image menu.
// No color transformation is performed; RGB and timing have one equal delay.
module testing60_calibration_notice #(parameter MEAS_ONLY=0)(
 input wire clk,rst_n,show,
 input wire [23:0] rgb_in,input wire de_in,hs_in,vs_in,
 output reg [23:0] rgb_out,output reg de_out,hs_out,vs_out
);
 reg [11:0] x,y;reg de_prev,vs_prev;
 function [34:0] glyph;input [7:0] c;begin case(c)
 "A":glyph=35'b01110100011000111111100011000110001;
 "B":glyph=35'b11110100011000111110100011000111110;
 "C":glyph=35'b01111100001000010000100001000001111;
 "D":glyph=35'b11110100011000110001100011000111110;
 "E":glyph=35'b11111100001000011110100001000011111;
 "I":glyph=35'b01110001000010000100001000010001110;
 "L":glyph=35'b10000100001000010000100001000011111;
 "M":glyph=35'b10001110111010110101100011000110001;
 "S":glyph=35'b01111100001000001110000010000111110;
 "N":glyph=35'b10001110011010110011100011000110001;
 default:glyph=0;endcase end endfunction
 reg [7:0] c;reg [34:0] bits;reg ink;
 integer col,row,index;
 always @*begin
  c=0;bits=0;ink=0;col=0;row=0;index=0;
  if(x>=1104 && x<1272 && y>=120 && y<134)begin
   index=(x-1104)/12;col=((x-1104)%12)/2;row=(y-120)/2;
   if(MEAS_ONLY!=0)begin
    case(index)
    0:c="M";1:c="E";2:c="A";3:c="S";5:c="N";6:c="E";7:c="E";
    8:c="D";9:c="S";11:c="C";12:c="A";13:c="L";
    default:c=0;endcase
   end else begin
   case(index)
   0:c="C";1:c="A";2:c="L";3:c="I";4:c="B";
   6:c="N";7:c="E";8:c="E";9:c="D";10:c="E";11:c="D";
   default:c=0;endcase
   end
   bits=glyph(c);
   if(col<5 && row<7)ink=bits[34-row*5-col];
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin x<=0;y<=0;de_prev<=0;vs_prev<=0;rgb_out<=0;de_out<=0;hs_out<=0;vs_out<=0;end
  else begin
   de_prev<=de_in;vs_prev<=vs_in;
   if(vs_in && !vs_prev)begin x<=0;y<=0;end
   else begin
    if(de_in)x<=x+1'b1;else x<=0;
    if(de_prev && !de_in)y<=y+1'b1;
   end
   rgb_out<=rgb_in;de_out<=de_in;hs_out<=hs_in;vs_out<=vs_in;
   if(show && de_in && x>=1096 && x<1278 && y>=112 && y<144)
    rgb_out<=ink ? 24'hffcc66 : 24'h302010;
  end
 end
endmodule
