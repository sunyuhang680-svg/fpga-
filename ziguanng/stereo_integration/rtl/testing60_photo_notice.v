`timescale 1ns/1ps
// Save/read progress is visible while the old live image remains available.
// All video signals have the same one-clock latency; no pixel color arithmetic.
module testing60_photo_notice(
 input wire clk,rst_n,show,input wire [2:0] status,
 input wire [23:0] rgb_in,input wire de_in,hs_in,vs_in,
 output reg [23:0] rgb_out,output reg de_out,hs_out,vs_out
);
 reg [11:0] x,y;reg de_prev,vs_prev;
 wire [11:0] relx=x-12'd16;
 wire [11:0] rely=y-12'd112;
 wire [4:0] char_index=relx[8:4];
 wire [2:0] col=relx[3:1],row=rely[3:1];
 reg [127:0] text_line;
 reg [7:0] letter;
 reg [34:0] bits;
 reg ink;
 function [34:0] glyph;input [7:0] c;begin case(c)
 "A":glyph=35'b01110100011000111111100011000110001;
 "C":glyph=35'b01111100001000010000100001000001111;
 "D":glyph=35'b11110100011000110001100011000111110;
 "E":glyph=35'b11111100001000011110100001000011111;
 "F":glyph=35'b11111100001000011110100001000010000;
 "G":glyph=35'b01111100001000010111100011000101111;
 "H":glyph=35'b10001100011000111111100011000110001;
 "I":glyph=35'b01110001000010000100001000010001110;
 "L":glyph=35'b10000100001000010000100001000011111;
 "M":glyph=35'b10001110111010110101100011000110001;
 "N":glyph=35'b10001110011010110011100011000110001;
 "O":glyph=35'b01110100011000110001100011000101110;
 "P":glyph=35'b11110100011000111110100001000010000;
 "R":glyph=35'b11110100011000111110101001001010001;
 "S":glyph=35'b01111100001000001110000010000111110;
 "T":glyph=35'b11111001000010000100001000010000100;
 "V":glyph=35'b10001100011000110001100010101000100;
 "W":glyph=35'b10001100011000110101101011101110001;
 "Y":glyph=35'b10001100010101000100001000010000100;
 "0":glyph=35'b01110100011001110101110011000101110;
 "1":glyph=35'b00100011000010000100001000010001110;
 default:glyph=0;endcase end endfunction
 always @*begin
  case(status)
   3'd1:text_line="MEMORY WAIT     ";
   3'd2:text_line="SAVING PHOTO    ";
   3'd3:text_line="LOADING PHOTO   ";
   3'd4:text_line="PHOTO READY     ";
   3'd5:text_line="PHOTO RETRY     ";
   3'd6:text_line="C0 SAVE WAIT    ";
   3'd7:text_line="C1 SAVE WAIT    ";
   default:text_line="                ";
  endcase
  letter=0;bits=0;ink=0;
  if(char_index<16)begin
   letter=text_line[127-char_index*8-:8];
   bits=glyph(letter);
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
   if(show && status!=0 && de_in && x>=16 && x<288 && y>=104 && y<136)
    rgb_out<=ink && y>=112 && y<126 ? (status==5 ? 24'hff6666 : (status==4 ? 24'h66ff99 : 24'hffcc66)) : 24'h201810;
  end
 end
endmodule
