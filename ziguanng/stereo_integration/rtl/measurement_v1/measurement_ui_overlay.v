`timescale 1ns/1ps
// One pixel-clock latency on RGB and all timing signals. All control inputs
// must be coherently transferred into pixel_clk by the integrating design.
module measurement_ui_overlay(
 input wire pixel_clk,rst_n,
 input wire [23:0] camera_rgb,input wire camera_de,camera_hs,camera_vs,
 input wire image_mode,menu_visible,entry_allowed,frame_ready,active,selected_index,manual_mode,
 input wire [1:0] left_valid,right_valid,pending,failed,
 input wire [9:0] left_ax,left_ay,left_bx,left_by,right_ax,right_ay,right_bx,right_by,
 input wire display_valid,display_busy,session_failed,input wire [31:0] display_length_q16,
 input wire [7:0] display_reason,reason_a,reason_b,
 output reg [23:0] video_rgb,output reg video_de,video_hs,video_vs
);
 reg [11:0] x,y; reg prev_de,prev_vs;
 wire line_end=prev_de && !camera_de;
 wire vs_fall=prev_vs && !camera_vs;
 function [34:0] font;
 input [7:0] c;
 begin case(c)
 "0":font=35'b01110100011001110101110011000101110;
 "1":font=35'b00100011000010000100001000010001110;
 "2":font=35'b01110100010000100010001000100011111;
 "3":font=35'b11110000010000101110000010000111110;
 "4":font=35'b00010001100101010010111110001000010;
 "5":font=35'b11111100001000011110000010000111110;
 "6":font=35'b01110100001000011110100011000101110;
 "7":font=35'b11111000010001000100010000100001000;
 "8":font=35'b01110100011000101110100011000101110;
 "9":font=35'b01110100011000101111000010000101110;
 "A":font=35'b01110100011000111111100011000110001;
 "B":font=35'b11110100011000111110100011000111110;
 "C":font=35'b01111100001000010000100001000001111;
 "D":font=35'b11110100011000110001100011000111110;
 "E":font=35'b11111100001000011110100001000011111;
 "F":font=35'b11111100001000011110100001000010000;
 "G":font=35'b01111100001000010111100011000101111;
 "H":font=35'b10001100011000111111100011000110001;
 "W":font=35'b10001100011000110101101011101110001;
 "I":font=35'b01110001000010000100001000010001110;
 "L":font=35'b10000100001000010000100001000011111;
 "M":font=35'b10001110111010110101100011000110001;
 "N":font=35'b10001110011010110011100011000110001;
 "O":font=35'b01110100011000110001100011000101110;
 "P":font=35'b11110100011000111110100001000010000;
 "R":font=35'b11110100011000111110101001001010001;
 "S":font=35'b01111100001000001110000010000111110;
 "T":font=35'b11111001000010000100001000010000100;
 "U":font=35'b10001100011000110001100011000101110;
 "V":font=35'b10001100011000110001100010101000100;
 "X":font=35'b10001100010101000100010101000110001;
 "Y":font=35'b10001100010101000100001000010000100;
 ".":font=35'b00000000000000000000000000010000100;
 "-":font=35'b00000000000000011111000000000000000;
 default:font=0;
 endcase end endfunction
 function pixel_char;
 input [7:0] c;input [11:0] px,py;input [11:0] ox,oy;
 reg [34:0] bits;reg [5:0] col,row;
 begin
  pixel_char=0;
  if(px>=ox && px<ox+10 && py>=oy && py<oy+14) begin
   col=(px-ox)>>1;row=(py-oy)>>1;bits=font(c);pixel_char=bits[34-(row*5+col)];
  end
 end endfunction
 function text_pixel;
 input [127:0] s;input [4:0] n;input [11:0] px,py,ox,oy;
 reg [4:0] i;reg [7:0] c;
 begin
  text_pixel=0;
  if(px>=ox && py>=oy && py<oy+14 && px<ox+n*12) begin
   i=(px-ox)/12;c=s[(n-1-i)*8 +: 8];
   text_pixel=pixel_char(c,px,py,ox+i*12,oy);
  end
 end endfunction
 function [7:0] dec_digit;input [15:0] n;begin dec_digit="0"+n;end endfunction
 function [7:0] hex_digit;input [3:0] n;begin hex_digit=(n<10)?("0"+n):("A"+n-10);end endfunction
 function marker;
 input [11:0] px,py;input [9:0] mx,my;input right;
 reg [11:0] cx,cy;reg [11:0] dx,dy;
 begin
  cx={2'b0,mx}+(right?12'd640:12'd0);cy={2'b0,my};
  dx=(px>cx)?px-cx:cx-px;dy=(py>cy)?py-cy:cy-py;
  marker=(dx<=7 && dy<=1)||(dy<=7 && dx<=1);
 end endfunction
 function marker_label;
 input [11:0] px,py;input [9:0] mx,my;input right;input [7:0] c;
 reg [11:0] ox,oy;
 begin
  ox=(right?12'd640:12'd0)+((mx>620)?mx-20:mx+8);
  oy=(my>700)?my-18:my+8;
  marker_label=pixel_char(c,px,py,ox,oy);
 end endfunction
 reg [23:0] draw;reg [127:0] label_text;reg [4:0] label_len;reg [11:0] label_x;
 reg [15:0] whole;reg [22:0] frac_mul;reg [6:0] frac;
 reg [127:0] number_text;reg [7:0] detail;
 always @* begin
  draw=camera_rgb;label_text=0;label_len=0;label_x=0;
  whole=display_length_q16[31:16];frac_mul=display_length_q16[15:0]*7'd100;frac=frac_mul>>16;
  number_text={dec_digit((whole/10000)%10),dec_digit((whole/1000)%10),dec_digit((whole/100)%10),dec_digit((whole/10)%10),dec_digit(whole%10),".",dec_digit(frac/10),dec_digit(frac%10),"M","M"};
  detail=failed[selected_index] ? (selected_index ? reason_b : reason_a) : failed[0] ? reason_a : failed[1] ? reason_b : display_reason;
  if(camera_de) begin
   if(!active && image_mode && menu_visible && entry_allowed && x>=1184 && y>=108 && y<156) begin
    draw=24'h315586;
    if(text_pixel("MEAS",4,x,y,1208,125)) draw=24'hffffff;
   end
   if(active) begin
    if((left_valid[0] && (marker(x,y,left_ax,left_ay,0) || marker_label(x,y,left_ax,left_ay,0,"A"))) || (right_valid[0] && (marker(x,y,right_ax,right_ay,1) || marker_label(x,y,right_ax,right_ay,1,"A")))) draw=24'h00ff70;
    if((left_valid[1] && (marker(x,y,left_bx,left_by,0) || marker_label(x,y,left_bx,left_by,0,"B"))) || (right_valid[1] && (marker(x,y,right_bx,right_by,1) || marker_label(x,y,right_bx,right_by,1,"B")))) draw=24'hffff00;
    if(y>=620 && y<656 && x<640) begin
     draw=24'h10243b;
     if(session_failed) begin if(text_pixel("FAIL SESSION",12,x,y,8,631)) draw=24'hff7070;end
     else if(display_busy) begin if(text_pixel("BUSY",4,x,y,8,631)) draw=24'hffffff;end
     else if(display_valid) begin if(text_pixel(number_text,10,x,y,8,631)) draw=24'hffffff;end
     else if((|failed) || display_reason!=0) begin
      if(text_pixel("FAIL",4,x,y,8,631) || pixel_char(hex_digit(detail[7:4]),x,y,68,631) || pixel_char(hex_digit(detail[3:0]),x,y,80,631)) draw=24'hff7070;
     end else if(text_pixel("NO RESULT",9,x,y,8,631)) draw=24'hffffff;
    end
    if(y>=660) begin
     if(x<128) begin label_text="EXIT";label_len=4;label_x=32;end
     else if(x<256) begin label_text="CLEAR";label_len=5;label_x=134;end
     else if(x<352) begin label_text="A";label_len=1;label_x=294;end
     else if(x<448) begin label_text="B";label_len=1;label_x=390;end
     else if(x<576) begin label_text="MANUAL";label_len=6;label_x=465;end
     else if(x<672) begin label_text="LEFT";label_len=4;label_x=592;end
     else if(x<768) begin label_text="RIGHT";label_len=5;label_x=681;end
     else if(x<864) begin label_text="UP";label_len=2;label_x=800;end
     else if(x<960) begin label_text="DOWN";label_len=4;label_x=880;end
     if(x<960) begin
      draw=24'h254365;
      if((x>=256 && x<352 && !selected_index)||(x>=352 && x<448 && selected_index)||(x>=448 && x<576 && manual_mode)) draw=24'h147d72;
      if(text_pixel(label_text,label_len,x,y,label_x,682)) draw=24'hffffff;
     end
    end
   end
  end
 end
 always @(posedge pixel_clk or negedge rst_n) begin
  if(!rst_n) begin x<=0;y<=0;prev_de<=0;prev_vs<=0;video_rgb<=0;video_de<=0;video_hs<=0;video_vs<=0;end
  else begin
   prev_de<=camera_de;prev_vs<=camera_vs;
   if(vs_fall) begin x<=0;y<=0;end
   else if(line_end) begin x<=0;y<=y+1'b1;end
   else if(camera_de) x<=x+1'b1;
   video_rgb<=draw;video_de<=camera_de;video_hs<=camera_hs;video_vs<=camera_vs;
  end
 end
endmodule
