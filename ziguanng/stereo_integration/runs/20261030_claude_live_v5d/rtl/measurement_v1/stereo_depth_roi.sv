// claude live_v5b: depth endpoints are planes Z_left=80/500mm in BOTH directions.
// ROI is a 255x31 pixel block for a 21x11 template. The predicted segment box is
// clamped to the image (no reject when part of the depth range leaves the image)
// and returned as the candidate top-left window cand_x0..cand_x1, cand_y0..cand_y1.
// Coefficients: gencoef.py (same method as gen_geometry.py, which reproduces the
// 100/200mm set exactly).
module stereo_depth_roi #(
 parameter integer MATCH_MARGIN_PIXELS=2,
 parameter integer MATCH_MARGIN_Y=3,
 parameter integer SW=255, parameter integer SH=31, parameter integer TW=21, parameter integer TH=11,
 parameter signed [63:0] MIN_PROJ_DEN_Q30=64'sd67108864
)(input wire clk,rst_n,start,input wire reverse_direction,input wire [9:0] pixel_u,pixel_v,
 output reg busy,done,valid,output reg [3:0] reject,output reg [9:0] roi_x,roi_y,
 output reg [7:0] cand_x0,cand_x1,output reg [4:0] cand_y0,cand_y1,
 output reg signed [31:0] endpoint0_x_q16,endpoint0_y_q16,endpoint1_x_q16,endpoint1_y_q16);
 reg [3:0] state;reg direction,ep,idx;reg [1:0] row,term;reg [9:0] u,v;
 reg signed [39:0] coefficient;reg signed [10:0] factor;reg signed [39:0] coefficient_q;reg signed [10:0] factor_q;reg signed [50:0] product;reg [1:0] mac_phase;
 reg signed [63:0] acc,nums[0:1];reg [31:0] den;reg [63:0] dividend,quot;reg [32:0] rem;reg [6:0] count;reg negative;
 reg signed [31:0] px[0:3];
 wire [32:0] trial={rem[31:0],dividend[63]};
 wire [33:0] div_difference={1'b0,trial}-{2'b0,den};wire ge=!div_difference[33];
 wire [32:0] rem_next=ge ? div_difference[32:0] : trial;
 // Each boundary operation is a separate registered stage. Preserve signed32
 // truncation and the v1 arithmetic order exactly; no combinational clamp chain.
 reg signed [31:0] minx,maxx,miny,maxy,lx,ly,hx,hy,tx,ty,ox,oy;
 reg signed [31:0] spanx,spany,sumx,sumy,endx,endy;
 reg span_bad,left_bad;
 always @*begin coefficient=0;case(direction*18+ep*9+row*3+term)
 0: coefficient=40'sd1139706314;
1: coefficient=40'sd1677825;
2: coefficient=-40'sd233488064725;
3: coefficient=-40'sd310375;
4: coefficient=40'sd1128016027;
5: coefficient=-40'sd9084330528;
6: coefficient=40'sd30574;
7: coefficient=-40'sd3688;
8: coefficient=40'sd1073741824;
9: coefficient=40'sd1086211861;
10: coefficient=40'sd1599073;
11: coefficient=-40'sd61793246478;
12: coefficient=-40'sd295807;
13: coefficient=40'sd1075070281;
14: coefficient=40'sd8493634147;
15: coefficient=40'sd29139;
16: coefficient=-40'sd3515;
17: coefficient=40'sd1073741824;
18: coefficient=40'sd1011566866;
19: coefficient=-40'sd785408;
20: coefficient=40'sd219961306946;
21: coefficient=40'sd46372;
22: coefficient=40'sd1028040746;
23: coefficient=40'sd8707762982;
24: coefficient=-40'sd28803;
25: coefficient=40'sd3554;
26: coefficient=40'sd1073741824;
27: coefficient=40'sd1061441970;
28: coefficient=-40'sd1379035;
29: coefficient=40'sd61096305285;
30: coefficient=40'sd519617;
31: coefficient=40'sd1074070182;
32: coefficient=-40'sd8466327873;
33: coefficient=-40'sd28803;
34: coefficient=40'sd3554;
35: coefficient=40'sd1073741824;
 default:coefficient=0;endcase case(term)0:factor=$signed({1'b0,u});1:factor=$signed({1'b0,v});default:factor=11'sd1;endcase end
 task fail(input [3:0] c);begin busy<=0;done<=1;valid<=0;reject<=c;roi_x<=0;roi_y<=0;cand_x0<=0;cand_x1<=0;cand_y0<=0;cand_y1<=0;endpoint0_x_q16<=0;endpoint0_y_q16<=0;endpoint1_x_q16<=0;endpoint1_y_q16<=0;state<=0;end endtask
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin state<=0;busy<=0;done<=0;valid<=0;reject<=0;roi_x<=0;roi_y<=0;endpoint0_x_q16<=0;endpoint0_y_q16<=0;endpoint1_x_q16<=0;endpoint1_y_q16<=0;direction<=0;ep<=0;idx<=0;row<=0;term<=0;u<=0;v<=0;acc<=0;den<=0;dividend<=0;quot<=0;rem<=0;count<=0;negative<=0;coefficient_q<=0;factor_q<=0;product<=0;mac_phase<=0;minx<=0;maxx<=0;miny<=0;maxy<=0;lx<=0;ly<=0;hx<=0;hy<=0;tx<=0;ty<=0;ox<=0;oy<=0;spanx<=0;spany<=0;sumx<=0;sumy<=0;endx<=0;endy<=0;span_bad<=0;left_bad<=0;cand_x0<=0;cand_x1<=0;cand_y0<=0;cand_y1<=0;end
 else begin done<=0;case(state)
 0:if(start)begin busy<=1;valid<=0;reject<=0;roi_x<=0;roi_y<=0;endpoint0_x_q16<=0;endpoint0_y_q16<=0;endpoint1_x_q16<=0;endpoint1_y_q16<=0;
 if(pixel_u>639||pixel_v>719)fail(1);else begin u<=pixel_u;v<=pixel_v;direction<=reverse_direction;ep<=0;row<=0;term<=0;state<=1;end end
 1:begin if(mac_phase==0)begin coefficient_q<=coefficient;factor_q<=factor;mac_phase<=1;end
 else if(mac_phase==1)begin product<=coefficient_q*factor_q;mac_phase<=2;end
 else begin mac_phase<=0;if(term==0)acc<=$signed(product);else acc<=acc+$signed(product);if(term==2)state<=2;else term<=term+1;end end
 2:begin if(row<2)begin nums[row]<=acc;row<=row+1;term<=0;state<=1;end
 else if(acc<MIN_PROJ_DEN_Q30)fail(2);else if(acc>64'sd4294967295)fail(3);else begin den<=acc[31:0];idx<=0;state<=3;end end
 3:if(nums[idx] > 64'sd140737488355327 || nums[idx] < -64'sd140737488355327)fail(3);
 else begin dividend<=(nums[idx][63] ? -nums[idx]:nums[idx])<<16;negative<=nums[idx][63];quot<=0;rem<=0;count<=64;state<=4;end
 4:begin dividend<=dividend<<1;rem<=rem_next;quot<={quot[62:0],ge};count<=count-1;if(count==1)state<=5;end
 5:if(quot>64'd2147483647)fail(3);else begin px[ep*2+idx]<=negative ? -$signed(quot[31:0]):$signed(quot[31:0]);
 if(!idx)begin idx<=1;state<=3;end else if(!ep)begin ep<=1;row<=0;term<=0;state<=1;end else state<=6;end
 6:begin minx<=px[0]<px[2] ? px[0]:px[2];maxx<=px[0]>px[2] ? px[0]:px[2];miny<=px[1]<px[3] ? px[1]:px[3];maxy<=px[1]>px[3] ? px[1]:px[3];state<=7;end
 7:begin lx<=(minx>>>16)-(TW/2)-MATCH_MARGIN_PIXELS;ly<=(miny>>>16)-(TH/2)-MATCH_MARGIN_Y;hx<=((maxx+32'sd65535)>>>16)+(TW/2)+MATCH_MARGIN_PIXELS;hy<=((maxy+32'sd65535)>>>16)+(TH/2)+MATCH_MARGIN_Y;state<=13;end
 13:begin lx<=lx<0 ? 0 : lx;ly<=ly<0 ? 0 : ly;hx<=hx>639 ? 639 : hx;hy<=hy>719 ? 719 : hy;state<=8;end
 8:begin spanx<=hx-lx+1;spany<=hy-ly+1;sumx<=lx+hx;sumy<=ly+hy;state<=9;end
 9:begin tx<=(sumx-(SW-1))>>>1;ty<=(sumy-(SH-1))>>>1;span_bad<=spanx>SW||spany>SH;left_bad<=spanx<TW||spany<TH;state<=10;end
 10:begin ox<=tx<0 ? 0 : tx>640-SW ? 640-SW : tx;oy<=ty<0 ? 0 : ty>720-SH ? 720-SH : ty;state<=11;end
 11:begin endx<=ox+(SW-1);endy<=oy+(SH-1);left_bad<=left_bad||lx<ox||ly<oy;state<=12;end
 12:if(span_bad)fail(4);
 else if(left_bad||hx>endx||hy>endy)fail(5);
 else begin roi_x<=ox[9:0];roi_y<=oy[9:0];cand_x0<=lx-ox;cand_x1<=hx-ox-(TW-1);cand_y0<=ly-oy;cand_y1<=hy-oy-(TH-1);endpoint0_x_q16<=px[0];endpoint0_y_q16<=px[1];endpoint1_x_q16<=px[2];endpoint1_y_q16<=px[3];busy<=0;done<=1;valid<=1;reject<=0;state<=0;end
 default:fail(3);endcase end end
endmodule
