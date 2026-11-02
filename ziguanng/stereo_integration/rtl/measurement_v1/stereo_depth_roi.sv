// Depth endpoints are planes Z_left=100/200mm in BOTH directions.
// ROI is a 127x63 pixel block. 11x11 template centres lie at local [5..121]x[5..57].
module stereo_depth_roi #(
 parameter integer MATCH_MARGIN_PIXELS=2,
 parameter signed [63:0] MIN_PROJ_DEN_Q30=64'sd67108864
)(input wire clk,rst_n,start,input wire reverse_direction,input wire [9:0] pixel_u,pixel_v,
 output reg busy,done,valid,output reg [3:0] reject,output reg [9:0] roi_x,roi_y,
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
 reg span_bad,boundary_bad,left_bad;
 always @*begin coefficient=0;case(direction*18+ep*9+row*3+term)
 0: coefficient=40'sd1074051305;
1: coefficient=-40'sd17990;
2: coefficient=-40'sd210913324473;
3: coefficient=-40'sd4942492;
4: coefficient=40'sd1069015319;
5: coefficient=40'sd7000477936;
6: coefficient=40'sd6031;
7: coefficient=-40'sd10200;
8: coefficient=40'sd1073741824;
9: coefficient=40'sd1073685348;
10: coefficient=-40'sd17984;
11: coefficient=-40'sd117251445847;
12: coefficient=-40'sd4940808;
13: coefficient=40'sd1068651078;
14: coefficient=40'sd7421270963;
15: coefficient=40'sd6029;
16: coefficient=-40'sd10197;
17: coefficient=40'sd1073741824;
18: coefficient=40'sd1073499294;
19: coefficient=40'sd2029985;
20: coefficient=40'sd210852449725;
21: coefficient=40'sd5002396;
22: coefficient=40'sd1079678850;
23: coefficient=-40'sd6056573162;
24: coefficient=-40'sd5982;
25: coefficient=40'sd10245;
26: coefficient=40'sd1073741824;
27: coefficient=40'sd1073869203;
28: coefficient=40'sd1136924;
29: coefficient=40'sd117257497513;
30: coefficient=40'sd5006475;
31: coefficient=40'sd1079518422;
32: coefficient=-40'sd6914494775;
33: coefficient=-40'sd5982;
34: coefficient=40'sd10245;
35: coefficient=40'sd1073741824;
 default:coefficient=0;endcase case(term)0:factor=$signed({1'b0,u});1:factor=$signed({1'b0,v});default:factor=11'sd1;endcase end
 task fail(input [3:0] c);begin busy<=0;done<=1;valid<=0;reject<=c;roi_x<=0;roi_y<=0;endpoint0_x_q16<=0;endpoint0_y_q16<=0;endpoint1_x_q16<=0;endpoint1_y_q16<=0;state<=0;end endtask
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin state<=0;busy<=0;done<=0;valid<=0;reject<=0;roi_x<=0;roi_y<=0;endpoint0_x_q16<=0;endpoint0_y_q16<=0;endpoint1_x_q16<=0;endpoint1_y_q16<=0;direction<=0;ep<=0;idx<=0;row<=0;term<=0;u<=0;v<=0;acc<=0;den<=0;dividend<=0;quot<=0;rem<=0;count<=0;negative<=0;coefficient_q<=0;factor_q<=0;product<=0;mac_phase<=0;minx<=0;maxx<=0;miny<=0;maxy<=0;lx<=0;ly<=0;hx<=0;hy<=0;tx<=0;ty<=0;ox<=0;oy<=0;spanx<=0;spany<=0;sumx<=0;sumy<=0;endx<=0;endy<=0;span_bad<=0;boundary_bad<=0;left_bad<=0;end
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
 7:begin lx<=(minx>>>16)-5-MATCH_MARGIN_PIXELS;ly<=(miny>>>16)-5-MATCH_MARGIN_PIXELS;hx<=((maxx+32'sd65535)>>>16)+5+MATCH_MARGIN_PIXELS;hy<=((maxy+32'sd65535)>>>16)+5+MATCH_MARGIN_PIXELS;state<=8;end
 8:begin spanx<=hx-lx+1;spany<=hy-ly+1;sumx<=lx+hx;sumy<=ly+hy;state<=9;end
 9:begin tx<=(sumx-126)>>>1;ty<=(sumy-62)>>>1;span_bad<=spanx>127||spany>63;boundary_bad<=lx<0||ly<0||hx>639||hy>719;state<=10;end
 10:begin ox<=tx<0 ? 0 : tx>513 ? 513 : tx;oy<=ty<0 ? 0 : ty>657 ? 657 : ty;state<=11;end
 11:begin endx<=ox+126;endy<=oy+62;left_bad<=lx<ox||ly<oy;state<=12;end
 12:if(span_bad)fail(4);
 else if(boundary_bad||left_bad||hx>endx||hy>endy)fail(5);
 else begin roi_x<=ox[9:0];roi_y<=oy[9:0];endpoint0_x_q16<=px[0];endpoint0_y_q16<=px[1];endpoint1_x_q16<=px[2];endpoint1_y_q16<=px[3];busy<=0;done<=1;valid<=1;reject<=0;state<=0;end
 default:fail(3);endcase end end
endmodule
