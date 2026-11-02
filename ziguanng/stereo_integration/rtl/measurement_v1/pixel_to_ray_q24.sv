// Static undistorted output pixels only; same Q30 coefficients as active left_ki/right_ki.
module pixel_to_ray_q24(input wire clk,rst_n,start,input wire right_eye,
 input wire [9:0] pixel_u,pixel_v,output reg busy,done,valid,output reg [3:0] reject,
 output reg signed [31:0] ray_x_q24,ray_y_q24);
 reg [1:0] state;reg eye,row;reg [1:0] term;reg [9:0] u,v;
 reg signed [39:0] coefficient;reg signed [10:0] factor;
 reg signed [39:0] coefficient_q;reg signed [10:0] factor_q;reg signed [50:0] product;reg [1:0] mac_phase;reg signed [63:0] acc;
 wire signed [63:0] scaled=acc>>>6;
 always @* begin coefficient=0;case(eye*6+row*3+term)
 0: coefficient=40'sd1855861;
1: coefficient=40'sd0;
2: coefficient=-40'sd623761588;
3: coefficient=40'sd0;
4: coefficient=40'sd1245620;
5: coefficient=-40'sd445208690;
6: coefficient=40'sd1857136;
7: coefficient=40'sd0;
8: coefficient=-40'sd587355621;
9: coefficient=40'sd0;
10: coefficient=40'sd1245870;
11: coefficient=-40'sd442072310;
 default:coefficient=0;endcase case(term)0:factor=$signed({1'b0,u});1:factor=$signed({1'b0,v});default:factor=11'sd1;endcase end
 task fail(input [3:0] c);begin busy<=0;done<=1;valid<=0;reject<=c;ray_x_q24<=0;ray_y_q24<=0;state<=0;end endtask
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin state<=0;busy<=0;done<=0;valid<=0;reject<=0;ray_x_q24<=0;ray_y_q24<=0;acc<=0;u<=0;v<=0;eye<=0;row<=0;term<=0;coefficient_q<=0;factor_q<=0;product<=0;mac_phase<=0;end
 else begin done<=0;case(state)
 0:if(start)begin busy<=1;valid<=0;reject<=0;ray_x_q24<=0;ray_y_q24<=0;
 if(pixel_u>639||pixel_v>719)fail(1);else begin u<=pixel_u;v<=pixel_v;eye<=right_eye;row<=0;term<=0;state<=1;end end
 1:begin if(mac_phase==0)begin coefficient_q<=coefficient;factor_q<=factor;mac_phase<=1;end
 else if(mac_phase==1)begin product<=coefficient_q*factor_q;mac_phase<=2;end
 else begin mac_phase<=0;if(term==0)acc<=$signed(product);else acc<=acc+$signed(product);if(term==2)state<=2;else term<=term+1;end end
 2:if(scaled < -64'sd2147483648 || scaled > 64'sd2147483647)fail(2);else if(!row)begin ray_x_q24<=scaled[31:0];row<=1;term<=0;state<=1;end else begin ray_y_q24<=scaled[31:0];busy<=0;done<=1;valid<=1;reject<=0;state<=0;end
 default:fail(2);endcase end end
endmodule
