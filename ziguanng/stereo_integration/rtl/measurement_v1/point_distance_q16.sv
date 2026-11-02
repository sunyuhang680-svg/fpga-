// Registered absolute value / square / accumulator stages, exact integer sqrt.
module point_distance_q16(input wire clk,input wire rst_n,input wire start,
 input wire signed [31:0] ax,ay,az,bx,by,bz,
 output reg busy,done,valid,output reg [3:0] reject,
 output reg [31:0] length_mm_q16);
 reg [3:0] state;reg [1:0] axis;
 reg signed [32:0] dx,dy,dz;
 reg [31:0] operand;
 reg [63:0] square;
 reg [65:0] sum;
 reg [63:0] radicand;
 // Post-iteration remainder <2*root+1 <=2^33-1; 34 bits are sufficient.
 reg [33:0] rem;
 reg [31:0] root;
 reg [5:0] count;
 wire [35:0] trial={rem,radicand[63:62]};
 wire [36:0] difference={1'b0,trial}-{3'b0,root,2'b01};
 wire take=!difference[36];
 function [31:0] magnitude(input signed [32:0] v);
 begin magnitude=v[32] ? -v : v;end endfunction
 task fail(input [3:0] c);begin busy<=0;done<=1;valid<=0;reject<=c;length_mm_q16<=0;state<=0;end endtask
 always @(posedge clk or negedge rst_n) begin
 if(!rst_n) begin state<=0;axis<=0;busy<=0;done<=0;valid<=0;reject<=0;length_mm_q16<=0;dx<=0;dy<=0;dz<=0;operand<=0;square<=0;sum<=0;radicand<=0;rem<=0;root<=0;count<=0;end
 else begin done<=0;case(state)
 0:if(start) begin dx<=$signed({ax[31],ax})-$signed({bx[31],bx});dy<=$signed({ay[31],ay})-$signed({by[31],by});dz<=$signed({az[31],az})-$signed({bz[31],bz});busy<=1;valid<=0;reject<=0;length_mm_q16<=0;axis<=0;state<=1;end
 1:begin case(axis)0:operand<=magnitude(dx);1:operand<=magnitude(dy);default:operand<=magnitude(dz);endcase state<=2;end
 2:begin square<=operand*operand;state<=3;end
 3:begin if(axis==0)sum<={2'b0,square};else sum<=sum+{2'b0,square};if(axis==2)state<=4;else begin axis<=axis+1'b1;state<=1;end end
 4:if(sum[65:64]!=0) fail(5); else begin radicand<=sum[63:0];rem<=0;root<=0;count<=32;state<=5;end
 5:begin radicand<=radicand<<2;rem<=take ? difference[33:0] : trial[33:0];root<={root[30:0],take};count<=count-1;if(count==1) state<=6;end
 6:begin length_mm_q16<=root;valid<=1;done<=1;busy<=0;reject<=0;state<=0;end
 default:fail(5);endcase end end
endmodule
