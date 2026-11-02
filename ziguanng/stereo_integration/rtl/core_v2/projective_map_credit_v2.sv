// Explicit v2 credit-reserved coordinate pipeline: arithmetic advances every clock.
// Reserve one of 128 output slots on each accepted input; release only on output.
// No numeric truncation or timing exceptions. Frame configuration must stay stable.
module projective_map_credit_v2 #(parameter TAG_W=44)(
 input wire clk,rst_n,input wire in_valid,output wire in_ready,
 input wire [11:0] in_x,in_y,input wire [TAG_W-1:0] in_tag,
 input wire [359:0] homography_flat,
 input wire signed [31:0] k1,k2,p1,p2,fx,fy,cx,cy,
 input wire [11:0] width,height,
 output wire out_valid,input wire out_ready,
 output wire signed [31:0] out_x,out_y,output wire out_inside,out_domain_error,
 output wire [TAG_W-1:0] out_tag
);
 localparam W=TAG_W+66;
 reg [7:0] reserved,queued;
 reg [6:0] wp,rp;
 reg head_valid;
 reg [W-1:0] head;
 reg [W-1:0] fifo[0:127];
 wire pv,unused_ready,pi,pe;
 wire signed [31:0] px,py;
 wire [TAG_W-1:0] pt;
 wire accept=in_valid && in_ready;
 wire emit=head_valid && out_ready;
 wire advance=!head_valid || out_ready;
 wire push=pv && (!advance || queued!=0);
 wire pop=advance && queued!=0;
 assign in_ready=reserved<8'd128;
 assign out_valid=head_valid;
 assign {out_x,out_y,out_inside,out_domain_error,out_tag}=head;
 projective_map_operand_v2 #(.TAG_W(TAG_W)) core(
  .clk(clk),.rst_n(rst_n),.in_valid(accept),.in_ready(unused_ready),
  .in_x(in_x),.in_y(in_y),.in_tag(in_tag),.homography_flat(homography_flat),
  .k1(k1),.k2(k2),.p1(p1),.p2(p2),.fx(fx),.fy(fy),.cx(cx),.cy(cy),
  .width(width),.height(height),.out_valid(pv),.out_ready(1'b1),
  .out_x(px),.out_y(py),.out_inside(pi),.out_domain_error(pe),.out_tag(pt));
 // The payload RAM is not reset: only valid/occupancy owns its contents.
 always @(posedge clk) begin
  if(push) fifo[wp]<={px,py,pi,pe,pt};
  if(advance) begin
   if(queued!=0) head<=fifo[rp];
   else if(pv) head<={px,py,pi,pe,pt};
  end
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin reserved<=0;queued<=0;wp<=0;rp<=0;head_valid<=0;end
  else begin
   case({accept,emit})
    2'b10:reserved<=reserved+1'b1;
    2'b01:reserved<=reserved-1'b1;
   endcase
   case({push,pop})
    2'b10:queued<=queued+1'b1;
    2'b01:queued<=queued-1'b1;
   endcase
   if(push) wp<=wp+1'b1;
   if(pop) rp<=rp+1'b1;
   if(advance) head_valid<=(queued!=0)||pv;
  end
 end
endmodule
