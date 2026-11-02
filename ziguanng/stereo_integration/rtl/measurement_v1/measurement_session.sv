// One frozen pair per measurement session. All signals are core_clk domain.
module measurement_session #(
 parameter [31:0] TIMEOUT_CYCLES=32'd3000000000,
 parameter [24:0] LEFT_BASE0=25'd0,LEFT_BASE1=25'd0,RIGHT_BASE0=25'd0,RIGHT_BASE1=25'd0
)(
 input wire clk,rst_n,active,input wire source_available,
 input wire left_idle,right_idle,input wire [1:0] left_pub,right_pub,
 input wire left_fault,right_fault,input wire display_ready,display_fault,
 output wire force_static,allow_left,allow_right,
 output reg locked,failed,output reg [31:0] frame_tag,
 output reg [24:0] left_base,right_base,output reg [1:0] locked_left_pub,locked_right_pub
);
 localparam OFF=0,WAIT_IDLE=1,CONFIG=2,LAUNCH=3,LOCKED=4,FAILED=5;
 reg [2:0] state;reg seen_left,seen_right;reg [1:0] baseline_left,baseline_right;
 reg [31:0] timer;
 assign force_static=(state==CONFIG || state==LAUNCH || state==LOCKED || state==FAILED);
 // Before active arrives in core domain normal jobs are uncontrolled; after
 // it arrives no new jobs start until both current jobs drain.
 assign allow_left=(!active && state==OFF) || (active && state==LAUNCH && !seen_left);
 assign allow_right=(!active && state==OFF) || (active && state==LAUNCH && !seen_right);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=OFF;seen_left<=0;seen_right<=0;baseline_left<=0;baseline_right<=0;
   timer<=0;locked<=0;failed<=0;frame_tag<=0;left_base<=0;right_base<=0;locked_left_pub<=0;locked_right_pub<=0;
  end else if(!active)begin
   state<=OFF;seen_left<=0;seen_right<=0;timer<=0;locked<=0;failed<=0;
  end else begin
   if(state!=FAILED && !(state==LOCKED && display_ready)) timer<=timer+1'b1;
   if((state!=FAILED && !(state==LOCKED && display_ready) && timer>=TIMEOUT_CYCLES) || left_fault || right_fault || display_fault)begin
    state<=FAILED;locked<=0;failed<=1;
   end else case(state)
    OFF:begin state<=WAIT_IDLE;timer<=0;locked<=0;failed<=0;end
    WAIT_IDLE:if(left_idle && right_idle && source_available)begin
     baseline_left<=left_pub;baseline_right<=right_pub;seen_left<=0;seen_right<=0;state<=CONFIG;
    end
    // Provides a complete core cycle for forced configuration to settle.
    CONFIG:state<=LAUNCH;
    LAUNCH:begin
     if(!left_idle)seen_left<=1;
     if(!right_idle)seen_right<=1;
     if(seen_left && seen_right && left_idle && right_idle && left_pub[1] && right_pub[1] &&
        left_pub!=baseline_left && right_pub!=baseline_right)begin
      locked_left_pub<=left_pub;locked_right_pub<=right_pub;
      left_base<=left_pub[0]?LEFT_BASE1:LEFT_BASE0;right_base<=right_pub[0]?RIGHT_BASE1:RIGHT_BASE0;
      frame_tag<=frame_tag+1'b1;locked<=1;state<=LOCKED;
     end
    end
    LOCKED:if(left_pub!=locked_left_pub || right_pub!=locked_right_pub || !left_idle || !right_idle)begin
      // Unexpected mutation voids the lock immediately; never expose its base.
      locked<=0;failed<=1;state<=FAILED;
    end
    FAILED:begin locked<=0;failed<=1;end
    default:begin locked<=0;failed<=1;state<=FAILED;end
   endcase
  end
 end
endmodule
