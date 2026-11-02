`timescale 1ns/1ps
// Camera-domain held-bank identity crosses as a coherent mailbox, never a
// separately synchronized multi-bit camera index.
module testing60_photo_hold_mailbox(
 input wire camera_clk,pixel_clk,rst_n,ui_hold_active,
 input wire [3:0] completed_valid,input wire [1:0] completed_index,
 output wire held_valid,output wire [1:0] held_index
);
 wire [2:0] current={ui_hold_active && completed_valid[completed_index],completed_index};
 (* ASYNC_REG="TRUE" *) reg [1:0] camera_reset;
 always @(posedge camera_clk or negedge rst_n)
  if(!rst_n)camera_reset<=0;else camera_reset<={camera_reset[0],1'b1};
 reg [2:0] previous;
 always @(posedge camera_clk or negedge camera_reset[1])
  if(!camera_reset[1])previous<=0;else previous<=current;
 stereo_event_cdc #(.WIDTH(3),.INITIAL_PUBLISH(1)) mailbox(
  .src_clk(camera_clk),.dst_clk(pixel_clk),.rst_n(rst_n),
  .src_event(current!=previous),.src_data(current),.src_ready(),
  .dst_event(),.dst_data({held_valid,held_index}));
endmodule

// Preserve the proven forward bridge. Return the PUBLIC native-reader
// accepted identity together with that request's base and requested identity.
// Pixel ACK is withheld until this return mailbox has arrived. Thus no ACK
// can validate a stale identity, including same tag/index at another base.
module testing60_photo_descriptor_bridge # (parameter ADDR_BITS=25)(
 input wire pixel_clk,mem_clk,rst_n,pixel_req,
 output wire pixel_ack,
 input wire [ADDR_BITS-1:0] pixel_base,input wire [1:0] pixel_index,input wire pixel_tag,
 output wire mem_req,input wire mem_ack,
 output wire [ADDR_BITS-1:0] mem_base,output wire [1:0] mem_index,output wire mem_tag,
 input wire [1:0] native_index,input wire native_tag,
 output reg [ADDR_BITS+2:0] accepted_descriptor,
 output reg accepted_matches_request
);
 wire bridge_ack;
 reg forward_request;
 reg [ADDR_BITS+2:0] forward_descriptor;
 display_read_descriptor_bridge #(.ADDR_BITS(ADDR_BITS)) forward(
  .pixel_clk(pixel_clk),.mem_clk(mem_clk),.rst_n(rst_n),
  .pixel_req(forward_request),.pixel_ack(bridge_ack),.pixel_base(forward_descriptor[ADDR_BITS-1:0]),
  .pixel_index(forward_descriptor[ADDR_BITS+1:ADDR_BITS]),.pixel_tag(forward_descriptor[ADDR_BITS+2]),
  .mem_req(mem_req),.mem_ack(mem_ack),.mem_base(mem_base),.mem_index(mem_index),.mem_tag(mem_tag));
 (* ASYNC_REG="TRUE" *) reg [1:0] pixel_reset,memory_reset;
 always @(posedge pixel_clk or negedge rst_n)
  if(!rst_n)pixel_reset<=0;else pixel_reset<={pixel_reset[0],1'b1};
 always @(posedge mem_clk or negedge rst_n)
  if(!rst_n)memory_reset<=0;else memory_reset<={memory_reset[0],1'b1};
 reg previous_mem_ack;
 always @(posedge mem_clk or negedge memory_reset[1])
  if(!memory_reset[1])previous_mem_ack<=0;else previous_mem_ack<=mem_ack;
 wire return_event;wire [ADDR_BITS+5:0] returned;
 stereo_event_cdc #(.WIDTH(ADDR_BITS+6)) response(
  .src_clk(mem_clk),.dst_clk(pixel_clk),.rst_n(rst_n),
  .src_event(mem_req && mem_ack && !previous_mem_ack),
  .src_data({mem_tag,mem_index,native_tag,native_index,mem_base}),
  .src_ready(),.dst_event(return_event),.dst_data(returned));
 // Serialize the entire inner four-phase exchange, INCLUDING its return
 // mailbox. Cancel drops the inner request but drains ACK high, metadata,
 // then ACK low before admitting a reshoot (even at the identical address).
 localparam IDLE=0,ACTIVE=1,DRAIN=2;
 reg [1:0] state;
 reg return_valid,ack_seen;
 always @(posedge pixel_clk or negedge pixel_reset[1])begin
  if(!pixel_reset[1])begin
   state<=IDLE;forward_request<=0;forward_descriptor<=0;return_valid<=0;ack_seen<=0;
   accepted_descriptor<=0;accepted_matches_request<=0;
  end
  else begin
   if(state!=IDLE && bridge_ack)ack_seen<=1;
   if(state!=IDLE && return_event)begin
    accepted_descriptor<=returned[ADDR_BITS+2:0];
    accepted_matches_request<=returned[ADDR_BITS+5:ADDR_BITS+3]==returned[ADDR_BITS+2:ADDR_BITS];
    return_valid<=1;
   end
   case(state)
    IDLE:if(pixel_req)begin
     state<=ACTIVE;forward_request<=1;return_valid<=0;ack_seen<=0;accepted_matches_request<=0;
     forward_descriptor<={pixel_tag,pixel_index,pixel_base};
    end
    ACTIVE:if(!pixel_req)begin forward_request<=0;state<=DRAIN;end
    DRAIN:if(ack_seen && return_valid && !bridge_ack)begin state<=IDLE;return_valid<=0;end
    default:begin state<=IDLE;forward_request<=0;return_valid<=0;ack_seen<=0;end
   endcase
  end
 end
 assign pixel_ack=(state==ACTIVE) && pixel_req && bridge_ack && return_valid;
endmodule
