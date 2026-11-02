`timescale 1ns/1ps
// Latest complete state, held through a toggle request/acknowledgment exchange.
// Events are accepted after src_ready; INITIAL_PUBLISH seeds state after reset.
module stereo_event_cdc #(parameter WIDTH=66,SYNC_RESET=1,INITIAL_PUBLISH=0)(
 input wire src_clk,dst_clk,rst_n,src_event,
 input wire[WIDTH-1:0]src_data,
 output reg dst_event,output reg[WIDTH-1:0]dst_data,
 output wire src_ready
);
 wire source_rst_n,destination_rst_n;
 generate if(SYNC_RESET!=0)begin:reset_domains
  (* ASYNC_REG="TRUE" *)reg[1:0]source_reset_sync,destination_reset_sync;
  always @(posedge src_clk or negedge rst_n)begin
   if(!rst_n)source_reset_sync<=0;else source_reset_sync<={source_reset_sync[0],1'b1};
  end
  always @(posedge dst_clk or negedge rst_n)begin
   if(!rst_n)destination_reset_sync<=0;else destination_reset_sync<={destination_reset_sync[0],1'b1};
  end
  assign source_rst_n=source_reset_sync[1];assign destination_rst_n=destination_reset_sync[1];
 end else begin:original_reset_contract
  assign source_rst_n=rst_n;assign destination_rst_n=rst_n;
 end endgenerate
 assign src_ready=source_rst_n;
 reg[WIDTH-1:0]latest,held;
 reg dirty,request,acknowledge,initial_pending;
 (* ASYNC_REG="TRUE" *)reg ack_meta,ack_sync,req_meta,req_sync;
 always @(posedge src_clk or negedge source_rst_n)begin
  if(!source_rst_n)begin
   latest<=0;held<=0;dirty<=0;request<=0;ack_meta<=0;ack_sync<=0;initial_pending<=INITIAL_PUBLISH!=0;
  end else begin
   ack_meta<=acknowledge;ack_sync<=ack_meta;
   if(request==ack_sync && dirty)begin held<=latest;request<=~request;dirty<=0;end
   // The seed and source event both capture current stable source-domain data.
   if(initial_pending || src_event)begin latest<=src_data;dirty<=1;initial_pending<=0;end
  end
 end
 always @(posedge dst_clk or negedge destination_rst_n)begin
  if(!destination_rst_n)begin req_meta<=0;req_sync<=0;acknowledge<=0;dst_data<=0;dst_event<=0;end
  else begin
   req_meta<=request;req_sync<=req_meta;dst_event<=0;
   if(req_sync!=acknowledge)begin dst_data<=held;dst_event<=1;acknowledge<=req_sync;end
  end
 end
endmodule
