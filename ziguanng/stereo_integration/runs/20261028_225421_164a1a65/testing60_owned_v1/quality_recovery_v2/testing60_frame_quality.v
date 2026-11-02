`timescale 1ns/1ps
// Per-bank validity requires BOTH a complete checked camera frame AND the
// memory writer's final transaction completion. Invalid raw banks cannot be
// leased to an upper renderer/measurement session.
module testing60_frame_quality(
 input wire camera_clk,core_clk,rst_n,
 input wire frame_boundary,frame_start,frame_complete,frame_abort,
 input wire [1:0] write_index,
 input wire write_req_ack,ddr_write_finish,
 output wire [3:0] valid_core,
 output wire [3:0] valid_camera
);
 // Assert reset asynchronously, release it through the owning clock domain.
 (* ASYNC_REG="TRUE" *) reg [1:0] camera_reset_sync,core_reset_sync;
 always @(posedge camera_clk or negedge rst_n)begin
  if(!rst_n)camera_reset_sync<=0;
  else camera_reset_sync<={camera_reset_sync[0],1'b1};
 end
 always @(posedge core_clk or negedge rst_n)begin
  if(!rst_n)core_reset_sync<=0;
  else core_reset_sync<={core_reset_sync[0],1'b1};
 end
 wire camera_rst_n=camera_reset_sync[1];
 wire core_rst_n=core_reset_sync[1];
 reg [1:0] memory_index;
 reg finish_previous,finish_event;
 reg [1:0] finished_index;
 // The mature writer acknowledges while its reset is held, and uses the same
 // stable index at that boundary. Retain it for the entire DDR transaction.
 always @(posedge core_clk or negedge core_rst_n)begin
  if(!core_rst_n)begin memory_index<=0;finish_previous<=0;finish_event<=0;finished_index<=0;end
  else begin
   finish_previous<=ddr_write_finish;
   finish_event<=ddr_write_finish && !finish_previous;
   if(write_req_ack)memory_index<=write_index;
   if(ddr_write_finish && !finish_previous)finished_index<=memory_index;
  end
 end
 wire finish_camera;wire [1:0] done_index_camera;
 stereo_event_cdc #(.WIDTH(2)) finish_cdc(
  .src_clk(core_clk),.dst_clk(camera_clk),.rst_n(rst_n),
  .src_event(finish_event),.src_data(finished_index),
  .dst_event(finish_camera),.dst_data(done_index_camera),.src_ready());
 reg [3:0] boundary_delay;
 reg [1:0] active_index;
 reg [3:0] pixels_ok,memory_ok;
 reg [3:0] published_previous;
 assign valid_camera=pixels_ok & memory_ok;
 always @(posedge camera_clk or negedge camera_rst_n)begin
  if(!camera_rst_n)begin boundary_delay<=0;active_index<=0;pixels_ok<=0;memory_ok<=0;published_previous<=0;end
  else begin
   boundary_delay<={boundary_delay[2:0],frame_boundary};
   published_previous<=valid_camera;
   if(finish_camera)memory_ok[done_index_camera]<=1;
   if(frame_complete)pixels_ok[active_index]<=1;
   if(frame_abort)pixels_ok[active_index]<=0;
   // The old gen rotates its index two clocks after CP-derived VS. Wait four
   // clocks before latching, and revoke that slot before any FIFO data is sent.
   if(boundary_delay[3])begin
    active_index<=write_index;
    pixels_ok[write_index]<=0;memory_ok[write_index]<=0;
   end
  end
 end
 stereo_event_cdc #(.WIDTH(4),.INITIAL_PUBLISH(1)) quality_cdc(
  .src_clk(camera_clk),.dst_clk(core_clk),.rst_n(rst_n),
  .src_event(valid_camera!=published_previous),.src_data(valid_camera),
  .dst_event(),.dst_data(valid_core),.src_ready());
endmodule

