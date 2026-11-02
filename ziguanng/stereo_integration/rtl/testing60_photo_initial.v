// INITIAL-QUALIFICATION-ONLY CANDIDATE; production editing uses legacy descriptor flow after SHOW.
`timescale 1ns/1ps
// Candidate only. All inputs are pixel-clock synchronous, coherent mailboxes.
// A lease means checked full-frame camera pixels AND final DDR write completed,
// with both banks reserved until lease_release. Existing {render_tag,index}
// is sufficient ONLY while the existing lease prevents bank reuse.
module testing60_photo_initial #(
 parameter integer ID_BITS=3,
 parameter integer PIXELS_PER_EYE=460800,
 parameter integer TIMEOUT_FRAMES=12
)(
 input wire clk,rst_n,photo_request,retry,memory_ready,configuration_change,
 input wire frame_start,frame_end,active_video_start,
 input wire lease_valid,input wire [ID_BITS-1:0] lease_left,lease_right,
 input wire consumer_req_left,consumer_req_right,
 output wire reader_req_left,reader_req_right,
 input wire reader_ack_left,reader_ack_right,
 input wire [ID_BITS-1:0] accepted_left,accepted_right,
 // qualified must follow this request's FIFO clear and actual fill, not a
 // stale high level. Feed an event/handshake monitor, never raw !empty alone.
 input wire fifo_qualified_left,fifo_qualified_right,
 input wire pop_left,pop_right,empty_left,empty_right,
 output reg [ID_BITS-1:0] selected_left,selected_right,
 output reg frozen_active,lease_release,failed,
 output wire busy,photo_hold,legacy_passthrough,output wire [2:0] status
);
 localparam IDLE=0,WAIT_LEASE=1,ARM=2,PROBE=3,READY=4,SHOW=5,FAULT=6,DRAIN=7,WAIT_DDR=8;
 reg [3:0] state;
 reg request_previous,retry_previous,retry_pending,lease_owned,configuration_pending;
 reg [31:0] age,left_count,right_count;
 reg left_sent,right_sent,left_ok,right_ok,frame_bad;
 reg previous_reader_req_left,previous_reader_req_right;
 reg [1:0] pending_left,pending_right;
 wire cancel=!photo_request;
 wire start_new=(photo_request&&!request_previous)||(retry&&!retry_previous);
 wire identity_valid=lease_valid;
 wire descriptor_changed=(lease_left!=selected_left || lease_right!=selected_right);
 wire enabled=(state==PROBE || state==READY || state==SHOW) && identity_valid && memory_ready;
 assign reader_req_left=(state==SHOW || enabled) && consumer_req_left;
 assign reader_req_right=(state==SHOW || enabled) && consumer_req_right;
 assign legacy_passthrough=(state==SHOW);
 assign busy=(state!=IDLE && state!=FAULT && state!=SHOW);
 assign photo_hold=photo_request || lease_owned;
 // 0 IDLE, 1 WAIT_DDR, 2 WAIT_FRAME, 3 WAIT_READ, 4 READY, 5 FAULT.
 assign status=failed || state==FAULT ? 3'd5 : state==IDLE?3'd0 :
  state==WAIT_DDR?3'd1 : (state==WAIT_LEASE || state==ARM || state==DRAIN)?3'd2 :
  state==PROBE?3'd3:3'd4;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;request_previous<=0;retry_previous<=0;retry_pending<=0;lease_owned<=0;configuration_pending<=0;age<=0;
   selected_left<=0;selected_right<=0;frozen_active<=0;
   lease_release<=0;failed<=0;left_sent<=0;right_sent<=0;
   left_ok<=0;right_ok<=0;left_count<=0;right_count<=0;
   frame_bad<=0;pending_left<=0;pending_right<=0;
   previous_reader_req_left<=0;previous_reader_req_right<=0;
  end else begin
   request_previous<=photo_request;retry_previous<=retry;
   if(retry&&!retry_previous)retry_pending<=1;
   lease_release<=0;
   if(state==SHOW)configuration_pending<=0;
   else if(configuration_change)configuration_pending<=1;
   previous_reader_req_left<=reader_req_left;
   previous_reader_req_right<=reader_req_right;
   // ACKs are sampled only after the corresponding request was asserted.
   // Correct identities and per-request FIFO qualification are both required.
   if(reader_req_left && !previous_reader_req_left)begin left_sent<=1;left_ok<=0;pending_left<=0;end
   else begin
    if(left_sent && reader_ack_left && accepted_left==selected_left)pending_left[0]<=1;
    if(pending_left[0] && !reader_ack_left)pending_left[1]<=1;
    if(pending_left[1] && fifo_qualified_left)left_ok<=1;
   end
   if(reader_req_right && !previous_reader_req_right)begin right_sent<=1;right_ok<=0;pending_right<=0;end
   else begin
    if(right_sent && reader_ack_right && accepted_right==selected_right)pending_right[0]<=1;
    if(pending_right[0] && !reader_ack_right)pending_right[1]<=1;
    if(pending_right[1] && fifo_qualified_right)right_ok<=1;
   end
   if((enabled || state==SHOW) && pop_left)begin
    if(empty_left || (state!=SHOW && !left_ok) || left_count>=PIXELS_PER_EYE)frame_bad<=1;
    else left_count<=left_count+1'b1;
   end
   if((enabled || state==SHOW) && pop_right)begin
    if(empty_right || (state!=SHOW && !right_ok) || right_count>=PIXELS_PER_EYE)frame_bad<=1;
    else right_count<=right_count+1'b1;
   end
   case(state)
    IDLE: if(start_new)begin state<=memory_ready?WAIT_LEASE:WAIT_DDR;failed<=0;age<=0;end
    WAIT_DDR: if(memory_ready)state<=WAIT_LEASE;
    WAIT_LEASE: if(lease_valid)begin
     selected_left<=lease_left;selected_right<=lease_right;state<=ARM;lease_owned<=1;
    end
    ARM: if(frame_start)begin
     state<=PROBE;left_sent<=0;right_sent<=0;left_ok<=0;right_ok<=0;
     pending_left<=0;pending_right<=0;left_count<=0;right_count<=0;frame_bad<=0;
    end
    PROBE: if(frame_end)begin
     if(identity_valid && left_ok && right_ok && !frame_bad &&
        left_count==PIXELS_PER_EYE && right_count==PIXELS_PER_EYE)state<=READY;
     else state<=ARM;
    end
    READY: if(frame_start)begin
     // A whole trial frame was read and checked before this boundary.
     frozen_active<=1;state<=SHOW;
     left_sent<=0;right_sent<=0;left_ok<=0;right_ok<=0;
     pending_left<=0;pending_right<=0;left_count<=0;right_count<=0;frame_bad<=0;
    end
    SHOW: begin
     selected_left<=lease_left;selected_right<=lease_right;
     if(!lease_valid || !memory_ready)failed<=1;
     if(frame_end && (frame_bad || left_count!=PIXELS_PER_EYE || right_count!=PIXELS_PER_EYE))begin
      failed<=1;
     end
     if(frame_start)begin
      left_sent<=0;right_sent<=0;left_ok<=0;right_ok<=0;
      pending_left<=0;pending_right<=0;left_count<=0;right_count<=0;frame_bad<=0;
     end
    end
    FAULT: if(frame_start)begin if(!lease_owned)frozen_active<=0;failed<=1;state<=frozen_active?SHOW:IDLE;end
    DRAIN: if(!lease_valid)state<=WAIT_LEASE;
   endcase
   if(frame_start && state!=IDLE && state!=SHOW && state!=FAULT)begin
    if(age>=TIMEOUT_FRAMES-1)begin state<=FAULT;failed<=1;end
    else age<=age+1'b1;
   end
   // Lease disappearance or replacement revokes all future read requests.
   if(state>=ARM && state<SHOW && (!identity_valid || !memory_ready))begin state<=FAULT;failed<=1;end
   // Existing raw/render publication selection may change after processing or
   // a UI transform. Retain the raw-photo lease; latch the new pair atomically
   // at blanking, return to live and qualify its actual reader identity anew.
   if(frame_start && state>=ARM && state<SHOW && identity_valid && memory_ready && descriptor_changed)begin
    selected_left<=lease_left;selected_right<=lease_right;frozen_active<=0;
    state<=PROBE;age<=0;left_sent<=0;right_sent<=0;left_ok<=0;right_ok<=0;
    pending_left<=0;pending_right<=0;left_count<=0;right_count<=0;frame_bad<=0;
   end
   // Initial trial must be restarted even if a configuration change keeps
   // the same camera bank/tag. After SHOW, legacy atomic descriptors own it.
   if(frame_start && state!=SHOW && photo_request && (configuration_pending || configuration_change))begin
    state<=memory_ready?WAIT_LEASE:WAIT_DDR;frozen_active<=0;
    configuration_pending<=0;age<=0;failed<=0;
    left_sent<=0;right_sent<=0;left_ok<=0;right_ok<=0;
    pending_left<=0;pending_right<=0;left_count<=0;right_count<=0;frame_bad<=0;
   end
   // Only UI cancel releases the held raw photograph. Read timeout, DDR loss,
   // renderer reconfiguration and retry retain it for inspection/retry.
   if(frame_start && (cancel || retry_pending || (retry&&!retry_previous)))begin
    if(cancel || !lease_owned)frozen_active<=0;lease_release<=cancel&&lease_owned;
    if(cancel)lease_owned<=0;
    state<=cancel?IDLE:(memory_ready?WAIT_LEASE:WAIT_DDR);age<=0;failed<=0;retry_pending<=0;
   end
  end
 end
endmodule

// Drop-in candidate display mux plus qualifier. It owns the selection flop,
// so lease_release and removal of the old photo happen on the SAME boundary.
// Do not cascade this frozen_active through the old selector's extra latch.
module testing60_photo_initial_select #(
 parameter integer ID_BITS=28,READER_DELAY=10,PIXELS_PER_EYE=460800,TIMEOUT_FRAMES=12
)(
 input wire clk,rst_n,photo_request,retry,memory_ready,configuration_change,
 input wire direct_hs,direct_vs,direct_de,input wire [23:0] direct_rgb,
 input wire frozen_hs,frozen_vs,frozen_de,input wire [15:0] frozen_rgb565,
 input wire pair_valid,input wire [ID_BITS-1:0] pair_left,pair_right,
 input wire consumer_req_left,consumer_req_right,
 output wire reader_req_left,reader_req_right,
 input wire reader_ack_left,reader_ack_right,
 input wire [ID_BITS-1:0] accepted_left,accepted_right,
 input wire [12:0] level_left,level_right,
 input wire pop_left,pop_right,empty_left,empty_right,
 output wire [ID_BITS-1:0] selected_left,selected_right,
 output wire hs,vs,de,output wire [23:0] rgb,
 output wire frozen_active,lease_release,failed,busy,photo_hold,legacy_passthrough,output wire [2:0] status
);
 reg [READER_DELAY-1:0] dhs,dvs,dde;
 reg [23:0] colors[0:READER_DELAY-1];
 reg prev_vs;integer i;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   dhs<=0;dvs<=0;dde<=0;prev_vs<=0;
   for(i=0;i<READER_DELAY;i=i+1)colors[i]<=0;
  end else begin
   dhs<={dhs[READER_DELAY-2:0],direct_hs};
   dvs<={dvs[READER_DELAY-2:0],direct_vs};
   dde<={dde[READER_DELAY-2:0],direct_de};
   colors[0]<=direct_rgb;
   for(i=1;i<READER_DELAY;i=i+1)colors[i]<=colors[i-1];
   prev_vs<=dvs[READER_DELAY-1];
  end
 end
 assign hs=dhs[READER_DELAY-1];assign vs=dvs[READER_DELAY-1];assign de=dde[READER_DELAY-1];
 wire boundary=vs&&!prev_vs;
 // Sample the incoming first active pixel at the same edge as the timing
 // pipeline presents it, so a failed preflight cannot leak a first bad pixel.
 reg frame_de_seen;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)frame_de_seen<=0;
  else if(boundary)frame_de_seen<=0;
  else if(dde[READER_DELAY-2])frame_de_seen<=1;
 end
 wire active_video_start=dde[READER_DELAY-2]&&!frame_de_seen;
 wire fifo_qualified_left,fifo_qualified_right;
 testing60_photo_initial_reader_ready left_ready(.clk(clk),.rst_n(rst_n),.request(reader_req_left),
 .ack(reader_ack_left),.identity_ok(accepted_left==selected_left),.empty(empty_left),.level(level_left),.qualified(fifo_qualified_left));
 testing60_photo_initial_reader_ready right_ready(.clk(clk),.rst_n(rst_n),.request(reader_req_right),
 .ack(reader_ack_right),.identity_ok(accepted_right==selected_right),.empty(empty_right),.level(level_right),.qualified(fifo_qualified_right));
 testing60_photo_initial #(.ID_BITS(ID_BITS),.PIXELS_PER_EYE(PIXELS_PER_EYE),.TIMEOUT_FRAMES(TIMEOUT_FRAMES)) control(
 .clk(clk),.rst_n(rst_n),.photo_request(photo_request),.retry(retry),.memory_ready(memory_ready),.configuration_change(configuration_change),.frame_start(boundary),.frame_end(boundary),.active_video_start(active_video_start),
 .lease_valid(pair_valid),.lease_left(pair_left),.lease_right(pair_right),
 .consumer_req_left(consumer_req_left),.consumer_req_right(consumer_req_right),
 .reader_req_left(reader_req_left),.reader_req_right(reader_req_right),
 .reader_ack_left(reader_ack_left),.reader_ack_right(reader_ack_right),
 .accepted_left(accepted_left),.accepted_right(accepted_right),
 .fifo_qualified_left(fifo_qualified_left),.fifo_qualified_right(fifo_qualified_right),
 .pop_left(pop_left),.pop_right(pop_right),.empty_left(empty_left),.empty_right(empty_right),
 .selected_left(selected_left),.selected_right(selected_right),.frozen_active(frozen_active),
 .lease_release(lease_release),.failed(failed),.busy(busy),.photo_hold(photo_hold),.legacy_passthrough(legacy_passthrough),.status(status));
 wire aligned=(frozen_hs==hs)&&(frozen_vs==vs)&&(frozen_de==de);
 assign rgb=!frozen_active?colors[READER_DELAY-1]:
  ((aligned&&frozen_de)?{frozen_rgb565[15:11],3'b0,frozen_rgb565[10:5],2'b0,frozen_rgb565[4:0],3'b0}:24'd0);
endmodule

// Observe a fresh request, accepted descriptor, ACK release, and actual FIFO
// clear then prefill. A stale full FIFO / matching old index is insufficient.
module testing60_photo_initial_reader_ready #(parameter PREFILL_PIXELS=1024)(
 input wire clk,rst_n,request,ack,identity_ok,empty,input wire [12:0] level,
 output reg qualified
);
 reg previous_request,armed,acked,ack_released,empty_seen;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin previous_request<=0;armed<=0;acked<=0;ack_released<=0;empty_seen<=0;qualified<=0;end
  else begin
   previous_request<=request;
   if(request&&!previous_request)begin
    armed<=1;acked<=0;ack_released<=0;empty_seen<=empty;qualified<=0;
   end else if(armed)begin
    if(empty)empty_seen<=1;
    if(ack && identity_ok)acked<=1;
    if(acked&&!ack)ack_released<=1;
    if(acked && ack_released && empty_seen && identity_ok && !empty && level>=PREFILL_PIXELS)qualified<=1;
    if(!identity_ok)qualified<=0;
   end
  end
 end
endmodule
