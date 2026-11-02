`timescale 1ns/1ps
// New orchestration only: mature renderer/core are instantiated unchanged.
// Correct -> commit scratch -> existing pan/zoom/edit renderer -> display ping-pong.
module stereo_algorithm_view #(
    parameter ADDR_BITS=25,BUSRT_BITS=10,
    parameter [ADDR_BITS-1:0] OUTPUT_BASE0=0,OUTPUT_BASE1=28800,
    parameter FRAME_HEIGHT=720,TEST_PATTERN=0,USE_AFFINE=1,ENABLED=0,
    parameter MIN_STEP_X=16384,
    parameter EXTERNAL_GAMMA_ROM=0
)(
    input wire mem_clk,rst,source_valid,
    input wire[ADDR_BITS-1:0]source_base,
    input wire[1:0]scale_shift,
    input wire[11:0]source_left,source_top,source_width,source_height,destination_left,destination_top,
    input wire apply_binary,input wire[47:0]adjust_values,input wire[3:0]doodle_index,
    output wire[16:0]doodle_address,
    input wire[16:0]view_step_q16,view_step_x_q16,input wire[1:0]view_rotation,
    input wire signed[27:0]view_origin_x,view_origin_y,
    input wire display_index_async,display_accept_valid_async,
    output wire front_index,frame_ready,
    output wire rd_burst_req,output wire[BUSRT_BITS-1:0]rd_burst_len,output wire[ADDR_BITS-1:0]rd_burst_addr,
    input wire rd_burst_data_valid,input wire[255:0]rd_burst_data,input wire rd_burst_finish,
    output wire wr_burst_req,output wire[BUSRT_BITS-1:0]wr_burst_len,output wire[ADDR_BITS-1:0]wr_burst_addr,
    input wire wr_burst_data_req,output wire[255:0]wr_burst_data,input wire wr_burst_finish,
    input wire source_camera,bypass_correction,input wire[1:0]feature,
    input wire imu_ready,input wire[127:0]imu_q,
    output wire snapshot_request,
    input wire snapshot_grant,snapshot_camera,
    input wire[ADDR_BITS-1:0]snapshot_base,input wire[31:0]snapshot_tag,
    output wire algorithm_fault,output wire[31:0]algorithm_cycles,
    // High while no correction/render job owns this view (paired CMP start).
    output wire job_idle,
    // Shared Gamma ROM port (EXTERNAL_GAMMA_ROM=1), mem_clk domain.
    output wire[16:0] gamma_rom_addr,input wire[7:0] gamma_rom_data
);
    localparam [ADDR_BITS-1:0] SCRATCH_BASE=OUTPUT_BASE0+57600;
    // Own algorithm domain: asynchronous assertion, two-cycle local release.
    // Disabled/test-pattern branch retains the mature reset contract.
    wire core_rst,renderer_rst,builder_rst,crop_rst,guard_rst,engine_rst,adapter_rst;
    generate if(ENABLED!=0 && TEST_PATTERN==0)begin:core_reset_domain
        // One metastability-catching first stage, with same-edge second-stage
        // release replicas per consumer group. No added reset-release cycle.
        // PDS maps syn_preserve to PAP_PRESERVE; ASYNC_REG is unsupported here.
        (* syn_preserve=1 *) reg[7:0]reset_sync;
        always @(posedge mem_clk or posedge rst)begin
            if(rst)reset_sync<=8'hff;
            else begin
                reset_sync[0]<=1'b0;
                reset_sync[7:1]<={7{reset_sync[0]}};
            end
        end
        assign core_rst=reset_sync[1];
        assign renderer_rst=reset_sync[2];
        assign builder_rst=reset_sync[3];
        assign crop_rst=reset_sync[4];
        assign guard_rst=reset_sync[5];
        assign engine_rst=reset_sync[6];
        assign adapter_rst=reset_sync[7];
    end else begin:original_reset_domain
        assign core_rst=rst;
        assign renderer_rst=rst;assign builder_rst=rst;assign crop_rst=rst;
        assign guard_rst=rst;assign engine_rst=rst;assign adapter_rst=rst;
    end endgenerate
    wire renderer_source_valid;
    wire[ADDR_BITS-1:0]renderer_source_base;
    wire renderer_accept;
    wire rr,wr;wire[BUSRT_BITS-1:0]rl,wl;wire[ADDR_BITS-1:0]ra,wa;wire[255:0]wd;
    wire renderer_selected;
    generate if(ENABLED!=0 && TEST_PATTERN==0)begin:timed_renderer
    frame_view_render_retimed #(.ADDR_BITS(ADDR_BITS),.BUSRT_BITS(BUSRT_BITS),.OUTPUT_BASE0(OUTPUT_BASE0),.OUTPUT_BASE1(OUTPUT_BASE1),
        .FRAME_HEIGHT(FRAME_HEIGHT),.TEST_PATTERN(TEST_PATTERN),.USE_AFFINE(USE_AFFINE),.MIN_STEP_X(MIN_STEP_X)) renderer(
        .mem_clk(mem_clk),.rst(renderer_rst),.source_valid(renderer_source_valid),.source_base(renderer_source_base),
        .scale_shift(scale_shift),.source_left(source_left),.source_top(source_top),.source_width(source_width),.source_height(source_height),
        .destination_left(destination_left),.destination_top(destination_top),.apply_binary((ENABLED==0)?apply_binary:1'b0),
        .adjust_values(adjust_values),.doodle_index(doodle_index),.doodle_address(doodle_address),
        .view_step_q16(view_step_q16),.view_step_x_q16(view_step_x_q16),.view_rotation(view_rotation),.view_origin_x(view_origin_x),.view_origin_y(view_origin_y),
        .display_index_async(display_index_async),.display_accept_valid_async(renderer_accept),.front_index(front_index),.frame_ready(frame_ready),
        .rd_burst_req(rr),.rd_burst_len(rl),.rd_burst_addr(ra),.rd_burst_data_valid(rd_burst_data_valid && renderer_selected),
        .rd_burst_data(rd_burst_data),.rd_burst_finish(rd_burst_finish && renderer_selected),
        .wr_burst_req(wr),.wr_burst_len(wl),.wr_burst_addr(wa),.wr_burst_data_req(wr_burst_data_req && renderer_selected),
        .wr_burst_data(wd),.wr_burst_finish(wr_burst_finish && renderer_selected));
    end else begin:mature_renderer
    frame_view_render #(.ADDR_BITS(ADDR_BITS),.BUSRT_BITS(BUSRT_BITS),.OUTPUT_BASE0(OUTPUT_BASE0),.OUTPUT_BASE1(OUTPUT_BASE1),
        .FRAME_HEIGHT(FRAME_HEIGHT),.TEST_PATTERN(TEST_PATTERN),.USE_AFFINE(USE_AFFINE)) renderer(
        .mem_clk(mem_clk),.rst(rst),.source_valid(renderer_source_valid),.source_base(renderer_source_base),
        .scale_shift(scale_shift),.source_left(source_left),.source_top(source_top),.source_width(source_width),.source_height(source_height),
        .destination_left(destination_left),.destination_top(destination_top),.apply_binary((ENABLED==0)?apply_binary:1'b0),
        .adjust_values(adjust_values),.doodle_index(doodle_index),.doodle_address(doodle_address),
        .view_step_q16(view_step_q16),.view_rotation(view_rotation),.view_origin_x(view_origin_x),.view_origin_y(view_origin_y),
        .display_index_async(display_index_async),.display_accept_valid_async(renderer_accept),.front_index(front_index),.frame_ready(frame_ready),
        .rd_burst_req(rr),.rd_burst_len(rl),.rd_burst_addr(ra),.rd_burst_data_valid(rd_burst_data_valid && renderer_selected),
        .rd_burst_data(rd_burst_data),.rd_burst_finish(rd_burst_finish && renderer_selected),
        .wr_burst_req(wr),.wr_burst_len(wl),.wr_burst_addr(wa),.wr_burst_data_req(wr_burst_data_req && renderer_selected),
        .wr_burst_data(wd),.wr_burst_finish(wr_burst_finish && renderer_selected));
    end endgenerate
generate if(ENABLED==0 || TEST_PATTERN!=0)begin:original_renderer
    assign renderer_source_valid=source_valid;assign renderer_source_base=source_base;
    assign renderer_accept=display_accept_valid_async;assign renderer_selected=1'b1;
    assign rd_burst_req=rr;assign rd_burst_addr=ra;assign rd_burst_len=rl;
    assign wr_burst_req=wr;assign wr_burst_addr=wa;assign wr_burst_len=wl;assign wr_burst_data=wd;
    assign algorithm_fault=1'b0;assign algorithm_cycles=0;
    assign snapshot_request=1'b0;
    assign job_idle=1'b1;
    assign gamma_rom_addr=17'd0;
end else begin:algorithm
    localparam WAIT_SOURCE=0,LATCH=1,POSE_START=2,POSE_WAIT=3,CONFIGURE=4,START=5,PROCESS=6,DRAIN=7,RENDER=8,HALT=9,RELEASE=10,POSE_GUARD_WAIT=11,POSE_CROP_WAIT=12;
    reg[3:0]state;
    reg[ADDR_BITS-1:0]input_base;
    reg[671:0]cfg;
    reg[359:0]ki;
    reg[127:0]reference_pose,current_pose;
    reg reference_valid,last_rotation_mode,raw_frame,last_front,last_ready;
    reg lease_acquired,leased_camera;
    reg[31:0]leased_tag;
    reg builder_start;
    wire builder_busy,builder_done,builder_error;
    wire[359:0]pose_h;
    wire[671:0]left_cfg,right_cfg;
    wire[359:0]left_ki,right_ki;
    wire[671:0]selected_cfg=leased_camera?right_cfg:left_cfg;
    wire[359:0]selected_ki=leased_camera?right_ki:left_ki;
    stereo_calibration_constants constants(.left_cfg(left_cfg),.right_cfg(right_cfg),.left_ki(left_ki),.right_ki(right_ki));
    imu_ray_homography builder(.clk(mem_clk),.rst_n(!builder_rst),.start(builder_start),.current_q(current_pose),.reference_q(reference_pose),
        .inverse_k(ki),.width(12'd640),.height(12'd720),.busy(builder_busy),.done(builder_done),.error(builder_error),.homography(pose_h));
    wire crop_busy,crop_done,crop_error;
    wire[359:0]cropped_pose_h;
    wire[16:0]crop_scale_q16;
    wire signed[21:0]crop_xmin=leased_camera?-22'sd612477:-22'sd626451;
    wire signed[21:0]crop_xmax=leased_camera?22'sd593148:22'sd557860;
    wire signed[21:0]crop_ymin=leased_camera?-22'sd339242:-22'sd332101;
    wire signed[21:0]crop_ymax=leased_camera?22'sd343658:22'sd338390;
    pose_auto_crop auto_crop(.clk(mem_clk),.rst_n(!crop_rst),
        .start(state==POSE_WAIT && builder_done && !builder_error),.homography(pose_h),
        .width(12'd640),.height(FRAME_HEIGHT[11:0]),
        .xmin_q20(crop_xmin),.xmax_q20(crop_xmax),.ymin_q20(crop_ymin),.ymax_q20(crop_ymax),
        .busy(crop_busy),.done(crop_done),.error(crop_error),.cropped_h(cropped_pose_h),.scale_q16(crop_scale_q16));
    wire ray_guard_busy,ray_guard_done,ray_guard_error;
    pose_ray_domain_guard ray_guard(.clk(mem_clk),.rst_n(!guard_rst),
        .start(state==POSE_CROP_WAIT && crop_done && !crop_error),.homography(cropped_pose_h),
        .width(12'd640),.height(FRAME_HEIGHT[11:0]),.busy(ray_guard_busy),.done(ray_guard_done),.error(ray_guard_error));
    wire core_cfg_ready,core_busy,core_done,core_error;
    wire crv,crr,csv,cwv,cwr;wire[31:0]cra,cwa,cws;wire[255:0]csd,cwd;
    wire br,bw;wire[BUSRT_BITS-1:0]brl,bwl;wire[ADDR_BITS-1:0]bra,bwa;wire[255:0]bwd;
    wire reads_idle,writes_idle,bridge_fault;
    reg sticky_fault,pose_invalid;
    assign algorithm_fault=sticky_fault || pose_invalid || bridge_fault || core_error;
    assign renderer_selected=(state==RENDER);
    assign job_idle=(state==WAIT_SOURCE);
    assign snapshot_request=lease_acquired || (state==WAIT_SOURCE && source_valid);
    assign renderer_source_valid=renderer_selected;
    assign renderer_source_base=raw_frame?input_base:SCRATCH_BASE;
    assign renderer_accept=display_accept_valid_async && renderer_selected;
    assign rd_burst_req=renderer_selected?rr:br;
    assign rd_burst_len=renderer_selected?rl:brl;
    assign rd_burst_addr=renderer_selected?ra:bra;
    assign wr_burst_req=renderer_selected?wr:bw;
    assign wr_burst_len=renderer_selected?wl:bwl;
    assign wr_burst_addr=renderer_selected?wa:bwa;
    assign wr_burst_data=renderer_selected?wd:bwd;
    undistort_closure_engine_v2 #(.EXTERNAL_GAMMA_ROM(EXTERNAL_GAMMA_ROM)) core(.gamma_rom_addr(gamma_rom_addr),.gamma_rom_data(gamma_rom_data),.clk(mem_clk),.rst_n(!engine_rst),.cfg_valid(state==CONFIGURE),.cfg_ready(core_cfg_ready),.cfg_flat(cfg),
        .start(state==START),.src_base({2'b00,input_base,5'b0}),.dst_base({2'b00,SCRATCH_BASE,5'b0}),
        .busy(core_busy),.done(core_done),.error(core_error),.rd_valid(crv),.rd_ready(crr),.rd_addr(cra),.rsp_valid(csv),.rsp_data(csd),
        .wr_valid(cwv),.wr_ready(cwr),.wr_addr(cwa),.wr_data(cwd),.wr_strb(cws),.frame_cycles(algorithm_cycles),
        .invalid_domain_pixels(),.applied_gamma_q8(),.target_gamma_q8(),.next_auto_gamma_q8(),.statistics_pixels(),.cache_hits(),.cache_misses());
    core_native_burst_adapter #(.ADDR_BITS(ADDR_BITS),.BURST_BITS(BUSRT_BITS)) bridge(
        .clk(mem_clk),.rst_n(!adapter_rst),.core_rd_valid(crv),.core_rd_ready(crr),.core_rd_addr(cra),.core_rsp_valid(csv),.core_rsp_data(csd),
        .core_wr_valid(cwv),.core_wr_ready(cwr),.core_wr_addr(cwa),.core_wr_data(cwd),.core_wr_strb(cws),
        .rd_burst_req(br),.rd_burst_addr(bra),.rd_burst_len(brl),.rd_burst_data_valid(rd_burst_data_valid && !renderer_selected),
        .rd_burst_data(rd_burst_data),.rd_burst_finish(rd_burst_finish && !renderer_selected),
        .wr_burst_req(bw),.wr_burst_addr(bwa),.wr_burst_len(bwl),.wr_burst_data_req(wr_burst_data_req && !renderer_selected),
        .wr_burst_data(bwd),.wr_burst_finish(wr_burst_finish && !renderer_selected),.reads_idle(reads_idle),.writes_idle(writes_idle),.fault(bridge_fault));
    always @(posedge mem_clk or posedge core_rst)begin
        if(core_rst)begin
            state<=WAIT_SOURCE;input_base<=0;cfg<=0;ki<=0;reference_pose<=128'd1073741824;current_pose<=128'd1073741824;
            reference_valid<=0;last_rotation_mode<=0;raw_frame<=0;last_front<=0;last_ready<=0;builder_start<=0;sticky_fault<=0;pose_invalid<=0;
            lease_acquired<=0;leased_camera<=0;leased_tag<=0;
        end else begin
            builder_start<=0;
            if(bridge_fault || core_error || (lease_acquired && (!snapshot_grant || snapshot_tag!=leased_tag)))begin sticky_fault<=1;state<=HALT;end
            else case(state)
                WAIT_SOURCE:if(source_valid && snapshot_grant)begin
                    input_base<=snapshot_base;leased_camera<=snapshot_camera;leased_tag<=snapshot_tag;
                    lease_acquired<=1;raw_frame<=bypass_correction;state<=LATCH;
                end
                LATCH:begin
                    last_front<=front_index;last_ready<=frame_ready;
                    cfg<=selected_cfg;ki<=selected_ki;cfg[640+:2]<=(feature==3)?2'd1:2'd0;
                    last_rotation_mode<=(feature==1);
                    if(feature==1 && imu_ready)begin
                        current_pose<=imu_q;
                        if(!reference_valid || !last_rotation_mode)begin reference_pose<=imu_q;reference_valid<=1;end
                    end else begin
                        current_pose<=128'd1073741824;reference_pose<=128'd1073741824;reference_valid<=0;
                    end
                    if(raw_frame)state<=RENDER;else state<=POSE_START;
                end
                POSE_START:begin builder_start<=1;state<=POSE_WAIT;end
                POSE_WAIT:if(builder_done)begin
                    if(builder_error)begin pose_invalid<=1;lease_acquired<=0;state<=RELEASE;end
                    else state<=POSE_CROP_WAIT;
                end
                POSE_CROP_WAIT:if(crop_done)begin
                    if(crop_error)begin pose_invalid<=1;lease_acquired<=0;state<=RELEASE;end
                    else begin cfg[359:0]<=cropped_pose_h;state<=POSE_GUARD_WAIT;end
                end
                POSE_GUARD_WAIT:if(ray_guard_done)begin
                    if(ray_guard_error)begin pose_invalid<=1;lease_acquired<=0;state<=RELEASE;end
                    else begin pose_invalid<=0;state<=CONFIGURE;end
                end
                CONFIGURE:if(core_cfg_ready)state<=START;
                START:state<=PROCESS;
                PROCESS:if(core_done)state<=DRAIN;
                DRAIN:if(reads_idle && writes_idle)begin lease_acquired<=0;state<=RENDER;end
                RENDER:if(frame_ready && (!last_ready || front_index!=last_front))begin lease_acquired<=0;state<=RELEASE;end
                RELEASE:if(!snapshot_grant)state<=WAIT_SOURCE;
                HALT:state<=HALT;
                default:state<=HALT;
            endcase
        end
    end
end endgenerate
endmodule
