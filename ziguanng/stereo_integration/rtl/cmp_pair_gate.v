`timescale 1ns/1ps
// CMP paired-frame gate (ENABLE_CMP_CONTROLS=1 only).
//
// In CMP both views render the SAME camera.  A pair job starts both views in
// the same core cycle, so stereo_frame_leases puts them into one cohort (same
// source frame and lease tag).  The view configuration is frozen at that start
// and held until the pair has been published, so both halves are built from
// one configuration version; the latest UI state is taken by the next pair.
// The two finished front buffers are published together: one mailbox carries
// both indices into the pixel domain, where they are applied only while
// neither HDMI reader is capturing a frame descriptor.  Until then the readers
// keep showing the previous complete pair.
//
// Outside CMP the gate is transparent (live config, live publication).
// Frames that were already in flight when CMP was entered are published as
// they finish, so the mature renderer's display hand-shake can never stall.
module cmp_pair_gate #(
    parameter CFG_BITS=8
)(
    input wire mem_clk,mem_rst,
    input wire cmp_mode,
    input wire cmp_key,                          // page/camera; a change re-enters
    input wire source_valid_in,
    output wire source_valid_out,
    input wire left_idle,right_idle,
    input wire[1:0]left_pub,right_pub,           // {frame_ready,front_index}
    input wire[CFG_BITS-1:0]cfg_live,
    input wire binary_live,                      // binary flag of cfg_live
    output reg[CFG_BITS-1:0]cfg_out,
    output wire pair_running,

    input wire pix_clk,pix_rst_n,
    input wire left_capture,right_capture,       // HDMI frame requests (pix_clk)
    output wire[1:0]left_pub_pixel,right_pub_pixel,
    output reg binary_pixel                      // valid for the frame now read
);
    localparam S_TRACK=2'd0,S_START=2'd1,S_RUN=2'd2;
    reg[1:0]state;
    reg[1:0]pub_l,pub_r;
    reg binary_pub,binary_snap;
    reg[4:0]last_sent;
    reg cmp_mode_d,cmp_key_d;
    wire cmp_entry=cmp_mode && (!cmp_mode_d || cmp_key!=cmp_key_d);
    wire[4:0]publish_word={binary_pub,pub_l,pub_r};
    assign source_valid_out=cmp_mode?(state==S_START && source_valid_in):source_valid_in;
    assign pair_running=(state!=S_TRACK);

    always @(posedge mem_clk or posedge mem_rst)begin
        if(mem_rst)begin
            state<=S_TRACK;pub_l<=0;pub_r<=0;binary_pub<=0;binary_snap<=0;cfg_out<=0;
            cmp_mode_d<=0;cmp_key_d<=0;
        end else begin
        cmp_mode_d<=cmp_mode;cmp_key_d<=cmp_key;
        // The configuration seen by a renderer can be latched whenever the
        // display accepts its previous frame, so cfg_out may only change at
        // points where no such latch can belong to a different version:
        // CMP entry (or page change) and the publication of a pair.  A
        // renderer that latches during TRACK therefore sees exactly the
        // version the next pair start will grant.
        if(!cmp_mode)begin
            state<=S_TRACK;pub_l<=left_pub;pub_r<=right_pub;binary_pub<=0;
            cfg_out<=cfg_live;binary_snap<=binary_live;
        end else case(state)
            S_TRACK:begin
                // Only frames started before CMP (or before a page change)
                // can finish here; they are passed through.
                pub_l<=left_pub;pub_r<=right_pub;
                if(cmp_entry)begin cfg_out<=cfg_live;binary_snap<=binary_live;end
                else if(left_idle && right_idle && source_valid_in)state<=S_START;
            end
            // cfg_out/binary_snap are frozen from here until publication.
            S_START:if(!left_idle && !right_idle)state<=S_RUN;
            // A view becomes idle only after it published (or abandoned) its
            // job, so both idle means the whole pair is finished.
            S_RUN:begin
                if(left_idle && right_idle)begin
                    pub_l<=left_pub;pub_r<=right_pub;binary_pub<=binary_snap;state<=S_TRACK;
                    // Next version, published together with this pair.
                    cfg_out<=cfg_live;binary_snap<=binary_live;
                end
            end
            default:state<=S_TRACK;
        endcase
        end
    end

    // One latest-state mailbox for both indices and the binary flag.
    reg send;
    always @(posedge mem_clk or posedge mem_rst)begin
        if(mem_rst)begin last_sent<=0;send<=0;end
        else begin send<=(publish_word!=last_sent);last_sent<=publish_word;end
    end
    wire[4:0]pixel_word;
    wire pixel_event;
    stereo_event_cdc #(.WIDTH(5),.INITIAL_PUBLISH(1)) mailbox(.src_clk(mem_clk),.dst_clk(pix_clk),.rst_n(!mem_rst),
        .src_event(send),.src_data(last_sent),.dst_event(pixel_event),.dst_data(pixel_word));

    // Apply only while neither reader is capturing: the right reader requests
    // a few clocks after the left one, so a change between the two captures
    // would split the pair.  Requests stay high until acknowledged.
    reg[4:0]applied;
    reg left_capture_d;
    (* ASYNC_REG="TRUE" *)reg[1:0]pix_reset_sync;
    always @(posedge pix_clk or negedge pix_rst_n)begin
        if(!pix_rst_n)pix_reset_sync<=2'b00;else pix_reset_sync<={pix_reset_sync[0],1'b1};
    end
    wire pix_local_rst_n=pix_reset_sync[1];
    assign left_pub_pixel=applied[3:2];
    assign right_pub_pixel=applied[1:0];
    always @(posedge pix_clk or negedge pix_local_rst_n)begin
        if(!pix_local_rst_n)begin applied<=0;left_capture_d<=0;binary_pixel<=0;end
        else begin
            left_capture_d<=left_capture;
            if(!left_capture && !right_capture)applied<=pixel_word;
            // The binary flag follows the indices the left reader latched.
            if(left_capture && !left_capture_d)binary_pixel<=applied[4];
        end
    end
endmodule
