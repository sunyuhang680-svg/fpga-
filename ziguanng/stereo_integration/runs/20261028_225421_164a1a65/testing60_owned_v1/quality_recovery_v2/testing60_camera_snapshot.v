`timescale 1ns/1ps
// Acquire only a complete, DDR-finished bank after a fresh frame boundary.
// An invalid candidate releases the mature ring freeze and retries while the
// same hold request remains asserted. Epochs count successful leases only.
module testing60_camera_snapshot(
    input wire camera_clk,core_clk,rst_n,
    input wire request_hold,
    input wire camera_vsync,
    input wire [1:0] completed_index,
    input wire [3:0] completed_valid,
    output reg freeze_ring,
    output wire snapshot_valid,
    output wire [1:0] snapshot_index,
    output wire [31:0] snapshot_tag
);
    (* ASYNC_REG="TRUE" *) reg [1:0] camera_reset_sync;
    always @(posedge camera_clk or negedge rst_n) begin
        if(!rst_n) camera_reset_sync<=0;
        else camera_reset_sync<={camera_reset_sync[0],1'b1};
    end
    wire camera_rst_n=camera_reset_sync[1];
    (* ASYNC_REG="TRUE" *) reg [1:0] request_sync;
    reg vs_last;
    reg [2:0] state;
    reg valid_cam,event_cam;
    reg [1:0] index_cam;
    reg [31:0] epoch;
    wire [34:0] descriptor;
    stereo_event_cdc #(.WIDTH(35),.INITIAL_PUBLISH(1)) mailbox(
        .src_clk(camera_clk),.dst_clk(core_clk),.rst_n(rst_n),
        .src_event(event_cam),.src_data({valid_cam,index_cam,epoch}),
        .dst_event(),.dst_data(descriptor),.src_ready());
    assign {snapshot_valid,snapshot_index,snapshot_tag}=descriptor;
    always @(posedge camera_clk or negedge camera_rst_n) begin
        if(!camera_rst_n) begin
            request_sync<=0;vs_last<=0;state<=0;freeze_ring<=0;
            valid_cam<=0;event_cam<=0;index_cam<=0;epoch<=0;
        end else begin
            request_sync<={request_sync[0],request_hold};
            vs_last<=camera_vsync;event_cam<=0;
            case(state)
                0: if(request_sync[1]) state<=1;
                1: if(!request_sync[1]) state<=0;
                   else if(camera_vsync && !vs_last) state<=2;
                2: begin
                    // Preserve the original ring's publication-before-freeze
                    // ordering: its read index changes on this camera edge.
                    if(!request_sync[1]) begin freeze_ring<=0;state<=0;end
                    else begin freeze_ring<=1;state<=3;end
                end
                3: begin
                    if(!request_sync[1]) begin freeze_ring<=0;state<=0;end
                    else if(!completed_valid[completed_index]) begin
                        // No descriptor/epoch is published for a bad bank.
                        freeze_ring<=0;state<=1;
                    end else begin
                        index_cam<=completed_index;epoch<=epoch+1'b1;
                        valid_cam<=1;event_cam<=1;state<=4;
                    end
                end
                4: if(!request_sync[1] || !completed_valid[index_cam]) begin
                    freeze_ring<=0;valid_cam<=0;event_cam<=1;
                    state<=request_sync[1] ? 3'd1 : 3'd0;
                end
                default: begin
                    freeze_ring<=0;valid_cam<=0;event_cam<=1;state<=0;
                end
            endcase
        end
    end
endmodule
