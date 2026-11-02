`timescale 1ns/1ps
// Acquire after a fresh frame boundary; existing camera ring implements freeze.
// snapshot_tag is a lease epoch, NOT an exposure-synchronization guarantee.
module camera_frame_snapshot(
    input wire camera_clk,core_clk,rst_n,
    input wire request_hold,
    input wire camera_vsync,input wire[1:0]completed_index,
    output reg freeze_ring,
    output wire snapshot_valid,output wire[1:0]snapshot_index,
    output wire[31:0]snapshot_tag
);
    (* ASYNC_REG="TRUE" *)reg[1:0]camera_reset_sync;
    always @(posedge camera_clk or negedge rst_n)begin
        if(!rst_n)camera_reset_sync<=0;else camera_reset_sync<={camera_reset_sync[0],1'b1};
    end
    wire camera_rst_n=camera_reset_sync[1];
    (* ASYNC_REG="TRUE" *) reg[1:0]request_sync;
    reg vs_last;
    reg[2:0]state;
    reg valid_cam,event_cam;
    reg[1:0]index_cam;
    reg[31:0]epoch;
    wire[34:0]descriptor;
    stereo_event_cdc #(.WIDTH(35),.INITIAL_PUBLISH(1)) mailbox(.src_clk(camera_clk),.dst_clk(core_clk),.rst_n(rst_n),
        .src_event(event_cam),.src_data({valid_cam,index_cam,epoch}),.dst_event(),.dst_data(descriptor));
    assign {snapshot_valid,snapshot_index,snapshot_tag}=descriptor;
    always @(posedge camera_clk or negedge camera_rst_n)begin
        if(!camera_rst_n)begin request_sync<=0;vs_last<=0;state<=0;freeze_ring<=0;valid_cam<=0;event_cam<=0;index_cam<=0;epoch<=0;end
        else begin
            request_sync<={request_sync[0],request_hold};vs_last<=camera_vsync;event_cam<=0;
            case(state)
                0:if(request_sync[1])state<=1;
                1:if(!request_sync[1])state<=0;
                    else if(camera_vsync && !vs_last)state<=2;
                2:begin
                    // The mature ring edge detector publishes one clock after VS.
                    // Let that publication happen BEFORE asserting freeze.
                    freeze_ring<=1;state<=3;
                end
                3:begin index_cam<=completed_index;epoch<=epoch+1'b1;valid_cam<=1;event_cam<=1;state<=4;end
                4:if(!request_sync[1])begin freeze_ring<=0;valid_cam<=0;event_cam<=1;state<=0;end
            endcase
        end
    end
endmodule
