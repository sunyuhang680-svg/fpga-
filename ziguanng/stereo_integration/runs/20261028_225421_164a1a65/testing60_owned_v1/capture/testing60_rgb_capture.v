`timescale 1ns/1ps
// Read-only, camera-clock side branch of direct_camera_hdmi.tap_cp_*.
// No signal is fed back into the 60 Hz direct-video/color pipeline.
module testing60_rgb_capture #(
    parameter integer SOURCE_WIDTH = 1280,
    parameter integer SOURCE_HEIGHT = 720,
    parameter integer FIFO_ADDR_BITS = 10,
    parameter integer ACK_TIMEOUT_CYCLES = 4096,
    parameter integer ACK_SETTLE_CYCLES = 4
)(
    input wire clk,
    input wire rst_n,
    // Already synchronous to clk. Sampled ONLY on cp coordinate (0,0).
    input wire capture_enable,
    input wire tap_cp_valid,
    input wire [10:0] tap_cp_x,
    input wire [10:0] tap_cp_y,
    input wire [23:0] tap_cp_rgb,
    // Connect to the SAME cmos_write_req_gen and frame_read_write instance.
    input wire write_req,
    input wire write_req_ack,
    output reg frame_vsync,
    output reg write_en,
    output reg [15:0] write_data,
    // Camera-domain status events. start and VS precede legacy index rotation.
    output reg frame_start,
    output reg frame_complete,
    output reg frame_abort,
    output reg capture_fault,
    output reg [3:0] fault_code,
    output wire busy
);
    localparam integer DEPTH = (1 << FIFO_ADDR_BITS);
    localparam integer OUTPUT_PIXELS = (SOURCE_WIDTH/2)*SOURCE_HEIGHT;
    localparam [2:0] H_IDLE=0, H_REQ=1, H_ACK_HIGH=2,
                     H_ACK_LOW=3, H_SETTLE=4, H_RUN=5;
    // Faults: 1 coordinates, 2 FIFO overflow, 3 request/ACK timeout,
    // 4 next frame arrived before previous frame was completely delivered.
    reg [15:0] pixels [0:DEPTH-1];
    reg [FIFO_ADDR_BITS-1:0] wr_ptr, rd_ptr;
    reg [FIFO_ADDR_BITS:0] queued;
    reg [10:0] expected_x, expected_y;
    reg active, input_done;
    reg [31:0] emitted;
    reg [2:0] handshake;
    reg [31:0] handshake_age, settle_count;
    reg first_previous;
    (* ASYNC_REG="TRUE" *) reg [1:0] ack_sync;
    wire first = tap_cp_valid && tap_cp_x==0 && tap_cp_y==0;
    wire frame_boundary = first && !first_previous;
    wire coordinate_ok = tap_cp_x==expected_x && tap_cp_y==expected_y;
    wire take = active && !input_done && tap_cp_valid && coordinate_ok;
    wire push = take && !tap_cp_x[0];
    wire pop = active && handshake==H_RUN && queued!=0;
    wire overflow = push && queued==DEPTH && !pop;
    wire bad_coordinate = active && tap_cp_valid && (!coordinate_ok || input_done);
    wire timed_out = active && handshake!=H_RUN &&
                     handshake_age >= ACK_TIMEOUT_CYCLES-1;
    assign busy = active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ack_sync <= 0;
        else ack_sync <= {ack_sync[0],write_req_ack};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_vsync<=0; write_en<=0; write_data<=0;
            frame_start<=0; frame_complete<=0; frame_abort<=0;
            capture_fault<=0; fault_code<=0;
            wr_ptr<=0; rd_ptr<=0; queued<=0;
            expected_x<=0; expected_y<=0; active<=0; input_done<=0;
            emitted<=0; handshake<=H_IDLE; handshake_age<=0; settle_count<=0;
            first_previous<=0;
        end else begin
            frame_vsync<=0; write_en<=0;
            frame_start<=0; frame_complete<=0; frame_abort<=0;
            first_previous<=first;
            if (frame_boundary) begin
                // Even a disabled frame supplies the legacy boundary needed
                // to publish the last complete frame and latch ring freeze.
                frame_vsync<=1;
                // A complete final output from the preceding clock is accepted
                // by the external FIFO on this edge; allow this exact boundary.
                if (active && !(input_done && queued==0 && emitted==OUTPUT_PIXELS)) begin
                    frame_abort<=1; capture_fault<=1; fault_code<=4;
                end else if (active) frame_complete<=1;
                // Never mistake an unfinished previous request/ACK for this
                // boundary's transaction. Such a frame is explicitly rejected.
                active<=capture_enable && !write_req && !ack_sync[1];
                input_done<=0; emitted<=0;
                wr_ptr<=0; rd_ptr<=0; queued<=0;
                handshake_age<=0; settle_count<=0;
                handshake<=(capture_enable && !write_req && !ack_sync[1]) ? H_REQ : H_IDLE;
                expected_x<=1; expected_y<=0;
                if (capture_enable && (write_req || ack_sync[1])) begin
                    frame_abort<=1; capture_fault<=1; fault_code<=3;
                end else if (capture_enable) begin
                    frame_start<=1;
                    // Preserve the old-frame abort indication in this cycle.
                    if (!active || (input_done && queued==0 && emitted==OUTPUT_PIXELS)) begin
                        capture_fault<=0; fault_code<=0;
                    end
                    pixels[0]<={tap_cp_rgb[23:19],tap_cp_rgb[15:10],tap_cp_rgb[7:3]};
                    wr_ptr<=1; queued<=1;
                end
            end else if (active) begin
                if (bad_coordinate || overflow || timed_out) begin
                    frame_abort<=1; capture_fault<=1;
                    fault_code<=bad_coordinate ? 4'd1 : (overflow ? 4'd2 : 4'd3);
                    active<=0; input_done<=0; queued<=0;
                    handshake<=H_IDLE;
                end else begin
                    if (handshake!=H_RUN) handshake_age<=handshake_age+1;
                    case (handshake)
                        H_REQ: if (write_req) handshake<=H_ACK_HIGH;
                        H_ACK_HIGH: if (ack_sync[1]) handshake<=H_ACK_LOW;
                        H_ACK_LOW: if (!ack_sync[1] && !write_req) begin
                            handshake<=H_SETTLE; settle_count<=0;
                        end
                        H_SETTLE: if (settle_count>=ACK_SETTLE_CYCLES-1)
                            handshake<=H_RUN;
                        else settle_count<=settle_count+1;
                        default: begin end
                    endcase
                    if (take) begin
                        if (tap_cp_x==SOURCE_WIDTH-1) begin
                            expected_x<=0;
                            if (tap_cp_y==SOURCE_HEIGHT-1) input_done<=1;
                            else expected_y<=expected_y+1'b1;
                        end else expected_x<=expected_x+1'b1;
                    end
                    if (push) begin
                        pixels[wr_ptr]<={tap_cp_rgb[23:19],tap_cp_rgb[15:10],tap_cp_rgb[7:3]};
                        wr_ptr<=wr_ptr+1'b1;
                    end
                    if (pop) begin
                        write_en<=1; write_data<=pixels[rd_ptr];
                        rd_ptr<=rd_ptr+1'b1; emitted<=emitted+1'b1;
                    end
                    case ({push,pop})
                        2'b10: queued<=queued+1'b1;
                        2'b01: queued<=queued-1'b1;
                        default: begin end
                    endcase
                    // The external FIFO samples the final registered write_en
                    // on this edge, before this completion event is observable.
                    if (input_done && queued==0 && emitted==OUTPUT_PIXELS) begin
                        frame_complete<=1; active<=0; handshake<=H_IDLE;
                        capture_fault<=0; fault_code<=0;
                    end
                end
            end
        end
    end
endmodule
