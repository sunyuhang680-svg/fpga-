// SPDX-License-Identifier: MIT
// Device-specific HID decoder: VID 222A, PID 0001, captured 2026-10-14.
// report_data[7:0] is byte 0 (Report ID), not the SPI status byte.
// Coordinates are RAW 0..16384, not HDMI pixel coordinates.
module ili222a_touch_decode (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         clear_points,
    input  wire         report_valid,
    input  wire [6:0]   report_len,
    input  wire [511:0] report_data,
    output reg          touch_event,
    output reg          point0_valid,
    output reg  [5:0]   point0_id,
    output reg  [15:0]  point0_x,
    output reg  [15:0]  point0_y,
    output reg          point1_valid,
    output reg  [5:0]   point1_id,
    output reg  [15:0]  point1_x,
    output reg  [15:0]  point1_y,
    output reg  [1:0]   touch_count,
    output reg          touch_overflow,
    output reg          report_error
);
    reg next_v0, next_v1, malformed;
    reg [5:0] next_id0, next_id1;
    reg [15:0] next_x0, next_y0, next_x1, next_y1;
    reg [9:0] active;
    reg [3:0] active_count;
    reg [7:0] flags_i, flags_j;
    reg [15:0] x_i, y_i;
    integer i, j;

    // Prefer the two already tracked Contact IDs. Then fill vacant channels
    // in descriptor-slot order. Slot order may change between USB reports.
    always @* begin
        next_v0 = 0; next_v1 = 0;
        next_id0 = point0_id; next_id1 = point1_id;
        next_x0 = point0_x; next_y0 = point0_y;
        next_x1 = point1_x; next_y1 = point1_y;
        active = 0; active_count = 0;
        malformed = (report_len != 64) || (report_data[447:440] > 10);
        flags_i = 0; flags_j = 0; x_i = 0; y_i = 0;
        for (i = 0; i < 10; i = i + 1) begin
            flags_i = report_data[(1+5*i)*8 +: 8];
            x_i = report_data[(2+5*i)*8 +: 16];
            y_i = report_data[(4+5*i)*8 +: 16];
            active[i] = (i < report_data[447:440]) && flags_i[6];
            if (active[i]) begin
                active_count = active_count + 1'b1;
                if ((x_i > 16384) || (y_i > 16384)) malformed = 1;
                for (j = 0; j < 10; j = j + 1) begin
                    flags_j = report_data[(1+5*j)*8 +: 8];
                    if ((j < i) && active[j] &&
                        (flags_i[5:0] == flags_j[5:0])) malformed = 1;
                end
                if (point0_valid && (flags_i[5:0] == point0_id)) begin
                    next_v0 = 1; next_x0 = x_i; next_y0 = y_i;
                end
                if (point1_valid && (flags_i[5:0] == point1_id)) begin
                    next_v1 = 1; next_x1 = x_i; next_y1 = y_i;
                end
            end
        end
        for (i = 0; i < 10; i = i + 1) begin
            flags_i = report_data[(1+5*i)*8 +: 8];
            if (active[i] && !(next_v0 && (flags_i[5:0] == next_id0)) &&
                             !(next_v1 && (flags_i[5:0] == next_id1))) begin
                if (!next_v0) begin
                    next_v0 = 1; next_id0 = flags_i[5:0];
                    next_x0 = report_data[(2+5*i)*8 +: 16];
                    next_y0 = report_data[(4+5*i)*8 +: 16];
                end else if (!next_v1) begin
                    next_v1 = 1; next_id1 = flags_i[5:0];
                    next_x1 = report_data[(2+5*i)*8 +: 16];
                    next_y1 = report_data[(4+5*i)*8 +: 16];
                end
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            touch_event <= 0; report_error <= 0;
            point0_valid <= 0; point0_id <= 0; point0_x <= 0; point0_y <= 0;
            point1_valid <= 0; point1_id <= 0; point1_x <= 0; point1_y <= 0;
            touch_count <= 0; touch_overflow <= 0;
        end else begin
            touch_event <= 0; report_error <= 0;
            if (clear_points) begin
                // Publish one release event if disconnect/reset removes a finger.
                touch_event <= point0_valid || point1_valid;
                point0_valid <= 0; point1_valid <= 0;
                touch_count <= 0; touch_overflow <= 0;
            end else if (report_valid && report_data[7:0] == 8'h04) begin
                if (malformed) report_error <= 1;
                else begin
                    touch_event <= 1;
                    point0_valid <= next_v0; point1_valid <= next_v1;
                    point0_id <= next_id0; point1_id <= next_id1;
                    point0_x <= next_x0; point0_y <= next_y0;
                    point1_x <= next_x1; point1_y <= next_y1;
                    touch_count <= {1'b0,next_v0} + {1'b0,next_v1};
                    touch_overflow <= (active_count > 2);
                end
            end
        end
    end
endmodule
