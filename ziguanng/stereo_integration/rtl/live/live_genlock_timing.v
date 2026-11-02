`timescale 1ns/1ps
// Live path display timing, locked to the camera.
//
// Same layout as the teammate sync_vg (positive syncs; per line: sync, back
// porch, 1280 active, front porch; per frame: sync, back porch, 720 active,
// front porch).  Two things differ:
//  * The line length follows the camera.  The average camera row period is
//    measured as (time between the completion of row 0 and row 512) / 512 in
//    pixel clocks and used as H_TOTAL from the next frame on (rounded down,
//    so the display never runs slower than the camera inside a frame).
//  * The last back-porch line repeats until the newest camera frame has
//    LAG_ROWS complete rows.  The active area then starts LAG_ROWS rows behind
//    the camera, which is the whole frame-to-frame alignment.
// Before the first measurement it runs free as standard 720p30
// (H_TOTAL_DEFAULT x 750); if the camera stops it gives up waiting after
// MAX_WAIT lines.
// Requests (x,y,de) lead the HDMI outputs: the consumer delays hs/vs/de by
// its own pipeline latency.
module live_genlock_timing #(
    parameter integer H_ACT=1280,H_FP=110,H_SYNC=40,H_BP_MIN=220,
    parameter integer H_TOTAL_DEFAULT=3300,
    parameter integer V_ACT=720,V_FP=5,V_SYNC=5,V_BP=20,
    parameter integer LAG_ROWS=20,
    parameter integer LATE_ROWS=12,       // start up to this many rows late, else skip the camera frame
    parameter integer MAX_WAIT=1023
)(
    input wire clk,rst_n,
    input wire[15:0]rows_total,frame_base,   // selected camera, from live_line_ring
    input wire frame_event,
    output reg hs,vs,de,
    output reg[11:0]x,y,
    output reg frame_start,                  // first clock of a frame's first active line
    output reg[11:0]h_total,                 // in use
    output reg[11:0]v_total_last,            // lines of the previous frame
    output reg[11:0]wait_last,               // extra back-porch lines of the previous frame
    output reg locked,                       // previous frame started on the camera
    output reg[31:0]row0_to_start            // clocks from camera row 0 complete to frame_start
);
    localparam integer H_MIN=H_ACT+H_FP+H_SYNC+H_BP_MIN;
    localparam integer V_FIRST=V_SYNC+V_BP;          // first active line
    localparam integer V_TOTAL=V_SYNC+V_BP+V_ACT+V_FP;

    // --- camera row period ---
    reg[15:0]rows_d;reg[15:0]frame_rows;reg[31:0]clock,t_row0;reg meas_valid,new_frame;
    reg[11:0]h_next;reg skip;
    wire row_done=(rows_total!=rows_d);
    wire[15:0]rows_in_frame=rows_total-frame_base;
    wire[31:0]span=clock-t_row0;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin rows_d<=0;clock<=0;t_row0<=0;meas_valid<=0;h_next<=H_TOTAL_DEFAULT;new_frame<=0;end
        else begin
            clock<=clock+32'd1;rows_d<=rows_total;
            if(frame_event)new_frame<=1;
            if(frame_start || skip)new_frame<=0;
            if(row_done && rows_in_frame==16'd1)t_row0<=clock;
            if(row_done && rows_in_frame==16'd513)begin
                meas_valid<=1;
                h_next<=(span[20:9]<H_MIN)?H_MIN[11:0]:(span[31:21]!=0)?12'd4095:span[20:9];
            end
        end
    end

    // --- raster ---
    reg[11:0]h,v,wait_cnt;reg h_measured;
    wire line_end=(h==h_total-12'd1);
    // Lock only once the row period is known and in use: a display line longer
    // than the camera row would let the camera overrun the line ring.
    wire go=h_measured && new_frame && !frame_event && rows_in_frame>=LAG_ROWS && rows_in_frame<=LAG_ROWS+LATE_ROWS;
    // Arriving too late (display frame longer than the camera frame) would let
    // the camera overrun the ring: skip that camera frame and wait for the next.
    wire too_late=h_measured && new_frame && !frame_event && rows_in_frame>LAG_ROWS+LATE_ROWS;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            h<=0;v<=0;wait_cnt<=0;h_total<=H_TOTAL_DEFAULT;h_measured<=0;hs<=0;vs<=0;de<=0;x<=0;y<=0;frame_start<=0;
            v_total_last<=0;wait_last<=0;locked<=0;row0_to_start<=0;skip<=0;
        end else begin
            frame_start<=0;skip<=0;
            if(!line_end)h<=h+12'd1;
            else begin
                h<=0;
                if(v==V_FIRST-1)begin
                    if(go || !h_measured || wait_cnt>=MAX_WAIT)begin
                        v<=V_FIRST;frame_start<=1;locked<=go;wait_last<=wait_cnt;wait_cnt<=0;
                        row0_to_start<=clock-t_row0;
                    end else begin wait_cnt<=wait_cnt+12'd1;skip<=too_late;end
                end else if(v==V_TOTAL-1)begin
                    v<=0;v_total_last<=V_TOTAL[11:0]+wait_last;
                    if(meas_valid)begin h_total<=h_next;h_measured<=1;end
                end else v<=v+12'd1;
            end
            hs<=(h<H_SYNC);
            vs<=(v<V_SYNC);
            de<=(v>=V_FIRST && v<V_FIRST+V_ACT && h>=H_SYNC+H_BP_MIN && h<H_SYNC+H_BP_MIN+H_ACT);
            x<=h-(H_SYNC+H_BP_MIN);y<=v-V_FIRST;
        end
    end
endmodule
