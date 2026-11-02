`timescale 1ns/1ps
// Chooses between the live stream and the DDR reader output (paused mode).
// Both are driven by the same live timing; the reader chain has a shorter
// pipeline, so it is delayed by READER_DELAY clocks to line up with the live
// stream.  The choice changes only at the start of vertical sync, so HDMI
// never sees a partial frame from either source.
module live_display_select #(
    parameter integer READER_DELAY=10
)(
    input wire clk,rst_n,
    input wire live_request,                 // pixel clock domain
    input wire r_hs,r_vs,r_de,input wire[15:0]r_pixel,
    input wire l_hs,l_vs,l_de,input wire[15:0]l_pixel,
    output reg hs,vs,de,output reg[15:0]pixel,
    output reg live_active
);
    reg[READER_DELAY-1:0]dhs,dvs,dde;reg[15:0]dpx[0:READER_DELAY-1];
    integer i;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)begin dhs<=0;dvs<=0;dde<=0;for(i=0;i<READER_DELAY;i=i+1)dpx[i]<=0;end
        else begin
            dhs<={dhs[READER_DELAY-2:0],r_hs};dvs<={dvs[READER_DELAY-2:0],r_vs};dde<={dde[READER_DELAY-2:0],r_de};
            dpx[0]<=r_pixel;for(i=1;i<READER_DELAY;i=i+1)dpx[i]<=dpx[i-1];
        end
    wire rhs=dhs[READER_DELAY-1],rvs=dvs[READER_DELAY-1],rde=dde[READER_DELAY-1];
    wire[15:0]rpx=dpx[READER_DELAY-1];
    reg l_vs_d;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)begin l_vs_d<=0;live_active<=0;hs<=0;vs<=0;de<=0;pixel<=0;end
        else begin
            l_vs_d<=l_vs;
            if(l_vs && !l_vs_d)live_active<=live_request;
            // sync from the live stream always (identical to the delayed reader sync)
            hs<=l_hs;vs<=l_vs;de<=l_de;
            pixel<=live_active?l_pixel:(rde?rpx:16'd0);
        end
endmodule
