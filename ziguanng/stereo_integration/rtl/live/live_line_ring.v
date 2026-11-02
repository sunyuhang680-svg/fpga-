`timescale 1ns/1ps
// Live path, one camera: the most recent 2^RING_LOG2 camera rows kept on chip.
//
// Camera domain: rows are counted with a free-running absolute index
// (rows_total = rows completed since reset).  Row A lives in ring slot
// A mod 2^RING_LOG2.  Storage is split into four banks by (slot parity,
// column parity) so the pixel side can read a full 2x2 bilinear
// neighbourhood in one clock.  frame_base is the absolute index of row 0 of
// the newest camera frame; it toggles frame_toggle when it changes.
//
// Pixel domain: rows_total crosses as Gray code (it only ever increments by
// one).  frame_base is held for a whole frame and is sampled two clocks after
// the synchronized toggle changed.
module live_line_ring #(
    parameter integer WIDTH=1280,
    parameter integer RING_LOG2=6,
    parameter integer BANK_AW=$clog2(((1<<RING_LOG2)/2)*(WIDTH/2))
)(
    input wire cam_clk,cam_rst_n,
    input wire cam_vsync,cam_href,cam_valid,
    input wire[15:0]cam_pixel,

    input wire pix_clk,pix_rst_n,
    output reg[15:0]rows_total_pix,      // rows completed (absolute)
    output reg[15:0]frame_base_pix,      // absolute index of row 0, newest frame
    output reg frame_event_pix,          // one pulse per new camera frame
    input wire[BANK_AW-1:0]rd_addr00,rd_addr01,rd_addr10,rd_addr11, // [row parity][col parity]
    output wire[15:0]rd_data00,rd_data01,rd_data10,rd_data11
);
    localparam integer HALF_W=WIDTH/2;
    localparam integer BANK_ROWS=(1<<RING_LOG2)/2;
    localparam integer BANK_DEPTH=BANK_ROWS*HALF_W;

    // ---------------- camera domain ----------------
    reg href_d,vs_seen,frame_toggle;
    reg[11:0]col;
    reg[15:0]rows_total,frame_base;
    wire line_start=cam_href && !href_d;
    wire[11:0]col_now=line_start?12'd0:col;
    wire we=cam_valid && cam_href && col_now<WIDTH;
    wire[RING_LOG2-1:0]slot=rows_total[RING_LOG2-1:0];
    wire[BANK_AW-1:0]wr_addr=slot[RING_LOG2-1:1]*HALF_W+col_now[11:1];
    always @(posedge cam_clk or negedge cam_rst_n)begin
        if(!cam_rst_n)begin
            href_d<=0;vs_seen<=0;frame_toggle<=0;col<=0;rows_total<=0;frame_base<=0;
        end else begin
            href_d<=cam_href;
            if(cam_vsync)vs_seen<=1;
            if(line_start && vs_seen)begin vs_seen<=0;frame_base<=rows_total;frame_toggle<=!frame_toggle;end
            col<=col_now+(we?12'd1:12'd0);
            if(!cam_href && href_d && col!=0)rows_total<=rows_total+16'd1;
        end
    end
    reg[15:0]rows_gray;
    always @(posedge cam_clk or negedge cam_rst_n)
        if(!cam_rst_n)rows_gray<=0;else rows_gray<=rows_total^(rows_total>>1);

    live_ring_bank #(.AW(BANK_AW),.DEPTH(BANK_DEPTH)) b00(.wclk(cam_clk),.we(we && !slot[0] && !col_now[0]),.waddr(wr_addr),.wdata(cam_pixel),.rclk(pix_clk),.raddr(rd_addr00),.rdata(rd_data00));
    live_ring_bank #(.AW(BANK_AW),.DEPTH(BANK_DEPTH)) b01(.wclk(cam_clk),.we(we && !slot[0] &&  col_now[0]),.waddr(wr_addr),.wdata(cam_pixel),.rclk(pix_clk),.raddr(rd_addr01),.rdata(rd_data01));
    live_ring_bank #(.AW(BANK_AW),.DEPTH(BANK_DEPTH)) b10(.wclk(cam_clk),.we(we &&  slot[0] && !col_now[0]),.waddr(wr_addr),.wdata(cam_pixel),.rclk(pix_clk),.raddr(rd_addr10),.rdata(rd_data10));
    live_ring_bank #(.AW(BANK_AW),.DEPTH(BANK_DEPTH)) b11(.wclk(cam_clk),.we(we &&  slot[0] &&  col_now[0]),.waddr(wr_addr),.wdata(cam_pixel),.rclk(pix_clk),.raddr(rd_addr11),.rdata(rd_data11));

    // ---------------- pixel domain ----------------
    (* ASYNC_REG="TRUE" *)reg[15:0]gray_meta,gray_sync;
    (* ASYNC_REG="TRUE" *)reg toggle_meta,toggle_sync;
    reg toggle_seen;reg[1:0]base_wait;
    function [15:0] gray2bin(input [15:0] g);integer i;begin
        gray2bin[15]=g[15];for(i=14;i>=0;i=i-1)gray2bin[i]=gray2bin[i+1]^g[i];
    end endfunction
    always @(posedge pix_clk or negedge pix_rst_n)begin
        if(!pix_rst_n)begin
            gray_meta<=0;gray_sync<=0;toggle_meta<=0;toggle_sync<=0;toggle_seen<=0;base_wait<=0;
            rows_total_pix<=0;frame_base_pix<=0;frame_event_pix<=0;
        end else begin
            gray_meta<=rows_gray;gray_sync<=gray_meta;rows_total_pix<=gray2bin(gray_sync);
            toggle_meta<=frame_toggle;toggle_sync<=toggle_meta;
            frame_event_pix<=0;
            if(toggle_sync!=toggle_seen)begin toggle_seen<=toggle_sync;base_wait<=2'd2;end
            else if(base_wait!=0)begin
                base_wait<=base_wait-2'd1;
                if(base_wait==2'd1)begin frame_base_pix<=frame_base;frame_event_pix<=1;end
            end
        end
    end
endmodule

// Simple dual-port RAM, write and read on independent clocks, registered read.
module live_ring_bank #(parameter integer AW=15,parameter integer DEPTH=20480)(
    input wire wclk,we,input wire[AW-1:0]waddr,input wire[15:0]wdata,
    input wire rclk,input wire[AW-1:0]raddr,output reg[15:0]rdata
);
    reg[15:0]mem[0:DEPTH-1];
    always @(posedge wclk)if(we)mem[waddr]<=wdata;
    always @(posedge rclk)rdata<=mem[raddr];
endmodule
