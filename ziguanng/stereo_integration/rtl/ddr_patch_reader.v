`timescale 1ns/1ps
// Rectangle reader for immutable RGB565 frame snapshots. No DDR write port.
module ddr_patch_reader #(
    parameter ADDR_BITS=25,
    parameter BURST_BITS=10,
    parameter FRAME_WIDTH=640,
    parameter FRAME_HEIGHT=720,
    parameter STRIDE_WORDS=40,
    parameter MAX_BURST=16
)(
    input wire clk,rst_n,
    input wire req_valid,output wire req_ready,
    input wire [ADDR_BITS-1:0] req_base_word,
    input wire [31:0] req_frame_tag,
    input wire frame_locked,input wire [31:0] current_frame_tag,
    input wire [11:0] req_x,req_y,req_width,req_height,
    output wire pix_valid,input wire pix_ready,
    output wire [15:0] pix_rgb565,
    output wire [11:0] pix_x,pix_y,
    output wire [31:0] pix_frame_tag,
    output wire pix_last,
    output reg done,output reg error,output reg [2:0] error_code,
    output wire busy,
    output wire rd_valid,input wire rd_ready,
    output wire [ADDR_BITS-1:0] rd_addr,
    output wire [BURST_BITS-1:0] rd_len,
    input wire rd_data_valid,input wire [255:0] rd_data,
    input wire rd_done,input wire rd_fault
);
    localparam IDLE=3'd0,PREPARE=3'd1,REQUEST=3'd2,FETCH=3'd3,EMIT=3'd4,FAULT=3'd5;
    localparam ERR_REGION=3'd1,ERR_FRAME=3'd2,ERR_PROTOCOL=3'd3,ERR_DDR=3'd4;
    reg [2:0] state;
    reg [ADDR_BITS-1:0] base,word_addr;
    reg [31:0] tag;
    reg [11:0] start_x,start_y,width,height,column,row;
    reg [12:0] words_left;
    reg [BURST_BITS-1:0] chunk_len;
    reg [5:0] beats;
    reg [8:0] buffer_pixel,chunk_pixels;
    reg [3:0] prefix;
    reg cancel,bad_response;
    reg [255:0] data_buffer [0:MAX_BURST-1];
    wire frame_ok=frame_locked && current_frame_tag==tag;
    wire [12:0] x_end={1'b0,req_x}+{1'b0,req_width};
    wire [12:0] y_end={1'b0,req_y}+{1'b0,req_height};
    wire region_ok=req_width!=0 && req_height!=0 && x_end<=FRAME_WIDTH && y_end<=FRAME_HEIGHT &&
        req_base_word<=({ADDR_BITS{1'b1}}-STRIDE_WORDS*FRAME_HEIGHT+1);
    wire [12:0] row_words=({9'd0,start_x[3:0]}+{1'b0,width}+13'd15)>>4;
    wire [9:0] available_pixels=({{(10-BURST_BITS){1'b0}},chunk_len}<<4)-prefix;
    wire [12:0] remaining_pixels={1'b0,width}-{1'b0,column};
    wire [6:0] observed_beats={1'b0,beats}+(rd_data_valid?7'd1:7'd0);
    assign req_ready=(state==IDLE) && !rd_fault;
    assign busy=(state!=IDLE);
    assign rd_valid=(state==REQUEST) && frame_ok && !cancel && !rd_fault;
    assign rd_addr=word_addr;
    assign rd_len=chunk_len;
    assign pix_valid=(state==EMIT) && frame_ok && !cancel && !rd_fault;
    assign pix_rgb565=data_buffer[buffer_pixel[8:4]][buffer_pixel[3:0]*16 +:16];
    assign pix_x=column;assign pix_y=row;assign pix_frame_tag=tag;
    assign pix_last=(row==height-1'b1 && column==width-1'b1);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;done<=0;error<=0;error_code<=0;cancel<=0;bad_response<=0;
            base<=0;word_addr<=0;tag<=0;start_x<=0;start_y<=0;width<=0;height<=0;
            column<=0;row<=0;words_left<=0;chunk_len<=0;beats<=0;buffer_pixel<=0;chunk_pixels<=0;prefix<=0;
        end else begin
            done<=0;error<=0;
            if(state!=IDLE && state!=FAULT && rd_fault) begin
                state<=FAULT;done<=1;error<=1;error_code<=ERR_DDR;
            end else case(state)
                IDLE: if(req_valid && req_ready) begin
                    error_code<=0;cancel<=0;bad_response<=0;
                    if(!region_ok) begin done<=1;error<=1;error_code<=ERR_REGION;end
                    else if(!frame_locked || current_frame_tag!=req_frame_tag) begin
                        done<=1;error<=1;error_code<=ERR_FRAME;
                    end else begin
                        base<=req_base_word;tag<=req_frame_tag;start_x<=req_x;start_y<=req_y;
                        width<=req_width;height<=req_height;column<=0;row<=0;prefix<=req_x[3:0];
                        word_addr<=req_base_word+req_y*STRIDE_WORDS+(req_x>>4);
                        words_left<=({9'd0,req_x[3:0]}+{1'b0,req_width}+13'd15)>>4;
                        state<=PREPARE;
                    end
                end
                PREPARE: if(!frame_ok) begin state<=IDLE;done<=1;error<=1;error_code<=ERR_FRAME;end
                    else begin chunk_len<=(words_left>MAX_BURST)?MAX_BURST:words_left;state<=REQUEST;end
                REQUEST: if(!frame_ok) begin state<=IDLE;done<=1;error<=1;error_code<=ERR_FRAME;end
                    else if(rd_valid && rd_ready) begin beats<=0;bad_response<=0;state<=FETCH;end
                FETCH: begin
                    if(!frame_ok) cancel<=1;
                    if(rd_data_valid) begin
                        if(beats<chunk_len && beats<MAX_BURST) data_buffer[beats]<=rd_data;
                        else bad_response<=1;
                        if(beats<63) beats<=beats+1'b1;
                    end
                    if(rd_done) begin
                        if(cancel || !frame_ok) begin state<=IDLE;done<=1;error<=1;error_code<=ERR_FRAME;end
                        else if(bad_response || observed_beats!=chunk_len) begin
                            state<=FAULT;done<=1;error<=1;error_code<=ERR_PROTOCOL;
                        end else begin
                            buffer_pixel<={5'd0,prefix};
                            chunk_pixels<=(remaining_pixels<available_pixels)?remaining_pixels:available_pixels;
                            state<=EMIT;
                        end
                    end
                end
                EMIT: if(!frame_ok) begin state<=IDLE;done<=1;error<=1;error_code<=ERR_FRAME;end
                    else if(pix_valid && pix_ready) begin
                        if(pix_last) begin state<=IDLE;done<=1;end
                        else if(chunk_pixels==1) begin
                            if(column==width-1'b1) begin
                                row<=row+1'b1;column<=0;prefix<=start_x[3:0];words_left<=row_words;
                                word_addr<=base+(start_y+row+1'b1)*STRIDE_WORDS+(start_x>>4);
                            end else begin
                                column<=column+1'b1;prefix<=0;words_left<=words_left-chunk_len;
                                word_addr<=word_addr+chunk_len;
                            end
                            state<=PREPARE;
                        end else begin
                            column<=column+1'b1;chunk_pixels<=chunk_pixels-1'b1;buffer_pixel<=buffer_pixel+1'b1;
                        end
                    end
                FAULT: state<=FAULT;
                default:state<=FAULT;
            endcase
        end
    end
endmodule
