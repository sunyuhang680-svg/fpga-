`timescale 1ns/1ps
module frame_fifo_read
#
(
	parameter MEM_DATA_BITS          = 32,
	parameter ADDR_BITS              = 23,
	parameter BUSRT_BITS             = 10,
	parameter FIFO_DEPTH             = 256,
	parameter BURST_SIZE             = 128
)               
(
	input                            rst,                  
	input                            mem_clk,                    // external memory controller user interface clock
	output reg                       rd_burst_req,               // to external memory controller,send out a burst read request  
	output reg[BUSRT_BITS - 1:0]     rd_burst_len,               // to external memory controller,data length of the burst read request, not bytes 
	output reg[ADDR_BITS - 1:0]      rd_burst_addr,              // to external memory controller,base address of the burst read request
	input                            rd_burst_data_valid,        // from external memory controller,read request data valid    
	input                            rd_burst_finish,            // from external memory controller,burst read finish
	input                            read_req,                   // data read module read request,keep '1' until read_req_ack = '1'
	output reg                       read_req_ack,               // data read module read request response
	output                           read_finish,                // data read module read request finish
	input                            read_buffer_tag,            // identifies post-processed display buffers
	output reg[1:0]                  read_index_latched,         // index actually accepted at the frame boundary
	output reg                       read_tag_latched,           // tag accepted together with read_index_latched
	input[ADDR_BITS - 1:0]           read_addr_0,                // data read module read request base address 0, used when read_addr_index = 0
	input[ADDR_BITS - 1:0]           read_addr_1,                // data read module read request base address 1, used when read_addr_index = 1
	input[ADDR_BITS - 1:0]           read_addr_2,                // data read module read request base address 1, used when read_addr_index = 2
	input[ADDR_BITS - 1:0]           read_addr_3,                // data read module read request base address 1, used when read_addr_index = 3
	input[1:0]                       read_addr_index,            // select valid base address from read_addr_0 read_addr_1 read_addr_2 read_addr_3
	input[ADDR_BITS - 1:0]           read_len,                   // data read module read request data length
	input                            window_enable,
	input[1:0]                       window_scale_shift,
	input[11:0]                      window_source_left,
	input[11:0]                      window_source_top,
	input[11:0]                      window_source_width,
	input[11:0]                      window_source_height,
	output reg                       fifo_aclr,                  // to fifo asynchronous clear
	input[15:0]                      wrusedw                     // from fifo write used words
);
localparam ONE                       = 256'd1;                   //256 bit '1'   you can use ONE[n-1:0] for n bit '1'
localparam ZERO                      = 256'd0;                   //256 bit '0'
//read state machine code
localparam S_IDLE                    = 0;                        //idle state,waiting for frame read
localparam S_ACK                     = 1;                        //read request response
localparam S_WAIT                    = 2;
localparam S_CHECK_FIFO              = 3;                        //check the FIFO status, ensure that there is enough space to burst read
localparam S_READ_BURST              = 4;                        //begin a burst read
localparam S_READ_BURST_END          = 5;                        //a burst read complete
localparam S_END                     = 6;                        //a frame of data is read to complete

reg                                  read_req_d0;                //asynchronous read request, synchronize to 'mem_clk' clock domain,first beat
reg                                  read_req_d1;                //second
reg                                  read_req_d2;                //third,Why do you need 3 ? Here's the design habit
reg[ADDR_BITS - 1:0]                 read_len_d0;                //asynchronous read_len(read data length), synchronize to 'mem_clk' clock domain first
reg[ADDR_BITS - 1:0]                 read_len_d1;                //second
reg[ADDR_BITS - 1:0]                 read_len_latch;             //lock read data length
reg[ADDR_BITS - 1:0]                 read_cnt;                   //read data counter
reg[3:0]                             state;                      //state machine
reg[1:0]                             read_addr_index_d0;         //synchronize to 'mem_clk' clock domain first
reg[1:0]                             read_addr_index_d1;         //synchronize to 'mem_clk' clock domain second
reg                                  read_buffer_tag_d0;
reg                                  read_buffer_tag_d1;
reg[15:0]                            wait_cnt;
reg                                  window_enable_d0,window_enable_d1,window_enable_latch;
reg[1:0]                             window_scale_d0,window_scale_d1,window_scale_latch;
reg[11:0]                            window_left_d0,window_left_d1,window_top_d0,window_top_d1;
reg[11:0]                            window_width_d0,window_width_d1,window_height_d0,window_height_d1;
reg[11:0]                            window_left_latch,window_top_latch,window_rows_total,window_output_row;
reg[6:0]                             window_line_words,window_row_words;
reg[ADDR_BITS-1:0]                   window_frame_base;
localparam [ADDR_BITS-1:0] SOURCE_WORDS_PER_LINE=40;
wire[ADDR_BITS-1:0] selected_read_base=(read_addr_index_d1==2'd0)?read_addr_0:
    (read_addr_index_d1==2'd1)?read_addr_1:
    (read_addr_index_d1==2'd2)?read_addr_2:read_addr_3;
wire[6:0] window_words_remaining=window_line_words-window_row_words;
wire[BUSRT_BITS-1:0] window_next_burst=(window_words_remaining>BURST_SIZE)?
    BURST_SIZE[BUSRT_BITS-1:0]:window_words_remaining;
wire[11:0] window_next_source_row=(window_output_row+1'b1)>>window_scale_latch;
wire[ADDR_BITS-1:0] window_next_row_addr=window_frame_base+
    (window_top_latch+window_next_source_row)*SOURCE_WORDS_PER_LINE+
    {8'd0,window_left_latch[11:4]};
assign read_finish = (state == S_END) ? 1'b1 : 1'b0;             //read finish at state 'S_END'
always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		read_req_d0    <=  1'b0;
		read_req_d1    <=  1'b0;
		read_req_d2    <=  1'b0;
		read_len_d0    <=  ZERO[ADDR_BITS - 1:0];               //equivalent to read_len_d0 <= 0;
		read_len_d1    <=  ZERO[ADDR_BITS - 1:0];               //equivalent to read_len_d1 <= 0;
		read_addr_index_d0 <= 2'b00;
		read_addr_index_d1 <= 2'b00;
		read_buffer_tag_d0 <= 1'b0;
		read_buffer_tag_d1 <= 1'b0;
		window_enable_d0 <= 1'b0;window_enable_d1 <= 1'b0;
		window_scale_d0<=0;window_scale_d1<=0;
		window_left_d0<=0;window_left_d1<=0;window_top_d0<=0;window_top_d1<=0;
		window_width_d0<=0;window_width_d1<=0;window_height_d0<=0;window_height_d1<=0;
	end
	else
	begin
		read_req_d0    <=  read_req;
		read_req_d1    <=  read_req_d0;
		read_req_d2    <=  read_req_d1;     
		read_len_d0    <=  read_len;
		read_len_d1    <=  read_len_d0; 
		read_addr_index_d0 <= read_addr_index;
		read_addr_index_d1 <= read_addr_index_d0;
		read_buffer_tag_d0 <= read_buffer_tag;
		read_buffer_tag_d1 <= read_buffer_tag_d0;
		window_enable_d0<=window_enable;window_enable_d1<=window_enable_d0;
		window_scale_d0<=window_scale_shift;window_scale_d1<=window_scale_d0;
		window_left_d0<=window_source_left;window_left_d1<=window_left_d0;
		window_top_d0<=window_source_top;window_top_d1<=window_top_d0;
		window_width_d0<=window_source_width;window_width_d1<=window_width_d0;
		window_height_d0<=window_source_height;window_height_d1<=window_height_d0;
	end 
end


always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		state <= S_IDLE;
		read_len_latch <= ZERO[ADDR_BITS - 1:0];
		rd_burst_addr <= ZERO[ADDR_BITS - 1:0];
		rd_burst_req <= 1'b0;
		read_cnt <= ZERO[ADDR_BITS - 1:0];
		fifo_aclr <= 1'b0;
		rd_burst_len <= ZERO[BUSRT_BITS - 1:0];
		read_req_ack <= 1'b0;
		read_index_latched <= 2'b00;
		read_tag_latched <= 1'b0;
		wait_cnt <= 16'd0;
		window_enable_latch<=0;window_scale_latch<=0;window_left_latch<=0;window_top_latch<=0;
		window_rows_total<=0;window_output_row<=0;window_line_words<=0;window_row_words<=0;
		window_frame_base<=0;
	end
	else
		case(state)
			//idle state,waiting for read, read_req_d2 == '1' goto the 'S_ACK'
			S_IDLE:
			begin
				if(read_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				read_req_ack <= 1'b0;
			end
			//'S_ACK' state completes the read request response, the FIFO reset, the address latch, and the data length latch
			S_ACK:
			begin
				if(read_req_d2 == 1'b0)
				begin
					state <= S_WAIT;
					wait_cnt <= 16'd0;
					fifo_aclr <= 1'b0;
					read_req_ack <= 1'b0;
				end
				else
				begin
					//read request response
					read_req_ack <= 1'b1;
					//FIFO reset
					fifo_aclr <= 1'b1;
					window_enable_latch<=window_enable_d1;
					window_scale_latch<=window_scale_d1;
					window_left_latch<=window_left_d1;
					window_top_latch<=window_top_d1;
					window_rows_total<=window_height_d1<<window_scale_d1;
					window_output_row<=0;
					window_line_words<=({8'd0,window_left_d1[3:0]}+window_width_d1+12'd15)>>4;
					window_row_words<=0;
					window_frame_base<=selected_read_base;
					// Publish the exact buffer identity only when the display
					// reader has really accepted a new frame request.  The
					// renderer uses this as its safe-to-reuse acknowledgement.
					read_index_latched<=read_addr_index_d1;
					read_tag_latched<=read_buffer_tag_d1;
					if(window_enable_d1)
						rd_burst_addr<=selected_read_base+
							window_top_d1*SOURCE_WORDS_PER_LINE+{8'd0,window_left_d1[11:4]};
					else rd_burst_addr<=selected_read_base;
					//latch data length
					read_len_latch <= read_len_d1;
				end
				//read data counter reset, read_cnt <= 0;
				read_cnt <= ZERO[ADDR_BITS - 1:0];
			end
			S_WAIT:
			begin
				if(wait_cnt >= 16'd200)
				begin
					state <= S_CHECK_FIFO;
				end
				else
				begin
					wait_cnt <= wait_cnt + 16'd1;
				end
			end
			S_CHECK_FIFO:
			begin
				//if there is a read request at this time, enter the 'S_ACK' state
				if(read_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				//if the FIFO space is a burst read request, goto burst read state
				else if(window_enable_latch && window_output_row>=window_rows_total)
				begin
					state<=S_END;
				end
				else if(window_enable_latch && window_line_words!=0 &&
					wrusedw < (FIFO_DEPTH - window_next_burst - 2))
				begin
					state<=S_READ_BURST;
					rd_burst_len<=window_next_burst;
					rd_burst_req<=1'b1;
				end
				else if(!window_enable_latch && wrusedw < (FIFO_DEPTH - BURST_SIZE - 2))
				begin
					state <= S_READ_BURST;
					rd_burst_len <= BURST_SIZE[BUSRT_BITS - 1:0];
					rd_burst_req <= 1'b1;
				end
			end
			
			S_READ_BURST:
			begin
				if(rd_burst_data_valid)
					rd_burst_req <= 1'b0;
				//burst finish  
				if(rd_burst_finish == 1'b1)
				begin
					state <= S_READ_BURST_END;
					//read counter + burst length
					if(window_enable_latch) begin
						if(window_row_words+rd_burst_len>=window_line_words) begin
							window_row_words<=0;
							window_output_row<=window_output_row+1'b1;
							rd_burst_addr<=window_next_row_addr;
						end else begin
							window_row_words<=window_row_words+rd_burst_len;
							rd_burst_addr<=rd_burst_addr+rd_burst_len;
						end
					end else begin
						read_cnt <= read_cnt + BURST_SIZE[ADDR_BITS - 1:0];
						rd_burst_addr <= rd_burst_addr + BURST_SIZE[ADDR_BITS - 1:0];
					end
				end     
			end
			S_READ_BURST_END:
			begin
				//if there is a read request at this time, enter the 'S_ACK' state
				if(read_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				//if the read counter value is less than the frame length, continue read,
				//otherwise the read is complete
				else if(window_enable_latch && window_output_row<window_rows_total)
				begin
					state<=S_CHECK_FIFO;
				end
				else if(!window_enable_latch && read_cnt < read_len_latch)
				begin
					state <= S_CHECK_FIFO;
				end
				else
				begin
					state <= S_END;
				end
			end
			S_END:
			begin
				state <= S_IDLE;
			end
			default:
				state <= S_IDLE;
		endcase
end
endmodule

// Build one complete 640x720 display frame in a separate DDR area.  Source
// rows are fetched into a small line store, transformed at one pixel per
// mem_clk, then written as a contiguous forty-word row.  The HDMI reader only
// sees completed front buffers and therefore never performs windowed/random
// reads in the pixel-clock domain.
module frame_view_render #(
	parameter ADDR_BITS = 25,
	parameter BUSRT_BITS = 10,
	parameter [ADDR_BITS-1:0] OUTPUT_BASE0 = 0,
	parameter [ADDR_BITS-1:0] OUTPUT_BASE1 = 0,
	parameter integer FRAME_HEIGHT = 720,
	// Diagnostic mode: build RGB565 color bars locally and issue no source
	// DDR reads.  This isolates processed-buffer write/read/display integrity.
	parameter integer TEST_PATTERN = 0,
	// 1: fine-grained inverse mapping; 0 retains the legacy crop/shift tests.
	parameter integer USE_AFFINE = 0
)(
	input wire mem_clk,
	input wire rst,
	input wire source_valid,
	input wire [ADDR_BITS-1:0] source_base,
	input wire [1:0] scale_shift,
	input wire [11:0] source_left,
	input wire [11:0] source_top,
	input wire [11:0] source_width,
	input wire [11:0] source_height,
	input wire [11:0] destination_left,
	input wire [11:0] destination_top,
	input wire apply_binary,
	input wire [47:0] adjust_values,
	input wire [3:0] doodle_index,
	output wire [16:0] doodle_address,
	input wire [16:0] view_step_q16,
	input wire [1:0] view_rotation,
	input wire signed [27:0] view_origin_x,
	input wire signed [27:0] view_origin_y,
	input wire display_index_async,
	input wire display_accept_valid_async,

	output reg front_index,
	output reg frame_ready,
	output reg rd_burst_req,
	output reg [BUSRT_BITS-1:0] rd_burst_len,
	output reg [ADDR_BITS-1:0] rd_burst_addr,
	input wire rd_burst_data_valid,
	input wire [255:0] rd_burst_data,
	input wire rd_burst_finish,
	output reg wr_burst_req,
	output reg [BUSRT_BITS-1:0] wr_burst_len,
	output reg [ADDR_BITS-1:0] wr_burst_addr,
	input wire wr_burst_data_req,
	output wire [255:0] wr_burst_data,
	input wire wr_burst_finish
);
	localparam [4:0] S_IDLE=0,S_LATCH=1,S_PREPARE=2,S_READ=3,
		S_GENERATE=4,S_WRITE=5,S_NEXT_ROW=6,S_PUBLISH=7,S_WAIT_DISPLAY=8,
		S_STORE_PIXEL=9,S_ROT_REQUEST=10,S_ROT_READ=11;
	localparam [11:0] FRAME_WIDTH=12'd640;
	localparam [6:0] WORDS_PER_LINE=7'd40;

	reg [4:0] state;
	reg back_index;
	reg display_index_meta,display_index_sync;
	reg display_accept_valid_meta,display_accept_valid_sync;
	reg [ADDR_BITS-1:0] source_base_latch;
	reg [1:0] scale_latch;
	reg [11:0] source_left_latch,source_top_latch;
	reg [11:0] source_width_latch,source_height_latch;
	reg [11:0] destination_left_latch,destination_top_latch;
	reg binary_latch;
	reg [11:0] output_row;
	reg [11:0] output_x;
	reg row_has_source;
	reg [3:0] line_prefix;
	reg [6:0] read_store_count;
	reg [6:0] read_words_remaining;
	reg [5:0] write_word_index;
	reg [6:0] write_words_remaining;
	reg [255:0] wr_burst_data_hold;
	reg [255:0] pixel_pack;
	reg [15:0] raw_pixel_stage;
	reg raw_valid_stage;
	reg [9:0] pixel_x_stage;
	reg [255:0] source_line [0:40];
	reg [255:0] output_line [0:39];
	reg [16:0] affine_step;
	reg [1:0] rotation_latch;
	reg signed [27:0] affine_origin_x;
	reg signed [31:0] affine_x,affine_y;
	reg [5:0] affine_first_word;
	wire signed [31:0] affine_last_x=$signed(affine_origin_x)+$signed({1'b0,affine_step})*32'sd639;
	wire [9:0] affine_first_pixel=affine_origin_x[27]?10'd0:affine_origin_x[25:16];
	wire [9:0] affine_last_pixel=(affine_last_x>=32'sd41943040)?10'd639:affine_last_x[25:16];
	wire [6:0] affine_read_words={1'b0,affine_last_pixel[9:4]}-
		{1'b0,affine_first_pixel[9:4]}+7'd1;
	wire [5:0] affine_word_index=affine_x[25:20]-affine_first_word;
	wire affine_row_valid=affine_y>=0 && affine_y<32'sd47185920 &&
		affine_origin_x<28'sd41943040 && affine_last_x>=0;

	wire [13:0] display_width={2'b0,source_width_latch}<<scale_latch;
	wire [13:0] display_height={2'b0,source_height_latch}<<scale_latch;
	wire [12:0] relative_x={1'b0,output_x}-{1'b0,destination_left_latch};
	wire [12:0] source_pixel_offset={9'd0,line_prefix}+(relative_x>>scale_latch);
	wire [12:0] requested_source_words_wide={9'd0,source_left_latch[3:0]}+
		{1'b0,source_width_latch}+13'd15;
	wire [6:0] requested_source_words=requested_source_words_wide[12:4];
	reg [15:0] generated_pixel;
	reg [15:0] processing_pixel;
	reg [6:0] pixel_sum;
	reg [47:0] adjust_latch;
	reg [15:0] adjusted_pixel;
	reg signed [13:0] work_r,work_g,work_b;
	reg signed [14:0] lum,delta;
	reg signed [14:0] previous_lum;
	wire [7:0] exposure_value=adjust_latch[7:0];
	wire [7:0] contrast_value=adjust_latch[15:8];
	wire [7:0] sharpen_value=adjust_latch[23:16];
	wire [7:0] bright_value=adjust_latch[31:24];
	wire [7:0] satur_value=adjust_latch[39:32];
	wire [7:0] temp_value=adjust_latch[47:40];
	// Deliberately wide demonstration range, with 128 exactly equal to 1.0.
	// Exposure spans 0..about 4x; contrast spans 0..about 3x.
	wire [10:0] exposure_gain=(exposure_value>=8'd128)?
		11'd256+(exposure_value-8'd128)*11'd6:{2'b0,exposure_value,1'b0};
	wire [9:0] contrast_gain=(contrast_value>=8'd128)?
		10'd256+(contrast_value-8'd128)*10'd4:{1'b0,contrast_value,1'b0};
	function [5:0] clip6; input signed [14:0] v; begin clip6=v<0?0:(v>63?63:v[5:0]); end endfunction
	function [4:0] clip5; input signed [14:0] v; begin clip5=v<0?0:(v>31?31:v[4:0]); end endfunction
	function [15:0] palette565; input [2:0] c; begin
		case(c)
		3'd0:palette565=16'hf800; 3'd1:palette565=16'hfd20;
		3'd2:palette565=16'hffe0; 3'd3:palette565=16'h07e0;
		3'd4:palette565=16'h07ff; 3'd5:palette565=16'h001f;
		3'd6:palette565=16'h781f; default:palette565=16'hffff;
		endcase
	end endfunction
	wire [9:0] doodle_source_x=affine_x[31] ? 10'd0 : affine_x[25:16];
	wire [9:0] doodle_source_y=affine_y[31] ? 10'd0 : affine_y[25:16];
	assign doodle_address=doodle_source_y[9:1]*17'd320+doodle_source_x[9:1];

	// The AXI bridge asserts wr_burst_data_req one clock before consuming the
	// corresponding 256-bit word.  Match the registered-output behaviour of
	// the original camera FIFO: capture the addressed word on the request and
	// hold it stable for the following AXI data beat.
	assign wr_burst_data=wr_burst_data_hold;

	always @* begin
		generated_pixel=16'h0000;
		processing_pixel=raw_pixel_stage;
		pixel_sum={2'b0,raw_pixel_stage[15:11]}+
			{1'b0,raw_pixel_stage[10:5]}+{2'b0,raw_pixel_stage[4:0]};
		if(TEST_PATTERN!=0) begin
			// Eight 80-pixel vertical bars.  The first bar is pure white so
			// periodic brightening/dimming remains visible on a known source.
			if(pixel_x_stage<10'd80)       generated_pixel=16'hffff; // white
			else if(pixel_x_stage<10'd160) generated_pixel=16'hffe0; // yellow
			else if(pixel_x_stage<10'd240) generated_pixel=16'h07ff; // cyan
			else if(pixel_x_stage<10'd320) generated_pixel=16'h07e0; // green
			else if(pixel_x_stage<10'd400) generated_pixel=16'hf81f; // magenta
			else if(pixel_x_stage<10'd480) generated_pixel=16'hf800; // red
			else if(pixel_x_stage<10'd560) generated_pixel=16'h001f; // blue
			else                            generated_pixel=16'h0000; // black
		end else if(!raw_valid_stage)
			generated_pixel=16'h0000;
		else begin
			// Existing algorithm output is the input of this final adjustment
			// stage. Therefore compare/binary mode, sliders and doodle compose
			// instead of bypassing one another.
			processing_pixel=binary_latch ?
				((pixel_sum>=7'd48)?16'hffff:16'h0000) : raw_pixel_stage;
			work_r=$signed({1'b0,processing_pixel[15:11],1'b0});
			work_g=$signed({1'b0,processing_pixel[10:5]});
			work_b=$signed({1'b0,processing_pixel[4:0],1'b0});
			work_r=(work_r*$signed({1'b0,exposure_gain}))>>>8;
			work_g=(work_g*$signed({1'b0,exposure_gain}))>>>8;
			work_b=(work_b*$signed({1'b0,exposure_gain}))>>>8;
			work_r=work_r+(($signed({1'b0,bright_value})-15'sd128)>>>1);
			work_g=work_g+(($signed({1'b0,bright_value})-15'sd128)>>>1);
			work_b=work_b+(($signed({1'b0,bright_value})-15'sd128)>>>1);
			work_r=15'sd32+(((work_r-15'sd32)*$signed({1'b0,contrast_gain}))>>>8);
			work_g=15'sd32+(((work_g-15'sd32)*$signed({1'b0,contrast_gain}))>>>8);
			work_b=15'sd32+(((work_b-15'sd32)*$signed({1'b0,contrast_gain}))>>>8);
			lum=(work_r+(work_g<<<1)+work_b)>>>2;
			delta=$signed({1'b0,satur_value})-15'sd128;
			work_r=work_r+(((work_r-lum)*delta)>>>7);
			work_g=work_g+(((work_g-lum)*delta)>>>7);
			work_b=work_b+(((work_b-lum)*delta)>>>7);
			// One-dimensional unsharp detail uses the previous source sample.
			// It is inexpensive, streaming-friendly and leaves the frame bit
			// exact at the neutral position.
			delta=$signed({1'b0,sharpen_value})-15'sd128;
			if(pixel_x_stage!=0) begin
				work_r=work_r+(((lum-previous_lum)*delta)>>>6);
				work_g=work_g+(((lum-previous_lum)*delta)>>>6);
				work_b=work_b+(((lum-previous_lum)*delta)>>>6);
			end
			work_r=work_r+(($signed({1'b0,temp_value})-15'sd128)>>>1);
			work_b=work_b-(($signed({1'b0,temp_value})-15'sd128)>>>1);
			adjusted_pixel={clip5(work_r>>>1),clip6(work_g),clip5(work_b>>>1)};
			generated_pixel=doodle_index[3]?palette565(doodle_index[2:0]):
				((adjust_latch==48'h808080808080)?processing_pixel:adjusted_pixel);
		end
	end

	always @(posedge mem_clk or posedge rst) begin
		if(rst) begin
			display_index_meta<=0;
			display_index_sync<=0;
			display_accept_valid_meta<=0;
			display_accept_valid_sync<=0;
		end else begin
			display_index_meta<=display_index_async;
			display_index_sync<=display_index_meta;
			display_accept_valid_meta<=display_accept_valid_async;
			display_accept_valid_sync<=display_accept_valid_meta;
		end
	end

	always @(posedge mem_clk or posedge rst) begin
		if(rst) begin
			state<=S_IDLE;
			front_index<=1'b0;
			back_index<=1'b1;
			frame_ready<=1'b0;
			rd_burst_req<=1'b0;
			rd_burst_len<=0;
			rd_burst_addr<=0;
			wr_burst_req<=1'b0;
			wr_burst_len<=0;
			wr_burst_addr<=0;
			write_word_index<=0;
			write_words_remaining<=0;
			wr_burst_data_hold<=0;
			read_store_count<=0;
			read_words_remaining<=0;
			output_row<=0;
			output_x<=0;
			row_has_source<=0;
			line_prefix<=0;
			pixel_pack<=0;
			raw_pixel_stage<=0;
			raw_valid_stage<=0;
			previous_lum<=0;
			pixel_x_stage<=0;
			source_base_latch<=0;
			scale_latch<=0;
			source_left_latch<=0;
			source_top_latch<=0;
			source_width_latch<=0;
			source_height_latch<=0;
			destination_left_latch<=0;
			destination_top_latch<=0;
			binary_latch<=0;
			adjust_latch<=48'h808080808080;
			rotation_latch<=0;
			affine_step<=17'd65536;affine_origin_x<=0;
			affine_x<=0;affine_y<=0;affine_first_word<=0;
		end else begin
			case(state)
				S_IDLE: begin
					rd_burst_req<=1'b0;
					wr_burst_req<=1'b0;
					if(source_valid)
						state<=S_LATCH;
				end
				S_LATCH: begin
					source_base_latch<=source_base;
					scale_latch<=scale_shift;
					source_left_latch<=source_left;
					source_top_latch<=source_top;
					source_width_latch<=source_width;
					source_height_latch<=source_height;
					destination_left_latch<=destination_left;
					destination_top_latch<=destination_top;
					binary_latch<=apply_binary;
					adjust_latch<=adjust_values;
					rotation_latch<=view_rotation;
					affine_step<=(view_step_q16<17'd16384 || view_step_q16>17'd65536)?
						17'd65536:view_step_q16;
					affine_origin_x<=view_origin_x;
					affine_y<=$signed(view_origin_y);
					back_index<=~front_index;
					output_row<=0;
					state<=S_PREPARE;
				end
				S_PREPARE: begin
					read_store_count<=0;
					write_word_index<=0;
					output_x<=0;
					pixel_pack<=0;
					affine_x<=$signed(affine_origin_x);
					line_prefix<=source_left_latch[3:0];
					if(TEST_PATTERN!=0) begin
						// Completely sever the raw-frame -> processed-frame read
						// path while retaining the real DDR write and display path.
						row_has_source<=1'b0;
						read_words_remaining<=0;
						rd_burst_req<=1'b0;
						state<=S_GENERATE;
					end else if(USE_AFFINE!=0 && rotation_latch!=0) begin
						// A quarter turn maps an output row to a source column.
						// Keep the full rotated image aspect ratio: the 720-wide
						// result is center-cropped to 640, with 40 black rows.
						if((rotation_latch==2'd1 || rotation_latch==2'd3) &&
						   (output_row<12'd40 || output_row>=12'd680)) begin
							row_has_source<=1'b0;state<=S_GENERATE;
						end else begin
							row_has_source<=1'b1;state<=S_ROT_REQUEST;
						end
					end else if(USE_AFFINE!=0) begin
						// Clip fetches to the source image, then sample with a
						// fractional accumulator. Out-of-image pixels are black.
						// One line buffer suffices for nearest-neighbour scaling;
						// DDR writes and sequential display reads stay unchanged.
						if(affine_row_valid) begin
							row_has_source<=1'b1;
							affine_first_word<=affine_first_pixel[9:4];
							read_words_remaining<=affine_read_words;
							rd_burst_len<=(affine_read_words>16)?10'd16:affine_read_words;
							rd_burst_addr<=source_base_latch+
								(affine_y>>>16)*40+affine_first_pixel[9:4];
							rd_burst_req<=1'b1;state<=S_READ;
						end else begin
							row_has_source<=1'b0;state<=S_GENERATE;
						end
					end else if(output_row>=destination_top_latch &&
						{2'b0,output_row}<({2'b0,destination_top_latch}+display_height)) begin
						row_has_source<=1'b1;
					read_words_remaining<=requested_source_words;
					// The DDR3 IP exposes only a four-bit AXI LEN field, so every
					// local burst must contain at most sixteen 256-bit words.
					rd_burst_len<=(requested_source_words>7'd16)?10'd16:
						requested_source_words;
						rd_burst_addr<=source_base_latch+
							(source_top_latch+((output_row-destination_top_latch)>>scale_latch))*40+
							{9'd0,source_left_latch[11:4]};
						rd_burst_req<=1'b1;
						state<=S_READ;
					end else begin
						row_has_source<=1'b0;
						state<=S_GENERATE;
					end
				end
				S_READ: begin
					if(rd_burst_data_valid) begin
						source_line[read_store_count]<=rd_burst_data;
						read_store_count<=read_store_count+1'b1;
						rd_burst_req<=1'b0;
					end
					if(rd_burst_finish) begin
					if(read_words_remaining>rd_burst_len) begin
						read_words_remaining<=read_words_remaining-rd_burst_len;
						rd_burst_addr<=rd_burst_addr+rd_burst_len;
						if((read_words_remaining-rd_burst_len)>7'd16)
							rd_burst_len<=10'd16;
						else
							rd_burst_len<=read_words_remaining-rd_burst_len;
						rd_burst_req<=1'b1;
					end else begin
						read_words_remaining<=0;
						rd_burst_req<=1'b0;
						output_x<=0;
						state<=S_GENERATE;
					end
					end
				end
				S_ROT_REQUEST: begin : rotate_request
					reg [11:0] rotate_source_x;
					reg [11:0] rotate_source_y;
					if(rotation_latch==2'd1) begin
						rotate_source_x=output_row-12'd40;
						rotate_source_y=12'd679-output_x;
					end else if(rotation_latch==2'd3) begin
						rotate_source_x=12'd679-output_row;
						rotate_source_y=12'd40+output_x;
					end else begin
						rotate_source_x=12'd639-output_x;
						rotate_source_y=12'd719-output_row;
					end
					affine_x<=$signed({rotate_source_x,16'd0});
					affine_y<=$signed({rotate_source_y,16'd0});
					rd_burst_len<=10'd1;
					rd_burst_addr<=source_base_latch+rotate_source_y*40+
						rotate_source_x[11:4];
					rd_burst_req<=1'b1;state<=S_ROT_READ;
				end
				S_ROT_READ: begin
					if(rd_burst_data_valid) begin
						raw_pixel_stage<=rd_burst_data[affine_x[19:16]*16 +: 16];
						raw_valid_stage<=1'b1;pixel_x_stage<=output_x[9:0];
						rd_burst_req<=1'b0;
					end
					if(rd_burst_finish) begin
						rd_burst_req<=1'b0;state<=S_STORE_PIXEL;
					end
				end
				S_GENERATE: begin
					// Register the line-store lookup before thresholding and
					// packing.  The original coordinate-to-pixel path missed the
					// routed core-clock timing even though RTL simulation passed.
					pixel_x_stage<=output_x[9:0];
					if(USE_AFFINE!=0) begin
						raw_valid_stage<=row_has_source && affine_x>=0 && affine_x<32'sd41943040;
						if(row_has_source && affine_x>=0 && affine_x<32'sd41943040)
							raw_pixel_stage<=source_line[affine_word_index][affine_x[19:16]*16 +: 16];
						else raw_pixel_stage<=16'h0000;
					end else if(row_has_source && output_x>=destination_left_latch &&
						{2'b0,output_x}<({2'b0,destination_left_latch}+display_width))
						begin raw_valid_stage<=1'b1;raw_pixel_stage<=source_line[source_pixel_offset[9:4]]
							[source_pixel_offset[3:0]*16 +: 16];end
					else begin raw_valid_stage<=1'b0;
						raw_pixel_stage<=16'h0000;
					end
					state<=S_STORE_PIXEL;
				end
				S_STORE_PIXEL: begin
					previous_lum<=({9'd0,processing_pixel[15:11],1'b0}+
						{8'd0,processing_pixel[10:5]}+{9'd0,processing_pixel[4:0],1'b0})>>>2;
					if(pixel_x_stage[3:0]==4'd15)
						output_line[pixel_x_stage[9:4]]<={generated_pixel,pixel_pack[239:0]};
					else
						pixel_pack[pixel_x_stage[3:0]*16 +: 16]<=generated_pixel;
					if(pixel_x_stage==FRAME_WIDTH-1'b1) begin
						write_word_index<=0;
						write_words_remaining<=WORDS_PER_LINE;
						wr_burst_len<=10'd16;
						wr_burst_addr<=(back_index?OUTPUT_BASE1:OUTPUT_BASE0)+output_row*40;
						wr_burst_req<=1'b1;
						state<=S_WRITE;
					end else if(rotation_latch!=0 && row_has_source) begin
						output_x<=output_x+1'b1;
						state<=S_ROT_REQUEST;
					end else begin
						output_x<=output_x+1'b1;
						affine_x<=affine_x+$signed({1'b0,affine_step});
						state<=S_GENERATE;
					end
				end
				S_WRITE: begin
					if(wr_burst_data_req) begin
						wr_burst_data_hold<=output_line[write_word_index];
						wr_burst_req<=1'b0;
						if(write_word_index<WORDS_PER_LINE-1'b1)
							write_word_index<=write_word_index+1'b1;
					end
					if(wr_burst_finish) begin
					if(write_words_remaining>wr_burst_len) begin
						write_words_remaining<=write_words_remaining-wr_burst_len;
						wr_burst_addr<=wr_burst_addr+wr_burst_len;
						if((write_words_remaining-wr_burst_len)>7'd16)
							wr_burst_len<=10'd16;
						else
							wr_burst_len<=write_words_remaining-wr_burst_len;
						wr_burst_req<=1'b1;
					end else begin
						write_words_remaining<=0;
						wr_burst_req<=1'b0;
						state<=S_NEXT_ROW;
					end
					end
				end
				S_NEXT_ROW: begin
					if(output_row==FRAME_HEIGHT-1'b1)
						state<=S_PUBLISH;
					else begin
						output_row<=output_row+1'b1;
						affine_y<=affine_y+$signed({1'b0,affine_step});
						state<=S_PREPARE;
					end
				end
				S_PUBLISH: begin
					front_index<=back_index;
					frame_ready<=1'b1;
					state<=S_WAIT_DISPLAY;
				end
				S_WAIT_DISPLAY: begin
					// Do not overwrite the old front buffer until the HDMI
					// reader has latched both the rendered-buffer tag and the
					// newly published index at an actual frame request.
					if(display_accept_valid_sync && display_index_sync==front_index)
						state<=S_LATCH;
				end
				default: state<=S_IDLE;
			endcase
		end
	end
endmodule
