module frame_view_render_retimed #(
	parameter ADDR_BITS = 25,
	parameter BUSRT_BITS = 10,
	parameter [ADDR_BITS-1:0] OUTPUT_BASE0 = 0,
	parameter [ADDR_BITS-1:0] OUTPUT_BASE1 = 0,
	parameter integer FRAME_HEIGHT = 720,
	// Diagnostic mode: build RGB565 color bars locally and issue no source
	// DDR reads.  This isolates processed-buffer write/read/display integrity.
	parameter integer TEST_PATTERN = 0,
	// 1: fine-grained inverse mapping; 0 retains the legacy crop/shift tests.
	parameter integer USE_AFFINE = 0,
	// Lowest accepted horizontal step.  16384 keeps the legacy 4x limit; the
	// CMP fullscreen view samples 640 source columns over 1280 display
	// columns and therefore needs half of the vertical step (down to 8192).
	parameter integer MIN_STEP_X = 16384
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
	input wire [16:0] view_step_x_q16,
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
		S_STORE_PIXEL=9,S_ROT_REQUEST=10,S_ROT_READ=11,S_DRAIN_ADJUST=12,
        S_AFFINE_ENDPOINT=13;
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
    // The preserved ingress stage drives only its adjacent second-stage
    // registers, allowing placement near the upstream arbiter mux. Payload,
    // valid and finish advance together, including one-beat last transfers.
    // The native request still withdraws on RAW valid below, not this pipeline.
    // Timing benefit is a placement hypothesis pending a new full PNR/STA.
    (* syn_preserve=1 *) reg[255:0]read_data_input_q;
    (* syn_preserve=1 *) reg read_valid_input_q,read_finish_input_q;
    reg read_valid_q,read_finish_q;
    reg[255:0]read_data_q;
    always @(posedge mem_clk)begin
        read_data_input_q<=rd_burst_data;
        read_data_q<=read_data_input_q;
    end
    always @(posedge mem_clk or posedge rst)begin
        if(rst)begin
            read_valid_input_q<=0;read_finish_input_q<=0;
            read_valid_q<=0;read_finish_q<=0;
        end else begin
            read_valid_input_q<=rd_burst_data_valid;read_finish_input_q<=rd_burst_finish;
            read_valid_q<=read_valid_input_q;read_finish_q<=read_finish_input_q;
        end
    end
	reg [16:0] affine_step,affine_step_x;
	reg [1:0] rotation_latch;
	reg signed [27:0] affine_origin_x;
	reg signed [31:0] affine_x,affine_y;
	reg [5:0] affine_first_word;
    // Geometry is constant through a frame. Register the endpoint before
    // clamp/range/burst decode, once per frame, to break the DSP->DDR chain.
    wire signed [31:0] affine_last_x_next=$signed(affine_origin_x)+$signed({1'b0,affine_step_x})*32'sd639;
    reg signed [31:0] affine_last_x;
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
	wire [15:0] generated_pixel;
	wire adjust_valid;
	wire [9:0] adjust_x;
	reg [47:0] adjust_latch;
	pixel_adjust_pipeline #(.TEST_PATTERN(TEST_PATTERN)) adjust_pipeline(
	    .clk(mem_clk),.rst(rst),.in_valid(state==S_STORE_PIXEL),.raw_valid(raw_valid_stage),
	    .binary_mode(binary_latch),.raw_pixel(raw_pixel_stage),.pixel_x(pixel_x_stage),
	    .adjust_values(adjust_latch),.doodle_index(doodle_index),.out_valid(adjust_valid),.out_x(adjust_x),.out_pixel(generated_pixel));
	wire [9:0] doodle_source_x=affine_x[31] ? 10'd0 : affine_x[25:16];
	wire [9:0] doodle_source_y=affine_y[31] ? 10'd0 : affine_y[25:16];
	assign doodle_address=doodle_source_y[9:1]*17'd320+doodle_source_x[9:1];

	// The AXI bridge asserts wr_burst_data_req one clock before consuming the
	// corresponding 256-bit word.  Match the registered-output behaviour of
	// the original camera FIFO: capture the addressed word on the request and
	// hold it stable for the following AXI data beat.
	assign wr_burst_data=wr_burst_data_hold;

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
			affine_step<=17'd65536;affine_step_x<=17'd65536;affine_origin_x<=0;
			affine_x<=0;affine_y<=0;affine_first_word<=0;affine_last_x<=0;
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
					affine_step_x<=(view_step_x_q16<MIN_STEP_X || view_step_x_q16>17'd65536)?
						17'd65536:view_step_x_q16;
					affine_origin_x<=view_origin_x;
					affine_y<=$signed(view_origin_y);
					back_index<=~front_index;
					output_row<=0;
                    state<=(USE_AFFINE!=0 && TEST_PATTERN==0)?S_AFFINE_ENDPOINT:S_PREPARE;
                end
                S_AFFINE_ENDPOINT: begin
                    affine_last_x<=affine_last_x_next;
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
					if(read_valid_q) begin
						source_line[read_store_count]<=read_data_q;
						read_store_count<=read_store_count+1'b1;
						rd_burst_req<=1'b0;
					end
					if(read_finish_q) begin
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
					if(read_valid_q) begin
						raw_pixel_stage<=read_data_q[affine_x[19:16]*16 +: 16];
						raw_valid_stage<=1'b1;pixel_x_stage<=output_x[9:0];
						rd_burst_req<=1'b0;
					end
					if(read_finish_q) begin
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
				    if(pixel_x_stage==FRAME_WIDTH-1'b1)state<=S_DRAIN_ADJUST;
				    else if(rotation_latch!=0 && row_has_source)begin output_x<=output_x+1'b1;state<=S_ROT_REQUEST;end
				    else begin output_x<=output_x+1'b1;affine_x<=affine_x+$signed({1'b0,affine_step_x});state<=S_GENERATE;end
				end
				S_DRAIN_ADJUST:if(adjust_valid && adjust_x==FRAME_WIDTH-1'b1)begin
				    write_word_index<=0;write_words_remaining<=WORDS_PER_LINE;wr_burst_len<=10'd16;
				    wr_burst_addr<=(back_index?OUTPUT_BASE1:OUTPUT_BASE0)+output_row*40;
				    wr_burst_req<=1'b1;state<=S_WRITE;
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
            // Native arbiter must see withdrawal before a one-beat finish.
            if((state==S_READ || state==S_ROT_READ) && rd_burst_data_valid)
                rd_burst_req<=1'b0;
            if(adjust_valid)begin
			    if(adjust_x[3:0]==4'd15)output_line[adjust_x[9:4]]<={generated_pixel,pixel_pack[239:0]};
			    else pixel_pack[adjust_x[3:0]*16+:16]<=generated_pixel;
			end
		end
	end
endmodule
