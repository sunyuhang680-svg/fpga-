module video_rect_read_data
#(
	parameter DATA_WIDTH = 16                       // Video data one clock data width
)
(
	input                       video_clk,          // Video pixel clock
	input                       rst,
	input[11:0]                 video_left_offset,
	input[11:0]                 video_top_offset,
	input[11:0]                 video_width,
	input[11:0]                 video_height,
	input                       view_enable,
	input[1:0]                  view_scale_shift,
	input[11:0]                 view_dst_left,
	input[11:0]                 view_dst_top,
	input[11:0]                 view_source_left,
	input[11:0]                 view_source_width,
	input[11:0]                 view_source_height,
	output reg                  read_req,           // Start reading a frame of data     
	input                       read_req_ack,       // Read request response
	(*mark_debug = "true"*)output                      read_en,            // Read data enable
	input[DATA_WIDTH - 1:0]     read_data,          // Read data
	input                       read_empty,
	input[12:0]                 read_level,
	input                       timing_hs,    
	input                       timing_vs,   
    output    reg[11:0]            x_cnt    ,
    output    reg[11:0]            y_cnt    ,
	(*mark_debug = "true"*)input                       timing_de,    
	(*mark_debug = "true"*)input[DATA_WIDTH - 1:0]     timing_data, 
	output                      hs,                 // horizontal synchronization
	output                      vs,                 // vertical synchronization
	(*mark_debug = "true"*)output                      de,                 // video valid
	output[DATA_WIDTH - 1:0]    vout_data           // video data
);
reg [11:0]    x_cnt_d0;
reg [11:0]    x_cnt_d1;
reg [11:0]    y_cnt_d0;
reg [11:0]    y_cnt_d1;
// Declared before their first procedural use for Verilog-2001 simulators.
(*mark_debug = "true"*)wire[11:0]             pos_x;
(*mark_debug = "true"*)wire[11:0]             pos_y;
wire                   pos_hs;
wire                   pos_vs;
(*mark_debug = "true"*)wire                   pos_de;
wire[DATA_WIDTH - 1:0] pos_data;

always@(posedge video_clk or posedge rst)    begin
	if(rst == 1'b1)
	begin
        x_cnt    <=    12'd0;
        x_cnt_d0 <=    12'd0;
        x_cnt_d1 <=    12'd0;
        y_cnt    <=    12'd0;
        y_cnt_d0 <=    12'd0;
        y_cnt_d1 <=    12'd0;
    end
    else
    begin
        x_cnt_d0     <=    pos_x;
        x_cnt_d1     <=    x_cnt_d0;
        x_cnt        <=    x_cnt_d1;
        y_cnt_d0     <=    pos_y;
        y_cnt_d1     <=    y_cnt_d0;
        y_cnt        <=    y_cnt_d1;

    end
end
reg[DATA_WIDTH - 1:0]  pos_data_d0;
reg[DATA_WIDTH - 1:0]  pos_data_d1;



//delay video_hs video_vs  video_de 2 clock cycles
reg                    pos_hs_d0;
reg                    pos_vs_d0;
reg                    pos_de_d0;
reg                    pos_hs_d1;
reg                    pos_vs_d1;
reg                    pos_de_d1;
reg                    pos_hs_d2;
reg                    pos_vs_d2;
reg                    pos_de_d2;
reg                    read_en_r;
reg                    read_en_r_d0;
reg                    normal_pixel_area_d0;
reg                    normal_pixel_area_d1;
reg                    view_pixel_area_d0;
reg                    view_pixel_area_d1;
reg                    view_pixel_area_previous;
reg[5:0]               discard_remaining;
reg                    view_stream_ready;
reg                    view_stream_fault;
reg                    view_fifo_reset_seen;

// The DDR reader can momentarily fall behind when a new crop is requested.
// Start a transformed frame only after one-and-a-half source lines are
// buffered.  If a later underflow occurs, abandon the remainder of that frame
// and restart cleanly at the next frame request instead of shifting every row.
localparam [12:0] VIEW_PREFILL_PIXELS = 13'd1024;

wire[13:0] view_display_width={2'b0,view_source_width}<<view_scale_shift;
wire[13:0] view_display_height={2'b0,view_source_height}<<view_scale_shift;
wire[12:0] view_left={1'b0,video_left_offset}+{1'b0,view_dst_left};
wire[12:0] view_top={1'b0,video_top_offset}+{1'b0,view_dst_top};
wire[12:0] view_rel_x={1'b0,pos_x}-view_left;
wire view_pixel_area=pos_de && ({1'b0,pos_x}>=view_left) &&
	({1'b0,pos_x}<view_left+view_display_width) && ({1'b0,pos_y}>=view_top) &&
	({1'b0,pos_y}<view_top+view_display_height);
wire view_sample_x=(view_scale_shift==0) ||
	((view_scale_shift==1) && !view_rel_x[0]) ||
	((view_scale_shift>=2) && !(|view_rel_x[1:0]));
wire[3:0] view_source_prefix=view_source_left[3:0];
wire[12:0] view_line_pixels={1'b0,view_source_width}+{9'd0,view_source_prefix};
wire[8:0] view_line_words=(view_line_pixels+13'd15)>>4;
wire[12:0] view_line_storage_pixels={view_line_words,4'b0000};
wire[5:0] view_source_tail=view_line_storage_pixels-view_line_pixels;
wire view_frame_request=pos_vs_d0 && !pos_vs;
wire normal_pixel_area=!view_enable && pos_de &&
	(pos_x>=video_left_offset) && (pos_x<video_left_offset+video_width) &&
	(pos_y>=video_top_offset) && (pos_y<video_top_offset+video_height);

reg[DATA_WIDTH - 1:0]  vout_data_r;
assign hs = pos_hs_d2;
assign vs = pos_vs_d2;
assign de = pos_de_d2;

always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		pos_hs_d0 <= 1'b0;
		pos_vs_d0 <= 1'b0;
		pos_de_d0 <= 1'b0;
		pos_hs_d1 <= 1'b0;
		pos_vs_d1 <= 1'b0;
		pos_de_d1 <= 1'b0;
		pos_data_d0 <= {DATA_WIDTH{1'b0}};
		pos_data_d1 <= {DATA_WIDTH{1'b0}};
		read_en_r_d0 <= 1'b0;
		normal_pixel_area_d0 <= 1'b0;
		normal_pixel_area_d1 <= 1'b0;
		view_pixel_area_d0 <= 1'b0;
		view_pixel_area_d1 <= 1'b0;
		view_pixel_area_previous <= 1'b0;
		discard_remaining <= 6'd0;
		view_stream_ready <= 1'b0;
		view_stream_fault <= 1'b0;
		view_fifo_reset_seen <= 1'b0;
	end
	else
	begin
		//delay pos hs vs de 3 clock cycles
		pos_hs_d0   <= pos_hs;
		pos_vs_d0   <= pos_vs;
		pos_de_d0   <= pos_de;
		pos_hs_d1   <= pos_hs_d0;
		pos_vs_d1   <= pos_vs_d0;
		pos_de_d1   <= pos_de_d0;	
		pos_hs_d2   <= pos_hs_d1;
		pos_vs_d2   <= pos_vs_d1;
		pos_de_d2   <= pos_de_d1;
		pos_data_d0 <= pos_data;
		pos_data_d1 <= pos_data_d0;
		read_en_r_d0 <= read_en_r;
		normal_pixel_area_d0 <= normal_pixel_area;
		normal_pixel_area_d1 <= normal_pixel_area_d0;
		view_pixel_area_d0 <= view_pixel_area;
		view_pixel_area_d1 <= view_pixel_area_d0;
		view_pixel_area_previous <= view_pixel_area;
		if(view_frame_request) begin
			discard_remaining<={2'd0,view_source_prefix};
			view_stream_ready<=1'b0;
			view_stream_fault<=1'b0;
			view_fifo_reset_seen<=1'b0;
		end
		else if(view_enable && view_pixel_area_previous && !view_pixel_area)
			discard_remaining<=view_source_tail+view_source_prefix;
		else if(view_enable && view_stream_ready && !view_stream_fault &&
			!view_pixel_area && discard_remaining!=0 && !read_empty)
			discard_remaining<=discard_remaining-1'b1;

		if(view_enable && !view_fifo_reset_seen && read_level==13'd0)
			view_fifo_reset_seen<=1'b1;
		if(view_enable && view_fifo_reset_seen && !view_stream_ready &&
			read_level>=VIEW_PREFILL_PIXELS)
			view_stream_ready<=1'b1;

		if(view_enable && !view_stream_fault) begin
			if(!view_pixel_area_previous && view_pixel_area &&
				(!view_stream_ready || discard_remaining!=0))
				view_stream_fault<=1'b1;
			else if(view_stream_ready && view_pixel_area && view_sample_x &&
				discard_remaining==0 && read_empty)
				view_stream_fault<=1'b1;
		end
	end
end

assign read_en = read_en_r; 
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		read_en_r <= 1'b0;
	else if(view_enable && view_stream_ready && !view_stream_fault &&
		!view_pixel_area && discard_remaining!=0 && !read_empty)
		read_en_r <= 1'b1;
	else if(view_enable && view_stream_ready && !view_stream_fault &&
		view_pixel_area && view_sample_x && discard_remaining==0 && !read_empty)
		read_en_r <= 1'b1;
	// Never pop an empty asynchronous FIFO.  The old unconditional read hid
	// display-side starvation as stale/invalid RGB565 pixels (fixed bands).
	else if(normal_pixel_area && !read_empty)
		read_en_r <= 1'b1;
	else
		read_en_r <= 1'b0;
end
/*always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		read_en_r <= 1'b0;
	else if( pos_de && pos_x >= 0 && pos_x <= 511  &&
	        pos_y >= 0 && pos_y <=383)
		read_en_r <= 1'b1;
	else
		read_en_r <= 1'b0;
end*/
assign vout_data = vout_data_r;
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		vout_data_r <= {DATA_WIDTH{1'b0}};
	else if(read_en_r_d0 == 1'b1)
		vout_data_r <= read_data;
	// Make a missing display pixel deterministic.  Black is preferable to
	// forwarding the FIFO's undefined/previous value and makes any remaining
	// bandwidth starvation visible without bleaching otherwise valid pixels.
	else if(normal_pixel_area_d1)
		vout_data_r <= {DATA_WIDTH{1'b0}};
	else if(view_enable && view_pixel_area_d1 && view_stream_ready && !view_stream_fault)
		vout_data_r <= vout_data_r;
	else
		vout_data_r <= pos_data_d1;
end

always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		read_req <= 1'b0;
	else if(pos_vs_d0 == 1'b1 & pos_vs == 1'b0) //vertical synchronization edge (the rising or falling edges are OK)
		read_req <= 1'b1;
	else if(read_req_ack == 1'b1)
		read_req <= 1'b0;
end

timing_gen_xy#(.DATA_WIDTH(DATA_WIDTH)) timing_gen_xy_m0(
	.rst_n    (~rst          ),
	.clk      (video_clk     ),
	.i_hs     (timing_hs     ),
	.i_vs     (timing_vs     ),
	.i_de     (timing_de     ),
	.i_data   (timing_data   ),
	.o_hs     (pos_hs        ),
	.o_vs     (pos_vs        ),
	.o_de     (pos_de        ),
	.o_data   (pos_data      ),
	.x        (pos_x         ),
	.y        (pos_y         )
);
endmodule 

// Final-view geometry shared by the DDR window reader and pixel expander.
// DDR requests remain aligned to sixteen RGB565 pixels; video_rect_read_data
// discards the sub-word prefix/tail during line blanking, so view movement is
// still pixel-granular. At 1x, dragging exposes black at the natural half-
// screen boundary. At 2x/4x, dragging moves a centered source crop.
module stereo_view_transform(
	input wire [1:0] zoom,
	input wire signed [11:0] pan_x,
	input wire signed [11:0] pan_y,
	output reg view_enable,
	output reg [1:0] scale_shift,
	output reg [11:0] source_left,
	output reg [11:0] source_top,
	output reg [11:0] source_width,
	output reg [11:0] source_height,
	output reg [11:0] destination_left,
	output reg [11:0] destination_top
);
	reg signed [12:0] px;
	reg signed [12:0] py;
	reg [11:0] magnitude_x;
	reg [11:0] magnitude_y;
	always @* begin
		view_enable=1'b0;scale_shift=2'd0;
		source_left=12'd0;source_top=12'd0;
		source_width=12'd640;source_height=12'd720;
		destination_left=12'd0;destination_top=12'd0;
		px=pan_x;py=pan_y;magnitude_x=0;magnitude_y=0;
		if(zoom==2'd0) begin
			if(px!=0 || py!=0) begin
				view_enable=1'b1;
				magnitude_x=px[12] ? -px : px;
				magnitude_y=py[12] ? -py : py;
				if(px>=0) destination_left=magnitude_x;
				else source_left=magnitude_x;
				if(py>=0) destination_top=magnitude_y;
				else source_top=magnitude_y;
				source_width=12'd640-magnitude_x;
				source_height=12'd720-magnitude_y;
			end
		end else if(zoom==2'd1) begin
			view_enable=1'b1;scale_shift=2'd1;
			source_width=12'd320;source_height=12'd360;
			source_left=(13'sd160-px);
			source_top=(13'sd180-py);
		end else begin
			view_enable=1'b1;scale_shift=2'd2;
			source_width=12'd160;source_height=12'd180;
			source_left=(13'sd240-px);
			source_top=(13'sd270-py);
		end
	end
endmodule
