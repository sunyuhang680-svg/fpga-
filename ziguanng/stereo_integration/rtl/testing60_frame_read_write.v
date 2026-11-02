`timescale 1ns/1ps
module testing60_frame_read_write
#
(
	parameter MEM_DATA_BITS          = 64,
	parameter READ_DATA_BITS         = 16,
	parameter WRITE_DATA_BITS        = 16,
	parameter ADDR_BITS              = 25,
	parameter BUSRT_BITS             = 10,
	parameter BURST_SIZE             = 64
)               
(
	input                            rst,                  
	input                            mem_clk,                    // external memory controller user interface clock
	output                           rd_burst_req,               // to external memory controller,send out a burst read request
	output[BUSRT_BITS - 1:0]         rd_burst_len,               // to external memory controller,data length of the burst read request, not bytes
	output[ADDR_BITS - 1:0]          rd_burst_addr,              // to external memory controller,base address of the burst read request 
	input                            rd_burst_data_valid,        // from external memory controller,read data valid 
	input[MEM_DATA_BITS - 1:0]       rd_burst_data,              // from external memory controller,read request data
	input                            rd_burst_finish,            // from external memory controller,burst read finish
	input                            read_clk,                   // data read module clock
	input                            read_req,                   // data read module read request,keep '1' until read_req_ack = '1'
	output                           read_req_ack,               // data read module read request response
	output                           read_finish,                // data read module read request finish
	input                            read_buffer_tag,            // identifies post-processed display buffers
	output[1:0]                      read_index_latched,         // index accepted by the DDR display reader
	output                           read_tag_latched,           // tag accepted with read_index_latched
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
	input                            read_en,                    // data read module read request for one data, read_data valid next clock
	output[READ_DATA_BITS  - 1:0]    read_data,                  // read data
	output                           read_empty,
	output[12:0]                     read_level,
	output                           wr_burst_req,               // to external memory controller,send out a burst write request
	output[BUSRT_BITS - 1:0]         wr_burst_len,               // to external memory controller,data length of the burst write request, not bytes
	output[ADDR_BITS - 1:0]          wr_burst_addr,              // to external memory controller,base address of the burst write request 
	input                            wr_burst_data_req,          // from external memory controller,write data request ,before data 1 clock
	output[MEM_DATA_BITS - 1:0]      wr_burst_data,              // to external memory controller,write data
	input                            wr_burst_finish,            // from external memory controller,burst write finish
	input                            write_clk,                  // data write module clock
	input                            write_req,                  // data write module write request,keep '1' until read_req_ack = '1'
	output                           write_req_ack,              // data write module write request response
	output                           write_finish,               // data write module write request finish
	input[ADDR_BITS - 1:0]           write_addr_0,               // data write module write request base address 0, used when write_addr_index = 0
	input[ADDR_BITS - 1:0]           write_addr_1,               // data write module write request base address 1, used when write_addr_index = 1
	input[ADDR_BITS - 1:0]           write_addr_2,               // data write module write request base address 1, used when write_addr_index = 2
	input[ADDR_BITS - 1:0]           write_addr_3,               // data write module write request base address 1, used when write_addr_index = 3
	input[1:0]                       write_addr_index,           // select valid base address from write_addr_0 write_addr_1 write_addr_2 write_addr_3
	input[ADDR_BITS - 1:0]           write_len,                  // data write module write request data length
	input                            write_en,                   // data write module write request for one data
	input[WRITE_DATA_BITS - 1:0]     write_data                  // write data
);
wire[15:0]                           wrusedw;                    // write used words
wire[15:0]                           rdusedw;                    // read used words
wire                                 read_fifo_aclr;             // fifo Asynchronous clear
wire                                 write_fifo_aclr;            // fifo Asynchronous clear
// Each FIFO port has asynchronous assertion and local synchronous release.
// Global reset also clears the FIFO (the supplied controllers clear aclr on rst).
wire read_fifo_reset_req = rst | read_fifo_aclr;
wire write_fifo_reset_req = rst | write_fifo_aclr;
(* ASYNC_REG="TRUE" *) reg [1:0] read_wr_release, read_rd_release;
(* ASYNC_REG="TRUE" *) reg [1:0] write_wr_release, write_rd_release;
always @(posedge mem_clk or posedge read_fifo_reset_req)
 if (read_fifo_reset_req) read_wr_release <= 2'b00;
 else read_wr_release <= {read_wr_release[0],1'b1};
always @(posedge read_clk or posedge read_fifo_reset_req)
 if (read_fifo_reset_req) read_rd_release <= 2'b00;
 else read_rd_release <= {read_rd_release[0],1'b1};
always @(posedge write_clk or posedge write_fifo_reset_req)
 if (write_fifo_reset_req) write_wr_release <= 2'b00;
 else write_wr_release <= {write_wr_release[0],1'b1};
always @(posedge mem_clk or posedge write_fifo_reset_req)
 if (write_fifo_reset_req) write_rd_release <= 2'b00;
 else write_rd_release <= {write_rd_release[0],1'b1};
wire read_wr_rst = ~read_wr_release[1];
wire read_rd_rst = ~read_rd_release[1];
wire write_wr_rst = ~write_wr_release[1];
wire write_rd_rst = ~write_rd_release[1];
// Extend the actual 9-bit water-level outputs for the 16-bit controllers.
assign wrusedw[15:9] = 7'd0;
assign rdusedw[15:9] = 7'd0;
// A request may wait in the original controller until the remote FIFO port has
// released reset and that fact has crossed to the memory domain.
(* ASYNC_REG="TRUE" *) reg read_remote_meta, read_remote_ready;
(* ASYNC_REG="TRUE" *) reg write_remote_meta, write_remote_ready;
always @(posedge mem_clk or posedge read_fifo_reset_req)
 if (read_fifo_reset_req) begin read_remote_meta <= 1'b0; read_remote_ready <= 1'b0; end
 else begin read_remote_meta <= read_rd_release[1]; read_remote_ready <= read_remote_meta; end
always @(posedge mem_clk or posedge write_fifo_reset_req)
 if (write_fifo_reset_req) begin write_remote_meta <= 1'b0; write_remote_ready <= 1'b0; end
 else begin write_remote_meta <= write_wr_release[1]; write_remote_ready <= write_remote_meta; end
wire rd_burst_req_raw, wr_burst_req_raw;
assign rd_burst_req = rd_burst_req_raw && !read_wr_rst && read_remote_ready;
assign wr_burst_req = wr_burst_req_raw && !write_rd_rst && write_remote_ready;
//instantiate an asynchronous FIFO 
afifo_16i_64o_512 write_buf(
    .wr_clk(write_clk),
    .wr_rst(write_wr_rst),
    .wr_en(write_en && !write_wr_rst),
    .wr_data(write_data),
    .wr_full(),
    .wr_water_level(),
    .almost_full(),
    .rd_clk(mem_clk),
    .rd_rst(write_rd_rst),
    .rd_en(wr_burst_data_req && !write_rd_rst),
    .rd_data(wr_burst_data),
    .rd_empty(),
    .rd_water_level(rdusedw[8:0]),
    .almost_empty());
frame_fifo_write
#
(
	.MEM_DATA_BITS              (MEM_DATA_BITS            ),
	.ADDR_BITS                  (ADDR_BITS                ),
	.BUSRT_BITS                 (BUSRT_BITS               ),
	.BURST_SIZE                 (BURST_SIZE               )
) 
frame_fifo_write_m0              
(  
	.rst                        (rst                      ),
	.mem_clk                    (mem_clk                  ),
	.wr_burst_req               (wr_burst_req_raw         ),
	.wr_burst_len               (wr_burst_len             ),
	.wr_burst_addr              (wr_burst_addr            ),
	.wr_burst_data_req          (wr_burst_data_req        ),
	.wr_burst_finish            (wr_burst_finish          ),
	.write_req                  (write_req                ),
	.write_req_ack              (write_req_ack            ),
	.write_finish               (write_finish             ),
	.write_addr_0               (write_addr_0             ),
	.write_addr_1               (write_addr_1             ),
	.write_addr_2               (write_addr_2             ),
	.write_addr_3               (write_addr_3             ),
	.write_addr_index           (write_addr_index         ),    
	.write_len                  (write_len                ),
	.fifo_aclr                  (write_fifo_aclr          ),
	.rdusedw                    (rdusedw                  ) 
	
);

//instantiate an asynchronous FIFO

afifo_64i_16o_128 read_buf (
    .wr_clk(mem_clk),
    .wr_rst(read_wr_rst),
    .wr_en(rd_burst_data_valid && !read_wr_rst),
    .wr_data(rd_burst_data),
    .wr_full(),
    .wr_water_level(wrusedw[8:0]),
    .almost_full(),
    .rd_clk(read_clk),
    .rd_rst(read_rd_rst),
    .rd_en(read_en && !read_rd_rst),
    .rd_data(read_data),
    .rd_empty(read_empty),
    .rd_water_level(read_level),
    .almost_empty());

frame_fifo_read
#
(
	.MEM_DATA_BITS              (MEM_DATA_BITS            ),
	.ADDR_BITS                  (ADDR_BITS                ),
	.BUSRT_BITS                 (BUSRT_BITS               ),
	// The generated 256-to-16 FIFO contains 256 write-side words.  Use the
	// complete reservoir so display reads can ride through DDR arbitration
	// gaps caused by the additional processed-frame writers.
	.FIFO_DEPTH                 (256                      ),
	.BURST_SIZE                 (BURST_SIZE               )
)
frame_fifo_read_m0
(
	.rst                        (rst                      ),
	.mem_clk                    (mem_clk                  ),
	.rd_burst_req               (rd_burst_req_raw         ),   
	.rd_burst_len               (rd_burst_len             ),  
	.rd_burst_addr              (rd_burst_addr            ),
	.rd_burst_data_valid        (rd_burst_data_valid      ),    
	.rd_burst_finish            (rd_burst_finish          ),
	.read_req                   (read_req                 ),
	.read_req_ack               (read_req_ack             ),
	.read_finish                (read_finish              ),
	.read_buffer_tag            (read_buffer_tag          ),
	.read_index_latched         (read_index_latched       ),
	.read_tag_latched           (read_tag_latched         ),
	.read_addr_0                (read_addr_0              ),
	.read_addr_1                (read_addr_1              ),
	.read_addr_2                (read_addr_2              ),
	.read_addr_3                (read_addr_3              ),
	.read_addr_index            (read_addr_index          ),    
	.read_len                   (read_len                 ),
	.window_enable              (window_enable            ),
	.window_scale_shift         (window_scale_shift       ),
	.window_source_left         (window_source_left       ),
	.window_source_top          (window_source_top        ),
	.window_source_width        (window_source_width      ),
	.window_source_height       (window_source_height     ),
	.fifo_aclr                  (read_fifo_aclr           ),
	.wrusedw                    (wrusedw                  )
);

endmodule



