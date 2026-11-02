module cmos_write_req_gen(
	input              rst,
	input              pclk,
	input              cmos_vsync,
	// Asserted while image mode is active.  The displayed completed frame is
	// retained and the writer skips that buffer until image mode is left.
	input              freeze_req,
	output reg         write_req,
	output reg[1:0]    write_addr_index,
	output reg[1:0]    read_addr_index,
	input              write_req_ack
);
reg cmos_vsync_d0;
reg cmos_vsync_d1;
reg freeze_d;
reg [1:0] frozen_read_addr_index;
always@(posedge pclk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		cmos_vsync_d0 <= 1'b0;
		cmos_vsync_d1 <= 1'b0;
		freeze_d <= 1'b0;
		frozen_read_addr_index <= 2'b0;
	end
	else
	begin
		cmos_vsync_d0 <= cmos_vsync;
		cmos_vsync_d1 <= cmos_vsync_d0;
		freeze_d <= freeze_req;
		// read_addr_index always names the last completed frame.  Capturing it
		// on image-mode entry therefore never selects the frame being written.
		if (freeze_req && !freeze_d)
			frozen_read_addr_index <= read_addr_index;
	end
end
always@(posedge pclk or posedge rst)
begin
	if(rst == 1'b1)
		write_req <= 1'b0;
	else if(cmos_vsync_d0 == 1'b1 && cmos_vsync_d1 == 1'b0)
		write_req <= 1'b1;
	else if(write_req_ack == 1'b1)
		write_req <= 1'b0;
end
always@(posedge pclk or posedge rst)
begin
	if(rst == 1'b1)
		write_addr_index <= 2'b0;
	else if(cmos_vsync_d0 == 1'b1 && cmos_vsync_d1 == 1'b0) begin
		// Slots 0/1/2 form the raw capture ring; slot 3 is reserved for the
		// processed-output address region.  During image mode rotate through
		// the two raw slots other than the frozen completed source.
		if (freeze_req) begin
			case (write_addr_index)
				2'd0: write_addr_index <= (frozen_read_addr_index==2'd1)?2'd2:2'd1;
				2'd1: write_addr_index <= (frozen_read_addr_index==2'd2)?2'd0:2'd2;
				default: write_addr_index <= (frozen_read_addr_index==2'd0)?2'd1:2'd0;
			endcase
		end else begin
			case (write_addr_index)
				2'd0: write_addr_index <= 2'd1;
				2'd1: write_addr_index <= 2'd2;
				default: write_addr_index <= 2'd0;
			endcase
		end
	end
end

always@(posedge pclk or posedge rst)
begin
	if(rst == 1'b1)
		read_addr_index <= 2'b0;
	else if(!freeze_req && cmos_vsync_d0 == 1'b1 && cmos_vsync_d1 == 1'b0)
		read_addr_index <= write_addr_index;
end
endmodule 
