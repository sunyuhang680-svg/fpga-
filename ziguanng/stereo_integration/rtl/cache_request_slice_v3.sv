// Isolated candidate: two-bank circular FIFO. Payload bank enables use push only.
module cache_request_slice_v3 #(parameter WIDTH=146)(
 input wire clk,rst_n,flush,
 input wire in_valid,output wire in_ready,input wire[WIDTH-1:0]in_data,
 output wire out_valid,input wire out_ready,output wire[WIDTH-1:0]out_data
);
 reg [WIDTH-1:0] bank0, bank1;
 reg read_ptr, write_ptr;
 reg [1:0] count;
 wire push = in_valid && in_ready;
 wire pop = out_valid && out_ready;
 assign in_ready = (count != 2'd2) && !flush;
 assign out_valid = (count != 2'd0) && !flush;
 assign out_data = read_ptr ? bank1 : bank0;
 always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
   read_ptr <= 1'b0;
   write_ptr <= 1'b0;
   count <= 2'd0;
  end else if (flush) begin
   read_ptr <= 1'b0;
   write_ptr <= 1'b0;
   count <= 2'd0;
  end else begin
   if (push) begin
    if (write_ptr) bank1 <= in_data;
    else bank0 <= in_data;
    write_ptr <= ~write_ptr;
   end
   if (pop) read_ptr <= ~read_ptr;
   case ({push,pop})
    2'b10: count <= count + 1'b1;
    2'b01: count <= count - 1'b1;
    default: count <= count;
   endcase
  end
 end
endmodule