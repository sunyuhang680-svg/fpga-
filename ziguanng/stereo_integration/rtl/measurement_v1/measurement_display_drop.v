// Public video_rect_read_data x/y/de outputs are delayed three clocks from
// the pixel-area decision. Match empty to exactly that same decision cycle.
// Both consumers use view_enable=0 and the fixed 640x720 half-screen ranges.
module measurement_display_drop(
 input wire clk,rst_n,
 input wire [11:0] left_x,left_y,right_x,right_y,
 input wire left_de,right_de,left_empty,right_empty,
 output wire drop
);
 reg [2:0] left_empty_delay,right_empty_delay;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin left_empty_delay<=3'b000;right_empty_delay<=3'b000;end
  else begin
   left_empty_delay<={left_empty_delay[1:0],left_empty};
   right_empty_delay<={right_empty_delay[1:0],right_empty};
  end
 end
 assign drop=rst_n &&
  ((left_de && left_x<12'd640 && left_y<12'd720 && left_empty_delay[2]) ||
   (right_de && right_x>=12'd640 && right_x<12'd1280 && right_y<12'd720 && right_empty_delay[2]));
endmodule
