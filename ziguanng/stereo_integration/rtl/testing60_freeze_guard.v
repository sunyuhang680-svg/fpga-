`timescale 1ns/1ps
// Camera-domain UI hold. Delay only a NEW UI hold until both CP VS and the
// mature generator publication edge have passed. Existing holds stay asserted.
module testing60_freeze_guard(
 input wire camera_clk,rst_n,ui_image_mode_sync,camera_vsync,
 input wire [1:0] completed_index,
 input wire [3:0] completed_valid,
 input wire algorithm_freeze,
 output wire ui_hold_active,
 output wire combined_freeze
);
 reg vs_delay;
 reg ui_freeze;
 assign ui_hold_active=ui_freeze;
 assign combined_freeze=algorithm_freeze || ui_freeze;
 always @(posedge camera_clk or negedge rst_n)begin
   if(!rst_n)begin vs_delay<=0;ui_freeze<=0;end
   else begin
     vs_delay<=camera_vsync;
     if(!ui_image_mode_sync || !completed_valid[completed_index])ui_freeze<=0;
     else if(!camera_vsync && !vs_delay)ui_freeze<=1;
   end
 end
endmodule
