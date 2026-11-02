`timescale 1ns/1ps
module measurement_ui_controller(
 input wire clk,rst_n,image_mode,menu_visible,entry_allowed,frame_ready,
 input wire touch_event,point0_valid,point1_valid,
 input wire [15:0] point0_x,point0_y,point1_x,point1_y,
 input wire match_toggle,match_index,match_valid,
 input wire [9:0] match_x,match_y, input wire [7:0] match_reason,
 input wire result_valid, input wire [31:0] length_mm_q16,
 input wire [7:0] result_reason, input wire busy,
 output reg active, output wire block_touch,
 output reg selection_toggle,selection_index,output reg [9:0] left_x,left_y,
 output reg manual_toggle,manual_index,output reg [9:0] right_x,right_y,
 output reg clear_toggle,
 output reg selected_index,manual_mode,
 output reg [1:0] left_valid,right_valid, pending, failed,
 output reg [9:0] left_ax,left_ay,left_bx,left_by,right_ax,right_ay,right_bx,right_by,
 output reg [7:0] reason_a,reason_b,
 output wire display_valid, output wire [31:0] display_length_q16,
 output wire [7:0] display_reason, output wire display_busy,
 output wire magnifier_valid,magnifier_right,output wire [9:0] magnifier_x,magnifier_y
);
 reg contact_down,match_seen,result_armed,frame_seen;
 reg [1:0] candidate_exists;
 wire [31:0] xp=point0_x*32'd1279, yp=point0_y*32'd719;
 wire [15:0] tx=xp>>14,ty=yp>>14;
 wire in_range=point0_x<=16'd16384 && point0_y<=16'd16384;
 wire entry=image_mode && menu_visible && entry_allowed && tx>=1184 && ty>=108 && ty<156;
 wire contact=point0_valid || point1_valid;
 wire first=touch_event && contact && !contact_down;
 assign block_touch=active || (touch_event && point0_valid && !point1_valid && in_range && entry);
 assign magnifier_right=manual_mode;
 assign magnifier_valid=active && image_mode && frame_ready && left_valid[selected_index] && (!manual_mode || candidate_exists[selected_index]);
 assign magnifier_x=manual_mode ? (selected_index ? right_bx : right_ax) : (selected_index ? left_bx : left_ax);
 assign magnifier_y=manual_mode ? (selected_index ? right_by : right_ay) : (selected_index ? left_by : left_ay);
 assign display_valid=active && frame_ready && result_armed && result_valid && !busy && (&left_valid) && (&right_valid) && !(|pending);
 assign display_length_q16=display_valid ? length_mm_q16 : 32'd0;
 assign display_reason=(active && result_armed && (&left_valid) && (&right_valid) && !(|pending)) ? result_reason : 8'd0;
 assign display_busy=active && (busy || (|pending));
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   active<=0;contact_down<=0;selection_toggle<=0;selection_index<=0;left_x<=0;left_y<=0;
   manual_toggle<=0;manual_index<=0;right_x<=0;right_y<=0;clear_toggle<=0;
   selected_index<=0;manual_mode<=0;left_valid<=0;right_valid<=0;pending<=0;failed<=0;
   left_ax<=0;left_ay<=0;left_bx<=0;left_by<=0;right_ax<=0;right_ay<=0;right_bx<=0;right_by<=0;
   reason_a<=0;reason_b<=0;match_seen<=0;result_armed<=0;candidate_exists<=0;frame_seen<=0;
  end else begin
   if(touch_event) contact_down<=contact;
   match_seen<=match_toggle;
   if(!image_mode || (active && frame_seen && !frame_ready)) begin
    if(active) clear_toggle<=~clear_toggle;
    active<=0;left_valid<=0;right_valid<=0;pending<=0;failed<=0;candidate_exists<=0;manual_mode<=0;selected_index<=0;result_armed<=0;frame_seen<=0;
   end else begin
    if(active && frame_ready) frame_seen<=1;
    if(!result_valid) result_armed<=1;
    if(active && match_toggle!=match_seen && pending[match_index]) begin
     pending[match_index]<=0;right_valid[match_index]<=match_valid && match_x<640 && match_y<720;failed[match_index]<=!(match_valid && match_x<640 && match_y<720);
     if(match_valid && match_x<640 && match_y<720) candidate_exists[match_index]<=1;
     if(!match_index) begin reason_a<=match_reason;if(match_valid && match_x<640 && match_y<720) begin right_ax<=match_x;right_ay<=match_y;end end
     else begin reason_b<=match_reason;if(match_valid && match_x<640 && match_y<720) begin right_bx<=match_x;right_by<=match_y;end end
    end
    if(first && point0_valid && !point1_valid && in_range) begin
     if(!active) begin
      if(entry) begin active<=1;frame_seen<=0;clear_toggle<=~clear_toggle;left_valid<=0;right_valid<=0;pending<=0;failed<=0;candidate_exists<=0;selected_index<=0;manual_mode<=0;result_armed<=0; end
     end else if(ty>=660) begin
      if(tx<128) begin active<=0;frame_seen<=0;clear_toggle<=~clear_toggle;left_valid<=0;right_valid<=0;pending<=0;failed<=0;candidate_exists<=0;result_armed<=0;end
      else if(tx<256) begin clear_toggle<=~clear_toggle;left_valid<=0;right_valid<=0;pending<=0;failed<=0;candidate_exists<=0;selected_index<=0;manual_mode<=0;result_armed<=0;end
      else if(tx<352) selected_index<=0;
      else if(tx<448) selected_index<=1;
      else if(tx<576) manual_mode<=~manual_mode;
      else if(tx<960 && frame_ready && !busy && !(|pending) && manual_mode && left_valid[selected_index] && candidate_exists[selected_index]) begin
       // Four equal-width keys: left, right, up, down.  Coordinates saturate.
       manual_index<=selected_index;
       if(!selected_index) begin
        right_x<= (tx<672 && right_ax>0) ? right_ax-1'b1 : (tx>=672 && tx<768 && right_ax<639) ? right_ax+1'b1 : right_ax;
        right_y<= (tx>=768 && tx<864 && right_ay>0) ? right_ay-1'b1 : (tx>=864 && right_ay<719) ? right_ay+1'b1 : right_ay;
       end else begin
        right_x<= (tx<672 && right_bx>0) ? right_bx-1'b1 : (tx>=672 && tx<768 && right_bx<639) ? right_bx+1'b1 : right_bx;
        right_y<= (tx>=768 && tx<864 && right_by>0) ? right_by-1'b1 : (tx>=864 && right_by<719) ? right_by+1'b1 : right_by;
       end
       manual_toggle<=~manual_toggle;pending[selected_index]<=1;right_valid[selected_index]<=0;failed[selected_index]<=0;result_armed<=0;
       if(!selected_index) begin
        right_ax<= (tx<672 && right_ax>0) ? right_ax-1'b1 : (tx>=672 && tx<768 && right_ax<639) ? right_ax+1'b1 : right_ax;
        right_ay<= (tx>=768 && tx<864 && right_ay>0) ? right_ay-1'b1 : (tx>=864 && right_ay<719) ? right_ay+1'b1 : right_ay;
       end else begin
        right_bx<= (tx<672 && right_bx>0) ? right_bx-1'b1 : (tx>=672 && tx<768 && right_bx<639) ? right_bx+1'b1 : right_bx;
        right_by<= (tx>=768 && tx<864 && right_by>0) ? right_by-1'b1 : (tx>=864 && right_by<719) ? right_by+1'b1 : right_by;
       end
      end
     end else if(frame_ready && !busy && !(|pending) && tx<640) begin
      selection_toggle<=~selection_toggle;selection_index<=selected_index;left_x<=tx;left_y<=ty;
      left_valid[selected_index]<=1;right_valid[selected_index]<=0;candidate_exists[selected_index]<=0;pending[selected_index]<=1;failed[selected_index]<=0;result_armed<=0;
      if(!selected_index) begin left_ax<=tx;left_ay<=ty;end else begin left_bx<=tx;left_by<=ty;end
     end else if(frame_ready && !busy && !(|pending) && manual_mode && left_valid[selected_index] && !(tx>=1152 && ty>=160 && ty<288)) begin
      manual_toggle<=~manual_toggle;manual_index<=selected_index;right_x<=tx-640;right_y<=ty;
      right_valid[selected_index]<=0;candidate_exists[selected_index]<=1;pending[selected_index]<=1;failed[selected_index]<=0;result_armed<=0;
      if(!selected_index) begin right_ax<=tx-640;right_ay<=ty;end else begin right_bx<=tx-640;right_by<=ty;end
     end
    end
   end
  end
 end
endmodule
