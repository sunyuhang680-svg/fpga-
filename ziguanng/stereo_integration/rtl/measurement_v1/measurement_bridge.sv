`timescale 1ns/1ps
module measurement_bridge #(
 parameter [24:0] LEFT_BASE0=0,LEFT_BASE1=0,RIGHT_BASE0=0,RIGHT_BASE1=0,
 parameter [31:0] SESSION_TIMEOUT=32'd3000000000
)(
 input wire sys_clk,core_clk,pixel_clk,rst_n,
 input wire image_mode,menu_visible,entry_allowed,
 input wire touch_event,point0_valid,point1_valid,input wire [15:0] point0_x,point0_y,point1_x,point1_y,
 output wire block_touch,active_sys,
 input wire source_available,left_idle,right_idle,input wire [1:0] left_pub,right_pub,
 input wire left_fault,right_fault,
 input wire left_desc_tag,left_desc_index,right_desc_tag,right_desc_index,
 input wire [1:0] left_pub_pixel,right_pub_pixel,
 output wire force_static,allow_left,allow_right,
 input wire [23:0] raw_rgb,legacy_rgb,input wire raw_de,raw_hs,raw_vs,legacy_de,legacy_hs,legacy_vs,
 input wire display_drop,
 output wire [23:0] video_rgb,output wire video_de,video_hs,video_vs,
 output wire patch_req_valid,input wire patch_req_ready,
 output wire [24:0] patch_req_base_word,output wire [31:0] patch_req_frame_tag,
 output wire patch_frame_locked,output wire [31:0] patch_current_frame_tag,
 output wire [11:0] patch_req_x,patch_req_y,patch_req_width,patch_req_height,
 input wire patch_pix_valid,output wire patch_pix_ready,input wire [15:0] patch_pix_rgb565,
 input wire [11:0] patch_pix_x,patch_pix_y,input wire [31:0] patch_pix_frame_tag,
 input wire patch_pix_last,patch_done,patch_error,input wire [2:0] patch_error_code,input wire patch_busy
);
 // Per-domain reset release; mailboxes provide their own local reset synchronizers.
 (* ASYNC_REG="TRUE" *) reg [1:0] sys_reset_sync,core_reset_sync,pixel_reset_sync;
 always @(posedge sys_clk or negedge rst_n)if(!rst_n)sys_reset_sync<=0;else sys_reset_sync<={sys_reset_sync[0],1'b1};
 always @(posedge core_clk or negedge rst_n)if(!rst_n)core_reset_sync<=0;else core_reset_sync<={core_reset_sync[0],1'b1};
 always @(posedge pixel_clk or negedge rst_n)if(!rst_n)pixel_reset_sync<=0;else pixel_reset_sync<={pixel_reset_sync[0],1'b1};
 wire sys_rst_n=sys_reset_sync[1],core_rst_n=core_reset_sync[1],pixel_rst_n=pixel_reset_sync[1];
 wire selection_toggle,selection_index,manual_toggle,manual_index,clear_toggle;
 wire [9:0] left_x,left_y,right_x,right_y;
 wire selected_index,manual_mode;wire [1:0] left_valid,right_valid,pending,failed;
 wire [9:0] left_ax,left_ay,left_bx,left_by,right_ax,right_ay,right_bx,right_by;
 wire [7:0] reason_a,reason_b;wire display_valid,display_busy;wire [31:0] display_length_q16;wire [7:0] display_reason;
 wire magnifier_valid,magnifier_right;wire [9:0] magnifier_x,magnifier_y;
 wire core_busy_sys,locked_sys,session_failed_sys,ack_s_sys,ack_m_sys,ack_c_sys;
 wire match_toggle_sys,match_index_sys,match_valid_sys;wire [9:0] match_x_sys,match_y_sys;
 wire [7:0] match_reason_sys,result_reason_sys;wire result_valid_sys;wire [31:0] length_sys;
 wire frame_ready_sys;
 wire [31:0] core_frame_tag_sys,ready_tag_sys;
 wire ready_level_sys;
 reg ready_epoch_armed;
 wire safe_entry=entry_allowed && !locked_sys && (ack_c_sys==clear_toggle);
 always @(posedge sys_clk or negedge sys_rst_n)begin
  if(!sys_rst_n)ready_epoch_armed<=0;
  else if(!active_sys)ready_epoch_armed<=0;
  else if(!locked_sys && ack_c_sys==clear_toggle)ready_epoch_armed<=1;
 end
 assign frame_ready_sys=active_sys && ready_epoch_armed && locked_sys && ready_level_sys && (ready_tag_sys==core_frame_tag_sys);
 wire cmd_pending_sys=(selection_toggle!=ack_s_sys)||(manual_toggle!=ack_m_sys)||(clear_toggle!=ack_c_sys);
 wire ui_busy=core_busy_sys || cmd_pending_sys || !frame_ready_sys || session_failed_sys;
 measurement_ui_controller ui(.clk(sys_clk),.rst_n(sys_rst_n),.image_mode(image_mode),.menu_visible(menu_visible),
  .entry_allowed(safe_entry),.frame_ready(frame_ready_sys),.touch_event(touch_event),.point0_valid(point0_valid),.point1_valid(point1_valid),
  .point0_x(point0_x),.point0_y(point0_y),.point1_x(point1_x),.point1_y(point1_y),
  .match_toggle(match_toggle_sys),.match_index(match_index_sys),.match_valid(match_valid_sys),.match_x(match_x_sys),.match_y(match_y_sys),.match_reason(match_reason_sys),
  .result_valid(result_valid_sys),.length_mm_q16(length_sys),.result_reason(result_reason_sys),.busy(ui_busy),
  .active(active_sys),.block_touch(block_touch),.selection_toggle(selection_toggle),.selection_index(selection_index),.left_x(left_x),.left_y(left_y),
  .manual_toggle(manual_toggle),.manual_index(manual_index),.right_x(right_x),.right_y(right_y),.clear_toggle(clear_toggle),
  .selected_index(selected_index),.manual_mode(manual_mode),.left_valid(left_valid),.right_valid(right_valid),.pending(pending),.failed(failed),
  .left_ax(left_ax),.left_ay(left_ay),.left_bx(left_bx),.left_by(left_by),.right_ax(right_ax),.right_ay(right_ay),.right_bx(right_bx),.right_by(right_by),
  .reason_a(reason_a),.reason_b(reason_b),.display_valid(display_valid),.display_length_q16(display_length_q16),.display_reason(display_reason),.display_busy(display_busy),
  .magnifier_valid(magnifier_valid),.magnifier_right(magnifier_right),.magnifier_x(magnifier_x),.magnifier_y(magnifier_y));

 // One latest complete command snapshot. UI suppresses new point requests
 // until the prior command is actually accepted and acknowledged by core.
 wire [45:0] command_sys={active_sys,clear_toggle,selection_toggle,selection_index,left_x,left_y,manual_toggle,manual_index,right_x,right_y};
 reg [45:0] command_previous;
 always @(posedge sys_clk or negedge sys_rst_n)if(!sys_rst_n)command_previous<=0;else command_previous<=command_sys;
 wire [45:0] command_core;wire command_event;
 stereo_event_cdc #(.WIDTH(46),.INITIAL_PUBLISH(1)) command_cdc(.src_clk(sys_clk),.dst_clk(core_clk),.rst_n(rst_n),
  .src_event(command_sys!=command_previous),.src_data(command_sys),.dst_event(command_event),.dst_data(command_core),.src_ready());
 wire c_active,c_clear,c_sel,c_sel_index,c_man,c_man_index;wire [9:0] c_lx,c_ly,c_rx,c_ry;
 assign {c_active,c_clear,c_sel,c_sel_index,c_lx,c_ly,c_man,c_man_index,c_rx,c_ry}=command_core;
 reg active_core,ack_s,ack_m,ack_c,command_pending,pending_manual,pending_index,dispatch_guard;
 reg [9:0] pending_x,pending_y;reg selection_event,manual_event,clear_event;
 wire pipeline_busy,session_locked,session_failed;wire [31:0] frame_tag;
 wire [24:0] left_base,right_base;wire [1:0] locked_left,locked_right;
 wire display_ready_core,display_fault_core;
 measurement_session #(.TIMEOUT_CYCLES(SESSION_TIMEOUT),.LEFT_BASE0(LEFT_BASE0),.LEFT_BASE1(LEFT_BASE1),.RIGHT_BASE0(RIGHT_BASE0),.RIGHT_BASE1(RIGHT_BASE1)) session(
  .clk(core_clk),.rst_n(core_rst_n),.active(active_core),.source_available(source_available),.left_idle(left_idle),.right_idle(right_idle),
  .left_pub(left_pub),.right_pub(right_pub),.left_fault(left_fault),.right_fault(right_fault),.display_ready(display_ready_core),.display_fault(display_fault_core),
  .force_static(force_static),.allow_left(allow_left),.allow_right(allow_right),.locked(session_locked),.failed(session_failed),
  .frame_tag(frame_tag),.left_base(left_base),.right_base(right_base),.locked_left_pub(locked_left),.locked_right_pub(locked_right));
 always @(posedge core_clk or negedge core_rst_n)begin
  if(!core_rst_n)begin active_core<=0;ack_s<=0;ack_m<=0;ack_c<=0;command_pending<=0;pending_manual<=0;pending_index<=0;
   pending_x<=0;pending_y<=0;selection_event<=0;manual_event<=0;clear_event<=0;dispatch_guard<=0;
  end else begin
   selection_event<=0;manual_event<=0;clear_event<=0;
   if(dispatch_guard && pipeline_busy)dispatch_guard<=0;
   if(command_event)begin
    active_core<=c_active;
    if(!c_active || c_clear!=ack_c)begin
     clear_event<=1;ack_c<=c_clear;ack_s<=c_sel;ack_m<=c_man;command_pending<=0;dispatch_guard<=0;
    end else if(!command_pending)begin
     if(c_sel!=ack_s)begin command_pending<=1;pending_manual<=0;pending_index<=c_sel_index;pending_x<=c_lx;pending_y<=c_ly;end
     else if(c_man!=ack_m)begin command_pending<=1;pending_manual<=1;pending_index<=c_man_index;pending_x<=c_rx;pending_y<=c_ry;end
    end
   end
   if(command_pending && active_core && session_locked && !pipeline_busy && !dispatch_guard && !clear_event &&
      !(command_event && (!c_active || c_clear!=ack_c)))begin
    if(pending_manual)begin manual_event<=1;ack_m<=c_man;end else begin selection_event<=1;ack_s<=c_sel;end
    command_pending<=0;dispatch_guard<=1;
   end
  end
 end
 wire match_event,match_index,match_valid;wire [9:0] match_x,match_y;wire [7:0] match_reason,result_reason;
 wire result_valid;wire [31:0] length_mm_q16;
 measurement_pipeline pipeline(.clk(core_clk),.rst_n(core_rst_n),.clear(clear_event),.frame_locked(session_locked && active_core),.frame_tag(frame_tag),
  .left_base(left_base),.right_base(right_base),.selection_event(selection_event),.selection_index(pending_index),.left_x(pending_x),.left_y(pending_y),
  .manual_event(manual_event),.manual_index(pending_index),.right_x(pending_x),.right_y(pending_y),.busy(pipeline_busy),
  .match_event(match_event),.match_index(match_index),.match_valid(match_valid),.match_x(match_x),.match_y(match_y),.match_reason(match_reason),
  .result_valid(result_valid),.length_mm_q16(length_mm_q16),.result_reason(result_reason),
  .patch_req_valid(patch_req_valid),.patch_req_ready(patch_req_ready),.patch_req_base_word(patch_req_base_word),.patch_req_frame_tag(patch_req_frame_tag),
  .patch_frame_locked(patch_frame_locked),.patch_current_frame_tag(patch_current_frame_tag),.patch_req_x(patch_req_x),.patch_req_y(patch_req_y),
  .patch_req_width(patch_req_width),.patch_req_height(patch_req_height),.patch_pix_valid(patch_pix_valid),.patch_pix_ready(patch_pix_ready),
  .patch_pix_rgb565(patch_pix_rgb565),.patch_pix_x(patch_pix_x),.patch_pix_y(patch_pix_y),.patch_pix_frame_tag(patch_pix_frame_tag),
  .patch_pix_last(patch_pix_last),.patch_done(patch_done),.patch_error(patch_error),.patch_error_code(patch_error_code),.patch_busy(patch_busy));
 reg match_toggle_core;
 always @(posedge core_clk or negedge core_rst_n)if(!core_rst_n)match_toggle_core<=0;else if(match_event)match_toggle_core<=~match_toggle_core;
 wire [109:0] status_core={frame_tag,pipeline_busy || command_pending || dispatch_guard,session_locked,session_failed,ack_s,ack_m,ack_c,
  match_toggle_core,match_index,match_valid,match_x,match_y,match_reason,result_valid,length_mm_q16,result_reason};
 reg [109:0] status_previous;always @(posedge core_clk or negedge core_rst_n)if(!core_rst_n)status_previous<=0;else status_previous<=status_core;
 wire [109:0] status_sys;
 stereo_event_cdc #(.WIDTH(110),.INITIAL_PUBLISH(1)) status_cdc(.src_clk(core_clk),.dst_clk(sys_clk),.rst_n(rst_n),
  .src_event(status_core!=status_previous),.src_data(status_core),.dst_event(),.dst_data(status_sys),.src_ready());
 assign {core_frame_tag_sys,core_busy_sys,locked_sys,session_failed_sys,ack_s_sys,ack_m_sys,ack_c_sys,match_toggle_sys,match_index_sys,match_valid_sys,
  match_x_sys,match_y_sys,match_reason_sys,result_valid_sys,length_sys,result_reason_sys}=status_sys;

 // A locked pair is usable for selection only after both memory descriptors
 // refer to that pair and a complete 1280x720 video raster has elapsed.
 wire [41:0] display_core={frame_tag,session_locked,session_failed,locked_left,locked_right,left_desc_tag,left_desc_index,right_desc_tag,right_desc_index};
 reg [41:0] display_previous;always @(posedge core_clk or negedge core_rst_n)if(!core_rst_n)display_previous<=0;else display_previous<=display_core;
 wire [41:0] display_pixel;
 stereo_event_cdc #(.WIDTH(42),.INITIAL_PUBLISH(1)) display_cdc(.src_clk(core_clk),.dst_clk(pixel_clk),.rst_n(rst_n),
  .src_event(display_core!=display_previous),.src_data(display_core),.dst_event(),.dst_data(display_pixel),.src_ready());
 wire p_locked,p_failed,p_ldtag,p_ldindex,p_rdtag,p_rdindex;wire [1:0] p_lpub,p_rpub;
 wire [31:0] p_frame_tag;
 assign {p_frame_tag,p_locked,p_failed,p_lpub,p_rpub,p_ldtag,p_ldindex,p_rdtag,p_rdindex}=display_pixel;
 reg prev_raw_vs,prev_raw_de,frame_started,frame_bad,frame_ready_pixel,display_fault_pixel;
 reg [10:0] line_pixels,line_count;reg [31:0] pixel_epoch;
 wire descriptor_match=p_locked && !p_failed && p_ldtag && p_rdtag && p_ldindex==p_lpub[0] && p_rdindex==p_rpub[0] &&
  left_pub_pixel==p_lpub && right_pub_pixel==p_rpub;
 always @(posedge pixel_clk or negedge pixel_rst_n)begin
  if(!pixel_rst_n)begin prev_raw_vs<=0;prev_raw_de<=0;frame_started<=0;frame_bad<=0;
   line_pixels<=0;line_count<=0;frame_ready_pixel<=0;display_fault_pixel<=0;pixel_epoch<=0;end
  else begin
   prev_raw_vs<=raw_vs;prev_raw_de<=raw_de;
   if(!descriptor_match || pixel_epoch!=p_frame_tag)begin
    frame_started<=0;frame_bad<=0;line_pixels<=0;line_count<=0;frame_ready_pixel<=0;display_fault_pixel<=0;pixel_epoch<=p_frame_tag;
   end else if(display_drop && (frame_started || frame_ready_pixel))begin
    // Sticky until this locked session has been revoked; never recapture a
    // display frame after a detected read drop under the old session tag.
    display_fault_pixel<=1;frame_ready_pixel<=0;frame_started<=0;frame_bad<=1;
   end else if(display_fault_pixel)begin
    frame_ready_pixel<=0;
   end else if(prev_raw_vs && !raw_vs)begin
    if(frame_started && !frame_bad && !prev_raw_de && line_pixels==0 && line_count==720)frame_ready_pixel<=1;
    frame_started<=1;frame_bad<=0;line_pixels<=0;line_count<=0;
   end else if(frame_started && !frame_ready_pixel)begin
    if(raw_de)begin
     if(line_pixels>=1280)frame_bad<=1;else line_pixels<=line_pixels+1'b1;
    end else if(prev_raw_de)begin
     if(line_pixels!=1280 || line_count>=720)frame_bad<=1;
     line_pixels<=0;line_count<=line_count+1'b1;
    end
   end
  end
 end
 wire [33:0] ready_pixel={display_fault_pixel,frame_ready_pixel,pixel_epoch};
 reg [33:0] ready_previous;always @(posedge pixel_clk or negedge pixel_rst_n)if(!pixel_rst_n)ready_previous<=0;else ready_previous<=ready_pixel;
 wire [33:0] ready_sys;
 stereo_event_cdc #(.WIDTH(34),.INITIAL_PUBLISH(1)) ready_cdc(.src_clk(pixel_clk),.dst_clk(sys_clk),.rst_n(rst_n),
  .src_event(ready_pixel!=ready_previous),.src_data(ready_pixel),.dst_event(),.dst_data(ready_sys),.src_ready());
 wire unused_ready_fault_sys;
 assign {unused_ready_fault_sys,ready_level_sys,ready_tag_sys}=ready_sys;
 wire [33:0] ready_core;
 stereo_event_cdc #(.WIDTH(34),.INITIAL_PUBLISH(1)) ready_core_cdc(.src_clk(pixel_clk),.dst_clk(core_clk),.rst_n(rst_n),
  .src_event(ready_pixel!=ready_previous),.src_data(ready_pixel),.dst_event(),.dst_data(ready_core),.src_ready());
 assign display_ready_core=ready_core[32] && ready_core[31:0]==frame_tag;
 assign display_fault_core=session_locked && ready_core[33] && ready_core[31:0]==frame_tag;

 // Display snapshot is an atomic mailbox; video processing never samples the
 // independently changing system-domain UI coordinates.
 typedef struct packed {
  logic image,menu,entry,ready,active,selected,manual_mode;
  logic [1:0] lv,rv,pend,fail;
  logic [9:0] lax,lay,lbx,lby,rax,ray,rbx,rby;
  logic [7:0] ra,rb;
  logic valid,busy,session_failed;logic [31:0] length;logic [7:0] reason;
  logic mag_valid,mag_right;logic [9:0] mag_x,mag_y;
 } display_state_t;
 display_state_t ds,dp,ds_previous;
 always @* begin
  ds={image_mode,menu_visible,safe_entry,frame_ready_sys,active_sys,selected_index,manual_mode,
      left_valid,right_valid,pending,failed,left_ax,left_ay,left_bx,left_by,right_ax,right_ay,right_bx,right_by,reason_a,reason_b,
      display_valid,display_busy,session_failed_sys,display_length_q16,display_reason,magnifier_valid,magnifier_right,magnifier_x,magnifier_y};
 end
 always @(posedge sys_clk or negedge sys_rst_n)if(!sys_rst_n)ds_previous<='0;else ds_previous<=ds;
 stereo_event_cdc #(.WIDTH($bits(display_state_t)),.INITIAL_PUBLISH(1)) ui_pixel_cdc(.src_clk(sys_clk),.dst_clk(pixel_clk),.rst_n(rst_n),
  .src_event(ds!=ds_previous),.src_data(ds),.dst_event(),.dst_data(dp),.src_ready());
 wire [23:0] mag_rgb;wire mag_de,mag_hs,mag_vs;
 measurement_magnifier magnifier(.pixel_clk(pixel_clk),.rst_n(pixel_rst_n),.camera_rgb(raw_rgb),.camera_de(raw_de),.camera_hs(raw_hs),.camera_vs(raw_vs),
  .magnifier_valid(dp.mag_valid && descriptor_match && frame_ready_pixel && !display_fault_pixel && !display_drop),.magnifier_right(dp.mag_right),.magnifier_x(dp.mag_x),.magnifier_y(dp.mag_y),.magnifier_visible(),
  .video_rgb(mag_rgb),.video_de(mag_de),.video_hs(mag_hs),.video_vs(mag_vs));
 // Switch video branches at the raw frame edge; inactive mode retains the old
 // compositor and adds only the entry overlay. Active mode bypasses all old UI.
 reg use_measurement;
 always @(posedge pixel_clk or negedge pixel_rst_n)if(!pixel_rst_n)use_measurement<=0;else if(prev_raw_vs && !raw_vs)use_measurement<=dp.active;
 measurement_ui_overlay overlay(.pixel_clk(pixel_clk),.rst_n(pixel_rst_n),
  .camera_rgb(use_measurement?mag_rgb:legacy_rgb),.camera_de(use_measurement?mag_de:legacy_de),
  .camera_hs(use_measurement?mag_hs:legacy_hs),.camera_vs(use_measurement?mag_vs:legacy_vs),
  .image_mode(dp.image),.menu_visible(dp.menu),.entry_allowed(dp.entry),.frame_ready(dp.ready),.active(dp.active),.selected_index(dp.selected),.manual_mode(dp.manual_mode),
  .left_valid(dp.lv),.right_valid(dp.rv),.pending(dp.pend),.failed(dp.fail),.left_ax(dp.lax),.left_ay(dp.lay),.left_bx(dp.lbx),.left_by(dp.lby),
  .right_ax(dp.rax),.right_ay(dp.ray),.right_bx(dp.rbx),.right_by(dp.rby),.display_valid(dp.valid && descriptor_match && frame_ready_pixel && !display_fault_pixel && !display_drop),.display_busy(dp.busy),.session_failed(dp.session_failed),
  .display_length_q16(dp.length),.display_reason(dp.reason),.reason_a(dp.ra),.reason_b(dp.rb),
  .video_rgb(video_rgb),.video_de(video_de),.video_hs(video_hs),.video_vs(video_vs));
endmodule
