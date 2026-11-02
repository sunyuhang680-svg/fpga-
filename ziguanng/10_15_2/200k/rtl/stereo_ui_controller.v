`timescale 1ns/1ps
// Control-plane only. Buttons select live/frozen video and menu visibility.
// Touch gestures maintain two independent view transforms, one for each
// 640x720 half of the display. A one-finger drag pans only the touched half;
// Continuous inverse-scale/origin gestures are handled below in the control
// clock, independently of the DDR/pixel clock.
module stereo_ui_controller #(
    parameter integer CLK_HZ = 27000000,
    parameter integer DEBOUNCE_MS = 20,
    parameter integer DOUBLE_TAP_MS = 350,
    parameter integer RELEASE_HOLD_MS = 40,
    parameter integer CONTACT_TIMEOUT_MS = 120,
    parameter integer SCREEN_W = 1280,
    parameter integer SCREEN_H = 720,
    parameter integer ENABLE_CMP_CONTROLS = 0
)(
    input wire clk, input wire rst_n,
    input wire key_mode_n, input wire key_menu_n,
    input wire touch_event,
    input wire point0_valid, input wire point1_valid,
    input wire [15:0] point0_x, input wire [15:0] point0_y,
    input wire [15:0] point1_x, input wire [15:0] point1_y,
    output reg image_mode, output reg menu_visible, output wire state_event,
    output reg [1:0] layout, output reg [1:0] feature,
    output reg [1:0] cmp_undistort, output reg [1:0] cmp_binary,
    output reg zoom_active, output reg [1:0] zoom_source,
    output reg interaction_active,
    // Image post-processing mode. The rightmost top-row key toggles it and
    // turns into RETURN while the mode is active. press_code / press_toggle
    // carry one short "flash" pulse per key press into the pixel domain.
    output reg post_mode, output reg [2:0] post_tool, output reg post_page,
    output reg [2:0] press_code, output reg press_toggle,
    // Sub-page of the selected tool: palette index for DOODLE, slider value
    // for the six adjustment tools (0..255, 128 = neutral).
    output wire [2:0] sub_color, output wire [7:0] knob_value,
    output wire [127:0] knob_values,
    output wire [16:0] left_step_q16, output wire [16:0] right_step_q16,
    output wire [1:0] left_rotation, output wire [1:0] right_rotation,
    output wire signed [27:0] left_origin_x, output wire signed [27:0] left_origin_y,
    output wire signed [27:0] right_origin_x, output wire signed [27:0] right_origin_y
);
    localparam [1:0] LAYOUT_STEREO=2'd0, LAYOUT_LEFT_COMPARE=2'd1,
                     LAYOUT_RIGHT_COMPARE=2'd2;
    localparam integer DB_CYCLES = (CLK_HZ/1000)*DEBOUNCE_MS;
    localparam integer TAP_CYCLES = (CLK_HZ/1000)*DOUBLE_TAP_MS;
    localparam integer RELEASE_HOLD_CYCLES = (CLK_HZ/1000)*RELEASE_HOLD_MS;
    localparam integer CONTACT_TIMEOUT_CYCLES = (CLK_HZ/1000)*CONTACT_TIMEOUT_MS;
    reg mode_s0,mode_s1,mode_last,mode_stable;
    reg menu_s0,menu_s1,menu_last,menu_stable;
    reg [31:0] mode_count,menu_count,tap_count;
    reg init_publish,tap_armed,contact_was_down;
    reg control_contact;
    reg [15:0] last_tap_x,last_tap_y;
    reg [31:0] interaction_hold_count;
    reg [31:0] contact_timeout_count;
    reg ui_event;
    wire gesture_event;
    assign state_event=ui_event || gesture_event;

    wire [31:0] mapped_x0_product={16'd0,point0_x}*(SCREEN_W-1);
    wire [31:0] mapped_y0_product={16'd0,point0_y}*(SCREEN_H-1);
    wire [31:0] mapped_x1_product={16'd0,point1_x}*(SCREEN_W-1);
    wire [31:0] mapped_y1_product={16'd0,point1_y}*(SCREEN_H-1);
    wire [15:0] touch0_x=mapped_x0_product>>14;
    wire [15:0] touch0_y=mapped_y0_product>>14;
    wire [15:0] touch1_x=mapped_x1_product>>14;
    wire [15:0] touch1_y=mapped_y1_product>>14;
    wire point0_in_range=(point0_x<=16'd16384) && (point0_y<=16'd16384);
    wire point1_in_range=(point1_x<=16'd16384) && (point1_y<=16'd16384);
    wire touch0_panel=(touch0_x>=(SCREEN_W/2));
    wire same_tap_panel=(touch0_panel==(last_tap_x>=(SCREEN_W/2)));
    // CMP (ENABLE_CMP_CONTROLS=1) shows ONE camera.  Its view transform is
    // that camera's panel transform; touches are remapped into that panel so
    // pan/pinch act on the whole picture.  Fullscreen: screen x/2 equals the
    // 640-wide panel coordinate.  Compare: both halves show the same view, so
    // either half maps to the panel with x mod 640.
    wire cmp_layout=ENABLE_CMP_CONTROLS && (layout==LAYOUT_LEFT_COMPARE || layout==LAYOUT_RIGHT_COMPARE);
    wire cmp_camera=(layout==LAYOUT_RIGHT_COMPARE);
    wire cmp_compare_view=cmp_layout && cmp_binary[cmp_camera];
    function [15:0] cmp_gesture_x;
        input [15:0] x;
        input enabled,compare,right_camera;
        reg [15:0] local_x;
        begin
            if(compare) local_x=(x>=16'd640)?x-16'd640:x;
            else local_x={1'b0,x[15:1]};
            if(local_x>16'd639) local_x=16'd639;
            if(!enabled) cmp_gesture_x=x;
            else cmp_gesture_x=right_camera?local_x+16'd640:local_x;
        end
    endfunction
    wire [15:0] gesture_x0=cmp_gesture_x(touch0_x,cmp_layout,cmp_compare_view,cmp_camera);
    wire [15:0] gesture_x1=cmp_gesture_x(touch1_x,cmp_layout,cmp_compare_view,cmp_camera);
    // In compare mode two fingers on different halves are two different
    // views of one picture, not one pinch: ignore that packet.
    wire cmp_cross_touch=cmp_compare_view && point0_valid && point1_valid && (touch0_panel!=(touch1_x>=(SCREEN_W/2)));
    wire [15:0] tap_dx=(touch0_x>last_tap_x)?touch0_x-last_tap_x:last_tap_x-touch0_x;
    wire [15:0] tap_dy=(touch0_y>last_tap_y)?touch0_y-last_tap_y:last_tap_y-touch0_y;
    wire top_control_hit=menu_visible && touch0_y>=16'd36 && touch0_y<16'd90 &&
        ((touch0_x>=16'd144 && touch0_x<16'd464) ||
         (touch0_x>=16'd480 && touch0_x<16'd800) ||
         (touch0_x>=16'd816 && touch0_x<16'd1136));
    // The four algorithm selectors are only meaningful in the normal view;
    // the post-processing page replaces them with eight tool cells.
    wire cmp_controls_visible=ENABLE_CMP_CONTROLS && (layout==LAYOUT_LEFT_COMPARE || layout==LAYOUT_RIGHT_COMPARE);
    wire cmp_button1=cmp_controls_visible && menu_visible && !post_mode && touch0_y>=16'd676 && touch0_y<16'd712 && touch0_x>=16'd286 && touch0_x<16'd634;
    wire cmp_button2=cmp_controls_visible && menu_visible && !post_mode && touch0_y>=16'd676 && touch0_y<16'd712 && touch0_x>=16'd646 && touch0_x<16'd994;
    wire bottom_control_hit=menu_visible && !post_mode && touch0_y>=16'd676 && touch0_y<16'd712 &&
        ((touch0_x>=16'd286 && touch0_x<16'd454) ||
         (touch0_x>=16'd466 && touch0_x<16'd634) ||
         (touch0_x>=16'd646 && touch0_x<16'd814) ||
         (touch0_x>=16'd826 && touch0_x<16'd994));
    wire reset_view_hit=menu_visible && touch0_x>=16'd16 && touch0_x<16'd112 &&
        touch0_y>=16'd16 && touch0_y<16'd66;
    // Rightmost key of the top row: EDIT enters the post-processing page,
    // the same key becomes RETURN and leaves it.
    wire edit_key_hit=menu_visible && touch0_x>=16'd1184 && touch0_x<16'd1280 &&
        touch0_y>=16'd32 && touch0_y<16'd94;
    // Eight 112-pixel tool cells on a 120-pixel pitch, starting at x=20.
    // Cell 7 ends at 972, which leaves the page selector its own group.
    wire post_item_band=menu_visible && post_mode && touch0_y>=16'd674 &&
        touch0_y<16'd714 && touch0_x>=16'd20 && touch0_x<16'd972;
    wire [2:0] post_item_sel=(touch0_x<16'd132)?3'd0:(touch0_x<16'd252)?3'd1:
        (touch0_x<16'd372)?3'd2:(touch0_x<16'd492)?3'd3:(touch0_x<16'd612)?3'd4:
        (touch0_x<16'd732)?3'd5:(touch0_x<16'd852)?3'd6:3'd7;
    wire [15:0] post_item_base=16'd20+{13'd0,post_item_sel}*16'd120;
    wire post_item_hit=post_item_band && touch0_x>=post_item_base &&
        touch0_x<post_item_base+16'd112;
    wire post_page_band=menu_visible && post_mode && touch0_y>=16'd674 &&
        touch0_y<16'd714;
    wire post_page_left_hit=post_page_band && touch0_x>=16'd1000 && touch0_x<16'd1120;
    wire post_page_right_hit=post_page_band && touch0_x>=16'd1128 && touch0_x<16'd1248;
    // Sub page of the selected tool, drawn in the strip above the tool row.
    // ERASER has no sub page at all.
    wire sub_page_active=post_mode && (post_tool!=3'd1);
    wire sub_band=menu_visible && sub_page_active && touch0_y>=16'd596 &&
        touch0_y<16'd652;
    // DOODLE: eight 64-pixel colour swatches on a 72-pixel pitch from x=40,
    // plus a full-width CLEAR key at the far right.
    wire doodle_page=sub_page_active && (post_tool==3'd0);
    wire [2:0] swatch_sel=(touch0_x<16'd112)?3'd0:(touch0_x<16'd184)?3'd1:
        (touch0_x<16'd256)?3'd2:(touch0_x<16'd328)?3'd3:(touch0_x<16'd400)?3'd4:
        (touch0_x<16'd472)?3'd5:(touch0_x<16'd544)?3'd6:3'd7;
    wire [15:0] swatch_base=16'd40+{13'd0,swatch_sel}*16'd72;
    wire swatch_hit=sub_band && doodle_page && touch0_x>=swatch_base &&
        touch0_x<swatch_base+16'd64;
    wire clear_hit=sub_band && doodle_page && touch0_x>=16'd1140 && touch0_x<16'd1250;
    // The six adjustment tools share one slider: track x=240..1049, knob
    // centre = 264 + 3*value, so the 0..255 range spans 765 pixels exactly.
    wire slider_hit=sub_band && !doodle_page && touch0_x>=16'd240 && touch0_x<16'd1050;
    wire slider_reset_hit=sub_band && !doodle_page && touch0_x>=16'd1184 && touch0_x<16'd1248;
    reg [2:0] sub_color_reg;
    reg [127:0] knob_q;
    reg slider_drag;
    // 3-bit tool + 1-bit page + byte offset requires seven bits.  The old
    // four-bit declaration truncated EXPOSURE (32), CONTRAST (48), etc. to
    // slot zero/eight: the knob moved, but the renderer's real fields stayed
    // at 128 forever.
    wire [6:0] knob_index={post_tool,post_page,3'b000};
    assign knob_value=knob_q[knob_index +: 8];
    assign knob_values=knob_q;
    assign sub_color=sub_color_reg;
    function [7:0] slider_value;
        input [15:0] x;
        reg [15:0] rel;
        begin
            if(x<=16'd264) slider_value=8'd0;
            else if(x>=16'd1029) slider_value=8'd255;
            else begin rel=x-16'd264; slider_value=rel/3; end
        end
    endfunction
    wire reset_views=touch_event && point0_valid && point0_in_range &&
        !contact_was_down && reset_view_hit;

    stereo_gesture_view gestures(
        .clk(clk),.rst_n(rst_n),.touch_event(touch_event),.menu_visible(menu_visible),
        .reset_views(reset_views),.sub_page_active(sub_page_active),.edit_lock(post_mode),
        .p0_valid(point0_valid && !cmp_cross_touch),.p1_valid(point1_valid && !cmp_cross_touch),
        .p0_in_range(point0_in_range && !cmp_cross_touch),.p1_in_range(point1_in_range && !cmp_cross_touch),
        .x0(gesture_x0[10:0]),.y0(touch0_y[10:0]),
        .x1(gesture_x1[10:0]),.y1(touch1_y[10:0]),.state_event(gesture_event),
        .left_step_q16(left_step_q16),.right_step_q16(right_step_q16),
        .left_rotation(left_rotation),.right_rotation(right_rotation),
        .left_origin_x(left_origin_x),.left_origin_y(left_origin_y),
        .right_origin_x(right_origin_x),.right_origin_y(right_origin_y));

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            mode_s0<=1; mode_s1<=1; mode_last<=1; mode_stable<=1; mode_count<=0;
            menu_s0<=1; menu_s1<=1; menu_last<=1; menu_stable<=1; menu_count<=0;
            post_mode<=0; post_tool<=3'd0; post_page<=1'b0;
            press_code<=3'd0; press_toggle<=1'b0;
            sub_color_reg<=3'd0; knob_q<={16{8'd128}}; slider_drag<=1'b0;
            image_mode<=0; menu_visible<=1; layout<=LAYOUT_STEREO; feature<=2'd0;
            zoom_active<=0; zoom_source<=0; ui_event<=0; init_publish<=1;
            interaction_active<=0;interaction_hold_count<=0;
            cmp_undistort<=2'b00;cmp_binary<=2'b00;
            tap_count<=0; tap_armed<=0; contact_was_down<=0;
            control_contact<=0;
            contact_timeout_count<=0;
            last_tap_x<=0; last_tap_y<=0;
        end else begin
            ui_event<=0;

            // Some panels occasionally omit the final zero-contact report.
            // Re-arm the press detector after a quiet interval, while normal
            // held contacts keep refreshing this watchdog with USB reports.
            if(touch_event) contact_timeout_count<=0;
            else if(contact_was_down || control_contact || slider_drag) begin
                if(contact_timeout_count>=CONTACT_TIMEOUT_CYCLES-1) begin
                    contact_timeout_count<=0;contact_was_down<=0;
                    control_contact<=0;slider_drag<=0;tap_armed<=0;
                end else contact_timeout_count<=contact_timeout_count+1'b1;
            end else contact_timeout_count<=0;

            // Interaction status only. Live capture need not stop: geometry
            // crosses in a mailbox and the renderer snapshots it per frame.
            if(point0_valid || point1_valid) begin
                interaction_active<=1'b1;
                interaction_hold_count<=0;
            end else if(interaction_active) begin
                if(interaction_hold_count>=RELEASE_HOLD_CYCLES-1) begin
                    interaction_active<=1'b0;
                    interaction_hold_count<=0;
                end else
                    interaction_hold_count<=interaction_hold_count+1'b1;
            end

            if(init_publish) begin ui_event<=1; init_publish<=0; end
            mode_s0<=key_mode_n; mode_s1<=mode_s0;
            menu_s0<=key_menu_n; menu_s1<=menu_s0;
            if(mode_s1==mode_last) begin
                if(mode_count<DB_CYCLES) mode_count<=mode_count+1'b1;
                else if(mode_stable!=mode_s1) begin
                    mode_stable<=mode_s1;
                    if(mode_stable && !mode_s1) begin
                        image_mode<=~image_mode; zoom_active<=0; ui_event<=1;
                    end
                end
            end else begin mode_last<=mode_s1; mode_count<=0; end

            if(menu_s1==menu_last) begin
                if(menu_count<DB_CYCLES) menu_count<=menu_count+1'b1;
                else if(menu_stable!=menu_s1) begin
                    menu_stable<=menu_s1;
                    if(menu_stable && !menu_s1) begin
                        menu_visible<=~menu_visible; ui_event<=1;
                    end
                end
            end else begin menu_last<=menu_s1; menu_count<=0; end

            if(tap_armed) begin
                if(tap_count==TAP_CYCLES-1) tap_armed<=0;
                else tap_count<=tap_count+1'b1;
            end

            if(touch_event) begin
                if(!point0_valid && !point1_valid) begin
                    contact_was_down<=0;
                    control_contact<=0;
                    slider_drag<=0;
                end else if(point0_valid && point0_in_range) begin
                    contact_was_down<=1;
                    if(control_contact) begin
                        // A press that began on a menu belongs to that menu
                        // until release, even when the finger leaves its box.
                        tap_armed<=0;
                    end else if(reset_view_hit && !contact_was_down) begin
                        // One explicit home action restores both halves to the
                        // original 1x, centered view and blinks the key blue.
                        zoom_active<=0;ui_event<=1;
                        press_code<=3'd1;press_toggle<=~press_toggle;
                        control_contact<=1;tap_armed<=0;
                    end else if(edit_key_hit && !contact_was_down) begin
                        // Rightmost top-row key: EDIT <-> RETURN.
                        post_mode<=~post_mode;
                        if(cmp_controls_visible) post_page<=layout==LAYOUT_RIGHT_COMPARE;
                        press_code<=3'd2;press_toggle<=~press_toggle;
                        zoom_active<=0;ui_event<=1;
                        control_contact<=1;tap_armed<=0;
                    end else if(post_item_hit && !contact_was_down) begin
                        // Eight post-processing tools, exclusive selection.
                        post_tool<=post_item_sel;
                        press_code<=3'd3;press_toggle<=~press_toggle;
                        ui_event<=1;
                        control_contact<=1;tap_armed<=0;
                    end else if((post_page_left_hit || post_page_right_hit) &&
                        !contact_was_down) begin
                        // Which half the current tool is applied to.
                        if(!cmp_controls_visible) post_page<=post_page_right_hit;
                        press_code<=3'd4;press_toggle<=~press_toggle;
                        ui_event<=1;
                        control_contact<=1;tap_armed<=0;
                    end else if(top_control_hit && !contact_was_down) begin
                        if(touch0_x<16'd464) layout<=LAYOUT_STEREO;
                        else if(touch0_x<16'd800) layout<=LAYOUT_LEFT_COMPARE;
                        else layout<=LAYOUT_RIGHT_COMPARE;
                        // CMP EDIT always targets the page's own camera.
                        if(ENABLE_CMP_CONTROLS && post_mode && touch0_x>=16'd480)
                            post_page<=(touch0_x>=16'd800);
                        zoom_active<=0;ui_event<=1;
                        control_contact<=1;tap_armed<=0;
                    end else if(swatch_hit && !contact_was_down) begin
                        // DOODLE palette: keep the chosen colour, black border.
                        sub_color_reg<=swatch_sel;
                        press_code<=3'd5;press_toggle<=~press_toggle;
                        ui_event<=1;
                        control_contact<=1;tap_armed<=0;
                    end else if(clear_hit && !contact_was_down) begin
                        // One-key clear blinks blue; the stroke buffer is a later
                        // stage, so only the press feedback exists today.
                        press_code<=3'd6;press_toggle<=~press_toggle;
                        ui_event<=1;
                        control_contact<=1;tap_armed<=0;
                    end else if(slider_reset_hit && !contact_was_down) begin
                        // Reset only the selected tool and selected eye.
                        knob_q[knob_index +: 8]<=8'd128;
                        press_code<=3'd7;press_toggle<=~press_toggle;
                        ui_event<=1;control_contact<=1;tap_armed<=0;
                    end else if(slider_drag && contact_was_down) begin
                        // Continuous drag: the knob follows the finger.
                        knob_q[knob_index +: 8]<=slider_value(touch0_x);
                        ui_event<=1;tap_armed<=0;
                    end else if(slider_hit && !contact_was_down) begin
                        slider_drag<=1;
                        knob_q[knob_index +: 8]<=slider_value(touch0_x);
                        ui_event<=1;tap_armed<=0;
                    end else if(cmp_controls_visible && menu_visible && !post_mode && touch0_y>=16'd676 && touch0_y<16'd712 && touch0_x>=16'd274 && touch0_x<16'd1006 && !contact_was_down) begin
                        if(cmp_button1) begin
                            cmp_undistort[layout==LAYOUT_RIGHT_COMPARE]<=~cmp_undistort[layout==LAYOUT_RIGHT_COMPARE];ui_event<=1;
                        end else if(cmp_button2) begin
                            cmp_binary[layout==LAYOUT_RIGHT_COMPARE]<=~cmp_binary[layout==LAYOUT_RIGHT_COMPARE];ui_event<=1;
                        end
                        control_contact<=1;tap_armed<=0;
                    end else if(bottom_control_hit && !cmp_controls_visible && !contact_was_down) begin
                        if(touch0_x<16'd454) feature<=2'd0;
                        else if(touch0_x<16'd634) feature<=2'd1;
                        else if(touch0_x<16'd814) feature<=2'd2;
                        else feature<=2'd3;
                        zoom_active<=0;ui_event<=1;
                        control_contact<=1;tap_armed<=0;
                    end else if(post_mode) begin
                        // EDIT owns the image area: drawing/erasing is handled
                        // by the canvas, never by pan, pinch or double-tap.
                        tap_armed<=0;
                    end else if(point1_valid) begin
                        tap_armed<=0;
                    end else begin
                        if(!contact_was_down) begin
                            if(tap_armed && same_tap_panel && tap_dx<=16 && tap_dy<=16) begin
                                zoom_active<=~zoom_active;
                                if(layout==LAYOUT_STEREO) zoom_source<=touch0_x<640 ? 2'd0:2'd1;
                                else if(layout==LAYOUT_LEFT_COMPARE) zoom_source<=touch0_x<640 ? 2'd0:2'd2;
                                else zoom_source<=touch0_x<640 ? 2'd1:2'd3;
                                tap_armed<=0;ui_event<=1;
                            end else begin
                                tap_armed<=1;tap_count<=0;last_tap_x<=touch0_x;last_tap_y<=touch0_y;
                            end
                        end else if(tap_dx>8 || tap_dy>8) begin
                            tap_armed<=0;
                        end
                    end
                end
            end
        end
    end
endmodule

// Half-resolution annotation plane for one 640x720 source image.  A stored
// four-bit value is {valid,palette}; zero is transparent.  Touch coordinates
// are converted back through the current inverse view transform, so marks are
// attached to source pixels and naturally follow later pan/zoom operations.
module source_doodle_canvas(
    input wire sys_clk,input wire mem_clk,input wire rst_n,
    input wire touch_event,input wire point_valid,input wire [15:0] point_x,input wire [15:0] point_y,
    input wire enable_draw,input wire enable_erase,input wire side_right,input wire [2:0] color,
    input wire clear_event,
    input wire [16:0] view_step_q16,input wire signed [27:0] view_origin_x,
    input wire signed [27:0] view_origin_y,
    input wire [16:0] read_address,output wire [3:0] read_data
);
    // {generation[7:0],valid,palette[2:0]}. Clearing increments generation,
    // so it is instantaneous and never requires a full-RAM reset sweep.
    (* ram_style="block" *) reg [11:0] canvas [0:115199];
    wire [31:0] mx_product={16'd0,point_x}*32'd1279;
    wire [31:0] my_product={16'd0,point_y}*32'd719;
    wire [10:0] screen_x=mx_product>>14;
    wire [9:0] screen_y=my_product>>14;
    wire touch_side=screen_x>=11'd640;
    wire [9:0] local_x=touch_side ? screen_x-11'd640 : screen_x[9:0];
    wire signed [31:0] source_x_q=$signed(view_origin_x)+$signed({1'b0,local_x})*$signed({1'b0,view_step_q16});
    wire signed [31:0] source_y_q=$signed(view_origin_y)+$signed({1'b0,screen_y})*$signed({1'b0,view_step_q16});
    wire source_valid=source_x_q>=0 && source_x_q<32'sd41943040 &&
        source_y_q>=0 && source_y_q<32'sd47185920;
    wire [8:0] center_x=source_x_q[25:17];
    wire [8:0] center_y=source_y_q[25:17];
    wire draw_command=touch_event && point_valid && touch_side==side_right &&
        screen_y>=10'd100 && screen_y<10'd596 && source_valid &&
        (enable_draw||enable_erase);
    wire release_command=touch_event && !point_valid;
    wire command_event=clear_event||release_command||draw_command;
    wire [23:0] command_data={clear_event,release_command,enable_erase,color,center_x,center_y};
    wire command_ready;
    wire [23:0] command_core;
    touch_event_cdc #(.WIDTH(24)) u_doodle_command_cdc(
        .src_clk(sys_clk),.dst_clk(mem_clk),.rst_n(rst_n),
        .src_event(command_event),.src_data(command_data),
        .dst_event(command_ready),.dst_data(command_core)
    );
    reg busy;
    reg signed [4:0] brush_x,brush_y;
    reg [3:0] brush_limit;
    reg [7:0] generation;
    reg [11:0] read_word;
    assign read_data=(read_word[11:4]==generation && read_word[3])?read_word[3:0]:4'd0;
    reg erase_latch;
    reg [2:0] color_latch;
    reg [8:0] line_x,line_y,target_x,target_y,last_x,last_y;
    reg last_valid;
    reg signed [11:0] line_dx,line_dy,line_err;
    reg signed [1:0] line_sx,line_sy;
    reg signed [10:0] write_x,write_y;
    reg [7:0] radius_squared;
    reg signed [12:0] step_err,step_e2;
    reg [8:0] step_x,step_y;
    wire [8:0] command_dx=command_core[17:9]>=last_x ? command_core[17:9]-last_x : last_x-command_core[17:9];
    wire [8:0] command_dy=command_core[8:0]>=last_y ? command_core[8:0]-last_y : last_y-command_core[8:0];
    integer write_address;

    // Both ports use mem_clk.  Keeping all accesses in one clock domain lets
    // PDS infer one true dual-port block RAM instead of expanding the canvas
    // into flip-flops; touch commands cross through the mailbox above.
    always @(posedge mem_clk or negedge rst_n) begin
        if(!rst_n) begin
            busy<=0;generation<=8'd1;brush_x<=-5'sd3;brush_y<=-5'sd3;brush_limit<=4'd3;
            erase_latch<=0;color_latch<=0;line_x<=0;line_y<=0;target_x<=0;target_y<=0;
            last_x<=0;last_y<=0;last_valid<=0;line_dx<=0;line_dy<=0;line_err<=0;
            line_sx<=1;line_sy<=1;
            read_word<=0;
        end else begin
            read_word<=canvas[read_address];
        if(command_ready && command_core[23]) begin
            generation<=generation+1'b1;busy<=0;last_valid<=0;
        end else if(command_ready && command_core[22]) begin
            busy<=0;last_valid<=0;
        end else if(command_ready) begin
            target_x<=command_core[17:9];target_y<=command_core[8:0];
            erase_latch<=command_core[21];color_latch<=command_core[20:18];
            brush_limit<=command_core[21]?4'd7:4'd3;
            brush_x<=command_core[21]?-5'sd7:-5'sd3;
            brush_y<=command_core[21]?-5'sd7:-5'sd3;
            if(last_valid) begin
                line_x<=last_x;line_y<=last_y;
                line_dx<=$signed({1'b0,command_dx});
                line_dy<=-$signed({1'b0,command_dy});
                line_err<=$signed({1'b0,command_dx})-$signed({1'b0,command_dy});
                line_sx<=command_core[17:9]>=last_x ? 2'sd1:-2'sd1;
                line_sy<=command_core[8:0]>=last_y ? 2'sd1:-2'sd1;
            end else begin
                line_x<=command_core[17:9];line_y<=command_core[8:0];
                line_dx<=0;line_dy<=0;line_err<=0;line_sx<=1;line_sy<=1;
            end
            last_x<=command_core[17:9];last_y<=command_core[8:0];last_valid<=1;busy<=1;
        end else if(busy) begin
            write_x=$signed({1'b0,line_x})+brush_x;
            write_y=$signed({1'b0,line_y})+brush_y;
            radius_squared=brush_x*brush_x+brush_y*brush_y;
            if(write_x>=0 && write_x<320 && write_y>=0 && write_y<360 &&
                radius_squared<=brush_limit*brush_limit) begin
                write_address=write_y*320+write_x;
                canvas[write_address]<=erase_latch ? {generation,4'd0} :
                    {generation,1'b1,color_latch};
            end
            if(brush_x==$signed({1'b0,brush_limit})) begin
                brush_x<=-$signed({1'b0,brush_limit});
                if(brush_y==$signed({1'b0,brush_limit})) begin
                    brush_y<=-$signed({1'b0,brush_limit});
                    if(line_x==target_x && line_y==target_y) busy<=0;
                    else begin
                        // Bresenham step between reports: every missing touch
                        // sample is filled with another circular brush stamp.
                        step_err=line_err;step_e2=line_err<<<1;
                        step_x=line_x;step_y=line_y;
                        if(step_e2>=line_dy) begin step_err=step_err+line_dy;step_x=$signed({1'b0,line_x})+line_sx;end
                        if(step_e2<=line_dx) begin step_err=step_err+line_dx;step_y=$signed({1'b0,line_y})+line_sy;end
                        line_err<=step_err;line_x<=step_x;line_y<=step_y;
                    end
                end
                else brush_y<=brush_y+1'b1;
            end else brush_x<=brush_x+1'b1;
        end
        end
    end
endmodule

// Mapping: source_q16 = origin_q16 + display_pixel * step_q16.
// step=65536 is 1x; step=16384 is 4x. No division is done in the
// pixel/DDR clock. Euclidean distance uses an 11-cycle integer square root;
// a 32-cycle divider computes step_begin * distance_begin / distance_now.
// A pinch keeps the original source point under the moving finger midpoint.
module stereo_gesture_view(
    input wire clk,rst_n,touch_event,menu_visible,reset_views,sub_page_active,edit_lock,
    input wire p0_valid,p1_valid,p0_in_range,p1_in_range,
    input wire [10:0] x0,y0,x1,y1,
    output reg state_event,
    output reg [16:0] left_step_q16,right_step_q16,
    output reg [1:0] left_rotation,right_rotation,
    output reg signed [27:0] left_origin_x,left_origin_y,right_origin_x,right_origin_y
);
    localparam IDLE=0,ROOT=1,SPAN=2,DIVIDE=3,ANCHOR=4,APPLY=5;
    reg [2:0] state;
    reg pending;
    reg qv0,qv1,qr0,qr1,qmenu,qedit;
    reg [10:0] qx0,qy0,qx1,qy1;
    wire [10:0] one_x=qv0?qx0:qx1,one_y=qv0?qy0:qy1;
    wire one_panel=one_x>=640;
    reg contact_down,blocked,owner,drag_active,pinch_active,rotate_committed;
    reg [10:0] last_x,last_y;
    reg [10:0] reference_span;
    reg [16:0] reference_step;
    reg signed [31:0] anchor_x,anchor_y;
    reg signed [11:0] reference_dx,reference_dy;
    wire signed [11:0] current_dx=$signed({1'b0,qx1})-$signed({1'b0,qx0});
    wire signed [11:0] current_dy=$signed({1'b0,qy1})-$signed({1'b0,qy0});
    wire signed [24:0] rotate_dot=reference_dx*current_dx+reference_dy*current_dy;
    wire signed [24:0] rotate_cross=reference_dx*current_dy-reference_dy*current_dx;
    wire [49:0] rotate_cross_sq=rotate_cross*rotate_cross;
    wire [49:0] rotate_dot_sq=rotate_dot*rotate_dot;
    // A pure finger-ID swap has dot<0 but cross==0; it is not a physical
    // turn and must never rotate the image.
    wire rotate_over_60=(rotate_cross!=0) &&
        ((rotate_dot<=0) || (rotate_cross_sq>rotate_dot_sq*3));
    reg work_owner;
    reg [11:0] center_x2,center_y2;
    reg [16:0] work_step;
    reg signed [31:0] work_x,work_y;
    wire [16:0] current_step=owner?right_step_q16:left_step_q16;
    wire signed [27:0] current_x=owner?right_origin_x:left_origin_x;
    wire signed [27:0] current_y=owner?right_origin_y:left_origin_y;
    wire signed [12:0] delta_x=$signed({2'b0,one_x})-$signed({2'b0,last_x});
    wire signed [12:0] delta_y=$signed({2'b0,one_y})-$signed({2'b0,last_y});
    wire [10:0] dist_x=(qx0>qx1)?qx0-qx1:qx1-qx0;
    wire [10:0] dist_y=(qy0>qy1)?qy0-qy1:qy1-qy0;
    wire [21:0] distance_squared=dist_x*dist_x+dist_y*dist_y;

    reg [21:0] root_bits;
    reg [23:0] root_rem;
    reg [10:0] root_value;
    reg [3:0] root_count;
    wire [23:0] root_next={root_rem[21:0],root_bits[21:20]};
    wire [23:0] root_trial=({13'd0,root_value}<<2)|24'd1;
    reg [31:0] quotient;
    reg [32:0] remainder;
    reg [10:0] divisor;
    reg [5:0] divide_count;
    wire [32:0] divide_next={remainder[31:0],quotient[31]};
    wire [16:0] clamped_step=(quotient>65536)?17'd65536:
        ((quotient<16384)?17'd16384:quotient[16:0]);
    wire [31:0] reference_center_x=center_x2*current_step;
    wire [31:0] reference_center_y=center_y2*current_step;
    wire [31:0] new_center_x=center_x2*clamped_step;
    wire [31:0] new_center_y=center_y2*clamped_step;

    function menu_hit;
        input [10:0] x,y;
        begin
            menu_hit=(x>=16 && x<112 && y>=16 && y<66) || (y>=36 && y<90 &&
                ((x>=144 && x<464)||(x>=480 && x<800)||(x>=816 && x<1136))) ||
                (y>=676 && y<712 &&
                ((x>=286 && x<454)||(x>=466 && x<634)||(x>=646 && x<814)||(x>=826 && x<994))) ||
                // EDIT/RETURN key and the post-processing tool / page row.
                (x>=1184 && x<1280 && y>=32 && y<94) ||
                (y>=674 && y<714 && ((x>=20 && x<972)||(x>=1000 && x<1248)));
        end
    endfunction
    // The whole top strip and the whole bottom strip belong to the UI, not to
    // the image: a contact that starts there may press keys but can never pan
    // or pinch. The tool sub-page strip joins the bottom strip while it is up.
    function guard_band;
        input [10:0] x,y;
        begin
            guard_band=(y<100) || (y>=660) || (sub_page_active && y>=596 && y<652);
        end
    endfunction
    wire packet_control=qedit || (qmenu && ((qv0 && (menu_hit(qx0,qy0)||guard_band(qx0,qy0)))||
        (qv1 && (menu_hit(qx1,qy1)||guard_band(qx1,qy1)))));
    function signed [27:0] clamp_origin;
        input signed [31:0] value;
        input [16:0] step;
        input integer extent;
        reg signed [31:0] low_limit,high_limit;
        begin
            // At 1x expose at most a narrow 64-pixel elastic border. Release
            // returns it to zero. Zoomed views keep the wider navigation area.
            if(step==17'd65536) begin
                low_limit=-32'sd4194304;
                high_limit=32'sd4194304;
            end else begin
                low_limit=-(extent-64)*$signed({1'b0,step});
                high_limit=extent*65536-64*$signed({1'b0,step});
            end
            if(value<low_limit) clamp_origin=low_limit[27:0];
            else if(value>high_limit) clamp_origin=high_limit[27:0];
            else clamp_origin=value[27:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;pending<=0;state_event<=0;
            qv0<=0;qv1<=0;qr0<=0;qr1<=0;qmenu<=0;qedit<=0;
            qx0<=0;qy0<=0;qx1<=0;qy1<=0;
            contact_down<=0;blocked<=0;owner<=0;drag_active<=0;pinch_active<=0;rotate_committed<=0;
            last_x<=0;last_y<=0;reference_span<=0;reference_step<=65536;
            anchor_x<=0;anchor_y<=0;work_owner<=0;
            center_x2<=0;center_y2<=0;work_step<=65536;work_x<=0;work_y<=0;
            root_bits<=0;root_rem<=0;root_value<=0;root_count<=0;
            quotient<=0;remainder<=0;divisor<=1;divide_count<=0;
            left_step_q16<=65536;right_step_q16<=65536;
            left_rotation<=0;right_rotation<=0;reference_dx<=0;reference_dy<=0;
            left_origin_x<=0;left_origin_y<=0;right_origin_x<=0;right_origin_y<=0;
        end else begin
            state_event<=0;
            if(reset_views) begin
                state<=IDLE;pending<=0;contact_down<=1;blocked<=1;
                drag_active<=0;pinch_active<=0;
                left_step_q16<=65536;right_step_q16<=65536;
                left_rotation<=0;right_rotation<=0;
                left_origin_x<=0;left_origin_y<=0;
                right_origin_x<=0;right_origin_y<=0;
                state_event<=1;
            end else case(state)
                IDLE: if(pending) begin
                    pending<=0;
                    if(!qv0 && !qv1) begin
                        // At 1x a released image springs back to its natural
                        // boundary. Zoomed views retain their chosen crop.
                        if(left_step_q16==17'd65536) begin left_origin_x<=0;left_origin_y<=0;end
                        if(right_step_q16==17'd65536) begin right_origin_x<=0;right_origin_y<=0;end
                        if((left_step_q16==17'd65536 && (left_origin_x!=0 || left_origin_y!=0)) ||
                           (right_step_q16==17'd65536 && (right_origin_x!=0 || right_origin_y!=0)))
                            state_event<=1;
                        contact_down<=0;blocked<=0;drag_active<=0;pinch_active<=0;rotate_committed<=0;
                    end else if((!qv0 || qr0) && (!qv1 || qr1)) begin
                        contact_down<=1;
                        if(!contact_down) begin owner<=one_panel;blocked<=packet_control;end
                        if((contact_down && blocked)||(!contact_down && packet_control)) begin
                            drag_active<=0;pinch_active<=0;
                        end else if(qv0 && qv1) begin
                            drag_active<=0;
                            if((qx0>=640)!=(qx1>=640) || (contact_down && one_panel!=owner) ||
                                guard_band(qx0,qy0) || guard_band(qx1,qy1))
                                pinch_active<=0;
                            else begin
                                work_owner<=one_panel;
                                center_x2<={1'b0,qx0}+{1'b0,qx1}-(one_panel?12'd1280:12'd0);
                                center_y2<={1'b0,qy0}+{1'b0,qy1};
                                root_bits<=distance_squared;
                                root_rem<=0;root_value<=0;root_count<=0;
                                state<=ROOT;
                            end
                        end else begin
                            pinch_active<=0;
                            last_x<=one_x;last_y<=one_y;
                            if(contact_down && one_panel!=owner) drag_active<=0;
                            else if(!contact_down || !drag_active) drag_active<=1;
                            else if(one_panel==owner && delta_x<=192 && delta_x>=-192 &&
                                delta_y<=192 && delta_y>=-192 && (delta_x!=0 || delta_y!=0)) begin
                                work_owner<=owner;work_step<=current_step;
                                work_x<=$signed(current_x)-delta_x*$signed({1'b0,current_step});
                                work_y<=$signed(current_y)-delta_y*$signed({1'b0,current_step});
                                state<=APPLY;
                            end
                        end
                    end
                end
                ROOT: begin
                    root_bits<=root_bits<<2;
                    if(root_next>=root_trial) begin
                        root_rem<=root_next-root_trial;
                        root_value<={root_value[9:0],1'b1};
                    end else begin
                        root_rem<=root_next;
                        root_value<={root_value[9:0],1'b0};
                    end
                    if(root_count==10) state<=SPAN;
                    else root_count<=root_count+1'b1;
                end
                SPAN: begin
                    if(root_value<24) begin
                        // Reject nearly coincident fingers; never divide by
                        // zero or amplify a one-pixel separation into a jump.
                        pinch_active<=0;state<=IDLE;
                    end else if(!pinch_active) begin
                        pinch_active<=1;reference_span<=root_value;reference_step<=current_step;
                        reference_dx<=current_dx;reference_dy<=current_dy;rotate_committed<=0;
                        anchor_x<=$signed(current_x)+$signed(reference_center_x>>1);
                        anchor_y<=$signed(current_y)+$signed(reference_center_y>>1);
                        state<=IDLE;
                    end else begin
                        if(!rotate_committed && rotate_over_60) begin
                            // Crossing sixty degrees is intentionally a hard
                            // quarter-turn snap; another gesture can advance
                            // to the next quadrant.
                            if(work_owner) begin
                                right_rotation<=right_rotation+(rotate_cross<0 ? 2'd1:2'd3);
                                right_step_q16<=17'd65536;right_origin_x<=0;right_origin_y<=0;
                            end else begin
                                left_rotation<=left_rotation+(rotate_cross<0 ? 2'd1:2'd3);
                                left_step_q16<=17'd65536;left_origin_x<=0;left_origin_y<=0;
                            end
                            rotate_committed<=1;state_event<=1;state<=IDLE;
                        end else begin
                            quotient<=reference_step*reference_span;
                            remainder<=0;divisor<=root_value;divide_count<=0;
                            state<=DIVIDE;
                        end
                    end
                end
                DIVIDE: begin
                    if(divide_next>={22'd0,divisor}) begin
                        remainder<=divide_next-{22'd0,divisor};
                        quotient<={quotient[30:0],1'b1};
                    end else begin
                        remainder<=divide_next;quotient<={quotient[30:0],1'b0};
                    end
                    if(divide_count==31) state<=ANCHOR;
                    else divide_count<=divide_count+1'b1;
                end
                ANCHOR: begin
                    work_step<=clamped_step;
                    work_x<=anchor_x-$signed(new_center_x>>1);
                    work_y<=anchor_y-$signed(new_center_y>>1);
                    state<=APPLY;
                end
                APPLY: begin
                    if(work_owner) begin
                        right_step_q16<=work_step;
                        right_origin_x<=clamp_origin(work_x,work_step,640);
                        right_origin_y<=clamp_origin(work_y,work_step,720);
                    end else begin
                        left_step_q16<=work_step;
                        left_origin_x<=clamp_origin(work_x,work_step,640);
                        left_origin_y<=clamp_origin(work_y,work_step,720);
                    end
                    state_event<=1;state<=IDLE;
                end
                default: state<=IDLE;
            endcase
            // Latest report mailbox while arithmetic is busy. Real HID
            // reports are milliseconds apart; each calculation takes <2 us.
            if(touch_event && !reset_views) begin
                pending<=1;qv0<=p0_valid;qv1<=p1_valid;qr0<=p0_in_range;qr1<=p1_in_range;
                qx0<=x0;qy0<=y0;qx1<=x1;qy1<=y1;qmenu<=menu_visible;qedit<=edit_lock;
            end
        end
    end
endmodule
