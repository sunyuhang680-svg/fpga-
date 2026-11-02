// SPDX-License-Identifier: MIT
// Two retained animated water-ripple markers.  Coordinates are intentionally
// not rendered in the normal UI; no full-frame RAM is needed.
// Updates are committed on frame_start (positive VS edge, during blanking).
module touch_menu_layer #(
    parameter integer PIXEL_CLK_HZ=74250000,
    parameter integer SCREEN_W=1280, SCREEN_H=720,
    parameter integer CROSS_RADIUS=12,
    parameter integer SHOW_DEBUG=0,
    parameter [23:0] COLOR0=24'hff3030, COLOR1=24'h30ff30
)(
    input wire clk, rst_n, frame_start,
    input wire chip_identified,
    input wire spi_probe_event,
    input wire [24:0] spi_probe_data,
    input wire diagnostic_event,
    input wire [511:0] diagnostic_data,
    input wire [15:0] pixel_x, pixel_y,
    input wire pixel_de, touch_event,
    input wire point0_valid, point1_valid,
    input wire [15:0] point0_x, point0_y, point1_x, point1_y,
    output wire menu_valid,
    output wire [23:0] menu_rgb,
    output reg clear_tick
);
    reg [31:0] second_counter;
    reg [5:0] ripple_age;
    reg pending0, pending1, show0, show1;
    reg chip_show;
    reg probe_pending_valid,probe_show_valid;
    reg [24:0] probe_pending,probe_show;
    reg [15:0] px0,py0,px1,py1,sx0,sy0,sx1,sy1;
    reg [65:0] coord_pending,coord_show;
    reg [511:0] diagnostic_pending,diagnostic_show;
    reg [31:0] digits_pending0,digits_pending1,digits_show0,digits_show1;
    localparam integer LABEL_W=180,LABEL_H=16,LABEL_GAP=4;
    localparam integer LABELS_FIT=(SCREEN_W>=LABEL_W && SCREEN_H>=LABEL_H);
    reg digits_busy;
    reg [3:0] digits_bit;
    reg [15:0] source_x0,source_y0,source_x1,source_y1;
    reg [15:0] accum_x0,accum_y0,accum_x1,accum_y1;
    // One double-dabble step per pixel clock, four coordinates in parallel.
    // Sixteen short steps avoid a long decimal converter in one clock period.
    function [15:0] bcd_step;
        input [15:0] previous;
        input next_bit;
        integer digit;
        reg [15:0] converted;
        begin
            converted=0;
            for(digit=0;digit<4;digit=digit+1)
                if(previous[digit*4 +: 4]>=5)
                    converted[digit*4 +: 4]=previous[digit*4 +: 4]+4'd3;
                else converted[digit*4 +: 4]=previous[digit*4 +: 4];
            bcd_step={converted[14:0],next_bit};
        end
    endfunction
    wire [15:0] next_x0=bcd_step(accum_x0,source_x0[15]);
    wire [15:0] next_y0=bcd_step(accum_y0,source_y0[15]);
    wire [15:0] next_x1=bcd_step(accum_x1,source_x1[15]);
    wire [15:0] next_y1=bcd_step(accum_y1,source_y1[15]);
    // Explicit 32-bit operands avoid narrow Verilog multiply intermediates.
    wire [31:0] mx0={16'd0,point0_x}*(SCREEN_W-1);
    wire [31:0] my0={16'd0,point0_y}*(SCREEN_H-1);
    wire [31:0] mx1={16'd0,point1_x}*(SCREEN_W-1);
    wire [31:0] my1={16'd0,point1_y}*(SCREEN_H-1);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            second_counter<=0; clear_tick<=0;ripple_age<=0;
            pending0<=0; pending1<=0; show0<=0; show1<=0;
            chip_show<=0;
            probe_pending_valid<=0;probe_show_valid<=0;probe_pending<=0;probe_show<=0;
            px0<=0;py0<=0;px1<=0;py1<=0;
            sx0<=0;sy0<=0;sx1<=0;sy1<=0;
            coord_pending<=0;coord_show<=0;
            diagnostic_pending<=0;diagnostic_show<=0;
            digits_pending0<=0;digits_pending1<=0;digits_show0<=0;digits_show1<=0;
            digits_busy<=0;digits_bit<=0;
            source_x0<=0;source_y0<=0;source_x1<=0;source_y1<=0;
            accum_x0<=0;accum_y0<=0;accum_x1<=0;accum_y1<=0;
        end else begin
            if(digits_busy)begin
                source_x0<=source_x0<<1;source_y0<=source_y0<<1;
                source_x1<=source_x1<<1;source_y1<=source_y1<<1;
                accum_x0<=next_x0;accum_y0<=next_y0;
                accum_x1<=next_x1;accum_y1<=next_y1;
                if(digits_bit==15)begin
                    digits_busy<=0;
                    digits_pending0<={next_x0,next_y0};digits_pending1<={next_x1,next_y1};
                end else digits_bit<=digits_bit+1'b1;
            end
            if(diagnostic_event)diagnostic_pending<=diagnostic_data;
            if (touch_event) begin
                coord_pending[65:64]<={point1_valid,point0_valid};
                if(point0_valid) coord_pending[31:0]<={point0_y,point0_x};
                if(point1_valid) coord_pending[63:32]<={point1_y,point1_x};
            end
            if (spi_probe_event) begin probe_pending<=spi_probe_data;probe_pending_valid<=1;end
            clear_tick<=0;
            if (second_counter==PIXEL_CLK_HZ-1) begin
                second_counter<=0; clear_tick<=1;
                pending0<=0; pending1<=0;
            end else second_counter<=second_counter+1'b1;
            // Releases leave the last mark visible until the next clear tick.
            // A new active report replaces both marks; same-cycle touch wins clear.
            if (touch_event && (point0_valid || point1_valid)) begin
                ripple_age<=0;
                pending0<=point0_valid && point0_x<=16384 && point0_y<=16384;
                pending1<=point1_valid && point1_x<=16384 && point1_y<=16384;
                px0<=mx0>>14; py0<=my0>>14; px1<=mx1>>14; py1<=my1>>14;
                if(SHOW_DEBUG==0 && LABELS_FIT)begin
                    digits_busy<=1;digits_bit<=0;
                    source_x0<=mx0>>14;source_y0<=my0>>14;
                    source_x1<=mx1>>14;source_y1<=my1>>14;
                    accum_x0<=0;accum_y0<=0;accum_x1<=0;accum_y1<=0;
                end
            end
            if (frame_start) begin
                if((show0 || show1) && ripple_age<6'd4)
                    ripple_age<=ripple_age+1'b1;
                coord_show<=coord_pending;
                diagnostic_show<=diagnostic_pending;
                chip_show<=chip_identified;
                probe_show<=probe_pending;probe_show_valid<=probe_pending_valid;
                // Keep position and text from the same touch tuple. If conversion
                // meets a frame edge, retain the old complete tuple one more frame.
                if(!digits_busy)begin
                    show0<=pending0; show1<=pending1;
                    sx0<=px0;sy0<=py0;sx1<=px1;sy1<=py1;
                    digits_show0<=digits_pending0;digits_show1<=digits_pending1;
                end
            end
        end
    end
    // Subtraction after ordering prevents underflow at screen edges.
    wire [15:0] dx0=(pixel_x>=sx0)?pixel_x-sx0:sx0-pixel_x;
    wire [15:0] dy0=(pixel_y>=sy0)?pixel_y-sy0:sy0-pixel_y;
    wire [15:0] dx1=(pixel_x>=sx1)?pixel_x-sx1:sx1-pixel_x;
    wire [15:0] dy1=(pixel_y>=sy1)?pixel_y-sy1:sy1-pixel_y;
    // One small circular wave. It expands from radius 5 to radius 8 and then
    // disappears after four frames (about 0.067 s at 60 Hz).
    wire ripple_box0=(dx0<=16'd11) && (dy0<=16'd11);
    wire ripple_box1=(dx1<=16'd11) && (dy1<=16'd11);
    wire [13:0] dx0_sq={7'd0,dx0[6:0]}*{7'd0,dx0[6:0]};
    wire [13:0] dy0_sq={7'd0,dy0[6:0]}*{7'd0,dy0[6:0]};
    wire [13:0] dx1_sq={7'd0,dx1[6:0]}*{7'd0,dx1[6:0]};
    wire [13:0] dy1_sq={7'd0,dy1[6:0]}*{7'd0,dy1[6:0]};
    wire [14:0] distance_sq0={1'b0,dx0_sq}+{1'b0,dy0_sq};
    wire [14:0] distance_sq1={1'b0,dx1_sq}+{1'b0,dy1_sq};
    wire [6:0] ring_radius=7'd5+{1'b0,ripple_age};
    wire [13:0] ring_square={7'd0,ring_radius}*{7'd0,ring_radius};
    // +/-2r around r^2 makes each circular wave approximately two pixels
    // thick, so it remains readable over both dark and bright image content.
    wire [14:0] ring_tolerance={7'd0,ring_radius,1'b0};
    wire circle0=(distance_sq0>={1'b0,ring_square}-ring_tolerance) &&
                 (distance_sq0<={1'b0,ring_square}+ring_tolerance);
    wire circle1=(distance_sq1>={1'b0,ring_square}-ring_tolerance) &&
                 (distance_sq1<={1'b0,ring_square}+ring_tolerance);
    wire hit0=show0 && ripple_age<6'd4 &&
        ripple_box0 && circle0;
    wire hit1=show1 && ripple_age<6'd4 &&
        ripple_box1 && circle1;
    // Persistent 12x12 status dot near top-left; independent of touch clear timer.
    // Red: no expected revision observed yet. Green: SPI read returned 0x13.
    wire status_hit=(SHOW_DEBUG!=0) && pixel_x>=24 && pixel_x<36 && pixel_y>=24 && pixel_y<36;
    wire probe_hit=(SHOW_DEBUG!=0) && pixel_x>=24 && pixel_x<36 && pixel_y>=48 && pixel_y<60;
    // 5x7 hexadecimal font, 2x scale, one blank column per character.
    function [34:0] font;
        input [4:0] ch;
        begin case(ch)
          0:font={5'b01110,5'b10001,5'b10011,5'b10101,5'b11001,5'b10001,5'b01110};
          1:font={5'b00100,5'b01100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
          2:font={5'b01110,5'b10001,5'b00001,5'b00010,5'b00100,5'b01000,5'b11111};
          3:font={5'b11110,5'b00001,5'b00001,5'b01110,5'b00001,5'b00001,5'b11110};
          4:font={5'b00010,5'b00110,5'b01010,5'b10010,5'b11111,5'b00010,5'b00010};
          5:font={5'b11111,5'b10000,5'b10000,5'b11110,5'b00001,5'b00001,5'b11110};
          6:font={5'b01110,5'b10000,5'b10000,5'b11110,5'b10001,5'b10001,5'b01110};
          7:font={5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b01000,5'b01000};
          8:font={5'b01110,5'b10001,5'b10001,5'b01110,5'b10001,5'b10001,5'b01110};
          9:font={5'b01110,5'b10001,5'b10001,5'b01111,5'b00001,5'b00001,5'b01110};
          10:font={5'b01110,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
          11:font={5'b11110,5'b10001,5'b10001,5'b11110,5'b10001,5'b10001,5'b11110};
          12:font={5'b01111,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b01111};
          13:font={5'b11110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b11110};
          14:font={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b11111};
          15:font={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b10000};
          16:font={5'b11110,5'b10001,5'b10001,5'b11110,5'b10100,5'b10010,5'b10001}; // R
          17:font={5'b00000,5'b00100,5'b00100,5'b00000,5'b00100,5'b00100,5'b00000}; // :
          19:font={5'b00000,5'b00000,5'b00000,5'b11111,5'b00000,5'b00000,5'b00000}; // -
          20:font={5'b10001,5'b10001,5'b01010,5'b00100,5'b01010,5'b10001,5'b10001}; // X
          21:font={5'b10001,5'b10001,5'b01010,5'b00100,5'b00100,5'b00100,5'b00100}; // Y
          22:font={5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01010,5'b00100}; // V
          default:font=0;
        endcase end
    endfunction
    wire text_area=(SHOW_DEBUG!=0) && pixel_x>=48 && pixel_x<216 && pixel_y>=48 && pixel_y<64;
    wire [15:0] text_x=pixel_x-16'd48,text_y=pixel_y-16'd48;
    wire [15:0] char_index=text_x/12;
    wire [2:0] glyph_x=(text_x%12)>>1,glyph_y=text_y>>1;
    reg [4:0] character;
    always @* begin
        character=18;
        case(char_index)
          0:character=16;1:character=17;
          2:character=probe_show_valid?{1'b0,probe_show[23:20]}:5'd19;
          3:character=probe_show_valid?{1'b0,probe_show[19:16]}:5'd19;
          5:character=10;6:character=17;
          7:character=probe_show_valid?{1'b0,probe_show[15:12]}:5'd19;
          8:character=probe_show_valid?{1'b0,probe_show[11:8]}:5'd19;
          10:character=11;11:character=17;
          12:character=probe_show_valid?{1'b0,probe_show[7:4]}:5'd19;
          13:character=probe_show_valid?{1'b0,probe_show[3:0]}:5'd19;
          default:character=18;
        endcase
    end
    wire [34:0] glyph_bits=font(character);
    wire text_ink=(glyph_x<5 && glyph_y<7)?glyph_bits[34-glyph_y*5-glyph_x]:1'b0;
    // Two raw-coordinate rows: "0 V:1 X:1234 Y:5678" (hexadecimal).
    wire coord_area=(SHOW_DEBUG!=0) && pixel_x>=48 && pixel_x<276 &&
        ((pixel_y>=80 && pixel_y<96)||(pixel_y>=104 && pixel_y<120));
    wire coord_row=(pixel_y>=104);
    wire [15:0] coord_tx=pixel_x-16'd48;
    wire [15:0] coord_ty=pixel_y-(coord_row?16'd104:16'd80);
    wire [15:0] coord_index=coord_tx/12;
    wire [2:0] coord_gx=(coord_tx%12)>>1,coord_gy=coord_ty>>1;
    wire [31:0] coord_xy=coord_row?coord_show[63:32]:coord_show[31:0];
    reg [4:0] coord_character;
    always @* begin
        coord_character=18;
        case(coord_index)
          0:coord_character=coord_row?5'd1:5'd0;
          2:coord_character=22;3:coord_character=17;
          4:coord_character=coord_row?{4'd0,coord_show[65]}:{4'd0,coord_show[64]};
          6:coord_character=20;7:coord_character=17;
          8:coord_character={1'b0,coord_xy[15:12]};
          9:coord_character={1'b0,coord_xy[11:8]};
          10:coord_character={1'b0,coord_xy[7:4]};
          11:coord_character={1'b0,coord_xy[3:0]};
          13:coord_character=21;14:coord_character=17;
          15:coord_character={1'b0,coord_xy[31:28]};
          16:coord_character={1'b0,coord_xy[27:24]};
          17:coord_character={1'b0,coord_xy[23:20]};
          18:coord_character={1'b0,coord_xy[19:16]};
          default:coord_character=18;
        endcase
    end
    wire [34:0] coord_bits=font(coord_character);
    wire coord_ink=(coord_gx<5 && coord_gy<7)?coord_bits[34-coord_gy*5-coord_gx]:1'b0;
    // Four diagnostic rows; row number then eight space-separated hex bytes.
    wire diag_area=(SHOW_DEBUG!=0) && pixel_x>=48 && pixel_x<360 && pixel_y>=136 && pixel_y<264;
    wire [15:0] diag_tx=pixel_x-16'd48,diag_ty=pixel_y-16'd136;
    wire [2:0] diag_row=diag_ty[6:4];
    wire [15:0] diag_ci=diag_tx/12;
    wire [2:0] diag_gx=(diag_tx%12)>>1,diag_gy=diag_ty[3:1];
    wire [15:0] diag_offset=diag_ci-2;
    wire [15:0] diag_byte=diag_offset/3;
    wire [7:0] diag_value=(diag_row==7)?diagnostic_show[511-diag_byte*8 -: 8]:
        (diag_row==6)?diagnostic_show[447-diag_byte*8 -: 8]:
        (diag_row==5)?diagnostic_show[383-diag_byte*8 -: 8]:
        (diag_row==4)?diagnostic_show[319-diag_byte*8 -: 8]:
        diagnostic_show[255-(diag_row*8+diag_byte)*8 -: 8];
    wire [4:0] diag_char=(diag_ci==0)?{2'd0,diag_row}:
        ((diag_ci>=2 && diag_byte<8 && (diag_offset%3)<2)?
        ((diag_offset%3)==0?{1'b0,diag_value[7:4]}:{1'b0,diag_value[3:0]}):5'd18);
    wire [34:0] diag_bits=font(diag_char);
    wire diag_ink=(diag_gx<5 && diag_gy<7)?diag_bits[34-diag_gy*5-diag_gx]:1'b0;
    // "0 X:0639 Y:0359", 15 characters, each 12x16 pixels.
    function [15:0] label_left;
        input [15:0] center;
        begin
            if(center+CROSS_RADIUS+LABEL_GAP+LABEL_W<=SCREEN_W)
                label_left=center+CROSS_RADIUS+LABEL_GAP;
            else if(center>=LABEL_W+CROSS_RADIUS+LABEL_GAP)
                label_left=center-LABEL_W-CROSS_RADIUS-LABEL_GAP;
            else label_left=0;
        end
    endfunction
    function [15:0] label_above;
        input [15:0] center;
        begin
            if(center>=LABEL_H+LABEL_GAP)label_above=center-LABEL_H-LABEL_GAP;
            else if(center+CROSS_RADIUS+LABEL_GAP+LABEL_H<=SCREEN_H)
                label_above=center+CROSS_RADIUS+LABEL_GAP;
            else label_above=0;
        end
    endfunction
    function [15:0] label_below;
        input [15:0] center;
        begin
            if(center+CROSS_RADIUS+LABEL_GAP+LABEL_H<=SCREEN_H)
                label_below=center+CROSS_RADIUS+LABEL_GAP;
            else label_below=label_above(center);
        end
    endfunction
    wire [15:0] label_x0=label_left(sx0),label_y0=label_above(sy0);
    wire [15:0] label_x1=label_left(sx1),label_y1_base=label_below(sy1);
    wire labels_overlap=show0 && show1 && label_x0<label_x1+LABEL_W &&
        label_x1<label_x0+LABEL_W && label_y0<label_y1_base+LABEL_H &&
        label_y1_base<label_y0+LABEL_H;
    wire [15:0] label_y1=labels_overlap?
        ((label_y0+2*LABEL_H+LABEL_GAP<=SCREEN_H)?label_y0+LABEL_H+LABEL_GAP:
        ((label_y0>=LABEL_H+LABEL_GAP)?label_y0-LABEL_H-LABEL_GAP:label_y1_base)):
        label_y1_base;
    wire label_hit0=1'b0 && (SHOW_DEBUG==0) && LABELS_FIT && show0 &&
        pixel_x>=label_x0 && pixel_x<label_x0+LABEL_W &&
        pixel_y>=label_y0 && pixel_y<label_y0+LABEL_H;
    wire label_hit1=1'b0 && (SHOW_DEBUG==0) && LABELS_FIT && show1 &&
        pixel_x>=label_x1 && pixel_x<label_x1+LABEL_W &&
        pixel_y>=label_y1 && pixel_y<label_y1+LABEL_H;
    wire label_channel=!label_hit0;
    wire [15:0] label_tx=pixel_x-(label_channel?label_x1:label_x0);
    wire [15:0] label_ty=pixel_y-(label_channel?label_y1:label_y0);
    wire [15:0] label_index=label_tx/12;
    wire [2:0] label_gx=(label_tx%12)>>1,label_gy=label_ty[3:1];
    wire [31:0] label_digits=label_channel?digits_show1:digits_show0;
    reg [4:0] label_character;
    always @* begin
        label_character=18;
        case(label_index)
            0:label_character=label_channel?5'd1:5'd0;
            2:label_character=20;
            3:label_character=17;
            4:label_character={1'b0,label_digits[31:28]};
            5:label_character={1'b0,label_digits[27:24]};
            6:label_character={1'b0,label_digits[23:20]};
            7:label_character={1'b0,label_digits[19:16]};
            9:label_character=21;
            10:label_character=17;
            11:label_character={1'b0,label_digits[15:12]};
            12:label_character={1'b0,label_digits[11:8]};
            13:label_character={1'b0,label_digits[7:4]};
            14:label_character={1'b0,label_digits[3:0]};
        endcase
    end
    wire [34:0] label_bits=font(label_character);
    wire label_ink=(label_gx<5 && label_gy<7)?label_bits[34-label_gy*5-label_gx]:1'b0;
    assign menu_valid=pixel_de && pixel_x<SCREEN_W && pixel_y<SCREEN_H &&
        (status_hit || probe_hit || text_area || coord_area || diag_area || hit0 || hit1 || label_hit0 || label_hit1);
    assign menu_rgb=status_hit?(chip_show?24'h00ff00:24'hff0000):
        probe_hit?(!probe_show_valid?24'h808080:(probe_show[24]?24'h00ff00:24'hff0000)):
        text_area?(text_ink?24'hffffff:24'h000000):
        diag_area?(diag_ink?24'hffff00:24'h000000):
        coord_area?(coord_ink?(coord_row?COLOR1:COLOR0):24'h000000):
        hit0?(ripple_age[1:0]==0?24'h00e5ff:(ripple_age[1:0]==1?24'hb050ff:24'h45ff92)):
        hit1?(ripple_age[1:0]==0?24'hff5ac8:(ripple_age[1:0]==1?24'hffd34a:24'h63a4ff)):
        (label_ink?(label_channel?COLOR1:COLOR0):24'h000000);
endmodule
