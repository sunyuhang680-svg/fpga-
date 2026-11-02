`timescale 1ns/1ps
// Six signed raw words, hexadecimal words and initialization diagnostics.
// A complete sys_clk tuple crosses via handshake, converts in 16 pixel clocks,
// then commits at VS only. RGB/DE/HS/VS all have one pixel-clock latency.
module icm42688_video_overlay #(
    parameter integer PANEL_X=24,
    parameter integer PANEL_Y=64,
    parameter integer SCOPE_TEST=0,
    parameter integer ANGLE_MODE=0
)(
    input wire sys_clk, pixel_clk, rst_n,
    input wire data_valid, init_done, error,
    input wire [3:0] error_code,
    input wire [7:0] who_am_i,
    input wire [31:0] sample_count,
    input wire signed [15:0] gyro_x_raw,gyro_y_raw,gyro_z_raw,
    input wire signed [15:0] accel_x_raw,accel_y_raw,accel_z_raw,
    input wire [23:0] camera_rgb,
    input wire camera_de,camera_hs,camera_vs,
    output reg [23:0] video_rgb,
    output reg video_de,video_hs,video_vs
);
    reg have_sample;
    reg [13:0] previous_status;
    wire [13:0] status_now={init_done,error,error_code,who_am_i};
    wire source_event=data_valid || status_now!=previous_status;
    always @(posedge sys_clk or negedge rst_n) begin
        if(!rst_n) begin have_sample<=0;previous_status<=0;end
        else begin
            previous_status<=status_now;
            if(data_valid) have_sample<=1;
        end
    end
    wire pixel_event;
    wire [142:0] pixel_data;
    touch_event_cdc #(.WIDTH(143)) u_imu_cdc (
        .src_clk(sys_clk),.dst_clk(pixel_clk),.rst_n(rst_n),
        .src_event(source_event),
        .src_data({have_sample || data_valid,init_done,error,error_code,
                   who_am_i,sample_count,gyro_x_raw,gyro_y_raw,gyro_z_raw,
                   accel_x_raw,accel_y_raw,accel_z_raw}),
        .dst_event(pixel_event),.dst_data(pixel_data)
    );

    reg [15:0] pixel_x,pixel_y;
    reg de_previous,vs_previous;
    wire frame_start=camera_vs && !vs_previous;
    reg [142:0] pending_tuple,shown_tuple;
    reg [15:0] magnitude_shift[0:5];
    reg [19:0] bcd_work[0:5],bcd_pending[0:5],bcd_shown[0:5];
    reg convert_busy,pending_ready;
    reg [3:0] convert_bit;
    integer axis;

    function [15:0] abs16;
        input [15:0] value;
        begin abs16=value[15] ? (~value+16'd1) : value;end
    endfunction
    function [19:0] bcd_step;
        input [19:0] previous;
        input next_bit;
        integer n;
        reg [19:0] adjusted;
        begin
            adjusted=previous;
            for(n=0;n<5;n=n+1)
                if(previous[n*4 +: 4]>=5)
                    adjusted[n*4 +: 4]=previous[n*4 +: 4]+4'd3;
            bcd_step={adjusted[18:0],next_bit};
        end
    endfunction
    always @(posedge pixel_clk or negedge rst_n) begin
        if(!rst_n) begin
            pixel_x<=0;pixel_y<=0;de_previous<=0;vs_previous<=0;
            pending_tuple<=0;shown_tuple<=0;convert_busy<=0;
            pending_ready<=0;convert_bit<=0;
            for(axis=0;axis<6;axis=axis+1) begin
                magnitude_shift[axis]<=0;bcd_work[axis]<=0;
                bcd_pending[axis]<=0;bcd_shown[axis]<=0;
            end
        end else begin
            de_previous<=camera_de;vs_previous<=camera_vs;
            if(frame_start) begin pixel_x<=0;pixel_y<=0;end
            else begin
                if(camera_de) pixel_x<=pixel_x+1'b1;else pixel_x<=0;
                if(!camera_de && de_previous) pixel_y<=pixel_y+1'b1;
            end
            if(pixel_event) begin
                pending_tuple<=pixel_data;convert_busy<=1;
                pending_ready<=0;convert_bit<=0;
                for(axis=0;axis<6;axis=axis+1) begin
                    magnitude_shift[axis]<=abs16(pixel_data[95-axis*16 -: 16]);
                    bcd_work[axis]<=0;
                end
            end else if(convert_busy) begin
                for(axis=0;axis<6;axis=axis+1) begin
                    bcd_work[axis]<=bcd_step(bcd_work[axis],magnitude_shift[axis][15]);
                    magnitude_shift[axis]<=magnitude_shift[axis]<<1;
                    if(convert_bit==15)
                        bcd_pending[axis]<=bcd_step(bcd_work[axis],magnitude_shift[axis][15]);
                end
                if(convert_bit==15) begin convert_busy<=0;pending_ready<=1;end
                else convert_bit<=convert_bit+1'b1;
            end
            if(frame_start && pending_ready && !convert_busy && !pixel_event) begin
                shown_tuple<=pending_tuple;
                for(axis=0;axis<6;axis=axis+1) bcd_shown[axis]<=bcd_pending[axis];
            end
        end
    end

    function [7:0] hex_ascii;
        input [3:0] value;
        begin hex_ascii=value<10 ? 8'd48+value : 8'd55+value;end
    endfunction
    // 5x7 font, two pixels per font dot; 12-pixel character advance.
    function [34:0] font;
        input [7:0] ch;
        begin
            case(ch)
                "0":font={5'b01110,5'b10001,5'b10011,5'b10101,5'b11001,5'b10001,5'b01110};
                "1":font={5'b00100,5'b01100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
                "2":font={5'b01110,5'b10001,5'b00001,5'b00010,5'b00100,5'b01000,5'b11111};
                "3":font={5'b11110,5'b00001,5'b00001,5'b01110,5'b00001,5'b00001,5'b11110};
                "4":font={5'b00010,5'b00110,5'b01010,5'b10010,5'b11111,5'b00010,5'b00010};
                "5":font={5'b11111,5'b10000,5'b10000,5'b11110,5'b00001,5'b00001,5'b11110};
                "6":font={5'b01110,5'b10000,5'b10000,5'b11110,5'b10001,5'b10001,5'b01110};
                "7":font={5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b01000,5'b01000};
                "8":font={5'b01110,5'b10001,5'b10001,5'b01110,5'b10001,5'b10001,5'b01110};
                "9":font={5'b01110,5'b10001,5'b10001,5'b01111,5'b00001,5'b00001,5'b01110};
                "A":font={5'b01110,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
                "B":font={5'b11110,5'b10001,5'b10001,5'b11110,5'b10001,5'b10001,5'b11110};
                "C":font={5'b01111,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b01111};
                "D":font={5'b11110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b11110};
                "E":font={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b11111};
                "F":font={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b10000};
                "G":font={5'b01111,5'b10000,5'b10000,5'b10111,5'b10001,5'b10001,5'b01111};
                "H":font={5'b10001,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
                "I":font={5'b01110,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
                "J":font={5'b00111,5'b00010,5'b00010,5'b00010,5'b10010,5'b10010,5'b01100};
                "L":font={5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b11111};
                ".":font={5'b00000,5'b00000,5'b00000,5'b00000,5'b00000,5'b00100,5'b00100};
                "M":font={5'b10001,5'b11011,5'b10101,5'b10101,5'b10001,5'b10001,5'b10001};
                "N":font={5'b10001,5'b11001,5'b10101,5'b10011,5'b10001,5'b10001,5'b10001};
                "O":font={5'b01110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
                "P":font={5'b11110,5'b10001,5'b10001,5'b11110,5'b10000,5'b10000,5'b10000};
                "S":font={5'b01111,5'b10000,5'b10000,5'b01110,5'b00001,5'b00001,5'b11110};
                "R":font={5'b11110,5'b10001,5'b10001,5'b11110,5'b10100,5'b10010,5'b10001};
                "T":font={5'b11111,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100};
                "U":font={5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
                "W":font={5'b10001,5'b10001,5'b10001,5'b10101,5'b10101,5'b11011,5'b10001};
                "X":font={5'b10001,5'b10001,5'b01010,5'b00100,5'b01010,5'b10001,5'b10001};
                "Y":font={5'b10001,5'b10001,5'b01010,5'b00100,5'b00100,5'b00100,5'b00100};
                "Z":font={5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b10000,5'b11111};
                "+":font={5'b00000,5'b00100,5'b00100,5'b11111,5'b00100,5'b00100,5'b00000};
                "-":font={5'b00000,5'b00000,5'b00000,5'b11111,5'b00000,5'b00000,5'b00000};
                ":":font={5'b00000,5'b00100,5'b00100,5'b00000,5'b00100,5'b00100,5'b00000};
                default:font=0;
            endcase
        end
    endfunction

    localparam [8*16-1:0] TITLE=SCOPE_TEST ? "PIN TEST J26-10 " : (ANGLE_MODE ? "IMU ANGLES      " : "ICM42688 J26-10 ");
    reg [7:0] character;
    reg [15:0] raw_word;
    reg [19:0] decimal_word;
    reg [34:0] glyph;
    reg panel_hit,text_hit;
    reg [23:0] text_color;
    integer local_x,local_y,text_row,text_col,dot_x,dot_y,axis_index;
    integer row_index,column_index;
    always @* begin
        panel_hit=camera_de && pixel_x>=PANEL_X && pixel_x<PANEL_X+272 &&
                  pixel_y>=PANEL_Y && pixel_y<PANEL_Y+236;
        text_hit=0;character=" ";raw_word=0;decimal_word=0;
        local_x=0;local_y=0;text_row=0;text_col=0;dot_x=0;dot_y=0;axis_index=0;
        row_index=0;column_index=0;
        text_color=24'hffffff;glyph=0;
        if(panel_hit && pixel_x>=PANEL_X+12 && pixel_y>=PANEL_Y+12) begin
            local_x=pixel_x-PANEL_X-12;local_y=pixel_y-PANEL_Y-12;
            // Fixed-cell comparisons avoid signed divide/modulo primitives
            // unsupported by PDS ADS. Every bound is an elaboration constant.
            text_row=15;text_col=20;
            for(row_index=0;row_index<9;row_index=row_index+1)
                if(local_y>=row_index*24 && local_y<row_index*24+24) begin
                    text_row=row_index;dot_y=(local_y-row_index*24)>>1;
                end
            for(column_index=0;column_index<20;column_index=column_index+1)
                if(local_x>=column_index*12 && local_x<column_index*12+12) begin
                    text_col=column_index;dot_x=(local_x-column_index*12)>>1;
                end
            if(text_row==0 && text_col<16)
                character=TITLE[(15-text_col)*8 +: 8];
            else if(text_row==1) begin
                case(text_col)
                    0:character=shown_tuple[140]?"E":(shown_tuple[141]?"R":(ANGLE_MODE?"C":"I"));
                    1:character=shown_tuple[140]?"R":(shown_tuple[141]?"E":(ANGLE_MODE?"A":"N"));
                    2:character=shown_tuple[140]?"R":(shown_tuple[141]?"A":(ANGLE_MODE?"L":"I"));
                    3:character=shown_tuple[140]?"O":(shown_tuple[141]?"D":(ANGLE_MODE?"I":"T"));
                    4:character=shown_tuple[140]?"R":(shown_tuple[141]?"Y":(ANGLE_MODE?"B":" "));
                    6:character="I";7:character="D";8:character=":";
                    9:character=hex_ascii(shown_tuple[135:132]);
                    10:character=hex_ascii(shown_tuple[131:128]);
                    12:character="E";13:character="R";14:character="R";
                    15:character=":";16:character=hex_ascii(shown_tuple[139:136]);
                    default:character=" ";
                endcase
                text_color=shown_tuple[140]?24'hff6060:(shown_tuple[141]?24'h70ff90:24'hffd060);
            end else if(text_row>=2 && text_row<=7) begin
                axis_index=text_row-2;
                raw_word=shown_tuple[95-axis_index*16 -: 16];
                decimal_word=bcd_shown[axis_index];
                text_color=axis_index<3 ? 24'h80e8ff : 24'hb0ff90;
                if(ANGLE_MODE && axis_index<3) begin
                    case(text_col)
                        0:character=(axis_index==0)?"R":((axis_index==1)?"P":"Y");
                        1:character=(axis_index==0)?"O":((axis_index==1)?"I":"A");
                        2:character=(axis_index==0)?"L":((axis_index==1)?"T":"W");
                        3:character=(axis_index==0)?"L":((axis_index==1)?"C":" ");
                        4:character=(axis_index==1)?"H":" ";
                        6:character=shown_tuple[142]?(raw_word[15]?"-":"+"):"-";
                        7,8,9:character=shown_tuple[142]?hex_ascii(decimal_word[(11-text_col)*4 +: 4]):"-";
                        10:character=".";
                        11,12:character=shown_tuple[142]?hex_ascii(decimal_word[(12-text_col)*4 +: 4]):"-";
                        14:character="D";15:character="E";16:character="G";
                        default:character=" ";
                    endcase
                end else begin
                case(text_col)
                    0:character=ANGLE_MODE ? "G" : (axis_index<3 ? "G":"A");
                    1:case(axis_index)
                        0,3:character="X";1,4:character="Y";default:character="Z";
                      endcase
                    2:character=":";
                    4:character=shown_tuple[142]?(raw_word[15]?"-":"+"):"-";
                    5,6,7,8,9:character=shown_tuple[142] ?
                        hex_ascii(decimal_word[(9-text_col)*4 +: 4]) : "-";
                    11:character="H";12:character=":";
                    13,14,15,16:character=shown_tuple[142] ?
                        hex_ascii(raw_word[(16-text_col)*4 +: 4]) : "-";
                    default:character=" ";
                endcase
                end
            end else if(text_row==8) begin
                case(text_col)
                    0:character="C";1:character="O";2:character="U";
                    3:character="N";4:character="T";5:character=":";
                    6,7,8,9,10,11,12,13:character=hex_ascii(shown_tuple[(13-text_col)*4+96 +: 4]);
                    default:character=" ";
                endcase
            end
            glyph=font(character);
            if(text_col<20 && dot_x<5 && dot_y<7 && text_row<=8)
                text_hit=glyph[34-dot_y*5-dot_x];
        end
    end
    always @(posedge pixel_clk or negedge rst_n) begin
        if(!rst_n) begin video_rgb<=0;video_de<=0;video_hs<=0;video_vs<=0;end
        else begin
            video_de<=camera_de;video_hs<=camera_hs;video_vs<=camera_vs;
            if(!camera_de) video_rgb<=0;
            else if(text_hit) video_rgb<=text_color;
            else if(panel_hit)
                video_rgb<={2'b00,camera_rgb[23:18],2'b00,camera_rgb[15:10],2'b00,camera_rgb[7:2]};
            else video_rgb<=camera_rgb;
        end
    end
endmodule
