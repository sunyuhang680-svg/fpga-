`timescale 1ns/1ps
// Pixel-plane UI for the first interaction stage.  The two DDR readers keep
// their normal stereo/compare assignments; double-tap state is intentionally
// ignored here so it cannot change the video sampling path.
module stereo_ui_layer #(
    // Flash duration is measured in pixel clocks so "one instant" stays the
    // same regardless of the video timing.
    parameter integer PIXEL_CLK_HZ = 74250000,
    parameter integer FLASH_MS = 150,
    parameter integer ENABLE_CMP_CONTROLS = 0
)(
    input wire pixel_clk, input wire rst_n,
    input wire [23:0] camera_rgb, input wire camera_de, camera_hs, camera_vs,
    input wire image_mode, input wire menu_visible, input wire [1:0] layout,
    input wire [1:0] feature, input wire zoom_active, input wire [1:0] zoom_source,
    input wire [1:0] cmp_undistort, input wire [1:0] cmp_binary,
    input wire cmp_binary_active,
    // Image post-processing page: eight tools, one target half, and a single
    // "flash" pulse (press_code/press_toggle) per key press.
    input wire post_mode, input wire [2:0] post_tool, input wire post_page,
    input wire [2:0] press_code, input wire press_toggle,
    // Sub page of the selected tool: palette colour or slider value.
    input wire [2:0] sub_color, input wire [7:0] knob_value,
    output reg [23:0] video_rgb, output reg video_de, video_hs, video_vs
);
    localparam [27:0] FLASH_CYCLES=(PIXEL_CLK_HZ/1000)*FLASH_MS;
    localparam [2:0] PRESS_RESET=3'd1,PRESS_EDIT=3'd2,PRESS_TOOL=3'd3,PRESS_PAGE=3'd4,
                     PRESS_COLOR=3'd5,PRESS_CLEAR=3'd6;
    localparam [1:0] LAYOUT_STEREO=2'd0, LAYOUT_LEFT_COMPARE=2'd1,
                     LAYOUT_RIGHT_COMPARE=2'd2;
    reg de_d0,de_d1,hs_d0,hs_d1,vs_d0,vs_d1;
    reg [23:0] rgb_d0,rgb_d1;
    // x/y count the incoming stream.  The UI uses x_d1/y_d1 because RGB and
    // DE are both deliberately delayed by two cycles before HDMI output.
    reg [11:0] x,y,x_d0,x_d1,y_d0,y_d1;
    reg prev_de,prev_vs;
    wire vs_fall=prev_vs && !camera_vs;
    wire line_end=prev_de && !camera_de;
    wire right_half=(x_d1>=12'd640);
    wire [7:0] luminance={2'b0,rgb_d1[23:18]} + {1'b0,rgb_d1[15:9]} + {2'b0,rgb_d1[7:2]};
    wire [23:0] binary_rgb=(luminance>=8'd96)?24'hffffff:24'h000000;
    wire compare_binary=ENABLE_CMP_CONTROLS ? (cmp_binary_active && right_half) : ((layout==LAYOUT_LEFT_COMPARE || layout==LAYOUT_RIGHT_COMPARE) && right_half);
    // A permanent navigation panel sits slightly below the top edge.  The
    // three large hit blocks match the touch regions in stereo_ui_controller.
    wire menu_panel=menu_visible &&
        (x_d1>=12'd128 && x_d1<12'd1152 && y_d1>=12'd32 && y_d1<12'd94);
    wire menu_y_hit=(y_d1>=12'd36 && y_d1<12'd90);
    wire tab0_hit=menu_y_hit && x_d1>=12'd144 && x_d1<12'd464;
    wire tab1_hit=menu_y_hit && x_d1>=12'd480 && x_d1<12'd800;
    wire tab2_hit=menu_y_hit && x_d1>=12'd816 && x_d1<12'd1136;
    wire tab_hit=tab0_hit || tab1_hit || tab2_hit;
    wire [1:0] tab=tab0_hit ? 2'd0 : tab1_hit ? 2'd1 : 2'd2;
    wire selected=(layout==tab);
    wire [23:0] tab_color=selected ? 24'h1473e6 : 24'h8e52d1;
    // Compact mode badge above the navigation row. A brighter purple keeps
    // the live/frozen state consistent with the three navigation tabs.
    wire mode_badge=menu_visible && x_d1>=12'd576 && x_d1<12'd704 &&
                    y_d1>=12'd4 && y_d1<12'd28;
    wire mode_badge_border=mode_badge &&
        (x_d1<12'd580 || x_d1>=12'd700 || y_d1<12'd8 || y_d1>=12'd24);
    wire reset_button=menu_visible && x_d1>=12'd16 && x_d1<12'd112 &&
                      y_d1>=12'd16 && y_d1<12'd66;
    wire reset_border=reset_button &&
        (x_d1<12'd20 || x_d1>=12'd108 || y_d1<12'd20 || y_d1>=12'd62);
    // Four compact algorithm selectors sit at the bottom so the camera image
    // remains unobstructed through the middle of the screen.
    wire cmp_ui_mode=ENABLE_CMP_CONTROLS && (layout==LAYOUT_LEFT_COMPARE || layout==LAYOUT_RIGHT_COMPARE);
    wire feature_panel=menu_visible && x_d1>=12'd274 && x_d1<12'd1006 &&
                       y_d1>=12'd670 && y_d1<12'd718;
    wire feature0_hit=x_d1>=12'd286 && x_d1<12'd454 && y_d1>=12'd676 && y_d1<12'd712;
    wire feature1_hit=x_d1>=12'd466 && x_d1<12'd634 && y_d1>=12'd676 && y_d1<12'd712;
    wire feature2_hit=x_d1>=12'd646 && x_d1<12'd814 && y_d1>=12'd676 && y_d1<12'd712;
    wire feature3_hit=x_d1>=12'd826 && x_d1<12'd994 && y_d1>=12'd676 && y_d1<12'd712;
    wire feature_hit=feature0_hit || feature1_hit || feature2_hit || feature3_hit;
    wire [1:0] feature_index=feature0_hit ? 2'd0 : feature1_hit ? 2'd1 :
                             feature2_hit ? 2'd2 : 2'd3;
    wire feature_selected=(feature==feature_index);
    wire feature_border=feature_hit &&
        ((x_d1>=12'd286 && x_d1<12'd290) || (x_d1>=12'd450 && x_d1<12'd454) ||
         (x_d1>=12'd466 && x_d1<12'd470) || (x_d1>=12'd630 && x_d1<12'd634) ||
         (x_d1>=12'd646 && x_d1<12'd650) || (x_d1>=12'd810 && x_d1<12'd814) ||
         (x_d1>=12'd826 && x_d1<12'd830) || (x_d1>=12'd990 && x_d1<12'd994) ||
         y_d1<12'd680 || y_d1>=12'd708);
    function [34:0] glyph;
        input [7:0] c;
        begin case(c)
          "A":glyph={5'b01110,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
          "B":glyph={5'b11110,5'b10001,5'b10001,5'b11110,5'b10001,5'b10001,5'b11110};
          "C":glyph={5'b01111,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b01111};
          "D":glyph={5'b11110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b11110};
          "E":glyph={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b11111};
          "F":glyph={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b10000};
          "G":glyph={5'b01111,5'b10000,5'b10000,5'b10111,5'b10001,5'b10001,5'b01110};
          "H":glyph={5'b10001,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
          "I":glyph={5'b01110,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
          "L":glyph={5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b11111};
          "M":glyph={5'b10001,5'b11011,5'b10101,5'b10101,5'b10001,5'b10001,5'b10001};
          "N":glyph={5'b10001,5'b11001,5'b10101,5'b10011,5'b10001,5'b10001,5'b10001};
          "O":glyph={5'b01110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
          "P":glyph={5'b11110,5'b10001,5'b10001,5'b11110,5'b10000,5'b10000,5'b10000};
          "R":glyph={5'b11110,5'b10001,5'b10001,5'b11110,5'b10100,5'b10010,5'b10001};
          "S":glyph={5'b01111,5'b10000,5'b10000,5'b01110,5'b00001,5'b00001,5'b11110};
          "T":glyph={5'b11111,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100};
          "V":glyph={5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01010,5'b00100};
          "W":glyph={5'b10001,5'b10001,5'b10001,5'b10101,5'b10101,5'b10101,5'b01010};
          "X":glyph={5'b10001,5'b10001,5'b01010,5'b00100,5'b01010,5'b10001,5'b10001};
          "U":glyph={5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
          "K":glyph={5'b10001,5'b10010,5'b10100,5'b11000,5'b10100,5'b10010,5'b10001};
          "J":glyph={5'b00111,5'b00010,5'b00010,5'b00010,5'b00010,5'b10010,5'b01100};
          "Q":glyph={5'b01110,5'b10001,5'b10001,5'b10001,5'b10101,5'b10010,5'b01101};
          "Y":glyph={5'b10001,5'b10001,5'b01010,5'b00100,5'b00100,5'b00100,5'b00100};
          "Z":glyph={5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b10000,5'b11111};
          " ":glyph=0;
          default:glyph=0;
        endcase end
    endfunction
    function [7:0] feature_char;
        input [1:0] item; input [3:0] index;
        begin
          case(item)
            0:case(index) 0:feature_char="D";1:feature_char="I";2:feature_char="S";3:feature_char="T";4:feature_char="O";5:feature_char="R";6:feature_char="T";default:feature_char=" ";endcase
            1:case(index) 0:feature_char="R";1:feature_char="O";2:feature_char="T";3:feature_char="A";4:feature_char="T";5:feature_char="E";default:feature_char=" ";endcase
            2:case(index) 0:feature_char="D";1:feature_char="I";2:feature_char="S";3:feature_char="T";4:feature_char="A";5:feature_char="N";6:feature_char="C";7:feature_char="E";default:feature_char=" ";endcase
            default:case(index) 0:feature_char="L";1:feature_char="I";2:feature_char="G";3:feature_char="H";4:feature_char="T";default:feature_char=" ";endcase
          endcase
        end
    endfunction
    function [7:0] mode_char;
        input frozen; input [2:0] index;
        begin
          if(frozen) begin
            case(index)
              0:mode_char="I";1:mode_char="M";2:mode_char="A";
              3:mode_char="G";4:mode_char="E";default:mode_char=" ";
            endcase
          end else begin
            case(index)
              0:mode_char="V";1:mode_char="I";2:mode_char="D";
              3:mode_char="E";4:mode_char="O";default:mode_char=" ";
            endcase
          end
        end
    endfunction
    function [7:0] menu_char;
        input [1:0] row; input [3:0] index;
        begin
          case(row)
            0:case(index) 0:menu_char="S";1:menu_char="T";2:menu_char="E";3:menu_char="R";4:menu_char="E";5:menu_char="O";default:menu_char=" ";endcase
            1:case(index) 0:menu_char="L";1:menu_char="E";2:menu_char="F";3:menu_char="T";5:menu_char="C";6:menu_char="M";7:menu_char="P";default:menu_char=" ";endcase
            default:case(index) 0:menu_char="R";1:menu_char="I";2:menu_char="G";3:menu_char="H";4:menu_char="T";6:menu_char="C";7:menu_char="M";8:menu_char="P";default:menu_char=" ";endcase
          endcase
        end
    endfunction
    // Eight post-processing tools, in screen order. Labels are kept at eight
    // characters or fewer so a 2x glyph string fits a 112-pixel cell.
    function [7:0] tool_char;
        input [2:0] tool; input [3:0] index;
        begin
          case(tool)
            0:case(index) 0:tool_char="D";1:tool_char="O";2:tool_char="O";3:tool_char="D";4:tool_char="L";5:tool_char="E";default:tool_char=" ";endcase
            1:case(index) 0:tool_char="E";1:tool_char="R";2:tool_char="A";3:tool_char="S";4:tool_char="E";5:tool_char="R";default:tool_char=" ";endcase
            2:case(index) 0:tool_char="E";1:tool_char="X";2:tool_char="P";3:tool_char="O";4:tool_char="S";5:tool_char="U";6:tool_char="R";7:tool_char="E";default:tool_char=" ";endcase
            3:case(index) 0:tool_char="C";1:tool_char="O";2:tool_char="N";3:tool_char="T";4:tool_char="R";5:tool_char="A";6:tool_char="S";7:tool_char="T";default:tool_char=" ";endcase
            4:case(index) 0:tool_char="S";1:tool_char="H";2:tool_char="A";3:tool_char="R";4:tool_char="P";5:tool_char="E";6:tool_char="N";default:tool_char=" ";endcase
            5:case(index) 0:tool_char="B";1:tool_char="R";2:tool_char="I";3:tool_char="G";4:tool_char="H";5:tool_char="T";default:tool_char=" ";endcase
            6:case(index) 0:tool_char="S";1:tool_char="A";2:tool_char="T";3:tool_char="U";4:tool_char="R";default:tool_char=" ";endcase
            default:case(index) 0:tool_char="T";1:tool_char="E";2:tool_char="M";3:tool_char="P";default:tool_char=" ";endcase
          endcase
        end
    endfunction
    // Half-width of the 16-pixel knob for each vertical offset, so the round
    // shape is a table lookup instead of two multipliers in the pixel clock.
    function [4:0] circle_span;
        input [11:0] dy;
        begin
          case(dy)
            12'd0:circle_span=5'd16; 12'd1:circle_span=5'd15;
            12'd2:circle_span=5'd15; 12'd3:circle_span=5'd15;
            12'd4:circle_span=5'd15; 12'd5:circle_span=5'd15;
            12'd6:circle_span=5'd14; 12'd7:circle_span=5'd14;
            12'd8:circle_span=5'd13; 12'd9:circle_span=5'd13;
            12'd10:circle_span=5'd12; 12'd11:circle_span=5'd11;
            12'd12:circle_span=5'd10; 12'd13:circle_span=5'd9;
            12'd14:circle_span=5'd7; 12'd15:circle_span=5'd5;
            default:circle_span=5'd0;
          endcase
        end
    endfunction
    function [23:0] palette;
        input [2:0] index;
        begin
          case(index)
            3'd0:palette=24'hffffff; 3'd1:palette=24'hf8483c;
            3'd2:palette=24'h48e05c; 3'd3:palette=24'h4878f8;
            3'd4:palette=24'hf8e04c; 3'd5:palette=24'h48e8e8;
            3'd6:palette=24'hf848e0; default:palette=24'hf89840;
          endcase
        end
    endfunction
    // 2x glyphs are clearly larger than the original 1x labels while keeping
    // the whole navigation panel only 62 pixels high.
    wire [11:0] menu_tx=tab0_hit ? x_d1-12'd268 :
                         tab1_hit ? x_d1-12'd592 : x_d1-12'd922;
    wire [3:0] menu_ci=menu_tx/12;
    wire [3:0] menu_cell_x=menu_tx%12;
    wire [2:0] menu_gx=menu_cell_x>>1;
    wire [3:0] menu_cell_y=y_d1-12'd56;
    wire [2:0] menu_gy=menu_cell_y>>1;
    wire [34:0] menu_bits=glyph(menu_char(tab,menu_ci));
    wire menu_text_x=(tab0_hit && x_d1>=12'd268 && x_d1<12'd340) ||
                      (tab1_hit && x_d1>=12'd592 && x_d1<12'd688) ||
                      (tab2_hit && x_d1>=12'd922 && x_d1<12'd1030);
    wire menu_text_y=(y_d1>=12'd56 && y_d1<12'd70);
    wire menu_ink=(menu_text_x && menu_text_y && menu_cell_x<10 && menu_gx<5 && menu_gy<7)?
        menu_bits[34-menu_gy*5-menu_gx]:1'b0;
    wire mode_text_area=mode_badge && x_d1>=12'd610 && x_d1<12'd670 &&
                        y_d1>=12'd9 && y_d1<12'd23;
    wire [6:0] mode_tx=x_d1-12'd610;
    wire [2:0] mode_ci=mode_tx/12;
    wire [3:0] mode_cell_x=mode_tx%12;
    wire [2:0] mode_gx=mode_cell_x>>1;
    wire [2:0] mode_gy=(y_d1-12'd9)>>1;
    wire [34:0] mode_bits=glyph(mode_char(image_mode,mode_ci));
    wire mode_ink=(mode_text_area && mode_cell_x<10 && mode_gx<5 && mode_gy<7)?
        mode_bits[34-mode_gy*5-mode_gx]:1'b0;
    wire cmp_text_undist=cmp_ui_mode && x_d1>=12'd382 && x_d1<12'd538 && y_d1>=12'd687 && y_d1<12'd701;
    wire cmp_text_binary=cmp_ui_mode && x_d1>=12'd760 && x_d1<12'd880 && y_d1>=12'd687 && y_d1<12'd701;
    wire cmp_text_area=cmp_text_undist || cmp_text_binary;
    wire [3:0] cmp_text_index=cmp_text_undist ? ((x_d1-12'd382)/12) : ((x_d1-12'd760)/12);
    wire [3:0] cmp_cell_x=cmp_text_undist ? ((x_d1-12'd382)%12) : ((x_d1-12'd760)%12);
    wire [2:0] cmp_gx=cmp_cell_x>>1;
    wire [2:0] cmp_gy=(y_d1-12'd687)>>1;
    function [7:0] cmp_control_char;
        input which; input enabled; input [3:0] index;
        begin
            cmp_control_char=" ";
            if(!which) begin
                case(index)
                    0:cmp_control_char="U";1:cmp_control_char="N";2:cmp_control_char="D";
                    3:cmp_control_char="I";4:cmp_control_char="S";5:cmp_control_char="T";
                    6:cmp_control_char="O";7:cmp_control_char="R";8:cmp_control_char="T";
                    10:cmp_control_char="O";
                    11:cmp_control_char=enabled?"N":"F";
                    12:if(!enabled)cmp_control_char="F";
                    default:cmp_control_char=" ";
                endcase
            end else begin
                case(index)
                    0:cmp_control_char="B";1:cmp_control_char="I";2:cmp_control_char="N";
                    3:cmp_control_char="A";4:cmp_control_char="R";5:cmp_control_char="Y";
                    7:cmp_control_char="O";
                    8:cmp_control_char=enabled?"N":"F";
                    9:if(!enabled)cmp_control_char="F";
                    default:cmp_control_char=" ";
                endcase
            end
        end
    endfunction
    wire [34:0] cmp_text_bits=glyph(cmp_control_char(cmp_text_binary,
        cmp_text_binary ? cmp_binary[layout==LAYOUT_RIGHT_COMPARE] : cmp_undistort[layout==LAYOUT_RIGHT_COMPARE],cmp_text_index));
    wire cmp_text_ink=cmp_text_area && cmp_cell_x<10 && cmp_gx<5 && cmp_gy<7 ?
        cmp_text_bits[34-cmp_gy*5-cmp_gx] : 1'b0;
    wire [11:0] feature_text_left=feature0_hit ? 12'd328 : feature1_hit ? 12'd514 :
                                  feature2_hit ? 12'd682 : 12'd880;
    wire [11:0] feature_text_right=feature0_hit ? 12'd412 : feature1_hit ? 12'd586 :
                                   feature2_hit ? 12'd778 : 12'd940;
    wire feature_text_area=feature_hit && x_d1>=feature_text_left &&
                           x_d1<feature_text_right && y_d1>=12'd687 && y_d1<12'd701;
    wire [11:0] feature_tx=x_d1-feature_text_left;
    wire [3:0] feature_ci=feature_tx/12;
    wire [3:0] feature_cell_x=feature_tx%12;
    wire [2:0] feature_gx=feature_cell_x>>1;
    wire [2:0] feature_gy=(y_d1-12'd687)>>1;
    wire [34:0] feature_bits=glyph(feature_char(feature_index,feature_ci));
    wire feature_ink=(feature_text_area && feature_cell_x<10 && feature_gx<5 && feature_gy<7)?
        feature_bits[34-feature_gy*5-feature_gx]:1'b0;
    wire reset_text_area=reset_button && x_d1>=12'd34 && x_d1<12'd94 &&
                         y_d1>=12'd34 && y_d1<12'd48;
    wire [5:0] reset_tx=x_d1-12'd34;
    wire [2:0] reset_ci=reset_tx/12;
    wire [3:0] reset_cell_x=reset_tx%12;
    wire [2:0] reset_gx=reset_cell_x>>1;
    wire [2:0] reset_gy=(y_d1-12'd34)>>1;
    wire [7:0] reset_char=reset_ci==0?"R":reset_ci==1?"E":reset_ci==2?"S":
        reset_ci==3?"E":reset_ci==4?"T":" ";
    wire [34:0] reset_bits=glyph(reset_char);
    wire reset_ink=(reset_text_area && reset_cell_x<10 && reset_gx<5 && reset_gy<7)?
        reset_bits[34-reset_gy*5-reset_gx]:1'b0;
    // --- Rightmost top-row key: EDIT (normal) / RETURN (post page) --------
    wire edit_key=menu_visible && x_d1>=12'd1184 && x_d1<12'd1280 &&
                  y_d1>=12'd32 && y_d1<12'd94;
    wire edit_border=edit_key && (x_d1<12'd1188 || x_d1>=12'd1276 ||
                      y_d1<12'd36 || y_d1>=12'd90);
    wire [11:0] edit_text_left=post_mode ? 12'd1196 : 12'd1208;
    wire [11:0] edit_text_right=edit_text_left+(post_mode ? 12'd72 : 12'd48);
    wire edit_text_area=edit_key && x_d1>=edit_text_left && x_d1<edit_text_right &&
                        y_d1>=12'd56 && y_d1<12'd70;
    wire [11:0] edit_tx=x_d1-edit_text_left;
    wire [3:0] edit_ci=edit_tx/12;
    wire [3:0] edit_cell_x=edit_tx%12;
    wire [2:0] edit_gx=edit_cell_x>>1;
    wire [2:0] edit_gy=(y_d1-12'd56)>>1;
    wire [7:0] edit_char=post_mode ?
        (edit_ci==0?"R":edit_ci==1?"E":edit_ci==2?"T":edit_ci==3?"U":
         edit_ci==4?"R":edit_ci==5?"N":" ") :
        (edit_ci==0?"E":edit_ci==1?"D":edit_ci==2?"I":edit_ci==3?"T":" ");
    wire [34:0] edit_bits=glyph(edit_char);
    wire edit_ink=(edit_text_area && edit_cell_x<10 && edit_gx<5 && edit_gy<7)?
        edit_bits[34-edit_gy*5-edit_gx]:1'b0;
    // --- Eight tool cells, 112 wide on a 120 pitch, starting at x=20 ------
    wire post_row=post_mode && y_d1>=12'd674 && y_d1<12'd714;
    wire post_item_zone=post_row && x_d1>=12'd20 && x_d1<12'd972;
    wire [2:0] item_sel=(x_d1<12'd132)?3'd0:(x_d1<12'd252)?3'd1:(x_d1<12'd372)?3'd2:
        (x_d1<12'd492)?3'd3:(x_d1<12'd612)?3'd4:(x_d1<12'd732)?3'd5:
        (x_d1<12'd852)?3'd6:3'd7;
    wire [11:0] item_base=12'd20+{9'd0,item_sel}*12'd120;
    wire item_cell=post_item_zone && x_d1>=item_base && x_d1<item_base+12'd112;
    wire item_border=item_cell && (x_d1<item_base+12'd2 || x_d1>=item_base+12'd110 ||
                      y_d1<12'd676 || y_d1>=12'd712);
    wire item_selected=(post_tool==item_sel);
    wire [11:0] item_tx=x_d1-(item_base+12'd8);
    wire [3:0] item_ci=item_tx/12;
    wire [3:0] item_cell_x=item_tx%12;
    wire [2:0] item_gx=item_cell_x>>1;
    wire [2:0] item_gy=(y_d1-12'd687)>>1;
    wire item_text_area=item_cell && y_d1>=12'd687 && y_d1<12'd701 && item_tx<12'd104;
    wire [34:0] item_bits=glyph(tool_char(item_sel,item_ci));
    wire item_ink=(item_text_area && item_cell_x<10 && item_gx<5 && item_gy<7)?
        item_bits[34-item_gy*5-item_gx]:1'b0;
    // --- Target page selector: which half the tool is applied to ----------
    wire page_left_hit=post_row && x_d1>=12'd1000 && x_d1<12'd1120;
    wire page_right_hit=post_row && x_d1>=12'd1128 && x_d1<12'd1248;
    wire page_hit=page_left_hit||page_right_hit;
    wire page_hit_selected=(page_left_hit && !post_page)||(page_right_hit && post_page);
    wire page_border=page_hit && ((page_left_hit && (x_d1<12'd1002 || x_d1>=12'd1118)) ||
                      (page_right_hit && (x_d1<12'd1130 || x_d1>=12'd1246)) ||
                      y_d1<12'd676 || y_d1>=12'd712);
    wire [11:0] page_text_left=page_left_hit ? 12'd1036 : 12'd1158;
    wire [11:0] page_text_right=page_left_hit ? 12'd1084 : 12'd1218;
    wire page_text_area=page_hit && x_d1>=page_text_left && x_d1<page_text_right &&
                        y_d1>=12'd687 && y_d1<12'd701;
    wire [11:0] page_tx=x_d1-page_text_left;
    wire [3:0] page_ci=page_tx/12;
    wire [3:0] page_cell_x=page_tx%12;
    wire [2:0] page_gx=page_cell_x>>1;
    wire [2:0] page_gy=(y_d1-12'd687)>>1;
    wire [7:0] page_char=page_left_hit ?
        (page_ci==0?"L":page_ci==1?"E":page_ci==2?"F":page_ci==3?"T":" ") :
        (page_ci==0?"R":page_ci==1?"I":page_ci==2?"G":page_ci==3?"H":page_ci==4?"T":" ");
    wire [34:0] page_bits=glyph(page_char);
    wire page_ink=(page_text_area && page_cell_x<10 && page_gx<5 && page_gy<7)?
        page_bits[34-page_gy*5-page_gx]:1'b0;
    // --- Tool sub page: colour palette (DOODLE) or a value slider ---------
    wire sub_active=post_mode && (post_tool!=3'd1);
    wire sub_band=sub_active && y_d1>=12'd596 && y_d1<12'd652 &&
                  x_d1>=12'd20 && x_d1<12'd1260;
    wire sub_border=sub_band && (x_d1<12'd22 || x_d1>=12'd1258 ||
                    y_d1<12'd598 || y_d1>=12'd650);
    wire doodle_page=sub_active && (post_tool==3'd0);
    wire [2:0] swatch_sel=(x_d1<12'd112)?3'd0:(x_d1<12'd184)?3'd1:
        (x_d1<12'd256)?3'd2:(x_d1<12'd328)?3'd3:(x_d1<12'd400)?3'd4:
        (x_d1<12'd472)?3'd5:(x_d1<12'd544)?3'd6:3'd7;
    wire [11:0] swatch_base=12'd40+{9'd0,swatch_sel}*12'd72;
    wire swatch_cell=doodle_page && x_d1>=swatch_base &&
        x_d1<swatch_base+12'd64 && y_d1>=12'd604 && y_d1<12'd644;
    wire swatch_border=swatch_cell && (x_d1<swatch_base+12'd3 ||
        x_d1>=swatch_base+12'd61 || y_d1<12'd607 || y_d1>=12'd641);
    wire swatch_selected=(sub_color==swatch_sel);
    wire clear_key=doodle_page && x_d1>=12'd1140 && x_d1<12'd1250 &&
                   y_d1>=12'd600 && y_d1<12'd648;
    wire clear_border=clear_key && (x_d1<12'd1144 || x_d1>=12'd1246 ||
                      y_d1<12'd604 || y_d1>=12'd644);
    wire clear_text_area=clear_key && x_d1>=12'd1165 && x_d1<12'd1225 &&
                         y_d1>=12'd617 && y_d1<12'd631;
    wire [5:0] clear_tx=x_d1-12'd1165;
    wire [2:0] clear_ci=clear_tx/12;
    wire [3:0] clear_cell_x=clear_tx%12;
    wire [2:0] clear_gx=clear_cell_x>>1;
    wire [2:0] clear_gy=(y_d1-12'd617)>>1;
    wire [7:0] clear_char=clear_ci==0?"C":clear_ci==1?"L":clear_ci==2?"E":
        clear_ci==3?"A":clear_ci==4?"R":" ";
    wire [34:0] clear_bits=glyph(clear_char);
    wire clear_ink=(clear_text_area && clear_cell_x<10 && clear_gx<5 && clear_gy<7)?
        clear_bits[34-clear_gy*5-clear_gx]:1'b0;
    // The six adjustment tools share one slider: black track plus a purple
    // knob whose centre moves 264 + 3*value pixels, i.e. 264..1029.
    wire slider_page=sub_active && (post_tool!=3'd0);
    wire [11:0] knob_center=12'd264+{9'd0,knob_value}+{10'd0,knob_value,1'b0};
    // A thin rail passes through the knob centre instead of looking like a
    // thick slot containing the knob.
    wire slider_track=slider_page && x_d1>=12'd240 && x_d1<12'd1050 &&
                      y_d1>=12'd623 && y_d1<12'd625;
    wire [11:0] knob_dx=(x_d1>knob_center)?x_d1-knob_center:knob_center-x_d1;
    wire [11:0] knob_dy=(y_d1>12'd624)?y_d1-12'd624:12'd624-y_d1;
    wire knob_circle=slider_page && knob_dx<=12'd12 && knob_dy<=12'd12 &&
                     knob_dx<={7'd0,circle_span(knob_dy)};
    // Direct numeric readout of the value that reached the pixel-domain UI.
    wire value_box=slider_page && x_d1>=12'd1080 && x_d1<12'd1172 &&
                   y_d1>=12'd606 && y_d1<12'd642;
    wire value_border=value_box && (x_d1<12'd1082 || x_d1>=12'd1170 ||
                      y_d1<12'd608 || y_d1>=12'd640);
    wire [7:0] value_hundreds=knob_value/8'd100;
    wire [7:0] value_tens=(knob_value%8'd100)/8'd10;
    wire [7:0] value_ones=knob_value%8'd10;
    wire value_text_area=value_box && x_d1>=12'd1108 && x_d1<12'd1144 &&
                         y_d1>=12'd617 && y_d1<12'd631;
    wire [5:0] value_tx=x_d1-12'd1108;
    wire [1:0] value_ci=value_tx/12;
    wire [3:0] value_cell_x=value_tx%12;
    wire [2:0] value_gx=value_cell_x>>1;
    wire [2:0] value_gy=(y_d1-12'd617)>>1;
    wire [7:0] value_char=(value_ci==0)?(8'd48+value_hundreds):
                           (value_ci==1)?(8'd48+value_tens):(8'd48+value_ones);
    wire [34:0] value_bits=glyph(value_char);
    wire value_ink=(value_text_area && value_cell_x<10 && value_gx<5 && value_gy<7)?
        value_bits[34-value_gy*5-value_gx]:1'b0;
    wire slider_reset_key=slider_page && x_d1>=12'd1184 && x_d1<12'd1248 &&
                          y_d1>=12'd606 && y_d1<12'd642;
    wire slider_reset_border=slider_reset_key && (x_d1<12'd1186 || x_d1>=12'd1246 ||
                             y_d1<12'd608 || y_d1>=12'd640);
    wire slider_reset_text=slider_reset_key && x_d1>=12'd1198 && x_d1<12'd1234 &&
                           y_d1>=12'd617 && y_d1<12'd631;
    wire [5:0] slider_reset_tx=x_d1-12'd1198;
    wire [1:0] slider_reset_ci=slider_reset_tx/12;
    wire [3:0] slider_reset_cell_x=slider_reset_tx%12;
    wire [2:0] slider_reset_gx=slider_reset_cell_x>>1;
    wire [2:0] slider_reset_gy=(y_d1-12'd617)>>1;
    wire [7:0] slider_reset_char=slider_reset_ci==0?"R":slider_reset_ci==1?"S":"T";
    wire [34:0] slider_reset_bits=glyph(slider_reset_char);
    wire slider_reset_ink=(slider_reset_text && slider_reset_cell_x<10 &&
        slider_reset_gx<5 && slider_reset_gy<7)?
        slider_reset_bits[34-slider_reset_gy*5-slider_reset_gx]:1'b0;
    // --- One-shot blue flash on key press ---------------------------------
    reg [2:0] flash_code;
    reg [27:0] flash_count;
    reg press_toggle_d;
    wire press_edge=press_toggle^press_toggle_d;
    wire flash_on=(flash_count!=28'd0);
    wire flash_reset=flash_on && (flash_code==PRESS_RESET);
    wire flash_edit=flash_on && (flash_code==PRESS_EDIT);
    wire flash_tool=flash_on && (flash_code==PRESS_TOOL);
    wire flash_page=flash_on && (flash_code==PRESS_PAGE);
    wire flash_clear=flash_on && (flash_code==PRESS_CLEAR);
    always @(posedge pixel_clk or negedge rst_n) begin
        if(!rst_n) begin
            de_d0<=0; de_d1<=0; hs_d0<=0; hs_d1<=0; vs_d0<=0; vs_d1<=0;
            rgb_d0<=0; rgb_d1<=0; x<=0; y<=0; x_d0<=0; x_d1<=0; y_d0<=0; y_d1<=0; prev_de<=0; prev_vs<=0;
            video_rgb<=0; video_de<=0; video_hs<=0; video_vs<=0;
            flash_code<=3'd0; flash_count<=28'd0; press_toggle_d<=1'b0;
        end else begin
            press_toggle_d<=press_toggle;
            if(press_edge) begin
                flash_code<=press_code; flash_count<=FLASH_CYCLES;
            end else if(flash_count!=28'd0) flash_count<=flash_count-1'b1;
            prev_de<=camera_de; prev_vs<=camera_vs;
            if(vs_fall) y<=0;
            else if(line_end) y<=y+1'b1;
            if(camera_de) x<=x+1'b1; else x<=0;
            x_d0<=x; x_d1<=x_d0; y_d0<=y; y_d1<=y_d0;
            de_d0<=camera_de; de_d1<=de_d0; hs_d0<=camera_hs; hs_d1<=hs_d0;
            vs_d0<=camera_vs; vs_d1<=vs_d0; rgb_d0<=camera_rgb; rgb_d1<=rgb_d0;
            video_de<=de_d1; video_hs<=hs_d1; video_vs<=vs_d1;
            if(!de_d1) video_rgb<=0;
            else if(reset_button) begin
                if(flash_reset) video_rgb<=24'h1f6fff;
                else if(reset_border) video_rgb<=24'hc084fc;
                else if(reset_ink) video_rgb<=24'hffffff;
                else video_rgb<=24'h6d35a8;
            end
            else if(edit_key) begin
                if(flash_edit) video_rgb<=24'h1f6fff;
                else if(edit_border) video_rgb<=24'hc084fc;
                else if(edit_ink) video_rgb<=24'hffffff;
                else video_rgb<=post_mode ? 24'h8e52d1 : 24'h2d1647;
            end
            else if(sub_band) begin
                if(swatch_cell) begin
                    if(swatch_border) video_rgb<=swatch_selected ? 24'h000000 : 24'hb0b0c0;
                    else video_rgb<=palette(swatch_sel);
                end
                else if(clear_key) begin
                    if(flash_clear) video_rgb<=24'h1f6fff;
                    else if(clear_border) video_rgb<=24'hc084fc;
                    else if(clear_ink) video_rgb<=24'hffffff;
                    else video_rgb<=24'h2d1647;
                end
                else if(knob_circle) video_rgb<=24'h9b5cf6;
                else if(slider_track) video_rgb<=24'h000000;
                else if(value_box) begin
                    if(value_ink) video_rgb<=24'hffffff;
                    else if(value_border) video_rgb<=24'hc084fc;
                    else video_rgb<=24'h2d1647;
                end
                else if(slider_reset_key) begin
                    if(slider_reset_ink) video_rgb<=24'hffffff;
                    else if(slider_reset_border) video_rgb<=24'hc084fc;
                    else video_rgb<=24'h6d35a8;
                end
                else if(sub_border) video_rgb<=24'h8e52d1;
                else video_rgb<=24'h1a0f2e;
            end
            else if(item_cell) begin
                if(flash_tool) video_rgb<=24'h1f6fff;
                else if(item_ink) video_rgb<=24'hffffff;
                else if(item_border) video_rgb<=item_selected ? 24'h6cb6ff : 24'h8e52d1;
                else video_rgb<=item_selected ? 24'h1473e6 : 24'h2d1647;
            end
            else if(page_hit && cmp_ui_mode && !page_hit_selected) begin
                // CMP EDIT is locked to the page's own camera: grey, inert.
                if(page_ink) video_rgb<=24'h8a8a8a;
                else if(page_border) video_rgb<=24'h5a5a5a;
                else video_rgb<=24'h303030;
            end
            else if(page_hit) begin
                if(flash_page && page_hit_selected) video_rgb<=24'h1f6fff;
                else if(page_ink) video_rgb<=24'hffffff;
                else if(page_border) video_rgb<=page_hit_selected ? 24'h6cb6ff : 24'h8e52d1;
                else video_rgb<=page_hit_selected ? 24'h1473e6 : 24'h2d1647;
            end
            else if(mode_badge) begin
                if(mode_badge_border || mode_ink) video_rgb<=24'hc084fc;
                else video_rgb<=24'h2d1647;
            end
            else if(menu_panel) begin
                if(menu_ink) video_rgb<=24'hffffff;
                else if(tab_hit) video_rgb<=tab_color;
                else video_rgb<=24'h2d1647;
            end
            else if(feature_panel && !post_mode && cmp_ui_mode) begin
                if(cmp_text_ink) video_rgb<=24'hffffff;
                else if(x_d1>=12'd286 && x_d1<12'd634 && y_d1>=12'd676 && y_d1<12'd712) begin
                    if(x_d1<12'd290 || x_d1>=12'd630 || y_d1<12'd680 || y_d1>=12'd708)
                        video_rgb<=cmp_undistort[layout==LAYOUT_RIGHT_COMPARE] ? 24'h6cb6ff : 24'hc084fc;
                    else video_rgb<=cmp_undistort[layout==LAYOUT_RIGHT_COMPARE] ? 24'h1473e6 : 24'h2d1647;
                end else if(x_d1>=12'd646 && x_d1<12'd994 && y_d1>=12'd676 && y_d1<12'd712) begin
                    if(x_d1<12'd650 || x_d1>=12'd990 || y_d1<12'd680 || y_d1>=12'd708)
                        video_rgb<=cmp_binary[layout==LAYOUT_RIGHT_COMPARE] ? 24'h6cb6ff : 24'hc084fc;
                    else video_rgb<=cmp_binary[layout==LAYOUT_RIGHT_COMPARE] ? 24'h1473e6 : 24'h2d1647;
                end else video_rgb<=rgb_d1;
            end
            else if(feature_panel && !post_mode) begin
                if(feature_ink) video_rgb<=24'hffffff;
                else if(feature_border) video_rgb<=feature_selected ? 24'h6cb6ff : 24'hc084fc;
                else if(feature_hit) video_rgb<=feature_selected ? 24'h1473e6 : 24'h8e52d1;
                else video_rgb<=24'h2d1647;
            end
            else if(compare_binary) video_rgb<=binary_rgb;
            else video_rgb<=rgb_d1;
        end
    end
endmodule

// Motion-gated camera-pose indicator. Quaternion math is evaluated in the
// IMU clock domain and only six projected basis components cross into the
// pixel domain. The shown tuple is committed at a frame boundary so the cube
// cannot tear while the attitude changes.
module imu_pose_overlay #(
    parameter integer SYS_CLK_HZ = 27000000,
    parameter integer MOTION_THRESHOLD_RAW = 250,
    parameter integer MOTION_HOLD_MS = 500,
    parameter integer ENABLE = 1
)(
    input  wire sys_clk,
    input  wire pixel_clk,
    input  wire rst_n,
    input  wire menu_visible_sys,
    input  wire menu_visible_pixel,
    input  wire imu_data_valid,
    input  wire imu_attitude_ready,
    input  wire imu_angle_valid,
    input  wire signed [15:0] gyro_x_raw,
    input  wire signed [15:0] gyro_y_raw,
    input  wire signed [15:0] gyro_z_raw,
    input  wire signed [31:0] q0,
    input  wire signed [31:0] q1,
    input  wire signed [31:0] q2,
    input  wire signed [31:0] q3,
    input  wire [23:0] camera_rgb,
    input  wire camera_de,
    input  wire camera_hs,
    input  wire camera_vs,
    output reg  [23:0] video_rgb,
    output reg  video_de,
    output reg  video_hs,
    output reg  video_vs
);
    localparam integer HOLD_CYCLES_RAW = (SYS_CLK_HZ/1000)*MOTION_HOLD_MS;
    localparam integer HOLD_CYCLES = (HOLD_CYCLES_RAW < 1) ? 1 : HOLD_CYCLES_RAW;
    localparam signed [63:0] ONE_Q30 = 64'sd1073741824;
    localparam signed [31:0] CUBE_HALF = 32'sd30;
    localparam signed [31:0] DEPTH_X = 32'sd15;
    localparam signed [31:0] DEPTH_Y = 32'sd10;
    localparam signed [15:0] CENTER_X = 16'sd105;
    localparam signed [15:0] CENTER_Y = 16'sd590;

    function [15:0] abs16;
        input signed [15:0] value;
        begin
            abs16 = value[15] ? (~value + 16'd1) : value;
        end
    endfunction

    wire moving_sample = (abs16(gyro_x_raw) >= MOTION_THRESHOLD_RAW) ||
                         (abs16(gyro_y_raw) >= MOTION_THRESHOLD_RAW) ||
                         (abs16(gyro_z_raw) >= MOTION_THRESHOLD_RAW);
    reg [31:0] motion_hold_count;
    reg motion_active_sys;
    reg motion_previous;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n || !menu_visible_sys || !ENABLE) begin
            motion_hold_count <= 32'd0;
            motion_active_sys <= 1'b0;
            motion_previous <= 1'b0;
        end else begin
            motion_previous <= motion_active_sys;
            if (imu_data_valid && imu_attitude_ready && moving_sample) begin
                motion_hold_count <= HOLD_CYCLES;
                motion_active_sys <= 1'b1;
            end else if (motion_active_sys) begin
                if (motion_hold_count == 0)
                    motion_active_sys <= 1'b0;
                else
                    motion_hold_count <= motion_hold_count - 1'b1;
            end
        end
    end

    // q0 is scalar (w), q1/q2/q3 are x/y/z. Products are Q60 and the
    // resulting rotation-matrix entries are reduced back to Q30.
    wire signed [63:0] q1q1 = q1*q1;
    wire signed [63:0] q2q2 = q2*q2;
    wire signed [63:0] q3q3 = q3*q3;
    wire signed [63:0] q0q1 = q0*q1;
    wire signed [63:0] q0q2 = q0*q2;
    wire signed [63:0] q0q3 = q0*q3;
    wire signed [63:0] q1q2 = q1*q2;
    wire signed [63:0] q1q3 = q1*q3;
    wire signed [63:0] q2q3 = q2*q3;
    wire signed [63:0] r00_w = ONE_Q30 - ((q2q2 + q3q3) >>> 29);
    wire signed [63:0] r01_w = (q1q2 - q0q3) >>> 29;
    wire signed [63:0] r02_w = (q1q3 + q0q2) >>> 29;
    wire signed [63:0] r10_w = (q1q2 + q0q3) >>> 29;
    wire signed [63:0] r11_w = ONE_Q30 - ((q1q1 + q3q3) >>> 29);
    wire signed [63:0] r12_w = (q2q3 - q0q1) >>> 29;
    wire signed [63:0] r20_w = (q1q3 - q0q2) >>> 29;
    wire signed [63:0] r21_w = (q2q3 + q0q1) >>> 29;
    wire signed [63:0] r22_w = ONE_Q30 - ((q1q1 + q2q2) >>> 29);
    wire signed [31:0] r00 = r00_w[31:0];
    wire signed [31:0] r01 = r01_w[31:0];
    wire signed [31:0] r02 = r02_w[31:0];
    wire signed [31:0] r10 = r10_w[31:0];
    wire signed [31:0] r11 = r11_w[31:0];
    wire signed [31:0] r12 = r12_w[31:0];
    wire signed [31:0] r20 = r20_w[31:0];
    wire signed [31:0] r21 = r21_w[31:0];
    wire signed [31:0] r22 = r22_w[31:0];

    // Camera/body mounting transform (right-handed):
    //   display X = IMU Z  (camera optical/forward direction)
    //   display Y = IMU Y
    //   display Z = -IMU X (physical up direction)
    // World Y is drawn to screen-right, world Z to screen-up, while world X
    // receives a small diagonal depth offset. This makes the calibrated pose
    // look like the real camera instead of the upside-down sensor package.
    wire signed [63:0] bx_x_w = r12*CUBE_HALF + r02*DEPTH_X;
    wire signed [63:0] bx_y_w = -r22*CUBE_HALF + r02*DEPTH_Y;
    wire signed [63:0] by_x_w = r11*CUBE_HALF + r01*DEPTH_X;
    wire signed [63:0] by_y_w = -r21*CUBE_HALF + r01*DEPTH_Y;
    wire signed [63:0] bz_x_w = -r10*CUBE_HALF - r00*DEPTH_X;
    wire signed [63:0] bz_y_w = r20*CUBE_HALF - r00*DEPTH_Y;
    wire signed [15:0] bx_x_sys = bx_x_w >>> 30;
    wire signed [15:0] bx_y_sys = bx_y_w >>> 30;
    wire signed [15:0] by_x_sys = by_x_w >>> 30;
    wire signed [15:0] by_y_sys = by_y_w >>> 30;
    wire signed [15:0] bz_x_sys = bz_x_w >>> 30;
    wire signed [15:0] bz_y_sys = bz_y_w >>> 30;

    wire pose_source_event = imu_angle_valid ||
                             (motion_active_sys != motion_previous);
    wire pose_pixel_event;
    wire [96:0] pose_pixel_data;
    touch_event_cdc #(.WIDTH(97)) u_pose_cdc (
        .src_clk(sys_clk),.dst_clk(pixel_clk),.rst_n(rst_n),
        .src_event(pose_source_event),
        .src_data({motion_active_sys,bx_x_sys,bx_y_sys,
                   by_x_sys,by_y_sys,bz_x_sys,bz_y_sys}),
        .dst_event(pose_pixel_event),.dst_data(pose_pixel_data)
    );

    reg [96:0] pose_pending;
    reg [96:0] pose_shown;
    reg [11:0] pixel_x,pixel_y;
    reg de_previous,vs_previous;
    wire frame_start = camera_vs && !vs_previous;
    wire line_end = de_previous && !camera_de;
    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            pose_pending <= 97'd0;
            pose_shown <= 97'd0;
            pixel_x <= 12'd0;
            pixel_y <= 12'd0;
            de_previous <= 1'b0;
            vs_previous <= 1'b0;
        end else begin
            de_previous <= camera_de;
            vs_previous <= camera_vs;
            if (pose_pixel_event)
                pose_pending <= pose_pixel_data;
            if (frame_start) begin
                pixel_y <= 12'd0;
                pose_shown <= pose_pending;
            end else if (line_end)
                pixel_y <= pixel_y + 1'b1;
            if (camera_de)
                pixel_x <= pixel_x + 1'b1;
            else
                pixel_x <= 12'd0;
        end
    end

    wire signed [15:0] bx_x = pose_shown[95:80];
    wire signed [15:0] bx_y = pose_shown[79:64];
    wire signed [15:0] by_x = pose_shown[63:48];
    wire signed [15:0] by_y = pose_shown[47:32];
    wire signed [15:0] bz_x = pose_shown[31:16];
    wire signed [15:0] bz_y = pose_shown[15:0];
    wire signed [15:0] px = $signed({4'b0000,pixel_x});
    wire signed [15:0] py = $signed({4'b0000,pixel_y});

    wire signed [15:0] v0x=CENTER_X-bx_x-by_x-bz_x;
    wire signed [15:0] v0y=CENTER_Y-bx_y-by_y-bz_y;
    wire signed [15:0] v1x=CENTER_X+bx_x-by_x-bz_x;
    wire signed [15:0] v1y=CENTER_Y+bx_y-by_y-bz_y;
    wire signed [15:0] v2x=CENTER_X+bx_x+by_x-bz_x;
    wire signed [15:0] v2y=CENTER_Y+bx_y+by_y-bz_y;
    wire signed [15:0] v3x=CENTER_X-bx_x+by_x-bz_x;
    wire signed [15:0] v3y=CENTER_Y-bx_y+by_y-bz_y;
    wire signed [15:0] v4x=CENTER_X-bx_x-by_x+bz_x;
    wire signed [15:0] v4y=CENTER_Y-bx_y-by_y+bz_y;
    wire signed [15:0] v5x=CENTER_X+bx_x-by_x+bz_x;
    wire signed [15:0] v5y=CENTER_Y+bx_y-by_y+bz_y;
    wire signed [15:0] v6x=CENTER_X+bx_x+by_x+bz_x;
    wire signed [15:0] v6y=CENTER_Y+bx_y+by_y+bz_y;
    wire signed [15:0] v7x=CENTER_X-bx_x+by_x+bz_x;
    wire signed [15:0] v7y=CENTER_Y-bx_y+by_y+bz_y;

    function line_pixel;
        input signed [15:0] test_x,test_y,x0,y0,x1,y1;
        input [2:0] thickness;
        reg signed [16:0] dx,dy,rx,ry,min_x,max_x,min_y,max_y,margin;
        reg signed [33:0] cross_value;
        reg [33:0] cross_abs;
        reg [16:0] dx_abs,dy_abs;
        reg [20:0] tolerance;
        begin
            dx=x1-x0; dy=y1-y0; rx=test_x-x0; ry=test_y-y0;
            min_x=(x0<x1)?x0:x1; max_x=(x0>x1)?x0:x1;
            min_y=(y0<y1)?y0:y1; max_y=(y0>y1)?y0:y1;
            margin={14'd0,thickness};
            dx_abs=dx[16]?-dx:dx; dy_abs=dy[16]?-dy:dy;
            cross_value=rx*dy-ry*dx;
            cross_abs=cross_value[33]?-cross_value:cross_value;
            tolerance=(dx_abs+dy_abs+1'b1)*thickness;
            line_pixel=(test_x>=min_x-margin && test_x<=max_x+margin &&
                        test_y>=min_y-margin && test_y<=max_y+margin &&
                        cross_abs<=tolerance);
        end
    endfunction

    function label_pixel;
        input signed [15:0] test_x,test_y,label_left,label_top;
        input [34:0] glyph_bits;
        reg signed [16:0] glyph_x,glyph_y;
        integer glyph_index;
        begin
            glyph_x=test_x-label_left;
            glyph_y=test_y-label_top;
            glyph_index=34-glyph_y*5-glyph_x;
            if(glyph_x>=0 && glyph_x<5 && glyph_y>=0 && glyph_y<7)
                label_pixel=glyph_bits[glyph_index];
            else
                label_pixel=1'b0;
        end
    endfunction

    localparam [34:0] GLYPH_X={5'b10001,5'b10001,5'b01010,5'b00100,
                               5'b01010,5'b10001,5'b10001};
    localparam [34:0] GLYPH_Y={5'b10001,5'b10001,5'b01010,5'b00100,
                               5'b00100,5'b00100,5'b00100};
    localparam [34:0] GLYPH_Z={5'b11111,5'b00001,5'b00010,5'b00100,
                               5'b01000,5'b10000,5'b11111};

    wire cube_hit =
        line_pixel(px,py,v0x,v0y,v1x,v1y,3'd2) ||
        line_pixel(px,py,v1x,v1y,v2x,v2y,3'd2) ||
        line_pixel(px,py,v2x,v2y,v3x,v3y,3'd2) ||
        line_pixel(px,py,v3x,v3y,v0x,v0y,3'd2) ||
        line_pixel(px,py,v4x,v4y,v5x,v5y,3'd2) ||
        line_pixel(px,py,v5x,v5y,v6x,v6y,3'd2) ||
        line_pixel(px,py,v6x,v6y,v7x,v7y,3'd2) ||
        line_pixel(px,py,v7x,v7y,v4x,v4y,3'd2) ||
        line_pixel(px,py,v0x,v0y,v4x,v4y,3'd2) ||
        line_pixel(px,py,v1x,v1y,v5x,v5y,3'd2) ||
        line_pixel(px,py,v2x,v2y,v6x,v6y,3'd2) ||
        line_pixel(px,py,v3x,v3y,v7x,v7y,3'd2);
    wire signed [15:0] x_end_x=CENTER_X+(bx_x<<<1);
    wire signed [15:0] x_end_y=CENTER_Y+(bx_y<<<1);
    wire signed [15:0] y_end_x=CENTER_X+(by_x<<<1);
    wire signed [15:0] y_end_y=CENTER_Y+(by_y<<<1);
    wire signed [15:0] z_end_x=CENTER_X+(bz_x<<<1);
    wire signed [15:0] z_end_y=CENTER_Y+(bz_y<<<1);
    // Two short swept-back segments form an arrowhead at every axis end.
    wire signed [15:0] x_arrow1_x=x_end_x-(bx_x>>>1)-(bx_y>>>2);
    wire signed [15:0] x_arrow1_y=x_end_y-(bx_y>>>1)+(bx_x>>>2);
    wire signed [15:0] x_arrow2_x=x_end_x-(bx_x>>>1)+(bx_y>>>2);
    wire signed [15:0] x_arrow2_y=x_end_y-(bx_y>>>1)-(bx_x>>>2);
    wire signed [15:0] y_arrow1_x=y_end_x-(by_x>>>1)-(by_y>>>2);
    wire signed [15:0] y_arrow1_y=y_end_y-(by_y>>>1)+(by_x>>>2);
    wire signed [15:0] y_arrow2_x=y_end_x-(by_x>>>1)+(by_y>>>2);
    wire signed [15:0] y_arrow2_y=y_end_y-(by_y>>>1)-(by_x>>>2);
    wire signed [15:0] z_arrow1_x=z_end_x-(bz_x>>>1)-(bz_y>>>2);
    wire signed [15:0] z_arrow1_y=z_end_y-(bz_y>>>1)+(bz_x>>>2);
    wire signed [15:0] z_arrow2_x=z_end_x-(bz_x>>>1)+(bz_y>>>2);
    wire signed [15:0] z_arrow2_y=z_end_y-(bz_y>>>1)-(bz_x>>>2);
    wire signed [15:0] x_label_x=x_end_x+(bx_x[15]?-16'sd10:16'sd4);
    wire signed [15:0] x_label_y=x_end_y+(bx_y[15]?-16'sd10:16'sd4);
    wire signed [15:0] y_label_x=y_end_x+(by_x[15]?-16'sd10:16'sd4);
    wire signed [15:0] y_label_y=y_end_y+(by_y[15]?-16'sd10:16'sd4);
    wire signed [15:0] z_label_x=z_end_x+(bz_x[15]?-16'sd10:16'sd4);
    wire signed [15:0] z_label_y=z_end_y+(bz_y[15]?-16'sd10:16'sd4);
    wire x_axis_hit=line_pixel(px,py,CENTER_X,CENTER_Y,x_end_x,x_end_y,3'd3) ||
                    line_pixel(px,py,x_end_x,x_end_y,x_arrow1_x,x_arrow1_y,3'd2) ||
                    line_pixel(px,py,x_end_x,x_end_y,x_arrow2_x,x_arrow2_y,3'd2) ||
                    label_pixel(px,py,x_label_x,x_label_y,GLYPH_X);
    wire y_axis_hit=line_pixel(px,py,CENTER_X,CENTER_Y,y_end_x,y_end_y,3'd3) ||
                    line_pixel(px,py,y_end_x,y_end_y,y_arrow1_x,y_arrow1_y,3'd2) ||
                    line_pixel(px,py,y_end_x,y_end_y,y_arrow2_x,y_arrow2_y,3'd2) ||
                    label_pixel(px,py,y_label_x,y_label_y,GLYPH_Y);
    wire z_axis_hit=line_pixel(px,py,CENTER_X,CENTER_Y,z_end_x,z_end_y,3'd3) ||
                    line_pixel(px,py,z_end_x,z_end_y,z_arrow1_x,z_arrow1_y,3'd2) ||
                    line_pixel(px,py,z_end_x,z_end_y,z_arrow2_x,z_arrow2_y,3'd2) ||
                    label_pixel(px,py,z_label_x,z_label_y,GLYPH_Z);
    wire pose_visible = ENABLE && menu_visible_pixel && pose_shown[96];

    // 75% camera + 25% lavender gives a colored yet see-through wireframe.
    wire [9:0] cube_mix_r={2'b0,camera_rgb[23:16]}+
                          ({2'b0,camera_rgb[23:16]}<<1)+10'd190;
    wire [9:0] cube_mix_g={2'b0,camera_rgb[15:8]}+
                          ({2'b0,camera_rgb[15:8]}<<1)+10'd150;
    wire [9:0] cube_mix_b={2'b0,camera_rgb[7:0]}+
                          ({2'b0,camera_rgb[7:0]}<<1)+10'd255;
    wire [23:0] cube_blend={cube_mix_r[9:2],cube_mix_g[9:2],cube_mix_b[9:2]};

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            video_rgb<=24'd0; video_de<=1'b0;
            video_hs<=1'b0; video_vs<=1'b0;
        end else begin
            video_de<=camera_de; video_hs<=camera_hs; video_vs<=camera_vs;
            if (!camera_de) video_rgb<=24'd0;
            else if (pose_visible && x_axis_hit) video_rgb<=24'h42f56c;
            else if (pose_visible && y_axis_hit) video_rgb<=24'hffe45c;
            else if (pose_visible && z_axis_hit) video_rgb<=24'h45a3ff;
            else if (pose_visible && cube_hit) video_rgb<=cube_blend;
            else video_rgb<=camera_rgb;
        end
    end
endmodule

