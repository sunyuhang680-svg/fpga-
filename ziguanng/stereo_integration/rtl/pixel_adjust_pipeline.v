`timescale 1ns/1ps
// Candidate: exact legacy RGB565 arithmetic widths and ordering, nine stages.
// No backpressure: the renderer accepts every tagged result into its line RAM.
module pixel_adjust_pipeline #(parameter TEST_PATTERN=0)(
    input wire clk,rst,in_valid,raw_valid,binary_mode,
    input wire [15:0] raw_pixel,
    input wire [9:0] pixel_x,
    input wire [47:0] adjust_values,
    input wire [3:0] doodle_index,
    output wire out_valid,
    output wire [9:0] out_x,
    output reg [15:0] out_pixel
);
    reg [8:0] valid;
    // raw processed pixel, adjust values, x, validity, palette, doodle enable,
    // pattern pixel, pattern enable, previous unadjusted source luminance.
    reg [123:0] meta[0:8];
    reg signed [13:0] r[0:7],g[0:7],b[0:7];
    reg signed [14:0] previous_lum,lum4,lum5;
    reg [10:0] exposure_gain0;
    reg [9:0] contrast_gain0,contrast_gain1,contrast_gain2;
    wire [6:0] pixel_sum={2'b0,raw_pixel[15:11]}+{1'b0,raw_pixel[10:5]}+{2'b0,raw_pixel[4:0]};
    wire [15:0] processed=(raw_valid && TEST_PATTERN==0 && binary_mode)?((pixel_sum>=48)?16'hffff:16'h0000):raw_pixel;
    wire signed [14:0] satur_delta=$signed({1'b0,meta[4][55:48]})-15'sd128;
    wire signed [14:0] sharpen_delta=$signed({1'b0,meta[5][39:32]})-15'sd128;
    wire signed [14:0] temp_delta=($signed({1'b0,meta[6][63:56]})-15'sd128)>>>1;
    function [5:0] clip6;input signed [14:0] v;begin clip6=v<0?0:(v>63?63:v[5:0]);end endfunction
    function [4:0] clip5;input signed [14:0] v;begin clip5=v<0?0:(v>31?31:v[4:0]);end endfunction
    function [15:0] palette565;input [2:0] c;begin
        case(c)
            0:palette565=16'hf800;1:palette565=16'hfd20;2:palette565=16'hffe0;3:palette565=16'h07e0;
            4:palette565=16'h07ff;5:palette565=16'h001f;6:palette565=16'h781f;default:palette565=16'hffff;
        endcase
    end endfunction
    function [15:0] bars;input [9:0] x;begin
        if(x<80)bars=16'hffff;else if(x<160)bars=16'hffe0;else if(x<240)bars=16'h07ff;
        else if(x<320)bars=16'h07e0;else if(x<400)bars=16'hf81f;else if(x<480)bars=16'hf800;
        else if(x<560)bars=16'h001f;else bars=0;
    end endfunction
    assign out_valid=valid[8];assign out_x=meta[8][73:64];
    integer i;
    always @(posedge clk or posedge rst)begin
        if(rst)begin
            valid<=0;previous_lum<=0;out_pixel<=0;lum4<=0;lum5<=0;
            exposure_gain0<=0;contrast_gain0<=0;contrast_gain1<=0;contrast_gain2<=0;
            for(i=0;i<9;i=i+1)meta[i]<=0;
            for(i=0;i<8;i=i+1)begin r[i]<=0;g[i]<=0;b[i]<=0;end
        end else begin
            valid<={valid[7:0],in_valid};
            meta[0]<={previous_lum,(TEST_PATTERN!=0),bars(pixel_x),doodle_index[3],palette565(doodle_index[2:0]),raw_valid,pixel_x,adjust_values,processed};
            for(i=1;i<9;i=i+1)meta[i]<=meta[i-1];
            if(in_valid)previous_lum<=({9'd0,processed[15:11],1'b0}+{8'd0,processed[10:5]}+{9'd0,processed[4:0],1'b0})>>>2;
            r[0]<=$signed({1'b0,processed[15:11],1'b0});g[0]<=$signed({1'b0,processed[10:5]});b[0]<=$signed({1'b0,processed[4:0],1'b0});
            exposure_gain0<=(adjust_values[7:0]>=128)?11'd256+(adjust_values[7:0]-8'd128)*11'd6:{2'b0,adjust_values[7:0],1'b0};
            contrast_gain0<=(adjust_values[15:8]>=128)?10'd256+(adjust_values[15:8]-8'd128)*10'd4:{1'b0,adjust_values[15:8],1'b0};
            contrast_gain1<=contrast_gain0;contrast_gain2<=contrast_gain1;
            // Keep the legacy expression widths, including truncation before
            // each arithmetic shift; widening the products changes its output.
            r[1]<=(r[0]*$signed({1'b0,exposure_gain0}))>>>8;
            g[1]<=(g[0]*$signed({1'b0,exposure_gain0}))>>>8;
            b[1]<=(b[0]*$signed({1'b0,exposure_gain0}))>>>8;
            r[2]<=r[1]+(($signed({1'b0,meta[1][47:40]})-15'sd128)>>>1);
            g[2]<=g[1]+(($signed({1'b0,meta[1][47:40]})-15'sd128)>>>1);
            b[2]<=b[1]+(($signed({1'b0,meta[1][47:40]})-15'sd128)>>>1);
            r[3]<=15'sd32+(((r[2]-15'sd32)*$signed({1'b0,contrast_gain2}))>>>8);
            g[3]<=15'sd32+(((g[2]-15'sd32)*$signed({1'b0,contrast_gain2}))>>>8);
            b[3]<=15'sd32+(((b[2]-15'sd32)*$signed({1'b0,contrast_gain2}))>>>8);
            r[4]<=r[3];g[4]<=g[3];b[4]<=b[3];lum4<=(r[3]+(g[3]<<<1)+b[3])>>>2;
            r[5]<=r[4]+(((r[4]-lum4)*satur_delta)>>>7);
            g[5]<=g[4]+(((g[4]-lum4)*satur_delta)>>>7);
            b[5]<=b[4]+(((b[4]-lum4)*satur_delta)>>>7);lum5<=lum4;
            if(meta[5][73:64]!=0)begin
                r[6]<=r[5]+(((lum5-$signed(meta[5][123:109]))*sharpen_delta)>>>6);
                g[6]<=g[5]+(((lum5-$signed(meta[5][123:109]))*sharpen_delta)>>>6);
                b[6]<=b[5]+(((lum5-$signed(meta[5][123:109]))*sharpen_delta)>>>6);
            end else begin r[6]<=r[5];g[6]<=g[5];b[6]<=b[5];end
            r[7]<=r[6]+temp_delta;g[7]<=g[6];b[7]<=b[6]-temp_delta;
            if(meta[7][108])out_pixel<=meta[7][107:92];
            else if(!meta[7][74])out_pixel<=0;
            else if(meta[7][91])out_pixel<=meta[7][90:75];
            else if(meta[7][63:16]==48'h808080808080)out_pixel<=meta[7][15:0];
            else out_pixel<={clip5(r[7]>>>1),clip6(g[7]),clip5(b[7]>>>1)};
        end
    end
endmodule
