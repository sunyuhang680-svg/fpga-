`timescale 1ns/1ps
// Standalone candidate: not registered in the board project yet.
// Crop about the output image center. A positive projective denominator makes
// all interior normalized rays convex combinations of the four corner rays.
// The caller MUST supply a certificate-bound safe box for the actual lens.
// Insets of one Q20 unit cover Q26 normalizer rounding. No pixel sampling is
// used to accept a crop; every quantized candidate is checked before publish.
module pose_auto_crop(
    input wire clk,rst_n,start,
    input wire [359:0] homography,
    input wire [11:0] width,height,
    input wire signed [21:0] xmin_q20,xmax_q20,ymin_q20,ymax_q20,
    output reg busy,done,error,
    output reg [359:0] cropped_h,
    output reg [16:0] scale_q16
);
    localparam IDLE=0,PRE_X=1,SAVE_X=2,PRE_Y=3,SAVE_Y=4,CENTER=5,
        PRE_A=6,SAVE_A=7,PRE_B=8,SAVE_B=9,PRE_C=10,SUM_C=11,SAVE_C=12,
        EDGE_X=13,SAVE_EDGE_X=14,EDGE_Y=15,SAVE_EDGE_Y=16,
        DOT_PAIR=17,DOT_OFFSET=18,PRE_BOUND=19,COMPARE=20,DECIDE=21,CHOOSE=22,MULTIPLY=31;
    reg [4:0] state,return_state;
    reg [359:0] held_h,candidate,best_h;
    reg [11:0] w,h;
    reg signed [21:0] box[0:3];
    reg signed [63:0] center[0:2],edge_x[0:2],edge_y[0:2],partial[0:2],dot[0:2];
    reg signed [63:0] product_x,product_y,operand_a;
    reg signed [21:0] operand_b;
    reg signed [85:0] product,translation,lhs;
    reg [1:0] row,corner,bound_index,phase;
    reg [16:0] current_scale,low,high,best_scale;
    reg candidate_good;
    wire signed [85:0] rounded16=round_shift(product,16);
    wire signed [85:0] rounded17=round_shift(translation,17);
    wire signed [39:0] original_offset=held_h[40*(row*3+2)+:40];
    function signed [85:0] round_shift;
        input signed [85:0] value;input integer n;
        begin
            round_shift=(value>>>n)+((value[n-1] && ((|(value & ((86'sd1<<(n-1))-1))) || !value[85]))?86'sd1:86'sd0);
        end
    endfunction
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;return_state<=IDLE;held_h<=0;candidate<=0;best_h<=0;cropped_h<=0;scale_q16<=0;
            w<=0;h<=0;busy<=0;done<=0;error<=0;row<=0;corner<=0;bound_index<=0;phase<=0;
            product_x<=0;product_y<=0;operand_a<=0;operand_b<=0;product<=0;translation<=0;lhs<=0;
            current_scale<=0;low<=0;high<=0;best_scale<=0;candidate_good<=0;
            for(k=0;k<4;k=k+1)box[k]<=0;
            for(k=0;k<3;k=k+1)begin center[k]<=0;edge_x[k]<=0;edge_y[k]<=0;partial[k]<=0;dot[k]<=0;end
        end else begin
            done<=0;
            case(state)
                IDLE:if(start)begin
                    held_h<=homography;w<=width;h<=height;row<=0;busy<=1;error<=0;
                    box[0]<=xmin_q20+22'sd1;box[1]<=xmax_q20-22'sd1;
                    box[2]<=ymin_q20+22'sd1;box[3]<=ymax_q20-22'sd1;
                    if(width<2 || height<2 || $signed({xmax_q20[21],xmax_q20})-$signed({xmin_q20[21],xmin_q20})<=3 ||
                        $signed({ymax_q20[21],ymax_q20})-$signed({ymin_q20[21],ymin_q20})<=3)begin
                        error<=1;busy<=0;done<=1;
                    end else state<=PRE_X;
                end
                MULTIPLY:begin product<=operand_a*operand_b;state<=return_state;end
                PRE_X:begin operand_a<=$signed(held_h[40*(row*3)+:40]);operand_b<=$signed({1'b0,w})-22'sd1;return_state<=SAVE_X;state<=MULTIPLY;end
                SAVE_X:begin product_x<=product;state<=PRE_Y;end
                PRE_Y:begin operand_a<=$signed(held_h[40*(row*3+1)+:40]);operand_b<=$signed({1'b0,h})-22'sd1;return_state<=SAVE_Y;state<=MULTIPLY;end
                SAVE_Y:begin product_y<=product;state<=CENTER;end
                CENTER:begin
                    center[row]<=product_x+product_y;
                    if(row==2)begin row<=0;current_scale<=17'd65536;phase<=0;candidate_good<=1;state<=PRE_A;end
                    else begin row<=row+1'b1;state<=PRE_X;end
                end
                PRE_A:begin operand_a<=$signed(held_h[40*(row*3)+:40]);operand_b<=$signed({1'b0,current_scale});return_state<=SAVE_A;state<=MULTIPLY;end
                SAVE_A:begin
                    candidate[40*(row*3)+:40]<=rounded16[39:0];
                    if(rounded16>86'sd549755813887 || rounded16< -86'sd549755813888)candidate_good<=0;
                    state<=PRE_B;
                end
                PRE_B:begin operand_a<=$signed(held_h[40*(row*3+1)+:40]);operand_b<=$signed({1'b0,current_scale});return_state<=SAVE_B;state<=MULTIPLY;end
                SAVE_B:begin
                    candidate[40*(row*3+1)+:40]<=rounded16[39:0];
                    if(rounded16>86'sd549755813887 || rounded16< -86'sd549755813888)candidate_good<=0;
                    state<=PRE_C;
                end
                PRE_C:begin operand_a<=center[row];operand_b<=22'sd65536-$signed({1'b0,current_scale});return_state<=SUM_C;state<=MULTIPLY;end
                SUM_C:begin translation<=product+($signed({{46{original_offset[39]}},original_offset})<<<17);state<=SAVE_C;end
                SAVE_C:begin
                    candidate[40*(row*3+2)+:40]<=rounded17[39:0];
                    if(rounded17>86'sd549755813887 || rounded17< -86'sd549755813888)candidate_good<=0;
                    if(row==2)begin row<=0;state<=EDGE_X;end
                    else begin row<=row+1'b1;state<=PRE_A;end
                end
                EDGE_X:begin operand_a<=$signed(candidate[40*(row*3)+:40]);operand_b<=$signed({1'b0,w})-22'sd1;return_state<=SAVE_EDGE_X;state<=MULTIPLY;end
                SAVE_EDGE_X:begin edge_x[row]<=product;state<=EDGE_Y;end
                EDGE_Y:begin operand_a<=$signed(candidate[40*(row*3+1)+:40]);operand_b<=$signed({1'b0,h})-22'sd1;return_state<=SAVE_EDGE_Y;state<=MULTIPLY;end
                SAVE_EDGE_Y:begin
                    edge_y[row]<=product;
                    if(row==2)begin corner<=0;state<=DOT_PAIR;end
                    else begin row<=row+1'b1;state<=EDGE_X;end
                end
                DOT_PAIR:begin
                    for(k=0;k<3;k=k+1)partial[k]<=(corner[0]?edge_x[k]:64'sd0)+(corner[1]?edge_y[k]:64'sd0);
                    state<=DOT_OFFSET;
                end
                DOT_OFFSET:begin
                    for(k=0;k<3;k=k+1)dot[k]<=partial[k]+$signed(candidate[40*(k*3+2)+:40]);
                    bound_index<=0;state<=PRE_BOUND;
                end
                PRE_BOUND:begin
                    if(dot[2]<64'sd4194304)begin candidate_good<=0;state<=DECIDE;end
                    else begin
                        operand_a<=dot[2];operand_b<=box[bound_index];
                        lhs<=$signed({{22{dot[bound_index[1]][63]}},dot[bound_index[1]]})<<<20;
                        return_state<=COMPARE;state<=MULTIPLY;
                    end
                end
                COMPARE:begin
                    if((!bound_index[0] && lhs<product) || (bound_index[0] && lhs>product))candidate_good<=0;
                    if(bound_index==3)begin
                        if(corner==3)state<=DECIDE;
                        else begin corner<=corner+1'b1;state<=DOT_PAIR;end
                    end else begin bound_index<=bound_index+1'b1;state<=PRE_BOUND;end
                end
                DECIDE:begin
                    if(phase==0)begin
                        if(candidate_good)begin cropped_h<=candidate;scale_q16<=current_scale;busy<=0;done<=1;state<=IDLE;end
                        else begin current_scale<=17'd8192;phase<=1;row<=0;candidate_good<=1;state<=PRE_A;end
                    end else if(phase==1)begin
                        if(!candidate_good)begin error<=1;busy<=0;done<=1;state<=IDLE;end
                        else begin best_h<=candidate;best_scale<=current_scale;low<=17'd8192;high<=17'd65535;phase<=2;state<=CHOOSE;end
                    end else begin
                        if(candidate_good)begin best_h<=candidate;best_scale<=current_scale;low<=current_scale;end
                        else high<=current_scale-1'b1;
                        state<=CHOOSE;
                    end
                end
                CHOOSE:begin
                    if(low>=high)begin cropped_h<=best_h;scale_q16<=best_scale;busy<=0;done<=1;state<=IDLE;end
                    else begin current_scale<=({1'b0,low}+{1'b0,high}+18'd1)>>1;row<=0;candidate_good<=1;state<=PRE_A;end
                end
                default:begin error<=1;busy<=0;done<=1;state<=IDLE;end
            endcase
        end
    end
endmodule
