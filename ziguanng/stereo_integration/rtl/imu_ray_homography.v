`timescale 1ns/1ps
// Camera x right/y down/z forward. q maps body to reference world.
// H=R(conj(current)*reference)*K^-1, signed40 Q30.
// Address, operands, product, accumulation and rounding use separate cycles.
module imu_ray_homography(
    input wire clk,rst_n,start,
    input wire [127:0] current_q,reference_q,
    input wire [359:0] inverse_k,
    input wire [11:0] width,height,
    output reg busy,done,error,
    output reg [359:0] homography
);
    // Four registered 16x20 partial products.  The first register boundary
    // keeps every DSP-to-FF path narrower than the original 32x40 multiply;
    // two explicit 72-bit adder stages then form the exact signed product.
    reg signed [31:0] mul_a,pair_a,pair_b;
    reg signed [39:0] mul_b;
    wire mul_busy,mul_valid;
    wire signed [71:0] mul_product;
    reg mul_start;
    signed_mul_32x40_pipe builder_multiplier(
        .clk(clk),.rst_n(rst_n),.start(mul_start),.a(mul_a),.b(mul_b),
        .busy(mul_busy),.valid(mul_valid),.product(mul_product));
    localparam IDLE=0,REL_SELECT=1,REL_MUL=2,REL_SIGN=3,REL_ACC=4,REL_ROUND=5,
        PAIR_SELECT=6,PAIR_MUL=7,PAIR_ROUND=8,ROTATION=9,MATRIX_INDEX=10,
        MATRIX_SELECT=11,MATRIX_MUL=12,MATRIX_ACC=13,MATRIX_ROUND=14,
        CHECK_MUL=15,CHECK_ADD=16,CHECK_SUM=17,CHECK=18;
    reg [4:0] state;
    reg signed [31:0] qc[0:3],qr[0:3],relative_q[0:3],products[0:9],rotation[0:8];
    reg [359:0] ki;
    reg [11:0] w,h;
    reg [1:0] qrow,term,hrow,hcol;
    reg [3:0] pair_index,rotation_index,ki_index;
    reg signed [79:0] accumulator,signed_product;
    reg signed [71:0] product;
    reg negate_product;
    reg signed [63:0] center_x,center_y,center_partial,center_denominator;
    reg signed [63:0] norm_a,norm_b,relative_norm;
    wire [1:0] ref_index=term^qrow;
    wire negative_term=(qrow==1 && (term==1 || term==2)) ||
        (qrow==2 && (term==2 || term==3)) || (qrow==3 && (term==1 || term==3));
    wire signed [39:0] h20=homography[240+:40],h21=homography[280+:40],h22=homography[320+:40];
    // Arithmetic floor plus correction: nearest, ties away from zero.
    // This avoids the old abs/add/negate carry-chain cascade.
    function signed [79:0] round_q30;
        input signed [79:0] value;
        begin
            round_q30=(value>>>30)+((value[29] && ((|value[28:0]) || !value[79]))?80'sd1:80'sd0);
        end
    endfunction
    wire signed [79:0] rounded_accumulator=round_q30(accumulator);
    always @* begin
        pair_a=relative_q[1];pair_b=relative_q[1];
        case(pair_index)
            0:begin pair_a=relative_q[1];pair_b=relative_q[1];end
            1:begin pair_a=relative_q[2];pair_b=relative_q[2];end
            2:begin pair_a=relative_q[3];pair_b=relative_q[3];end
            3:begin pair_a=relative_q[1];pair_b=relative_q[2];end
            4:begin pair_a=relative_q[1];pair_b=relative_q[3];end
            5:begin pair_a=relative_q[2];pair_b=relative_q[3];end
            6:begin pair_a=relative_q[1];pair_b=relative_q[0];end
            7:begin pair_a=relative_q[2];pair_b=relative_q[0];end
            8:begin pair_a=relative_q[3];pair_b=relative_q[0];end
            9:begin pair_a=relative_q[0];pair_b=relative_q[0];end
        endcase
    end
    integer j;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;busy<=0;done<=0;error<=0;homography<=0;ki<=0;w<=0;h<=0;
            accumulator<=0;signed_product<=0;product<=0;mul_a<=0;mul_b<=0;mul_start<=0;negate_product<=0;
            qrow<=0;term<=0;pair_index<=0;hrow<=0;hcol<=0;rotation_index<=0;ki_index<=0;
            center_x<=0;center_y<=0;center_partial<=0;center_denominator<=0;norm_a<=0;norm_b<=0;relative_norm<=0;
            for(j=0;j<4;j=j+1) begin qc[j]<=0;qr[j]<=0;relative_q[j]<=0;end
            for(j=0;j<9;j=j+1) begin products[j]<=0;rotation[j]<=0;end
            products[9]<=0;
        end else begin
            done<=0;
            mul_start<=0;
            case(state)
                IDLE:if(start) begin
                    for(j=0;j<4;j=j+1) begin qc[j]<=$signed(current_q[j*32+:32]);qr[j]<=$signed(reference_q[j*32+:32]);end
                    ki<=inverse_k;w<=width;h<=height;error<=0;busy<=1;
                    accumulator<=0;qrow<=0;term<=0;state<=REL_SELECT;
                end
                REL_SELECT:begin mul_a<=qc[term];mul_b<=qr[ref_index];negate_product<=negative_term;mul_start<=1;state<=REL_MUL;end
                REL_MUL:if(mul_valid) begin product<=mul_product;state<=REL_SIGN;end
                REL_SIGN:begin signed_product<=negate_product?-$signed(product):$signed(product);state<=REL_ACC;end
                REL_ACC:begin
                    accumulator<=accumulator+signed_product;
                    if(term==3) state<=REL_ROUND;
                    else begin term<=term+1'b1;state<=REL_SELECT;end
                end
                REL_ROUND:begin
                    relative_q[qrow]<=rounded_accumulator;accumulator<=0;term<=0;
                    if(qrow==3) begin pair_index<=0;state<=PAIR_SELECT;end
                    else begin qrow<=qrow+1'b1;state<=REL_SELECT;end
                end
                PAIR_SELECT:begin mul_a<=pair_a;mul_b<=pair_b;mul_start<=1;state<=PAIR_MUL;end
                PAIR_MUL:if(mul_valid) begin product<=mul_product;state<=PAIR_ROUND;end
                PAIR_ROUND:begin
                    products[pair_index]<=round_q30($signed(product));
                    if(pair_index==9) state<=ROTATION;
                    else begin pair_index<=pair_index+1'b1;state<=PAIR_SELECT;end
                end
                ROTATION:begin
                    rotation[0]<=32'sd1073741824-((products[1]+products[2])<<<1);
                    rotation[1]<=(products[3]-products[8])<<<1;
                    rotation[2]<=(products[4]+products[7])<<<1;
                    rotation[3]<=(products[3]+products[8])<<<1;
                    rotation[4]<=32'sd1073741824-((products[0]+products[2])<<<1);
                    rotation[5]<=(products[5]-products[6])<<<1;
                    rotation[6]<=(products[4]-products[7])<<<1;
                    rotation[7]<=(products[5]+products[6])<<<1;
                    rotation[8]<=32'sd1073741824-((products[0]+products[1])<<<1);
                    accumulator<=0;hrow<=0;hcol<=0;term<=0;state<=MATRIX_INDEX;
                end
                MATRIX_INDEX:begin
                    case(hrow)
                        0:rotation_index<={2'b00,term};
                        1:rotation_index<=4'd3+term;
                        default:rotation_index<=4'd6+term;
                    endcase
                    case(term)
                        0:ki_index<={2'b00,hcol};
                        1:ki_index<=4'd3+hcol;
                        default:ki_index<=4'd6+hcol;
                    endcase
                    state<=MATRIX_SELECT;
                end
                MATRIX_SELECT:begin mul_a<=rotation[rotation_index];mul_b<=ki[40*ki_index+:40];mul_start<=1;state<=MATRIX_MUL;end
                MATRIX_MUL:if(mul_valid) begin product<=mul_product;state<=MATRIX_ACC;end
                MATRIX_ACC:begin
                    accumulator<=accumulator+$signed(product);
                    if(term==2) state<=MATRIX_ROUND;
                    else begin term<=term+1'b1;state<=MATRIX_INDEX;end
                end
                MATRIX_ROUND:begin
                    homography[40*(hrow*3+hcol)+:40]<=rounded_accumulator[39:0];
                    if(rounded_accumulator>80'sd549755813887 || rounded_accumulator< -80'sd549755813888) error<=1;
                    accumulator<=0;term<=0;
                    if(hcol==2) begin
                        hcol<=0;
                        if(hrow==2) state<=CHECK_MUL;
                        else begin hrow<=hrow+1'b1;state<=MATRIX_INDEX;end
                    end else begin hcol<=hcol+1'b1;state<=MATRIX_INDEX;end
                end
                CHECK_MUL:begin
                    center_x<=$signed(h20)*$signed({1'b0,(w-1'b1)});
                    center_y<=$signed(h21)*$signed({1'b0,(h-1'b1)});
                    norm_a<=$signed(products[0])+$signed(products[1]);
                    norm_b<=$signed(products[2])+$signed(products[9]);state<=CHECK_ADD;
                end
                CHECK_ADD:begin center_partial<=center_x+center_y;relative_norm<=norm_a+norm_b;state<=CHECK_SUM;end
                CHECK_SUM:begin center_denominator<=center_partial+($signed(h22)<<<1);state<=CHECK;end
                CHECK:begin
                    if(center_denominator<64'sd8388608 || relative_norm<64'sd966367642 || relative_norm>64'sd1181116006 || w<2 || h<2) error<=1;
                    busy<=0;done<=1;state<=IDLE;
                end
                default:begin busy<=0;error<=1;done<=1;state<=IDLE;end
            endcase
        end
    end
endmodule

module signed_mul_32x40_pipe(
    input wire clk,rst_n,start,
    input wire signed [31:0] a,
    input wire signed [39:0] b,
    output wire busy,valid,
    output wire signed [71:0] product
);
    localparam IDLE=2'd0,PARTIALS=2'd1,PAIR_SUMS=2'd2,FINAL_SUM=2'd3;
    reg [1:0] state;
    reg [35:0] p00;
    reg signed [35:0] p01,p10,p11;
    reg signed [71:0] sum0,sum1;
    wire [15:0] a_low=a[15:0];
    wire signed [15:0] a_high=a[31:16];
    wire [19:0] b_low=b[19:0];
    wire signed [19:0] b_high=b[39:20];
    wire signed [71:0] p00_extended=$signed({36'b0,p00});
    wire signed [71:0] p01_extended={{36{p01[35]}},p01};
    wire signed [71:0] p10_extended={{36{p10[35]}},p10};
    wire signed [71:0] p11_extended={{36{p11[35]}},p11};
    assign busy=(state!=IDLE);
    assign valid=(state==FINAL_SUM);
    assign product=sum0+sum1;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;p00<=0;p01<=0;p10<=0;p11<=0;sum0<=0;sum1<=0;
        end else begin
            case(state)
                IDLE:if(start) begin
                    // a = a_low + (a_high << 16), b = b_low + (b_high << 20)
                    p00<=a_low*b_low;
                    p01<=$signed({1'b0,a_low})*$signed(b_high);
                    p10<=a_high*$signed({1'b0,b_low});
                    p11<=a_high*b_high;
                    state<=PARTIALS;
                end
                PARTIALS:state<=PAIR_SUMS;
                PAIR_SUMS:begin
                    sum0<=p00_extended+(p01_extended<<<20);
                    sum1<=(p10_extended<<<16)+(p11_extended<<<36);
                    state<=FINAL_SUM;
                end
                FINAL_SUM:state<=IDLE;
                default:state<=IDLE;
            endcase
        end
    end
endmodule
