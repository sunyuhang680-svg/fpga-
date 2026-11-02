// Explicit v2 retimed homogeneous normalization, 42 enabled stages, II=1.
// Exact signed53 affine sums and the original26 restoring fractional bits.
// All data, valid and tags freeze together. Configuration is frame-stable.
module projective_normalizer_pipelined_v2 #(parameter TAG_W=44)(
    input wire clk,rst_n,input wire in_valid,output wire in_ready,
    input wire[11:0] in_x,in_y,input wire[TAG_W-1:0] in_tag,
    input wire[359:0] homography_flat,
    output wire out_valid,input wire out_ready,
    output reg signed[31:0] out_x,out_y,output wire out_domain,
    output wire[TAG_W-1:0] out_tag
);
    wire signed[39:0] h[0:8];genvar g;
    generate for(g=0;g<9;g=g+1) begin:H assign h[g]=homography_flat[g*40+:40];end endgenerate
    reg[41:0] valid,domain;reg[TAG_W-1:0] tag[0:41];
    wire ce=!valid[41] || out_ready;
    reg signed[12:0] x0,y0;
    wire signed[52:0] product[0:5],pair_sum[0:2],affine[0:2];
    generate for(g=0;g<3;g=g+1) begin:AFFINE
        multiply_balanced_v2 #(.A_W(40),.B_W(13)) mx(.clk(clk),.ce(ce),.a(h[g*3]),.b(x0),.product(product[g*2]));
        multiply_balanced_v2 #(.A_W(40),.B_W(13)) my(.clk(clk),.ce(ce),.a(h[g*3+1]),.b(y0),.product(product[g*2+1]));
        add_pipeline_v2 #(.W(53)) add_products(.clk(clk),.ce(ce),.a(product[g*2]),.b(product[g*2+1]),.sum(pair_sum[g]));
        add_pipeline_v2 #(.W(53)) add_constant(.clk(clk),.ce(ce),.a(pair_sum[g]),.b({{13{h[g*3+2][39]}},h[g*3+2]}),.sum(affine[g]));
    end endgenerate
    // affine is stage9: input register +5 multiply +2 product sum +2 offset.
    reg signed[52:0] ax10,ay10,d10,ax11,ay11,d11;
    reg signx10,signy10,signx11,signy11;
    reg x_ge_two11,x_ge_one11,y_ge_two11,y_ge_one11;
    wire domain11=d10>=53'sd4194304 && ax10<=($signed(d10)<<<1) && ay10<=($signed(d10)<<<1);
    reg[52:0] den[12:38],rx[12:38],ry[12:38];
    reg[27:0] qx[12:38],qy[12:38];reg sx[12:38],sy[12:38];
    reg increment_x39,increment_y39,sx39,sy39,sx40,sy40;
    reg[27:0] qx39,qy39;reg[28:0] rounded_x40,rounded_y40;
    assign in_ready=ce;assign out_valid=valid[41];assign out_domain=domain[41];assign out_tag=tag[41];
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            valid<=0;domain<=0;out_x<=0;out_y<=0;
            for(i=0;i<42;i=i+1) tag[i]<=0;
        end else if(ce) begin
            valid<={valid[40:0],in_valid};domain[0]<=1;tag[0]<=in_tag;
            for(i=1;i<42;i=i+1) begin tag[i]<=tag[i-1];domain[i]<=domain[i-1];end
            domain[11]<=domain11;
            out_x<=sx40 ? -$signed({3'b0,rounded_x40}) : $signed({3'b0,rounded_x40});
            out_y<=sy40 ? -$signed({3'b0,rounded_y40}) : $signed({3'b0,rounded_y40});
        end
    end
    // Datapath reset is unnecessary: cleared valid bits hide old contents.
    always @(posedge clk) if(ce) begin
        x0<=$signed({1'b0,in_x});y0<=$signed({1'b0,in_y});
        ax10<=affine[0]<0 ? -affine[0] : affine[0];ay10<=affine[1]<0 ? -affine[1] : affine[1];
        d10<=affine[2];signx10<=affine[0]<0;signy10<=affine[1]<0;
        ax11<=ax10;ay11<=ay10;d11<=d10;signx11<=signx10;signy11<=signy10;
        x_ge_two11<=ax10>=($signed(d10)<<<1);x_ge_one11<=ax10>=d10;
        y_ge_two11<=ay10>=($signed(d10)<<<1);y_ge_one11<=ay10>=d10;
        den[12]<=domain[11] ? d11 : 53'd1073741824;
        qx[12]<=!domain[11] ? 0 : x_ge_two11 ? 2 : x_ge_one11 ? 1 : 0;
        qy[12]<=!domain[11] ? 0 : y_ge_two11 ? 2 : y_ge_one11 ? 1 : 0;
        rx[12]<=!domain[11] ? 0 : x_ge_two11 ? ax11-($signed(d11)<<<1) : x_ge_one11 ? ax11-d11 : ax11;
        ry[12]<=!domain[11] ? 0 : y_ge_two11 ? ay11-($signed(d11)<<<1) : y_ge_one11 ? ay11-d11 : ay11;
        sx[12]<=domain[11] && signx11;sy[12]<=domain[11] && signy11;
        for(i=13;i<=38;i=i+1) begin
            den[i]<=den[i-1];sx[i]<=sx[i-1];sy[i]<=sy[i-1];
            qx[i]<=(qx[i-1]<<1) | ((rx[i-1]<<1)>=den[i-1]);
            qy[i]<=(qy[i-1]<<1) | ((ry[i-1]<<1)>=den[i-1]);
            rx[i]<=((rx[i-1]<<1)>=den[i-1]) ? (rx[i-1]<<1)-den[i-1] : rx[i-1]<<1;
            ry[i]<=((ry[i-1]<<1)>=den[i-1]) ? (ry[i-1]<<1)-den[i-1] : ry[i-1]<<1;
        end
        increment_x39<=({1'b0,rx[38]}<<1)>={1'b0,den[38]};
        increment_y39<=({1'b0,ry[38]}<<1)>={1'b0,den[38]};
        qx39<=qx[38];qy39<=qy[38];sx39<=sx[38];sy39<=sy[38];
        rounded_x40<={1'b0,qx39}+increment_x39;rounded_y40<={1'b0,qy39}+increment_y39;
        sx40<=sx39;sy40<=sy39;
    end
endmodule
