// Exact signed multiply with registered <=17-bit signed products, followed
// by two-stage short-limb adders. LATENCY=2+2*$clog2(NA*NB) enabled clocks.
module multiply_operand_pipe_v2 #(parameter A_W=32, B_W=32)(
    input wire clk,ce,input wire signed[A_W-1:0] a,
    input wire signed[B_W-1:0] b,output wire signed[A_W+B_W-1:0] product
);
    localparam NA=(A_W+15)/16,NB=(B_W+15)/16;
    localparam LEVELS=$clog2(NA*NB),LEAVES=1<<LEVELS,W=A_W+B_W;
    (* syn_preserve = 1, syn_keep = 1 *) reg signed[16:0] ac[0:NA-1],bc[0:NB-1];
    wire[W-1:0] tree[0:LEVELS][0:LEAVES-1];
    genvar i,j,l;
    generate
        for(i=0;i<NA;i=i+1) begin: a_chunk
            always @(posedge clk) if(ce) begin
                if(i==NA-1) ac[i]<=$signed(a[A_W-1:i*16]);
                else ac[i]<=$signed({1'b0,a[i*16+:16]});
            end
        end
        for(i=0;i<NB;i=i+1) begin: b_chunk
            always @(posedge clk) if(ce) begin
                if(i==NB-1) bc[i]<=$signed(b[B_W-1:i*16]);
                else bc[i]<=$signed({1'b0,b[i*16+:16]});
            end
        end
        for(i=0;i<LEAVES;i=i+1) begin: leaf
            reg[W-1:0] value;
            if(i<NA*NB) begin
                wire signed[33:0] partial_product=ac[i/NB]*bc[i%NB];
                always @(posedge clk) if(ce)
                    value<=$signed({{(W-34){partial_product[33]}},partial_product}) << (16*((i/NB)+(i%NB)));
            end else always @(posedge clk) if(ce) value<=0;
            assign tree[0][i]=value;
        end
        for(l=1;l<=LEVELS;l=l+1) begin: level
            for(j=0;j<(LEAVES>>l);j=j+1) begin: node
                add_pipeline_v2 #(.W(W)) adder(.clk(clk),.ce(ce),.a(tree[l-1][2*j]),.b(tree[l-1][2*j+1]),.sum(tree[l][j]));
            end
        end
    endgenerate
    assign product=tree[LEVELS][0];
endmodule
