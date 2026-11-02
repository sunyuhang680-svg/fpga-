// Exact modulo-2**W sum in two enabled stages. First compute independent
// <=16-bit limb sums; then a short carry prefix and limb increments.
// No datapath reset; the caller owns valid/metadata and freezes both stages.
module add_pipeline_v2 #(parameter W=72)(
    input wire clk,ce,input wire[W-1:0] a,b,output wire[W-1:0] sum
);
    localparam CHUNKS=(W+15)/16;
    reg[16:0] partial_sum[0:CHUNKS-1];
    wire[CHUNKS:0] carry;
    assign carry[0]=0;
    genvar i;
    generate for(i=0;i<CHUNKS;i=i+1) begin: limb
        localparam BITS=(i==CHUNKS-1) ? W-i*16 : 16;
        reg[BITS-1:0] result;
        always @(posedge clk) if(ce) begin
            partial_sum[i]<={1'b0,a[i*16+:BITS]}+{1'b0,b[i*16+:BITS]};
            result<=partial_sum[i][BITS-1:0]+carry[i];
        end
        assign carry[i+1]=partial_sum[i][16] || ((&partial_sum[i][15:0]) && carry[i]);
        assign sum[i*16+:BITS]=result;
    end endgenerate
endmodule
