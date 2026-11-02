// Atomic 672-bit geometry + enhancement snapshot. Bits 640..651: mode and manual Q8; remaining bits zero.
// Explicit v2 atomic frame config. Packed layout: H[359:0], coefficients and
// intrinsics [615:360], width[627:616], height[639:628]. Direct core port,
// NOT the v1 register bus. The caller must hold submit_flat when backpressured.
module processing_bank_validated_v2(
    input wire clk,rst_n,
    input wire submit_valid,output wire submit_ready,input wire [671:0] submit_flat,
    input wire frame_start,frame_done,
    output reg start_accepted,busy,pending,config_valid,error,
    output reg [671:0] active_flat
);
    reg [671:0] snapshot;
    wire [11:0] w=submit_flat[616+:12],h=submit_flat[628+:12];
    wire signed [31:0] k1=submit_flat[360+:32],k2=submit_flat[392+:32];
    wire signed [31:0] p1=submit_flat[424+:32],p2=submit_flat[456+:32];
    wire signed [31:0] fx=submit_flat[488+:32],fy=submit_flat[520+:32];
    wire signed [31:0] cx=submit_flat[552+:32],cy=submit_flat[584+:32];
    wire [1:0] mode=submit_flat[640+:2];
    wire [9:0] manual_gamma=submit_flat[642+:10];
    wire [5:0] group_ok={
        (mode<=2 && manual_gamma>=128 && manual_gamma<=512 && submit_flat[671:652]==0),
        (w>=16 && w<=1920 && w[3:0]==0 && h>=2 && h<=1080),
        (fx>=32'sd65536 && fx<=32'sd536870912 && fy>=32'sd65536 && fy<=32'sd536870912),
        (k1>=-32'sd536870912 && k1<=32'sd536870912 && k2>=-32'sd536870912 && k2<=32'sd536870912),
        (p1>=-32'sd67108864 && p1<=32'sd67108864 && p2>=-32'sd67108864 && p2<=32'sd67108864),
        (cx>=0 && cx<($signed({1'b0,w})<<<12) && cy>=0 && cy<($signed({1'b0,h})<<<12))
    };
    reg [5:0] valid_groups;
    wire snapshot_ok=&valid_groups;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) valid_groups<=0;
        else if(submit_valid && submit_ready) valid_groups<=group_ok;
    end
    assign submit_ready=!pending;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            snapshot<=0;active_flat<=0;start_accepted<=0;busy<=0;pending<=0;config_valid<=0;error<=0;
        end else begin
            start_accepted<=0;
            if(submit_valid && submit_ready) begin snapshot<=submit_flat;pending<=1;end
            if(frame_done) busy<=0;
            if(frame_start && !busy) begin
                if(pending) begin
                    pending<=0;config_valid<=snapshot_ok;
                    if(snapshot_ok) begin active_flat<=snapshot;start_accepted<=1;busy<=1;end
                    else error<=1;
                end else if(config_valid) begin start_accepted<=1;busy<=1;end
            end
        end
    end
endmodule
