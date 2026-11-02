// Two-slot ready/valid buffer. Input ready depends only on a register.
// No combinational downstream-ready path to the upstream arithmetic CE tree.
module coord_register_slice_v2 #(parameter WIDTH=109)(
    input wire clk,rst_n,
    input wire in_valid,output wire in_ready,input wire[WIDTH-1:0] in_data,
    output wire out_valid,input wire out_ready,output wire[WIDTH-1:0] out_data
);
    reg head_valid,skid_valid;
    reg[WIDTH-1:0] head_data,skid_data;
    assign in_ready=!skid_valid;
    assign out_valid=head_valid;
    assign out_data=head_data;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin head_valid<=0;skid_valid<=0;end
        else if(out_ready || !head_valid) begin
            if(skid_valid) begin
                head_data<=skid_data;head_valid<=1;skid_valid<=0;
            end else begin
                head_data<=in_data;head_valid<=in_valid;
            end
        end else if(in_valid && in_ready) begin
            skid_data<=in_data;skid_valid<=1;
        end
    end
endmodule
