// New v2 coordinate core. Existing v1 map_pipeline ports/behavior are unchanged.
// Normalize H first, then reuse Brown-Conrady stages, before a single sampler.
module projective_map_operand_v2 #(parameter TAG_W=44)(
    input wire clk,rst_n,
    input wire in_valid,output wire in_ready,
    input wire [11:0] in_x,in_y,input wire [TAG_W-1:0] in_tag,
    input wire [359:0] homography_flat,
    input wire signed [31:0] k1,k2,p1,p2,fx,fy,cx,cy,
    input wire [11:0] width,height,
    output wire out_valid,input wire out_ready,
    output wire signed [31:0] out_x,out_y,
    output wire out_inside,out_domain_error,
    output wire [TAG_W-1:0] out_tag
);
    wire norm_valid,norm_ready,norm_domain;
    wire signed [31:0] nx,ny;
    wire [TAG_W-1:0] norm_tag;
    wire [TAG_W:0] mapped_tag;
    wire mapped_inside,mapped_error;
    wire signed [39:0] constant_x=$signed({{8{nx[31]}},nx})<<<4;
    wire signed [39:0] constant_y=$signed({{8{ny[31]}},ny})<<<4;
    projective_normalizer_pipelined_v2 #(.TAG_W(TAG_W)) norm(
        .clk(clk),.rst_n(rst_n),.in_valid(in_valid),.in_ready(in_ready),
        .in_x(in_x),.in_y(in_y),.in_tag(in_tag),.homography_flat(homography_flat),
        .out_valid(norm_valid),.out_ready(norm_ready),.out_x(nx),.out_y(ny),
        .out_domain(norm_domain),.out_tag(norm_tag));
    distortion_operand_v2 #(.TAG_W(TAG_W+1)) distortion(
        .clk(clk),.rst_n(rst_n),.in_valid(norm_valid),.in_ready(norm_ready),
        .in_x(nx),.in_y(ny),.in_tag({norm_domain,norm_tag}),
        .k1(k1),.k2(k2),.p1(p1),.p2(p2),.fx(fx),.fy(fy),.cx(cx),.cy(cy),
        .width(width),.height(height),.out_valid(out_valid),.out_ready(out_ready),
        .out_x(out_x),.out_y(out_y),.out_inside(mapped_inside),
        .out_domain_error(mapped_error),.out_tag(mapped_tag));
    assign out_tag=mapped_tag[TAG_W-1:0];
    assign out_inside=mapped_inside && mapped_tag[TAG_W];
    assign out_domain_error=mapped_error || !mapped_tag[TAG_W];
endmodule
