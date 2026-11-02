// Three elastic stages. One rounding after the four weighted samples.
module bilinear_rgb565 #(parameter TAG_W=44)(
    input wire clk,rst_n,
    input wire in_valid,output wire in_ready,
    input wire [15:0] p00,p10,p01,p11,
    input wire [7:0] ax,ay,
    input wire [TAG_W-1:0] in_tag,
    output wire out_valid,input wire out_ready,
    output wire [15:0] out_pixel,output wire [TAG_W-1:0] out_tag
);
    function automatic [7:0] red(input [15:0] p);red={p[15:11],p[15:13]};endfunction
    function automatic [7:0] green(input [15:0] p);green={p[10:5],p[10:9]};endfunction
    function automatic [7:0] blue(input [15:0] p);blue={p[4:0],p[4:2]};endfunction
    reg [2:0] v;
    wire ce=~v[2] | out_ready;
    reg [TAG_W-1:0] t0,t1,t2;
    reg [7:0] ay0;
    reg [16:0] r0,r1,g0,g1,b0,b1;
    reg [25:0] rr,gg,bb;
    reg [15:0] result;
    wire [8:0] bx=9'd256-{1'b0,ax};
    wire [8:0] by=9'd256-{1'b0,ay0};
    wire [25:0] rround=rr+26'd32768,ground=gg+26'd32768,bround=bb+26'd32768;
    assign in_ready=ce;assign out_valid=v[2];assign out_pixel=result;assign out_tag=t2;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin v<=0;result<=0;t0<=0;t1<=0;t2<=0;end
        else if(ce) begin
            v<={v[1:0],in_valid};t0<=in_tag;t1<=t0;t2<=t1;ay0<=ay;
            r0<=red(p00)*bx+red(p10)*{1'b0,ax};r1<=red(p01)*bx+red(p11)*{1'b0,ax};
            g0<=green(p00)*bx+green(p10)*{1'b0,ax};g1<=green(p01)*bx+green(p11)*{1'b0,ax};
            b0<=blue(p00)*bx+blue(p10)*{1'b0,ax};b1<=blue(p01)*bx+blue(p11)*{1'b0,ax};
            rr<=r0*by+r1*{1'b0,ay0};gg<=g0*by+g1*{1'b0,ay0};bb<=b0*by+b1*{1'b0,ay0};
            result<={rround[23:19],ground[23:18],bround[23:19]};
        end
    end
endmodule
