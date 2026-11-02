// Explicit v2 six-stage luminance LUT: partial weights, Y, lookup, delta,
// channel add, saturate. Global elastic freeze includes pixel metadata.
module luminance_lut_pipelined_v2 #(parameter TAG_W=32)(
    input wire clk,rst_n,input wire lut_wr_valid,output wire lut_wr_ready,
    input wire[7:0] lut_wr_addr,lut_wr_data,input wire lut_commit,
    input wire frame_start,frame_done,frame_enable,
    output reg frame_active,pending,active_lut_valid,error,output reg[31:0] active_lut_version,
    input wire in_valid,output wire in_ready,input wire[15:0] in_rgb565,
    input wire in_inside,input wire[TAG_W-1:0] in_tag,
    output wire out_valid,input wire out_ready,output reg[15:0] out_rgb565,output reg[TAG_W-1:0] out_tag
);
    reg[7:0] lut[0:511];reg active_bank,enabled;reg[255:0] written;
    reg[5:0] v,inside_pipe,apply_pipe;
    reg[15:0] rgb[0:4];reg[TAG_W-1:0] tag[0:4];
    reg[15:0] rg0,b0;reg[7:0] y1,y2,mapped2;reg signed[9:0] delta3;
    reg signed[10:0] rv4,gv4,bv4;
    wire[7:0] r={in_rgb565[15:11],in_rgb565[15:13]};
    wire[7:0] g={in_rgb565[10:5],in_rgb565[10:9]};
    wire[7:0] b={in_rgb565[4:0],in_rgb565[4:2]};
    wire[7:0] r3={rgb[3][15:11],rgb[3][15:13]},g3={rgb[3][10:5],rgb[3][10:9]},b3={rgb[3][4:0],rgb[3][4:2]};
    wire ce=!v[5] || out_ready;
    function automatic[7:0] sat(input signed[10:0] value);
        begin sat=value<0 ? 0 : value>255 ? 255 : value[7:0];end
    endfunction
    wire[7:0] rr=sat(rv4),gg=sat(gv4),bb=sat(bv4);
    assign out_valid=v[5];assign in_ready=frame_active && ce;assign lut_wr_ready=!pending;
    integer i;
    always @(posedge clk) if(ce) begin
        rg0<=r*16'd77+g*16'd150;b0<=b*16'd29+16'd128;
        y1<=(rg0+b0)>>8;y2<=y1;mapped2<=lut[{active_bank,y1}];
        delta3<=$signed({2'b0,mapped2})-$signed({2'b0,y2});
        rv4<=$signed({3'b0,r3})+delta3;gv4<=$signed({3'b0,g3})+delta3;bv4<=$signed({3'b0,b3})+delta3;
        rgb[0]<=in_rgb565;tag[0]<=in_tag;
        for(i=1;i<5;i=i+1) begin rgb[i]<=rgb[i-1];tag[i]<=tag[i-1];end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            active_bank<=0;enabled<=0;written<=0;frame_active<=0;pending<=0;
            active_lut_valid<=0;active_lut_version<=0;error<=0;v<=0;inside_pipe<=0;apply_pipe<=0;
            out_rgb565<=0;out_tag<=0;
        end else begin
            if(lut_wr_valid && lut_wr_ready) begin lut[{!active_bank,lut_wr_addr}]<=lut_wr_data;written[lut_wr_addr]<=1;end
            if(lut_commit && !pending) begin if(&written) pending<=1;else error<=1;end
            if(frame_done) frame_active<=0;
            if(frame_start && !frame_active && !(|v)) begin
                frame_active<=1;enabled<=frame_enable;
                if(pending) begin active_bank<=!active_bank;active_lut_valid<=1;active_lut_version<=active_lut_version+1'b1;pending<=0;written<=0;end
            end
            if(ce) begin
                v<={v[4:0],in_valid && frame_active};
                inside_pipe<={inside_pipe[4:0],in_inside};apply_pipe<={apply_pipe[4:0],enabled && active_lut_valid};
                if(v[4]) begin
                    out_tag<=tag[4];
                    out_rgb565<=!inside_pipe[4] ? 0 : !apply_pipe[4] ? rgb[4] : {rr[7:3],gg[7:2],bb[7:3]};
                end
            end
        end
    end
endmodule
