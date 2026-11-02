// Explicit v2 address preparation and pixel selection pipeline. Same cache tag
// layout and bilinear samples as v1. Frame dimensions/base remain stable.
// Five elastic preparation stages split round/edge/multiply/add/address logic.
module pixel_sampler_select_v2 #(parameter TAG_W=44)(
    input wire clk,rst_n,input wire in_valid,output wire in_ready,
    input wire[31:0] sx,sy,input wire map_inside,input wire[TAG_W-1:0] in_tag,
    input wire[11:0] width,height,input wire[31:0] src_base,
    output wire cache_valid,input wire cache_ready,output wire[31:0] cache_addr_a,cache_addr_b,
    output wire cache_bypass,output wire[TAG_W+35:0] cache_tag,
    input wire response_valid,output wire response_ready,input wire[255:0] data_a,data_b,
    input wire[TAG_W+35:0] response_tag,
    output reg pixel_valid,input wire pixel_ready,output reg[15:0] p00,p10,p01,p11,
    output reg[7:0] ax,ay,output reg[TAG_W-1:0] pixel_tag
);
    reg[4:0] prepared,inside_pipe;
    reg[TAG_W-1:0] tag_pipe[0:4];
    reg[7:0] alpha_x[0:4],alpha_y[0:4];
    reg[19:0] qx0,qy0;
    reg[11:0] x0_1,x1_1,y0_1,y1_1,x0_2,x1_2;
    reg[23:0] row0_2,row1_2;
    reg[24:0] ia0_3,ia1_3,ib0_3,ib1_3;
    reg[31:0] aa0,aa1,bb0,bb1;
    reg second;
    reg[31:0] next_a,next_b;
    reg[TAG_W+35:0] next_tag;
    wire dispatch_ready=!second && cache_ready;
    wire ce=!prepared[4] || dispatch_ready;
    wire[31:0] rounded_x=(sx+32'd8)>>4,rounded_y=(sy+32'd8)>>4;
    wire[11:0] first_x=qx0[19:8],first_y=qy0[19:8];
    wire cross_line=inside_pipe[4] && (aa0[31:5]!=aa1[31:5] || bb0[31:5]!=bb1[31:5]);
    wire[TAG_W+35:0] first_tag={2'b0,(cross_line ? 2'd1 : 2'd0),bb1[4:1],aa1[4:1],bb0[4:1],aa0[4:1],alpha_y[4],alpha_x[4],tag_pipe[4]};
    assign in_ready=ce;
    assign cache_valid=second || prepared[4];
    assign cache_addr_a=second ? next_a : {aa0[31:5],5'b0};
    assign cache_addr_b=second ? next_b : {bb0[31:5],5'b0};
    assign cache_tag=second ? next_tag : first_tag;
    assign cache_bypass=!second && !inside_pipe[4];
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            prepared<=0;inside_pipe<=0;qx0<=0;qy0<=0;
            x0_1<=0;x1_1<=0;y0_1<=0;y1_1<=0;x0_2<=0;x1_2<=0;row0_2<=0;row1_2<=0;
            ia0_3<=0;ia1_3<=0;ib0_3<=0;ib1_3<=0;aa0<=0;aa1<=0;bb0<=0;bb1<=0;
            for(i=0;i<5;i=i+1) begin tag_pipe[i]<=0;alpha_x[i]<=0;alpha_y[i]<=0;end
        end else if(ce) begin
            prepared<={prepared[3:0],in_valid};inside_pipe<={inside_pipe[3:0],map_inside};
            tag_pipe[0]<=in_tag;alpha_x[0]<=rounded_x[7:0];alpha_y[0]<=rounded_y[7:0];
            for(i=1;i<5;i=i+1) begin tag_pipe[i]<=tag_pipe[i-1];alpha_x[i]<=alpha_x[i-1];alpha_y[i]<=alpha_y[i-1];end
            qx0<=rounded_x[19:0];qy0<=rounded_y[19:0];
            x0_1<=first_x;y0_1<=first_y;
            x1_1<=first_x==width-1 ? first_x : first_x+1'b1;
            y1_1<=first_y==height-1 ? first_y : first_y+1'b1;
            row0_2<=y0_1*width;row1_2<=y1_1*width;x0_2<=x0_1;x1_2<=x1_1;
            ia0_3<=row0_2+x0_2;ia1_3<=row0_2+x1_2;ib0_3<=row1_2+x0_2;ib1_3<=row1_2+x1_2;
            aa0<=src_base+({7'b0,ia0_3}<<1);aa1<=src_base+({7'b0,ia1_3}<<1);
            bb0<=src_base+({7'b0,ib0_3}<<1);bb1<=src_base+({7'b0,ib1_3}<<1);
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin second<=0;next_a<=0;next_b<=0;next_tag<=0;end
        else begin
            if(second && cache_ready) second<=0;
            if(prepared[4] && dispatch_ready && cross_line) begin
                second<=1;next_a<={aa1[31:5],5'b0};next_b<={bb1[31:5],5'b0};
                next_tag<={2'b0,2'd2,first_tag[TAG_W+31:0]};
            end
        end
    end
    // Register a four-way64-bit group before the four-way16-bit word.
    reg select_valid0,select_valid1;
    reg[63:0] group00,group10,group01,group11;
    reg[15:0] selected00,selected10,selected01,selected11,saved00,saved01;
    reg[TAG_W+35:0] select_tag0,select_tag1;
    wire[3:0] ia0=response_tag[TAG_W+16+:4],ib0=response_tag[TAG_W+20+:4];
    wire[3:0] ia1=response_tag[TAG_W+24+:4],ib1=response_tag[TAG_W+28+:4];
    wire[1:0] phase=select_tag1[TAG_W+32+:2];
    wire pixel_ce=!pixel_valid || pixel_ready;
    assign response_ready=pixel_ce;
    always @(posedge clk) if(pixel_ce) begin
        group00<=data_a[ia0[3:2]*64+:64];group10<=data_a[ia1[3:2]*64+:64];
        group01<=data_b[ib0[3:2]*64+:64];group11<=data_b[ib1[3:2]*64+:64];
        selected00<=group00[select_tag0[TAG_W+16+:2]*16+:16];
        selected10<=group10[select_tag0[TAG_W+24+:2]*16+:16];
        selected01<=group01[select_tag0[TAG_W+20+:2]*16+:16];
        selected11<=group11[select_tag0[TAG_W+28+:2]*16+:16];
        select_tag0<=response_tag;select_tag1<=select_tag0;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            select_valid0<=0;select_valid1<=0;saved00<=0;saved01<=0;pixel_valid<=0;
            p00<=0;p10<=0;p01<=0;p11<=0;ax<=0;ay<=0;pixel_tag<=0;
        end else if(pixel_ce) begin
            select_valid0<=response_valid;select_valid1<=select_valid0;
            pixel_valid<=select_valid1 && phase!=1;
            if(select_valid1 && phase==1) begin saved00<=selected00;saved01<=selected01;end
            if(select_valid1 && phase!=1) begin
                p00<=phase==2 ? saved00 : selected00;p01<=phase==2 ? saved01 : selected01;
                p10<=selected10;p11<=selected11;
                ax<=select_tag1[TAG_W+:8];ay<=select_tag1[TAG_W+8+:8];pixel_tag<=select_tag1[TAG_W-1:0];
            end
        end
    end
endmodule
