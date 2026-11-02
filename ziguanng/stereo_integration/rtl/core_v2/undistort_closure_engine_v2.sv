// Explicit enhanced derivative: geometry-valid bit survives cache and sampler; one immutable frame snapshot.
// Explicit v2 derivative of undistort_engine; direct atomic geometry config.
// v1 module and memory/cache/sampler interfaces remain unchanged.
// Frame-to-frame accelerator. Native memory addresses are BYTE addresses.
// Source/destination must be disjoint and 32B aligned. Width must be a multiple
// of 16 (16..1920); height is 2..1080. Each write is one masked 256-bit beat.
module undistort_closure_engine_v2(
    input wire clk,rst_n,
    input wire cfg_valid,output wire cfg_ready,
    input wire [671:0] cfg_flat,
    input wire start,input wire [31:0] src_base,dst_base,
    output reg busy,output reg done,output reg error,
    output wire rd_valid,input wire rd_ready,output wire [31:0] rd_addr,
    input wire rsp_valid,input wire [255:0] rsp_data,
    output reg wr_valid,input wire wr_ready,
    output reg [31:0] wr_addr,output reg [255:0] wr_data,
    output wire [31:0] wr_strb,
    output reg [31:0] frame_cycles,
    output reg [31:0] invalid_domain_pixels,
    output wire [9:0] applied_gamma_q8,target_gamma_q8,next_auto_gamma_q8,
    output wire [21:0] statistics_pixels,
    output wire [31:0] cache_hits,cache_misses
);
    wire [671:0] cfg;
    wire [11:0] width=cfg[616+:12],height=cfg[628+:12];
    wire pending,configured,cfg_error,bank_busy,bank_started;
    reg apply;
    processing_bank_validated_v2 u_config(.clk(clk),.rst_n(rst_n),.submit_valid(cfg_valid),.submit_ready(cfg_ready),
        .submit_flat(cfg_flat),.frame_start(apply),.frame_done(!busy),.start_accepted(bank_started),
        .busy(bank_busy),.pending(pending),.config_valid(configured),.error(cfg_error),.active_flat(cfg));
    reg launch,producing,waiting_config,config_applied;
    reg [31:0] source,dest;
    reg [11:0] tile_x,tile_y,local_x,local_y;
    wire [11:0] dx=tile_x+local_x,dy=tile_y+local_y;
    wire [31:0] offset=(dy*width+dx)<<1;
    wire [43:0] dest_tag={dx,offset};
    wire map_ready,map_valid,map_out_ready,map_inside,domain_bad;
    wire [31:0] sx,sy;
    wire [43:0] map_tag;
    projective_map_credit_v2 u_map(.clk(clk),.rst_n(rst_n),.in_valid(producing),.in_ready(map_ready),
        .in_x(dx),.in_y(dy),.in_tag(dest_tag),.homography_flat(cfg[359:0]),
        .k1(cfg[360+:32]),.k2(cfg[392+:32]),.p1(cfg[424+:32]),.p2(cfg[456+:32]),
        .fx(cfg[488+:32]),.fy(cfg[520+:32]),.cx(cfg[552+:32]),.cy(cfg[584+:32]),
        .width(width),.height(height),.out_valid(map_valid),.out_ready(map_out_ready),
        .out_x(sx),.out_y(sy),.out_inside(map_inside),.out_domain_error(domain_bad),.out_tag(map_tag));
    wire sample_valid,sample_ready,sample_inside;
    wire[31:0] sample_sx,sample_sy;
    wire[43:0] sample_tag;
    coord_register_slice_v2 #(.WIDTH(109)) u_map_buffer(
        .clk(clk),.rst_n(rst_n),.in_valid(map_valid),.in_ready(map_out_ready),
        .in_data({sx,sy,map_inside,map_tag}),
        .out_valid(sample_valid),.out_ready(sample_ready),
        .out_data({sample_sx,sample_sy,sample_inside,sample_tag}));
    wire cv,cr,cb,rv,rr,pv,pr;
    wire [31:0] ca,cc;
    wire [80:0] ct,rt;
    wire [255:0] da,db;
    wire [15:0] p00,p10,p01,p11;
    wire [7:0] ax,ay;
    wire [44:0] pixel_tag;
    pixel_sampler_select_v2 #(.TAG_W(45)) u_sampler(.clk(clk),.rst_n(rst_n),.in_valid(sample_valid),.in_ready(sample_ready),
        .sx(sample_sx),.sy(sample_sy),.map_inside(sample_inside),.in_tag({sample_inside,sample_tag}),.width(width),.height(height),.src_base(source),
        .cache_valid(cv),.cache_ready(cr),.cache_addr_a(ca),.cache_addr_b(cc),.cache_bypass(cb),.cache_tag(ct),
        .response_valid(rv),.response_ready(rr),.data_a(da),.data_b(db),.response_tag(rt),
        .pixel_valid(pv),.pixel_ready(pr),.p00(p00),.p10(p10),.p01(p01),.p11(p11),
        .ax(ax),.ay(ay),.pixel_tag(pixel_tag));
    // Decouple RAM tag comparison from sampler and map-buffer clock enables.
    wire queued_valid,queued_ready,queued_bypass;
    wire[31:0]queued_a,queued_b;
    wire[80:0]queued_tag;
    cache_request_slice_v3 #(.WIDTH(146)) u_cache_request_buffer(
        .clk(clk),.rst_n(rst_n),.flush(launch),
        .in_valid(cv),.in_ready(cr),.in_data({cb,ca,cc,ct}),
        .out_valid(queued_valid),.out_ready(queued_ready),
        .out_data({queued_bypass,queued_a,queued_b,queued_tag}));
    wire cache_error;
    pair_cache_select_v2 #(.TAG_W(81)) u_cache(.clk(clk),.rst_n(rst_n),.invalidate(launch),
        .in_valid(queued_valid),.in_ready(queued_ready),.addr_a(queued_a),.addr_b(queued_b),.bypass(queued_bypass),.in_tag(queued_tag),
        .out_valid(rv),.out_ready(rr),.data_a(da),.data_b(db),.out_tag(rt),
        .rd_valid(rd_valid),.rd_ready(rd_ready),.rd_addr(rd_addr),.rsp_valid(rsp_valid),.rsp_data(rsp_data),
        .hit_pairs(cache_hits),.miss_lines(cache_misses),.protocol_error(cache_error));
    wire bv,br;
    wire [15:0] bp;
    wire [44:0] bt;
    bilinear_rgb565 #(.TAG_W(45)) u_bilinear(.clk(clk),.rst_n(rst_n),.in_valid(pv),.in_ready(pr),
        .p00(p00),.p10(p10),.p01(p01),.p11(p11),.ax(ax),.ay(ay),.in_tag(pixel_tag),
        .out_valid(bv),.out_ready(br),.out_pixel(bp),.out_tag(bt));
    wire enhancement_ready,enhancement_done,enhancement_error,ev,er,enhancement_busy,elast;
    wire [15:0] ep;
    wire [43:0] et;
    reg writes_finished,enhancement_finished;
    wire valid_dimensions=(width>=16 && width<=1920 && width[3:0]==0 && height>=2 && height<=1080);
    wire [31:0] frame_bytes=(width*height)<<1;
    wire [21:0] total_frame_pixels={10'b0,width}*{10'b0,height};
    wire valid_regions=(source<=32'h40000000-frame_bytes && dest<=32'h40000000-frame_bytes &&
                         (source+frame_bytes<=dest || dest+frame_bytes<=source));
    adaptive_luminance_stream_v2 u_enhancement(.clk(clk),.rst_n(rst_n),
        .frame_start(launch && valid_regions),.frame_ready(enhancement_ready),
        .frame_mode(cfg[640+:2]),.manual_gamma_q8(cfg[642+:10]),.frame_pixels(total_frame_pixels),
        .busy(enhancement_busy),.frame_done(enhancement_done),.error(enhancement_error),
        .applied_gamma_q8(applied_gamma_q8),.target_gamma_q8(target_gamma_q8),.next_auto_gamma_q8(next_auto_gamma_q8),
        .statistics_pixels(statistics_pixels),
        .in_valid(bv),.in_ready(br),.in_rgb565(bp),.in_inside(bt[44]),
        .in_last(bt[31:0]==((width*height-1)<<1)),.in_tag(bt[43:0]),
        .out_valid(ev),.out_ready(er),.out_rgb565(ep),.out_tag(et),.out_last(elast));
    reg [3:0] pack_count;
    reg [255:0] pack_data;
    reg [31:0] pack_addr;
    reg last_write;
    wire [255:0] packed_next=pack_data | ({{240{1'b0}},ep} << (pack_count*16));
    assign er=!wr_valid || wr_ready;
    assign wr_strb=32'hffffffff;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            busy<=0;done<=0;error<=0;apply<=0;launch<=0;producing<=0;waiting_config<=0;config_applied<=0;
            source<=0;dest<=0;tile_x<=0;tile_y<=0;local_x<=0;local_y<=0;
            pack_count<=0;pack_data<=0;pack_addr<=0;
            wr_valid<=0;wr_addr<=0;wr_data<=0;last_write<=0;frame_cycles<=0;
            invalid_domain_pixels<=0;writes_finished<=0;enhancement_finished<=0;
        end else begin
            done<=0;apply<=0;launch<=0;
            if(bank_started) config_applied<=1;
            if(cfg_error || cache_error || enhancement_error) error<=1;
            if(busy) frame_cycles<=frame_cycles+1;
            if(map_valid && map_out_ready && domain_bad) begin error<=1;invalid_domain_pixels<=invalid_domain_pixels+1;end
            if(start && !busy && !waiting_config) begin
                writes_finished<=0;enhancement_finished<=0;
                config_applied<=0;
                error<=0;source<=src_base;dest<=dst_base;
                if(src_base[4:0]!=0 || dst_base[4:0]!=0 || src_base==dst_base) error<=1;
                else if(pending) begin busy<=1;apply<=1;waiting_config<=1;end
                else if(configured && valid_dimensions) begin busy<=1;apply<=1;waiting_config<=1;end
                else begin busy<=0;error<=1;end
            end
            // A newly queued NEXT frame keeps pending high. Wait on the bank's
            // accepted-start event, rather than on the next snapshot queue.
            if(waiting_config && (config_applied || (cfg_error && !configured)) && enhancement_ready) begin
                waiting_config<=0;
                if(configured && valid_dimensions) begin busy<=1;launch<=1;end
                else begin busy<=0;error<=1;end
            end
            if(launch) begin
                frame_cycles<=0;writes_finished<=0;enhancement_finished<=0;producing<=valid_regions;tile_x<=0;tile_y<=0;local_x<=0;local_y<=0;
                invalid_domain_pixels<=0;
                pack_count<=0;pack_data<=0;last_write<=0;
                if(!valid_regions) begin busy<=0;error<=1;end
            end
            if(producing && map_ready) begin
                if(local_x==15) begin
                    local_x<=0;
                    if(local_y==15 || dy==height-1) begin
                        local_y<=0;
                        if(tile_x+16>=width) begin
                            tile_x<=0;
                            if(tile_y+16>=height) producing<=0;
                            else tile_y<=tile_y+16;
                        end else tile_x<=tile_x+16;
                    end else local_y<=local_y+1'b1;
                end else local_x<=local_x+1'b1;
            end
            if(wr_valid && wr_ready) begin
                wr_valid<=0;
                if(last_write) writes_finished<=1;
            end
            if(enhancement_done) enhancement_finished<=1;
            if(busy && writes_finished && enhancement_finished) begin busy<=0;done<=1;end
            if(ev && er) begin
                if(pack_count==0) pack_addr<=dest+et[31:0];
                if(pack_count==15) begin
                    wr_data<=packed_next;wr_addr<=pack_addr;wr_valid<=1;
                    pack_count<=0;pack_data<=0;
                    last_write<=(et[31:0]==((width*height-1)<<1));
                end else begin pack_data<=packed_next;pack_count<=pack_count+1'b1;end
            end
        end
    end
endmodule
