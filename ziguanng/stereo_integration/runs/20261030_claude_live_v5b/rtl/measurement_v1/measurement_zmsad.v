`timescale 1ns/1ps
// claude live_v5b: zero-mean SAD matcher, band-limited and neighbour-excluded.
// Replaces stereo_integration/rtl/measurement_v1/measurement_zmsad.v (original unchanged).
// - SWxSH search REGION (default 255x31), TWxTH template (default 21x11). Pixel units.
// - Only candidate top-left positions inside [cand_x0..cand_x1]x[cand_y0..cand_y1]
//   are evaluated (the predicted epipolar segment box from stereo_depth_roi).
// - Pass 0 finds the best cost (first minimum in raster order).
// - Pass 1 recomputes costs for the candidates with |x-best_x|>EXCL and keeps
//   their minimum as second_cost. Neighbour exclusion is along x (disparity)
//   only, because y is already fixed by the epipolar band.
// - Cost pipeline issues one pixel per clock. Constant multiplies are shift-add
//   (APM is fully used in this design).
// Clear cancels an epoch and discards both loaded patches and previous results.
module measurement_zmsad #(
    parameter SW=255, SH=31, TW=21, TH=11,
    parameter MIN_CONTRAST=12, UNIQUE_PERCENT=10, MAX_MEAN_ERROR=32, EXCL=2
)(
    input wire clk, rst_n, clear,
    input wire load_valid, load_search,
    input wire [12:0] load_address,
    input wire [7:0] load_pixel,
    input wire [7:0] cand_x0, cand_x1,
    input wire [4:0] cand_y0, cand_y1,
    input wire start,
    output wire busy,
    output reg done, valid,
    output reg [7:0] reason,
    output reg [7:0] match_x,
    output reg [4:0] match_y,
    output reg [31:0] best_cost, second_cost
);
    localparam N=TW*TH, NS=SW*SH;
    localparam IDLE=0, SUM_RUN=1, COST_RUN=2, EVALUATE=3, ADVANCE=4, FINISH=5;
    localparam BAD_LOAD=1, LOW_TEXTURE=2, AMBIGUOUS=3, HIGH_COST=4;
    localparam [31:0] NONE=32'hffffffff;
    localparam [31:0] COST_LIMIT=N*N*MAX_MEAN_ERROR;

    function [39:0] cmul;   // x*k as a sum of shifted copies; k is a constant
        input [31:0] x; input integer k;
        integer i;
        begin
            cmul=0;
            for(i=0;i<12;i=i+1) if(k[i]) cmul=cmul+({8'd0,x}<<i);
        end
    endfunction

    reg [2:0] state;
    reg [7:0] reference [0:N-1];
    reg [7:0] search [0:NS-1];
    reg [13:0] reference_count, search_count;
    reg load_fault;
    reg [7:0] template_min, template_max;
    reg [15:0] template_sum, candidate_sum;
    reg pass;                       // 0: best, 1: second (neighbour-excluded)
    reg [7:0] cx, best_x;
    reg [4:0] cy, best_y;
    reg [12:0] cand_base, row_base; // address of (cx,cy) / (cand_x0,cy)
    // issue side
    reg [8:0] issue_index;          // pixel index being issued, 0..N-1
    reg [4:0] issue_px;
    reg [12:0] issue_addr;
    reg issuing;
    // pipeline
    reg [7:0] rd_search, rd_ref;
    reg [6:0] vpipe;                // valid per stage
    reg [17:0] l_scaled, r_scaled;
    reg signed [18:0] l_cent, r_cent;
    reg signed [19:0] diff;
    reg [19:0] adiff;
    reg [31:0] cost;
    wire [7:0] search_q = search[issue_addr];
    wire [7:0] ref_q    = reference[issue_index];

    wire bounds_bad = (cand_x0>cand_x1) || (cand_y0>cand_y1) ||
                      ({1'b0,cand_x1}>SW-TW) || ({1'b0,cand_y1}>SH-TH);
    wire [7:0] dx_best = (cx>best_x) ? cx-best_x : best_x-cx;
    wire skip_here = pass && (dx_best<=EXCL);
    wire last_cand = (cx==cand_x1) && (cy==cand_y1);
    wire [39:0] best_scaled   = cmul(best_cost,100);
    wire [39:0] second_scaled = cmul(second_cost,100-UNIQUE_PERCENT);

    assign busy=(state!=IDLE);

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;done<=0;valid<=0;reason<=0;match_x<=0;match_y<=0;
            reference_count<=0;search_count<=0;load_fault<=0;
            template_min<=255;template_max<=0;template_sum<=0;candidate_sum<=0;
            pass<=0;cx<=0;cy<=0;best_x<=0;best_y<=0;cand_base<=0;row_base<=0;
            issue_index<=0;issue_px<=0;issue_addr<=0;issuing<=0;
            rd_search<=0;rd_ref<=0;vpipe<=0;l_scaled<=0;r_scaled<=0;l_cent<=0;r_cent<=0;diff<=0;adiff<=0;
            cost<=0;best_cost<=NONE;second_cost<=NONE;
        end else if(clear) begin
            state<=IDLE;done<=0;valid<=0;reason<=0;match_x<=0;match_y<=0;
            reference_count<=0;search_count<=0;load_fault<=0;
            template_min<=255;template_max<=0;template_sum<=0;candidate_sum<=0;
            pass<=0;issuing<=0;vpipe<=0;cost<=0;best_cost<=NONE;second_cost<=NONE;
        end else begin
            done<=0;
            if(load_valid && state==IDLE) begin
                valid<=0;
                if(load_search) begin
                    if(search_count<NS && load_address==search_count) begin
                        search[load_address]<=load_pixel;search_count<=search_count+1'b1;
                    end else load_fault<=1;
                end else begin
                    if(reference_count<N && load_address==reference_count) begin
                        reference[load_address[8:0]]<=load_pixel;reference_count<=reference_count+1'b1;
                        template_sum<=template_sum+load_pixel;
                        if(load_pixel<template_min)template_min<=load_pixel;
                        if(load_pixel>template_max)template_max<=load_pixel;
                    end else load_fault<=1;
                end
            end
            // ---- issue: one pixel address per clock, row-major in the template
            if(issuing) begin
                if(issue_index==N-1) issuing<=0;
                else begin
                    issue_index<=issue_index+1'b1;
                    if(issue_px==TW-1) begin issue_px<=0;issue_addr<=issue_addr+(SW-TW+1);end
                    else begin issue_px<=issue_px+1'b1;issue_addr<=issue_addr+1'b1;end
                end
            end
            // ---- pipeline: read, scale, centre, difference, absolute, accumulate
            vpipe<={vpipe[5:0],issuing};
            rd_search<=search_q; rd_ref<=ref_q;                          // stage 1
            l_scaled<=cmul(rd_ref,N); r_scaled<=cmul(rd_search,N);       // stage 2
            l_cent<=$signed({1'b0,l_scaled})-$signed({3'b0,template_sum});// stage 3
            r_cent<=$signed({1'b0,r_scaled})-$signed({3'b0,candidate_sum});
            diff<={l_cent[18],l_cent}-{r_cent[18],r_cent};               // stage 4
            adiff<=diff[19] ? -diff : diff;                              // stage 5
            case(state)
                IDLE: if(start) begin
                    valid<=0;reason<=0;match_x<=0;match_y<=0;
                    best_cost<=NONE;second_cost<=NONE;
                    if(load_valid || load_fault || reference_count!=N || search_count!=NS || bounds_bad) begin
                        done<=1;reason<=BAD_LOAD;
                    end else if(template_max-template_min<MIN_CONTRAST) begin
                        done<=1;reason<=LOW_TEXTURE;
                    end else begin
                        pass<=0;cx<=cand_x0;cy<=cand_y0;
                        cand_base<=cmul(cand_y0,SW)+cand_x0;row_base<=cmul(cand_y0,SW)+cand_x0;
                        state<=ADVANCE;
                    end
                end
                // ADVANCE: start the current candidate, or skip it in pass 1.
                ADVANCE: begin
                    if(skip_here) state<=EVALUATE;
                    else begin
                        candidate_sum<=0;cost<=0;
                        issue_index<=0;issue_px<=0;issue_addr<=cand_base;issuing<=1;
                        state<=SUM_RUN;
                    end
                end
                SUM_RUN: begin
                    if(vpipe[0]) candidate_sum<=candidate_sum+rd_search;
                    if(!issuing && !vpipe[0] && vpipe==0 && issue_index==N-1) begin
                        issue_index<=0;issue_px<=0;issue_addr<=cand_base;issuing<=1;
                        state<=COST_RUN;
                    end
                end
                COST_RUN: begin
                    if(vpipe[4]) cost<=cost+adiff;
                    if(!issuing && vpipe==0 && issue_index==N-1) state<=EVALUATE;
                end
                EVALUATE: begin
                    if(!skip_here) begin
                        if(!pass) begin
                            if(cost<best_cost) begin best_cost<=cost;best_x<=cx;best_y<=cy;end
                        end else if(cost<second_cost) second_cost<=cost;
                    end
                    if(last_cand) begin
                        if(!pass) begin
                            pass<=1;cx<=cand_x0;cy<=cand_y0;
                            cand_base<=cmul(cand_y0,SW)+cand_x0;row_base<=cmul(cand_y0,SW)+cand_x0;
                            state<=ADVANCE;
                        end else state<=FINISH;
                    end else begin
                        if(cx==cand_x1) begin
                            cx<=cand_x0;cy<=cy+1'b1;
                            row_base<=row_base+SW;cand_base<=row_base+SW;
                        end else begin cx<=cx+1'b1;cand_base<=cand_base+1'b1;end
                        state<=ADVANCE;
                    end
                end
                FINISH: begin
                    done<=1;state<=IDLE;
                    match_x<=best_x+TW/2;match_y<=best_y+TH/2;
                    if(best_cost>COST_LIMIT)reason<=HIGH_COST;
                    else if(second_cost==NONE || best_scaled>=second_scaled)reason<=AMBIGUOUS;
                    else begin valid<=1;reason<=0;end
                end
                default:begin state<=IDLE;done<=1;valid<=0;reason<=BAD_LOAD;end
            endcase
        end
    end
endmodule
