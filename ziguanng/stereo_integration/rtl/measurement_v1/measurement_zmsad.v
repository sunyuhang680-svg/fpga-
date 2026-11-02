`timescale 1ns/1ps
// Bounded zero-mean SAD on immutable patches; one read and one accumulate cycle.
// 127x63 is the search REGION, 11x11 is the reference template. Pixel units.
// Clear cancels an epoch and discards both loaded patches and previous results.
module measurement_zmsad #(
    parameter SW=127, SH=63, TW=11, TH=11,
    parameter MIN_CONTRAST=20, UNIQUE_PERCENT=10, MAX_MEAN_ERROR=32
)(
    input wire clk, rst_n, clear,
    input wire load_valid, load_search,
    input wire [12:0] load_address,
    input wire [7:0] load_pixel,
    input wire start,
    output wire busy,
    output reg done, valid,
    output reg [7:0] reason,
    output reg [6:0] match_x,
    output reg [5:0] match_y,
    output reg [23:0] best_cost, second_cost
);
    localparam N=TW*TH, NS=SW*SH;
    localparam IDLE=0, SUM_READ=1, SUM_ADD=2, COST_READ=3, COST_SCALE=4,
               COST_CENTER=5, COST_DIFF=6, COST_ABS=7, COST_ADD=8,
               EVALUATE=9, FINISH=10;
    localparam BAD_LOAD=1, LOW_TEXTURE=2, AMBIGUOUS=3, HIGH_COST=4;
    reg [3:0] state;
    reg [7:0] reference [0:N-1];
    reg [7:0] search [0:NS-1];
    reg [13:0] reference_count, search_count;
    reg load_fault;
    reg [7:0] template_min, template_max;
    reg [15:0] template_sum, candidate_sum;
    reg [6:0] candidate_x;
    reg [5:0] candidate_y;
    reg [3:0] pixel_x,pixel_y;
    reg [7:0] pixel_index, left_sample,right_sample;
    reg [23:0] cost;
    wire [13:0] search_address=(candidate_y+pixel_y)*SW+candidate_x+pixel_x;
    // Separate BRAM output, constant multiply, mean removal, difference,
    // absolute value, and accumulator. No new timing exceptions are needed.
    reg [15:0] left_scaled,right_scaled;
    reg signed [17:0] left_centered,right_centered;
    reg signed [18:0] difference;
    reg [18:0] absolute_difference;
    wire [31:0] best_scaled=best_cost*100;
    wire [31:0] second_scaled=second_cost*(100-UNIQUE_PERCENT);
    assign busy=(state!=IDLE);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;done<=0;valid<=0;reason<=0;match_x<=0;match_y<=0;
            reference_count<=0;search_count<=0;load_fault<=0;
            template_min<=255;template_max<=0;template_sum<=0;candidate_sum<=0;
            candidate_x<=0;candidate_y<=0;pixel_x<=0;pixel_y<=0;pixel_index<=0;
            left_sample<=0;right_sample<=0;cost<=0;best_cost<=24'hffffff;second_cost<=24'hffffff;
            left_scaled<=0;right_scaled<=0;left_centered<=0;right_centered<=0;difference<=0;absolute_difference<=0;
        end else if(clear) begin
            state<=IDLE;done<=0;valid<=0;reason<=0;match_x<=0;match_y<=0;
            reference_count<=0;search_count<=0;load_fault<=0;
            template_min<=255;template_max<=0;template_sum<=0;candidate_sum<=0;
            candidate_x<=0;candidate_y<=0;pixel_x<=0;pixel_y<=0;pixel_index<=0;
            cost<=0;best_cost<=24'hffffff;second_cost<=24'hffffff;
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
                        reference[load_address]<=load_pixel;reference_count<=reference_count+1'b1;
                        template_sum<=template_sum+load_pixel;
                        if(load_pixel<template_min)template_min<=load_pixel;
                        if(load_pixel>template_max)template_max<=load_pixel;
                    end else load_fault<=1;
                end
            end
            case(state)
                IDLE: if(start) begin
                    valid<=0;reason<=0;match_x<=0;match_y<=0;
                    best_cost<=24'hffffff;second_cost<=24'hffffff;
                    if(load_valid || load_fault || reference_count!=N || search_count!=NS) begin
                        done<=1;reason<=BAD_LOAD;
                    end else if(template_max-template_min<MIN_CONTRAST) begin
                        done<=1;reason<=LOW_TEXTURE;
                    end else begin
                        candidate_x<=0;candidate_y<=0;pixel_x<=0;pixel_y<=0;pixel_index<=0;
                        candidate_sum<=0;cost<=0;state<=SUM_READ;
                    end
                end
                SUM_READ: begin right_sample<=search[search_address];state<=SUM_ADD;end
                SUM_ADD: begin
                    candidate_sum<=candidate_sum+right_sample;
                    if(pixel_index==N-1)begin
                        pixel_x<=0;pixel_y<=0;pixel_index<=0;state<=COST_READ;
                    end else begin
                        pixel_index<=pixel_index+1'b1;
                        if(pixel_x==TW-1)begin pixel_x<=0;pixel_y<=pixel_y+1'b1;end
                        else pixel_x<=pixel_x+1'b1;
                        state<=SUM_READ;
                    end
                end
                COST_READ: begin
                    right_sample<=search[search_address];left_sample<=reference[pixel_index];state<=COST_SCALE;
                end
                COST_SCALE:begin left_scaled<=left_sample*N;right_scaled<=right_sample*N;state<=COST_CENTER;end
                COST_CENTER:begin
                    left_centered<=$signed({1'b0,left_scaled})-$signed({1'b0,template_sum});
                    right_centered<=$signed({1'b0,right_scaled})-$signed({1'b0,candidate_sum});state<=COST_DIFF;
                end
                COST_DIFF:begin difference<={left_centered[17],left_centered}-{right_centered[17],right_centered};state<=COST_ABS;end
                COST_ABS:begin absolute_difference<=difference[18]?-difference:difference;state<=COST_ADD;end
                COST_ADD: begin
                    cost<=cost+absolute_difference;
                    if(pixel_index==N-1)state<=EVALUATE;
                    else begin
                        pixel_index<=pixel_index+1'b1;
                        if(pixel_x==TW-1)begin pixel_x<=0;pixel_y<=pixel_y+1'b1;end
                        else pixel_x<=pixel_x+1'b1;
                        state<=COST_READ;
                    end
                end
                EVALUATE: begin
                    if(cost<best_cost)begin
                        second_cost<=best_cost;best_cost<=cost;
                        match_x<=candidate_x+TW/2;match_y<=candidate_y+TH/2;
                    end else if(cost<second_cost)second_cost<=cost;
                    if(candidate_x==SW-TW && candidate_y==SH-TH)state<=FINISH;
                    else begin
                        if(candidate_x==SW-TW)begin candidate_x<=0;candidate_y<=candidate_y+1'b1;end
                        else candidate_x<=candidate_x+1'b1;
                        pixel_x<=0;pixel_y<=0;pixel_index<=0;candidate_sum<=0;cost<=0;state<=SUM_READ;
                    end
                end
                FINISH: begin
                    done<=1;state<=IDLE;
                    if(best_cost>N*N*MAX_MEAN_ERROR)reason<=HIGH_COST;
                    else if(second_cost==24'hffffff || best_scaled>=second_scaled)reason<=AMBIGUOUS;
                    else begin valid<=1;reason<=0;end
                end
                default:begin state<=IDLE;done<=1;valid<=0;reason<=BAD_LOAD;end
            endcase
        end
    end
endmodule
