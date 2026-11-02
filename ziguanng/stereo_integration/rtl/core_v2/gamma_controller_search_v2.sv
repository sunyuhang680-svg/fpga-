// Exact monotone integer Gamma search. Quantized sums decrease with Gamma.
// Keep current ties; otherwise clamp Gamma=256 to the optimal interval.
// At most22 full histogram sums; same target/smoothing ports as exhaustive v2.
// Same Q8/tie/smoothing rules as gamma_controller_v2; explicit timing stages.
// ROM sample -> multiply -> accumulate -> difference -> absolute -> compare ->
// signed smoothing steps. Avoid a ROM-to-multiply-to-smooth single-cycle path.
module gamma_controller_search_v2(
    input wire clk,rst_n,input wire job_valid,output wire job_ready,
    input wire[21:0] valid_pixels,input wire[9:0] applied_gamma_q8,
    output wire[7:0] hist_addr,input wire[21:0] hist_data,output reg hist_release,
    output wire[16:0] table_addr,input wire[7:0] table_data,
    output reg result_valid,input wire result_ready,
    output reg[9:0] target_gamma_q8,next_gamma_q8,output reg error
);
    localparam IDLE=0,HREAD=1,HCOPY=2,RREAD=3,SAMPLE=4,MULTIPLY=5,ACCUM=6,
        DIFFERENCE=7,ABSOLUTE=8,COMPARE=9,SMOOTH_DIFF=10,SMOOTH_MAG=11,SMOOTH_STEP=12,SMOOTH_APPLY=13,RESULT=14,CHOOSE=15,BUILD_BOUNDS=16;
    reg[4:0] state;
    reg[7:0] bin,rom_sample;
    reg[21:0] histogram_copy[0:255],count,hist_sample;
    reg[31:0] observed_count;
    reg[28:0] sum,completed_sum,product,best_error,candidate_error;
    reg signed[29:0] difference;
    reg signed[10:0] smooth_difference;
    reg[10:0] magnitude;
    reg negative;
    reg[6:0] smooth_step;
    reg[9:0] current,gamma,best;
    reg[8:0] candidate;
    reg baseline;
    localparam CURRENT=0,FIND_CROSS=1,EVAL_RIGHT=2,EVAL_LEFT=3,EVAL_UNIT=4,FIND_BOUND=5;
    reg[2:0] reason;
    reg[9:0] search_lo,search_hi,crossing;
    reg[28:0] current_error,right_error,left_error,optimal_error,lower_sum,upper_sum,threshold;
    reg strict_search;
    reg[5:0] mean_evaluations;
    wire[28:0] desired_sum={7'b0,count}<<7;
    wire search_test=reason==FIND_CROSS ? completed_sum<=desired_sum :
        strict_search ? completed_sum<threshold : completed_sum<=threshold;
    wire[9:0] next_lo=search_test ? search_lo : gamma+10'd1;
    wire[9:0] next_hi=search_test ? gamma : search_hi;
    wire[10:0] midpoint_sum={1'b0,next_lo}+{1'b0,next_hi};
    wire[28:0] minimum_error=right_error<left_error ? right_error : left_error;
    assign job_ready=state==IDLE;
    assign hist_addr=bin;
    assign table_addr=((gamma-10'd128)<<8)+bin;
    function automatic[9:0] candidate_q8(input[8:0] idx);
        begin
            if(idx>256) candidate_q8=idx+128;
            else if(idx[0]) candidate_q8=256-((idx+1)>>1);
            else candidate_q8=256+(idx>>1);
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            reason<=CURRENT;search_lo<=128;search_hi<=513;crossing<=128;
            current_error<=0;right_error<=29'h1fffffff;left_error<=29'h1fffffff;optimal_error<=0;
            lower_sum<=0;upper_sum<=0;threshold<=0;strict_search<=0;mean_evaluations<=0;
            state<=IDLE;bin<=0;count<=0;current<=256;gamma<=256;best<=256;
            observed_count<=0;sum<=0;completed_sum<=0;product<=0;best_error<=0;candidate_error<=0;
            rom_sample<=0;hist_sample<=0;difference<=0;smooth_difference<=0;magnitude<=0;negative<=0;smooth_step<=0;
            candidate<=0;baseline<=1;hist_release<=0;result_valid<=0;target_gamma_q8<=256;next_gamma_q8<=256;error<=0;
        end else begin
            hist_release<=0;
            case(state)
                IDLE:if(job_valid) begin
                    current<=applied_gamma_q8;gamma<=applied_gamma_q8;best<=applied_gamma_q8;
                    reason<=CURRENT;right_error<=29'h1fffffff;left_error<=29'h1fffffff;mean_evaluations<=0;
                    count<=valid_pixels;bin<=0;sum<=0;observed_count<=0;baseline<=1;candidate<=0;
                    if(applied_gamma_q8<128 || applied_gamma_q8>512 || valid_pixels>2073600) begin
                        error<=1;hist_release<=1;target_gamma_q8<=256;next_gamma_q8<=256;result_valid<=1;state<=RESULT;
                    end else if(valid_pixels==0) begin
                        hist_release<=1;target_gamma_q8<=applied_gamma_q8;next_gamma_q8<=applied_gamma_q8;result_valid<=1;state<=RESULT;
                    end else state<=HREAD;
                end
                HREAD:state<=HCOPY;
                HCOPY:begin
                    histogram_copy[bin]<=hist_data;observed_count<=observed_count+hist_data;
                    if(bin==255) begin
                        hist_release<=1;bin<=0;
                        if(observed_count+hist_data!=count) begin
                            error<=1;target_gamma_q8<=current;next_gamma_q8<=current;result_valid<=1;state<=RESULT;
                        end else state<=RREAD;
                    end else begin bin<=bin+1;state<=HREAD;end
                end
                RREAD:state<=SAMPLE;
                SAMPLE:begin rom_sample<=table_data;hist_sample<=histogram_copy[bin];state<=MULTIPLY;end
                MULTIPLY:begin product<=hist_sample*rom_sample;state<=ACCUM;end
                ACCUM:begin
                    if(bin==255) begin completed_sum<=sum+product;sum<=0;bin<=0;state<=DIFFERENCE;end
                    else begin sum<=sum+product;bin<=bin+1;state<=RREAD;end
                end
                DIFFERENCE:begin difference<=$signed({1'b0,completed_sum})-$signed({1'b0,desired_sum});state<=ABSOLUTE;end
                ABSOLUTE:begin candidate_error<=difference<0 ? -difference : difference;state<=COMPARE;end
                COMPARE:begin
                    mean_evaluations<=mean_evaluations+1;
                    case(reason)
                        CURRENT:begin
                            current_error<=candidate_error;search_lo<=128;search_hi<=513;gamma<=320;
                            reason<=FIND_CROSS;state<=RREAD;
                        end
                        FIND_CROSS:begin
                            search_lo<=next_lo;search_hi<=next_hi;
                            if(next_lo<next_hi) begin gamma<=midpoint_sum[10:1];state<=RREAD;end
                            else begin
                                crossing<=next_lo;
                                if(next_lo<=512) begin gamma<=next_lo;reason<=EVAL_RIGHT;end
                                else begin gamma<=512;right_error<=29'h1fffffff;reason<=EVAL_LEFT;end
                                state<=RREAD;
                            end
                        end
                        EVAL_RIGHT:begin
                            right_error<=candidate_error;
                            if(crossing>128) begin gamma<=crossing-1'b1;reason<=EVAL_LEFT;state<=RREAD;end
                            else begin left_error<=29'h1fffffff;state<=CHOOSE;end
                        end
                        EVAL_LEFT:begin left_error<=candidate_error;state<=CHOOSE;end
                        EVAL_UNIT:begin
                            if(completed_sum>=lower_sum && completed_sum<=upper_sum) begin
                                target_gamma_q8<=256;state<=SMOOTH_DIFF;
                            end else begin
                                strict_search<=completed_sum<lower_sum;
                                threshold<=completed_sum<lower_sum ? lower_sum : upper_sum;
                                search_lo<=128;search_hi<=513;gamma<=320;reason<=FIND_BOUND;state<=RREAD;
                            end
                        end
                        FIND_BOUND:begin
                            search_lo<=next_lo;search_hi<=next_hi;
                            if(next_lo<next_hi) begin gamma<=midpoint_sum[10:1];state<=RREAD;end
                            else begin target_gamma_q8<=strict_search ? next_lo-1'b1 : next_lo;state<=SMOOTH_DIFF;end
                        end
                        default:begin error<=1;target_gamma_q8<=current;state<=SMOOTH_DIFF;end
                    endcase
                end
                CHOOSE:begin
                    optimal_error<=minimum_error;
                    if(current_error<=minimum_error) begin target_gamma_q8<=current;state<=SMOOTH_DIFF;end
                    else state<=BUILD_BOUNDS;
                end
                BUILD_BOUNDS:begin
                    // All minimum-error Gamma values form one contiguous
                    // interval: desired-E <= S(gamma) <= desired+E.
                    lower_sum<=desired_sum-optimal_error;upper_sum<=desired_sum+optimal_error;
                    gamma<=256;reason<=EVAL_UNIT;state<=RREAD;
                end
                SMOOTH_DIFF:begin smooth_difference<=$signed({1'b0,target_gamma_q8})-$signed({1'b0,current});state<=SMOOTH_MAG;end
                SMOOTH_MAG:begin negative<=smooth_difference<0;magnitude<=smooth_difference<0 ? -smooth_difference : smooth_difference;state<=SMOOTH_STEP;end
                SMOOTH_STEP:begin smooth_step<=(magnitude+11'd4)>>3;state<=SMOOTH_APPLY;end
                SMOOTH_APPLY:begin next_gamma_q8<=negative ? current-smooth_step : current+smooth_step;result_valid<=1;state<=RESULT;end
                RESULT:if(result_ready) begin result_valid<=0;state<=IDLE;end
            endcase
        end
    end
endmodule
