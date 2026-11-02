// Explicit v2 retimed histogram: partial luminance, bin, synchronous counter
// read, then increment/write. Forward the preceding write on repeated bins.
// Last input drains before bank publication; no partial histogram is exposed.
module luminance_histogram_pipelined_v2(
    input wire clk,rst_n,input wire frame_start,output wire frame_ready,
    input wire in_valid,output wire in_ready,input wire[15:0] in_rgb565,
    input wire in_inside,in_last,output reg hist_valid,input wire hist_ready,
    input wire[7:0] hist_addr,output reg[21:0] hist_data,
    output reg[21:0] valid_pixels,total_pixels,output reg error
);
    localparam INIT=0,IDLE=1,CLEAR=2,COLLECT=3,DRAIN=4;
    reg[2:0] state,v,inside_pipe,last_pipe;
    reg[8:0] clear_index;
    reg collect_bank,completed_bank;
    // Two explicit simple-dual-port copies: RMW read and snapshot read have
    // independent addresses, while both receive the identical write stream.
    // A single three-address array inferred flip-flops in actual PDS mapping.
    (* ram_style="block" *) reg[21:0] counter_mem[0:511],snapshot_mem[0:511];
    reg[21:0] live_valid,live_total;
    reg[15:0] rg0,b0;
    reg[7:0] bin1,bin2,forward_bin;
    reg[21:0] read_value2,forward_value;
    reg forward_valid;
    wire[7:0] r={in_rgb565[15:11],in_rgb565[15:13]};
    wire[7:0] g={in_rgb565[10:5],in_rgb565[10:9]};
    wire[7:0] b={in_rgb565[4:0],in_rgb565[4:2]};
    wire accept=in_valid && in_ready;
    wire writing=v[2] && inside_pipe[2] && (state==COLLECT || state==DRAIN);
    wire[21:0] incremented=(forward_valid && forward_bin==bin2 ? forward_value : read_value2)+22'd1;
    wire clearing=state==INIT || state==CLEAR;
    wire mem_write=clearing || writing;
    wire[8:0] mem_address=state==INIT ? clear_index : state==CLEAR ? {collect_bank,clear_index[7:0]} : {collect_bank,bin2};
    wire[21:0] mem_value=clearing ? 22'd0 : incremented;
    assign frame_ready=state==IDLE;
    assign in_ready=state==COLLECT && (!in_last || !hist_valid);
    always @(posedge clk) begin
        rg0<=r*16'd77+g*16'd150;b0<=b*16'd29+16'd128;
        bin1<=(rg0+b0)>>8;bin2<=bin1;
    end
    // No asynchronous reset on RAM ports or their data registers. INIT
    // clears every word before frame_ready; hist_data is ignored when !valid.
    always @(posedge clk) begin
        if(mem_write) begin counter_mem[mem_address]<=mem_value;snapshot_mem[mem_address]<=mem_value;end
        read_value2<=counter_mem[{collect_bank,bin1}];
        hist_data<=snapshot_mem[{completed_bank,hist_addr}];
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=INIT;clear_index<=0;collect_bank<=0;completed_bank<=1;
            hist_valid<=0;valid_pixels<=0;total_pixels<=0;
            live_valid<=0;live_total<=0;error<=0;v<=0;inside_pipe<=0;last_pipe<=0;
            forward_valid<=0;forward_bin<=0;forward_value<=0;
        end else begin
            v<={v[1:0],accept};inside_pipe<={inside_pipe[1:0],in_inside};last_pipe<={last_pipe[1:0],in_last};
            if(hist_valid && hist_ready) hist_valid<=0;
            forward_valid<=writing;
            if(writing) begin
                forward_bin<=bin2;forward_value<=incremented;
            end
            case(state)
                INIT:begin
                    if(clear_index==511) begin clear_index<=0;state<=IDLE;end
                    else clear_index<=clear_index+1'b1;
                end
                IDLE:if(frame_start) begin state<=CLEAR;clear_index<=0;live_total<=0;live_valid<=0;forward_valid<=0;end
                CLEAR:begin
                    if(clear_index==255) begin clear_index<=0;state<=COLLECT;end
                    else clear_index<=clear_index+1'b1;
                end
                COLLECT:if(accept) begin
                    live_total<=live_total+1'b1;
                    if(live_total>=22'd2073600) error<=1;
                    if(in_inside) live_valid<=live_valid+1'b1;
                    if(in_last) state<=DRAIN;
                end
            endcase
            if((state==COLLECT || state==DRAIN) && v[2] && last_pipe[2]) begin
                completed_bank<=collect_bank;collect_bank<=!collect_bank;state<=IDLE;
                valid_pixels<=live_valid;total_pixels<=live_total;hist_valid<=1;
            end
        end
    end
endmodule
