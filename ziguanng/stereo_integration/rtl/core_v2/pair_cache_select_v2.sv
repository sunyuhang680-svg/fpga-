// Explicit v2 cache: synchronous duplicated tag+epoch RAM and data RAM.
// One elastic lookup stage, II=1 on hits. No 1024-bit indexed valid-vector write.
// Epoch wrap clears headers before accepting requests; reset also clears them.
module pair_cache_select_v2 #(parameter TAG_W=80,SET_BITS=8,EPOCH_BITS=8)(
    input wire clk,rst_n,invalidate,input wire in_valid,output wire in_ready,
    input wire[31:0] addr_a,addr_b,input wire bypass,input wire[TAG_W-1:0] in_tag,
    output reg out_valid,input wire out_ready,output reg[255:0] data_a,data_b,
    output reg[TAG_W-1:0] out_tag,output wire rd_valid,input wire rd_ready,output wire[31:0] rd_addr,
    input wire rsp_valid,input wire[255:0] rsp_data,output reg[31:0] hit_pairs,miss_lines,output reg protocol_error
);
    localparam SETS=1<<SET_BITS,KEY_BITS=27-SET_BITS,HEADER_BITS=KEY_BITS+EPOCH_BITS;
    localparam LOOK=0,COMMAND=1,RESPONSE=2,RELOAD=3,CLEAR=4;
    reg[2:0] state;
    reg[EPOCH_BITS-1:0] epoch;
    reg[SET_BITS-1:0] clear_set;
    reg slot_valid,slot_bypass;
    reg[31:0] slot_a,slot_b;
    reg[TAG_W-1:0] slot_tag;
    wire[SET_BITS-1:0] set_a=slot_a[5+:SET_BITS],set_b=slot_b[5+:SET_BITS];
    wire[KEY_BITS-1:0] key_a=slot_a[31:5+SET_BITS],key_b=slot_b[31:5+SET_BITS];
    reg[31:0] fill_addr;
    reg[SET_BITS-1:0] fill_set;
    reg[1:0] fill_way;
    wire[HEADER_BITS-1:0] header_a[0:3],header_b[0:3];
    wire[255:0] qa[0:3],qb[0:3];
    reg hit_a,hit_b;reg[1:0] way_a,way_b;
    integer w;
    always @* begin
        hit_a=0;hit_b=0;way_a=0;way_b=0;
        for(w=0;w<4;w=w+1) begin
            if(header_a[w]=={epoch,key_a}) begin hit_a=1;way_a=w;end
            if(header_b[w]=={epoch,key_b}) begin hit_b=1;way_b=w;end
        end
    end
    reg select_valid,select_bypass;
    reg [1:0] select_way_a,select_way_b;
    reg [TAG_W-1:0] select_tag;
    reg [255:0] pipe_a[0:3],pipe_b[0:3];
    wire output_capacity=!out_valid || out_ready;
    wire capacity=!select_valid || output_capacity;
    genvar c;
    generate for(c=0;c<4;c=c+1) begin: SELECT_PAYLOAD
        always @(posedge clk) if(capacity) begin pipe_a[c]<=qa[c];pipe_b[c]<=qb[c];end
    end endgenerate
    always @(posedge clk) if(capacity) begin
        select_way_a<=way_a;select_way_b<=way_b;select_bypass<=slot_bypass;select_tag<=slot_tag;
    end
    wire slot_hit=slot_bypass || (hit_a && hit_b);
    assign in_ready=state==LOOK && capacity && (!slot_valid || slot_hit) && !invalidate;
    wire accept=in_valid && in_ready;
    wire publish=state==LOOK && slot_valid && slot_hit && capacity && !invalidate;
    wire fill_en=state==RESPONSE && rsp_valid;
    assign rd_valid=state==COMMAND;
    assign rd_addr=fill_addr;
    // Speculate the next synchronous RAM lookup independently of tag hits.
    // A miss ignores this speculative result, fills, then RELOADs its own slot.
    // Preserve the current lookup when the elastic output stages are stalled.
    wire read_en=(state==LOOK && capacity && !invalidate) || state==RELOAD;
    wire[SET_BITS-1:0] read_a=(state==RELOAD) ? set_a : addr_a[5+:SET_BITS];
    wire[SET_BITS-1:0] read_b=(state==RELOAD) ? set_b : addr_b[5+:SET_BITS];
    (* ram_style="block" *) reg[1:0] victims_a[0:SETS-1],victims_b[0:SETS-1];
    reg[1:0] victim_a,victim_b;
    always @(posedge clk) begin
        if(state==CLEAR) begin victims_a[clear_set]<=0;victims_b[clear_set]<=0;end
        else if(fill_en) begin victims_a[fill_set]<=fill_way+1'b1;victims_b[fill_set]<=fill_way+1'b1;end
        if(read_en) begin victim_a<=victims_a[read_a];victim_b<=victims_b[read_b];end
    end
    genvar g;
    generate for(g=0;g<4;g=g+1) begin: WAY
        (* ram_style="block" *) reg[255:0] mem_a[0:SETS-1],mem_b[0:SETS-1];
        (* ram_style="block" *) reg[HEADER_BITS-1:0] tags_a[0:SETS-1],tags_b[0:SETS-1];
        reg[255:0] ra,rb;
        reg[HEADER_BITS-1:0] ta,tb;
        always @(posedge clk) begin
            if(state==CLEAR) begin tags_a[clear_set]<=0;tags_b[clear_set]<=0;end
            else if(fill_en && fill_way==g) begin
                tags_a[fill_set]<={epoch,fill_addr[31:5+SET_BITS]};tags_b[fill_set]<={epoch,fill_addr[31:5+SET_BITS]};
            end
            if(fill_en && fill_way==g) begin mem_a[fill_set]<=rsp_data;mem_b[fill_set]<=rsp_data;end
            if(read_en) begin ra<=mem_a[read_a];rb<=mem_b[read_b];ta<=tags_a[read_a];tb<=tags_b[read_b];end
        end
        assign qa[g]=ra;assign qb[g]=rb;assign header_a[g]=ta;assign header_b[g]=tb;
    end endgenerate
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=CLEAR;epoch<=1;clear_set<=0;slot_valid<=0;slot_bypass<=1;slot_a<=0;slot_b<=0;slot_tag<=0;
            out_valid<=0;select_valid<=0;out_tag<=0;data_a<=0;data_b<=0;fill_addr<=0;fill_set<=0;fill_way<=0;
            hit_pairs<=0;miss_lines<=0;protocol_error<=0;
        end else begin
            if(output_capacity) begin
                out_valid<=select_valid;
                if(select_valid) begin
                    out_tag<=select_tag;
                    data_a<=select_bypass ? 0 : pipe_a[select_way_a];
                    data_b<=select_bypass ? 0 : pipe_b[select_way_b];
                end
            end
            if(capacity) select_valid<=publish;
            if(publish) begin
                slot_valid<=0;if(!slot_bypass) hit_pairs<=hit_pairs+1;
            end
            if(accept) begin slot_valid<=1;slot_a<=addr_a;slot_b<=addr_b;slot_bypass<=bypass;slot_tag<=in_tag;end
            case(state)
                CLEAR:if(clear_set==SETS-1) begin clear_set<=0;state<=LOOK;end else clear_set<=clear_set+1;
                LOOK:if(slot_valid && !slot_hit && capacity && !invalidate) begin
                    fill_addr<=!hit_a ? {slot_a[31:5],5'b0} : {slot_b[31:5],5'b0};
                    fill_set<=!hit_a ? set_a : set_b;fill_way<=!hit_a ? victim_a : victim_b;state<=COMMAND;
                end
                COMMAND:if(rd_ready) state<=RESPONSE;
                RESPONSE:if(rsp_valid) begin miss_lines<=miss_lines+1;state<=RELOAD;end
                RELOAD:state<=LOOK;
                default:state<=CLEAR;
            endcase
            if(rsp_valid && state!=RESPONSE) protocol_error<=1;
            if(invalidate) begin
                if((state!=LOOK && state!=CLEAR) || slot_valid || select_valid || out_valid) protocol_error<=1;
                slot_valid<=0;select_valid<=0;out_valid<=0;hit_pairs<=0;miss_lines<=0;
                if(&epoch) begin epoch<=1;clear_set<=0;state<=CLEAR;end else epoch<=epoch+1'b1;
            end
        end
    end
endmodule
