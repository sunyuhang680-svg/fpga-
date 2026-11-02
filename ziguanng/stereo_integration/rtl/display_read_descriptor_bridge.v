`timescale 1ns/1ps
// Bundle the selected address, buffer index and tag with each HDMI read request.
// The source holds the bundle through the entire four-phase req/ack exchange.
// No asynchronous address mux feeds the mature memory-domain frame reader.
module display_read_descriptor_bridge #(
    parameter ADDR_BITS=25
)(
    input wire pixel_clk,mem_clk,rst_n,
    input wire pixel_req,
    output wire pixel_ack,
    input wire[ADDR_BITS-1:0]pixel_base,
    input wire[1:0]pixel_index,
    input wire pixel_tag,
    output reg mem_req,
    input wire mem_ack,
    output reg[ADDR_BITS-1:0]mem_base,
    output reg[1:0]mem_index,
    output reg mem_tag
);
    localparam IDLE=0,WAIT_ACK=1,WAIT_REQ_LOW=2,WAIT_ACK_LOW=3;
    reg[1:0]source_state;
    reg source_active;
    reg[ADDR_BITS+2:0]held;
    (* ASYNC_REG="TRUE" *)reg ack_meta,ack_sync;
    (* ASYNC_REG="TRUE" *)reg active_meta,active_sync;
    reg[1:0]memory_state;
    // rst_n (ddr_init_done) comes from the DDR domain. Assert asynchronously,
    // release synchronously in each domain so no flop sees a recovery race.
    // [Claude review 2026-10-22] added; protocol unchanged.
    (* ASYNC_REG="TRUE" *)reg[1:0]pixel_reset_sync,memory_reset_sync;
    wire pixel_rst_n=pixel_reset_sync[1],memory_rst_n=memory_reset_sync[1];
    always @(posedge pixel_clk or negedge rst_n)begin
        if(!rst_n)pixel_reset_sync<=2'b00;else pixel_reset_sync<={pixel_reset_sync[0],1'b1};
    end
    always @(posedge mem_clk or negedge rst_n)begin
        if(!rst_n)memory_reset_sync<=2'b00;else memory_reset_sync<={memory_reset_sync[0],1'b1};
    end
    assign pixel_ack=(source_state!=IDLE) && ack_sync;
    always @(posedge pixel_clk or negedge pixel_rst_n)begin
        if(!pixel_rst_n)begin
            source_state<=IDLE;source_active<=0;held<=0;ack_meta<=0;ack_sync<=0;
        end else begin
            ack_meta<=mem_ack;ack_sync<=ack_meta;
            case(source_state)
                IDLE:if(pixel_req)begin
                    held<={pixel_tag,pixel_index,pixel_base};source_active<=1;source_state<=WAIT_ACK;
                end
                WAIT_ACK:if(ack_sync)begin
                    if(pixel_req)source_state<=WAIT_REQ_LOW;
                    else begin source_active<=0;source_state<=WAIT_ACK_LOW;end
                end
                WAIT_REQ_LOW:if(!pixel_req)begin source_active<=0;source_state<=WAIT_ACK_LOW;end
                WAIT_ACK_LOW:if(!ack_sync)source_state<=IDLE;
                default:source_state<=IDLE;
            endcase
        end
    end
    always @(posedge mem_clk or negedge memory_rst_n)begin
        if(!memory_rst_n)begin
            active_meta<=0;active_sync<=0;memory_state<=0;
            mem_req<=0;mem_base<=0;mem_index<=0;mem_tag<=0;
        end else begin
            active_meta<=source_active;active_sync<=active_meta;
            case(memory_state)
                0:if(active_sync)begin
                    {mem_tag,mem_index,mem_base}<=held;mem_req<=1;memory_state<=1;
                end
                1:if(!active_sync)begin mem_req<=0;memory_state<=2;end
                2:if(!mem_ack)memory_state<=0;
                default:memory_state<=0;
            endcase
        end
    end
endmodule
