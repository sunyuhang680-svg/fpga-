`timescale 1ns/1ps
// Existing aggregate read channel plus one independent, bounded auxiliary client.
// Addresses: 256-bit word indices. Lengths: number of 256-bit beats.
module ddr_aux_read_mux #(
    parameter ADDR_BITS=25,
    parameter BURST_BITS=10,
    parameter ENABLED=1,
    parameter TIMEOUT_CYCLES=4096
)(
    input wire clk,rst_n,
    input wire legacy_req,
    input wire [ADDR_BITS-1:0] legacy_addr,
    input wire [BURST_BITS-1:0] legacy_len,
    input wire legacy_urgent,
    output wire legacy_data_valid,
    output wire [255:0] legacy_data,
    output wire legacy_done,
    input wire aux_valid,
    output wire aux_ready,
    input wire [ADDR_BITS-1:0] aux_addr,
    input wire [BURST_BITS-1:0] aux_len,
    output wire aux_data_valid,
    output wire [255:0] aux_data,
    output wire aux_done,
    output wire fault,
    output wire mem_req,
    output wire [ADDR_BITS-1:0] mem_addr,
    output wire [BURST_BITS-1:0] mem_len,
    input wire mem_ready,
    input wire mem_data_valid,
    input wire [255:0] mem_data,
    input wire mem_done
);
generate if(ENABLED==0) begin: bypass
    assign mem_req=legacy_req;
    assign mem_addr=legacy_addr;
    assign mem_len=legacy_len;
    assign legacy_data_valid=mem_data_valid;
    assign legacy_data=mem_data;
    assign legacy_done=mem_done;
    assign aux_ready=1'b0; assign aux_data_valid=1'b0;
    assign aux_data=256'd0; assign aux_done=1'b0; assign fault=1'b0;
end else begin: arb
    localparam IDLE=2'd0,ACTIVE=2'd1,DRAIN=2'd2,HALTED=2'd3;
    reg [1:0] state;
    reg owner_aux,last_aux,timeout_fault,aux_blocked;
    reg [31:0] elapsed;
    reg [ADDR_BITS-1:0] held_addr;
    reg [BURST_BITS-1:0] held_len;
    wire legacy_pending=legacy_req && (legacy_len!=0);
    wire aux_legal=(aux_len>0 && aux_len<=16);
    // [Claude review 2026-10-22] A fault only blocks the auxiliary client.
    // The legacy display/renderer channel keeps being served, so an aux
    // problem can never freeze the teammate's HDMI path.
    wire select_aux=!aux_blocked && aux_valid && aux_legal &&
        (!legacy_pending || (!legacy_urgent && !last_aux));
    wire select_any=legacy_pending || select_aux;
    wire grant=(state==IDLE) && mem_ready && select_any && !mem_data_valid && !mem_done;
    assign mem_req=grant;
    assign mem_addr=(state==IDLE)?(select_aux?aux_addr:legacy_addr):held_addr;
    assign mem_len=(state==IDLE)?(select_aux?aux_len:legacy_len):held_len;
    assign aux_ready=grant && select_aux;
    assign legacy_data_valid=(state==ACTIVE || state==DRAIN) && !owner_aux && mem_data_valid;
    assign legacy_data=mem_data;
    assign legacy_done=(state==ACTIVE || state==DRAIN) && !owner_aux && mem_done;
    // A timed-out auxiliary response is drained but never exposed as valid pixels.
    assign aux_data_valid=(state==ACTIVE) && owner_aux && !timeout_fault && mem_data_valid;
    assign aux_data=mem_data;
    assign aux_done=(state==ACTIVE || state==DRAIN) && owner_aux && mem_done;
    assign fault=timeout_fault;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;owner_aux<=0;last_aux<=1;timeout_fault<=0;aux_blocked<=0;
            elapsed<=0;held_addr<=0;held_len<=0;
        end else case(state)
            // Unsolicited beat: record, block aux, never route it; legacy continues
            // (grant is already suppressed while mem_data_valid/mem_done is high).
            IDLE: if(mem_data_valid || mem_done) begin
                timeout_fault<=1;aux_blocked<=1;
            end else if(grant) begin
                owner_aux<=select_aux;last_aux<=select_aux;elapsed<=0;
                held_addr<=mem_addr;held_len<=mem_len;state<=ACTIVE;
            end
            // Only auxiliary bursts are timed; a slow legacy burst is normal DDR
            // contention and must not be treated as a fault.
            ACTIVE: if(mem_done) state<=IDLE;
                else if(owner_aux && elapsed>=TIMEOUT_CYCLES-1) begin
                    timeout_fault<=1;aux_blocked<=1;state<=DRAIN;
                end else if(owner_aux) elapsed<=elapsed+1'b1;
            DRAIN: if(mem_done) state<=IDLE;
            default: state<=IDLE;
        endcase
    end
end endgenerate
endmodule
