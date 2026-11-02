// SPDX-License-Identifier: MIT
// Latest-state mailbox. Held bus remains stable until destination acknowledges.
// Reports faster than the handshake are coalesced to the newest complete state.
module touch_event_cdc #(parameter WIDTH=66)(
    input wire src_clk, dst_clk, rst_n,
    input wire src_event,
    input wire [WIDTH-1:0] src_data,
    output reg dst_event,
    output reg [WIDTH-1:0] dst_data
);
    reg [WIDTH-1:0] latest, held;
    reg dirty, request, acknowledge;
    (* ASYNC_REG="TRUE" *) reg ack_meta, ack_sync, req_meta, req_sync;
    always @(posedge src_clk or negedge rst_n) begin
        if (!rst_n) begin
            latest<=0; held<=0; dirty<=0; request<=0;
            ack_meta<=0; ack_sync<=0;
        end else begin
            ack_meta<=acknowledge; ack_sync<=ack_meta;
            if ((request==ack_sync) && dirty) begin
                held<=latest; request<=~request; dirty<=0;
            end
            if (src_event) begin latest<=src_data; dirty<=1; end
        end
    end
    always @(posedge dst_clk or negedge rst_n) begin
        if (!rst_n) begin
            req_meta<=0; req_sync<=0; acknowledge<=0;
            dst_data<=0; dst_event<=0;
        end else begin
            req_meta<=request; req_sync<=req_meta; dst_event<=0;
            if (req_sync!=acknowledge) begin
                dst_data<=held; dst_event<=1; acknowledge<=req_sync;
            end
        end
    end
endmodule
