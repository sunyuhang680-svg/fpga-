`timescale 1ns/1ps
// Bridge native byte-addressed core transactions to the existing one-beat bus.
// core write acceptance means buffered, NOT committed; wait writes_idle before publishing.
module core_native_burst_adapter #(
    parameter ADDR_BITS=25,parameter BURST_BITS=10
)(
    input wire clk,rst_n,
    input wire core_rd_valid,output wire core_rd_ready,input wire[31:0]core_rd_addr,
    output reg core_rsp_valid,output reg[255:0]core_rsp_data,
    input wire core_wr_valid,output wire core_wr_ready,input wire[31:0]core_wr_addr,
    input wire[255:0]core_wr_data,input wire[31:0]core_wr_strb,
    output reg rd_burst_req,output reg[ADDR_BITS-1:0]rd_burst_addr,output wire[BURST_BITS-1:0]rd_burst_len,
    input wire rd_burst_data_valid,input wire[255:0]rd_burst_data,input wire rd_burst_finish,
    output reg wr_burst_req,output reg[ADDR_BITS-1:0]wr_burst_addr,output wire[BURST_BITS-1:0]wr_burst_len,
    input wire wr_burst_data_req,output reg[255:0]wr_burst_data,input wire wr_burst_finish,
    output wire reads_idle,writes_idle,output reg fault
);
    reg read_pending,read_seen,write_pending,write_seen;
    assign core_rd_ready=!read_pending && !fault;
    assign core_wr_ready=!write_pending && !fault;
    assign rd_burst_len=1;assign wr_burst_len=1;
    assign reads_idle=!read_pending;assign writes_idle=!write_pending;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            read_pending<=0;read_seen<=0;write_pending<=0;write_seen<=0;fault<=0;
            rd_burst_req<=0;rd_burst_addr<=0;core_rsp_valid<=0;core_rsp_data<=0;
            wr_burst_req<=0;wr_burst_addr<=0;wr_burst_data<=0;
        end else begin
            core_rsp_valid<=0;
            if(core_rd_valid && core_rd_ready)begin
                if(core_rd_addr[4:0]!=0 || core_rd_addr[31:30]!=0)fault<=1;
                else begin read_pending<=1;read_seen<=0;rd_burst_req<=1;rd_burst_addr<=core_rd_addr[29:5];end
            end
            if(rd_burst_data_valid)begin
                if(!read_pending || read_seen)fault<=1;
                else begin core_rsp_data<=rd_burst_data;core_rsp_valid<=1;read_seen<=1;rd_burst_req<=0;end
            end
            if(rd_burst_finish)begin
                if(!read_pending || (!read_seen && !rd_burst_data_valid))fault<=1;
                read_pending<=0;rd_burst_req<=0;
            end
            if(core_wr_valid && core_wr_ready)begin
                if(core_wr_addr[4:0]!=0 || core_wr_addr[31:30]!=0 || core_wr_strb!=32'hffffffff)fault<=1;
                else begin
                    write_pending<=1;write_seen<=0;wr_burst_req<=1;
                    wr_burst_addr<=core_wr_addr[29:5];wr_burst_data<=core_wr_data;
                end
            end
            if(wr_burst_data_req)begin
                if(!write_pending || write_seen)fault<=1;
                write_seen<=1;wr_burst_req<=0;
            end
            if(wr_burst_finish)begin
                if(!write_pending || (!write_seen && !wr_burst_data_req))fault<=1;
                write_pending<=0;wr_burst_req<=0;
            end
        end
    end
endmodule
