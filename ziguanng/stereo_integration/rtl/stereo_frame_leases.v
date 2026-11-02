`timescale 1ns/1ps
// Two clients / two cameras. An active cohort cannot be extended by late clients.
// Releasing the cohort drops freeze, waits for acknowledgement, then acquires fresh.
module stereo_frame_leases(
    input wire clk,rst_n,
    input wire left_request,right_request,left_camera,right_camera,
    input wire cam0_valid,cam1_valid,
    input wire[1:0]cam0_index,cam1_index,
    input wire[31:0]cam0_tag,cam1_tag,
    output wire cam0_hold,cam1_hold,
    output wire left_grant,right_grant,left_granted_camera,right_granted_camera,
    output wire[24:0]left_base,right_base,
    output wire[31:0]left_tag,right_tag
);
    reg[1:0]state0,state1,owners0,owners1;
    wire[1:0]requests={right_request,left_request};
    wire[1:0]want0={right_request&&!right_camera,left_request&&!left_camera};
    wire[1:0]want1={right_request&&right_camera,left_request&&left_camera};
    wire[1:0]remaining0=owners0&requests,remaining1=owners1&requests;
    wire[1:0]grant0=(state0==1 && cam0_valid)?remaining0:2'b00;
    wire[1:0]grant1=(state1==1 && cam1_valid)?remaining1:2'b00;
    wire[24:0]base0=(cam0_index==0)?25'd0:(cam0_index==1)?25'd2073600:25'd4147200;
    wire[24:0]base1=(cam1_index==0)?25'd8294400:(cam1_index==1)?25'd10368000:25'd12441600;
    assign cam0_hold=state0==1;assign cam1_hold=state1==1;
    assign left_grant=grant0[0] || grant1[0];assign right_grant=grant0[1] || grant1[1];
    assign left_granted_camera=grant1[0];assign right_granted_camera=grant1[1];
    assign left_base=grant1[0]?base1:base0;assign right_base=grant1[1]?base1:base0;
    assign left_tag=grant1[0]?cam1_tag:cam0_tag;assign right_tag=grant1[1]?cam1_tag:cam0_tag;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin state0<=0;state1<=0;owners0<=0;owners1<=0;end
        else begin
            case(state0)
                0:if((want0&~owners1)!=0)begin owners0<=want0&~owners1;state0<=1;end
                1:begin owners0<=remaining0;if(remaining0==0 && cam0_valid)state0<=2;end
                2:if(!cam0_valid)state0<=0;
            endcase
            case(state1)
                0:if((want1&~owners0)!=0)begin owners1<=want1&~owners0;state1<=1;end
                1:begin owners1<=remaining1;if(remaining1==0 && cam1_valid)state1<=2;end
                2:if(!cam1_valid)state1<=0;
            endcase
        end
    end
endmodule
