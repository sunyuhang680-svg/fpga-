`timescale 1ns/1ps
// Optional preflight, not yet wired into the board controller.
// With positive affine denominator the normalized image is in the convex hull
// of its four normalized corners. The L1 ball is convex, so these four checks
// bound every output ray without division. A separate calibration certificate
// must prove Brown(x,y) remains inside the mapper's +/-4 arithmetic domain.
// This guard does NOT prove source-image bounds, positive lens Jacobian, or crop.
module pose_ray_domain_guard(
    input wire clk,rst_n,start,
    input wire[359:0]homography,
    input wire[11:0]width,height,
    output reg busy,done,error
);
    localparam IDLE=0,COORD=1,MUL=2,SUM=3,MAGNITUDE=4,BOUND=5;
    reg[2:0]state;
    reg[1:0]corner;
    reg[359:0]held_h;
    reg[11:0]w,h;
    reg signed[12:0]x,y;
    reg signed[52:0]px[0:2],py[0:2];
    reg signed[63:0]dot[0:2];
    reg[63:0]magnitude_sum;
    reg signed[63:0]denominator;
    wire[63:0]abs_x=dot[0][63]?-dot[0]:dot[0];
    wire[63:0]abs_y=dot[1][63]?-dot[1]:dot[1];
    integer row;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;corner<=0;held_h<=0;w<=0;h<=0;x<=0;y<=0;
            magnitude_sum<=0;denominator<=0;busy<=0;done<=0;error<=0;
            for(row=0;row<3;row=row+1)begin px[row]<=0;py[row]<=0;dot[row]<=0;end
        end else begin
            done<=0;
            case(state)
                IDLE:if(start)begin
                    held_h<=homography;w<=width;h<=height;corner<=0;busy<=1;error<=0;
                    if(width<2 || height<2)begin error<=1;busy<=0;done<=1;end
                    else state<=COORD;
                end
                COORD:begin
                    x<=corner[0]?$signed({1'b0,w})-13'sd1:13'sd0;
                    y<=corner[1]?$signed({1'b0,h})-13'sd1:13'sd0;
                    state<=MUL;
                end
                MUL:begin
                    for(row=0;row<3;row=row+1)begin
                        px[row]<=$signed(held_h[40*(row*3)+:40])*x;
                        py[row]<=$signed(held_h[40*(row*3+1)+:40])*y;
                    end
                    state<=SUM;
                end
                SUM:begin
                    for(row=0;row<3;row=row+1)
                        dot[row]<=$signed(px[row])+$signed(py[row])+$signed(held_h[40*(row*3+2)+:40]);
                    state<=MAGNITUDE;
                end
                MAGNITUDE:begin
                    magnitude_sum<=abs_x+abs_y;denominator<=dot[2];state<=BOUND;
                end
                BOUND:begin
                    // Max H and 12-bit coordinates fit well below 64-bit range.
                    if(denominator<64'sd4194304 || (magnitude_sum<<3)>
                        $unsigned((denominator<<3)+(denominator<<1)+denominator))begin
                        error<=1;busy<=0;done<=1;state<=IDLE;
                    end else if(corner==3)begin busy<=0;done<=1;state<=IDLE;end
                    else begin corner<=corner+1'b1;state<=COORD;end
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
