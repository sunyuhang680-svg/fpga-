`timescale 1ns/1ps
// Live path: per-pixel undistortion straight from the on-chip row ring.
//
// One request per clock: output pixel (x,y) of a W x H frame.  Its source
// position is the same model the DDR engine uses, evaluated at full camera
// resolution:
//   xn = ax*x + bx, yn = ay*y + by                (normalized, Q30)
//   r2 = xn^2+yn^2, rad = 1 + k1 r2 + k2 r2^2
//   xd = xn rad + 2 p1 xn yn + p2 (r2 + 2 xn^2)
//   yd = yn rad + p1 (r2 + 2 yn^2) + 2 p2 xn yn
//   us = fx xd + cx, vs = fy yd + cy               (source pixels, Q16)
// bypass=1 gives us=x, vs=y (raw camera through the same path).
// The 2x2 neighbourhood is read from the ring of the selected camera and
// blended with 8-bit weights (bilinear_rgb565).  Pixels outside the camera
// image are black; pixels whose rows are not (or no longer) in the ring are
// black and counted in row_miss.  The request tag comes out with the pixel
// after a fixed LATENCY.
module live_undistort_stream #(
    parameter integer WIDTH=1280,
    parameter integer HEIGHT=720,
    parameter integer RING_LOG2=6,
    parameter integer TAG_W=3,
    parameter integer BANK_AW=$clog2(((1<<RING_LOG2)/2)*(WIDTH/2))
)(
    input wire clk,rst_n,
    input wire frame_start,               // latch camera, bypass and row base
    input wire cam_sel_in,bypass_in,
    input wire[383:0]coeff0,coeff1,       // live_calibration_constants format
    input wire[15:0]base0,base1,rows0,rows1,
    input wire req_valid,
    input wire[11:0]req_x,req_y,
    input wire[TAG_W-1:0]req_tag,
    output wire[BANK_AW-1:0]addr00,addr01,addr10,addr11,
    input wire[15:0]ram0_00,ram0_01,ram0_10,ram0_11,
    input wire[15:0]ram1_00,ram1_01,ram1_10,ram1_11,
    output wire out_valid,
    output wire[15:0]out_pixel,
    output wire[TAG_W-1:0]out_tag,
    output reg row_miss,                  // pulse: a pixel needed a missing row
    // debug/verification view of the source coordinate (not used by Top)
    output reg dbg_valid,output reg signed[31:0]dbg_us,dbg_vs
);
    localparam integer HALF_W=WIDTH/2;
    localparam integer N=11;              // request delay line length (s1..s12)
    localparam signed [33:0] ONE=34'sd1073741824;
    localparam signed [31:0] UMAX=(WIDTH-1)*65536,VMAX=(HEIGHT-1)*65536;

    reg cam_sel,bypass;reg[15:0]base;
    reg signed[31:0]ax,bx,ay,by,k1,k2,p1,p2,fx,fy,cx,cy;
    wire[383:0]coeff=cam_sel_in?coeff1:coeff0;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            cam_sel<=0;bypass<=0;base<=0;
            {ax,bx,ay,by,k1,k2,p1,p2,fx,fy,cx,cy}<=0;
        end else if(frame_start)begin
            cam_sel<=cam_sel_in;bypass<=bypass_in;base<=cam_sel_in?base1:base0;
            {cy,cx,fy,fx,p2,p1,k2,k1,by,ay,bx,ax}<=coeff;
        end
    end
    wire[15:0]rows=cam_sel?rows1:rows0;

    // valid/tag/x/y travel with the data
    reg[N:0]v;reg[TAG_W-1:0]t[0:N];reg[11:0]xq[0:N];reg[11:0]yq[0:N];
    integer i;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin v<=0;for(i=0;i<=N;i=i+1)begin t[i]<=0;xq[i]<=0;yq[i]<=0;end end
        else begin
            v<={v[N-1:0],req_valid};t[0]<=req_tag;xq[0]<=req_x;yq[0]<=req_y;
            for(i=1;i<=N;i=i+1)begin t[i]<=t[i-1];xq[i]<=xq[i-1];yq[i]<=yq[i-1];end
        end
    end

    // s1..s12: coordinate map
    reg signed[47:0]pu,pv;                          // s1  Q40
    reg signed[33:0]xn,yn;                          // s2  Q30
    reg signed[67:0]pxx,pyy,pxy;                    // s3
    reg signed[33:0]xn3,yn3,xn4,yn4,xn5,yn5,xn6,yn6,xn7,yn7,xn8,yn8;
    reg signed[33:0]r2,tx,ty,xy;                    // s4
    reg signed[67:0]pr4,pk1,pp2tx,pp1ty,pp1xy,pp2xy;// s5
    reg signed[33:0]r4,k1r2,addx,addy;              // s6
    reg signed[67:0]pk2;reg signed[33:0]s7,addx7,addy7;      // s7
    reg signed[33:0]rad,addx8,addy8;                // s8
    reg signed[67:0]pxr,pyr;reg signed[33:0]addx9,addy9;     // s9
    reg signed[33:0]xd,yd;                          // s10
    reg signed[67:0]pus,pvs;                        // s11
    reg signed[31:0]us,vs;                          // s12 Q16
    function signed[33:0] q30(input signed[67:0]p);q30=p>>>30;endfunction
    always @(posedge clk)begin
        pu<=ax*$signed({1'b0,req_x});pv<=ay*$signed({1'b0,req_y});
        xn<=(pu>>>10)+bx;yn<=(pv>>>10)+by;
        pxx<=xn*xn;pyy<=yn*yn;pxy<=xn*yn;xn3<=xn;yn3<=yn;
        r2<=q30(pxx)+q30(pyy);tx<=q30(pxx)+q30(pyy)+(q30(pxx)<<<1);ty<=q30(pxx)+q30(pyy)+(q30(pyy)<<<1);
        xy<=q30(pxy);xn4<=xn3;yn4<=yn3;
        pr4<=r2*r2;pk1<=k1*r2;pp2tx<=p2*tx;pp1ty<=p1*ty;pp1xy<=p1*xy;pp2xy<=p2*xy;xn5<=xn4;yn5<=yn4;
        r4<=q30(pr4);k1r2<=q30(pk1);addx<=q30(pp2tx)+(q30(pp1xy)<<<1);addy<=q30(pp1ty)+(q30(pp2xy)<<<1);xn6<=xn5;yn6<=yn5;
        pk2<=k2*r4;s7<=ONE+k1r2;addx7<=addx;addy7<=addy;xn7<=xn6;yn7<=yn6;
        rad<=s7+q30(pk2);addx8<=addx7;addy8<=addy7;xn8<=xn7;yn8<=yn7;
        pxr<=xn8*rad;pyr<=yn8*rad;addx9<=addx8;addy9<=addy8;
        xd<=q30(pxr)+addx9;yd<=q30(pyr)+addy9;
        pus<=fx*xd;pvs<=fy*yd;
        if(bypass)begin us<={4'd0,xq[10],16'd0};vs<={4'd0,yq[10],16'd0};end
        else begin us<=q30(pus)+cx;vs<=q30(pvs)+cy;end
    end
    always @(posedge clk or negedge rst_n)
        if(!rst_n)begin dbg_valid<=0;dbg_us<=0;dbg_vs<=0;end
        else begin dbg_valid<=v[11];dbg_us<=us;dbg_vs<=vs;end

    // s13: integer/fraction, availability, bank addresses
    wire in_img=us>=0 && vs>=0 && us<=UMAX && vs<=VMAX;
    wire[11:0]ui=us[27:16],vi=vs[27:16];
    wire[7:0]fxw=(ui==WIDTH-1)?8'd0:us[15:8];
    wire[7:0]fyw=(vi==HEIGHT-1)?8'd0:vs[15:8];
    wire[15:0]a0=base+{4'd0,vi};
    wire[15:0]a1=a0+((fyw!=0)?16'd1:16'd0);
    wire signed[15:0]d0=rows-a0,d1=rows-a1;  // rows completed beyond each needed row
    wire avail=(d1>=16'sd1) && (d0<=(16'sd1<<<RING_LOG2)-16'sd2);
    wire[15:0]a0p=a0+16'd1;
    wire[RING_LOG2-1:0]s0=a0[RING_LOG2-1:0],s1=a0p[RING_LOG2-1:0];
    wire[11:0]c0=ui,c1=(ui>=WIDTH-1)?ui:ui+12'd1;
    wire[RING_LOG2-2:0]slot_even=s0[0]?s1[RING_LOG2-1:1]:s0[RING_LOG2-1:1];
    wire[RING_LOG2-2:0]slot_odd =s0[0]?s0[RING_LOG2-1:1]:s1[RING_LOG2-1:1];
    wire[11:0]col_even=c0[0]?c1:c0,col_odd=c0[0]?c0:c1;
    reg[BANK_AW-1:0]ra00,ra01,ra10,ra11;
    reg row0_odd,col0_odd,in13,ok13;reg[7:0]fx13,fy13;reg[TAG_W-1:0]t13;reg v13;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            ra00<=0;ra01<=0;ra10<=0;ra11<=0;row0_odd<=0;col0_odd<=0;in13<=0;ok13<=0;fx13<=0;fy13<=0;t13<=0;v13<=0;row_miss<=0;
        end else begin
            ra00<=slot_even*HALF_W+col_even[11:1];ra01<=slot_even*HALF_W+col_odd[11:1];
            ra10<=slot_odd*HALF_W+col_even[11:1];ra11<=slot_odd*HALF_W+col_odd[11:1];
            row0_odd<=s0[0];col0_odd<=c0[0];in13<=in_img;ok13<=in_img && avail;
            fx13<=fxw;fy13<=fyw;t13<=t[11];v13<=v[11];
            row_miss<=v[11] && in_img && !avail;
        end
    end
    assign addr00=ra00;assign addr01=ra01;assign addr10=ra10;assign addr11=ra11;

    // s14: RAM data valid (registered read); s15: arrange the 2x2 neighbourhood
    reg row0_odd14,col0_odd14,ok14,cam14,v14;reg[7:0]fx14,fy14;reg[TAG_W-1:0]t14;
    reg[15:0]p00,p10,p01,p11;reg[7:0]fx15,fy15;reg[TAG_W:0]t15;reg v15;
    wire[15:0]e00=cam14?ram1_00:ram0_00,e01=cam14?ram1_01:ram0_01,e10=cam14?ram1_10:ram0_10,e11=cam14?ram1_11:ram0_11;
    // e[row parity][col parity]; p[col offset][row offset] as in bilinear_rgb565
    wire[15:0]r0c0=row0_odd14?(col0_odd14?e11:e10):(col0_odd14?e01:e00);
    wire[15:0]r0c1=row0_odd14?(col0_odd14?e10:e11):(col0_odd14?e00:e01);
    wire[15:0]r1c0=row0_odd14?(col0_odd14?e01:e00):(col0_odd14?e11:e10);
    wire[15:0]r1c1=row0_odd14?(col0_odd14?e00:e01):(col0_odd14?e10:e11);
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            row0_odd14<=0;col0_odd14<=0;ok14<=0;cam14<=0;v14<=0;fx14<=0;fy14<=0;t14<=0;
            p00<=0;p10<=0;p01<=0;p11<=0;fx15<=0;fy15<=0;t15<=0;v15<=0;
        end else begin
            row0_odd14<=row0_odd;col0_odd14<=col0_odd;ok14<=ok13;cam14<=cam_sel;v14<=v13;fx14<=fx13;fy14<=fy13;t14<=t13;
            p00<=r0c0;p10<=r0c1;p01<=r1c0;p11<=r1c1;fx15<=fx14;fy15<=fy14;t15<={ok14,t14};v15<=v14;
        end
    end
    wire[TAG_W:0]bt;wire[15:0]bp;wire bv;
    bilinear_rgb565 #(.TAG_W(TAG_W+1)) blend(.clk(clk),.rst_n(rst_n),.in_valid(v15),.in_ready(),
        .p00(p00),.p10(p10),.p01(p01),.p11(p11),.ax(fx15),.ay(fy15),.in_tag(t15),
        .out_valid(bv),.out_ready(1'b1),.out_pixel(bp),.out_tag(bt));
    assign out_valid=bv;
    assign out_pixel=bt[TAG_W]?bp:16'd0;
    assign out_tag=bt[TAG_W-1:0];
endmodule
