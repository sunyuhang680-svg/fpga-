`timescale 1ns/1ps
// Live mode statistics, drawn over the video in the top-left corner.
//
//   LIVE CAM0          (CAM1, or RAW when correction is bypassed)
//   FPS  30.00         display frames per second, from the measured frame period
//   LAT  752US         camera row complete -> same row on HDMI (row0_to_start)
//   LINE 2795 V 750    line length in use / lines of the last frame
//   MISS 0             rows the ring did not hold in the last frame (0 when locked)
//
// 5x7 font, each font pixel drawn 2x2, white on a dark box.  Values are
// latched once per frame and converted to decimal with sequential
// dividers, so the drawing path is a font lookup and a mux.
// The output is the input stream delayed by two clocks.
module live_stats_overlay #(
    parameter integer CLK_HZ=74250000,
    parameter integer X0=16,Y0=16
)(
    input wire clk,rst_n,
    input wire enable,
    input wire hs_in,vs_in,de_in,input wire[15:0]pixel_in,
    output reg hs_out,vs_out,de_out,output reg[15:0]pixel_out,
    input wire frame_start,
    input wire cam_sel,bypass,locked,
    input wire[31:0]row0_to_start,
    input wire[11:0]h_total,v_total_last,
    input wire[15:0]miss_count
);
    localparam integer COLS=16,ROWS=5,CW=12,CH=18;
    localparam integer BOX_W=COLS*CW+8,BOX_H=ROWS*CH+6;

    // ---- frame period ----
    reg[31:0]cnt,period;reg new_period;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)begin cnt<=0;period<=0;new_period<=0;end
        else begin
            new_period<=0;
            if(frame_start)begin period<=cnt+32'd1;cnt<=0;new_period<=1;end
            else if(cnt!=32'hffffffff)cnt<=cnt+32'd1;
        end

    // ---- numbers -> decimal digits (one value at a time) ----
    // fps*100 = CLK_HZ*100/period, lat_us = row0_to_start*1e6/CLK_HZ
    localparam [39:0] FPS_NUM=CLK_HZ*64'd100;
    localparam [2:0] S_IDLE=0,S_DIV=1,S_PREP=2,S_DIG=3,S_STORE=4;
    reg[2:0]st,job;
    reg[39:0]num,quo,rem;reg[31:0]den;reg[5:0]bit_i;
    reg[19:0]val;reg[3:0]dg[0:4];reg[2:0]dpos;
    reg[3:0]d_fps[0:4],d_lat[0:4],d_ht[0:4],d_vt[0:4],d_miss[0:4];
    reg locked_l,cam_l,bypass_l;
    reg[31:0]lat_l;reg[11:0]ht_l,vt_l;reg[15:0]miss_l;
    wire[40:0]rem_sh={rem[39:0],num[bit_i]};
    integer k;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            st<=S_IDLE;job<=0;num<=0;den<=1;quo<=0;rem<=0;bit_i<=0;val<=0;dpos<=0;
            locked_l<=0;cam_l<=0;bypass_l<=0;lat_l<=0;ht_l<=0;vt_l<=0;miss_l<=0;
            for(k=0;k<5;k=k+1)begin dg[k]<=0;d_fps[k]<=0;d_lat[k]<=0;d_ht[k]<=0;d_vt[k]<=0;d_miss[k]<=0;end
        end else case(st)
            S_IDLE:if(new_period)begin
                locked_l<=locked;cam_l<=cam_sel;bypass_l<=bypass;
                lat_l<=row0_to_start;ht_l<=h_total;vt_l<=v_total_last;miss_l<=miss_count;
                job<=0;begin num<=FPS_NUM;den<=(period==0)?32'd1:period;quo<=0;rem<=0;bit_i<=6'd39;st<=S_DIV;end
            end
            S_DIV:begin   // restoring division, one quotient bit per clock
                if(rem_sh>={9'd0,den})begin rem<=rem_sh-{9'd0,den};quo[bit_i]<=1'b1;end
                else rem<=rem_sh[39:0];
                if(bit_i==0)st<=S_PREP;else bit_i<=bit_i-6'd1;
            end
            S_PREP:begin val<=(quo[39:20]!=0)?20'd99999:(quo[19:0]>20'd99999)?20'd99999:quo[19:0];dpos<=0;st<=S_DIG;end
            S_DIG:begin   // five decimal digits, least significant first
                dg[dpos]<=val%10;val<=val/10;
                if(dpos==3'd4)st<=S_STORE;else dpos<=dpos+3'd1;
            end
            default:begin
                for(k=0;k<5;k=k+1)case(job)
                    3'd0:d_fps[k]<=dg[k];3'd1:d_lat[k]<=dg[k];3'd2:d_ht[k]<=dg[k];3'd3:d_vt[k]<=dg[k];default:d_miss[k]<=dg[k];
                endcase
                job<=job+3'd1;
                case(job)
                    3'd0:begin num<=((lat_l[31:20]!=0)?40'd1048575:{8'd0,lat_l})*40'd1000000;den<=CLK_HZ;quo<=0;rem<=0;bit_i<=6'd39;st<=S_DIV;end
                    3'd1:begin num<={28'd0,ht_l};den<=1;quo<=0;rem<=0;bit_i<=6'd39;st<=S_DIV;end
                    3'd2:begin num<={28'd0,vt_l};den<=1;quo<=0;rem<=0;bit_i<=6'd39;st<=S_DIV;end
                    3'd3:begin num<={24'd0,miss_l};den<=1;quo<=0;rem<=0;bit_i<=6'd39;st<=S_DIV;end
                    default:st<=S_IDLE;
                endcase
            end
        endcase
    end

    // ---- text ----
    // character codes: 0-9 digits, 10 ' ', 11 '.', 12.. letters
    localparam [5:0] SP=10,DOT=11,A_=12,C_=13,E_=14,F_=15,I_=16,L_=17,M_=18,N_=19,P_=20,S_=21,T_=22,U_=23,V_=24,W_=25,R_=26,K_=27,O_=28;
    function [5:0] nz;input[3:0]d;input lead;nz=lead?SP:{2'd0,d};endfunction
    reg[5:0]ch;
    reg[4:0]tc;reg[2:0]tr;
    always @*begin
        ch=SP;
        case(tr)
        3'd0:case(tc)
            0:ch=L_;1:ch=I_;2:ch=V_;3:ch=E_;5:ch=bypass_l?R_:C_;6:ch=A_;7:ch=bypass_l?W_:M_;8:ch=bypass_l?SP:(cam_l?6'd1:6'd0);
            10:ch=locked_l?L_:SP;11:ch=locked_l?O_:SP;12:ch=locked_l?C_:SP;13:ch=locked_l?K_:SP;default:ch=SP;endcase
        3'd1:case(tc)
            0:ch=F_;1:ch=P_;2:ch=S_;
            5:ch=nz(d_fps[4],d_fps[4]==0);6:ch=nz(d_fps[3],d_fps[4]==0&&d_fps[3]==0);7:ch=d_fps[2];8:ch=DOT;9:ch=d_fps[1];10:ch=d_fps[0];default:ch=SP;endcase
        3'd2:case(tc)
            0:ch=L_;1:ch=A_;2:ch=T_;
            5:ch=nz(d_lat[4],d_lat[4]==0);6:ch=nz(d_lat[3],d_lat[4]==0&&d_lat[3]==0);7:ch=nz(d_lat[2],d_lat[4]==0&&d_lat[3]==0&&d_lat[2]==0);
            8:ch=nz(d_lat[1],d_lat[4]==0&&d_lat[3]==0&&d_lat[2]==0&&d_lat[1]==0);9:ch=d_lat[0];10:ch=U_;11:ch=S_;default:ch=SP;endcase
        3'd3:case(tc)
            0:ch=L_;1:ch=I_;2:ch=N_;3:ch=E_;5:ch=d_ht[3];6:ch=d_ht[2];7:ch=d_ht[1];8:ch=d_ht[0];
            10:ch=V_;12:ch=nz(d_vt[3],d_vt[3]==0);13:ch=d_vt[2];14:ch=d_vt[1];15:ch=d_vt[0];default:ch=SP;endcase
        default:case(tc)
            0:ch=M_;1:ch=I_;2:ch=S_;3:ch=S_;
            5:ch=nz(d_miss[4],d_miss[4]==0);6:ch=nz(d_miss[3],d_miss[4]==0&&d_miss[3]==0);7:ch=nz(d_miss[2],d_miss[4]==0&&d_miss[3]==0&&d_miss[2]==0);
            8:ch=nz(d_miss[1],d_miss[4]==0&&d_miss[3]==0&&d_miss[2]==0&&d_miss[1]==0);9:ch=d_miss[0];default:ch=SP;endcase
        endcase
    end
    // 5x7 font, row r (0 top), bit 4 = leftmost column
    function [4:0] font;input[5:0]c;input[2:0]r;
        reg[34:0]g;begin
        case(c)
            0:g=35'b01110_10001_10011_10101_11001_10001_01110;
            1:g=35'b00100_01100_00100_00100_00100_00100_01110;
            2:g=35'b01110_10001_00001_00010_00100_01000_11111;
            3:g=35'b11111_00010_00100_00010_00001_10001_01110;
            4:g=35'b00010_00110_01010_10010_11111_00010_00010;
            5:g=35'b11111_10000_11110_00001_00001_10001_01110;
            6:g=35'b00110_01000_10000_11110_10001_10001_01110;
            7:g=35'b11111_00001_00010_00100_01000_01000_01000;
            8:g=35'b01110_10001_10001_01110_10001_10001_01110;
            9:g=35'b01110_10001_10001_01111_00001_00010_01100;
            DOT:g=35'b00000_00000_00000_00000_00000_01100_01100;
            A_:g=35'b01110_10001_10001_11111_10001_10001_10001;
            C_:g=35'b01110_10001_10000_10000_10000_10001_01110;
            E_:g=35'b11111_10000_10000_11110_10000_10000_11111;
            F_:g=35'b11111_10000_10000_11110_10000_10000_10000;
            I_:g=35'b01110_00100_00100_00100_00100_00100_01110;
            L_:g=35'b10000_10000_10000_10000_10000_10000_11111;
            M_:g=35'b10001_11011_10101_10101_10001_10001_10001;
            N_:g=35'b10001_10001_11001_10101_10011_10001_10001;
            P_:g=35'b11110_10001_10001_11110_10000_10000_10000;
            S_:g=35'b01111_10000_10000_01110_00001_00001_11110;
            T_:g=35'b11111_00100_00100_00100_00100_00100_00100;
            U_:g=35'b10001_10001_10001_10001_10001_10001_01110;
            V_:g=35'b10001_10001_10001_10001_10001_01010_00100;
            W_:g=35'b10001_10001_10001_10101_10101_10101_01010;
            R_:g=35'b11110_10001_10001_11110_10100_10010_10001;
            K_:g=35'b10001_10010_10100_11000_10100_10010_10001;
            O_:g=35'b01110_10001_10001_10001_10001_10001_01110;
            default:g=35'd0;
        endcase
        font=g[34-5*r -: 5];
    end endfunction

    // ---- raster position ----
    // x/y count the active area; the character cell position is kept with
    // running counters (no dividers in the pixel path).
    // CW=12: 5 font columns x2 + 2 gap; CH=18: 7 font rows x2 + 4 gap.
    reg[11:0]x,y;reg de_d,vs_d;
    reg[4:0]col;reg[3:0]sub_x;reg[3:0]row;reg[4:0]sub_y;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)begin x<=0;y<=0;de_d<=0;vs_d<=0;col<=0;sub_x<=0;row<=0;sub_y<=0;end
        else begin
            de_d<=de_in;vs_d<=vs_in;
            if(vs_in && !vs_d)y<=0;
            else if(de_d && !de_in)begin
                y<=y+12'd1;
                if(y+12'd1==Y0+3)begin row<=0;sub_y<=0;end
                else if(sub_y==CH-1)begin sub_y<=0;row<=row+4'd1;end
                else sub_y<=sub_y+5'd1;
            end
            if(de_in)x<=x+12'd1;else x<=0;
            if(de_in)begin
                if(x==X0+3)begin col<=0;sub_x<=0;end
                else if(sub_x==CW-1)begin sub_x<=0;col<=col+5'd1;end
                else sub_x<=sub_x+4'd1;
            end
        end
    wire in_box=enable && x>=X0 && x<X0+BOX_W && y>=Y0 && y<Y0+BOX_H;
    always @*begin tc=col;tr=row[2:0];end
    wire glyph_area=x>=X0+4 && y>=Y0+3 && col<COLS && row<ROWS && sub_x<10 && sub_y<14;
    wire[4:0]bits=font(ch,sub_y[4:1]);
    wire on=glyph_area && bits[3'd4-sub_x[3:1]];

    reg hs1,vs1,de1,box1,on1;reg[15:0]p1;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)begin hs1<=0;vs1<=0;de1<=0;box1<=0;on1<=0;p1<=0;hs_out<=0;vs_out<=0;de_out<=0;pixel_out<=0;end
        else begin
            hs1<=hs_in;vs1<=vs_in;de1<=de_in;p1<=pixel_in;box1<=in_box && de_in;on1<=on;
            hs_out<=hs1;vs_out<=vs1;de_out<=de1;
            // box: darken to 1/4, text white
            pixel_out<=!box1?p1:on1?16'hffff:{2'b0,p1[15:13],3'b0,p1[10:8],3'b0,p1[4:2]};
        end
endmodule
