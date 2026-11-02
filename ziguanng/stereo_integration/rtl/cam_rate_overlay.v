`timescale 1ns/1ps
// Camera rate readout, drawn over the video in the bottom-left corner:
//
//   C0 FPS  15.70 CLK  96000K ROW  720
//   C1 FPS  15.70 CLK  96000K ROW  720
//
// FPS: camera frames per second, from the VSYNC period measured on the pixel
//      clock.  CLK: camera clock in kHz, camera clocks counted during a 1 ms
//      gate made on the pixel clock.  ROW: HREF pulses per camera frame.
// Camera-side values are latched in the camera domain and held for at least
// a frame (ROW) or 1 ms (CLK) before the pixel clock samples them; see
// cam_rate_overlay.fdc.  5x7 font drawn 2x2 as in live_stats_overlay.  The
// output is the input stream delayed by two clocks.
module cam_rate_meter #(
    parameter integer CLK_HZ=74250000
)(
    input wire clk,rst_n,                 // pixel clock
    input wire gate,gate_sample,          // pixel clock: 1 ms gate, sample strobe while gate is low
    input wire cam_clk,cam_vsync,cam_href,
    output reg[31:0]period,               // pixel clocks per camera frame, 0 when stopped
    output reg[19:0]khz,
    output reg[15:0]rows
);
    // ---- camera clock domain ----
    (* ASYNC_REG="TRUE" *) reg gate_meta,gate_sync;
    reg gate_d,vs_d,hs_d,tog;
    reg[19:0]cyc,cyc_hold;
    reg[15:0]row_cnt,row_hold;
    always @(posedge cam_clk or negedge rst_n)
        if(!rst_n)begin gate_meta<=0;gate_sync<=0;gate_d<=0;vs_d<=0;hs_d<=0;tog<=0;cyc<=0;cyc_hold<=0;row_cnt<=0;row_hold<=0;end
        else begin
            gate_meta<=gate;gate_sync<=gate_meta;gate_d<=gate_sync;
            if(gate_sync && !gate_d)cyc<=20'd1;
            else if(gate_sync && cyc!=20'hfffff)cyc<=cyc+20'd1;
            if(!gate_sync && gate_d)cyc_hold<=cyc;
            vs_d<=cam_vsync;hs_d<=cam_href;
            if(cam_vsync && !vs_d)begin row_hold<=row_cnt;row_cnt<=0;tog<=~tog;end
            else if(cam_href && !hs_d && row_cnt!=16'hffff)row_cnt<=row_cnt+16'd1;
        end
    // ---- pixel clock domain ----
    (* ASYNC_REG="TRUE" *) reg tog_meta,tog_sync;
    reg tog_d,tog_e1,tog_e2;
    reg[31:0]cnt;
    localparam [31:0] STOP=CLK_HZ;        // no frame for 1 s: stopped
    always @(posedge clk or negedge rst_n)
        if(!rst_n)begin tog_meta<=0;tog_sync<=0;tog_d<=0;tog_e1<=0;tog_e2<=0;cnt<=0;period<=0;khz<=0;rows<=0;end
        else begin
            tog_meta<=tog;tog_sync<=tog_meta;tog_d<=tog_sync;
            tog_e1<=(tog_sync!=tog_d);tog_e2<=tog_e1;
            if(tog_sync!=tog_d)begin period<=cnt+32'd1;cnt<=0;end
            else begin
                if(cnt!=32'hffffffff)cnt<=cnt+32'd1;
                if(cnt>=STOP)period<=0;
            end
            if(tog_e2)rows<=row_hold;    // held for the whole camera frame
            if(gate_sample)khz<=cyc_hold;// held while the gate is low
        end
endmodule

module cam_rate_overlay #(
    parameter integer CLK_HZ=74250000,
    parameter integer X0=16,Y0=664
)(
    input wire clk,rst_n,
    input wire cam0_clk,cam0_vsync,cam0_href,
    input wire cam1_clk,cam1_vsync,cam1_href,
    input wire hs_in,vs_in,de_in,input wire[23:0]rgb_in,
    output reg hs_out,vs_out,de_out,output reg[23:0]rgb_out
);
    localparam integer COLS=34,ROWS=2,CW=12,CH=18;
    localparam integer BOX_W=COLS*CW+8,BOX_H=ROWS*CH+6;

    // ---- 1 ms gate: high 1 ms, low 1 ms; sample in the middle of the low half ----
    localparam integer GATE=CLK_HZ/1000;
    reg[16:0]gcnt;reg gate,gate_sample;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)begin gcnt<=0;gate<=0;gate_sample<=0;end
        else begin
            gate_sample<=(!gate && gcnt==GATE/2);
            if(gcnt==GATE-1)begin gcnt<=0;gate<=~gate;end else gcnt<=gcnt+17'd1;
        end
    wire[31:0]per0,per1;wire[19:0]khz0,khz1;wire[15:0]rows0,rows1;
    cam_rate_meter #(.CLK_HZ(CLK_HZ)) m0(.clk(clk),.rst_n(rst_n),.gate(gate),.gate_sample(gate_sample),
        .cam_clk(cam0_clk),.cam_vsync(cam0_vsync),.cam_href(cam0_href),.period(per0),.khz(khz0),.rows(rows0));
    cam_rate_meter #(.CLK_HZ(CLK_HZ)) m1(.clk(clk),.rst_n(rst_n),.gate(gate),.gate_sample(gate_sample),
        .cam_clk(cam1_clk),.cam_vsync(cam1_vsync),.cam_href(cam1_href),.period(per1),.khz(khz1),.rows(rows1));

    // ---- numbers -> six decimal digits, one value at a time, once per display frame ----
    localparam [39:0] FPS_NUM=CLK_HZ*64'd100;
    localparam [2:0] S_IDLE=0,S_DIV=1,S_PREP=2,S_DIG=3,S_STORE=4;
    reg[2:0]st,job;
    reg[39:0]num,quo,rem;reg[31:0]den;reg[5:0]bit_i;
    reg[19:0]val;reg[3:0]dg[0:5];reg[2:0]dpos;
    reg[23:0]dv[0:5];                      // per job, 4 bits per digit, digit 0 least significant
    reg[39:0]n_next;reg[31:0]d_next;
    reg[31:0]per_l[0:1];reg[19:0]khz_l[0:1];reg[15:0]rows_l[0:1];
    reg vs_q;
    always @*begin
        n_next=40'd0;d_next=32'd1;
        case(job)
            3'd0:n_next={20'd0,khz_l[0]};
            3'd1:n_next={24'd0,rows_l[0]};
            3'd2:begin n_next=(per_l[1]==0)?40'd0:FPS_NUM;d_next=(per_l[1]==0)?32'd1:per_l[1];end
            3'd3:n_next={20'd0,khz_l[1]};
            3'd4:n_next={24'd0,rows_l[1]};
            default:;
        endcase
    end
    wire[40:0]rem_sh={rem[39:0],num[bit_i]};
    integer k;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            st<=S_IDLE;job<=0;num<=0;den<=1;quo<=0;rem<=0;bit_i<=0;val<=0;dpos<=0;vs_q<=0;
            for(k=0;k<2;k=k+1)begin per_l[k]<=0;khz_l[k]<=0;rows_l[k]<=0;end
            for(k=0;k<6;k=k+1)begin dg[k]<=0;dv[k]<=0;end
        end else begin
            vs_q<=vs_in;
            case(st)
            S_IDLE:if(vs_in && !vs_q)begin
                per_l[0]<=per0;per_l[1]<=per1;khz_l[0]<=khz0;khz_l[1]<=khz1;rows_l[0]<=rows0;rows_l[1]<=rows1;
                job<=0;num<=(per0==0)?40'd0:FPS_NUM;den<=(per0==0)?32'd1:per0;quo<=0;rem<=0;bit_i<=6'd39;st<=S_DIV;
            end
            S_DIV:begin   // restoring division, one quotient bit per clock
                if(rem_sh>={9'd0,den})begin rem<=rem_sh-{9'd0,den};quo[bit_i]<=1'b1;end
                else rem<=rem_sh[39:0];
                if(bit_i==0)st<=S_PREP;else bit_i<=bit_i-6'd1;
            end
            // fps x100 shows 3+2 digits, the others 6
            S_PREP:begin val<=(job==3'd0||job==3'd3)?((quo>40'd99999)?20'd99999:quo[19:0]):((quo>40'd999999)?20'd999999:quo[19:0]);dpos<=0;st<=S_DIG;end
            S_DIG:begin
                dg[dpos]<=val%10;val<=val/10;
                if(dpos==3'd5)st<=S_STORE;else dpos<=dpos+3'd1;
            end
            default:begin
                dv[job]<={dg[5],dg[4],dg[3],dg[2],dg[1],dg[0]};
                job<=job+3'd1;
                // jobs: 0 fps0, 1 khz0, 2 rows0, 3 fps1, 4 khz1, 5 rows1
                num<=n_next;den<=d_next;quo<=0;rem<=0;bit_i<=6'd39;
                st<=(job==3'd5)?S_IDLE:S_DIV;
            end
            endcase
        end
    end

    // ---- text ----
    localparam [5:0] SP=10,DOT=11,C_=13,F_=15,K_=27,L_=17,O_=28,P_=20,R_=26,S_=21,W_=25;
    reg[5:0]tc;reg[2:0]tr;reg[5:0]ch;
    // digit i (0 = least significant) of a field n digits wide, leading zeros blank above `keep`
    function [5:0] dig;input[23:0]d;input integer i;input integer keep;
        reg lead;integer m;begin
        lead=1;for(m=5;m>i;m=m-1)if(d[4*m+:4]!=0)lead=0;
        dig=(lead && i>keep && d[4*i+:4]==0)?SP:{2'd0,d[4*i+:4]};
    end endfunction
    wire[23:0]f_fps=tr[0]?dv[3]:dv[0];
    wire[23:0]f_khz=tr[0]?dv[4]:dv[1];
    wire[23:0]f_row=tr[0]?dv[5]:dv[2];
    always @*begin
        case(tc)
            0:ch=C_;1:ch=tr[0]?6'd1:6'd0;
            3:ch=F_;4:ch=P_;5:ch=S_;
            7:ch=dig(f_fps,4,2);8:ch=dig(f_fps,3,2);9:ch=dig(f_fps,2,2);10:ch=DOT;11:ch=dig(f_fps,1,0);12:ch=dig(f_fps,0,0);
            14:ch=C_;15:ch=L_;16:ch=K_;
            18:ch=dig(f_khz,5,0);19:ch=dig(f_khz,4,0);20:ch=dig(f_khz,3,0);21:ch=dig(f_khz,2,0);22:ch=dig(f_khz,1,0);23:ch=dig(f_khz,0,0);24:ch=K_;
            26:ch=R_;27:ch=O_;28:ch=W_;
            30:ch=dig(f_row,3,0);31:ch=dig(f_row,2,0);32:ch=dig(f_row,1,0);33:ch=dig(f_row,0,0);
            default:ch=SP;
        endcase
    end
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
            C_:g=35'b01110_10001_10000_10000_10000_10001_01110;
            F_:g=35'b11111_10000_10000_11110_10000_10000_10000;
            K_:g=35'b10001_10010_10100_11000_10100_10010_10001;
            L_:g=35'b10000_10000_10000_10000_10000_10000_11111;
            O_:g=35'b01110_10001_10001_10001_10001_10001_01110;
            P_:g=35'b11110_10001_10001_11110_10000_10000_10000;
            R_:g=35'b11110_10001_10001_11110_10100_10010_10001;
            S_:g=35'b01111_10000_10000_01110_00001_00001_11110;
            W_:g=35'b10001_10001_10001_10101_10101_10101_01010;
            default:g=35'd0;
        endcase
        font=g[34-5*r -: 5];
    end endfunction

    // ---- raster position (same scheme as live_stats_overlay) ----
    reg[11:0]x,y;reg de_d,vs_d;
    reg[5:0]col;reg[3:0]sub_x;reg[3:0]row;reg[4:0]sub_y;
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
                else if(sub_x==CW-1)begin sub_x<=0;col<=col+6'd1;end
                else sub_x<=sub_x+4'd1;
            end
        end
    wire in_box=x>=X0 && x<X0+BOX_W && y>=Y0 && y<Y0+BOX_H;
    always @*begin tc=col;tr=row[2:0];end
    wire glyph_area=x>=X0+4 && y>=Y0+3 && col<COLS && row<ROWS && sub_x<10 && sub_y<14;
    wire[4:0]bits=font(ch,sub_y[4:1]);
    wire on=glyph_area && bits[3'd4-sub_x[3:1]];

    reg hs1,vs1,de1,box1,on1;reg[23:0]p1;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)begin hs1<=0;vs1<=0;de1<=0;box1<=0;on1<=0;p1<=0;hs_out<=0;vs_out<=0;de_out<=0;rgb_out<=0;end
        else begin
            hs1<=hs_in;vs1<=vs_in;de1<=de_in;p1<=rgb_in;box1<=in_box && de_in;on1<=on;
            hs_out<=hs1;vs_out<=vs1;de_out<=de1;
            // box: darken to 1/4, text white
            rgb_out<=!box1?p1:on1?24'hffffff:{2'b0,p1[23:18],2'b0,p1[15:10],2'b0,p1[7:2]};
        end
endmodule
