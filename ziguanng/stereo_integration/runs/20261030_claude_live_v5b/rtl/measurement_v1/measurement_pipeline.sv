// Immutable STATIC UNDISTORTED 640x720 frame buffers, in core_clk domain.
// claude live_v5b: 21x11 template, 255x31 search region, matcher limited to the
// predicted segment box; MANUAL triangulates the tapped right point directly.
// External ddr_patch_reader must use FRAME_WIDTH=640, HEIGHT=720, STRIDE_WORDS=40.
module measurement_pipeline #(
 parameter [31:0] MAX_STAGE_CYCLES=32'd8000000
)(input wire clk,rst_n,clear,frame_locked,input wire [31:0] frame_tag,
 input wire [24:0] left_base,right_base,
 input wire selection_event,selection_index,input wire [9:0] left_x,left_y,
 input wire manual_event,manual_index,input wire [9:0] right_x,right_y,
 output wire busy,output reg match_event,match_index,match_valid,
 output reg [9:0] match_x,match_y,output reg [7:0] match_reason,
 output reg result_valid,output reg [31:0] length_mm_q16,output reg [7:0] result_reason,
 output wire patch_req_valid,input wire patch_req_ready,
 output wire [24:0] patch_req_base_word,output wire [31:0] patch_req_frame_tag,
 output wire patch_frame_locked,output wire [31:0] patch_current_frame_tag,
 output wire [11:0] patch_req_x,patch_req_y,patch_req_width,patch_req_height,
 input wire patch_pix_valid,output wire patch_pix_ready,input wire [15:0] patch_pix_rgb565,
 input wire [11:0] patch_pix_x,patch_pix_y,input wire [31:0] patch_pix_frame_tag,
 input wire patch_pix_last,patch_done,patch_error,input wire [2:0] patch_error_code,
 input wire patch_busy);
 localparam integer TWH=10,THH=5,TWD=21,THD=11,SWD=255,SHD=31;
 localparam IDLE=0,DRAIN=1,ROI_START=2,ROI_WAIT=3,MATCH_CLEAR=4,PATCH_REQ=5,PATCH_LOAD=6,
 MATCH_START=7,MATCH_WAIT=8,RAY_L_START=9,RAY_L_WAIT=10,RAY_R_START=11,RAY_R_WAIT=12,
 TRI_START=13,TRI_WAIT=14,LEN_START=15,LEN_WAIT=16;
 reg [4:0] state,previous_state;reg [31:0] watchdog,epoch_tag,job_tag;
 reg [1:0] selected,xyz_valid;reg [9:0] selected_x[0:1],selected_y[0:1];
 reg signed [31:0] point_x[0:1],point_y[0:1],point_z[0:1];
 reg job_index,reverse_pass,load_search,manual_job;
 reg [7:0] cand_x0,cand_x1;reg [4:0] cand_y0,cand_y1;
 reg [9:0] lx,ly,rx,ry,roi_origin_x,roi_origin_y;
 reg [24:0] job_left_base,job_right_base;
 reg [13:0] received;reg [11:0] expected_x,expected_y;reg got_last;
 reg signed [31:0] lray_x,lray_y,rray_x,rray_y;
 wire epoch_cancel=clear||!frame_locked||(frame_tag!=epoch_tag);
 assign busy=(state!=IDLE)||epoch_cancel||patch_busy;
 assign patch_frame_locked=frame_locked&&!clear&&(frame_tag==epoch_tag)&&(state!=DRAIN);
 assign patch_current_frame_tag=frame_tag;
 assign patch_req_valid=(state==PATCH_REQ)&&patch_frame_locked;
 assign patch_req_frame_tag=job_tag;
 assign patch_req_base_word=load_search ? (reverse_pass ? job_left_base:job_right_base) : (reverse_pass ? job_right_base:job_left_base);
 assign patch_req_x=load_search ? {2'b0,roi_origin_x} : (reverse_pass ? {2'b0,rx}-TWH:{2'b0,lx}-TWH);
 assign patch_req_y=load_search ? {2'b0,roi_origin_y} : (reverse_pass ? {2'b0,ry}-THH:{2'b0,ly}-THH);
 assign patch_req_width=load_search ? SWD:TWD;
 assign patch_req_height=load_search ? SHD:THD;
 assign patch_pix_ready=(state==PATCH_LOAD)&&patch_frame_locked;
 wire [13:0] expected_count=load_search ? SWD*SHD:TWD*THD;
 wire pixel_accept=patch_pix_valid&&patch_pix_ready;
 wire pixel_ok=(patch_pix_frame_tag==job_tag)&&(patch_pix_x==expected_x)&&(patch_pix_y==expected_y)&&
                  (received<expected_count)&&(patch_pix_last==(received==expected_count-1));
 wire [7:0] red8={patch_pix_rgb565[15:11],patch_pix_rgb565[15:13]};
 wire [7:0] green8={patch_pix_rgb565[10:5],patch_pix_rgb565[10:9]};
 wire [7:0] blue8={patch_pix_rgb565[4:0],patch_pix_rgb565[4:2]};
 wire [9:0] gray_sum={2'b0,red8}+{1'b0,green8,1'b0}+{2'b0,blue8};
 wire matcher_load=pixel_accept&&pixel_ok&&!patch_error&&!epoch_cancel;
 wire mb,md,mv;wire [7:0] mr;wire [7:0] mx;wire [4:0] my;
 measurement_zmsad matcher(.clk(clk),.rst_n(rst_n),.clear(epoch_cancel||state==DRAIN||state==MATCH_CLEAR),
 .load_valid(matcher_load),.load_search(load_search),.load_address(received[12:0]),.load_pixel(gray_sum[9:2]),
 .cand_x0(cand_x0),.cand_x1(cand_x1),.cand_y0(cand_y0),.cand_y1(cand_y1),
 .start(state==MATCH_START&&!epoch_cancel),.busy(mb),.done(md),.valid(mv),.reason(mr),.match_x(mx),.match_y(my),.best_cost(),.second_cost());
 wire rob,rod,rov;wire [3:0] ror;wire [9:0] rox,roy;wire [7:0] rcx0,rcx1;wire [4:0] rcy0,rcy1;
 stereo_depth_roi predictor(.clk(clk),.rst_n(rst_n),.start(state==ROI_START&&!epoch_cancel),.reverse_direction(reverse_pass),
 .pixel_u(reverse_pass ? rx:lx),.pixel_v(reverse_pass ? ry:ly),.busy(rob),.done(rod),.valid(rov),.reject(ror),.roi_x(rox),.roi_y(roy),
 .cand_x0(rcx0),.cand_x1(rcx1),.cand_y0(rcy0),.cand_y1(rcy1),
 .endpoint0_x_q16(),.endpoint0_y_q16(),.endpoint1_x_q16(),.endpoint1_y_q16());
 wire prb,prd,prv;wire [3:0] prr;wire signed [31:0] prx,pry;
 pixel_to_ray_q24 rays(.clk(clk),.rst_n(rst_n),.start((state==RAY_L_START||state==RAY_R_START)&&!epoch_cancel),.right_eye(state==RAY_R_START),
 .pixel_u(state==RAY_R_START ? rx:lx),.pixel_v(state==RAY_R_START ? ry:ly),.busy(prb),.done(prd),.valid(prv),.reject(prr),.ray_x_q24(prx),.ray_y_q24(pry));
 wire tb,td,tv;wire [3:0] tr;wire signed [31:0] tx,ty,tz;
 stereo_triangulate_q24 triangulator(.clk(clk),.rst_n(rst_n),.start(state==TRI_START&&!epoch_cancel),
 .xl_q24(lray_x),.yl_q24(lray_y),.xr_q24(rray_x),.yr_q24(rray_y),.busy(tb),.done(td),.valid(tv),.reject(tr),.x_mm_q16(tx),.y_mm_q16(ty),.z_mm_q16(tz));
 wire lb,ld,lv;wire [3:0] lr;wire [31:0] length_value;
 point_distance_q16 distance(.clk(clk),.rst_n(rst_n),.start(state==LEN_START&&!epoch_cancel),
 .ax(point_x[0]),.ay(point_y[0]),.az(point_z[0]),.bx(point_x[1]),.by(point_y[1]),.bz(point_z[1]),
 .busy(lb),.done(ld),.valid(lv),.reject(lr),.length_mm_q16(length_value));
 wire [9:0] back_x=roi_origin_x+mx,back_y=roi_origin_y+my;
 wire [9:0] back_dx=back_x>lx ? back_x-lx:lx-back_x,back_dy=back_y>ly ? back_y-ly:ly-back_y;
 task fail(input [7:0] code);begin
 match_event<=1;match_index<=job_index;match_valid<=0;match_x<=0;match_y<=0;match_reason<=code;
 xyz_valid[job_index]<=0;result_valid<=0;length_mm_q16<=0;result_reason<=code;state<=DRAIN;
 end endtask
 integer i;
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin state<=DRAIN;previous_state<=DRAIN;watchdog<=0;epoch_tag<=0;job_tag<=0;selected<=0;xyz_valid<=0;
 match_event<=0;match_index<=0;match_valid<=0;match_x<=0;match_y<=0;match_reason<=0;result_valid<=0;length_mm_q16<=0;result_reason<=0;
 job_index<=0;reverse_pass<=0;load_search<=0;manual_job<=0;cand_x0<=0;cand_x1<=0;cand_y0<=0;cand_y1<=0;lx<=0;ly<=0;rx<=0;ry<=0;roi_origin_x<=0;roi_origin_y<=0;job_left_base<=0;job_right_base<=0;
 received<=0;expected_x<=0;expected_y<=0;got_last<=0;lray_x<=0;lray_y<=0;rray_x<=0;rray_y<=0;
 for(i=0;i<2;i=i+1)begin selected_x[i]<=0;selected_y[i]<=0;point_x[i]<=0;point_y[i]<=0;point_z[i]<=0;end end
 else begin
 match_event<=0;previous_state<=state;
 if(state!=previous_state||state==IDLE||state==DRAIN)watchdog<=0;else watchdog<=watchdog+1'b1;
 if(selection_event||manual_event)begin result_valid<=0;length_mm_q16<=0;result_reason<=0;end
 if(epoch_cancel)begin
 epoch_tag<=frame_tag;selected<=0;xyz_valid<=0;result_valid<=0;length_mm_q16<=0;result_reason<=clear ? 8'd0:8'h71;
 match_event<=0;match_valid<=0;match_x<=0;match_y<=0;match_reason<=0;state<=DRAIN;watchdog<=0;
 end else if(state!=IDLE&&state!=DRAIN&&watchdog>=MAX_STAGE_CYCLES)fail(8'h70);
 else case(state)
 DRAIN:if(!patch_busy&&!rob&&!prb&&!tb&&!lb&&!mb)state<=IDLE;
 IDLE:if(!patch_busy&&(selection_event||manual_event))begin
 job_index<=selection_event ? selection_index:manual_index;job_tag<=frame_tag;job_left_base<=left_base;job_right_base<=right_base;
 match_valid<=0;match_x<=0;match_y<=0;match_reason<=0;result_valid<=0;length_mm_q16<=0;result_reason<=0;
 if(selection_event)begin
 selected[selection_index]<=1;selected_x[selection_index]<=left_x;selected_y[selection_index]<=left_y;xyz_valid[selection_index]<=0;
 lx<=left_x;ly<=left_y;rx<=0;ry<=0;reverse_pass<=0;manual_job<=0;state<=ROI_START;
 end else begin
 lx<=selected_x[manual_index];ly<=selected_y[manual_index];rx<=right_x;ry<=right_y;reverse_pass<=1;manual_job<=1;xyz_valid[manual_index]<=0;
 state<=ROI_START;
 end end
 ROI_START:if(lx<TWH||lx>639-TWH||ly<THH||ly>719-THH||(reverse_pass&&(rx<TWH||rx>639-TWH||ry<THH||ry>719-THH))||!selected[job_index])fail(8'h01);
 else if(manual_job)state<=RAY_L_START; // the user placed the right point: no re-match
 else state<=ROI_WAIT;
 ROI_WAIT:if(rod)begin if(!rov)fail({4'h2,ror});else begin roi_origin_x<=rox;roi_origin_y<=roy;cand_x0<=rcx0;cand_x1<=rcx1;cand_y0<=rcy0;cand_y1<=rcy1;state<=MATCH_CLEAR;end end
 MATCH_CLEAR:begin load_search<=0;state<=PATCH_REQ;end
 PATCH_REQ:if(patch_req_valid&&patch_req_ready)begin received<=0;expected_x<=0;expected_y<=0;got_last<=0;state<=PATCH_LOAD;end
 PATCH_LOAD:begin
 if(patch_error)fail({5'b00010,patch_error_code});
 else if(pixel_accept&&!pixel_ok)fail(8'h18);
 else begin
 if(pixel_accept)begin received<=received+1'b1;if(patch_pix_last)got_last<=1;
 if(expected_x==patch_req_width-1)begin expected_x<=0;expected_y<=expected_y+1'b1;end else expected_x<=expected_x+1'b1;end
 if(patch_done)begin if(received!=expected_count||!got_last)fail(8'h19);
 else if(!load_search)begin load_search<=1;state<=PATCH_REQ;end else state<=MATCH_START;end
 end end
 MATCH_START:state<=MATCH_WAIT;
 MATCH_WAIT:if(md)begin if(!mv)fail({4'h3,mr[3:0]});
 else if(!reverse_pass)begin rx<=back_x;ry<=back_y;reverse_pass<=1;state<=ROI_START;end
 else if(back_dx>1||back_dy>1)fail(8'h40);else state<=RAY_L_START;end
 RAY_L_START:state<=RAY_L_WAIT;
 RAY_L_WAIT:if(prd)begin if(!prv)fail({4'h5,prr});else begin lray_x<=prx;lray_y<=pry;state<=RAY_R_START;end end
 RAY_R_START:state<=RAY_R_WAIT;
 RAY_R_WAIT:if(prd)begin if(!prv)fail({4'h5,prr});else begin rray_x<=prx;rray_y<=pry;state<=TRI_START;end end
 TRI_START:state<=TRI_WAIT;
 TRI_WAIT:if(td)begin if(!tv)fail({4'h6,tr});else begin
 point_x[job_index]<=tx;point_y[job_index]<=ty;point_z[job_index]<=tz;xyz_valid[job_index]<=1;
 match_event<=1;match_index<=job_index;match_valid<=1;match_x<=rx;match_y<=ry;match_reason<=0;
 if(xyz_valid[!job_index])state<=LEN_START;else state<=IDLE;
 end end
 LEN_START:state<=LEN_WAIT;
 LEN_WAIT:if(ld)begin if(lv)begin result_valid<=1;length_mm_q16<=length_value;result_reason<=0;state<=IDLE;end
 else begin result_valid<=0;length_mm_q16<=0;result_reason<={4'h7,lr};state<=IDLE;end end
 default:fail(8'h7f);
 endcase end end
endmodule
