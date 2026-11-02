// Complete autonomous v2 enhancement stream. Atomic frame config, a shared
// synchronous Gamma ROM, previous-complete-frame statistics, and bounded stalls.
// frame_done includes output drain AND computation of the next automatic Gamma.
module adaptive_luminance_stream_v2 #(parameter TAG_W=44,parameter ROM_FILE="gamma_q8_v2.hex",parameter EXTERNAL_ROM=0)(
    input wire clk,rst_n,input wire frame_start,output wire frame_ready,
    input wire[1:0] frame_mode,input wire[9:0] manual_gamma_q8,input wire[21:0] frame_pixels,
    output wire busy,output reg frame_done,output reg error,
    output reg[9:0] applied_gamma_q8,target_gamma_q8,next_auto_gamma_q8,
    output wire[21:0] statistics_pixels,
    input wire in_valid,output wire in_ready,input wire[15:0] in_rgb565,
    input wire in_inside,in_last,input wire[TAG_W-1:0] in_tag,
    output wire out_valid,input wire out_ready,output wire[15:0] out_rgb565,
    output wire[TAG_W-1:0] out_tag,output wire out_last,
    // EXTERNAL_ROM=1: one Gamma ROM shared by both eyes (same content, same clock).
    output wire[16:0] rom_addr,input wire[7:0] rom_ext_data
);
    localparam IDLE=0,LREAD=1,LWRITE=2,COMMIT=3,ARM=4,RUN=5,WAIT=6;
    reg[2:0] state;
    reg[7:0] load_bin;
    reg[1:0] mode;
    reg[21:0] expected_pixels,accepted_pixels;
    reg histogram_start,lut_start,lut_commit,drained,calculated;
    wire histogram_frame_ready,hist_in_ready,hist_valid,hist_release,hist_error;
    wire[7:0] hist_addr;
    wire[21:0] hist_data,valid_pixels,total_pixels;
    assign statistics_pixels=valid_pixels;
    wire lut_in_ready,lut_write_ready,lut_frame_active,lut_pending,lut_valid,lut_error;
    wire[31:0] lut_version;
    wire controller_ready,result_valid,controller_error;
    wire[9:0] controller_target,controller_next;
    wire[16:0] controller_addr;
    wire[7:0] rom_data;
    wire[16:0] loader_addr=((applied_gamma_q8-10'd128)<<8)+load_bin;
    assign rom_addr=controller_ready ? loader_addr : controller_addr;
    generate if(EXTERNAL_ROM!=0)begin:shared_rom
        assign rom_data=rom_ext_data;
    end else begin:own_rom
        gamma_rom_v2 #(.FILE(ROM_FILE)) u_rom(.clk(clk),.addr(rom_addr),.data(rom_data));
    end endgenerate
    gamma_controller_search_v2 u_controller(.clk(clk),.rst_n(rst_n),.job_valid(hist_valid),.job_ready(controller_ready),
        .valid_pixels(valid_pixels),.applied_gamma_q8(applied_gamma_q8),
        .hist_addr(hist_addr),.hist_data(hist_data),.hist_release(hist_release),
        .table_addr(controller_addr),.table_data(rom_data),.result_valid(result_valid),.result_ready(1'b1),
        .target_gamma_q8(controller_target),.next_gamma_q8(controller_next),.error(controller_error));
    wire final_input=accepted_pixels==expected_pixels-1;
    assign frame_ready=state==IDLE && histogram_frame_ready && controller_ready;
    assign busy=state!=IDLE;
    assign in_ready=state==RUN && hist_in_ready && lut_in_ready;
    luminance_histogram_pipelined_v2 u_hist(.clk(clk),.rst_n(rst_n),.frame_start(histogram_start),.frame_ready(histogram_frame_ready),
        .in_valid(in_valid && state==RUN && lut_in_ready),.in_ready(hist_in_ready),
        .in_rgb565(in_rgb565),.in_inside(in_inside),.in_last(final_input),
        .hist_valid(hist_valid),.hist_ready(hist_release),.hist_addr(hist_addr),.hist_data(hist_data),
        .valid_pixels(valid_pixels),.total_pixels(total_pixels),.error(hist_error));
    luminance_lut_pipelined_v2 #(.TAG_W(TAG_W+1)) u_lut(.clk(clk),.rst_n(rst_n),
        .lut_wr_valid(state==LWRITE),.lut_wr_ready(lut_write_ready),.lut_wr_addr(load_bin),.lut_wr_data(rom_data),
        .lut_commit(lut_commit),.frame_start(lut_start),.frame_done(out_valid && out_ready && out_last),
        .frame_enable(mode!=0),.frame_active(lut_frame_active),.pending(lut_pending),
        .active_lut_valid(lut_valid),.error(lut_error),.active_lut_version(lut_version),
        .in_valid(in_valid && state==RUN && hist_in_ready),.in_ready(lut_in_ready),
        .in_rgb565(in_rgb565),.in_inside(in_inside),.in_tag({final_input,in_tag}),
        .out_valid(out_valid),.out_ready(out_ready),.out_rgb565(out_rgb565),.out_tag({out_last,out_tag}));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;load_bin<=0;mode<=0;expected_pixels<=0;accepted_pixels<=0;
            histogram_start<=0;lut_start<=0;lut_commit<=0;drained<=0;calculated<=0;
            applied_gamma_q8<=256;target_gamma_q8<=256;next_auto_gamma_q8<=256;frame_done<=0;error<=0;
        end else begin
            histogram_start<=0;lut_start<=0;lut_commit<=0;frame_done<=0;
            if(hist_error || lut_error || controller_error) error<=1;
            if(result_valid) begin
                target_gamma_q8<=controller_target;next_auto_gamma_q8<=controller_next;calculated<=1;
            end
            if(out_valid && out_ready && out_last) drained<=1;
            case(state)
                IDLE:if(frame_start && frame_ready) begin
                    if(frame_mode>2 || manual_gamma_q8<128 || manual_gamma_q8>512 || frame_pixels==0 || frame_pixels>2073600) error<=1;
                    else begin
                        error<=0;mode<=frame_mode;expected_pixels<=frame_pixels;accepted_pixels<=0;
                        applied_gamma_q8<=frame_mode==0 ? 256 : frame_mode==2 ? manual_gamma_q8 : next_auto_gamma_q8;
                        drained<=0;calculated<=0;load_bin<=0;state<=LREAD;
                    end
                end
                LREAD:state<=LWRITE;
                LWRITE:if(lut_write_ready) begin
                    if(load_bin==255) begin lut_commit<=1;state<=COMMIT;end
                    else begin load_bin<=load_bin+1;state<=LREAD;end
                end
                COMMIT:if(lut_pending) state<=ARM;
                ARM:begin histogram_start<=1;lut_start<=1;state<=RUN;end
                RUN:if(in_valid && in_ready) begin
                    accepted_pixels<=accepted_pixels+1;
                    if(in_last!=final_input) error<=1;
                    if(final_input) state<=WAIT;
                end
                WAIT:if(drained && calculated) begin frame_done<=1;state<=IDLE;end
            endcase
        end
    end
endmodule
