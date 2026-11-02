module testing60_doodle_canvas_lite(
    input wire sys_clk,input wire mem_clk,input wire rst_n,
    input wire touch_event,input wire point_valid,input wire [15:0] point_x,input wire [15:0] point_y,
    input wire enable_draw,input wire enable_erase,input wire side_right,input wire [2:0] color,
    input wire clear_event,
    input wire [16:0] view_step_q16,input wire signed [27:0] view_origin_x,
    input wire signed [27:0] view_origin_y,
    input wire [16:0] read_address,output wire [3:0] read_data
);
    // {valid,palette[2:0]} only (was 12 bits with an 8-bit clear generation).
    // CLEAR now sweeps the 115200 cells once (~115200 mem_clk cycles); reads
    // return 0 during the sweep, so the visible clear is still instantaneous.
    (* ram_style="block" *) reg [3:0] canvas [0:115199];
    wire [31:0] mx_product={16'd0,point_x}*32'd1279;
    wire [31:0] my_product={16'd0,point_y}*32'd719;
    wire [10:0] screen_x=mx_product>>14;
    wire [9:0] screen_y=my_product>>14;
    wire touch_side=screen_x>=11'd640;
    wire [9:0] local_x=touch_side ? screen_x-11'd640 : screen_x[9:0];
    wire signed [31:0] source_x_q=$signed(view_origin_x)+$signed({1'b0,local_x})*$signed({1'b0,view_step_q16});
    wire signed [31:0] source_y_q=$signed(view_origin_y)+$signed({1'b0,screen_y})*$signed({1'b0,view_step_q16});
    wire source_valid=source_x_q>=0 && source_x_q<32'sd41943040 &&
        source_y_q>=0 && source_y_q<32'sd47185920;
    wire [8:0] center_x=source_x_q[25:17];
    wire [8:0] center_y=source_y_q[25:17];
    wire draw_command=touch_event && point_valid && touch_side==side_right &&
        screen_y>=10'd100 && screen_y<10'd596 && source_valid &&
        (enable_draw||enable_erase);
    wire release_command=touch_event && !point_valid;
    wire command_event=clear_event||release_command||draw_command;
    wire [23:0] command_data={clear_event,release_command,enable_erase,color,center_x,center_y};
    wire command_ready;
    wire [23:0] command_core;
    touch_event_cdc #(.WIDTH(24)) u_doodle_command_cdc(
        .src_clk(sys_clk),.dst_clk(mem_clk),.rst_n(rst_n),
        .src_event(command_event),.src_data(command_data),
        .dst_event(command_ready),.dst_data(command_core)
    );
    reg busy;
    reg signed [4:0] brush_x,brush_y;
    reg [3:0] brush_limit;
    reg clearing;
    reg [16:0] clear_address;
    reg [3:0] read_word;
    assign read_data=(!clearing && read_word[3])?read_word:4'd0;
    reg erase_latch;
    reg [2:0] color_latch;
    reg [8:0] line_x,line_y,target_x,target_y,last_x,last_y;
    reg last_valid;
    reg signed [11:0] line_dx,line_dy,line_err;
    reg signed [1:0] line_sx,line_sy;
    reg signed [10:0] write_x,write_y;
    reg [7:0] radius_squared;
    reg signed [12:0] step_err,step_e2;
    reg [8:0] step_x,step_y;
    wire [8:0] command_dx=command_core[17:9]>=last_x ? command_core[17:9]-last_x : last_x-command_core[17:9];
    wire [8:0] command_dy=command_core[8:0]>=last_y ? command_core[8:0]-last_y : last_y-command_core[8:0];
    // Keep one write per cycle, but register the RAM address/data/enable.
    // The synchronous read forwards the pending write to preserve the old
    // read-before-write behavior exactly. A reset drains a previously accepted
    // write before clearing the pipeline, just as the original memory retained it.
    wire signed [10:0] issue_x=$signed({1'b0,line_x})+brush_x;
    wire signed [10:0] issue_y=$signed({1'b0,line_y})+brush_y;
    wire [7:0] issue_radius=brush_x*brush_x+brush_y*brush_y;
    wire issue_valid=busy && !clearing && !command_ready && issue_x>=0 && issue_x<320 &&
        issue_y>=0 && issue_y<360 && issue_radius<=brush_limit*brush_limit;
    reg pending_write;
    reg [16:0] pending_address;
    reg [3:0] pending_data;
    always @(posedge mem_clk) begin
        if(!rst_n) pending_write<=1'b0;
        else begin
            pending_write<=issue_valid || clearing;
            pending_address<=clearing ? clear_address : issue_y*320+issue_x;
            pending_data<=(clearing || erase_latch) ? 4'd0 : {1'b1,color_latch};
        end
        if(pending_write) canvas[pending_address]<=pending_data;
    end

    // Both ports use mem_clk.  Keeping all accesses in one clock domain lets
    // PDS infer one true dual-port block RAM instead of expanding the canvas
    // into flip-flops; touch commands cross through the mailbox above.
    always @(posedge mem_clk or negedge rst_n) begin
        if(!rst_n) begin
            busy<=0;clearing<=1;clear_address<=0;brush_x<=-5'sd3;brush_y<=-5'sd3;brush_limit<=4'd3;
            erase_latch<=0;color_latch<=0;line_x<=0;line_y<=0;target_x<=0;target_y<=0;
            last_x<=0;last_y<=0;last_valid<=0;line_dx<=0;line_dy<=0;line_err<=0;
            line_sx<=1;line_sy<=1;
            read_word<=0;
        end else begin
            read_word<=pending_write && pending_address==read_address ? pending_data : canvas[read_address];
        if(clearing) begin
            // Sweep; also clears whatever an FPGA power-up left in the RAM.
            // Touch commands arriving during the ~1 ms sweep are dropped.
            if(clear_address==17'd115199) clearing<=0;
            else clear_address<=clear_address+1'b1;
            busy<=0;last_valid<=0;
        end else if(command_ready && command_core[23]) begin
            clearing<=1;clear_address<=0;busy<=0;last_valid<=0;
        end else if(command_ready && command_core[22]) begin
            busy<=0;last_valid<=0;
        end else if(command_ready) begin
            target_x<=command_core[17:9];target_y<=command_core[8:0];
            erase_latch<=command_core[21];color_latch<=command_core[20:18];
            brush_limit<=command_core[21]?4'd7:4'd3;
            brush_x<=command_core[21]?-5'sd7:-5'sd3;
            brush_y<=command_core[21]?-5'sd7:-5'sd3;
            if(last_valid) begin
                line_x<=last_x;line_y<=last_y;
                line_dx<=$signed({1'b0,command_dx});
                line_dy<=-$signed({1'b0,command_dy});
                line_err<=$signed({1'b0,command_dx})-$signed({1'b0,command_dy});
                line_sx<=command_core[17:9]>=last_x ? 2'sd1:-2'sd1;
                line_sy<=command_core[8:0]>=last_y ? 2'sd1:-2'sd1;
            end else begin
                line_x<=command_core[17:9];line_y<=command_core[8:0];
                line_dx<=0;line_dy<=0;line_err<=0;line_sx<=1;line_sy<=1;
            end
            last_x<=command_core[17:9];last_y<=command_core[8:0];last_valid<=1;busy<=1;
        end else if(busy) begin
            if(brush_x==$signed({1'b0,brush_limit})) begin
                brush_x<=-$signed({1'b0,brush_limit});
                if(brush_y==$signed({1'b0,brush_limit})) begin
                    brush_y<=-$signed({1'b0,brush_limit});
                    if(line_x==target_x && line_y==target_y) busy<=0;
                    else begin
                        // Bresenham step between reports: every missing touch
                        // sample is filled with another circular brush stamp.
                        step_err=line_err;step_e2=line_err<<<1;
                        step_x=line_x;step_y=line_y;
                        if(step_e2>=line_dy) begin step_err=step_err+line_dy;step_x=$signed({1'b0,line_x})+line_sx;end
                        if(step_e2<=line_dx) begin step_err=step_err+line_dx;step_y=$signed({1'b0,line_y})+line_sy;end
                        line_err<=step_err;line_x<=step_x;line_y<=step_y;
                    end
                end
                else brush_y<=brush_y+1'b1;
            end else brush_x<=brush_x+1'b1;
        end
        end
    end
endmodule