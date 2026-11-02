// SPDX-License-Identifier: MIT
// Standalone MAX3421E host + two-contact HID output, Verilog-2001.
// Target ONLY: directly attached FULL-SPEED VID:PID 222A:0001,
// captured HID descriptor CRC32 68073076. No USB hub support.
// The SPI pins below are named from the FPGA/master perspective.
// Standalone square-wave output test used only when Top enables meter mode.
module max3421e_mosi_meter #(parameter integer CLK_HZ=27000000, OUTPUT_HZ=1000)(
    input wire clk,rst_n,
    output reg mosi_level
);
    localparam integer HALF_PERIOD_RAW=CLK_HZ/(2*OUTPUT_HZ);
    localparam integer HALF_PERIOD=(HALF_PERIOD_RAW<1)?1:HALF_PERIOD_RAW;
    reg [31:0] meter_counter;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin meter_counter<=0;mosi_level<=0;end
        else if (meter_counter==HALF_PERIOD-1) begin
            meter_counter<=0;mosi_level<=~mosi_level;
        end else meter_counter<=meter_counter+1'b1;
    end
endmodule

module max3421e_touch #(
    parameter integer CLK_HZ = 27000000,
    parameter integer SPI_HZ = 3000000,
    parameter integer READ_VENDOR_FEATURE = 1,
    parameter integer SPI_PROBE_ONLY = 0,
    parameter integer RCVBC_SCOPE_TEST = 0,
    parameter integer CS_HIGH_CYCLES = (CLK_HZ + 199) / 200,
    parameter integer INIT_TIMEOUT_MS = 30000,
    parameter integer TRANSFER_TIMEOUT_MS = 2000
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        spi_miso,
    input  wire        max_irq_n,
    output wire        spi_sclk,
    output wire        spi_mosi,
    output wire        spi_cs_n,
    output wire        max_reset_n,
    output wire        vbus_en,
    output wire        touch_event,
    output wire        point0_valid,
    output wire [5:0]  point0_id,
    output wire [15:0] point0_x,
    output wire [15:0] point0_y,
    output wire        point1_valid,
    output wire [5:0]  point1_id,
    output wire [15:0] point1_x,
    output wire [15:0] point1_y,
    output wire [1:0]  touch_count,
    output wire        touch_overflow,
    output wire        usb_connected,
    output wire        usb_ready,
    output wire        chip_identified,
    output wire        spi_probe_done, spi_probe_pass, spi_probe_event,
    output wire [7:0]  revision_raw, probe_read_a, probe_read_b,
    output wire [7:0]  error_code,
    output wire [7:0]  debug_state,
    output wire [3:0]  last_hrsl,
    output wire        report_error,
    output reg         diagnostic_event,
    output reg [511:0] diagnostic_data
);
    wire report_valid;
    wire [6:0] report_len;
    wire [511:0] report_data;
    reg was_ready;
    reg [31:0] diagnostic_timer;
    reg [15:0] received_count,accepted_count,bad_count,other_count;
    reg [7:0] last_error,saved_len,saved_id,saved_count,saved_flags,saved_reason;
    reg [63:0] saved_raw;
    reg [15:0] saved_x,saved_y;
    wire [3:0] enumeration_stage;
    wire [63:0] initialization_diagnostic;
    wire [63:0] receive_diagnostic;
    wire [63:0] setup_readback_diagnostic;
    wire [63:0] first_in_diagnostic;
    wire [511:0] rcvbc_test_data;
    reg [3:0] last_error_stage;
    reg trace_sclk;
    reg [2:0] trace_bit;
    reg [7:0] trace_shift,trace_command;
    reg trace_has_command;
    reg [3:0] trace_byte;
    reg [63:0] setup_wire_bytes;
    reg [7:0] trace_miso_shift,trace_status,raw_rcvbc,raw_rcvbc_status;
    reg [15:0] raw_rcvbc_reads;
    wire [7:0] scope_count;
    wire scope_active;
    // Observe the serialized FPGA output, rather than just the request variable.
    // Sample one system clock after SCLK rises; MOSI stays stable through high.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            trace_sclk<=0;trace_bit<=0;trace_shift<=0;trace_command<=0;
            trace_has_command<=0;trace_byte<=0;setup_wire_bytes<=0;
            trace_miso_shift<=0;trace_status<=0;raw_rcvbc<=0;
            raw_rcvbc_status<=0;raw_rcvbc_reads<=0;
        end else begin
            trace_sclk<=spi_sclk;
            if(spi_cs_n)begin trace_bit<=0;trace_has_command<=0;trace_byte<=0;end
            else if(spi_sclk && !trace_sclk)begin
                trace_shift<={trace_shift[6:0],spi_mosi};
                trace_miso_shift<={trace_miso_shift[6:0],spi_miso};
                trace_bit<=trace_bit+1'b1;
                if(trace_bit==7)begin
                    if(!trace_has_command)begin
                        trace_command<={trace_shift[6:0],spi_mosi};trace_has_command<=1;
                        trace_status<={trace_miso_shift[6:0],spi_miso};
                    end else begin
                        if(trace_command==8'h30 && trace_byte==0)begin
                            raw_rcvbc<={trace_miso_shift[6:0],spi_miso};
                            raw_rcvbc_status<=trace_status;raw_rcvbc_reads<=raw_rcvbc_reads+1'b1;
                        end
                        if(trace_command==8'h22 && trace_byte<8)
                            setup_wire_bytes[(7-trace_byte)*8 +: 8]<={trace_shift[6:0],spi_mosi};
                        if(trace_byte<8)trace_byte<=trace_byte+1'b1;
                    end
                end
            end
        end
    end
    // Persistent counters/error history survive host retries; external reset clears them.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            diagnostic_event<=0;diagnostic_data<=0;diagnostic_timer<=0;
            received_count<=0;accepted_count<=0;bad_count<=0;other_count<=0;
            last_error<=0;saved_len<=0;saved_id<=0;saved_count<=0;saved_flags<=0;
            saved_raw<=0;saved_x<=0;saved_y<=0;saved_reason<=0;
            last_error_stage<=0;
        end else begin
            diagnostic_event<=0;
            if(error_code!=0)begin last_error<=error_code;last_error_stage<=enumeration_stage;end
            if(report_valid) begin
                received_count<=received_count+1'b1;
                if(report_data[7:0]!=8'h04)other_count<=other_count+1'b1;
                saved_len<={1'b0,report_len};saved_id<=report_data[7:0];
                saved_count<=report_data[447:440];saved_flags<=report_data[15:8];
                saved_raw<=report_data[63:0];saved_x<=report_data[31:16];saved_y<=report_data[47:32];
                saved_reason<={4'd0,(report_data[47:32]>16384 || report_data[31:16]>16384),
                    (report_data[447:440]>10),(report_len!=64),(report_data[7:0]!=8'h04)};
            end
            if(report_error)begin bad_count<=bad_count+1'b1;saved_reason[4]<=1;end
            if(touch_event && usb_ready)accepted_count<=accepted_count+1'b1;
            if(diagnostic_timer >= (CLK_HZ/10)-1) begin
                diagnostic_timer<=0;diagnostic_event<=1;
                // A3/A4 reuse the existing eight-row display while receive is held.
                if(RCVBC_SCOPE_TEST==4)begin
                    diagnostic_data<=rcvbc_test_data;
                    // Menu row 6: reports / accepted touch / decoder errors / other IDs.
                    diagnostic_data[447:384]<={received_count,accepted_count,bad_count,other_count};
                end else if((RCVBC_SCOPE_TEST==2 || RCVBC_SCOPE_TEST==3) && scope_active)
                    diagnostic_data<=rcvbc_test_data;
                else diagnostic_data<={ first_in_diagnostic,
                    setup_readback_diagnostic,receive_diagnostic,initialization_diagnostic,{7'd0,usb_connected},{7'd0,usb_ready},debug_state,error_code,
                    last_error,{4'd0,last_hrsl},saved_len,saved_id,
                    received_count,accepted_count,bad_count,other_count,
                    saved_raw[7:0],saved_raw[15:8],saved_raw[23:16],saved_raw[31:24],
                    saved_raw[39:32],saved_raw[47:40],saved_raw[55:48],saved_raw[63:56],
                    saved_count,saved_flags,saved_x,saved_y,saved_reason,{last_error_stage,enumeration_stage}};
            end else diagnostic_timer<=diagnostic_timer+1'b1;
        end
    end
    always @(posedge clk or negedge rst_n)
        if (!rst_n) was_ready <= 0;
        else was_ready <= usb_ready;

    max3421e_touch_host #(.CLK_HZ(CLK_HZ), .SPI_HZ(SPI_HZ),
        .READ_VENDOR_FEATURE(READ_VENDOR_FEATURE), .SPI_PROBE_ONLY(SPI_PROBE_ONLY),
        .RCVBC_SCOPE_TEST(RCVBC_SCOPE_TEST),
        .CS_HIGH_CYCLES(CS_HIGH_CYCLES),.INIT_TIMEOUT_MS(INIT_TIMEOUT_MS),
        .TRANSFER_TIMEOUT_MS(TRANSFER_TIMEOUT_MS)) host (
        .clk(clk), .rst_n(rst_n), .spi_miso(spi_miso), .max_irq_n(max_irq_n),
        .spi_sclk(spi_sclk), .spi_mosi(spi_mosi), .spi_cs_n(spi_cs_n),
        .max_reset_n(max_reset_n), .vbus_en(vbus_en),
        .usb_connected(usb_connected), .usb_ready(usb_ready),
        .chip_identified(chip_identified),
        .spi_probe_done(spi_probe_done),.spi_probe_pass(spi_probe_pass),
        .spi_probe_event(spi_probe_event),.revision_raw(revision_raw),
        .probe_read_a(probe_read_a),.probe_read_b(probe_read_b),
        .error_code(error_code), .debug_state(debug_state), .last_hrsl(last_hrsl),
        .enumeration_stage(enumeration_stage),
        .initialization_diagnostic(initialization_diagnostic),
        .receive_diagnostic(receive_diagnostic),
        .setup_readback_diagnostic(setup_readback_diagnostic),
        .first_in_diagnostic(first_in_diagnostic),
        .rcvbc_test_data(rcvbc_test_data),
        .scope_count(scope_count),.scope_active(scope_active),
        .report_valid(report_valid), .report_len(report_len), .report_data(report_data)
    );
    ili222a_touch_decode decoder (
        .clk(clk), .rst_n(rst_n), .clear_points(was_ready && !usb_ready),
        .report_valid(report_valid), .report_len(report_len), .report_data(report_data),
        .touch_event(touch_event), .point0_valid(point0_valid), .point0_id(point0_id),
        .point0_x(point0_x), .point0_y(point0_y), .point1_valid(point1_valid),
        .point1_id(point1_id), .point1_x(point1_x), .point1_y(point1_y),
        .touch_count(touch_count), .touch_overflow(touch_overflow), .report_error(report_error)
    );
endmodule

// Communication layer. All transactions are serialized through the original
// transport module. Timeouts and failures restart enumeration, releasing points.
module max3421e_touch_host #(
    parameter integer CLK_HZ = 27000000,
    parameter integer SPI_HZ = 3000000,
    parameter integer READ_VENDOR_FEATURE = 1,
    parameter integer SPI_PROBE_ONLY = 0,
    parameter integer RCVBC_SCOPE_TEST = 0,
    parameter integer CS_HIGH_CYCLES = (CLK_HZ + 199) / 200,
    parameter integer INIT_TIMEOUT_MS = 30000,
    parameter integer TRANSFER_TIMEOUT_MS = 2000
) (
    input wire clk, input wire rst_n,
    input wire spi_miso, input wire max_irq_n,
    output wire spi_sclk, output wire spi_mosi, output wire spi_cs_n,
    output reg max_reset_n, output wire vbus_en,
    output reg usb_connected, output reg usb_ready,
    output reg chip_identified,
    output reg spi_probe_done, spi_probe_pass, spi_probe_event,
    output reg [7:0] revision_raw, probe_read_a, probe_read_b,
    output reg [7:0] error_code, output wire [7:0] debug_state,
    output reg [3:0] last_hrsl,
    output reg report_valid, output reg [6:0] report_len,
    output reg [511:0] report_data,
    output wire [3:0] enumeration_stage,
    output wire [63:0] initialization_diagnostic,
    output reg [63:0] receive_diagnostic,
    output reg [63:0] setup_readback_diagnostic,
    output wire [63:0] first_in_diagnostic,
    output wire [511:0] rcvbc_test_data,
    output reg [7:0] scope_count,
    output reg scope_active
);
    localparam [4:0] R_USBIRQ=13, R_USBCTL=15, R_CPUCTL=16, R_PINCTL=17,
        R_REVISION=18, R_HIRQ=25, R_HIEN=26, R_MODE=27, R_PERADDR=28,
        R_HCTL=29, R_HXFR=30, R_HRSL=31, R_SUDFIFO=4, R_GPINIEN=23;
    localparam [7:0]
        S_BOOT=0, S_RELEASE=1, S_PINCTL=2, S_REVISION=3, S_REV_CHECK=4,
        S_CHIP_RESET=5, S_CHIP_RELEASE=6, S_OSC=7, S_OSC_CHECK=8,
        S_MODE=9, S_HIEN=10, S_CPU=11, S_SCAN_ADDR=12, S_SCAN=13,
        S_SCAN_POLL=14, S_SCAN_POLL_CHECK=15, S_SCAN_EVAL=16,
        S_RESET_BUS=17, S_RESET_POLL=18, S_RESET_CHECK=19,
        S_SOF=20, S_CLEAR=21, S_ENUM=22, S_C_SETUP=23,
        S_C_SETUP_DONE=24, S_C_DATA=25, S_C_DATA_DONE=26,
        S_C_COPY=27, S_C_STATUS=28, S_C_STATUS_DONE=29,
        S_ENUM_CHECK=30, S_ADDRESS=31, S_ENUM_NEXT=32,
        S_POLL=33, S_POLL_CHECK=34, S_POLL_LAUNCH=35, S_POLL_DONE=36,
        S_FAULT=37, S_DETACH=38, S_DELAY=39, S_IO_ISSUE=40, S_IO_WAIT=41,
        S_X_BEGIN=42, S_X_CLEAR=43, S_X_LAUNCH=44, S_X_POLL=45,
        S_X_IRQ=46, S_X_RESULT=47, S_X_DECIDE=48, S_X_ACK=49,
        S_X_POST=50, S_X_RECEIVE=51, S_X_RECEIVED=52, S_X_RETRY=53,
        S_X_RETURN=54, S_REPORT=55,
        S_PROBE_SAVE=56,S_PROBE_A_WRITE=57,S_PROBE_A_READ=58,
        S_PROBE_A_SAVE=59,S_PROBE_B_READ=60,S_PROBE_B_SAVE=61,S_PROBE_DONE=62,
        S_RX_COUNT_SAVE=63,S_RX_IRQ_SAVE=64,
        S_RCVBC_SCOPE_READ=66,S_RCVBC_SCOPE_UPDATE=67,
        S_MODE_VERIFY=68,S_MODE_CHECK=69,S_FRAME_WAIT=70,S_FRAME_CHECK=71,
        S_CONTROL_ADDR_VERIFY=72,S_CONTROL_ADDR_CHECK=73,S_CONTROL_MODE_CHECK=74,
        S_X_ARM_VERIFY=75,S_X_ARM_CHECK=76,
        S_SETUP_READBACK_CHECK=77,S_SETUP_VERIFIED=78,
        S_FIRST_IN_COUNT_CHECK=79,
        S_DIAG_COUNT=80,S_DIAG_COUNT_SAVE=81,S_DIAG_IRQ_SAVE=82,
        S_DIAG_PRE_COUNT=83,S_DIAG_PRE_IRQ=84,S_DIAG_FIFO_SAVE=85,
        S_DIAG_POST_COUNT=86,S_DIAG_POST_IRQ=87,S_DIAG_HOLD=88,
        S_RELEASE_CONFIG=89,S_RELEASE_CONFIG_SAVE=90,
        S_RELEASE_COUNT0=91,S_RELEASE_IRQ0=92,S_RELEASE_DECIDE0=93,
        S_RELEASE_FIFO0=94,S_RELEASE_FIFO0_COUNT=95,S_RELEASE_FIFO0_IRQ=96,
        S_RELEASE_CLEAR=97,S_RELEASE_COUNT1=98,S_RELEASE_IRQ1=99,
        S_RELEASE_WAIT=100,S_RELEASE_COUNT2=101,S_RELEASE_IRQ2=102,
        S_RELEASE_DECIDE2=103,S_RELEASE_FIFO1=104,S_RELEASE_FINAL_COUNT=105,
        S_RELEASE_FINAL_IRQ=106,S_RELEASE_DONE=107,
        S_A5_RELEASE=108,S_A5_WAIT=109,S_A5_IRQ=110,S_A5_IRQ_CHECK=111,
        S_A5_COUNT_CHECK=112,S_A5_FIFO_CHECK=113,S_A5_ACK_DONE=114;
    reg [7:0] probe_original;
    localparam integer MS_CYCLES=(CLK_HZ+999)/1000;
    reg [7:0] state, io_return, delay_return, x_return;
    reg [31:0] ms_counter;
    wire ms_tick=(ms_counter==MS_CYCLES-1);
    reg [15:0] delay_ms, init_ms, x_ms;
    reg debounced, ep1_toggle, x_active;
    reg [2:0] command_op;
    reg [4:0] command_reg;
    reg [6:0] command_len;
    reg [511:0] command_tx;
    wire command_ready, command_done, receive_packet;
    wire [2:0] command_error;
    wire [6:0] receive_len;
    wire [511:0] receive_data;
    reg [511:0] io_data;
    reg [6:0] io_len;
    reg io_has_packet;
    reg [7:0] x_token, x_hrsl;
    reg x_receive, x_retry;
    reg [2:0] x_attempt;
    reg [3:0] x_result;
    reg [511:0] x_data;
    reg [6:0] x_len;
    reg [3:0] enum_step;
    reg [15:0] ctrl_length, ctrl_received;
    wire [7:0] logical_phase=(state==S_IO_ISSUE || state==S_IO_WAIT)?io_return:
        ((state==S_DELAY)?delay_return:state);
    reg [7:0] last_usb_irq,last_hctl,last_bus_hrsl,timeout_phase;
    reg [7:0] failure_detail,failure_length,failure_enum;
    reg [7:0] receive_count_before,receive_irq_before;
    reg first_in_locked;
    reg [7:0] first_setup_hrsl,first_setup_hirq,first_pre_hirq,first_pre_count;
    reg [7:0] first_post_hirq,first_post_count,first_post_hrsl,arm_hirq;
    assign first_in_diagnostic={8'ha2,first_setup_hrsl,first_setup_hirq,
        first_pre_hirq,first_pre_count,first_post_hirq,first_post_count,first_post_hrsl};
    // RCVBC_SCOPE_TEST=2 is a one-shot diagnostic, not an enumeration workaround.
    // No HIRQ writes or USB transfers occur after the first successful IN.
    reg [7:0] diag_phase,diag_samples,diag_first_hrsl,diag_first_hirq;
    reg [7:0] diag_first_count,diag_last_count,diag_pre_count,diag_post_count;
    reg [7:0] diag_pre_hirq,diag_post_hirq,diag_io_error;
    reg [63:0] diag_counts,diag_irqs;
    reg [143:0] diag_fifo;
    reg diag_done,diag_fifo_read,diag_any18,diag_all_zero,diag_all_pending;
    localparam [143:0] EXPECTED_DEVICE_DESCRIPTOR=
        144'h0100020100020001222a4000000002000112;
    wire diag_fifo_match=diag_fifo_read && diag_fifo==EXPECTED_DEVICE_DESCRIPTOR;
    wire [7:0] diag_flags={diag_done,diag_all_pending,diag_all_zero,diag_any18,
        diag_fifo_match,diag_fifo_read,diag_first_hirq[2],diag_pre_hirq[2]};
    // A4: retain both observations around exactly one RCVDAV W1C.
    // Fixed 18-byte reads are diagnostic observations, never enumeration data.
    reg [7:0] release_phase,release_io_error,release_hrsl,release_writes;
    reg [7:0] release_count0,release_irq0,release_count1,release_irq1;
    reg [7:0] release_count2,release_irq2,release_fifo0_count,release_fifo0_irq;
    reg [7:0] release_final_count,release_final_irq;
    reg [7:0] release_mode,release_pinctl,release_usbctl,release_address;
    reg [1:0] release_config_index;
    reg [143:0] release_fifo0,release_fifo1;
    reg release_done,release_read0,release_read1;
    wire release_match0=release_read0 && release_fifo0==EXPECTED_DEVICE_DESCRIPTOR;
    wire release_match1=release_read1 && release_fifo1==EXPECTED_DEVICE_DESCRIPTOR;
    wire [7:0] release_flags={release_done,release_match1,release_match0,
        release_read1,release_read0,release_irq2[2],release_irq1[2],release_irq0[2]};
    wire [511:0] release_test_data={first_in_diagnostic,
        release_fifo1[135:128],release_fifo1[143:136],release_mode,release_pinctl,
        release_usbctl,release_address,release_final_count,release_final_irq,
        release_fifo1[71:64],release_fifo1[79:72],release_fifo1[87:80],release_fifo1[95:88],
        release_fifo1[103:96],release_fifo1[111:104],release_fifo1[119:112],release_fifo1[127:120],
        release_fifo1[7:0],release_fifo1[15:8],release_fifo1[23:16],release_fifo1[31:24],
        release_fifo1[39:32],release_fifo1[47:40],release_fifo1[55:48],release_fifo1[63:56],
        8'ha4,release_phase,release_flags,release_hrsl,
        release_count0,release_irq0,release_count1,release_irq1,
        release_fifo0[7:0],release_fifo0[15:8],release_fifo0[23:16],release_fifo0[31:24],
        release_fifo0[39:32],release_fifo0[47:40],release_fifo0[55:48],release_fifo0[63:56],
        release_fifo0[71:64],release_fifo0[79:72],release_fifo0[87:80],release_fifo0[95:88],
        release_fifo0[103:96],release_fifo0[111:104],release_fifo0[119:112],release_fifo0[127:120],
        release_fifo0[135:128],release_fifo0[143:136],release_count2,release_irq2,
        release_fifo0_count,release_fifo0_irq,release_writes,release_io_error};
    reg [7:0] a5_phase,a5_before_count,a5_before_irq,a5_after_count,a5_after_irq;
    reg [7:0] a5_recovery_phase,a5_first_len;
    reg a5_recovery_used,a5_recovery_success,a5_descriptor_valid,a5_frozen;
    reg a5_ready_seen,a5_report_seen,a5_later_zero;
    reg [143:0] a5_descriptor;
    wire [7:0] a5_flags={a5_frozen,a5_ready_seen,a5_descriptor_valid,
        a5_recovery_used,a5_recovery_success,usb_connected,a5_report_seen,a5_later_zero};
    wire [511:0] a5_test_data={
        failure_enum,timeout_phase,failure_detail,failure_length,ctrl_received,ctrl_length,
        64'd0,
        report_data[7:0],report_data[15:8],report_data[23:16],report_data[31:24],
        report_data[39:32],report_data[47:40],report_data[55:48],report_data[63:56],
        a5_descriptor[135:128],a5_descriptor[143:136],release_mode,release_pinctl,
        release_usbctl,release_address,{1'b0,report_len},report_data[7:0],
        8'ha5,a5_phase,{4'd0,enum_step},{7'd0,usb_ready},error_code,
        logical_phase,{4'd0,last_hrsl},a5_flags,
        a5_before_count,a5_before_irq,a5_after_count,a5_after_irq,
        {7'd0,a5_recovery_used},{7'd0,a5_recovery_success},a5_recovery_phase,a5_first_len,
        a5_descriptor[7:0],a5_descriptor[15:8],a5_descriptor[23:16],a5_descriptor[31:24],
        a5_descriptor[39:32],a5_descriptor[47:40],a5_descriptor[55:48],a5_descriptor[63:56],
        a5_descriptor[71:64],a5_descriptor[79:72],a5_descriptor[87:80],a5_descriptor[95:88],
        a5_descriptor[103:96],a5_descriptor[111:104],a5_descriptor[119:112],a5_descriptor[127:120]};
    assign rcvbc_test_data=(RCVBC_SCOPE_TEST==4)?a5_test_data:
        (RCVBC_SCOPE_TEST==3)?release_test_data:{first_in_diagnostic,setup_readback_diagnostic,
        diag_fifo[135:128],diag_fifo[143:136],diag_pre_hirq,diag_post_hirq,
        diag_first_count,diag_last_count,8'h12,diag_io_error,
        diag_fifo[71:64],diag_fifo[79:72],diag_fifo[87:80],diag_fifo[95:88],
        diag_fifo[103:96],diag_fifo[111:104],diag_fifo[119:112],diag_fifo[127:120],
        // The renderer stores rows 0..3 from bit 255 down to bit 0.
        8'ha3,diag_phase,diag_flags,diag_samples,diag_first_hrsl,diag_first_hirq,
        diag_pre_count,diag_post_count,
        diag_counts,diag_irqs,
        diag_fifo[7:0],diag_fifo[15:8],diag_fifo[23:16],diag_fifo[31:24],
        diag_fifo[39:32],diag_fifo[47:40],diag_fifo[55:48],diag_fifo[63:56]};
    assign initialization_diagnostic={logical_phase,timeout_phase,last_usb_irq,last_hctl,
        last_bus_hrsl,failure_detail,failure_length,failure_enum};
    assign enumeration_stage=enum_step;
    reg [63:0] setup_packet;
    reg [6:0] copy_index;
    reg [511:0] ctrl_first;
    reg [31:0] ctrl_crc;
    wire [7:0] copy_byte=x_data[copy_index*8 +: 8];
    wire [7:0] setup_mismatch;
    genvar setup_byte_no;
    generate for(setup_byte_no=0;setup_byte_no<8;setup_byte_no=setup_byte_no+1)begin:g_setup_compare
        assign setup_mismatch[setup_byte_no]=
            (io_data[setup_byte_no*8 +: 8]!=setup_packet[setup_byte_no*8 +: 8]);
    end endgenerate

    assign debug_state=state;
    // Active-high request for an EXTERNAL 5-V USB power switch, if present.
    // Never connect this logic signal directly to the USB VBUS power pin.
    assign vbus_en=rst_n;

    // Keep the proven scope-friendly CS interval in full USB mode as well.
    max3421e_spi_transport #(.CLK_HZ(CLK_HZ), .SPI_HZ(SPI_HZ),
        .CS_HIGH_CYCLES(CS_HIGH_CYCLES)) transport (
        .clk(clk), .rst_n(rst_n), .cmd_valid(state==S_IO_ISSUE),
        .cmd_ready(command_ready), .cmd_op(command_op), .cmd_reg(command_reg),
        .cmd_len(command_len), .cmd_tx_data(command_tx), .busy(),
        .done(command_done), .error_code(command_error), .has_packet(receive_packet),
        .rx_len(receive_len), .rx_data(receive_data), .rx_byte_valid(),
        .rx_byte(), .rx_byte_index(), .rx_byte_last(), .status_byte(),
        .irq_n(max_irq_n), .irq_active(), .spi_cs_n(spi_cs_n),
        .spi_sclk(spi_sclk), .spi_mosi(spi_mosi), .spi_miso(spi_miso)
    );

    function [31:0] crc_byte;
        input [31:0] crc;
        input [7:0] data;
        reg [31:0] c;
        integer bitno;
        begin
            c=crc ^ {24'd0,data};
            for (bitno=0; bitno<8; bitno=bitno+1)
                c=c[0] ? ((c>>1)^32'hedb88320) : (c>>1);
            crc_byte=c;
        end
    endfunction

    // Extra waits between samples; SPI transactions also take time.
    function [15:0] diag_wait_ms;
        input [2:0] finished_index;
        begin
            case(finished_index)
                0:diag_wait_ms=0;
                1:diag_wait_ms=10;
                2:diag_wait_ms=50;
                3:diag_wait_ms=100;
                4:diag_wait_ms=250;
                5:diag_wait_ms=500;
                default:diag_wait_ms=1000;
            endcase
        end
    endfunction

    // Tasks only assign registers in this one clocked state machine.
    task issue_io;
        input [2:0] op;
        input [4:0] r;
        input [6:0] length;
        input [511:0] data;
        input [7:0] return_to;
        begin
            command_op<=op; command_reg<=r; command_len<=length;
            command_tx<=data; io_return<=return_to; state<=S_IO_ISSUE;
        end
    endtask
    task read_reg;
        input [4:0] r;
        input [7:0] return_to;
        begin issue_io(0,r,1,512'd0,return_to); end
    endtask
    task write_reg;
        input [4:0] r;
        input [7:0] data, return_to;
        begin issue_io(1,r,1,{504'd0,data},return_to); end
    endtask
    task pause_ms;
        input [15:0] duration;
        input [7:0] return_to;
        begin
            // +1 ensures at least the requested duration for any tick phase.
            delay_ms<=duration+1'b1; delay_return<=return_to; state<=S_DELAY;
        end
    endtask
    task transaction;
        input [7:0] token;
        input receive_payload;
        input [7:0] return_to;
        begin
            x_token<=token; x_receive<=receive_payload; x_return<=return_to;
            state<=S_X_BEGIN;
        end
    endtask
    task fail;
        input [7:0] code;
        begin
            // Retain the exact failure site across automatic initialization retries.
            timeout_phase<=logical_phase;
            failure_enum<={4'd0,enum_step};
            receive_diagnostic<={x_token,x_hrsl,receive_irq_before,receive_count_before,
                ctrl_length[7:0],ctrl_received[7:0],{1'b0,io_len},{1'b0,x_len}};
            failure_length<={1'b0,x_len};failure_detail<=0;
            if(code==8'h42)begin failure_detail<=8'h1b;failure_length<=io_data[7:0];end
            if(code==8'h43)begin failure_detail<=8'h1c;failure_length<=io_data[7:0];end
            if(code==8'h44)begin failure_detail<=8'h19;failure_length<=io_data[7:0];end
            if(code==8'h45)begin failure_detail<=8'h04;failure_length<=setup_mismatch;end
            if(code==8'h46)begin failure_detail<=arm_hirq;failure_length<=io_data[7:0];end
            if(code==8'h40) begin
                if(logical_phase==S_X_RECEIVED) begin
                    failure_detail<=1;failure_length<={1'b0,io_len};
                end else if(logical_phase==S_C_COPY) begin
                    failure_detail<=(x_len==0)?2:3;
                end
            end
            error_code<=code; state<=S_FAULT; x_active<=0; init_ms<=0;
            usb_ready<=0; usb_connected<=0;
            if(RCVBC_SCOPE_TEST==4)begin
                // A5 is also used by the normal build for its guarded first-
                // descriptor recovery. Do not turn a transient read/USB error
                // into a permanent lock: retain diagnostics, then let S_FAULT
                // reset the MAX3421E and enumerate again after the backoff.
                a5_frozen<=0;a5_phase<=8'he0;scope_active<=1;
            end
        end
    endtask
    task control_fail;
        begin
            // Feature 06 is a read-only vendor query seen on Windows.
            // A STALL on this optional query does not block touch operation.
            if ((enum_step==9) && (x_result==5)) begin
                usb_ready<=1; error_code<=0; state<=S_POLL;
            end
            else fail(8'h30 | {4'd0,x_result});
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ms_counter<=0;
        else if (ms_tick) ms_counter<=0;
        else ms_counter<=ms_counter+1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=S_BOOT; max_reset_n<=0; usb_connected<=0; usb_ready<=0;
            last_usb_irq<=0;last_hctl<=0;last_bus_hrsl<=0;timeout_phase<=0;
            failure_detail<=0;failure_length<=0;failure_enum<=0;
            receive_count_before<=0;receive_irq_before<=0;receive_diagnostic<=0;
            setup_readback_diagnostic<=0;
            first_in_locked<=0;arm_hirq<=0;
            first_setup_hrsl<=8'hff;first_setup_hirq<=8'hff;
            first_pre_hirq<=8'hff;first_pre_count<=8'hff;
            first_post_hirq<=8'hff;first_post_count<=8'hff;first_post_hrsl<=8'hff;
            scope_count<=0;scope_active<=0;
            diag_phase<=0;diag_samples<=0;diag_first_hrsl<=8'hff;diag_first_hirq<=8'hff;
            diag_first_count<=8'hff;diag_last_count<=8'hff;
            diag_pre_count<=8'hff;diag_post_count<=8'hff;
            diag_pre_hirq<=8'hff;diag_post_hirq<=8'hff;diag_io_error<=0;
            diag_counts<=64'hffffffffffffffff;diag_irqs<=64'hffffffffffffffff;
            diag_fifo<=0;diag_done<=0;diag_fifo_read<=0;
            diag_any18<=0;diag_all_zero<=1;diag_all_pending<=1;
            release_phase<=0;release_io_error<=0;release_hrsl<=8'hff;release_writes<=0;
            release_count0<=8'hff;release_irq0<=8'hff;
            release_count1<=8'hff;release_irq1<=8'hff;
            release_count2<=8'hff;release_irq2<=8'hff;
            release_fifo0_count<=8'hff;release_fifo0_irq<=8'hff;
            release_final_count<=8'hff;release_final_irq<=8'hff;
            release_mode<=8'hff;release_pinctl<=8'hff;release_usbctl<=8'hff;release_address<=8'hff;
            release_config_index<=0;release_fifo0<=0;release_fifo1<=0;
            release_done<=0;release_read0<=0;release_read1<=0;
            a5_phase<=0;a5_before_count<=8'hff;a5_before_irq<=8'hff;
            a5_after_count<=8'hff;a5_after_irq<=8'hff;
            a5_recovery_phase<=0;a5_first_len<=0;a5_descriptor<=0;
            a5_recovery_used<=0;a5_recovery_success<=0;a5_descriptor_valid<=0;
            a5_frozen<=0;a5_ready_seen<=0;a5_report_seen<=0;a5_later_zero<=0;
            chip_identified<=0;
            spi_probe_done<=0;spi_probe_pass<=0;spi_probe_event<=0;
            revision_raw<=0;probe_read_a<=0;probe_read_b<=0;probe_original<=0;
            error_code<=0; last_hrsl<=0; report_valid<=0; report_len<=0; report_data<=0;
            io_return<=0; delay_return<=0; x_return<=0;
            delay_ms<=0; init_ms<=0; x_ms<=0; debounced<=0; ep1_toggle<=0;
            x_active<=0; command_op<=0; command_reg<=0; command_len<=0; command_tx<=0;
            io_data<=0; io_len<=0; io_has_packet<=0; x_token<=0; x_hrsl<=0;
            x_receive<=0; x_retry<=0; x_attempt<=0; x_result<=0; x_data<=0; x_len<=0;
            enum_step<=0; setup_packet<=0; ctrl_length<=0;
            ctrl_received<=0; copy_index<=0; ctrl_first<=0; ctrl_crc<=32'hffffffff;
        end else begin
            report_valid<=0;
            spi_probe_event<=0;
            if(RCVBC_SCOPE_TEST==4)begin
                if(usb_ready)begin a5_ready_seen<=1;a5_phase<=5;end
                if(report_valid)a5_report_seen<=1;
            end
            if (ms_tick) begin
                if (usb_ready || state==S_BOOT || state==S_FAULT) init_ms<=0;
                else if (init_ms<INIT_TIMEOUT_MS+1) init_ms<=init_ms+1'b1;
                if (x_active && x_ms<TRANSFER_TIMEOUT_MS+1) x_ms<=x_ms+1'b1;
            end
            if (!usb_ready && init_ms>=INIT_TIMEOUT_MS &&
                !(((RCVBC_SCOPE_TEST==2 || RCVBC_SCOPE_TEST==3) && scope_active) ||
                    (RCVBC_SCOPE_TEST==4 && a5_frozen))) fail(8'h20);
            else if (x_active && x_ms>=TRANSFER_TIMEOUT_MS) fail(8'h21);
            else case (state)
                S_BOOT: begin
                    usb_ready<=0; usb_connected<=0; max_reset_n<=0;
                    debounced<=0; ep1_toggle<=0; init_ms<=0; x_active<=0;
                    if(RCVBC_SCOPE_TEST==4)begin
                        // Recovery is allowed once per enumeration attempt,
                        // not once per FPGA power cycle. A failed attempt must
                        // therefore start the next attempt with a fresh guard.
                        a5_recovery_used<=0;a5_recovery_success<=0;
                        a5_descriptor_valid<=0;a5_frozen<=0;
                        a5_recovery_phase<=0;a5_first_len<=0;
                    end
                    pause_ms(10,S_RELEASE);
                end
                S_RELEASE: begin max_reset_n<=1; pause_ms(20,S_PINCTL); end
                S_PINCTL: write_reg(R_PINCTL,8'h18,S_REVISION); // full duplex, low level INT
                S_REVISION: read_reg(R_REVISION,S_REV_CHECK);
                S_REV_CHECK: begin
                    revision_raw<=io_data[7:0];
                    if (io_data[7:0]==8'h13) begin
                        // Sticky evidence: a real SPI revision read succeeded
                        // since external reset. USB attachment is not required.
                        chip_identified<=1;
                    end
                    // GPINIEN is a read/write mask, not GPIO output data.
                    // Save, test both polarities, then restore before USB starts.
                    if (SPI_PROBE_ONLY!=0) state<=S_PROBE_A_WRITE;
                    else read_reg(R_GPINIEN,S_PROBE_SAVE);
                end
                S_PROBE_SAVE: begin
                    probe_original<=io_data[7:0];state<=S_PROBE_A_WRITE;
                end
                S_PROBE_A_WRITE: write_reg(R_GPINIEN,8'h55,S_PROBE_A_READ);
                S_PROBE_A_READ: read_reg(R_GPINIEN,S_PROBE_A_SAVE);
                S_PROBE_A_SAVE: begin
                    if (SPI_PROBE_ONLY!=0) begin
                        // Fixed diagnostic: A is the written/expected value;
                        // B is the latest actual read. Publish every read.
                        probe_read_a<=8'h55;
                        probe_read_b<=io_data[7:0];
                        spi_probe_done<=1;spi_probe_event<=1;
                        spi_probe_pass<=(io_data[7:0]==8'h55);
                        error_code<=(io_data[7:0]==8'h55)?8'h00:8'h08;
                        init_ms<=0;
                        pause_ms(100,S_PROBE_A_READ);
                    end else begin
                        probe_read_a<=io_data[7:0];
                        write_reg(R_GPINIEN,8'haa,S_PROBE_B_READ);
                    end
                end
                S_PROBE_B_READ: read_reg(R_GPINIEN,S_PROBE_B_SAVE);
                S_PROBE_B_SAVE: begin
                    probe_read_b<=io_data[7:0];
                    write_reg(R_GPINIEN,probe_original,S_PROBE_DONE);
                end
                S_PROBE_DONE: begin
                    spi_probe_done<=1;spi_probe_event<=1;
                    spi_probe_pass<=(probe_read_a==8'h55 && probe_read_b==8'haa);
                    if (SPI_PROBE_ONLY!=0) begin
                        init_ms<=0;
                        if (probe_read_a!=8'h55 || probe_read_b!=8'haa) error_code<=8'h08;
                        else if (revision_raw!=8'h13) error_code<=8'h01;
                        else error_code<=0;
                        pause_ms(500,S_PINCTL);
                    end else if (probe_read_a!=8'h55 || probe_read_b!=8'haa) fail(8'h08);
                    else if (revision_raw!=8'h13) fail(8'h01);
                    else state<=S_CHIP_RESET;
                end
                S_CHIP_RESET: write_reg(R_USBCTL,8'h20,S_X_RETRY);
                // Explicit reset hold independent of SPI speed or CS gap.
                S_X_RETRY: pause_ms(1,S_CHIP_RELEASE);
                S_CHIP_RELEASE: write_reg(R_USBCTL,8'h00,S_OSC);
                S_OSC: read_reg(R_USBIRQ,S_OSC_CHECK);
                S_OSC_CHECK: begin
                    if (io_data[0]) state<=S_MODE;
                    else pause_ms(1,S_OSC);
                end
                S_MODE: write_reg(R_MODE,8'hc1,S_MODE_VERIFY);
                S_MODE_VERIFY: read_reg(R_MODE,S_MODE_CHECK);
                S_MODE_CHECK: begin
                    if((io_data[7:0]&8'hc3)!=8'hc1)fail(8'h42);
                    else state<=S_HIEN;
                end
                S_HIEN: write_reg(R_HIEN,8'ha4,S_CPU); // HXFRDN, CONDET, RCVDAV
                S_CPU: write_reg(R_CPUCTL,8'h01,S_SCAN_ADDR);
                S_SCAN_ADDR: write_reg(R_PERADDR,8'h00,S_SCAN);
                S_SCAN: write_reg(R_HCTL,8'h04,S_SCAN_POLL);
                S_SCAN_POLL: read_reg(R_HCTL,S_SCAN_POLL_CHECK);
                S_SCAN_POLL_CHECK: begin
                    // Match USB Host Shield and observed silicon: SAMPLEBUS=1
                    // is the completion condition, not a bit to wait to clear.
                    if (io_data[2]) read_reg(R_HRSL,S_SCAN_EVAL);
                    else state<=S_SCAN_POLL;
                end
                S_SCAN_EVAL: begin
                    init_ms<=0;
                    if (io_data[7:6]==2'b10) begin
                        if (!debounced) begin debounced<=1; pause_ms(100,S_SCAN); end
                        else begin usb_connected<=1; state<=S_RESET_BUS; end
                    end else begin
                        debounced<=0;
                        if (io_data[7:6]==2'b01) error_code<=8'h07; // unsupported LS target
                        pause_ms(10,S_SCAN);
                    end
                end
                S_RESET_BUS: write_reg(R_HCTL,8'h03,S_RESET_POLL);
                S_RESET_POLL: read_reg(R_HCTL,S_RESET_CHECK);
                S_RESET_CHECK: begin
                    if (io_data[0]) pause_ms(1,S_RESET_POLL);
                    else state<=S_SOF;
                end
                S_SOF: begin
                    // The SIE generates the subsequent 1-ms SOFs itself.
                    write_reg(R_MODE,8'hc9,S_CLEAR);
                end
                S_CLEAR: begin
                    enum_step<=0;
                    // Require a NEW frame marker, not just elapsed wall time.
                    write_reg(R_HIRQ,8'h40,S_FRAME_WAIT);
                end
                S_FRAME_WAIT: read_reg(R_HIRQ,S_FRAME_CHECK);
                S_FRAME_CHECK: begin
                    if(io_data[6])pause_ms(100,S_ENUM_NEXT);
                    else pause_ms(1,S_FRAME_WAIT);
                end
                S_ENUM_NEXT: begin
                    // step 0 is entered once after bus reset; later steps use
                    // S_ENUM_CHECK to advance. Never clear RCVDAV mid-transfer.
                    write_reg(R_HIRQ,8'hf7,S_ENUM);
                end
                S_ENUM: begin
                    ctrl_received<=0; ctrl_first<=0; ctrl_crc<=32'hffffffff;
                    case (enum_step)
                        // Target-specific comparison: this screen has EP0 MPS=64.
                        // Request the complete 18-byte descriptor at address zero.
                        0: begin setup_packet<=64'h0012000001000680; ctrl_length<=18; end
                        1: begin setup_packet<=64'h0000000000010500; ctrl_length<=0; end
                        2: begin setup_packet<=64'h0012000001000680; ctrl_length<=18; end
                        3: begin setup_packet<=64'h0009000002000680; ctrl_length<=9; end
                        4: begin setup_packet<=64'h0029000002000680; ctrl_length<=41; end
                        5: begin setup_packet<=64'h0000000000010900; ctrl_length<=0; end
                        6: begin setup_packet<=64'h0000000000000a21; ctrl_length<=0; end
                        7: begin setup_packet<=64'h0352000022000681; ctrl_length<=850; end
                        8: begin setup_packet<=64'h00020000030201a1; ctrl_length<=2; end
                        9: begin setup_packet<=64'h01010000030601a1; ctrl_length<=257; end
                        default: begin setup_packet<=0; ctrl_length<=0; end
                    endcase
                    write_reg(R_HCTL,8'h50,S_CONTROL_ADDR_VERIFY); // control DATA0 before SETUP
                end
                S_CONTROL_ADDR_VERIFY: read_reg(R_PERADDR,S_CONTROL_ADDR_CHECK);
                S_CONTROL_ADDR_CHECK: begin
                    if(io_data[7:0]!=((enum_step<=1)?8'd0:8'd1))fail(8'h43);
                    else read_reg(R_MODE,S_CONTROL_MODE_CHECK);
                end
                S_CONTROL_MODE_CHECK: begin
                    if((io_data[7:0]&8'hcb)!=8'hc9)fail(8'h42);
                    else if(RCVBC_SCOPE_TEST==4 && enum_step==0)begin
                        a5_phase<=1;release_config_index<=0;state<=S_RELEASE_CONFIG;
                    end
                    else state<=S_C_SETUP;
                end
                S_C_SETUP: issue_io(3,R_SUDFIFO,8,{448'd0,setup_packet},S_C_SETUP_DONE);
                S_C_SETUP_DONE: issue_io(2,R_SUDFIFO,8,512'd0,S_SETUP_READBACK_CHECK);
                S_SETUP_READBACK_CHECK: begin
                    // Screen row 6 now contains bytes returned by the MAX,
                    // rather than a trace of the FPGA's own MOSI output.
                    setup_readback_diagnostic<={io_data[7:0],io_data[15:8],io_data[23:16],io_data[31:24],
                        io_data[39:32],io_data[47:40],io_data[55:48],io_data[63:56]};
                    if(setup_mismatch!=0)fail(8'h45);
                    // Reload all eight bytes after readback, so transmission
                    // never depends on an assumed FIFO read-pointer state.
                    else issue_io(3,R_SUDFIFO,8,{448'd0,setup_packet},S_SETUP_VERIFIED);
                end
                S_SETUP_VERIFIED: transaction(8'h10,0,S_C_DATA);
                S_C_DATA: begin
                    if (x_result!=0) control_fail;
                    else if (ctrl_length==0) state<=S_C_STATUS;
                    else write_reg(R_HCTL,8'h20,S_C_DATA_DONE); // DATA1 for first IN
                end
                S_C_DATA_DONE: transaction(8'h00,1,S_C_COPY);
                S_C_COPY: begin
                    if (x_result!=0) control_fail;
                    else if(x_len==0 && enum_step==9)state<=S_C_STATUS;
                    else if ((x_len==0) || (ctrl_received+{9'd0,x_len}>ctrl_length)) fail(8'h40);
                    else begin copy_index<=0; state<=S_C_STATUS_DONE; end
                end
                S_C_STATUS_DONE: begin
                    // Copy and checksum one payload byte per system clock.
                    ctrl_crc<=crc_byte(ctrl_crc,copy_byte);
                    if ((ctrl_received+{9'd0,copy_index})<64)
                        ctrl_first[(ctrl_received+{9'd0,copy_index})*8 +: 8]<=copy_byte;
                    if (copy_index+7'd1==x_len) begin
                        ctrl_received<=ctrl_received+{9'd0,x_len};
                        if (ctrl_received+{9'd0,x_len}==ctrl_length) state<=S_C_STATUS;
                        // A short packet terminates a control IN data stage.
                        // Validate required descriptor lengths after status.
                        else if (x_len<64) state<=S_C_STATUS;
                        else state<=S_C_DATA_DONE;
                    end else copy_index<=copy_index+1'b1;
                end
                S_C_STATUS: begin
                    if (ctrl_length==0) transaction(8'h80,0,S_ENUM_CHECK); // HS-IN
                    else transaction(8'ha0,0,S_ENUM_CHECK);              // HS-OUT
                end
                S_ENUM_CHECK: begin
                    if (x_result!=0) control_fail;
                    else case (enum_step)
                        0: begin
                            if ((ctrl_received<8) || (ctrl_first[7:0]!=18) || (ctrl_first[15:8]!=1) ||
                                (ctrl_first[63:56]!=64)) fail(8'h02);
                            else begin enum_step<=1; state<=S_ENUM; end
                        end
                        1: begin enum_step<=2; pause_ms(10,S_ADDRESS); end
                        2: begin
                            if ((ctrl_received<18) || (ctrl_first[79:64]!=16'h222a) ||
                                (ctrl_first[95:80]!=16'h0001)) fail(8'h03);
                            else begin enum_step<=3; state<=S_ENUM; end
                        end
                        3: begin
                            if (ctrl_received<9 || ctrl_first[31:16]!=41 || ctrl_first[39:32]!=1 ||
                                ctrl_first[47:40]!=1) fail(8'h04);
                            else begin enum_step<=4; state<=S_ENUM; end
                        end
                        4: begin
                            if (ctrl_received<41 || ctrl_first[119:112]!=3 || ctrl_first[111:104]!=2 ||
                                ctrl_first[95:88]!=0 || ctrl_first[103:96]!=0 ||
                                ctrl_first[215:200]!=850 || ctrl_first[239:232]!=8'h81 ||
                                ctrl_first[255:248]!=64 || ctrl_first[263:256]!=0 ||
                                ctrl_first[247:240]!=3 || ctrl_first[271:264]!=1)
                                fail(8'h04);
                            else begin enum_step<=5; state<=S_ENUM; end
                        end
                        5: begin enum_step<=6; ep1_toggle<=0; state<=S_ENUM; end
                        6: begin enum_step<=7; state<=S_ENUM; end
                        7: begin
                            if (ctrl_received!=850 || (~ctrl_crc)!=32'h68073076) fail(8'h05);
                            else begin enum_step<=8; state<=S_ENUM; end
                        end
                        8: begin
                            if (ctrl_received<2 || ctrl_first[15:0]!=16'h0a02) fail(8'h06);
                            else if (READ_VENDOR_FEATURE!=0) begin enum_step<=9; state<=S_ENUM; end
                            else begin usb_ready<=1; error_code<=0; state<=S_POLL; end
                        end
                        9: begin usb_ready<=1; error_code<=0; state<=S_POLL; end
                        default: fail(8'h04);
                    endcase
                end
                S_ADDRESS: write_reg(R_PERADDR,8'h01,S_ENUM);
                S_POLL: pause_ms(1,S_POLL_CHECK);
                S_POLL_CHECK: read_reg(R_HIRQ,S_POLL_LAUNCH);
                S_POLL_LAUNCH: begin
                    if (io_data[5]) state<=S_DETACH;
                    else write_reg(R_HCTL,ep1_toggle ? 8'h20 : 8'h10,S_POLL_DONE);
                end
                S_POLL_DONE: transaction(8'h01,1,S_REPORT);
                // A distinct return state keeps enumeration IN data private.
                S_REPORT: begin
                    if (x_result==0) begin
                        ep1_toggle<=x_hrsl[4];
                        report_valid<=1; report_len<=x_len; report_data<=x_data;
                        state<=S_POLL;
                    end else if (x_result==4) state<=S_POLL; // NAK is normal idle
                    else fail(8'h30 | {4'd0,x_result});
                end
                S_DETACH: begin
                    // Connection-change IRQ can be a short cable/power glitch.
                    // Always restart enumeration; diagnostic A5 mode must not
                    // freeze normal touch operation until external reset.
                    usb_connected<=0; usb_ready<=0; x_active<=0;
                    error_code<=0; init_ms<=0; state<=S_BOOT;
                end
                S_FAULT: begin
                    usb_ready<=0; usb_connected<=0; x_active<=0; init_ms<=0;
                    // Error remains visible through the next initialization.
                    pause_ms(500,S_BOOT);
                end
                S_DELAY: if (ms_tick) begin
                    if (delay_ms<=1) state<=delay_return;
                    else delay_ms<=delay_ms-1'b1;
                end
                S_IO_ISSUE: if (command_ready) state<=S_IO_WAIT;
                S_IO_WAIT: if (command_done) begin
                    if(command_op==0) begin
                        if(command_reg==R_USBIRQ)last_usb_irq<=receive_data[7:0];
                        if(command_reg==R_HCTL)last_hctl<=receive_data[7:0];
                        if(command_reg==R_HRSL)last_bus_hrsl<=receive_data[7:0];
                    end
                    io_data<=receive_data; io_len<=receive_len; io_has_packet<=receive_packet;
                    if(command_error!=0 && (RCVBC_SCOPE_TEST==2 || RCVBC_SCOPE_TEST==3) && scope_active)begin
                        if(RCVBC_SCOPE_TEST==3)begin
                            release_io_error<={5'd0,command_error};release_phase<=8'he0;release_done<=1;
                        end else begin
                            diag_io_error<={5'd0,command_error};diag_phase<=8'he0;diag_done<=1;
                        end
                        x_active<=0;state<=S_DIAG_HOLD;
                    end else if (command_error!=0) fail(8'h10);
                    else state<=io_return;
                end
                S_X_BEGIN: begin
                    x_active<=1; x_ms<=0; x_attempt<=0; x_data<=0; x_len<=0;
                    x_result<=0; state<=S_X_CLEAR;
                end
                S_X_CLEAR: write_reg(R_HIRQ,8'h80,S_X_ARM_VERIFY);
                S_X_ARM_VERIFY: read_reg(R_HIRQ,S_X_ARM_CHECK);
                S_X_ARM_CHECK: begin
                    if(io_data[7])fail(8'h44);
                    else if(enum_step==0 && x_token==8'h00 && ctrl_received==0)begin
                        // Any pending receive here predates the next IN
                        // attempt, including attempts after a control NAK.
                        arm_hirq<=io_data[7:0];
                        if(!first_in_locked)first_pre_hirq<=io_data[7:0];
                        read_reg(5'd6,S_FIRST_IN_COUNT_CHECK);
                    end
                    else state<=S_X_LAUNCH;
                end
                S_FIRST_IN_COUNT_CHECK: begin
                    if(!first_in_locked)first_pre_count<=io_data[7:0];
                    // Preserve unexpected pending data for diagnosis instead
                    // of silently clearing or misattributing it to the new IN.
                    if(arm_hirq[2])begin first_in_locked<=1;fail(8'h46);end
                    else state<=S_X_LAUNCH;
                end
                S_X_LAUNCH: write_reg(R_HXFR,x_token,S_X_POLL);
                S_X_POLL: read_reg(R_HIRQ,S_X_IRQ);
                S_X_IRQ: begin
                    if(!first_in_locked && enum_step==0 && io_data[7])begin
                        if(x_token==8'h10)first_setup_hirq<=io_data[7:0];
                        if(x_token==8'h00)first_post_hirq<=io_data[7:0];
                    end
                    if (io_data[5]) begin x_active<=0; state<=S_DETACH; end
                    else if (io_data[7]) state<=S_X_RESULT;
                    else state<=S_X_POLL;
                end
                S_X_RESULT: read_reg(R_HRSL,S_X_DECIDE);    
                S_X_DECIDE: begin
                    if(!first_in_locked && enum_step==0)begin
                        if(x_token==8'h10)first_setup_hrsl<=io_data[7:0];
                        if(x_token==8'h00)first_post_hrsl<=io_data[7:0];
                    end
                    x_hrsl<=io_data[7:0]; x_result<=io_data[3:0];
                    last_hrsl<=io_data[3:0];
                    x_retry<=0;
                    if ((io_data[3:0]==4) && !usb_ready) x_retry<=1;
                    else if ((io_data[3:0]!=0) && (io_data[3:0]!=4) &&
                             (io_data[3:0]!=5) && (x_attempt<2)) begin
                        x_attempt<=x_attempt+1'b1; x_retry<=1;
                    end
                    if(RCVBC_SCOPE_TEST==3 && enum_step==0 && x_token==8'h00 &&
                        x_receive && io_data[3:0]==0)begin
                        scope_active<=1;x_active<=0;init_ms<=0;usb_ready<=0;
                        release_hrsl<=io_data[7:0];release_phase<=1;
                        release_config_index<=0;state<=S_RELEASE_CONFIG;
                    end else if(RCVBC_SCOPE_TEST==2 && enum_step==0 && x_token==8'h00 &&
                        x_receive && io_data[3:0]==0)begin
                        // Bypass OP_RECEIVE and all acknowledgement writes.
                        scope_active<=1;x_active<=0;init_ms<=0;usb_ready<=0;
                        diag_first_hrsl<=io_data[7:0];diag_first_hirq<=first_post_hirq;
                        diag_phase<=1;state<=S_DIAG_COUNT;
                    end else state<=S_X_ACK;
                end
                S_X_ACK: begin
                    // Drain a successful IN before acknowledging HXFRDN,
                    // matching the reference driver's RCVDAV-first ordering.
                    if(x_result==0 && x_receive && !x_retry)state<=S_X_POST;
                    else write_reg(R_HIRQ,8'h80,S_X_POST);
                end
                S_X_POST: begin
                    if (x_retry) pause_ms(1,S_X_CLEAR);
                    else if ((x_result==0) && x_receive) state<=S_X_RECEIVE;
                    else state<=S_X_RETURN;
                end
                S_X_RECEIVE: read_reg(5'd6,S_RX_COUNT_SAVE);
                S_RX_COUNT_SAVE: begin
                    if(!first_in_locked && enum_step==0 && x_token==8'h00)begin
                        first_post_count<=io_data[7:0];first_in_locked<=1;
                    end
                    receive_count_before<=io_data[7:0];
                    read_reg(R_HIRQ,S_RX_IRQ_SAVE);
                end
                S_RX_IRQ_SAVE: begin
                    receive_irq_before<=io_data[7:0];
                    // The optional scope mode freezes this pending packet.
                    // Full mode consumes only the count reported by the MAX.
                    if(RCVBC_SCOPE_TEST==1 && enum_step==0)begin
                        scope_active<=1;x_active<=0;init_ms<=0;
                        state<=S_RCVBC_SCOPE_READ;
                    end else if(RCVBC_SCOPE_TEST==4 && enum_step==0 &&
                        x_token==8'h00 && ctrl_received==0)begin
                        a5_before_count<=receive_count_before;a5_before_irq<=io_data[7:0];
                        if(!io_data[2])fail(8'h50);
                        else if(receive_count_before==0)begin
                            if(a5_recovery_used)fail(8'h51);
                            else begin
                                a5_recovery_used<=1;a5_phase<=2;a5_recovery_phase<=1;
                                state<=S_A5_RELEASE;
                            end
                        end else issue_io(4,0,0,512'd0,S_X_RECEIVED);
                    end else if(RCVBC_SCOPE_TEST==4 && x_token==8'h00 && enum_step<9 &&
                        receive_count_before==0 && io_data[2])begin
                        // Do not apply a descriptor-specific workaround to later packets.
                        a5_later_zero<=1;fail(8'h54);
                    end else issue_io(4,0,0,512'd0,S_X_RECEIVED);
                end
                S_RCVBC_SCOPE_READ: begin
                    init_ms<=0;
                    read_reg(5'd6,S_RCVBC_SCOPE_UPDATE);
                end
                S_RCVBC_SCOPE_UPDATE: begin
                    scope_count<=io_data[7:0];init_ms<=0;
                    receive_diagnostic<={x_token,x_hrsl,receive_irq_before,io_data[7:0],
                        ctrl_length[7:0],ctrl_received[7:0],{1'b0,io_len},8'd0};
                    pause_ms(100,S_RCVBC_SCOPE_READ);
                end
                // A4 keeps the first IN frozen: no new tokens or watchdog retries.
                S_RELEASE_CONFIG: begin
                    case(release_config_index)
                        0:read_reg(R_MODE,S_RELEASE_CONFIG_SAVE);
                        1:read_reg(R_PINCTL,S_RELEASE_CONFIG_SAVE);
                        2:read_reg(R_USBCTL,S_RELEASE_CONFIG_SAVE);
                        3:read_reg(R_PERADDR,S_RELEASE_CONFIG_SAVE);
                    endcase
                end
                S_RELEASE_CONFIG_SAVE: begin
                    case(release_config_index)
                        0:release_mode<=io_data[7:0];
                        1:release_pinctl<=io_data[7:0];
                        2:release_usbctl<=io_data[7:0];
                        3:release_address<=io_data[7:0];
                    endcase
                    if(release_config_index==3)begin
                        if(RCVBC_SCOPE_TEST==4)state<=S_C_SETUP;
                        else state<=S_RELEASE_COUNT0;
                    end
                    else begin release_config_index<=release_config_index+1'b1;state<=S_RELEASE_CONFIG;end
                end
                S_RELEASE_COUNT0: read_reg(5'd6,S_RELEASE_IRQ0);
                S_RELEASE_IRQ0: begin
                    release_count0<=io_data[7:0];
                    first_post_count<=io_data[7:0];first_in_locked<=1;
                    read_reg(R_HIRQ,S_RELEASE_DECIDE0);
                end
                S_RELEASE_DECIDE0: begin
                    release_irq0<=io_data[7:0];
                    if(!io_data[2])begin release_phase<=8'he1;release_done<=1;state<=S_DIAG_HOLD;end
                    else if(release_count0!=0 && release_count0!=18)begin
                        release_phase<=8'he2;release_done<=1;state<=S_DIAG_HOLD;
                    end else begin
                        release_phase<=2;issue_io(2,5'd1,7'd18,512'd0,S_RELEASE_FIFO0);
                    end
                end
                S_RELEASE_FIFO0: begin
                    release_fifo0<=io_data[143:0];release_read0<=1;
                    read_reg(5'd6,S_RELEASE_FIFO0_COUNT);
                end
                S_RELEASE_FIFO0_COUNT: begin
                    release_fifo0_count<=io_data[7:0];read_reg(R_HIRQ,S_RELEASE_FIFO0_IRQ);
                end
                S_RELEASE_FIFO0_IRQ: begin
                    release_fifo0_irq<=io_data[7:0];
                    if(!io_data[2])begin release_phase<=8'he1;release_done<=1;state<=S_DIAG_HOLD;end
                    else begin release_phase<=3;state<=S_RELEASE_CLEAR;end
                end
                S_RELEASE_CLEAR: begin
                    // Programming Guide pp.15,47,48: unload THEN release.
                    // Deliberate diagnostic discard if the observed count was zero.
                    release_writes<=release_writes+1'b1;
                    write_reg(R_HIRQ,8'h04,S_RELEASE_COUNT1);
                end
                S_RELEASE_COUNT1: read_reg(5'd6,S_RELEASE_IRQ1);
                S_RELEASE_IRQ1: begin
                    release_count1<=io_data[7:0];read_reg(R_HIRQ,S_RELEASE_WAIT);
                end
                S_RELEASE_WAIT: begin
                    release_irq1<=io_data[7:0];release_phase<=4;
                    pause_ms(10,S_RELEASE_COUNT2);
                end
                S_RELEASE_COUNT2: read_reg(5'd6,S_RELEASE_IRQ2);
                S_RELEASE_IRQ2: begin
                    release_count2<=io_data[7:0];read_reg(R_HIRQ,S_RELEASE_DECIDE2);
                end
                S_RELEASE_DECIDE2: begin
                    release_irq2<=io_data[7:0];
                    if(!io_data[2])begin
                        release_final_count<=release_count2;release_final_irq<=io_data[7:0];
                        state<=S_RELEASE_DONE;
                    end else if(release_count2!=0 && release_count2!=18)begin
                        release_phase<=8'he3;release_done<=1;state<=S_DIAG_HOLD;
                    end else issue_io(2,5'd1,7'd18,512'd0,S_RELEASE_FIFO1);
                end
                S_RELEASE_FIFO1: begin
                    release_fifo1<=io_data[143:0];release_read1<=1;
                    read_reg(5'd6,S_RELEASE_FINAL_COUNT);
                end
                S_RELEASE_FINAL_COUNT: begin
                    release_final_count<=io_data[7:0];read_reg(R_HIRQ,S_RELEASE_FINAL_IRQ);
                end
                S_RELEASE_FINAL_IRQ: begin release_final_irq<=io_data[7:0];state<=S_RELEASE_DONE;end
                S_RELEASE_DONE: begin release_done<=1;release_phase<=5;state<=S_DIAG_HOLD;end
                // A5 deliberately omits A4's blind FIFO read. It tests whether
                // releasing a zero-count pending buffer alone exposes the descriptor.
                S_A5_RELEASE: write_reg(R_HIRQ,8'h04,S_A5_WAIT);
                S_A5_WAIT: begin
                    a5_phase<=3;a5_recovery_phase<=2;
                    pause_ms(10,S_A5_IRQ);
                end
                S_A5_IRQ: read_reg(R_HIRQ,S_A5_IRQ_CHECK);
                S_A5_IRQ_CHECK: begin
                    a5_after_irq<=io_data[7:0];a5_recovery_phase<=3;
                    if(!io_data[2])begin a5_recovery_phase<=8'he1;fail(8'h50);end
                    else read_reg(5'd6,S_A5_COUNT_CHECK);
                end
                S_A5_COUNT_CHECK: begin
                    a5_after_count<=io_data[7:0];a5_recovery_phase<=4;
                    if(io_data[7:0]==0)begin a5_recovery_phase<=8'he2;fail(8'h51);end
                    else if(io_data[7:0]!=18)begin a5_recovery_phase<=8'he3;fail(8'h52);end
                    else begin
                        a5_recovery_phase<=5;
                        // Length is from the actual register read, not a replacement constant.
                        issue_io(2,5'd1,io_data[6:0],512'd0,S_A5_FIFO_CHECK);
                    end
                end
                S_A5_FIFO_CHECK: begin
                    a5_descriptor<=io_data[143:0];a5_first_len<={1'b0,io_len};
                    if(io_len!=18 || io_data[143:0]!=EXPECTED_DEVICE_DESCRIPTOR)begin
                        a5_recovery_phase<=8'he4;fail(8'h53);
                    end else begin
                        a5_recovery_success<=1;a5_descriptor_valid<=1;
                        a5_phase<=4;a5_recovery_phase<=6;
                        x_data<=io_data;x_len<=io_len;
                        write_reg(R_HIRQ,8'h04,S_A5_ACK_DONE);
                    end
                end
                S_A5_ACK_DONE: write_reg(R_HIRQ,8'h80,S_X_RETURN);
                S_DIAG_COUNT: read_reg(5'd6,S_DIAG_COUNT_SAVE);
                S_DIAG_COUNT_SAVE: begin
                    diag_counts[(7-diag_samples[2:0])*8 +: 8]<=io_data[7:0];
                    diag_last_count<=io_data[7:0];scope_count<=io_data[7:0];
                    if(diag_samples==0)begin
                        diag_first_count<=io_data[7:0];
                        first_post_count<=io_data[7:0];first_in_locked<=1;
                    end
                    if(io_data[7:0]!=0)diag_all_zero<=0;
                    if(io_data[7:0]==18)diag_any18<=1;
                    read_reg(R_HIRQ,S_DIAG_IRQ_SAVE);
                end
                S_DIAG_IRQ_SAVE: begin
                    diag_irqs[(7-diag_samples[2:0])*8 +: 8]<=io_data[7:0];
                    if(!io_data[2])diag_all_pending<=0;
                    diag_samples<=diag_samples+1'b1;
                    if(diag_samples==7)begin
                        diag_phase<=2;read_reg(5'd6,S_DIAG_PRE_COUNT);
                    end else if(diag_wait_ms(diag_samples[2:0])==0)state<=S_DIAG_COUNT;
                    else pause_ms(diag_wait_ms(diag_samples[2:0]),S_DIAG_COUNT);
                end
                S_DIAG_PRE_COUNT: begin
                    diag_pre_count<=io_data[7:0];read_reg(R_HIRQ,S_DIAG_PRE_IRQ);
                end
                S_DIAG_PRE_IRQ: begin
                    diag_pre_hirq<=io_data[7:0];
                    if(!io_data[2])begin
                        // Never read a FIFO with no pending receive indication.
                        diag_phase<=8'he1;diag_done<=1;state<=S_DIAG_HOLD;
                    end else begin
                        diag_phase<=3;
                        // Diagnostic-only 18-byte observation after ALL count samples.
                        // This is not a reported length and is never passed to HID/enum.
                        issue_io(2,5'd1,7'd18,512'd0,S_DIAG_FIFO_SAVE);
                    end
                end
                S_DIAG_FIFO_SAVE: begin
                    diag_fifo<=io_data[143:0];diag_fifo_read<=1;diag_phase<=4;
                    read_reg(5'd6,S_DIAG_POST_COUNT);
                end
                S_DIAG_POST_COUNT: begin
                    diag_post_count<=io_data[7:0];read_reg(R_HIRQ,S_DIAG_POST_IRQ);
                end
                S_DIAG_POST_IRQ: begin
                    diag_post_hirq<=io_data[7:0];diag_phase<=5;diag_done<=1;
                    state<=S_DIAG_HOLD;
                end
                S_DIAG_HOLD: begin
                    // Freeze until external reset. SIE-generated SOF can continue.
                    init_ms<=0;x_active<=0;usb_ready<=0;
                end
                S_X_RECEIVED: begin
                    if (!io_has_packet) fail(8'h40);
                    else if(RCVBC_SCOPE_TEST==4 && enum_step==0 && x_token==8'h00)begin
                        a5_descriptor<=io_data[143:0];a5_first_len<={1'b0,io_len};
                        if(io_len!=18 || io_data[143:0]!=EXPECTED_DEVICE_DESCRIPTOR)fail(8'h53);
                        else begin
                            a5_descriptor_valid<=1;a5_phase<=4;
                            x_data<=io_data;x_len<=io_len;
                            write_reg(R_HIRQ,8'h80,S_X_RETURN);
                        end
                    end
                    else begin
                        x_data<=io_data;x_len<=io_len;
                        write_reg(R_HIRQ,8'h80,S_X_RETURN);
                    end
                end
                S_X_RETURN: begin x_active<=0; state<=x_return; end
                default: fail(8'hff);
            endcase
        end
    end
endmodule
