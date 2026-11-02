`timescale 1ns / 1ps
// ICM-42688-P raw six-axis reader. No floating point, scaling or bias removal.
// Datasheet: TDK DS-000347 rev 1.6. Four-wire SPI, mode 0, MSB first.
// All outputs belong to clk. rst_n may assert asynchronously; synchronize its
// release in the enclosing clk domain. No connection to INT1/INT2 is required.
module icm42688 #(
    parameter integer CLK_HZ = 27000000,
    parameter integer SPI_HZ = 2250000, 
    parameter integer POWERUP_WAIT_US = 10000,
    parameter integer RESET_WAIT_US = 2000,
    parameter integer SENSOR_STARTUP_US = 50000,
    parameter integer POLL_INTERVAL_US = 100,
    parameter integer DATA_TIMEOUT_US = 100000
)(
    input  wire clk,
    input  wire rst_n,
    input  wire spi_miso,
    output wire spi_sclk,
    output wire spi_mosi,
    output wire spi_cs_n,
    output reg signed [15:0] gyro_x_raw,
    output reg signed [15:0] gyro_y_raw,
    output reg signed [15:0] gyro_z_raw,
    output reg signed [15:0] accel_x_raw,
    output reg signed [15:0] accel_y_raw,
    output reg signed [15:0] accel_z_raw,
    output reg data_valid,
    output reg init_done,
    output wire error,
    output reg [3:0] error_code,
    output reg [7:0] who_am_i,
    output reg [7:0] last_int_status,
    output reg [31:0] sample_count,
    // Host observation time in clk cycles, modulo 2^32; NOT sensor time.
    // Latched when DATA_RDY is observed, and committed with the six outputs.
    output reg [31:0] sample_timestamp
);
    // Avoid CLK_HZ * microseconds in a 32-bit constant expression.
    localparam integer CYCLES_PER_US = (CLK_HZ + 999999) / 1000000;
    localparam integer POWERUP_CYCLES = CYCLES_PER_US * POWERUP_WAIT_US;
    localparam integer RESET_CYCLES = CYCLES_PER_US * RESET_WAIT_US;
    localparam integer STARTUP_CYCLES = CYCLES_PER_US * SENSOR_STARTUP_US;
    localparam integer POLL_CYCLES = CYCLES_PER_US * POLL_INTERVAL_US;
    localparam integer WHO_RETRY_CYCLES = CYCLES_PER_US * 1000; // 1 ms
    localparam integer DATA_TIMEOUT_CYCLES = CYCLES_PER_US * DATA_TIMEOUT_US;
    localparam integer SPI_HALF_CYCLES = (CLK_HZ + 2*SPI_HZ-1)/(2*SPI_HZ);
    // Longer than a maximum 12-byte read including CS guards and handshakes.
    localparam integer XFER_TIMEOUT_CYCLES = 256*SPI_HALF_CYCLES + 256;

    localparam [4:0] S_POWERUP=0, S_BANK_RESET=1, S_RESET=2,
        S_RESET_WAIT=3, S_BANK_ZERO=4, S_WHO=5, S_WHO_CHECK=6,
        S_RESET_STATUS=7, S_RESET_CHECK=8, S_CONFIG_WRITE=9,
        S_CONFIG_PAUSE=10, S_CONFIG_READ=11, S_CONFIG_CHECK=12,
        S_CLEAR_STATUS=13, S_READY=14, S_STATUS=15, S_STATUS_CHECK=16,
        S_READ_DATA=17, S_COMMIT=18, S_ISSUE=19, S_TRANSFER=20, S_ERROR=21, S_WHO_RETRY=22;
    reg [4:0] state, return_state;
    reg [3:0] config_index;
    reg [31:0] wait_counter, transfer_counter, no_data_counter;
    reg [31:0] clock_counter, pending_timestamp;
    reg req_read;
    reg [6:0] req_address;
    reg [3:0] req_length;
    reg [7:0] req_write_data;
    wire req_ready, transfer_done;
    wire [95:0] read_data;
    assign error = (error_code != 0);

    // Every entry is written and read back. Other filter/AAF registers retain
    // their documented reset values; this is not the car's custom AAF setup.
    function [6:0] config_address;
        input [3:0] index;
        begin
            case (index)
                0: config_address=7'h4c; // big-endian sensor words
                1: config_address=7'h4f; // gyro +/-1000 dps, 1 kHz
                2: config_address=7'h50; // accel +/-4 g, 1 kHz
                3: config_address=7'h52; // both UI filter bandwidths ODR/4
                4: config_address=7'h63; // DATA_RDY clears on status read
                5: config_address=7'h64; // clear INT_ASYNC_RESET
                6: config_address=7'h65; // enable DATA_RDY source on INT1
                default: config_address=7'h4e; // both sensors low-noise
            endcase
        end
    endfunction
    function [7:0] config_value;
        input [3:0] index;
        begin
            case (index)
                0: config_value=8'h30;
                1: config_value=8'h26;
                2: config_value=8'h46;
                3: config_value=8'h11;
                4: config_value=8'h00;
                5: config_value=8'h00;
                6: config_value=8'h08;
                default: config_value=8'h0f;
            endcase
        end
    endfunction

    // Launch one complete transaction, then resume at next_state.
    task launch;
        input do_read;
        input [6:0] address;
        input [3:0] length;
        input [7:0] write_data;
        input [4:0] next_state;
        begin
            req_read<=do_read; req_address<=address;
            req_length<=length; req_write_data<=write_data;
            return_state<=next_state; transfer_counter<=0; state<=S_ISSUE;
        end
    endtask
    task fail;
        input [3:0] code;
        begin
            error_code<=code; init_done<=0; state<=S_ERROR;
        end
    endtask

    icm42688_spi_master #(.CLK_HZ(CLK_HZ), .SPI_HZ(SPI_HZ)) u_spi (
        .clk(clk), .rst_n(rst_n), .req_valid(state==S_ISSUE),
        .req_ready(req_ready), .req_read(req_read),
        .req_address(req_address), .req_length(req_length),
        .req_write_data(req_write_data), .done(transfer_done),
        .read_data(read_data), .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi), .spi_cs_n(spi_cs_n), .spi_miso(spi_miso)
    );

    // synthesis translate_off
    initial begin
        if (CLK_HZ < 1000000 || SPI_HZ <= 0 || SPI_HZ > 24000000 ||
            CLK_HZ < 2*SPI_HZ || POWERUP_WAIT_US < 1000 ||
            RESET_WAIT_US < 1000 || SENSOR_STARTUP_US < 45000 ||
            POLL_INTERVAL_US < 1 || DATA_TIMEOUT_US < 1000) begin
            $display("ERROR: invalid ICM42688 parameters");
            $finish;
        end
    end
    // synthesis translate_on

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=S_POWERUP; return_state<=S_ERROR; config_index<=0;
            wait_counter<=0; transfer_counter<=0; no_data_counter<=0;
            clock_counter<=0; pending_timestamp<=0;
            req_read<=0; req_address<=0; req_length<=1; req_write_data<=0;
            gyro_x_raw<=0; gyro_y_raw<=0; gyro_z_raw<=0;
            accel_x_raw<=0; accel_y_raw<=0; accel_z_raw<=0;
            data_valid<=0; init_done<=0; error_code<=0;
            who_am_i<=0; last_int_status<=0;
            sample_count<=0; sample_timestamp<=0;
        end else begin
            clock_counter<=clock_counter+1'b1;
            data_valid<=0;
            if (init_done && no_data_counter < DATA_TIMEOUT_CYCLES)
                no_data_counter<=no_data_counter+1'b1;
            case (state)
                S_POWERUP: begin
                    if (wait_counter==POWERUP_CYCLES-1) begin
                        wait_counter<=0; state<=S_BANK_RESET;
                    end else wait_counter<=wait_counter+1'b1;
                end
                S_BANK_RESET: launch(0,7'h76,1,8'h00,S_RESET);
                S_RESET: begin
                    wait_counter<=0;
                    launch(0,7'h11,1,8'h01,S_RESET_WAIT);
                end
                S_RESET_WAIT: begin
                    if (wait_counter==RESET_CYCLES-1) begin
                        wait_counter<=0; state<=S_BANK_ZERO;
                    end else wait_counter<=wait_counter+1'b1;
                end
                S_BANK_ZERO: launch(0,7'h76,1,8'h00,S_WHO);
                S_WHO: launch(1,7'h75,1,0,S_WHO_CHECK);
                S_WHO_CHECK: begin
                    who_am_i<=read_data[7:0];
                    if (read_data[7:0]!=8'h47) begin
                        // Keep showing the last bad ID, but never latch/stop here.
                        error_code<=4'd1; init_done<=0;
                        wait_counter<=0; state<=S_WHO_RETRY;
                    end else begin
                        error_code<=0; state<=S_RESET_STATUS;
                    end
                end
                S_WHO_RETRY: begin
                    // Leave CS high between complete WHO_AM_I transactions.
                    // No retry limit, configuration writes or repeated reset.
                    if (wait_counter==WHO_RETRY_CYCLES-1) begin
                        wait_counter<=0; state<=S_WHO;
                    end else wait_counter<=wait_counter+1'b1;
                end
                S_RESET_STATUS: launch(1,7'h2d,1,0,S_RESET_CHECK);
                S_RESET_CHECK: begin
                    last_int_status<=read_data[7:0];
                    if (!read_data[4]) fail(4'd2);
                    else begin config_index<=0; state<=S_CONFIG_WRITE; end
                end
                S_CONFIG_WRITE: begin
                    wait_counter<=0;
                    launch(0,config_address(config_index),1,
                           config_value(config_index),S_CONFIG_PAUSE);
                end
                S_CONFIG_PAUSE: begin
                    // After OFF->LN there are NO bus transactions for 50 ms.
                    // Also covers the 200 us no-write restriction and startup.
                    if (config_index!=7) state<=S_CONFIG_READ;
                    else if (wait_counter==STARTUP_CYCLES-1) begin
                        wait_counter<=0; state<=S_CONFIG_READ;
                    end else wait_counter<=wait_counter+1'b1;
                end
                S_CONFIG_READ: launch(1,config_address(config_index),1,0,S_CONFIG_CHECK);
                S_CONFIG_CHECK: begin
                    if (read_data[7:0]!=config_value(config_index)) fail(4'd3);
                    else if (config_index==7) state<=S_CLEAR_STATUS;
                    else begin config_index<=config_index+1'b1; state<=S_CONFIG_WRITE; end
                end
                S_CLEAR_STATUS: launch(1,7'h2d,1,0,S_READY);
                S_READY: begin
                    // Discard accumulated startup DATA_RDY before publishing.
                    if (!init_done) begin
                        init_done<=1; no_data_counter<=0; wait_counter<=0;
                    end else if (wait_counter==POLL_CYCLES-1) begin
                        wait_counter<=0; state<=S_STATUS;
                    end else wait_counter<=wait_counter+1'b1;
                end
                S_STATUS: launch(1,7'h2d,1,0,S_STATUS_CHECK);
                S_STATUS_CHECK: begin
                    last_int_status<=read_data[7:0];
                    // Bit 7 is reserved/zero. Detect a disconnected, pulled-up
                    // MISO instead of publishing six 0xffff words as a sample.
                    if (read_data[7]) fail(4'd5);
                    else if (read_data[4]) fail(4'd2); // unexpected sensor reset
                    else if (read_data[3]) begin
                        pending_timestamp<=clock_counter; state<=S_READ_DATA;
                    end else begin wait_counter<=0; state<=S_READY; end
                end
                S_READ_DATA: launch(1,7'h1f,12,0,S_COMMIT);
                S_COMMIT: begin
                    // read_data byte 0 is [7:0]; sensor words are MSB first.
                    accel_x_raw<={read_data[7:0],read_data[15:8]};
                    accel_y_raw<={read_data[23:16],read_data[31:24]};
                    accel_z_raw<={read_data[39:32],read_data[47:40]};
                    gyro_x_raw<={read_data[55:48],read_data[63:56]};
                    gyro_y_raw<={read_data[71:64],read_data[79:72]};
                    gyro_z_raw<={read_data[87:80],read_data[95:88]};
                    sample_timestamp<=pending_timestamp;
                    sample_count<=sample_count+1'b1;
                    data_valid<=1; no_data_counter<=0; wait_counter<=0;
                    state<=S_READY;
                end
                S_ISSUE: begin
                    if (req_ready) begin transfer_counter<=0; state<=S_TRANSFER; end
                    else if (transfer_counter==XFER_TIMEOUT_CYCLES-1) fail(4'd6);
                    else transfer_counter<=transfer_counter+1'b1;
                end
                S_TRANSFER: begin
                    if (transfer_done) state<=return_state;
                    else if (transfer_counter==XFER_TIMEOUT_CYCLES-1) fail(4'd6);
                    else transfer_counter<=transfer_counter+1'b1;
                end
                S_ERROR: begin init_done<=0; end // reset rst_n to retry
                default: fail(4'd6);
            endcase
            // Wall-clock watchdog includes status polls and burst transfers.
            if (init_done && no_data_counter>=DATA_TIMEOUT_CYCLES && state!=S_COMMIT)
                fail(4'd4);
        end
    end
endmodule

// Dedicated bounded transaction engine: mode 0, CS held for command+payload.
// Write length is one byte; read length is 1..12 bytes. Internal use only.
module icm42688_spi_master #(
    parameter integer CLK_HZ=27000000,
    parameter integer SPI_HZ=2250000
)(
    input wire clk, rst_n,
    input wire req_valid,
    output wire req_ready,
    input wire req_read,
    input wire [6:0] req_address,
    input wire [3:0] req_length,
    input wire [7:0] req_write_data,
    output reg done,
    output reg [95:0] read_data,
    output reg spi_sclk, spi_mosi, spi_cs_n,
    input wire spi_miso
);
    localparam integer HALF_CYCLES=(CLK_HZ+2*SPI_HZ-1)/(2*SPI_HZ);
    // >=100 ns each for CS setup, hold and inter-transaction high time.
    localparam integer GUARD_CYCLES=(CLK_HZ+9999999)/10000000;
    localparam [2:0] IDLE=0, SETUP=1, SHIFT=2, HOLD=3, GAP=4;
    reg [2:0] state, bit_index;
    reg [3:0] byte_index, length_latched;
    reg read_latched;
    reg [7:0] tx_shift, rx_shift, write_latched;
    reg [31:0] counter;
    assign req_ready=(state==IDLE);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=GAP; bit_index<=7; byte_index<=0; length_latched<=1;
            read_latched<=0; tx_shift<=0; rx_shift<=0; write_latched<=0;
            counter<=0; done<=0; read_data<=0;
            spi_sclk<=0; spi_mosi<=0; spi_cs_n<=1;
        end else begin
            done<=0;
            case(state)
                IDLE: if(req_valid) begin
                    read_latched<=req_read;
                    length_latched<=req_read ? req_length : 4'd1;
                    write_latched<=req_write_data;
                    tx_shift<={req_read,req_address};
                    spi_mosi<=req_read; spi_cs_n<=0; spi_sclk<=0;
                    bit_index<=7; byte_index<=0; rx_shift<=0;
                    read_data<=0; counter<=0; state<=SETUP;
                end
                SETUP: begin
                    if(counter==GUARD_CYCLES-1) begin counter<=0; state<=SHIFT; end
                    else counter<=counter+1'b1;
                end
                SHIFT: begin
                    if(counter==HALF_CYCLES-1) begin
                        counter<=0;
                        if(!spi_sclk) begin
                            // Sample at SCLK rising edge; data was launched by
                            // the sensor on the previous SCLK falling edge.
                            spi_sclk<=1;
                            rx_shift<={rx_shift[6:0],spi_miso};
                            if(bit_index==0 && byte_index!=0 && read_latched)
                                read_data[(byte_index-1)*8 +: 8]<={rx_shift[6:0],spi_miso};
                        end else begin
                            spi_sclk<=0;
                            if(bit_index==0) begin
                                if(byte_index==length_latched) state<=HOLD;
                                else begin
                                    byte_index<=byte_index+1'b1; bit_index<=7; rx_shift<=0;
                                    tx_shift<=read_latched ? 8'h00 : write_latched;
                                    spi_mosi<=read_latched ? 1'b0 : write_latched[7];
                                end
                            end else begin
                                bit_index<=bit_index-1'b1;
                                tx_shift<={tx_shift[6:0],1'b0}; spi_mosi<=tx_shift[6];
                            end
                        end
                    end else counter<=counter+1'b1;
                end
                HOLD: begin
                    if(counter==GUARD_CYCLES-1) begin
                        spi_cs_n<=1; spi_mosi<=0; counter<=0; state<=GAP;
                    end else counter<=counter+1'b1;
                end
                GAP: begin
                    if(counter==GUARD_CYCLES-1) begin counter<=0; done<=1; state<=IDLE; end
                    else counter<=counter+1'b1;
                end
                default: begin spi_cs_n<=1; spi_sclk<=0; counter<=0; state<=GAP; end
            endcase
        end
    end
endmodule

// Continuous scope patterns. CS stays inactive, so no IMU register is written.
// At 27 MHz: 500 us per symbol, SCLK 1 kHz, MOSI 011 repeating at 2 kbit/s.
module icm42688_pin_test #(
    parameter integer CLK_HZ=27000000,
    parameter integer BIT_HZ=2000
)(
    input wire clk, rst_n,
    output reg sclk,
    output wire mosi, cs_n
);
    localparam integer DIV_RAW=CLK_HZ/BIT_HZ;
    localparam integer DIV_CYCLES=(DIV_RAW<1)?1:DIV_RAW;
    reg [31:0] tick_count;
    reg [1:0] phase;
    assign cs_n=1'b1;
    assign mosi=(phase!=0);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            tick_count<=0;
            phase<=0;
            sclk<=0;
        end else if(tick_count==DIV_CYCLES-1) begin
            tick_count<=0;
            sclk<=~sclk;
            if(phase==2) phase<=0;
            else phase<=phase+1'b1;
        end else tick_count<=tick_count+1'b1;
    end
endmodule

// Attitude implementation kept in this registered source for PDS project compatibility.
`timescale 1ns/1ps
// Integer-only shared math unit: atan2, sin/cos and unsigned integer sqrt.
// Angles: signed degrees Q16. Trigonometric results: signed Q30.
module imu_attitude_math (
    input wire clk,rst_n,start,
    input wire [1:0] operation, // 0 atan2(y,x), 1 sincos(angle), 2 sqrt(x)
    input wire signed [63:0] x_in,y_in,
    input wire signed [31:0] angle_in,
    output reg busy,done,
    output reg signed [63:0] x_out,y_out,
    output reg signed [31:0] angle_out
);
    reg [1:0] mode;
    reg [5:0] iteration;
    reg signed [63:0] x,y;
    reg signed [31:0] z;
    reg [63:0] radicand;
    reg [31:0] root;
    wire [31:0] candidate=root | (32'h80000000 >> iteration);
    wire [63:0] candidate_square={32'b0,candidate}*{32'b0,candidate};
    function signed [31:0] atan_table;
        input [5:0] index;
        begin
            case(index)
                0:atan_table=2949120; 1:atan_table=1740967;
                2:atan_table=919879; 3:atan_table=466945;
                4:atan_table=234379; 5:atan_table=117304;
                6:atan_table=58666; 7:atan_table=29335;
                8:atan_table=14668; 9:atan_table=7334;
                10:atan_table=3667; 11:atan_table=1833;
                12:atan_table=917; 13:atan_table=458;
                14:atan_table=229; 15:atan_table=115;
                16:atan_table=57; 17:atan_table=29;
                18:atan_table=14; 19:atan_table=7;
                20:atan_table=4; 21:atan_table=2;
                22:atan_table=1; default:atan_table=0;
            endcase
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            busy<=0;done<=0;mode<=0;iteration<=0;x<=0;y<=0;z<=0;
            radicand<=0;root<=0;x_out<=0;y_out<=0;angle_out<=0;
        end else begin
            done<=0;
            if(start && !busy) begin
                busy<=1;mode<=operation;iteration<=0;
                if(operation==2) begin radicand<=x_in;root<=0;end
                else if(operation==1 && (angle_in==0 || angle_in==5898240 || angle_in== -5898240)) begin
                    // Exact axes avoid injecting a tiny fictitious cross-axis rate.
                    x_out<=(angle_in==0)?64'sd1073741824:64'sd0;
                    y_out<=(angle_in==0)?64'sd0:((angle_in>0)?64'sd1073741824:-64'sd1073741824);
                    angle_out<=angle_in;busy<=0;done<=1;
                end else if(operation==1) begin
                    y<=0;
                    if(angle_in>5898240) begin x<=-64'sd652032874;z<=angle_in-11796480;end
                    else if(angle_in< -5898240) begin x<=-64'sd652032874;z<=angle_in+11796480;end
                    else begin x<=64'sd652032874;z<=angle_in;end
                end else begin
                    // Scale small integer vectors before vectoring iterations.
                    if(x_in<0) begin
                        x<=-(x_in<<<16);y<=-(y_in<<<16);
                        z<=(y_in>=0)?32'sd11796480:-32'sd11796480;
                    end else begin x<=x_in<<<16;y<=y_in<<<16;z<=0;end
                end
            end else if(busy) begin
                if(mode==2) begin
                    if(candidate_square<=radicand) root<=candidate;
                    if(iteration==31) begin
                        x_out<=(candidate_square<=radicand)?{32'b0,candidate}:{32'b0,root};
                        y_out<=0;angle_out<=0;busy<=0;done<=1;
                    end else iteration<=iteration+1'b1;
                end else if(iteration==23) begin
                    x_out<=x;y_out<=y;angle_out<=z;busy<=0;done<=1;
                end else begin
                    if(mode==0) begin
                        if(y>0) begin x<=x+(y>>>iteration);y<=y-(x>>>iteration);z<=z+atan_table(iteration);end
                        else if(y<0) begin x<=x-(y>>>iteration);y<=y+(x>>>iteration);z<=z-atan_table(iteration);end
                    end else begin
                        if(z>=0) begin x<=x-(y>>>iteration);y<=y+(x>>>iteration);z<=z-atan_table(iteration);end
                        else begin x<=x+(y>>>iteration);y<=y-(x>>>iteration);z<=z+atan_table(iteration);end
                    end
                    iteration<=iteration+1'b1;
                end
            end
        end
    end
endmodule

// Startup gravity alignment + stationary gyro bias, then gyro-only quaternion.
// No Kalman/PI correction. The three atan2 output definitions match gryo_my.c.
module icm42688_attitude #(
    parameter integer CLK_HZ=27000000,
    parameter integer SETTLE_SAMPLES=256,
    parameter integer CALIB_LOG2=10,
    parameter integer MAX_GYRO_SPREAD=128,
    parameter integer MAX_ACCEL_SPREAD=512
)(
    input wire clk,rst_n,data_valid,
    input wire [31:0] sample_timestamp,
    input wire signed [15:0] gyro_x_raw,gyro_y_raw,gyro_z_raw,
    input wire signed [15:0] accel_x_raw,accel_y_raw,accel_z_raw,
    output reg ready,angle_valid,
    output wire busy,
    output reg timing_error,
    output reg [15:0] calibration_count,
    output reg signed [31:0] gyro_bias_x_q8,gyro_bias_y_q8,gyro_bias_z_q8,
    output reg signed [31:0] q0,q1,q2,q3,
    output reg signed [31:0] roll_q16,pitch_q16,yaw_q16,
    output reg signed [15:0] roll_cdeg,pitch_cdeg,yaw_cdeg
);
    localparam integer CALIB_SAMPLES=1<<CALIB_LOG2;
    // Matches C driver: raw * 1000 / 32768 degrees/s at +/-1000 dps.
    localparam signed [63:0] STEP_GAIN=(64'sd4797524517163+CLK_HZ/2)/CLK_HZ;
    localparam integer MAX_DT=CLK_HZ/50; // reject gaps >20 ms
    localparam [5:0] COLLECT=0,MEAN=1,CHECK=2,GRAV_SQRT=3,
        INIT_ROLL=4,INIT_PITCH=5,ROLL_SC=6,PITCH_SC=7,INIT_Q=8,
        IDLE=9,STEP=10,UPDATE_Q=11,NORM=12,NORM_APPLY=13,
        OUTPUT_ROLL=14,OUTPUT_PITCH=15,OUTPUT_YAW=16,PUBLISH=17,
        MATH_WAIT=18,STEP_SCALE=19;
    reg [5:0] state,math_return;
    reg [15:0] settle_count;
    reg signed [47:0] sum_gx,sum_gy,sum_gz,sum_ax,sum_ay,sum_az;
    reg signed [31:0] lo_gx,hi_gx,lo_gy,hi_gy,lo_gz,hi_gz;
    reg signed [31:0] lo_ax,hi_ax,lo_ay,hi_ay,lo_az,hi_az;
    reg signed [31:0] mean_ax,mean_ay,mean_az;
    reg signed [31:0] initial_roll,initial_pitch,cos_r,sin_r,cos_p,sin_p;
    reg signed [63:0] gravity_projection;
    reg signed [31:0] delta_x,delta_y,delta_z;
    reg signed [24:0] corr_gx,corr_gy,corr_gz;
    reg signed [45:0] rate_time_x,rate_time_y,rate_time_z;
    reg signed [63:0] normal_scale;
    reg [31:0] previous_stamp,dt_cycles;
    reg [1:0] normalize_pass;
    reg math_start;
    reg [1:0] math_op;
    reg signed [63:0] math_x,math_y;
    reg signed [31:0] math_angle;
    wire math_busy,math_done;
    wire signed [63:0] math_x_out,math_y_out;
    wire signed [31:0] math_angle_out;
    wire [31:0] elapsed=sample_timestamp-previous_stamp;
    assign busy=(state!=IDLE && state!=COLLECT);
    function signed [63:0] mul;
        input signed [31:0] a,b;
        begin mul=a*b;end
    endfunction
    function signed [31:0] bias_average;
        input signed [47:0] value;
        begin
            if(CALIB_LOG2>=8) bias_average=value>>>(CALIB_LOG2-8);
            else bias_average=value<<<(8-CALIB_LOG2);
        end
    endfunction
    function signed [15:0] centidegrees;
        input signed [31:0] value;
        reg signed [63:0] scaled;
        begin scaled=$signed(value)*64'sd100;if(scaled<0) centidegrees=-((-scaled)>>>16);
            else centidegrees=scaled>>>16;end
    endfunction
    wire signed [63:0] gravity_norm2=mul(mean_ax,mean_ax)+mul(mean_ay,mean_ay)+mul(mean_az,mean_az);
    wire signed [63:0] norm_q30=(mul(q0,q0)+mul(q1,q1)+mul(q2,q2)+mul(q3,q3))>>>30;
    wire signed [63:0] r32=(mul(q2,q3)+mul(q0,q1))>>>29;
    wire signed [63:0] r33=64'sd1073741824-((mul(q1,q1)+mul(q2,q2))>>>29);
    wire signed [63:0] minus_r31=(mul(q0,q2)-mul(q1,q3))>>>29;
    wire signed [63:0] r21=(mul(q1,q2)+mul(q0,q3))>>>29;
    wire signed [63:0] r11=64'sd1073741824-((mul(q2,q2)+mul(q3,q3))>>>29);
    imu_attitude_math u_math (
        .clk(clk),.rst_n(rst_n),.start(math_start),.operation(math_op),
        .x_in(math_x),.y_in(math_y),.angle_in(math_angle),
        .busy(math_busy),.done(math_done),.x_out(math_x_out),.y_out(math_y_out),.angle_out(math_angle_out)
    );
    task launch_math;
        input [1:0] op;
        input signed [63:0] xin,yin;
        input signed [31:0] ain;
        input [5:0] next_state;
        begin
            math_op<=op;math_x<=xin;math_y<=yin;math_angle<=ain;
            math_start<=1;math_return<=next_state;state<=MATH_WAIT;
        end
    endtask
    task clear_window;
        begin
            calibration_count<=0;
            sum_gx<=0;sum_gy<=0;sum_gz<=0;sum_ax<=0;sum_ay<=0;sum_az<=0;
            lo_gx<=32767;lo_gy<=32767;lo_gz<=32767;
            hi_gx<=-32768;hi_gy<=-32768;hi_gz<=-32768;
            lo_ax<=32767;lo_ay<=32767;lo_az<=32767;
            hi_ax<=-32768;hi_ay<=-32768;hi_az<=-32768;
        end
    endtask
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=COLLECT;math_return<=0;settle_count<=0;clear_window();
            ready<=0;angle_valid<=0;timing_error<=0;
            gyro_bias_x_q8<=0;gyro_bias_y_q8<=0;gyro_bias_z_q8<=0;
            q0<=1073741824;q1<=0;q2<=0;q3<=0;
            roll_q16<=0;pitch_q16<=0;yaw_q16<=0;roll_cdeg<=0;pitch_cdeg<=0;yaw_cdeg<=0;
            mean_ax<=0;mean_ay<=0;mean_az<=0;gravity_projection<=0;initial_roll<=0;initial_pitch<=0;
            cos_r<=0;sin_r<=0;cos_p<=0;sin_p<=0;
            delta_x<=0;delta_y<=0;delta_z<=0;corr_gx<=0;corr_gy<=0;corr_gz<=0;
            rate_time_x<=0;rate_time_y<=0;rate_time_z<=0;
            previous_stamp<=0;dt_cycles<=0;normal_scale<=0;normalize_pass<=0;
            math_start<=0;math_op<=0;math_x<=0;math_y<=0;math_angle<=0;
        end else begin
            math_start<=0;angle_valid<=0;
            // A pending sample must never silently overwrite computation.
            if(data_valid && state!=COLLECT && state!=IDLE) timing_error<=1;
            case(state)
                COLLECT: if(data_valid) begin
                    previous_stamp<=sample_timestamp;
                    if(settle_count<SETTLE_SAMPLES) settle_count<=settle_count+1'b1;
                    else begin
                        sum_gx<=sum_gx+gyro_x_raw;sum_gy<=sum_gy+gyro_y_raw;sum_gz<=sum_gz+gyro_z_raw;
                        sum_ax<=sum_ax+accel_x_raw;sum_ay<=sum_ay+accel_y_raw;sum_az<=sum_az+accel_z_raw;
                        if(gyro_x_raw<lo_gx) lo_gx<=gyro_x_raw;if(gyro_x_raw>hi_gx) hi_gx<=gyro_x_raw;
                        if(gyro_y_raw<lo_gy) lo_gy<=gyro_y_raw;if(gyro_y_raw>hi_gy) hi_gy<=gyro_y_raw;
                        if(gyro_z_raw<lo_gz) lo_gz<=gyro_z_raw;if(gyro_z_raw>hi_gz) hi_gz<=gyro_z_raw;
                        if(accel_x_raw<lo_ax) lo_ax<=accel_x_raw;if(accel_x_raw>hi_ax) hi_ax<=accel_x_raw;
                        if(accel_y_raw<lo_ay) lo_ay<=accel_y_raw;if(accel_y_raw>hi_ay) hi_ay<=accel_y_raw;
                        if(accel_z_raw<lo_az) lo_az<=accel_z_raw;if(accel_z_raw>hi_az) hi_az<=accel_z_raw;
                        calibration_count<=calibration_count+1'b1;
                        if(calibration_count==CALIB_SAMPLES-1) state<=MEAN;
                    end
                end
                MEAN: begin
                    mean_ax<=sum_ax>>>CALIB_LOG2;mean_ay<=sum_ay>>>CALIB_LOG2;mean_az<=sum_az>>>CALIB_LOG2;
                    state<=CHECK;
                end
                CHECK: begin
                    // Magnitude 0.8..1.2 g, plus spread gate; no confidence score.
                    if(gravity_norm2<64'sd42948362 || gravity_norm2>64'sd96632832 ||
                       hi_gx-lo_gx>MAX_GYRO_SPREAD || hi_gy-lo_gy>MAX_GYRO_SPREAD || hi_gz-lo_gz>MAX_GYRO_SPREAD ||
                       hi_ax-lo_ax>MAX_ACCEL_SPREAD || hi_ay-lo_ay>MAX_ACCEL_SPREAD || hi_az-lo_az>MAX_ACCEL_SPREAD) begin
                        clear_window();state<=COLLECT;
                    end else begin
                        gyro_bias_x_q8<=bias_average(sum_gx);gyro_bias_y_q8<=bias_average(sum_gy);gyro_bias_z_q8<=bias_average(sum_gz);
                        launch_math(2,mul(mean_ay,mean_ay)+mul(mean_az,mean_az),0,0,GRAV_SQRT);
                    end
                end
                GRAV_SQRT: begin
                    gravity_projection<=math_x_out;
                    launch_math(0,{{32{mean_az[31]}},mean_az},{{32{mean_ay[31]}},mean_ay},0,INIT_ROLL);
                end
                INIT_ROLL: begin
                    initial_roll<=math_angle_out;
                    launch_math(0,gravity_projection, -{{32{mean_ax[31]}},mean_ax},0,INIT_PITCH);
                end
                INIT_PITCH: begin
                    initial_pitch<=math_angle_out;
                    launch_math(1,0,0,initial_roll>>>1,ROLL_SC);
                end
                ROLL_SC: begin
                    cos_r<=math_x_out;sin_r<=math_y_out;
                    launch_math(1,0,0,initial_pitch>>>1,PITCH_SC);
                end
                PITCH_SC: begin cos_p<=math_x_out;sin_p<=math_y_out;state<=INIT_Q;end
                INIT_Q: begin
                    q0<=mul(cos_r,cos_p)>>>30;q1<=mul(sin_r,cos_p)>>>30;
                    q2<=mul(cos_r,sin_p)>>>30;q3<=-(mul(sin_r,sin_p)>>>30);
                    normalize_pass<=0;state<=NORM;
                end
                IDLE: if(data_valid) begin
                    previous_stamp<=sample_timestamp;
                    if(elapsed==0 || elapsed>MAX_DT || elapsed>32'd1048575) timing_error<=1;
                    else begin
                        dt_cycles<=elapsed;
                        corr_gx<=($signed(gyro_x_raw)*64'sd256)-gyro_bias_x_q8;
                        corr_gy<=($signed(gyro_y_raw)*64'sd256)-gyro_bias_y_q8;
                        corr_gz<=($signed(gyro_z_raw)*64'sd256)-gyro_bias_z_q8;
                        state<=STEP;
                    end
                end
                STEP: begin
                    rate_time_x<=corr_gx*$signed({1'b0,dt_cycles[19:0]});
                    rate_time_y<=corr_gy*$signed({1'b0,dt_cycles[19:0]});
                    rate_time_z<=corr_gz*$signed({1'b0,dt_cycles[19:0]});
                    state<=STEP_SCALE;
                end
                STEP_SCALE: begin
                    delta_x<=($signed(rate_time_x)*STEP_GAIN)>>>32;
                    delta_y<=($signed(rate_time_y)*STEP_GAIN)>>>32;
                    delta_z<=($signed(rate_time_z)*STEP_GAIN)>>>32;
                    state<=UPDATE_Q;
                end
                UPDATE_Q: begin
                    q0<=q0+((-mul(q1,delta_x)-mul(q2,delta_y)-mul(q3,delta_z))>>>30);
                    q1<=q1+(( mul(q0,delta_x)+mul(q2,delta_z)-mul(q3,delta_y))>>>30);
                    q2<=q2+(( mul(q0,delta_y)-mul(q1,delta_z)+mul(q3,delta_x))>>>30);
                    q3<=q3+(( mul(q0,delta_z)+mul(q1,delta_y)-mul(q2,delta_x))>>>30);
                    normalize_pass<=0;state<=NORM;
                end
                NORM: begin normal_scale<=(64'sd3221225472-norm_q30)>>>1;state<=NORM_APPLY;end
                NORM_APPLY: begin
                    q0<=($signed(q0)*normal_scale)>>>30;q1<=($signed(q1)*normal_scale)>>>30;
                    q2<=($signed(q2)*normal_scale)>>>30;q3<=($signed(q3)*normal_scale)>>>30;
                    if(normalize_pass<2) begin normalize_pass<=normalize_pass+1'b1;state<=NORM;end
                    else state<=OUTPUT_ROLL;
                end
                OUTPUT_ROLL: launch_math(0,r33,r32,0,OUTPUT_PITCH);
                OUTPUT_PITCH: begin
                    roll_q16<=math_angle_out;roll_cdeg<=centidegrees(math_angle_out);
                    launch_math(0,r33,minus_r31,0,OUTPUT_YAW);
                end
                OUTPUT_YAW: begin
                    pitch_q16<=math_angle_out;pitch_cdeg<=centidegrees(math_angle_out);
                    launch_math(0,r11,r21,0,PUBLISH);
                end
                PUBLISH: begin
                    yaw_q16<=math_angle_out;yaw_cdeg<=centidegrees(math_angle_out);
                    ready<=1;angle_valid<=1;state<=IDLE;
                end
                MATH_WAIT: if(math_done) state<=math_return;
                default: begin timing_error<=1;ready<=0;clear_window();state<=COLLECT;end
            endcase
        end
    end
endmodule
