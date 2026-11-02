// SPDX-License-Identifier: MIT
// MAX3421E host-mode SPI transport, synthesizable Verilog-2001.
// Reference: M5-Max3421E-USBShield hcd_max3421.c and MAX3421E Rev.4.
// This module does not enumerate USB devices or decode HID coordinates.
// All inputs/outputs except SPI and irq_n belong to the clk domain.
module max3421e_spi_transport #(
    parameter integer CLK_HZ = 50000000,
    parameter integer SPI_HZ = 5000000,
    parameter integer CS_SETUP_CYCLES = (CLK_HZ + 9999999) / 10000000,
    parameter integer CS_HOLD_CYCLES = (CLK_HZ + 9999999) / 10000000,
    parameter integer CS_HIGH_CYCLES = (CLK_HZ + 2499999) / 2500000
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         cmd_valid,
    output wire         cmd_ready,
    input  wire [2:0]   cmd_op,
    input  wire [4:0]   cmd_reg,
    input  wire [6:0]   cmd_len,
    input  wire [511:0] cmd_tx_data,
    output reg          busy,
    output reg          done,
    output reg  [2:0]   error_code,
    output reg          has_packet,
    output reg  [6:0]   rx_len,
    output reg  [511:0] rx_data,
    output wire         rx_byte_valid,
    output wire [7:0]   rx_byte,
    output wire [5:0]   rx_byte_index,
    output wire         rx_byte_last,
    output reg  [7:0]   status_byte,
    input  wire         irq_n,
    output wire         irq_active,
    output wire         spi_cs_n,
    output wire         spi_sclk,
    output wire         spi_mosi,
    input  wire         spi_miso
);
    // cmd_reg is the UNshifted register number, e.g. REVISION = 18.
    localparam [2:0] OP_REG_READ = 0, OP_REG_WRITE = 1,
                     OP_FIFO_READ = 2, OP_FIFO_WRITE = 3,
                     OP_RECEIVE = 4;
    localparam [2:0] ERR_NONE = 0, ERR_OP = 1, ERR_ARGUMENT = 2,
                     ERR_PACKET_LENGTH = 3;
    localparam [4:0] R_RCVFIFO = 1, R_SNDFIFO = 2, R_SUDFIFO = 4,
                     R_RCVBC = 6, R_HIRQ = 25;
    localparam [3:0] S_IDLE = 0, S_DIRECT_ISSUE = 1, S_DIRECT_WAIT = 2,
                     S_IRQ_ISSUE = 3, S_IRQ_WAIT = 4,
                     S_COUNT_ISSUE = 5, S_COUNT_WAIT = 6,
                     S_FIFO_ISSUE = 7, S_FIFO_WAIT = 8,
                     S_ACK_ISSUE = 9, S_ACK_WAIT = 10;

    reg [3:0] state;
    reg [4:0] saved_reg;
    reg saved_write;
    reg [6:0] active_len;
    reg [511:0] saved_tx;
    reg [6:0] packet_len;
    reg irq_meta, irq_sync;
    reg m_req_valid, m_write;
    reg [4:0] m_reg;
    reg [6:0] m_len;
    reg [511:0] m_tx;
    wire m_ready, m_done, m_status_valid, m_byte_valid;
    wire [7:0] m_status, m_byte;
    wire [5:0] m_byte_index;
    wire [511:0] m_rx;

    assign cmd_ready = (state == S_IDLE) && m_ready;
    assign irq_active = !irq_sync;
    // Streaming bytes have no backpressure. Results are committed at done.
    assign rx_byte_valid = m_byte_valid &&
                           (((state == S_DIRECT_WAIT) && !saved_write) ||
                            (state == S_FIFO_WAIT));
    assign rx_byte = m_byte;
    assign rx_byte_index = m_byte_index;
    assign rx_byte_last = rx_byte_valid &&
                         ({1'b0, m_byte_index} == (active_len - 7'd1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin irq_meta <= 1'b1; irq_sync <= 1'b1; end
        else begin irq_meta <= irq_n; irq_sync <= irq_meta; end
    end

    // The SPI bus is exclusively owned by this sequencer.
    always @* begin
        m_req_valid = 1'b0;
        m_write = 1'b0;
        m_reg = 5'd0;
        m_len = 7'd1;
        m_tx = 512'd0;
        case (state)
            S_DIRECT_ISSUE: begin
                m_req_valid = 1'b1;
                m_write = saved_write;
                m_reg = saved_reg;
                m_len = active_len;
                m_tx = saved_tx;
            end
            S_IRQ_ISSUE: begin m_req_valid = 1'b1; m_reg = R_HIRQ; end
            S_COUNT_ISSUE: begin m_req_valid = 1'b1; m_reg = R_RCVBC; end
            S_FIFO_ISSUE: begin
                m_req_valid = 1'b1; m_reg = R_RCVFIFO; m_len = packet_len;
            end
            S_ACK_ISSUE: begin
                m_req_valid = 1'b1; m_write = 1'b1; m_reg = R_HIRQ;
                m_tx[7:0] = 8'h04; // W1C: clear ONLY RCVDAVIRQ.
            end
            default: begin end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            saved_reg <= 0; saved_write <= 0; saved_tx <= 0;
            active_len <= 0; packet_len <= 0;
            busy <= 0; done <= 0; error_code <= ERR_NONE;
            has_packet <= 0; rx_len <= 0; rx_data <= 0; status_byte <= 0;
        end else begin
            done <= 1'b0;
            if (m_status_valid) status_byte <= m_status;
            case (state)
                S_IDLE: if (cmd_valid && cmd_ready) begin
                    busy <= 1'b1;
                    error_code <= ERR_NONE;
                    has_packet <= 1'b0;
                    rx_len <= 0; rx_data <= 0; packet_len <= 0;
                    saved_reg <= cmd_reg; saved_tx <= cmd_tx_data;
                    saved_write <= (cmd_op == OP_REG_WRITE) ||
                                   (cmd_op == OP_FIFO_WRITE);
                    active_len <= ((cmd_op == OP_REG_READ) ||
                                   (cmd_op == OP_REG_WRITE)) ? 7'd1 : cmd_len;
                    case (cmd_op)
                        OP_REG_READ, OP_REG_WRITE: state <= S_DIRECT_ISSUE;
                        OP_FIFO_READ: begin
                            // SUDFIFO supports an eight-byte readback after an
                            // eight-byte write (MAX3421E Programming Guide).
                            if ((cmd_len == 0) || (cmd_len > 64) ||
                                !((cmd_reg == R_RCVFIFO) ||
                                  ((cmd_reg == R_SUDFIFO) && (cmd_len == 8)))) begin
                                error_code <= ERR_ARGUMENT; busy <= 0; done <= 1;
                            end else state <= S_DIRECT_ISSUE;
                        end
                        OP_FIFO_WRITE: begin
                            if ((cmd_len == 0) || (cmd_len > 64) ||
                                !((cmd_reg == R_SNDFIFO) ||
                                  ((cmd_reg == R_SUDFIFO) && (cmd_len == 8)))) begin
                                error_code <= ERR_ARGUMENT; busy <= 0; done <= 1;
                            end else state <= S_DIRECT_ISSUE;
                        end
                        OP_RECEIVE: state <= S_IRQ_ISSUE;
                        default: begin error_code <= ERR_OP; busy <= 0; done <= 1; end
                    endcase
                end
                S_DIRECT_ISSUE: if (m_ready) state <= S_DIRECT_WAIT;
                S_DIRECT_WAIT: if (m_done) begin
                    if (!saved_write) begin rx_data <= m_rx; rx_len <= active_len; end
                    busy <= 0; done <= 1; state <= S_IDLE;
                end
                S_IRQ_ISSUE: if (m_ready) state <= S_IRQ_WAIT;
                S_IRQ_WAIT: if (m_done) begin
                    if (m_rx[2]) state <= S_COUNT_ISSUE;
                    else begin busy <= 0; done <= 1; state <= S_IDLE; end
                end
                S_COUNT_ISSUE: if (m_ready) state <= S_COUNT_WAIT;
                S_COUNT_WAIT: if (m_done) begin
                    if (m_rx[7:0] > 8'd64) begin
                        // Preserve the FIFO and IRQ on an invalid count.
                        error_code <= ERR_PACKET_LENGTH;
                        busy <= 0; done <= 1; state <= S_IDLE;
                    end else begin
                        packet_len <= m_rx[6:0]; active_len <= m_rx[6:0];
                        if (m_rx[7:0] == 0) state <= S_ACK_ISSUE;
                        else state <= S_FIFO_ISSUE;
                    end
                end
                S_FIFO_ISSUE: if (m_ready) state <= S_FIFO_WAIT;
                S_FIFO_WAIT: if (m_done) begin rx_data <= m_rx; state <= S_ACK_ISSUE; end
                S_ACK_ISSUE: if (m_ready) state <= S_ACK_WAIT;
                S_ACK_WAIT: if (m_done) begin
                    rx_len <= packet_len; has_packet <= 1;
                    busy <= 0; done <= 1; state <= S_IDLE;
                end
                default: begin state <= S_IDLE; busy <= 0; end
            endcase
        end
    end

    max3421e_spi_master #(
        .CLK_HZ(CLK_HZ), .SPI_HZ(SPI_HZ),
        .CS_SETUP_CYCLES(CS_SETUP_CYCLES), .CS_HOLD_CYCLES(CS_HOLD_CYCLES),
        .CS_HIGH_CYCLES(CS_HIGH_CYCLES)
    ) spi_engine (
        .clk(clk), .rst_n(rst_n), .req_valid(m_req_valid), .req_ready(m_ready),
        .req_reg(m_reg), .req_write(m_write), .req_len(m_len), .req_tx_data(m_tx),
        .done(m_done), .rx_data(m_rx), .status_valid(m_status_valid),
        .status_byte(m_status), .rx_byte_valid(m_byte_valid), .rx_byte(m_byte),
        .rx_byte_index(m_byte_index), .spi_cs_n(spi_cs_n),
        .spi_sclk(spi_sclk), .spi_mosi(spi_mosi), .spi_miso(spi_miso)
    );
endmodule

// Internal byte engine. SPI mode 0, MSB first, one command per CS assertion.
// req_len: 0..64 DATA bytes, not counting the command/status byte.
// The public transport checks request arguments before invoking this engine.
module max3421e_spi_master #(
    parameter integer CLK_HZ = 50000000,
    parameter integer SPI_HZ = 5000000,
    parameter integer CS_SETUP_CYCLES = (CLK_HZ + 9999999) / 10000000,
    parameter integer CS_HOLD_CYCLES = (CLK_HZ + 9999999) / 10000000,
    parameter integer CS_HIGH_CYCLES = (CLK_HZ + 2499999) / 2500000
) (
    input wire clk, input wire rst_n,
    input wire req_valid, output wire req_ready,
    input wire [4:0] req_reg, input wire req_write,
    input wire [6:0] req_len, input wire [511:0] req_tx_data,
    output reg done, output reg [511:0] rx_data,
    output reg status_valid, output reg [7:0] status_byte,
    output reg rx_byte_valid, output reg [7:0] rx_byte,
    output reg [5:0] rx_byte_index,
    output reg spi_cs_n, output reg spi_sclk, output reg spi_mosi,
    input wire spi_miso
);
    localparam integer SAFE_SPI_HZ = (SPI_HZ > 0) ? SPI_HZ : 1;
    localparam integer HALF_CYCLES =
        (CLK_HZ + 2 * SAFE_SPI_HZ - 1) / (2 * SAFE_SPI_HZ);
    localparam [2:0] E_IDLE = 0, E_SETUP = 1, E_SHIFT = 2,
                     E_HOLD = 3, E_GAP = 4, E_RECOVERY = 5;
    reg [2:0] state, bit_index;
    reg [31:0] delay_count;
    reg [6:0] byte_position, length_latched;
    reg write_latched;
    reg [7:0] tx_shift, rx_shift;
    reg [511:0] payload_shift;
    assign req_ready = (state == E_IDLE);

    // synthesis translate_off
    initial begin
        if ((CLK_HZ <= 0) || (SPI_HZ <= 0) || (SPI_HZ > 26000000) ||
            (CLK_HZ < 2 * SPI_HZ) || (CS_SETUP_CYCLES < 1) ||
            (CS_HOLD_CYCLES < ((CLK_HZ + 33333332) / 33333333)) ||
            (CS_HIGH_CYCLES < ((CLK_HZ + 4999999) / 5000000))) begin
            $display("ERROR: invalid MAX3421E SPI clock/CS parameters");
            $finish;
        end
    end
    // synthesis translate_on

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= E_RECOVERY; bit_index <= 7; delay_count <= 0;
            byte_position <= 0; length_latched <= 0; write_latched <= 0;
            tx_shift <= 0; rx_shift <= 0; payload_shift <= 0;
            spi_cs_n <= 1; spi_sclk <= 0; spi_mosi <= 0;
            done <= 0; rx_data <= 0; status_valid <= 0; status_byte <= 0;
            rx_byte_valid <= 0; rx_byte <= 0; rx_byte_index <= 0;
        end else begin
            done <= 0; status_valid <= 0; rx_byte_valid <= 0;
            case (state)
                E_IDLE: if (req_valid && req_ready) begin
                    // {Reg[4:0], reserved=0, DIR, ACKSTAT=0}
                    tx_shift <= {req_reg, 1'b0, req_write, 1'b0};
                    spi_mosi <= req_reg[4]; spi_cs_n <= 0; spi_sclk <= 0;
                    length_latched <= req_len; write_latched <= req_write;
                    payload_shift <= req_tx_data;
                    byte_position <= 0; bit_index <= 7; rx_shift <= 0;
                    rx_data <= 0; delay_count <= 0; state <= E_SETUP;
                end
                E_SETUP: begin
                    if (delay_count == CS_SETUP_CYCLES - 1) begin
                        delay_count <= 0; state <= E_SHIFT;
                    end else delay_count <= delay_count + 1;
                end
                E_SHIFT: begin
                    if (delay_count == HALF_CYCLES - 1) begin
                        delay_count <= 0;
                        if (!spi_sclk) begin
                            // MISO is source-synchronous; sample its raw pin.
                            spi_sclk <= 1;
                            rx_shift <= {rx_shift[6:0], spi_miso};
                            if (bit_index == 0) begin
                                if (byte_position == 0) begin
                                    status_byte <= {rx_shift[6:0], spi_miso};
                                    status_valid <= 1;
                                end else if (!write_latched) begin
                                    rx_data[(byte_position - 1) * 8 +: 8] <=
                                        {rx_shift[6:0], spi_miso};
                                    rx_byte <= {rx_shift[6:0], spi_miso};
                                    rx_byte_index <= byte_position - 1'b1;
                                    rx_byte_valid <= 1;
                                end
                            end
                        end else begin
                            spi_sclk <= 0;
                            if (bit_index == 0) begin
                                if (byte_position == length_latched) begin
                                    state <= E_HOLD;
                                end else begin
                                    byte_position <= byte_position + 1;
                                    bit_index <= 7; rx_shift <= 0;
                                    tx_shift <= write_latched ? payload_shift[7:0] : 8'd0;
                                    spi_mosi <= write_latched ? payload_shift[7] : 1'b0;
                                    payload_shift <= {8'd0, payload_shift[511:8]};
                                end
                            end else begin
                                bit_index <= bit_index - 1'b1;
                                tx_shift <= {tx_shift[6:0], 1'b0};
                                spi_mosi <= tx_shift[6];
                            end
                        end
                    end else delay_count <= delay_count + 1;
                end
                E_HOLD: begin
                    if (delay_count == CS_HOLD_CYCLES - 1) begin
                        spi_cs_n <= 1; spi_mosi <= 0;
                        delay_count <= 0; state <= E_GAP;
                    end else delay_count <= delay_count + 1;
                end
                E_GAP: begin
                    if (delay_count == CS_HIGH_CYCLES - 1) begin
                        delay_count <= 0; done <= 1; state <= E_IDLE;
                    end else delay_count <= delay_count + 1;
                end
                E_RECOVERY: begin
                    // Also honor CS high time after reset aborts a transaction.
                    if (delay_count == CS_HIGH_CYCLES - 1) begin
                        delay_count <= 0; state <= E_IDLE;
                    end else delay_count <= delay_count + 1;
                end
                default: begin
                    state <= E_RECOVERY; delay_count <= 0;
                    spi_cs_n <= 1; spi_sclk <= 0;
                end
            endcase
        end
    end
endmodule
