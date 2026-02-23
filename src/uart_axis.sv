`timescale 1ns / 1ps

// ============================================================================
// uart_axis.sv — AXI-Stream wrapper for the original uart.sv core
//
// Instantiates uart.sv internally and presents two 8-bit AXI-Stream ports:
//
//   m_axis (master, RX) — Received bytes from the serial line.
//       tdata[7:0] = byte, tvalid pulses for one cycle per byte.
//       tready back-pressure: if downstream deasserts tready, bytes are held
//       until accepted (UART hardware FIFO is only 1 byte deep, so prolonged
//       back-pressure will lose data — use a FIFO downstream).
//
//   s_axis (slave, TX)  — Bytes to transmit on the serial line.
//       tdata[7:0] = byte.  tready deasserted while UART TX is busy.
//       Transfer occurs when (tvalid && tready); byte is latched and sent.
//
// This module replaces the direct rx_valid/rx_byte + tx_start/tx_byte/tx_busy
// handshake with standard AXI-Stream, so any upstream (FIFOs, cmd_proc
// adapters, etc.) can plug in with the same protocol.
// ============================================================================

module uart_axis #(
    parameter int CLK_FREQ  = 100_000_000,
    parameter int BAUD_RATE = 115_200
)(
    input  logic       clk,
    input  logic       rst_n,

    // Serial pins
    input  logic       rx_serial,
    output logic       tx_serial,

    // AXI-Stream master — received bytes (RX)
    output logic [7:0] m_axis_tdata,
    output logic       m_axis_tvalid,
    input  logic       m_axis_tready,

    // AXI-Stream slave — bytes to transmit (TX)
    input  logic [7:0] s_axis_tdata,
    input  logic       s_axis_tvalid,
    output logic       s_axis_tready
);

    // ---- Internal wires to original uart core ----
    logic       core_rx_valid;
    logic [7:0] core_rx_byte;
    logic       core_tx_start;
    logic [7:0] core_tx_byte;
    logic       core_tx_busy;
    logic       core_tx_done;

    // ---- Original UART core (UNCHANGED) ----
    uart #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) core (
        .clk      (clk),
        .rst_n    (rst_n),
        .rx_serial(rx_serial),
        .rx_valid (core_rx_valid),
        .rx_byte  (core_rx_byte),
        .tx_start (core_tx_start),
        .tx_byte  (core_tx_byte),
        .tx_busy  (core_tx_busy),
        .tx_serial(tx_serial),
        .tx_done  (core_tx_done)
    );

    // =========================================================================
    // RX path → AXI-Stream Master
    //
    // core_rx_valid is a single-cycle pulse.  We latch the byte and hold
    // m_axis_tvalid until downstream accepts.  If a new byte arrives while
    // the previous one hasn't been accepted, it is LOST (same as the original
    // design without a FIFO).  Add a downstream axis FIFO to prevent this.
    // =========================================================================
    logic [7:0] rx_data_reg;
    logic       rx_valid_reg;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rx_data_reg  <= '0;
            rx_valid_reg <= 1'b0;
        end else begin
            if (rx_valid_reg && m_axis_tready)
                rx_valid_reg <= 1'b0;
            if (core_rx_valid) begin
                rx_data_reg  <= core_rx_byte;
                rx_valid_reg <= 1'b1;
            end
        end
    end

    assign m_axis_tdata  = rx_data_reg;
    assign m_axis_tvalid = rx_valid_reg;

    // =========================================================================
    // TX path ← AXI-Stream Slave
    //
    // Accept a byte when tvalid && tready (UART not busy).  Latch and start
    // transmission immediately.  tready is deasserted until the byte finishes.
    // =========================================================================
    typedef enum logic [1:0] {
        TX_IDLE,
        TX_SEND,
        TX_WAIT
    } tx_state_t;

    tx_state_t tx_state = TX_IDLE;
    logic [7:0] tx_data_hold;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            tx_state      <= TX_IDLE;
            core_tx_start <= 1'b0;
            core_tx_byte  <= '0;
            tx_data_hold  <= '0;
        end else begin
            core_tx_start <= 1'b0;

            case (tx_state)
                TX_IDLE: begin
                    if (s_axis_tvalid && !core_tx_busy) begin
                        tx_data_hold <= s_axis_tdata;
                        tx_state     <= TX_SEND;
                    end
                end

                TX_SEND: begin
                    core_tx_byte  <= tx_data_hold;
                    core_tx_start <= 1'b1;
                    tx_state      <= TX_WAIT;
                end

                TX_WAIT: begin
                    if (core_tx_done)
                        tx_state <= TX_IDLE;
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    assign s_axis_tready = (tx_state == TX_IDLE) && !core_tx_busy;

endmodule
