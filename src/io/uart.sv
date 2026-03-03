`timescale 1ns / 1ps

module uart #(
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 115_200
) (
    input logic clk,
    input logic rst_n,

    input  logic       rx_serial,
    output logic       rx_valid,
    output logic [7:0] rx_byte,

    input  logic       tx_start,
    input  logic [7:0] tx_byte,
    output logic       tx_busy,
    output logic       tx_serial,
    output logic       tx_done
);

  localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

  typedef enum logic [2:0] {
    IDLE,
    START_BIT,
    DATA_BITS,
    STOP_BIT,
    CLEANUP
  } state_t;

  // --- RX LOGIC ---
  state_t        rx_state = IDLE;
  logic   [31:0] rx_clk_count = 0;
  logic   [ 2:0] rx_bit_index = 0;
  logic   [ 7:0] rx_data_temp = 0;

  logic rx_sync_1, rx_sync;

  always_ff @(posedge clk) begin
    rx_sync_1 <= rx_serial;
    rx_sync   <= rx_sync_1;
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      rx_state     <= IDLE;
      rx_valid     <= 0;
      rx_byte      <= 0;
      rx_clk_count <= 0;
      rx_bit_index <= 0;
    end else begin
      case (rx_state)
        IDLE: begin
          rx_valid     <= 0;
          rx_clk_count <= 0;
          rx_bit_index <= 0;
          if (rx_sync == 1'b0) rx_state <= START_BIT;
        end

        START_BIT: begin
          if (rx_clk_count == (CLKS_PER_BIT - 1) / 2) begin
            if (rx_sync == 1'b0) begin
              rx_clk_count <= 0;
              rx_state     <= DATA_BITS;
            end else rx_state <= IDLE;
          end else rx_clk_count <= rx_clk_count + 1;
        end

        DATA_BITS: begin
          if (rx_clk_count < CLKS_PER_BIT - 1) rx_clk_count <= rx_clk_count + 1;
          else begin
            rx_clk_count <= 0;
            rx_data_temp[rx_bit_index] <= rx_sync;
            if (rx_bit_index < 7) rx_bit_index <= rx_bit_index + 1;
            else begin
              rx_bit_index <= 0;
              rx_state     <= STOP_BIT;
            end
          end
        end

        STOP_BIT: begin
          if (rx_clk_count < CLKS_PER_BIT - 1) begin
            rx_clk_count <= rx_clk_count + 1;
          end else begin
            // Pulse valid signal here so it's ready exactly at the end of the bit
            rx_valid <= 1'b1;
            rx_byte  <= rx_data_temp;
            rx_clk_count <= 0;
            rx_state <= CLEANUP;
          end
        end

        CLEANUP: begin
          // Ensure valid is only high for one cycle and return to IDLE
          rx_valid <= 1'b0;
          rx_state <= IDLE;
        end

        default: rx_state <= IDLE;
      endcase
    end
  end

  // --- TX LOGIC ---
  state_t        tx_state = IDLE;
  logic   [31:0] tx_clk_count = 0;
  logic   [ 2:0] tx_bit_index = 0;
  logic   [ 7:0] tx_data_temp = 0;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      tx_state     <= IDLE;
      tx_busy      <= 0;
      tx_serial    <= 1;
      tx_done      <= 0;
      tx_clk_count <= 0;
      tx_bit_index <= 0;
    end else begin
      case (tx_state)
        IDLE: begin
          tx_serial <= 1;
          tx_done   <= 0;
          if (tx_start) begin
            tx_busy      <= 1;
            tx_data_temp <= tx_byte;
            tx_state     <= START_BIT;
            tx_clk_count <= 0;
          end else begin
            tx_busy <= 0;
          end
        end

        START_BIT: begin
          tx_serial <= 0;
          if (tx_clk_count < CLKS_PER_BIT - 1) tx_clk_count <= tx_clk_count + 1;
          else begin
            tx_clk_count <= 0;
            tx_state     <= DATA_BITS;
          end
        end

        DATA_BITS: begin
          tx_serial <= tx_data_temp[tx_bit_index];
          if (tx_clk_count < CLKS_PER_BIT - 1) tx_clk_count <= tx_clk_count + 1;
          else begin
            tx_clk_count <= 0;
            if (tx_bit_index < 7) tx_bit_index <= tx_bit_index + 1;
            else begin
              tx_bit_index <= 0;
              tx_state     <= STOP_BIT;
            end
          end
        end

        STOP_BIT: begin
          tx_serial <= 1;
          if (tx_clk_count < CLKS_PER_BIT - 1) tx_clk_count <= tx_clk_count + 1;
          else begin
            tx_done  <= 1;
            tx_state <= CLEANUP;
          end
        end

        CLEANUP: begin
          tx_done  <= 1;
          tx_state <= IDLE;
        end

        default: tx_state <= IDLE;
      endcase
    end
  end
endmodule