`timescale 1ns / 1ps

module cmd #(
    parameter int CLK_FREQ   = 100_000_000,
    parameter int BAUD_RATE  = 115_200,
    parameter int FIFO_DEPTH = 16
) (
    input  logic sys_clk,
    input  logic rst_n,
    input  logic tx_din,
    output logic rx_dout,

    input logic [3:0] sw_effect,

    // DD3 delay controls
    output logic [ 7:0] tone_val,
    output logic [ 7:0] level_val,
    output logic [ 7:0] feedback_val,
    output logic [31:0] time_val,
    // CE-5 chorus controls
    output logic [ 7:0] chorus_rate_val,
    output logic [ 7:0] chorus_depth_val,
    output logic [ 7:0] chorus_efx_val,
    output logic [ 7:0] chorus_eqhi_val,
    output logic [ 7:0] chorus_eqlo_val,
    // Phaser controls
    output logic [ 7:0] phaser_speed_val,
    output logic        phaser_fben_val,
    // Tremolo controls
    output logic [ 7:0] trem_rate_val,
    output logic [ 7:0] trem_depth_val,
    output logic        trem_shape_val
);

  // ---------------------------------------------------------------
  // UART <-> FIFO signals
  // ---------------------------------------------------------------
  logic [7:0] uart_rx_data;
  logic       uart_rx_valid;
  logic [7:0] uart_tx_data;
  logic       uart_tx_start;
  logic       uart_tx_busy;
  logic       uart_tx_done;

  // ---------------------------------------------------------------
  // RX FIFO signals  (UART -> cmd_proc)
  // ---------------------------------------------------------------
  logic [7:0] rx_fifo_rd_data;
  logic       rx_fifo_empty;
  logic       rx_fifo_full;
  logic       rx_fifo_rd_en;

  // ---------------------------------------------------------------
  // TX FIFO signals  (cmd_proc -> UART)
  // ---------------------------------------------------------------
  logic [7:0] tx_fifo_wr_data;
  logic       tx_fifo_wr_en;
  logic       tx_fifo_empty;
  logic       tx_fifo_full;
  logic [7:0] tx_fifo_rd_data;
  logic       tx_fifo_rd_en;

  // ---------------------------------------------------------------
  // cmd_proc interface signals
  // ---------------------------------------------------------------
  logic [7:0] proc_rx_byte;
  logic       proc_rx_valid;
  logic       proc_tx_start;
  logic [7:0] proc_tx_byte;
  logic       proc_tx_busy;
  logic       proc_tx_done;

  // =================================================================
  //  UART Core
  // =================================================================
  uart #(
      .CLK_FREQ (CLK_FREQ),
      .BAUD_RATE(BAUD_RATE)
  ) uart_inst (
      .clk      (sys_clk),
      .rst_n    (rst_n),
      .rx_serial(tx_din),
      .rx_valid (uart_rx_valid),
      .rx_byte  (uart_rx_data),
      .tx_start (uart_tx_start),
      .tx_byte  (uart_tx_data),
      .tx_busy  (uart_tx_busy),
      .tx_serial(rx_dout),
      .tx_done  (uart_tx_done)
  );

  // =================================================================
  //  RX FIFO : UART receiver -> cmd_proc
  //  Write when UART produces a byte, read when cmd_proc is ready
  // =================================================================
  fifo #(
      .DATA_WIDTH(8),
      .DEPTH     (FIFO_DEPTH)
  ) rx_fifo_inst (
      .clk    (sys_clk),
      .rst_n  (rst_n),
      .wr_en  (uart_rx_valid & ~rx_fifo_full),
      .wr_data(uart_rx_data),
      .full   (rx_fifo_full),
      .rd_en  (rx_fifo_rd_en),
      .rd_data(rx_fifo_rd_data),
      .empty  (rx_fifo_empty),
      .count  ()
  );

  // Present one byte at a time to cmd_proc with a single-cycle valid pulse.
  // rd_en is asserted for one cycle; the data is available combinationally.
  // RX drain: The FIFO has combinational rd_data (async read).
  // When we assert rd_en, rd_data is valid that cycle, and rd_ptr advances
  // on the next posedge. We register the data and present it as a valid pulse.
  logic       rx_drain_pending;
  logic [7:0] rx_drain_byte;

  always_ff @(posedge sys_clk) begin
    if (!rst_n) begin
      rx_drain_pending <= 0;
      rx_drain_byte    <= 0;
    end else begin
      rx_drain_pending <= 0;
      if (!rx_fifo_empty && !rx_drain_pending) begin
        rx_drain_pending <= 1;
        rx_drain_byte    <= rx_fifo_rd_data;  // capture before ptr advances
      end
    end
  end

  // Pop the FIFO in the same cycle we capture the data
  assign rx_fifo_rd_en = !rx_fifo_empty && !rx_drain_pending;
  assign proc_rx_valid = rx_drain_pending;
  assign proc_rx_byte  = rx_drain_byte;

  // =================================================================
  //  TX FIFO : cmd_proc -> UART transmitter
  //  Write when cmd_proc issues tx_start, drain into UART when idle
  // =================================================================
  fifo #(
      .DATA_WIDTH(8),
      .DEPTH     (FIFO_DEPTH)
  ) tx_fifo_inst (
      .clk    (sys_clk),
      .rst_n  (rst_n),
      .wr_en  (tx_fifo_wr_en),
      .wr_data(tx_fifo_wr_data),
      .full   (tx_fifo_full),
      .rd_en  (tx_fifo_rd_en),
      .rd_data(tx_fifo_rd_data),
      .empty  (tx_fifo_empty),
      .count  ()
  );

  // cmd_proc writes into the TX FIFO via its existing tx_start / tx_byte
  assign tx_fifo_wr_en = proc_tx_start & ~tx_fifo_full;
  assign tx_fifo_wr_data = proc_tx_byte;

  // Tell cmd_proc the "UART" is busy when the TX FIFO is full
  assign proc_tx_busy = tx_fifo_full;

  // TX drain state machine – feeds bytes from TX FIFO into the real UART
  typedef enum logic [1:0] {
    TX_DRN_IDLE,
    TX_DRN_START,
    TX_DRN_WAIT
  } tx_drain_state_t;

  tx_drain_state_t tx_drn_state;
  logic [7:0] tx_drn_byte;

  always_ff @(posedge sys_clk) begin
    if (!rst_n) begin
      tx_drn_state  <= TX_DRN_IDLE;
      uart_tx_start <= 0;
      uart_tx_data  <= 0;
      tx_fifo_rd_en <= 0;
      tx_drn_byte   <= 0;
    end else begin
      uart_tx_start <= 0;
      tx_fifo_rd_en <= 0;

      case (tx_drn_state)
        TX_DRN_IDLE: begin
          if (!tx_fifo_empty && !uart_tx_busy) begin
            // Capture data and pop FIFO in same cycle (combinational read)
            tx_drn_byte   <= tx_fifo_rd_data;
            tx_fifo_rd_en <= 1;
            tx_drn_state  <= TX_DRN_START;
          end
        end

        TX_DRN_START: begin
          // Present captured byte to UART
          uart_tx_data  <= tx_drn_byte;
          uart_tx_start <= 1;
          tx_drn_state  <= TX_DRN_WAIT;
        end

        TX_DRN_WAIT: begin
          if (uart_tx_done) begin
            tx_drn_state <= TX_DRN_IDLE;
          end
        end

        default: tx_drn_state <= TX_DRN_IDLE;
      endcase
    end
  end

  // Generate a fake tx_done pulse back to cmd_proc when FIFO accepts the byte
  // This lets cmd_proc advance immediately without waiting for the wire.
  assign proc_tx_done = proc_tx_start & ~tx_fifo_full;

  // =================================================================
  //  Command Processor
  // =================================================================
  cmd_proc cmd_proc_inst (
      .clk             (sys_clk),
      .rst_n           (rst_n),
      .rx_valid        (proc_rx_valid),
      .rx_byte         (proc_rx_byte),
      .tx_start        (proc_tx_start),
      .tx_byte         (proc_tx_byte),
      .tx_busy         (proc_tx_busy),
      .tx_done         (proc_tx_done),
      .sw_effect       (sw_effect),
      .tone_val        (tone_val),
      .level_val       (level_val),
      .feedback_val    (feedback_val),
      .time_val        (time_val),
      .chorus_rate_val (chorus_rate_val),
      .chorus_depth_val(chorus_depth_val),
      .chorus_efx_val  (chorus_efx_val),
      .chorus_eqhi_val (chorus_eqhi_val),
      .chorus_eqlo_val (chorus_eqlo_val),
      .phaser_speed_val(phaser_speed_val),
      .phaser_fben_val (phaser_fben_val),
      .trem_rate_val   (trem_rate_val),
      .trem_depth_val  (trem_depth_val),
      .trem_shape_val  (trem_shape_val)
  );

endmodule
