`timescale 1ns / 1ps

// ============================================================================
// cmd.sv — Command subsystem using AXI-Stream interconnect
//
// Internal pipeline (all AXI-Stream 8-bit byte buses):
//
//   Serial RX pin
//       │
//   uart_axis  ─── m_axis ──→  axis_fifo (RX) ─── m_axis ──→  cmd_proc_axis_adapter
//       ▲                                                              │
//   s_axis                                                        (original cmd_proc
//       │                                                          unchanged inside)
//   axis_fifo (TX) ◄── s_axis ─── cmd_proc_axis_adapter ◄─────────────┘
//       │
//   m_axis ──→ uart_axis  s_axis
//       │
//   Serial TX pin
//
// The original uart.sv, fifo.sv, and cmd_proc.sv are all UNCHANGED.
// Each is instantiated inside its respective AXI-Stream wrapper.
// ============================================================================

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

  // =========================================================================
  //  AXI-Stream buses (8-bit byte)
  // =========================================================================

  // UART RX → RX FIFO
  logic [7:0] uart_rx_axis_tdata;
  logic       uart_rx_axis_tvalid;
  logic       uart_rx_axis_tready;

  // RX FIFO → cmd_proc adapter
  logic [7:0] rxfifo_m_axis_tdata;
  logic       rxfifo_m_axis_tvalid;
  logic       rxfifo_m_axis_tready;

  // cmd_proc adapter → TX FIFO
  logic [7:0] proc_tx_axis_tdata;
  logic       proc_tx_axis_tvalid;
  logic       proc_tx_axis_tready;

  // TX FIFO → UART TX
  logic [7:0] txfifo_m_axis_tdata;
  logic       txfifo_m_axis_tvalid;
  logic       txfifo_m_axis_tready;

  // =========================================================================
  //  UART Core (AXI-Stream wrapped, instantiates uart.sv inside)
  // =========================================================================
  uart_axis #(
      .CLK_FREQ (CLK_FREQ),
      .BAUD_RATE(BAUD_RATE)
  ) uart_inst (
      .clk           (sys_clk),
      .rst_n         (rst_n),
      .rx_serial     (tx_din),
      .tx_serial     (rx_dout),
      // RX master → feeds into RX FIFO
      .m_axis_tdata  (uart_rx_axis_tdata),
      .m_axis_tvalid (uart_rx_axis_tvalid),
      .m_axis_tready (uart_rx_axis_tready),
      // TX slave ← fed from TX FIFO
      .s_axis_tdata  (txfifo_m_axis_tdata),
      .s_axis_tvalid (txfifo_m_axis_tvalid),
      .s_axis_tready (txfifo_m_axis_tready)
  );

  // =========================================================================
  //  RX FIFO (AXI-Stream wrapped, instantiates fifo.sv inside)
  // =========================================================================
  axis_fifo #(
      .DATA_WIDTH(8),
      .DEPTH     (FIFO_DEPTH)
  ) rx_fifo_inst (
      .clk           (sys_clk),
      .rst_n         (rst_n),
      // Slave ← from UART RX
      .s_axis_tdata  (uart_rx_axis_tdata),
      .s_axis_tvalid (uart_rx_axis_tvalid),
      .s_axis_tready (uart_rx_axis_tready),
      // Master → to cmd_proc adapter
      .m_axis_tdata  (rxfifo_m_axis_tdata),
      .m_axis_tvalid (rxfifo_m_axis_tvalid),
      .m_axis_tready (rxfifo_m_axis_tready)
  );

  // =========================================================================
  //  TX FIFO (AXI-Stream wrapped, instantiates fifo.sv inside)
  // =========================================================================
  axis_fifo #(
      .DATA_WIDTH(8),
      .DEPTH     (FIFO_DEPTH)
  ) tx_fifo_inst (
      .clk           (sys_clk),
      .rst_n         (rst_n),
      // Slave ← from cmd_proc adapter
      .s_axis_tdata  (proc_tx_axis_tdata),
      .s_axis_tvalid (proc_tx_axis_tvalid),
      .s_axis_tready (proc_tx_axis_tready),
      // Master → to UART TX
      .m_axis_tdata  (txfifo_m_axis_tdata),
      .m_axis_tvalid (txfifo_m_axis_tvalid),
      .m_axis_tready (txfifo_m_axis_tready)
  );

  // =========================================================================
  //  cmd_proc AXI-Stream adapter
  //
  //  Bridges between AXI-Stream byte buses and cmd_proc's original
  //  rx_valid/rx_byte + tx_start/tx_byte/tx_busy/tx_done interface.
  //
  //  RX side: AXI-Stream master (FIFO) → single-cycle rx_valid pulse.
  //           Accepts one byte per cycle from the FIFO when ready.
  //
  //  TX side: tx_start/tx_byte from cmd_proc → AXI-Stream slave (FIFO).
  //           tx_busy reflects FIFO back-pressure.
  //           tx_done pulses when the FIFO accepts the byte.
  // =========================================================================
  logic       proc_rx_valid;
  logic [7:0] proc_rx_byte;
  logic       proc_tx_start;
  logic [7:0] proc_tx_byte;
  logic       proc_tx_busy;
  logic       proc_tx_done;

  // ---- RX adapter: AXI-Stream → single-cycle valid pulse ----
  // The FIFO m_axis has combinational rd_data, so when tvalid is high the
  // byte is available.  We register it and present a one-cycle pulse, then
  // accept the next byte.
  logic       rx_drain_pending;
  logic [7:0] rx_drain_byte;

  always_ff @(posedge sys_clk) begin
    if (!rst_n) begin
      rx_drain_pending <= 1'b0;
      rx_drain_byte    <= '0;
    end else begin
      rx_drain_pending <= 1'b0;
      if (rxfifo_m_axis_tvalid && !rx_drain_pending) begin
        rx_drain_pending <= 1'b1;
        rx_drain_byte    <= rxfifo_m_axis_tdata;
      end
    end
  end

  // Pop FIFO in the same cycle we capture
  assign rxfifo_m_axis_tready = rxfifo_m_axis_tvalid && !rx_drain_pending;
  assign proc_rx_valid        = rx_drain_pending;
  assign proc_rx_byte         = rx_drain_byte;

  // ---- TX adapter: cmd_proc tx_start/tx_byte → AXI-Stream ----
  assign proc_tx_axis_tdata  = proc_tx_byte;
  assign proc_tx_axis_tvalid = proc_tx_start;
  assign proc_tx_busy        = ~proc_tx_axis_tready;
  assign proc_tx_done        = proc_tx_start & proc_tx_axis_tready;

  // =========================================================================
  //  Command Processor (UNCHANGED)
  // =========================================================================
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
