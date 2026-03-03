`timescale 1ns / 1ps

// ============================================================================
// cmd.sv - Command subsystem with flat register write bus output
//
// Internal: uart_axis → axis_fifo (RX) → cmd_proc_v2 → axis_fifo (TX) → uart_axis
// Output: wr_en / wr_addr / wr_data - single-cycle write pulses to ctrl_bus
// ============================================================================

module cmd #(
    parameter int CLK_FREQ   = 100_000_000,
    parameter int BAUD_RATE  = 115_200,
    parameter int FIFO_DEPTH = 16
) (
    input  logic       sys_clk,
    input  logic       rst_n,
    input  logic       tx_din,
    output logic       rx_dout,

    // Flat register write bus
    output logic       wr_en,
    output logic [7:0] wr_addr,
    output logic [7:0] wr_data
);

  // AXI-Stream byte buses
  logic [7:0] uart_rx_tdata;
  logic       uart_rx_tvalid, uart_rx_tready;
  logic [7:0] rxfifo_m_tdata;
  logic       rxfifo_m_tvalid, rxfifo_m_tready;
  logic [7:0] proc_tx_tdata;
  logic       proc_tx_tvalid, proc_tx_tready;
  logic [7:0] txfifo_m_tdata;
  logic       txfifo_m_tvalid, txfifo_m_tready;

  uart_axis #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) uart_inst (
      .clk(sys_clk), .rst_n(rst_n),
      .rx_serial(tx_din), .tx_serial(rx_dout),
      .m_axis_tdata(uart_rx_tdata),   .m_axis_tvalid(uart_rx_tvalid), .m_axis_tready(uart_rx_tready),
      .s_axis_tdata(txfifo_m_tdata),  .s_axis_tvalid(txfifo_m_tvalid),.s_axis_tready(txfifo_m_tready)
  );

  axis_fifo #(.DATA_WIDTH(8), .DEPTH(FIFO_DEPTH)) rx_fifo (
      .clk(sys_clk), .rst_n(rst_n),
      .s_axis_tdata(uart_rx_tdata),   .s_axis_tvalid(uart_rx_tvalid), .s_axis_tready(uart_rx_tready),
      .m_axis_tdata(rxfifo_m_tdata),  .m_axis_tvalid(rxfifo_m_tvalid),.m_axis_tready(rxfifo_m_tready)
  );

  axis_fifo #(.DATA_WIDTH(8), .DEPTH(FIFO_DEPTH)) tx_fifo (
      .clk(sys_clk), .rst_n(rst_n),
      .s_axis_tdata(proc_tx_tdata),   .s_axis_tvalid(proc_tx_tvalid), .s_axis_tready(proc_tx_tready),
      .m_axis_tdata(txfifo_m_tdata),  .m_axis_tvalid(txfifo_m_tvalid),.m_axis_tready(txfifo_m_tready)
  );

  // ---- RX FIFO → cmd_proc adapter ----
  logic       proc_rx_valid;
  logic [7:0] proc_rx_byte;
  logic       proc_tx_start;
  logic [7:0] proc_tx_byte;
  logic       proc_tx_busy;
  logic       proc_tx_done;
  logic       rx_drain_pending;
  logic [7:0] rx_drain_byte;

  always_ff @(posedge sys_clk) begin
    if (!rst_n) begin
      rx_drain_pending <= 1'b0;
      rx_drain_byte    <= '0;
    end else begin
      rx_drain_pending <= 1'b0;
      if (rxfifo_m_tvalid && !rx_drain_pending) begin
        rx_drain_pending <= 1'b1;
        rx_drain_byte    <= rxfifo_m_tdata;
      end
    end
  end

  assign rxfifo_m_tready = rxfifo_m_tvalid && !rx_drain_pending;
  assign proc_rx_valid   = rx_drain_pending;
  assign proc_rx_byte    = rx_drain_byte;

  // ---- cmd_proc → TX FIFO adapter ----
  assign proc_tx_tdata  = proc_tx_byte;
  assign proc_tx_tvalid = proc_tx_start;
  assign proc_tx_busy   = ~proc_tx_tready;
  assign proc_tx_done   = proc_tx_start & proc_tx_tready;

  // ---- Command Processor v2 ----
  logic       proc_wr_en;
  logic [7:0] proc_wr_addr;
  logic [7:0] proc_wr_data;

  cmd_proc proc (
      .clk(sys_clk), .rst_n(rst_n),
      .rx_valid(proc_rx_valid), .rx_byte(proc_rx_byte),
      .tx_start(proc_tx_start), .tx_byte(proc_tx_byte),
      .tx_busy(proc_tx_busy),   .tx_done(proc_tx_done),
      .wr_en(proc_wr_en), .wr_addr(proc_wr_addr), .wr_data(proc_wr_data)
  );

  // ---- Toggle CDC: convert wr_en pulse → toggle + held addr/data ----
  // The toggle flips each time proc fires a write.  The audio side detects
  // the flip and captures addr/data which are held stable from this clock.
  logic       wr_toggle = 0;
  logic [7:0] wr_addr_hold = 0;
  logic [7:0] wr_data_hold = 0;

  always_ff @(posedge sys_clk) begin
    if (!rst_n)
      wr_toggle <= 1'b0;
    else if (proc_wr_en) begin
      wr_toggle    <= ~wr_toggle;
      wr_addr_hold <= proc_wr_addr;
      wr_data_hold <= proc_wr_data;
    end
  end

  assign wr_en   = wr_toggle;     // reuse port: now carries toggle, not pulse
  assign wr_addr = wr_addr_hold;
  assign wr_data = wr_data_hold;

endmodule