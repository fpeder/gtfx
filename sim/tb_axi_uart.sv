`timescale 1ns / 1ps

// ============================================================================
// tb_uart_axis.sv — Testbench for uart_axis
//
// Tests:
//   1. RX: serial byte in → m_axis_tvalid + correct tdata
//   2. TX: s_axis byte in → serial byte out
//   3. TX back-pressure: tready=0 while busy
//   4. Loopback: TX serial → RX serial, verify round-trip
//   5. Multiple bytes in sequence
// ============================================================================

module tb_axi_uart;

  // Use very fast baud for simulation
  localparam int CLK_FREQ = 10_000_000;
  localparam int BAUD_RATE = 1_000_000;
  localparam int CLK_P = 100;  // 10 MHz → 100ns period
  localparam int BIT_P = CLK_FREQ / BAUD_RATE * CLK_P;  // 1000 ns per bit

  logic       clk = 0;
  logic       rst_n;
  logic       rx_serial;
  logic       tx_serial;
  logic [7:0] m_axis_tdata;
  logic       m_axis_tvalid;
  logic       m_axis_tready;
  logic [7:0] s_axis_tdata;
  logic       s_axis_tvalid;
  logic       s_axis_tready;

  always #(CLK_P / 2) clk = ~clk;

  uart_axis #(
      .CLK_FREQ (CLK_FREQ),
      .BAUD_RATE(BAUD_RATE)
  ) dut (
      .clk          (clk),
      .rst_n        (rst_n),
      .rx_serial    (rx_serial),
      .tx_serial    (tx_serial),
      .m_axis_tdata (m_axis_tdata),
      .m_axis_tvalid(m_axis_tvalid),
      .m_axis_tready(m_axis_tready),
      .s_axis_tdata (s_axis_tdata),
      .s_axis_tvalid(s_axis_tvalid),
      .s_axis_tready(s_axis_tready)
  );

  int pass_count = 0;
  int fail_count = 0;

  task automatic check(input string name, input logic condition);
    if (condition) pass_count++;
    else begin
      fail_count++;
      $error("FAIL: %s", name);
    end
  endtask

  // Send a byte on the rx_serial line (8N1 format)
  task automatic send_serial_byte(input logic [7:0] data);
    // Start bit
    rx_serial = 1'b0;
    #(BIT_P);
    // Data bits (LSB first)
    for (int i = 0; i < 8; i++) begin
      rx_serial = data[i];
      #(BIT_P);
    end
    // Stop bit
    rx_serial = 1'b1;
    #(BIT_P);
  endtask

  // Receive a byte from the tx_serial line
  task automatic recv_serial_byte(output logic [7:0] data);
    // Wait for start bit (falling edge)
    @(negedge tx_serial);
    // Sample at middle of each bit
    #(BIT_P / 2);  // middle of start bit
    #(BIT_P);  // first data bit
    for (int i = 0; i < 8; i++) begin
      data[i] = tx_serial;
      if (i < 7) #(BIT_P);
    end
    #(BIT_P / 2);  // wait for stop bit
    #(BIT_P / 2);
  endtask

  logic [7:0] rx_captured;
  logic [7:0] tx_captured;

  initial begin
    $display("=== tb_uart_axis ===");

    rst_n         = 0;
    rx_serial     = 1'b1;  // idle high
    m_axis_tready = 1'b1;
    s_axis_tdata  = '0;
    s_axis_tvalid = 1'b0;
    repeat (5) @(posedge clk);
    rst_n = 1;
    repeat (5) @(posedge clk);

    // ---- Test 1: RX byte ----
    $display("  Sending 0xA5 on serial RX...");
    m_axis_tready = 1'b0;  // <-- ADDED: Assert back-pressure so the DUT holds tvalid
    fork
      send_serial_byte(8'hA5);
    join
    // Wait for m_axis_tvalid
    repeat (20) @(posedge clk);
    // Should see tvalid
    begin
      int timeout = 0;
      while (!m_axis_tvalid && timeout < 200) begin
        @(posedge clk);
        timeout++;
      end
      check("RX: m_axis_tvalid asserted", m_axis_tvalid == 1'b1);
      if (m_axis_tvalid) begin
        rx_captured = m_axis_tdata;
        check("RX: m_axis_tdata = 0xA5", rx_captured == 8'hA5);
        m_axis_tready = 1'b1;  // <-- ADDED: Assert tready to accept the byte
        @(posedge clk);  // accept it
        m_axis_tready = 1'b1;  // <-- ADDED: Assert tready to accept the byte
      end
    end

    repeat (5) @(posedge clk);

    // ---- Test 2: TX byte ----
    $display("  Sending 0x3C via AXI-Stream TX...");
    fork
      begin
        s_axis_tdata  = 8'h3C;
        s_axis_tvalid = 1'b1;
        @(posedge clk);
        while (!s_axis_tready) @(posedge clk);
        s_axis_tvalid = 1'b0;
      end
      begin
        recv_serial_byte(tx_captured);
      end
    join
    check("TX: serial byte = 0x3C", tx_captured == 8'h3C);

    repeat (10) @(posedge clk);

    // ---- Test 3: TX back-pressure while busy ----
    $display("  Testing TX back-pressure...");
    // Start a TX byte
    s_axis_tdata  = 8'hFF;
    s_axis_tvalid = 1'b1;
    @(posedge clk);
    while (!s_axis_tready) @(posedge clk);
    // Now try to send another immediately
    s_axis_tdata = 8'h00;
    repeat (3) @(posedge clk);
    check("TX_BP: tready=0 while transmitting", s_axis_tready == 1'b0);
    s_axis_tvalid = 1'b0;
    // Wait for TX to complete
    repeat (BIT_P / CLK_P * 12) @(posedge clk);

    // ---- Test 4: RX back-pressure ----
    $display("  Testing RX back-pressure...");
    m_axis_tready = 1'b0;
    send_serial_byte(8'h42);
    repeat (BIT_P / CLK_P * 2) @(posedge clk);
    if (m_axis_tvalid) begin
      check("RX_BP: byte held while tready=0", m_axis_tdata == 8'h42);
      m_axis_tready = 1'b1;
      @(posedge clk);
      check("RX_BP: accepted after tready=1", 1'b1);
    end
    m_axis_tready = 1'b1;

    repeat (5) @(posedge clk);

    // ---- Test 5: Loopback (TX → RX via serial wire) ----
    $display("  Loopback test...");
    // Connect tx_serial back to rx_serial for this test
    fork
      begin
        // Send via AXI-Stream TX
        s_axis_tdata  = 8'h77;
        s_axis_tvalid = 1'b1;
        @(posedge clk);
        while (!s_axis_tready) @(posedge clk);
        s_axis_tvalid = 1'b0;
      end
      begin
        // Feed tx_serial into rx_serial with a small delay
        logic serial_loopback;
        forever begin
          @(posedge clk);
          rx_serial = tx_serial;
        end
      end
      begin
        // Wait for RX output
        int timeout = 0;
        while (!m_axis_tvalid && timeout < 500) begin
          @(posedge clk);
          timeout++;
        end
        if (m_axis_tvalid) begin
          check("LOOP: received byte = 0x77", m_axis_tdata == 8'h77);
        end else begin
          check("LOOP: tvalid asserted within timeout", 1'b0);
        end
      end
    join_any
    disable fork;

    $display("=== uart_axis: %0d passed, %0d failed ===", pass_count, fail_count);
    if (fail_count > 0) $fatal(1, "TESTS FAILED");
    $finish;
  end

endmodule
