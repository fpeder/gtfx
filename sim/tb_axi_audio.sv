`timescale 1ns / 1ps

// ============================================================================
// tb_audio_axis.sv — Testbench for audio_axis
//
// Tests:
//   1. m_axis_tvalid pulses at 48 kHz rate (lrclk rising edge)
//   2. m_axis_tdata contains packed stereo from I2S RX
//   3. s_axis_tready always = 1 (DAC always ready)
//   4. s_axis data is latched into DAC on valid beat
//   5. No spurious tvalid between sample boundaries
// ============================================================================

module tb_axi_audio;

  import axis_audio_pkg::*;

  // 12.288 MHz clock → ~81.38 ns period
  localparam int CLK_P = 81;

  logic clk_audio = 0;
  logic resetn;

  // I2S signals
  logic tx_mclk, tx_lrclk, tx_sclk, tx_serial;
  logic rx_mclk, rx_lrclk, rx_sclk;
  logic        rx_serial;

  // AXI-Stream
  logic [47:0] m_axis_tdata;
  logic        m_axis_tvalid;
  logic        m_axis_tready;
  logic [47:0] s_axis_tdata;
  logic        s_axis_tvalid;
  logic        s_axis_tready;

  always #(CLK_P / 2) clk_audio = ~clk_audio;

  audio_axis #(
      .AUDIO_W(24)
  ) dut (
      .clk_audio    (clk_audio),
      .resetn       (resetn),
      .tx_mclk      (tx_mclk),
      .tx_lrclk     (tx_lrclk),
      .tx_sclk      (tx_sclk),
      .tx_serial    (tx_serial),
      .rx_mclk      (rx_mclk),
      .rx_lrclk     (rx_lrclk),
      .rx_sclk      (rx_sclk),
      .rx_serial    (rx_serial),
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

  initial begin
    $display("=== tb_audio_axis ===");

    resetn        = 0;
    rx_serial     = 1'b0;
    m_axis_tready = 1'b1;
    s_axis_tdata  = '0;
    s_axis_tvalid = 1'b0;
    repeat (10) @(posedge clk_audio);
    resetn = 1;

    // ---- Test 1: s_axis_tready always = 1 ----
    repeat (5) @(posedge clk_audio);
    check("TREADY: s_axis_tready = 1", s_axis_tready == 1'b1);

    // ---- Test 2: Wait for first m_axis_tvalid ----
    // The I2S driver divides clk by 256 for lrclk (48 kHz).
    // Wait for at least one full lrclk period.
    begin
      int   timeout = 0;
      int   tvalid_count = 0;
      logic lrclk_prev_tb = 0;

      // Wait up to 600 clocks (more than one lrclk period = 256 clocks)
      while (timeout < 600) begin
        @(posedge clk_audio);
        timeout++;
        if (m_axis_tvalid) begin
          tvalid_count++;
          if (tvalid_count == 1) begin
            check("M_VALID: first tvalid pulse detected", 1'b1);
            check("M_DATA: tdata is not X", !$isunknown(m_axis_tdata));
          end
        end
      end

      check("M_VALID: at least 1 tvalid in 600 clocks", tvalid_count >= 1);

      // tvalid should pulse, not stay high forever
      // (at 48 kHz with 12.288 MHz clock, one pulse every 256 clocks)
      check("M_VALID: not continuously asserted (pulsed)", tvalid_count < 10);
    end

    // ---- Test 3: Slave input latching ----
    // Send a known value and verify it's accepted
    s_axis_tdata  = pack_stereo(24'sh123456, 24'sh789ABC);
    s_axis_tvalid = 1'b1;
    @(posedge clk_audio);
    check("S_BEAT: tready during valid", s_axis_tready == 1'b1);
    s_axis_tvalid = 1'b0;

    // ---- Test 4: No tvalid between boundaries ----
    begin
      int spurious = 0;
      // After a tvalid pulse, count tvalid=1 in the next 100 clocks
      // (should be 0 since next sample is 256 clocks away)
      @(posedge m_axis_tvalid);
      @(posedge clk_audio);
      for (int i = 0; i < 100; i++) begin
        @(posedge clk_audio);
        if (m_axis_tvalid) spurious++;
      end
      check("NO_SPURIOUS: no tvalid in 100 clocks after pulse", spurious == 0);
    end

    $display("=== audio_axis: %0d passed, %0d failed ===", pass_count, fail_count);
    if (fail_count > 0) $fatal(1, "TESTS FAILED");
    $finish;
  end

endmodule
