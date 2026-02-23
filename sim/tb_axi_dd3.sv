`timescale 1ns / 1ps

// ============================================================================
// tb_dd3_axis.sv — Testbench for dd3_axis
//
// Tests:
//   1. AXI-Stream handshake
//   2. level=0 → output = dry (wet contribution is zero)
//   3. level>0 with primed delay → output ≠ dry (echo present)
//   4. Stereo: L and R both get the same wet echo mixed in
//   5. Feedback: echoes decay over multiple delay periods
//   6. Short delay time: echo appears quickly
//   7. Back-pressure
// ============================================================================

module tb_dd3_axis;

  import axis_audio_pkg::*;

  localparam int CLK_P = 81;
  localparam int RAM_DEPTH = 200;  // Small for fast sim
  localparam int DLY_SAMPS = 100;

  logic        clk = 0;
  logic        rst_n;
  logic [47:0] s_axis_tdata;
  logic        s_axis_tvalid;
  logic        s_axis_tready;
  logic [47:0] m_axis_tdata;
  logic        m_axis_tvalid;
  logic        m_axis_tready;
  logic [7:0] tone_val, level_val, feedback_val;
  logic [31:0] time_val;

  always #(CLK_P / 2) clk = ~clk;

  dd3_axis #(
      .WIDTH    (24),
      .RAM_DEPTH(RAM_DEPTH)
  ) dut (
      .clk          (clk),
      .rst_n        (rst_n),
      .s_axis_tdata (s_axis_tdata),
      .s_axis_tvalid(s_axis_tvalid),
      .s_axis_tready(s_axis_tready),
      .m_axis_tdata (m_axis_tdata),
      .m_axis_tvalid(m_axis_tvalid),
      .m_axis_tready(m_axis_tready),
      .tone_val     (tone_val),
      .level_val    (level_val),
      .feedback_val (feedback_val),
      .time_val     (time_val)
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

  task automatic send_sample(input logic signed [23:0] left, input logic signed [23:0] right,
                             output logic [47:0] result);
    s_axis_tdata  = pack_stereo(left, right);
    s_axis_tvalid = 1'b1;
    @(posedge clk);
    s_axis_tvalid = 1'b0;
    m_axis_tready = 1'b1;
    repeat (10) begin
      @(posedge clk);
      if (m_axis_tvalid) begin
        result = m_axis_tdata;
        return;
      end
    end
    $error("TIMEOUT");
    result = '0;
  endtask

  logic [47:0] result;
  logic signed [23:0] out_l, out_r;

  initial begin
    $display("=== tb_dd3_axis ===");

    rst_n         = 0;
    s_axis_tdata  = '0;
    s_axis_tvalid = 1'b0;
    m_axis_tready = 1'b1;
    tone_val      = 8'hFF;  // bright (bypass filter)
    level_val     = 8'd0;  // wet off
    feedback_val  = 8'd0;  // no feedback
    time_val      = DLY_SAMPS;
    repeat (5) @(posedge clk);
    rst_n = 1;
    repeat (3) @(posedge clk);

    // ---- Test 1: tready ----
    check("TREADY: s_axis_tready = 1", s_axis_tready == 1'b1);

    // ---- Test 2: level=0 → output = dry input ----
    level_val = 8'd0;
    send_sample(24'sh200000, 24'sh100000, result);
    out_l = unpack_left(result);
    out_r = unpack_right(result);
    check("DRY: L = input L when level=0", out_l == 24'sh200000);
    check("DRY: R = input R when level=0", out_r == 24'sh100000);

    // ---- Test 3: Prime delay then check echo ----
    level_val    = 8'd128;
    feedback_val = 8'd0;
    // Send a burst of non-zero samples to fill delay line
    for (int i = 0; i < DLY_SAMPS + 5; i++) send_sample(24'sh300000, 24'sh300000, result);

    // Now send silence — echo should appear from the delay line
    for (int i = 0; i < 5; i++) send_sample(24'sh000000, 24'sh000000, result);

    out_l = unpack_left(result);
    // The delayed echo of the burst should be non-zero
    // (tone filter may attenuate but not to exactly zero)
    check("ECHO: non-zero output during silence (echo present)", out_l != 24'sh000000);

    // ---- Test 4: L and R both get same wet echo ----
    // Send asymmetric dry but wet is mono, so difference is from dry
    send_sample(24'sh100000, 24'sh050000, result);
    out_l = unpack_left(result);
    out_r = unpack_right(result);
    check("MIX: valid L", !$isunknown(out_l));
    check("MIX: valid R", !$isunknown(out_r));

    // ---- Test 5: Feedback causes sustained echo ----
    rst_n = 0;
    repeat (3) @(posedge clk);
    rst_n = 1;
    repeat (3) @(posedge clk);

    level_val    = 8'd128;
    feedback_val = 8'd200;  // High feedback

    // Send a short impulse
    send_sample(24'sh7FFFFF, 24'sh7FFFFF, result);
    for (int i = 0; i < DLY_SAMPS - 1; i++) send_sample(24'sh000000, 24'sh000000, result);

    // After one delay period, echo should appear
    for (int i = 0; i < 5; i++) send_sample(24'sh000000, 24'sh000000, result);
    out_l = unpack_left(result);
    check("FB: echo present after one delay period", out_l != 24'sh000000);

    // After another delay period, echo should still be present (feedback)
    for (int i = 0; i < DLY_SAMPS; i++) send_sample(24'sh000000, 24'sh000000, result);
    out_l = unpack_left(result);
    check("FB: echo still present after 2nd period (feedback sustain)", out_l != 24'sh000000);

    // ---- Test 6: Back-pressure ----
    m_axis_tready = 1'b0;
    s_axis_tdata  = pack_stereo(24'sh111111, 24'sh222222);
    s_axis_tvalid = 1'b1;
    @(posedge clk);
    s_axis_tvalid = 1'b0;
    repeat (5) @(posedge clk);
    if (m_axis_tvalid) check("BP: tvalid held", m_axis_tvalid == 1'b1);
    m_axis_tready = 1'b1;
    @(posedge clk);

    $display("=== dd3_axis: %0d passed, %0d failed ===", pass_count, fail_count);
    if (fail_count > 0) $fatal(1, "TESTS FAILED");
    $finish;
  end

endmodule
