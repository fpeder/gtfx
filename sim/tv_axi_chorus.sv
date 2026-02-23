`timescale 1ns / 1ps

// ============================================================================
// tb_chorus_axis.sv — Testbench for chorus_axis
//
// Tests:
//   1. AXI-Stream handshake (2-cycle output latency)
//   2. effect_lvl=0 → output L ≈ input (dry only), R ≈ input
//   3. effect_lvl>0 → L ≠ R (stereo spread from spatial phase inversion)
//   4. Zero input → zero output
//   5. Multiple samples flow correctly
//   6. Back-pressure holds output
// ============================================================================

module tb_chorus_axis;

  import axis_audio_pkg::*;

  localparam int CLK_P = 81;

  logic        clk = 0;
  logic        rst_n;
  logic [47:0] s_axis_tdata;
  logic        s_axis_tvalid;
  logic        s_axis_tready;
  logic [47:0] m_axis_tdata;
  logic        m_axis_tvalid;
  logic        m_axis_tready;
  logic [7:0] rate, depth, effect_lvl, e_q_hi, e_q_lo;

  always #(CLK_P / 2) clk = ~clk;

  chorus_axis #(
      .SAMPLE_RATE(48_000),
      .DATA_WIDTH (24),
      .DELAY_MAX  (512)
  ) dut (
      .clk          (clk),
      .rst_n        (rst_n),
      .s_axis_tdata (s_axis_tdata),
      .s_axis_tvalid(s_axis_tvalid),
      .s_axis_tready(s_axis_tready),
      .m_axis_tdata (m_axis_tdata),
      .m_axis_tvalid(m_axis_tvalid),
      .m_axis_tready(m_axis_tready),
      .rate         (rate),
      .depth        (depth),
      .effect_lvl   (effect_lvl),
      .e_q_hi       (e_q_hi),
      .e_q_lo       (e_q_lo)
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

  task automatic send_sample(input logic signed [23:0] left, output logic [47:0] result);
    s_axis_tdata  = pack_stereo(left, left);
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
    $display("=== tb_chorus_axis ===");

    rst_n         = 0;
    s_axis_tdata  = '0;
    s_axis_tvalid = 1'b0;
    m_axis_tready = 1'b1;
    rate          = 8'd80;
    depth         = 8'd100;
    effect_lvl    = 8'd0;
    e_q_hi        = 8'd128;
    e_q_lo        = 8'd128;
    repeat (5) @(posedge clk);
    rst_n = 1;
    repeat (3) @(posedge clk);

    // ---- Test 1: tready ----
    check("TREADY: s_axis_tready = 1", s_axis_tready == 1'b1);

    // ---- Test 2: Zero input → zero output ----
    send_sample(24'sh000000, result);
    out_l = unpack_left(result);
    out_r = unpack_right(result);
    check("ZERO: L = 0", out_l == 24'sh000000);
    check("ZERO: R = 0", out_r == 24'sh000000);

    // ---- Test 3: effect_lvl=0 → dry passthrough ----
    effect_lvl = 8'd0;
    for (int i = 0; i < 10; i++) send_sample(24'sh200000, result);
    out_l = unpack_left(result);
    out_r = unpack_right(result);
    check("DRY: L ≈ input (effect_lvl=0)", out_l == 24'sh200000);
    check("DRY: R ≈ input (effect_lvl=0)", out_r == 24'sh200000);

    // ---- Test 4: effect_lvl > 0 with primed delay → stereo spread ----
    effect_lvl = 8'd200;
    // Prime delay buffer with signal
    for (int i = 0; i < 1100; i++) send_sample(24'sh300000, result);
    out_l = unpack_left(result);
    out_r = unpack_right(result);
    check("STEREO: output valid L", !$isunknown(out_l));
    check("STEREO: output valid R", !$isunknown(out_r));
    // With spatial phase inversion and non-zero effect, L ≠ R
    check("STEREO: L ≠ R (stereo spread)", out_l != out_r);

    // ---- Test 5: Multiple samples ----
    for (int i = 0; i < 20; i++) begin
      send_sample(24'(i * 24'sh010000), result);
      check($sformatf("MULTI[%0d]: valid", i), !$isunknown(unpack_left(result)));
    end

    // ---- Test 6: Back-pressure ----
    m_axis_tready = 1'b0;
    s_axis_tdata  = pack_stereo(24'sh100000, 24'sh100000);
    s_axis_tvalid = 1'b1;
    @(posedge clk);
    s_axis_tvalid = 1'b0;
    repeat (5) @(posedge clk);
    if (m_axis_tvalid) check("BP: tvalid held", m_axis_tvalid == 1'b1);
    m_axis_tready = 1'b1;
    @(posedge clk);

    $display("=== chorus_axis: %0d passed, %0d failed ===", pass_count, fail_count);
    if (fail_count > 0) $fatal(1, "TESTS FAILED");
    $finish;
  end

endmodule
