// =============================================================================
// tube_distortion_tb.sv
// Testbench for tube_distortion top-level module
// Drives 1 kHz sine wave at 80% FS, verifies tube asymmetry signature.
// =============================================================================

`timescale 1ns / 1ps

module tb_tube_distortion;

  localparam int CLK_PER = 10;  // 100 MHz
  localparam int FS_CYCLES = 2083;  // 48 kHz tick @ 100 MHz

  // ── DUT signals ──────────────────────────────────────────────────────────
  logic               clk = 0;
  logic               rst_n = 0;
  logic               sample_en = 0;
  logic               valid_out;
  logic signed [23:0] audio_in;
  logic signed [23:0] audio_out;

  logic        [ 7:0] gain = 8'd80;
  logic        [ 7:0] tone_bass = 8'd128;
  logic        [ 7:0] tone_mid = 8'd128;
  logic        [ 7:0] tone_treble = 8'd128;
  logic        [ 7:0] level = 8'd200;

  initial audio_in = '0;

  // ── Clock ─────────────────────────────────────────────────────────────────
  always #(CLK_PER / 2) clk = ~clk;

  // ── 48 kHz sample enable ─────────────────────────────────────────────────
  int sample_cnt = 0;
  always_ff @(posedge clk) begin
    sample_cnt++;
    sample_en <= (sample_cnt == FS_CYCLES);
    if (sample_cnt == FS_CYCLES) sample_cnt <= 0;
  end

  // ── DUT instantiation ────────────────────────────────────────────────────
  tube_distortion dut (.*);

  // ── 1 kHz sine stimulus at 80% FS ────────────────────────────────────────
  real phase = 0.0;
  always_ff @(posedge clk)
    if (sample_en) begin
      audio_in <= $rtoi($sin(phase) * real'(2**22) * 0.8);
      phase    <= phase + 2.0 * 3.14159265358979 * 1000.0 / 48000.0;
    end

  // ── Asymmetry accumulator ────────────────────────────────────────────────
  // Tube asymmetry produces unequal positive/negative half-cycle magnitudes.
  // Expect >2% difference from the LUT's DC bias (+0.20).
  longint pos_acc = 0;
  longint neg_acc = 0;
  int     out_cnt = 0;
  longint diff_abs = 0;

  always_ff @(posedge clk) begin
    if (valid_out) begin
      out_cnt <= out_cnt + 1;
      if (audio_out > 0) pos_acc <= pos_acc + longint'(audio_out);
      else neg_acc <= neg_acc - longint'(audio_out);
    end
  end

  // ── Print first 10 output samples ────────────────────────────────────────
  int disp_cnt = 0;
  always_ff @(posedge clk)
    if (valid_out && disp_cnt < 10) begin
      $display("[%3d]  in=%12d  out=%12d", disp_cnt, audio_in, audio_out);
      disp_cnt <= disp_cnt + 1;
    end

  // ── Main stimulus + pass/fail check ──────────────────────────────────────
  initial begin
    $dumpfile("tube_distortion.vcd");
    $dumpvars(0, tb_tube_distortion);

    // Release reset after a few clocks
    #(CLK_PER * 15) rst_n = 1;

    // Run 200 ms worth of audio (9600 samples @ 48 kHz)
    repeat (9600) @(posedge clk iff sample_en);

    // ── Asymmetry check ──────────────────────────────────────────────────
    diff_abs = (pos_acc > neg_acc) ? pos_acc - neg_acc : neg_acc - pos_acc;

    $display("---------------------------------------------------");
    $display("Asymmetry check:");
    $display("  pos_acc = %0d", pos_acc);
    $display("  neg_acc = %0d", neg_acc);
    $display("  diff    = %0d%%", diff_abs * 100 / ((pos_acc + neg_acc) / 2 + 1));

    if (diff_abs * 100 / ((pos_acc + neg_acc) / 2 + 1) > 2)
      $display("  PASS – tube asymmetry confirmed (even harmonics present)");
    else $display("  WARN – low asymmetry, verify LUT bias");

    $display("---------------------------------------------------");
    $display("Simulation complete. out_cnt = %0d", out_cnt);
    $finish;
  end

endmodule
