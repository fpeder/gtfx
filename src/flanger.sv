`timescale 1ns / 1ps

// ============================================================================
// flanger.sv - MXR M117R-style Flanger Effect Core
//
// Interface matches the existing effect core pattern:
//   sample_en strobe, signed audio_in/audio_out, 8-bit cfg_slice registers.
//
// Register map (via cfg_slice from ctrl_bus):
//   [0] = manual   (8-bit, base delay:  0x00=min, 0xFF=max)
//   [1] = width    (8-bit, LFO depth:   0x00=none, 0xFF=full)
//   [2] = speed    (8-bit, LFO rate:    0x00=slowest, 0xFF=fastest)
//   [3] = regen    (8-bit, feedback:    0x00=-max, 0x80=zero, 0xFF=+max)
//   [7] = bypass   (bit 0, managed by axis_effect_slot)
//
// Signal path (models the original MXR M117R):
//
//                     +-------- feedback (regen) --------+
//                     |                                  |
//   audio_in -->(+)--> delay_line --> interp --> LPF -->--+--> (+) --> audio_out
//                                               wet           dry+wet / 2
//
// The biquad_tdf2 models the SAD1024 BBD's reconstruction low-pass (~6 kHz
// at 48 kHz Fs).  All saturation uses the shared `saturate` module.
//
// Delay range: ~0.25 ms - ~5 ms at 48 kHz (12-240 samples)
// LFO: triangle wave, ~0.05-11 Hz
// Feedback: signed (negative = through-zero flanging)
//
// Latency: 1 cycle (sample_en -> audio_out registered)
//   axis_effect_slot uses effect_valid = sample_en_d1, same as tremolo etc.
//
// Pipeline (all happens in a SINGLE sample_en cycle):
//   sample_en edge:
//     - LFO advances (registered)
//     - delay_mem[wr_ptr] written with (audio_in + feedback) (feedback from
//       previous sample's wet_filtered -- one-sample loop delay, same as analog)
//     - tap_a, tap_b read COMBINATIONALLY (same cycle)
//     - wet_interp computed combinationally from taps
//     - biquad_tdf2 clocked: latches wet_interp, outputs wet_filtered (from
//       PREVIOUS sample's computation -- inherent 1-sample biquad delay)
//     - mix = (audio_in + wet_filtered) / 2, saturated, registered to audio_out
//
// This means feedback and the LPF both operate with a 1-sample delay in the
// loop, which is correct -- the analog circuit has a similar propagation delay
// through the BBD + reconstruction filter, and at audio rates (48 kHz) one
// sample of loop delay is inaudible.
// ============================================================================

module flanger #(
    parameter int WIDTH     = 24,
    parameter int MAX_DELAY = 240,  // 5 ms @ 48 kHz
    parameter int MIN_DELAY = 12,   // 0.25 ms @ 48 kHz
    parameter int LFO_ACC_W = 28,   // LFO phase accumulator width
    parameter int FRAC_W    = 10    // fractional delay bits for interpolation
) (
    input logic clk,
    input logic rst_n,
    input logic sample_en,

    // Audio
    input  logic signed [WIDTH-1:0] audio_in,
    output logic signed [WIDTH-1:0] audio_out,

    // Control registers (8-bit each, directly from cfg_slice)
    input logic [7:0] manual_val,  // cfg_slice[0]
    input logic [7:0] width_val,   // cfg_slice[1]
    input logic [7:0] speed_val,   // cfg_slice[2]
    input logic [7:0] regen_val    // cfg_slice[3] (0x80 = centre / zero)
);

  // =========================================================================
  // Local parameters
  // =========================================================================
  localparam int DELAY_RANGE = MAX_DELAY - MIN_DELAY;  // 228
  localparam int ADDR_W = $clog2(MAX_DELAY);

  // LFO max phase increment: ~11 Hz at 48 kHz
  localparam int LFO_INC_MAX = int'(11.0 * (2.0 ** LFO_ACC_W) / 48000.0);

  // Internal width with headroom for feedback summation / dry-wet mix
  localparam int INT_W = WIDTH + 2;

  // =========================================================================
  // BBD reconstruction low-pass filter coefficients
  // =========================================================================
  // 2nd-order Butterworth LPF, Fc = 6 kHz, Fs = 48 kHz
  //
  //   b0 =  0.09763107  b1 =  0.19526215  b2 =  0.09763107
  //   a1 = -0.94280904  a2 =  0.33333333
  //
  // Quantised to Q2.22 (COEFF_W = 24, FRAC = 22):
  //   Range ±2.0, sufficient for all coefficients (|a1| < 1.0).
  //   Previous Q1.23 overflowed on a1_neg (~0.94 × 2^23 > 8388607).
  localparam int COEFF_W = 24;
  localparam int FRAC = 22;

  localparam logic signed [COEFF_W-1:0] LPF_B0     =  24'sd409494;   //  0.09763107 * 2^22
  localparam logic signed [COEFF_W-1:0] LPF_B1     =  24'sd818989;   //  0.19526215 * 2^22
  localparam logic signed [COEFF_W-1:0] LPF_B2     =  24'sd409494;   //  0.09763107 * 2^22
  localparam logic signed [COEFF_W-1:0] LPF_A1_NEG =  24'sd3954428;  //  0.94280904 * 2^22
  localparam logic signed [COEFF_W-1:0] LPF_A2_NEG = -24'sd1398101;  // -0.33333333 * 2^22

  // =========================================================================
  // LFO - Triangle wave via phase accumulator
  // =========================================================================
  logic [LFO_ACC_W-1:0] lfo_acc;
  logic [LFO_ACC_W-1:0] lfo_inc;
  logic [          7:0] lfo_tri;  // unsigned 8-bit triangle output

  assign lfo_inc = LFO_ACC_W'((int'(speed_val) * LFO_INC_MAX) >> 8);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) lfo_acc <= '0;
    else if (sample_en) lfo_acc <= lfo_acc + lfo_inc;
  end

  // Triangle shaping: MSB selects rising/falling half
  always_comb begin
    logic [LFO_ACC_W-2:0] ramp;
    ramp = lfo_acc[LFO_ACC_W-2:0];
    if (lfo_acc[LFO_ACC_W-1]) lfo_tri = ~ramp[LFO_ACC_W-2-:8];  // falling
    else lfo_tri = ramp[LFO_ACC_W-2-:8];  // rising
  end

  // =========================================================================
  // Delay time calculation
  // =========================================================================
  logic [ADDR_W+FRAC_W-1:0] base_delay_frac;
  logic [ADDR_W+FRAC_W-1:0] lfo_mod_frac;
  logic [ADDR_W+FRAC_W:0] total_delay_raw;
  logic [ADDR_W+FRAC_W-1:0] total_delay_clamped;

  logic [ADDR_W-1:0] delay_int;
  logic [FRAC_W-1:0] delay_frac;

  always_comb begin
    base_delay_frac = (ADDR_W+FRAC_W)'(
            (MIN_DELAY << FRAC_W) +
            ((int'(manual_val) * DELAY_RANGE) << FRAC_W) / 256
        );

    // Use longint (64-bit) to avoid overflow:
    // worst case: 255 * 255 * 228 * 1024 = ~15 billion, exceeds 32-bit int
    lfo_mod_frac = (ADDR_W+FRAC_W)'(
            (longint'(width_val) * longint'(lfo_tri) * longint'(DELAY_RANGE) * longint'(1 << FRAC_W)) / (256 * 256)
        );

    total_delay_raw = (ADDR_W+FRAC_W+1)'(signed'({1'b0, base_delay_frac}) +
                           signed'({1'b0, lfo_mod_frac}) -
                           signed'({1'b0, (ADDR_W+FRAC_W)'(
                               (longint'(width_val) * longint'(DELAY_RANGE) * longint'(1 << FRAC_W)) / (256 * 2)
                           )}));

    if (total_delay_raw[ADDR_W+FRAC_W])
      total_delay_clamped = (ADDR_W + FRAC_W)'(MIN_DELAY << FRAC_W);
    else if (total_delay_raw > (ADDR_W + FRAC_W + 1)'(MAX_DELAY << FRAC_W))
      total_delay_clamped = (ADDR_W + FRAC_W)'(MAX_DELAY << FRAC_W);
    else total_delay_clamped = total_delay_raw[ADDR_W+FRAC_W-1:0];

    delay_int  = total_delay_clamped[FRAC_W+:ADDR_W];
    delay_frac = total_delay_clamped[FRAC_W-1:0];
  end

  // =========================================================================
  // Feedback (REGEN) - signed
  // =========================================================================
  // regen_val 0x00 = -max, 0x80 = zero, 0xFF = +max
  logic signed [8:0] regen_signed;
  assign regen_signed = signed'({1'b0, regen_val}) - 9'sd128;

  // =========================================================================
  // Delay line (circular buffer) - COMBINATIONAL reads
  // =========================================================================
  // Key difference from previous version: tap reads are combinational
  // (not registered) so wet_interp is available in the same cycle as
  // sample_en.  This keeps the flanger at 1-cycle latency like tremolo
  // and phaser, matching the slot wrapper's expectation.
  // =========================================================================
  logic signed [ WIDTH-1:0] delay_mem[0:MAX_DELAY-1];
  logic        [ADDR_W-1:0] wr_ptr;

  logic [ADDR_W-1:0] rd_addr_a, rd_addr_b;
  logic signed [WIDTH-1:0] tap_a, tap_b;
  logic signed [WIDTH-1:0] wet_interp;

  // Read addresses (circular subtraction modulo MAX_DELAY)
  // MAX_DELAY (240) is not power-of-two, so binary subtraction wraps at
  // 256 - addresses 240-255 are stale/invalid.  Must wrap explicitly.
  always_comb begin
    if (wr_ptr >= delay_int)
      rd_addr_a = wr_ptr - delay_int;
    else
      rd_addr_a = wr_ptr + ADDR_W'(MAX_DELAY) - delay_int;

    if (wr_ptr >= delay_int + ADDR_W'(1))
      rd_addr_b = wr_ptr - delay_int - ADDR_W'(1);
    else
      rd_addr_b = wr_ptr + ADDR_W'(MAX_DELAY) - delay_int - ADDR_W'(1);
  end

  // Combinational tap reads
  assign tap_a = delay_mem[rd_addr_a];
  assign tap_b = delay_mem[rd_addr_b];

  // =========================================================================
  // Linear interpolation:  wet = tap_a + frac * (tap_b - tap_a)
  // =========================================================================
  logic signed [       WIDTH:0] interp_diff;
  logic signed [WIDTH+FRAC_W:0] interp_prod;

  always_comb begin
    interp_diff = signed'({tap_b[WIDTH-1], tap_b}) - signed'({tap_a[WIDTH-1], tap_a});
    interp_prod = interp_diff * signed'({1'b0, delay_frac});
    wet_interp  = tap_a + WIDTH'(interp_prod >>> FRAC_W);
  end

  // =========================================================================
  // BBD reconstruction low-pass filter (biquad_tdf2)
  // =========================================================================
  // Enabled on sample_en.  On the rising edge:
  //   - x_in  = wet_interp (combinationally valid this cycle)
  //   - y_out = wet_filtered (result from PREVIOUS sample -- 1-sample delay)
  //
  // This is correct: the biquad's inherent 1-sample latency models the
  // analog reconstruction filter's propagation delay.  wet_filtered used
  // in the feedback path and output mix is the previous sample's filtered
  // output, which is stable and valid when sample_en fires.
  // =========================================================================
  logic signed [WIDTH-1:0] wet_filtered;

  biquad_tdf2 #(
      .DATA_W (WIDTH),
      .COEFF_W(COEFF_W),
      .FRAC   (FRAC)
  ) u_bbd_lpf (
      .clk   (clk),
      .rst_n (rst_n),
      .en    (sample_en),
      .x_in  (wet_interp),
      .y_out (wet_filtered),
      .b0    (LPF_B0),
      .b1    (LPF_B1),
      .b2    (LPF_B2),
      .a1_neg(LPF_A1_NEG),
      .a2_neg(LPF_A2_NEG)
  );

  // =========================================================================
  // Feedback path
  // =========================================================================
  // feedback uses wet_filtered which is the PREVIOUS sample's biquad output
  // (stable before sample_en edge).  This 1-sample feedback delay is
  // identical to the analog circuit's propagation through BBD + filter.
  //
  // feedback = wet_filtered * regen_signed / 128
  //   → max gain ≈ ±0.99 at regen extremes (127/128)

  logic signed [INT_W-1:0] feedback_scaled;
  logic signed [INT_W-1:0] delay_input_wide;

  always_comb begin
    feedback_scaled  = INT_W'((longint'(wet_filtered) * longint'(regen_signed)) >>> 7);
    delay_input_wide = INT_W'(signed'(audio_in)) + feedback_scaled;
  end

  // --- Piecewise-linear soft-clip on delay input (models BBD/opamp saturation) ---
  //
  // The analog MXR M117R soft-clips at the BBD input summing node, NOT on
  // feedback alone.  We apply soft-clip to (audio_in + feedback) before the
  // delay write.  This prevents the hard-edge clicks that a pure saturator
  // produces at high regen settings.
  //
  // Transfer function (signed, symmetric):
  //   |x| ≤ T     :  y = x                         (gain = 1.0, linear)
  //   T < |x|     :  y = sign(x) · (T + (|x|-T)/2) (gain = 0.5, compressed)
  //   final clamp :  |y| ≤ PEAK                      (safety net)
  //
  // T = 0.5 · full-scale = 2^(WIDTH-2).  The 50% knee is conservative but
  // eliminates audible hard-clip artefacts.  The 2:1 compression above the
  // knee keeps the signal musically usable at high regen.
  // Implementation: one compare, one subtract, one shift, one add.

  localparam signed [WIDTH-1:0] CLIP_KNEE = (1 << (WIDTH - 2));  // 0.5 * full-scale
  localparam signed [WIDTH-1:0] CLIP_PEAK = {1'b0, {(WIDTH-1){1'b1}}};  // +max

  logic signed [WIDTH-1:0] delay_input_sat;

  always_comb begin
    logic signed [INT_W-1:0] x;
    logic        [INT_W-2:0] x_abs;
    logic        [INT_W-2:0] y_abs;

    x     = delay_input_wide;
    x_abs = x[INT_W-1] ? INT_W'(-x) : x;

    if (x_abs <= INT_W'(CLIP_KNEE)) begin
      // Linear region
      delay_input_sat = x[WIDTH-1:0];
    end else begin
      // Compressed region: knee + (excess >> 1)
      y_abs = (INT_W-1)'(CLIP_KNEE) + ((x_abs - (INT_W-1)'(CLIP_KNEE)) >> 1);
      // Hard clamp as safety net
      if (y_abs > (INT_W-1)'(CLIP_PEAK))
        y_abs = (INT_W-1)'(CLIP_PEAK);
      delay_input_sat = x[INT_W-1] ? -WIDTH'(y_abs) : WIDTH'(y_abs);
    end
  end

  // Write delay line and advance pointer
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wr_ptr <= '0;
      for (int i = 0; i < MAX_DELAY; i++) delay_mem[i] <= '0;
    end else if (sample_en) begin
      delay_mem[wr_ptr] <= delay_input_sat;
      wr_ptr <= (wr_ptr == ADDR_W'(MAX_DELAY - 1)) ? '0 : wr_ptr + ADDR_W'(1);
    end
  end

  // =========================================================================
  // Output mix:  (dry + wet_filtered) / 2
  // =========================================================================
  // Uses wet_filtered (previous sample's biquad output, stable) and
  // audio_in (current sample, combinationally valid).
  // Both are valid at sample_en time.

  logic signed [INT_W-1:0] mix;
  logic signed [WIDTH-1:0] mix_sat;

  always_comb begin
    mix = (INT_W'(signed'(audio_in)) + INT_W'(signed'(wet_filtered))) >>> 1;
  end

  saturate #(
      .IN_W (INT_W),
      .OUT_W(WIDTH)
  ) u_sat_mix (
      .din (mix),
      .dout(mix_sat)
  );

  // =========================================================================
  // Registered output  (1-cycle latency, same as tremolo / phaser)
  // =========================================================================
  // audio_out is registered on sample_en.  The slot wrapper reads it on
  // effect_valid = sample_en_d1 (one cycle later), at which point it is
  // stable.  This matches the exact timing contract of all other 1-cycle
  // effect cores.

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) audio_out <= '0;
    else if (sample_en) audio_out <= mix_sat;
  end

endmodule