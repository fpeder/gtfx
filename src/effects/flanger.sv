`timescale 1ns / 1ps

// ============================================================================
// flanger.sv - MXR M117R-style Flanger Effect Core
//
// Fixed-point format summary (parametric):
//   Audio:        signed  Q1.(WIDTH-1)           e.g. Q1.23
//   LFO signed:   signed  Q1.(LFO_W-1)           e.g. Q1.15
//   lfo_tri:      unsigned Q0.CTRL_W              [0, 255] for CTRL_W=8
//   Regen:        signed  Q1.(CTRL_W-1)           centre-offset, ±127/128 max
//   Feedback:     Q1.AUDIO_FRAC × Q1.(CTRL_W-1)  >> (CTRL_W-1) → Q1.AUDIO_FRAC
//   Biquad:       Q2.COEFF_FRAC coefficients      e.g. Q2.22
//
// Controls (CTRL_W-bit unsigned, default 8):
//   manual_val, width_val, speed_val, regen_val, mix_val

module flanger #(
    parameter int WIDTH      = 24,   // Audio sample width (signed Q1.(WIDTH-1))
    parameter int CTRL_W     = 8,    // Control knob width (unsigned)
    parameter int LFO_W      = 16,   // LFO output width (signed Q1.(LFO_W-1))
    parameter int MAX_DELAY  = 256,  // 5.33 ms @ 48 kHz  (must be power of 2)
    parameter int MIN_DELAY  = 16,   // 0.33 ms @ 48 kHz
    parameter int LFO_ACC_W  = 28,   // LFO phase accumulator width
    parameter int FRAC_W     = 10,   // Fractional delay bits for interpolation
    parameter int COEFF_W    = 18,   // Biquad coefficient width (18b fits 1 DSP48E1)
    parameter int COEFF_FRAC = 16    // Biquad coefficient fractional bits (Q2.16)
) (
    input logic clk,
    input logic rst_n,
    input logic sample_en,

    input  logic signed [WIDTH-1:0] audio_in,
    output logic signed [WIDTH-1:0] audio_out,

    input logic [CTRL_W-1:0] manual_val,
    input logic [CTRL_W-1:0] width_val,
    input logic [CTRL_W-1:0] speed_val,
    input logic [CTRL_W-1:0] regen_val,
    input logic [CTRL_W-1:0] mix_val
);

  // =========================================================================
  // Q-Format Constants
  // =========================================================================
  localparam int AUDIO_FRAC   = WIDTH - 1;                  // Q1.(WIDTH-1) frac bits
  localparam int LFO_FRAC     = LFO_W - 1;                  // Q1.(LFO_W-1) frac bits
  localparam int LFO_PEAK     = (1 << LFO_FRAC) - 1;        // Max positive signed LFO
  localparam int CTRL_MAX     = (1 << CTRL_W) - 1;           // Max control value
  localparam int CTRL_HALF    = 1 << (CTRL_W - 1);           // Centre value (128)
  localparam int REGEN_FRAC   = CTRL_W - 1;                  // Regen gain Q1.(CTRL_W-1)

  // Feedback product: Q1.AUDIO_FRAC × Q1.REGEN_FRAC → Q2.(AUDIO_FRAC+REGEN_FRAC)
  //   >> REGEN_FRAC → Q1.AUDIO_FRAC with 1 extra integer bit
  localparam int FB_PROD_W    = WIDTH + CTRL_W;

  // Internal width with headroom for feedback summation / dry-wet mix
  localparam int INT_W = WIDTH + 2;

  localparam int DELAY_RANGE = MAX_DELAY - MIN_DELAY;
  localparam int ADDR_W      = $clog2(MAX_DELAY);

  // =========================================================================
  // BBD Reconstruction LPF coefficients (Q2.COEFF_FRAC)
  //
  // 2nd-order Butterworth, Fc = 12 kHz (fs/4), Fs = 48 kHz
  // Quantised to Q2.COEFF_FRAC (range ±2.0):
  localparam logic signed [COEFF_W-1:0] LPF_B0     =  COEFF_W'(int'( 0.29289 * (2.0**COEFF_FRAC)));
  localparam logic signed [COEFF_W-1:0] LPF_B1     =  COEFF_W'(int'( 0.58579 * (2.0**COEFF_FRAC)));
  localparam logic signed [COEFF_W-1:0] LPF_B2     =  COEFF_W'(int'( 0.29289 * (2.0**COEFF_FRAC)));
  localparam logic signed [COEFF_W-1:0] LPF_A1_NEG =  COEFF_W'(int'( 0.0     * (2.0**COEFF_FRAC)));
  localparam logic signed [COEFF_W-1:0] LPF_A2_NEG = -COEFF_W'(int'( 0.17157 * (2.0**COEFF_FRAC)));

  // =========================================================================
  // LFO - Triangle wave via lfo_core
  //
  // Unified formula: inc = INC_BASE + (speed_val * INC_SCALE) >> INC_SHIFT
  //   INC_BASE  = 0, INC_SCALE = LFO_INC_MAX, INC_SHIFT = 8 (CTRL_W)
  // Output: wave_unsigned Q0.LFO_W >> LFO_TO_CTRL_SHIFT → Q0.CTRL_W
  // =========================================================================
  localparam int LFO_INC_MAX = int'(11.0 * (2.0 ** LFO_ACC_W) / 48000.0);
  localparam int LFO_TO_CTRL_SHIFT = LFO_W - CTRL_W;       // e.g. 16 - 8 = 8

  logic [LFO_W-1:0]  lfo_unsigned;                          // Q0.LFO_W
  logic [CTRL_W-1:0] lfo_tri;                               // Q0.CTRL_W unsigned

  lfo_core #(
      .PHASE_W   (LFO_ACC_W),
      .CTRL_W    (CTRL_W),
      .LFO_W     (LFO_W),
      .INC_BASE  (0),
      .INC_SCALE (LFO_INC_MAX),
      .INC_SHIFT (CTRL_W),
      .TABLE_BITS(8),
      .WAVE_TYPE ("TRIANGLE")
  ) u_flanger_lfo (
      .clk          (clk),
      .rst_n        (rst_n),
      .sample_en    (sample_en),
      .rate_val     (speed_val),
      .phase_out    (),
      .wave_signed  (),
      .wave_unsigned(lfo_unsigned)
  );

  assign lfo_tri = CTRL_W'(lfo_unsigned >> LFO_TO_CTRL_SHIFT);

  // =========================================================================
  // Delay time calculation
  // =========================================================================
  logic [ADDR_W+FRAC_W-1:0] base_delay_frac;
  logic [ADDR_W+FRAC_W-1:0] lfo_mod_frac;
  logic [ADDR_W+FRAC_W:0]   total_delay_raw;
  logic [ADDR_W+FRAC_W-1:0] total_delay_clamped;
  logic [ADDR_W-1:0]        delay_int;
  logic [FRAC_W-1:0]        delay_frac;

  always_comb begin
    // Base delay: MIN_DELAY + manual_val/256 × DELAY_RANGE (fixed-point with FRAC_W frac bits)
    base_delay_frac = (ADDR_W+FRAC_W)'(
        (MIN_DELAY << FRAC_W) +
        ((int'(manual_val) * DELAY_RANGE) << FRAC_W) / (1 << CTRL_W)
    );

    // LFO modulation: width_val/256 × lfo_tri/256 × DELAY_RANGE (fixed-point)
    lfo_mod_frac = (ADDR_W+FRAC_W)'(
        (longint'(width_val) * longint'(lfo_tri) * longint'(DELAY_RANGE) * longint'(1 << FRAC_W))
        / ((1 << CTRL_W) * (1 << CTRL_W))
    );

    // Combine: base + LFO − half-width offset (centres sweep around manual tap)
    total_delay_raw = (ADDR_W+FRAC_W+1)'(
        signed'({1'b0, base_delay_frac}) +
        signed'({1'b0, lfo_mod_frac}) -
        signed'({1'b0, (ADDR_W+FRAC_W)'(
            (longint'(width_val) * longint'(DELAY_RANGE) * longint'(1 << FRAC_W))
            / ((1 << CTRL_W) * 2)
        )})
    );

    // Clamp to [MIN_DELAY, MAX_DELAY] (negative = underflow via MSB check)
    if (total_delay_raw[ADDR_W+FRAC_W])
      total_delay_clamped = (ADDR_W+FRAC_W)'(MIN_DELAY << FRAC_W);
    else if (total_delay_raw > (ADDR_W+FRAC_W+1)'(MAX_DELAY << FRAC_W))
      total_delay_clamped = (ADDR_W+FRAC_W)'(MAX_DELAY << FRAC_W);
    else
      total_delay_clamped = total_delay_raw[ADDR_W+FRAC_W-1:0];

    // Split into integer sample count and fractional interpolation bits
    delay_int  = total_delay_clamped[FRAC_W +: ADDR_W];
    delay_frac = total_delay_clamped[FRAC_W-1:0];
  end

  // =========================================================================
  // Feedback (REGEN) - signed Q1.(CTRL_W-1)
  //
  // regen_val 0x00 = -max, 0x80 = zero, 0xFF = +max
  // regen_signed = regen_val - CTRL_HALF → range [-128, +127] → Q1.REGEN_FRAC
  // Max gain = ±127/128 ≈ ±0.99
  // =========================================================================
  logic signed [CTRL_W:0] regen_signed;                     // Q1.REGEN_FRAC
  assign regen_signed = signed'({1'b0, regen_val}) - (CTRL_W+1)'(CTRL_HALF);

  // =========================================================================
  // Delay line - combinational reads + interpolation
  // =========================================================================
  logic [ADDR_W-1:0]        wr_ptr;
  logic [ADDR_W-1:0]        rd_ptr_a, rd_ptr_b;
  logic signed [WIDTH-1:0]  tap_a, tap_b;                   // Q1.AUDIO_FRAC
  logic signed [WIDTH-1:0]  wet_interp;                      // Q1.AUDIO_FRAC

  // PoT depth: natural unsigned subtraction wraps correctly
  always_comb begin
    rd_ptr_a = wr_ptr - delay_int;
    rd_ptr_b = wr_ptr - delay_int - ADDR_W'(1);
  end

  delay_line #(
      .DATA_W    (WIDTH),
      .DEPTH     (MAX_DELAY),
      .NUM_TAPS  (2),
      .INTERP_EN (1),
      .FRAC_W    (FRAC_W)
  ) u_delay (
      .clk           (clk),
      .rst_n         (rst_n),
      .wr_en         (sample_en),
      .wr_data       (delay_input_sat),
      .wr_ptr_o      (wr_ptr),
      .wr_ptr_next_o (),
      .rd_en         (1'b0),
      .rd_ptr        ('{rd_ptr_a, rd_ptr_b}),
      .rd_data       ('{tap_a, tap_b}),
      .interp_frac   (delay_frac),
      .interp_out    (wet_interp)
  );

  // =========================================================================
  // BBD Reconstruction LPF (biquad_tdf2)
  // =========================================================================
  logic signed [WIDTH-1:0] wet_filtered;                     // Q1.AUDIO_FRAC

  biquad_tdf2 #(
      .DATA_W (WIDTH),
      .COEFF_W(COEFF_W),
      .FRAC   (COEFF_FRAC)
  ) u_bbd_lpf (
      .clk   (clk),   .rst_n(rst_n), .en(sample_en),
      .x_in  (wet_interp), .y_out(wet_filtered),
      .b0(LPF_B0), .b1(LPF_B1), .b2(LPF_B2),
      .a1_neg(LPF_A1_NEG), .a2_neg(LPF_A2_NEG)
  );

  // =========================================================================
  // Feedback path
  //
  // Q1.AUDIO_FRAC × Q1.REGEN_FRAC → Q2.(AUDIO_FRAC+REGEN_FRAC)
  //   >> REGEN_FRAC → Q1.AUDIO_FRAC (with 1 extra integer bit)
  // =========================================================================
  logic signed [INT_W-1:0] feedback_scaled;                  // Q1.AUDIO_FRAC + guard
  logic signed [INT_W-1:0] delay_input_wide;                 // Q1.AUDIO_FRAC + guard

  always_comb begin
    feedback_scaled  = INT_W'((longint'(wet_filtered) * longint'(regen_signed)) >>> REGEN_FRAC);
    delay_input_wide = INT_W'(signed'(audio_in)) + feedback_scaled;
  end

  // =========================================================================
  // Soft-clip (piecewise-linear, models BBD/opamp saturation)
  //
  // Knee at 0.5 full-scale = 2^(WIDTH-2), 2:1 compression above knee.
  // =========================================================================
  logic signed [WIDTH-1:0] delay_input_sat;                  // Q1.AUDIO_FRAC

  soft_clip #(
      .IN_W      (INT_W),
      .OUT_W     (WIDTH),
      .COMP_SHIFT(1)       // 2:1 compression
  ) u_soft_clip (
      .din (delay_input_wide),
      .dout(delay_input_sat)
  );

  // =========================================================================
  // Output mix: dry + wet_scaled  (additive — body always preserved)
  //
  // mix_val=0x00: pure dry
  // mix_val=0x60: dry + 37.5% wet (default)
  // mix_val=0xFF: dry + ~100% wet (maximum depth)
  // =========================================================================
  logic signed [INT_W-1:0] wet_scaled;                       // Q1.AUDIO_FRAC + guard
  logic signed [INT_W-1:0] mix;                              // Q1.AUDIO_FRAC + guard
  logic signed [WIDTH-1:0] mix_sat;                          // Q1.AUDIO_FRAC

  always_comb begin
    wet_scaled = INT_W'((longint'(wet_filtered) * longint'(mix_val)
                         + longint'(1 << (CTRL_W-1))) >>> CTRL_W);
    mix = INT_W'(signed'(audio_in)) + wet_scaled;
  end

  saturate #(.IN_W(INT_W), .OUT_W(WIDTH)) u_sat_mix (
      .din(mix), .dout(mix_sat)
  );

  // =========================================================================
  // Registered output
  // =========================================================================
  always_ff @(posedge clk) begin
    if (!rst_n)          audio_out <= '0;
    else if (sample_en)  audio_out <= mix_sat;
  end

endmodule