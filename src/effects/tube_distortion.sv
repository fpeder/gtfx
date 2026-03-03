// =============================================================================
// tube_distortion.sv
// 12AX7 Tube Distortion  -  Top-level
//
// Pipeline (9 clock-cycle latency):
//   Stage 0  pre-gain multiply (1× to 256×)
//   Stage 1  saturation clamp
//   Stage 2  address + fraction split
//   Stage 3  dual LUT read (y0, y1)  [BRAM registered output]
//   Stage 4  interp: delta = y1 - y0
//   Stage 5  interp: product = delta * frac
//   Stage 6  interp: y_out = y0 + round(product >> 12)
//   Stage 7  3-band Baxandall tone biquads (Bass / Mid / Treble)
//   Stage 8  tone mix + output level scale
//
// Dependencies (compile order):
//   saturate.sv
//   tube_lut_rom.sv
//   linear_interp.sv
//   biquad_tdf2.sv
//   tube_distortion.sv   ← this file
// =============================================================================

`timescale 1ns / 1ps

module tube_distortion #(
    parameter int DATA_W   = 24,
    parameter int COEFF_W  = 24,
    parameter int LUT_ADDR = 12,  // 4096-entry LUT
    parameter int FRAC_W   = 12   // must equal DATA_W - LUT_ADDR
) (
    input logic clk,
    input logic rst_n,
    input logic sample_en, // one pulse per audio sample

    // Audio I/O - 24-bit signed 2's complement
    input  logic signed [DATA_W-1:0] audio_in,
    output logic signed [DATA_W-1:0] audio_out,
    output logic                     valid_out,

    // Controls - unsigned, 0=min 255=max
    input logic [7:0] gain,         // pre-gain: 0=1×  255≈64×
    input logic [7:0] tone_bass,    // bass shelf:   128=flat
    input logic [7:0] tone_mid,     // mid peak:     128=flat
    input logic [7:0] tone_treble,  // treble shelf: 128=flat
    input logic [7:0] level         // output attenuation
);

  // ── Pipeline valid shift register ────────────────────────────────────────
  localparam int PIPE = 9;
  logic [PIPE-1:0] vld;

  always_ff @(posedge clk)
    if (!rst_n) vld <= '0;
    else vld <= {vld[PIPE-2:0], sample_en};

  // =========================================================================
  // STAGE 0 : Pre-gain multiply
  // gain 0→1×, 255→256×  (linear, much gentler than the old shift method)
  // =========================================================================
  localparam int GAIN_W = DATA_W + 9;  // headroom for 24 × 9-bit multiply

  logic signed [GAIN_W-1:0] gained;
  // {0, gain} + 1 → unsigned 1..256, i.e. gain_val=0 → 1×, gain_val=255 → 256×
  wire signed [8:0] gain_mul = {1'b0, gain} + 9'sd1;

  always_ff @(posedge clk)
    if (!rst_n) gained <= '0;
    else if (sample_en) gained <= audio_in * gain_mul;

  // =========================================================================
  // STAGE 1 : Saturation clamp to DATA_W bits
  // =========================================================================
  wire signed [DATA_W-1:0] gained_sat_comb;

  saturate #(
      .IN_W (GAIN_W),
      .OUT_W(DATA_W)
  ) u_sat_gain (
      .din (gained),
      .dout(gained_sat_comb)
  );

  logic signed [DATA_W-1:0] gained_sat;

  always_ff @(posedge clk)
    if (!rst_n) gained_sat <= '0;
    else gained_sat <= gained_sat_comb;

  // =========================================================================
  // STAGE 2 : Split LUT address and interpolation fraction
  //   audio_in[23:12]  →  offset-binary LUT index  (0..4095)
  //   audio_in[11:0]   →  interpolation fraction    (0..4095)
  // =========================================================================
  logic [LUT_ADDR-1:0] addr_a, addr_b;
  logic [  FRAC_W-1:0] lut_frac;

  // XOR flips MSB: 2's complement → offset-binary for symmetric LUT addressing
  wire  [LUT_ADDR-1:0] idx_comb = gained_sat[DATA_W-1:FRAC_W] ^ {1'b1, {(LUT_ADDR - 1) {1'b0}}};

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      addr_a   <= '0;
      addr_b   <= '0;
      lut_frac <= '0;
    end else begin
      addr_a   <= idx_comb;
      addr_b   <= idx_comb + 1'b1;
      lut_frac <= gained_sat[FRAC_W-1:0];
    end
  end

  // =========================================================================
  // STAGE 3 : Dual LUT read  (two instances → dual-port BRAM)
  // =========================================================================
  logic signed [DATA_W-1:0] lut_y0, lut_y1;

  tube_lut_rom #(
      .ADDR_W(LUT_ADDR),
      .DATA_W(DATA_W)
  ) u_rom_y0 (
      .clk (clk),
      .addr(addr_a),
      .dout(lut_y0)
  );
  tube_lut_rom #(
      .ADDR_W(LUT_ADDR),
      .DATA_W(DATA_W)
  ) u_rom_y1 (
      .clk (clk),
      .addr(addr_b),
      .dout(lut_y1)
  );

  // Align fraction with the 1-cycle ROM read latency
  logic [FRAC_W-1:0] frac_d1;

  always_ff @(posedge clk)
    if (!rst_n) frac_d1 <= '0;
    else frac_d1 <= lut_frac;

  // =========================================================================
  // STAGES 4-6 : Pipelined linear interpolation  (3-cycle module)
  // =========================================================================
  logic signed [DATA_W-1:0] interp_out;

  linear_interp #(
      .DATA_W(DATA_W),
      .FRAC_W(FRAC_W)
  ) u_interp (
      .clk  (clk),
      .y0   (lut_y0),
      .y1   (lut_y1),
      .frac (frac_d1),
      .y_out(interp_out)
  );

  // =========================================================================
  // STAGE 7 : 3-band Baxandall tone stack
  //
  // All coefficients Q1.23.  Computed for fs=48 kHz with bilinear transform.
  //   Bass  shelf  fc=250 Hz  Q=0.707
  //   Mid   peak   fc=1 kHz   Q=1.5
  //   Treble shelf fc=4 kHz   Q=0.707
  //
  // a1_neg / a2_neg carry the pre-negated denominator coefficients.
  // =========================================================================

  // Bass shelf (Q1.23)
  localparam signed [23:0] BASS_B0  = 24'sh004C52;  //  0.01876
  localparam signed [23:0] BASS_B1  = 24'sh0098A4;  //  0.03751
  localparam signed [23:0] BASS_B2  = 24'sh004C52;  //  0.01876
  localparam signed [23:0] BASS_A1N = 24'sh7E9EEA;  //  0.98878 (negated a1)
  localparam signed [23:0] BASS_A2N = -24'sh008598;  // -0.02629 (negated a2)

  // Mid peak (Q1.23)
  localparam signed [23:0] MID_B0  = 24'sh045C6B;   //  0.27148
  localparam signed [23:0] MID_B1  = -24'sh08B8D6;  // -0.54297
  localparam signed [23:0] MID_B2  = 24'sh045C6B;   //  0.27148
  localparam signed [23:0] MID_A1N = 24'sh7A6E8F;   //  0.95612 (negated a1)
  localparam signed [23:0] MID_A2N = -24'sh045C6B;  // -0.27148 (negated a2)

  // Treble shelf (Q1.23)
  localparam signed [23:0] TREB_B0  = 24'sh25375F;  //  0.29126
  localparam signed [23:0] TREB_B1  = -24'sh4A6EBE; // -0.58252
  localparam signed [23:0] TREB_B2  = 24'sh25375F;  //  0.29126
  localparam signed [23:0] TREB_A1N = 24'sh64B4A1;  //  0.78707 (negated a1)
  localparam signed [23:0] TREB_A2N = -24'sh1BB8AA; // -0.21672 (negated a2)

  wire signed [8:0] bg = {1'b0, tone_bass}   - 9'sh80;
  wire signed [8:0] mg = {1'b0, tone_mid}    - 9'sh80;
  wire signed [8:0] tg = {1'b0, tone_treble} - 9'sh80;

  wire bq_en = vld[6];

  logic signed [DATA_W-1:0] bass_y, mid_y, treb_y;

  biquad_tdf2 #(
      .DATA_W(DATA_W),
      .COEFF_W(24),
      .FRAC(23)
  ) u_bass (
      .clk(clk),
      .rst_n(rst_n),
      .en(bq_en),
      .x_in(interp_out),
      .y_out(bass_y),
      .b0(BASS_B0),
      .b1(BASS_B1),
      .b2(BASS_B2),
      .a1_neg(BASS_A1N),
      .a2_neg(BASS_A2N)
  );

  biquad_tdf2 #(
      .DATA_W(DATA_W),
      .COEFF_W(24),
      .FRAC(23)
  ) u_mid (
      .clk(clk),
      .rst_n(rst_n),
      .en(bq_en),
      .x_in(interp_out),
      .y_out(mid_y),
      .b0(MID_B0),
      .b1(MID_B1),
      .b2(MID_B2),
      .a1_neg(MID_A1N),
      .a2_neg(MID_A2N)
  );

  biquad_tdf2 #(
      .DATA_W(DATA_W),
      .COEFF_W(24),
      .FRAC(23)
  ) u_treb (
      .clk(clk),
      .rst_n(rst_n),
      .en(bq_en),
      .x_in(interp_out),
      .y_out(treb_y),
      .b0(TREB_B0),
      .b1(TREB_B1),
      .b2(TREB_B2),
      .a1_neg(TREB_A1N),
      .a2_neg(TREB_A2N)
  );

  // =========================================================================
  // STAGE 8 : Tone mix  +  output level scale  +  saturation
  //
  // Each band: (biquad_out * tone_offset) >>> 7
  // Computed in full precision then explicitly truncated to MIX_W before sum.
  // =========================================================================
  localparam int PROD_W = DATA_W + 9 + 9;  // 42 bits: 33-bit sign-ext × 9-bit gain
  localparam int MIX_W  = DATA_W + 4;      // 28 bits is plenty: 24-bit dry + 3 × ~24-bit terms
  localparam int LVL_W  = DATA_W + 8;      // 32 bits

  // Full-precision products, shifted
  wire signed [PROD_W-1:0] bass_prod = ($signed({{9{bass_y[DATA_W-1]}}, bass_y}) * bg) >>> 7;
  wire signed [PROD_W-1:0] mid_prod  = ($signed({{9{mid_y[DATA_W-1]}},  mid_y})  * mg) >>> 7;
  wire signed [PROD_W-1:0] treb_prod = ($signed({{9{treb_y[DATA_W-1]}}, treb_y}) * tg) >>> 7;

  // Saturate each band term to MIX_W before summing
  wire signed [MIX_W-1:0] bass_term, mid_term, treb_term;

  saturate #(.IN_W(PROD_W), .OUT_W(MIX_W)) u_sat_bass (.din(bass_prod), .dout(bass_term));
  saturate #(.IN_W(PROD_W), .OUT_W(MIX_W)) u_sat_mid  (.din(mid_prod),  .dout(mid_term));
  saturate #(.IN_W(PROD_W), .OUT_W(MIX_W)) u_sat_treb (.din(treb_prod), .dout(treb_term));

  wire signed [MIX_W-1:0] mix_raw =
          {{(MIX_W-DATA_W){interp_out[DATA_W-1]}}, interp_out}
        + bass_term
        + mid_term
        + treb_term;

  wire signed [DATA_W-1:0] mix_sat_comb;

  saturate #(
      .IN_W (MIX_W),
      .OUT_W(DATA_W)
  ) u_sat_mix (
      .din (mix_raw),
      .dout(mix_sat_comb)
  );

  // Level: Q1.AF × Q0.8 (unsigned level) → Q1.(AF+8), [LVL_W-1:8] extracts Q1.AF
  wire signed [LVL_W-1:0] level_scaled = {{8{mix_sat_comb[DATA_W-1]}}, mix_sat_comb} * $signed(
      {1'b0, level}
  );

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      audio_out <= '0;
      valid_out <= 1'b0;
    end else begin
      audio_out <= level_scaled[LVL_W-1:8];
      valid_out <= vld[PIPE-1];
    end
  end

endmodule : tube_distortion