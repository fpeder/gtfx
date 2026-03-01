// =============================================================================
// tube_distortion.sv
// 12AX7 Tube Distortion  -  Top-level
//
// Pipeline (9 clock-cycle latency):
//   Stage 0  pre-gain multiply (NOT shift - avoids huge overdrives)
//   Stage 1  saturation clamp
//   Stage 2  address + fraction split
//   Stage 3  dual LUT read (y0, y1)  [BRAM registered output]
//   Stage 4  interp: delta = y1 - y0
//   Stage 5  interp: product = delta * frac
//   Stage 6  interp: y_out = y0 + round(product >> 12)
//   Stage 7  3-band Baxandall tone biquads (Bass / Mid / Treble)
//   Stage 8  tone mix + output level scale
//
// FIX LOG:
//   - Pre-gain changed from arithmetic shift to multiply. The original
//     gain_shift of 1..64 bits on a 24-bit signal meant gain[7:2]>=7
//     shifted all signal bits out, leaving only the saturated rail
//     (±full-scale square wave = noise).  Now uses a 1×..64× multiply
//     which gives a smooth, usable gain range.
//   - Output level scaling unchanged (was correct in logic, just masked
//     by the destroyed signal from stage 0).
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
    input logic [7:0] gain,         // pre-gain: 0≈1×  255≈64×
    input logic [7:0] tone_bass,    // bass shelf:   128=flat
    input logic [7:0] tone_mid,     // mid peak:     128=flat
    input logic [7:0] tone_treble,  // treble shelf: 128=flat
    input logic [7:0] level         // output attenuation
);

  // ── Pipeline valid shift register ────────────────────────────────────────
  localparam int PIPE = 9;
  logic [PIPE-1:0] vld;

  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) vld <= '0;
    else vld <= {vld[PIPE-2:0], sample_en};

  // =========================================================================
  // STAGE 0 : Pre-gain via MULTIPLY  (not shift!)
  //
  //   gain_mult = gain[7:2] + 1  →  range 1 .. 64
  //   gained    = audio_in * gain_mult
  //
  // A multiply by 1..64 is the correct analogue of a preamp gain of
  // 0 dB .. ~36 dB, plenty for tube distortion.  The original shift-based
  // approach produced unusable results because <<< 7 on a 24-bit value
  // already puts the signal at ±2^30, saturating instantly → square wave.
  // =========================================================================
  localparam int GAIN_W = DATA_W + 7;  // 31-bit headroom for product

  logic signed [GAIN_W-1:0] gained;
  wire  [6:0] gain_mult = {1'b0, gain[7:2]} + 7'd1;  // 1 .. 64

  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) gained <= '0;
    else if (sample_en) gained <= audio_in * $signed({1'b0, gain_mult});

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

  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) gained_sat <= '0;
    else gained_sat <= gained_sat_comb;

  // =========================================================================
  // STAGE 2 : Split LUT address and interpolation fraction
  //   gained_sat[23:12]  →  offset-binary LUT index  (0..4095)
  //   gained_sat[11:0]   →  interpolation fraction    (0..4095)
  // =========================================================================
  logic [LUT_ADDR-1:0] addr_a, addr_b;
  logic [  FRAC_W-1:0] lut_frac;

  // Flip MSB to convert signed → offset-binary for the LUT
  wire  [LUT_ADDR-1:0] idx_comb = gained_sat[DATA_W-1:FRAC_W] ^ {1'b1, {(LUT_ADDR - 1) {1'b0}}};

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      addr_a   <= '0;
      addr_b   <= '0;
      lut_frac <= '0;
    end else begin
      addr_a   <= idx_comb;
      // Clamp addr_b at max to avoid LUT wrap-around at full-scale
      addr_b   <= (idx_comb == {LUT_ADDR{1'b1}}) ? idx_comb : idx_comb + 1'b1;
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

  always_ff @(posedge clk or negedge rst_n)
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

  // Bass shelf
  localparam signed [23:0] BASS_B0 = 24'sh004C52;
  localparam signed [23:0] BASS_B1 = 24'sh0098A4;
  localparam signed [23:0] BASS_B2 = 24'sh004C52;
  localparam signed [23:0] BASS_A1N = 24'sh7E9EEA;
  localparam signed [23:0] BASS_A2N = -24'sh008598;

  // Mid peak
  localparam signed [23:0] MID_B0 = 24'sh045C6B;
  localparam signed [23:0] MID_B1 = -24'sh08B8D6;
  localparam signed [23:0] MID_B2 = 24'sh045C6B;
  localparam signed [23:0] MID_A1N = 24'sh7A6E8F;
  localparam signed [23:0] MID_A2N = -24'sh045C6B;

  // Treble shelf
  localparam signed [23:0] TREB_B0 = 24'sh25375F;
  localparam signed [23:0] TREB_B1 = -24'sh4A6EBE;
  localparam signed [23:0] TREB_B2 = 24'sh25375F;
  localparam signed [23:0] TREB_A1N = 24'sh64B4A1;
  localparam signed [23:0] TREB_A2N = -24'sh1BB8AA;

  // Tone band gains: 128=0 dB, 0=-12 dB, 255=+12 dB  (Q1.7 signed offset)
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
  // mix = dry + bg*bass + mg*mid + tg*treble   (Q1.7 gains → shift 7)
  // out = mix * level / 256                    (Q0.8 scale  → shift 8)
  // =========================================================================
  localparam int MIX_W = DATA_W + 9;  // 33 bits - overflow guard
  localparam int LVL_W = DATA_W + 8;  // 32 bits

  wire signed [MIX_W-1:0] mix_raw =
          {{9{interp_out[DATA_W-1]}}, interp_out}
        + (({{9{bass_y[DATA_W-1]}}, bass_y} * bg) >>> 7)
        + (({{9{mid_y[DATA_W-1]}},  mid_y}  * mg) >>> 7)
        + (({{9{treb_y[DATA_W-1]}}, treb_y} * tg) >>> 7);

  wire signed [DATA_W-1:0] mix_sat_comb;

  saturate #(
      .IN_W (MIX_W),
      .OUT_W(DATA_W)
  ) u_sat_mix (
      .din (mix_raw),
      .dout(mix_sat_comb)
  );

  wire signed [LVL_W-1:0] level_scaled = {{8{mix_sat_comb[DATA_W-1]}}, mix_sat_comb} * $signed(
      {1'b0, level}
  );

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      audio_out <= '0;
      valid_out <= 1'b0;
    end else begin
      audio_out <= level_scaled[LVL_W-1:8];
      valid_out <= vld[PIPE-1];
    end
  end

endmodule : tube_distortion