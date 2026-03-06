`timescale 1ns / 1ps

// ============================================================================
// wah.sv - Wah / Auto-Wah Effect Core
//
// State-variable filter (SVF) with peaked-lowpass response (bandpass peak +
// lowpass body) — more efficient than
// biquad for swept filters (only 2 multiplies for frequency and damping).
//
// Three modes:
//   0 = Manual:   freq_val sets filter frequency directly
//   1 = Auto LFO: LFO sweeps frequency. freq_val = rate, depth_val = range
//   2 = Envelope: envelope follower tracks pick dynamics (Cry Baby style)
//                 freq_val[7:5] = sensitivity, depth_val = sweep range,
//                 decay_val[7:5] = decay speed
//
// Frequency range: ~200 Hz – 2.5 kHz (wide wah sweep)
//
// Parameters (from cfg_slice):
//   [0] freq       - manual frequency / LFO rate / envelope sensitivity (0-255)
//   [1] resonance  - filter Q (0=mild, 255=near self-oscillation)
//   [2] depth      - auto mode sweep depth (0=none, 255=full F_MIN→F_MAX)
//   [3] mode       - 0=manual, 1=LFO auto, 2/3=envelope
//   [4] mix        - dry/wet blend (0=dry, 255=full wet)
//   [5] decay      - envelope decay speed (mode 2 only)
//   [7] bypass     - bypass control (bit 0)
//
// Latency: 1 cycle (sample_en → audio_out registered)
// DSP cost: ~4 DSP48E1 (SVF freq, SVF damping, BP gain comp, mix)
// ============================================================================

module wah #(
    parameter int DATA_W = 24,
    parameter int CTRL_W = 8
) (
    input logic clk,
    input logic rst_n,
    input logic sample_en,

    input  logic signed [DATA_W-1:0] audio_in,
    output logic signed [DATA_W-1:0] audio_out,

    input logic [CTRL_W-1:0] freq_val,
    input logic [CTRL_W-1:0] resonance_val,
    input logic [CTRL_W-1:0] depth_val,
    input logic [CTRL_W-1:0] mode_val,
    input logic [CTRL_W-1:0] mix_val,
    input logic [CTRL_W-1:0] decay_val
);

  // =========================================================================
  // Constants
  // =========================================================================
  localparam int INT_W = DATA_W + 4;  // internal width with headroom

  // SVF coefficient width: Q2.14 (16-bit, fits DSP48E1)
  localparam int COEFF_W = 16;
  localparam int COEFF_FRAC = 14;

  // Frequency coefficient range: maps to ~200 Hz – 2.5 kHz @ 48 kHz
  // f_coeff = 2 * sin(pi * f / fs)
  // At 200 Hz:  2 * sin(pi * 200/48000)  ≈ 0.0262 → 429 in Q2.14
  // At 2500 Hz: 2 * sin(pi * 2500/48000) ≈ 0.3249 → 5324 in Q2.14
  localparam int F_MIN = 429;    // ~200 Hz
  localparam int F_MAX = 5324;   // ~2500 Hz
  localparam int F_RANGE = F_MAX - F_MIN;

  // =========================================================================
  // LFO (for auto mode)
  // =========================================================================
  localparam int LFO_W = 16;
  logic [LFO_W-1:0] lfo_unsigned;

  lfo_core #(
      .PHASE_W   (32),
      .CTRL_W    (CTRL_W),
      .LFO_W     (LFO_W),
      .INC_BASE  (17895),    // ~0.2 Hz at rate=0
      .INC_SCALE (803406),   // up to ~9 Hz at rate=255
      .INC_SHIFT (8),
      .TABLE_BITS(8),
      .WAVE_TYPE ("SINE")
  ) u_wah_lfo (
      .clk          (clk),
      .rst_n        (rst_n),
      .sample_en    (sample_en),
      .rate_val     (freq_val),
      .phase_out    (),
      .wave_signed  (),
      .wave_unsigned(lfo_unsigned)
  );

  // =========================================================================
  // Envelope follower (for mode 2 — Cry Baby style)
  //
  // Leaky peak detector: attack (>>>4, ~0.8ms), variable decay.
  // decay_val[7:5] maps to shift 9–16 (~22ms to ~1.4s @ 48 kHz).
  // Zero DSP48E1 cost — shifts and adds only.
  // =========================================================================
  logic [DATA_W-1:0] abs_in;
  logic [DATA_W-1:0] env_level;

  // Absolute value of audio input
  always_comb begin
    abs_in = audio_in[DATA_W-1] ? DATA_W'(-audio_in) : DATA_W'(audio_in);
  end

  // Decay shift: decay_val[7:5] → shift of 9..16
  logic [3:0] decay_shift;
  always_comb begin
    decay_shift = 4'd9 + 4'(decay_val[7:5]);
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      env_level <= '0;
    end else if (sample_en) begin
      if (abs_in > env_level) begin
        // Attack: close gap by 1/16 per sample (~0.8ms to 90%)
        env_level <= env_level + ((abs_in - env_level) >>> 4);
      end else begin
        // Variable decay
        env_level <= env_level - (env_level >>> decay_shift);
      end
    end
  end

  // =========================================================================
  // Frequency control selection
  // =========================================================================
  logic [COEFF_W-1:0] f_target;
  logic [CTRL_W-1:0] lfo_scaled;
  logic [CTRL_W-1:0] sweep_val;
  logic [CTRL_W-1:0] env_scaled;

  // Normalize envelope to 0–255 using sensitivity from freq_val[7:5]
  // Sensitivity shift: 8 + freq_val[7:5] → shifts 8..15
  logic [3:0] sens_shift;
  always_comb begin
    sens_shift = 4'd8 + 4'(freq_val[7:5]);
  end

  always_comb begin
    lfo_scaled = '0;
    sweep_val  = '0;
    env_scaled = '0;

    case (mode_val[1:0])
      2'b00: begin
        // Manual: freq_val directly maps to frequency
        f_target = COEFF_W'(F_MIN + (int'(freq_val) * F_RANGE) / 256);
      end
      2'b01: begin
        // Auto LFO: LFO sweeps frequency, depth_val scales range
        lfo_scaled = lfo_unsigned[LFO_W-1:LFO_W-CTRL_W];                // Q0.8: 0–255
        sweep_val  = CTRL_W'((int'(depth_val) * int'(lfo_scaled)) / 256); // depth-scaled
        f_target   = COEFF_W'(F_MIN + (int'(sweep_val) * F_RANGE) / 256);
      end
      default: begin
        // Envelope follower (modes 2 & 3): pick dynamics → freq sweep
        // Normalize envelope to 0–255, clamp
        if ((env_level >>> sens_shift) > DATA_W'(255))
          env_scaled = 8'hFF;
        else
          env_scaled = CTRL_W'(env_level >>> sens_shift);
        // Apply depth scaling
        sweep_val  = CTRL_W'((int'(depth_val) * int'(env_scaled)) / 256);
        f_target   = COEFF_W'(F_MIN + (int'(sweep_val) * F_RANGE) / 256);
      end
    endcase
  end

  // =========================================================================
  // Frequency smoother — one-pole LPF on f_target
  //
  // Prevents abrupt f_coeff changes from exciting SVF transients (the main
  // cause of envelope-mode squealing). >>>3 gives ~8-sample time constant
  // (~0.17 ms) — transparent for manual/LFO, tames envelope jumps.
  // =========================================================================
  logic [COEFF_W-1:0] f_coeff;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      f_coeff <= F_MIN[COEFF_W-1:0];
    end else if (sample_en) begin
      f_coeff <= COEFF_W'(int'(f_coeff) + ((int'(f_target) - int'(f_coeff)) >>> 3));
    end
  end

  // =========================================================================
  // Damping coefficient from resonance
  //
  // damping = 2 * (1 - Q_factor)
  // resonance_val=0 → damping ≈ 0.5 (Q ≈ 2, mild wah)
  // resonance_val=255 → damping ≈ 0.2 (Q ≈ 5, sharp resonant peak)
  //
  // Q2.14: damping = 8192 (0.5) down to 3277 (0.2)
  // =========================================================================
  localparam int D_MAX = 8192;   // 0.5 in Q2.14 → Q_min ≈ 2
  localparam int D_MIN = 3277;   // 0.2 in Q2.14 → Q_max ≈ 5
  localparam int D_RANGE = D_MAX - D_MIN;

  logic [COEFF_W-1:0] d_coeff;

  always_comb begin
    // Higher resonance_val = lower damping = higher Q
    d_coeff = COEFF_W'(D_MAX - (int'(resonance_val) * D_RANGE) / 256);
  end

  // =========================================================================
  // State Variable Filter (SVF)
  //
  // Classic SVF topology:
  //   hp = x - lp - d*bp
  //   bp += f * hp
  //   lp += f * bp
  //
  // All in Q1.(DATA_W-1) with Q2.COEFF_FRAC coefficients
  // =========================================================================
  logic signed [DATA_W-1:0] svf_lp;   // lowpass output
  logic signed [DATA_W-1:0] svf_bp;   // bandpass output (wah output)
  logic signed [INT_W-1:0]  svf_hp;   // highpass (internal)

  logic signed [INT_W-1:0] f_bp_prod;
  logic signed [INT_W-1:0] d_bp_prod;
  logic signed [INT_W-1:0] bp_next;
  logic signed [INT_W-1:0] lp_next;

  always_comb begin
    // d * bp: Q2.COEFF_FRAC × Q1.(DATA_W-1) >> COEFF_FRAC
    d_bp_prod = INT_W'((longint'(signed'({1'b0, d_coeff})) * longint'(svf_bp)) >>> COEFF_FRAC);

    // hp = x - lp - d*bp
    svf_hp = INT_W'(signed'(audio_in)) - INT_W'(signed'(svf_lp)) - d_bp_prod;

    // f * hp: Q2.COEFF_FRAC × Q1.(INT_W-1) >> COEFF_FRAC
    f_bp_prod = INT_W'((longint'(signed'({1'b0, f_coeff})) * longint'(svf_hp)) >>> COEFF_FRAC);

    // bp_next = bp + f * hp
    bp_next = INT_W'(signed'(svf_bp)) + f_bp_prod;

    // lp_next = lp + f * bp (using current bp, not bp_next)
    lp_next = INT_W'(signed'(svf_lp)) +
              INT_W'((longint'(signed'({1'b0, f_coeff})) * longint'(svf_bp)) >>> COEFF_FRAC);
  end

  // Saturate bp and lp back to DATA_W
  logic signed [DATA_W-1:0] bp_sat, lp_sat;

  saturate #(.IN_W(INT_W), .OUT_W(DATA_W)) u_sat_bp (.din(bp_next), .dout(bp_sat));
  saturate #(.IN_W(INT_W), .OUT_W(DATA_W)) u_sat_lp (.din(lp_next), .dout(lp_sat));

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      svf_bp <= '0;
      svf_lp <= '0;
    end else if (sample_en) begin
      svf_bp <= bp_sat;
      svf_lp <= lp_sat;
    end
  end

  // =========================================================================
  // Gain-compensated bandpass
  //
  // SVF bandpass gain ≈ Q at resonance. Scale by 2*d_coeff (≈ 2/Q) to tame
  // the resonant peak while keeping wah character (~2× boost at all Q).
  // 2*d_coeff ≤ 1.0 so result always fits in DATA_W — no saturate needed.
  // =========================================================================
  logic signed [DATA_W-1:0] bp_comp;

  always_comb begin
    bp_comp = DATA_W'((longint'(svf_bp) * longint'(signed'({1'b0, d_coeff}))) >>> (COEFF_FRAC - 1));
  end

  // =========================================================================
  // Wah wet signal: bandpass peak + lowpass body (peaked-lowpass response)
  // Adding 50% of LP preserves bass body like a real inductor-wah circuit.
  // =========================================================================
  logic signed [INT_W-1:0] wah_wet_wide;
  logic signed [DATA_W-1:0] wah_wet;

  always_comb begin
    wah_wet_wide = INT_W'(signed'(bp_comp)) + (INT_W'(signed'(svf_lp)) >>> 1);
  end

  saturate #(.IN_W(INT_W), .OUT_W(DATA_W)) u_sat_wah (.din(wah_wet_wide), .dout(wah_wet));

  // =========================================================================
  // Dry/wet mix
  //
  // mix_val=0: pure dry, 255: pure wet (peaked lowpass)
  // =========================================================================
  localparam int MIX_W = DATA_W + CTRL_W + 1;
  logic signed [MIX_W-1:0] mix_sum;
  logic signed [MIX_W-1:0] mix_shifted;
  logic signed [DATA_W-1:0] mix_out;

  always_comb begin
    mix_sum = MIX_W'(longint'(audio_in) * longint'(9'(255) - 9'(mix_val))
            + longint'(wah_wet)         * longint'({1'b0, mix_val}));
    mix_shifted = mix_sum >>> CTRL_W;
  end

  saturate #(.IN_W(MIX_W), .OUT_W(DATA_W)) u_sat_mix (
      .din (mix_shifted),
      .dout(mix_out)
  );

  // =========================================================================
  // Registered output
  // =========================================================================
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      audio_out <= '0;
    end else if (sample_en) begin
      audio_out <= mix_out;
    end
  end

endmodule
