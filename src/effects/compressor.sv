`timescale 1ns / 1ps

// ============================================================================
// compressor.sv - Dynamic Range Compressor Effect Core
//
// Peak envelope follower with gain reduction above threshold.
// Smooth gain changes via attack/release time constants.
// Makeup gain boosts output to compensate for compression.
//
// Parameters (from cfg_slice):
//   [0] threshold  - compression threshold (0-255)
//   [1] ratio      - compression ratio (0=1:1, 255=∞:1)
//   [2] attack     - attack time (0=instant, 255=slow)
//   [3] release    - release time (0=fast, 255=slow)
//   [4] makeup     - makeup gain (0=0dB, 255=+24dB approx)
//   [7] bypass     - bypass control (bit 0)
//
// Algorithm:
//   1. Peak envelope follower tracks signal level
//   2. When envelope > threshold: gain = threshold / envelope (approx)
//   3. Smooth gain via attack/release
//   4. Apply gain × makeup to output
//
// Latency: 1 cycle (sample_en → audio_out registered)
// DSP cost: ~3-4 DSP48E1
// ============================================================================

module compressor #(
    parameter int DATA_W = 24,
    parameter int CTRL_W = 8
) (
    input logic clk,
    input logic rst_n,
    input logic sample_en,

    input  logic signed [DATA_W-1:0] audio_in,
    output logic signed [DATA_W-1:0] audio_out,

    input logic [CTRL_W-1:0] threshold_val,
    input logic [CTRL_W-1:0] ratio_val,
    input logic [CTRL_W-1:0] attack_val,
    input logic [CTRL_W-1:0] release_val,
    input logic [CTRL_W-1:0] makeup_val
);

  // =========================================================================
  // Constants
  // =========================================================================
  localparam int ENV_W = DATA_W;
  localparam int GAIN_W = 16;        // Q0.16 gain
  localparam int GAIN_UNITY = (1 << GAIN_W) - 1;
  localparam int THR_SHIFT = ENV_W - CTRL_W;  // 16

  // =========================================================================
  // Envelope follower (peak with separate attack/release smoothing)
  // =========================================================================
  logic [ENV_W-1:0] envelope;
  logic [ENV_W-1:0] abs_in;

  always_comb begin
    abs_in = audio_in[DATA_W-1] ? ENV_W'(-audio_in) : ENV_W'(audio_in);
  end

  // Envelope attack/release coefficients
  // attack: env rises at rate proportional to 1/(1+attack_val*4)
  // release: env falls at rate proportional to 1/(1+release_val*4)
  logic [ENV_W-1:0] env_attack_delta;
  logic [ENV_W-1:0] env_release_delta;

  always_comb begin
    // Fast attack: difference / (1 + attack_val)
    if (abs_in > envelope)
      env_attack_delta = (abs_in - envelope) >>> (attack_val[CTRL_W-1:5]);  // 3-bit shift = /1../8
    else
      env_attack_delta = '0;

    // Release: env * (1/(256+release_val*16))
    env_release_delta = envelope >>> (4 + release_val[CTRL_W-1:5]);  // /16../128
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      envelope <= '0;
    end else if (sample_en) begin
      if (abs_in > envelope) begin
        // Attack
        if (attack_val == '0)
          envelope <= abs_in;  // instant attack
        else
          envelope <= envelope + env_attack_delta;
      end else begin
        // Release
        if (envelope > env_release_delta)
          envelope <= envelope - env_release_delta;
        else
          envelope <= '0;
      end
    end
  end

  // =========================================================================
  // Gain computation
  //
  // Above threshold: reduce gain proportional to (1 - 1/ratio)
  // gain_reduction = (envelope - threshold) * ratio_factor / envelope
  // target_gain = GAIN_UNITY - gain_reduction_scaled
  //
  // Simplified approach: when envelope > threshold,
  //   target_gain = GAIN_UNITY * threshold / envelope  (for inf:1 ratio)
  //   Blend between unity and full compression based on ratio_val
  // =========================================================================
  logic [ENV_W-1:0] threshold_scaled;
  logic above_threshold;
  logic [GAIN_W-1:0] target_gain;

  always_comb begin
    threshold_scaled = ENV_W'(threshold_val) << THR_SHIFT;
    above_threshold = (envelope > threshold_scaled) && (threshold_scaled != '0);
  end

  // Compute compressed gain: threshold / envelope (Q0.GAIN_W)
  // Use division approximation: threshold << GAIN_W / envelope
  logic [ENV_W+GAIN_W-1:0] div_num;
  logic [GAIN_W-1:0] compressed_gain;

  always_comb begin
    div_num = {threshold_scaled, {GAIN_W{1'b0}}};
    if (envelope == '0 || !above_threshold)
      compressed_gain = GAIN_W'(GAIN_UNITY);
    else
      compressed_gain = GAIN_W'(div_num / {{GAIN_W{1'b0}}, envelope});
  end

  // Blend between unity gain and compressed gain based on ratio_val
  // ratio_val=0 → unity (no compression), 255 → full compression
  always_comb begin
    if (!above_threshold) begin
      target_gain = GAIN_W'(GAIN_UNITY);
    end else begin
      // target = unity * (255-ratio)/256 + compressed * ratio/256
      target_gain = GAIN_W'(
        (32'(GAIN_UNITY) * 32'(8'(255) - ratio_val) +
         32'(compressed_gain) * 32'(ratio_val)) >> 8
      );
    end
  end

  // =========================================================================
  // Smooth gain (simple 1-pole filter on gain)
  // =========================================================================
  logic [GAIN_W-1:0] smooth_gain;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      smooth_gain <= GAIN_W'(GAIN_UNITY);
    end else if (sample_en) begin
      if (target_gain < smooth_gain) begin
        // Gain reducing (attack phase of compressor)
        smooth_gain <= smooth_gain - ((smooth_gain - target_gain) >>> (attack_val[CTRL_W-1:5]));
      end else begin
        // Gain recovering (release phase)
        smooth_gain <= smooth_gain + ((target_gain - smooth_gain) >>> (3 + release_val[CTRL_W-1:5]));
      end
    end
  end

  // =========================================================================
  // Makeup gain: 1.0x to ~4.0x
  // makeup_val: 0 → 256 (1.0x), 255 → 1024 (4.0x) in Q8.8
  // =========================================================================
  localparam int MAKEUP_W = 16;
  logic [MAKEUP_W-1:0] makeup_gain;

  always_comb begin
    // 256 + makeup_val * 3 → range 256 (1.0x) to 1021 (~4.0x) in Q8.8
    makeup_gain = MAKEUP_W'(16'd256 + 16'(makeup_val) * 16'd3);
  end

  // =========================================================================
  // Apply gain + makeup
  //
  // out = audio_in * smooth_gain / GAIN_UNITY * makeup_gain / 256
  // =========================================================================
  localparam int PROD_W = DATA_W + GAIN_W;
  logic signed [PROD_W-1:0] gained;
  logic signed [PROD_W-1:0] madeup;
  logic signed [DATA_W-1:0] final_out;

  always_comb begin
    // Apply compression gain: Q1.(DATA_W-1) × Q0.GAIN_W >> GAIN_W
    gained = (longint'(audio_in) * longint'(signed'({1'b0, smooth_gain}))) >>> GAIN_W;

    // Apply makeup: × Q8.8 >> 8
    madeup = (gained * longint'(signed'({1'b0, makeup_gain}))) >>> 8;
  end

  // Saturate to DATA_W
  saturate #(
      .IN_W (PROD_W),
      .OUT_W(DATA_W)
  ) u_sat (
      .din (madeup),
      .dout(final_out)
  );

  // =========================================================================
  // Registered output
  // =========================================================================
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      audio_out <= '0;
    end else if (sample_en) begin
      audio_out <= final_out;
    end
  end

endmodule
