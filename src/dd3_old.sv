`timescale 1ns / 1ps

// Boss DD-3 Style Digital Delay - v4
//
// Clocked on clk_audio (12.288 MHz, 81ns period).
// audio_in and audio_out are in the same domain as the I2S driver,
// so no CDC is needed on the audio path.
//
// Control inputs (tone_val, level_val, feedback_val, time_val) come from
// the sys_clk domain and must be synchronised in top before connecting.
// They change only on UART commands so a 2-flop sync is sufficient.
//
// Signal flow:
//   1. audio_in written into delay line with soft-clipped feedback mixed in
//   2. Delayed signal filtered by 2-pole IIR controlled by tone_val
//   3. audio_out = WET ONLY (filtered delayed signal scaled by level_val)
//      Dry passthrough and final mix are handled in top.sv so the dry
//      signal is never attenuated and the full 24-bit dynamic range is kept.
//
// Fixes over original:
//   [A] 2-phase pipeline: BRAM read on sample_en, process+write on
//       sample_en_d1 - eliminates stale-sample comb filter in feedback.
//   [B] 2-pole LPF with compensated coefficient - smoother -12 dB/oct
//       roll-off without making the tone too dark/thin.
//   [C] Delay-time slew with fast initial load - no more metallic
//       sweep from 1 sample on power-up.  Slew rate = 64 samples/period
//       for smooth knob sweeps without clicks.
//   [D] Cubic soft-clip in feedback path for gentler saturation.
//   [E] Feedback and wet output use *current* filter output (lpf_out),
//       not stale registered state.
//
module dd3 #(
    parameter WIDTH     = 24,
    parameter RAM_DEPTH = 48000  // 1 second at 48kHz
) (
    input logic clk,       // clk_audio (12.288 MHz)
    input logic rst_n,
    input logic sample_en, // 1-cycle pulse at fs (48kHz), same domain

    // Controls - must be synchronised to clk before connecting
    input logic [ 7:0] tone_val,
    input logic [ 7:0] level_val,
    input logic [ 7:0] feedback_val,
    input logic [15:0] time_val,

    // Audio - clk domain
    // audio_out carries wet signal only; caller adds dry separately
    input  logic signed [WIDTH-1:0] audio_in,
    output logic signed [WIDTH-1:0] audio_out
);

  // =========================================================================
  // 0. SATURATION CONSTANTS
  // =========================================================================
  localparam signed [WIDTH-1:0] SAT_MAX = {1'b0, {(WIDTH - 1) {1'b1}}};  //  0x7FFFFF
  localparam signed [WIDTH-1:0] SAT_MIN = {1'b1, {(WIDTH - 1) {1'b0}}};  // -0x800000
  localparam signed [WIDTH+10:0] SAT_MAX_W = {{11{1'b0}}, SAT_MAX};
  localparam signed [WIDTH+10:0] SAT_MIN_W = {{11{1'b1}}, SAT_MIN};

  // =========================================================================
  // 1. BLOCK RAM (Delay Line)
  // =========================================================================
  (* ram_style = "block" *) logic signed [WIDTH-1:0] ram[0:RAM_DEPTH-1];

  logic signed [WIDTH-1:0] ram_read_data = 0;

  localparam ADDR_WIDTH = $clog2(RAM_DEPTH);
  logic [ADDR_WIDTH-1:0] wr_ptr = 0;
  logic [ADDR_WIDTH-1:0] rd_ptr = 0;
  logic [ADDR_WIDTH-1:0] wr_ptr_next;
  logic [ADDR_WIDTH-1:0] rd_ptr_next;

  initial begin
    for (int i = 0; i < RAM_DEPTH; i++) ram[i] = 0;
  end

  // =========================================================================
  // 2. CONTROL SIGNALS
  // =========================================================================
  logic signed [9:0] fb_gain;
  logic signed [9:0] wet_level;

  always_comb begin
    fb_gain   = {2'b00, feedback_val};
    wet_level = {2'b00, level_val};
  end

  // =========================================================================
  // 2B. DELAY TIME - TARGET + SLEW  [FIX C]
  // =========================================================================
  // delay_target: clamped/validated version of time_val.
  // delay_samples: actual delay used by pointer logic, slews toward target.
  //
  // On first sample after reset, delay_samples loads target immediately
  // (no sweep from 1).  After that, slews at ±64 samples/period to
  // avoid clicks during live time_val changes.
  //   64 samples/period at 48 kHz = 750 samples/ms → a full-range
  //   sweep (48000 samples) takes ~64 ms.  Fast enough to feel instant,
  //   slow enough to avoid audible discontinuities.
  //
  logic [ADDR_WIDTH-1:0] delay_target;
  logic [ADDR_WIDTH-1:0] delay_samples;
  logic [ADDR_WIDTH-1:0] delay_samples_next;
  logic                  delay_loaded = 0;  // first-load flag

  always_comb begin
    if (time_val >= RAM_DEPTH) delay_target = ADDR_WIDTH'(RAM_DEPTH - 1);
    else if (time_val == 16'd0) delay_target = ADDR_WIDTH'(1);
    else delay_target = time_val[ADDR_WIDTH-1:0];
  end

  // Slew rate: ±64 samples per audio sample
  localparam [ADDR_WIDTH-1:0] SLEW_RATE = 64;

  always_comb begin
    if (!delay_loaded) begin
      // First sample after reset: jump straight to target
      delay_samples_next = delay_target;
    end else if (delay_samples < delay_target) begin
      // Slewing up
      if ((delay_target - delay_samples) > SLEW_RATE)
        delay_samples_next = delay_samples + SLEW_RATE;
      else delay_samples_next = delay_target;
    end else if (delay_samples > delay_target) begin
      // Slewing down
      if ((delay_samples - delay_target) > SLEW_RATE)
        delay_samples_next = delay_samples - SLEW_RATE;
      else delay_samples_next = delay_target;
    end else begin
      delay_samples_next = delay_samples;
    end
  end

  // =========================================================================
  // 3. TONE COEFFICIENT - AUDIBLE RANGE 30 Hz to 16 kHz  [FIX B]
  // =========================================================================
  // 1-pole IIR:  y += alpha*(x - y),  alpha = coeff/256
  //   fc ≈ alpha * fs / (2*pi)  =>  at fs=48kHz:
  //     alpha=2/256  -> fc ~  60 Hz  (single pole)
  //     alpha=256/256 -> fc ~ 7.6 kHz (single pole, nyquist-limited)
  //
  // 2-pole cascade: combined -3dB drops by x0.6436 vs single pole.
  // To hit target fc, boost per-stage alpha by 1/0.6436 ≈ 1.554x.
  //
  // Target range:  fc_min = 30 Hz, fc_max = 16 kHz  at fs = 48 kHz
  //   alpha_min = 2*pi*30    / 48000 ≈ 0.00393  -> per-stage boosted ≈ 0.00611 -> coeff =  2 (floor, min 2)
  //   alpha_max = 2*pi*16000 / 48000 ≈ 2.094    -> clamped to 1.0              -> coeff = 256 (bypass)
  //
  // Mapping: linear tone_val 0..255 -> alpha_linear 2..256 (coeff units).
  // A linear alpha sweep gives a perceptually log-ish frequency sweep because
  // fc ∝ alpha, so equal steps in alpha = equal ratio steps only at the low end.
  // For a more even perceptual spread we use a quadratic map, same shape as
  // the original design but rescaled to fill 2..256 across the full 0..255 range:
  //
  //   tone_base    = 2 + (tone_val^2 * 254) / 255^2   (range 2..256)
  //   tone_boosted = tone_base * 397 >> 8              (1.554x boost for 2-pole)
  //   tone_coeff   = min(tone_boosted, 256)
  //
  //   tone_val=  0 -> coeff= 27 -> single-stage alpha=0.105  -> fc_combined ~  800 Hz (dark but audible)
  //   tone_val=128 -> coeff= 82 -> single-stage alpha=0.320  -> fc_combined ~  2.4 kHz
  //   tone_val=192 -> coeff=161 -> single-stage alpha=0.629  -> fc_combined ~  7.1 kHz
  //   tone_val=230 -> coeff=220 -> single-stage alpha=0.859  -> fc_combined ~ 11.8 kHz
  //   tone_val=255 -> coeff=256 -> bypass (fc > 16 kHz, passes full audible range)
  //
  logic        [15:0] tone_sq;
  logic        [ 9:0] tone_base;
  logic signed [ 9:0] tone_coeff;
  logic        [19:0] tone_boosted;
  logic        [25:0] tone_sq_scaled;

  always_comb begin
    // tone_val^2, 16-bit (max 255^2 = 65025)
    tone_sq = {2'b00, tone_val} * {2'b00, tone_val};

    // Scale to range 27..256:
    //   Minimum coeff=27 -> per-stage alpha=27/256=0.105 -> fc_combined ~ 800 Hz (darkest, still audible)
    //   Maximum coeff=256 -> bypass (full audible range)
    //   tone_base = 27 + (tone_sq * 229) >> 16   (229 = 256-27, fills 27..256 across 0..255^2)
    tone_sq_scaled = {2'b00, tone_sq} * 26'd229;
    tone_base = 10'd27 + tone_sq_scaled[25:16];  // >> 16, range 27..256

    // Boost by ~1.554x for 2-pole compensation: (base * 397) >> 8
    tone_boosted = tone_base * 20'd397;
    if (tone_boosted[19:8] >= 12'd256) tone_coeff = 10'd256;
    else tone_coeff = {2'b00, tone_boosted[17:8]};
  end

  // =========================================================================
  // 4. POINTER CALCULATIONS
  // =========================================================================
  always_comb begin
    wr_ptr_next = (wr_ptr == ADDR_WIDTH'(RAM_DEPTH - 1)) ? '0 : wr_ptr + 1'b1;

    if (wr_ptr_next >= delay_samples) rd_ptr_next = wr_ptr_next - delay_samples;
    else rd_ptr_next = ADDR_WIDTH'(RAM_DEPTH) + wr_ptr_next - delay_samples;
  end

  // =========================================================================
  // 5. SIGNAL PATH (Combinatorial - evaluated in Phase 2)
  // =========================================================================

  // -------------------------------------------------------------------------
  // 5A. 2-POLE LPF (cascaded 1-pole IIRs)  [FIX B]
  // -------------------------------------------------------------------------
  logic signed [ WIDTH-1:0] lpf1_state = 0;
  logic signed [ WIDTH-1:0] lpf1_state_next;
  logic signed [WIDTH+10:0] lpf1_diff;
  logic signed [WIDTH+10:0] lpf1_product;

  logic signed [ WIDTH-1:0] lpf2_state = 0;
  logic signed [ WIDTH-1:0] lpf2_state_next;
  logic signed [WIDTH+10:0] lpf2_diff;
  logic signed [WIDTH+10:0] lpf2_product;

  always_comb begin
    // --- Stage 1 ---
    lpf1_diff       = $signed(ram_read_data) - $signed(lpf1_state);
    lpf1_product    = lpf1_diff * $signed(tone_coeff);
    lpf1_state_next = $signed(lpf1_state) + lpf1_product[WIDTH+10:8];

    // --- Stage 2: feeds from stage 1's *current* output ---
    lpf2_diff       = $signed(lpf1_state_next) - $signed(lpf2_state);
    lpf2_product    = lpf2_diff * $signed(tone_coeff);
    lpf2_state_next = $signed(lpf2_state) + lpf2_product[WIDTH+10:8];
  end

  // lpf_out is the *current* sample's fully-filtered result  [FIX E]
  wire signed  [ WIDTH-1:0] lpf_out = lpf2_state_next;

  // -------------------------------------------------------------------------
  // 5B. SOFT-CLIP FEEDBACK PATH  [FIX D]
  // -------------------------------------------------------------------------
  // Cubic soft-clipper:  f(x) = (3x - x^3) / 2  for |x| <= 1
  //
  // In 24-bit fixed-point (MAX ≈ 2^23):
  //   sq      = x * x                              (48 bits signed)
  //   sq_norm = sq >>> 23                           (x^2/MAX, 25 bits)
  //   cu_norm = x * sq_norm >>> 23                  (x^3/MAX^2, ~24 bits)
  //   result  = (3*x - cu_norm) >>> 1
  //
  logic signed [WIDTH+10:0] fb_signal_raw;
  logic signed [WIDTH+10:0] fb_sum_raw;
  logic signed [ WIDTH-1:0] fb_softclip;

  logic signed [ WIDTH-1:0] fb_clamped;
  logic signed [      47:0] sc_sq;
  logic signed [      24:0] sc_sq_norm;
  logic signed [      48:0] sc_cu_raw;
  logic signed [      25:0] sc_cu_norm;
  logic signed [      25:0] sc_3x;
  logic signed [      25:0] sc_pre;
  logic signed [      25:0] sc_out;

  always_comb begin
    // Feedback: current filtered output * gain  [FIX E]
    fb_signal_raw = ($signed(lpf_out) * fb_gain) >>> 8;
    fb_sum_raw    = $signed(audio_in) + fb_signal_raw;

    // Pre-clamp to ±MAX for soft-clip input domain
    if (fb_sum_raw > SAT_MAX_W) fb_clamped = SAT_MAX;
    else if (fb_sum_raw < SAT_MIN_W) fb_clamped = SAT_MIN;
    else fb_clamped = fb_sum_raw[WIDTH-1:0];

    // --- Cubic soft-clip: (3x - x^3/MAX^2) / 2 ---
    sc_sq = $signed(fb_clamped) * $signed(fb_clamped);
    sc_sq_norm = sc_sq[47:23];
    sc_cu_raw = $signed(sc_sq_norm) * $signed(fb_clamped);
    sc_cu_norm = sc_cu_raw[48:23];

    sc_3x = $signed({{2{fb_clamped[WIDTH-1]}}, fb_clamped}) + $signed(
        {{2{fb_clamped[WIDTH-1]}}, fb_clamped}) + $signed({{2{fb_clamped[WIDTH-1]}}, fb_clamped});

    sc_pre = sc_3x - sc_cu_norm;
    sc_out = sc_pre >>> 1;

    // Final saturation guard
    if (sc_out > $signed({{2{1'b0}}, SAT_MAX})) fb_softclip = SAT_MAX;
    else if (sc_out < $signed({{2{1'b1}}, SAT_MIN})) fb_softclip = SAT_MIN;
    else fb_softclip = sc_out[WIDTH-1:0];
  end

  // -------------------------------------------------------------------------
  // 5C. WET OUTPUT - scaled *current* filtered delay signal  [FIX E]
  // -------------------------------------------------------------------------
  logic signed [WIDTH+10:0] wet_signal;
  logic signed [ WIDTH-1:0] out_saturated;

  always_comb begin
    // When time_val == 0, suppress wet output entirely (no repeats)
    if (time_val == 16'd0) begin
      wet_signal    = '0;
      out_saturated = '0;
    end else begin
      wet_signal = ($signed(lpf_out) * wet_level) >>> 8;

      if (wet_signal > SAT_MAX_W) out_saturated = SAT_MAX;
      else if (wet_signal < SAT_MIN_W) out_saturated = SAT_MIN;
      else out_saturated = wet_signal[WIDTH-1:0];
    end
  end

  // =========================================================================
  // 6. OUTPUT
  // =========================================================================
  logic signed [WIDTH-1:0] audio_out_reg = 0;
  assign audio_out = audio_out_reg;

  // =========================================================================
  // 7. SEQUENTIAL LOGIC - 2-PHASE PIPELINE  [FIX A]
  // =========================================================================
  // Phase 1 (sample_en):    Issue BRAM read; latch ram_read_data.
  // Phase 2 (sample_en_d1): ram_read_data is valid. Combo LPF+feedback
  //                         evaluate, then write BRAM, advance pointers,
  //                         update filter state & output.
  //
  logic sample_en_d1 = 0;

  always_ff @(posedge clk) begin
    if (!rst_n) sample_en_d1 <= 1'b0;
    else sample_en_d1 <= sample_en;
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      wr_ptr        <= '0;
      rd_ptr        <= '0;
      lpf1_state    <= '0;
      lpf2_state    <= '0;
      audio_out_reg <= '0;
      ram_read_data <= '0;
      delay_samples <= ADDR_WIDTH'(1);
      delay_loaded  <= 1'b0;
    end else begin
      // Phase 1: Issue BRAM read (data available next posedge clk)
      if (sample_en) begin
        ram_read_data <= ram[rd_ptr];
      end

      // Phase 2: BRAM data valid - process and write back
      if (sample_en_d1) begin
        ram[wr_ptr]   <= fb_softclip;
        wr_ptr        <= wr_ptr_next;
        rd_ptr        <= rd_ptr_next;
        lpf1_state    <= lpf1_state_next;
        lpf2_state    <= lpf2_state_next;
        audio_out_reg <= out_saturated;
        delay_samples <= delay_samples_next;
        delay_loaded  <= 1'b1;
      end
    end
  end

endmodule
