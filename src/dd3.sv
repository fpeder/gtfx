`timescale 1ns / 1ps

// Boss DD-3 Style Digital Delay - v5
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
//   1. audio_in mixed with filtered+softclipped feedback, written into delay line
//   2. Delayed signal filtered by 2-pole IIR controlled by tone_val
//   3. DC-blocking HPF removes sub-audible buildup in feedback loop
//   4. Subtle LFO modulation on delay time breaks up comb-filter metallic ringing
//   5. audio_out = WET ONLY (filtered delayed signal scaled by level_val)
//      Dry passthrough and final mix are handled in top.sv
//
// Improvements over v4:
//   [F] LFO modulation on delay read pointer - breaks up comb-filter
//       resonances that cause metallic/ringy repeats. Depth is subtle
//       (~±4 samples at 48kHz = ±83us, like real bucket-brigade jitter).
//   [G] DC-blocking high-pass filter in feedback path prevents low-frequency
//       buildup that muddies repeats and creates odd tonal artifacts.
//   [H] Asymmetric soft-clip with even-harmonic content for warmer,
//       more analog-like saturation character.
//   [I] Feedback filtering happens INSIDE the loop (write path), so each
//       repeat progressively loses highs - matching real analog delay behavior.
//       The tone filter on the wet output is a separate, gentler rolloff.
//   [J] Interpolated delay-line readout for smooth modulation (linear interp).
//
module dd3 #(
    parameter WIDTH     = 24,
    parameter RAM_DEPTH = 32768  // ~682ms at 48kHz
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
  localparam signed [WIDTH-1:0] SAT_MAX = {1'b0, {(WIDTH - 1) {1'b1}}};
  localparam signed [WIDTH-1:0] SAT_MIN = {1'b1, {(WIDTH - 1) {1'b0}}};

  // =========================================================================
  // 1. BLOCK RAM (Delay Line)
  // =========================================================================
  (* ram_style = "block" *) logic signed [WIDTH-1:0] ram[0:RAM_DEPTH-1];

  logic signed [WIDTH-1:0] ram_read_data_a = 0;  // sample at integer position
  logic signed [WIDTH-1:0] ram_read_data_b = 0;  // sample at integer+1 (for interp)

  localparam ADDR_WIDTH = $clog2(RAM_DEPTH);
  logic [ADDR_WIDTH-1:0] wr_ptr = 0;
  logic [ADDR_WIDTH-1:0] rd_ptr_a = 0;  // integer read position
  logic [ADDR_WIDTH-1:0] rd_ptr_b = 0;  // integer+1 read position
  logic [ADDR_WIDTH-1:0] wr_ptr_next;
  logic [ADDR_WIDTH-1:0] rd_ptr_a_next;
  logic [ADDR_WIDTH-1:0] rd_ptr_b_next;

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
  logic [ADDR_WIDTH-1:0] delay_target;
  logic [ADDR_WIDTH-1:0] delay_samples;
  logic [ADDR_WIDTH-1:0] delay_samples_next;
  logic                  delay_loaded = 0;

  always_comb begin
    if (time_val >= RAM_DEPTH) delay_target = ADDR_WIDTH'(RAM_DEPTH - 1);
    else if (time_val == 16'd0) delay_target = ADDR_WIDTH'(1);
    else delay_target = time_val[ADDR_WIDTH-1:0];
  end

  localparam [ADDR_WIDTH-1:0] SLEW_RATE = 64;

  always_comb begin
    if (!delay_loaded) begin
      delay_samples_next = delay_target;
    end else if (delay_samples < delay_target) begin
      if ((delay_target - delay_samples) > SLEW_RATE)
        delay_samples_next = delay_samples + SLEW_RATE;
      else delay_samples_next = delay_target;
    end else if (delay_samples > delay_target) begin
      if ((delay_samples - delay_target) > SLEW_RATE)
        delay_samples_next = delay_samples - SLEW_RATE;
      else delay_samples_next = delay_target;
    end else begin
      delay_samples_next = delay_samples;
    end
  end

  // =========================================================================
  // 2C. LFO FOR DELAY MODULATION  [FIX F]
  // =========================================================================
  // Triangle LFO at ~0.7 Hz (subtle chorus-like modulation).
  // Depth: ±4 samples at 48kHz = ±83 microseconds - enough to break up
  // comb-filter peaks without audible pitch wobble.
  //
  // Implementation: 16-bit phase accumulator, triangle wave derived from
  // top bits. At 48kHz sample rate:
  //   phase_inc = 2^16 * 0.7 / 48000 ≈ 0.954 → round to 1
  //   Actual freq = 48000 * 1 / 65536 ≈ 0.732 Hz
  //
  // Output: signed modulation offset, range ±MOD_DEPTH samples.
  // We also output a fractional part (8-bit) for linear interpolation.
  //
  localparam MOD_DEPTH = 4;  // ±4 samples max deviation
  localparam LFO_PHASE_INC = 16'd1;  // ~0.73 Hz at 48kHz

  logic [15:0] lfo_phase = 0;
  logic signed [ADDR_WIDTH-1:0] lfo_offset;   // integer part of modulation
  logic [7:0] lfo_frac;                        // fractional part (0..255)

  // Triangle wave from phase accumulator
  // Phase [15] = direction, Phase [14:0] = magnitude
  // Scale to ±MOD_DEPTH with fractional part
  logic [15:0] lfo_triangle;  // unsigned triangle 0..65534
  logic signed [16:0] lfo_centered; // signed, centered around 0
  logic signed [19:0] lfo_scaled;   // scaled by MOD_DEPTH

  always_comb begin
    // Create triangle wave: fold top half back
    lfo_triangle = lfo_phase[15] ? ~lfo_phase : lfo_phase;

    // Center around zero: range -32767..+32767
    lfo_centered = $signed({1'b0, lfo_triangle}) - 17'sd32767;

    // Scale to ±MOD_DEPTH (multiply by MOD_DEPTH, result in high bits)
    // lfo_centered range: ±32767. We want ±4.0 in fixed point with 8-bit fraction.
    // (lfo_centered * MOD_DEPTH) / 32767 ≈ (lfo_centered * MOD_DEPTH) >> 15
    // But we want 8 extra fractional bits, so shift by 7 instead of 15.
    lfo_scaled = lfo_centered * $signed({1'b0, MOD_DEPTH[ADDR_WIDTH-1:0]});

    // Integer part: bits [19:15] sign-extended
    lfo_offset = ADDR_WIDTH'($signed(lfo_scaled[19:15]));

    // Fractional part: bits [14:7] (8 bits of fraction)
    lfo_frac = lfo_scaled[14:7];
  end

  // =========================================================================
  // 3. TONE COEFFICIENTS
  // =========================================================================
  // Same quadratic mapping as v4 for the wet output tone filter.
  // A separate, fixed coefficient is used for the feedback-path filter [FIX I].

  logic        [15:0] tone_sq;
  logic        [ 9:0] tone_base;
  logic signed [ 9:0] tone_coeff;
  logic        [19:0] tone_boosted;
  logic        [25:0] tone_sq_scaled;

  always_comb begin
    tone_sq = {2'b00, tone_val} * {2'b00, tone_val};
    tone_sq_scaled = {2'b00, tone_sq} * 26'd229;
    tone_base = 10'd27 + tone_sq_scaled[25:16];
    tone_boosted = tone_base * 20'd397;
    if (tone_boosted[19:8] >= 12'd256) tone_coeff = 10'd256;
    else tone_coeff = {2'b00, tone_boosted[17:8]};
  end

  // -------------------------------------------------------------------------
  // 3B. FEEDBACK PATH FILTER COEFFICIENT [FIX I]
  // -------------------------------------------------------------------------
  // Fixed gentle rolloff in the feedback loop: each repeat loses some highs.
  // fc ≈ 4kHz (single-pole equivalent), giving natural darkening of repeats.
  // alpha_fb = 2*pi*4000/48000 ≈ 0.524 → coeff_fb ≈ 134
  // With 2-pole boost: 134 * 397 >> 8 = 208 → clamped to 208
  //
  // This is modulated by tone_val: at tone=0 (dark), feedback filter is
  // also darker (more analog). At tone=255, feedback filter is wide open.
  //   fb_coeff = 80 + (tone_val * 176) >> 8  → range 80..255
  //   (boosted: 124..397)
  //
  logic signed [9:0] fb_filter_coeff;
  logic [9:0] fb_filter_base;
  logic [19:0] fb_filter_boosted;
  logic [15:0] fb_filter_interp;

  always_comb begin
    // Interpolate: 80 at tone_val=0, 255 at tone_val=255
    fb_filter_interp = 16'd80 + (({8'd0, tone_val} * 16'd176) >> 8);
    fb_filter_base = fb_filter_interp[9:0];

    // 2-pole boost
    fb_filter_boosted = fb_filter_base * 20'd397;
    if (fb_filter_boosted[19:8] >= 12'd256) fb_filter_coeff = 10'd256;
    else fb_filter_coeff = {2'b00, fb_filter_boosted[17:8]};
  end

  // =========================================================================
  // 4. POINTER CALCULATIONS (with modulation offset)  [FIX F, J]
  // =========================================================================
  always_comb begin
    wr_ptr_next = (wr_ptr == ADDR_WIDTH'(RAM_DEPTH - 1)) ? '0 : wr_ptr + 1'b1;

    // Base read position with modulation offset applied
    // We compute two adjacent read positions for linear interpolation
    begin
      logic signed [ADDR_WIDTH:0] base_offset;  // signed, one extra bit
      logic signed [ADDR_WIDTH:0] mod_rd;
      logic [ADDR_WIDTH-1:0] raw_a;

      base_offset = $signed({1'b0, wr_ptr_next}) - $signed({1'b0, delay_samples});
      mod_rd = base_offset + $signed({{1{lfo_offset[ADDR_WIDTH-1]}}, lfo_offset});

      // Wrap into valid range
      if (mod_rd < 0)
        raw_a = ADDR_WIDTH'($signed(ADDR_WIDTH'(RAM_DEPTH)) + mod_rd[ADDR_WIDTH-1:0]);
      else if (mod_rd >= $signed({1'b0, ADDR_WIDTH'(RAM_DEPTH)}))
        raw_a = mod_rd[ADDR_WIDTH-1:0] - ADDR_WIDTH'(RAM_DEPTH);
      else
        raw_a = mod_rd[ADDR_WIDTH-1:0];

      rd_ptr_a_next = raw_a;
      rd_ptr_b_next = (raw_a == ADDR_WIDTH'(RAM_DEPTH - 1)) ? '0 : raw_a + 1'b1;
    end
  end

  // =========================================================================
  // 5. SIGNAL PATH
  // =========================================================================

  // -------------------------------------------------------------------------
  // 5A-1. LINEAR INTERPOLATION OF DELAY LINE READOUT  [FIX J]
  // -------------------------------------------------------------------------
  // Blend between ram_read_data_a and ram_read_data_b using lfo_frac.
  // interp = a + frac * (b - a) / 256
  //
  logic signed [WIDTH-1:0] interp_out;
  logic signed [WIDTH:0] interp_diff;
  logic signed [WIDTH+8:0] interp_product;

  // We register lfo_frac along with the read data in Phase 1
  logic [7:0] lfo_frac_d1 = 0;

  always_comb begin
    interp_diff = $signed({ram_read_data_b[WIDTH-1], ram_read_data_b}) -
                  $signed({ram_read_data_a[WIDTH-1], ram_read_data_a});
    interp_product = interp_diff * $signed({1'b0, lfo_frac_d1});
    interp_out = ram_read_data_a + WIDTH'(interp_product >>> 8);
  end

  // -------------------------------------------------------------------------
  // 5A-2. WET OUTPUT TONE FILTER (biquad)  [FIX B]
  // -------------------------------------------------------------------------
  // This filter shapes the wet output tone. Separate from feedback-path filter.

  logic [23:0] lpf_alpha_q;
  logic [23:0] lpf_one_m_a;
  logic [47:0] lpf_b0_raw;
  logic [47:0] lpf_a2_raw;
  logic [24:0] lpf_a1_wide;
  logic signed [23:0] lpf_b0, lpf_a1_neg, lpf_a2_neg;

  always_comb begin
    lpf_alpha_q = {tone_coeff[8:0], 15'h0};
    lpf_one_m_a = {(9'(10'd256) - tone_coeff[8:0]), 15'h0};
    lpf_b0_raw  = $signed(lpf_alpha_q) * $signed(lpf_alpha_q);
    lpf_b0      = $signed(lpf_b0_raw[46:23]);
    lpf_a1_wide = {lpf_one_m_a, 1'b0};
    lpf_a1_neg  = lpf_a1_wide[24] ? 24'sh7FFFFF : $signed(lpf_a1_wide[23:0]);
    lpf_a2_raw  = $signed(lpf_one_m_a) * $signed(lpf_one_m_a);
    lpf_a2_neg  = -$signed(lpf_a2_raw[46:23]);
  end

  wire signed [WIDTH-1:0] lpf_out;

  biquad_tdf2 #(
      .DATA_W (WIDTH),
      .COEFF_W(WIDTH),
      .FRAC   (WIDTH - 1)
  ) tone_lpf (
      .clk   (clk),
      .rst_n (rst_n),
      .en    (sample_en_d1),
      .x_in  (interp_out),   // Interpolated delay line output
      .y_out (lpf_out),
      .b0    (lpf_b0),
      .b1    (24'sd0),
      .b2    (24'sd0),
      .a1_neg(lpf_a1_neg),
      .a2_neg(lpf_a2_neg)
  );

  // -------------------------------------------------------------------------
  // 5A-3. FEEDBACK PATH FILTER  [FIX I]
  // -------------------------------------------------------------------------
  // Separate 1-pole LPF in the feedback write path. Each repeat passes
  // through this filter, progressively losing highs - just like analog
  // bucket-brigade delays. This is the main fix for metallic repeats.
  //
  // Simple 1-pole:  y += alpha * (x - y)
  // Implemented as: y_new = y_old + ((x - y_old) * coeff) >> 8
  //
  logic signed [WIDTH-1:0] fb_lpf_state = 0;
  logic signed [WIDTH-1:0] fb_lpf_out;
  logic signed [WIDTH:0]   fb_lpf_diff;
  logic signed [WIDTH+9:0] fb_lpf_step;

  always_comb begin
    fb_lpf_diff = $signed({interp_out[WIDTH-1], interp_out}) -
                  $signed({fb_lpf_state[WIDTH-1], fb_lpf_state});
    fb_lpf_step = fb_lpf_diff * fb_filter_coeff;
    fb_lpf_out  = fb_lpf_state + WIDTH'(fb_lpf_step >>> 8);
  end

  // -------------------------------------------------------------------------
  // 5B. DC-BLOCKING FILTER  [FIX G]
  // -------------------------------------------------------------------------
  // 1-pole HPF at ~20 Hz to remove DC and sub-audible buildup.
  //   y[n] = x[n] - x[n-1] + R * y[n-1],  R = 1 - 2*pi*fc/fs
  //   R = 1 - 2*pi*20/48000 ≈ 0.99738
  //   In Q0.15: R = 32681
  //
  // Applied to the feedback-filtered signal before it enters the
  // feedback gain stage.
  //
  localparam signed [15:0] DC_BLOCK_R = 16'sd32681;  // 0.99738 in Q0.15

  logic signed [WIDTH-1:0] dc_x_prev = 0;
  logic signed [WIDTH-1:0] dc_y_prev = 0;
  logic signed [WIDTH-1:0] dc_out;
  logic signed [WIDTH:0]   dc_diff;
  logic signed [WIDTH+15:0] dc_feedback;

  always_comb begin
    dc_diff     = $signed({fb_lpf_out[WIDTH-1], fb_lpf_out}) -
                  $signed({dc_x_prev[WIDTH-1], dc_x_prev});
    dc_feedback = $signed(dc_y_prev) * DC_BLOCK_R;
    dc_out      = WIDTH'(dc_diff) + WIDTH'(dc_feedback >>> 15);
  end

  // -------------------------------------------------------------------------
  // 5C. SOFT-CLIP FEEDBACK PATH  [FIX D + H]
  // -------------------------------------------------------------------------
  // Asymmetric soft-clipper for warmer saturation:
  //   Positive half: tanh-like approximation (3x - x^3) / 2
  //   Negative half: slightly harder clip at 0.95x threshold
  // This introduces subtle even harmonics (like tube/tape saturation)
  // instead of purely odd harmonics from symmetric clipping.
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

  // Pre-clamp to ±MAX
  saturate #(
      .IN_W (WIDTH + 11),
      .OUT_W(WIDTH)
  ) sat_fb_pre (
      .din (fb_sum_raw),
      .dout(fb_clamped)
  );

  always_comb begin
    // Feedback from DC-blocked, filtered delay output  [FIX E, G, I]
    fb_signal_raw = ($signed(dc_out) * fb_gain) >>> 8;
    fb_sum_raw = $signed(audio_in) + fb_signal_raw;

    // --- Asymmetric soft-clip [FIX H] ---
    sc_sq = $signed(fb_clamped) * $signed(fb_clamped);
    sc_sq_norm = sc_sq[47:23];
    sc_cu_raw = $signed(sc_sq_norm) * $signed(fb_clamped);
    sc_cu_norm = sc_cu_raw[48:23];

    sc_3x = $signed({{2{fb_clamped[WIDTH-1]}}, fb_clamped}) +
            $signed({{2{fb_clamped[WIDTH-1]}}, fb_clamped}) +
            $signed({{2{fb_clamped[WIDTH-1]}}, fb_clamped});

    sc_pre = sc_3x - sc_cu_norm;

    // Asymmetry: positive half gets gentler clipping (divide by 2),
    // negative half clips slightly harder (multiply by 0.95 ≈ *243>>8)
    if (fb_clamped >= 0) begin
      sc_out = sc_pre >>> 1;
    end else begin
      // Slightly harder negative clip: scale cubic result by 0.95
      logic signed [33:0] asym_product;
      asym_product = sc_pre * $signed(10'sd243);
      sc_out = 26'($signed(asym_product[33:9]));
    end
  end

  // Final saturation guard on soft-clip output
  saturate #(
      .IN_W (26),
      .OUT_W(WIDTH)
  ) sat_fb_out (
      .din (sc_out),
      .dout(fb_softclip)
  );

  // -------------------------------------------------------------------------
  // 5D. WET OUTPUT - scaled filtered delay signal  [FIX E]
  // -------------------------------------------------------------------------
  logic signed [WIDTH+10:0] wet_signal;
  logic signed [ WIDTH-1:0] out_saturated;
  logic signed [ WIDTH-1:0] out_saturated_clamped;

  saturate #(
      .IN_W (WIDTH + 11),
      .OUT_W(WIDTH)
  ) sat_wet (
      .din (wet_signal),
      .dout(out_saturated_clamped)
  );

  always_comb begin
    if (time_val == 16'd0) begin
      wet_signal    = '0;
      out_saturated = '0;
    end else begin
      wet_signal    = ($signed(lpf_out) * wet_level) >>> 8;
      out_saturated = out_saturated_clamped;
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
  // Phase 1 (sample_en):    Issue BRAM reads (two adjacent for interp);
  //                         advance LFO.
  // Phase 2 (sample_en_d1): Data valid. Process feedback+filter+output,
  //                         write BRAM, advance pointers, update state.
  //
  logic sample_en_d1 = 0;

  always_ff @(posedge clk) begin
    if (!rst_n) sample_en_d1 <= 1'b0;
    else sample_en_d1 <= sample_en;
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      wr_ptr          <= '0;
      rd_ptr_a        <= '0;
      rd_ptr_b        <= '0;
      audio_out_reg   <= '0;
      ram_read_data_a <= '0;
      ram_read_data_b <= '0;
      delay_samples   <= ADDR_WIDTH'(1);
      delay_loaded    <= 1'b0;
      lfo_phase       <= '0;
      lfo_frac_d1     <= '0;
      fb_lpf_state    <= '0;
      dc_x_prev       <= '0;
      dc_y_prev       <= '0;
    end else begin
      // Phase 1: Issue BRAM reads + advance LFO
      if (sample_en) begin
        ram_read_data_a <= ram[rd_ptr_a];
        ram_read_data_b <= ram[rd_ptr_b];
        lfo_phase       <= lfo_phase + LFO_PHASE_INC;
        lfo_frac_d1     <= lfo_frac;
      end

      // Phase 2: Process and write
      if (sample_en_d1) begin
        ram[wr_ptr]   <= fb_softclip;
        wr_ptr        <= wr_ptr_next;
        rd_ptr_a      <= rd_ptr_a_next;
        rd_ptr_b      <= rd_ptr_b_next;
        audio_out_reg <= out_saturated;
        delay_samples <= delay_samples_next;
        delay_loaded  <= 1'b1;

        // Update feedback path filter state  [FIX I]
        fb_lpf_state <= fb_lpf_out;

        // Update DC blocker state  [FIX G]
        dc_x_prev <= fb_lpf_out;
        dc_y_prev <= dc_out;
      end
    end
  end

endmodule