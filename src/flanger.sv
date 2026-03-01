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
//                     ┌──────── feedback (regen) ────────┐
//                     │                                  │
//   audio_in ──►(+)──► delay_line ──► interp ──► LPF ──►├──► (+) ──► audio_out
//                                                wet     │    dry+wet / 2
//                                                        │
//                                              biquad_tdf2 (BBD recon filter)
//
// The biquad models the SAD1024 BBD's reconstruction low-pass (~6 kHz at
// 48 kHz Fs).  This gives the swept signal its characteristic dark, thick
// tone — a defining quality of analog bucket-brigade flangers.
//
// All saturation uses the shared `saturate` module (no inline functions).
//
// Delay range: ~0.25 ms - ~5 ms at 48 kHz (12-240 samples)
// LFO: triangle wave, ~0.05-11 Hz
// Feedback: signed (negative = through-zero flanging)
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
  // 2nd-order Butterworth LPF, Fc ~ 6 kHz, Fs = 48 kHz
  // Analog prototype:  wc = 2*pi*6000
  // Bilinear transform with pre-warp -> digital coefficients:
  //
  //   b0 =  0.06745527  b1 =  0.13491055  b2 =  0.06745527
  //   a1 = -1.14298050  a2 =  0.41280159
  //
  // Quantised to Q1.23 (COEFF_W = 24, FRAC = 23):
  localparam int COEFF_W = 24;
  localparam int FRAC = 23;

  localparam logic signed [COEFF_W-1:0] LPF_B0 = 24'sd565870;  //  0.06745527 * 2^23
  localparam logic signed [COEFF_W-1:0] LPF_B1 = 24'sd1131740;  //  0.13491055 * 2^23
  localparam logic signed [COEFF_W-1:0] LPF_B2 = 24'sd565870;  //  0.06745527 * 2^23
  // Pre-negated for biquad_tdf2 (pass -a1, -a2)
  localparam logic signed [COEFF_W-1:0] LPF_A1_NEG = 24'sd9584882;  // +1.14298050 * 2^23 (= -a1)
  localparam logic signed [COEFF_W-1:0] LPF_A2_NEG = -24'sd3462816;  // -0.41280159 * 2^23 (= -a2)

  // =========================================================================
  // LFO - Triangle wave via phase accumulator
  // =========================================================================
  logic [LFO_ACC_W-1:0] lfo_acc;
  logic [LFO_ACC_W-1:0] lfo_inc;
  logic [          7:0] lfo_tri;  // unsigned 8-bit triangle output

  // Scale speed_val (0-255) to phase increment (0-LFO_INC_MAX)
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
  // total_delay = MIN_DELAY + manual*RANGE/256 +/- width*lfo*RANGE/(256*256)
  // Fixed-point result: [integer . FRAC_W fractional] samples

  logic [ADDR_W+FRAC_W-1:0] base_delay_frac;
  logic [ADDR_W+FRAC_W-1:0] lfo_mod_frac;
  logic [ADDR_W+FRAC_W:0] total_delay_raw;
  logic [ADDR_W+FRAC_W-1:0] total_delay_clamped;

  logic [ADDR_W-1:0] delay_int;
  logic [FRAC_W-1:0] delay_frac;

  always_comb begin
    // Base delay from MANUAL knob
    base_delay_frac = (ADDR_W+FRAC_W)'(
            (MIN_DELAY << FRAC_W) +
            ((int'(manual_val) * DELAY_RANGE) << FRAC_W) / 256
        );

    // LFO modulation scaled by WIDTH
    lfo_mod_frac = (ADDR_W+FRAC_W)'(
            (int'(width_val) * int'(lfo_tri) * DELAY_RANGE * (1 << FRAC_W)) / (256 * 256)
        );

    // Combine (centred: LFO swings +/-half around base)
    total_delay_raw = (ADDR_W+FRAC_W+1)'(signed'({1'b0, base_delay_frac}) +
                           signed'({1'b0, lfo_mod_frac}) -
                           signed'({1'b0, (ADDR_W+FRAC_W)'(
                               (int'(width_val) * DELAY_RANGE * (1 << FRAC_W)) / (256 * 2)
                           )}));

    // Clamp to valid range
    if (total_delay_raw[ADDR_W+FRAC_W])  // negative
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

  logic signed [ INT_W-1:0] feedback_scaled;
  logic signed [ INT_W-1:0] delay_input_wide;

  // =========================================================================
  // Delay line (circular buffer)
  // =========================================================================
  logic signed [ WIDTH-1:0] delay_mem        [0:MAX_DELAY-1];
  logic        [ADDR_W-1:0] wr_ptr;

  logic [ADDR_W-1:0] rd_addr_a, rd_addr_b;
  logic signed [WIDTH-1:0] tap_a, tap_b;
  logic signed [WIDTH-1:0] wet_interp;

  // Read address computation
  always_comb begin
    rd_addr_a = wr_ptr - delay_int;
    rd_addr_b = wr_ptr - delay_int - ADDR_W'(1);
  end

  // --- Saturate delay-line input (shared saturate module) ---
  logic signed [WIDTH-1:0] delay_input_sat;

  saturate #(
      .IN_W (INT_W),
      .OUT_W(WIDTH)
  ) u_sat_delay_in (
      .din (delay_input_wide),
      .dout(delay_input_sat)
  );

  // Write: input + feedback into delay line
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wr_ptr <= '0;
      for (int i = 0; i < MAX_DELAY; i++) delay_mem[i] <= '0;
    end else if (sample_en) begin
      delay_mem[wr_ptr] <= delay_input_sat;
      wr_ptr <= (wr_ptr == ADDR_W'(MAX_DELAY - 1)) ? '0 : wr_ptr + ADDR_W'(1);
    end
  end

  // Read taps (registered for timing)
  always_ff @(posedge clk) begin
    if (sample_en) begin
      tap_a <= delay_mem[rd_addr_a];
      tap_b <= delay_mem[rd_addr_b];
    end
  end

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
  // Models the SAD1024's clock-reconstruction / anti-imaging filter.
  // The original M117R has a ~6 kHz rolloff on the delayed path which
  // gives the swept signal its characteristic warm, dark character and
  // prevents harsh aliasing artefacts from the BBD clock.
  //
  // sample_en_d1 is used as the enable because wet_interp is valid
  // one cycle after sample_en (tap_a/tap_b are registered reads).

  logic                    sample_en_d1;
  logic signed [WIDTH-1:0] wet_filtered;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) sample_en_d1 <= 1'b0;
    else sample_en_d1 <= sample_en;
  end

  biquad_tdf2 #(
      .DATA_W (WIDTH),
      .COEFF_W(COEFF_W),
      .FRAC   (FRAC)
  ) u_bbd_lpf (
      .clk   (clk),
      .rst_n (rst_n),
      .en    (sample_en_d1),
      .x_in  (wet_interp),
      .y_out (wet_filtered),
      .b0    (LPF_B0),
      .b1    (LPF_B1),
      .b2    (LPF_B2),
      .a1_neg(LPF_A1_NEG),
      .a2_neg(LPF_A2_NEG)
  );

  // =========================================================================
  // Feedback path  (taken from the filtered wet signal)
  // =========================================================================
  // feedback = wet_filtered * regen_signed / 128
  always_comb begin
    feedback_scaled  = INT_W'((longint'(wet_filtered) * longint'(regen_signed)) >>> 7);
    delay_input_wide = INT_W'(signed'(audio_in)) + feedback_scaled;
  end

  // =========================================================================
  // Output mix:  (dry + wet_filtered) / 2
  // =========================================================================
  logic signed [INT_W-1:0] mix;
  logic signed [WIDTH-1:0] mix_sat;

  always_comb begin
    mix = (INT_W'(signed'(audio_in)) + INT_W'(signed'(wet_filtered))) >>> 1;
  end

  // --- Saturate output mix (shared saturate module) ---
  saturate #(
      .IN_W (INT_W),
      .OUT_W(WIDTH)
  ) u_sat_mix (
      .din (mix),
      .dout(mix_sat)
  );

  // =========================================================================
  // Registered output
  // =========================================================================
  // Pipeline:  sample_en  -> (tap read registered)
  //            sample_en_d1 -> (biquad processes wet_interp)
  //            sample_en_d2 -> output register captures mix_sat
  //
  // Total latency: 2 cycles from sample_en to audio_out valid.
  // axis_effect_slot gen_flanger uses sample_en_d2 as effect_valid.

  logic sample_en_d2;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      audio_out    <= '0;
      sample_en_d2 <= 1'b0;
    end else begin
      sample_en_d2 <= sample_en_d1;
      if (sample_en_d2) audio_out <= mix_sat;
    end
  end

endmodule
