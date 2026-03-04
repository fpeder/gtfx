`timescale 1ns / 1ps

// ============================================================================
// tube_distortion.sv - Cascaded Soft-Clip Tube Distortion
//
// Three-stage cascaded soft-clip architecture modeled on real tube amp
// topology: input coupling HPF, two preamp gain/clip stages with inter-stage
// filtering, 3-band Baxandall tone stack between preamp and power amp,
// and a gentle power stage soft-clip.
//
// At low gain only stage 1 clips (warm breakup); at high gain all stages
// saturate (rich harmonics). Tone controls shape the distortion character
// because the tone stack sits between preamp and power amp stages.
//
// Signal path:
//   audio_in -> [HPF 80Hz] -> [x gain1 -> soft_clip 2:1] -> [LPF 8kHz]
//            -> [x gain2 -> soft_clip 4:1] -> [LPF 10kHz] -> [x2 restore]
//            -> [Bass/Mid/Treble tone stack]
//            -> [x2 -> soft_clip 2:1 power stage]
//            -> [x level/512] -> audio_out
//
// Latency: 1 cycle (sample_en -> audio_out registered)
//
// Dependencies:
//   saturate.sv, soft_clip.sv, biquad_tdf2.sv
// ============================================================================

module tube_distortion #(
    parameter int DATA_W = 24
) (
    input logic clk,
    input logic rst_n,
    input logic sample_en,

    // Audio I/O - 24-bit signed 2's complement
    input  logic signed [DATA_W-1:0] audio_in,
    output logic signed [DATA_W-1:0] audio_out,
    output logic                     valid_out,

    // Controls - unsigned, 0=min 255=max
    input logic [7:0] gain,         // pre-gain: cascaded stages
    input logic [7:0] tone_bass,    // bass shelf:   128=flat
    input logic [7:0] tone_mid,     // mid peak:     128=flat
    input logic [7:0] tone_treble,  // treble shelf: 128=flat
    input logic [7:0] level         // output attenuation
);

  // =========================================================================
  // Local parameters
  // =========================================================================
  localparam int INT_W = DATA_W + 4;  // 28-bit internal width

  // --- Soft-clip thresholds ---
  // Stage 1 (preamp triode 1): gentle, wide -- 2:1 above 0.5 FS, no hard limit
  localparam signed [DATA_W-1:0] CLIP1_KNEE = signed'(DATA_W'(1 << (DATA_W - 2)));  // 0.5 FS

  // Stage 2 (preamp triode 2): tighter -- 4:1 above 0.375 FS, cap at 0.5 FS
  localparam signed [DATA_W-1:0] CLIP2_KNEE = signed'(DATA_W'((3 * (1 << (DATA_W-1))) / 8));  // 0.375 FS
  localparam signed [DATA_W-1:0] CLIP2_PEAK = signed'(DATA_W'(1 << (DATA_W - 2)));             // 0.50 FS

  // Stage 3 (power amp): warm compression -- 2:1 above 0.5 FS, cap at 0.75 FS
  localparam signed [DATA_W-1:0] CLIP3_KNEE = signed'(DATA_W'(1 << (DATA_W - 2)));             // 0.5 FS
  localparam signed [DATA_W-1:0] CLIP3_PEAK = signed'(DATA_W'((3 * (1 << (DATA_W-1))) / 4));   // 0.75 FS

  // =========================================================================
  // Filter coefficients -- all Q4.20 (COEFF_W=24, FRAC=20)
  // =========================================================================
  localparam int COEFF_W = 24;
  localparam int FRAC    = 20;

  // --- Input coupling HPF: 2nd-order Butterworth, Fc = 80 Hz ---
  // Removes DC and sub-bass rumble (models coupling capacitor).
  //   b0 =  0.9926225428  b1 = -1.9852450855  b2 =  0.9926225428
  //   a1 = -1.9851906579  a2 =  0.9852995131
  localparam logic signed [COEFF_W-1:0] HPF_B0     =  24'sd1040840;
  localparam logic signed [COEFF_W-1:0] HPF_B1     = -24'sd2081680;
  localparam logic signed [COEFF_W-1:0] HPF_B2     =  24'sd1040840;
  localparam logic signed [COEFF_W-1:0] HPF_A1_NEG =  24'sd2081623;
  localparam logic signed [COEFF_W-1:0] HPF_A2_NEG = -24'sd1033161;

  // --- Post-clip smoothing 1: 2nd-order Butterworth LPF, Fc = 8 kHz ---
  // Warmer than 10 kHz, tames aliasing from stage 1 clipping.
  //   b0 =  0.1550510257  b1 =  0.3101020514  b2 =  0.1550510257
  //   a1 = -0.6202041029  a2 =  0.2404082058
  localparam logic signed [COEFF_W-1:0] SM1_B0     =  24'sd162583;
  localparam logic signed [COEFF_W-1:0] SM1_B1     =  24'sd325166;
  localparam logic signed [COEFF_W-1:0] SM1_B2     =  24'sd162583;
  localparam logic signed [COEFF_W-1:0] SM1_A1_NEG =  24'sd650331;
  localparam logic signed [COEFF_W-1:0] SM1_A2_NEG = -24'sd252086;

  // --- Post-clip smoothing 2: 2nd-order Butterworth LPF, Fc = 10 kHz ---
  // Same as big_muff smoothing filter.
  //   b0 =  0.2201947003  b1 =  0.4403894005  b2 =  0.2201947003
  //   a1 = -0.3075663598  a2 =  0.1883451609
  localparam logic signed [COEFF_W-1:0] SM2_B0     =  24'sd230891;
  localparam logic signed [COEFF_W-1:0] SM2_B1     =  24'sd461782;
  localparam logic signed [COEFF_W-1:0] SM2_B2     =  24'sd230891;
  localparam logic signed [COEFF_W-1:0] SM2_A1_NEG =  24'sd322507;
  localparam logic signed [COEFF_W-1:0] SM2_A2_NEG = -24'sd197494;

  // --- Tone Bass LPF: 2nd-order Butterworth, Fc = 250 Hz ---
  //   b0 =  0.0002616527  b1 =  0.0005233054  b2 =  0.0002616527
  //   a1 = -1.9537279491  a2 =  0.9547745599
  localparam logic signed [COEFF_W-1:0] BASS_B0     =  24'sd274;
  localparam logic signed [COEFF_W-1:0] BASS_B1     =  24'sd549;
  localparam logic signed [COEFF_W-1:0] BASS_B2     =  24'sd274;
  localparam logic signed [COEFF_W-1:0] BASS_A1_NEG =  24'sd2048632;
  localparam logic signed [COEFF_W-1:0] BASS_A2_NEG = -24'sd1001154;

  // --- Tone Mid BPF: 2nd-order, Fc = 1 kHz, Q = 1.5 ---
  //   b0 =  0.0416946495  b1 =  0.0000000000  b2 = -0.0416946495
  //   a1 = -1.9002138308  a2 =  0.9166107011
  localparam logic signed [COEFF_W-1:0] MID_B0     =  24'sd43720;
  localparam logic signed [COEFF_W-1:0] MID_B1     =  24'sd0;
  localparam logic signed [COEFF_W-1:0] MID_B2     = -24'sd43720;
  localparam logic signed [COEFF_W-1:0] MID_A1_NEG =  24'sd1992519;
  localparam logic signed [COEFF_W-1:0] MID_A2_NEG = -24'sd961136;

  // --- Tone Treble HPF: 2nd-order Butterworth, Fc = 4 kHz ---
  //   b0 =  0.6893061688  b1 = -1.3786123375  b2 =  0.6893061688
  //   a1 = -1.2796324250  a2 =  0.4775922501
  localparam logic signed [COEFF_W-1:0] TREB_B0     =  24'sd722790;
  localparam logic signed [COEFF_W-1:0] TREB_B1     = -24'sd1445580;
  localparam logic signed [COEFF_W-1:0] TREB_B2     =  24'sd722790;
  localparam logic signed [COEFF_W-1:0] TREB_A1_NEG =  24'sd1341792;
  localparam logic signed [COEFF_W-1:0] TREB_A2_NEG = -24'sd500792;

  // =========================================================================
  // Per-stage gain from gain knob (same formula as big_muff)
  // =========================================================================
  // Stage 1: 1.0x to  4.0x (Q8.8: 256..1024)
  // Stage 2: 1.5x to 16.0x (Q8.8: 384..4096)
  localparam int GAIN_W = 16;
  logic [GAIN_W-1:0] stage1_gain;
  logic [GAIN_W-1:0] stage2_gain;

  always_comb begin
    // 256 + gain * 768/256 -> 1.0x (gain=0) to 4.0x (gain=255)
    stage1_gain = GAIN_W'(16'd256 + ((32'(gain) * 32'd768)  >> 8));
    // 384 + gain * 3712/256 -> 1.5x (gain=0) to 16.0x (gain=255)
    stage2_gain = GAIN_W'(16'd384 + ((32'(gain) * 32'd3712) >> 8));
  end

  // =========================================================================
  // Input coupling HPF (80 Hz -- removes DC and sub-bass)
  // =========================================================================
  logic signed [DATA_W-1:0] hpf_out;

  biquad_tdf2 #(
      .DATA_W (DATA_W),
      .COEFF_W(COEFF_W),
      .FRAC   (FRAC)
  ) u_hpf (
      .clk   (clk),
      .rst_n (rst_n),
      .en    (sample_en),
      .x_in  (audio_in),
      .y_out (hpf_out),
      .b0    (HPF_B0),
      .b1    (HPF_B1),
      .b2    (HPF_B2),
      .a1_neg(HPF_A1_NEG),
      .a2_neg(HPF_A2_NEG)
  );

  // =========================================================================
  // Clipping stage 1 + post-clip smoothing (8 kHz)
  // =========================================================================
  logic signed [INT_W-1:0] stage1_gained;
  logic signed [DATA_W-1:0] stage1_clipped;
  logic signed [DATA_W-1:0] stage1_smooth;

  always_comb begin
    stage1_gained = INT_W'((longint'(hpf_out) * longint'(signed'({1'b0, stage1_gain}))) >>> 8);
  end

  soft_clip #(
      .IN_W      (INT_W),
      .OUT_W     (DATA_W),
      .COMP_SHIFT(1),          // 2:1 compression
      .KNEE      (CLIP1_KNEE)
      // PEAK = default (max positive) -- gentle, no hard limit
  ) u_clip1 (
      .din (stage1_gained),
      .dout(stage1_clipped)
  );

  biquad_tdf2 #(
      .DATA_W (DATA_W),
      .COEFF_W(COEFF_W),
      .FRAC   (FRAC)
  ) u_smooth1 (
      .clk   (clk),
      .rst_n (rst_n),
      .en    (sample_en),
      .x_in  (stage1_clipped),
      .y_out (stage1_smooth),
      .b0    (SM1_B0),
      .b1    (SM1_B1),
      .b2    (SM1_B2),
      .a1_neg(SM1_A1_NEG),
      .a2_neg(SM1_A2_NEG)
  );

  // =========================================================================
  // Clipping stage 2 + post-clip smoothing (10 kHz)
  // =========================================================================
  logic signed [INT_W-1:0] stage2_gained;
  logic signed [DATA_W-1:0] stage2_clipped;
  logic signed [DATA_W-1:0] stage2_smooth;

  always_comb begin
    stage2_gained = INT_W'((longint'(stage1_smooth) * longint'(signed'({1'b0, stage2_gain}))) >>> 8);
  end

  soft_clip #(
      .IN_W      (INT_W),
      .OUT_W     (DATA_W),
      .COMP_SHIFT(2),          // 4:1 compression
      .KNEE      (CLIP2_KNEE),
      .PEAK      (CLIP2_PEAK)
  ) u_clip2 (
      .din (stage2_gained),
      .dout(stage2_clipped)
  );

  biquad_tdf2 #(
      .DATA_W (DATA_W),
      .COEFF_W(COEFF_W),
      .FRAC   (FRAC)
  ) u_smooth2 (
      .clk   (clk),
      .rst_n (rst_n),
      .en    (sample_en),
      .x_in  (stage2_clipped),
      .y_out (stage2_smooth),
      .b0    (SM2_B0),
      .b1    (SM2_B1),
      .b2    (SM2_B2),
      .a1_neg(SM2_A1_NEG),
      .a2_neg(SM2_A2_NEG)
  );

  // =========================================================================
  // Post-clip level restore (x2)
  // =========================================================================
  // Clipped signal peaks at +/-CLIP2_PEAK (50% FS).  Shift up to use full
  // dynamic range for the tone filters.
  logic signed [DATA_W-1:0] restored;

  always_comb begin
    if (stage2_smooth >= CLIP2_PEAK)
      restored = {1'b0, {(DATA_W-1){1'b1}}};
    else if (stage2_smooth <= -CLIP2_PEAK)
      restored = {1'b1, {(DATA_W-1){1'b0}}};
    else
      restored = DATA_W'(stage2_smooth <<< 1);
  end

  // =========================================================================
  // 3-band Baxandall tone stack
  //
  // Three parallel biquads (Q4.20), same frequencies as previous design:
  //   Bass  LPF  250 Hz  Butterworth
  //   Mid   BPF  1 kHz   Q=1.5
  //   Treble HPF 4 kHz   Butterworth
  //
  // Placed between preamp and power amp so tone controls shape the
  // distortion character, not just post-filter the result.
  // =========================================================================
  logic signed [DATA_W-1:0] bass_y, mid_y, treb_y;

  biquad_tdf2 #(
      .DATA_W (DATA_W),
      .COEFF_W(COEFF_W),
      .FRAC   (FRAC)
  ) u_bass (
      .clk   (clk),
      .rst_n (rst_n),
      .en    (sample_en),
      .x_in  (restored),
      .y_out (bass_y),
      .b0    (BASS_B0),
      .b1    (BASS_B1),
      .b2    (BASS_B2),
      .a1_neg(BASS_A1_NEG),
      .a2_neg(BASS_A2_NEG)
  );

  biquad_tdf2 #(
      .DATA_W (DATA_W),
      .COEFF_W(COEFF_W),
      .FRAC   (FRAC)
  ) u_mid (
      .clk   (clk),
      .rst_n (rst_n),
      .en    (sample_en),
      .x_in  (restored),
      .y_out (mid_y),
      .b0    (MID_B0),
      .b1    (MID_B1),
      .b2    (MID_B2),
      .a1_neg(MID_A1_NEG),
      .a2_neg(MID_A2_NEG)
  );

  biquad_tdf2 #(
      .DATA_W (DATA_W),
      .COEFF_W(COEFF_W),
      .FRAC   (FRAC)
  ) u_treb (
      .clk   (clk),
      .rst_n (rst_n),
      .en    (sample_en),
      .x_in  (restored),
      .y_out (treb_y),
      .b0    (TREB_B0),
      .b1    (TREB_B1),
      .b2    (TREB_B2),
      .a1_neg(TREB_A1_NEG),
      .a2_neg(TREB_A2_NEG)
  );

  // =========================================================================
  // Tone mix: dry + bg*bass + mg*mid + tg*treble
  //
  // Each band: (biquad_out * tone_offset) >>> 7
  // =========================================================================
  wire signed [8:0] bg = {1'b0, tone_bass}   - 9'sh80;
  wire signed [8:0] mg = {1'b0, tone_mid}    - 9'sh80;
  wire signed [8:0] tg = {1'b0, tone_treble} - 9'sh80;

  localparam int PROD_W = DATA_W + 9 + 9;  // 42 bits
  localparam int MIX_W  = DATA_W + 4;      // 28 bits

  wire signed [PROD_W-1:0] bass_prod = ($signed({{9{bass_y[DATA_W-1]}}, bass_y}) * bg) >>> 7;
  wire signed [PROD_W-1:0] mid_prod  = ($signed({{9{mid_y[DATA_W-1]}},  mid_y})  * mg) >>> 7;
  wire signed [PROD_W-1:0] treb_prod = ($signed({{9{treb_y[DATA_W-1]}}, treb_y}) * tg) >>> 7;

  wire signed [MIX_W-1:0] bass_term, mid_term, treb_term;

  saturate #(.IN_W(PROD_W), .OUT_W(MIX_W)) u_sat_bass (.din(bass_prod), .dout(bass_term));
  saturate #(.IN_W(PROD_W), .OUT_W(MIX_W)) u_sat_mid  (.din(mid_prod),  .dout(mid_term));
  saturate #(.IN_W(PROD_W), .OUT_W(MIX_W)) u_sat_treb (.din(treb_prod), .dout(treb_term));

  wire signed [MIX_W-1:0] mix_raw =
          {{(MIX_W-DATA_W){restored[DATA_W-1]}}, restored}
        + bass_term
        + mid_term
        + treb_term;

  wire signed [DATA_W-1:0] tone_sat;

  saturate #(
      .IN_W (MIX_W),
      .OUT_W(DATA_W)
  ) u_sat_tone (
      .din (mix_raw),
      .dout(tone_sat)
  );

  // =========================================================================
  // Power stage: x2 gain -> soft_clip (models push-pull power tubes)
  // =========================================================================
  logic signed [INT_W-1:0] power_gained;
  logic signed [DATA_W-1:0] power_out;

  always_comb begin
    power_gained = INT_W'(longint'(tone_sat) <<< 1);
  end

  soft_clip #(
      .IN_W      (INT_W),
      .OUT_W     (DATA_W),
      .COMP_SHIFT(1),          // 2:1 compression
      .KNEE      (CLIP3_KNEE),
      .PEAK      (CLIP3_PEAK)
  ) u_clip3 (
      .din (power_gained),
      .dout(power_out)
  );

  // =========================================================================
  // Output level + saturation
  //
  // level/512: 0x00=silent, 0x80=0.25x, 0xFF=~0.5x
  // Halved range (>>>9 vs >>>8) tames the high RMS from cascaded soft-clips.
  // =========================================================================
  logic signed [INT_W-1:0] vol_scaled;
  logic signed [DATA_W-1:0] vol_out;

  always_comb begin
    vol_scaled = INT_W'((longint'(power_out) * longint'({1'b0, level})) >>> 9);
  end

  saturate #(
      .IN_W (INT_W),
      .OUT_W(DATA_W)
  ) u_sat_vol (
      .din (vol_scaled),
      .dout(vol_out)
  );

  // =========================================================================
  // Registered output (1-cycle latency)
  // =========================================================================
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      audio_out <= '0;
      valid_out <= 1'b0;
    end else begin
      valid_out <= sample_en;
      if (sample_en) audio_out <= vol_out;
    end
  end

endmodule : tube_distortion
