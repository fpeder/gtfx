`timescale 1ns / 1ps

// ============================================================
// chorus.sv - BOSS CE-5 style stereo chorus
//
// Fixed-point format summary (parametric):
//   Audio:       signed  Q1.(DATA_WIDTH-1)     e.g. Q1.23
//   LFO:         signed  Q1.(LFO_W-1)          e.g. Q1.15
//   Mod depth:   unsigned (LFO_W-1)-bit        depth[7:1] → 7-bit amplitude
//   mod_prod:    signed  Q1.(LFO_W-1) × Q0.7   >> (LFO_W-1) → integer offset
//   EQ gain:     unsigned Q0.CTRL_W            128 = unity (>> CTRL_W-1)
//   Effect level:unsigned Q0.CTRL_W            >> CTRL_W normalises product
//
// Controls (CTRL_W-bit unsigned, default 8):
//   rate, depth, effect_lvl, e_q_hi, e_q_lo

module chorus #(
    parameter int DATA_WIDTH = 24,   // Audio sample width (signed Q1.(DATA_WIDTH-1))
    parameter int DELAY_MAX  = 2048, // Delay buffer depth (samples)
    parameter int CTRL_W     = 8,    // Control knob width (unsigned Q0.CTRL_W)
    parameter int LFO_W      = 16    // LFO output width (signed Q1.(LFO_W-1))
)(
    input  logic                          clk,
    input  logic                          rst_n,
    input  logic                          sample_en,
    input  logic signed [DATA_WIDTH-1:0]  audio_in,
    output logic signed [DATA_WIDTH-1:0]  audio_out_l,
    output logic signed [DATA_WIDTH-1:0]  audio_out_r,
    input  logic [CTRL_W-1:0]             rate,
    input  logic [CTRL_W-1:0]             depth,
    input  logic [CTRL_W-1:0]             effect_lvl,
    input  logic [CTRL_W-1:0]             e_q_hi,
    input  logic [CTRL_W-1:0]             e_q_lo
);

    // =========================================================================
    // Q-Format Constants
    // =========================================================================
    localparam int AUDIO_FRAC  = DATA_WIDTH - 1;             // Audio fractional bits
    localparam int LFO_FRAC    = LFO_W - 1;                  // Signed LFO frac bits
    localparam int LFO_PEAK    = (1 << LFO_FRAC) - 1;        // Max positive LFO
    localparam int CTRL_FRAC   = CTRL_W;                      // Q0.CTRL_W shift amount
    localparam int CTRL_UNITY  = 1 << (CTRL_W - 1);          // 128 = unity for EQ gain

    // Mod product: Q1.LFO_FRAC × Q0.(CTRL_W-1) = Q1.(LFO_FRAC+CTRL_W-1)
    // Total width = LFO_W + CTRL_W - 1
    localparam int MOD_PROD_W  = LFO_W + CTRL_W - 1;

    // EQ product: Q1.AUDIO_FRAC × Q1.CTRL_W = Q2.(AUDIO_FRAC+CTRL_W)
    localparam int EQ_PROD_W   = DATA_WIDTH + CTRL_W + 1;
    // EQ shift: CTRL_W-1 so that ctrl=128 → unity (0.5 × 2 = 1.0)
    localparam int EQ_SHIFT    = CTRL_W - 1;

    // Mix product: Q1.AUDIO_FRAC × Q0.CTRL_W = Q1.(AUDIO_FRAC+CTRL_W)
    // Plus dry×CTRL_UNITY, all >> CTRL_W → Q1.AUDIO_FRAC
    localparam int MIX_PROD_W  = DATA_WIDTH + CTRL_W + 11;  // headroom for sum

    // --------------------------------------------------------
    // 1.  LFO (Single Phase) via lfo_core
    //
    // Unified formula: inc = INC_BASE + (rate * INC_SCALE) >> INC_SHIFT
    //   INC_BASE  = 8948 (LFO_INC_MIN)
    //   INC_SCALE = 706880 (LFO_INC_RNG = 715828 - 8948)
    //   INC_SHIFT = 8 (CTRL_W)
    // --------------------------------------------------------
    logic signed [LFO_W-1:0] lfo_val;                      // Q1.LFO_FRAC

    lfo_core #(
        .PHASE_W   (32),
        .CTRL_W    (CTRL_W),
        .LFO_W     (LFO_W),
        .INC_BASE  (8948),
        .INC_SCALE (706880),
        .INC_SHIFT (8),
        .TABLE_BITS(10),
        .WAVE_TYPE ("SINE")
    ) u_chorus_lfo (
        .clk          (clk),
        .rst_n        (rst_n),
        .sample_en    (sample_en),
        .rate_val     (rate),
        .phase_out    (),
        .wave_signed  (lfo_val),
        .wave_unsigned()
    );

    // --------------------------------------------------------
    // 2.  Delay line (Single Mono Tap)
    // --------------------------------------------------------
    localparam int ADDR_W      = $clog2(DELAY_MAX);
    localparam int CENTRE_TAP  = 960;                        // 960 samples = 20 ms centre delay @ 48 kHz
    localparam int MOD_AMP_W   = CTRL_W - 1;                // depth[7:1] → 7-bit

    logic [MOD_AMP_W-1:0] mod_amp;                          // Q0.(CTRL_W-1) unsigned
    assign mod_amp = depth[CTRL_W-1:1];

    // lfo_val [Q1.LFO_FRAC] × mod_amp [Q0.MOD_AMP_W] = Q1.(LFO_FRAC+MOD_AMP_W)
    // >> LFO_FRAC → signed integer sample offset
    logic signed [MOD_PROD_W-1:0] mod_prod;                 // Q1.(LFO_FRAC+MOD_AMP_W)
    logic signed [CTRL_W:0]       mod_off;                   // integer offset

    always_comb begin
        mod_prod = lfo_val * $signed({1'b0, mod_amp});
        // FIX #2: Wrap bit-slice in $signed() to preserve sign on negative
        //         modulation offsets.  Without this, the MSB of mod_off was
        //         zero-filled, turning small negative values into large
        //         positive ones - causing the read pointer to jump hundreds
        //         of samples and producing clicks at the LFO zero-crossing.
        mod_off  = $signed(mod_prod[MOD_PROD_W-1 : LFO_FRAC]);
    end

    logic [ADDR_W-1:0]            wr_ptr;
    logic [ADDR_W-1:0]            wr_ptr_next;
    logic [ADDR_W-1:0]            rd_ptr;
    logic signed [DATA_WIDTH-1:0] wet_raw;                   // Q1.AUDIO_FRAC

    // FIX #1: Use wr_ptr_next instead of wr_ptr so the read address never
    //         collides with the address currently being written.
    always_comb begin
        rd_ptr = wr_ptr_next - ADDR_W'(CENTRE_TAP) - ADDR_W'(mod_off);
    end

    delay_line #(
        .DATA_W   (DATA_WIDTH),
        .DEPTH    (DELAY_MAX),
        .NUM_TAPS (1)
    ) u_delay (
        .clk           (clk),
        .rst_n         (rst_n),
        .wr_en         (sample_en),
        .wr_data       (audio_in),
        .wr_ptr_o      (wr_ptr),
        .wr_ptr_next_o (wr_ptr_next),
        .rd_en         (1'b0),
        .rd_ptr        ('{rd_ptr}),
        .rd_data       ('{wet_raw}),
        .interp_frac   ('0),
        .interp_out    ()
    );

    // --------------------------------------------------------
    // 3.  2-Band EQ
    //
    // Biquad crossover: Q1.17 coefficients (18-bit, fits 1 DSP48E1)
    //   b0 = 0.125 × 2^17, a1_neg = 0.875 × 2^17
    // --------------------------------------------------------
    localparam int XO_CW   = 18;
    localparam int XO_FRAC = 17;
    localparam logic signed [XO_CW-1:0] XO_B0     = XO_CW'(int'(0.125 * (2.0 ** XO_FRAC)));
    localparam logic signed [XO_CW-1:0] XO_A1_NEG = XO_CW'(int'(0.875 * (2.0 ** XO_FRAC)));

    logic signed [DATA_WIDTH-1:0] lpf;                      // Q1.AUDIO_FRAC

    biquad_tdf2 #(
        .DATA_W  (DATA_WIDTH),
        .COEFF_W (XO_CW),
        .FRAC    (XO_FRAC)
    ) crossover_lpf (
        .clk    (clk),    .rst_n (rst_n),   .en (sample_en),
        .x_in   (wet_raw), .y_out (lpf),
        .b0     (XO_B0),   .b1 (XO_CW'(0)), .b2 (XO_CW'(0)),
        .a1_neg (XO_A1_NEG), .a2_neg (XO_CW'(0))
    );

    logic signed [DATA_WIDTH-1:0] low_band;                  // Q1.AUDIO_FRAC
    logic signed [DATA_WIDTH-1:0] high_band;                 // Q1.AUDIO_FRAC

    assign low_band  = lpf;
    assign high_band = wet_raw - lpf;

    // EQ gain: Q1.AUDIO_FRAC × Q1.CTRL_W → Q2.(AUDIO_FRAC+CTRL_W)
    //   >> EQ_SHIFT (= CTRL_W-1) so that knob=128 → unity gain
    logic signed [EQ_PROD_W-1:0] low_eq_prod;               // Q2.(AUDIO_FRAC+CTRL_W)
    logic signed [EQ_PROD_W-1:0] hi_eq_prod;                // Q2.(AUDIO_FRAC+CTRL_W)
    logic signed [DATA_WIDTH-1:0] final_wet;                  // Q1.AUDIO_FRAC

    always_comb begin
        low_eq_prod = $signed(low_band)  * $signed({1'b0, e_q_lo});
        hi_eq_prod  = $signed(high_band) * $signed({1'b0, e_q_hi});
        // >> EQ_SHIFT: knob=CTRL_UNITY(128) → gain 1.0
        final_wet   = DATA_WIDTH'((low_eq_prod + hi_eq_prod) >>> EQ_SHIFT);
    end

    // --------------------------------------------------------
    // 4.  Additive Wet/Dry Mix with Saturation
    //
    // FIX #3: Dry multiplier changed from (CTRL_UNITY <<< 1) = 256 to
    //         CTRL_UNITY = 128.  After the >> CTRL_W normalisation this
    //         gives dry = 0.5 and wet = 0 … ~1.0 (at effect_lvl=255).
    //         Total worst-case peak ≈ 1.5, well within saturation range.
    //         Previously dry = 1.0 + wet ≈ 1.0 = 2.0 caused constant
    //         hard clipping and a noisy/harsh tone.
    // --------------------------------------------------------
    localparam int MIX_W = DATA_WIDTH + CTRL_W + 1;         // headroom for ± summation

    logic signed [MIX_W-1:0] mix_sum_l, mix_sum_r;
    logic signed [MIX_W-1:0] shifted_l, shifted_r;
    logic signed [DATA_WIDTH-1:0] mix_l, mix_r;

    always_comb begin
        // Dry × CTRL_UNITY (128).  Wet × effect_lvl (0-255).
        // Both >> CTRL_W normalises: dry = 0.5, wet = 0 … ~1.0
        mix_sum_l = ($signed(audio_in) * MIX_W'(CTRL_UNITY))
                  + ($signed(final_wet) * $signed({1'b0, effect_lvl}));
        mix_sum_r = ($signed(audio_in) * MIX_W'(CTRL_UNITY))
                  - ($signed(final_wet) * $signed({1'b0, effect_lvl}));

        shifted_l = mix_sum_l >>> CTRL_FRAC;                // >> CTRL_W → Q1.AUDIO_FRAC
        shifted_r = mix_sum_r >>> CTRL_FRAC;
    end

    saturate #(.IN_W(MIX_W), .OUT_W(DATA_WIDTH)) sat_l (.din(shifted_l), .dout(mix_l));
    saturate #(.IN_W(MIX_W), .OUT_W(DATA_WIDTH)) sat_r (.din(shifted_r), .dout(mix_r));

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            audio_out_l <= '0;
            audio_out_r <= '0;
        end else if (sample_en) begin
            audio_out_l <= mix_l;
            audio_out_r <= mix_r;
        end
    end

endmodule