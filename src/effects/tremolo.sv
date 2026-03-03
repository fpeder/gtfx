`timescale 1ns / 1ps

// Optical Tremolo (Fender/Vox style)
//
// Fixed-point format summary (parametric):
//   Audio:       signed  Q1.(WIDTH-1)          e.g. Q1.23 for WIDTH=24
//   LFO signed:  signed  Q1.(LFO_W-1)          e.g. Q1.15 for LFO_W=16
//   lfo_norm:    unsigned Q0.LFO_W              range [0, 2·LFO_PEAK]
//   depth_norm:  unsigned Q0.LFO_W              range [0, 2^LFO_W - 1]
//   gain:        unsigned Q1.LFO_W              range [0, GAIN_UNITY]
//   AM product:  signed  Q2.(WIDTH-1+LFO_W)    >> LFO_W → Q1.(WIDTH-1)
//
// Controls (CTRL_W-bit unsigned, default 8):
//   rate_val  : LFO rate        0 ≈ 0.5 Hz, max ≈ 12 Hz
//   depth_val : Effect depth    0 = bypass, max = full chop
//   shape_sel : 0 = sine, 1 = triangle

module tremolo #(
    parameter int WIDTH  = 24,   // Audio sample width (signed Q1.(WIDTH-1))
    parameter int CTRL_W = 8,    // Control knob width (unsigned Q0.CTRL_W)
    parameter int LFO_W  = 16    // LFO / gain fractional width
)(
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    sample_en,

    input  logic [CTRL_W-1:0]       rate_val,
    input  logic [CTRL_W-1:0]       depth_val,
    input  logic                    shape_sel,

    input  logic signed [WIDTH-1:0] audio_in,
    output logic signed [WIDTH-1:0] audio_out
);

    // =========================================================================
    // Q-Format Constants (all derived from parameters)
    // =========================================================================
    localparam int AUDIO_FRAC  = WIDTH - 1;               // Audio fractional bits
    localparam int LFO_FRAC    = LFO_W - 1;               // Signed LFO frac bits
    localparam int LFO_PEAK    = (1 << LFO_FRAC) - 1;     // Max positive signed LFO
    localparam int GAIN_FRAC   = LFO_W;                    // Gain Q1.GAIN_FRAC
    localparam int GAIN_UNITY  = 1 << GAIN_FRAC;           // 1.0 in Q1.GAIN_FRAC
    localparam int CTRL_MAX    = (1 << CTRL_W) - 1;        // Max control value (255)
    localparam int DEPTH_SCALE = GAIN_UNITY / CTRL_MAX;    // Ctrl → Q0.LFO_W scale

    // AM product: Q1.AUDIO_FRAC × Q1.GAIN_FRAC → Q2.(AUDIO_FRAC+GAIN_FRAC)
    localparam int AM_PROD_W = WIDTH + GAIN_FRAC + 2;

    // =========================================================================
    // 1. PHASE ACCUMULATOR + TRIANGLE via lfo_core
    // =========================================================================
    localparam int LFO_PHASE_W = 32;

    logic [LFO_PHASE_W-1:0] lfo_phase;
    logic [LFO_W-1:0]       lfo_tri;                       // Q0.LFO_W unsigned

    lfo_core #(
        .PHASE_W   (LFO_PHASE_W),
        .CTRL_W    (CTRL_W),
        .LFO_W     (LFO_W),
        .INC_BASE  (44739),     // 0.5 Hz × 2^32 / 48000
        .INC_SCALE (4035),      // (12Hz - 0.5Hz) × 2^32 / 48000 / 255
        .INC_SHIFT (0),
        .TABLE_BITS(8),
        .WAVE_TYPE ("TRIANGLE")
    ) u_trem_lfo (
        .clk          (clk),
        .rst_n        (rst_n),
        .sample_en    (sample_en),
        .rate_val     (rate_val),
        .phase_out    (lfo_phase),
        .wave_signed  (),
        .wave_unsigned(lfo_tri)
    );

    // =========================================================================
    // 2. LFO WAVEFORM SELECTION
    //
    // Triangle comes from lfo_core (wave_unsigned).
    // Sine (unipolar |sin| hump) uses a separate waveform_lut driven by phase_out.
    //   |Q1.(LFO_W-1)| << 1 → Q0.LFO_W unsigned, range [0, 2·LFO_PEAK]
    // =========================================================================
    logic [LFO_W-1:0] lfo_sine;                           // Q0.LFO_W unsigned
    logic [LFO_W-1:0] lfo_norm;                           // Q0.LFO_W unsigned

    logic signed [LFO_W-1:0] lfo_sine_signed;             // Q1.(LFO_W-1) signed

    waveform_lut #(
        .TABLE_BITS (8),
        .OUT_WIDTH  (LFO_W),
        .WAVE_TYPE  ("SINE"),
        .PHASE_W    (LFO_PHASE_W)
    ) u_trem_sine (
        .phase_in  (lfo_phase),
        .wave_out  (lfo_sine_signed),
        .frac_out  ()
    );

    // |Q1.(LFO_W-1)| << 1 → Q0.LFO_W unsigned, range [0, 2·LFO_PEAK]
    logic [LFO_W-1:0] lfo_sine_abs;
    assign lfo_sine_abs = lfo_sine_signed[LFO_W-1] ? $unsigned(-lfo_sine_signed)
                                                    : $unsigned( lfo_sine_signed);
    assign lfo_sine = {lfo_sine_abs[LFO_W-2:0], 1'b0};

    always_comb begin
        lfo_norm = shape_sel ? lfo_tri : lfo_sine;
    end

    // =========================================================================
    // 3. GAIN COMPUTATION  (all unsigned Q0.LFO_W or Q1.LFO_W)
    //
    // gain [Q1.GAIN_FRAC] = 1.0 - depth_norm × (1.0 - lfo_norm)
    //
    //   depth_norm   [Q0.LFO_W]:   depth_val × DEPTH_SCALE
    //   one_minus_lfo[Q1.LFO_W]:   GAIN_UNITY - lfo_norm
    //   reduction    [Q0.2·LFO_W]: depth_norm × one_minus_lfo
    //   gain         [Q1.LFO_W]:   GAIN_UNITY - (reduction >> LFO_W)
    // =========================================================================
    logic [LFO_W:0]       depth_norm;                      // Q0.LFO_W in (LFO_W+1) bits
    logic [LFO_W:0]       one_minus_lfo;                   // Q1.LFO_W
    logic [2*LFO_W+1:0]  mod_reduction;                   // Q0.(2·LFO_W)
    logic [LFO_W:0]       gain;                            // Q1.GAIN_FRAC

    always_comb begin
        depth_norm    = (LFO_W+1)'(depth_val) * (LFO_W+1)'(DEPTH_SCALE);
        one_minus_lfo = (LFO_W+1)'(GAIN_UNITY) - {1'b0, lfo_norm};
        mod_reduction = {(LFO_W+1)'(0), depth_norm} * {(LFO_W+1)'(0), one_minus_lfo};
        // >> LFO_W to extract Q0.LFO_W portion of the Q0.(2·LFO_W) product
        gain          = (LFO_W+1)'(GAIN_UNITY) - (LFO_W+1)'(mod_reduction[2*LFO_W-1 : LFO_W]);
    end

    // =========================================================================
    // 4. AMPLITUDE MODULATION
    //
    // Q1.AUDIO_FRAC × Q1.GAIN_FRAC → Q2.(AUDIO_FRAC + GAIN_FRAC)
    // Extract [AUDIO_FRAC+GAIN_FRAC : GAIN_FRAC] → Q1.AUDIO_FRAC (= WIDTH bits)
    // =========================================================================
    logic signed [AM_PROD_W-1:0] product;                  // Q2.(AUDIO_FRAC+GAIN_FRAC)
    logic signed [WIDTH-1:0]     am_out;                   // Q1.AUDIO_FRAC

    always_comb begin
        product = $signed(audio_in) * $signed({1'b0, gain});
        am_out  = product[AUDIO_FRAC + GAIN_FRAC : GAIN_FRAC];
    end

    // =========================================================================
    // 5. OUTPUT REGISTER
    // =========================================================================
    logic signed [WIDTH-1:0] audio_out_reg;
    assign audio_out = audio_out_reg;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            audio_out_reg <= '0;
        end else if (sample_en) begin
            audio_out_reg <= am_out;
        end
    end

endmodule