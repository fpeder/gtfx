`timescale 1ns / 1ps

// MXR Phase 90 Style Phaser
//
// Fixed-point format summary (parametric):
//   Audio:       signed  Q1.(WIDTH-1)               e.g. Q1.23 for WIDTH=24
//   LFO signed:  signed  Q1.(LFO_W-1)               e.g. Q1.15 for LFO_W=16
//   lfo_norm:    unsigned Q0.LFO_W                   range [0, 2·LFO_PEAK]
//   ap_coeff c:  unsigned Q0.COEFF_W                 range [C_MIN, C_MAX]
//   c×sample:    signed  Q1.(WIDTH-1+COEFF_W)        >> COEFF_W → Q1.(WIDTH-1)
//   Accumulator: signed  (WIDTH+COEFF_W) bits        with COEFF_W guard bits
//
// Controls (CTRL_W-bit unsigned, default 8):
//   speed_val   : LFO rate        0 ≈ 0.5 Hz, max ≈ 5 Hz
//   feedback_en : 0 = Script (no R28), 1 = Block (R28)

module phaser #(
    parameter int WIDTH   = 24,  // Audio sample width (signed Q1.(WIDTH-1))
    parameter int CTRL_W  = 8,   // Control knob width (unsigned)
    parameter int LFO_W   = 16,  // LFO output width (signed Q1.(LFO_W-1))
    parameter int COEFF_W = 16   // All-pass coefficient width (unsigned Q0.COEFF_W)
)(
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    sample_en,

    input  logic [CTRL_W-1:0]       speed_val,
    input  logic                    feedback_en,

    input  logic signed [WIDTH-1:0] audio_in,
    output logic signed [WIDTH-1:0] audio_out
);

    // =========================================================================
    // Q-Format Constants
    // =========================================================================
    localparam int AUDIO_FRAC = WIDTH - 1;                  // Q1.(WIDTH-1) frac bits
    localparam int LFO_FRAC   = LFO_W - 1;                  // Q1.(LFO_W-1) frac bits
    localparam int LFO_PEAK   = (1 << LFO_FRAC) - 1;        // Max positive LFO value

    // All-pass products: Q0.COEFF_W × Q1.AUDIO_FRAC → Q1.(COEFF_W+AUDIO_FRAC)
    // Total product width = COEFF_W + 1 (sign-ext) + WIDTH = WIDTH + COEFF_W + 1
    localparam int AP_PROD_W  = WIDTH + COEFF_W + 1;
    // Accumulator with COEFF_W guard bits for multi-term summation
    localparam int ACC_W      = WIDTH + COEFF_W;

    // =========================================================================
    // 1. LFO - TRIANGLE WAVE VIA lfo_core
    // =========================================================================
    logic [LFO_W-1:0] lfo_norm;                             // Q0.LFO_W unsigned

    lfo_core #(
        .PHASE_W   (32),
        .CTRL_W    (CTRL_W),
        .LFO_W     (LFO_W),
        .INC_BASE  (44739),     // 0.5 Hz × 2^32 / 48000
        .INC_SCALE (1580),      // (5Hz - 0.5Hz) × 2^32 / 48000 / 255
        .INC_SHIFT (0),
        .TABLE_BITS(8),
        .WAVE_TYPE ("TRIANGLE")
    ) u_phaser_lfo (
        .clk          (clk),
        .rst_n        (rst_n),
        .sample_en    (sample_en),
        .rate_val     (speed_val),
        .phase_out    (),
        .wave_signed  (),
        .wave_unsigned(lfo_norm)
    );

    // =========================================================================
    // 2. BREAK FREQUENCY COEFFICIENT
    //
    // lfo_norm [Q0.LFO_W] squared → lfo_sq [Q0.(2·LFO_W)]
    // Extract top LFO_W bits → lfo_sq_q [Q0.LFO_W]
    // ap_coeff [Q0.COEFF_W] = C_MIN + C_SPAN × lfo_sq_q >> LFO_W
    // =========================================================================
    localparam int C_MIN  = 137;
    localparam int C_MAX  = 2731;
    localparam int C_SPAN = C_MAX - C_MIN;

    logic [2*LFO_W-1:0]    lfo_sq;                         // Q0.(2·LFO_W)
    logic [LFO_W-1:0]      lfo_sq_q;                       // Q0.LFO_W (top half)
    logic [COEFF_W-1:0]    ap_coeff;                        // Q0.COEFF_W

    always_comb begin
        lfo_sq    = lfo_norm * lfo_norm;                    // Q0.LFO_W × Q0.LFO_W
        lfo_sq_q  = lfo_sq[2*LFO_W-1 : LFO_W];            // >> LFO_W → Q0.LFO_W
        // C_MIN + (C_SPAN × lfo_sq_q) >> LFO_W → integer in [C_MIN, C_MAX]
        ap_coeff  = COEFF_W'(C_MIN) + COEFF_W'((C_SPAN * {{LFO_W{1'b0}}, lfo_sq_q}) >> LFO_W);
    end

    // =========================================================================
    // 3. ALL-PASS STAGE STATES
    // =========================================================================
    localparam int NUM_STAGES = 4;

    logic signed [WIDTH-1:0] x_prev    [0:NUM_STAGES-1];   // Q1.AUDIO_FRAC
    logic signed [WIDTH-1:0] y_prev    [0:NUM_STAGES-1];   // Q1.AUDIO_FRAC
    logic signed [WIDTH-1:0] y_next    [0:NUM_STAGES-1];   // Q1.AUDIO_FRAC
    logic signed [WIDTH-1:0] stage_in  [0:NUM_STAGES-1];   // Q1.AUDIO_FRAC

    // =========================================================================
    // 4. FEEDBACK PATH (R28 - Block Logo)
    // =========================================================================
    logic signed [WIDTH-1:0] fb_val;                        // Q1.AUDIO_FRAC
    logic signed [WIDTH:0]   stage1_sum;                    // Q1.AUDIO_FRAC + 1 guard
    logic signed [WIDTH-1:0] stage1_sat;                    // Q1.AUDIO_FRAC

    always_comb begin
        // Feedback: y_prev[NUM_STAGES-1] × 0.5 (>>> 1)
        fb_val     = feedback_en ? $signed(y_prev[NUM_STAGES-1]) >>> 1 : '0;

        stage_in[0] = audio_in;
        stage1_sum  = $signed(y_next[0]) + $signed(fb_val);
        stage_in[1] = stage1_sat;
        stage_in[2] = y_next[1];
        stage_in[3] = y_next[2];
    end

    saturate #(.IN_W(WIDTH+1), .OUT_W(WIDTH)) sat_fb (
        .din(stage1_sum), .dout(stage1_sat)
    );

    // =========================================================================
    // 5. ALL-PASS DATAPATH
    //
    // y[n] = x[n] - c·x[n] - x[n-1] + y[n-1] - c·y[n-1]
    //
    // Products c×sample:
    //   Q0.COEFF_W (unsigned, sign-extended to Q1.COEFF_W) × Q1.AUDIO_FRAC
    //   = Q1.(COEFF_W + AUDIO_FRAC) in AP_PROD_W bits
    //   >> COEFF_W extracts Q1.AUDIO_FRAC with COEFF_W guard bits → ACC_W bits
    //
    // Accumulator ap_sum [ACC_W bits] holds all 5 terms at Q1.AUDIO_FRAC
    //   with COEFF_W guard bits, then saturated back to WIDTH.
    // =========================================================================
    genvar s;
    generate
        for (s = 0; s < NUM_STAGES; s++) begin : gen_ap

            // c × sample products: (COEFF_W+1)-bit signed × WIDTH-bit signed
            logic signed [AP_PROD_W-1:0] cx;                // Q1.(COEFF_W+AUDIO_FRAC)
            logic signed [AP_PROD_W-1:0] cy;                // Q1.(COEFF_W+AUDIO_FRAC)
            logic signed [ACC_W-1:0]     ap_sum;             // Q1.AUDIO_FRAC + guard

            always_comb begin
                // Q0.COEFF_W → Q1.COEFF_W (sign-extend with 0), then multiply
                cx     = $signed({1'b0, ap_coeff}) * $signed(stage_in[s]);
                cy     = $signed({1'b0, ap_coeff}) * $signed(y_prev[s]);

                // All terms at ACC_W bits: sign-extend audio from WIDTH, shift products >> COEFF_W
                ap_sum = $signed({{COEFF_W{stage_in[s][WIDTH-1]}}, stage_in[s]})
                       - $signed(cx[AP_PROD_W-1 : COEFF_W])
                       - $signed({{COEFF_W{x_prev[s][WIDTH-1]}}, x_prev[s]})
                       + $signed({{COEFF_W{y_prev[s][WIDTH-1]}}, y_prev[s]})
                       - $signed(cy[AP_PROD_W-1 : COEFF_W]);
            end

            saturate #(.IN_W(ACC_W), .OUT_W(WIDTH)) sat_ap (
                .din(ap_sum), .dout(y_next[s])
            );
        end
    endgenerate

    // =========================================================================
    // 6. OUTPUT MIXER - 50/50 dry + wet
    // =========================================================================
    logic signed [WIDTH:0]   mix_sum;                       // Q1.AUDIO_FRAC + 1 guard
    logic signed [WIDTH-1:0] mix_out;                       // Q1.AUDIO_FRAC

    always_comb begin
        mix_sum = ($signed(audio_in)              >>> 1)
                + ($signed(y_next[NUM_STAGES-1]) >>> 1);
        mix_out = mix_sum[WIDTH-1:0];
    end

    // =========================================================================
    // 7. OUTPUT REGISTER
    // =========================================================================
    logic signed [WIDTH-1:0] audio_out_reg;
    assign audio_out = audio_out_reg;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            audio_out_reg <= '0;
            for (int i = 0; i < NUM_STAGES; i++) begin
                x_prev[i] <= '0;
                y_prev[i] <= '0;
            end
        end else if (sample_en) begin
            audio_out_reg <= mix_out;
            for (int i = 0; i < NUM_STAGES; i++) begin
                x_prev[i] <= stage_in[i];
                y_prev[i] <= y_next[i];
            end
        end
    end

endmodule