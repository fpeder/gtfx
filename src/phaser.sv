`timescale 1ns / 1ps

// MXR Phase 90 Style Phaser
//
// Clocked on clk_audio (12.288 MHz).
// sample_en is a 1-cycle pulse at fs (48 kHz).
//
// Circuit topology (Block Logo version):
//   - 4 cascaded first-order all-pass stages
//   - All 4 stages share the same LFO-modulated break frequency
//   - Feedback from stage 4 output back to stage 2 input (R28, switchable)
//   - Output mixer: 50/50 sum of dry (audio_in) and wet (stage 4 output)
//     This 50/50 mix is what creates the notches in the frequency response.
//     Without this mix there is no phaser effect, only phase rotation.
//
// All-pass stage bilinear transform of H(s) = (s - wb) / (s + wb):
//   H(z) = (1 - c) / (1 + c)  where  c = wb * T / 2  = wb / (2*fs)
//   Difference equation:
//     y[n] = c * (x[n] - y[n-1]) + x[n-1]   ... (direct form)
//     which expands to:  y[n] = c*x[n] + x[n-1] - c*y[n-1]
//
// Break frequency modulation:
//   The JFET maps LFO voltage to resistance nonlinearly. Approximated as:
//     wb = wb_min + (wb_max - wb_min) * lfo_norm^2
//   where lfo_norm is the triangle LFO normalised to [0, 1].
//   Analog Phase 90 range: ~100 Hz to ~2 kHz break frequency.
//
// Fixed-point:
//   Audio samples:  signed Q0.23  (24-bit, full scale = +-1.0)
//   Coefficient c:  unsigned 16-bit, representing [0, 1) as [0, 65535]
//   Products:       40-bit before shift, truncated back to 24-bit
//
// Controls:
//   speed_val    [7:0] : LFO rate.  0 = ~0.5 Hz,  255 = ~5 Hz
//   feedback_en  [0]   : enable R28 feedback (0 = Script mode, 1 = Block mode)
//
// Ports match dd3.sv conventions.
//
module phaser #(
    parameter WIDTH = 24
)(
    input  logic             clk,        // clk_audio (12.288 MHz)
    input  logic             rst_n,
    input  logic             sample_en,  // 1-cycle pulse at fs (48 kHz)

    // Controls - synchronise to clk before connecting
    input  logic [7:0]       speed_val,   // LFO rate
    input  logic             feedback_en, // 0 = script (no R28), 1 = block (R28)

    // Audio
    input  logic signed [WIDTH-1:0] audio_in,
    output logic signed [WIDTH-1:0] audio_out
);

    // =========================================================================
    // 0. SATURATION CONSTANTS
    // =========================================================================
    localparam signed [WIDTH-1:0]  SAT_MAX   = {1'b0, {(WIDTH-1){1'b1}}};
    localparam signed [WIDTH-1:0]  SAT_MIN   = {1'b1, {(WIDTH-1){1'b0}}};
    // Extended width for multiply-accumulate: WIDTH + 16 coefficient bits
    localparam int                 MWIDTH    = WIDTH + 16;
    localparam signed [MWIDTH-1:0] SAT_MAX_W = {{16{1'b0}}, SAT_MAX};
    localparam signed [MWIDTH-1:0] SAT_MIN_W = {{16{1'b1}}, SAT_MIN};

    // =========================================================================
    // 1. LFO - TRIANGLE WAVE VIA PHASE ACCUMULATOR
    //
    // Analog LFO: triangle, ~0.5 Hz to ~5 Hz.
    // Phase accumulator: 32 bits.
    // Increment per sample = lfo_freq * 2^32 / fs
    //   0.5 Hz -> inc =  0.5 * 2^32 / 48000 ~ 44739
    //   5.0 Hz -> inc = 5.0  * 2^32 / 48000 ~ 447392
    //
    // speed_val maps [0,255] -> [44739, 447392] linearly:
    //   inc = 44739 + speed_val * 1580
    //
    // Triangle: fold the top half of phase range.
    // Output lfo_norm is unsigned 16-bit [0, 65535] representing [0, 1).
    // =========================================================================
    logic [31:0] lfo_phase      = 32'h0;
    logic [31:0] lfo_phase_next;
    logic [31:0] lfo_inc;
    logic [15:0] lfo_norm;   // unsigned [0, 65535]

    always_comb begin
        // inc = 44739 + speed_val * 1580
        lfo_inc        = 32'd44739 + ({24'h0, speed_val} * 32'd1580);
        lfo_phase_next = lfo_phase + lfo_inc;
    end

    always_comb begin
        logic [15:0] phase_top;
        phase_top = lfo_phase[31:16];
        // Mirror top half to produce triangle
        lfo_norm  = phase_top[15] ? ~phase_top : phase_top;
    end

    // =========================================================================
    // 2. BREAK FREQUENCY COEFFICIENT
    //
    // JFET nonlinearity approximated as quadratic:
    //   wb_norm = wb_min_norm + (wb_max_norm - wb_min_norm) * lfo_norm^2
    //
    // Target break frequency range: 100 Hz to 2000 Hz.
    // Coefficient c = wb / (2*fs) = f_break / fs
    //   c_min = 100  / 48000 ~ 0.00208  -> Q16 = 137
    //   c_max = 2000 / 48000 ~ 0.04167  -> Q16 = 2731
    //
    // lfo_norm^2 is 32-bit; top 16 bits give norm^2 in Q16.
    // c = 137 + (2731 - 137) * lfo_sq16 = 137 + 2594 * lfo_sq16
    // =========================================================================
    logic [31:0] lfo_sq;      // lfo_norm^2, Q32
    logic [15:0] lfo_sq16;    // top 16 bits = lfo_sq in Q16
    logic [15:0] ap_coeff;    // c in Q16 unsigned [0, 65535]

    localparam int C_MIN  = 137;
    localparam int C_MAX  = 2731;
    localparam int C_SPAN = C_MAX - C_MIN; // 2594

    always_comb begin
        lfo_sq    = lfo_norm * lfo_norm;       // 32-bit, Q32
        lfo_sq16  = lfo_sq[31:16];             // Q16
        // ap_coeff = C_MIN + C_SPAN * lfo_sq16 (result is Q16)
        ap_coeff  = 16'(C_MIN) + 16'((C_SPAN * {16'h0, lfo_sq16}) >> 16);
    end

    // =========================================================================
    // 3. ALL-PASS STAGE STATES
    //
    // 4 stages. Each holds:
    //   x_prev[s] : x[n-1]
    //   y_prev[s] : y[n-1]
    // =========================================================================
    logic signed [WIDTH-1:0] x_prev [0:3];
    logic signed [WIDTH-1:0] y_prev [0:3];
    logic signed [WIDTH-1:0] y_next [0:3];

    // Stage inputs: stage_in[0] is the input to stage 0 (after feedback),
    // stage_in[1..3] are driven by the previous stage output.
    // Feedback (R28) adds a fraction of stage 4 output back to stage 2 input.
    logic signed [WIDTH-1:0] stage_in [0:3];

    // =========================================================================
    // 4. FEEDBACK PATH (R28 - Block Logo)
    //
    // In the Block Logo, R28 = 24K feeds stage 4 output back to the input
    // of stage 2 summing node. The mixing resistors are all equal so the
    // feedback fraction is 0.5 relative to the forward signal at that node.
    // Represented here as a right-shift by 1 (divide by 2).
    //
    // Stage 0 input: audio_in (no feedback at stage 0)
    // Stage 1 input: y_next[0] + feedback_en ? (y_prev[3] >> 1) : 0
    // Stages 2,3:    previous stage output only
    //
    // Uses y_prev[3] (registered) not y_next[3] to avoid combinatorial loop.
    // =========================================================================
    logic signed [WIDTH-1:0]   fb_val;
    logic signed [WIDTH:0]     stage1_sum;
    logic signed [WIDTH-1:0]   stage1_sat;

    always_comb begin
        // Feedback value: half of last-stage registered output
        fb_val     = feedback_en ? $signed(y_prev[3]) >>> 1 : '0;

        // Stage 0 input is audio_in directly
        stage_in[0] = audio_in;

        // Stage 1 input: stage 0 output + feedback
        stage1_sum  = $signed(y_next[0]) + $signed(fb_val);
        if      (stage1_sum > $signed({1'b0, SAT_MAX})) stage1_sat = SAT_MAX;
        else if (stage1_sum < $signed({1'b1, SAT_MIN})) stage1_sat = SAT_MIN;
        else                                             stage1_sat = stage1_sum[WIDTH-1:0];
        stage_in[1] = stage1_sat;

        // Stages 2 and 3 are fed directly from previous stage
        stage_in[2] = y_next[1];
        stage_in[3] = y_next[2];
    end

    // =========================================================================
    // 5. ALL-PASS DATAPATH
    //
    // y[n] = c*x[n] + x[n-1] - c*y[n-1]
    //
    // c is Q16 unsigned. x,y are signed Q0.(WIDTH-1).
    // Product is WIDTH+16 bits; discard bottom 16 bits (>> 16) to get
    // back to WIDTH-bit result.
    //
    // SAT_MAX_W / SAT_MIN_W are MWIDTH = WIDTH+16 wide, sign-extended
    // from the WIDTH-bit saturation limits.
    // =========================================================================
    genvar s;
    generate
        for (s = 0; s < 4; s++) begin : gen_ap

            // Product width: 17-bit signed coeff * 24-bit signed sample = 41 bits
            logic signed [WIDTH+16:0] cx;      //  c * x[n]
            logic signed [WIDTH+16:0] cy;      //  c * y[n-1]
            logic signed [MWIDTH-1:0] ap_sum;  // cx + x[n-1] - cy  (MWIDTH = WIDTH+16)

            always_comb begin
                cx     = $signed({1'b0, ap_coeff}) * $signed(stage_in[s]);
                cy     = $signed({1'b0, ap_coeff}) * $signed(y_prev[s]);

                // y[n] = c*x[n] + x[n-1] - c*y[n-1]
                // Right-shift by 16 to remove Q16 fractional bits.
                ap_sum = $signed(cx[WIDTH+16:16])
                       + $signed({{16{x_prev[s][WIDTH-1]}}, x_prev[s]})
                       - $signed(cy[WIDTH+16:16]);

                if      (ap_sum > SAT_MAX_W) y_next[s] = SAT_MAX;
                else if (ap_sum < SAT_MIN_W) y_next[s] = SAT_MIN;
                else                         y_next[s] = ap_sum[WIDTH-1:0];
            end
        end
    endgenerate

    // =========================================================================
    // 6. OUTPUT MIXER
    //
    // The phaser notch comes from summing dry + wet at equal gain (50/50).
    // This is R8/R16 in the analog circuit - equal resistors to the PNP base.
    // Because both inputs are halved before summing, the output is in range.
    // No saturation needed here since each path is <= full scale and they
    // are added at half gain each: sum = (dry + wet) / 2.
    //
    // Note: summing at 50/50 attenuates the overall level by 3 dB when the
    // two signals are correlated. This matches the analog circuit behaviour.
    // =========================================================================
    logic signed [WIDTH:0]   mix_sum;
    logic signed [WIDTH-1:0] mix_out;

    always_comb begin
        // 50/50: parenthesise each shift explicitly to avoid precedence bugs.
        // >>> binds tighter than + so without parens:
        //   a >>> 1 + b >>> 1  parses as  a >>> (1 + b) >>> 1  -- wrong.
        mix_sum = ($signed(audio_in)  >>> 1)
                + ($signed(y_next[3]) >>> 1);
        mix_out = mix_sum[WIDTH-1:0];
    end

    // =========================================================================
    // 7. OUTPUT REGISTER
    // =========================================================================
    logic signed [WIDTH-1:0] audio_out_reg = '0;
    assign audio_out = audio_out_reg;

    // =========================================================================
    // 8. SEQUENTIAL LOGIC - single always_ff, updates on sample_en only
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            lfo_phase     <= '0;
            audio_out_reg <= '0;
            for (int i = 0; i < 4; i++) begin
                x_prev[i] <= '0;
                y_prev[i] <= '0;
            end
        end else if (sample_en) begin
            lfo_phase     <= lfo_phase_next;
            audio_out_reg <= mix_out;
            for (int i = 0; i < 4; i++) begin
                x_prev[i] <= stage_in[i];
                y_prev[i] <= y_next[i];
            end
        end
    end

endmodule