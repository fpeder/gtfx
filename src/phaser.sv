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
// All-pass stage low-frequency difference equation:
//   H(z) = ((1 - c) - z^-1) / (1 - (1 - c)*z^-1)
//   y[n] = x[n] - c*x[n] - x[n-1] + y[n-1] - c*y[n-1]
//
// Break frequency modulation:
//   The JFET maps LFO voltage to resistance nonlinearly.
//   Approximated as:
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
//   speed_val    [7:0] : LFO rate.
//   0 = ~0.5 Hz,  255 = ~5 Hz
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
        // Shift by 1 to restore full 16-bit amplitude [0, 65535]
        lfo_norm  = phase_top[15] ? ~(phase_top << 1) : (phase_top << 1);
    end

    // =========================================================================
    // 2. BREAK FREQUENCY COEFFICIENT
    // =========================================================================
    logic [31:0] lfo_sq;
    logic [15:0] lfo_sq16;    // top 16 bits = lfo_sq in Q16
    logic [15:0] ap_coeff;

    localparam int C_MIN  = 137;
    localparam int C_MAX  = 2731;
    localparam int C_SPAN = C_MAX - C_MIN;

    always_comb begin
        lfo_sq    = lfo_norm * lfo_norm;
        lfo_sq16  = lfo_sq[31:16];
        // ap_coeff = C_MIN + C_SPAN * lfo_sq16 (result is Q16)
        ap_coeff  = 16'(C_MIN) + 16'((C_SPAN * {16'h0, lfo_sq16}) >> 16);
    end

    // =========================================================================
    // 3. ALL-PASS STAGE STATES
    // =========================================================================
    logic signed [WIDTH-1:0] x_prev [0:3];
    logic signed [WIDTH-1:0] y_prev [0:3];
    logic signed [WIDTH-1:0] y_next [0:3];
    logic signed [WIDTH-1:0] stage_in [0:3];
    
    // =========================================================================
    // 4. FEEDBACK PATH (R28 - Block Logo)
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
    // Fixed low-frequency equation:
    // y[n] = x[n] - c*x[n] - x[n-1] + y[n-1] - c*y[n-1]
    // =========================================================================
    genvar s;
    generate
        for (s = 0; s < 4; s++) begin : gen_ap

            // Product width: 17-bit signed coeff * 24-bit signed sample = 41 bits
            logic signed [WIDTH+16:0] cx;
            logic signed [WIDTH+16:0] cy;
            logic signed [MWIDTH-1:0] ap_sum;

            always_comb begin
                cx     = $signed({1'b0, ap_coeff}) * $signed(stage_in[s]);
                cy     = $signed({1'b0, ap_coeff}) * $signed(y_prev[s]);
                
                // y[n] = x[n] - c*x[n] - x[n-1] + y[n-1] - c*y[n-1]
                ap_sum = $signed({{16{stage_in[s][WIDTH-1]}}, stage_in[s]})
                       - $signed(cx[WIDTH+16:16])
                       - $signed({{16{x_prev[s][WIDTH-1]}}, x_prev[s]})
                       + $signed({{16{y_prev[s][WIDTH-1]}}, y_prev[s]})
                       - $signed(cy[WIDTH+16:16]);
                       
                if      (ap_sum > SAT_MAX_W) y_next[s] = SAT_MAX;
                else if (ap_sum < SAT_MIN_W) y_next[s] = SAT_MIN;
                else                         y_next[s] = ap_sum[WIDTH-1:0];
            end
        end
    endgenerate

    // =========================================================================
    // 6. OUTPUT MIXER
    // =========================================================================
    logic signed [WIDTH:0]   mix_sum;
    logic signed [WIDTH-1:0] mix_out;

    always_comb begin
        // 50/50: parenthesise each shift explicitly to avoid precedence bugs.
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