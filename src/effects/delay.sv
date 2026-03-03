`timescale 1ns / 1ps

// ============================================================================
// boss_dd3.sv - Boss DD-3 Digital Delay Effect Processor
//
// Interface:
//   - feedback   [7:0]   0 = single echo, 255 ≈ infinite repeats
//   - effect_lvl [7:0]   0 = full dry, 255 = full wet
//   - tone       [7:0]   feedback tone: 0 = dark (heavy LPF), 255 = bright
//   - delay_time [15:0]  delay in samples (600..38400 @ 48 kHz)
//
// Architecture:
//   - 100% dry pass-through + variable wet mix
//   - Analog-style feedback with saturation per repeat
//   - Variable tone LPF on feedback path (models 12-bit bandwidth +
//     the DD-3's characteristic darkening on repeats)
//   - 2-phase BRAM pipeline via delay_line (REG_RD=1)
//
// Latency: 3 clk cycles from sample_en to valid audio_out.
// ============================================================================

module delay #(
    parameter int AUDIO_W   = 24,
    parameter int RAM_DEPTH = 65536
) (
    input  logic                          clk,
    input  logic                          rst_n,

    input  logic                          sample_en,

    input  logic signed [AUDIO_W-1:0]     audio_in,
    output logic signed [AUDIO_W-1:0]     audio_out,

    input  logic [15:0]                   time_val,
    input  logic [7:0]                    feedback_val,
    input  logic [7:0]                    level_val,
    input  logic [7:0]                    tone_val
);

    localparam int ADDR_W = $clog2(RAM_DEPTH);

    // ================================================================
    // Pipeline strobes
    // ================================================================
    logic sample_en_d1, sample_en_d2;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            sample_en_d1 <= 1'b0;
            sample_en_d2 <= 1'b0;
        end else begin
            sample_en_d1 <= sample_en;
            sample_en_d2 <= sample_en_d1;
        end
    end

    // ================================================================
    // Read pointer
    // ================================================================
    logic [ADDR_W-1:0] wr_ptr;
    logic [ADDR_W-1:0] rd_ptr;

    assign rd_ptr = wr_ptr - ADDR_W'(time_val);

    // ================================================================
    // Delay line (BRAM, 1 tap, registered read)
    // ================================================================
    logic signed [AUDIO_W-1:0] delay_raw;
    logic signed [AUDIO_W-1:0] fb_write_data;

    delay_line #(
        .DATA_W    (AUDIO_W),
        .DEPTH     (RAM_DEPTH),
        .NUM_TAPS  (1),
        .REG_RD    (1),
        .INTERP_EN (0)
    ) u_delay (
        .clk            (clk),
        .rst_n          (rst_n),
        .wr_en          (sample_en_d1),
        .wr_data        (fb_write_data),
        .wr_ptr_o       (wr_ptr),
        .wr_ptr_next_o  (),
        .rd_en          (sample_en),
        .rd_ptr         ('{rd_ptr}),
        .rd_data        ('{delay_raw}),
        .interp_frac    ('0),
        .interp_out     ()
    );

    // ================================================================
    // Tone control — variable single-pole LPF on feedback path
    //
    //   y += alpha * (x - y)
    //
    //   alpha = (tone + 48) / 512          (Q9 fixed-point)
    //   tone = 0   → alpha = 48/512  ≈ 0.094  (~720 Hz @ 48 kHz)
    //   tone = 255 → alpha = 303/512 ≈ 0.592  (~6.8 kHz)
    // ================================================================
    logic signed [AUDIO_W-1:0]       lpf_state;
    logic signed [AUDIO_W:0]         lpf_diff;
    logic signed [AUDIO_W+9:0]       lpf_prod;
    logic signed [AUDIO_W-1:0]       delay_filt;

    always_comb begin
        lpf_diff = $signed({delay_raw[AUDIO_W-1], delay_raw}) -
                   $signed({lpf_state[AUDIO_W-1], lpf_state});
        lpf_prod = lpf_diff * $signed({2'b0, {1'b0, tone_val} + 9'd48}); // alpha = (tone+48)/512
    end

    always_ff @(posedge clk) begin
        if (!rst_n)
            lpf_state <= '0;
        else if (sample_en_d1)
            lpf_state <= lpf_state + AUDIO_W'((lpf_prod + (1 << 8)) >>> 9); // Q9 round
    end

    assign delay_filt = lpf_state;

    // ================================================================
    // Delayed input register
    // ================================================================
    logic signed [AUDIO_W-1:0] audio_in_d1;

    always_ff @(posedge clk) begin
        if (!rst_n) audio_in_d1 <= '0;
        else if (sample_en) audio_in_d1 <= audio_in;
    end

    // ================================================================
    // Feedback path  (8-bit gain, saturated write-back)
    //   fb_scaled = delay_filt * feedback / 256
    //   write     = sat(fb_scaled + audio_in)
    // ================================================================
    logic signed [AUDIO_W+8:0]    fb_product;
    logic signed [AUDIO_W-1:0]    fb_scaled;
    logic signed [AUDIO_W:0]      fb_sum;

    always_comb begin
        fb_product = $signed(delay_filt) * $signed({1'b0, feedback_val});
        fb_scaled  = AUDIO_W'((fb_product + (1 << 7)) >>> 8); // Q8 round
        fb_sum     = $signed({fb_scaled[AUDIO_W-1], fb_scaled}) +
                     $signed({audio_in_d1[AUDIO_W-1], audio_in_d1});
    end

    saturate #(.IN_W(AUDIO_W+1), .OUT_W(AUDIO_W)) u_fb_sat (
        .din  (fb_sum),
        .dout (fb_write_data)
    );

    // ================================================================
    // Output mix:  out = dry + wet
    //   dry  = audio_in          (always 100%)
    //   wet  = delay_filt * effect_lvl / 256
    // ================================================================
    logic signed [AUDIO_W-1:0] delay_filt_d1, audio_in_d2;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            delay_filt_d1 <= '0;
            audio_in_d2   <= '0;
        end else if (sample_en_d1) begin
            delay_filt_d1 <= delay_filt;
            audio_in_d2   <= audio_in_d1;
        end
    end

    logic signed [AUDIO_W+8:0]  wet_product;
    logic signed [AUDIO_W-1:0]  wet_scaled;
    logic signed [AUDIO_W:0]    mix_sum;
    logic signed [AUDIO_W-1:0]  mix_out;

    always_comb begin
        wet_product = $signed(delay_filt_d1) * $signed({1'b0, level_val});
        wet_scaled  = AUDIO_W'((wet_product + (1 << 7)) >>> 8); // Q8 round
        mix_sum     = $signed({audio_in_d2[AUDIO_W-1], audio_in_d2}) +
                      $signed({wet_scaled[AUDIO_W-1], wet_scaled});
    end

    saturate #(.IN_W(AUDIO_W+1), .OUT_W(AUDIO_W)) u_mix_sat (
        .din  (mix_sum),
        .dout (mix_out)
    );

    // ================================================================
    // Output register
    // ================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) audio_out <= '0;
        else if (sample_en_d2) audio_out <= mix_out;
    end

endmodule
