`timescale 1ns / 1ps

// Boss DD-3 Style Digital Delay
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
//   1. audio_in written into delay line with feedback mixed in
//   2. Delayed signal filtered by 1-pole IIR controlled by tone_val
//   3. audio_out = WET ONLY (filtered delayed signal scaled by level_val)
//      Dry passthrough and final mix are handled in top.sv so the dry
//      signal is never attenuated and the full 24-bit dynamic range is kept.
//
// FIX LOG:
//   - Compensated for 1-cycle BRAM read latency in pointer calculation so
//     rd_ptr leads by one extra sample; this ensures ram_read_data is
//     correct relative to the current wr_ptr. Eliminates pitch glitches
//     and the metallic intermodulation on repeats.
//   - LPF accumulator (lpf_state_next) is now saturated before registration
//     to prevent silent wraparound overflow, which caused click artifacts.
//   - Feedback now uses lpf_state_next (combinatorial, current-sample LPF
//     output) rather than the registered lpf_state (one sample stale).
//     This removes the extra-echo smear that sounded metallic at high fb.
//   - Widened intermediate products to WIDTH+18 to give adequate headroom
//     before truncation/saturation.
//
module dd3 #(
    parameter WIDTH     = 24,
    parameter RAM_DEPTH = 48000   // 1 second at 48kHz
)(
    input  logic             clk,       // clk_audio (12.288 MHz)
    input  logic             rst_n,
    input  logic             sample_en, // 1-cycle pulse at fs (48kHz), same domain

    // Controls - must be synchronised to clk before connecting
    input  logic [7:0]       tone_val,
    input  logic [7:0]       level_val,
    input  logic [7:0]       feedback_val,
    input  logic [15:0]      time_val,

    // Audio - clk domain
    // audio_out carries wet signal only; caller adds dry separately
    input  logic signed [WIDTH-1:0] audio_in,
    output logic signed [WIDTH-1:0] audio_out
);

    // =========================================================================
    // 0. SATURATION CONSTANTS
    // =========================================================================
    localparam signed [WIDTH-1:0]   SAT_MAX   = {1'b0, {(WIDTH-1){1'b1}}};  //  0x7FFFFF
    localparam signed [WIDTH-1:0]   SAT_MIN   = {1'b1, {(WIDTH-1){1'b0}}};  // -0x800000
    // Wide saturator for 34-bit intermediates (WIDTH + 10 product bits)
    localparam signed [WIDTH+9:0]   SAT_MAX_W = {{10{1'b0}}, SAT_MAX};
    localparam signed [WIDTH+9:0]   SAT_MIN_W = {{10{1'b1}}, SAT_MIN};

    // =========================================================================
    // 1. BLOCK RAM (Delay Line)
    // =========================================================================
    (* ram_style = "block" *) logic signed [WIDTH-1:0] ram [0:RAM_DEPTH-1];

    logic signed [WIDTH-1:0] ram_read_data = 0;

    localparam ADDR_WIDTH = $clog2(RAM_DEPTH);
    logic [ADDR_WIDTH-1:0] wr_ptr      = 0;
    logic [ADDR_WIDTH-1:0] rd_ptr      = 0;
    logic [ADDR_WIDTH-1:0] wr_ptr_next;
    logic [ADDR_WIDTH-1:0] rd_ptr_next;
    logic [ADDR_WIDTH-1:0] delay_samples;

    initial begin
        for (int i = 0; i < RAM_DEPTH; i++) ram[i] = 0;
    end

    // =========================================================================
    // 2. CONTROL SIGNALS (Combinatorial)
    // =========================================================================
    logic [9:0] fb_gain;
    logic [9:0] wet_level;

    always_comb begin
        fb_gain   = {2'b00, feedback_val};   // 0..255 → 0..255/256 gain
        wet_level = {2'b00, level_val};
    end

    always_comb begin
        if (time_val >= RAM_DEPTH)  delay_samples = ADDR_WIDTH'(RAM_DEPTH - 1);
        else if (time_val == 0)     delay_samples = ADDR_WIDTH'(1);
        else                        delay_samples = time_val[ADDR_WIDTH-1:0];
    end

    // =========================================================================
    // 3. TONE COEFFICIENT
    // =========================================================================
    // Quadratic mapping: coeff = (tone_val^2 >> 6) + 1
    //   tone_val=  0 -> coeff=  1 -> fc ~  30 Hz  (very dark)
    //   tone_val= 64 -> coeff= 65 -> fc ~ 2.2 kHz
    //   tone_val=102 -> coeff=164 -> fc ~ 7.0 kHz
    //   tone_val=128 -> coeff=256 -> bypass
    //   tone_val=255 -> bypass (clamped)
    //
    logic [15:0]  tone_sq;
    logic [9:0]   tone_coeff;          // 1..256

    always_comb begin
        tone_sq = {2'b00, tone_val} * {2'b00, tone_val};
        if (tone_sq[15:6] >= 10'd256)
            tone_coeff = 10'd256;
        else
            tone_coeff = {2'b00, tone_sq[13:6]} + 10'd1;
    end

    // =========================================================================
    // 4. POINTER CALCULATIONS
    // =========================================================================
    // BRAM has a 1-cycle registered read latency: ram_read_data on cycle N+1
    // reflects ram[rd_ptr] issued on cycle N.  To compensate, rd_ptr must
    // point one sample *ahead* of the nominal read position.  Equivalently,
    // we offset rd_ptr by +1 here so the data that emerges one cycle later
    // corresponds to the correct (wr_ptr - delay_samples) position.
    logic [ADDR_WIDTH-1:0] rd_lead;

    always_comb begin
        wr_ptr_next = (wr_ptr == ADDR_WIDTH'(RAM_DEPTH - 1)) ? '0 : wr_ptr + 1'b1;

        // rd points ONE ahead to compensate for registered BRAM latency
        rd_lead = (wr_ptr_next == ADDR_WIDTH'(RAM_DEPTH - 1)) ? '0 : wr_ptr_next + 1'b1;

        if (rd_lead >= delay_samples)
            rd_ptr_next = rd_lead - delay_samples;
        else
            rd_ptr_next = ADDR_WIDTH'(RAM_DEPTH) + rd_lead - delay_samples;
    end

    // =========================================================================
    // 5. SIGNAL PATH (Combinatorial)
    // =========================================================================

    // 5A. 1-pole IIR LPF on registered BRAM output
    //
    //   lpf_state_next = lpf_state + ((ram_read_data - lpf_state) * tone_coeff) >> 8
    //
    //   tone_coeff = 256  → alpha = 1.0 → transparent bypass
    //   tone_coeff = 1    → alpha ≈ 1/256 → very heavy LP
    //
    // Product is (WIDTH + 10) bits wide.  We truncate after >> 8, leaving
    // WIDTH+2 bits, then saturate back to WIDTH before registering.
    //
    logic signed [WIDTH-1:0]   lpf_state      = 0;
    logic signed [WIDTH-1:0]   lpf_state_next;
    logic signed [WIDTH+9:0]   lpf_diff;        // WIDTH+1 bits needed, padded to WIDTH+10
    logic signed [WIDTH+19:0]  lpf_product;     // (WIDTH+10) * 10-bit coeff
    logic signed [WIDTH+9:0]   lpf_increment;   // after >> 8
    logic signed [WIDTH+9:0]   lpf_sum_wide;

    always_comb begin
        lpf_diff      = $signed({{10{ram_read_data[WIDTH-1]}}, ram_read_data})
                      - $signed({{10{lpf_state[WIDTH-1]}},     lpf_state});
        lpf_product   = lpf_diff * $signed({1'b0, tone_coeff});  // tone_coeff always ≥ 0
        lpf_increment = lpf_product[WIDTH+17:8];                 // >> 8, keep WIDTH+10 bits
        lpf_sum_wide  = $signed({{10{lpf_state[WIDTH-1]}}, lpf_state}) + lpf_increment;

        // Saturate to WIDTH before registering
        if      (lpf_sum_wide > SAT_MAX_W)  lpf_state_next = SAT_MAX;
        else if (lpf_sum_wide < SAT_MIN_W)  lpf_state_next = SAT_MIN;
        else                                lpf_state_next = lpf_sum_wide[WIDTH-1:0];
    end

    // 5B. Feedback path
    //   Use lpf_state_next (current sample, combinatorial) so feedback is
    //   not an extra sample stale.  Stale feedback causes pre-echo smear
    //   that sounds metallic, especially at high feedback settings.
    logic signed [WIDTH+9:0]  fb_signal;
    logic signed [WIDTH+9:0]  fb_sum;
    logic signed [WIDTH-1:0]  fb_saturated;

    always_comb begin
        fb_signal = ($signed({{10{lpf_state_next[WIDTH-1]}}, lpf_state_next})
                     * $signed({2'b00, fb_gain})) >>> 8;
        fb_sum    = $signed({{10{audio_in[WIDTH-1]}}, audio_in}) + fb_signal;

        if      (fb_sum > SAT_MAX_W)  fb_saturated = SAT_MAX;
        else if (fb_sum < SAT_MIN_W)  fb_saturated = SAT_MIN;
        else                          fb_saturated = fb_sum[WIDTH-1:0];
    end

    // 5C. Wet output - scaled filtered delay signal (dry mixed in top.sv)
    logic signed [WIDTH+9:0]  wet_signal;
    logic signed [WIDTH-1:0]  out_saturated;

    always_comb begin
        wet_signal = ($signed({{10{lpf_state_next[WIDTH-1]}}, lpf_state_next})
                      * $signed({2'b00, wet_level})) >>> 8;

        if      (wet_signal > SAT_MAX_W)  out_saturated = SAT_MAX;
        else if (wet_signal < SAT_MIN_W)  out_saturated = SAT_MIN;
        else                              out_saturated = wet_signal[WIDTH-1:0];
    end

    // =========================================================================
    // 6. OUTPUT
    // =========================================================================
    logic signed [WIDTH-1:0] audio_out_reg = 0;
    assign audio_out = audio_out_reg;

    // =========================================================================
    // 7. SEQUENTIAL LOGIC
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr        <= '0;
            rd_ptr        <= '0;
            lpf_state     <= '0;
            audio_out_reg <= '0;
            ram_read_data <= '0;
        end else if (sample_en) begin
            ram_read_data <= ram[rd_ptr];
            ram[wr_ptr]   <= fb_saturated;
            wr_ptr        <= wr_ptr_next;
            rd_ptr        <= rd_ptr_next;
            lpf_state     <= lpf_state_next;
            audio_out_reg <= out_saturated;
        end
    end

endmodule