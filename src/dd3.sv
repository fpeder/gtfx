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
    input  logic [31:0]      time_val,

    // Audio - clk domain
    // audio_out carries wet signal only; caller adds dry separately
    input  logic signed [WIDTH-1:0] audio_in,
    output logic signed [WIDTH-1:0] audio_out
);

    // =========================================================================
    // 0. SATURATION CONSTANTS
    // =========================================================================
    localparam signed [WIDTH-1:0]  SAT_MAX   = {1'b0, {(WIDTH-1){1'b1}}};  //  0x7FFFFF
    localparam signed [WIDTH-1:0]  SAT_MIN   = {1'b1, {(WIDTH-1){1'b0}}};  // -0x800000
    localparam signed [WIDTH+10:0] SAT_MAX_W = {{11{1'b0}}, SAT_MAX};
    localparam signed [WIDTH+10:0] SAT_MIN_W = {{11{1'b1}}, SAT_MIN};

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
    logic signed [9:0] fb_gain;
    logic signed [9:0] wet_level;

    always_comb begin
        fb_gain   = {2'b00, feedback_val};
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
    logic [15:0]       tone_sq;
    logic signed [9:0] tone_coeff;

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
    always_comb begin
        wr_ptr_next = (wr_ptr == ADDR_WIDTH'(RAM_DEPTH - 1)) ? '0 : wr_ptr + 1'b1;

        if (wr_ptr_next >= delay_samples)
            rd_ptr_next = wr_ptr_next - delay_samples;
        else
            rd_ptr_next = ADDR_WIDTH'(RAM_DEPTH) + wr_ptr_next - delay_samples;
    end

    // =========================================================================
    // 5. SIGNAL PATH (Combinatorial)
    // =========================================================================

    // 5A. LPF (1-pole IIR on BRAM output)
    logic signed [WIDTH-1:0]  lpf_state      = 0;
    logic signed [WIDTH-1:0]  lpf_state_next;
    logic signed [WIDTH+10:0] lpf_diff;
    logic signed [WIDTH+10:0] lpf_product;
    logic signed [WIDTH+10:0] lpf_increment;

    always_comb begin
        lpf_diff       = $signed(ram_read_data) - $signed(lpf_state);
        lpf_product    = lpf_diff * $signed(tone_coeff);
        lpf_increment  = lpf_product >>> 8;
        lpf_state_next = $signed(lpf_state) + lpf_increment;
    end

    // 5B. Feedback path - write (audio_in + filtered_delay * fb_gain) into RAM
    logic signed [WIDTH+10:0] fb_signal;
    logic signed [WIDTH+10:0] fb_sum;
    logic signed [WIDTH-1:0]  fb_saturated;

    always_comb begin
        fb_signal = ($signed(lpf_state) * fb_gain) >>> 8;
        fb_sum    = $signed(audio_in) + fb_signal;

        if      (fb_sum > SAT_MAX_W)  fb_saturated = SAT_MAX;
        else if (fb_sum < SAT_MIN_W)  fb_saturated = SAT_MIN;
        else                          fb_saturated = fb_sum[WIDTH-1:0];
    end

    // 5C. Wet output only - scaled filtered delay signal
    // Dry is NOT mixed here; top.sv adds dout_l at full unity gain.
    // This preserves the full dynamic range of the dry signal regardless
    // of level_val setting.
    logic signed [WIDTH+10:0] wet_signal;
    logic signed [WIDTH-1:0]  out_saturated;

    always_comb begin
        wet_signal = ($signed(lpf_state) * wet_level) >>> 8;

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