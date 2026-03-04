`timescale 1ns / 1ps

// ============================================================================
// delay.sv - Strymon Timeline Multi-Mode Delay
//
// Three selectable delay characters (mode = grit_val[1:0]):
//   0 = Digital  — clean repeats, full-bandwidth filter
//   1 = Tape     — soft-clip saturation + LFO wow/flutter
//   2 = Analog   — narrow LP rolloff, subtle modulation (BBD style)
//
// Interface:
//   repeats_val [7:0]   feedback gain (0 = single echo, 255 ≈ infinite)
//   mix_val     [7:0]   dry/wet crossfade (0 = dry, 0x80 = 50/50, FF = wet)
//   filter_val  [7:0]   feedback LP cutoff (0 = dark, FF = bright)
//   time_val    [15:0]  delay in samples (600..38400 @ 48 kHz)
//   mod_val     [7:0]   LFO modulation depth (0 = off, FF = max flutter)
//   grit_val    [7:0]   [7:2] = saturation amount, [1:0] = mode
//
// Architecture:
//   2-phase BRAM pipeline via delay_line (REG_RD=1)
//   Latency: 3 clk cycles from sample_en to valid audio_out
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
    input  logic [7:0]                    repeats_val,
    input  logic [7:0]                    mix_val,
    input  logic [7:0]                    filter_val,
    input  logic [7:0]                    mod_val,
    input  logic [7:0]                    grit_val
);

    localparam int ADDR_W = $clog2(RAM_DEPTH);

    // Mode decode
    logic [1:0] mode;
    logic [5:0] grit_level;
    assign mode       = grit_val[1:0];
    assign grit_level = grit_val[7:2];

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
    // LFO for modulation (wow/flutter)
    //
    // Slow sine LFO (~1 Hz at rate=0x40).  Output scaled by mod_val
    // to produce ±32 sample offset on the read pointer.
    // Tape mode: full depth.  Analog mode: half depth.
    // Digital mode: no modulation.
    // ================================================================
    logic signed [15:0] lfo_wave;  // Q1.15 signed

    lfo_core #(
        .PHASE_W   (32),
        .CTRL_W    (8),
        .LFO_W     (16),
        .INC_BASE  (22370),       // ~0.5 Hz base at 48 kHz
        .INC_SCALE (2018),        // ~0.5–2.5 Hz range
        .INC_SHIFT (0),
        .TABLE_BITS(8),
        .WAVE_TYPE ("SINE")
    ) u_lfo (
        .clk          (clk),
        .rst_n        (rst_n),
        .sample_en    (sample_en),
        .rate_val     (8'h40),    // fixed moderate rate
        .phase_out    (),
        .wave_signed  (lfo_wave),
        .wave_unsigned()
    );

    // Scale LFO by mod_val → ±32 samples max (Q1.15 * 8 >> 15 = ±max_offset)
    // mod_scale: tape = mod_val, analog = mod_val/2, digital = 0
    logic [7:0]  mod_scale;
    logic signed [23:0] lfo_product;   // Q1.15 * Q0.8 = Q1.23
    logic signed [ADDR_W-1:0] lfo_offset;

    always_comb begin
        case (mode)
            2'd1:    mod_scale = mod_val;                    // tape: full
            2'd2:    mod_scale = {1'b0, mod_val[7:1]};      // analog: half
            default: mod_scale = 8'd0;                       // digital: none
        endcase
        // lfo_wave (Q1.15) * mod_scale (Q0.8) = Q1.23
        // Shift right by 18 → ±32 range (max mod_val=255, wave=32767: 255*32767>>18 ≈ 31)
        lfo_product = lfo_wave * $signed({1'b0, mod_scale});
        lfo_offset  = ADDR_W'(lfo_product >>> 18);
    end

    // ================================================================
    // Read pointer (with modulation offset)
    // ================================================================
    logic [ADDR_W-1:0] wr_ptr;
    logic [ADDR_W-1:0] rd_ptr;

    assign rd_ptr = wr_ptr - ADDR_W'(time_val) + $unsigned(lfo_offset);

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
    // Tone control — mode-dependent single-pole LPF on feedback
    //
    //   y += alpha * (x - y)
    //
    //   Digital: alpha = (filter_val + 48) / 512
    //   Tape:    alpha = (filter_val + 48) / 512   (same range)
    //   Analog:  alpha = (filter_val/2 + 24) / 512 (darker, BBD rolloff)
    // ================================================================
    logic signed [AUDIO_W-1:0]       lpf_state;
    logic signed [AUDIO_W:0]         lpf_diff;
    logic signed [AUDIO_W+9:0]       lpf_prod;
    logic signed [AUDIO_W-1:0]       delay_filt;
    logic [8:0]                      alpha_val;

    always_comb begin
        // Mode-dependent alpha
        if (mode == 2'd2)
            alpha_val = {1'b0, filter_val[7:1]} + 9'd24;   // analog: narrower
        else
            alpha_val = {1'b0, filter_val} + 9'd48;        // digital/tape

        lpf_diff = $signed({delay_raw[AUDIO_W-1], delay_raw}) -
                   $signed({lpf_state[AUDIO_W-1], lpf_state});
        lpf_prod = lpf_diff * $signed({2'b0, alpha_val});   // Q9 multiply
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
    // Tape saturation — soft_clip on feedback path (mode 1 only)
    //
    // Apply grit: scale filtered signal up by grit_level, soft clip,
    // then use as feedback source.  Other modes pass delay_filt through.
    // ================================================================
    logic signed [AUDIO_W-1:0] fb_source;

    // Gain stage: boost by grit_level for saturation headroom
    // grit_level [5:0] → gain = 1 + grit_level/16 (range 1.0..4.9)
    logic signed [AUDIO_W+5:0]  grit_product;
    logic signed [AUDIO_W+1:0]  grit_boosted;
    logic signed [AUDIO_W-1:0]  clip_out;

    always_comb begin
        grit_product = $signed(delay_filt) * $signed({1'b0, grit_level + 6'd16});
        grit_boosted = (AUDIO_W+2)'((grit_product + (1 << 3)) >>> 4); // /16, round
    end

    soft_clip #(
        .IN_W       (AUDIO_W + 2),
        .OUT_W      (AUDIO_W),
        .COMP_SHIFT (1),
        .KNEE       ((1 << (AUDIO_W - 2))),
        .PEAK       ({1'b0, {(AUDIO_W-1){1'b1}}})
    ) u_tape_clip (
        .din  (grit_boosted),
        .dout (clip_out)
    );

    // Select feedback source based on mode
    always_comb begin
        if (mode == 2'd1 && grit_level != 6'd0)
            fb_source = clip_out;       // tape with grit
        else
            fb_source = delay_filt;     // digital/analog: clean
    end

    // ================================================================
    // Feedback path  (8-bit gain, saturated write-back)
    //   fb_scaled = fb_source * repeats / 256
    //   write     = sat(fb_scaled + audio_in)
    // ================================================================
    logic signed [AUDIO_W+8:0]    fb_product;
    logic signed [AUDIO_W-1:0]    fb_scaled;
    logic signed [AUDIO_W:0]      fb_sum;

    always_comb begin
        fb_product = $signed(fb_source) * $signed({1'b0, repeats_val});
        fb_scaled  = AUDIO_W'((fb_product + (1 << 7)) >>> 8); // Q8 round
        fb_sum     = $signed({fb_scaled[AUDIO_W-1], fb_scaled}) +
                     $signed({audio_in_d1[AUDIO_W-1], audio_in_d1});
    end

    saturate #(.IN_W(AUDIO_W+1), .OUT_W(AUDIO_W)) u_fb_sat (
        .din  (fb_sum),
        .dout (fb_write_data)
    );

    // ================================================================
    // Output mix:  true dry/wet crossfade
    //   out = dry * (255 - mix) / 256 + wet * mix / 256
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

    logic signed [AUDIO_W+8:0]  dry_product, wet_product;
    logic signed [AUDIO_W-1:0]  dry_scaled, wet_scaled;
    logic signed [AUDIO_W:0]    mix_sum;
    logic signed [AUDIO_W-1:0]  mix_out;

    always_comb begin
        dry_product = $signed(audio_in_d2)   * $signed({1'b0, 8'hFF - mix_val});
        wet_product = $signed(delay_filt_d1) * $signed({1'b0, mix_val});
        dry_scaled  = AUDIO_W'((dry_product + (1 << 7)) >>> 8);
        wet_scaled  = AUDIO_W'((wet_product + (1 << 7)) >>> 8);
        mix_sum     = $signed({dry_scaled[AUDIO_W-1], dry_scaled}) +
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
