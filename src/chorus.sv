// ============================================================
// chorus.sv  -  BOSS CE-5 style stereo chorus
//
// Clocked on clk_audio (12.288 MHz).
// All widths are explicit to avoid XSim X-propagation issues.
//
// Key fixes vs original:
//  1. sine_table replaced with SINE_ROM localparam array
//     so values exist at elaboration time, eliminating
//     the initial-block race that caused X propagation.
//  2. All registers (wr_ptr, lfo_phase, lpf, audio_out)
//     now have matching `initial` blocks alongside their
//     reset logic, ensuring no X state at t=0 in simulation.
// ============================================================

module chorus #(
    parameter int SAMPLE_RATE = 48_000,
    parameter int DATA_WIDTH  = 24,
    parameter int DELAY_MAX   = 512   // must be power-of-2
)(
    input  logic                   clk,
    input  logic                   rst_n,
    input  logic                   sample_en,
    input  logic signed [DATA_WIDTH-1:0] audio_in,
    output logic signed [DATA_WIDTH-1:0] audio_out_l,
    output logic signed [DATA_WIDTH-1:0] audio_out_r,
    input  logic [7:0]             rate,
    input  logic [7:0]             depth,
    input  logic [7:0]             effect_lvl,
    input  logic [7:0]             e_q_hi,
    input  logic [7:0]             e_q_lo
);

    // --------------------------------------------------------
    // 1.  LFO  (sine via quarter-wave ROM, L/R 90° apart)
    // --------------------------------------------------------
    localparam int LFO_ACC_W = 32;
    localparam int LFO_OUT_W = 16;

    localparam [LFO_ACC_W-1:0] LFO_INC_MIN = 32'd8_948;
    localparam [LFO_ACC_W-1:0] LFO_INC_MAX = 32'd715_828;
    localparam [LFO_ACC_W-1:0] LFO_INC_RNG = LFO_INC_MAX - LFO_INC_MIN;

    logic [LFO_ACC_W-1:0] lfo_phase_l;
    logic [LFO_ACC_W-1:0] lfo_phase_r;
    logic [LFO_ACC_W-1:0] lfo_inc;

    // rate * LFO_INC_RNG fits in 8+32=40 bits unsigned
    logic [39:0] rate_product;
    always_comb begin
        rate_product = {32'd0, rate} * LFO_INC_RNG;
        lfo_inc      = LFO_INC_MIN + rate_product[39:8];
    end

    // --------------------------------------------------------
    // Quarter-wave sine ROM as localparam.
    // Values exist at elaboration time - no initial-block race.
    // Generated: round(sin(i*pi/512) * 32767) for i = 0..255
    // --------------------------------------------------------
    localparam int SINE_TBL_BITS = 8;
    localparam int SINE_TBL_SIZE = 256;

    localparam logic signed [15:0] SINE_ROM [0:255] = '{
        16'sh0000, 16'sh00c9, 16'sh0192, 16'sh025b,
        16'sh0324, 16'sh03ed, 16'sh04b6, 16'sh057f,
        16'sh0648, 16'sh0711, 16'sh07d9, 16'sh08a2,
        16'sh096a, 16'sh0a33, 16'sh0afb, 16'sh0bc4,
        16'sh0c8c, 16'sh0d54, 16'sh0e1c, 16'sh0ee3,
        16'sh0fab, 16'sh1072, 16'sh113a, 16'sh1201,
        16'sh12c8, 16'sh138f, 16'sh1455, 16'sh151c,
        16'sh15e2, 16'sh16a8, 16'sh176e, 16'sh1833,
        16'sh18f9, 16'sh19be, 16'sh1a82, 16'sh1b47,
        16'sh1c0b, 16'sh1ccf, 16'sh1d93, 16'sh1e57,
        16'sh1f1a, 16'sh1fdd, 16'sh209f, 16'sh2161,
        16'sh2223, 16'sh22e5, 16'sh23a6, 16'sh2467,
        16'sh2528, 16'sh25e8, 16'sh26a8, 16'sh2767,
        16'sh2826, 16'sh28e5, 16'sh29a3, 16'sh2a61,
        16'sh2b1f, 16'sh2bdc, 16'sh2c99, 16'sh2d55,
        16'sh2e11, 16'sh2ecc, 16'sh2f87, 16'sh3041,
        16'sh30fb, 16'sh31b5, 16'sh326e, 16'sh3326,
        16'sh33df, 16'sh3496, 16'sh354d, 16'sh3604,
        16'sh36ba, 16'sh376f, 16'sh3824, 16'sh38d9,
        16'sh398c, 16'sh3a40, 16'sh3af2, 16'sh3ba5,
        16'sh3c56, 16'sh3d07, 16'sh3db8, 16'sh3e68,
        16'sh3f17, 16'sh3fc5, 16'sh4073, 16'sh4121,
        16'sh41ce, 16'sh427a, 16'sh4325, 16'sh43d0,
        16'sh447a, 16'sh4524, 16'sh45cd, 16'sh4675,
        16'sh471c, 16'sh47c3, 16'sh4869, 16'sh490f,
        16'sh49b4, 16'sh4a58, 16'sh4afb, 16'sh4b9d,
        16'sh4c3f, 16'sh4ce0, 16'sh4d81, 16'sh4e20,
        16'sh4ebf, 16'sh4f5d, 16'sh4ffb, 16'sh5097,
        16'sh5133, 16'sh51ce, 16'sh5268, 16'sh5302,
        16'sh539b, 16'sh5432, 16'sh54c9, 16'sh5560,
        16'sh55f5, 16'sh568a, 16'sh571d, 16'sh57b0,
        16'sh5842, 16'sh58d3, 16'sh5964, 16'sh59f3,
        16'sh5a82, 16'sh5b0f, 16'sh5b9c, 16'sh5c28,
        16'sh5cb3, 16'sh5d3e, 16'sh5dc7, 16'sh5e4f,
        16'sh5ed7, 16'sh5f5d, 16'sh5fe3, 16'sh6068,
        16'sh60eb, 16'sh616e, 16'sh61f0, 16'sh6271,
        16'sh62f1, 16'sh6370, 16'sh63ee, 16'sh646c,
        16'sh64e8, 16'sh6563, 16'sh65dd, 16'sh6656,
        16'sh66cf, 16'sh6746, 16'sh67bc, 16'sh6832,
        16'sh68a6, 16'sh6919, 16'sh698b, 16'sh69fd,
        16'sh6a6d, 16'sh6adc, 16'sh6b4a, 16'sh6bb7,
        16'sh6c23, 16'sh6c8e, 16'sh6cf8, 16'sh6d61,
        16'sh6dc9, 16'sh6e30, 16'sh6e96, 16'sh6efb,
        16'sh6f5e, 16'sh6fc1, 16'sh7022, 16'sh7083,
        16'sh70e2, 16'sh7140, 16'sh719d, 16'sh71f9,
        16'sh7254, 16'sh72ae, 16'sh7307, 16'sh735e,
        16'sh73b5, 16'sh740a, 16'sh745f, 16'sh74b2,
        16'sh7504, 16'sh7555, 16'sh75a5, 16'sh75f3,
        16'sh7641, 16'sh768d, 16'sh76d8, 16'sh7722,
        16'sh776b, 16'sh77b3, 16'sh77fa, 16'sh783f,
        16'sh7884, 16'sh78c7, 16'sh7909, 16'sh794a,
        16'sh7989, 16'sh79c8, 16'sh7a05, 16'sh7a41,
        16'sh7a7c, 16'sh7ab6, 16'sh7aee, 16'sh7b26,
        16'sh7b5c, 16'sh7b91, 16'sh7bc5, 16'sh7bf8,
        16'sh7c29, 16'sh7c59, 16'sh7c88, 16'sh7cb6,
        16'sh7ce3, 16'sh7d0e, 16'sh7d39, 16'sh7d62,
        16'sh7d89, 16'sh7db0, 16'sh7dd5, 16'sh7dfa,
        16'sh7e1d, 16'sh7e3e, 16'sh7e5f, 16'sh7e7e,
        16'sh7e9c, 16'sh7eb9, 16'sh7ed5, 16'sh7eef,
        16'sh7f09, 16'sh7f21, 16'sh7f37, 16'sh7f4d,
        16'sh7f61, 16'sh7f74, 16'sh7f86, 16'sh7f97,
        16'sh7fa6, 16'sh7fb4, 16'sh7fc1, 16'sh7fcd,
        16'sh7fd8, 16'sh7fe1, 16'sh7fe9, 16'sh7ff0,
        16'sh7ff5, 16'sh7ff9, 16'sh7ffd, 16'sh7ffe
    };

    function automatic logic signed [LFO_OUT_W-1:0] lfo_sine(
        input logic [LFO_ACC_W-1:0] phase
    );
        logic [1:0]                   q;
        logic [SINE_TBL_BITS-1:0]     idx;
        logic signed [LFO_OUT_W-1:0]  s;
        q   = phase[31:30];
        idx = q[0] ? ~phase[29:22] : phase[29:22];
        s   = SINE_ROM[idx];
        return q[1] ? (-s) : s;
    endfunction

    // LFO phase registers - initial blocks match reset values
    // so there is no X at t=0 (before rst_n is ever asserted).
    initial begin
        lfo_phase_l = '0;
        lfo_phase_r = 32'h4000_0000;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            lfo_phase_l <= '0;
            lfo_phase_r <= 32'h4000_0000;
        end else if (sample_en) begin
            lfo_phase_l <= lfo_phase_l + lfo_inc;
            lfo_phase_r <= lfo_phase_r + lfo_inc;
        end
    end

    // --------------------------------------------------------
    // 2.  Delay line (circular buffer in distributed RAM)
    // --------------------------------------------------------
    localparam int ADDR_W     = $clog2(DELAY_MAX);   // 9
    localparam int CENTRE_TAP = 154;

    logic signed [DATA_WIDTH-1:0] delay_buf [0:DELAY_MAX-1];
    logic [ADDR_W-1:0]            wr_ptr;

    // Initialize delay buffer and write pointer at t=0
    initial begin
        for (int i = 0; i < DELAY_MAX; i++) delay_buf[i] = '0;
        wr_ptr = '0;
    end

    // Modulation depth
    logic [6:0] mod_amp;
    assign mod_amp = depth[7:1];  // 0..127

    // LFO output values (16-bit signed)
    logic signed [LFO_OUT_W-1:0] lfo_l_val, lfo_r_val;
    assign lfo_l_val = lfo_sine(lfo_phase_l);
    assign lfo_r_val = lfo_sine(lfo_phase_r);

    // Modulation offset: (lfo_val * mod_amp) >>> 15
    // 16s * 8u = 24s bits product; >>> 15 gives ~9s bits
    logic signed [23:0] mod_prod_l, mod_prod_r;
    logic signed [8:0]  mod_off_l,  mod_off_r;

    always_comb begin
        mod_prod_l = lfo_l_val * $signed({1'b0, mod_amp});
        mod_prod_r = lfo_r_val * $signed({1'b0, mod_amp});
        mod_off_l  = mod_prod_l[23:15];   // arithmetic shift right 15
        mod_off_r  = mod_prod_r[23:15];
    end

    // Read pointers (wrapping subtraction on ADDR_W bits)
    logic [ADDR_W-1:0] rd_ptr_l, rd_ptr_r;
    always_comb begin
        rd_ptr_l = wr_ptr - ADDR_W'(CENTRE_TAP) - ADDR_W'(mod_off_l);
        rd_ptr_r = wr_ptr - ADDR_W'(CENTRE_TAP) - ADDR_W'(mod_off_r);
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr <= '0;
        end else if (sample_en) begin
            delay_buf[wr_ptr] <= audio_in;
            wr_ptr            <= wr_ptr + 1'b1;
        end
    end

    // --------------------------------------------------------
    // 3.  1-pole IIR tone filter on wet path
    //     y[n] = y[n-1] + alpha * (x[n] - y[n-1])
    //     alpha = e_q_hi / 256
    // --------------------------------------------------------
    logic signed [DATA_WIDTH-1:0] wet_l_raw, wet_r_raw;
    assign wet_l_raw = delay_buf[rd_ptr_l];
    assign wet_r_raw = delay_buf[rd_ptr_r];

    logic signed [DATA_WIDTH-1:0] lpf_l;
    logic signed [DATA_WIDTH-1:0] lpf_r;

    // diff = x - y : DATA_WIDTH+1 bits to avoid overflow
    logic signed [DATA_WIDTH:0] diff_l, diff_r;
    // diff * alpha : (DATA_WIDTH+1) * 9 unsigned = DATA_WIDTH+10 bits
    logic signed [DATA_WIDTH+9:0] fprod_l, fprod_r;
    // After >> 8 and truncate to DATA_WIDTH
    logic signed [DATA_WIDTH-1:0] finc_l, finc_r;

    always_comb begin
        diff_l  = {wet_l_raw[DATA_WIDTH-1], wet_l_raw} - {lpf_l[DATA_WIDTH-1], lpf_l};
        diff_r  = {wet_r_raw[DATA_WIDTH-1], wet_r_raw} - {lpf_r[DATA_WIDTH-1], lpf_r};

        // Multiply: 25-bit signed * 9-bit signed = 34-bit signed
        fprod_l = diff_l * $signed({1'b0, e_q_hi});
        fprod_r = diff_r * $signed({1'b0, e_q_hi});

        // Arithmetic shift right 8, then take bottom DATA_WIDTH bits
        finc_l  = DATA_WIDTH'(fprod_l >>> 8);
        finc_r  = DATA_WIDTH'(fprod_r >>> 8);
    end

    initial begin
        lpf_l = '0;
        lpf_r = '0;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            lpf_l <= '0;
            lpf_r <= '0;
        end else if (sample_en) begin
            lpf_l <= lpf_l + finc_l;
            lpf_r <= lpf_r + finc_r;
        end
    end

    // --------------------------------------------------------
    // 4.  Wet/Dry mix
    //     out = dry * (256 - effect_lvl) + wet * effect_lvl
    //     then >> 8
    //
    //     gains are 9-bit unsigned (0..256), treated as signed for multiply
    //     24s * 10s = 34s product
    //     sum of two 34s = 35s, then >>> 8 and truncate to 24
    // --------------------------------------------------------
    logic signed [9:0]  dry_gain;   // 10-bit signed: holds 0..256
    logic signed [9:0]  wet_gain;   // 10-bit signed: holds 0..255

    logic signed [33:0] dry_prod_l, dry_prod_r;
    logic signed [33:0] wet_prod_l, wet_prod_r;
    logic signed [34:0] mix_sum_l,  mix_sum_r;
    logic signed [DATA_WIDTH-1:0] mix_l, mix_r;

    always_comb begin
        dry_gain = 10'(10'd256 - {2'd0, effect_lvl});
        wet_gain = {2'd0, effect_lvl};

        dry_prod_l = $signed(audio_in) * dry_gain;
        dry_prod_r = $signed(audio_in) * dry_gain;
        wet_prod_l = $signed(lpf_l)    * wet_gain;
        wet_prod_r = $signed(lpf_r)    * wet_gain;

        mix_sum_l = {dry_prod_l[33], dry_prod_l} + {wet_prod_l[33], wet_prod_l};
        mix_sum_r = {dry_prod_r[33], dry_prod_r} + {wet_prod_r[33], wet_prod_r};

        mix_l = DATA_WIDTH'(mix_sum_l >>> 8);
        mix_r = DATA_WIDTH'(mix_sum_r >>> 8);
    end

    initial begin
        audio_out_l = '0;
        audio_out_r = '0;
    end

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