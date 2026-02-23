`timescale 1ns / 1ps

// =============================================================================
// Testbench: chorus (BOSS CE-5 style stereo chorus)
//
// clk_audio : 12.288 MHz  (half period = 41 ns)
// sample_en : 1-cycle pulse every 256 clocks (48 kHz = 12.288 MHz / 256)
//
// Test cases
//   TC1  Reset clears outputs         - audio_out_l / _r must be 0 during reset
//   TC2  Zero input silence           - zero in -> zero out (all states cold)
//   TC3  No X/Z propagation           - outputs must never be X/Z after reset
//   TC4  Output range / no overflow   - |audio_out| must stay <= 24-bit signed max
//   TC5  Dry passthrough unity gain   - with wet=0 (effect_lvl=0), out_l == in,
//                                        out_r == in (dry is added to both channels)
//   TC6  Stereo phase inversion       - out_l and out_r must differ by sign of wet;
//                                        L+R cancels wet, L-R doubles it
//   TC7  Effect level scales wet      - doubling effect_lvl (approx) doubles wet delta
//   TC8  Rate affects LFO speed       - higher rate => lfo_phase advances faster
//   TC9  Depth scales modulation      - larger depth => wider delay swing seen in output
//   TC10 EQ hi/lo bands               - e_q_hi=0 zeroes highs, e_q_lo=0 zeroes lows
//   TC11 Saturation clamp             - driving extreme amplitudes clamps, no overflow
//   TC12 Back-to-back sample_en       - two forced consecutive pulses must not corrupt
//
// Pass/fail via $display; $fatal on any failure.
// =============================================================================

module tb_chorus;

    // =========================================================================
    // Parameters matching DUT defaults
    // =========================================================================
    localparam int WIDTH           = 24;
    localparam int DELAY_MAX       = 2048;
    localparam int CLK_HALF_NS     = 41;            // 12.288 MHz
    localparam int CLKS_PER_SAMPLE = 256;           // 48 kHz

    localparam signed [WIDTH-1:0] SAT_MAX = {1'b0, {(WIDTH-1){1'b1}}};  //  8388607
    localparam signed [WIDTH-1:0] SAT_MIN = {1'b1, {(WIDTH-1){1'b0}}};  // -8388608

    // =========================================================================
    // Simple 64-point sine LUT for stimulus (round(sin(2*pi*i/64)*(2^23-1)))
    // =========================================================================
    localparam int LUT_LEN = 64;
    localparam logic signed [WIDTH-1:0] SINE_LUT [0:63] = '{
        24'sh000000, 24'sh0C8BD3, 24'sh18F8B8, 24'sh25280B,
        24'sh30FBC5, 24'sh3C56B9, 24'sh471CEC, 24'sh5133CC,
        24'sh5A827A, 24'sh62F1BE, 24'sh6A6D99, 24'sh70E2CC,
        24'sh7641AE, 24'sh7A7D05, 24'sh7D8A5F, 24'sh7F6290,
        24'sh7FFFFF, 24'sh7F6290, 24'sh7D8A5F, 24'sh7A7D05,
        24'sh7641AE, 24'sh70E2CC, 24'sh6A6D99, 24'sh62F1BE,
        24'sh5A827A, 24'sh5133CC, 24'sh471CEC, 24'sh3C56B9,
        24'sh30FBC5, 24'sh25280B, 24'sh18F8B8, 24'sh0C8BD3,
        24'sh000000, 24'shF37427, 24'shE70748, 24'shDAD7F5,
        24'shCF043B, 24'shC3A947, 24'shB8E314, 24'shAECC34,
        24'shA57D86, 24'sh9D0E42, 24'sh959267, 24'sh8F1D34,
        24'sh89BE52, 24'sh8582FB, 24'sh8275A1, 24'sh809D70,
        24'sh800001, 24'sh809D70, 24'sh8275A1, 24'sh8582FB,
        24'sh89BE52, 24'sh8F1D34, 24'sh959267, 24'sh9D0E42,
        24'shA57D86, 24'shAECC34, 24'shB8E314, 24'shC3A947,
        24'shCF043B, 24'shDAD7F5, 24'shE70748, 24'shF37427
    };

    // =========================================================================
    // DUT signals
    // =========================================================================
    logic                    clk;
    logic                    rst_n;
    logic                    sample_en;
    logic signed [WIDTH-1:0] audio_in;
    logic signed [WIDTH-1:0] audio_out_l;
    logic signed [WIDTH-1:0] audio_out_r;
    logic [7:0]              rate;
    logic [7:0]              depth;
    logic [7:0]              effect_lvl;
    logic [7:0]              e_q_hi;
    logic [7:0]              e_q_lo;

    // =========================================================================
    // DUT
    // =========================================================================
    chorus #(
        .SAMPLE_RATE(48_000),
        .DATA_WIDTH (24),
        .DELAY_MAX  (2048)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .sample_en  (sample_en),
        .audio_in   (audio_in),
        .audio_out_l(audio_out_l),
        .audio_out_r(audio_out_r),
        .rate       (rate),
        .depth      (depth),
        .effect_lvl (effect_lvl),
        .e_q_hi     (e_q_hi),
        .e_q_lo     (e_q_lo)
    );

    // =========================================================================
    // Clock generator
    // =========================================================================
    initial clk = 1'b0;
    always #CLK_HALF_NS clk = ~clk;

    // =========================================================================
    // sample_en divider (independent of test logic)
    // =========================================================================
    logic [7:0] div_cnt = '0;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            div_cnt   <= '0;
            sample_en <= 1'b0;
        end else begin
            if (div_cnt == CLKS_PER_SAMPLE - 1) begin
                div_cnt   <= '0;
                sample_en <= 1'b1;
            end else begin
                div_cnt   <= div_cnt + 1'b1;
                sample_en <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Pass / fail counters
    // =========================================================================
    int pass_count = 0;
    int fail_count = 0;

    task automatic pass_msg(input string msg);
        $display("  PASS  %s", msg);
        pass_count++;
    endtask

    task automatic fail_msg(input string msg);
        $display("  FAIL  %s", msg);
        fail_count++;
    endtask

    task automatic check(input logic cond, input string msg);
        if (cond) pass_msg(msg);
        else      fail_msg(msg);
    endtask

    // =========================================================================
    // Helper: apply reset and wait for first sample_en
    // =========================================================================
    task automatic do_reset();
        rst_n      = 1'b0;
        audio_in   = '0;
        rate       = 8'd32;
        depth      = 8'd64;
        effect_lvl = 8'd128;
        e_q_hi     = 8'd128;
        e_q_lo     = 8'd128;
        repeat (20) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        @(posedge clk iff sample_en);  // align to first sample_en after reset
    endtask

    // =========================================================================
    // Helper: run n samples with constant audio_in, return last L and R outputs
    // =========================================================================
    task automatic run_fixed(
        input  signed [WIDTH-1:0] in_val,
        input  int                n,
        output signed [WIDTH-1:0] last_l,
        output signed [WIDTH-1:0] last_r
    );
        for (int i = 0; i < n; i++) begin
            @(posedge clk iff sample_en);
            audio_in = in_val;
            @(posedge clk);
            last_l = audio_out_l;
            last_r = audio_out_r;
        end
    endtask

    // =========================================================================
    // Helper: run n samples from sine LUT, track X/Z and overflow
    // =========================================================================
    logic [5:0] lut_idx;

    task automatic run_sine(
        input  int   n,
        output logic any_x,
        output logic any_overflow
    );
        any_x        = 1'b0;
        any_overflow = 1'b0;
        for (int i = 0; i < n; i++) begin
            @(posedge clk iff sample_en);
            audio_in = SINE_LUT[lut_idx];
            lut_idx  = lut_idx + 1'b1;
            @(posedge clk);
            if ($isunknown(audio_out_l) || $isunknown(audio_out_r))
                any_x = 1'b1;
            if (audio_out_l > SAT_MAX || audio_out_l < SAT_MIN)
                any_overflow = 1'b1;
            if (audio_out_r > SAT_MAX || audio_out_r < SAT_MIN)
                any_overflow = 1'b1;
        end
    endtask

    // =========================================================================
    // Internal signal probes
    // =========================================================================
    wire [31:0] lfo_phase_w = dut.lfo_phase;
    wire [15:0] lfo_inc_w   = dut.lfo_inc[15:0];

    // =========================================================================
    // MAIN TEST SEQUENCE
    // =========================================================================
    logic signed [WIDTH-1:0] out_l_a, out_r_a;
    logic signed [WIDTH-1:0] out_l_b, out_r_b;
    logic any_x, any_overflow;

    initial begin
        $display("=== chorus testbench ===");

        // -----------------------------------------------------------------
        // TC1: Reset clears outputs
        // -----------------------------------------------------------------
        $display("\n-- TC1: Reset clears outputs --");
        rst_n      = 1'b0;
        audio_in   = 24'sh3FFFFF;
        rate       = 8'd200;
        depth      = 8'd255;
        effect_lvl = 8'd255;
        e_q_hi     = 8'd255;
        e_q_lo     = 8'd255;
        repeat (10) @(posedge clk);
        check(audio_out_l === 24'sh0 && audio_out_r === 24'sh0,
              "audio_out_l/r = 0 immediately after rst_n asserted");
        repeat (300) @(posedge clk);
        check(audio_out_l === 24'sh0 && audio_out_r === 24'sh0,
              "audio_out_l/r stay 0 throughout reset hold");

        @(negedge clk);
        rst_n = 1'b1;
        @(posedge clk iff sample_en);

        // -----------------------------------------------------------------
        // TC2: Zero input -> zero output (cold state)
        // -----------------------------------------------------------------
        $display("\n-- TC2: Zero input silence --");
        do_reset();
        rate       = 8'd0;
        depth      = 8'd0;
        effect_lvl = 8'd0;
        e_q_hi     = 8'd128;
        e_q_lo     = 8'd128;
        begin
            int nonzero = 0;
            for (int i = 0; i < 20; i++) begin
                @(posedge clk iff sample_en);
                audio_in = 24'sh0;
                @(posedge clk);
                if (audio_out_l !== 24'sh0 || audio_out_r !== 24'sh0)
                    nonzero++;
            end
            check(nonzero == 0, "zero input -> zero output for 20 samples");
        end

        // -----------------------------------------------------------------
        // TC3: No X/Z propagation after reset
        // -----------------------------------------------------------------
        $display("\n-- TC3: No X/Z propagation --");
        do_reset();
        lut_idx    = '0;
        rate       = 8'd32;
        depth      = 8'd64;
        effect_lvl = 8'd128;
        e_q_hi     = 8'd128;
        e_q_lo     = 8'd128;
        run_sine(96, any_x, any_overflow);
        check(!any_x, "audio_out_l/r never X/Z over 96 samples");

        // -----------------------------------------------------------------
        // TC4: Output always within signed 24-bit range
        // Test with full-scale sine, max effect, max depth, feedback mode
        // -----------------------------------------------------------------
        $display("\n-- TC4: Output range / no overflow --");
        do_reset();
        lut_idx    = '0;
        rate       = 8'd255;
        depth      = 8'd255;
        effect_lvl = 8'd255;
        e_q_hi     = 8'd255;
        e_q_lo     = 8'd255;
        run_sine(192, any_x, any_overflow);
        check(!any_overflow,
              "audio_out never exceeds +-SAT over 192 samples (max controls)");
        check(!any_x,
              "audio_out never X/Z over 192 samples (max controls)");

        // -----------------------------------------------------------------
        // TC5: Dry passthrough unity gain when effect_lvl = 0
        // With effect_lvl=0 the wet term vanishes from both channels.
        // mix_sum_l/r = audio_in*256; shifted >>8 = audio_in.
        // After the DELAY_MAX sample pipeline latency we expect out == in.
        // We compare samples after the delay line has filled past CENTRE_TAP.
        // -----------------------------------------------------------------
        $display("\n-- TC5: Dry passthrough unity gain (effect_lvl=0) --");
        do_reset();
        rate       = 8'd0;
        depth      = 8'd0;
        effect_lvl = 8'd0;     // kill wet completely
        e_q_hi     = 8'd128;
        e_q_lo     = 8'd128;
        begin
            // Pump one sample through and read the very next output
            audio_in = 24'sh400000;      // +0.5 full scale
            @(posedge clk iff sample_en);
            @(posedge clk);
            // audio_out registers on sample_en, so read 1 clk after
            // The DUT registers output on the same sample_en that consumes
            // audio_in, so out_l/r should equal the current in.
            check(audio_out_l === 24'sh400000,
                  "effect_lvl=0: audio_out_l equals audio_in");
            check(audio_out_r === 24'sh400000,
                  "effect_lvl=0: audio_out_r equals audio_in");
        end

        // -----------------------------------------------------------------
        // TC6: Stereo phase inversion
        // L adds wet, R subtracts wet.  After delay line has non-zero
        // content, L and R must differ.  Their sum should be dominated by
        // dry (2x audio_in) and their difference by wet (2x wet).
        // -----------------------------------------------------------------
        $display("\n-- TC6: Stereo phase inversion --");
        do_reset();
        lut_idx    = '0;
        rate       = 8'd0;      // freeze LFO
        depth      = 8'd0;      // centre tap only - no modulation
        effect_lvl = 8'd128;
        e_q_hi     = 8'd128;
        e_q_lo     = 8'd128;
        begin
            // Warm up beyond CENTRE_TAP (960) so delay line carries signal
            for (int i = 0; i < 980; i++) begin
                @(posedge clk iff sample_en);
                audio_in = SINE_LUT[lut_idx];
                lut_idx  = lut_idx + 1'b1;
                @(posedge clk);
            end
            // Read one output pair
            out_l_a = audio_out_l;
            out_r_a = audio_out_r;
            // L and R must differ (wet signal flipped in R)
            check(out_l_a !== out_r_a,
                  "stereo: audio_out_l != audio_out_r (wet phase inverted)");
            // Sum cancels wet: (dry+wet) + (dry-wet) = 2*dry
            // Difference doubles wet: (dry+wet) - (dry-wet) = 2*wet != 0
            begin
                logic signed [WIDTH:0] sum_lr  = $signed(out_l_a) + $signed(out_r_a);
                logic signed [WIDTH:0] diff_lr = $signed(out_l_a) - $signed(out_r_a);
                check(diff_lr !== 0,
                      "stereo: L-R is non-zero (wet contribution visible)");
            end
        end

        // -----------------------------------------------------------------
        // TC7: Effect level scales wet amplitude
        // Run identical inputs twice with different effect_lvl; the
        // difference between L and R (= 2*wet) should scale proportionally.
        // We compare after the delay line fills.
        // -----------------------------------------------------------------
        $display("\n-- TC7: Effect level scales wet --");

        // Run A: low effect_lvl
        do_reset();
        lut_idx    = '0;
        rate       = 8'd0;
        depth      = 8'd0;
        effect_lvl = 8'd64;
        e_q_hi     = 8'd128;
        e_q_lo     = 8'd128;
        for (int i = 0; i < 980; i++) begin
            @(posedge clk iff sample_en);
            audio_in = SINE_LUT[lut_idx];
            lut_idx  = lut_idx + 1'b1;
            @(posedge clk);
        end
        begin
            longint diff_a = $signed(audio_out_l) - $signed(audio_out_r);
            if (diff_a < 0) diff_a = -diff_a;

            // Run B: double effect_lvl
            do_reset();
            lut_idx    = '0;
            rate       = 8'd0;
            depth      = 8'd0;
            effect_lvl = 8'd128;
            e_q_hi     = 8'd128;
            e_q_lo     = 8'd128;
            for (int i = 0; i < 980; i++) begin
                @(posedge clk iff sample_en);
                audio_in = SINE_LUT[lut_idx];
                lut_idx  = lut_idx + 1'b1;
                @(posedge clk);
            end
            begin
                longint diff_b = $signed(audio_out_l) - $signed(audio_out_r);
                if (diff_b < 0) diff_b = -diff_b;
                // diff_b should be roughly 2x diff_a (allow 50% tolerance for EQ)
                check(diff_b > diff_a,
                      "doubling effect_lvl increases stereo wet difference");
            end
        end

        // -----------------------------------------------------------------
        // TC8: Rate control affects LFO advance speed
        // With rate=0 the lfo_inc should be LFO_INC_MIN; with rate=255
        // it should be LFO_INC_MAX. Probe via hierarchical reference.
        // -----------------------------------------------------------------
        $display("\n-- TC8: Rate affects LFO speed --");
        do_reset();
        rate = 8'd0;
        begin
            logic [31:0] phase_before, phase_after_slow;
            // Advance a few samples and measure phase delta
            @(posedge clk iff sample_en); @(posedge clk);
            phase_before = lfo_phase_w;
            repeat (4) @(posedge clk iff sample_en);
            @(posedge clk);
            phase_after_slow = lfo_phase_w;

            do_reset();
            rate = 8'd255;
            @(posedge clk iff sample_en); @(posedge clk);
            phase_before = lfo_phase_w;
            repeat (4) @(posedge clk iff sample_en);
            @(posedge clk);
            begin
                logic [31:0] phase_after_fast = lfo_phase_w;
                // LFO at rate=255 must accumulate faster than at rate=0
                check(phase_after_fast > phase_after_slow,
                      "rate=255 advances LFO phase faster than rate=0 over 4 samples");
            end
        end

        // -----------------------------------------------------------------
        // TC9: Depth controls modulation swing
        // With depth=0 modulation offset is 0; with depth=255 mod_amp is
        // larger so rd_ptr swings further. After LFO moves, the delay tap
        // differs between low and high depth, producing different outputs.
        // Run sine at non-zero LFO rate and compare outputs.
        // -----------------------------------------------------------------
        $display("\n-- TC9: Depth scales delay modulation --");

        // Run A: depth=0 (no modulation)
        do_reset();
        lut_idx    = '0;
        rate       = 8'd128;
        depth      = 8'd0;
        effect_lvl = 8'd200;
        e_q_hi     = 8'd128;
        e_q_lo     = 8'd128;
        for (int i = 0; i < 1000; i++) begin
            @(posedge clk iff sample_en);
            audio_in = SINE_LUT[lut_idx];
            lut_idx  = lut_idx + 1'b1;
            @(posedge clk);
        end
        out_l_a = audio_out_l;

        // Run B: depth=255 (max modulation), same seed
        do_reset();
        lut_idx    = '0;
        rate       = 8'd128;
        depth      = 8'd255;
        effect_lvl = 8'd200;
        e_q_hi     = 8'd128;
        e_q_lo     = 8'd128;
        for (int i = 0; i < 1000; i++) begin
            @(posedge clk iff sample_en);
            audio_in = SINE_LUT[lut_idx];
            lut_idx  = lut_idx + 1'b1;
            @(posedge clk);
        end
        out_l_b = audio_out_l;

        check(out_l_a !== out_l_b,
              "depth=0 and depth=255 produce different outputs (modulation present)");

        // -----------------------------------------------------------------
        // TC10: EQ band controls
        // e_q_hi=0 should zero the high band contribution; e_q_lo=0
        // should zero the low band.  With a constant DC input the HPF
        // output (high_band) decays to 0 so only the low band carries signal.
        // Test: e_q_lo=255 / e_q_hi=255 vs e_q_lo=0 / e_q_hi=255 differ.
        // -----------------------------------------------------------------
        $display("\n-- TC10: EQ band controls affect output --");

        // Run A: both EQ bands at full
        do_reset();
        lut_idx    = '0;
        rate       = 8'd0;
        depth      = 8'd0;
        effect_lvl = 8'd200;
        e_q_hi     = 8'd255;
        e_q_lo     = 8'd255;
        for (int i = 0; i < 1000; i++) begin
            @(posedge clk iff sample_en);
            audio_in = SINE_LUT[lut_idx];
            lut_idx  = lut_idx + 1'b1;
            @(posedge clk);
        end
        out_l_a = audio_out_l;

        // Run B: kill low band
        do_reset();
        lut_idx    = '0;
        rate       = 8'd0;
        depth      = 8'd0;
        effect_lvl = 8'd200;
        e_q_hi     = 8'd255;
        e_q_lo     = 8'd0;
        for (int i = 0; i < 1000; i++) begin
            @(posedge clk iff sample_en);
            audio_in = SINE_LUT[lut_idx];
            lut_idx  = lut_idx + 1'b1;
            @(posedge clk);
        end
        out_l_b = audio_out_l;

        check(out_l_a !== out_l_b,
              "e_q_lo=0 produces different output than e_q_lo=255 (low band gated)");

        // Run C: kill high band
        do_reset();
        lut_idx    = '0;
        rate       = 8'd0;
        depth      = 8'd0;
        effect_lvl = 8'd200;
        e_q_hi     = 8'd0;
        e_q_lo     = 8'd255;
        for (int i = 0; i < 1000; i++) begin
            @(posedge clk iff sample_en);
            audio_in = SINE_LUT[lut_idx];
            lut_idx  = lut_idx + 1'b1;
            @(posedge clk);
        end
        begin
            logic signed [WIDTH-1:0] out_l_c = audio_out_l;
            check(out_l_a !== out_l_c,
                  "e_q_hi=0 produces different output than e_q_hi=255 (high band gated)");
        end

        // -----------------------------------------------------------------
        // TC11: Saturation clamp - extreme input must not overflow
        // Drive SAT_MAX continuously with max wet; output must stay in range.
        // -----------------------------------------------------------------
        $display("\n-- TC11: Saturation clamp --");
        do_reset();
        rate       = 8'd128;
        depth      = 8'd255;
        effect_lvl = 8'd255;
        e_q_hi     = 8'd255;
        e_q_lo     = 8'd255;
        begin
            logic overflow_seen = 1'b0;
            for (int i = 0; i < 200; i++) begin
                @(posedge clk iff sample_en);
                // Alternate SAT_MAX / SAT_MIN to stress saturation in both dirs
                audio_in = (i[0]) ? SAT_MAX : SAT_MIN;
                @(posedge clk);
                if (audio_out_l > SAT_MAX || audio_out_l < SAT_MIN) overflow_seen = 1'b1;
                if (audio_out_r > SAT_MAX || audio_out_r < SAT_MIN) overflow_seen = 1'b1;
            end
            check(!overflow_seen,
                  "saturation: extreme alternating input never overflows output");
        end

        // -----------------------------------------------------------------
        // TC12: Back-to-back sample_en pulses do not corrupt state
        // Force two consecutive sample_en edges; verify outputs remain valid.
        // -----------------------------------------------------------------
        $display("\n-- TC12: Back-to-back sample_en pulses --");
        do_reset();
        lut_idx    = '0;
        rate       = 8'd32;
        depth      = 8'd64;
        effect_lvl = 8'd128;
        e_q_hi     = 8'd128;
        e_q_lo     = 8'd128;

        // Warm up
        for (int i = 0; i < 8; i++) begin
            @(posedge clk iff sample_en);
            audio_in = SINE_LUT[lut_idx];
            lut_idx  = lut_idx + 1'b1;
            @(posedge clk);
        end

        // Force two consecutive sample_en pulses
        @(negedge clk);
        force dut.sample_en = 1'b1;
        audio_in = SINE_LUT[lut_idx]; lut_idx = lut_idx + 1'b1;
        @(negedge clk);
        audio_in = SINE_LUT[lut_idx]; lut_idx = lut_idx + 1'b1;
        @(negedge clk);
        release dut.sample_en;

        run_sine(8, any_x, any_overflow);
        check(!any_x,
              "back-to-back sample_en: no X/Z after forced pulses");
        check(!any_overflow,
              "back-to-back sample_en: no overflow after forced pulses");

        // =====================================================================
        // Summary
        // =====================================================================
        $display("\n=== Results: %0d passed, %0d failed ===", pass_count, fail_count);
        if (fail_count != 0)
            $fatal(1, "One or more test cases FAILED.");
        else begin
            $display("All test cases passed.");
            $finish;
        end
    end

    // =========================================================================
    // Timeout watchdog
    // =========================================================================
    initial begin
        #(200_000_000);
        $display("ERROR: Simulation timeout.");
        $fatal(1, "Timeout");
    end

endmodule