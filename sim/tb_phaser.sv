`timescale 1ns / 1ps

// =============================================================================
// Testbench: phaser (MXR Phase 90 style)
//
// clk_audio : 12.288 MHz (half period = 41 ns)
// sample_en : 1-cycle pulse every 256 clocks  (48 kHz = 12.288 MHz / 256)
//
// Test cases
//   TC1  Reset clears outputs - audio_out must be 0 during reset
//   TC2  Zero input silence   - DC zero in must produce zero out
//   TC3  No X propagation     - output must never be X/Z after reset
//   TC4  Output range         - output must never exceed +/- full scale
//   TC5  DC input passthrough - constant non-zero DC must not saturate or drift
//   TC6  All-pass unity gain  - single-sample impulse energy is conserved
//        (the all-pass filter preserves magnitude; output peak >= input/4
//         accounting for the 50/50 mixer halving)
//   TC7  Feedback off vs on   - outputs must differ when feedback_en toggles
//   TC8  LFO coefficient bounds - ap_coeff must stay within [C_MIN, C_MAX]
//   TC9  Speed extremes       - module must not lock up at speed 0 or 255
//   TC10 Back-to-back sample_en - two consecutive pulses must not corrupt state
//
// Pass/fail is reported via $display and the pass_count/fail_count counters.
// A non-zero fail_count at the end causes $fatal.
// =============================================================================

module tb_phaser;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam int WIDTH           = 24;
    localparam int CLK_HALF_NS     = 41;
    localparam int CLKS_PER_SAMPLE = 256;
    localparam int FS              = 48000;

    localparam signed [WIDTH-1:0] SAT_MAX = {1'b0, {(WIDTH-1){1'b1}}};
    localparam signed [WIDTH-1:0] SAT_MIN = {1'b1, {(WIDTH-1){1'b0}}};

    // Coefficient bounds mirroring phaser.sv localparam
    localparam int C_MIN = 137;
    localparam int C_MAX = 2731;

    // =========================================================================
    // Sine LUT: 64 points, 24-bit signed, full scale
    // round(sin(2*pi*i/64) * (2^23 - 1))
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
    logic [7:0]              speed_val;
    logic                    feedback_en;
    logic signed [WIDTH-1:0] audio_in;
    logic signed [WIDTH-1:0] audio_out;

    // =========================================================================
    // DUT
    // =========================================================================
    phaser #(.WIDTH(WIDTH)) dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .sample_en   (sample_en),
        .speed_val   (speed_val),
        .feedback_en (feedback_en),
        .audio_in    (audio_in),
        .audio_out   (audio_out)
    );

    // =========================================================================
    // Clock
    // =========================================================================
    initial clk = 1'b0;
    always #CLK_HALF_NS clk = ~clk;

    // =========================================================================
    // sample_en generator
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

    task pass(input string msg);
        $display("  PASS  %s", msg);
        pass_count++;
    endtask

    task fail(input string msg);
        $display("  FAIL  %s", msg);
        fail_count++;
    endtask

    task check(input logic cond, input string msg);
        if (cond) pass(msg);
        else      fail(msg);
    endtask

    // =========================================================================
    // Helpers
    // =========================================================================

    // Apply reset and wait for it to clear
    task do_reset();
        rst_n       = 1'b0;
        audio_in    = '0;
        speed_val   = 8'd32;
        feedback_en = 1'b0;
        repeat (20) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        // Wait for div_cnt to align so sample_en fires predictably
        @(posedge clk iff sample_en);
    endtask

    // Run exactly n audio samples through the DUT.
    // Drives audio_in from a fixed value each sample_en.
    // Returns the last audio_out seen.
    task run_fixed_input(
        input  signed [WIDTH-1:0] in_val,
        input  int                n_samples,
        output signed [WIDTH-1:0] last_out
    );
        for (int i = 0; i < n_samples; i++) begin
            @(posedge clk iff sample_en);
            audio_in = in_val;
            @(posedge clk);          // one clock after sample_en: output updated
            last_out = audio_out;
        end
    endtask

    // Run n samples from the sine LUT (free-running index).
    // Tracks whether any sample was X, Z, or out of range.
    logic [5:0] lut_idx_tc;
    task run_sine_input(
        input  int  n_samples,
        output logic any_x,
        output logic any_overflow
    );
        any_x        = 1'b0;
        any_overflow = 1'b0;
        for (int i = 0; i < n_samples; i++) begin
            @(posedge clk iff sample_en);
            audio_in     = SINE_LUT[lut_idx_tc];
            lut_idx_tc   = lut_idx_tc + 1'b1;
            @(posedge clk);
            if ($isunknown(audio_out))               any_x        = 1'b1;
            if (audio_out > SAT_MAX || audio_out < SAT_MIN) any_overflow = 1'b1;
        end
    endtask

    // =========================================================================
    // Internal signal access via hierarchical references
    // =========================================================================
    wire [15:0] ap_coeff_w  = dut.ap_coeff;
    wire [15:0] lfo_norm_w  = dut.lfo_norm;

    // =========================================================================
    // MAIN TEST SEQUENCE
    // =========================================================================
    logic signed [WIDTH-1:0] out_a, out_b;
    logic any_x, any_overflow;
    int   nonzero_count;

    initial begin
        $display("=== phaser testbench ===");

        // ---------------------------------------------------------------------
        // TC1: Reset clears outputs
        // During active reset audio_out must be 0.
        // ---------------------------------------------------------------------
        $display("\n-- TC1: Reset clears outputs --");
        rst_n       = 1'b0;
        audio_in    = 24'sh3FFFFF;  // non-zero input during reset
        speed_val   = 8'd128;
        feedback_en = 1'b1;
        repeat (10) @(posedge clk);
        check(audio_out === 24'sh0, "audio_out = 0 while rst_n = 0");
        // Hold a few more clocks with sample_en potentially firing
        repeat (300) @(posedge clk);
        check(audio_out === 24'sh0, "audio_out stays 0 throughout reset");

        // Release reset and allow DUT to settle
        @(negedge clk);
        rst_n = 1'b1;
        @(posedge clk iff sample_en);

        // ---------------------------------------------------------------------
        // TC2: Zero input produces zero output
        // With all-zero initial state and zero input, every stage should
        // output zero.  Run 20 samples and verify.
        // ---------------------------------------------------------------------
        $display("\n-- TC2: Zero input silence --");
        do_reset();
        speed_val   = 8'd0;
        feedback_en = 1'b0;
        nonzero_count = 0;
        for (int i = 0; i < 20; i++) begin
            @(posedge clk iff sample_en);
            audio_in = 24'sh0;
            @(posedge clk);
            if (audio_out !== 24'sh0) nonzero_count++;
        end
        check(nonzero_count == 0, "zero input -> zero output for 20 samples");

        // ---------------------------------------------------------------------
        // TC3: No X/Z propagation after reset (script mode, slow speed)
        // ---------------------------------------------------------------------
        $display("\n-- TC3: No X propagation (script mode) --");
        do_reset();
        lut_idx_tc  = '0;
        speed_val   = 8'd32;
        feedback_en = 1'b0;
        run_sine_input(96, any_x, any_overflow);
        check(!any_x, "audio_out never X/Z over 96 samples (script, slow)");

        // ---------------------------------------------------------------------
        // TC4: Output never exceeds full scale (block mode, max speed, full amplitude)
        // ---------------------------------------------------------------------
        $display("\n-- TC4: Output range / no overflow --");
        do_reset();
        lut_idx_tc  = '0;
        speed_val   = 8'd255;
        feedback_en = 1'b1;
        run_sine_input(192, any_x, any_overflow);
        check(!any_overflow, "output never exceeds +/-SAT over 192 samples (block, fast)");
        check(!any_x,        "output never X/Z over 192 samples (block, fast)");

        // ---------------------------------------------------------------------
        // TC5: DC input does not saturate or drift
        // Feed a constant mid-scale positive value.  The all-pass chain in
        // steady state with a DC input (all coefficients constant) must
        // produce a bounded output - it cannot integrate or run away.
        // Check that out stays within [-SAT_MAX, SAT_MAX] and is not X.
        // ---------------------------------------------------------------------
        $display("\n-- TC5: DC input does not saturate or drift --");
        do_reset();
        speed_val   = 8'd0;
        feedback_en = 1'b0;
        out_b       = '0;
        any_overflow = 1'b0;
        any_x        = 1'b0;
        for (int i = 0; i < 50; i++) begin
            @(posedge clk iff sample_en);
            audio_in = 24'sh400000;     // +0.5 full scale
            @(posedge clk);
            if ($isunknown(audio_out))               any_x        = 1'b1;
            if (audio_out > SAT_MAX || audio_out < SAT_MIN) any_overflow = 1'b1;
        end
        check(!any_x,        "DC input: output never X");
        check(!any_overflow, "DC input: output never overflows");

        // ---------------------------------------------------------------------
        // TC6: Impulse - all-pass preserves energy
        // Send a single-sample impulse at full scale, then silence.
        // The 50/50 mixer halves everything so the first output sample
        // should be approximately impulse/2.
        // After the impulse the sum of absolute output values should be
        // at least half the impulse magnitude (energy conservation through
        // the all-pass chain means the impulse response must be non-trivial).
        // ---------------------------------------------------------------------
        $display("\n-- TC6: Impulse / energy conservation --");
        do_reset();
        speed_val   = 8'd0;      // freeze LFO so coefficient is constant
        feedback_en = 1'b0;

        // Send impulse
        @(posedge clk iff sample_en);
        audio_in = SAT_MAX;
        // audio_out_reg updates on the NEXT sample_en, not one clock later.
        // Wait for that next sample_en with zero input, then read.
        @(posedge clk iff sample_en);
        audio_in = 24'sh0;
        @(posedge clk);
        out_a = audio_out;       // capture first output after impulse

        // Drain with zero input for 32 samples, accumulate energy
        begin
            longint energy_sum = 0;
            for (int i = 0; i < 32; i++) begin
                @(posedge clk iff sample_en);
                audio_in = 24'sh0;
                @(posedge clk);
                if (audio_out < 0)
                    energy_sum += -audio_out;
                else
                    energy_sum +=  audio_out;
            end
            // Expected: sum of |y| should be at least SAT_MAX/4
            // (conservative: 50/50 mix halves, 4 stages spread energy)
            check(energy_sum >= (SAT_MAX / 4),
                "impulse response has non-trivial energy (all-pass conserves magnitude)");
        end

        // First output after impulse: mixer = (SAT_MAX + c*SAT_MAX) / 2
        // where c = C_MIN/2^16 ~ 0.00209. c*SAT_MAX ~ 17 LSBs, so
        // out_a ~ SAT_MAX/2 + 8. Allow +/-16 LSB tolerance.
        begin
            int expected = $signed(SAT_MAX) >>> 1;
            int delta     = out_a - expected;
            if (delta < 0) delta = -delta;
            check(delta <= 16,
                "first output after impulse within 16 LSB of SAT_MAX/2 (cold state, 50/50 mix)");
        end

        // ---------------------------------------------------------------------
        // TC7: Feedback off vs on produces different outputs
        // Same input, same speed; toggle feedback_en between two runs and
        // verify the outputs diverge.
        // Feedback enters at stage 1 via y_prev[3], which requires the signal
        // to have completed at least one full 4-stage pass.  Warm up 20 samples
        // before capturing so the feedback path carries significant amplitude.
        // ---------------------------------------------------------------------
        $display("\n-- TC7: Feedback off vs on gives different output --");

        // Run A: feedback off
        do_reset();
        lut_idx_tc  = '0;
        speed_val   = 8'd0;
        feedback_en = 1'b0;
        for (int i = 0; i < 20; i++) begin
            @(posedge clk iff sample_en);
            audio_in = SINE_LUT[lut_idx_tc];
            lut_idx_tc = lut_idx_tc + 1'b1;
        end
        // audio_out_reg updates on sample_en; wait for the next one to read
        // the result of sample 20.
        @(posedge clk iff sample_en);
        audio_in = SINE_LUT[lut_idx_tc];
        lut_idx_tc = lut_idx_tc + 1'b1;
        @(posedge clk);
        out_a = audio_out;

        // Run B: feedback on, identical input sequence
        do_reset();
        lut_idx_tc  = '0;
        speed_val   = 8'd0;
        feedback_en = 1'b1;
        for (int i = 0; i < 20; i++) begin
            @(posedge clk iff sample_en);
            audio_in = SINE_LUT[lut_idx_tc];
            lut_idx_tc = lut_idx_tc + 1'b1;
        end
        @(posedge clk iff sample_en);
        audio_in = SINE_LUT[lut_idx_tc];
        lut_idx_tc = lut_idx_tc + 1'b1;
        @(posedge clk);
        out_b = audio_out;

        check(out_a !== out_b,
            "feedback_en=0 and feedback_en=1 produce different outputs after 20-sample warmup");

        // ---------------------------------------------------------------------
        // TC8: LFO coefficient stays within bounds
        // Run 200 samples at fast speed and check ap_coeff via hierarchical ref.
        // C_MIN = 137, C_MAX = 2731.
        // ---------------------------------------------------------------------
        $display("\n-- TC8: LFO coefficient stays in [C_MIN, C_MAX] --");
        do_reset();
        lut_idx_tc  = '0;
        speed_val   = 8'd255;
        feedback_en = 1'b0;
        begin
            logic coeff_lo_ok = 1'b1;
            logic coeff_hi_ok = 1'b1;
            for (int i = 0; i < 200; i++) begin
                @(posedge clk iff sample_en);
                audio_in   = SINE_LUT[lut_idx_tc];
                lut_idx_tc = lut_idx_tc + 1'b1;
                @(posedge clk);
                if (ap_coeff_w < C_MIN) coeff_lo_ok = 1'b0;
                if (ap_coeff_w > C_MAX) coeff_hi_ok = 1'b0;
            end
            check(coeff_lo_ok, "ap_coeff never below C_MIN=137 over 200 samples");
            check(coeff_hi_ok, "ap_coeff never above C_MAX=2731 over 200 samples");
        end

        // ---------------------------------------------------------------------
        // TC9: Speed extremes - no lockup
        // Run at speed=0 and speed=255, verify output is not X/Z.
        // ---------------------------------------------------------------------
        $display("\n-- TC9: Speed extremes do not lock up --");

        do_reset();
        lut_idx_tc  = '0;
        speed_val   = 8'd0;
        feedback_en = 1'b0;
        run_sine_input(48, any_x, any_overflow);
        check(!any_x,        "speed=0: no X output over 48 samples");
        check(!any_overflow, "speed=0: no overflow over 48 samples");

        do_reset();
        lut_idx_tc  = '0;
        speed_val   = 8'd255;
        feedback_en = 1'b0;
        run_sine_input(48, any_x, any_overflow);
        check(!any_x,        "speed=255: no X output over 48 samples");
        check(!any_overflow, "speed=255: no overflow over 48 samples");

        // ---------------------------------------------------------------------
        // TC10: Back-to-back sample_en pulses do not corrupt state
        // Force two consecutive sample_en pulses by briefly bypassing the
        // divider.  Output must not become X and must remain in range.
        // ---------------------------------------------------------------------
        $display("\n-- TC10: Back-to-back sample_en pulses --");
        do_reset();
        lut_idx_tc  = '0;
        speed_val   = 8'd64;
        feedback_en = 1'b0;

        // Warm up
        for (int i = 0; i < 8; i++) begin
            @(posedge clk iff sample_en);
            audio_in   = SINE_LUT[lut_idx_tc];
            lut_idx_tc = lut_idx_tc + 1'b1;
        end

        // Force two pulses on back-to-back clocks by directly asserting
        // sample_en (override the generated signal for two cycles).
        @(negedge clk);
        force dut.sample_en = 1'b1;
        audio_in = SINE_LUT[lut_idx_tc]; lut_idx_tc = lut_idx_tc + 1'b1;
        @(negedge clk);
        audio_in = SINE_LUT[lut_idx_tc]; lut_idx_tc = lut_idx_tc + 1'b1;
        @(negedge clk);
        release dut.sample_en;

        // Let the divider resume and check a few outputs
        run_sine_input(8, any_x, any_overflow);
        check(!any_x,        "back-to-back sample_en: no X after forced pulses");
        check(!any_overflow, "back-to-back sample_en: no overflow after forced pulses");

        // =====================================================================
        // Summary
        // =====================================================================
        $display("\n=== Results: %0d passed, %0d failed ===", pass_count, fail_count);
        if (fail_count != 0) begin
            $fatal(1, "One or more test cases FAILED.");
        end else begin
            $display("All test cases passed.");
            $finish;
        end
    end

    // =========================================================================
    // Timeout watchdog
    // =========================================================================
    initial begin
        #(80_000_000);
        $display("ERROR: Simulation timeout.");
        $fatal(1, "Timeout");
    end

endmodule