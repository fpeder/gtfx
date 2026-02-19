`timescale 1ns / 1ps

module delay_tb;

    // =========================================================================
    // Parameters -- small RAM for fast simulation
    // =========================================================================
    localparam WIDTH     = 24;
    localparam RAM_DEPTH = 64;
    localparam DELAY     = 10;

    // Pipeline latency: ram_read_data(+1) -> lpf_state(+1) -> audio_out(+1) = 3
    // Plus 1 extra because ram[wr_ptr] writes the OLD ram_write_data.
    // Total echo latency = DELAY + 3 sample_en edges after the impulse.
    localparam PIPE = 3;

    localparam signed [WIDTH-1:0] POS_MAX =  (2**(WIDTH-1)) - 1;
    localparam signed [WIDTH-1:0] NEG_MIN = -(2**(WIDTH-1));

    // =========================================================================
    // Clock & Reset
    // =========================================================================
    logic clk = 0;
    logic rst_n = 0;
    always #5 clk = ~clk;  // 100 MHz

    // =========================================================================
    // DUT Signals
    // =========================================================================
    logic             sample_en;
    logic [7:0]       tone_val;
    logic [7:0]       level_val;
    logic [7:0]       feedback_val;
    logic [31:0]      time_val;
    logic signed [WIDTH-1:0] audio_in;
    logic signed [WIDTH-1:0] audio_out;

    delay #(
        .WIDTH     (WIDTH),
        .RAM_DEPTH (RAM_DEPTH)
    ) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .sample_en    (sample_en),
        .tone_val     (tone_val),
        .level_val    (level_val),
        .feedback_val (feedback_val),
        .time_val     (time_val),
        .audio_in     (audio_in),
        .audio_out    (audio_out)
    );

    // =========================================================================
    // Test Tracking
    // =========================================================================
    int test_passed = 0;
    int test_failed = 0;

    task check(input string name, input logic pass, input string msg = "");
        if (pass) begin
            $display("  [PASS] %s %s", name, msg);
            test_passed++;
        end else begin
            $display("  [FAIL] %s %s", name, msg);
            test_failed++;
        end
    endtask

    // =========================================================================
    // Helper Tasks
    // =========================================================================

    // Pulse sample_en for one clock and wait one idle clock.
    // The output is captured AFTER the posedge where sample_en was high,
    // so audio_out reflects the newly registered value.
    task automatic pulse_sample();
        @(posedge clk);
        sample_en = 1;
        @(posedge clk);
        sample_en = 0;
    endtask

    // Reset DUT and set default controls
    task automatic do_reset();
        rst_n     = 0;
        sample_en = 0;
        audio_in  = 0;
        tone_val     = 8'hFF;
        level_val    = 8'hFF;
        feedback_val = 8'h00;
        time_val     = DELAY;
        repeat (5) @(posedge clk);
        rst_n = 1;
        @(posedge clk);
    endtask

    // =========================================================================
    // Capture buffer -- record audio_out after each sample_en pulse
    // =========================================================================
    logic signed [WIDTH-1:0] cap [0:255];
    int cap_len;

    task automatic cap_reset();
        cap_len = 0;
    endtask

    // Run N sample_en pulses with given audio_in, capturing audio_out each time.
    task automatic run_cap(input int n, input logic signed [WIDTH-1:0] val);
        audio_in = val;
        repeat (n) begin
            pulse_sample();
            if (cap_len < 256) begin
                cap[cap_len] = audio_out;
                cap_len++;
            end
        end
    endtask

    // Dump capture buffer
    task automatic dump_cap(input int first, input int last);
        for (int i = first; i <= last && i < cap_len; i++) begin
            $display("    [%0d] = %0d", i, cap[i]);
        end
    endtask

    // Find abs-peak in capture buffer range [lo..hi]
    function automatic logic signed [WIDTH-1:0] peak_in(int lo, int hi);
        logic signed [WIDTH-1:0] p;
        p = 0;
        for (int i = lo; i <= hi; i++) begin
            if (i < cap_len) begin
                if (cap[i] > p)  p =  cap[i];
                if (-cap[i] > p) p = -cap[i];
            end
        end
        return p;
    endfunction

    // Find index of first non-zero sample in range [lo..hi], -1 if none.
    function automatic int first_nonzero(int lo, int hi);
        for (int i = lo; i <= hi; i++) begin
            if (i < cap_len && cap[i] != 0) return i;
        end
        return -1;
    endfunction

    // =========================================================================
    // Test Stimulus
    // =========================================================================
    initial begin
        $display("\n========================================");
        $display("   delay Module Testbench");
        $display("========================================\n");

        // =================================================================
        // TEST 1: Reset
        // =================================================================
        $display("TEST 1: Reset clears output");
        do_reset();
        check("Output is zero after reset", audio_out === 0,
              $sformatf("Got %0d", audio_out));

        // =================================================================
        // TEST 2: Dry passthrough
        // =================================================================
        // With fb=0 and empty delay line, lpf_state=0, so:
        //   audio_out = audio_in + (0 * level)>>>8 = audio_in
        // This is immediate on the first sample_en (registered output).
        $display("\nTEST 2: Dry passthrough (no feedback, empty delay line)");
        do_reset();

        cap_reset();
        run_cap(1, 24'sd100000);

        $display("  cap[0] = %0d (expect 100000)", cap[0]);
        check("First sample passthrough", cap[0] == 24'sd100000,
              $sformatf("Got %0d", cap[0]));

        // =================================================================
        // TEST 3: Impulse echo at correct delay
        // =================================================================
        // Impulse at sample 0, silence after. Echo expected near sample
        // DELAY+PIPE due to registered pipeline stages.
        $display("\nTEST 3: Impulse echo after delay");
        do_reset();

        cap_reset();
        run_cap(1, 24'sd500000);             // sample 0: impulse
        run_cap(DELAY + PIPE + 5, 24'sd0);   // silence, wait for echo

        $display("  Captured around echo region (DELAY=%0d, PIPE=%0d):", DELAY, PIPE);
        dump_cap(0, DELAY + PIPE + 4);

        begin
            int echo_idx;
            // Skip sample 0 (that's the direct passthrough, not the echo)
            echo_idx = first_nonzero(DELAY, DELAY + PIPE + 2);
            $display("  First non-zero echo at index: %0d (expected ~%0d)", echo_idx, DELAY + PIPE);
            check("Echo appears in expected window",
                  echo_idx >= DELAY && echo_idx <= DELAY + PIPE + 2,
                  $sformatf("idx=%0d", echo_idx));
        end

        // =================================================================
        // TEST 4: Feedback decay
        // =================================================================
        $display("\nTEST 4: Feedback causes decaying echoes");
        do_reset();
        feedback_val = 8'h80;  // ~50%
        tone_val     = 8'hFF;

        cap_reset();
        run_cap(1, 24'sd400000);
        run_cap(4 * (DELAY + PIPE) + 10, 24'sd0);

        // Find peaks in three successive echo windows
        begin
            logic signed [WIDTH-1:0] p1, p2, p3;
            int w;
            w = DELAY + PIPE;
            p1 = peak_in(w - 1,     w + 3);
            p2 = peak_in(2*w - 1, 2*w + 3);
            p3 = peak_in(3*w - 1, 3*w + 3);
            $display("  Echo peaks: 1st=%0d  2nd=%0d  3rd=%0d (window=%0d)", p1, p2, p3, w);
            check("1st echo exists (> 0)", p1 > 0, $sformatf("%0d", p1));
            check("1st echo > 2nd echo (decay)", p1 > p2,
                  $sformatf("%0d > %0d", p1, p2));
        end

        // =================================================================
        // TEST 5: Tone bright vs dark
        // =================================================================
        $display("\nTEST 5: Tone bright (FF) vs dark (00)");

        // -- Bright --
        do_reset();
        feedback_val = 8'h00;
        tone_val     = 8'hFF;

        cap_reset();
        run_cap(1, 24'sd400000);
        run_cap(DELAY + PIPE + 5, 24'sd0);

        begin
            logic signed [WIDTH-1:0] bright_peak;
            bright_peak = peak_in(DELAY, DELAY + PIPE + 2);
            $display("  Bright echo peak: %0d", bright_peak);

            // -- Dark --
            do_reset();
            feedback_val = 8'h00;
            tone_val     = 8'h00;

            cap_reset();
            run_cap(1, 24'sd400000);
            run_cap(DELAY + PIPE + 5, 24'sd0);

            begin
                logic signed [WIDTH-1:0] dark_peak;
                dark_peak = peak_in(DELAY, DELAY + PIPE + 2);
                $display("  Dark echo peak:   %0d", dark_peak);
                check("Bright echo > dark echo",
                      bright_peak > dark_peak,
                      $sformatf("bright=%0d dark=%0d", bright_peak, dark_peak));
            end
        end

        // =================================================================
        // TEST 6: Level scaling
        // =================================================================
        $display("\nTEST 6: Level scaling (full vs half)");

        // -- Full level --
        do_reset();
        level_val    = 8'hFF;
        feedback_val = 8'h00;
        tone_val     = 8'hFF;

        cap_reset();
        run_cap(1, 24'sd400000);
        run_cap(DELAY + PIPE + 5, 24'sd0);

        begin
            logic signed [WIDTH-1:0] full_pk;
            full_pk = peak_in(DELAY, DELAY + PIPE + 2);

            // -- Half level --
            do_reset();
            level_val    = 8'h80;
            feedback_val = 8'h00;
            tone_val     = 8'hFF;

            cap_reset();
            run_cap(1, 24'sd400000);
            run_cap(DELAY + PIPE + 5, 24'sd0);

            begin
                logic signed [WIDTH-1:0] half_pk;
                half_pk = peak_in(DELAY, DELAY + PIPE + 2);
                $display("  Full level echo: %0d", full_pk);
                $display("  Half level echo: %0d", half_pk);
                check("Full > half", full_pk > half_pk,
                      $sformatf("full=%0d half=%0d", full_pk, half_pk));
                if (full_pk > 0) begin
                    check("Half ~ 50% of full",
                          half_pk > (full_pk / 4) && half_pk < (full_pk * 3 / 4),
                          $sformatf("ratio=%0d%%", (half_pk * 100) / full_pk));
                end
            end
        end

        // =================================================================
        // TEST 7: Silence in -> silence out
        // =================================================================
        // Note: do_reset() resets pointers but not RAM contents. Previous
        // tests leave data in RAM, so we flush the entire delay line with
        // silence first (RAM_DEPTH + pipeline samples), then capture.
        $display("\nTEST 7: Silence in -> silence out");
        do_reset();
        feedback_val = 8'h80;
        tone_val     = 8'hFF;

        // Flush: run enough silence to overwrite all RAM and drain pipeline
        run_cap(RAM_DEPTH + PIPE + 5, 24'sd0);

        // Now capture fresh silence
        cap_reset();
        run_cap(DELAY + PIPE + 20, 24'sd0);

        begin
            logic all_zero;
            int   fail_idx;
            all_zero = 1;
            fail_idx = -1;
            for (int i = 0; i < cap_len; i++) begin
                if (cap[i] !== 0) begin
                    all_zero = 0;
                    if (fail_idx == -1) fail_idx = i;
                end
            end
            if (!all_zero) begin
                $display("  First non-zero at [%0d] = %0d", fail_idx, cap[fail_idx]);
            end
            check("All outputs zero with silent input", all_zero);
        end

        // =================================================================
        // TEST 8: Different delay times
        // =================================================================
        $display("\nTEST 8: Short delay (5) vs long delay (20)");

        // -- Short --
        do_reset();
        time_val = 5;

        cap_reset();
        run_cap(1, 24'sd300000);
        run_cap(30, 24'sd0);

        begin
            int short_idx;
            short_idx = first_nonzero(2, 30);  // skip sample 0 (passthrough)

            // -- Long --
            do_reset();
            time_val = 20;

            cap_reset();
            run_cap(1, 24'sd300000);
            run_cap(40, 24'sd0);

            begin
                int long_idx;
                long_idx = first_nonzero(2, 40);
                $display("  Short delay echo at: %0d", short_idx);
                $display("  Long delay echo at:  %0d", long_idx);
                check("Long echo arrives later", long_idx > short_idx,
                      $sformatf("long=%0d short=%0d", long_idx, short_idx));
            end
        end

        // =================================================================
        // TEST 9: Saturation (no overflow wrap)
        // =================================================================
        $display("\nTEST 9: Saturation -- no overflow wrap");
        do_reset();
        feedback_val = 8'hFF;
        tone_val     = 8'hFF;
        level_val    = 8'hFF;

        cap_reset();
        run_cap(1, POS_MAX);
        run_cap(4 * (DELAY + PIPE), 24'sd0);

        begin
            logic no_wrap;
            no_wrap = 1;
            for (int i = 0; i < cap_len; i++) begin
                // A wrap would cause a large negative value from a positive input
                if ($signed(cap[i]) < -(POS_MAX / 2)) no_wrap = 0;
            end
            check("No overflow wrap", no_wrap);
        end

        // =================================================================
        // TEST 10: Zero level -> output = audio_in only
        // =================================================================
        $display("\nTEST 10: Zero level -> output equals audio_in");
        do_reset();
        level_val    = 8'h00;
        feedback_val = 8'h00;
        tone_val     = 8'hFF;

        // Prime the delay line with data so there would be wet content
        run_cap(DELAY + PIPE + 5, 24'sd200000);

        // Now switch input and check output = new audio_in (no wet)
        cap_reset();
        run_cap(5, 24'sd123456);

        $display("  Output samples with level=0:");
        dump_cap(0, 4);

        // audio_out = audio_in + (lpf_state * 0)>>>8 = audio_in
        // Should be true immediately since level=0 zeroes the wet path
        check("Output = audio_in when level=0",
              cap[0] == 24'sd123456,
              $sformatf("Got cap[0]=%0d", cap[0]));

        // =================================================================
        // TEST 11: Negative audio input
        // =================================================================
        $display("\nTEST 11: Negative impulse produces negative echo");
        do_reset();
        feedback_val = 8'h00;
        tone_val     = 8'hFF;

        cap_reset();
        run_cap(1, -24'sd400000);
        run_cap(DELAY + PIPE + 5, 24'sd0);

        begin
            logic found_neg;
            found_neg = 0;
            for (int i = DELAY; i <= DELAY + PIPE + 2 && i < cap_len; i++) begin
                if ($signed(cap[i]) < 0) found_neg = 1;
            end
            check("Negative echo found", found_neg);
        end

        // =================================================================
        // TEST 12: time_val clamping
        // =================================================================
        $display("\nTEST 12: time_val clamping");
        do_reset();

        time_val = 32'hFFFFFFFF;
        #1;  // let comb logic settle
        check("Clamp to RAM_DEPTH-1", dut.delay_samples == RAM_DEPTH - 1,
              $sformatf("Got %0d, expected %0d", dut.delay_samples, RAM_DEPTH - 1));

        time_val = 32'd0;
        #1;
        check("Clamp zero to 1", dut.delay_samples == 1,
              $sformatf("Got %0d", dut.delay_samples));

        time_val = 32'd30;
        #1;
        check("Normal value passthrough", dut.delay_samples == 30,
              $sformatf("Got %0d", dut.delay_samples));

        // =================================================================
        // TEST 13: Tone=FF is true bypass (echo ~ input amplitude)
        // =================================================================
        $display("\nTEST 13: Tone FF gives near-unity echo");
        do_reset();
        feedback_val = 8'h00;
        tone_val     = 8'hFF;
        level_val    = 8'hFF;

        cap_reset();
        run_cap(1, 24'sd400000);
        run_cap(DELAY + PIPE + 5, 24'sd0);

        begin
            logic signed [WIDTH-1:0] pk;
            pk = peak_in(DELAY, DELAY + PIPE + 2);
            // With level=0xFF (255/256) and tone bypass, echo should be
            // close to 400000 * 255/256 ~ 398437
            $display("  Echo peak = %0d (expect ~398437)", pk);
            check("Echo within 5% of expected",
                  pk > 24'sd370000 && pk < 24'sd410000,
                  $sformatf("Got %0d", pk));
        end

        // =================================================================
        // Summary
        // =================================================================
        $display("\n========================================");
        $display("           TEST SUMMARY");
        $display("========================================");
        $display("  Passed: %0d", test_passed);
        $display("  Failed: %0d", test_failed);

        if (test_failed == 0)
            $display("\n  *** ALL TESTS PASSED ***\n");
        else
            $display("\n  *** SOME TESTS FAILED ***\n");

        repeat (20) @(posedge clk);
        $finish;
    end

    // Timeout
    initial begin
        #10000000;  // 10ms
        $display("\n*** TIMEOUT ***");
        $finish;
    end

endmodule