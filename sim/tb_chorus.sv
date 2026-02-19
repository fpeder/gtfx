`timescale 1ns / 1ps

// ============================================================
// chorus_tb.sv - Testbench for chorus module
//
// Clock:      12.288 MHz  (clk_audio, period ≈ 81.38 ns)
// Sample rate: 48 kHz     (sample_en pulse every 256 clk cycles)
// ============================================================

module chorus_tb;

    localparam int SAMPLE_RATE = 48_000;
    localparam int DATA_WIDTH  = 24;
    localparam int DELAY_MAX   = 512;

    localparam real CLK_PERIOD_NS = 81.380208;
    localparam int  CLKS_PER_SAMPLE = 256;

    // --------------------------------------------------------
    // Signals
    // --------------------------------------------------------
    logic                          clk = 0;
    logic                          rst_n = 0;
    logic                          sample_en = 0;
    logic signed [DATA_WIDTH-1:0]  audio_in = '0;
    logic signed [DATA_WIDTH-1:0]  audio_out_l;
    logic signed [DATA_WIDTH-1:0]  audio_out_r;
    logic [7:0]                    rate;
    logic [7:0]                    depth;
    logic [7:0]                    effect_lvl;
    logic [7:0]                    e_q_hi;
    logic [7:0]                    e_q_lo;

    // --------------------------------------------------------
    // DUT
    // --------------------------------------------------------
    chorus #(
        .SAMPLE_RATE(SAMPLE_RATE),
        .DATA_WIDTH (DATA_WIDTH),
        .DELAY_MAX  (DELAY_MAX)
    ) dut (.*);

    // --------------------------------------------------------
    // Clock: 12.288 MHz
    // --------------------------------------------------------
    always #(CLK_PERIOD_NS / 2.0) clk = ~clk;

    // --------------------------------------------------------
    // sample_en: 1-cycle pulse every 256 clk cycles
    // --------------------------------------------------------
    int clk_cnt = 0;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            clk_cnt   <= 0;
            sample_en <= 0;
        end else begin
            if (clk_cnt == CLKS_PER_SAMPLE - 1) begin
                clk_cnt   <= 0;
                sample_en <= 1;
            end else begin
                clk_cnt   <= clk_cnt + 1;
                sample_en <= 0;
            end
        end
    end

    // --------------------------------------------------------
    // 1 kHz sine LUT (48 samples/cycle, ~25% full scale)
    // --------------------------------------------------------
    localparam int SINE_SAMPLES = 48;
    localparam real PI = 3.14159265358979;
    logic signed [DATA_WIDTH-1:0] sine_lut [0:SINE_SAMPLES-1];
    int sample_idx = 0;

    initial begin
        for (int i = 0; i < SINE_SAMPLES; i++)
            sine_lut[i] = DATA_WIDTH'($rtoi(
                $sin(2.0 * PI * i / SINE_SAMPLES) * (2.0**(DATA_WIDTH-2) - 1)));
    end

    // --------------------------------------------------------
    // Helper tasks
    // --------------------------------------------------------
    task automatic wait_sample();
        @(posedge clk);
        while (!sample_en) @(posedge clk);
    endtask

    task automatic feed_sine(int n);
        for (int i = 0; i < n; i++) begin
            wait_sample();
            audio_in <= sine_lut[sample_idx];
            sample_idx = (sample_idx + 1) % SINE_SAMPLES;
        end
    endtask

    task automatic feed_silence(int n);
        for (int i = 0; i < n; i++) begin
            wait_sample();
            audio_in <= '0;
        end
    endtask

    task automatic feed_dc(int n, logic signed [DATA_WIDTH-1:0] dc_val);
        for (int i = 0; i < n; i++) begin
            wait_sample();
            audio_in <= dc_val;
        end
    endtask

    // --------------------------------------------------------
    // Test sequence
    // --------------------------------------------------------
    int pass_cnt = 0;
    int fail_cnt = 0;

    initial begin
        $dumpfile("chorus_tb.vcd");
        $dumpvars(0, chorus_tb);

        // Default knobs
        rate       = 8'd80;
        depth      = 8'd100;
        effect_lvl = 8'd128;
        e_q_hi     = 8'd200;
        e_q_lo     = 8'd128;

        // ===========================================================
        // TEST 1: Reset - outputs must be zero
        // ===========================================================
        $display("=== TEST 1: Reset ===");
        rst_n = 0;
        repeat (20) @(posedge clk);

        if (audio_out_l === 24'd0 && audio_out_r === 24'd0) begin
            $display("  PASS: Outputs zero during reset");
            pass_cnt++;
        end else begin
            $display("  FAIL: L=0x%06h R=0x%06h (expected 0)", audio_out_l, audio_out_r);
            fail_cnt++;
        end

        @(posedge clk);
        rst_n = 1;

        // ===========================================================
        // TEST 2: No X/Z after reset with default knobs
        //         Feed sine for 100 samples, check outputs are known
        // ===========================================================
        $display("=== TEST 2: No X/Z with default knobs ===");
        sample_idx = 0;
        feed_sine(100);
        begin
            int xz_cnt = 0;
            for (int i = 0; i < 100; i++) begin
                wait_sample();
                audio_in <= sine_lut[sample_idx];
                sample_idx = (sample_idx + 1) % SINE_SAMPLES;
                if ($isunknown(audio_out_l) || $isunknown(audio_out_r)) begin
                    if (xz_cnt < 5)
                        $display("  DEBUG: X/Z at sample %0d L=0x%06h R=0x%06h",
                                 i, audio_out_l, audio_out_r);
                    xz_cnt++;
                end
            end
            if (xz_cnt == 0) begin
                $display("  PASS: All outputs known");
                pass_cnt++;
            end else begin
                $display("  FAIL: %0d samples with X/Z", xz_cnt);
                $display("  DEBUG: lfo_inc=0x%08h phase_l=0x%08h phase_r=0x%08h",
                         dut.lfo_inc, dut.lfo_phase_l, dut.lfo_phase_r);
                $display("  DEBUG: mod_off_l=%0d mod_off_r=%0d",
                         dut.mod_off_l, dut.mod_off_r);
                $display("  DEBUG: lpf_l=0x%06h lpf_r=0x%06h",
                         dut.lpf_l, dut.lpf_r);
                $display("  DEBUG: dry_gain_s=%0d wet_gain_s=%0d",
                         dut.dry_gain, dut.wet_gain);
                fail_cnt++;
            end
        end

        // ===========================================================
        // TEST 3: Bypass - effect_lvl=0 (100% dry), L == R
        // ===========================================================
        $display("=== TEST 3: Bypass (effect_lvl=0) ===");
        effect_lvl = 8'd0;
        sample_idx = 0;

        feed_sine(DELAY_MAX + 64);

        begin
            int bypass_ok = 1;
            for (int i = 0; i < 48; i++) begin
                wait_sample();
                audio_in <= sine_lut[sample_idx];
                sample_idx = (sample_idx + 1) % SINE_SAMPLES;
                if ($isunknown(audio_out_l) || $isunknown(audio_out_r)) begin
                    $display("  DEBUG: X/Z at bypass sample %0d", i);
                    bypass_ok = 0;
                end else if (audio_out_l !== audio_out_r) begin
                    $display("  DEBUG: L=0x%06h != R=0x%06h at sample %0d",
                             audio_out_l, audio_out_r, i);
                    bypass_ok = 0;
                end
            end
            if (bypass_ok) begin
                $display("  PASS: L == R in bypass mode");
                pass_cnt++;
            end else begin
                $display("  FAIL: Bypass mode errors");
                fail_cnt++;
            end
        end

        // ===========================================================
        // TEST 4: Full wet - L and R should differ (quadrature LFO)
        // ===========================================================
        $display("=== TEST 4: Full wet (effect_lvl=FF), L/R quadrature ===");
        effect_lvl = 8'hFF;
        rate       = 8'd200;
        depth      = 8'd254;
        e_q_hi     = 8'hFF;
        sample_idx = 0;

        // Prime delay buffer
        feed_sine(DELAY_MAX + 200);

        begin
            int lr_differ_cnt = 0;
            int xz_cnt = 0;
            for (int i = 0; i < 500; i++) begin
                wait_sample();
                audio_in <= sine_lut[sample_idx];
                sample_idx = (sample_idx + 1) % SINE_SAMPLES;
                if ($isunknown(audio_out_l) || $isunknown(audio_out_r)) begin
                    xz_cnt++;
                end else if (audio_out_l != audio_out_r) begin
                    lr_differ_cnt++;
                end
            end
            $display("  INFO: L/R differ in %0d/500 known samples (%0d X/Z)",
                     lr_differ_cnt, xz_cnt);
            if (xz_cnt > 0) begin
                $display("  FAIL: X/Z in outputs");
                fail_cnt++;
            end else if (lr_differ_cnt > 50) begin
                $display("  PASS: L/R quadrature confirmed");
                pass_cnt++;
            end else begin
                $display("  FAIL: L/R not diverging enough");
                $display("  DEBUG: lfo_phase_l=0x%08h lfo_phase_r=0x%08h",
                         dut.lfo_phase_l, dut.lfo_phase_r);
                $display("  DEBUG: lfo_l_val=%0d lfo_r_val=%0d mod_amp=%0d",
                         dut.lfo_l_val, dut.lfo_r_val, dut.mod_amp);
                $display("  DEBUG: mod_off_l=%0d mod_off_r=%0d",
                         dut.mod_off_l, dut.mod_off_r);
                $display("  DEBUG: rd_ptr_l=%0d rd_ptr_r=%0d wr_ptr=%0d",
                         dut.rd_ptr_l, dut.rd_ptr_r, dut.wr_ptr);
                $display("  DEBUG: lpf_l=0x%06h lpf_r=0x%06h",
                         dut.lpf_l, dut.lpf_r);
                $display("  DEBUG: audio_out_l=0x%06h audio_out_r=0x%06h",
                         audio_out_l, audio_out_r);
                fail_cnt++;
            end
        end

        // ===========================================================
        // TEST 5: Knob sweep - no X/Z across full range
        // ===========================================================
        $display("=== TEST 5: Knob sweep ===");
        effect_lvl = 8'd128;
        e_q_hi     = 8'd200;
        depth      = 8'd100;
        sample_idx = 0;
        begin
            int xz_found = 0;

            for (int r = 0; r < 256; r += 32) begin
                rate = r[7:0];
                for (int i = 0; i < 20; i++) begin
                    wait_sample();
                    audio_in <= sine_lut[sample_idx];
                    sample_idx = (sample_idx + 1) % SINE_SAMPLES;
                    if ($isunknown(audio_out_l) || $isunknown(audio_out_r))
                        xz_found++;
                end
            end

            rate = 8'd128;
            for (int d = 0; d < 256; d += 32) begin
                depth = d[7:0];
                for (int i = 0; i < 20; i++) begin
                    wait_sample();
                    audio_in <= sine_lut[sample_idx];
                    sample_idx = (sample_idx + 1) % SINE_SAMPLES;
                    if ($isunknown(audio_out_l) || $isunknown(audio_out_r))
                        xz_found++;
                end
            end

            depth = 8'd100;
            for (int eq = 0; eq < 256; eq += 32) begin
                e_q_hi = eq[7:0];
                for (int i = 0; i < 20; i++) begin
                    wait_sample();
                    audio_in <= sine_lut[sample_idx];
                    sample_idx = (sample_idx + 1) % SINE_SAMPLES;
                    if ($isunknown(audio_out_l) || $isunknown(audio_out_r))
                        xz_found++;
                end
            end

            // Also sweep effect_lvl (this was the broken path)
            e_q_hi = 8'd200;
            for (int e = 0; e <= 255; e += 32) begin
                effect_lvl = e[7:0];
                for (int i = 0; i < 10; i++) begin
                    wait_sample();
                    audio_in <= sine_lut[sample_idx];
                    sample_idx = (sample_idx + 1) % SINE_SAMPLES;
                    if ($isunknown(audio_out_l) || $isunknown(audio_out_r))
                        xz_found++;
                end
            end

            if (xz_found == 0) begin
                $display("  PASS: No X/Z during knob sweeps");
                pass_cnt++;
            end else begin
                $display("  FAIL: %0d X/Z detections during knob sweeps", xz_found);
                fail_cnt++;
            end
        end

        // ===========================================================
        // TEST 6: Silence - output decays to near-zero
        // ===========================================================
        $display("=== TEST 6: Silence input ===");
        effect_lvl = 8'd128;
        rate       = 8'd80;
        depth      = 8'd100;
        e_q_hi     = 8'd200;

        feed_silence(1500);

        begin
            int silence_ok = 1;
            for (int i = 0; i < 100; i++) begin
                wait_sample();
                audio_in <= '0;
                if (!$isunknown(audio_out_l) && !$isunknown(audio_out_r)) begin
                    if ($signed(audio_out_l) > 24'sd83886 ||
                        $signed(audio_out_l) < -24'sd83886 ||
                        $signed(audio_out_r) > 24'sd83886 ||
                        $signed(audio_out_r) < -24'sd83886)
                        silence_ok = 0;
                end else begin
                    silence_ok = 0;
                end
            end
            if (silence_ok) begin
                $display("  PASS: Output near zero with silent input");
                pass_cnt++;
            end else begin
                $display("  FAIL: Output not near zero or X/Z present");
                fail_cnt++;
            end
        end

        // ===========================================================
        // TEST 7: DC input - no X/Z
        // ===========================================================
        $display("=== TEST 7: DC input stability ===");
        effect_lvl = 8'hFF;
        rate       = 8'd128;
        depth      = 8'd128;
        e_q_hi     = 8'hFF;

        feed_dc(1000, 24'sh3FFFFF);

        begin
            int dc_ok = 1;
            for (int i = 0; i < 200; i++) begin
                wait_sample();
                audio_in <= 24'sh3FFFFF;
                if ($isunknown(audio_out_l) || $isunknown(audio_out_r))
                    dc_ok = 0;
            end
            if (dc_ok) begin
                $display("  PASS: DC input stable");
                pass_cnt++;
            end else begin
                $display("  FAIL: X/Z with DC input");
                fail_cnt++;
            end
        end

        // ===========================================================
        // TEST 8: Zero depth - L == R (no modulation)
        // ===========================================================
        $display("=== TEST 8: Zero depth (no modulation) ===");
        effect_lvl = 8'hFF;
        rate       = 8'd128;
        depth      = 8'd0;
        e_q_hi     = 8'hFF;
        sample_idx = 0;

        feed_sine(DELAY_MAX + 200);

        begin
            int ok = 1;
            for (int i = 0; i < 100; i++) begin
                wait_sample();
                audio_in <= sine_lut[sample_idx];
                sample_idx = (sample_idx + 1) % SINE_SAMPLES;
                if ($isunknown(audio_out_l) || $isunknown(audio_out_r)) begin
                    ok = 0;
                end else if (audio_out_l !== audio_out_r) begin
                    ok = 0;
                end
            end
            if (ok) begin
                $display("  PASS: L == R with zero depth");
                pass_cnt++;
            end else begin
                $display("  FAIL: L != R or X/Z with zero depth");
                fail_cnt++;
            end
        end

        // ===========================================================
        $display("");
        $display("========================================");
        $display("  RESULTS: %0d passed, %0d failed", pass_cnt, fail_cnt);
        $display("========================================");
        $finish;
    end

    // Timeout
    initial begin
        #500_000_000;
        $display("ERROR: Simulation timeout!");
        $finish;
    end

endmodule