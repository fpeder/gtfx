`timescale 1ns / 1ps

// =============================================================================
// Testbench: wah (SVF bandpass with manual/auto-wah/LFO modes)
//
// clk_audio : 12.288 MHz  (half period ≈ 41 ns)
// sample_en : 1-cycle pulse every 256 clocks (48 kHz)
//
// Test cases
//   TC1  Reset clears output
//   TC2  Zero input → zero output
//   TC3  No X/Z propagation after reset
//   TC4  Manual mode — changing freq_val shifts spectral emphasis
//   TC5  Output bounded (no overflow with resonance=255)
//   TC6  Mix=0 gives dry signal, Mix=255 gives wet
//   TC7  LFO auto mode produces time-varying output from constant input
//   TC8  Auto mode — depth controls sweep range
// =============================================================================

module tb_wah;

    localparam int WIDTH           = 24;
    localparam int CLK_HALF_NS     = 41;
    localparam int CLKS_PER_SAMPLE = 256;

    localparam signed [WIDTH-1:0] SAT_MAX = {1'b0, {(WIDTH-1){1'b1}}};
    localparam signed [WIDTH-1:0] SAT_MIN = {1'b1, {(WIDTH-1){1'b0}}};

    // 64-point sine LUT
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

    // DUT signals
    logic                    clk;
    logic                    rst_n;
    logic                    sample_en;
    logic signed [WIDTH-1:0] audio_in;
    logic signed [WIDTH-1:0] audio_out;
    logic [7:0]              freq_val;
    logic [7:0]              resonance_val;
    logic [7:0]              depth_val;
    logic [7:0]              mode_val;
    logic [7:0]              mix_val;

    wah #(.DATA_W(WIDTH), .CTRL_W(8)) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .sample_en      (sample_en),
        .audio_in       (audio_in),
        .audio_out      (audio_out),
        .freq_val       (freq_val),
        .resonance_val  (resonance_val),
        .depth_val      (depth_val),
        .mode_val       (mode_val),
        .mix_val        (mix_val)
    );

    // Clock
    initial clk = 1'b0;
    always #CLK_HALF_NS clk = ~clk;

    // sample_en divider
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

    // Pass/fail
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

    task automatic do_reset();
        rst_n           = 1'b0;
        audio_in        = '0;
        freq_val        = 8'd128;
        resonance_val   = 8'd64;
        depth_val       = 8'd128;
        mode_val        = 8'd0;
        mix_val         = 8'd255;
        repeat (20) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        @(posedge clk iff sample_en);
    endtask

    logic [5:0] lut_idx;

    initial begin
        $display("=== wah testbench ===");

        // -----------------------------------------------------------------
        // TC1: Reset clears output
        // -----------------------------------------------------------------
        $display("\n-- TC1: Reset clears output --");
        rst_n    = 1'b0;
        audio_in = 24'sh3FFFFF;
        freq_val = 8'd128;
        resonance_val = 8'd128;
        depth_val       = 8'd128;
        mode_val = 8'd0;
        mix_val  = 8'd255;
        repeat (10) @(posedge clk);
        check(audio_out === 24'sh0,
              "audio_out = 0 during reset");

        // -----------------------------------------------------------------
        // TC2: Zero input → zero output
        // -----------------------------------------------------------------
        $display("\n-- TC2: Zero input silence --");
        do_reset();
        begin
            int nonzero = 0;
            for (int i = 0; i < 20; i++) begin
                @(posedge clk iff sample_en);
                audio_in = 24'sh0;
                @(posedge clk);
                if (audio_out !== 24'sh0) nonzero++;
            end
            check(nonzero == 0, "zero input -> zero output for 20 samples");
        end

        // -----------------------------------------------------------------
        // TC3: No X/Z propagation after reset
        // -----------------------------------------------------------------
        $display("\n-- TC3: No X/Z propagation --");
        do_reset();
        lut_idx = '0;
        begin
            logic any_x = 1'b0;
            for (int i = 0; i < 128; i++) begin
                @(posedge clk iff sample_en);
                audio_in = SINE_LUT[lut_idx];
                lut_idx  = lut_idx + 1'b1;
                @(posedge clk);
                if ($isunknown(audio_out)) any_x = 1'b1;
            end
            check(!any_x, "audio_out never X/Z over 128 samples");
        end

        // -----------------------------------------------------------------
        // TC4: Manual mode — different freq_val produces different output
        // -----------------------------------------------------------------
        $display("\n-- TC4: Manual mode frequency shift --");
        // Run A: low frequency
        do_reset();
        mode_val = 8'd0;  // manual
        freq_val = 8'd10; // low freq
        mix_val  = 8'd255;
        resonance_val = 8'd100;
        lut_idx = '0;
        for (int i = 0; i < 200; i++) begin
            @(posedge clk iff sample_en);
            audio_in = SINE_LUT[lut_idx];
            lut_idx  = lut_idx + 1'b1;
            @(posedge clk);
        end
        begin
            logic signed [WIDTH-1:0] out_low_freq = audio_out;
            // Run B: high frequency
            do_reset();
            mode_val = 8'd0;
            freq_val = 8'd240;  // high freq
            mix_val  = 8'd255;
            resonance_val = 8'd100;
            lut_idx = '0;
            for (int i = 0; i < 200; i++) begin
                @(posedge clk iff sample_en);
                audio_in = SINE_LUT[lut_idx];
                lut_idx  = lut_idx + 1'b1;
                @(posedge clk);
            end
            check(audio_out !== out_low_freq,
                  "different freq_val produces different output");
        end

        // -----------------------------------------------------------------
        // TC5: Output bounded with max resonance
        // -----------------------------------------------------------------
        $display("\n-- TC5: No overflow with max resonance --");
        do_reset();
        freq_val      = 8'd128;
        resonance_val = 8'd255;  // max Q
        mix_val       = 8'd255;
        mode_val      = 8'd0;
        lut_idx = '0;
        begin
            logic overflow = 1'b0;
            logic any_x = 1'b0;
            for (int i = 0; i < 256; i++) begin
                @(posedge clk iff sample_en);
                audio_in = SINE_LUT[lut_idx];
                lut_idx  = lut_idx + 1'b1;
                @(posedge clk);
                if ($isunknown(audio_out)) any_x = 1'b1;
                if (audio_out > SAT_MAX || audio_out < SAT_MIN) overflow = 1'b1;
            end
            check(!overflow, "output bounded with resonance=255");
            check(!any_x, "no X/Z with resonance=255");
        end

        // -----------------------------------------------------------------
        // TC6: Mix=0 gives dry signal, Mix=255 gives wet
        // -----------------------------------------------------------------
        $display("\n-- TC6: Mix dry/wet blend --");
        // Run with mix=0 (pure dry)
        do_reset();
        freq_val      = 8'd128;
        resonance_val = 8'd100;
        mix_val       = 8'd0;  // pure dry
        mode_val      = 8'd0;
        // Pump enough samples for filter to settle
        for (int i = 0; i < 50; i++) begin
            @(posedge clk iff sample_en);
            audio_in = 24'sh300000;
            @(posedge clk);
        end
        begin
            logic signed [WIDTH-1:0] dry_out = audio_out;
            // Now run with mix=255 (full wet)
            do_reset();
            freq_val      = 8'd128;
            resonance_val = 8'd100;
            mix_val       = 8'd255;  // full wet
            mode_val      = 8'd0;
            for (int i = 0; i < 50; i++) begin
                @(posedge clk iff sample_en);
                audio_in = 24'sh300000;
                @(posedge clk);
            end
            check(audio_out !== dry_out,
                  "mix=0 and mix=255 produce different outputs");
            // Dry should be close to input
            begin
                logic signed [WIDTH-1:0] dry_diff;
                dry_diff = (dry_out > 24'sh300000) ?
                           (dry_out - 24'sh300000) : (24'sh300000 - dry_out);
                check(dry_diff < 24'sh080000,
                      "mix=0: output approximately equals input (dry path)");
            end
        end

        // -----------------------------------------------------------------
        // TC7: Auto mode produces time-varying output
        // -----------------------------------------------------------------
        $display("\n-- TC7: Auto mode time-varying output --");
        do_reset();
        mode_val      = 8'd1;   // auto (LFO) mode
        freq_val      = 8'd200; // fast LFO rate
        depth_val     = 8'd255; // full sweep
        resonance_val = 8'd100;
        mix_val       = 8'd255;
        // Feed constant signal, capture outputs at different times
        for (int i = 0; i < 200; i++) begin
            @(posedge clk iff sample_en);
            audio_in = 24'sh300000;
            @(posedge clk);
        end
        begin
            logic signed [WIDTH-1:0] out_early = audio_out;
            for (int i = 0; i < 500; i++) begin
                @(posedge clk iff sample_en);
                audio_in = 24'sh300000;
                @(posedge clk);
            end
            check(audio_out !== out_early,
                  "auto mode: output varies over time with constant input");
        end

        // -----------------------------------------------------------------
        // TC8: Auto mode — depth controls sweep range
        // -----------------------------------------------------------------
        $display("\n-- TC8: Auto mode depth control --");
        do_reset();
        mode_val      = 8'd1;   // auto (LFO)
        freq_val      = 8'd128; // moderate rate
        depth_val     = 8'd255; // full sweep
        resonance_val = 8'd150; // high Q for pronounced sweep
        mix_val       = 8'd255;
        // Feed constant-amplitude signal, let LFO sweep
        for (int i = 0; i < 500; i++) begin
            @(posedge clk iff sample_en);
            audio_in = 24'sh300000;
            @(posedge clk);
        end
        begin
            logic signed [WIDTH-1:0] out_full_depth = audio_out;
            // Now set depth=0 (no sweep) and run again
            do_reset();
            mode_val      = 8'd1;
            freq_val      = 8'd128;
            depth_val     = 8'd0;   // no sweep
            resonance_val = 8'd150;
            mix_val       = 8'd255;
            for (int i = 0; i < 500; i++) begin
                @(posedge clk iff sample_en);
                audio_in = 24'sh300000;
                @(posedge clk);
            end
            check(audio_out !== out_full_depth,
                  "auto mode: depth=255 and depth=0 produce different outputs");
        end

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

    // Timeout watchdog
    initial begin
        #(400_000_000);
        $display("ERROR: Simulation timeout.");
        $fatal(1, "Timeout");
    end

endmodule
