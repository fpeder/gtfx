`timescale 1ns / 1ps

// =============================================================================
// Testbench: reverb (Freeverb / Schroeder-Moorer)
//
// clk_audio : 12.288 MHz  (half period ≈ 41 ns)
// sample_en : 1-cycle pulse every 256 clocks (48 kHz)
//
// Test cases
//   TC1  Reset clears output
//   TC2  Zero input → zero output
//   TC3  No X/Z propagation after reset
//   TC4  Impulse produces decaying tail
//   TC5  Mix=0 → dry, Mix=255 → wet
//   TC6  Decay parameter affects tail length
// =============================================================================

module tb_reverb;

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
    logic signed [WIDTH-1:0] audio_out_l;
    logic signed [WIDTH-1:0] audio_out_r;
    logic                    valid_out;
    logic [7:0]              decay;
    logic [7:0]              damping;
    logic [7:0]              mix;
    logic [7:0]              pre_dly;
    logic [7:0]              tone;
    logic [7:0]              level;

    reverb #(.DATA_W(WIDTH), .CTRL_W(8)) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .sample_en  (sample_en),
        .audio_in   (audio_in),
        .audio_out_l(audio_out_l),
        .audio_out_r(audio_out_r),
        .valid_out  (valid_out),
        .decay      (decay),
        .damping    (damping),
        .mix        (mix),
        .pre_dly    (pre_dly),
        .tone       (tone),
        .level      (level)
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
        rst_n   = 1'b0;
        audio_in = '0;
        decay   = 8'd128;
        damping = 8'd96;
        mix     = 8'd96;
        pre_dly = 8'd20;
        tone    = 8'd128;
        level   = 8'd128;
        repeat (20) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        @(posedge clk iff sample_en);
    endtask

    logic [5:0] lut_idx;

    initial begin
        $display("=== reverb testbench ===");

        // -----------------------------------------------------------------
        // TC1: Reset clears output
        // -----------------------------------------------------------------
        $display("\n-- TC1: Reset clears output --");
        rst_n    = 1'b0;
        audio_in = 24'sh3FFFFF;
        decay   = 8'd200;
        damping = 8'd128;
        mix     = 8'd200;
        pre_dly = 8'd0;
        tone    = 8'd128;
        level   = 8'd200;
        repeat (10) @(posedge clk);
        check(audio_out_l === 24'sh0 && audio_out_r === 24'sh0,
              "audio_out_l/r = 0 during reset");

        // -----------------------------------------------------------------
        // TC2: Zero input → zero output
        // -----------------------------------------------------------------
        $display("\n-- TC2: Zero input silence --");
        do_reset();
        mix   = 8'd0;  // dry only
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
        mix = 8'd128;
        lut_idx = '0;
        begin
            logic any_x = 1'b0;
            for (int i = 0; i < 128; i++) begin
                @(posedge clk iff sample_en);
                audio_in = SINE_LUT[lut_idx];
                lut_idx  = lut_idx + 1'b1;
                @(posedge clk);
                if ($isunknown(audio_out_l) || $isunknown(audio_out_r))
                    any_x = 1'b1;
            end
            check(!any_x, "audio_out_l/r never X/Z over 128 samples");
        end

        // -----------------------------------------------------------------
        // TC4: Impulse produces decaying tail
        // -----------------------------------------------------------------
        $display("\n-- TC4: Impulse → decaying tail --");
        do_reset();
        decay   = 8'd200;
        damping = 8'd64;
        mix     = 8'd200;
        pre_dly = 8'd0;
        level   = 8'd200;
        // Send one loud sample then silence
        @(posedge clk iff sample_en);
        audio_in = 24'sh600000;
        @(posedge clk);
        begin
            logic any_output_after = 1'b0;
            // Run 2000 samples of silence, check for reverb tail
            for (int i = 0; i < 2000; i++) begin
                @(posedge clk iff sample_en);
                audio_in = 24'sh0;
                @(posedge clk);
                if (audio_out_l !== 24'sh0 || audio_out_r !== 24'sh0)
                    any_output_after = 1'b1;
            end
            check(any_output_after,
                  "impulse produces non-zero output during reverb tail");
        end

        // -----------------------------------------------------------------
        // TC5: Mix=0 → dry, Mix=255 → wet
        // -----------------------------------------------------------------
        $display("\n-- TC5: Mix dry/wet --");
        // Run A: mix=0 (dry)
        do_reset();
        mix   = 8'd0;
        level = 8'd200;
        for (int i = 0; i < 50; i++) begin
            @(posedge clk iff sample_en);
            audio_in = 24'sh300000;
            @(posedge clk);
        end
        begin
            logic signed [WIDTH-1:0] dry_l = audio_out_l;
            // Run B: mix=255 (wet)
            do_reset();
            mix   = 8'd255;
            level = 8'd200;
            for (int i = 0; i < 50; i++) begin
                @(posedge clk iff sample_en);
                audio_in = 24'sh300000;
                @(posedge clk);
            end
            check(audio_out_l !== dry_l,
                  "mix=0 and mix=255 produce different L outputs");
        end

        // -----------------------------------------------------------------
        // TC6: Decay parameter affects tail length
        // -----------------------------------------------------------------
        $display("\n-- TC6: Decay affects tail --");
        // Run A: short decay
        do_reset();
        decay   = 8'd20;
        mix     = 8'd200;
        level   = 8'd200;
        // Send impulse
        @(posedge clk iff sample_en);
        audio_in = 24'sh600000;
        @(posedge clk);
        // Run 1500 silence samples, count non-zero outputs
        begin
            int active_short = 0;
            for (int i = 0; i < 1500; i++) begin
                @(posedge clk iff sample_en);
                audio_in = 24'sh0;
                @(posedge clk);
                if (audio_out_l !== 24'sh0) active_short++;
            end

            // Run B: long decay
            do_reset();
            decay   = 8'd240;
            mix     = 8'd200;
            level   = 8'd200;
            @(posedge clk iff sample_en);
            audio_in = 24'sh600000;
            @(posedge clk);
            begin
                int active_long = 0;
                for (int i = 0; i < 1500; i++) begin
                    @(posedge clk iff sample_en);
                    audio_in = 24'sh0;
                    @(posedge clk);
                    if (audio_out_l !== 24'sh0) active_long++;
                end
                check(active_long >= active_short,
                      "longer decay produces equal or more non-zero tail samples");
            end
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
        #(500_000_000);  // 500ms - reverb needs longer for tail tests
        $display("ERROR: Simulation timeout.");
        $fatal(1, "Timeout");
    end

endmodule
