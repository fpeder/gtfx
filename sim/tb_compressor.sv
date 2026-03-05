`timescale 1ns / 1ps

// =============================================================================
// Testbench: compressor
//
// clk_audio : 12.288 MHz  (half period ≈ 41 ns)
// sample_en : 1-cycle pulse every 256 clocks (48 kHz)
//
// Test cases
//   TC1  Reset clears output
//   TC2  Zero input → zero output
//   TC3  No X/Z propagation after reset
//   TC4  Below-threshold signal passes at unity gain
//   TC5  Above-threshold signal is reduced in level
//   TC6  Higher ratio = more compression
//   TC7  Makeup gain boosts output level
// =============================================================================

module tb_compressor;

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
    logic [7:0]              threshold_val;
    logic [7:0]              ratio_val;
    logic [7:0]              attack_val;
    logic [7:0]              release_val;
    logic [7:0]              makeup_val;

    compressor #(.DATA_W(WIDTH), .CTRL_W(8)) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .sample_en    (sample_en),
        .audio_in     (audio_in),
        .audio_out    (audio_out),
        .threshold_val(threshold_val),
        .ratio_val    (ratio_val),
        .attack_val   (attack_val),
        .release_val  (release_val),
        .makeup_val   (makeup_val)
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
        rst_n         = 1'b0;
        audio_in      = '0;
        threshold_val = 8'd128;
        ratio_val     = 8'd128;
        attack_val    = 8'd0;
        release_val   = 8'd0;
        makeup_val    = 8'd0;
        repeat (20) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        @(posedge clk iff sample_en);
    endtask

    logic [5:0] lut_idx;
    logic signed [WIDTH-1:0] out_val;

    initial begin
        $display("=== compressor testbench ===");

        // -----------------------------------------------------------------
        // TC1: Reset clears output
        // -----------------------------------------------------------------
        $display("\n-- TC1: Reset clears output --");
        rst_n    = 1'b0;
        audio_in = 24'sh3FFFFF;
        threshold_val = 8'd128;
        ratio_val     = 8'd128;
        attack_val    = 8'd0;
        release_val   = 8'd0;
        makeup_val    = 8'd0;
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
        // TC4: Below-threshold signal passes at unity gain
        // -----------------------------------------------------------------
        $display("\n-- TC4: Below threshold = unity gain --");
        do_reset();
        threshold_val = 8'd250;  // very high threshold
        ratio_val     = 8'd255;  // max compression
        attack_val    = 8'd0;
        release_val   = 8'd0;
        makeup_val    = 8'd0;    // no makeup
        // Feed moderate signal (well below threshold)
        for (int i = 0; i < 50; i++) begin
            @(posedge clk iff sample_en);
            audio_in = 24'sh100000;
            @(posedge clk);
        end
        // Output should be close to input (unity with 1.0x makeup)
        begin
            logic signed [WIDTH-1:0] abs_diff;
            abs_diff = (audio_out > audio_in) ?
                       (audio_out - audio_in) : (audio_in - audio_out);
            check(abs_diff < 24'sh040000,
                  "below threshold: output ≈ input (within tolerance)");
        end

        // -----------------------------------------------------------------
        // TC5: Above-threshold signal is reduced in level
        // -----------------------------------------------------------------
        $display("\n-- TC5: Above threshold = level reduced --");
        do_reset();
        threshold_val = 8'd20;   // low threshold
        ratio_val     = 8'd255;  // max compression ratio
        attack_val    = 8'd0;    // instant attack
        release_val   = 8'd0;
        makeup_val    = 8'd0;
        // Feed loud signal
        for (int i = 0; i < 100; i++) begin
            @(posedge clk iff sample_en);
            audio_in = 24'sh600000;
            @(posedge clk);
        end
        begin
            logic signed [WIDTH-1:0] abs_out;
            abs_out = audio_out[WIDTH-1] ? -audio_out : audio_out;
            check(abs_out < 24'sh600000,
                  "above threshold: output < input (compressed)");
            check(abs_out > 24'sh000000,
                  "above threshold: output > 0 (not silenced)");
        end

        // -----------------------------------------------------------------
        // TC6: Higher ratio = more compression
        // -----------------------------------------------------------------
        $display("\n-- TC6: Higher ratio = more compression --");
        // Run A: low ratio
        do_reset();
        threshold_val = 8'd30;
        ratio_val     = 8'd50;   // mild compression
        attack_val    = 8'd0;
        release_val   = 8'd0;
        makeup_val    = 8'd0;
        for (int i = 0; i < 100; i++) begin
            @(posedge clk iff sample_en);
            audio_in = 24'sh600000;
            @(posedge clk);
        end
        begin
            logic signed [WIDTH-1:0] out_low_ratio = audio_out;
            // Run B: high ratio
            do_reset();
            threshold_val = 8'd30;
            ratio_val     = 8'd250;  // heavy compression
            attack_val    = 8'd0;
            release_val   = 8'd0;
            makeup_val    = 8'd0;
            for (int i = 0; i < 100; i++) begin
                @(posedge clk iff sample_en);
                audio_in = 24'sh600000;
                @(posedge clk);
            end
            begin
                logic signed [WIDTH-1:0] abs_low  = out_low_ratio[WIDTH-1] ? -out_low_ratio : out_low_ratio;
                logic signed [WIDTH-1:0] abs_high = audio_out[WIDTH-1] ? -audio_out : audio_out;
                check(abs_high <= abs_low,
                      "higher ratio produces equal or lower output level");
            end
        end

        // -----------------------------------------------------------------
        // TC7: Makeup gain boosts output level
        // -----------------------------------------------------------------
        $display("\n-- TC7: Makeup gain boosts output --");
        // Run A: no makeup
        do_reset();
        threshold_val = 8'd30;
        ratio_val     = 8'd200;
        attack_val    = 8'd0;
        release_val   = 8'd0;
        makeup_val    = 8'd0;
        for (int i = 0; i < 100; i++) begin
            @(posedge clk iff sample_en);
            audio_in = 24'sh400000;
            @(posedge clk);
        end
        begin
            logic signed [WIDTH-1:0] out_no_makeup = audio_out;
            // Run B: with makeup
            do_reset();
            threshold_val = 8'd30;
            ratio_val     = 8'd200;
            attack_val    = 8'd0;
            release_val   = 8'd0;
            makeup_val    = 8'd200;  // substantial makeup gain
            for (int i = 0; i < 100; i++) begin
                @(posedge clk iff sample_en);
                audio_in = 24'sh400000;
                @(posedge clk);
            end
            begin
                logic signed [WIDTH-1:0] abs_no  = out_no_makeup[WIDTH-1] ? -out_no_makeup : out_no_makeup;
                logic signed [WIDTH-1:0] abs_yes = audio_out[WIDTH-1] ? -audio_out : audio_out;
                check(abs_yes > abs_no,
                      "makeup gain increases output level");
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
        #(200_000_000);
        $display("ERROR: Simulation timeout.");
        $fatal(1, "Timeout");
    end

endmodule
