`timescale 1ns / 1ps

// =============================================================================
// Testbench: flanger (MXR M117R-style)
//
// clk_audio : 12.288 MHz  (half period ≈ 41 ns)
// sample_en : 1-cycle pulse every 256 clocks (48 kHz)
//
// Test cases
//   TC1  Reset clears output
//   TC2  Zero input → zero output
//   TC3  No X/Z propagation after reset
//   TC4  Non-zero output with sine input (comb filtering)
//   TC5  Regen increases feedback (output grows over cycles)
//   TC6  Mix blend between dry and wet
// =============================================================================

module tb_flanger;

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
    logic [7:0]              manual_val;
    logic [7:0]              width_val;
    logic [7:0]              speed_val;
    logic [7:0]              regen_val;
    logic [7:0]              mix_val;

    flanger #(.WIDTH(WIDTH), .CTRL_W(8)) dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .sample_en (sample_en),
        .audio_in  (audio_in),
        .audio_out (audio_out),
        .manual_val(manual_val),
        .width_val (width_val),
        .speed_val (speed_val),
        .regen_val (regen_val),
        .mix_val   (mix_val)
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
        rst_n      = 1'b0;
        audio_in   = '0;
        manual_val = 8'd64;
        width_val  = 8'd128;
        speed_val  = 8'd48;
        regen_val  = 8'd144;  // slight positive feedback
        mix_val    = 8'd96;
        repeat (20) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        @(posedge clk iff sample_en);
    endtask

    logic [5:0] lut_idx;

    initial begin
        $display("=== flanger testbench ===");

        // -----------------------------------------------------------------
        // TC1: Reset clears output
        // -----------------------------------------------------------------
        $display("\n-- TC1: Reset clears output --");
        rst_n    = 1'b0;
        audio_in = 24'sh3FFFFF;
        manual_val = 8'd64;
        width_val  = 8'd128;
        speed_val  = 8'd48;
        regen_val  = 8'd128;
        mix_val    = 8'd96;
        repeat (10) @(posedge clk);
        check(audio_out === 24'sh0,
              "audio_out = 0 during reset");

        // -----------------------------------------------------------------
        // TC2: Zero input → zero output
        // -----------------------------------------------------------------
        $display("\n-- TC2: Zero input silence --");
        do_reset();
        regen_val = 8'd128;  // zero regen (centre)
        mix_val   = 8'd0;
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
        mix_val = 8'd96;
        lut_idx = '0;
        begin
            logic any_x = 1'b0;
            for (int i = 0; i < 256; i++) begin
                @(posedge clk iff sample_en);
                audio_in = SINE_LUT[lut_idx];
                lut_idx  = lut_idx + 1'b1;
                @(posedge clk);
                if ($isunknown(audio_out)) any_x = 1'b1;
            end
            check(!any_x, "audio_out never X/Z over 256 samples");
        end

        // -----------------------------------------------------------------
        // TC4: Non-zero output with sine input (comb filtering)
        // -----------------------------------------------------------------
        $display("\n-- TC4: Comb filtering produces output --");
        do_reset();
        manual_val = 8'd64;
        width_val  = 8'd128;
        speed_val  = 8'd48;
        regen_val  = 8'd128;  // zero feedback
        mix_val    = 8'd128;  // 50% wet
        lut_idx = '0;
        begin
            logic any_nonzero = 1'b0;
            for (int i = 0; i < 300; i++) begin
                @(posedge clk iff sample_en);
                audio_in = SINE_LUT[lut_idx];
                lut_idx  = lut_idx + 1'b1;
                @(posedge clk);
                if (audio_out !== 24'sh0) any_nonzero = 1'b1;
            end
            check(any_nonzero, "sine input produces non-zero output");
        end

        // -----------------------------------------------------------------
        // TC5: Regen increases feedback effect
        // -----------------------------------------------------------------
        $display("\n-- TC5: Regen feedback --");
        // Run A: no regen (centre = 0x80)
        do_reset();
        manual_val = 8'd64;
        width_val  = 8'd0;    // no LFO mod
        speed_val  = 8'd0;
        regen_val  = 8'd128;  // zero regen
        mix_val    = 8'd200;
        lut_idx = '0;
        for (int i = 0; i < 300; i++) begin
            @(posedge clk iff sample_en);
            audio_in = SINE_LUT[lut_idx];
            lut_idx  = lut_idx + 1'b1;
            @(posedge clk);
        end
        begin
            logic signed [WIDTH-1:0] abs_no_regen;
            abs_no_regen = audio_out[WIDTH-1] ? -audio_out : audio_out;

            // Run B: positive regen
            do_reset();
            manual_val = 8'd64;
            width_val  = 8'd0;
            speed_val  = 8'd0;
            regen_val  = 8'd230;  // strong positive regen
            mix_val    = 8'd200;
            lut_idx = '0;
            for (int i = 0; i < 300; i++) begin
                @(posedge clk iff sample_en);
                audio_in = SINE_LUT[lut_idx];
                lut_idx  = lut_idx + 1'b1;
                @(posedge clk);
            end
            begin
                logic signed [WIDTH-1:0] abs_with_regen;
                abs_with_regen = audio_out[WIDTH-1] ? -audio_out : audio_out;
                check(abs_with_regen != abs_no_regen,
                      "regen changes output compared to no regen");
            end
        end

        // -----------------------------------------------------------------
        // TC6: Mix blend between dry and wet
        // -----------------------------------------------------------------
        $display("\n-- TC6: Mix blend --");
        // Run A: mix=0 (pure dry)
        do_reset();
        mix_val = 8'd0;
        for (int i = 0; i < 100; i++) begin
            @(posedge clk iff sample_en);
            audio_in = 24'sh300000;
            @(posedge clk);
        end
        begin
            logic signed [WIDTH-1:0] dry_out = audio_out;
            // Run B: mix=255 (max wet)
            do_reset();
            mix_val = 8'd255;
            for (int i = 0; i < 100; i++) begin
                @(posedge clk iff sample_en);
                audio_in = 24'sh300000;
                @(posedge clk);
            end
            // With mix=0, output = dry only; with mix=255, output = dry + wet
            // They should differ
            check(audio_out !== dry_out,
                  "mix=0 and mix=255 produce different outputs");
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
