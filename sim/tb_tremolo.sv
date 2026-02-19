`timescale 1ns / 1ps

// =============================================================================
// tb_tremolo.sv  -  Testbench for tremolo.sv
//
// Tests:
//   1. RESET          - output stays zero while rst_n is asserted
//   2. BYPASS         - depth=0 means output == input (within 1 LSB rounding)
//   3. FULL_DEPTH_SINE    - depth=255, shape=sine:  output peak ≈ input,
//                           output trough ≈ 0  (verified over one LFO cycle)
//   4. FULL_DEPTH_TRI - depth=255, shape=triangle: same envelope test
//   5. GAIN_MONOTONIC - at mid-depth (128), output envelope is always ≤ input
//   6. RATE_CHANGE    - switching rate_val mid-stream does not cause glitches
//                       (no sample exceeds ±full-scale)
//   7. SATURATION     - full-scale input with gain=1.0 never overflows
//
// Clock:  12.288 MHz  (period = ~81.38 ns)
// sample_en period: 1/48000 s = 20833 ns → every 256 clk cycles
//
// Envelope tracking: the testbench computes a simple peak detector over a
// sliding window of WINDOW_SAMPLES samples to find the modulation envelope.
// =============================================================================

module tb_tremolo;

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    localparam int    WIDTH          = 24;
    localparam int    CLK_PERIOD_NS  = 82;          // ~12.195 MHz (close enough)
    localparam int    FS_PERIOD_NS   = 20833;        // 48 kHz sample period
    localparam int    CLK_PER_SAMPLE = FS_PERIOD_NS / CLK_PERIOD_NS; // ~254

    localparam signed [WIDTH-1:0] FULL_SCALE_POS =  24'sh7FFFFF;
    localparam signed [WIDTH-1:0] FULL_SCALE_NEG = -24'sh800000;
    localparam signed [WIDTH-1:0] MID_SCALE_POS  =  24'sh400000;  // ~0.5 FS

    // One LFO cycle at rate=128 (~6 Hz):
    //   inc = 44739 + 128*4035 = 44739 + 516480 = 561219
    //   cycles per LFO period = 2^32 / 561219 ≈ 7654 samples
    localparam int RATE_MID         = 128;
    localparam int SAMPLES_PER_LFO  = 7700;   // slightly over, safe margin
    localparam int WINDOW_SAMPLES   = 32;      // peak-detector window

    // -------------------------------------------------------------------------
    // DUT signals
    // -------------------------------------------------------------------------
    logic                    clk        = 0;
    logic                    rst_n      = 0;
    logic                    sample_en  = 0;
    logic [7:0]              rate_val   = RATE_MID;
    logic [7:0]              depth_val  = 8'd0;
    logic                    shape_sel  = 0;
    logic signed [WIDTH-1:0] audio_in   = '0;
    logic signed [WIDTH-1:0] audio_out;

    // -------------------------------------------------------------------------
    // DUT instantiation
    // -------------------------------------------------------------------------
    tremolo #(
        .WIDTH(WIDTH)
    ) dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .sample_en(sample_en),
        .rate_val (rate_val),
        .depth_val(depth_val),
        .shape_sel(shape_sel),
        .audio_in (audio_in),
        .audio_out(audio_out)
    );

    // -------------------------------------------------------------------------
    // Clock generator
    // -------------------------------------------------------------------------
    always #(CLK_PERIOD_NS / 2) clk = ~clk;

    // -------------------------------------------------------------------------
    // sample_en generator (one cycle pulse every CLK_PER_SAMPLE clocks)
    // -------------------------------------------------------------------------
    int sample_clk_cnt = 0;

    always_ff @(posedge clk) begin
        if (sample_clk_cnt == CLK_PER_SAMPLE - 1) begin
            sample_en       <= 1;
            sample_clk_cnt  <= 0;
        end else begin
            sample_en      <= 0;
            sample_clk_cnt <= sample_clk_cnt + 1;
        end
    end

    // -------------------------------------------------------------------------
    // Task: wait for N sample_en pulses
    // -------------------------------------------------------------------------
    task automatic wait_samples(input int n);
        for (int i = 0; i < n; i++) begin
            @(posedge clk iff sample_en);
            @(posedge clk);          // wait one extra cycle for output reg update
        end
    endtask

    // -------------------------------------------------------------------------
    // Task: collect N output samples into an array and return peak & trough
    // -------------------------------------------------------------------------
    task automatic collect_envelope(
        input  int n,
        output longint peak,
        output longint trough
    );
        longint s;
        peak   = longint'(FULL_SCALE_NEG);
        trough = longint'(FULL_SCALE_POS);
        for (int i = 0; i < n; i++) begin
            @(posedge clk iff sample_en);
            @(posedge clk);
            s = longint'($signed(audio_out));
            if (s > peak)   peak   = s;
            if (s < trough) trough = s;
        end
    endtask

    // -------------------------------------------------------------------------
    // Pass/fail counters
    // -------------------------------------------------------------------------
    int pass_cnt = 0;
    int fail_cnt = 0;

    task automatic check(input string name, input logic cond);
        if (cond) begin
            $display("  PASS  %s", name);
            pass_cnt++;
        end else begin
            $display("  FAIL  %s", name);
            fail_cnt++;
        end
    endtask

    // -------------------------------------------------------------------------
    // Stimulus & checks
    // -------------------------------------------------------------------------
    initial begin
        $dumpfile("tb_tremolo.vcd");
        $dumpvars(0, tb_tremolo);

        $display("=================================================");
        $display(" tb_tremolo  -  tremolo.sv testbench");
        $display("=================================================");

        // =====================================================================
        // 1. RESET CHECK
        // =====================================================================
        $display("\n[1] Reset check");
        rst_n     = 0;
        depth_val = 8'd255;
        audio_in  = MID_SCALE_POS;
        repeat (CLK_PER_SAMPLE * 4) @(posedge clk);

        check("output == 0 during reset", audio_out == '0);

        // Release reset
        @(posedge clk);
        rst_n = 1;
        wait_samples(2);

        // =====================================================================
        // 2. BYPASS (depth = 0)
        // =====================================================================
        $display("\n[2] Bypass check (depth=0)");
        depth_val = 8'd0;
        audio_in  = MID_SCALE_POS;
        wait_samples(4);

        // Allow 1 LSB tolerance for Q16 rounding (depth=0 gives gain=0x10000 = 1.0 exactly)
        check("bypass: out ≈ in (pos)", $signed(audio_out) >= ($signed(MID_SCALE_POS) - 1)
                                     && $signed(audio_out) <= ($signed(MID_SCALE_POS) + 1));

        audio_in = FULL_SCALE_NEG;
        wait_samples(4);
        check("bypass: out ≈ in (neg)", $signed(audio_out) >= ($signed(FULL_SCALE_NEG) - 1)
                                     && $signed(audio_out) <= ($signed(FULL_SCALE_NEG) + 1));

        audio_in = '0;
        wait_samples(4);
        check("bypass: out == 0 for zero input", audio_out == '0);

        // =====================================================================
        // 3. FULL DEPTH - SINE LFO
        //    Drive a constant positive input; over one full LFO cycle the
        //    envelope must reach close to input (peak) and close to 0 (trough).
        // =====================================================================
        $display("\n[3] Full depth - sine LFO");
        depth_val = 8'd255;
        shape_sel = 0;            // sine
        audio_in  = MID_SCALE_POS;
        rate_val  = RATE_MID;

        begin
            longint peak, trough;
            // Wait for LFO to settle from reset position, then measure 1.5 cycles
            wait_samples(SAMPLES_PER_LFO / 4);
            collect_envelope(SAMPLES_PER_LFO + SAMPLES_PER_LFO / 2, peak, trough);

            // Peak should be within 1% of input amplitude
            check("sine: peak >= 99% of input",
                  peak >= (longint'($signed(MID_SCALE_POS)) * 99 / 100));

            // Trough should be close to 0 (within 2% of full scale)
            check("sine: trough ~= 0 (< 2% FS)",
                  trough <= (longint'(FULL_SCALE_POS) * 2 / 100));

            // Output must never exceed input (gain_q16 <= 0xFFFF < 1.0, so out <= in-1 LSB)
            // Allow 2 LSB tolerance for rounding in the Q16 multiply.
            check("sine: no gain boost (out <= in)",
                  peak <= longint'($signed(MID_SCALE_POS)) + 2);
        end

        // =====================================================================
        // 4. FULL DEPTH - TRIANGLE LFO
        // =====================================================================
        $display("\n[4] Full depth - triangle LFO");
        shape_sel = 1;            // triangle
        audio_in  = MID_SCALE_POS;

        begin
            longint peak, trough;
            wait_samples(SAMPLES_PER_LFO / 4);
            collect_envelope(SAMPLES_PER_LFO + SAMPLES_PER_LFO / 2, peak, trough);

            check("tri: peak >= 99% of input",
                  peak >= (longint'($signed(MID_SCALE_POS)) * 99 / 100));

            check("tri: trough ~= 0 (< 2% FS)",
                  trough <= (longint'(FULL_SCALE_POS) * 2 / 100));

            check("tri: no gain boost (out <= in)",
                  peak <= longint'($signed(MID_SCALE_POS)) + 2);
        end

        // =====================================================================
        // 5. GAIN MONOTONIC - MID DEPTH
        //    At depth=128 the output should always be <= |input|.
        // =====================================================================
        $display("\n[5] Gain <= 1 at mid depth (depth=128)");
        depth_val = 8'd128;
        shape_sel = 0;
        audio_in  = MID_SCALE_POS;

        begin
            longint peak, trough;
            wait_samples(SAMPLES_PER_LFO / 4);
            collect_envelope(SAMPLES_PER_LFO * 2, peak, trough);

            check("mid-depth: peak <= input",
                  peak <= longint'($signed(MID_SCALE_POS)) + 2);

            check("mid-depth: trough > 0 (partial depth)",
                  trough > 0);

            check("mid-depth: trough < 50% FS (modulated)",
                  trough < longint'($signed(MID_SCALE_POS)));
        end

        // =====================================================================
        // 6. RATE CHANGE - no output glitch
        //    Switch rate mid-stream and verify no sample exceeds full-scale.
        // =====================================================================
        $display("\n[6] Rate change - no overflow glitch");
        depth_val = 8'd200;
        shape_sel = 0;
        audio_in  = MID_SCALE_POS;
        rate_val  = 8'd20;

        begin
            logic glitch_detected;
            glitch_detected = 0;
            wait_samples(200);

            // Switch to fast rate abruptly
            rate_val = 8'd240;

            for (int i = 0; i < 500; i++) begin
                @(posedge clk iff sample_en);
                @(posedge clk);
                // With positive input and gain in [0,1): output must be in [0, input]
                // Allow 2 LSB rounding slack.
                if ($signed(audio_out) > $signed(MID_SCALE_POS) + 2 ||
                    $signed(audio_out) < $signed(24'sh000000))
                    glitch_detected = 1;
            end

            check("rate change: no overflow glitch", !glitch_detected);
        end

        // =====================================================================
        // 7. SATURATION - full-scale input at depth=0 (gain=1.0)
        //    Output must not wrap or overflow.
        // =====================================================================
        $display("\n[7] Saturation / full-scale passthrough");
        depth_val = 8'd0;
        audio_in  = FULL_SCALE_POS;
        wait_samples(4);
        check("sat: pos FS in, depth=0 -> pos FS out",
              $signed(audio_out) == $signed(FULL_SCALE_POS));

        audio_in = FULL_SCALE_NEG;
        wait_samples(4);
        check("sat: neg FS in, depth=0 -> neg FS out",
              $signed(audio_out) == $signed(FULL_SCALE_NEG));

        // Full depth + full scale: gain oscillates [~0, 1.0]; output stays in [0, FS_POS]
        // At maximum attenuation gain=0x00001 (1/65536), so trough is ~64 counts, not 0.
        depth_val = 8'd255;
        shape_sel = 0;
        audio_in  = FULL_SCALE_POS;

        begin
            logic overflow;
            overflow = 0;
            for (int i = 0; i < SAMPLES_PER_LFO * 2; i++) begin
                @(posedge clk iff sample_en);
                @(posedge clk);
                // Output must be in [0, FS_POS]; never negative, never above input
                if ($signed(audio_out) > $signed(FULL_SCALE_POS) ||
                    $signed(audio_out) < $signed(24'sh000000))
                    overflow = 1;
            end
            check("sat: full-depth + FS input never overflows or goes negative", !overflow);
        end

        // =====================================================================
        // 8. ZERO INPUT
        //    Any depth / shape / rate: zero in must give zero out.
        //    Flush for 4 samples first to clear any residual from test 7.
        // =====================================================================
        $display("\n[8] Zero input -> zero output for all depths");
        audio_in = '0;
        wait_samples(4);   // flush residual from previous test

        for (int d = 0; d <= 255; d += 51) begin
            depth_val = d[7:0];
            wait_samples(4);
            // Sample several times to confirm it stays at zero
            begin
                logic nonzero_seen;
                nonzero_seen = 0;
                for (int s = 0; s < 4; s++) begin
                    @(posedge clk iff sample_en);
                    @(posedge clk);
                    if (audio_out != '0) nonzero_seen = 1;
                end
                check($sformatf("zero_in: depth=%0d -> out=0", d), !nonzero_seen);
            end
        end

        // =====================================================================
        // Summary
        // =====================================================================
        $display("\n=================================================");
        $display(" Results:  %0d passed,  %0d failed", pass_cnt, fail_cnt);
        $display("=================================================");

        if (fail_cnt == 0)
            $display(" ALL TESTS PASSED");
        else
            $display(" SOME TESTS FAILED - review output above");

        $finish;
    end

    // -------------------------------------------------------------------------
    // Timeout watchdog
    // -------------------------------------------------------------------------
    initial begin
        #(CLK_PERIOD_NS * CLK_PER_SAMPLE * 100_000);
        $display("TIMEOUT - simulation took too long");
        $finish;
    end

endmodule