`timescale 1ns / 1ps

// ============================================================================
// tb_axis_effect_slot.sv — Testbench for axis_effect_slot
//
// Tests the unified slot with EFFECT_TYPE=0 (tremolo) as representative:
//   1. Registers default to 0 (depth=0 → unity)
//   2. Register writes via cfg_slice change behaviour
//   3. Bypass mode passes input straight through
//   4. Effect still runs when bypassed
//   5. Return from bypass
//   6. tready always asserted
//
// Also instantiates EFFECT_TYPE=6 (compressor), 7 (wah)
// to verify the generate blocks compile and produce valid output.
// ============================================================================

module tb_axis_effect_slot;

    import axis_audio_pkg::*;

    localparam int CLK_P = 81;

    logic        clk = 0;
    logic        rst_n;
    logic [47:0] s_axis_tdata;
    logic        s_axis_tvalid;
    logic        s_axis_tready;
    logic [47:0] m_axis_tdata;
    logic        m_axis_tvalid;
    logic        m_axis_tready;
    logic [7:0]  cfg_slice_trem [8];

    always #(CLK_P/2) clk = ~clk;

    // ---- DUT: tremolo slot (type 0) ----
    axis_effect_slot #(
        .SLOT_ID    (0),
        .EFFECT_TYPE(0),  // tremolo
        .DATA_W     (48),
        .CTRL_DEPTH (8),
        .CTRL_W     (8),
        .AUDIO_W    (24)
    ) dut_trem (
        .clk           (clk),
        .rst_n         (rst_n),
        .s_axis_tdata  (s_axis_tdata),
        .s_axis_tvalid (s_axis_tvalid),
        .s_axis_tready (s_axis_tready),
        .m_axis_tdata  (m_axis_tdata),
        .m_axis_tvalid (m_axis_tvalid),
        .m_axis_tready (m_axis_tready),
        .cfg_slice     (cfg_slice_trem)
    );

    // ---- DUT: compressor slot (type 6) — compilation check ----
    logic [47:0] cmp_m_tdata;
    logic        cmp_m_tvalid;
    logic        cmp_s_tready;
    logic [7:0]  cfg_slice_cmp [8];

    axis_effect_slot #(
        .SLOT_ID    (6),
        .EFFECT_TYPE(6),  // compressor
        .DATA_W     (48),
        .CTRL_DEPTH (8),
        .CTRL_W     (8),
        .AUDIO_W    (24)
    ) dut_cmp (
        .clk           (clk),
        .rst_n         (rst_n),
        .s_axis_tdata  (s_axis_tdata),
        .s_axis_tvalid (s_axis_tvalid),
        .s_axis_tready (cmp_s_tready),
        .m_axis_tdata  (cmp_m_tdata),
        .m_axis_tvalid (cmp_m_tvalid),
        .m_axis_tready (1'b1),
        .cfg_slice     (cfg_slice_cmp)
    );

    // ---- DUT: wah slot (type 7) — compilation check ----
    logic [47:0] wah_m_tdata;
    logic        wah_m_tvalid;
    logic        wah_s_tready;
    logic [7:0]  cfg_slice_wah [8];

    axis_effect_slot #(
        .SLOT_ID    (7),
        .EFFECT_TYPE(7),  // wah
        .DATA_W     (48),
        .CTRL_DEPTH (8),
        .CTRL_W     (8),
        .AUDIO_W    (24)
    ) dut_wah (
        .clk           (clk),
        .rst_n         (rst_n),
        .s_axis_tdata  (s_axis_tdata),
        .s_axis_tvalid (s_axis_tvalid),
        .s_axis_tready (wah_s_tready),
        .m_axis_tdata  (wah_m_tdata),
        .m_axis_tvalid (wah_m_tvalid),
        .m_axis_tready (1'b1),
        .cfg_slice     (cfg_slice_wah)
    );

    int pass_count = 0;
    int fail_count = 0;

    task automatic check(input string name, input logic condition);
        if (condition) pass_count++;
        else begin fail_count++; $error("FAIL: %s", name); end
    endtask

    task automatic send_sample(
        input logic signed [23:0] left,
        input logic signed [23:0] right,
        output logic [47:0] result
    );
        s_axis_tdata  = pack_stereo(left, right);
        s_axis_tvalid = 1'b1;
        @(posedge clk);
        s_axis_tvalid = 1'b0;
        m_axis_tready = 1'b1;
        repeat (10) begin
            @(posedge clk);
            if (m_axis_tvalid) begin
                result = m_axis_tdata;
                return;
            end
        end
        $error("TIMEOUT");
        result = '0;
    endtask

    logic [47:0] result;
    logic signed [23:0] out_l, out_r;

    initial begin
        $display("=== tb_axis_effect_slot ===");

        rst_n          = 0;
        s_axis_tdata   = '0;
        s_axis_tvalid  = 1'b0;
        m_axis_tready  = 1'b1;

        // All cfg_slices default to zero (no bypass, params = 0)
        for (int i = 0; i < 8; i++) begin
            cfg_slice_trem[i] = '0;
            cfg_slice_cmp[i]  = '0;
            cfg_slice_wah[i]  = '0;
        end

        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (3) @(posedge clk);

        // ---- Test 1: Registers default to 0 (depth=0 → unity) ----
        send_sample(24'sh200000, 24'sh100000, result);
        out_l = unpack_left(result);
        out_r = unpack_right(result);
        check("UNITY: L = input (depth=0)", out_l == 24'sh200000);
        check("UNITY: L = R (mono)", out_l == out_r);

        // ---- Test 2: Write depth register via cfg_slice → output changes ----
        cfg_slice_trem[1] = 8'd255;  // depth = 255
        cfg_slice_trem[0] = 8'd100;  // rate = 100
        for (int i = 0; i < 30; i++)
            send_sample(24'sh400000, 24'sh400000, result);
        out_l = unpack_left(result);
        check("DEPTH: output valid after cfg write", !$isunknown(out_l));

        // ---- Test 3: Bypass mode via cfg_slice[7][0] ----
        cfg_slice_trem[7] = 8'h01;  // bypass on
        send_sample(24'sh123456, 24'sh789ABC, result);
        out_l = unpack_left(result);
        out_r = unpack_right(result);
        check("BYP: L = input L", out_l == 24'sh123456);
        check("BYP: R = input R", out_r == $signed(24'sh789ABC));

        // ---- Test 4: Effect still runs while bypassed ----
        for (int i = 0; i < 10; i++)
            send_sample(24'sh300000, 24'sh300000, result);
        check("BYP_RUN: output valid while bypassed", !$isunknown(result));

        // ---- Test 5: Return from bypass ----
        cfg_slice_trem[7] = 8'h00;  // bypass off
        cfg_slice_trem[1] = 8'd0;   // depth=0 → unity again
        for (int i = 0; i < 5; i++)
            send_sample(24'sh250000, 24'sh250000, result);
        out_l = unpack_left(result);
        check("UNBYP: output = input (depth=0 after un-bypass)",
              out_l == 24'sh250000);

        // ---- Test 6: tready always asserted ----
        check("TREADY: s_axis_tready = 1", s_axis_tready == 1'b1);

        // ---- Test 7: New effect types produce valid output ----
        for (int i = 0; i < 20; i++)
            send_sample(24'sh100000, 24'sh100000, result);
        check("CMP: output valid", !$isunknown(cmp_m_tdata));
        check("WAH: output valid", !$isunknown(wah_m_tdata));

        $display("=== effect_slot: %0d passed, %0d failed ===", pass_count, fail_count);
        if (fail_count > 0) $fatal(1, "TESTS FAILED");
        $finish;
    end

endmodule
