`timescale 1ns / 1ps

// ============================================================================
// tb_crossbar_routing_examples.sv
//
// Demonstrates practical routing configurations through the crossbar.
// Uses a 6-port crossbar (4 slots + ADC + DAC) with labeled masters/slaves.
//
// Port map:
//   Master 0 = ADC                      Slave 0 = Slot 0 input (tremolo)
//   Master 1 = Slot 0 output (tremolo)  Slave 1 = Slot 1 input (phaser)
//   Master 2 = Slot 1 output (phaser)   Slave 2 = Slot 2 input (chorus)
//   Master 3 = Slot 2 output (chorus)   Slave 3 = Slot 3 input (delay)
//   Master 4 = Slot 3 output (delay)    Slave 4 = (unused / future)
//   Master 5 = (none)                   Slave 5 = DAC input
//
// route[k] selects which master feeds slave port k.
//
// Each test:
//   1. Sets the route configuration
//   2. Drives distinct data on each master
//   3. Verifies each slave receives the correct master's data
// ============================================================================

module tb_crossbar_routing_examples;

    localparam int NP    = 6;
    localparam int DW    = 48;
    localparam int SEL_W = $clog2(NP);

    logic [SEL_W-1:0] route [NP];
    logic [DW-1:0]    m_tdata  [NP];
    logic              m_tvalid [NP];
    logic              m_tready [NP];
    logic [DW-1:0]    s_tdata  [NP];
    logic              s_tvalid [NP];
    logic              s_tready [NP];

    axis_crossbar #(.N_PORTS(NP), .DATA_W(DW), .SEL_W(SEL_W)) xbar (.*);

    int pass_count = 0;
    int fail_count = 0;

    task automatic check(input string name, input logic condition);
        if (condition) pass_count++;
        else begin fail_count++; $error("FAIL: %s", name); end
    endtask

    // Label data so we can trace where it came from
    //   ADC=0xADC, slot0=0x100, slot1=0x200, slot2=0x300, slot3=0x400
    localparam logic [DW-1:0] ADC_DATA   = 48'h00_0ADC_00_0ADC;
    localparam logic [DW-1:0] SLOT0_DATA = 48'h00_0100_00_0100;
    localparam logic [DW-1:0] SLOT1_DATA = 48'h00_0200_00_0200;
    localparam logic [DW-1:0] SLOT2_DATA = 48'h00_0300_00_0300;
    localparam logic [DW-1:0] SLOT3_DATA = 48'h00_0400_00_0400;
    localparam logic [DW-1:0] NONE_DATA  = 48'h00_0000_00_0000;

    task automatic load_masters();
        m_tdata[0] = ADC_DATA;     m_tvalid[0] = 1'b1;
        m_tdata[1] = SLOT0_DATA;   m_tvalid[1] = 1'b1;
        m_tdata[2] = SLOT1_DATA;   m_tvalid[2] = 1'b1;
        m_tdata[3] = SLOT2_DATA;   m_tvalid[3] = 1'b1;
        m_tdata[4] = SLOT3_DATA;   m_tvalid[4] = 1'b1;
        m_tdata[5] = NONE_DATA;    m_tvalid[5] = 1'b0;
        for (int i = 0; i < NP; i++) s_tready[i] = 1'b1;
    endtask

    initial begin
        $display("");
        $display("==========================================================");
        $display("  Crossbar Routing Examples");
        $display("==========================================================");

        load_masters();

        // ==================================================================
        // EXAMPLE 1: Default Linear Chain
        //
        //   ADC → Tremolo → Phaser → Chorus → Delay → DAC
        //
        //   route[0]=0  slot0 ← ADC
        //   route[1]=1  slot1 ← slot0
        //   route[2]=2  slot2 ← slot1
        //   route[3]=3  slot3 ← slot2
        //   route[5]=4  DAC   ← slot3
        // ==================================================================
        $display("");
        $display("--- Example 1: Default Linear Chain ---");
        $display("    ADC -> Tremolo -> Phaser -> Chorus -> Delay -> DAC");

        route[0] = 3'd0;  // slot0(trem) ← ADC
        route[1] = 3'd1;  // slot1(pha)  ← slot0
        route[2] = 3'd2;  // slot2(cho)  ← slot1
        route[3] = 3'd3;  // slot3(dly)  ← slot2
        route[4] = 3'd0;  // (unused sink)
        route[5] = 3'd4;  // DAC ← slot3
        #1;

        check("EX1: slot0 input ← ADC",    s_tdata[0] == ADC_DATA);
        check("EX1: slot1 input ← slot0",  s_tdata[1] == SLOT0_DATA);
        check("EX1: slot2 input ← slot1",  s_tdata[2] == SLOT1_DATA);
        check("EX1: slot3 input ← slot2",  s_tdata[3] == SLOT2_DATA);
        check("EX1: DAC   input ← slot3",  s_tdata[5] == SLOT3_DATA);

        // ==================================================================
        // EXAMPLE 2: Reversed Order
        //
        //   ADC → Delay → Chorus → Phaser → Tremolo → DAC
        //
        //   route[3]=0  slot3(dly) ← ADC
        //   route[2]=4  slot2(cho) ← slot3
        //   route[1]=3  slot1(pha) ← slot2
        //   route[0]=2  slot0(trm) ← slot1
        //   route[5]=1  DAC        ← slot0
        // ==================================================================
        $display("");
        $display("--- Example 2: Reversed Order ---");
        $display("    ADC -> Delay -> Chorus -> Phaser -> Tremolo -> DAC");

        route[3] = 3'd0;  // slot3(dly) ← ADC
        route[2] = 3'd4;  // slot2(cho) ← slot3
        route[1] = 3'd3;  // slot1(pha) ← slot2
        route[0] = 3'd2;  // slot0(trm) ← slot1
        route[5] = 3'd1;  // DAC ← slot0
        route[4] = 3'd0;
        #1;

        check("EX2: slot3(dly) ← ADC",     s_tdata[3] == ADC_DATA);
        check("EX2: slot2(cho) ← slot3",   s_tdata[2] == SLOT3_DATA);
        check("EX2: slot1(pha) ← slot2",   s_tdata[1] == SLOT2_DATA);
        check("EX2: slot0(trm) ← slot1",   s_tdata[0] == SLOT1_DATA);
        check("EX2: DAC ← slot0(trm)",     s_tdata[5] == SLOT0_DATA);

        // ==================================================================
        // EXAMPLE 3: Skip Effects (ADC → Delay only → DAC)
        //
        //   Only the delay effect is active, everything else skipped.
        //
        //   route[3]=0  slot3(dly) ← ADC
        //   route[5]=4  DAC        ← slot3
        //   (slots 0-2 unused — route to ADC as don't-care)
        // ==================================================================
        $display("");
        $display("--- Example 3: Single Effect (Delay Only) ---");
        $display("    ADC -> Delay -> DAC");

        route[0] = 3'd0;  // don't care (slot0 idle)
        route[1] = 3'd0;  // don't care
        route[2] = 3'd0;  // don't care
        route[3] = 3'd0;  // slot3(dly) ← ADC
        route[5] = 3'd4;  // DAC ← slot3
        route[4] = 3'd0;
        #1;

        check("EX3: slot3(dly) ← ADC",  s_tdata[3] == ADC_DATA);
        check("EX3: DAC ← slot3",       s_tdata[5] == SLOT3_DATA);

        // ==================================================================
        // EXAMPLE 4: Fan-Out (ADC feeds Tremolo AND Phaser in parallel)
        //
        //   ADC ─┬─→ Tremolo → Chorus → DAC
        //        └─→ Phaser  → Delay  → (unused)
        //
        //   Both slot0 and slot1 read from ADC (master 0).
        //   Two independent chains from the same source.
        // ==================================================================
        $display("");
        $display("--- Example 4: Fan-Out (ADC to two parallel chains) ---");
        $display("    ADC -+-> Tremolo -> Chorus -> DAC");
        $display("         +-> Phaser  -> Delay");

        route[0] = 3'd0;  // slot0(trm) ← ADC
        route[1] = 3'd0;  // slot1(pha) ← ADC   *** SAME SOURCE ***
        route[2] = 3'd1;  // slot2(cho) ← slot0
        route[3] = 3'd2;  // slot3(dly) ← slot1
        route[5] = 3'd3;  // DAC ← slot2(cho)
        route[4] = 3'd0;
        #1;

        check("EX4: slot0(trm) ← ADC",    s_tdata[0] == ADC_DATA);
        check("EX4: slot1(pha) ← ADC",    s_tdata[1] == ADC_DATA);
        check("EX4: slot2(cho) ← slot0",  s_tdata[2] == SLOT0_DATA);
        check("EX4: slot3(dly) ← slot1",  s_tdata[3] == SLOT1_DATA);
        check("EX4: DAC ← slot2(cho)",    s_tdata[5] == SLOT2_DATA);
        check("EX4: ADC tready=1 (fan-out OR)", m_tready[0] == 1'b1);

        // ==================================================================
        // EXAMPLE 5: Feedback Loop (Delay output → Delay input)
        //
        //   WARNING: this creates a combinational loop in the crossbar
        //   since it's purely combinational.  In practice you'd need the
        //   effect's registered output to break the loop (which the slot
        //   provides).  Here we just verify the route register accepts it.
        //
        //   route[3]=4  slot3(dly) ← slot3 output (self-feedback)
        // ==================================================================
        $display("");
        $display("--- Example 5: Self-Feedback Route (delay → delay) ---");
        $display("    Slot 3 output fed back to Slot 3 input");

        route[0] = 3'd0;
        route[1] = 3'd1;
        route[2] = 3'd2;
        route[3] = 3'd4;  // slot3 ← slot3 output (master 4)
        route[5] = 3'd4;  // DAC ← slot3
        route[4] = 3'd0;
        #1;

        check("EX5: slot3 input = slot3 output (feedback route)",
              s_tdata[3] == SLOT3_DATA);
        check("EX5: DAC also gets slot3", s_tdata[5] == SLOT3_DATA);

        // ==================================================================
        // EXAMPLE 6: Clean Bypass (ADC → DAC, no effects)
        //
        //   route[5]=0  DAC ← ADC directly
        // ==================================================================
        $display("");
        $display("--- Example 6: Full Bypass (ADC -> DAC) ---");
        $display("    No effects in path");

        route[0] = 3'd0;
        route[1] = 3'd0;
        route[2] = 3'd0;
        route[3] = 3'd0;
        route[5] = 3'd0;  // DAC ← ADC directly
        route[4] = 3'd0;
        #1;

        check("EX6: DAC ← ADC (full bypass)", s_tdata[5] == ADC_DATA);

        // ==================================================================
        // EXAMPLE 7: Guitar Amp Style (Overdrive → Chorus → Delay)
        //
        //   ADC → Phaser(as overdrive) → Chorus → Delay → DAC
        //   Tremolo slot unused.
        // ==================================================================
        $display("");
        $display("--- Example 7: 3-Effect Chain (skip tremolo) ---");
        $display("    ADC -> Phaser -> Chorus -> Delay -> DAC");

        route[0] = 3'd0;  // slot0(trm) ← ADC (idle, bypassed)
        route[1] = 3'd0;  // slot1(pha) ← ADC
        route[2] = 3'd2;  // slot2(cho) ← slot1(pha)
        route[3] = 3'd3;  // slot3(dly) ← slot2(cho)
        route[5] = 3'd4;  // DAC ← slot3(dly)
        route[4] = 3'd0;
        #1;

        check("EX7: slot1(pha) ← ADC",    s_tdata[1] == ADC_DATA);
        check("EX7: slot2(cho) ← slot1",  s_tdata[2] == SLOT1_DATA);
        check("EX7: slot3(dly) ← slot2",  s_tdata[3] == SLOT2_DATA);
        check("EX7: DAC ← slot3(dly)",    s_tdata[5] == SLOT3_DATA);

        // ==================================================================
        // EXAMPLE 8: Dynamic Reconfiguration
        //
        //   Start with linear chain, then hot-swap to bypass mid-stream.
        // ==================================================================
        $display("");
        $display("--- Example 8: Dynamic Reconfiguration ---");
        $display("    Linear chain -> hot-swap to full bypass");

        // Start: linear
        route[0] = 3'd0;
        route[1] = 3'd1;
        route[2] = 3'd2;
        route[3] = 3'd3;
        route[5] = 3'd4;
        route[4] = 3'd0;
        #1;
        check("EX8a: DAC ← slot3 (linear)", s_tdata[5] == SLOT3_DATA);

        // Hot-swap to bypass
        route[5] = 3'd0;
        #1;
        check("EX8b: DAC ← ADC (bypass)", s_tdata[5] == ADC_DATA);

        // Hot-swap back to linear
        route[5] = 3'd4;
        #1;
        check("EX8c: DAC ← slot3 (back to linear)", s_tdata[5] == SLOT3_DATA);

        // ==================================================================
        $display("");
        $display("==========================================================");
        $display("  Results: %0d passed, %0d failed", pass_count, fail_count);
        $display("==========================================================");
        if (fail_count > 0) $fatal(1, "TESTS FAILED");
        $finish;
    end

endmodule
