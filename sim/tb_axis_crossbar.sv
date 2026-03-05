`timescale 1ns / 1ps

// ============================================================================
// tb_axis_crossbar.sv — Testbench for axis_crossbar
//
// Tests:
//   1. Identity routing (port k → port k)
//   2. Custom routing (reorder)
//   3. Fan-out (two sinks reading same master, tready OR'd)
//   4. Dynamic route change mid-stream
//   5. Out-of-range route → silence
//   6. tready back-pressure propagation
// ============================================================================

module tb_axis_crossbar;

    localparam int NP    = 12;
    localparam int DW    = 48;
    localparam int SEL_W = $clog2(NP);

    logic [SEL_W-1:0] route [NP];

    logic [DW-1:0] m_tdata  [NP];
    logic          m_tvalid [NP];
    logic          m_tready [NP];
    logic [DW-1:0] s_tdata  [NP];
    logic          s_tvalid [NP];
    logic          s_tready [NP];

    axis_crossbar #(.N_PORTS(NP), .DATA_W(DW), .SEL_W(SEL_W)) dut (.*);

    int pass_count = 0;
    int fail_count = 0;

    task automatic check(input string name, input logic condition);
        if (condition) pass_count++;
        else begin fail_count++; $error("FAIL: %s", name); end
    endtask

    // Convenience: set all master ports to distinct known values
    task automatic set_masters();
        for (int i = 0; i < NP; i++) begin
            m_tdata[i]  = DW'(48'hA00000_000000 + i * 48'h010101_010101);
            m_tvalid[i] = 1'b1;
        end
    endtask

    initial begin
        $display("=== tb_axis_crossbar ===");

        // ---- Default: all sinks ready, all masters valid ----
        for (int i = 0; i < NP; i++) begin
            route[i]    = SEL_W'(i);   // identity
            s_tready[i] = 1'b1;
        end
        set_masters();
        #1;

        // ---- Test 1: Identity routing ----
        for (int i = 0; i < NP; i++) begin
            check($sformatf("IDENT[%0d]: data matches", i),
                  s_tdata[i] == m_tdata[i]);
            check($sformatf("IDENT[%0d]: tvalid matches", i),
                  s_tvalid[i] == m_tvalid[i]);
        end

        // ---- Test 2: Custom routing (reverse) ----
        for (int i = 0; i < NP; i++)
            route[i] = SEL_W'(NP - 1 - i);
        #1;
        for (int i = 0; i < NP; i++) begin
            check($sformatf("REV[%0d]: data from port %0d", i, NP-1-i),
                  s_tdata[i] == m_tdata[NP-1-i]);
        end

        // ---- Test 3: Fan-out (port 0 and 1 both read from master 2) ----
        for (int i = 0; i < NP; i++) route[i] = SEL_W'(i);
        route[0] = SEL_W'(2);
        route[1] = SEL_W'(2);
        s_tready[0] = 1'b1;
        s_tready[1] = 1'b1;
        #1;
        check("FANOUT: s[0] gets m[2] data",  s_tdata[0] == m_tdata[2]);
        check("FANOUT: s[1] gets m[2] data",  s_tdata[1] == m_tdata[2]);
        check("FANOUT: m[2] tready = 1 (OR of two sinks)", m_tready[2] == 1'b1);

        // Fan-out with one sink not ready
        s_tready[0] = 1'b0;
        s_tready[1] = 1'b1;
        #1;
        check("FANOUT_BP: m[2] tready still 1 (OR)", m_tready[2] == 1'b1);
        s_tready[0] = 1'b0;
        s_tready[1] = 1'b0;
        #1;
        check("FANOUT_BP: m[2] tready = 0 (both sinks blocked)", m_tready[2] == 1'b0);

        // ---- Test 4: tready back-pressure (single sink) ----
        for (int i = 0; i < NP; i++) begin
            route[i]    = SEL_W'(i);
            s_tready[i] = 1'b1;
        end
        s_tready[3] = 1'b0;
        #1;
        check("BP: m[3] tready = 0 when s[3] not ready", m_tready[3] == 1'b0);
        check("BP: m[0] tready = 1 (unrelated)", m_tready[0] == 1'b1);

        // ---- Test 5: tvalid propagation ----
        m_tvalid[4] = 1'b0;
        route[2] = SEL_W'(4);
        #1;
        check("VALID: s[2] tvalid = 0 when m[4] not valid", s_tvalid[2] == 1'b0);
        m_tvalid[4] = 1'b1;

        // ---- Test 6: Out-of-range route → silence ----
        route[0] = SEL_W'(NP);  // beyond valid range
        #1;
        check("OOR: s[0] tdata = 0", s_tdata[0] == '0);
        check("OOR: s[0] tvalid = 0", s_tvalid[0] == 1'b0);

        // ---- Test 7: Dynamic reconfiguration ----
        for (int i = 0; i < NP; i++) route[i] = SEL_W'(i);
        set_masters();
        for (int i = 0; i < NP; i++) s_tready[i] = 1'b1;
        #1;
        check("DYN: before — s[0] = m[0]", s_tdata[0] == m_tdata[0]);
        route[0] = SEL_W'(3);
        #1;
        check("DYN: after  — s[0] = m[3]", s_tdata[0] == m_tdata[3]);

        $display("=== axis_crossbar: %0d passed, %0d failed ===", pass_count, fail_count);
        if (fail_count > 0) $fatal(1, "TESTS FAILED");
        $finish;
    end

endmodule
