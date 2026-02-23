`timescale 1ns / 1ps

// ============================================================================
// tb_axis_bypass_mux.sv — Testbench for axis_bypass_mux
//
// Tests:
//   1. Bypass mode (enable=0): bypass data passes through, effect drained
//   2. Effect mode (enable=1): effect data passes through, bypass drained
//   3. tready back-pressure correctly routed to active path
//   4. Dynamic switching mid-stream
//   5. Both paths idle (tvalid=0) produces no spurious output
// ============================================================================

module tb_axi_fifo;

  logic        enable;
  logic [47:0] s_axis_byp_tdata;
  logic        s_axis_byp_tvalid;
  logic        s_axis_byp_tready;
  logic [47:0] s_axis_efx_tdata;
  logic        s_axis_efx_tvalid;
  logic        s_axis_efx_tready;
  logic [47:0] m_axis_tdata;
  logic        m_axis_tvalid;
  logic        m_axis_tready;

  axis_bypass_mux dut (.*);

  int pass_count = 0;
  int fail_count = 0;

  task automatic check(input string name, input logic condition);
    if (condition) begin
      pass_count++;
    end else begin
      fail_count++;
      $error("FAIL: %s", name);
    end
  endtask

  initial begin
    $display("=== tb_axis_bypass_mux ===");

    // ---- Test 1: Bypass mode ----
    enable            = 1'b0;
    s_axis_byp_tdata  = 48'hAABBCC_112233;
    s_axis_byp_tvalid = 1'b1;
    s_axis_efx_tdata  = 48'hDDEEFF_445566;
    s_axis_efx_tvalid = 1'b1;
    m_axis_tready     = 1'b1;
    #1;

    check("BYP: m_axis_tdata = bypass data", m_axis_tdata == 48'hAABBCC_112233);
    check("BYP: m_axis_tvalid = bypass tvalid", m_axis_tvalid == 1'b1);
    check("BYP: s_axis_byp_tready follows m_axis_tready", s_axis_byp_tready == 1'b1);
    check("BYP: s_axis_efx_tready = 1 (drained)", s_axis_efx_tready == 1'b1);

    // ---- Test 2: Bypass with back-pressure ----
    m_axis_tready = 1'b0;
    #1;
    check("BYP+BP: s_axis_byp_tready = 0 (back-pressure forwarded)", s_axis_byp_tready == 1'b0);
    check("BYP+BP: s_axis_efx_tready still 1 (drained)", s_axis_efx_tready == 1'b1);

    // ---- Test 3: Effect mode ----
    enable        = 1'b1;
    m_axis_tready = 1'b1;
    #1;

    check("EFX: m_axis_tdata = effect data", m_axis_tdata == 48'hDDEEFF_445566);
    check("EFX: m_axis_tvalid = effect tvalid", m_axis_tvalid == 1'b1);
    check("EFX: s_axis_efx_tready follows m_axis_tready", s_axis_efx_tready == 1'b1);
    check("EFX: s_axis_byp_tready = 1 (drained)", s_axis_byp_tready == 1'b1);

    // ---- Test 4: Effect with back-pressure ----
    m_axis_tready = 1'b0;
    #1;
    check("EFX+BP: s_axis_efx_tready = 0 (back-pressure forwarded)", s_axis_efx_tready == 1'b0);
    check("EFX+BP: s_axis_byp_tready still 1 (drained)", s_axis_byp_tready == 1'b1);

    // ---- Test 5: Both paths idle ----
    m_axis_tready     = 1'b1;
    s_axis_byp_tvalid = 1'b0;
    s_axis_efx_tvalid = 1'b0;
    enable            = 1'b0;
    #1;
    check("IDLE BYP: m_axis_tvalid = 0", m_axis_tvalid == 1'b0);
    enable = 1'b1;
    #1;
    check("IDLE EFX: m_axis_tvalid = 0", m_axis_tvalid == 1'b0);

    // ---- Test 6: Dynamic switch ----
    enable            = 1'b0;
    s_axis_byp_tdata  = 48'h111111_222222;
    s_axis_byp_tvalid = 1'b1;
    s_axis_efx_tdata  = 48'h333333_444444;
    s_axis_efx_tvalid = 1'b1;
    m_axis_tready     = 1'b1;
    #1;
    check("DYN: bypass selected first", m_axis_tdata == 48'h111111_222222);
    enable = 1'b1;
    #1;
    check("DYN: effect selected after switch", m_axis_tdata == 48'h333333_444444);

    $display("=== bypass_mux: %0d passed, %0d failed ===", pass_count, fail_count);
    if (fail_count > 0) $fatal(1, "TESTS FAILED");
    $finish;
  end

endmodule
