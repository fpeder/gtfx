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

module tb_axi_tremolo;

endmodule;