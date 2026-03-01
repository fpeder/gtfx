`timescale 1ns / 1ps

// ============================================================================
// axis_crossbar.sv - N-port AXI-Stream routing crossbar (FIXED)
//
// FIX: Rewrote m_tready generation to avoid dynamic-index read-modify-write
//      inside combinational loop, which can mis-synthesize in Vivado.
//      Each m_tready[j] is now computed with an explicit inner loop that
//      checks all slave ports for routes pointing to master j.
// ============================================================================

module axis_crossbar #(
    parameter int N_PORTS = 5,
    parameter int DATA_W  = 48,
    parameter int SEL_W   = $clog2(N_PORTS)
)(
    input  logic [SEL_W-1:0]  route [N_PORTS],

    // Master ports (sources feeding INTO the crossbar)
    input  logic [DATA_W-1:0] m_tdata  [N_PORTS],
    input  logic              m_tvalid [N_PORTS],
    output logic              m_tready [N_PORTS],

    // Slave ports (sinks reading FROM the crossbar)
    output logic [DATA_W-1:0] s_tdata  [N_PORTS],
    output logic              s_tvalid [N_PORTS],
    input  logic              s_tready [N_PORTS]
);

    // ---- Slave-side muxing (unchanged) ----
    always_comb begin
        for (int k = 0; k < N_PORTS; k++) begin
            if (route[k] < SEL_W'(N_PORTS)) begin
                s_tdata[k]  = m_tdata[route[k]];
                s_tvalid[k] = m_tvalid[route[k]];
            end else begin
                s_tdata[k]  = '0;
                s_tvalid[k] = 1'b0;
            end
        end
    end

    // ---- Master-side tready: OR of all slave ports that select this master ----
    // Written as an explicit double loop so synthesis tools see clean logic
    // instead of a dynamic-index read-modify-write accumulation.
    always_comb begin
        for (int j = 0; j < N_PORTS; j++) begin
            m_tready[j] = 1'b0;
            for (int k = 0; k < N_PORTS; k++) begin
                if (route[k] == SEL_W'(j))
                    m_tready[j] = m_tready[j] | s_tready[k];
            end
        end
    end

endmodule