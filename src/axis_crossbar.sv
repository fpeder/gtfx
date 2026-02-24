`timescale 1ns / 1ps

// ============================================================================
// axis_crossbar.sv — N-port AXI-Stream routing crossbar
//
// Pure combinational switch: each output (slave) port has a SEL_W-bit route
// register that selects which input (master) port it reads from.
//
// Port naming follows the convention of an interconnect:
//   - Master ports are INPUTS  to the crossbar (sources push data in)
//   - Slave  ports are OUTPUTS from the crossbar (sinks pull data out)
//
// Port indexing convention:
//   port 0           = ADC source (master only — feeds data in)
//   port 1..N_SLOTS  = effect slots (both master and slave)
//   port N_SLOTS+1   = DAC sink (slave only — consumes data out)
//
// Multiple sinks may select the same master (fan-out).
// tready is OR-reduced across all sinks selecting a given master.
//
// Timing: at 48 kHz sample rate on a 12 MHz+ clock, the combinational
// path has >200 cycles of slack per sample.  Register outputs if needed.
// ============================================================================

module axis_crossbar #(
    parameter int N_PORTS = 6,
    parameter int DATA_W  = 48,
    parameter int SEL_W   = $clog2(N_PORTS)
)(
    // Route configuration: route[k] selects which master feeds slave port k
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

    always_comb begin
        // Default: no master is being read
        for (int i = 0; i < N_PORTS; i++)
            m_tready[i] = 1'b0;

        for (int k = 0; k < N_PORTS; k++) begin
            // Bounds-check: if route points beyond valid range, output silence
            if (route[k] < SEL_W'(N_PORTS)) begin
                s_tdata[k]          = m_tdata[route[k]];
                s_tvalid[k]         = m_tvalid[route[k]];
                m_tready[route[k]]  = m_tready[route[k]] | s_tready[k];
            end else begin
                s_tdata[k]  = '0;
                s_tvalid[k] = 1'b0;
            end
        end
    end

endmodule
