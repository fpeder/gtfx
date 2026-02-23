`timescale 1ns / 1ps

// ============================================================================
// axis_bypass_mux.sv — AXI-Stream 2:1 bypass mux (combinational)
// enable=1 → effect path;  enable=0 → bypass path
// ============================================================================

module axis_bypass_mux (
    input  logic        enable,

    // Bypass path
    input  logic [47:0] s_axis_byp_tdata,
    input  logic        s_axis_byp_tvalid,
    output logic        s_axis_byp_tready,

    // Effect path
    input  logic [47:0] s_axis_efx_tdata,
    input  logic        s_axis_efx_tvalid,
    output logic        s_axis_efx_tready,

    // Output
    output logic [47:0] m_axis_tdata,
    output logic        m_axis_tvalid,
    input  logic        m_axis_tready
);

    always_comb begin
        if (enable) begin
            m_axis_tdata       = s_axis_efx_tdata;
            m_axis_tvalid      = s_axis_efx_tvalid;
            s_axis_efx_tready  = m_axis_tready;
            s_axis_byp_tready  = 1'b1;
        end else begin
            m_axis_tdata       = s_axis_byp_tdata;
            m_axis_tvalid      = s_axis_byp_tvalid;
            s_axis_byp_tready  = m_axis_tready;
            s_axis_efx_tready  = 1'b1;
        end
    end

endmodule
