`timescale 1ns / 1ps

// ============================================================================
// chorus_axis.sv — AXI-Stream wrapper for the original chorus.sv core
// Instantiates chorus.sv internally.  Mono in, stereo out.
// ============================================================================

module chorus_axis #(
    parameter int SAMPLE_RATE = 48_000,
    parameter int DATA_WIDTH  = 24,
    parameter int DELAY_MAX   = 2048
)(
    input  logic        clk,
    input  logic        rst_n,

    // AXI-Stream slave (input, 48-bit stereo)
    input  logic [47:0] s_axis_tdata,
    input  logic        s_axis_tvalid,
    output logic        s_axis_tready,

    // AXI-Stream master (output, 48-bit stereo)
    output logic [47:0] m_axis_tdata,
    output logic        m_axis_tvalid,
    input  logic        m_axis_tready,

    // Controls
    input  logic [7:0]  rate,
    input  logic [7:0]  depth,
    input  logic [7:0]  effect_lvl,
    input  logic [7:0]  e_q_hi,
    input  logic [7:0]  e_q_lo
);

    import axis_audio_pkg::*;

    assign s_axis_tready = 1'b1;
    wire beat_in = s_axis_tvalid & s_axis_tready;

    // ---- Drive core ----
    logic                         sample_en_reg = 1'b0;
    logic signed [DATA_WIDTH-1:0] audio_in_reg  = '0;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            sample_en_reg <= 1'b0;
            audio_in_reg  <= '0;
        end else begin
            sample_en_reg <= beat_in;
            if (beat_in)
                audio_in_reg <= unpack_left(s_axis_tdata);
        end
    end

    // ---- Original core (UNCHANGED) ----
    logic signed [DATA_WIDTH-1:0] core_out_l, core_out_r;

    chorus #(
        .SAMPLE_RATE(SAMPLE_RATE),
        .DATA_WIDTH (DATA_WIDTH),
        .DELAY_MAX  (DELAY_MAX)
    ) core (
        .clk        (clk),
        .rst_n      (rst_n),
        .sample_en  (sample_en_reg),
        .audio_in   (audio_in_reg),
        .audio_out_l(core_out_l),
        .audio_out_r(core_out_r),
        .rate       (rate),
        .depth      (depth),
        .effect_lvl (effect_lvl),
        .e_q_hi     (e_q_hi),
        .e_q_lo     (e_q_lo)
    );

    // ---- Capture stereo output ----
    logic        sample_en_d1 = 1'b0;
    logic [47:0] m_data_reg   = '0;
    logic        m_valid_reg  = 1'b0;

    assign m_axis_tdata  = m_data_reg;
    assign m_axis_tvalid = m_valid_reg;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            sample_en_d1 <= 1'b0;
            m_data_reg   <= '0;
            m_valid_reg  <= 1'b0;
        end else begin
            sample_en_d1 <= sample_en_reg;
            if (m_valid_reg && m_axis_tready)
                m_valid_reg <= 1'b0;
            if (sample_en_d1) begin
                m_data_reg  <= pack_stereo(core_out_l, core_out_r);
                m_valid_reg <= 1'b1;
            end
        end
    end

endmodule
