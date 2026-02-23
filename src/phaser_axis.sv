`timescale 1ns / 1ps

// ============================================================================
// phaser_axis.sv — AXI-Stream wrapper for the original phaser.sv core
// Instantiates phaser.sv internally.  Mono effect: L processed, replicated to R.
// ============================================================================

module phaser_axis #(
    parameter int WIDTH = 24
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
    input  logic [7:0]  speed_val,
    input  logic        feedback_en
);

    import axis_audio_pkg::*;

    assign s_axis_tready = 1'b1;
    wire beat_in = s_axis_tvalid & s_axis_tready;

    // ---- Drive core ----
    logic                    sample_en_reg = 1'b0;
    logic signed [WIDTH-1:0] audio_in_reg  = '0;

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
    logic signed [WIDTH-1:0] core_out;

    phaser #(.WIDTH(WIDTH)) core (
        .clk         (clk),
        .rst_n       (rst_n),
        .sample_en   (sample_en_reg),
        .speed_val   (speed_val),
        .feedback_en (feedback_en),
        .audio_in    (audio_in_reg),
        .audio_out   (core_out)
    );

    // ---- Capture output ----
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
                m_data_reg  <= pack_stereo(core_out, core_out);
                m_valid_reg <= 1'b1;
            end
        end
    end

endmodule
