`timescale 1ns / 1ps

// ============================================================================
// audio_axis.sv — I2S audio with AXI-Stream master (ADC) & slave (DAC)
// Instantiates the original i2s2 core internally.
// ============================================================================

module audio_axis #(
    parameter int AUDIO_W = 24
) (
    input  logic clk_audio,
    input  logic resetn,

    output logic tx_mclk,
    output logic tx_lrclk,
    output logic tx_sclk,
    output logic tx_serial,
    output logic rx_mclk,
    output logic rx_lrclk,
    output logic rx_sclk,
    input  logic rx_serial,

    // AXI-Stream master — ADC samples out (48-bit stereo)
    output logic [47:0] m_axis_tdata,
    output logic        m_axis_tvalid,
    input  logic        m_axis_tready,

    // AXI-Stream slave — processed samples in (48-bit stereo)
    input  logic [47:0] s_axis_tdata,
    input  logic        s_axis_tvalid,
    output logic        s_axis_tready
);

    import axis_audio_pkg::*;

    logic [AUDIO_W-1:0] din_l, din_r;
    logic [AUDIO_W-1:0] dout_l, dout_r;

    i2s2 #(.AUDIO_W(AUDIO_W)) driver (
        .clk       (clk_audio),
        .resetn    (resetn),
        .tx_mclk   (tx_mclk),
        .tx_lrclk  (tx_lrclk),
        .tx_sclk   (tx_sclk),
        .tx_serial (tx_serial),
        .tx_data_l (din_l),
        .tx_data_r (din_r),
        .rx_mclk   (rx_mclk),
        .rx_lrclk  (rx_lrclk),
        .rx_sclk   (rx_sclk),
        .rx_serial (rx_serial),
        .rx_data_l (dout_l),
        .rx_data_r (dout_r)
    );

    logic lrclk_prev;
    logic sample_strobe;

    always_ff @(posedge clk_audio) begin
        if (!resetn) lrclk_prev <= 1'b0;
        else         lrclk_prev <= tx_lrclk;
    end
    assign sample_strobe = tx_lrclk & ~lrclk_prev;

    // ---- Master (ADC → pipeline) ----
    logic [47:0] m_data_reg;
    logic        m_valid_reg;

    always_ff @(posedge clk_audio) begin
        if (!resetn) begin
            m_data_reg  <= '0;
            m_valid_reg <= 1'b0;
        end else begin
            if (m_valid_reg && m_axis_tready)
                m_valid_reg <= 1'b0;
            if (sample_strobe) begin
                m_data_reg  <= pack_stereo($signed(dout_l), $signed(dout_r));
                m_valid_reg <= 1'b1;
            end
        end
    end

    assign m_axis_tdata  = m_data_reg;
    assign m_axis_tvalid = m_valid_reg;

    // ---- Slave (pipeline → DAC) ----
    always_ff @(posedge clk_audio) begin
        if (!resetn) begin
            din_l <= '0;
            din_r <= '0;
        end else if (s_axis_tvalid && s_axis_tready) begin
            din_l <= unpack_left(s_axis_tdata);
            din_r <= unpack_right(s_axis_tdata);
        end
    end

    assign s_axis_tready = 1'b1;

endmodule
