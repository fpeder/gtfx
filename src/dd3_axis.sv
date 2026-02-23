`timescale 1ns / 1ps

// ============================================================================
// dd3_axis.sv — AXI-Stream wrapper for the original dd3.sv core
// Instantiates dd3.sv internally.  Wet-only core + built-in dry+wet sat. mix.
// ============================================================================

module dd3_axis #(
    parameter int WIDTH     = 24,
    parameter int RAM_DEPTH = 48000
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
    input  logic [7:0]  tone_val,
    input  logic [7:0]  level_val,
    input  logic [7:0]  feedback_val,
    input  logic [31:0] time_val
);

    import axis_audio_pkg::*;

    localparam signed [WIDTH-1:0] SAT_MAX = {1'b0, {(WIDTH-1){1'b1}}};
    localparam signed [WIDTH-1:0] SAT_MIN = {1'b1, {(WIDTH-1){1'b0}}};

    assign s_axis_tready = 1'b1;
    wire beat_in = s_axis_tvalid & s_axis_tready;

    // ---- Drive core & hold dry samples ----
    logic                    sample_en_reg = 1'b0;
    logic signed [WIDTH-1:0] audio_in_reg  = '0;
    logic signed [WIDTH-1:0] dry_l_hold    = '0;
    logic signed [WIDTH-1:0] dry_r_hold    = '0;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            sample_en_reg <= 1'b0;
            audio_in_reg  <= '0;
            dry_l_hold    <= '0;
            dry_r_hold    <= '0;
        end else begin
            sample_en_reg <= beat_in;
            if (beat_in) begin
                audio_in_reg <= unpack_left(s_axis_tdata);
                dry_l_hold   <= unpack_left(s_axis_tdata);
                dry_r_hold   <= unpack_right(s_axis_tdata);
            end
        end
    end

    // ---- Original core (UNCHANGED, wet-only output) ----
    logic signed [WIDTH-1:0] wet_out;

    dd3 #(
        .WIDTH    (WIDTH),
        .RAM_DEPTH(RAM_DEPTH)
    ) core (
        .clk          (clk),
        .rst_n        (rst_n),
        .sample_en    (sample_en_reg),
        .tone_val     (tone_val),
        .level_val    (level_val),
        .feedback_val (feedback_val),
        .time_val     (time_val),
        .audio_in     (audio_in_reg),
        .audio_out    (wet_out)
    );

    // ---- Dry + Wet saturating mix (was in original top.sv) ----
    logic signed [WIDTH:0]   sum_l, sum_r;
    logic signed [WIDTH-1:0] mixed_l, mixed_r;

    always_comb begin
        sum_l = $signed({dry_l_hold[WIDTH-1], dry_l_hold})
              + $signed({wet_out[WIDTH-1],     wet_out});
        sum_r = $signed({dry_r_hold[WIDTH-1], dry_r_hold})
              + $signed({wet_out[WIDTH-1],     wet_out});

        mixed_l = (sum_l > $signed({1'b0, SAT_MAX})) ?  SAT_MAX :
                  (sum_l < $signed({1'b1, SAT_MIN})) ?  SAT_MIN :
                   sum_l[WIDTH-1:0];
        mixed_r = (sum_r > $signed({1'b0, SAT_MAX})) ?  SAT_MAX :
                  (sum_r < $signed({1'b1, SAT_MIN})) ?  SAT_MIN :
                   sum_r[WIDTH-1:0];
    end

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
                m_data_reg  <= pack_stereo(mixed_l, mixed_r);
                m_valid_reg <= 1'b1;
            end
        end
    end

endmodule
