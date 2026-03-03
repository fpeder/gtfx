`timescale 1ns / 1ps

// ============================================================================
// lfo_core.sv - Shared LFO Module
//
// Encapsulates: phase accumulator + rate-to-increment mapping + waveform_lut
// + signed-to-unsigned conversion.
//
// Unified increment formula:
//   inc = INC_BASE + (rate_val * INC_SCALE) >> INC_SHIFT
//
// Outputs:
//   phase_out     - raw phase accumulator (for driving additional waveform_luts)
//   wave_signed   - Q1.(LFO_W-1) signed waveform from waveform_lut
//   wave_unsigned - Q0.LFO_W unsigned: signed + LFO_PEAK offset
// ============================================================================

module lfo_core #(
    parameter int PHASE_W   = 32,          // Phase accumulator width
    parameter int CTRL_W    = 8,           // Control knob width (unsigned)
    parameter int LFO_W     = 16,          // LFO output width (signed Q1.(LFO_W-1))
    parameter int INC_BASE  = 44739,       // Base phase increment (0 Hz offset)
    parameter int INC_SCALE = 4035,        // Scale factor for rate_val
    parameter int INC_SHIFT = 0,           // Right-shift on rate_val * INC_SCALE
    parameter int TABLE_BITS = 8,          // waveform_lut table size (log2)
    parameter string WAVE_TYPE = "TRIANGLE"
) (
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    sample_en,
    input  logic [CTRL_W-1:0]      rate_val,

    output logic [PHASE_W-1:0]     phase_out,
    output logic signed [LFO_W-1:0] wave_signed,
    output logic [LFO_W-1:0]       wave_unsigned
);

    localparam int LFO_FRAC = LFO_W - 1;
    localparam int LFO_PEAK = (1 << LFO_FRAC) - 1;

    // =========================================================================
    // Phase accumulator
    // =========================================================================
    logic [PHASE_W-1:0] lfo_phase;
    logic [PHASE_W-1:0] lfo_inc;

    // inc = INC_BASE + (rate_val * INC_SCALE) >> INC_SHIFT
    always_comb begin
        lfo_inc = PHASE_W'(INC_BASE)
                + PHASE_W'(({{PHASE_W{1'b0}}, rate_val} * (PHASE_W+CTRL_W)'(INC_SCALE)) >> INC_SHIFT);
    end

    always_ff @(posedge clk) begin
        if (!rst_n)          lfo_phase <= '0;
        else if (sample_en)  lfo_phase <= lfo_phase + lfo_inc;
    end

    assign phase_out = lfo_phase;

    // =========================================================================
    // Waveform LUT
    // =========================================================================
    waveform_lut #(
        .TABLE_BITS (TABLE_BITS),
        .OUT_WIDTH  (LFO_W),
        .WAVE_TYPE  (WAVE_TYPE),
        .PHASE_W    (PHASE_W)
    ) u_lfo_wave (
        .phase_in  (lfo_phase),
        .wave_out  (wave_signed),
        .frac_out  ()
    );

    // =========================================================================
    // Signed → unsigned conversion: add LFO_PEAK offset
    // Q1.(LFO_W-1) signed → Q0.LFO_W unsigned, range [0, 2·LFO_PEAK]
    // =========================================================================
    assign wave_unsigned = $unsigned(wave_signed + LFO_W'(LFO_PEAK));

endmodule
