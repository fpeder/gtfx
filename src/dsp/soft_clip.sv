`timescale 1ns / 1ps

// ============================================================================
// soft_clip.sv - Piecewise-Linear Soft Saturation
//
// Pure combinational module (like saturate.sv). Applies symmetric piecewise-
// linear compression:
//   - Below KNEE:  linear pass-through (1:1)
//   - Above KNEE:  compressed at 2^COMP_SHIFT : 1 ratio
//   - Clamped at PEAK
//
// Parameters:
//   IN_W       - Input width (signed)
//   OUT_W      - Output width (signed)
//   COMP_SHIFT - Compression ratio as right-shift: 1 = 2:1, 2 = 4:1
//   KNEE       - Knee threshold (unsigned magnitude, in OUT_W-1 bits)
//   PEAK       - Peak ceiling  (unsigned magnitude, in OUT_W-1 bits)
// ============================================================================

module soft_clip #(
    parameter int IN_W       = 26,
    parameter int OUT_W      = 24,
    parameter int COMP_SHIFT = 1,
    parameter signed [OUT_W-1:0] KNEE = (1 << (OUT_W - 2)),
    parameter signed [OUT_W-1:0] PEAK = {1'b0, {(OUT_W-1){1'b1}}}
) (
    input  logic signed [IN_W-1:0]  din,
    output logic signed [OUT_W-1:0] dout
);

    logic [IN_W-2:0]  x_abs;
    logic [IN_W-2:0]  y_abs;

    always_comb begin
        x_abs = din[IN_W-1] ? (IN_W-1)'(-din) : (IN_W-1)'(din);

        if (x_abs <= (IN_W-1)'(KNEE)) begin
            dout = din[OUT_W-1:0];
        end else begin
            y_abs = (IN_W-1)'(KNEE) + ((x_abs - (IN_W-1)'(KNEE)) >> COMP_SHIFT);
            if (y_abs > (IN_W-1)'(PEAK))
                y_abs = (IN_W-1)'(PEAK);
            dout = din[IN_W-1] ? -OUT_W'(y_abs) : OUT_W'(y_abs);
        end
    end

endmodule
