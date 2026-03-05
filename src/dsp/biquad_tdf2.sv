// =============================================================================
// biquad_tdf2.sv
// Transposed Direct Form II biquad filter  -  1 cycle per sample
//
// Transfer function:  H(z) = (b0 + b1*z^-1 + b2*z^-2)
//                           / (1  + a1*z^-1 + a2*z^-2)
//
// Coefficients are Q1.FRAC fixed-point signed integers.
// Pass -a1 and -a2 as a1_neg / a2_neg (pre-negated for accumulate).
// =============================================================================

`timescale 1ns / 1ps

module biquad_tdf2 #(
    parameter int DATA_W  = 24,
    parameter int COEFF_W = 24,
    parameter int FRAC    = 23
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     en,
    input  logic signed [DATA_W-1:0] x_in,
    output logic signed [DATA_W-1:0] y_out,

    // Numerator
    input logic signed [COEFF_W-1:0] b0,
    input logic signed [COEFF_W-1:0] b1,
    input logic signed [COEFF_W-1:0] b2,
    // Denominator (pre-negated: pass -a1, -a2)
    input logic signed [COEFF_W-1:0] a1_neg,
    input logic signed [COEFF_W-1:0] a2_neg
);
  localparam int ACC_W = DATA_W + COEFF_W;

  logic signed [ACC_W-1:0] s1, s2;

  localparam signed [ACC_W-1:0] ROUND = ACC_W'(1) <<< (FRAC - 1);

  wire signed [ACC_W-1:0] xe = {{COEFF_W{x_in[DATA_W-1]}}, x_in};
  wire signed [ACC_W-1:0] y_full = (xe * b0 + s1 + ROUND) >>> FRAC;
  wire signed [ACC_W-1:0] s1_next = xe * b1 + y_full * a1_neg + s2;
  wire signed [ACC_W-1:0] s2_next = xe * b2 + y_full * a2_neg;

  // FIX #4: Saturate y_full before truncating to DATA_W bits.
  //         Raw truncation (y_full[DATA_W-1:0]) wraps on overflow,
  //         causing massive clicks when the IIR transiently overshoots.
  wire signed [DATA_W-1:0] y_sat;
  saturate #(.IN_W(ACC_W), .OUT_W(DATA_W)) u_y_sat (
      .din  (y_full),
      .dout (y_sat)
  );

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      s1    <= '0;
      s2    <= '0;
      y_out <= '0;
    end else if (en) begin
      s1    <= s1_next;
      s2    <= s2_next;
      y_out <= y_sat;
    end
  end

endmodule : biquad_tdf2