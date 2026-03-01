// =============================================================================
// linear_interp.sv
// 3-cycle pipelined linear interpolator
//
//   y_out = y0 + round( (y1 - y0) * frac / 2^FRAC_W )
//
// Pipeline stages:
//   Stage 1 : delta = y1 - y0
//   Stage 2 : product = delta * frac
//   Stage 3 : y_out = y0 + round(product >> FRAC_W)
// =============================================================================

`timescale 1ns / 1ps

module linear_interp #(
    parameter int DATA_W = 24,
    parameter int FRAC_W = 12
) (
    input  logic                     clk,
    input  logic signed [DATA_W-1:0] y0,
    input  logic signed [DATA_W-1:0] y1,
    input  logic        [FRAC_W-1:0] frac,
    output logic signed [DATA_W-1:0] y_out
);
  localparam int PROD_W = DATA_W + 1 + FRAC_W;  // 37

  // Stage-1 pipeline registers
  logic signed [  DATA_W:0] delta_p1;
  logic signed [DATA_W-1:0] y0_p1;
  logic        [FRAC_W-1:0] frac_p1;

  // Stage-2 pipeline registers
  logic signed [PROD_W-1:0] prod_p2;
  logic signed [DATA_W-1:0] y0_p2;

  // Combinational stage inputs (no automatic variables)
  wire signed  [  DATA_W:0] delta_comb = {y1[DATA_W-1], y1} - {y0[DATA_W-1], y0};
  wire signed  [PROD_W-1:0] prod_comb = delta_p1 * $signed({1'b0, frac_p1});
  wire signed  [PROD_W-1:0] rounded = prod_p2 + (PROD_W'(1) <<< (FRAC_W - 1));
  wire signed  [DATA_W-1:0] correction = rounded[PROD_W-1:FRAC_W];

  always_ff @(posedge clk) begin
    // Stage 1
    delta_p1 <= delta_comb;
    y0_p1    <= y0;
    frac_p1  <= frac;
    // Stage 2
    prod_p2  <= prod_comb;
    y0_p2    <= y0_p1;
    // Stage 3
    y_out    <= y0_p2 + correction;
  end

endmodule : linear_interp
