// =============================================================================
// tube_lut_rom.sv
// 12AX7 triode transfer function ROM  –  4096 × 24-bit (infers block RAM)
//
// Transfer function:
//   x   = (i/2048 - 1.0)*2.5 + 0.20   (positive DC bias → even harmonics)
//   y   = 0.93*tanh(x)                 fundamental soft clip
//       + 0.05*tanh(1.7*x^2*sgn(x))   2nd harmonic distortion
//       + 0.02*tanh(3x)                3rd harmonic grit
// =============================================================================

`timescale 1ns / 1ps

module tube_lut_rom #(
    parameter int ADDR_W = 12,
    parameter int DATA_W = 24
) (
    input  logic                     clk,
    input  logic        [ADDR_W-1:0] addr,
    output logic signed [DATA_W-1:0] dout
);
  localparam int DEPTH = 2 ** ADDR_W;

  logic signed [DATA_W-1:0] mem[0:DEPTH-1];

  initial begin : rom_init
    real x, y, sgn;
    for (int i = 0; i < DEPTH; i++) begin
      x   = (real'(i) / real'(DEPTH / 2) - 1.0) * 2.5 + 0.20;
      sgn = (x >= 0.0) ? 1.0 : -1.0;
      y   = 0.93 * $tanh(x) + 0.05 * $tanh(1.7 * x * x * sgn) + 0.02 * $tanh(3.0 * x);
      if (y > 1.0) y = 1.0;
      if (y < -1.0) y = -1.0;
      mem[i] = DATA_W'(signed'($rtoi(y * real'(2 ** (DATA_W - 1) - 1))));
    end
  end

  always_ff @(posedge clk) dout <= mem[addr];

endmodule : tube_lut_rom
