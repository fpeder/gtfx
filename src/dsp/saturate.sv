// =============================================================================
// saturate.sv
// Combinational signed saturation clamp:  IN_W bits → OUT_W bits
//
// If the value fits in OUT_W bits (all guard bits match the sign),
// it passes through unchanged. Otherwise clamps to max/min.
// =============================================================================

`timescale 1ns / 1ps

module saturate #(
    parameter int IN_W  = 33,
    parameter int OUT_W = 24
) (
    input  logic signed [ IN_W-1:0] din,
    output logic signed [OUT_W-1:0] dout
);
  localparam int GUARD = IN_W - OUT_W;

  wire sign_ok = (din[IN_W-1:OUT_W-1] == {(GUARD + 1) {din[IN_W-1]}});

  assign dout = sign_ok     ? din[OUT_W-1:0]             :
                  din[IN_W-1] ? {1'b1, {(OUT_W-1){1'b0}}} :
                                {1'b0, {(OUT_W-1){1'b1}}};

endmodule : saturate
