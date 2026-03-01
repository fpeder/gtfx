`timescale 1ns / 1ps

// ============================================================================
// axis_audio_pkg.sv — Shared types, constants, and helpers
//
// Audio AXI-Stream:  tdata[47:0] = { left[23:0], right[23:0] }
// Byte  AXI-Stream:  tdata[7:0]  = one UART byte
// ============================================================================

package axis_audio_pkg;

  // ---- Audio constants ----
  localparam int AUDIO_WIDTH = 24;
  localparam int TDATA_WIDTH = 2 * AUDIO_WIDTH;  // 48

  // ---- Routing / slot constants ----
  localparam int MAX_SLOTS = 8;
  localparam int CTRL_REG_WIDTH = 8;  // each register is 8 bits
  localparam int CTRL_REGS_PER_SLOT = 8;  // 8 registers per slot

  // ---- Pack / unpack helpers ----
  function automatic logic [TDATA_WIDTH-1:0] pack_stereo(
      input logic signed [AUDIO_WIDTH-1:0] left, input logic signed [AUDIO_WIDTH-1:0] right);
    return {left, right};
  endfunction

  function automatic logic signed [AUDIO_WIDTH-1:0] unpack_left(
      input logic [TDATA_WIDTH-1:0] tdata);
    return tdata[TDATA_WIDTH-1-:AUDIO_WIDTH];
  endfunction

  function automatic logic signed [AUDIO_WIDTH-1:0] unpack_right(
      input logic [TDATA_WIDTH-1:0] tdata);
    return tdata[AUDIO_WIDTH-1:0];
  endfunction

  // ---- Saturation helper ----
  function automatic logic signed [AUDIO_WIDTH-1:0] saturate(
      input logic signed [AUDIO_WIDTH:0] val);
    localparam signed [AUDIO_WIDTH-1:0] POS_MAX = {1'b0, {(AUDIO_WIDTH - 1) {1'b1}}};
    localparam signed [AUDIO_WIDTH-1:0] NEG_MIN = {1'b1, {(AUDIO_WIDTH - 1) {1'b0}}};
    if (val > $signed({1'b0, POS_MAX})) return POS_MAX;
    else if (val < $signed({1'b1, NEG_MIN})) return NEG_MIN;
    else return val[AUDIO_WIDTH-1:0];
  endfunction

endpackage
