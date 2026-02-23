`timescale 1ns / 1ps

// ============================================================================
// axis_audio_pkg.sv — Shared types & constants for AXI-Stream audio pipeline
//
// Audio bus:  tdata[47:0] = { left[23:0] , right[23:0] }
// Byte bus:   tdata[7:0]  = one UART byte
//
// Transfer occurs on rising clock edge when (tvalid && tready).
// ============================================================================

package axis_audio_pkg;

    localparam int AUDIO_WIDTH = 24;
    localparam int TDATA_WIDTH = 2 * AUDIO_WIDTH;  // 48 bits

    function automatic logic [TDATA_WIDTH-1:0] pack_stereo(
        input logic signed [AUDIO_WIDTH-1:0] left,
        input logic signed [AUDIO_WIDTH-1:0] right
    );
        return {left, right};
    endfunction

    function automatic logic signed [AUDIO_WIDTH-1:0] unpack_left(
        input logic [TDATA_WIDTH-1:0] tdata
    );
        return tdata[TDATA_WIDTH-1 -: AUDIO_WIDTH];
    endfunction

    function automatic logic signed [AUDIO_WIDTH-1:0] unpack_right(
        input logic [TDATA_WIDTH-1:0] tdata
    );
        return tdata[AUDIO_WIDTH-1:0];
    endfunction

endpackage
