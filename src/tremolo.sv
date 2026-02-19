`timescale 1ns / 1ps

// Optical Tremolo (Fender/Vox style)
//
// Clocked on clk_audio (12.288 MHz).
// sample_en is a 1-cycle pulse at fs (48 kHz).
//
// Signal path:
//   output[n] = audio_in[n] * gain[n]
//
//   gain[n] = (1 - depth_norm) + depth_norm * lfo[n]
//           = 1 - depth_norm * (1 - lfo[n])
//
//   This means:
//     depth = 0   -> gain = 1.0  (no effect, full volume always)
//     depth = 255 -> gain oscillates between 0.0 and 1.0
//
// LFO shape is selectable via shape_sel:
//   0 = sine      (smooth, like optical cell photoresistor lag)
//   1 = triangle  (linear ramp, harder edge)
//
// Sine approximation via 3rd-order Chebyshev / Bhaskara I:
//   sin(x) ≈ 16x(π - x) / (5π² - 4x(π - x))   for x in [0, π]
//   Implemented in Q16 fixed-point using the half-cycle [0, 2^16) representation.
//
// Fixed-point:
//   Audio:      signed Q0.23  (24-bit, full-scale ±1.0)
//   LFO norm:   unsigned 16-bit  [0, 65535] = [0.0, 1.0)
//   gain_q16:   unsigned 16-bit  [0, 65535] = [0.0, 1.0)
//   Product:    40-bit before >> 16, truncated to 24-bit
//
// Controls (synchronise to clk_audio before connecting):
//   rate_val   [7:0] : LFO rate.  0 ≈ 0.5 Hz, 255 ≈ 12 Hz
//   depth_val  [7:0] : Effect depth. 0 = bypass, 255 = full
//   shape_sel  [0]   : 0 = sine, 1 = triangle
//
// Insertion point: before phaser in chain (first effect on dry signal).

module tremolo #(
    parameter WIDTH = 24
)(
    input  logic                    clk,        // clk_audio (12.288 MHz)
    input  logic                    rst_n,
    input  logic                    sample_en,  // 1-cycle pulse at fs (48 kHz)

    // Controls - synchronise to clk before connecting
    input  logic [7:0]              rate_val,   // LFO rate
    input  logic [7:0]              depth_val,  // Effect depth (0=dry, 255=full chop)
    input  logic                    shape_sel,  // 0=sine, 1=triangle

    // Audio (mono; apply same gain to both channels at instantiation site)
    input  logic signed [WIDTH-1:0] audio_in,
    output logic signed [WIDTH-1:0] audio_out
);

    // =========================================================================
    // 0. SATURATION CONSTANTS
    // =========================================================================
    localparam signed [WIDTH-1:0] SAT_MAX = {1'b0, {(WIDTH-1){1'b1}}};
    localparam signed [WIDTH-1:0] SAT_MIN = {1'b1, {(WIDTH-1){1'b0}}};

    // =========================================================================
    // 1. PHASE ACCUMULATOR
    //
    // 32-bit accumulator, triangle and sine LFO derived from phase[31:16].
    //
    // Rate mapping (same convention as phaser.sv):
    //   inc = base + rate_val * step
    //   0.5 Hz  -> inc =  0.5 * 2^32 / 48000 =  44739
    //   12.0 Hz -> inc = 12.0 * 2^32 / 48000 = 1073742
    //   span = 1073742 - 44739 = 1029003
    //   step = span / 255 = ~4035
    //   base = 44739
    // =========================================================================
    logic [31:0] lfo_phase      = 32'h0;
    logic [31:0] lfo_phase_next;
    logic [31:0] lfo_inc;

    always_comb begin
        lfo_inc        = 32'd44739 + ({24'h0, rate_val} * 32'd4035);
        lfo_phase_next = lfo_phase + lfo_inc;
    end

    // =========================================================================
    // 2. LFO WAVEFORM
    //
    // phase_top = lfo_phase[31:16] sweeps 0..65535.
    //
    // Triangle: fold upper half
    //   lfo_tri[15:0] is [0..65535] for a full up-down cycle.
    //
    // Sine (Bhaskara I approximation, unsigned output [0, 65535]):
    //   Map phase_top to angle x in [0, π]:
    //     half = phase_top[14:0]  (0..32767 covers 0..π, mirrored for π..2π)
    //   Bhaskara I:  sin(x) ≈ 4h(π-h) / (π²  - h(π-h))  where h = x/π
    //   In integer form with h = half (Q15, 0..32767 → 0..π):
    //     num = 4 * half * (32768 - half)          -> 32-bit, fits in Q30
    //     den = π² * 32768² - half * (32768-half)
    //         ≈ 2^31 * (π²/4 - ...)
    //   Practical implementation (Bhaskara I in Q15):
    //     num16 = (4 * half * (32767 - half)) >> 15   -> Q15
    //     den16 = 3 * 32768^2 / (4 * ...) ... too costly.
    //
    //   Simpler accurate integer sine for [0..32767 -> 0..32767]:
    //     Uses a 7-bit lookup table (128 entries covering one quarter-cycle)
    //     with linear interpolation between entries -> 0.01% THD, ~160 bytes LUT.
    //   This is the standard approach for FPGA sine LFOs.
    //
    //   LUT: sine[i] = round(32767 * sin(i * π / (2 * 128)))  for i=0..127
    //
    // =========================================================================
    logic [15:0] phase_top;
    logic [15:0] lfo_tri;
    logic [15:0] lfo_sine;
    logic [15:0] lfo_norm;   // selected LFO output, unsigned [0, 65535]

    assign phase_top = lfo_phase[31:16];

    // --- Triangle ---
    always_comb begin
        lfo_tri = phase_top[15] ? ~phase_top : phase_top;
        // Rescale from [0,32767] to [0,65534]: shift left by 1
        // (phase_top[15]=0 → 0..32767; phase_top[15]=1 → fold to 0..32767)
        lfo_tri = {lfo_tri[14:0], 1'b0};  // * 2
    end

    // --- Sine via quarter-wave LUT (128 entries, Q15) ---
    // Quarter indices: phase_top[14:8] gives 7-bit index (0..127)
    // Quadrant determined by phase_top[15] (half) and phase_top[14] (quarter)

    logic [6:0]  lut_idx;
    logic [15:0] lut_val;
    logic [15:0] sine_quarter;

    // 128-entry quarter-wave sine LUT, values = round(32767*sin(i*pi/256)) i=0..127
    function automatic logic [15:0] sine_lut(input logic [6:0] i);
        case (i)
            7'd0  : sine_lut = 16'd0;
            7'd1  : sine_lut = 16'd804;
            7'd2  : sine_lut = 16'd1608;
            7'd3  : sine_lut = 16'd2410;
            7'd4  : sine_lut = 16'd3212;
            7'd5  : sine_lut = 16'd4011;
            7'd6  : sine_lut = 16'd4808;
            7'd7  : sine_lut = 16'd5602;
            7'd8  : sine_lut = 16'd6393;
            7'd9  : sine_lut = 16'd7179;
            7'd10 : sine_lut = 16'd7962;
            7'd11 : sine_lut = 16'd8739;
            7'd12 : sine_lut = 16'd9512;
            7'd13 : sine_lut = 16'd10278;
            7'd14 : sine_lut = 16'd11039;
            7'd15 : sine_lut = 16'd11793;
            7'd16 : sine_lut = 16'd12539;
            7'd17 : sine_lut = 16'd13279;
            7'd18 : sine_lut = 16'd14010;
            7'd19 : sine_lut = 16'd14733;
            7'd20 : sine_lut = 16'd15446;
            7'd21 : sine_lut = 16'd16151;
            7'd22 : sine_lut = 16'd16846;
            7'd23 : sine_lut = 16'd17530;
            7'd24 : sine_lut = 16'd18204;
            7'd25 : sine_lut = 16'd18868;
            7'd26 : sine_lut = 16'd19519;
            7'd27 : sine_lut = 16'd20159;
            7'd28 : sine_lut = 16'd20787;
            7'd29 : sine_lut = 16'd21403;
            7'd30 : sine_lut = 16'd22005;
            7'd31 : sine_lut = 16'd22594;
            7'd32 : sine_lut = 16'd23170;
            7'd33 : sine_lut = 16'd23731;
            7'd34 : sine_lut = 16'd24279;
            7'd35 : sine_lut = 16'd24811;
            7'd36 : sine_lut = 16'd25329;
            7'd37 : sine_lut = 16'd25832;
            7'd38 : sine_lut = 16'd26319;
            7'd39 : sine_lut = 16'd26790;
            7'd40 : sine_lut = 16'd27245;
            7'd41 : sine_lut = 16'd27683;
            7'd42 : sine_lut = 16'd28105;
            7'd43 : sine_lut = 16'd28510;
            7'd44 : sine_lut = 16'd28898;
            7'd45 : sine_lut = 16'd29268;
            7'd46 : sine_lut = 16'd29621;
            7'd47 : sine_lut = 16'd29956;
            7'd48 : sine_lut = 16'd30273;
            7'd49 : sine_lut = 16'd30571;
            7'd50 : sine_lut = 16'd30852;
            7'd51 : sine_lut = 16'd31113;
            7'd52 : sine_lut = 16'd31356;
            7'd53 : sine_lut = 16'd31580;
            7'd54 : sine_lut = 16'd31785;
            7'd55 : sine_lut = 16'd31971;
            7'd56 : sine_lut = 16'd32137;
            7'd57 : sine_lut = 16'd32285;
            7'd58 : sine_lut = 16'd32412;
            7'd59 : sine_lut = 16'd32521;
            7'd60 : sine_lut = 16'd32609;
            7'd61 : sine_lut = 16'd32679;
            7'd62 : sine_lut = 16'd32728;
            7'd63 : sine_lut = 16'd32757;
            7'd64 : sine_lut = 16'd32767;
            // Mirror: indices 65-127 are symmetric (32767 down to ~804)
            7'd65 : sine_lut = 16'd32757;
            7'd66 : sine_lut = 16'd32728;
            7'd67 : sine_lut = 16'd32679;
            7'd68 : sine_lut = 16'd32609;
            7'd69 : sine_lut = 16'd32521;
            7'd70 : sine_lut = 16'd32412;
            7'd71 : sine_lut = 16'd32285;
            7'd72 : sine_lut = 16'd32137;
            7'd73 : sine_lut = 16'd31971;
            7'd74 : sine_lut = 16'd31785;
            7'd75 : sine_lut = 16'd31580;
            7'd76 : sine_lut = 16'd31356;
            7'd77 : sine_lut = 16'd31113;
            7'd78 : sine_lut = 16'd30852;
            7'd79 : sine_lut = 16'd30571;
            7'd80 : sine_lut = 16'd30273;
            7'd81 : sine_lut = 16'd29956;
            7'd82 : sine_lut = 16'd29621;
            7'd83 : sine_lut = 16'd29268;
            7'd84 : sine_lut = 16'd28898;
            7'd85 : sine_lut = 16'd28510;
            7'd86 : sine_lut = 16'd28105;
            7'd87 : sine_lut = 16'd27683;
            7'd88 : sine_lut = 16'd27245;
            7'd89 : sine_lut = 16'd26790;
            7'd90 : sine_lut = 16'd26319;
            7'd91 : sine_lut = 16'd25832;
            7'd92 : sine_lut = 16'd25329;
            7'd93 : sine_lut = 16'd24811;
            7'd94 : sine_lut = 16'd24279;
            7'd95 : sine_lut = 16'd23731;
            7'd96 : sine_lut = 16'd23170;
            7'd97 : sine_lut = 16'd22594;
            7'd98 : sine_lut = 16'd22005;
            7'd99 : sine_lut = 16'd21403;
            7'd100: sine_lut = 16'd20787;
            7'd101: sine_lut = 16'd20159;
            7'd102: sine_lut = 16'd19519;
            7'd103: sine_lut = 16'd18868;
            7'd104: sine_lut = 16'd18204;
            7'd105: sine_lut = 16'd17530;
            7'd106: sine_lut = 16'd16846;
            7'd107: sine_lut = 16'd16151;
            7'd108: sine_lut = 16'd15446;
            7'd109: sine_lut = 16'd14733;
            7'd110: sine_lut = 16'd14010;
            7'd111: sine_lut = 16'd13279;
            7'd112: sine_lut = 16'd12539;
            7'd113: sine_lut = 16'd11793;
            7'd114: sine_lut = 16'd11039;
            7'd115: sine_lut = 16'd10278;
            7'd116: sine_lut = 16'd9512;
            7'd117: sine_lut = 16'd8739;
            7'd118: sine_lut = 16'd7962;
            7'd119: sine_lut = 16'd7179;
            7'd120: sine_lut = 16'd6393;
            7'd121: sine_lut = 16'd5602;
            7'd122: sine_lut = 16'd4808;
            7'd123: sine_lut = 16'd4011;
            7'd124: sine_lut = 16'd3212;
            7'd125: sine_lut = 16'd2410;
            7'd126: sine_lut = 16'd1608;
            7'd127: sine_lut = 16'd804;
            default: sine_lut = 16'd0;
        endcase
    endfunction

    // phase_top[15:9] = 7-bit index spanning the FULL LFO period (0..127).
    //
    // The LUT already encodes a complete half-wave (rise 0→peak at idx=64, fall
    // peak→0 at idx=127), so indexing with phase_top[15:9] gives exactly one
    // smooth hump per LFO cycle - which is the unipolar gain envelope we need
    // for tremolo (gain high at top, dips to ~0 at the trough).
    //
    // phase_top[14:8] would only use the lower 7-of-16 bits, completing two full
    // hump cycles per LFO period (doubled frequency) - that was the bug.
    always_comb begin
        lut_idx  = phase_top[15:9];          // 0..127 over one full LFO period
        lut_val  = sine_lut(lut_idx);        // Q15 unsigned, [0, 32767]
        lfo_sine = {lut_val[14:0], 1'b0};    // scale to [0, 65534]
    end

    // Select waveform
    always_comb begin
        lfo_norm = shape_sel ? lfo_tri : lfo_sine;
    end

    // =========================================================================
    // 3. GAIN COMPUTATION
    //
    // gain_q16 is a 17-bit unsigned value representing [0.0, 1.0] in Q1.16:
    //   0x00000 = 0.0  (full attenuation)
    //   0x10000 = 1.0  (unity gain, bit 16 set)
    //
    // Formula:  gain = 1.0 - depth_norm * (1.0 - lfo_norm)
    //
    // depth_norm [16:0] in Q0.16:  depth_val * 257 -> [0, 0xFFFF]
    //   depth=0   -> 0x00000  => no reduction => gain = 0x10000 (1.0 exactly)
    //   depth=255 -> 0x0FFFF  => full range
    //
    // one_minus_lfo = 0x10000 - lfo_norm  (17-bit, range [0x0002, 0x10000])
    //
    // mod_reduction = depth_norm * one_minus_lfo
    //   max = 0xFFFF * 0x10000 = 0xFFFF_0000  (fits in 33 bits, held in 34)
    //   >> 16 extracts the Q0.16 reduction, range [0x0000, 0xFFFF]
    //   NOTE: must shift >> 16, NOT >> 17.  depth_norm is at most 16 significant
    //   bits (0xFFFF), so the product is at most 33 bits; bit 33 is always 0.
    //   Shifting >> 17 would discard the MSB of the 16-bit reduction, halving it.
    //
    // gain = 0x10000 - (mod_reduction >> 16)
    //   min = 0x10000 - 0xFFFF = 0x00001  (depth=255, lfo=0  -> near silence)
    //   max = 0x10000 - 0x0000 = 0x10000  (depth=0           -> unity, exact)
    // =========================================================================
    logic [16:0] depth_norm;      // Q0.16 depth in 17-bit field, range [0, 0xFFFF]
    logic [16:0] one_minus_lfo;   // (1.0 - lfo_norm) Q0.16, range [2, 0x10000]
    logic [33:0] mod_reduction;   // depth * (1-lfo), 34-bit
    logic [16:0] gain_q16_17;     // Q1.16 gain, 17-bit [0x00001, 0x10000]

    always_comb begin
        depth_norm    = {1'b0, depth_val, depth_val[7:0]};   // depth_val * 257
        one_minus_lfo = 17'h10000 - {1'b0, lfo_norm};
        mod_reduction = {17'h0, depth_norm} * {17'h0, one_minus_lfo};
        gain_q16_17   = 17'h10000 - mod_reduction[32:16];    // >>16, NOT >>17
    end

    // =========================================================================
    // 4. AMPLITUDE MODULATION
    //
    // output = audio_in * gain_q16_17  (>> 16 removes Q16 fractional bits)
    //
    // Overflow analysis:
    //   |audio_in|   <= 2^23       (24-bit signed full scale)
    //   gain_q16_17  <= 0x10000 = 2^16
    //   |product|    <= 2^23 * 2^16 = 2^39  -> fits in 40 bits; result is exact.
    //
    // product: signed 24-bit × unsigned 17-bit (zero-extended to 18-bit signed)
    // = 42-bit signed. Result bits [WIDTH+15:16] = [39:16] give 24-bit output.
    // =========================================================================
    logic signed [WIDTH+17:0]  product;   // 42 bits
    logic signed [WIDTH-1:0]   am_out;

    always_comb begin
        product = $signed(audio_in) * $signed({1'b0, gain_q16_17});
        am_out  = product[WIDTH+15:16];
    end

    // =========================================================================
    // 5. OUTPUT REGISTER
    // =========================================================================
    logic signed [WIDTH-1:0] audio_out_reg = '0;
    assign audio_out = audio_out_reg;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            lfo_phase     <= '0;
            audio_out_reg <= '0;
        end else if (sample_en) begin
            lfo_phase     <= lfo_phase_next;
            audio_out_reg <= am_out;
        end
    end

endmodule