// ============================================================================
// waveform_lut.sv - Generic Synthesis-Time Waveform LUT
//
// Produces a compile-time lookup table for common LFO / modulation waveforms.
// All values are computed at elaboration - pure combinational readout, no
// runtime initialisation, no inferred RAM.
//
// Supported waveforms (WAVE_TYPE parameter):
//   "SINE"      - Quarter-wave symmetric sine,  signed output  [-PEAK, +PEAK]
//   "TRIANGLE"  - Linear triangle,               signed output  [-PEAK, +PEAK]
//   "SAWTOOTH"  - Rising ramp,                   signed output  [-PEAK, +PEAK]
//   "SQUARE"    - 50 % duty square,              signed output  {+PEAK, -PEAK}
//
// The module accepts a PHASE_W-bit phase accumulator value and outputs the
// corresponding waveform sample.  The upper TABLE_BITS bits select the table
// entry; remaining lower bits are exposed as `frac_out` for optional linear
// interpolation by the caller.
//
// For "SINE", only a quarter-wave table (TABLE_SIZE/4 entries) is stored
// internally and the remaining three quadrants are reconstructed from symmetry,
// saving 75 % of the LUT resources.
//
// Parameters:
//   TABLE_BITS  log2 of the number of effective table entries per cycle.
//               TABLE_SIZE = 2**TABLE_BITS.  Typical: 8 (256 entries).
//   OUT_WIDTH   Bit-width of each signed output sample.  Typical: 16.
//   WAVE_TYPE   Selects the waveform shape.
//   PHASE_W     Width of the external phase accumulator.  Typical: 32.
//   FRAC_W      (Derived - do not override.)
//
// ============================================================================

module waveform_lut #(
    parameter int    TABLE_BITS = 8,
    parameter int    OUT_WIDTH  = 16,
    parameter string WAVE_TYPE  = "SINE",
    parameter int    PHASE_W    = 32,
    // Derived - do not override
    parameter int    FRAC_W     = (PHASE_W > TABLE_BITS) ? (PHASE_W - TABLE_BITS) : 1
) (
    input  logic [PHASE_W-1:0]          phase_in,
    output logic signed [OUT_WIDTH-1:0] wave_out,
    output logic [FRAC_W-1:0]           frac_out
);

    // ====================================================================
    // Derived constants
    // ====================================================================
    localparam int TABLE_SIZE = 1 << TABLE_BITS;
    localparam int PEAK       = (1 << (OUT_WIDTH - 1)) - 1;   // +full-scale

    // Quarter-wave constants (sine)
    localparam int QTR_BITS = TABLE_BITS - 2;
    localparam int QTR_SIZE = 1 << QTR_BITS;                  // TABLE_SIZE / 4

    // ====================================================================
    // Phase decomposition
    // ====================================================================
    logic [TABLE_BITS-1:0] tbl_idx;

    assign tbl_idx = phase_in[PHASE_W-1 -: TABLE_BITS];

    generate
        if (PHASE_W > TABLE_BITS) begin : g_frac_real
            assign frac_out = phase_in[PHASE_W-TABLE_BITS-1 : 0];
        end else begin : g_frac_zero
            assign frac_out = '0;
        end
    endgenerate

    // ====================================================================
    //  SINE - quarter-wave LUT with symmetry reconstruction
    // ====================================================================
    generate if (WAVE_TYPE == "SINE") begin : g_sine

        function automatic logic [OUT_WIDTH-2:0] calc_qtr_sine(int i);
            real angle, val;
            angle = 3.14159265358979323846 * real'(i) / real'(2 * QTR_SIZE);
            val   = $sin(angle) * real'(PEAK);
            return (OUT_WIDTH-1)'(int'(val + 0.5));
        endfunction

        logic [OUT_WIDTH-2:0] SINE_QTR [0:QTR_SIZE-1];

        initial begin
            for (int i = 0; i < QTR_SIZE; i++)
                SINE_QTR[i] = calc_qtr_sine(i);
        end

        //   Q0 (00): +LUT[ idx                ]
        //   Q1 (01): +LUT[ QTR_SIZE-1 - idx   ]   (mirror)
        //   Q2 (10): -LUT[ idx                ]   (negate)
        //   Q3 (11): -LUT[ QTR_SIZE-1 - idx   ]   (mirror + negate)

        logic [1:0]                      quadrant;
        logic [QTR_BITS-1:0]             sub_idx;
        logic [QTR_BITS-1:0]             rom_addr;
        logic [OUT_WIDTH-2:0]            rom_val;
        logic signed [OUT_WIDTH-1:0]     sine_val;

        assign quadrant = tbl_idx[TABLE_BITS-1 : TABLE_BITS-2];
        assign sub_idx  = tbl_idx[QTR_BITS-1:0];

        always_comb begin
            rom_addr = quadrant[0] ? (QTR_BITS'(QTR_SIZE - 1) - sub_idx)
                                   : sub_idx;
            rom_val  = SINE_QTR[rom_addr];
            sine_val = quadrant[1] ? -$signed({1'b0, rom_val})
                                   :  $signed({1'b0, rom_val});
        end

        assign wave_out = sine_val;

    // ====================================================================
    //  TRIANGLE - computed directly from phase bits (no ROM)
    // ====================================================================
    end else if (WAVE_TYPE == "TRIANGLE") begin : g_triangle

        logic                        half;
        logic [TABLE_BITS-1:0]       fold_idx;
        logic signed [OUT_WIDTH-1:0] tri_val;

        assign half = tbl_idx[TABLE_BITS-1];

        always_comb begin
            fold_idx = half ? ~tbl_idx : tbl_idx;
            tri_val  = OUT_WIDTH'(-PEAK + ((PEAK * 2 * int'({1'b0, fold_idx})) / (TABLE_SIZE / 2)));
        end

        assign wave_out = tri_val;

    // ====================================================================
    //  SAWTOOTH - phase bits are the waveform (no ROM)
    // ====================================================================
    end else if (WAVE_TYPE == "SAWTOOTH") begin : g_sawtooth

        logic signed [OUT_WIDTH-1:0] saw_val;

        always_comb begin
            saw_val = OUT_WIDTH'(-PEAK + (PEAK * 2 * int'({1'b0, tbl_idx})) / TABLE_SIZE);
        end

        assign wave_out = saw_val;

    // ====================================================================
    //  SQUARE - single bit (no ROM)
    // ====================================================================
    end else if (WAVE_TYPE == "SQUARE") begin : g_square

        assign wave_out = tbl_idx[TABLE_BITS-1] ? -OUT_WIDTH'(PEAK) : OUT_WIDTH'(PEAK);

    // ====================================================================
    //  Default - tie off to zero
    // ====================================================================
    end else begin : g_unknown
        // synopsys translate_off
        initial $warning("waveform_lut: unknown WAVE_TYPE '%s' - output tied to 0", WAVE_TYPE);
        // synopsys translate_on
        assign wave_out = '0;
    end
    endgenerate

endmodule