// ============================================================================
// delay_line.sv - Generic Delay Line with Exposed Write / Read Pointers
//
// Circular-buffer delay line for audio effect cores.  Supports both
// power-of-2 and non-power-of-2 depths with correct pointer wrapping,
// and both combinational reads (register-file) and registered reads (BRAM).
//
// Read modes (REG_RD):
//   0 - Combinational:  rd_data[i] = mem[rd_ptr[i]]  (same-cycle).
//       Suitable for small delays in distributed RAM / FFs.
//       `rd_en` is ignored; reads are always available.
//
//   1 - Registered:     rd_data[i] latches mem[rd_ptr[i]] on posedge when
//       rd_en is asserted.  One-cycle read latency.  Required for BRAM
//       inference on large buffers.  Caller presents rd_ptr on the same
//       cycle as rd_en; data appears on rd_data one clock later.
//
// Separate wr_en / rd_en strobes support 2-phase pipelines (e.g. DD-3):
//   Phase 1 (rd_en):  Issue BRAM reads.
//   Phase 2 (wr_en):  Read data valid; process and write back.
// For single-phase designs, tie rd_en = wr_en (or leave unconnected when
// REG_RD=0).
//
// Parameters:
//   DATA_W      Sample width (signed).  Default: 24.
//   DEPTH       Buffer depth in samples.  Any positive integer ≥ 2.
//   NUM_TAPS    Number of independent read ports.  Default: 1.
//   REG_RD      0 = combinational reads, 1 = registered reads.  Default: 0.
//   INTERP_EN   Enable built-in linear interpolation between taps 0 and 1.
//               Requires NUM_TAPS >= 2.  Default: 0.
//   FRAC_W      Fractional-delay bits for interpolation.  Default: 10.
//
// Ports:
//   clk, rst_n            Clock and active-low reset.
//   wr_en                 Write-enable strobe.
//   wr_data               Sample to write at current wr_ptr.
//   wr_ptr_o              Current write pointer (where next write goes).
//   wr_ptr_next_o         Next write pointer (combinational, = wr_ptr after
//                         the current write completes).
//   rd_en                 Read-enable strobe (REG_RD=1 only; ignored when 0).
//   rd_ptr[NUM_TAPS]      Read pointers (caller-computed).
//   rd_data[NUM_TAPS]     Read data (combinational or registered).
//   interp_frac           Fractional delay for built-in interpolation.
//   interp_out            Interpolated output = tap[0] + frac*(tap[1]-tap[0]).
//
// Example (chorus - single tap, power-of-2, combinational reads):
//
//   delay_line #(.DATA_W(24), .DEPTH(2048), .NUM_TAPS(1)) u_delay (
//       .clk(clk), .rst_n(rst_n),
//       .wr_en(sample_en), .wr_data(audio_in),
//       .wr_ptr_o(wr_ptr), .wr_ptr_next_o(),
//       .rd_en(1'b0), .rd_ptr('{rd_ptr}), .rd_data('{wet_raw}),
//       .interp_frac('0), .interp_out()
//   );
//
// Example (flanger - two taps + interpolation, non-PoT, combinational):
//
//   delay_line #(.DATA_W(24), .DEPTH(240), .NUM_TAPS(2),
//                .INTERP_EN(1), .FRAC_W(10)) u_delay (
//       .clk(clk), .rst_n(rst_n),
//       .wr_en(sample_en), .wr_data(delay_input_sat),
//       .wr_ptr_o(wr_ptr), .wr_ptr_next_o(),
//       .rd_en(1'b0), .rd_ptr('{rd_ptr_a, rd_ptr_b}),
//       .rd_data('{tap_a, tap_b}),
//       .interp_frac(delay_frac), .interp_out(wet_interp)
//   );
//
// Example (DD-3 - two taps + interpolation, PoT BRAM, registered reads):
//
//   delay_line #(.DATA_W(24), .DEPTH(32768), .NUM_TAPS(2),
//                .REG_RD(1), .INTERP_EN(1), .FRAC_W(8)) u_delay (
//       .clk(clk), .rst_n(rst_n),
//       .wr_en(sample_en_d1), .wr_data(fb_softclip),
//       .wr_ptr_o(wr_ptr), .wr_ptr_next_o(wr_ptr_next),
//       .rd_en(sample_en), .rd_ptr('{rd_ptr_a_next, rd_ptr_b_next}),
//       .rd_data('{ram_read_data_a, ram_read_data_b}),
//       .interp_frac(lfo_frac_d1), .interp_out(interp_out)
//   );
//
// ============================================================================

module delay_line #(
    parameter int DATA_W    = 24,
    parameter int DEPTH     = 2048,
    parameter int NUM_TAPS  = 1,
    parameter int REG_RD    = 0,
    parameter int INTERP_EN = 0,
    parameter int FRAC_W    = 10
) (
    input  logic                          clk,
    input  logic                          rst_n,

    // Write port
    input  logic                          wr_en,
    input  logic signed [DATA_W-1:0]      wr_data,
    output logic [ADDR_W-1:0]             wr_ptr_o,
    output logic [ADDR_W-1:0]             wr_ptr_next_o,

    // Read ports
    input  logic                          rd_en,
    input  logic [ADDR_W-1:0]             rd_ptr   [NUM_TAPS],
    output logic signed [DATA_W-1:0]      rd_data  [NUM_TAPS],

    // Interpolation (active when INTERP_EN=1)
    input  logic [FRAC_W-1:0]            interp_frac,
    output logic signed [DATA_W-1:0]      interp_out
);

    // ====================================================================
    // Derived constants
    // ====================================================================
    localparam int ADDR_W = $clog2(DEPTH);
    localparam bit IS_POT = (DEPTH & (DEPTH - 1)) == 0;

    // ====================================================================
    // Buffer storage
    //
    // The ram_style attribute hints for block RAM when REG_RD=1 (large
    // buffers with registered reads).  Synthesis tools ignore the hint
    // when the array is too small for BRAM, so it is safe to apply
    // unconditionally.
    // ====================================================================
    (* ram_style = "block" *)
    logic signed [DATA_W-1:0] mem [0:DEPTH-1];

    initial begin
        for (int i = 0; i < DEPTH; i++) mem[i] = '0;
    end

    // ====================================================================
    // Write pointer with PoT / non-PoT wrapping
    // ====================================================================
    logic [ADDR_W-1:0] wr_ptr;
    logic [ADDR_W-1:0] wr_ptr_next;

    assign wr_ptr_o      = wr_ptr;
    assign wr_ptr_next_o = wr_ptr_next;

    initial wr_ptr = '0;

    // Next-pointer (combinational, always valid)
    generate if (IS_POT) begin : g_pot_next
        assign wr_ptr_next = wr_ptr + ADDR_W'(1);
    end else begin : g_npot_next
        assign wr_ptr_next = (wr_ptr == ADDR_W'(DEPTH - 1)) ? '0
                                                             : wr_ptr + ADDR_W'(1);
    end endgenerate

    // Pointer advance on write
    always_ff @(posedge clk) begin
        if (!rst_n)
            wr_ptr <= '0;
        else if (wr_en)
            wr_ptr <= wr_ptr_next;
    end

    // Memory write
    always_ff @(posedge clk) begin
        if (wr_en)
            mem[wr_ptr] <= wr_data;
    end

    // ====================================================================
    // Read ports
    // ====================================================================
    genvar t;
    generate
        if (REG_RD) begin : g_reg_rd
            // Registered reads - one-cycle latency, BRAM-compatible.
            for (t = 0; t < NUM_TAPS; t++) begin : g_tap
                initial rd_data[t] = '0;
                always_ff @(posedge clk) begin
                    if (!rst_n)
                        rd_data[t] <= '0;
                    else if (rd_en)
                        rd_data[t] <= mem[rd_ptr[t]];
                end
            end
        end else begin : g_comb_rd
            // Combinational reads - zero latency, distributed RAM / FFs.
            for (t = 0; t < NUM_TAPS; t++) begin : g_tap
                assign rd_data[t] = mem[rd_ptr[t]];
            end
        end
    endgenerate

    // ====================================================================
    // Built-in linear interpolation (optional)
    //
    // interp_out = tap[0] + frac * (tap[1] - tap[0])
    //
    // For REG_RD=1 the caller must present interp_frac aligned with the
    // registered rd_data (i.e. delayed by one cycle if necessary).
    // ====================================================================
    generate if (INTERP_EN && NUM_TAPS >= 2) begin : g_interp

        logic signed [DATA_W:0]          interp_diff;
        logic signed [DATA_W+FRAC_W:0]   interp_prod;

        always_comb begin
            interp_diff = $signed({rd_data[1][DATA_W-1], rd_data[1]})
                        - $signed({rd_data[0][DATA_W-1], rd_data[0]});
            interp_prod = interp_diff * $signed({1'b0, interp_frac});
            interp_out  = rd_data[0] + DATA_W'(interp_prod >>> FRAC_W);
        end

    end else begin : g_no_interp

        assign interp_out = (NUM_TAPS >= 1) ? rd_data[0] : '0;

    end endgenerate

endmodule