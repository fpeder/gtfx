`timescale 1ns / 1ps

// ============================================================================
// reverb.sv - Freeverb (Schroeder-Moorer) Reverb Effect Core
//
// Algorithm: 8 parallel lowpass-feedback comb filters → 4 series allpass
// diffusers, with stereo decorrelation via offset allpass lengths for L/R.
//
// Interface matches the existing effect core pattern:
//   sample_en strobe, signed audio_in, stereo audio_out_l/r, 8-bit controls.
//
// Register map (via cfg_slice from ctrl_bus):
//   [0] = decay     (comb feedback gain: 0x00=short, 0xFF=long)
//   [1] = damping   (HF absorption:     0x00=bright, 0xFF=dark)
//   [2] = mix       (wet amount:        0x00=dry, 0x60=default, 0xFF=full wet added)
//   [3] = pre_dly   (pre-delay:         0x00..0xFF → 0..255 samples)
//   [4] = tone      (output tone:       LP filter cutoff)
//   [5] = level     (output level)
//   [7] = bypass    (bit 0, managed by axis_effect_slot)
//
// Pipeline: single-cycle (combinational delay line reads with REG_RD=0).
//           effect_valid = sample_en_d1 in the slot wrapper.
//
// Resource estimate: ~10 BRAM36 out of 135 available on xc7a100t.
// ============================================================================

module reverb #(
    parameter int DATA_W = 24,
    parameter int CTRL_W = 8
) (
    input  logic                       clk,
    input  logic                       rst_n,
    input  logic                       sample_en,

    // Audio
    input  logic signed [DATA_W-1:0]   audio_in,
    output logic signed [DATA_W-1:0]   audio_out_l,
    output logic signed [DATA_W-1:0]   audio_out_r,

    // Controls from cfg_slice
    input  logic [CTRL_W-1:0]          decay,      // comb feedback gain
    input  logic [CTRL_W-1:0]          damping,    // HF absorption
    input  logic [CTRL_W-1:0]          mix,        // dry/wet
    input  logic [CTRL_W-1:0]          pre_dly,    // pre-delay (samples)
    input  logic [CTRL_W-1:0]          tone,       // output tone
    input  logic [CTRL_W-1:0]          level       // output level
);

  // =========================================================================
  // Constants
  // =========================================================================
  localparam int INT_W  = DATA_W + 4;  // internal headroom
  localparam int N_COMB = 8;
  localparam int N_AP   = 4;

  // Comb filter depths (mutually prime, scaled for 48 kHz)
  localparam int COMB_DEPTHS [N_COMB] = '{1557, 1617, 1491, 1422,
                                           1277, 1356, 1188, 1116};

  // Allpass depths — L channel
  localparam int AP_DEPTHS_L [N_AP] = '{556, 441, 341, 225};
  // Allpass depths — R channel (slightly offset for stereo decorrelation)
  localparam int AP_DEPTHS_R [N_AP] = '{579, 464, 358, 238};

  // Fixed allpass coefficient g = 0.5 in Q0.8 → 128
  localparam logic [CTRL_W-1:0] AP_COEFF = 8'd128;

  // Max depth for delay lines (for address width calculation)
  localparam int COMB_MAX  = 2048;
  localparam int AP_MAX    = 1024;
  localparam int PRE_MAX   = 256;

  localparam int COMB_ADDR_W = $clog2(COMB_MAX);
  localparam int AP_ADDR_W   = $clog2(AP_MAX);
  localparam int PRE_ADDR_W  = $clog2(PRE_MAX);

  // =========================================================================
  // Pre-delay line
  // =========================================================================
  logic [PRE_ADDR_W-1:0] pre_wr_ptr;
  logic [PRE_ADDR_W-1:0] pre_rd_ptr_arr [1];
  logic signed [DATA_W-1:0] pre_rd_data [1];
  logic signed [DATA_W-1:0] pre_out;

  always_comb begin
    // pre_dly=0 → 0 delay, pre_dly=FF → 255 sample delay
    pre_rd_ptr_arr[0] = pre_wr_ptr - PRE_ADDR_W'(pre_dly);
  end

  delay_line #(
      .DATA_W  (DATA_W),
      .DEPTH   (PRE_MAX),
      .NUM_TAPS(1),
      .REG_RD  (0)
  ) u_pre_delay (
      .clk          (clk),
      .rst_n        (rst_n),
      .wr_en        (sample_en),
      .wr_data      (audio_in),
      .wr_ptr_o     (pre_wr_ptr),
      .wr_ptr_next_o(),
      .rd_en        (1'b0),
      .rd_ptr       (pre_rd_ptr_arr),
      .rd_data      (pre_rd_data),
      .interp_frac  ('0),
      .interp_out   ()
  );

  assign pre_out = pre_rd_data[0];

  // =========================================================================
  // 8 Parallel Comb Filters with LP damping in feedback path
  //
  // Each comb: out = buf[wr_ptr - M]
  //            buf[wr_ptr] = in + feedback * lpf(out)
  //
  // 1-pole LP damping: lpf_state = (1 - damp) * out + damp * lpf_state_prev
  //   where damp = damping / 256  (Q0.8)
  // =========================================================================
  logic signed [DATA_W-1:0] comb_out [N_COMB];

  // Feedback gain: decay maps 0..255 → ~0.5..0.98 range
  // feedback = 0.5 + decay * 0.48 / 256   in Q0.8: 128 + decay * 123 / 256
  logic [CTRL_W:0] feedback_q8;  // Q0.9 to hold up to ~250
  always_comb begin
    feedback_q8 = 9'(128) + 9'((16'(decay) * 16'd123) >> 8);
  end

  genvar ci;
  generate
    for (ci = 0; ci < N_COMB; ci++) begin : gen_comb
      localparam int DEPTH_I = COMB_DEPTHS[ci];
      localparam int ADDR_W_I = $clog2(DEPTH_I);

      logic [ADDR_W_I-1:0] c_wr_ptr;
      logic [ADDR_W_I-1:0] c_wr_ptr_next;
      logic [ADDR_W_I-1:0] c_rd_ptr_arr [1];
      logic signed [DATA_W-1:0] c_rd_data [1];
      logic signed [DATA_W-1:0] c_buf_out;

      // Read pointer: oldest sample (use wr_ptr_next for correct non-PoT wrap)
      always_comb begin
        c_rd_ptr_arr[0] = c_wr_ptr_next;
      end

      // 1-pole LP damping state
      logic signed [DATA_W-1:0] lpf_state;

      // Feedback input (LP-filtered comb output scaled by feedback)
      logic signed [DATA_W-1:0] lpf_out;
      logic signed [INT_W-1:0]  fb_scaled;
      logic signed [DATA_W-1:0] fb_sat;
      logic signed [DATA_W-1:0] wr_sample;
      logic signed [DATA_W:0]   comb_wr_sum;

      always_comb begin
        c_buf_out = c_rd_data[0];

        // 1-pole LP: lpf = (1 - damp/256) * buf_out + (damp/256) * lpf_state
        // Q1.23 result
        lpf_out = DATA_W'(
          (longint'(c_buf_out) * longint'(9'(256) - 9'(damping)) +
           longint'(lpf_state) * longint'({1'b0, damping})) >>> 8
        );

        // Scale by feedback gain:  Q1.23 × Q0.9 >>> 8 → Q1.23
        fb_scaled = INT_W'(
          (longint'(lpf_out) * longint'(signed'({1'b0, feedback_q8}))) >>> 8
        );
      end

      saturate #(
          .IN_W (INT_W),
          .OUT_W(DATA_W)
      ) u_comb_sat (
          .din (fb_scaled),
          .dout(fb_sat)
      );

      // Write: input + feedback (saturate to prevent overflow)
      assign comb_wr_sum = (DATA_W+1)'(longint'(pre_out) + longint'(fb_sat));

      saturate #(
          .IN_W (DATA_W+1),
          .OUT_W(DATA_W)
      ) u_comb_wr_sat (
          .din (comb_wr_sum),
          .dout(wr_sample)
      );

      delay_line #(
          .DATA_W  (DATA_W),
          .DEPTH   (DEPTH_I),
          .NUM_TAPS(1),
          .REG_RD  (0)
      ) u_comb_dl (
          .clk          (clk),
          .rst_n        (rst_n),
          .wr_en        (sample_en),
          .wr_data      (wr_sample),
          .wr_ptr_o     (c_wr_ptr),
          .wr_ptr_next_o(c_wr_ptr_next),
          .rd_en        (1'b0),
          .rd_ptr       (c_rd_ptr_arr),
          .rd_data      (c_rd_data),
          .interp_frac  ('0),
          .interp_out   ()
      );

      // Update LP damping state
      always_ff @(posedge clk) begin
        if (!rst_n)
          lpf_state <= '0;
        else if (sample_en)
          lpf_state <= lpf_out;
      end

      assign comb_out[ci] = c_buf_out;
    end
  endgenerate

  // =========================================================================
  // Sum comb outputs (÷8 to prevent overflow)
  // =========================================================================
  logic signed [DATA_W-1:0] comb_sum;

  always_comb begin
    automatic longint acc = 0;
    for (int i = 0; i < N_COMB; i++)
      acc = acc + longint'(comb_out[i]);
    comb_sum = DATA_W'(acc >>> 3);  // ÷8
  end

  // =========================================================================
  // 4 Series Allpass Diffusers (L channel)
  //
  // Each allpass: buf_out = buf[n-M]
  //               out = -g*in + buf_out
  //               buf[n] = in + g*buf_out
  // g = 0.5 (AP_COEFF/256)
  // =========================================================================
  logic signed [DATA_W-1:0] ap_chain_l [N_AP+1];
  assign ap_chain_l[0] = comb_sum;

  genvar ai;
  generate
    for (ai = 0; ai < N_AP; ai++) begin : gen_ap_l
      localparam int DEPTH_I = AP_DEPTHS_L[ai];
      localparam int ADDR_W_I = $clog2(DEPTH_I);

      logic [ADDR_W_I-1:0] a_wr_ptr;
      logic [ADDR_W_I-1:0] a_wr_ptr_next;
      logic [ADDR_W_I-1:0] a_rd_ptr_arr [1];
      logic signed [DATA_W-1:0] a_rd_data [1];
      logic signed [DATA_W-1:0] a_buf_out;
      logic signed [DATA_W-1:0] ap_out;
      logic signed [DATA_W-1:0] ap_wr_data;

      always_comb begin
        a_rd_ptr_arr[0] = a_wr_ptr_next;
        a_buf_out = a_rd_data[0];

        // out = -g*in + buf_out  (g = 0.5 → shift right 1)
        ap_out = DATA_W'(
          -longint'(ap_chain_l[ai]) / 2 + longint'(a_buf_out)
        );

        // buf[n] = in + g*buf_out
        ap_wr_data = DATA_W'(
          longint'(ap_chain_l[ai]) + longint'(a_buf_out) / 2
        );
      end

      delay_line #(
          .DATA_W  (DATA_W),
          .DEPTH   (DEPTH_I),
          .NUM_TAPS(1),
          .REG_RD  (0)
      ) u_ap_dl (
          .clk          (clk),
          .rst_n        (rst_n),
          .wr_en        (sample_en),
          .wr_data      (ap_wr_data),
          .wr_ptr_o     (a_wr_ptr),
          .wr_ptr_next_o(a_wr_ptr_next),
          .rd_en        (1'b0),
          .rd_ptr       (a_rd_ptr_arr),
          .rd_data      (a_rd_data),
          .interp_frac  ('0),
          .interp_out   ()
      );

      assign ap_chain_l[ai+1] = ap_out;
    end
  endgenerate

  // =========================================================================
  // 4 Series Allpass Diffusers (R channel — offset depths)
  // =========================================================================
  logic signed [DATA_W-1:0] ap_chain_r [N_AP+1];
  assign ap_chain_r[0] = comb_sum;

  generate
    for (ai = 0; ai < N_AP; ai++) begin : gen_ap_r
      localparam int DEPTH_I = AP_DEPTHS_R[ai];
      localparam int ADDR_W_I = $clog2(DEPTH_I);

      logic [ADDR_W_I-1:0] a_wr_ptr;
      logic [ADDR_W_I-1:0] a_wr_ptr_next;
      logic [ADDR_W_I-1:0] a_rd_ptr_arr [1];
      logic signed [DATA_W-1:0] a_rd_data [1];
      logic signed [DATA_W-1:0] a_buf_out;
      logic signed [DATA_W-1:0] ap_out;
      logic signed [DATA_W-1:0] ap_wr_data;

      always_comb begin
        a_rd_ptr_arr[0] = a_wr_ptr_next;
        a_buf_out = a_rd_data[0];

        ap_out = DATA_W'(
          -longint'(ap_chain_r[ai]) / 2 + longint'(a_buf_out)
        );

        ap_wr_data = DATA_W'(
          longint'(ap_chain_r[ai]) + longint'(a_buf_out) / 2
        );
      end

      delay_line #(
          .DATA_W  (DATA_W),
          .DEPTH   (DEPTH_I),
          .NUM_TAPS(1),
          .REG_RD  (0)
      ) u_ap_dl (
          .clk          (clk),
          .rst_n        (rst_n),
          .wr_en        (sample_en),
          .wr_data      (ap_wr_data),
          .wr_ptr_o     (a_wr_ptr),
          .wr_ptr_next_o(a_wr_ptr_next),
          .rd_en        (1'b0),
          .rd_ptr       (a_rd_ptr_arr),
          .rd_data      (a_rd_data),
          .interp_frac  ('0),
          .interp_out   ()
      );

      assign ap_chain_r[ai+1] = ap_out;
    end
  endgenerate

  // =========================================================================
  // Output tone filter (1-pole LP controlled by tone knob)
  //
  // Simple 1-pole: y = (1 - tone/256)*x + (tone/256)*y_prev
  // tone=0x00 → no filtering (bright), tone=0xFF → heavy LP (dark)
  // =========================================================================
  logic signed [DATA_W-1:0] wet_l, wet_r;
  logic signed [DATA_W-1:0] tone_l_state, tone_r_state;

  always_comb begin
    wet_l = DATA_W'(
      (longint'(ap_chain_l[N_AP]) * longint'(9'(256) - 9'(tone)) +
       longint'(tone_l_state)     * longint'({1'b0, tone})) >>> 8
    );
    wet_r = DATA_W'(
      (longint'(ap_chain_r[N_AP]) * longint'(9'(256) - 9'(tone)) +
       longint'(tone_r_state)     * longint'({1'b0, tone})) >>> 8
    );
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      tone_l_state <= '0;
      tone_r_state <= '0;
    end else if (sample_en) begin
      tone_l_state <= wet_l;
      tone_r_state <= wet_r;
    end
  end

  // =========================================================================
  // Level scaling
  // =========================================================================
  logic signed [INT_W-1:0] lvl_l, lvl_r;
  logic signed [DATA_W-1:0] lvl_l_sat, lvl_r_sat;

  always_comb begin
    // Q1.23 × Q0.8 >>> 7  (level=0x80 → unity gain)
    lvl_l = INT_W'((longint'(wet_l) * longint'({1'b0, level})) >>> 7);
    lvl_r = INT_W'((longint'(wet_r) * longint'({1'b0, level})) >>> 7);
  end

  saturate #(.IN_W(INT_W), .OUT_W(DATA_W)) u_sat_l (.din(lvl_l), .dout(lvl_l_sat));
  saturate #(.IN_W(INT_W), .OUT_W(DATA_W)) u_sat_r (.din(lvl_r), .dout(lvl_r_sat));

  // =========================================================================
  // Dry/wet mix (additive — body always preserved)
  //
  // mix=0x00: pure dry
  // mix=0x60: dry + 37.5% wet (default)
  // mix=0xFF: dry + ~100% wet (maximum reverb)
  // =========================================================================
  logic signed [INT_W-1:0] mix_l_raw, mix_r_raw;
  logic signed [DATA_W-1:0] mix_l_sat, mix_r_sat;

  always_comb begin
    mix_l_raw = INT_W'(signed'(audio_in)) + INT_W'(
      (longint'(lvl_l_sat) * longint'(mix)
       + longint'(1 << (CTRL_W-1))) >>> CTRL_W
    );
    mix_r_raw = INT_W'(signed'(audio_in)) + INT_W'(
      (longint'(lvl_r_sat) * longint'(mix)
       + longint'(1 << (CTRL_W-1))) >>> CTRL_W
    );
  end

  saturate #(.IN_W(INT_W), .OUT_W(DATA_W)) u_sat_ml (.din(mix_l_raw), .dout(mix_l_sat));
  saturate #(.IN_W(INT_W), .OUT_W(DATA_W)) u_sat_mr (.din(mix_r_raw), .dout(mix_r_sat));

  // =========================================================================
  // Registered output (1-cycle latency)
  // =========================================================================
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      audio_out_l <= '0;
      audio_out_r <= '0;
    end else if (sample_en) begin
      audio_out_l <= mix_l_sat;
      audio_out_r <= mix_r_sat;
    end
  end

endmodule
