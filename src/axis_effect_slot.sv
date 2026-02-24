`timescale 1ns / 1ps

// ============================================================================
// axis_effect_slot.sv — Unified AXI-Stream effect slot
//
// Generic wrapper providing:
//   - AXI-Stream slave (input) and master (output) with 48-bit stereo tdata
//   - Local register file (written by ctrl_bus)
//   - sample_en / audio_in / audio_out bridge to the original effect core
//   - Bypass mux (bypass=1 → straight passthrough, effect still clocked)
//   - Parameterised effect selection via EFFECT_TYPE
//
// EFFECT_TYPE values:
//   0 = tremolo  (mono in, mono out)
//   1 = phaser   (mono in, mono out)
//   2 = chorus   (mono in, stereo out)
//   3 = dd3      (mono in, wet-only out + dry/wet mix)
//
// Register map (per slot, 8 bytes):
//   tremolo: [0]=rate  [1]=depth  [2]=shape(bit0)
//   phaser:  [0]=speed [1]=feedback_en(bit0)
//   chorus:  [0]=rate  [1]=depth  [2]=effect_lvl [3]=eq_hi [4]=eq_lo
//   dd3:     [0]=tone  [1]=level  [2]=feedback   [3..6]=time(32-bit LE)
// ============================================================================

module axis_effect_slot #(
    parameter int SLOT_ID = 0,
    parameter int EFFECT_TYPE = 0,  // 0=trem, 1=pha, 2=cho, 3=dd3
    parameter int DATA_W = 48,
    parameter int CTRL_DEPTH = 8,
    parameter int CTRL_W = 8,
    parameter int AUDIO_W = 24,
    // Effect-specific parameters
    parameter int CHORUS_DELAY_MAX = 512,
    parameter int DD3_RAM_DEPTH = 48000,
    // Default register values at reset, packed: {reg7, reg6, ..., reg1, reg0}
    // Bit layout: INIT_PACKED[7:0]=regs[0], [15:8]=regs[1], ..., [63:56]=regs[7]
    parameter logic [CTRL_DEPTH*CTRL_W-1:0] INIT_PACKED = '0
) (
    input logic clk,
    input logic rst_n,

    // AXI-Stream slave (input)
    input  logic [DATA_W-1:0] s_axis_tdata,
    input  logic              s_axis_tvalid,
    output logic              s_axis_tready,

    // AXI-Stream master (output)
    output logic [DATA_W-1:0] m_axis_tdata,
    output logic              m_axis_tvalid,
    input  logic              m_axis_tready,

    // Control register write port (from ctrl_bus)
    input logic                          ctrl_wr,
    input logic [$clog2(CTRL_DEPTH)-1:0] ctrl_addr,
    input logic [            CTRL_W-1:0] ctrl_wdata,

    // Bypass
    input logic bypass
);

  import axis_audio_pkg::*;

  // =====================================================================
  // Local register file
  // =====================================================================
  logic [CTRL_W-1:0] regs[CTRL_DEPTH];

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      for (int i = 0; i < CTRL_DEPTH; i++) regs[i] <= INIT_PACKED[i*CTRL_W+:CTRL_W];
    end else if (ctrl_wr) begin
      regs[ctrl_addr] <= ctrl_wdata;
    end
  end

  // =====================================================================
  // AXI-Stream → sample_en bridge
  // =====================================================================
  assign s_axis_tready = 1'b1;
  wire                       beat_in = s_axis_tvalid & s_axis_tready;

  logic                      sample_en_reg;
  logic signed [AUDIO_W-1:0] audio_in_reg;
  logic signed [AUDIO_W-1:0] dry_l_hold, dry_r_hold;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      sample_en_reg <= 1'b0;
      audio_in_reg  <= '0;
      dry_l_hold    <= '0;
      dry_r_hold    <= '0;
    end else begin
      sample_en_reg <= beat_in;
      if (beat_in) begin
        audio_in_reg <= unpack_left(s_axis_tdata);
        dry_l_hold   <= unpack_left(s_axis_tdata);
        dry_r_hold   <= unpack_right(s_axis_tdata);
      end
    end
  end

  // =====================================================================
  // Effect core (selected by EFFECT_TYPE parameter)
  // =====================================================================
  logic signed [AUDIO_W-1:0] effect_out_l;
  logic signed [AUDIO_W-1:0] effect_out_r;

  generate
    // ---- TREMOLO (type 0): mono in → mono out ----
    if (EFFECT_TYPE == 0) begin : gen_tremolo
      logic signed [AUDIO_W-1:0] core_out;

      tremolo #(
          .WIDTH(AUDIO_W)
      ) core (
          .clk      (clk),
          .rst_n    (rst_n),
          .sample_en(sample_en_reg),
          .rate_val (regs[0]),
          .depth_val(regs[1]),
          .shape_sel(regs[2][0]),
          .audio_in (audio_in_reg),
          .audio_out(core_out)
      );

      assign effect_out_l = core_out;
      assign effect_out_r = core_out;

      // ---- PHASER (type 1): mono in → mono out ----
    end else if (EFFECT_TYPE == 1) begin : gen_phaser
      logic signed [AUDIO_W-1:0] core_out;

      phaser #(
          .WIDTH(AUDIO_W)
      ) core (
          .clk        (clk),
          .rst_n      (rst_n),
          .sample_en  (sample_en_reg),
          .speed_val  (regs[0]),
          .feedback_en(regs[1][0]),
          .audio_in   (audio_in_reg),
          .audio_out  (core_out)
      );

      assign effect_out_l = core_out;
      assign effect_out_r = core_out;

      // ---- CHORUS (type 2): mono in → stereo out ----
    end else if (EFFECT_TYPE == 2) begin : gen_chorus
      chorus #(
          .SAMPLE_RATE(48_000),
          .DATA_WIDTH (AUDIO_W),
          .DELAY_MAX  (CHORUS_DELAY_MAX)
      ) core (
          .clk        (clk),
          .rst_n      (rst_n),
          .sample_en  (sample_en_reg),
          .audio_in   (audio_in_reg),
          .audio_out_l(effect_out_l),
          .audio_out_r(effect_out_r),
          .rate       (regs[0]),
          .depth      (regs[1]),
          .effect_lvl (regs[2]),
          .e_q_hi     (regs[3]),
          .e_q_lo     (regs[4])
      );

      // ---- DD3 DELAY (type 3): mono in → wet-only out + dry/wet mix ----
    end else if (EFFECT_TYPE == 3) begin : gen_dd3
      logic signed [AUDIO_W-1:0] wet_out;

      // Reconstruct 32-bit time_val from 4 consecutive 8-bit registers (LE)
      logic [31:0] time_val_32;
      assign time_val_32 = {regs[6], regs[5], regs[4], regs[3]};

      dd3 #(
          .WIDTH    (AUDIO_W),
          .RAM_DEPTH(DD3_RAM_DEPTH)
      ) core (
          .clk         (clk),
          .rst_n       (rst_n),
          .sample_en   (sample_en_reg),
          .tone_val    (regs[0]),
          .level_val   (regs[1]),
          .feedback_val(regs[2]),
          .time_val    (time_val_32),
          .audio_in    (audio_in_reg),
          .audio_out   (wet_out)
      );

      // Dry + wet saturating mix
      logic signed [AUDIO_W:0] sum_l, sum_r;
      always_comb begin
        sum_l = $signed({dry_l_hold[AUDIO_W-1], dry_l_hold}) +
            $signed({wet_out[AUDIO_W-1], wet_out});
        sum_r = $signed({dry_r_hold[AUDIO_W-1], dry_r_hold}) +
            $signed({wet_out[AUDIO_W-1], wet_out});
      end
      assign effect_out_l = saturate(sum_l);
      assign effect_out_r = saturate(sum_r);

      // ---- DEFAULT: passthrough ----
    end else begin : gen_passthrough
      assign effect_out_l = dry_l_hold;
      assign effect_out_r = dry_r_hold;
    end
  endgenerate

  // =====================================================================
  // Output capture + bypass mux
  // =====================================================================
  logic              sample_en_d1;
  logic [DATA_W-1:0] efx_data;
  logic              efx_valid;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      sample_en_d1 <= 1'b0;
      efx_data     <= '0;
      efx_valid    <= 1'b0;
    end else begin
      sample_en_d1 <= sample_en_reg;

      if (efx_valid && m_axis_tready) efx_valid <= 1'b0;

      if (sample_en_d1) begin
        efx_data  <= pack_stereo(effect_out_l, effect_out_r);
        efx_valid <= 1'b1;
      end
    end
  end

  // ---- Bypass mux ----
  // bypass=1 → forward input directly (combinational, zero latency)
  // bypass=0 → use effect output (registered, 2-cycle latency)
  //
  // When bypassed, the effect still runs (keeping LFO phase etc. alive)
  // but its output is not selected.

  logic [DATA_W-1:0] byp_data;
  logic              byp_valid;

  // Hold bypass data for one cycle to align with beat_in
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      byp_data  <= '0;
      byp_valid <= 1'b0;
    end else begin
      if (byp_valid && m_axis_tready) byp_valid <= 1'b0;
      if (beat_in) begin
        byp_data  <= s_axis_tdata;
        byp_valid <= 1'b1;
      end
    end
  end

  assign m_axis_tdata  = bypass ? byp_data : efx_data;
  assign m_axis_tvalid = bypass ? byp_valid : efx_valid;

endmodule

