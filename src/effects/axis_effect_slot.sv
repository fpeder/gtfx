`timescale 1ns / 1ps

// ============================================================================
// axis_effect_slot.sv - Unified AXI-Stream effect slot
//
// Generic wrapper providing:
//   - AXI-Stream slave (input) and master (output) with 48-bit stereo tdata
//   - Parameters read directly from shared cfg_mem (no local register file)
//   - sample_en / audio_in / audio_out bridge to the original effect core
//   - Bypass mux (bypass=1 → straight passthrough, effect still clocked)
//   - Parameterised effect selection via EFFECT_TYPE
//
// EFFECT_TYPE values:
//   0 = tremolo        (mono in, mono out,   1-cycle latency)
//   1 = phaser         (mono in, mono out,   1-cycle latency)
//   2 = chorus         (mono in, stereo out, 1-cycle latency)
//   3 = delay          (mono in, dry/wet crossfade, 3-cycle latency)
//   4 = flanger       (mono in, mono out,   1-cycle latency)
//   5 = reverb        (mono in, stereo out, 6-cycle pipeline)
//   6 = compressor    (mono in, mono out,   1-cycle latency)
//   7 = wah           (mono in, mono out,   1-cycle latency)
//
// Register map (per slot, 8 bytes, indexed from cfg_mem[SLOT_ID*CTRL_DEPTH]):
//   tremolo:         [0]=rate  [1]=depth  [2]=shape(bit0)           [7]=bypass(bit0)
//   phaser:          [0]=speed [1]=feedback_en(bit0)                [7]=bypass(bit0)
//   chorus:          [0]=rate  [1]=depth  [2]=effect_lvl [3..4]=eq  [7]=bypass(bit0)
//   delay:           [0]=rpt [1]=mix [2]=flt [3..4]=time [5]=mod [6]=grit+mode [7]=bypass(bit0)
//   flanger:         [0]=manual [1]=width [2]=speed [3]=regen [4]=mix [7]=bypass(bit0)
//   reverb:          [0]=decay [1]=damping [2]=mix [3]=pre_dly [4]=tone [5]=level [7]=bypass(bit0)
//   compressor:      [0]=threshold [1]=ratio  [2]=attack  [3]=release [4]=makeup [7]=bypass(bit0)
//   wah:             [0]=freq [1]=resonance [2]=depth [3]=mode [4]=mix [7]=bypass(bit0)
//
// bypass is read directly from cfg_slice[7][0].  The separate bypass input port
// has been removed; ctrl_bus writes the bypass bit into cfg_mem[slot*CTRL_DEPTH+7].
//
// Key change vs previous version
// --------------------------------
// The local register file (regs[]) and the ctrl_wr/ctrl_addr/ctrl_wdata
// port trio have been removed.  This slot now receives a slice of the
// shared cfg_mem from ctrl_bus:
//
//   input logic [CTRL_W-1:0] cfg_slice [CTRL_DEPTH]
//
// ctrl_bus drives cfg_slice = cfg_mem[SLOT_ID*CTRL_DEPTH +: CTRL_DEPTH].
// Parameters are read directly from cfg_slice - no local copy or alias.
// Parameters are always up-to-date one cycle after a ctrl_bus write.
// ============================================================================

module axis_effect_slot #(
    parameter int SLOT_ID          = 0,
    parameter int EFFECT_TYPE      = 0,      // 0=tremolo, 1=phaser, 2=chorus, 3=delay, 4=flanger, 5=reverb, 6=compressor, 7=wah
    parameter int DATA_W           = 48,
    parameter int CTRL_DEPTH       = 8,
    parameter int CTRL_W           = 8,
    parameter int AUDIO_W          = 24,
    // Effect-specific parameters
    parameter int CHORUS_DELAY_MAX = 2048,
    parameter int DELAY_RAM_DEPTH  = 65536
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

    // ---- Shared config slice (replaces ctrl_wr/ctrl_addr/ctrl_wdata) ----
    // Driven by ctrl_bus: cfg_slice = cfg_mem[SLOT_ID*CTRL_DEPTH +: CTRL_DEPTH]
    input logic [CTRL_W-1:0] cfg_slice[CTRL_DEPTH]
);

  import axis_audio_pkg::*;

  // bypass is stored in cfg_slice[7][0] - written by ctrl_bus when
  // "set <efx> on/off" is received; no separate port needed.
  wire bypass = cfg_slice[7][0];

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
  // Shared sample_en delay (1-cycle latency strobe)
  //
  // All effect types use this as effect_valid.
  // Hoisted here to avoid duplicating the identical logic in every branch.
  // =====================================================================
  logic sample_en_d1;
  always_ff @(posedge clk) sample_en_d1 <= (!rst_n) ? 1'b0 : sample_en_reg;

  // =====================================================================
  // Effect core (selected by EFFECT_TYPE parameter)
  // =====================================================================
  logic signed [AUDIO_W-1:0] effect_out_l;
  logic signed [AUDIO_W-1:0] effect_out_r;
  logic                      effect_valid;  // strobe when effect_out_* is ready

  generate
    // ---- TREMOLO (type 0): mono in → mono out ----
    if (EFFECT_TYPE == 0) begin : gen_tremolo
      logic signed [AUDIO_W-1:0] core_out;

      tremolo #(
          .WIDTH (AUDIO_W),
          .CTRL_W(CTRL_W)
      ) core (
          .clk      (clk),
          .rst_n    (rst_n),
          .sample_en(sample_en_reg),
          .rate_val (cfg_slice[0]),
          .depth_val(cfg_slice[1]),
          .shape_sel(cfg_slice[2][0]),
          .audio_in (audio_in_reg),
          .audio_out(core_out)
      );

      assign effect_out_l = core_out;
      assign effect_out_r = core_out;
      assign effect_valid = sample_en_d1;

      // ---- PHASER (type 1): mono in → mono out ----
    end else if (EFFECT_TYPE == 1) begin : gen_phaser
      logic signed [AUDIO_W-1:0] core_out;

      phaser #(
          .WIDTH (AUDIO_W),
          .CTRL_W(CTRL_W)
      ) core (
          .clk        (clk),
          .rst_n      (rst_n),
          .sample_en  (sample_en_reg),
          .speed_val  (cfg_slice[0]),
          .feedback_en(cfg_slice[1][0]),
          .audio_in   (audio_in_reg),
          .audio_out  (core_out)
      );

      assign effect_out_l = core_out;
      assign effect_out_r = core_out;
      assign effect_valid = sample_en_d1;

      // ---- CHORUS (type 2): mono in → stereo out ----
    end else if (EFFECT_TYPE == 2) begin : gen_chorus
      chorus #(
          .DATA_WIDTH(AUDIO_W),
          .DELAY_MAX (CHORUS_DELAY_MAX),
          .CTRL_W    (CTRL_W)
      ) core (
          .clk        (clk),
          .rst_n      (rst_n),
          .sample_en  (sample_en_reg),
          .audio_in   (audio_in_reg),
          .audio_out_l(effect_out_l),
          .audio_out_r(effect_out_r),
          .rate       (cfg_slice[0]),
          .depth      (cfg_slice[1]),
          .effect_lvl (cfg_slice[2]),
          .e_q_hi     (cfg_slice[3]),
          .e_q_lo     (cfg_slice[4])
      );

      assign effect_valid = sample_en_d1;

      // ---- TIMELINE DELAY (type 3): mono in → dry/wet crossfade ----
    end else if (EFFECT_TYPE == 3) begin : gen_delay
      logic signed [AUDIO_W-1:0] core_out;

      // Reconstruct 16-bit time_val from two consecutive 8-bit registers (LE)
      logic [15:0] time_val_16;
      assign time_val_16 = {cfg_slice[4], cfg_slice[3]};

      delay #(
          .AUDIO_W     (AUDIO_W),
          .RAM_DEPTH   (DELAY_RAM_DEPTH)
      ) core (
          .clk         (clk),
          .rst_n       (rst_n),
          .sample_en   (sample_en_reg),
          .repeats_val (cfg_slice[0]),
          .mix_val     (cfg_slice[1]),
          .filter_val  (cfg_slice[2]),
          .time_val    (time_val_16),
          .mod_val     (cfg_slice[5]),
          .grit_val    (cfg_slice[6]),
          .audio_in    (audio_in_reg),
          .audio_out   (core_out)
      );

      assign effect_out_l = core_out;
      assign effect_out_r = core_out;
      assign effect_valid = sample_en_d1;

      // ---- FLANGER (type 4): mono in → mono out ----
    end else if (EFFECT_TYPE == 4) begin : gen_flanger
      logic signed [AUDIO_W-1:0] core_out;

      flanger #(
          .WIDTH    (AUDIO_W),
          .CTRL_W   (CTRL_W),
          .MAX_DELAY(256),
          .MIN_DELAY(16)
      ) core (
          .clk       (clk),
          .rst_n     (rst_n),
          .sample_en (sample_en_reg),
          .audio_in  (audio_in_reg),
          .audio_out (core_out),
          .manual_val(cfg_slice[0]),
          .width_val (cfg_slice[1]),
          .speed_val (cfg_slice[2]),
          .regen_val (cfg_slice[3]),
          .mix_val   (cfg_slice[4])
      );

      assign effect_out_l = core_out;
      assign effect_out_r = core_out;
      assign effect_valid = sample_en_d1;

      // ---- REVERB (type 5): mono in → stereo out, 6-cycle pipeline ----
    end else if (EFFECT_TYPE == 5) begin : gen_reverb
      logic reverb_valid;

      reverb #(
          .DATA_W(AUDIO_W),
          .CTRL_W(CTRL_W)
      ) core (
          .clk        (clk),
          .rst_n      (rst_n),
          .sample_en  (sample_en_reg),
          .audio_in   (audio_in_reg),
          .audio_out_l(effect_out_l),
          .audio_out_r(effect_out_r),
          .valid_out  (reverb_valid),
          .decay      (cfg_slice[0]),
          .damping    (cfg_slice[1]),
          .mix        (cfg_slice[2]),
          .pre_dly    (cfg_slice[3]),
          .tone       (cfg_slice[4]),
          .level      (cfg_slice[5])
      );

      assign effect_valid = reverb_valid;

      // ---- COMPRESSOR (type 6): mono in → mono out ----
    end else if (EFFECT_TYPE == 6) begin : gen_compressor
      logic signed [AUDIO_W-1:0] core_out;

      compressor #(
          .DATA_W(AUDIO_W),
          .CTRL_W(CTRL_W)
      ) core (
          .clk           (clk),
          .rst_n         (rst_n),
          .sample_en     (sample_en_reg),
          .audio_in      (audio_in_reg),
          .audio_out     (core_out),
          .threshold_val (cfg_slice[0]),
          .ratio_val     (cfg_slice[1]),
          .attack_val    (cfg_slice[2]),
          .release_val   (cfg_slice[3]),
          .makeup_val    (cfg_slice[4])
      );

      assign effect_out_l = core_out;
      assign effect_out_r = core_out;
      assign effect_valid = sample_en_d1;

      // ---- WAH (type 7): mono in → mono out ----
    end else if (EFFECT_TYPE == 7) begin : gen_wah
      logic signed [AUDIO_W-1:0] core_out;

      wah #(
          .DATA_W(AUDIO_W),
          .CTRL_W(CTRL_W)
      ) core (
          .clk            (clk),
          .rst_n          (rst_n),
          .sample_en      (sample_en_reg),
          .audio_in       (audio_in_reg),
          .audio_out      (core_out),
          .freq_val       (cfg_slice[0]),
          .resonance_val  (cfg_slice[1]),
          .depth_val      (cfg_slice[2]),
          .mode_val       (cfg_slice[3]),
          .mix_val        (cfg_slice[4]),
          .decay_val      (cfg_slice[5])
      );

      assign effect_out_l = core_out;
      assign effect_out_r = core_out;
      assign effect_valid = sample_en_d1;

      // ---- DEFAULT: passthrough ----
    end else begin : gen_passthrough
      assign effect_out_l = dry_l_hold;
      assign effect_out_r = dry_r_hold;
      assign effect_valid = sample_en_d1;
    end
  endgenerate

  // =====================================================================
  // Output capture + bypass mux
  //
  // All effect types use effect_valid = sample_en_d1 as the output strobe.
  // =====================================================================
  logic [DATA_W-1:0] efx_data;
  logic              efx_valid;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      efx_data  <= '0;
      efx_valid <= 1'b0;
    end else begin
      if (efx_valid && m_axis_tready) efx_valid <= 1'b0;

      if (effect_valid) begin
        efx_data  <= pack_stereo(effect_out_l, effect_out_r);
        efx_valid <= 1'b1;
      end
    end
  end

  // ---- Bypass mux with crossfade ----
  // bypass=1 → forward input directly
  // bypass=0 → use effect output
  // Effect still runs in bypass to keep LFO phase / delay tails alive.
  //
  // To prevent audible clicks when bypass toggles, a short linear crossfade
  // ramps between dry and wet paths over FADE_LEN samples (~1.3 ms at 48 kHz).
  //
  // Timing: efx_valid is the sole output strobe in all modes (bypass, active,
  // crossfade). The effect always runs, so efx_valid fires once per sample
  // regardless of bypass state. The bypass path just holds the dry data for
  // the crossfade mix — no independent valid strobe. This prevents the
  // double-valid-pulse that would occur if bypass and effect valid strobes
  // fired at different times during crossfade.

  logic [DATA_W-1:0] byp_data;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      byp_data <= '0;
    end else if (beat_in) begin
      byp_data <= s_axis_tdata;
    end
  end

  // Crossfade counter
  localparam int FADE_LEN = 64;            // samples to crossfade
  localparam int FADE_W   = $clog2(FADE_LEN+1); // bits for counter 0..FADE_LEN

  logic [FADE_W-1:0] fade_pos;             // 0 = full bypass, FADE_LEN = full effect

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      fade_pos <= '0;
    end else if (effect_valid) begin
      // Step on effect_valid only (once per sample) to keep crossfade
      // in sync with the audio rate.
      if (bypass && fade_pos != '0)
        fade_pos <= fade_pos - 1;
      else if (!bypass && fade_pos != FADE_W'(FADE_LEN))
        fade_pos <= fade_pos + 1;
    end
  end

  // Crossfade mix: out = (byp * (FADE_LEN - fade_pos) + efx * fade_pos) / FADE_LEN
  // Applied per-channel to the packed stereo data.

  logic signed [AUDIO_W-1:0] byp_l, byp_r, efx_l, efx_r;
  logic signed [AUDIO_W-1:0] mix_l, mix_r;

  always_comb begin
    byp_l = unpack_left(byp_data);
    byp_r = unpack_right(byp_data);
    efx_l = unpack_left(efx_data);
    efx_r = unpack_right(efx_data);

    // When fully bypassed (fade_pos==0) or fully active (fade_pos==FADE_LEN),
    // the multiply reduces to identity - no rounding error in steady state.
    mix_l = AUDIO_W'(
      (longint'(byp_l) * longint'(FADE_LEN - int'(fade_pos)) +
       longint'(efx_l) * longint'(fade_pos)) / longint'(FADE_LEN));
    mix_r = AUDIO_W'(
      (longint'(byp_r) * longint'(FADE_LEN - int'(fade_pos)) +
       longint'(efx_r) * longint'(fade_pos)) / longint'(FADE_LEN));
  end

  assign m_axis_tdata  = pack_stereo(mix_l, mix_r);
  assign m_axis_tvalid = efx_valid;

endmodule
