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
//   3 = dd3            (mono in, wet+dry mix,1-cycle latency)
//   4 = tube_distortion(mono in, mono out,   valid_out-driven, 9-cycle latency)
//   5 = flanger       (mono in, mono out,   1-cycle latency)
//   6 = big_muff      (mono in, mono out,   1-cycle latency)
//
// Register map (per slot, 8 bytes, indexed from cfg_mem[SLOT_ID*CTRL_DEPTH]):
//   tremolo:         [0]=rate  [1]=depth  [2]=shape(bit0)           [7]=bypass(bit0)
//   phaser:          [0]=speed [1]=feedback_en(bit0)                [7]=bypass(bit0)
//   chorus:          [0]=rate  [1]=depth  [2]=effect_lvl [3..4]=eq  [7]=bypass(bit0)
//   dd3:             [0]=tone  [1]=level  [2]=feedback [3..4]=time  [7]=bypass(bit0)
//   tube_distortion: [0]=gain  [1]=bass   [2]=mid [3]=treble [4]=lvl [7]=bypass(bit0)
//   flanger:         [0]=manual [1]=width [2]=speed [3]=regen       [7]=bypass(bit0)
//   big_muff:        [0]=sustain [1]=tone [2]=volume                [7]=bypass(bit0)
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
    parameter int EFFECT_TYPE      = 0,      // 0=trem, 1=pha, 2=cho, 3=dd3, 4=tube, 5=flanger, 6=big_muff
    parameter int DATA_W           = 48,
    parameter int CTRL_DEPTH       = 8,
    parameter int CTRL_W           = 8,
    parameter int AUDIO_W          = 24,
    // Effect-specific parameters
    parameter int CHORUS_DELAY_MAX = 2048,
    parameter int DD3_RAM_DEPTH    = 32768,
    parameter int TUBE_LUT_ADDR    = 12,     // tube: 4096-entry LUT
    parameter int TUBE_FRAC_W      = 12      // tube: interpolation fraction bits
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
          .WIDTH(AUDIO_W)
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

      // 1-cycle latency cores use sample_en_reg delayed by 1
      logic sample_en_d1;
      always_ff @(posedge clk) sample_en_d1 <= (!rst_n) ? 1'b0 : sample_en_reg;
      assign effect_valid = sample_en_d1;

      // ---- PHASER (type 1): mono in → mono out ----
    end else if (EFFECT_TYPE == 1) begin : gen_phaser
      logic signed [AUDIO_W-1:0] core_out;

      phaser #(
          .WIDTH(AUDIO_W)
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

      logic sample_en_d1;
      always_ff @(posedge clk) sample_en_d1 <= (!rst_n) ? 1'b0 : sample_en_reg;
      assign effect_valid = sample_en_d1;

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
          .rate       (cfg_slice[0]),
          .depth      (cfg_slice[1]),
          .effect_lvl (cfg_slice[2]),
          .e_q_hi     (cfg_slice[3]),
          .e_q_lo     (cfg_slice[4])
      );

      logic sample_en_d1;
      always_ff @(posedge clk) sample_en_d1 <= (!rst_n) ? 1'b0 : sample_en_reg;
      assign effect_valid = sample_en_d1;

      // ---- DD3 DELAY (type 3): mono in → wet-only out + dry/wet mix ----
    end else if (EFFECT_TYPE == 3) begin : gen_dd3
      logic signed [AUDIO_W-1:0] wet_out;

      // Reconstruct 16-bit time_val from two consecutive 8-bit registers (LE)
      logic [15:0] time_val_16;
      assign time_val_16 = {cfg_slice[4], cfg_slice[3]};

      dd3 #(
          .WIDTH    (AUDIO_W),
          .RAM_DEPTH(DD3_RAM_DEPTH)
      ) core (
          .clk         (clk),
          .rst_n       (rst_n),
          .sample_en   (sample_en_reg),
          .tone_val    (cfg_slice[0]),
          .level_val   (cfg_slice[1]),
          .feedback_val(cfg_slice[2]),
          .time_val    (time_val_16),
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

      logic sample_en_d1;
      always_ff @(posedge clk) sample_en_d1 <= (!rst_n) ? 1'b0 : sample_en_reg;
      assign effect_valid = sample_en_d1;

      // ---- TUBE DISTORTION (type 4): mono in → mono out, valid_out-driven ----
      //
      // tube_distortion has a 10-cycle pipelined latency and drives its own
      // valid_out strobe, used directly as effect_valid.
      // cfg_slice mapping:
      //   [0]=gain  [1]=tone_bass  [2]=tone_mid  [3]=tone_treble  [4]=level
    end else if (EFFECT_TYPE == 4) begin : gen_tube
      logic signed [AUDIO_W-1:0] core_out;
      logic                      core_valid;

      tube_distortion #(
          .DATA_W  (AUDIO_W),
          .LUT_ADDR(TUBE_LUT_ADDR),
          .FRAC_W  (TUBE_FRAC_W)
      ) core (
          .clk        (clk),
          .rst_n      (rst_n),
          .sample_en  (sample_en_reg),
          .audio_in   (audio_in_reg),
          .audio_out  (core_out),
          .valid_out  (core_valid),
          .gain       (cfg_slice[0]),
          .tone_bass  (cfg_slice[1]),
          .tone_mid   (cfg_slice[2]),
          .tone_treble(cfg_slice[3]),
          .level      (cfg_slice[4])
      );

      assign effect_out_l = core_out;
      assign effect_out_r = core_out;
      assign effect_valid = core_valid;

      // ---- FLANGER (type 5): mono in → mono out ----
    end else if (EFFECT_TYPE == 5) begin : gen_flanger
      logic signed [AUDIO_W-1:0] core_out;

      flanger #(
          .WIDTH    (AUDIO_W),
          .MAX_DELAY(240),
          .MIN_DELAY(12)
      ) core (
          .clk       (clk),
          .rst_n     (rst_n),
          .sample_en (sample_en_reg),
          .audio_in  (audio_in_reg),
          .audio_out (core_out),
          .manual_val(cfg_slice[0]),
          .width_val (cfg_slice[1]),
          .speed_val (cfg_slice[2]),
          .regen_val (cfg_slice[3])
      );

      assign effect_out_l = core_out;
      assign effect_out_r = core_out;

      // 1-cycle latency, same as tremolo / phaser / chorus
      logic sample_en_d1;
      always_ff @(posedge clk) sample_en_d1 <= (!rst_n) ? 1'b0 : sample_en_reg;
      assign effect_valid = sample_en_d1;

      // ---- BIG MUFF (type 6): mono in → mono out ----
    end else if (EFFECT_TYPE == 6) begin : gen_big_muff
      logic signed [AUDIO_W-1:0] core_out;

      big_muff #(
          .WIDTH(AUDIO_W)
      ) core (
          .clk        (clk),
          .rst_n      (rst_n),
          .sample_en  (sample_en_reg),
          .audio_in   (audio_in_reg),
          .audio_out  (core_out),
          .sustain_val(cfg_slice[0]),
          .tone_val   (cfg_slice[1]),
          .volume_val (cfg_slice[2])
      );

      assign effect_out_l = core_out;
      assign effect_out_r = core_out;

      logic sample_en_d1;
      always_ff @(posedge clk) sample_en_d1 <= (!rst_n) ? 1'b0 : sample_en_reg;
      assign effect_valid = sample_en_d1;

      // ---- DEFAULT: passthrough ----
    end else begin : gen_passthrough
      assign effect_out_l = dry_l_hold;
      assign effect_out_r = dry_r_hold;
      logic sample_en_d1;
      always_ff @(posedge clk) sample_en_d1 <= (!rst_n) ? 1'b0 : sample_en_reg;
      assign effect_valid = sample_en_d1;
    end
  endgenerate

  // =====================================================================
  // Output capture + bypass mux
  //
  // All effect types now use effect_valid as the output strobe.
  // Types 0-3 (1-cycle latency) produce effect_valid = sample_en_d1,
  // which is functionally identical to the previous sample_en_d1 logic.
  // Type 4 (tube) uses valid_out from the core's 9-cycle pipeline.
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
  // bypass=1 → forward input directly (registered, 1-cycle latency)
  // bypass=0 → use effect output
  // Effect still runs in bypass to keep LFO phase / delay tails alive.
  //
  // To prevent audible clicks when bypass toggles, a short linear crossfade
  // ramps between dry and wet paths over FADE_LEN samples (~1.3 ms at 48 kHz).

  logic [DATA_W-1:0] byp_data;
  logic              byp_valid;

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

  // Crossfade counter
  localparam int FADE_LEN = 64;            // samples to crossfade
  localparam int FADE_W   = $clog2(FADE_LEN+1); // bits for counter 0..FADE_LEN

  logic [FADE_W-1:0] fade_pos;             // 0 = full bypass, FADE_LEN = full effect
  logic               bypass_prev;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      fade_pos    <= '0;
      bypass_prev <= 1'b1;
    end else begin
      bypass_prev <= bypass;

      // When bypass changes, the fade ramps toward the new target.
      // Only step on sample boundaries (effect_valid or byp_valid) to keep
      // the crossfade in sync with the audio rate.
      if (effect_valid || byp_valid) begin
        if (bypass && fade_pos != '0)
          fade_pos <= fade_pos - 1;
        else if (!bypass && fade_pos != FADE_W'(FADE_LEN))
          fade_pos <= fade_pos + 1;
      end

      // Snap to target if bypass hasn't changed for a full ramp
      // (handles startup: bypass=1 → fade_pos stays 0)
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

  // Output uses whichever valid strobe fires (both paths run in parallel)
  logic              out_valid;
  logic [DATA_W-1:0] out_data;

  assign out_data  = pack_stereo(mix_l, mix_r);
  // During crossfade both paths are blended, so either valid is sufficient.
  // In steady state only one path matters but both are always running.
  assign out_valid = (fade_pos == '0)              ? byp_valid :
                     (fade_pos == FADE_W'(FADE_LEN)) ? efx_valid :
                     (efx_valid | byp_valid);

  assign m_axis_tdata  = out_data;
  assign m_axis_tvalid = out_valid;

endmodule