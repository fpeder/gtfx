// ============================================================================
// top.sv — AXI-Stream Audio Effects Pipeline
//
// Signal chain:  ADC → [tremolo] → [phaser] → [chorus] → [delay] → DAC
//
// Every inter-module link uses AXI-Stream (tdata/tvalid/tready).
// Audio bus: tdata[47:0] = { left[23:0], right[23:0] }
// UART bus:  tdata[7:0]  = one byte  (inside cmd.sv)
//
// Each effect wrapper instantiates the ORIGINAL UNCHANGED core internally.
// ============================================================================

module top (
    input logic sys_clk,
    input logic resetn,

    // I2S
    output logic i2s2_tx_mclk,
    output logic i2s2_tx_lrclk,
    output logic i2s2_tx_sclk,
    output logic i2s2_tx,
    output logic i2s2_rx_mclk,
    output logic i2s2_rx_lrclk,
    output logic i2s2_rx_sclk,
    input  logic i2s2_rx,

    // UART
    input  logic uart_tx_din,
    output logic uart_rx_dout,

    // Buttons & LEDs
    input  logic [1:0] btns,
    output logic [3:0] led,

    // Effect enables
    input logic [3:0] sw_effect
);

  import axis_audio_pkg::*;

  // ===========================================================================
  // Command processor (sys_clk domain)
  // ===========================================================================
  logic [ 7:0] tone_val;
  logic [ 7:0] level_val;
  logic [ 7:0] feedback_val;
  logic [31:0] time_val;
  logic [ 7:0] chorus_rate_val;
  logic [ 7:0] chorus_depth_val;
  logic [ 7:0] chorus_efx_val;
  logic [ 7:0] chorus_eqhi_val;
  logic [ 7:0] chorus_eqlo_val;
  logic [ 7:0] phaser_speed_val;
  logic        phaser_fben_val;
  logic [ 7:0] trem_rate_val;
  logic [ 7:0] trem_depth_val;
  logic        trem_shape_val;

  cmd #(
      .CLK_FREQ (100_000_000),
      .BAUD_RATE(115_200)
  ) cmd_inst (
      .sys_clk(sys_clk),
      .rst_n(resetn),
      .tx_din(uart_tx_din),
      .rx_dout(uart_rx_dout),
      .sw_effect(sw_effect),
      .tone_val(tone_val),
      .level_val(level_val),
      .feedback_val(feedback_val),
      .time_val(time_val),
      .chorus_rate_val(chorus_rate_val),
      .chorus_depth_val(chorus_depth_val),
      .chorus_efx_val(chorus_efx_val),
      .chorus_eqhi_val(chorus_eqhi_val),
      .chorus_eqlo_val(chorus_eqlo_val),
      .phaser_speed_val(phaser_speed_val),
      .phaser_fben_val(phaser_fben_val),
      .trem_rate_val(trem_rate_val),
      .trem_depth_val(trem_depth_val),
      .trem_shape_val(trem_shape_val)
  );

  // ===========================================================================
  // Audio clock (12.288 MHz)
  // ===========================================================================
  logic clk_audio, locked;

  clk_wiz_0 clk_wiz (
      .clk_in1 (sys_clk),
      .resetn  (resetn),
      .locked  (locked),
      .clk_out1(clk_audio)
  );

  // ===========================================================================
  // CDC: sys_clk → clk_audio (double-flop)
  // ===========================================================================
  logic [7:0]  tone_val_s,       tone_val_audio;
  logic [7:0]  level_val_s,      level_val_audio;
  logic [7:0]  fb_val_s,         fb_val_audio;
  logic [31:0] time_val_s,       time_val_audio;
  logic [7:0]  ch_rate_s,        ch_rate_audio;
  logic [7:0]  ch_depth_s,       ch_depth_audio;
  logic [7:0]  ch_efx_s,         ch_efx_audio;
  logic [7:0]  ch_eqhi_s,        ch_eqhi_audio;
  logic [7:0]  ch_eqlo_s,        ch_eqlo_audio;
  logic [7:0]  pha_speed_s,      pha_speed_audio;
  logic        pha_fben_s,       pha_fben_audio;
  logic [7:0]  trem_rate_s,      trem_rate_audio;
  logic [7:0]  trem_depth_s,     trem_depth_audio;
  logic        trem_shape_s,     trem_shape_audio;
  logic [3:0]  sw_s,             sw_audio;

  always_ff @(posedge clk_audio) begin
    tone_val_s      <= tone_val;        tone_val_audio   <= tone_val_s;
    level_val_s     <= level_val;       level_val_audio  <= level_val_s;
    fb_val_s        <= feedback_val;    fb_val_audio     <= fb_val_s;
    time_val_s      <= time_val;        time_val_audio   <= time_val_s;
    ch_rate_s       <= chorus_rate_val; ch_rate_audio    <= ch_rate_s;
    ch_depth_s      <= chorus_depth_val;ch_depth_audio   <= ch_depth_s;
    ch_efx_s        <= chorus_efx_val;  ch_efx_audio     <= ch_efx_s;
    ch_eqhi_s       <= chorus_eqhi_val; ch_eqhi_audio    <= ch_eqhi_s;
    ch_eqlo_s       <= chorus_eqlo_val; ch_eqlo_audio    <= ch_eqlo_s;
    pha_speed_s     <= phaser_speed_val;pha_speed_audio  <= pha_speed_s;
    pha_fben_s      <= phaser_fben_val; pha_fben_audio   <= pha_fben_s;
    trem_rate_s     <= trem_rate_val;   trem_rate_audio  <= trem_rate_s;
    trem_depth_s    <= trem_depth_val;  trem_depth_audio <= trem_depth_s;
    trem_shape_s    <= trem_shape_val;  trem_shape_audio <= trem_shape_s;
    sw_s            <= sw_effect;       sw_audio         <= sw_s;
  end

  // ===========================================================================
  // AXI-Stream Audio I/O (wraps i2s2 internally)
  // ===========================================================================
  logic [47:0] axis_src_tdata;
  logic        axis_src_tvalid;
  logic        axis_src_tready;

  logic [47:0] axis_sink_tdata;
  logic        axis_sink_tvalid;
  logic        axis_sink_tready;

  audio_axis #(.AUDIO_W(24)) audio_inst (
      .clk_audio    (clk_audio),
      .resetn       (resetn),
      .tx_mclk      (i2s2_tx_mclk),
      .tx_lrclk     (i2s2_tx_lrclk),
      .tx_sclk      (i2s2_tx_sclk),
      .tx_serial    (i2s2_tx),
      .rx_mclk      (i2s2_rx_mclk),
      .rx_lrclk     (i2s2_rx_lrclk),
      .rx_sclk      (i2s2_rx_sclk),
      .rx_serial    (i2s2_rx),
      .m_axis_tdata (axis_src_tdata),
      .m_axis_tvalid(axis_src_tvalid),
      .m_axis_tready(axis_src_tready),
      .s_axis_tdata (axis_sink_tdata),
      .s_axis_tvalid(axis_sink_tvalid),
      .s_axis_tready(axis_sink_tready)
  );

  // ===========================================================================
  //  EFFECT CHAIN:  src → [tremolo] → [phaser] → [chorus] → [delay] → sink
  // ===========================================================================

  // ---- Stage 0: Tremolo (sw_audio[0]) ------------------------------------
  logic [47:0] axis_efx0_tdata;
  logic        axis_efx0_tvalid, axis_efx0_tready;
  logic [47:0] axis_stg0_tdata;
  logic        axis_stg0_tvalid, axis_stg0_tready;
  logic        axis_byp0_tready;

  tremolo_axis tremolo_inst (
      .clk            (clk_audio),
      .rst_n          (resetn),
      .s_axis_tdata   (axis_src_tdata),
      .s_axis_tvalid  (axis_src_tvalid),
      .s_axis_tready  (),
      .m_axis_tdata   (axis_efx0_tdata),
      .m_axis_tvalid  (axis_efx0_tvalid),
      .m_axis_tready  (axis_efx0_tready),
      .rate_val       (trem_rate_audio),
      .depth_val      (trem_depth_audio),
      .shape_sel      (trem_shape_audio)
  );

  axis_bypass_mux mux0 (
      .enable            (sw_audio[0]),
      .s_axis_byp_tdata  (axis_src_tdata),
      .s_axis_byp_tvalid (axis_src_tvalid),
      .s_axis_byp_tready (axis_byp0_tready),
      .s_axis_efx_tdata  (axis_efx0_tdata),
      .s_axis_efx_tvalid (axis_efx0_tvalid),
      .s_axis_efx_tready (axis_efx0_tready),
      .m_axis_tdata      (axis_stg0_tdata),
      .m_axis_tvalid     (axis_stg0_tvalid),
      .m_axis_tready     (axis_stg0_tready)
  );

  assign axis_src_tready = sw_audio[0] ? 1'b1 : axis_byp0_tready;

  // ---- Stage 1: Phaser (sw_audio[1]) ------------------------------------
  logic [47:0] axis_efx1_tdata;
  logic        axis_efx1_tvalid, axis_efx1_tready;
  logic [47:0] axis_stg1_tdata;
  logic        axis_stg1_tvalid, axis_stg1_tready;
  logic        axis_byp1_tready;

  phaser_axis phaser_inst (
      .clk            (clk_audio),
      .rst_n          (resetn),
      .s_axis_tdata   (axis_stg0_tdata),
      .s_axis_tvalid  (axis_stg0_tvalid),
      .s_axis_tready  (),
      .m_axis_tdata   (axis_efx1_tdata),
      .m_axis_tvalid  (axis_efx1_tvalid),
      .m_axis_tready  (axis_efx1_tready),
      .speed_val      (pha_speed_audio),
      .feedback_en    (pha_fben_audio)
  );

  axis_bypass_mux mux1 (
      .enable            (sw_audio[1]),
      .s_axis_byp_tdata  (axis_stg0_tdata),
      .s_axis_byp_tvalid (axis_stg0_tvalid),
      .s_axis_byp_tready (axis_byp1_tready),
      .s_axis_efx_tdata  (axis_efx1_tdata),
      .s_axis_efx_tvalid (axis_efx1_tvalid),
      .s_axis_efx_tready (axis_efx1_tready),
      .m_axis_tdata      (axis_stg1_tdata),
      .m_axis_tvalid     (axis_stg1_tvalid),
      .m_axis_tready     (axis_stg1_tready)
  );

  assign axis_stg0_tready = sw_audio[1] ? 1'b1 : axis_byp1_tready;

  // ---- Stage 2: Chorus (sw_audio[2]) ------------------------------------
  logic [47:0] axis_efx2_tdata;
  logic        axis_efx2_tvalid, axis_efx2_tready;
  logic [47:0] axis_stg2_tdata;
  logic        axis_stg2_tvalid, axis_stg2_tready;
  logic        axis_byp2_tready;

  chorus_axis #(
      .SAMPLE_RATE(48_000),
      .DATA_WIDTH (24),
      .DELAY_MAX  (512)
  ) chorus_inst (
      .clk            (clk_audio),
      .rst_n          (resetn),
      .s_axis_tdata   (axis_stg1_tdata),
      .s_axis_tvalid  (axis_stg1_tvalid),
      .s_axis_tready  (),
      .m_axis_tdata   (axis_efx2_tdata),
      .m_axis_tvalid  (axis_efx2_tvalid),
      .m_axis_tready  (axis_efx2_tready),
      .rate           (ch_rate_audio),
      .depth          (ch_depth_audio),
      .effect_lvl     (ch_efx_audio),
      .e_q_hi         (ch_eqhi_audio),
      .e_q_lo         (ch_eqlo_audio)
  );

  axis_bypass_mux mux2 (
      .enable            (sw_audio[2]),
      .s_axis_byp_tdata  (axis_stg1_tdata),
      .s_axis_byp_tvalid (axis_stg1_tvalid),
      .s_axis_byp_tready (axis_byp2_tready),
      .s_axis_efx_tdata  (axis_efx2_tdata),
      .s_axis_efx_tvalid (axis_efx2_tvalid),
      .s_axis_efx_tready (axis_efx2_tready),
      .m_axis_tdata      (axis_stg2_tdata),
      .m_axis_tvalid     (axis_stg2_tvalid),
      .m_axis_tready     (axis_stg2_tready)
  );

  assign axis_stg1_tready = sw_audio[2] ? 1'b1 : axis_byp2_tready;

  // ---- Stage 3: DD3 Delay (sw_audio[3]) ---------------------------------
  logic [47:0] axis_efx3_tdata;
  logic        axis_efx3_tvalid, axis_efx3_tready;
  logic [47:0] axis_stg3_tdata;
  logic        axis_stg3_tvalid, axis_stg3_tready;
  logic        axis_byp3_tready;

  dd3_axis delay_inst (
      .clk            (clk_audio),
      .rst_n          (resetn),
      .s_axis_tdata   (axis_stg2_tdata),
      .s_axis_tvalid  (axis_stg2_tvalid),
      .s_axis_tready  (),
      .m_axis_tdata   (axis_efx3_tdata),
      .m_axis_tvalid  (axis_efx3_tvalid),
      .m_axis_tready  (axis_efx3_tready),
      .tone_val       (tone_val_audio),
      .level_val      (level_val_audio),
      .feedback_val   (fb_val_audio),
      .time_val       (time_val_audio)
  );

  axis_bypass_mux mux3 (
      .enable            (sw_audio[3]),
      .s_axis_byp_tdata  (axis_stg2_tdata),
      .s_axis_byp_tvalid (axis_stg2_tvalid),
      .s_axis_byp_tready (axis_byp3_tready),
      .s_axis_efx_tdata  (axis_efx3_tdata),
      .s_axis_efx_tvalid (axis_efx3_tvalid),
      .s_axis_efx_tready (axis_efx3_tready),
      .m_axis_tdata      (axis_stg3_tdata),
      .m_axis_tvalid     (axis_stg3_tvalid),
      .m_axis_tready     (axis_stg3_tready)
  );

  assign axis_stg2_tready = sw_audio[3] ? 1'b1 : axis_byp3_tready;

  // ===========================================================================
  // Pipeline output → DAC
  // ===========================================================================
  assign axis_sink_tdata  = axis_stg3_tdata;
  assign axis_sink_tvalid = axis_stg3_tvalid;
  assign axis_stg3_tready = axis_sink_tready;

  // ===========================================================================
  // LEDs
  // ===========================================================================
  assign led[0] = sw_effect[0];
  assign led[1] = sw_effect[1];
  assign led[2] = sw_effect[2];
  assign led[3] = sw_effect[3];

endmodule
