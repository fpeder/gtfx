module top (
    input logic sys_clk,
    input logic resetn,
    //
    output logic i2s2_tx_mclk,
    output logic i2s2_tx_lrclk,
    output logic i2s2_tx_sclk,
    output logic i2s2_tx,
    output logic i2s2_rx_mclk,
    output logic i2s2_rx_lrclk,
    output logic i2s2_rx_sclk,
    input logic i2s2_rx,
    //
    input logic uart_tx_din,
    output logic uart_rx_dout,
    //
    input logic [1:0] btns,
    output logic [3:0] led,
    //
    // Effect enables (independent, one bit per effect):
    //   sw[0] = tremolo enable
    //   sw[1] = phaser  enable
    //   sw[2] = chorus  enable
    //   sw[3] = delay   enable
    // Signal chain order: dry -> tremolo -> phaser -> chorus -> delay -> out
    input logic [3:0] sw_effect
);

  // ---------------------------------------------------------------------------
  // Command processor (sys_clk domain)
  // ---------------------------------------------------------------------------
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
  ) cmd (
      .sys_clk(sys_clk),
      .rst_n(resetn),
      .tx_din(uart_tx_din),
      .rx_dout(uart_rx_dout),
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

  // ---------------------------------------------------------------------------
  // Audio clock (12.288 MHz)
  // ---------------------------------------------------------------------------
  logic clk_audio, locked;

  clk_wiz_0 clk_wiz (
      .clk_in1 (sys_clk),
      .resetn  (resetn),
      .locked  (locked),
      .clk_out1(clk_audio)
  );

  // ---------------------------------------------------------------------------
  // CDC: sys_clk -> clk_audio (double-flop)
  // ---------------------------------------------------------------------------
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

  // ---------------------------------------------------------------------------
  // I2S driver (clk_audio domain)
  // ---------------------------------------------------------------------------
  logic [23:0] din_l, din_r;
  logic [23:0] dout_l, dout_r;

  audio audio_inst (
      .clk_audio(clk_audio),
      .resetn(resetn),
      .tx_mclk(i2s2_tx_mclk),
      .tx_lrclk(i2s2_tx_lrclk),
      .tx_sclk(i2s2_tx_sclk),
      .tx_serial(i2s2_tx),
      .rx_mclk(i2s2_rx_mclk),
      .rx_lrclk(i2s2_rx_lrclk),
      .rx_sclk(i2s2_rx_sclk),
      .rx_serial(i2s2_rx),
      .din_l(din_l),
      .din_r(din_r),
      .dout_l(dout_l),
      .dout_r(dout_r)
  );

  // ---------------------------------------------------------------------------
  // sample_en: lrclk rising-edge detect, clk_audio domain
  // ---------------------------------------------------------------------------
  logic lrclk_prev;
  logic sample_en;

  always_ff @(posedge clk_audio) begin
    lrclk_prev <= i2s2_tx_lrclk;
  end
  assign sample_en = i2s2_tx_lrclk & ~lrclk_prev;

  // ===========================================================================
  // EFFECT CHAIN:  dry -> [tremolo] -> [phaser] -> [chorus] -> [delay] -> out
  //
  // Each stage has a bypass mux controlled by its sw_audio bit.
  // Tremolo is mono (same gain applied to both L and R at the end).
  // The chain carries L; R branches off after chorus for stereo spread.
  // ===========================================================================

  // ---- Stage 1 : Tremolo (mono; sw_audio[0]) --------------------------------
  logic signed [23:0] trem_out;

  tremolo #(
      .WIDTH(24)
  ) tremolo_inst (
      .clk      (clk_audio),
      .rst_n    (resetn),
      .sample_en(sample_en),
      .rate_val (trem_rate_audio),
      .depth_val(trem_depth_audio),
      .shape_sel(trem_shape_audio),
      .audio_in (dout_l),
      .audio_out(trem_out)
  );

  // sw[0]=1 -> insert tremolo; sw[0]=0 -> bypass
  logic signed [23:0] post_trem;
  assign post_trem = sw_audio[0] ? trem_out : dout_l;

  // ---- Stage 2 : Phaser (mono; sw_audio[1]) ---------------------------------
  logic signed [23:0] phaser_out;

  phaser #(
      .WIDTH(24)
  ) phaser_inst (
      .clk        (clk_audio),
      .rst_n      (resetn),
      .sample_en  (sample_en),
      .speed_val  (pha_speed_audio),
      .feedback_en(pha_fben_audio),
      .audio_in   (post_trem),
      .audio_out  (phaser_out)
  );

  // sw[1]=1 -> insert phaser; sw[1]=0 -> bypass
  logic signed [23:0] post_phaser;
  assign post_phaser = sw_audio[1] ? phaser_out : post_trem;

  // ---- Stage 3 : Chorus (mono in, stereo out; sw_audio[2]) -----------------
  logic signed [23:0] chorus_out_l, chorus_out_r;

  chorus #(
      .SAMPLE_RATE(48_000),
      .DATA_WIDTH (24),
      .DELAY_MAX  (512)
  ) chorus_inst (
      .clk        (clk_audio),
      .rst_n      (resetn),
      .sample_en  (sample_en),
      .audio_in   (post_phaser),
      .audio_out_l(chorus_out_l),
      .audio_out_r(chorus_out_r),
      .rate       (ch_rate_audio),
      .depth      (ch_depth_audio),
      .effect_lvl (ch_efx_audio),
      .e_q_hi     (ch_eqhi_audio),
      .e_q_lo     (ch_eqlo_audio)
  );

  // sw[2]=1 -> insert chorus; sw[2]=0 -> bypass (R mirrors L)
  logic signed [23:0] post_chorus_l, post_chorus_r;
  assign post_chorus_l = sw_audio[2] ? chorus_out_l : post_phaser;
  assign post_chorus_r = sw_audio[2] ? chorus_out_r : post_phaser;

  // ---- Stage 4 : DD3 Delay (wet-only output; sw_audio[3]) ------------------
  logic signed [23:0] delay_wet;

  dd3 #(
      .WIDTH(24)
  ) delay_inst (
      .clk         (clk_audio),
      .rst_n       (resetn),
      .sample_en   (sample_en),
      .tone_val    (tone_val_audio),
      .level_val   (level_val_audio),
      .feedback_val(fb_val_audio),
      .time_val    (time_val_audio),
      .audio_in    (post_chorus_l),
      .audio_out   (delay_wet)
  );

  // Mix wet echo with post-chorus signal (stereo, saturating)
  logic signed [24:0] dly_sum_l, dly_sum_r;
  logic signed [23:0] delay_mixed_l, delay_mixed_r;

  always_comb begin
    dly_sum_l = $signed({post_chorus_l[23], post_chorus_l})
              + $signed({delay_wet[23],     delay_wet});
    dly_sum_r = $signed({post_chorus_r[23], post_chorus_r})
              + $signed({delay_wet[23],     delay_wet});

    delay_mixed_l = (dly_sum_l > 25'sh7FFFFF)  ?  24'sh7FFFFF :
                    (dly_sum_l < -25'sh800000)  ? -24'sh800000 :
                     dly_sum_l[23:0];
    delay_mixed_r = (dly_sum_r > 25'sh7FFFFF)  ?  24'sh7FFFFF :
                    (dly_sum_r < -25'sh800000)  ? -24'sh800000 :
                     dly_sum_r[23:0];
  end

  // sw[3]=1 -> insert delay; sw[3]=0 -> bypass
  logic signed [23:0] post_delay_l, post_delay_r;
  assign post_delay_l = sw_audio[3] ? delay_mixed_l : post_chorus_l;
  assign post_delay_r = sw_audio[3] ? delay_mixed_r : post_chorus_r;

  // ---------------------------------------------------------------------------
  // Final output
  // When all switches off -> full dry bypass on both channels
  // ---------------------------------------------------------------------------
  always_comb begin
    if (sw_audio == 4'b0000) begin
      din_l = dout_l;
      din_r = dout_r;
    end else begin
      din_l = post_delay_l;
      din_r = post_delay_r;
    end
  end

  // ---------------------------------------------------------------------------
  // LED indicators
  //   led[0] = tremolo active
  //   led[1] = phaser  active
  //   led[2] = chorus  active
  //   led[3] = delay   active
  // ---------------------------------------------------------------------------
  assign led[0] = sw_effect[0];  // tremolo
  assign led[1] = sw_effect[1];  // phaser
  assign led[2] = sw_effect[2];  // chorus
  assign led[3] = sw_effect[3];  // delay

endmodule
