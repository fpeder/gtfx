module audio #(
    parameter int AUDIO_W = 24
) (
    input  logic clk_audio,
    input  logic resetn,
    output logic tx_mclk,
    output logic tx_lrclk,
    output logic tx_sclk,
    output logic tx_serial,

    output logic rx_mclk,
    output logic rx_lrclk,
    output logic rx_sclk,
    input  logic rx_serial,

    input  logic [AUDIO_W-1:0] din_l,
    input  logic [AUDIO_W-1:0] din_r,
    output logic [AUDIO_W-1:0] dout_l,
    output logic [AUDIO_W-1:0] dout_r
);

  i2s2 #(
      .AUDIO_W(24)
  ) driver (
      .clk   (clk_audio),
      .resetn(resetn),

      .tx_mclk  (tx_mclk),
      .tx_lrclk (tx_lrclk),
      .tx_sclk  (tx_sclk),
      .tx_serial(tx_serial),
      .tx_data_l(din_l),
      .tx_data_r(din_r),

      .rx_mclk  (rx_mclk),
      .rx_lrclk (rx_lrclk),
      .rx_sclk  (rx_sclk),
      .rx_serial(rx_serial),
      .rx_data_l(dout_l),
      .rx_data_r(dout_r)
  );

endmodule

