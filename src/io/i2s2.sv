module i2s2 #(
    parameter int AUDIO_W = 24
) (
    input clk,
    input logic resetn,

    output logic tx_mclk,
    output logic tx_lrclk,
    output logic tx_sclk,
    output logic tx_serial,

    output logic rx_mclk,
    output logic rx_lrclk,
    output logic rx_sclk,
    input  logic rx_serial,

    input  logic [AUDIO_W-1:0] tx_data_l,
    input  logic [AUDIO_W-1:0] tx_data_r,
    output logic [AUDIO_W-1:0] rx_data_l,
    output logic [AUDIO_W-1:0] rx_data_r
);

  logic [8:0] clk_cnt;
  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) clk_cnt <= 0;
    else clk_cnt <= clk_cnt + 1;
  end

  assign tx_mclk  = clk;
  assign tx_sclk  = clk_cnt[1];
  assign tx_lrclk = clk_cnt[7];
  assign rx_mclk  = tx_mclk;
  assign rx_lrclk = tx_lrclk;
  assign rx_sclk  = tx_sclk;

  logic sclk_fall;
  assign sclk_fall = (clk_cnt[1:0] == 2'b11);

  // --- 2. Transmitter (DAC) ---
  logic [AUDIO_W-1:0] shift_tx;
  logic               load_tx;

  assign load_tx = (clk_cnt[6:0] == 7'd3);

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      shift_tx <= 0;
    end else if (load_tx && sclk_fall) begin
      if (tx_lrclk) shift_tx <= tx_data_r;
      else shift_tx <= tx_data_l;
    end else if (sclk_fall) begin
      shift_tx <= {shift_tx[22:0], 1'b0};
    end
  end

  assign tx_serial = shift_tx[23];

  // --- 3. Receiver (ADC) ---
  logic [AUDIO_W-1:0] shift_rx;
  logic [AUDIO_W-1:0] rx_latched_l;
  logic [AUDIO_W-1:0] rx_latched_r;
  logic               latch_rx;

  assign latch_rx = (clk_cnt[6:0] == 7'd99);

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      shift_rx     <= 0;
      rx_latched_l <= 0;
      rx_latched_r <= 0;
    end else begin
      if (sclk_fall) begin
        shift_rx <= {shift_rx[22:0], rx_serial};
      end
      if (latch_rx && sclk_fall) begin
        if (tx_lrclk) rx_latched_r <= shift_rx;
        else rx_latched_l <= shift_rx;
      end
    end
  end

  assign rx_data_l = rx_latched_l;
  assign rx_data_r = rx_latched_r;

endmodule
