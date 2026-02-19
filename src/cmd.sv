`timescale 1ns / 1ps

module cmd #(
    parameter int CLK_FREQ  = 100_000_000,
    parameter int BAUD_RATE = 115_200
) (
    input logic sys_clk,
    input logic rst_n,
    input logic tx_din,
    output logic rx_dout,
    // DD3 delay controls
    output logic [7:0] tone_val,
    output logic [7:0] level_val,
    output logic [7:0] feedback_val,
    output logic [31:0] time_val,
    // CE-5 chorus controls
    output logic [7:0] chorus_rate_val,
    output logic [7:0] chorus_depth_val,
    output logic [7:0] chorus_efx_val,
    output logic [7:0] chorus_eqhi_val,
    output logic [7:0] chorus_eqlo_val,
    // Phaser controls
    output logic [7:0] phaser_speed_val,
    output logic       phaser_fben_val,
    // Tremolo controls
    output logic [7:0] trem_rate_val,
    output logic [7:0] trem_depth_val,
    output logic       trem_shape_val
);

  logic [7:0] rx_data;
  logic       rx_valid;
  logic [7:0] tx_data;
  logic       tx_start;
  logic       tx_busy;
  logic       tx_done;

  uart #(
      .CLK_FREQ (CLK_FREQ),
      .BAUD_RATE(BAUD_RATE)
  ) uart_inst (
      .clk(sys_clk),
      .rst_n(rst_n),
      .rx_serial(tx_din),
      .rx_valid(rx_valid),
      .rx_byte(rx_data),
      .tx_start(tx_start),
      .tx_byte(tx_data),
      .tx_busy(tx_busy),
      .tx_serial(rx_dout),
      .tx_done(tx_done)
  );

  cmd_proc cmd_proc_inst (
      .clk(sys_clk),
      .rst_n(rst_n),
      .rx_valid(rx_valid),
      .rx_byte(rx_data),
      .tx_start(tx_start),
      .tx_byte(tx_data),
      .tx_busy(tx_busy),
      .tx_done(tx_done),
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

endmodule
