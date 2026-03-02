// ============================================================================
// top.sv - AXI-Stream Audio Effects Pipeline with Routing Matrix
//
// Uses the ORIGINAL audio.sv (thin i2s2 wrapper) for I2S - no audio_axis.
// ADC/DAC data is bridged to/from the AXI-Stream crossbar with simple
// pack/unpack wires and a sample_en edge detect, matching the proven
// original design's I2S interface exactly.
//
// Symmetric 8-port crossbar (N_XBAR = N_SLOTS + 1 = 8):
//
//   Port  Master (source)          Slave (sink)
//   ----  ----------------------   ----------------------
//    0    ADC                      DAC
//    1    Slot 0 (tremolo)         Slot 0 input
//    2    Slot 1 (phaser)          Slot 1 input
//    3    Slot 2 (chorus)          Slot 2 input
//    4    Slot 3 (dd3 delay)       Slot 3 input
//    5    Slot 4 (tube_distortion) Slot 4 input
//    6    Slot 5 (flanger)         Slot 5 input
//    7    Slot 6 (big_muff)        Slot 6 input
//
// Default linear chain:
//   route[7]=0 BMF←ADC  route[5]=7 TUB←BMF  route[2]=5 PHA←TUB
//   route[6]=2 FLN←PHA  route[3]=6 CHO←FLN  route[1]=3 TRM←CHO
//   route[4]=1 DLY←TRM  route[0]=4 DAC←DLY
//
// Bypass is software-controlled via "set <efx> on/off" CLI commands.
// Each slot reads its own bypass bit from cfg_slice[7][0] (no sw_effect port).
// ============================================================================

module top #(
    parameter int N_SLOTS  = 7,
    parameter int REGS_PER = 8,
    parameter int REG_W    = 8
) (
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
    output logic [3:0] led
);

  import axis_audio_pkg::*;

  localparam int N_XBAR = N_SLOTS + 1;
  localparam int SEL_W = $clog2(N_XBAR);
  localparam int CFG_DEPTH = N_SLOTS * REGS_PER;

  // =========================================================================
  // Clock
  // =========================================================================
  logic clk_audio, locked;

  clk_wiz_0 clk_wiz (
      .clk_in1 (sys_clk),
      .resetn  (resetn),
      .locked  (locked),
      .clk_out1(clk_audio)
  );

  // =========================================================================
  // Command subsystem (sys_clk domain)
  // =========================================================================
  logic       cmd_wr_toggle;
  logic [7:0] cmd_wr_addr;
  logic [7:0] cmd_wr_data;

  cmd #(
      .CLK_FREQ (100_000_000),
      .BAUD_RATE(115_200)
  ) cmd_inst (
      .sys_clk(sys_clk),
      .rst_n  (resetn),
      .tx_din (uart_tx_din),
      .rx_dout(uart_rx_dout),
      .wr_en  (cmd_wr_toggle),
      .wr_addr(cmd_wr_addr),
      .wr_data(cmd_wr_data)
  );

  // =========================================================================
  // CDC: sys_clk → clk_audio (toggle handshake)
  // =========================================================================
  logic tog_s1, tog_s2, tog_prev;
  logic [7:0] addr_s1, addr_s2;
  logic [7:0] data_s1, data_s2;
  logic       wr_en_audio;
  logic [7:0] wr_addr_audio;
  logic [7:0] wr_data_audio;

  always_ff @(posedge clk_audio) begin
    if (!resetn) begin
      tog_s1   <= 1'b0;
      tog_s2   <= 1'b0;
      tog_prev <= 1'b0;
      addr_s1  <= 8'd0;
      addr_s2  <= 8'd0;
      data_s1  <= 8'd0;
      data_s2  <= 8'd0;
    end else begin
      tog_s1   <= cmd_wr_toggle;
      tog_s2   <= tog_s1;
      addr_s1  <= cmd_wr_addr;
      addr_s2  <= addr_s1;
      data_s1  <= cmd_wr_data;
      data_s2  <= data_s1;
      tog_prev <= tog_s2;
    end
  end

  assign wr_en_audio   = tog_s2 ^ tog_prev;
  assign wr_addr_audio = addr_s2;
  assign wr_data_audio = data_s2;

  // =========================================================================
  // Control Bus  (shared config RAM)
  // =========================================================================
  logic [REG_W-1:0] cfg_mem [CFG_DEPTH];   // shared config RAM
  logic [SEL_W-1:0] route   [N_XBAR];

  ctrl_bus #(
      .N_SLOTS (N_SLOTS),
      .REGS_PER(REGS_PER),
      .REG_W   (REG_W),
      .N_XBAR  (N_XBAR),
      .SEL_W   (SEL_W)
  ) ctrl_inst (
      .clk    (clk_audio),
      .rst_n  (resetn),
      .wr_en  (wr_en_audio),
      .wr_addr(wr_addr_audio),
      .wr_data(wr_data_audio),
      .rd_data(),
      .cfg_mem(cfg_mem),
      .route  (route)
  );

  // =========================================================================
  // I2S Audio - uses original audio.sv (proven working i2s2 wrapper)
  // =========================================================================
  logic [23:0] din_l, din_r;  // TX to DAC
  logic [23:0] dout_l, dout_r;  // RX from ADC

  audio #(
      .AUDIO_W(24)
  ) audio_inst (
      .clk_audio(clk_audio),
      .resetn   (resetn),
      .tx_mclk  (i2s2_tx_mclk),
      .tx_lrclk (i2s2_tx_lrclk),
      .tx_sclk  (i2s2_tx_sclk),
      .tx_serial(i2s2_tx),
      .rx_mclk  (i2s2_rx_mclk),
      .rx_lrclk (i2s2_rx_lrclk),
      .rx_sclk  (i2s2_rx_sclk),
      .rx_serial(i2s2_rx),
      .din_l    (din_l),
      .din_r    (din_r),
      .dout_l   (dout_l),
      .dout_r   (dout_r)
  );

  // ---- sample_en: lrclk rising-edge detect (same as original top.sv) ----
  logic lrclk_prev, sample_en;
  always_ff @(posedge clk_audio) begin
    lrclk_prev <= i2s2_tx_lrclk;
  end
  assign sample_en = i2s2_tx_lrclk & ~lrclk_prev;

  // =========================================================================
  // ADC → AXI-Stream master (port 0)
  //
  // On sample_en, pack stereo ADC data and assert tvalid for 1 cycle.
  // =========================================================================
  logic [47:0] adc_tdata;
  logic        adc_tvalid;

  always_ff @(posedge clk_audio) begin
    if (!resetn) begin
      adc_tdata  <= '0;
      adc_tvalid <= 1'b0;
    end else begin
      adc_tvalid <= sample_en;
      if (sample_en) adc_tdata <= pack_stereo($signed(dout_l), $signed(dout_r));
    end
  end

  // =========================================================================
  // DAC ← AXI-Stream slave (port 0)
  //
  // Whenever the crossbar delivers valid data, latch it for I2S TX.
  // din_l/din_r are continuous register outputs read by i2s2 at load_tx.
  // =========================================================================
  logic [47:0] dac_tdata;
  logic        dac_tvalid;

  always_ff @(posedge clk_audio) begin
    if (!resetn) begin
      din_l <= '0;
      din_r <= '0;
    end else if (dac_tvalid) begin
      din_l <= unpack_left(dac_tdata);
      din_r <= unpack_right(dac_tdata);
    end
  end

  // =========================================================================
  // Crossbar (5 symmetric ports)
  // =========================================================================
  logic [47:0] xbar_m_tdata [N_XBAR];
  logic        xbar_m_tvalid[N_XBAR];
  logic        xbar_m_tready[N_XBAR];
  logic [47:0] xbar_s_tdata [N_XBAR];
  logic        xbar_s_tvalid[N_XBAR];
  logic        xbar_s_tready[N_XBAR];

  axis_crossbar #(
      .N_PORTS(N_XBAR),
      .DATA_W (48),
      .SEL_W  (SEL_W)
  ) xbar (
      .route   (route),
      .m_tdata (xbar_m_tdata),
      .m_tvalid(xbar_m_tvalid),
      .m_tready(xbar_m_tready),
      .s_tdata (xbar_s_tdata),
      .s_tvalid(xbar_s_tvalid),
      .s_tready(xbar_s_tready)
  );

  // ---- Port 0: ADC master + DAC slave ----
  assign xbar_m_tdata[0]  = adc_tdata;
  assign xbar_m_tvalid[0] = adc_tvalid;
  // adc_tready not used (fire-and-forget)

  assign dac_tdata        = xbar_s_tdata[0];
  assign dac_tvalid       = xbar_s_tvalid[0];
  assign xbar_s_tready[0] = 1'b1;  // DAC always accepts

  // =========================================================================
  // Effect Slots
  // =========================================================================
  localparam int EFFECT_TYPES[N_SLOTS] = '{0, 1, 2, 3, 4, 5, 6};
  localparam int TUBE_LUT_ADDR = 12;  // 4096-entry 12AX7 LUT
  localparam int TUBE_FRAC_W = 12;  // interpolation fraction bits

  for (genvar i = 0; i < N_SLOTS; i++) begin : gen_slot
    axis_effect_slot #(
        .SLOT_ID         (i),
        .EFFECT_TYPE     (EFFECT_TYPES[i]),
        .DATA_W          (48),
        .CTRL_DEPTH      (REGS_PER),
        .CTRL_W          (REG_W),
        .AUDIO_W         (24),
        .CHORUS_DELAY_MAX(2048),
        .DD3_RAM_DEPTH   (32768),
        .TUBE_LUT_ADDR   (TUBE_LUT_ADDR),
        .TUBE_FRAC_W     (TUBE_FRAC_W)
    ) slot_inst (
        .clk          (clk_audio),
        .rst_n        (resetn),
        .s_axis_tdata (xbar_s_tdata[i+1]),
        .s_axis_tvalid(xbar_s_tvalid[i+1]),
        .s_axis_tready(xbar_s_tready[i+1]),
        .m_axis_tdata (xbar_m_tdata[i+1]),
        .m_axis_tvalid(xbar_m_tvalid[i+1]),
        .m_axis_tready(xbar_m_tready[i+1]),
        .cfg_slice    (cfg_mem[i*REGS_PER+:REGS_PER])
        // bypass read internally from cfg_slice[7][0]
    );
  end

  // =========================================================================
  // LEDs
  // =========================================================================
  assign led = {2'b00, locked, btns[0]};

endmodule