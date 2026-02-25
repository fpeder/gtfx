// ============================================================================
// top.sv - AXI-Stream Audio Effects Pipeline with Routing Matrix
//
// Symmetric 5-port crossbar (N_XBAR = N_SLOTS + 1):
//
//   Port  Master (source)     Slave (sink)
//   ----  ----------------    ----------------
//    0    ADC                 DAC
//    1    Slot 0 (tremolo)    Slot 0 input
//    2    Slot 1 (phaser)     Slot 1 input
//    3    Slot 2 (chorus)     Slot 2 input
//    4    Slot 3 (delay)      Slot 3 input
//
// route[k] = which master feeds slave port k.
// Source and sink use the SAME numbering:
//   0=ADC/DAC  1=TRM  2=PHA  3=CHO  4=DLY
//
// Default linear chain:
//   route[0] = 4  →  DAC    ← DLY out
//   route[1] = 0  →  TRM in ← ADC
//   route[2] = 1  →  PHA in ← TRM out
//   route[3] = 2  →  CHO in ← PHA out
//   route[4] = 3  →  DLY in ← CHO out
// ============================================================================

module top #(
    parameter int N_SLOTS = 4
)(
    input  logic       sys_clk,
    input  logic       resetn,

    // I2S
    output logic       i2s2_tx_mclk,
    output logic       i2s2_tx_lrclk,
    output logic       i2s2_tx_sclk,
    output logic       i2s2_tx,
    output logic       i2s2_rx_mclk,
    output logic       i2s2_rx_lrclk,
    output logic       i2s2_rx_sclk,
    input  logic       i2s2_rx,

    // UART
    input  logic       uart_tx_din,
    output logic       uart_rx_dout,

    // Buttons & LEDs
    input  logic [1:0] btns,
    output logic [3:0] led,

    // Effect enable switches
    input  logic [3:0] sw_effect
);

    import axis_audio_pkg::*;

    localparam int N_XBAR = N_SLOTS + 1;  // 5 ports: ADC/DAC + 4 slots
    localparam int SEL_W  = $clog2(N_XBAR);

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
        .sys_clk  (sys_clk),
        .rst_n    (resetn),
        .tx_din   (uart_tx_din),
        .rx_dout  (uart_rx_dout),
        .sw_effect(sw_effect),
        .wr_en    (cmd_wr_toggle),   // toggle signal (not a pulse)
        .wr_addr  (cmd_wr_addr),     // held stable between toggles
        .wr_data  (cmd_wr_data)      // held stable between toggles
    );

    // =========================================================================
    // CDC: sys_clk → clk_audio (toggle handshake)
    //
    // cmd.sv flips wr_toggle on each register write and holds addr/data stable.
    // We double-flop the toggle, XOR-detect the change to produce a 1-cycle
    // write pulse in the audio domain.
    // =========================================================================
    logic       tog_s1, tog_s2, tog_prev;
    logic [7:0] addr_s1, addr_s2;
    logic [7:0] data_s1, data_s2;
    logic       wr_en_audio;
    logic [7:0] wr_addr_audio;
    logic [7:0] wr_data_audio;

    always_ff @(posedge clk_audio) begin
        tog_s1  <= cmd_wr_toggle;   tog_s2  <= tog_s1;
        addr_s1 <= cmd_wr_addr;     addr_s2 <= addr_s1;
        data_s1 <= cmd_wr_data;     data_s2 <= data_s1;
        tog_prev <= tog_s2;
    end

    assign wr_en_audio   = tog_s2 ^ tog_prev;
    assign wr_addr_audio = addr_s2;
    assign wr_data_audio = data_s2;

    // CDC for switches
    logic [3:0] sw_s, sw_audio;
    always_ff @(posedge clk_audio) begin
        sw_s     <= sw_effect;
        sw_audio <= sw_s;
    end

    // =========================================================================
    // Control Bus
    // =========================================================================
    logic             slot_wr    [N_SLOTS];
    logic [$clog2(8)-1:0] slot_addr [N_SLOTS];
    logic [7:0]       slot_wdata [N_SLOTS];
    logic [SEL_W-1:0] route      [N_XBAR];
    logic             ctrl_bypass[N_SLOTS];

    ctrl_bus #(
        .N_SLOTS (N_SLOTS),
        .REGS_PER(8),
        .REG_W   (8),
        .N_XBAR  (N_XBAR),
        .SEL_W   (SEL_W)
    ) ctrl_inst (
        .clk       (clk_audio),
        .rst_n     (resetn),
        .wr_en     (wr_en_audio),
        .wr_addr   (wr_addr_audio),
        .wr_data   (wr_data_audio),
        .rd_data   (),
        .slot_wr   (slot_wr),
        .slot_addr (slot_addr),
        .slot_wdata(slot_wdata),
        .route     (route),
        .bypass    (ctrl_bypass)
    );

    logic bypass_merged [N_SLOTS];
    always_comb begin
        for (int i = 0; i < N_SLOTS; i++)
            bypass_merged[i] = ctrl_bypass[i] | ~sw_audio[i];
    end

    // =========================================================================
    // AXI-Stream Audio I/O
    // =========================================================================
    logic [47:0] adc_tdata;
    logic        adc_tvalid, adc_tready;
    logic [47:0] dac_tdata;
    logic        dac_tvalid, dac_tready;

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
        .m_axis_tdata (adc_tdata),
        .m_axis_tvalid(adc_tvalid),
        .m_axis_tready(adc_tready),
        .s_axis_tdata (dac_tdata),
        .s_axis_tvalid(dac_tvalid),
        .s_axis_tready(dac_tready)
    );

    // =========================================================================
    // Crossbar (5 symmetric ports)
    // =========================================================================
    logic [47:0] xbar_m_tdata  [N_XBAR];
    logic        xbar_m_tvalid [N_XBAR];
    logic        xbar_m_tready [N_XBAR];
    logic [47:0] xbar_s_tdata  [N_XBAR];
    logic        xbar_s_tvalid [N_XBAR];
    logic        xbar_s_tready [N_XBAR];

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

    // ---- Port 0: ADC (master) + DAC (slave) ----
    assign xbar_m_tdata [0] = adc_tdata;
    assign xbar_m_tvalid[0] = adc_tvalid;
    assign adc_tready       = xbar_m_tready[0];

    assign dac_tdata        = xbar_s_tdata [0];
    assign dac_tvalid       = xbar_s_tvalid[0];
    assign xbar_s_tready[0] = dac_tready;

    // =========================================================================
    // Effect Slots
    //
    // Slot i occupies crossbar port (i+1):
    //   Slot input  ← crossbar slave port  (i+1)
    //   Slot output → crossbar master port (i+1)
    // =========================================================================

    localparam int EFFECT_TYPES [N_SLOTS] = '{0, 1, 2, 3};

    //                                                          reg7   reg6   reg5   reg4   reg3   reg2   reg1   reg0
    localparam logic [63:0] INIT_SLOT0 = {8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'hB4, 8'h3C};
    localparam logic [63:0] INIT_SLOT1 = {8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h50};
    localparam logic [63:0] INIT_SLOT2 = {8'h00, 8'h00, 8'h00, 8'h80, 8'hC8, 8'h80, 8'h64, 8'h50};
    localparam logic [63:0] INIT_SLOT3 = {8'h00, 8'h00, 8'h00, 8'h07, 8'hD0, 8'h64, 8'h80, 8'hFF};

    localparam logic [63:0] SLOT_INITS [0:3] = '{INIT_SLOT0, INIT_SLOT1, INIT_SLOT2, INIT_SLOT3};

    for (genvar i = 0; i < N_SLOTS; i++) begin : gen_slot
        axis_effect_slot #(
            .SLOT_ID          (i),
            .EFFECT_TYPE      (EFFECT_TYPES[i]),
            .DATA_W           (48),
            .CTRL_DEPTH       (8),
            .CTRL_W           (8),
            .AUDIO_W          (24),
            .CHORUS_DELAY_MAX (2048),
            .DD3_RAM_DEPTH    (48000),
            .INIT_PACKED      (SLOT_INITS[i])
        ) slot_inst (
            .clk           (clk_audio),
            .rst_n         (resetn),
            .s_axis_tdata  (xbar_s_tdata [i + 1]),
            .s_axis_tvalid (xbar_s_tvalid[i + 1]),
            .s_axis_tready (xbar_s_tready[i + 1]),
            .m_axis_tdata  (xbar_m_tdata [i + 1]),
            .m_axis_tvalid (xbar_m_tvalid[i + 1]),
            .m_axis_tready (xbar_m_tready[i + 1]),
            .ctrl_wr       (slot_wr[i]),
            .ctrl_addr     (slot_addr[i]),
            .ctrl_wdata    (slot_wdata[i]),
            .bypass        (bypass_merged[i])
        );
    end

    // =========================================================================
    // LEDs
    // =========================================================================
    assign led = sw_effect;

endmodule