// ============================================================================
// top.sv - AXI-Stream Audio Effects Pipeline with Routing Matrix
//
// Architecture:
//   - N_SLOTS effect slots, each with a uniform AXI-Stream + control interface
//   - axis_crossbar routes any source to any sink at runtime
//   - ctrl_bus provides centralised register file for all controls + routing
//   - audio_axis bridges I2S ↔ AXI-Stream
//
// Crossbar port map (N_XBAR = N_SLOTS + 2):
//   port 0             = ADC master (source)
//   port 1..N_SLOTS    = effect slot masters (source = slot output)
//   port N_SLOTS+1     = DAC sink
//   slots consume from crossbar slave ports 0..N_SLOTS-1
//   DAC consumes from crossbar slave port N_SLOTS+1 (= N_XBAR-1)
//
// Default routing (linear chain):
//   route[0] = 0  →  slot 0 ← ADC
//   route[1] = 1  →  slot 1 ← slot 0
//   route[2] = 2  →  slot 2 ← slot 1
//   route[3] = 3  →  slot 3 ← slot 2
//   route[5] = 4  →  DAC    ← slot 3
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

    // Effect enable switches (directly wired to bypass)
    input  logic [3:0] sw_effect
);

    import axis_audio_pkg::*;

    localparam int N_XBAR = N_SLOTS + 2;
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
    // Command subsystem (sys_clk domain) - unchanged modules inside
    // =========================================================================
    // cmd_proc outputs a flat register write interface.
    // We adapt it to ctrl_bus in the audio domain via CDC.

    logic       cmd_wr_en;
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
        // Flat register write port (directly from cmd_proc)
        .wr_en    (cmd_wr_en),
        .wr_addr  (cmd_wr_addr),
        .wr_data  (cmd_wr_data)
    );

    // =========================================================================
    // CDC: sys_clk → clk_audio for control writes
    //
    // Register writes are slow (UART rate) so a simple handshake is safe.
    // We double-flop the write strobe and data.
    // =========================================================================
    logic       wr_en_s1, wr_en_s2, wr_en_audio;
    logic [7:0] wr_addr_s1, wr_addr_s2, wr_addr_audio;
    logic [7:0] wr_data_s1, wr_data_s2, wr_data_audio;
    logic       wr_en_prev;

    always_ff @(posedge clk_audio) begin
        wr_en_s1   <= cmd_wr_en;    wr_en_s2   <= wr_en_s1;
        wr_addr_s1 <= cmd_wr_addr;  wr_addr_s2 <= wr_addr_s1;
        wr_data_s1 <= cmd_wr_data;  wr_data_s2 <= wr_data_s1;
        wr_en_prev <= wr_en_s2;
    end

    // Rising-edge detect on wr_en for single-cycle pulse in audio domain
    assign wr_en_audio   = wr_en_s2 & ~wr_en_prev;
    assign wr_addr_audio = wr_addr_s2;
    assign wr_data_audio = wr_data_s2;

    // Also CDC the switch inputs for bypass
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

    // Merge HW switches with register-controlled bypass
    // Either source can bypass an effect
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
    // Crossbar
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

    // ---- Port 0: ADC source (master input to crossbar) ----
    assign xbar_m_tdata [0] = adc_tdata;
    assign xbar_m_tvalid[0] = adc_tvalid;
    assign adc_tready       = xbar_m_tready[0];

    // ---- Port N_XBAR-1: DAC sink (slave output from crossbar) ----
    assign dac_tdata                    = xbar_s_tdata [N_XBAR-1];
    assign dac_tvalid                   = xbar_s_tvalid[N_XBAR-1];
    assign xbar_s_tready[N_XBAR-1]     = dac_tready;

    // ---- Tie off unused ports ----
    // Master N_XBAR-1 (port 5): no source drives this - tie to silence
    assign xbar_m_tdata [N_XBAR-1] = '0;
    assign xbar_m_tvalid[N_XBAR-1] = 1'b0;
    // Slave N_SLOTS (port 4): no slot consumes this - always ready to drain
    assign xbar_s_tready[N_SLOTS]  = 1'b1;

    // =========================================================================
    // Effect Slots (generate loop)
    //
    // EFFECT_TYPE: 0=tremolo, 1=phaser, 2=chorus, 3=dd3
    //
    // Crossbar wiring per slot i:
    //   Slot input  ← crossbar slave port i    (xbar_s_tdata[i])
    //   Slot output → crossbar master port i+1 (xbar_m_tdata[i+1])
    // =========================================================================

    // Effect type array - edit this to change which effect is in which slot
    localparam int EFFECT_TYPES [N_SLOTS] = '{0, 1, 2, 3};

    // Default register values per slot, packed as 64-bit constants.
    // Bit layout: [7:0]=reg0, [15:8]=reg1, ..., [63:56]=reg7
    // Verilog concat order: {reg7, reg6, reg5, reg4, reg3, reg2, reg1, reg0}
    //
    // Slot 0 (tremolo): reg0=rat(3C) reg1=dep(B4) reg2=shp(00)
    // Slot 1 (phaser):  reg0=spd(50) reg1=fbn(00)
    // Slot 2 (chorus):  reg0=rat(50) reg1=dep(64) reg2=efx(80) reg3=eqh(C8) reg4=eql(80)
    // Slot 3 (dd3):     reg0=ton(FF) reg1=lvl(80) reg2=fdb(64)
    //                   reg3=tim[0](D0) reg4=tim[1](07) reg5=tim[2](00) reg6=tim[3](00)
    //                                                          reg7   reg6   reg5   reg4   reg3   reg2   reg1   reg0
    localparam logic [63:0] INIT_SLOT0 = {8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'hB4, 8'h3C};
    localparam logic [63:0] INIT_SLOT1 = {8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h50};
    localparam logic [63:0] INIT_SLOT2 = {8'h00, 8'h00, 8'h00, 8'h80, 8'hC8, 8'h80, 8'h64, 8'h50};
    localparam logic [63:0] INIT_SLOT3 = {8'h00, 8'h00, 8'h00, 8'h07, 8'hD0, 8'h64, 8'h80, 8'hFF};

    // Pack all slot inits into one indexable array of packed values
    localparam logic [63:0] SLOT_INITS [0:3] = '{INIT_SLOT0, INIT_SLOT1, INIT_SLOT2, INIT_SLOT3};

    for (genvar i = 0; i < N_SLOTS; i++) begin : gen_slot
        axis_effect_slot #(
            .SLOT_ID          (i),
            .EFFECT_TYPE      (EFFECT_TYPES[i]),
            .DATA_W           (48),
            .CTRL_DEPTH       (8),
            .CTRL_W           (8),
            .AUDIO_W          (24),
            .CHORUS_DELAY_MAX (512),
            .DD3_RAM_DEPTH    (48000),
            .INIT_PACKED      (SLOT_INITS[i])
        ) slot_inst (
            .clk           (clk_audio),
            .rst_n         (resetn),
            // Audio from/to crossbar
            .s_axis_tdata  (xbar_s_tdata [i]),
            .s_axis_tvalid (xbar_s_tvalid[i]),
            .s_axis_tready (xbar_s_tready[i]),
            .m_axis_tdata  (xbar_m_tdata [i + 1]),
            .m_axis_tvalid (xbar_m_tvalid[i + 1]),
            .m_axis_tready (xbar_m_tready[i + 1]),
            // Control
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