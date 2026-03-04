`timescale 1ns / 1ps

// ============================================================================
// ctrl_bus.sv - Centralised control register file (shared config RAM)
//
// All slot parameters live in a single flat array:
//   cfg_mem[slot * REGS_PER + reg_idx]
//
// Bypass is stored at cfg_mem[slot * REGS_PER + 7][0].
// axis_effect_slot reads bypass directly from cfg_slice[7][0].
// The separate bypass[] output port has been removed.
//
// Address map (flat, 8-bit addresses):
//
//   0x00..0x07 : Slot 0  - tremolo   [0]=rate [1]=depth [2]=shape         [7]=bypass
//   0x08..0x0F : Slot 1  - phaser    [0]=speed [1]=feedback_en            [7]=bypass
//   0x10..0x17 : Slot 2  - chorus    [0]=rate [1]=depth [2]=efx [3..4]=eq [7]=bypass
//   0x18..0x1F : Slot 3  - delay     [0]=rpt [1]=mix [2]=flt [3..4]=time [5]=mod [6]=grit+mode [7]=bypass
//   0x20..0x27 : Slot 4  - tube_dist [0]=gain [1]=bas [2]=mid [3]=tre [4]=lvl [7]=bypass
//   0x28..0x2F : Slot 5  - flanger   [0]=man [1]=wid [2]=spd [3]=reg [4]=mix [7]=bypass
//   0x30..0x37 : Slot 6  - big_muff  [0]=sus [1]=ton [2]=vol             [7]=bypass
//   0x38..0x3F : Slot 7  - reverb   [0]=dec [1]=dmp [2]=mix [3]=pre [4]=ton [5]=lvl [7]=bypass
//   0x40..0x48 : route[0] .. route[N_XBAR-1]
//
// "set <efx> on"  writes 0x00 to addr 0x07/0x0F/0x17/0x1F/0x27 (bypass off = active)
// "set <efx> off" writes 0x01 to those addrs                    (bypass on  = bypassed)
// Both fall within the normal cfg_mem write path - no special logic needed.
//
// Removed vs previous version:
//   bypass[] output port
//   BYPASS_BASE / BYPASS_END address range and its always_ff block
// ============================================================================

module ctrl_bus #(
    parameter int N_SLOTS    = 7,
    parameter int REGS_PER   = 8,
    parameter int REG_W      = 8,
    parameter int N_XBAR     = N_SLOTS + 1,
    parameter int SEL_W      = $clog2(N_XBAR),
    parameter int ADDR_W     = 8,
    parameter int CFG_DEPTH  = N_SLOTS * REGS_PER  // MUST equal N_SLOTS * REGS_PER
)(
    input  logic              clk,
    input  logic              rst_n,

    // ---- Write port (from cmd_proc adapter) ----
    input  logic              wr_en,
    input  logic [ADDR_W-1:0] wr_addr,
    input  logic [REG_W-1:0]  wr_data,

    // ---- Read-back port (registered, one cycle after wr_addr presented) ----
    output logic [REG_W-1:0]  rd_data,

    // ---- Shared config RAM ----
    // Each slot reads: cfg_mem[SLOT_ID * REGS_PER +: REGS_PER]
    // cfg_mem[slot*REGS_PER + 7][0] = bypass bit for that slot
    output logic [REG_W-1:0]  cfg_mem [CFG_DEPTH],

    // ---- Crossbar routing ----
    output logic [SEL_W-1:0]  route   [N_XBAR]
);

    // =========================================================================
    // Address map constants
    // =========================================================================
    localparam int SLOT_END   = N_SLOTS * REGS_PER;   // 0x28
    localparam int ROUTE_BASE = 8'h40;
    localparam int ROUTE_END  = ROUTE_BASE + N_XBAR;

    // =========================================================================
    // Config RAM  (slot registers; bypass at [slot*REGS_PER+7])
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < CFG_DEPTH; i++)
                cfg_mem[i] <= '0;
            // All effects default to bypassed - activate individually via "set <efx> on"
            cfg_mem[7]  <= 8'h01;   // trm bypass
            cfg_mem[15] <= 8'h01;   // pha bypass
            cfg_mem[23] <= 8'h01;   // cho bypass
            cfg_mem[31] <= 8'h01;   // dly bypass
            cfg_mem[39] <= 8'h01;   // tub bypass
            cfg_mem[47] <= 8'h01;   // fln bypass
            cfg_mem[55] <= 8'h01;   // bmf bypass

            // ---- Slot 0 (tremolo): rate=0x3C depth=0xB4 ----
            cfg_mem[0] <= 8'h3C;
            cfg_mem[1] <= 8'hB4;

            // ---- Slot 1 (phaser): speed=0x50 ----
            cfg_mem[8] <= 8'h50;

            // ---- Slot 2 (chorus): rate/depth/efx/eq ----
            cfg_mem[16] <= 8'h50;
            cfg_mem[17] <= 8'h64;
            cfg_mem[18] <= 8'h80;
            cfg_mem[19] <= 8'hC8;
            cfg_mem[20] <= 8'h80;

            // ---- Slot 3 (timeline delay): repeats/mix/filter/time/mod/grit ----
            cfg_mem[24] <= 8'h64;   // repeats
            cfg_mem[25] <= 8'h80;   // mix
            cfg_mem[26] <= 8'hFF;   // filter
            cfg_mem[27] <= 8'h00;   // time low
            cfg_mem[28] <= 8'h30;   // time high
            cfg_mem[29] <= 8'h00;   // mod
            cfg_mem[30] <= 8'h00;   // grit+mode (digital)

            // ---- Slot 4 (tube_distortion): gain/bass/mid/treble/level ----
            cfg_mem[32] <= 8'h60;   // gain   (medium crunch)
            cfg_mem[33] <= 8'h80;   // tone_bass   (flat)
            cfg_mem[34] <= 8'h80;   // tone_mid    (flat)
            cfg_mem[35] <= 8'h80;   // tone_treble (flat)
            cfg_mem[36] <= 8'hC0;   // level  (0xC0/512 ≈ 0.375x, tames distortion RMS)
            // cfg_mem[39] = 0x01 bypass on - set above

            // ---- Slot 5 (flanger): manual/width/speed/regen/mix ----
            cfg_mem[40] <= 8'h40;   // manual (tighter ~1.6 ms)
            cfg_mem[41] <= 8'h80;   // width  (50% sweep)
            cfg_mem[42] <= 8'h30;   // speed
            cfg_mem[43] <= 8'h90;   // regen  (12.5% feedback)
            cfg_mem[44] <= 8'h60;   // mix    (37.5% wet)
            // cfg_mem[47] = 0x01 bypass on - set above

            // ---- Slot 6 (big_muff): sustain/tone/volume ----
            cfg_mem[48] <= 8'h80;   // sustain
            cfg_mem[49] <= 8'h80;   // tone
            cfg_mem[50] <= 8'hA0;   // volume
            // cfg_mem[55] = 0x01 bypass on - set above

            // ---- Slot 7 (reverb): decay/damping/mix/pre_dly/tone/level ----
            cfg_mem[56] <= 8'h80;   // decay
            cfg_mem[57] <= 8'h60;   // damping
            cfg_mem[58] <= 8'h60;   // mix (37.5% wet, additive)
            cfg_mem[59] <= 8'h20;   // pre-delay
            cfg_mem[60] <= 8'h80;   // tone
            cfg_mem[61] <= 8'h80;   // level
            cfg_mem[63] <= 8'h01;   // rev bypass

        end else if (wr_en && wr_addr < ADDR_W'(SLOT_END)) begin
            cfg_mem[wr_addr] <= wr_data;
        end
    end

    // =========================================================================
    // Route registers
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            // Default chain: ADC→BMF→TUB→PHA→FLN→CHO→TRM→DLY→REV→DAC
            route[0] <= SEL_W'(8);   // DAC ← REV  (port 8)
            route[1] <= SEL_W'(3);   // TRM ← CHO  (port 3)
            route[2] <= SEL_W'(5);   // PHA ← TUB  (port 5)
            route[3] <= SEL_W'(6);   // CHO ← FLN  (port 6)
            route[4] <= SEL_W'(1);   // DLY ← TRM  (port 1)
            route[5] <= SEL_W'(7);   // TUB ← BMF  (port 7)
            route[6] <= SEL_W'(2);   // FLN ← PHA  (port 2)
            route[7] <= SEL_W'(0);   // BMF ← ADC  (port 0)
            route[8] <= SEL_W'(4);   // REV ← DLY  (port 4)
        end else if (wr_en &&
                     wr_addr >= ADDR_W'(ROUTE_BASE) &&
                     wr_addr <  ADDR_W'(ROUTE_END)) begin
            route[wr_addr - ADDR_W'(ROUTE_BASE)] <= wr_data[SEL_W-1:0];
        end
    end

    // =========================================================================
    // Read-back  (registered, one cycle latency)
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rd_data <= '0;
        end else begin
            rd_data <= '0;
            if (wr_addr < ADDR_W'(SLOT_END))
                rd_data <= cfg_mem[wr_addr];
            else if (wr_addr >= ADDR_W'(ROUTE_BASE) && wr_addr < ADDR_W'(ROUTE_END))
                rd_data <= REG_W'(route[wr_addr - ADDR_W'(ROUTE_BASE)]);
        end
    end

endmodule