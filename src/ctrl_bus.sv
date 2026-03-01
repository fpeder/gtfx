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
//   0x18..0x1F : Slot 3  - dd3       [0]=tone [1]=lvl [2]=fdb [3..4]=time [7]=bypass
//   0x20..0x27 : Slot 4  - tube_dist [0]=gain [1]=bas [2]=mid [3]=tre [4]=lvl [7]=bypass
//   0x40..0x45 : route[0] .. route[N_XBAR-1]
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
    parameter int N_SLOTS    = 5,
    parameter int REGS_PER   = 8,
    parameter int REG_W      = 8,
    parameter int N_XBAR     = N_SLOTS + 1,
    parameter int SEL_W      = $clog2(N_XBAR),
    parameter int ADDR_W     = 8,
    parameter int CFG_DEPTH  = N_SLOTS * REGS_PER
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

            // ---- Slot 3 (dd3): tone/level/feedback/time ----
            cfg_mem[24] <= 8'hFF;
            cfg_mem[25] <= 8'h80;
            cfg_mem[26] <= 8'h64;
            cfg_mem[27] <= 8'h00;
            cfg_mem[28] <= 8'h30;

            // ---- Slot 4 (tube_distortion): gain/bass/mid/treble/level ----
            cfg_mem[32] <= 8'h40;   // gain
            cfg_mem[33] <= 8'h80;   // tone_bass
            cfg_mem[34] <= 8'h80;   // tone_mid
            cfg_mem[35] <= 8'h80;   // tone_treble
            cfg_mem[36] <= 8'hA0;   // level
            // cfg_mem[39] = 0x01 bypass on - set above

        end else if (wr_en && wr_addr < ADDR_W'(SLOT_END)) begin
            cfg_mem[wr_addr] <= wr_data;
        end
    end

    // =========================================================================
    // Route registers
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            // Default chain: ADC → TUB → TRM → PHA → CHO → DLY → DAC
            route[0] <= SEL_W'(4);   // DAC ← DLY  (port 4)
            route[1] <= SEL_W'(5);   // TRM ← TUB  (port 5)
            route[2] <= SEL_W'(1);   // PHA ← TRM  (port 1)
            route[3] <= SEL_W'(2);   // CHO ← PHA  (port 2)
            route[4] <= SEL_W'(3);   // DLY ← CHO  (port 3)
            route[5] <= SEL_W'(0);   // TUB ← ADC  (port 0)
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