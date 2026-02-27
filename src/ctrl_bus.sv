`timescale 1ns / 1ps

// ============================================================================
// ctrl_bus.sv - Centralised control register file (shared config RAM)
//
// All slot parameters live in a single flat array:
//   cfg_mem[slot * REGS_PER + reg_idx]
//
// This array is exposed as an output so every axis_effect_slot can read its
// own slice combinationally - no per-slot write strobes needed.
//
// Address map (flat, 8-bit addresses):
//
//   0x00..0x07 : Slot 0 registers  (REGS_PER = 8)
//   0x08..0x0F : Slot 1 registers
//   0x10..0x17 : Slot 2 registers
//   0x18..0x1F : Slot 3 registers
//   ...
//   0x40..0x44 : route[0] .. route[N_XBAR-1]   (crossbar routing)
//   0x48..0x4B : bypass[0] .. bypass[N_SLOTS-1]
//
// Compared to the forwarded-write approach:
//   REMOVED  slot_wr / slot_addr / slot_wdata arrays (N_SLOTS × 3 ports)
//   ADDED    cfg_mem output - one flat array, slots index it themselves
// ============================================================================

module ctrl_bus #(
    parameter int N_SLOTS    = 4,
    parameter int REGS_PER   = 8,
    parameter int REG_W      = 8,
    parameter int N_XBAR     = N_SLOTS + 1,
    parameter int SEL_W      = $clog2(N_XBAR),
    parameter int ADDR_W     = 8,
    // Total config memory depth
    parameter int CFG_DEPTH  = N_SLOTS * REGS_PER
)(
    input  logic              clk,
    input  logic              rst_n,

    // ---- Write port (from cmd_proc adapter) ----
    input  logic              wr_en,
    input  logic [ADDR_W-1:0] wr_addr,
    input  logic [REG_W-1:0]  wr_data,

    // ---- Read-back port ----
    // Registered one cycle after wr_addr is presented.
    output logic [REG_W-1:0]  rd_data,

    // ---- Shared config RAM ----
    // Each slot reads: cfg_mem[SLOT_ID * REGS_PER + reg_idx]
    // Combinational output; slots see new values the cycle after a write.
    output logic [REG_W-1:0]  cfg_mem [CFG_DEPTH],

    // ---- Crossbar routing ----
    output logic [SEL_W-1:0]  route   [N_XBAR],

    // ---- Per-slot bypass ----
    output logic              bypass  [N_SLOTS]
);

    // =========================================================================
    // Address map constants
    // =========================================================================
    localparam int SLOT_END    = N_SLOTS * REGS_PER;   // 0x20 for 4 slots
    localparam int ROUTE_BASE  = 8'h40;
    localparam int ROUTE_END   = ROUTE_BASE + N_XBAR;
    localparam int BYPASS_BASE = 8'h48;
    localparam int BYPASS_END  = BYPASS_BASE + N_SLOTS;

    // =========================================================================
    // Config RAM  (slot registers)
    // cfg_mem is an output port driven directly by the register - no internal
    // shadow copy needed.
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < CFG_DEPTH; i++)
                cfg_mem[i] <= '0;
            // Slot 0 (tremolo): rate=0x3C depth=0xB4 shape=0x00
            cfg_mem[0] <= 8'h3C;
            cfg_mem[1] <= 8'hB4;
            // Slot 1 (phaser): speed=0x50 feedback_en=0x00
            cfg_mem[8] <= 8'h50;
            // Slot 2 (chorus): rate=0x50 depth=0x64 effect_lvl=0x80 eq_hi=0xC8 eq_lo=0x80
            cfg_mem[16] <= 8'h50; cfg_mem[17] <= 8'h64; cfg_mem[18] <= 8'h80;
            cfg_mem[19] <= 8'hC8; cfg_mem[20] <= 8'h80;
            // Slot 3 (dd3): tone=0xFF level=0x80 feedback=0x64 time=0x07D0 (LE: D0,07)
            cfg_mem[24] <= 8'hFF; cfg_mem[25] <= 8'h80; cfg_mem[26] <= 8'h64;
            cfg_mem[27] <= 8'hD0; cfg_mem[28] <= 8'h07;
        end else if (wr_en && wr_addr < ADDR_W'(SLOT_END)) begin
            cfg_mem[wr_addr] <= wr_data;
        end
    end

    // =========================================================================
    // Route registers
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            // Default: linear chain  slot[i] ← slot[i-1] or ADC
            route[0] <= SEL_W'(N_SLOTS);        // DAC ← last slot
            for (int i = 1; i < N_XBAR; i++)
                route[i] <= SEL_W'(i - 1);
        end else if (wr_en &&
                     wr_addr >= ADDR_W'(ROUTE_BASE) &&
                     wr_addr <  ADDR_W'(ROUTE_END)) begin
            route[wr_addr - ADDR_W'(ROUTE_BASE)] <= wr_data[SEL_W-1:0];
        end
    end

    // =========================================================================
    // Bypass registers
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < N_SLOTS; i++)
                bypass[i] <= 1'b0;
        end else if (wr_en &&
                     wr_addr >= ADDR_W'(BYPASS_BASE) &&
                     wr_addr <  ADDR_W'(BYPASS_END)) begin
            bypass[wr_addr - ADDR_W'(BYPASS_BASE)] <= wr_data[0];
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
            else if (wr_addr >= ADDR_W'(BYPASS_BASE) && wr_addr < ADDR_W'(BYPASS_END))
                rd_data <= REG_W'(bypass[wr_addr - ADDR_W'(BYPASS_BASE)]);
        end
    end

endmodule