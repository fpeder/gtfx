`timescale 1ns / 1ps

// ============================================================================
// ctrl_bus.sv - Centralised control register file
//
// Single write port from cmd_proc.  Decodes addresses into:
//   - Per-slot control registers (forwarded to each slot's local reg file)
//   - Crossbar route registers
//   - Per-slot bypass flags
//
// Address map (flat, 8-bit addresses):
//
//   0x00..0x07 : Slot 0 registers  (CTRL_REGS_PER_SLOT = 8)
//   0x08..0x0F : Slot 1 registers
//   0x10..0x17 : Slot 2 registers
//   0x18..0x1F : Slot 3 registers
//   ...
//   0x40..0x45 : route[0] .. route[N_XBAR-1]   (crossbar routing)
//   0x48..0x4B : bypass[0] .. bypass[N_SLOTS-1]
//
// The route and bypass registers are stored locally and exposed as outputs.
// Slot registers are forwarded via slot_wr / slot_addr / slot_wdata.
// ============================================================================

module ctrl_bus #(
    parameter int N_SLOTS    = 4,
    parameter int REGS_PER   = 8,
    parameter int REG_W      = 8,
    parameter int N_XBAR     = N_SLOTS + 2,
    parameter int SEL_W      = $clog2(N_XBAR),
    parameter int ADDR_W     = 8          // flat address width
)(
    input  logic             clk,
    input  logic             rst_n,

    // Write port (from cmd_proc adapter)
    input  logic             wr_en,
    input  logic [ADDR_W-1:0] wr_addr,
    input  logic [REG_W-1:0] wr_data,

    // Read port (optional, active on wr_addr)
    output logic [REG_W-1:0] rd_data,

    // Per-slot forwarded writes
    output logic             slot_wr    [N_SLOTS],
    output logic [$clog2(REGS_PER)-1:0] slot_addr [N_SLOTS],
    output logic [REG_W-1:0] slot_wdata [N_SLOTS],

    // Crossbar routing
    output logic [SEL_W-1:0] route      [N_XBAR],

    // Per-slot bypass
    output logic             bypass     [N_SLOTS]
);

    // ---- Address map constants ----
    localparam int SLOT_BASE   = 0;                             // 0x00
    localparam int SLOT_END    = N_SLOTS * REGS_PER;            // 0x20 for 4 slots
    localparam int ROUTE_BASE  = 8'h40;
    localparam int ROUTE_END   = ROUTE_BASE + N_XBAR;
    localparam int BYPASS_BASE = 8'h48;
    localparam int BYPASS_END  = BYPASS_BASE + N_SLOTS;

    // ---- Route registers ----
    logic [SEL_W-1:0] route_regs [N_XBAR];

    // ---- Bypass registers ----
    logic bypass_regs [N_SLOTS];

    // ---- Default routing: linear chain ----
    // Master ports:  0=ADC, 1=slot0_out, 2=slot1_out, 3=slot2_out, 4=slot3_out
    // Slave ports:   0=slot0_in, 1=slot1_in, 2=slot2_in, 3=slot3_in, 4=(unused), 5=DAC
    //
    // Default: route[0]=0(ADC), route[1]=1(slot0), ..., route[N_XBAR-1]=N_SLOTS(last slot out)
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < N_XBAR; i++)
                route_regs[i] <= (i < N_XBAR - 1) ? SEL_W'(i) : SEL_W'(N_SLOTS);
            for (int i = 0; i < N_SLOTS; i++)
                bypass_regs[i] <= 1'b0;
        end else if (wr_en) begin
            // Route write
            if (wr_addr >= ADDR_W'(ROUTE_BASE) && wr_addr < ADDR_W'(ROUTE_END))
                route_regs[wr_addr - ADDR_W'(ROUTE_BASE)] <= wr_data[SEL_W-1:0];
            // Bypass write
            if (wr_addr >= ADDR_W'(BYPASS_BASE) && wr_addr < ADDR_W'(BYPASS_END))
                bypass_regs[wr_addr - ADDR_W'(BYPASS_BASE)] <= wr_data[0];
        end
    end

    // ---- Output route and bypass ----
    always_comb begin
        for (int i = 0; i < N_XBAR; i++)
            route[i] = route_regs[i];
        for (int i = 0; i < N_SLOTS; i++)
            bypass[i] = bypass_regs[i];
    end

    // ---- Slot register forwarding ----
    // Decode: if wr_addr is in [slot_base, slot_end), forward to correct slot
    always_comb begin
        for (int i = 0; i < N_SLOTS; i++) begin
            slot_wr[i]    = 1'b0;
            slot_addr[i]  = '0;
            slot_wdata[i] = '0;
        end

        if (wr_en && wr_addr < ADDR_W'(SLOT_END)) begin
            automatic int slot_idx = int'(wr_addr) / REGS_PER;
            automatic int reg_idx  = int'(wr_addr) % REGS_PER;
            if (slot_idx < N_SLOTS) begin
                slot_wr[slot_idx]    = 1'b1;
                slot_addr[slot_idx]  = reg_idx[$clog2(REGS_PER)-1:0];
                slot_wdata[slot_idx] = wr_data;
            end
        end
    end

    // ---- Read-back (optional) ----
    always_comb begin
        rd_data = '0;
        if (wr_addr >= ADDR_W'(ROUTE_BASE) && wr_addr < ADDR_W'(ROUTE_END))
            rd_data = REG_W'(route_regs[wr_addr - ADDR_W'(ROUTE_BASE)]);
        else if (wr_addr >= ADDR_W'(BYPASS_BASE) && wr_addr < ADDR_W'(BYPASS_END))
            rd_data = REG_W'(bypass_regs[wr_addr - ADDR_W'(BYPASS_BASE)]);
    end

endmodule