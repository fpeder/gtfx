`timescale 1ns / 1ps

// ============================================================================
// tb_ctrl_bus.sv — Testbench for ctrl_bus
//
// Tests:
//   1. Default routing matches expected chain
//   2. Slot register writes are forwarded correctly
//   3. Route register writes update crossbar config (ROUTE_BASE=0x60)
//   4. Bypass via cfg_mem[slot*8 + 7][0]
//   5. Address boundary — slot 0 vs slot 1 boundary
//   6. Reset defaults match expected values
//   7. Read-back of route and cfg_mem registers
//   8. Out-of-range writes don't corrupt
// ============================================================================

module tb_ctrl_bus;

    localparam int N_SLOTS = 10;
    localparam int N_XBAR  = N_SLOTS + 1;  // 11
    localparam int SEL_W   = $clog2(N_XBAR);
    localparam int CLK_P   = 10;
    localparam int CFG_DEPTH = N_SLOTS * 8;

    logic       clk = 0;
    logic       rst_n;
    logic       wr_en;
    logic [7:0] wr_addr;
    logic [7:0] wr_data;
    logic [7:0] rd_data;

    logic [7:0]       cfg_mem [CFG_DEPTH];
    logic [SEL_W-1:0] route   [N_XBAR];

    always #(CLK_P/2) clk = ~clk;

    ctrl_bus #(
        .N_SLOTS(N_SLOTS),
        .REGS_PER(8),
        .REG_W(8),
        .N_XBAR(N_XBAR),
        .SEL_W(SEL_W)
    ) dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .wr_en    (wr_en),
        .wr_addr  (wr_addr),
        .wr_data  (wr_data),
        .rd_data  (rd_data),
        .cfg_mem  (cfg_mem),
        .route    (route)
    );

    int pass_count = 0;
    int fail_count = 0;

    task automatic check(input string name, input logic condition);
        if (condition) pass_count++;
        else begin fail_count++; $error("FAIL: %s", name); end
    endtask

    task automatic write_reg(input logic [7:0] addr, input logic [7:0] data);
        wr_addr = addr;
        wr_data = data;
        wr_en   = 1'b1;
        @(posedge clk);
        #1;
        wr_en = 1'b0;
        @(posedge clk);
        #1;
    endtask

    initial begin
        $display("=== tb_ctrl_bus ===");

        rst_n   = 0;
        wr_en   = 0;
        wr_addr = '0;
        wr_data = '0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk); #1;

        // ---- Test 1: Default routing matches expected chain ----
        // ADC→WAH→CMP→PHA→FLN→CHO→TRM→DLY→REV→DAC
        check("DEFAULT_ROUTE[0] = 6 (DAC←REV)",  route[0]  == SEL_W'(6));
        check("DEFAULT_ROUTE[1] = 3 (TRM←CHO)",  route[1]  == SEL_W'(3));
        check("DEFAULT_ROUTE[2] = 7 (PHA←CMP)",  route[2]  == SEL_W'(7));
        check("DEFAULT_ROUTE[3] = 5 (CHO←FLN)",  route[3]  == SEL_W'(5));
        check("DEFAULT_ROUTE[4] = 1 (DLY←TRM)",  route[4]  == SEL_W'(1));
        check("DEFAULT_ROUTE[5] = 2 (FLN←PHA)",  route[5]  == SEL_W'(2));
        check("DEFAULT_ROUTE[6] = 4 (REV←DLY)",  route[6]  == SEL_W'(4));
        check("DEFAULT_ROUTE[7] = 8 (CMP←WAH)",  route[7]  == SEL_W'(8));
        check("DEFAULT_ROUTE[8] = 0 (WAH←ADC)",  route[8]  == SEL_W'(0));

        // ---- Test 2: Default bypass — all effects bypassed at boot ----
        check("DEFAULT_BYP: trm bypass=1",  cfg_mem[7]  == 8'h01);
        check("DEFAULT_BYP: pha bypass=1",  cfg_mem[15] == 8'h01);
        check("DEFAULT_BYP: cho bypass=1",  cfg_mem[23] == 8'h01);
        check("DEFAULT_BYP: dly bypass=1",  cfg_mem[31] == 8'h01);
        check("DEFAULT_BYP: tub bypass=1",  cfg_mem[39] == 8'h01);
        check("DEFAULT_BYP: fln bypass=1",  cfg_mem[47] == 8'h01);
        check("DEFAULT_BYP: bmf bypass=1",  cfg_mem[55] == 8'h01);
        check("DEFAULT_BYP: rev bypass=1",  cfg_mem[63] == 8'h01);
        check("DEFAULT_BYP: cmp bypass=1",  cfg_mem[71] == 8'h01);
        check("DEFAULT_BYP: wah bypass=1",  cfg_mem[79] == 8'h01);

        // ---- Test 3: Default parameter values (spot check) ----
        check("DEFAULT: trm rate=0x3C",     cfg_mem[0]  == 8'h3C);
        check("DEFAULT: trm depth=0xB4",    cfg_mem[1]  == 8'hB4);
        check("DEFAULT: pha speed=0x50",    cfg_mem[8]  == 8'h50);
        check("DEFAULT: cmp threshold=0x60",cfg_mem[64] == 8'h60);
        check("DEFAULT: cmp ratio=0x40",    cfg_mem[65] == 8'h40);
        check("DEFAULT: wah freq=0x80",     cfg_mem[72] == 8'h80);

        // ---- Test 4: Slot 0 register write (addr 0x02 = slot0 reg2) ----
        write_reg(8'h02, 8'hAB);
        check("SLOT0_WR: cfg_mem[2] = 0xAB", cfg_mem[2] == 8'hAB);

        // ---- Test 5: Slot 8 register write (addr 0x40 = compressor threshold) ----
        write_reg(8'h40, 8'hCC);
        check("SLOT8_WR: cfg_mem[0x40] = 0xCC", cfg_mem[8'h40] == 8'hCC);

        // ---- Test 6: Slot 9 register write (addr 0x48 = wah freq) ----
        write_reg(8'h48, 8'hDD);
        check("SLOT9_WR: cfg_mem[0x48] = 0xDD", cfg_mem[8'h48] == 8'hDD);

        // ---- Test 7: Bypass via cfg_mem write ----
        // Enable tremolo (set bypass = 0)
        write_reg(8'h07, 8'h00);
        check("BYP_OFF: trm bypass=0 after write", cfg_mem[7] == 8'h00);
        // Disable tremolo (set bypass = 1)
        write_reg(8'h07, 8'h01);
        check("BYP_ON: trm bypass=1 after write", cfg_mem[7] == 8'h01);

        // Enable compressor (slot 8 bypass at addr 0x47)
        write_reg(8'h47, 8'h00);
        check("BYP_OFF: cmp bypass=0 after write", cfg_mem[8'h47] == 8'h00);

        // ---- Test 8: Route register write at ROUTE_BASE=0x60 ----
        write_reg(8'h60, 8'h03);  // route[0] = 3
        check("ROUTE[0] = 3 after write", route[0] == SEL_W'(3));

        write_reg(8'h63, 8'h00);  // route[3] = 0
        check("ROUTE[3] = 0 after write", route[3] == SEL_W'(0));

        // Others unchanged
        check("ROUTE[1] still 3 (default)", route[1] == SEL_W'(3));
        check("ROUTE[2] still 5 (default)", route[2] == SEL_W'(5));

        // ---- Test 9: Route read-back ----
        wr_addr = 8'h60;
        wr_en = 1'b0;
        @(posedge clk); #1;  // one-cycle latency for read-back
        check("ROUTE_RD[0] = 3", rd_data == 8'(3));

        wr_addr = 8'h61;
        @(posedge clk); #1;
        check("ROUTE_RD[1] = 3", rd_data == 8'(3));

        // ---- Test 10: Config read-back ----
        wr_addr = 8'h00;
        @(posedge clk); #1;
        check("CFG_RD[0] = trm rate (0x3C)", rd_data == 8'h3C);

        // ---- Test 11: Out-of-range write doesn't corrupt ----
        write_reg(8'h50, 8'hFF);  // between SLOT_END(0x50) and ROUTE_BASE(0x60) - gap
        // Should not affect any cfg_mem or route
        check("OOR: cfg_mem[0] unchanged", cfg_mem[0] == 8'h3C);
        check("OOR: route[0] still 3", route[0] == SEL_W'(3));

        // ---- Test 12: Reset clears everything ----
        rst_n = 0;
        repeat (2) @(posedge clk);
        rst_n = 1;
        @(posedge clk); #1;
        check("RESET: route[0] back to default (8)", route[0] == SEL_W'(8));
        check("RESET: route[3] back to default (6)", route[3] == SEL_W'(6));
        check("RESET: trm bypass back to 1",  cfg_mem[7]  == 8'h01);
        check("RESET: cmp bypass back to 1",  cfg_mem[71] == 8'h01);
        check("RESET: cfg_mem[2] back to 0",  cfg_mem[2]  == 8'h00);

        $display("=== ctrl_bus: %0d passed, %0d failed ===", pass_count, fail_count);
        if (fail_count > 0) $fatal(1, "TESTS FAILED");
        $finish;
    end

endmodule
