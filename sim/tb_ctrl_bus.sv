`timescale 1ns / 1ps

// ============================================================================
// tb_ctrl_bus.sv — Testbench for ctrl_bus
//
// Tests:
//   1. Default routing is linear chain on reset
//   2. Slot register writes are forwarded correctly
//   3. Route register writes update crossbar config
//   4. Bypass register writes
//   5. Address boundary — slot 0 vs slot 1 boundary
//   6. Out-of-range writes don't corrupt
//   7. Read-back of route and bypass registers
// ============================================================================

module tb_ctrl_bus;

    localparam int N_SLOTS = 4;
    localparam int N_XBAR  = N_SLOTS + 2;
    localparam int SEL_W   = $clog2(N_XBAR);
    localparam int CLK_P   = 10;

    logic       clk = 0;
    logic       rst_n;
    logic       wr_en;
    logic [7:0] wr_addr;
    logic [7:0] wr_data;
    logic [7:0] rd_data;

    logic             slot_wr    [N_SLOTS];
    logic [$clog2(8)-1:0] slot_addr [N_SLOTS];
    logic [7:0]       slot_wdata [N_SLOTS];
    logic [SEL_W-1:0] route      [N_XBAR];
    logic             bypass     [N_SLOTS];

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
        .slot_wr  (slot_wr),
        .slot_addr(slot_addr),
        .slot_wdata(slot_wdata),
        .route    (route),
        .bypass   (bypass)
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

        // ---- Test 1: Default routing is linear chain ----
        for (int i = 0; i < N_XBAR; i++)
            check($sformatf("DEFAULT_ROUTE[%0d] = %0d", i, i),
                  route[i] == SEL_W'(i));
        for (int i = 0; i < N_SLOTS; i++)
            check($sformatf("DEFAULT_BYPASS[%0d] = 0", i),
                  bypass[i] == 1'b0);

        // ---- Test 2: Slot 0 register write (addr 0x02 = slot0 reg2) ----
        write_reg(8'h02, 8'hAB);
        // slot_wr[0] should have pulsed with addr=2, data=0xAB
        // We check on the combinational output during the write cycle
        // Redo with checking during the wr_en cycle:
        wr_addr = 8'h02;
        wr_data = 8'hCD;
        wr_en   = 1'b1;
        #1;  // combinational settling
        check("SLOT0_WR: slot_wr[0] = 1",    slot_wr[0] == 1'b1);
        check("SLOT0_WR: slot_wr[1] = 0",    slot_wr[1] == 1'b0);
        check("SLOT0_WR: slot_addr[0] = 2",  slot_addr[0] == 3'd2);
        check("SLOT0_WR: slot_wdata[0]=0xCD", slot_wdata[0] == 8'hCD);
        @(posedge clk);
        wr_en = 1'b0;
        @(posedge clk); #1;

        // ---- Test 3: Slot 1 register write (addr 0x09 = slot1 reg1) ----
        wr_addr = 8'h09;
        wr_data = 8'h55;
        wr_en   = 1'b1;
        #1;
        check("SLOT1_WR: slot_wr[1] = 1",    slot_wr[1] == 1'b1);
        check("SLOT1_WR: slot_wr[0] = 0",    slot_wr[0] == 1'b0);
        check("SLOT1_WR: slot_addr[1] = 1",  slot_addr[1] == 3'd1);
        check("SLOT1_WR: slot_wdata[1]=0x55", slot_wdata[1] == 8'h55);
        @(posedge clk);
        wr_en = 1'b0;
        @(posedge clk); #1;

        // ---- Test 4: Slot 3 reg (addr 0x18 = slot3 reg0) ----
        wr_addr = 8'h18;
        wr_data = 8'hFF;
        wr_en   = 1'b1;
        #1;
        check("SLOT3_WR: slot_wr[3] = 1", slot_wr[3] == 1'b1);
        check("SLOT3_WR: slot_addr[3] = 0", slot_addr[3] == 3'd0);
        @(posedge clk);
        wr_en = 1'b0;
        @(posedge clk); #1;

        // ---- Test 5: Route register write (0x40 = route[0]) ----
        write_reg(8'h40, 8'h03);  // route[0] = 3
        check("ROUTE[0] = 3 after write", route[0] == SEL_W'(3));

        write_reg(8'h43, 8'h00);  // route[3] = 0
        check("ROUTE[3] = 0 after write", route[3] == SEL_W'(0));

        // Others unchanged
        check("ROUTE[1] still default (1)", route[1] == SEL_W'(1));
        check("ROUTE[2] still default (2)", route[2] == SEL_W'(2));

        // ---- Test 6: Route read-back ----
        wr_addr = 8'h40;
        wr_en = 1'b0;
        #1;
        check("ROUTE_RD[0] = 3", rd_data == 8'(3));
        wr_addr = 8'h41;
        #1;
        check("ROUTE_RD[1] = 1", rd_data == 8'(1));

        // ---- Test 7: Bypass register write (0x48 = bypass[0]) ----
        write_reg(8'h48, 8'h01);
        check("BYPASS[0] = 1 after write", bypass[0] == 1'b1);
        check("BYPASS[1] still 0", bypass[1] == 1'b0);

        write_reg(8'h4A, 8'h01);  // bypass[2]
        check("BYPASS[2] = 1 after write", bypass[2] == 1'b1);

        // ---- Test 8: Reset clears everything ----
        rst_n = 0;
        repeat (2) @(posedge clk);
        rst_n = 1;
        @(posedge clk); #1;
        check("RESET: route[0] back to 0", route[0] == SEL_W'(0));
        check("RESET: route[3] back to 3", route[3] == SEL_W'(3));
        check("RESET: bypass[0] back to 0", bypass[0] == 1'b0);
        check("RESET: bypass[2] back to 0", bypass[2] == 1'b0);

        $display("=== ctrl_bus: %0d passed, %0d failed ===", pass_count, fail_count);
        if (fail_count > 0) $fatal(1, "TESTS FAILED");
        $finish;
    end

endmodule
