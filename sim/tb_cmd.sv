`timescale 1ns / 1ps

module tb_cmd;

  // ---------------------------------------------------------------
  // Parameters - use high baud rate to keep simulation fast
  // ---------------------------------------------------------------
  localparam int CLK_FREQ   = 10_000_000;   // 10 MHz
  localparam int BAUD_RATE  = 1_000_000;    // 1 Mbaud
  localparam int FIFO_DEPTH = 16;
  localparam int CLK_PERIOD = 100;           // 100 ns -> 10 MHz
  localparam int BIT_PERIOD = CLK_FREQ / BAUD_RATE * CLK_PERIOD; // ns per UART bit

  // ---------------------------------------------------------------
  // DUT signals
  // ---------------------------------------------------------------
  logic        clk = 0;
  logic        rst_n = 0;
  logic        tx_din = 1;    // serial line into DUT RX (idle high)
  logic        rx_dout;       // serial line out of DUT TX
  logic [ 3:0] sw_effect = 4'b1111;

  logic [ 7:0] tone_val, level_val, feedback_val;
  logic [31:0] time_val;
  logic [ 7:0] chorus_rate_val, chorus_depth_val, chorus_efx_val;
  logic [ 7:0] chorus_eqhi_val, chorus_eqlo_val;
  logic [ 7:0] phaser_speed_val;
  logic        phaser_fben_val;
  logic [ 7:0] trem_rate_val, trem_depth_val;
  logic        trem_shape_val;

  // ---------------------------------------------------------------
  // Clock generation
  // ---------------------------------------------------------------
  always #(CLK_PERIOD / 2) clk = ~clk;

  // ---------------------------------------------------------------
  // DUT instantiation
  // ---------------------------------------------------------------
  cmd #(
      .CLK_FREQ  (CLK_FREQ),
      .BAUD_RATE (BAUD_RATE),
      .FIFO_DEPTH(FIFO_DEPTH)
  ) dut (
      .sys_clk   (clk),
      .rst_n     (rst_n),
      .tx_din    (tx_din),
      .rx_dout   (rx_dout),
      .sw_effect (sw_effect),
      .tone_val        (tone_val),
      .level_val       (level_val),
      .feedback_val    (feedback_val),
      .time_val        (time_val),
      .chorus_rate_val (chorus_rate_val),
      .chorus_depth_val(chorus_depth_val),
      .chorus_efx_val  (chorus_efx_val),
      .chorus_eqhi_val (chorus_eqhi_val),
      .chorus_eqlo_val (chorus_eqlo_val),
      .phaser_speed_val(phaser_speed_val),
      .phaser_fben_val (phaser_fben_val),
      .trem_rate_val   (trem_rate_val),
      .trem_depth_val  (trem_depth_val),
      .trem_shape_val  (trem_shape_val)
  );

  // ---------------------------------------------------------------
  // UART TX task - send one byte serially into DUT (tx_din)
  // ---------------------------------------------------------------
  task automatic uart_send_byte(input [7:0] data);
    // Start bit
    tx_din = 0;
    #(BIT_PERIOD);
    // Data bits (LSB first)
    for (int i = 0; i < 8; i++) begin
      tx_din = data[i];
      #(BIT_PERIOD);
    end
    // Stop bit
    tx_din = 1;
    #(BIT_PERIOD);
  endtask

  // ---------------------------------------------------------------
  // UART TX task - send a string (each char as a byte)
  // ---------------------------------------------------------------
  task automatic uart_send_string(input string s);
    for (int i = 0; i < s.len(); i++) begin
      uart_send_byte(s[i]);
    end
  endtask

  // ---------------------------------------------------------------
  // UART TX task - send a string followed by CR
  // ---------------------------------------------------------------
  task automatic uart_send_cmd(input string s);
    uart_send_string(s);
    uart_send_byte(8'h0D);  // CR
  endtask

  // ---------------------------------------------------------------
  // UART RX task - receive one byte from DUT (rx_dout)
  // ---------------------------------------------------------------
  task automatic uart_recv_byte(output [7:0] data);
    // Wait for start bit (falling edge)
    @(negedge rx_dout);
    // Sample at mid-bit
    #(BIT_PERIOD / 2);
    // Verify start bit
    if (rx_dout !== 0) begin
      $display("[%0t] ERROR: Expected start bit=0, got %b", $time, rx_dout);
    end
    // Sample 8 data bits
    for (int i = 0; i < 8; i++) begin
      #(BIT_PERIOD);
      data[i] = rx_dout;
    end
    // Wait through stop bit
    #(BIT_PERIOD);
  endtask

  // ---------------------------------------------------------------
  // Background RX collector - captures everything DUT transmits
  // ---------------------------------------------------------------
  logic [7:0] rx_log     [0:511];
  int         rx_log_idx = 0;
  string      rx_string  = "";
  logic       rx_collector_en = 1;

  always begin
    logic [7:0] b;
    if (rx_collector_en) begin
      uart_recv_byte(b);
      rx_log[rx_log_idx] = b;
      rx_log_idx++;
      if (b >= 8'h20 && b <= 8'h7E)
        rx_string = {rx_string, string'(b)};
      else if (b == 8'h0D)
        rx_string = {rx_string, "\\r"};
      else if (b == 8'h0A)
        rx_string = {rx_string, "\\n"};
      else if (b == 8'h08)
        rx_string = {rx_string, "<BS>"};
      else
        rx_string = {rx_string, "."};
    end else begin
      #(CLK_PERIOD);
    end
  end

  // ---------------------------------------------------------------
  // Helper: wait for DUT to finish transmitting
  //   Line must stay HIGH (idle) for idle_bits consecutive bit
  //   periods before we declare transmission complete.
  // ---------------------------------------------------------------
  task automatic wait_tx_idle(input int idle_bits = 20);
    int idle_count;
    idle_count = 0;
    while (idle_count < idle_bits) begin
      #(BIT_PERIOD);
      if (rx_dout === 1'b1)
        idle_count++;
      else
        idle_count = 0;
    end
  endtask

  // ---------------------------------------------------------------
  // Helper: clear the RX log
  // ---------------------------------------------------------------
  task automatic clear_rx_log();
    rx_log_idx = 0;
    rx_string  = "";
  endtask

  // ---------------------------------------------------------------
  // Helper: print captured response
  // ---------------------------------------------------------------
  task automatic print_rx_log(input string label);
    $display("[%0t] --- %s --- captured %0d bytes: \"%s\"", $time, label, rx_log_idx, rx_string);
  endtask

  // ---------------------------------------------------------------
  // FIFO internal probes (hierarchy access for verification)
  // ---------------------------------------------------------------
  wire [$clog2(FIFO_DEPTH):0] rx_fifo_count = dut.rx_fifo_inst.count;
  wire [$clog2(FIFO_DEPTH):0] tx_fifo_count = dut.tx_fifo_inst.count;
  wire rx_fifo_full  = dut.rx_fifo_inst.full;
  wire rx_fifo_empty = dut.rx_fifo_inst.empty;
  wire tx_fifo_full  = dut.tx_fifo_inst.full;
  wire tx_fifo_empty = dut.tx_fifo_inst.empty;

  // ---------------------------------------------------------------
  // Test counters
  // ---------------------------------------------------------------
  int test_pass = 0;
  int test_fail = 0;

  task automatic check(input string name, input logic cond);
    if (cond) begin
      $display("[PASS] %s", name);
      test_pass++;
    end else begin
      $display("[FAIL] %s", name);
      test_fail++;
    end
  endtask

  // ===============================================================
  // Main test sequence
  // ===============================================================
  initial begin
    $dumpfile("tb_cmd.vcd");
    $dumpvars(0, cmd_tb);

    $display("========================================");
    $display(" CMD + FIFO Testbench");
    $display(" CLK_FREQ=%0d  BAUD=%0d  FIFO_DEPTH=%0d", CLK_FREQ, BAUD_RATE, FIFO_DEPTH);
    $display(" BIT_PERIOD=%0d ns", BIT_PERIOD);
    $display("========================================");

    // ---- Reset ----
    rst_n = 0;
    #(CLK_PERIOD * 20);
    rst_n = 1;
    #(CLK_PERIOD * 5);

    // ============================================================
    // TEST 1: Boot prompt
    //   After reset, cmd_proc sends CR+LF then "Arty> "
    //   All routed through the TX FIFO -> UART
    // ============================================================
    $display("\n--- TEST 1: Boot prompt ---");
    wait_tx_idle(30);
    print_rx_log("Boot");
    check("Boot: received bytes > 0", rx_log_idx > 0);
    check("Boot: contains prompt chars", rx_log_idx >= 8);
    clear_rx_log();

    // ============================================================
    // TEST 2: Empty command (just CR) -> should get CR+LF + prompt
    // ============================================================
    $display("\n--- TEST 2: Empty CR ---");
    uart_send_byte(8'h0D);
    wait_tx_idle(30);
    print_rx_log("Empty CR");
    check("Empty CR: received response", rx_log_idx > 0);
    clear_rx_log();

    // ============================================================
    // TEST 3: Echo test - send a char and verify it echoes back
    // ============================================================
    $display("\n--- TEST 3: Character echo ---");
    uart_send_byte("h");
    wait_tx_idle(20);
    print_rx_log("Echo 'h'");
    check("Echo: received at least 1 byte", rx_log_idx >= 1);
    check("Echo: first byte is 'h'", rx_log[0] == "h");
    clear_rx_log();

    // ============================================================
    // TEST 4: Status command with all effects enabled
    //   First clear the 'h' in cmd_buf with backspace
    // ============================================================
    $display("\n--- TEST 4: Status command (all effects) ---");
    sw_effect = 4'b1111;
    uart_send_byte(8'h08);  // BS to clear 'h'
    wait_tx_idle(20);
    clear_rx_log();
    uart_send_cmd("st");
    wait_tx_idle(50);
    print_rx_log("Status");
    check("Status: received substantial response", rx_log_idx > 20);
    clear_rx_log();

    // ============================================================
    // TEST 5: Status command bypass mode (no effects)
    // ============================================================
    $display("\n--- TEST 5: Status command (bypass) ---");
    sw_effect = 4'b0000;
    uart_send_cmd("st");
    wait_tx_idle(40);
    print_rx_log("Bypass status");
    check("Bypass: received response", rx_log_idx > 0);
    clear_rx_log();
    sw_effect = 4'b1111;

    // ============================================================
    // TEST 6: Set delay level command
    // ============================================================
    $display("\n--- TEST 6: Set delay level ---");
    uart_send_cmd("set dly level FF");
    wait_tx_idle(30);
    print_rx_log("set dly level FF");
    #(CLK_PERIOD * 10);
    $display("  level_val = 0x%02h", level_val);
    check("Set dly level: level_val == 0xFF", level_val == 8'hFF);
    clear_rx_log();

    // ============================================================
    // TEST 7: Set delay feedback
    // ============================================================
    $display("\n--- TEST 7: Set delay feedback ---");
    uart_send_cmd("set dly feedback A0");
    wait_tx_idle(30);
    print_rx_log("set dly feedback A0");
    #(CLK_PERIOD * 10);
    $display("  feedback_val = 0x%02h", feedback_val);
    check("Set dly feedback: feedback_val == 0xA0", feedback_val == 8'hA0);
    clear_rx_log();

    // ============================================================
    // TEST 8: Set delay time (32-bit value)
    // ============================================================
    $display("\n--- TEST 8: Set delay time ---");
    uart_send_cmd("set dly time 00001000");
    wait_tx_idle(30);
    print_rx_log("set dly time");
    #(CLK_PERIOD * 10);
    $display("  time_val = 0x%08h", time_val);
    check("Set dly time: time_val == 0x00001000", time_val == 32'h00001000);
    clear_rx_log();

    // ============================================================
    // TEST 9: Set delay tone
    // ============================================================
    $display("\n--- TEST 9: Set delay tone ---");
    uart_send_cmd("set dly tone 80");
    wait_tx_idle(30);
    print_rx_log("set dly tone");
    #(CLK_PERIOD * 10);
    $display("  tone_val = 0x%02h", tone_val);
    check("Set dly tone: tone_val == 0x80", tone_val == 8'h80);
    clear_rx_log();

    // ============================================================
    // TEST 10: Set chorus rate
    // ============================================================
    $display("\n--- TEST 10: Set chorus rate ---");
    uart_send_cmd("set cho rate C0");
    wait_tx_idle(30);
    print_rx_log("set cho rate");
    #(CLK_PERIOD * 10);
    $display("  chorus_rate_val = 0x%02h", chorus_rate_val);
    check("Set cho rate: chorus_rate_val == 0xC0", chorus_rate_val == 8'hC0);
    clear_rx_log();

    // ============================================================
    // TEST 11: Set chorus depth
    // ============================================================
    $display("\n--- TEST 11: Set chorus depth ---");
    uart_send_cmd("set cho depth B0");
    wait_tx_idle(30);
    print_rx_log("set cho depth");
    #(CLK_PERIOD * 10);
    $display("  chorus_depth_val = 0x%02h", chorus_depth_val);
    check("Set cho depth: chorus_depth_val == 0xB0", chorus_depth_val == 8'hB0);
    clear_rx_log();

    // ============================================================
    // TEST 12: Set phaser speed
    // ============================================================
    $display("\n--- TEST 12: Set phaser speed ---");
    uart_send_cmd("set pha speed 55");
    wait_tx_idle(30);
    print_rx_log("set pha speed");
    #(CLK_PERIOD * 10);
    $display("  phaser_speed_val = 0x%02h", phaser_speed_val);
    check("Set pha speed: phaser_speed_val == 0x55", phaser_speed_val == 8'h55);
    clear_rx_log();

    // ============================================================
    // TEST 13: Set tremolo rate
    // ============================================================
    $display("\n--- TEST 13: Set tremolo rate ---");
    uart_send_cmd("set trm rate 40");
    wait_tx_idle(30);
    print_rx_log("set trm rate");
    #(CLK_PERIOD * 10);
    $display("  trem_rate_val = 0x%02h", trem_rate_val);
    check("Set trm rate: trem_rate_val == 0x40", trem_rate_val == 8'h40);
    clear_rx_log();

    // ============================================================
    // TEST 14: Invalid command -> "Err" response
    // ============================================================
    $display("\n--- TEST 14: Invalid command ---");
    uart_send_cmd("xyz");
    wait_tx_idle(30);
    print_rx_log("Invalid cmd");
    check("Invalid cmd: received response", rx_log_idx > 0);
    clear_rx_log();

    // ============================================================
    // TEST 15: Backspace handling
    //   Send 'x', wait for echo to fully complete, clear log,
    //   then send DEL and wait for the 3-byte response to finish.
    // ============================================================
    $display("\n--- TEST 15: Backspace ---");
    uart_send_byte("x");
    wait_tx_idle(20);       // wait for 'x' echo to fully transmit
    clear_rx_log();
    uart_send_byte(8'h7F);  // DEL (treated as backspace by cmd_proc)
    wait_tx_idle(20);       // wait for BS+SPC+BS (3 bytes) to finish
    print_rx_log("Backspace");
    check("Backspace: received 3 bytes (BS SPC BS)", rx_log_idx == 3);
    if (rx_log_idx >= 3) begin
      check("Backspace: byte0 == 0x08", rx_log[0] == 8'h08);
      check("Backspace: byte1 == 0x20", rx_log[1] == 8'h20);
      check("Backspace: byte2 == 0x08", rx_log[2] == 8'h08);
    end
    clear_rx_log();
    // Send CR to return to clean state
    uart_send_byte(8'h0D);
    wait_tx_idle(30);
    clear_rx_log();

    // ============================================================
    // TEST 16: Burst input - send multiple chars rapidly
    //   Tests that the RX FIFO buffers without dropping bytes
    // ============================================================
    $display("\n--- TEST 16: Burst RX (FIFO buffering) ---");
    uart_send_cmd("set dly level 42");
    wait_tx_idle(30);
    print_rx_log("Burst set dly level 42");
    #(CLK_PERIOD * 10);
    $display("  level_val = 0x%02h", level_val);
    check("Burst: level_val == 0x42", level_val == 8'h42);
    clear_rx_log();

    // ============================================================
    // TEST 17: Verify TX FIFO drains correctly after long status
    // ============================================================
    $display("\n--- TEST 17: TX FIFO drain after status ---");
    sw_effect = 4'b1111;
    uart_send_cmd("st");
    wait_tx_idle(50);
    print_rx_log("Full status drain");
    #(CLK_PERIOD * 20);
    check("TX FIFO empty after drain", tx_fifo_empty === 1'b1);
    check("RX FIFO empty after processing", rx_fifo_empty === 1'b1);
    clear_rx_log();

    // ============================================================
    // TEST 18: Set multiple registers in sequence
    // ============================================================
    $display("\n--- TEST 18: Sequential commands ---");
    uart_send_cmd("set dly level 11");
    wait_tx_idle(30);
    clear_rx_log();
    uart_send_cmd("set dly feedback 22");
    wait_tx_idle(30);
    clear_rx_log();
    uart_send_cmd("set dly tone 33");
    wait_tx_idle(30);
    clear_rx_log();
    #(CLK_PERIOD * 10);
    $display("  level_val    = 0x%02h", level_val);
    $display("  feedback_val = 0x%02h", feedback_val);
    $display("  tone_val     = 0x%02h", tone_val);
    check("Seq: level_val == 0x11",    level_val    == 8'h11);
    check("Seq: feedback_val == 0x22", feedback_val == 8'h22);
    check("Seq: tone_val == 0x33",     tone_val     == 8'h33);

    // ============================================================
    // TEST 19: Set chorus EFX level
    // ============================================================
    $display("\n--- TEST 19: Set chorus efx ---");
    uart_send_cmd("set cho efx 99");
    wait_tx_idle(30);
    print_rx_log("set cho efx");
    #(CLK_PERIOD * 10);
    $display("  chorus_efx_val = 0x%02h", chorus_efx_val);
    check("Set cho efx: chorus_efx_val == 0x99", chorus_efx_val == 8'h99);
    clear_rx_log();

    // ============================================================
    // TEST 20: FIFO status check - both FIFOs idle
    // ============================================================
    $display("\n--- TEST 20: Final FIFO state ---");
    #(CLK_PERIOD * 100);
    check("Final: TX FIFO empty", tx_fifo_empty === 1'b1);
    check("Final: RX FIFO empty", rx_fifo_empty === 1'b1);
    check("Final: TX FIFO not full", tx_fifo_full === 1'b0);
    check("Final: RX FIFO not full", rx_fifo_full === 1'b0);

    // ============================================================
    // Summary
    // ============================================================
    $display("\n========================================");
    $display(" Test Summary: %0d PASSED, %0d FAILED", test_pass, test_fail);
    $display("========================================");
    if (test_fail > 0)
      $display(" *** SOME TESTS FAILED ***");
    else
      $display(" ALL TESTS PASSED");
    $display("");

    #(CLK_PERIOD * 10);
    $finish;
  end

  // ---------------------------------------------------------------
  // Timeout watchdog
  // ---------------------------------------------------------------
  initial begin
    #(500_000_000);  // 500 ms
    $display("[TIMEOUT] Simulation exceeded max time");
    $finish;
  end

endmodule