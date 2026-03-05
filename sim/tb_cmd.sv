`timescale 1ns / 1ps

// ============================================================================
// tb_cmd.sv — Testbench for cmd (UART + cmd_proc + FIFOs)
//
// The cmd module now outputs a flat register write bus:
//   wr_en (toggle), wr_addr, wr_data
// Each toggle edge on wr_en captures a new register write.
//
// Tests:
//   1. Boot prompt
//   2. Empty CR → prompt
//   3. Character echo
//   4. Set delay repeats
//   5. Set chorus rate
//   6. Set phaser speed
//   7. Set tremolo rate
//   8. Set compressor threshold
//   9. Set compressor ratio (new effect)
//  10. Set wah frequency (new effect)
//  11. Set big muff sustain
//  12. Set reverb decay
//  13. Invalid command → Err
//  14. Backspace handling
//  15. Sequential commands
// ============================================================================

module tb_cmd;

  // ---------------------------------------------------------------
  // Parameters - use high baud rate to keep simulation fast
  // ---------------------------------------------------------------
  localparam int CLK_FREQ   = 10_000_000;
  localparam int BAUD_RATE  = 1_000_000;
  localparam int FIFO_DEPTH = 16;
  localparam int CLK_PERIOD = 100;           // 100 ns -> 10 MHz
  localparam int BIT_PERIOD = CLK_FREQ / BAUD_RATE * CLK_PERIOD;

  // ---------------------------------------------------------------
  // DUT signals
  // ---------------------------------------------------------------
  logic        clk = 0;
  logic        rst_n = 0;
  logic        tx_din = 1;    // serial line into DUT RX (idle high)
  logic        rx_dout;       // serial line out of DUT TX

  // Flat register write bus (toggle CDC)
  logic        wr_en;
  logic [7:0]  wr_addr;
  logic [7:0]  wr_data;

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
      .wr_en     (wr_en),
      .wr_addr   (wr_addr),
      .wr_data   (wr_data)
  );

  // ---------------------------------------------------------------
  // Write bus monitor — capture register writes on toggle edges
  // ---------------------------------------------------------------
  logic        wr_en_prev = 0;
  logic [7:0]  last_wr_addr;
  logic [7:0]  last_wr_data;
  logic        wr_seen;
  int          wr_count = 0;

  always_ff @(posedge clk) begin
    wr_en_prev <= wr_en;
    if (wr_en !== wr_en_prev) begin
      last_wr_addr <= wr_addr;
      last_wr_data <= wr_data;
      wr_seen      <= 1'b1;
      wr_count     <= wr_count + 1;
    end else begin
      wr_seen <= 1'b0;
    end
  end

  task automatic clear_wr_log();
    wr_count = 0;
  endtask

  // Wait for a write to occur (with timeout)
  task automatic wait_for_write(input int timeout_bits = 100);
    int waited = 0;
    while (!wr_seen && waited < timeout_bits) begin
      @(posedge clk);
      waited++;
    end
  endtask

  // ---------------------------------------------------------------
  // UART TX task - send one byte serially into DUT (tx_din)
  // ---------------------------------------------------------------
  task automatic uart_send_byte(input [7:0] data);
    tx_din = 0;           // Start bit
    #(BIT_PERIOD);
    for (int i = 0; i < 8; i++) begin
      tx_din = data[i];   // Data bits (LSB first)
      #(BIT_PERIOD);
    end
    tx_din = 1;           // Stop bit
    #(BIT_PERIOD);
  endtask

  task automatic uart_send_string(input string s);
    for (int i = 0; i < s.len(); i++)
      uart_send_byte(s[i]);
  endtask

  task automatic uart_send_cmd(input string s);
    uart_send_string(s);
    uart_send_byte(8'h0D);  // CR
  endtask

  // ---------------------------------------------------------------
  // UART RX task - receive one byte from DUT (rx_dout)
  // ---------------------------------------------------------------
  task automatic uart_recv_byte(output [7:0] data);
    @(negedge rx_dout);
    #(BIT_PERIOD / 2);
    if (rx_dout !== 0)
      $display("[%0t] ERROR: Expected start bit=0, got %b", $time, rx_dout);
    for (int i = 0; i < 8; i++) begin
      #(BIT_PERIOD);
      data[i] = rx_dout;
    end
    #(BIT_PERIOD);
  endtask

  // ---------------------------------------------------------------
  // Background RX collector
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

  task automatic clear_rx_log();
    rx_log_idx = 0;
    rx_string  = "";
  endtask

  task automatic print_rx_log(input string label);
    $display("[%0t] --- %s --- captured %0d bytes: \"%s\"", $time, label, rx_log_idx, rx_string);
  endtask

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
    $dumpvars(0, tb_cmd);

    $display("========================================");
    $display(" CMD Testbench (flat write bus)");
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
    // ============================================================
    $display("\n--- TEST 1: Boot prompt ---");
    wait_tx_idle(30);
    print_rx_log("Boot");
    check("Boot: received bytes > 0", rx_log_idx > 0);
    clear_rx_log();

    // ============================================================
    // TEST 2: Empty command (just CR) -> should get prompt
    // ============================================================
    $display("\n--- TEST 2: Empty CR ---");
    uart_send_byte(8'h0D);
    wait_tx_idle(30);
    print_rx_log("Empty CR");
    check("Empty CR: received response", rx_log_idx > 0);
    clear_rx_log();

    // ============================================================
    // TEST 3: Character echo
    // ============================================================
    $display("\n--- TEST 3: Character echo ---");
    uart_send_byte("h");
    wait_tx_idle(20);
    print_rx_log("Echo 'h'");
    check("Echo: received at least 1 byte", rx_log_idx >= 1);
    check("Echo: first byte is 'h'", rx_log[0] == "h");
    clear_rx_log();
    // Clear the 'h' from buffer
    uart_send_byte(8'h7F);
    wait_tx_idle(20);
    clear_rx_log();

    // ============================================================
    // TEST 4: Set delay repeats (addr 0x18)
    // ============================================================
    $display("\n--- TEST 4: Set delay repeats ---");
    clear_wr_log();
    uart_send_cmd("set dly rpt 80");
    wait_tx_idle(40);
    print_rx_log("set dly rpt 80");
    check("Set dly rpt: write occurred", wr_count > 0);
    check("Set dly rpt: addr=0x18", last_wr_addr == 8'h18);
    check("Set dly rpt: data=0x80", last_wr_data == 8'h80);
    clear_rx_log();

    // ============================================================
    // TEST 5: Set chorus rate (addr 0x10)
    // ============================================================
    $display("\n--- TEST 5: Set chorus rate ---");
    clear_wr_log();
    uart_send_cmd("set cho rate C0");
    wait_tx_idle(40);
    print_rx_log("set cho rate C0");
    check("Set cho rate: write occurred", wr_count > 0);
    check("Set cho rate: addr=0x10", last_wr_addr == 8'h10);
    check("Set cho rate: data=0xC0", last_wr_data == 8'hC0);
    clear_rx_log();

    // ============================================================
    // TEST 6: Set phaser speed (addr 0x08)
    // ============================================================
    $display("\n--- TEST 6: Set phaser speed ---");
    clear_wr_log();
    uart_send_cmd("set pha speed 55");
    wait_tx_idle(40);
    print_rx_log("set pha speed 55");
    check("Set pha speed: write occurred", wr_count > 0);
    check("Set pha speed: addr=0x08", last_wr_addr == 8'h08);
    check("Set pha speed: data=0x55", last_wr_data == 8'h55);
    clear_rx_log();

    // ============================================================
    // TEST 7: Set tremolo rate (addr 0x00)
    // ============================================================
    $display("\n--- TEST 7: Set tremolo rate ---");
    clear_wr_log();
    uart_send_cmd("set trm rate 40");
    wait_tx_idle(40);
    print_rx_log("set trm rate 40");
    check("Set trm rate: write occurred", wr_count > 0);
    check("Set trm rate: addr=0x00", last_wr_addr == 8'h00);
    check("Set trm rate: data=0x40", last_wr_data == 8'h40);
    clear_rx_log();

    // ============================================================
    // TEST 8: Set compressor threshold (addr 0x40)
    // ============================================================
    $display("\n--- TEST 8: Set compressor threshold ---");
    clear_wr_log();
    uart_send_cmd("set cmp thr 80");
    wait_tx_idle(40);
    print_rx_log("set cmp thr 80");
    check("Set cmp thr: write occurred", wr_count > 0);
    check("Set cmp thr: addr=0x40", last_wr_addr == 8'h40);
    check("Set cmp thr: data=0x80", last_wr_data == 8'h80);
    clear_rx_log();

    // ============================================================
    // TEST 9: Set compressor ratio (addr 0x41)
    // ============================================================
    $display("\n--- TEST 9: Set compressor ratio ---");
    clear_wr_log();
    uart_send_cmd("set cmp rat 40");
    wait_tx_idle(40);
    print_rx_log("set cmp rat 40");
    check("Set cmp rat: write occurred", wr_count > 0);
    check("Set cmp rat: addr=0x41", last_wr_addr == 8'h41);
    check("Set cmp rat: data=0x40", last_wr_data == 8'h40);
    clear_rx_log();

    // ============================================================
    // TEST 10: Set wah frequency (addr 0x48)
    // ============================================================
    $display("\n--- TEST 10: Set wah frequency ---");
    clear_wr_log();
    uart_send_cmd("set wah frq A0");
    wait_tx_idle(40);
    print_rx_log("set wah frq A0");
    check("Set wah frq: write occurred", wr_count > 0);
    check("Set wah frq: addr=0x48", last_wr_addr == 8'h48);
    check("Set wah frq: data=0xA0", last_wr_data == 8'hA0);
    clear_rx_log();

    // ============================================================
    // TEST 11: Set big muff sustain (addr 0x30)
    // ============================================================
    $display("\n--- TEST 11: Set big muff sustain ---");
    clear_wr_log();
    uart_send_cmd("set bmf sus FF");
    wait_tx_idle(40);
    print_rx_log("set bmf sus FF");
    check("Set bmf sus: write occurred", wr_count > 0);
    check("Set bmf sus: addr=0x30", last_wr_addr == 8'h30);
    check("Set bmf sus: data=0xFF", last_wr_data == 8'hFF);
    clear_rx_log();

    // ============================================================
    // TEST 12: Set reverb decay (addr 0x38)
    // ============================================================
    $display("\n--- TEST 12: Set reverb decay ---");
    clear_wr_log();
    uart_send_cmd("set rev dec C0");
    wait_tx_idle(40);
    print_rx_log("set rev dec C0");
    check("Set rev dec: write occurred", wr_count > 0);
    check("Set rev dec: addr=0x38", last_wr_addr == 8'h38);
    check("Set rev dec: data=0xC0", last_wr_data == 8'hC0);
    clear_rx_log();

    // ============================================================
    // TEST 13: Invalid command -> "Err" response
    // ============================================================
    $display("\n--- TEST 13: Invalid command ---");
    uart_send_cmd("xyz");
    wait_tx_idle(30);
    print_rx_log("Invalid cmd");
    check("Invalid cmd: received response", rx_log_idx > 0);
    clear_rx_log();

    // ============================================================
    // TEST 14: Backspace handling
    // ============================================================
    $display("\n--- TEST 14: Backspace ---");
    uart_send_byte("x");
    wait_tx_idle(20);
    clear_rx_log();
    uart_send_byte(8'h7F);  // DEL
    wait_tx_idle(20);
    print_rx_log("Backspace");
    check("Backspace: received 3 bytes (BS SPC BS)", rx_log_idx == 3);
    if (rx_log_idx >= 3) begin
      check("Backspace: byte0 == 0x08", rx_log[0] == 8'h08);
      check("Backspace: byte1 == 0x20", rx_log[1] == 8'h20);
      check("Backspace: byte2 == 0x08", rx_log[2] == 8'h08);
    end
    clear_rx_log();
    uart_send_byte(8'h0D);
    wait_tx_idle(30);
    clear_rx_log();

    // ============================================================
    // TEST 15: Sequential commands
    // ============================================================
    $display("\n--- TEST 15: Sequential commands ---");
    clear_wr_log();
    uart_send_cmd("set cmp atk 10");
    wait_tx_idle(30);
    check("Seq cmp atk: addr=0x42", last_wr_addr == 8'h42);
    check("Seq cmp atk: data=0x10", last_wr_data == 8'h10);
    clear_rx_log();

    clear_wr_log();
    uart_send_cmd("set cmp mak 60");
    wait_tx_idle(30);
    check("Seq cmp mak: addr=0x44", last_wr_addr == 8'h44);
    check("Seq cmp mak: data=0x60", last_wr_data == 8'h60);
    clear_rx_log();

    clear_wr_log();
    uart_send_cmd("set wah res B0");
    wait_tx_idle(30);
    check("Seq wah res: addr=0x49", last_wr_addr == 8'h49);
    check("Seq wah res: data=0xB0", last_wr_data == 8'hB0);
    clear_rx_log();

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
    #(500_000_000);
    $display("[TIMEOUT] Simulation exceeded max time");
    $finish;
  end

endmodule
