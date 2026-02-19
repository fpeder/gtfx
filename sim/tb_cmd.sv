`timescale 1ns / 1ps

module cmd_proc_tb;

  // Signals
  logic        clk = 0;
  logic        rst_n;
  logic        rx_valid;
  logic [ 7:0] rx_byte;
  logic        tx_done;
  logic        tx_busy;
  logic        tx_start;
  logic [ 7:0] tx_byte;
  logic [ 7:0] level_val;
  logic [ 7:0] feedback_val;
  logic [31:0] time_val;

  // Test tracking
  int          test_passed = 0;
  int          test_failed = 0;

  // Clock generation - 100MHz
  always #5 clk = ~clk;

  // DUT instantiation
  cmd_proc dut (.*);

  // TX capture logic - runs in parallel
  string tx_output = "";

  always @(posedge clk) begin
    if (tx_start && !tx_busy) begin
      tx_output = {tx_output, string'(tx_byte)};

      // Respond with handshake
      tx_busy <= #1 1;
      repeat (3) @(posedge clk);
      tx_done <= #1 1;
      @(posedge clk);
      tx_done <= #1 0;
      tx_busy <= #1 0;
    end
  end

  // Helper tasks
  task send_char(input logic [7:0] c);
    @(posedge clk);
    rx_byte  = c;
    rx_valid = 1;
    @(posedge clk);
    rx_valid = 0;
    repeat (100) @(posedge clk);  // Wait for echo
  endtask

  task send_string(input string s);
    for (int i = 0; i < s.len(); i++) begin
      send_char(s[i]);
    end
  endtask

  task send_enter();
    tx_output = "";  // Clear echoed characters before sending Enter
    send_char(8'h0D);
    repeat (300) @(posedge clk);  // Wait for response
  endtask

  task clear_output();
    tx_output = "";
  endtask

  task check(input string name, input logic pass, input string msg = "");
    if (pass) begin
      $display("  [PASS] %s %s", name, msg);
      test_passed++;
    end else begin
      $display("  [FAIL] %s %s", name, msg);
      test_failed++;
    end
  endtask

  // Test stimulus
  initial begin
    $display("\n========================================");
    $display("   cmd_proc Comprehensive Testbench");
    $display("========================================\n");

    // Initialize
    rst_n = 0;
    rx_valid = 0;
    rx_byte = 0;
    tx_done = 0;
    tx_busy = 0;

    // Reset
    repeat (5) @(posedge clk);
    rst_n = 1;
    repeat (200) @(posedge clk);

    $display("TEST 1: Boot Prompt");
    $display("  Output: '%s'", tx_output);
    check("Boot prompt has \\r\\n", tx_output[0] == 8'h0D && tx_output[1] == 8'h0A);
    check("Boot prompt has Arty>", tx_output.substr(2, 7) == "Arty> ");
    clear_output();

    $display("\nTEST 2: Empty Enter");
    send_enter();
    $display("  Output: '%s'", tx_output);
    check("Empty enter has \\r\\n", tx_output[0] == 8'h0D && tx_output[1] == 8'h0A);
    check("Empty enter has Arty>", tx_output.substr(2, 7) == "Arty> ");
    clear_output();

    $display("\nTEST 3: Character Echo");
    send_char("h");
    check("Echo 'h'", tx_output == "h");
    clear_output();
    send_char("i");
    check("Echo 'i'", tx_output == "i");
    clear_output();
    send_enter();
    check("Unknown cmd has \\r\\n", tx_output[0] == 8'h0D && tx_output[1] == 8'h0A);
    check("Unknown cmd has Arty>", tx_output.substr(2, 7) == "Arty> ");
    clear_output();

    $display("\nTEST 4: Status Command");
    send_string("st");
    send_enter();
    $display("  Output: '%s'", tx_output);
    check("Status has \\r\\n", tx_output[0] == 8'h0D && tx_output[1] == 8'h0A);
    check("Status has L:", tx_output.substr(2, 3) == "L:");
    check("Status has F:", tx_output.substr(7, 8) == "F:");
    check("Status has T:", tx_output.substr(12, 13) == "T:");
    check("Status ends with prompt", tx_output.substr(tx_output.len() - 6, tx_output.len() - 1
          ) == "Arty> ");
    clear_output();

    $display("\nTEST 5: Set Level to 0xAB");
    send_string("set level AB");
    send_enter();
    $display("  Output: '%s' (len=%0d)", tx_output, tx_output.len());
    for (int i = 0; i < tx_output.len(); i++) begin
      $write("    [%0d]=0x%02X ", i, tx_output[i]);
      if ((i + 1) % 8 == 0) $display("");
    end
    $display("");
    // Output is: \r\nOK\r\nArty> 
    check("Enter echo \\r\\n", tx_output[0] == 8'h0D && tx_output[1] == 8'h0A);
    check("Set level OK", tx_output.substr(2, 3) == "OK");
    check("Level register = 0xAB", level_val == 8'hAB, $sformatf("Got 0x%02X", level_val));
    clear_output();

    $display("\nTEST 6: Set Feedback to 0xCD");
    send_string("set feedback CD");
    send_enter();
    $display("  Output: '%s'", tx_output);
    check("Set feedback OK", tx_output.substr(2, 3) == "OK");
    check("Feedback register = 0xCD", feedback_val == 8'hCD, $sformatf("Got 0x%02X", feedback_val));
    clear_output();

    $display("\nTEST 7: Set Time to 0xDEADBEEF");
    send_string("set time DEADBEEF");
    send_enter();
    $display("  Output: '%s'", tx_output);
    check("Set time OK", tx_output.substr(2, 3) == "OK");
    check("Time register = 0xDEADBEEF", time_val == 32'hDEADBEEF, $sformatf("Got 0x%08X", time_val
          ));
    clear_output();

    $display("\nTEST 8: Verify Status After Sets");
    send_string("st");
    send_enter();
    $display("  Output: '%s'", tx_output);
    // Output is: \r\nL:AB F:CD T:DEADBEEF\r\nArty> 
    check("Status shows AB", tx_output.substr(4, 5) == "AB");
    check("Status shows CD", tx_output.substr(9, 10) == "CD");
    check("Status shows DEADBEEF", tx_output.substr(14, 21) == "DEADBEEF");
    clear_output();

    $display("\nTEST 9: Backspace");
    send_string("stx");
    send_char(8'h08);  // Backspace
    repeat (50) @(posedge clk);
    clear_output();  // Clear the backspace echo
    send_enter();
    $display("  Output: '%s'", tx_output);
    check("Backspace corrects to 'st'", tx_output.substr(2, 3) == "L:");
    clear_output();

    $display("\nTEST 10: Invalid Set Command");
    send_string("set invalid 11");
    send_enter();
    $display("  Output: '%s'", tx_output);
    check("Error response", tx_output.substr(2, 4) == "Err");
    clear_output();

    $display("\nTEST 11: Lowercase Hex Digits");
    send_string("set level ff");
    $display("  Before enter: level_val = 0x%02X", level_val);
    $display("  cmd_buf contents:");
    for (int i = 0; i < 15; i++) begin
      $display("    cmd_buf[%0d] = 0x%02X '%c'", i, dut.cmd_buf[i],
               (dut.cmd_buf[i] >= 32 && dut.cmd_buf[i] < 127) ? dut.cmd_buf[i] : 8'h2E);
    end
    send_enter();
    $display("  After enter: level_val = 0x%02X", level_val);
    $display("  Output: '%s'", tx_output);
    check("Lowercase hex OK", tx_output.substr(2, 3) == "OK");
    check("Level = 0xFF", level_val == 8'hFF, $sformatf("Got 0x%02X", level_val));
    clear_output();

    $display("\nTEST 12: DEL Key (0x7F)");
    send_string("abc");
    send_char(8'h7F);  // DEL
    repeat (50) @(posedge clk);
    clear_output();
    send_enter();
    check("DEL has \\r\\n", tx_output[0] == 8'h0D && tx_output[1] == 8'h0A);
    check("DEL has Arty>", tx_output.substr(2, 7) == "Arty> ");
    clear_output();

    $display("\nTEST 13: Enter Echo Format");
    send_string("test");
    clear_output();
    send_enter();
    check("Enter sends \\r first", tx_output[0] == 8'h0D);
    check("Enter sends \\n second", tx_output[1] == 8'h0A);
    clear_output();

    // Summary
    $display("\n========================================");
    $display("           TEST SUMMARY");
    $display("========================================");
    $display("  Passed: %0d", test_passed);
    $display("  Failed: %0d", test_failed);

    if (test_failed == 0) begin
      $display("\n  *** ALL TESTS PASSED ***\n");
    end else begin
      $display("\n  *** SOME TESTS FAILED ***\n");
    end

    repeat (20) @(posedge clk);
    $finish;
  end

  // Timeout
  initial begin
    #500000;  // 500us
    $display("\n*** TIMEOUT ***");
    $finish;
  end

endmodule
