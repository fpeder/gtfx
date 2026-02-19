`timescale 1ns / 1ps

// =============================================================================
// tb_cmd_proc.sv  -  Directed self-checking testbench for cmd_proc.sv
//
// Strategy:
//   - Emulate UART byte arrival via rx_valid/rx_byte
//   - Emulate UART TX handshake via tx_busy/tx_done
//   - Capture every tx_byte the DUT sends into a string buffer
//   - Compare captured output against expected strings
// =============================================================================

module tb_cmd_proc;

  // -----------------------------------------------------------------------
  // Clock
  // -----------------------------------------------------------------------
  localparam int CLK_PERIOD = 10; // 100 MHz, 10 ns
  logic clk = 0;
  always #(CLK_PERIOD/2) clk = ~clk;

  // -----------------------------------------------------------------------
  // DUT signals
  // -----------------------------------------------------------------------
  logic        rst_n        = 0;
  logic        rx_valid     = 0;
  logic [ 7:0] rx_byte      = 0;
  logic        tx_done      = 0;
  logic        tx_busy      = 0;
  logic        tx_start;
  logic [ 7:0] tx_byte;

  logic [ 3:0] sw_effect    = 4'b0000; // ADDED: switch state

  logic [ 7:0] tone_val;
  logic [ 7:0] level_val;
  logic [ 7:0] feedback_val;
  logic [31:0] time_val;

  logic [ 7:0] chorus_rate_val;
  logic [ 7:0] chorus_depth_val;
  logic [ 7:0] chorus_efx_val;
  logic [ 7:0] chorus_eqhi_val;
  logic [ 7:0] chorus_eqlo_val;

  logic [ 7:0] phaser_speed_val;
  logic        phaser_fben_val;

  logic [ 7:0] trem_rate_val;
  logic [ 7:0] trem_depth_val;
  logic        trem_shape_val;

  // -----------------------------------------------------------------------
  // DUT instance
  // -----------------------------------------------------------------------
  cmd_proc dut (
    .clk             (clk),
    .rst_n           (rst_n),
    .rx_valid        (rx_valid),
    .rx_byte         (rx_byte),
    .tx_done         (tx_done),
    .tx_busy         (tx_busy),
    .tx_start        (tx_start),
    .tx_byte         (tx_byte),
    .sw_effect       (sw_effect),      // ADDED: bind switch port
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

  // -----------------------------------------------------------------------
  // TX capture buffer + parallel auto-ack
  // -----------------------------------------------------------------------
  localparam int TX_BUF_MAX = 4096;
  logic [7:0] tx_captured [0:TX_BUF_MAX-1];
  int         tx_cap_len = 0; 

  always @(posedge clk) begin : tx_autoack
    if (tx_start && !tx_busy) begin
      if (tx_cap_len < TX_BUF_MAX)
        tx_captured[tx_cap_len] = tx_byte;
      tx_cap_len = tx_cap_len + 1;
      tx_busy    <= 1;
    end else if (tx_busy && !tx_done) begin
      tx_done <= 1;
    end else if (tx_done) begin
      tx_done <= 0;
      tx_busy <= 0;
    end
  end

  // -----------------------------------------------------------------------
  // Task: send a single byte as if received from UART
  // -----------------------------------------------------------------------
  task automatic send_byte(input logic [7:0] b);
    int wait_cnt;
    @(posedge clk);
    rx_byte  = b;
    rx_valid = 1;
    @(posedge clk);
    rx_valid = 0;
    rx_byte  = 0;
    
    wait_cnt = 0;
    while (!tx_busy && wait_cnt < 20) begin
      @(posedge clk);
      wait_cnt++;
    end

    wait_cnt = 0;
    while (wait_cnt < 10) begin
      @(posedge clk);
      if (tx_busy) wait_cnt = 0;
      else         wait_cnt++;
    end
  endtask

  // -----------------------------------------------------------------------
  // Task: send a null-terminated string then CR
  // -----------------------------------------------------------------------
  task automatic send_cmd(input string s);
    for (int i = 0; i < s.len(); i++)
      send_byte(s[i]);
    send_byte(8'h0D); // CR
  endtask

  // -----------------------------------------------------------------------
  // drain_tx: collect TX bytes from start_idx onward.
  // -----------------------------------------------------------------------
  task automatic drain_tx(input int timeout_cycles, input int start_idx,
                          input int skip_count, output string result);
    int rd, last_cap, idle_cnt, skipped;
    logic [7:0] b;
    result   = "";
    rd       = start_idx;
    last_cap = tx_cap_len;
    idle_cnt = 0;
    skipped  = 0;

    while (idle_cnt < timeout_cycles) begin
      @(posedge clk);
      if (tx_cap_len != last_cap) begin
        while (rd < tx_cap_len) begin
          b = tx_captured[rd];
          rd++;
          if (b >= 32 && b <= 126) begin // printable
            if (skipped < skip_count)
              skipped++;
            else
              result = {result, string'(b)};
          end
        end
        last_cap = tx_cap_len;
        idle_cnt = 0;
      end else begin
        idle_cnt++;
      end
    end
  endtask

  // -----------------------------------------------------------------------
  // Checkers
  // -----------------------------------------------------------------------
  int pass_cnt = 0;
  int fail_cnt = 0;

  task automatic check_str(input string test_name, input string got, input string expected);
    if (got == expected) begin
      $display("  PASS  %s", test_name);
      pass_cnt++;
    end else begin
      $display("  FAIL  %s\n        expected: \"%s\"\n        got:      \"%s\"", test_name, expected, got);
      fail_cnt++;
    end
  endtask

  task automatic check_byte(input string test_name, input logic [7:0] got, input logic [7:0] expected);
    if (got === expected) begin
      $display("  PASS  %s: port=%02X", test_name, expected);
      pass_cnt++;
    end else begin
      $display("  FAIL  %s: port=%02X  (got=%02x, expected=%02x)", test_name, expected, got, expected);
      fail_cnt++;
    end
  endtask

  task automatic check_word(input string test_name, input logic [31:0] got, input logic [31:0] expected);
    if (got === expected) begin
      $display("  PASS  %s: port=%08X", test_name, expected);
      pass_cnt++;
    end else begin
      $display("  FAIL  %s: port=%08X  (got=%08x, expected=%08x)", test_name, expected, got, expected);
      fail_cnt++;
    end
  endtask

  task automatic check_contains(input string test_name, input string haystack, input string needle);
    int idx;
    idx = 0;
    for (int i = 0; i <= haystack.len() - needle.len(); i++) begin
      if (haystack.substr(i, i + needle.len() - 1) == needle) begin
        idx = 1;
        break;
      end
    end
    if (idx) begin
      $display("  PASS  %s", test_name);
      pass_cnt++;
    end else begin
      $display("  FAIL  %s\n        needle not found: \"%s\"\n        in:               \"%s\"", test_name, needle, haystack);
      fail_cnt++;
    end
  endtask

  // ADDED: Check that a string is NOT present
  task automatic check_not_contains(input string test_name, input string haystack, input string needle);
    int idx;
    idx = 0;
    for (int i = 0; i <= haystack.len() - needle.len(); i++) begin
      if (haystack.substr(i, i + needle.len() - 1) == needle) begin
        idx = 1;
        break;
      end
    end
    if (!idx) begin
      $display("  PASS  %s", test_name);
      pass_cnt++;
    end else begin
      $display("  FAIL  %s\n        needle unexpectedly found: \"%s\"\n        in:               \"%s\"", test_name, needle, haystack);
      fail_cnt++;
    end
  endtask

  task automatic check_bit(input string test_name, input logic got, input logic expected);
    if (got === expected) begin
      $display("  PASS  %s", test_name);
      pass_cnt++;
    end else begin
      $display("  FAIL  %s (got=%0b, expected=%0b)", test_name, got, expected);
      fail_cnt++;
    end
  endtask

  // -----------------------------------------------------------------------
  // Helper: send command, drain response
  // -----------------------------------------------------------------------
  localparam int DRAIN_TIMEOUT = 2000;
  task automatic cmd_and_drain(input string cmd, output string response);
    int pre_snap;
    pre_snap = tx_cap_len;
    send_cmd(cmd);
    drain_tx(DRAIN_TIMEOUT, pre_snap, cmd.len(), response);
  endtask

  // -----------------------------------------------------------------------
  // Main Test Sequence
  // -----------------------------------------------------------------------
  initial begin
    string resp_g;
    int snap13;

    rst_n = 0;
    sw_effect = 4'b0000;
    #(CLK_PERIOD * 5);
    rst_n = 1;

    // =====================================================================
    // 1. BOOT
    // =====================================================================
    $display("\n[1] Boot banner");
    begin
      drain_tx(DRAIN_TIMEOUT, 0, 0, resp_g);
      check_str("boot: ends with prompt", resp_g, "Arty> ");
    end

    // =====================================================================
    // 2. EMPTY ENTER
    // =====================================================================
    $display("\n[2] Empty enter");
    begin
      cmd_and_drain("", resp_g);
      check_str("empty enter: prompt only", resp_g, "Arty> ");
    end

    // =====================================================================
    // 3. UNKNOWN COMMAND
    // =====================================================================
    $display("\n[3] Unknown command");
    $display("=== TB VERSION: skip_count build, tx_cap before=%0d ===", tx_cap_len);
    begin
      int pre3;
      pre3 = tx_cap_len;
      cmd_and_drain("xyz", resp_g);
      check_str("unknown cmd: Err+prompt", resp_g, "ErrArty> ");
    end

    // =====================================================================
    // 4. STATUS DYNAMIC SWITCHING
    // =====================================================================
    $display("\n[4] Status dynamic switching (sw_effect)");
    begin
      // A. All Off -> Should print BYPASS
      sw_effect = 4'b0000;
      cmd_and_drain("st", resp_g);
      check_contains("st sw=0000: BYPASS", resp_g, "BYPASS");
      check_not_contains("st sw=0000: no DLY", resp_g, "DLY");

      // B. Chorus only (bit 2) -> Should print CHO, but NOT DLY/PHA/TRM
      sw_effect = 4'b0100;
      cmd_and_drain("st", resp_g);
      check_contains("st sw=0100: has CHO", resp_g, "CHO R:50 D:64 E:80 H:C8 L:80");
      check_not_contains("st sw=0100: no DLY", resp_g, "DLY");
      check_not_contains("st sw=0100: no TRM", resp_g, "TRM");

      // C. All On -> Should print all 4 lines
      sw_effect = 4'b1111;
      cmd_and_drain("st", resp_g);
      check_contains("st sw=1111: has DLY", resp_g, "DLY L:80 F:64 N:FF T:000007D0");
      check_contains("st sw=1111: has CHO", resp_g, "CHO R:50 D:64 E:80 H:C8 L:80");
      check_contains("st sw=1111: has PHA", resp_g, "PHA S:50 B:0");
      check_contains("st sw=1111: has TRM", resp_g, "TRM R:3C D:B4 W:0");
      check_contains("st sw=1111: ends with prompt", resp_g, "Arty> ");
      
      // Leave sw_effect = 1111 so the rest of the tests can successfully read back 'st' lines
    end

    // =====================================================================
    // 5. SET DLY
    // =====================================================================
    $display("\n[5] set dly commands");
    begin
      cmd_and_drain("set dly level A0", resp_g);
      check_contains("set dly level: OK", resp_g, "OK");
      check_byte("set dly level: port=A0", level_val, 8'hA0);

      cmd_and_drain("set dly feedback 42", resp_g);
      check_contains("set dly feedback: OK", resp_g, "OK");
      check_byte("set dly feedback: port=42", feedback_val, 8'h42);

      cmd_and_drain("set dly tone 1F", resp_g);
      check_contains("set dly tone: OK", resp_g, "OK");
      check_byte("set dly tone: port=1F", tone_val, 8'h1F);

      cmd_and_drain("set dly time DEADBEEF", resp_g);
      check_contains("set dly time: OK", resp_g, "OK");
      check_word("set dly time: port=DEADBEEF", time_val, 32'hDEADBEEF);

      cmd_and_drain("st", resp_g);
      check_contains("st after dly: L:A0", resp_g, "L:A0");
      check_contains("st after dly: F:42", resp_g, "F:42");
      check_contains("st after dly: N:1F", resp_g, "N:1F");
      check_contains("st after dly: T:DEADBEEF", resp_g, "T:DEADBEEF");
    end

    // =====================================================================
    // 6. SET CHO
    // =====================================================================
    $display("\n[6] set cho commands");
    begin
      cmd_and_drain("set cho rate 55", resp_g);
      check_contains("set cho rate: OK", resp_g, "OK");
      check_byte("set cho rate: port=55", chorus_rate_val, 8'h55);

      cmd_and_drain("set cho depth 77", resp_g);
      check_contains("set cho depth: OK", resp_g, "OK");
      check_byte("set cho depth: port=77", chorus_depth_val, 8'h77);

      cmd_and_drain("set cho efx AA", resp_g);
      check_contains("set cho efx: OK", resp_g, "OK");
      check_byte("set cho efx: port=AA", chorus_efx_val, 8'hAA);

      cmd_and_drain("set cho eqhi BB", resp_g);
      check_contains("set cho eqhi: OK", resp_g, "OK");
      check_byte("set cho eqhi: port=BB", chorus_eqhi_val, 8'hBB);

      cmd_and_drain("set cho eqlo CC", resp_g);
      check_contains("set cho eqlo: OK", resp_g, "OK");
      check_byte("set cho eqlo: port=CC", chorus_eqlo_val, 8'hCC);

      cmd_and_drain("st", resp_g);
      check_contains("st after cho: R:55", resp_g, "R:55");
      check_contains("st after cho: D:77", resp_g, "D:77");
      check_contains("st after cho: E:AA", resp_g, "E:AA");
      check_contains("st after cho: H:BB", resp_g, "H:BB");
      check_contains("st after cho: L:CC", resp_g, "L:CC");
    end

    // =====================================================================
    // 7. SET PHA
    // =====================================================================
    $display("\n[7] set pha commands");
    begin
      cmd_and_drain("set pha speed E0", resp_g);
      check_contains("set pha speed: OK", resp_g, "OK");
      check_byte("set pha speed: port=E0", phaser_speed_val, 8'hE0);

      cmd_and_drain("set pha fben 1", resp_g);
      check_contains("set pha fben: OK", resp_g, "OK");
      check_bit("set pha fben: port=1", phaser_fben_val, 1'b1);

      cmd_and_drain("set pha fben 0", resp_g);
      check_bit("set pha fben=0: port=0", phaser_fben_val, 1'b0);

      cmd_and_drain("st", resp_g);
      check_contains("st after pha: S:E0", resp_g, "S:E0");
      check_contains("st after pha: B:0", resp_g, "B:0");
    end

    // =====================================================================
    // 8. SET TRM
    // =====================================================================
    $display("\n[8] set trm commands");
    begin
      cmd_and_drain("set trm rate 20", resp_g);
      check_contains("set trm rate: OK", resp_g, "OK");
      check_byte("set trm rate: port=20", trem_rate_val, 8'h20);

      cmd_and_drain("set trm depth FF", resp_g);
      check_contains("set trm depth: OK", resp_g, "OK");
      check_byte("set trm depth: port=FF", trem_depth_val, 8'hFF);

      cmd_and_drain("set trm shape 1", resp_g);
      check_contains("set trm shape: OK", resp_g, "OK");
      check_bit("set trm shape: port=1", trem_shape_val, 1'b1);

      cmd_and_drain("set trm shape 0", resp_g);
      check_bit("set trm shape=0: port=0", trem_shape_val, 1'b0);

      cmd_and_drain("st", resp_g);
      check_contains("st after trm: R:20", resp_g, "R:20");
      check_contains("st after trm: D:FF", resp_g, "D:FF");
      check_contains("st after trm: W:0", resp_g, "W:0");
    end

    // =====================================================================
    // 9. BACKSPACE
    // =====================================================================
    $display("\n[9] Backspace editing");
    begin
      send_byte("s"); send_byte("e"); send_byte("t"); send_byte(" ");
      send_byte("d"); send_byte("l"); send_byte("y"); send_byte(" ");
      send_byte("t"); send_byte("o"); send_byte("n"); send_byte("e");
      send_byte(" "); send_byte("Z"); send_byte("Z");
      send_byte(8'h08); send_byte(8'h08); // BS BS
      send_byte("0"); send_byte("9");
      
      cmd_and_drain("", resp_g); // sends CR and drains
      check_contains("backspace+retype: OK", resp_g, "OK");
      check_byte("backspace+retype: tone=09", tone_val, 8'h09);
    end

    // =====================================================================
    // 10. ECHO
    // =====================================================================
    $display("\n[10] Character echo");
    begin
      int pre;
      pre = tx_cap_len;
      send_byte(8'h41); // 'A'
      drain_tx(DRAIN_TIMEOUT, pre, 0, resp_g);
      check_byte("echo: 'A' echoed", tx_captured[pre], 8'h41);

      pre = tx_cap_len;
      send_byte(8'h42); // 'B'
      drain_tx(DRAIN_TIMEOUT, pre, 0, resp_g);
      check_byte("echo: 'B' echoed", tx_captured[pre], 8'h42);

      // clean up dangling 'A''B' before next tests
      send_byte(8'h0D);
      drain_tx(DRAIN_TIMEOUT, tx_cap_len, 0, resp_g);
    end

    // =====================================================================
    // 11. REG_OUTPUTS
    // =====================================================================
    $display("\n[11] Register output ports");
    begin
      cmd_and_drain("set dly level FE", resp_g);
      check_byte("port level_val=FE", level_val, 8'hFE);
      cmd_and_drain("set dly feedback 12", resp_g);
      check_byte("port feedback_val=12", feedback_val, 8'h12);
      cmd_and_drain("set dly tone 34", resp_g);
      check_byte("port tone_val=34", tone_val, 8'h34);
      cmd_and_drain("set dly time 00001234", resp_g);
      check_word("port time_val=1234", time_val, 32'h1234);

      cmd_and_drain("set cho rate 11", resp_g);
      check_byte("port chorus_rate_val=11", chorus_rate_val, 8'h11);
      cmd_and_drain("set cho depth 22", resp_g);
      check_byte("port chorus_depth_val=22", chorus_depth_val, 8'h22);
      cmd_and_drain("set cho efx 33", resp_g);
      check_byte("port chorus_efx_val=33", chorus_efx_val, 8'h33);
      cmd_and_drain("set cho eqhi 44", resp_g);
      check_byte("port chorus_eqhi_val=44", chorus_eqhi_val, 8'h44);
      cmd_and_drain("set cho eqlo 55", resp_g);
      check_byte("port chorus_eqlo_val=55", chorus_eqlo_val, 8'h55);

      cmd_and_drain("set pha speed C0", resp_g);
      check_byte("port phaser_speed_val=C0", phaser_speed_val, 8'hC0);
      cmd_and_drain("set pha fben 1", resp_g);
      check_bit("port phaser_fben_val=1", phaser_fben_val, 1'b1);

      cmd_and_drain("set trm rate 7F", resp_g);
      check_byte("port trem_rate_val=7F", trem_rate_val, 8'h7F);
      cmd_and_drain("set trm depth 88", resp_g);
      check_byte("port trem_depth_val=88", trem_depth_val, 8'h88);
      cmd_and_drain("set trm shape 1", resp_g);
      check_bit("port trem_shape_val=1", trem_shape_val, 1'b1);
    end

    // =====================================================================
    // 12. LOWERCASE HEX
    // =====================================================================
    $display("\n[12] Lowercase hex digits");
    begin
      cmd_and_drain("set dly level ab", resp_g);
      check_contains("lowercase: OK", resp_g, "OK");
      check_byte("lowercase: level_val=AB", level_val, 8'hAB);

      cmd_and_drain("set dly time cafebabe", resp_g);
      check_word("lowercase: time=CAFEBABE", time_val, 32'hCAFEBABE);
    end

    // =====================================================================
    // 13. BUFFER OVERFLOW - >31 chars before CR
    // =====================================================================
    $display("\n[13] Buffer overflow (>31 chars)");
    begin
      snap13 = tx_cap_len;
      for (int i = 0; i < 35; i++) send_byte("x");
      send_byte(8'h0D);
      drain_tx(DRAIN_TIMEOUT, snap13, 0, resp_g);
      check_contains("overflow: DUT responds (Err+prompt)", resp_g, "Err");
    end

    // =====================================================================
    // Summary
    // =====================================================================
    $display("\n=================================================");
    $display(" Results:  %0d passed,  %0d failed", pass_cnt, fail_cnt);
    $display("=================================================");
    if (fail_cnt == 0)
      $display(" ALL TESTS PASSED");
    else
      $display(" SOME TESTS FAILED");

    $finish;
  end

  // -----------------------------------------------------------------------
  // Watchdog
  // -----------------------------------------------------------------------
  initial begin
    #(CLK_PERIOD * 10_000_000);
    $display("TIMEOUT");
    $finish;
  end
endmodule