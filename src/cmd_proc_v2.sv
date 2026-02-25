module cmd_proc_v2 (
    input  logic       clk,
    input  logic       rst_n,
    input  logic       rx_valid,
    input  logic [7:0] rx_byte,
    input  logic       tx_done,
    input  logic       tx_busy,
    output logic       tx_start,
    output logic [7:0] tx_byte,

    input  logic [3:0] sw_effect,

    // Flat register write bus (to ctrl_bus)
    output logic       wr_en,
    output logic [7:0] wr_addr,
    output logic [7:0] wr_data
);

  // ===========================================================================
  // ctrl_bus Address Map
  //
  //  Slot registers (8 regs per slot):
  //    0x00..0x07 : Slot 0 (tremolo)  [0]=rate [1]=depth [2]=shape
  //    0x08..0x0F : Slot 1 (phaser)   [0]=speed [1]=feedback_en
  //    0x10..0x17 : Slot 2 (chorus)   [0]=rate [1]=depth [2]=efx [3]=eqhi [4]=eqlo
  //    0x18..0x1F : Slot 3 (dd3)      [0]=tone [1]=level [2]=feedback [3..6]=time(LE)
  //
  //  Infrastructure (symmetric 5-port: 0=ADC/DAC, 1=TRM, 2=PHA, 3=CHO, 4=DLY):
  //    0x40..0x44 : route[0..4]
  //    0x48..0x4B : bypass[0..3]
  //
  // CLI format: set <efx> <prm> <hex>     (all qualifiers 3 chars)
  //
  //   set trm rat <hex>       → 0x00     set rte <N> <hex>   → 0x40+N
  //   set trm dep <hex>       → 0x01     set byp <N> <hex>   → 0x48+N
  //   set trm shp <hex>       → 0x02
  //   set pha spd <hex>       → 0x08     wr <addr><data>     → raw write
  //   set pha fbn <hex>       → 0x09     status              → full dump
  //   set cho rat <hex>       → 0x10
  //   set cho dep <hex>       → 0x11
  //   set cho efx <hex>       → 0x12
  //   set cho eqh <hex>       → 0x13
  //   set cho eql <hex>       → 0x14
  //   set dly ton <hex>       → 0x18
  //   set dly lvl <hex>       → 0x19
  //   set dly fdb <hex>       → 0x1A
  //   set dly tim <hex8>      → 0x1B..0x1E (4 writes, LE)
  // ===========================================================================

  localparam logic [7:0] CHAR_CR  = 8'h0D;
  localparam logic [7:0] CHAR_LF  = 8'h0A;
  localparam logic [7:0] CHAR_BS  = 8'h08;
  localparam logic [7:0] CHAR_DEL = 8'h7F;
  localparam logic [7:0] CHAR_SPC = 8'h20;
  localparam logic [7:0] CHAR_0   = 8'h30;
  localparam logic [7:0] CHAR_A   = 8'h41;

  localparam logic [7:0] MSG_PROMPT[0:2] = '{">"," "," "};
  localparam int LEN_PROMPT = 2;
  localparam logic [7:0] MSG_OK[0:3]  = '{"O","K", CHAR_CR, CHAR_LF};
  localparam int LEN_OK = 4;
  localparam logic [7:0] MSG_ERR[0:4] = '{"E","r","r", CHAR_CR, CHAR_LF};
  localparam int LEN_ERR = 5;

  typedef enum logic [5:0] {
    S_BOOT,
    S_IDLE,
    S_RX_CHAR,
    S_BACKSPACE,
    S_CHECK_CMD,
    S_PARSE_INIT,
    S_PARSE_LOOP,
    S_WRITE_REG,
    S_WRITE_TIME,
    S_ST_ROUTE,
    S_ST_TRM,
    S_ST_PHA,
    S_ST_CHO,
    S_ST_DLY,
    S_TX_START,
    S_TX_WAIT,
    S_LOAD_PROMPT,
    S_LOAD_STRING
  } state_t;

  state_t state = S_BOOT;
  state_t return_state = S_IDLE;

  logic [7:0] cmd_buf [0:31];
  logic [5:0] cmd_idx = 0;
  logic [7:0] tx_buf [0:63];
  logic [5:0] tx_len = 0;
  logic [5:0] tx_idx = 0;

  logic [31:0] parse_acc = 0;
  logic [ 5:0] parse_ptr = 0;
  logic [ 7:0] target_addr = 0;
  logic        target_is_time = 0;
  logic        rx_pending = 0;
  logic [ 7:0] rx_latch = 0;
  logic [ 1:0] time_wr_idx = 0;
  logic [ 4:0] cdc_wait = 0;        // delay counter between writes for CDC crossing

  // Shadow registers for status display
  logic [7:0] shadow [0:31];       // mirrors 0x00..0x1F
  logic [7:0] shadow_route [0:4];  // mirrors route[0..4]
  logic [7:0] shadow_byp [0:3];    // mirrors bypass[0..3]

  // Write bus
  logic       wr_en_reg = 0;
  logic [7:0] wr_addr_reg = 0;
  logic [7:0] wr_data_reg = 0;
  assign wr_en   = wr_en_reg;
  assign wr_addr = wr_addr_reg;
  assign wr_data = wr_data_reg;

  function automatic logic [7:0] n2h(input [3:0] n);
    return (n < 10) ? (n + CHAR_0) : (n - 10 + CHAR_A);
  endfunction

  // 3-char match at cmd_buf[start]
  function automatic logic m3(input int start,
                              input logic [7:0] c0, c1, c2);
    return (cmd_buf[start] == c0 && cmd_buf[start+1] == c1 && cmd_buf[start+2] == c2);
  endfunction

  function automatic logic str_match(input int start, input string s);
    for (int i = 0; i < s.len(); i++)
      if (cmd_buf[start+i] != s.getc(i)) return 0;
    return 1;
  endfunction

  // ---- Build a status line: "<3char> <hex pairs...> CR LF" ----
  // p = write pointer into tx_buf, returns new p
  task automatic st_hdr(
    input logic [7:0] c0, c1, c2,
    inout int p
  );
    tx_buf[p] <= c0; p++;
    tx_buf[p] <= c1; p++;
    tx_buf[p] <= c2; p++;
    tx_buf[p] <= ":"; p++;
  endtask

  task automatic st_hex(input logic [7:0] val, inout int p);
    tx_buf[p] <= n2h(val[7:4]); p++;
    tx_buf[p] <= n2h(val[3:0]); p++;
    tx_buf[p] <= " ";           p++;
  endtask

  task automatic st_nl(inout int p);
    tx_buf[p] <= CHAR_CR; p++;
    tx_buf[p] <= CHAR_LF; p++;
  endtask

  // ===========================================================================
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      state      <= S_BOOT;
      cmd_idx    <= 0;
      tx_start   <= 0;
      rx_pending <= 0;
      wr_en_reg  <= 0;
      for (int i = 0; i < 32; i++) shadow[i] <= '0;
      // Slot 0 (tremolo): rat=3C dep=B4 shp=00
      shadow[0]  <= 8'h3C; shadow[1]  <= 8'hB4;
      // Slot 1 (phaser): spd=50 fbn=00
      shadow[8]  <= 8'h50;
      // Slot 2 (chorus): rat=50 dep=64 efx=80 eqh=C8 eql=80
      shadow[16] <= 8'h50; shadow[17] <= 8'h64; shadow[18] <= 8'h80;
      shadow[19] <= 8'hC8; shadow[20] <= 8'h80;
      // Slot 3 (dd3): ton=FF lvl=80 fdb=64 tim=000007D0 (LE: D0,07,00,00)
      shadow[24] <= 8'hFF; shadow[25] <= 8'h80; shadow[26] <= 8'h64;
      shadow[27] <= 8'hD0; shadow[28] <= 8'h07;
      for (int i = 0; i < 5;  i++) shadow_route[i] <= (i == 0) ? 8'd4 : 8'(i - 1);
      for (int i = 0; i < 4;  i++) shadow_byp[i] <= '0;
    end else begin
      wr_en_reg <= 1'b0;

      if (rx_valid) begin
        rx_pending <= 1;
        rx_latch   <= rx_byte;
      end

      case (state)
        // ============================================================
        S_BOOT: begin
          tx_buf[0] <= CHAR_CR;
          tx_buf[1] <= CHAR_LF;
          tx_len    <= 2;
          tx_idx    <= 0;
          return_state <= S_LOAD_PROMPT;
          state <= S_TX_START;
        end

        S_IDLE: begin
          tx_start <= 0;
          if (rx_pending) begin
            rx_pending <= 0;
            state <= S_RX_CHAR;
          end
        end

        S_RX_CHAR: begin
          if (rx_latch == CHAR_CR || rx_latch == CHAR_LF) begin
            cmd_buf[cmd_idx] <= 8'h00;
            tx_buf[0] <= CHAR_CR;
            tx_buf[1] <= CHAR_LF;
            tx_len    <= 2;
            tx_idx    <= 0;
            return_state <= (cmd_idx == 0) ? S_LOAD_PROMPT : S_CHECK_CMD;
            state <= S_TX_START;
          end else if (rx_latch == CHAR_BS || rx_latch == CHAR_DEL) begin
            state <= S_BACKSPACE;
          end else if (cmd_idx < 31) begin
            cmd_buf[cmd_idx] <= rx_latch;
            cmd_idx <= cmd_idx + 1;
            tx_buf[0] <= rx_latch;
            tx_len <= 1;
            tx_idx <= 0;
            return_state <= S_IDLE;
            state <= S_TX_START;
          end else state <= S_IDLE;
        end

        S_BACKSPACE: begin
          if (cmd_idx > 0) begin
            cmd_idx <= cmd_idx - 1;
            tx_buf[0] <= CHAR_BS;
            tx_buf[1] <= CHAR_SPC;
            tx_buf[2] <= CHAR_BS;
            tx_len <= 3;
            tx_idx <= 0;
            return_state <= S_IDLE;
            state <= S_TX_START;
          end else state <= S_IDLE;
        end

        // ============================================================
        // Command dispatch
        //
        // "set" format:  set <EFX> <PRM> <VAL>
        //   positions:   0123 4567 89AB CDEF
        //   cmd_buf[0..2]="set" [3]=' ' [4..6]=efx [7]=' ' [8..10]=prm [11]=' ' [12..]=val
        //
        // "set rte <N> <VAL>" and "set byp <N> <VAL>":
        //   [4..6]="rte"/"byp" [7]=' ' [8]=digit [9]=' ' [10..]=val
        //
        // "wr <AABB>"
        // "status"
        // ============================================================
        S_CHECK_CMD: begin
          cmd_idx <= 0;
          target_is_time <= 0;

          // ---- "status" ----
          if (str_match(0, "status")) begin
            state <= S_ST_ROUTE;

          // ---- "wr <hex_addr><hex_data>" ----
          end else if (m3(0, "w", "r", " ")) begin
            parse_ptr   <= 3;
            target_addr <= 8'hFF;  // sentinel: raw write
            state <= S_PARSE_INIT;

          // ---- "set" commands ----
          end else if (m3(0, "s", "e", "t") && cmd_buf[3] == " ") begin

            // ---- set trm <prm> <val> ----
            if (m3(4, "t", "r", "m") && cmd_buf[7] == " ") begin
              if      (m3(8, "r","a","t")) begin target_addr <= 8'h00; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8, "d","e","p")) begin target_addr <= 8'h01; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8, "s","h","p")) begin target_addr <= 8'h02; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else begin tx_len<=LEN_ERR; tx_idx<=0; rx_latch<=3; state<=S_LOAD_STRING; return_state<=S_LOAD_PROMPT; end

            // ---- set pha <prm> <val> ----
            end else if (m3(4, "p", "h", "a") && cmd_buf[7] == " ") begin
              if      (m3(8, "s","p","d")) begin target_addr <= 8'h08; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8, "f","b","n")) begin target_addr <= 8'h09; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else begin tx_len<=LEN_ERR; tx_idx<=0; rx_latch<=3; state<=S_LOAD_STRING; return_state<=S_LOAD_PROMPT; end

            // ---- set cho <prm> <val> ----
            end else if (m3(4, "c", "h", "o") && cmd_buf[7] == " ") begin
              if      (m3(8, "r","a","t")) begin target_addr <= 8'h10; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8, "d","e","p")) begin target_addr <= 8'h11; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8, "e","f","x")) begin target_addr <= 8'h12; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8, "e","q","h")) begin target_addr <= 8'h13; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8, "e","q","l")) begin target_addr <= 8'h14; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else begin tx_len<=LEN_ERR; tx_idx<=0; rx_latch<=3; state<=S_LOAD_STRING; return_state<=S_LOAD_PROMPT; end

            // ---- set dly <prm> <val> ----
            end else if (m3(4, "d", "l", "y") && cmd_buf[7] == " ") begin
              if      (m3(8, "t","o","n")) begin target_addr <= 8'h18; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8, "l","v","l")) begin target_addr <= 8'h19; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8, "f","d","b")) begin target_addr <= 8'h1A; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8, "t","i","m")) begin target_addr <= 8'h1B; parse_ptr <= 12; target_is_time <= 1; state <= S_PARSE_INIT; end
              else begin tx_len<=LEN_ERR; tx_idx<=0; rx_latch<=3; state<=S_LOAD_STRING; return_state<=S_LOAD_PROMPT; end

            // ---- set rte <N> <val> ----
            end else if (m3(4, "r", "t", "e") && cmd_buf[7] == " ") begin
              target_addr <= 8'h40 + (cmd_buf[8] - CHAR_0);
              parse_ptr <= 10;
              state <= S_PARSE_INIT;

            // ---- set byp <N> <val> ----
            end else if (m3(4, "b", "y", "p") && cmd_buf[7] == " ") begin
              target_addr <= 8'h48 + (cmd_buf[8] - CHAR_0);
              parse_ptr <= 10;
              state <= S_PARSE_INIT;

            end else begin
              tx_len<=LEN_ERR; tx_idx<=0; rx_latch<=3;
              state<=S_LOAD_STRING; return_state<=S_LOAD_PROMPT;
            end

          // ---- Unknown command ----
          end else begin
            tx_len<=LEN_ERR; tx_idx<=0; rx_latch<=3;
            state<=S_LOAD_STRING; return_state<=S_LOAD_PROMPT;
          end
        end

        // ============================================================
        // "status" - multi-stage output
        //
        // Stage 1: Routes
        // Stage 2: TRM (if active)
        // Stage 3: PHA (if active)
        // Stage 4: CHO (if active)
        // Stage 5: DLY (if active)
        // ============================================================

        // ---- Routes ----
        // Symmetric: 0=ADC 1=TRM 2=PHA 3=CHO 4=DLY 5=DAC
        // Skip route[0] (ADC sink is dummy), show ports 1-5
        S_ST_ROUTE: begin
          automatic int p = 0;

          // TRM (port 1)
          tx_buf[p]<="T"; p++; tx_buf[p]<="R"; p++; tx_buf[p]<="M"; p++;
          tx_buf[p]<="<"; p++;
          case (shadow_route[1][2:0])
            3'd0: begin tx_buf[p]<="A"; p++; tx_buf[p]<="D"; p++; tx_buf[p]<="C"; p++; end
            3'd1: begin tx_buf[p]<="T"; p++; tx_buf[p]<="R"; p++; tx_buf[p]<="M"; p++; end
            3'd2: begin tx_buf[p]<="P"; p++; tx_buf[p]<="H"; p++; tx_buf[p]<="A"; p++; end
            3'd3: begin tx_buf[p]<="C"; p++; tx_buf[p]<="H"; p++; tx_buf[p]<="O"; p++; end
            3'd4: begin tx_buf[p]<="D"; p++; tx_buf[p]<="L"; p++; tx_buf[p]<="Y"; p++; end
            3'd5: begin tx_buf[p]<="D"; p++; tx_buf[p]<="A"; p++; tx_buf[p]<="C"; p++; end
            default: begin tx_buf[p]<="?"; p++; tx_buf[p]<="?"; p++; tx_buf[p]<="?"; p++; end
          endcase
          tx_buf[p]<=" "; p++;

          // PHA (port 2)
          tx_buf[p]<="P"; p++; tx_buf[p]<="H"; p++; tx_buf[p]<="A"; p++;
          tx_buf[p]<="<"; p++;
          case (shadow_route[2][2:0])
            3'd0: begin tx_buf[p]<="A"; p++; tx_buf[p]<="D"; p++; tx_buf[p]<="C"; p++; end
            3'd1: begin tx_buf[p]<="T"; p++; tx_buf[p]<="R"; p++; tx_buf[p]<="M"; p++; end
            3'd2: begin tx_buf[p]<="P"; p++; tx_buf[p]<="H"; p++; tx_buf[p]<="A"; p++; end
            3'd3: begin tx_buf[p]<="C"; p++; tx_buf[p]<="H"; p++; tx_buf[p]<="O"; p++; end
            3'd4: begin tx_buf[p]<="D"; p++; tx_buf[p]<="L"; p++; tx_buf[p]<="Y"; p++; end
            3'd5: begin tx_buf[p]<="D"; p++; tx_buf[p]<="A"; p++; tx_buf[p]<="C"; p++; end
            default: begin tx_buf[p]<="?"; p++; tx_buf[p]<="?"; p++; tx_buf[p]<="?"; p++; end
          endcase
          tx_buf[p]<=" "; p++;

          // CHO (port 3)
          tx_buf[p]<="C"; p++; tx_buf[p]<="H"; p++; tx_buf[p]<="O"; p++;
          tx_buf[p]<="<"; p++;
          case (shadow_route[3][2:0])
            3'd0: begin tx_buf[p]<="A"; p++; tx_buf[p]<="D"; p++; tx_buf[p]<="C"; p++; end
            3'd1: begin tx_buf[p]<="T"; p++; tx_buf[p]<="R"; p++; tx_buf[p]<="M"; p++; end
            3'd2: begin tx_buf[p]<="P"; p++; tx_buf[p]<="H"; p++; tx_buf[p]<="A"; p++; end
            3'd3: begin tx_buf[p]<="C"; p++; tx_buf[p]<="H"; p++; tx_buf[p]<="O"; p++; end
            3'd4: begin tx_buf[p]<="D"; p++; tx_buf[p]<="L"; p++; tx_buf[p]<="Y"; p++; end
            3'd5: begin tx_buf[p]<="D"; p++; tx_buf[p]<="A"; p++; tx_buf[p]<="C"; p++; end
            default: begin tx_buf[p]<="?"; p++; tx_buf[p]<="?"; p++; tx_buf[p]<="?"; p++; end
          endcase
          tx_buf[p]<=" "; p++;

          // DLY (port 4)
          tx_buf[p]<="D"; p++; tx_buf[p]<="L"; p++; tx_buf[p]<="Y"; p++;
          tx_buf[p]<="<"; p++;
          case (shadow_route[4][2:0])
            3'd0: begin tx_buf[p]<="A"; p++; tx_buf[p]<="D"; p++; tx_buf[p]<="C"; p++; end
            3'd1: begin tx_buf[p]<="T"; p++; tx_buf[p]<="R"; p++; tx_buf[p]<="M"; p++; end
            3'd2: begin tx_buf[p]<="P"; p++; tx_buf[p]<="H"; p++; tx_buf[p]<="A"; p++; end
            3'd3: begin tx_buf[p]<="C"; p++; tx_buf[p]<="H"; p++; tx_buf[p]<="O"; p++; end
            3'd4: begin tx_buf[p]<="D"; p++; tx_buf[p]<="L"; p++; tx_buf[p]<="Y"; p++; end
            3'd5: begin tx_buf[p]<="D"; p++; tx_buf[p]<="A"; p++; tx_buf[p]<="C"; p++; end
            default: begin tx_buf[p]<="?"; p++; tx_buf[p]<="?"; p++; tx_buf[p]<="?"; p++; end
          endcase
          tx_buf[p]<=" "; p++;

          // DAC (port 5)
          tx_buf[p]<="D"; p++; tx_buf[p]<="A"; p++; tx_buf[p]<="C"; p++;
          tx_buf[p]<="<"; p++;
          case (shadow_route[5][2:0])
            3'd0: begin tx_buf[p]<="A"; p++; tx_buf[p]<="D"; p++; tx_buf[p]<="C"; p++; end
            3'd1: begin tx_buf[p]<="T"; p++; tx_buf[p]<="R"; p++; tx_buf[p]<="M"; p++; end
            3'd2: begin tx_buf[p]<="P"; p++; tx_buf[p]<="H"; p++; tx_buf[p]<="A"; p++; end
            3'd3: begin tx_buf[p]<="C"; p++; tx_buf[p]<="H"; p++; tx_buf[p]<="O"; p++; end
            3'd4: begin tx_buf[p]<="D"; p++; tx_buf[p]<="L"; p++; tx_buf[p]<="Y"; p++; end
            3'd5: begin tx_buf[p]<="D"; p++; tx_buf[p]<="A"; p++; tx_buf[p]<="C"; p++; end
            default: begin tx_buf[p]<="?"; p++; tx_buf[p]<="?"; p++; tx_buf[p]<="?"; p++; end
          endcase
          tx_buf[p]<=CHAR_CR; p++;
          tx_buf[p]<=CHAR_LF; p++;

          // "BYP: 0 0 0 0\r\n"  - show effective bypass (hw switch inverted OR register)
          tx_buf[p] <= "B"; p++;
          tx_buf[p] <= "Y"; p++;
          tx_buf[p] <= "P"; p++;
          tx_buf[p] <= ":"; p++;
          for (int i = 0; i < 4; i++) begin
            tx_buf[p] <= (sw_effect[i] && !shadow_byp[i][0]) ? "0" : "1";  p++;
            tx_buf[p] <= " ";                       p++;
          end
          tx_buf[p] <= CHAR_CR; p++;
          tx_buf[p] <= CHAR_LF; p++;
          tx_len <= 6'(p);
          tx_idx <= 0;
          return_state <= S_ST_TRM;
          state <= S_TX_START;
        end

        // ---- TRM (slot 0) ----
        S_ST_TRM: begin
          if (sw_effect[0]) begin
            automatic int p = 0;
            // "TRM rat:3C dep:B4 shp:00\r\n"
            tx_buf[p]<="T"; p++; tx_buf[p]<="R"; p++; tx_buf[p]<="M"; p++; tx_buf[p]<=" "; p++;
            tx_buf[p]<="r"; p++; tx_buf[p]<="a"; p++; tx_buf[p]<="t"; p++; tx_buf[p]<=":"; p++;
            tx_buf[p]<=n2h(shadow[0][7:4]); p++; tx_buf[p]<=n2h(shadow[0][3:0]); p++;
            tx_buf[p]<=" "; p++;
            tx_buf[p]<="d"; p++; tx_buf[p]<="e"; p++; tx_buf[p]<="p"; p++; tx_buf[p]<=":"; p++;
            tx_buf[p]<=n2h(shadow[1][7:4]); p++; tx_buf[p]<=n2h(shadow[1][3:0]); p++;
            tx_buf[p]<=" "; p++;
            tx_buf[p]<="s"; p++; tx_buf[p]<="h"; p++; tx_buf[p]<="p"; p++; tx_buf[p]<=":"; p++;
            tx_buf[p]<=n2h(shadow[2][7:4]); p++; tx_buf[p]<=n2h(shadow[2][3:0]); p++;
            tx_buf[p]<=CHAR_CR; p++; tx_buf[p]<=CHAR_LF; p++;
            tx_len <= 6'(p);
            tx_idx <= 0;
            return_state <= S_ST_PHA;
            state <= S_TX_START;
          end else state <= S_ST_PHA;
        end

        // ---- PHA (slot 1) ----
        S_ST_PHA: begin
          if (sw_effect[1]) begin
            automatic int p = 0;
            tx_buf[p]<="P"; p++; tx_buf[p]<="H"; p++; tx_buf[p]<="A"; p++; tx_buf[p]<=" "; p++;
            tx_buf[p]<="s"; p++; tx_buf[p]<="p"; p++; tx_buf[p]<="d"; p++; tx_buf[p]<=":"; p++;
            tx_buf[p]<=n2h(shadow[8][7:4]); p++; tx_buf[p]<=n2h(shadow[8][3:0]); p++;
            tx_buf[p]<=" "; p++;
            tx_buf[p]<="f"; p++; tx_buf[p]<="b"; p++; tx_buf[p]<="n"; p++; tx_buf[p]<=":"; p++;
            tx_buf[p]<=n2h(shadow[9][7:4]); p++; tx_buf[p]<=n2h(shadow[9][3:0]); p++;
            tx_buf[p]<=CHAR_CR; p++; tx_buf[p]<=CHAR_LF; p++;
            tx_len <= 6'(p);
            tx_idx <= 0;
            return_state <= S_ST_CHO;
            state <= S_TX_START;
          end else state <= S_ST_CHO;
        end

        // ---- CHO (slot 2) ----
        S_ST_CHO: begin
          if (sw_effect[2]) begin
            automatic int p = 0;
            tx_buf[p]<="C"; p++; tx_buf[p]<="H"; p++; tx_buf[p]<="O"; p++; tx_buf[p]<=" "; p++;
            tx_buf[p]<="r"; p++; tx_buf[p]<="a"; p++; tx_buf[p]<="t"; p++; tx_buf[p]<=":"; p++;
            tx_buf[p]<=n2h(shadow[5'h10][7:4]); p++; tx_buf[p]<=n2h(shadow[5'h10][3:0]); p++;
            tx_buf[p]<=" "; p++;
            tx_buf[p]<="d"; p++; tx_buf[p]<="e"; p++; tx_buf[p]<="p"; p++; tx_buf[p]<=":"; p++;
            tx_buf[p]<=n2h(shadow[5'h11][7:4]); p++; tx_buf[p]<=n2h(shadow[5'h11][3:0]); p++;
            tx_buf[p]<=" "; p++;
            tx_buf[p]<="e"; p++; tx_buf[p]<="f"; p++; tx_buf[p]<="x"; p++; tx_buf[p]<=":"; p++;
            tx_buf[p]<=n2h(shadow[5'h12][7:4]); p++; tx_buf[p]<=n2h(shadow[5'h12][3:0]); p++;
            tx_buf[p]<=" "; p++;
            tx_buf[p]<="e"; p++; tx_buf[p]<="q"; p++; tx_buf[p]<="h"; p++; tx_buf[p]<=":"; p++;
            tx_buf[p]<=n2h(shadow[5'h13][7:4]); p++; tx_buf[p]<=n2h(shadow[5'h13][3:0]); p++;
            tx_buf[p]<=" "; p++;
            tx_buf[p]<="e"; p++; tx_buf[p]<="q"; p++; tx_buf[p]<="l"; p++; tx_buf[p]<=":"; p++;
            tx_buf[p]<=n2h(shadow[5'h14][7:4]); p++; tx_buf[p]<=n2h(shadow[5'h14][3:0]); p++;
            tx_buf[p]<=CHAR_CR; p++; tx_buf[p]<=CHAR_LF; p++;
            tx_len <= 6'(p);
            tx_idx <= 0;
            return_state <= S_ST_DLY;
            state <= S_TX_START;
          end else state <= S_ST_DLY;
        end

        // ---- DLY (slot 3) ----
        S_ST_DLY: begin
          if (sw_effect[3]) begin
            automatic int p = 0;
            tx_buf[p]<="D"; p++; tx_buf[p]<="L"; p++; tx_buf[p]<="Y"; p++; tx_buf[p]<=" "; p++;
            tx_buf[p]<="t"; p++; tx_buf[p]<="o"; p++; tx_buf[p]<="n"; p++; tx_buf[p]<=":"; p++;
            tx_buf[p]<=n2h(shadow[5'h18][7:4]); p++; tx_buf[p]<=n2h(shadow[5'h18][3:0]); p++;
            tx_buf[p]<=" "; p++;
            tx_buf[p]<="l"; p++; tx_buf[p]<="v"; p++; tx_buf[p]<="l"; p++; tx_buf[p]<=":"; p++;
            tx_buf[p]<=n2h(shadow[5'h19][7:4]); p++; tx_buf[p]<=n2h(shadow[5'h19][3:0]); p++;
            tx_buf[p]<=" "; p++;
            tx_buf[p]<="f"; p++; tx_buf[p]<="d"; p++; tx_buf[p]<="b"; p++; tx_buf[p]<=":"; p++;
            tx_buf[p]<=n2h(shadow[5'h1A][7:4]); p++; tx_buf[p]<=n2h(shadow[5'h1A][3:0]); p++;
            tx_buf[p]<=" "; p++;
            tx_buf[p]<="t"; p++; tx_buf[p]<="i"; p++; tx_buf[p]<="m"; p++; tx_buf[p]<=":"; p++;
            tx_buf[p]<=n2h(shadow[5'h1E][7:4]); p++; tx_buf[p]<=n2h(shadow[5'h1E][3:0]); p++;
            tx_buf[p]<=n2h(shadow[5'h1D][7:4]); p++; tx_buf[p]<=n2h(shadow[5'h1D][3:0]); p++;
            tx_buf[p]<=n2h(shadow[5'h1C][7:4]); p++; tx_buf[p]<=n2h(shadow[5'h1C][3:0]); p++;
            tx_buf[p]<=n2h(shadow[5'h1B][7:4]); p++; tx_buf[p]<=n2h(shadow[5'h1B][3:0]); p++;
            tx_buf[p]<=CHAR_CR; p++; tx_buf[p]<=CHAR_LF; p++;
            tx_len <= 6'(p);
            tx_idx <= 0;
            return_state <= S_LOAD_PROMPT;
            state <= S_TX_START;
          end else begin
            state <= S_LOAD_PROMPT;
          end
        end

        // ============================================================
        // Hex parser
        // ============================================================
        S_PARSE_INIT: begin
          parse_acc <= 0;
          state <= S_PARSE_LOOP;
        end

        S_PARSE_LOOP: begin
          logic [7:0] c;
          logic [3:0] nibble;
          c = cmd_buf[parse_ptr];

          if (c == CHAR_SPC) begin
            parse_ptr <= parse_ptr + 1;
          end else if (c >= "0" && c <= "9") begin
            nibble = c - "0";
            parse_acc <= {parse_acc[27:0], nibble};
            parse_ptr <= parse_ptr + 1;
          end else if (c >= "A" && c <= "F") begin
            nibble = c - "A" + 10;
            parse_acc <= {parse_acc[27:0], nibble};
            parse_ptr <= parse_ptr + 1;
          end else if (c >= "a" && c <= "f") begin
            nibble = c - "a" + 10;
            parse_acc <= {parse_acc[27:0], nibble};
            parse_ptr <= parse_ptr + 1;
          end else begin
            // Done parsing
            if (target_addr == 8'hFF) begin
              // Raw write mode: acc = {addr, data}
              wr_addr_reg <= parse_acc[15:8];
              wr_data_reg <= parse_acc[7:0];
              wr_en_reg   <= 1'b1;
              if (parse_acc[15:8] < 8'h20)
                shadow[parse_acc[12:8]] <= parse_acc[7:0];
              else if (parse_acc[15:8] >= 8'h40 && parse_acc[15:8] < 8'h45)
                shadow_route[parse_acc[10:8]] <= parse_acc[7:0];
              else if (parse_acc[15:8] >= 8'h48 && parse_acc[15:8] < 8'h4C)
                shadow_byp[parse_acc[9:8]] <= parse_acc[7:0];
              tx_len<=LEN_OK; tx_idx<=0; rx_latch<=2;
              state<=S_LOAD_STRING; return_state<=S_LOAD_PROMPT;
            end else if (target_is_time) begin
              time_wr_idx <= 0;
              state <= S_WRITE_TIME;
            end else begin
              state <= S_WRITE_REG;
            end
          end
        end

        // ---- Single 8-bit register write ----
        S_WRITE_REG: begin
          wr_addr_reg <= target_addr;
          wr_data_reg <= parse_acc[7:0];
          wr_en_reg   <= 1'b1;
          // Update shadows
          if (target_addr < 8'h20)
            shadow[target_addr[4:0]] <= parse_acc[7:0];
          else if (target_addr >= 8'h40 && target_addr < 8'h45)
            shadow_route[target_addr[2:0]] <= parse_acc[7:0];
          else if (target_addr >= 8'h48 && target_addr < 8'h4C)
            shadow_byp[target_addr[1:0]] <= parse_acc[7:0];
          tx_len<=LEN_OK; tx_idx<=0; rx_latch<=2;
          state<=S_LOAD_STRING; return_state<=S_LOAD_PROMPT;
        end

        // ---- 32-bit time: 4 sequential byte writes (LE) ----
        // ---- 32-bit time: 4 byte writes with CDC wait between each ----
        // Each write needs ~25 sys_clk cycles for the toggle to cross
        // the CDC into the audio domain before the next write.
        S_WRITE_TIME: begin
          if (cdc_wait != 0) begin
            // Waiting for CDC to settle
            cdc_wait <= cdc_wait - 1;
          end else if (!wr_en_reg) begin
            // Drive phase: assert wr_en with current byte
            wr_addr_reg <= target_addr + {6'd0, time_wr_idx};
            case (time_wr_idx)
              2'd0: begin wr_data_reg <= parse_acc[7:0];   shadow[target_addr[4:0]+0] <= parse_acc[7:0];   end
              2'd1: begin wr_data_reg <= parse_acc[15:8];  shadow[target_addr[4:0]+1] <= parse_acc[15:8];  end
              2'd2: begin wr_data_reg <= parse_acc[23:16]; shadow[target_addr[4:0]+2] <= parse_acc[23:16]; end
              2'd3: begin wr_data_reg <= parse_acc[31:24]; shadow[target_addr[4:0]+3] <= parse_acc[31:24]; end
            endcase
            wr_en_reg <= 1'b1;
          end else begin
            // Gap phase: deassert wr_en, start CDC wait, advance
            wr_en_reg <= 1'b0;
            if (time_wr_idx == 2'd3) begin
              tx_len<=LEN_OK; tx_idx<=0; rx_latch<=2;
              state<=S_LOAD_STRING; return_state<=S_LOAD_PROMPT;
            end else begin
              time_wr_idx <= time_wr_idx + 1;
              cdc_wait <= 5'd31;  // wait 32 sys_clk cycles (~320ns) for CDC
            end
          end
        end

        // ============================================================
        // TX engine
        // ============================================================
        S_LOAD_PROMPT: begin
          tx_idx <= 0;
          tx_len <= LEN_PROMPT;
          rx_latch <= 1;
          state <= S_LOAD_STRING;
          return_state <= S_IDLE;
        end

        S_LOAD_STRING: begin
          if (tx_idx < tx_len) begin
            if (rx_latch == 1)      tx_buf[tx_idx] <= MSG_PROMPT[tx_idx];
            else if (rx_latch == 2) tx_buf[tx_idx] <= MSG_OK[tx_idx];
            else if (rx_latch == 3) tx_buf[tx_idx] <= MSG_ERR[tx_idx];
            tx_idx <= tx_idx + 1;
          end else begin
            tx_idx <= 0;
            state  <= S_TX_START;
          end
        end

        S_TX_START: begin
          if (tx_idx < tx_len) begin
            if (!tx_busy) begin
              tx_byte  <= tx_buf[tx_idx];
              tx_start <= 1;
              state    <= S_TX_WAIT;
            end
          end else state <= return_state;
        end

        S_TX_WAIT: begin
          tx_start <= 0;
          if (tx_done) begin
            tx_idx <= tx_idx + 1;
            state  <= S_TX_START;
          end
        end
      endcase
    end
  end
endmodule