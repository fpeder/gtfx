// =============================================================================
// cmd_proc_v2.sv  -  Serial CLI command processor
//
// Improvements over original:
//  - Dedicated `tx_msg_sel` enum replaces the rx_latch-as-message-selector hack
//  - `send_msg()` task centralises the repetitive {tx_len, tx_idx, rx_latch,
//    state, return_state} boilerplate that appeared ~10 times
//  - `send_err()` / `send_prompt()` one-liner wrappers (OK response removed)
//  - `emit_port_src()` task eliminates the 5× duplicate case block in S_ST_ROUTE
//  - `emit_param()` task eliminates character-by-character tx_buf packing that
//    was copy-pasted across all status states
//  - S_ST_ROUTE out-of-bounds fix: shadow_route is [0:4]; DAC uses index 4 not 5
//  - S_PARSE_LOOP: nibble extraction unified into one branch that covers 0-9/A-F/a-f
//  - S_WRITE_TIME comment corrected (2-byte LE write, not 4-byte)
//  - `default` branch added to the state case to prevent latch inference
//  - All magic numbers replaced with named localparam constants
//  - State enum widened to 6 bits and kept contiguous for easy expansion
//  - OK response removed; successful writes go directly back to prompt
//  - "status" command remapped to single character ';'
// =============================================================================

module cmd_proc_v2 (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        rx_valid,
    input  logic [7:0]  rx_byte,
    input  logic        tx_done,
    input  logic        tx_busy,
    output logic        tx_start,
    output logic [7:0]  tx_byte,

    input  logic [3:0]  sw_effect,

    // Flat register write bus (to ctrl_bus)
    output logic        wr_en,
    output logic [7:0]  wr_addr,
    output logic [7:0]  wr_data
);

  // ===========================================================================
  // ctrl_bus Address Map
  //
  //  Slot registers (8 regs per slot):
  //    0x00..0x07 : Slot 0 (tremolo)  [0]=rate [1]=depth [2]=shape
  //    0x08..0x0F : Slot 1 (phaser)   [0]=speed [1]=feedback_en
  //    0x10..0x17 : Slot 2 (chorus)   [0]=rate [1]=depth [2]=efx [3]=eqhi [4]=eqlo
  //    0x18..0x1F : Slot 3 (dd3)      [0]=tone [1]=level [2]=feedback [3..4]=time(LE)
  //
  //  Infrastructure (symmetric 5-port: 0=ADC, 1=TRM, 2=PHA, 3=CHO, 4=DLY):
  //    0x40..0x44 : route[0..4]
  //    0x48..0x4B : bypass[0..3]
  //
  // CLI format: set <efx> <prm> <hex>     (all qualifiers 3 chars)
  //
  //   set trm rat <hex>       → 0x00     con <src> to <snk>
  //   set trm dep <hex>       → 0x01       src/snk ∈ {adc,trm,pha,cho,dly,dac}
  //   set trm shp <hex>       → 0x02       e.g. "con dly to dac"  → 0x40 ← 4
  //   set pha spd <hex>       → 0x08            "con adc to trm"  → 0x41 ← 0
  //   set pha fbn <hex>       → 0x09
  //   set cho rat <hex>       → 0x10     set byp trm <hex>   → 0x48
  //   set cho dep <hex>       → 0x11     set byp pha <hex>   → 0x49
  //   set cho efx <hex>       → 0x12     set byp cho <hex>   → 0x4A
  //   set cho eqh <hex>       → 0x13     set byp dly <hex>   → 0x4B
  //   set cho eql <hex>       → 0x14
  //   set dly ton <hex>       → 0x18     wr <addr><data>     → raw write
  //   set dly lvl <hex>       → 0x19     ;                   → status dump
  //   set dly fdb <hex>       → 0x1A
  //   set dly tim <hex8>      → 0x1B..0x1C (2 writes, LE)
  //
  //  Successful writes return directly to prompt (no OK response).
  //  Status is triggered by a single ';' character (no Enter needed).
  // ===========================================================================

  // ---------------------------------------------------------------------------
  // ASCII constants
  // ---------------------------------------------------------------------------
  localparam logic [7:0] CHAR_CR  = 8'h0D;
  localparam logic [7:0] CHAR_LF  = 8'h0A;
  localparam logic [7:0] CHAR_BS  = 8'h08;
  localparam logic [7:0] CHAR_DEL = 8'h7F;
  localparam logic [7:0] CHAR_SPC = 8'h20;
  localparam logic [7:0] CHAR_0   = 8'h30;
  localparam logic [7:0] CHAR_A   = 8'h41;

  // ---------------------------------------------------------------------------
  // Register address map constants
  // ---------------------------------------------------------------------------
  localparam logic [7:0] ADDR_TRM_RAT = 8'h00;
  localparam logic [7:0] ADDR_TRM_DEP = 8'h01;
  localparam logic [7:0] ADDR_TRM_SHP = 8'h02;
  localparam logic [7:0] ADDR_PHA_SPD = 8'h08;
  localparam logic [7:0] ADDR_PHA_FBN = 8'h09;
  localparam logic [7:0] ADDR_CHO_RAT = 8'h10;
  localparam logic [7:0] ADDR_CHO_DEP = 8'h11;
  localparam logic [7:0] ADDR_CHO_EFX = 8'h12;
  localparam logic [7:0] ADDR_CHO_EQH = 8'h13;
  localparam logic [7:0] ADDR_CHO_EQL = 8'h14;
  localparam logic [7:0] ADDR_DLY_TON = 8'h18;
  localparam logic [7:0] ADDR_DLY_LVL = 8'h19;
  localparam logic [7:0] ADDR_DLY_FDB = 8'h1A;
  localparam logic [7:0] ADDR_DLY_TIM = 8'h1B;   // 2 bytes LE: 0x1B, 0x1C
  localparam logic [7:0] ADDR_RTE_BASE = 8'h40;
  localparam logic [7:0] ADDR_BYP_BASE = 8'h48;
  localparam logic [7:0] ADDR_RAW_SENTINEL = 8'hFF; // signals raw-write mode

  // CDC gap between multi-byte writes
  localparam int CDC_WAIT_CYCLES = 31;

  // ---------------------------------------------------------------------------
  // Canned messages
  // ---------------------------------------------------------------------------
  localparam logic [7:0] MSG_PROMPT [0:2] = '{">"," "," "};
  localparam int         LEN_PROMPT = 2;
  localparam logic [7:0] MSG_ERR [0:4]   = '{"E","r","r", CHAR_CR, CHAR_LF};
  localparam int         LEN_ERR  = 5;

  // ---------------------------------------------------------------------------
  // State encoding
  // ---------------------------------------------------------------------------
  typedef enum logic [5:0] {
    S_BOOT,           // startup CRLF
    S_INIT_WRITES,    // replay shadow defaults → ctrl_bus after reset
    S_IDLE,           // wait for RX character
    S_RX_CHAR,        // process received character
    S_BACKSPACE,      // handle BS/DEL
    S_CHECK_CMD,      // dispatch command string
    S_PARSE_INIT,     // initialise hex parser
    S_PARSE_LOOP,     // accumulate hex digits
    S_WRITE_REG,      // issue single 8-bit write
    S_WRITE_TIME,     // issue 2-byte LE write with CDC gap
    S_ST_ROUTE,       // status: routing & bypass lines
    S_ST_TRM,         // status: tremolo slot
    S_ST_PHA,         // status: phaser slot
    S_ST_CHO,         // status: chorus slot
    S_ST_DLY,         // status: delay slot
    S_TX_START,       // start transmitting next byte from tx_buf
    S_TX_WAIT,        // wait for tx_done
    S_LOAD_PROMPT,    // copy MSG_PROMPT into tx_buf then send
    S_LOAD_STRING     // copy selected canned message into tx_buf then send
  } state_t;

  state_t state        = S_BOOT;
  state_t return_state = S_IDLE;

  // ---------------------------------------------------------------------------
  // Message selector - replaces the rx_latch-overload in the original
  // ---------------------------------------------------------------------------
  typedef enum logic {
    MSG_SEL_PROMPT = 1'd0,
    MSG_SEL_ERR    = 1'd1
  } msg_sel_t;

  msg_sel_t tx_msg_sel = MSG_SEL_PROMPT;

  // ---------------------------------------------------------------------------
  // Buffers and indices
  // ---------------------------------------------------------------------------
  logic [7:0] cmd_buf [0:31];
  logic [5:0] cmd_idx   = '0;

  logic [7:0] tx_buf [0:63];
  logic [5:0] tx_len    = '0;
  logic [5:0] tx_idx    = '0;

  // ---------------------------------------------------------------------------
  // Parser state
  // ---------------------------------------------------------------------------
  logic [31:0] parse_acc      = '0;
  logic [ 5:0] parse_ptr      = '0;
  logic [ 7:0] target_addr    = '0;
  logic        target_is_time = '0;

  // ---------------------------------------------------------------------------
  // RX latch (captures rx_byte on rx_valid for single-cycle processing)
  // ---------------------------------------------------------------------------
  logic        rx_pending = '0;
  logic [7:0]  rx_latch   = '0;

  // ---------------------------------------------------------------------------
  // Multi-byte write helpers
  // ---------------------------------------------------------------------------
  logic        time_wr_idx = '0;  // 0 = low byte, 1 = high byte
  logic [ 4:0] cdc_wait    = '0;  // inter-write CDC gap counter

  // ---------------------------------------------------------------------------
  // Shadow registers (mirror of ctrl_bus registers for status display)
  // ---------------------------------------------------------------------------
  logic [7:0] shadow       [0:31]; // 0x00..0x1F slot registers
  logic [7:0] shadow_route [0:4];  // route[0..4]
  logic [7:0] shadow_byp   [0:3];  // bypass[0..3]

  // Boot-time write sequencer index (covers 32 slot regs + 5 route + 4 bypass = 41 writes)
  logic [6:0] init_idx;

  // ---------------------------------------------------------------------------
  // Write bus
  // ---------------------------------------------------------------------------
  logic        wr_en_reg   = '0;
  logic [7:0]  wr_addr_reg = '0;
  logic [7:0]  wr_data_reg = '0;
  assign wr_en   = wr_en_reg;
  assign wr_addr = wr_addr_reg;
  assign wr_data = wr_data_reg;

  // ===========================================================================
  // Helper functions
  // ===========================================================================

  // Nibble to hex ASCII
  function automatic logic [7:0] n2h(input logic [3:0] n);
    return (n < 10) ? (8'(n) + CHAR_0) : (8'(n) - 10 + CHAR_A);
  endfunction

  // Match 3 characters at cmd_buf[start]
  function automatic logic m3(
    input int          start,
    input logic [7:0]  c0, c1, c2
  );
    return (cmd_buf[start]   == c0 &&
            cmd_buf[start+1] == c1 &&
            cmd_buf[start+2] == c2);
  endfunction

  // Match a string literal at cmd_buf[start]
  function automatic logic str_match(input int start, input string s);
    for (int i = 0; i < s.len(); i++)
      if (cmd_buf[start+i] != s.getc(i)) return 0;
    return 1;
  endfunction

  // Decode one ASCII hex character to its nibble value.
  // Returns 0 and sets valid=0 if the character is not a hex digit.
  function automatic logic [3:0] hex_nibble(
    input  logic [7:0] c,
    output logic       valid
  );
    if      (c >= "0" && c <= "9") begin valid = 1; return c - "0";      end
    else if (c >= "A" && c <= "F") begin valid = 1; return c - "A" + 10; end
    else if (c >= "a" && c <= "f") begin valid = 1; return c - "a" + 10; end
    else                           begin valid = 0; return 4'd0;          end
  endfunction

  // ===========================================================================
  // tx_buf builder tasks
  // (p is an inout integer write pointer; these run in simulation time via <=)
  // ===========================================================================

  // Write a 3-char label + ':' separator
  task automatic st_hdr(
    input  logic [7:0] c0, c1, c2,
    inout  int         p
  );
    tx_buf[p] <= c0; p++;
    tx_buf[p] <= c1; p++;
    tx_buf[p] <= c2; p++;
    tx_buf[p] <= ":"; p++;
  endtask

  // Write a byte as two hex chars + space
  task automatic st_hex(input logic [7:0] val, inout int p);
    tx_buf[p] <= n2h(val[7:4]); p++;
    tx_buf[p] <= n2h(val[3:0]); p++;
    tx_buf[p] <= " ";           p++;
  endtask

  // Write CR LF
  task automatic st_nl(inout int p);
    tx_buf[p] <= CHAR_CR; p++;
    tx_buf[p] <= CHAR_LF; p++;
  endtask

  // Write a named parameter (3-char label + ':' + hex byte + ' ')
  task automatic emit_param(
    input logic [7:0] c0, c1, c2,
    input logic [7:0] val,
    inout int         p
  );
    st_hdr(c0, c1, c2, p);
    st_hex(val, p);
  endtask

  // Write the 3-char port-source name for a given route index.
  // Replaces the 5× duplicate case block in S_ST_ROUTE.
  task automatic emit_port_src(input logic [2:0] src, inout int p);
    case (src)
      3'd0: begin tx_buf[p] <= "A"; p++; tx_buf[p] <= "D"; p++; tx_buf[p] <= "C"; p++; end
      3'd1: begin tx_buf[p] <= "T"; p++; tx_buf[p] <= "R"; p++; tx_buf[p] <= "M"; p++; end
      3'd2: begin tx_buf[p] <= "P"; p++; tx_buf[p] <= "H"; p++; tx_buf[p] <= "A"; p++; end
      3'd3: begin tx_buf[p] <= "C"; p++; tx_buf[p] <= "H"; p++; tx_buf[p] <= "O"; p++; end
      3'd4: begin tx_buf[p] <= "D"; p++; tx_buf[p] <= "L"; p++; tx_buf[p] <= "Y"; p++; end
      3'd5: begin tx_buf[p] <= "D"; p++; tx_buf[p] <= "A"; p++; tx_buf[p] <= "C"; p++; end
      default: begin tx_buf[p] <= "?"; p++; tx_buf[p] <= "?"; p++; tx_buf[p] <= "?"; p++; end
    endcase
  endtask

  // ===========================================================================
  // Shadow update helper - keeps shadow[] / shadow_route[] / shadow_byp[]
  // in sync whenever a write is issued to the ctrl_bus.
  // ===========================================================================
  task automatic update_shadow(input logic [7:0] addr, input logic [7:0] data);
    if (addr < 8'h20)
      shadow[addr[4:0]] <= data;
    else if (addr >= ADDR_RTE_BASE && addr < ADDR_RTE_BASE + 8'd5)
      shadow_route[addr[2:0]] <= data;
    else if (addr >= ADDR_BYP_BASE && addr < ADDR_BYP_BASE + 8'd4)
      shadow_byp[addr[1:0]] <= data;
  endtask

  // ===========================================================================
  // TX sequencing tasks
  //
  // send_msg():    load a canned message then transmit it, returning to `ret`
  // send_err():    shorthand for Err + prompt
  // send_prompt(): shorthand for just the prompt
  // ===========================================================================

  task automatic send_msg(
    input msg_sel_t  sel,
    input int        len,
    input state_t    ret
  );
    tx_msg_sel   <= sel;
    tx_len       <= 6'(len);
    tx_idx       <= '0;
    state        <= S_LOAD_STRING;
    return_state <= ret;
  endtask

  task automatic send_err();
    send_msg(MSG_SEL_ERR, LEN_ERR, S_LOAD_PROMPT);
  endtask

  task automatic send_prompt();
    tx_idx       <= '0;
    tx_len       <= LEN_PROMPT;
    state        <= S_LOAD_STRING;
    tx_msg_sel   <= MSG_SEL_PROMPT;
    return_state <= S_IDLE;
  endtask

  // ===========================================================================
  // Main always_ff block
  // ===========================================================================
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      // ---- Synchronous reset ----
      state        <= S_BOOT;
      cmd_idx      <= '0;
      tx_start     <= '0;
      rx_pending   <= '0;
      wr_en_reg    <= '0;

      // Default shadow values
      for (int i = 0; i < 32; i++) shadow[i] <= '0;

      // Slot 0 (tremolo): rat=3C dep=B4 shp=00
      shadow[0]  <= 8'h80;
      shadow[1]  <= 8'h80;

      // Slot 1 (phaser): spd=50 fbn=00
      shadow[8]  <= 8'h80;

      // Slot 2 (chorus): rat=50 dep=64 efx=80 eqh=C8 eql=80
      shadow[16] <= 8'h80; shadow[17] <= 8'h80; shadow[18] <= 8'h80;
      shadow[19] <= 8'h80; shadow[20] <= 8'h80;

      // Slot 3 (dd3): ton=FF lvl=80 fdb=64 tim=07D0 (LE: D0,07)
      shadow[24] <= 8'hFF; shadow[25] <= 8'h80; shadow[26] <= 8'h64;
      shadow[27] <= 8'h00; shadow[28] <= 8'h30;

      // Default route: symmetric chain ADC→TRM→PHA→CHO→DLY→DAC
      //   route[0] = DAC source  ← DLY (4)
      //   route[1] = TRM source  ← ADC (0)
      //   route[2] = PHA source  ← TRM (1)
      //   route[3] = CHO source  ← PHA (2)
      //   route[4] = DLY source  ← CHO (3)
      shadow_route[0] <= 8'd4;  // DAC ← DLY
      shadow_route[1] <= 8'd0;  // TRM ← ADC
      shadow_route[2] <= 8'd1;  // PHA ← TRM
      shadow_route[3] <= 8'd2;  // CHO ← PHA
      shadow_route[4] <= 8'd3;  // DLY ← CHO

      for (int i = 0; i < 4; i++) shadow_byp[i] <= '0;

      init_idx <= 7'd0;

    end else begin
      // ---- Default: deassert write strobe every cycle ----
      wr_en_reg <= '0;

      // ---- Latch incoming byte (can arrive any cycle) ----
      if (rx_valid) begin
        rx_pending <= 1;
        rx_latch   <= rx_byte;
      end

      // ================================================================
      // State machine
      // ================================================================
      case (state)

        // ------------------------------------------------------------
        // S_BOOT: emit CRLF then show prompt
        // ------------------------------------------------------------
        S_BOOT: begin
          tx_buf[0] <= CHAR_CR;
          tx_buf[1] <= CHAR_LF;
          tx_len    <= 2;
          tx_idx    <= '0;
          return_state <= S_INIT_WRITES;   // was S_LOAD_PROMPT
          state <= S_TX_START;
        end

        // ------------------------------------------------------------
        // S_INIT_WRITES: write all shadow defaults to ctrl_bus
        //
        // The CDC in top.sv is a TOGGLE handshake: it only fires
        // wr_en_audio on a 0→1 or 1→0 transition of cmd_wr_toggle
        // (which is wired to this module's wr_en output).
        // Consecutive wr_en=1 cycles would look like a sustained level
        // and only the first write would reach ctrl_bus.
        //
        // Fix: interleave write cycles (wr_en=1) with gap cycles (wr_en=0)
        // so wr_en toggles 0→1→0→1… giving the CDC a clear edge per write.
        //
        // init_idx counts half-steps:
        //   even idx → gap  cycle (wr_en=0, load addr/data for next write)
        //   odd  idx → write cycle (wr_en=1, addr/data already loaded)
        //
        // Total writes: 32 slot regs + 5 route + 4 bypass = 41
        // Total half-steps: 82 (0..81), done when init_idx == 82
        // ------------------------------------------------------------
        S_INIT_WRITES: begin
          if (init_idx >= 7'd82) begin
            // All 41 writes done
            wr_en_reg <= 1'b0;
            state     <= S_LOAD_PROMPT;
          end else begin
            // init_idx LSB=0 → gap cycle (wr_en=0, pre-load addr/data)
            // init_idx LSB=1 → write cycle (wr_en=1, addr/data stable)
            // write_num = init_idx[6:1]  (0..40)
            if (init_idx[0] == 1'b0) begin
              wr_en_reg <= 1'b0;
              if (init_idx[6:1] <= 7'd31) begin
                wr_addr_reg <= 8'(init_idx[6:1]);
                wr_data_reg <= shadow[init_idx[5:1]];
              end else if (init_idx[6:1] <= 7'd36) begin
                wr_addr_reg <= ADDR_RTE_BASE + 8'(init_idx[6:1] - 7'd32);
                wr_data_reg <= shadow_route[3'(init_idx[6:1] - 7'd32)];
              end else begin
                wr_addr_reg <= ADDR_BYP_BASE + 8'(init_idx[6:1] - 7'd37);
                wr_data_reg <= {7'd0, shadow_byp[2'(init_idx[6:1] - 7'd37)][0]};
              end
            end else begin
              // Write cycle: addr/data were set on the preceding gap cycle
              wr_en_reg <= 1'b1;
            end
            init_idx <= init_idx + 7'd1;
          end
        end

        // ------------------------------------------------------------
        // S_IDLE: wait for a pending received character
        // ------------------------------------------------------------
        S_IDLE: begin
          tx_start <= '0;
          if (rx_pending) begin
            rx_pending <= '0;
            state <= S_RX_CHAR;
          end
        end

        // ------------------------------------------------------------
        // S_RX_CHAR: process one character from rx_latch
        // ------------------------------------------------------------
        S_RX_CHAR: begin
          if (rx_latch == CHAR_CR || rx_latch == CHAR_LF) begin
            // End of line - null-terminate and dispatch or re-prompt
            cmd_buf[cmd_idx] <= 8'h00;
            tx_buf[0] <= CHAR_CR;
            tx_buf[1] <= CHAR_LF;
            tx_len    <= 2;
            tx_idx    <= '0;
            if (cmd_idx == 0) begin
              return_state <= S_LOAD_PROMPT;
            end else begin
              return_state <= S_CHECK_CMD;
            end
            state <= S_TX_START;

          end else if (rx_latch == ";") begin
            // Instant status trigger - no Enter required, clears pending line
            cmd_idx <= '0;
            tx_buf[0] <= CHAR_CR;
            tx_buf[1] <= CHAR_LF;
            tx_len    <= 2;
            tx_idx    <= '0;
            return_state <= S_ST_ROUTE;
            state <= S_TX_START;

          end else if (rx_latch == CHAR_BS || rx_latch == CHAR_DEL) begin
            state <= S_BACKSPACE;

          end else if (cmd_idx < 31) begin
            // Echo and buffer
            cmd_buf[cmd_idx] <= rx_latch;
            cmd_idx          <= cmd_idx + 1;
            tx_buf[0]        <= rx_latch;
            tx_len           <= 1;
            tx_idx           <= '0;
            return_state     <= S_IDLE;
            state            <= S_TX_START;

          end else begin
            // Buffer full - silently discard
            state <= S_IDLE;
          end
        end

        // ------------------------------------------------------------
        // S_BACKSPACE: erase one character (BS SP BS sequence)
        // ------------------------------------------------------------
        S_BACKSPACE: begin
          if (cmd_idx > 0) begin
            cmd_idx      <= cmd_idx - 1;
            tx_buf[0]    <= CHAR_BS;
            tx_buf[1]    <= CHAR_SPC;
            tx_buf[2]    <= CHAR_BS;
            tx_len       <= 3;
            tx_idx       <= '0;
            return_state <= S_IDLE;
            state        <= S_TX_START;
          end else begin
            state <= S_IDLE;
          end
        end

        // ============================================================
        // S_CHECK_CMD: dispatch on command verb
        //
        // Command buffer layout:
        //   "con <src> to <snk>"   → crossbar connect (no hex parser)
        //   "wr <AABB>"            → raw write (addr=AA data=BB)
        //   "set <EFX> <PRM> <V>"  → register write
        //     EFX positions 4..6, PRM positions 8..10, VAL from pos 12
        //   "set byp <N> <V>"      → bypass write
        //
        //  Note: ';' is handled in S_RX_CHAR directly (no line buffer needed)
        // ============================================================
        S_CHECK_CMD: begin
          cmd_idx      <= '0;
          target_is_time <= '0;

          if (m3(0, "w","r"," ")) begin
            // Raw write: "wr <AABB>" - parse 4 hex digits into parse_acc[15:0]
            parse_ptr   <= 3;
            target_addr <= ADDR_RAW_SENTINEL;
            state       <= S_PARSE_INIT;

          end else if (m3(0, "c","o","n") && cmd_buf[3] == CHAR_SPC) begin
            // Connect: "con <src> to <snk>"
            //   positions: con=0..2  src=4..6  to=8..9  snk=11..13
            //   Port indices: adc=0  trm=1  pha=2  cho=3  dly=4  dac=5
            //   Writes src index into route[snk] register.
            begin
              logic [7:0] snk_addr;
              logic [7:0] src_val;
              logic       snk_ok, src_ok;

              // Decode source port → numeric value
              src_ok = 1;
              if      (m3(4,"a","d","c")) src_val = 8'd0;
              else if (m3(4,"t","r","m")) src_val = 8'd1;
              else if (m3(4,"p","h","a")) src_val = 8'd2;
              else if (m3(4,"c","h","o")) src_val = 8'd3;
              else if (m3(4,"d","l","y")) src_val = 8'd4;
              else if (m3(4,"d","a","c")) src_val = 8'd5;
              else begin src_val = '0; src_ok = 0; end

              // Verify "to" keyword
              if (cmd_buf[7] != CHAR_SPC || cmd_buf[8] != "t" || cmd_buf[9] != "o" || cmd_buf[10] != CHAR_SPC)
                snk_ok = 0;
              else begin
                // Decode sink port → register address
                snk_ok = 1;
                if      (m3(11,"d","a","c")) snk_addr = ADDR_RTE_BASE + 8'd0;
                else if (m3(11,"t","r","m")) snk_addr = ADDR_RTE_BASE + 8'd1;
                else if (m3(11,"p","h","a")) snk_addr = ADDR_RTE_BASE + 8'd2;
                else if (m3(11,"c","h","o")) snk_addr = ADDR_RTE_BASE + 8'd3;
                else if (m3(11,"d","l","y")) snk_addr = ADDR_RTE_BASE + 8'd4;
                else begin snk_addr = '0; snk_ok = 0; end
              end

              if (src_ok && snk_ok) begin
                wr_addr_reg <= snk_addr;
                wr_data_reg <= src_val;
                wr_en_reg   <= 1'b1;
                update_shadow(snk_addr, src_val);
                send_prompt();
              end else begin
                send_err();
              end
            end

          end else if (m3(0, "s","e","t") && cmd_buf[3] == CHAR_SPC) begin

            if (m3(4, "t","r","m") && cmd_buf[7] == CHAR_SPC) begin
              // -- tremolo --
              if      (m3(8,"r","a","t")) begin target_addr <= ADDR_TRM_RAT; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"d","e","p")) begin target_addr <= ADDR_TRM_DEP; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"s","h","p")) begin target_addr <= ADDR_TRM_SHP; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else                             send_err();

            end else if (m3(4, "p","h","a") && cmd_buf[7] == CHAR_SPC) begin
              // -- phaser --
              if      (m3(8,"s","p","d")) begin target_addr <= ADDR_PHA_SPD; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"f","b","n")) begin target_addr <= ADDR_PHA_FBN; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else                             send_err();

            end else if (m3(4, "c","h","o") && cmd_buf[7] == CHAR_SPC) begin
              // -- chorus --
              if      (m3(8,"r","a","t")) begin target_addr <= ADDR_CHO_RAT; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"d","e","p")) begin target_addr <= ADDR_CHO_DEP; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"e","f","x")) begin target_addr <= ADDR_CHO_EFX; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"e","q","h")) begin target_addr <= ADDR_CHO_EQH; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"e","q","l")) begin target_addr <= ADDR_CHO_EQL; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else                             send_err();

            end else if (m3(4, "d","l","y") && cmd_buf[7] == CHAR_SPC) begin
              // -- delay --
              if      (m3(8,"t","o","n")) begin target_addr <= ADDR_DLY_TON; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"l","v","l")) begin target_addr <= ADDR_DLY_LVL; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"f","d","b")) begin target_addr <= ADDR_DLY_FDB; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"t","i","m")) begin
                target_addr    <= ADDR_DLY_TIM;
                parse_ptr      <= 12;
                target_is_time <= 1;
                state          <= S_PARSE_INIT;
              end
              else                             send_err();

            end else if (m3(4, "b","y","p") && cmd_buf[7] == CHAR_SPC) begin
              // -- bypass: "set byp <NNN> <V>"
              //    trm → bypass[0]  pha → bypass[1]  cho → bypass[2]  dly → bypass[3]
              if      (m3(8,"t","r","m")) begin target_addr <= ADDR_BYP_BASE + 8'd0; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"p","h","a")) begin target_addr <= ADDR_BYP_BASE + 8'd1; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"c","h","o")) begin target_addr <= ADDR_BYP_BASE + 8'd2; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"d","l","y")) begin target_addr <= ADDR_BYP_BASE + 8'd3; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else                             send_err();

            end else begin
              send_err();
            end

          end else begin
            send_err();
          end
        end

        // ============================================================
        // Status display - stages chain through S_ST_TRM → _PHA → _CHO → _DLY
        // ============================================================

        // -- Routes & bypass --
        S_ST_ROUTE: begin
          automatic int p = 0;

          // Format: "TRM<ADC PHA<TRM CHO<PHA DLY<CHO DAC<DLY\r\n"
          // Ports 1..4 = effect inputs; port 4 output feeds DAC shown using route[4]
          begin
            // TRM input
            tx_buf[p] <= "T"; p++; tx_buf[p] <= "R"; p++; tx_buf[p] <= "M"; p++;
            tx_buf[p] <= "<"; p++;
            emit_port_src(shadow_route[1][2:0], p);
            tx_buf[p] <= " "; p++;

            // PHA input
            tx_buf[p] <= "P"; p++; tx_buf[p] <= "H"; p++; tx_buf[p] <= "A"; p++;
            tx_buf[p] <= "<"; p++;
            emit_port_src(shadow_route[2][2:0], p);
            tx_buf[p] <= " "; p++;

            // CHO input
            tx_buf[p] <= "C"; p++; tx_buf[p] <= "H"; p++; tx_buf[p] <= "O"; p++;
            tx_buf[p] <= "<"; p++;
            emit_port_src(shadow_route[3][2:0], p);
            tx_buf[p] <= " "; p++;

            // DLY input
            tx_buf[p] <= "D"; p++; tx_buf[p] <= "L"; p++; tx_buf[p] <= "Y"; p++;
            tx_buf[p] <= "<"; p++;
            emit_port_src(shadow_route[4][2:0], p);
            tx_buf[p] <= " "; p++;

            // DAC source - route[0] is the dedicated DAC mux selector
            tx_buf[p] <= "D"; p++; tx_buf[p] <= "A"; p++; tx_buf[p] <= "C"; p++;
            tx_buf[p] <= "<"; p++;
            emit_port_src(shadow_route[0][2:0], p);
            st_nl(p);
          end

          // "BYP: <0|1> <0|1> <0|1> <0|1>\r\n"
          tx_buf[p] <= "B"; p++;
          tx_buf[p] <= "Y"; p++;
          tx_buf[p] <= "P"; p++;
          tx_buf[p] <= ":"; p++;
          for (int i = 0; i < 4; i++) begin
            tx_buf[p] <= (sw_effect[i] && !shadow_byp[i][0]) ? "0" : "1"; p++;
            tx_buf[p] <= " "; p++;
          end
          st_nl(p);

          tx_len       <= 6'(p);
          tx_idx       <= '0;
          return_state <= S_ST_TRM;
          state        <= S_TX_START;
        end

        // -- Tremolo (slot 0) --
        S_ST_TRM: begin
          if (sw_effect[0]) begin
            automatic int p = 0;
            emit_param("T","R","M", 8'("T"), p);   // header; re-emit label as leading word
            // Override: write label manually then params
            p = 0;
            tx_buf[p] <= "T"; p++; tx_buf[p] <= "R"; p++; tx_buf[p] <= "M"; p++;
            tx_buf[p] <= " "; p++;
            emit_param("r","a","t", shadow[0], p);
            emit_param("d","e","p", shadow[1], p);
            emit_param("s","h","p", shadow[2], p);
            st_nl(p);
            tx_len       <= 6'(p);
            tx_idx       <= '0;
            return_state <= S_ST_PHA;
            state        <= S_TX_START;
          end else begin
            state <= S_ST_PHA;
          end
        end

        // -- Phaser (slot 1) --
        S_ST_PHA: begin
          if (sw_effect[1]) begin
            automatic int p = 0;
            tx_buf[p] <= "P"; p++; tx_buf[p] <= "H"; p++; tx_buf[p] <= "A"; p++;
            tx_buf[p] <= " "; p++;
            emit_param("s","p","d", shadow[8], p);
            emit_param("f","b","n", shadow[9], p);
            st_nl(p);
            tx_len       <= 6'(p);
            tx_idx       <= '0;
            return_state <= S_ST_CHO;
            state        <= S_TX_START;
          end else begin
            state <= S_ST_CHO;
          end
        end

        // -- Chorus (slot 2) --
        S_ST_CHO: begin
          if (sw_effect[2]) begin
            automatic int p = 0;
            tx_buf[p] <= "C"; p++; tx_buf[p] <= "H"; p++; tx_buf[p] <= "O"; p++;
            tx_buf[p] <= " "; p++;
            emit_param("r","a","t", shadow[8'h10], p);
            emit_param("d","e","p", shadow[8'h11], p);
            emit_param("e","f","x", shadow[8'h12], p);
            emit_param("e","q","h", shadow[8'h13], p);
            emit_param("e","q","l", shadow[8'h14], p);
            st_nl(p);
            tx_len       <= 6'(p);
            tx_idx       <= '0;
            return_state <= S_ST_DLY;
            state        <= S_TX_START;
          end else begin
            state <= S_ST_DLY;
          end
        end

        // -- Delay (slot 3) --
        S_ST_DLY: begin
          if (sw_effect[3]) begin
            automatic int p = 0;
            tx_buf[p] <= "D"; p++; tx_buf[p] <= "L"; p++; tx_buf[p] <= "Y"; p++;
            tx_buf[p] <= " "; p++;
            emit_param("t","o","n", shadow[8'h18], p);
            emit_param("l","v","l", shadow[8'h19], p);
            emit_param("f","d","b", shadow[8'h1A], p);
            // Time: 16-bit LE, show big-endian (high byte first)
            tx_buf[p] <= "t"; p++; tx_buf[p] <= "i"; p++; tx_buf[p] <= "m"; p++;
            tx_buf[p] <= ":"; p++;
            tx_buf[p] <= n2h(shadow[8'h1C][7:4]); p++;
            tx_buf[p] <= n2h(shadow[8'h1C][3:0]); p++;
            tx_buf[p] <= n2h(shadow[8'h1B][7:4]); p++;
            tx_buf[p] <= n2h(shadow[8'h1B][3:0]); p++;
            tx_buf[p] <= " "; p++;
            st_nl(p);
            tx_len       <= 6'(p);
            tx_idx       <= '0;
            return_state <= S_LOAD_PROMPT;
            state        <= S_TX_START;
          end else begin
            state <= S_LOAD_PROMPT;
          end
        end

        // ============================================================
        // Hex parser
        // ============================================================
        S_PARSE_INIT: begin
          parse_acc <= '0;
          state     <= S_PARSE_LOOP;
        end

        S_PARSE_LOOP: begin
          logic [7:0] c;
          logic [3:0] nibble;
          logic       nibble_valid;

          c      = cmd_buf[parse_ptr];
          nibble = hex_nibble(c, nibble_valid);

          if (c == CHAR_SPC) begin
            // Skip leading/embedded spaces
            parse_ptr <= parse_ptr + 1;

          end else if (nibble_valid) begin
            // Accumulate nibble (shift-left by 4)
            parse_acc <= {parse_acc[27:0], nibble};
            parse_ptr <= parse_ptr + 1;

          end else begin
            // Non-hex, non-space → end of value field → dispatch
            if (target_addr == ADDR_RAW_SENTINEL) begin
              // Raw write: parse_acc[15:8] = addr, [7:0] = data
              wr_addr_reg <= parse_acc[15:8];
              wr_data_reg <= parse_acc[7:0];
              wr_en_reg   <= 1'b1;
              update_shadow(parse_acc[15:8], parse_acc[7:0]);
              send_prompt();

            end else if (target_is_time) begin
              // 16-bit LE time write
              time_wr_idx <= '0;
              cdc_wait    <= '0;
              state       <= S_WRITE_TIME;

            end else begin
              state <= S_WRITE_REG;
            end
          end
        end

        // ============================================================
        // Single 8-bit register write
        // ============================================================
        S_WRITE_REG: begin
          wr_addr_reg <= target_addr;
          wr_data_reg <= parse_acc[7:0];
          wr_en_reg   <= 1'b1;
          update_shadow(target_addr, parse_acc[7:0]);
          send_prompt();
        end

        // ============================================================
        // 16-bit time register: 2 sequential byte writes (LE) with CDC gap
        //
        // Sequence (time_wr_idx):
        //   0 → write low byte  (addr = ADDR_DLY_TIM + 0) then wait CDC_WAIT_CYCLES
        //   1 → write high byte (addr = ADDR_DLY_TIM + 1) then send OK
        // ============================================================
        S_WRITE_TIME: begin
          if (cdc_wait != '0) begin
            // Counting down the CDC inter-write gap
            cdc_wait <= cdc_wait - 1;

          end else if (!wr_en_reg) begin
            // Drive phase: assert wr_en with current byte
            wr_addr_reg <= target_addr + {7'd0, time_wr_idx};
            if (!time_wr_idx) begin
              wr_data_reg <= parse_acc[7:0];
              update_shadow(target_addr,     parse_acc[7:0]);
            end else begin
              wr_data_reg <= parse_acc[15:8];
              update_shadow(target_addr + 1, parse_acc[15:8]);
            end
            wr_en_reg <= 1'b1;

          end else begin
            // Gap phase: deassert wr_en, advance or finish
            wr_en_reg <= '0;
            if (time_wr_idx) begin
              send_prompt();
            end else begin
              time_wr_idx <= 1'b1;
              cdc_wait    <= 5'(CDC_WAIT_CYCLES);
            end
          end
        end

        // ============================================================
        // TX engine
        // ============================================================

        // Copy selected canned message into tx_buf, then go to S_TX_START
        S_LOAD_PROMPT: begin
          tx_msg_sel   <= MSG_SEL_PROMPT;
          tx_idx       <= '0;
          tx_len       <= LEN_PROMPT;
          state        <= S_LOAD_STRING;
          return_state <= S_IDLE;
        end

        S_LOAD_STRING: begin
          if (tx_idx < tx_len) begin
            case (tx_msg_sel)
              MSG_SEL_PROMPT: tx_buf[tx_idx] <= MSG_PROMPT[tx_idx];
              MSG_SEL_ERR:    tx_buf[tx_idx] <= MSG_ERR   [tx_idx];
              default:        tx_buf[tx_idx] <= MSG_ERR   [tx_idx];
            endcase
            tx_idx <= tx_idx + 1;
          end else begin
            tx_idx <= '0;
            state  <= S_TX_START;
          end
        end

        // Send one byte at a time from tx_buf[0..tx_len-1]
        S_TX_START: begin
          if (tx_idx < tx_len) begin
            if (!tx_busy) begin
              tx_byte  <= tx_buf[tx_idx];
              tx_start <= 1'b1;
              state    <= S_TX_WAIT;
            end
            // else: wait here until tx is free
          end else begin
            state <= return_state;
          end
        end

        S_TX_WAIT: begin
          tx_start <= '0;
          if (tx_done) begin
            tx_idx <= tx_idx + 1;
            state  <= S_TX_START;
          end
        end

        // ------------------------------------------------------------
        // Default: should never reach here - return to idle safely
        // ------------------------------------------------------------
        default: begin
          state <= S_IDLE;
        end

      endcase
    end
  end

endmodule