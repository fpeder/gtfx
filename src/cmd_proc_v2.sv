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
//  - S_ST_ROUTE out-of-bounds fix: shadow_route is [0:6]; DAC uses index 0
//  - S_PARSE_LOOP: nibble extraction unified into one branch that covers 0-9/A-F/a-f
//  - S_WRITE_TIME comment corrected (2-byte LE write, not 4-byte)
//  - `default` branch added to the state case to prevent latch inference
//  - All magic numbers replaced with named localparam constants
//  - State enum widened to 6 bits and kept contiguous for easy expansion
//  - OK response removed; successful writes go directly back to prompt
//  - "status" command remapped to single character ';'
//  - Slot 4 (tube_distortion) added: gain / tone_bass / tone_mid / tone_treble / level
//  - sw_effect input removed; bypass state is entirely software-controlled
//  - "set <efx> on/off" commands write bypass[slot]=0/1 directly (no rerouting)
//  - Routing modifiable at runtime via "con <src> <dst>" command
//  - Status display shows full routing table from shadow_route
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

    // Flat register write bus (to ctrl_bus)
    output logic        wr_en,
    output logic [7:0]  wr_addr,
    output logic [7:0]  wr_data
);

  // ===========================================================================
  // ctrl_bus Address Map
  //
  //  Slot registers (8 regs per slot):
  //    0x00..0x07 : Slot 0 (tremolo)        [0]=rate  [1]=depth  [2]=shape
  //    0x08..0x0F : Slot 1 (phaser)         [0]=speed [1]=feedback_en
  //    0x10..0x17 : Slot 2 (chorus)         [0]=rate  [1]=depth  [2]=efx  [3]=eqhi  [4]=eqlo
  //    0x18..0x1F : Slot 3 (dd3)            [0]=tone  [1]=level  [2]=feedback  [3..4]=time(LE)
  //    0x20..0x27 : Slot 4 (tube_distortion)[0]=gain  [1]=tone_bass [2]=tone_mid [3]=tone_treble [4]=level
  //    0x28..0x2F : Slot 5 (flanger)       [0]=manual [1]=width [2]=speed [3]=regen
  //
  //  Infrastructure (symmetric 7-port: 0=ADC, 1=TRM, 2=PHA, 3=CHO, 4=DLY, 5=TUB, 6=FLN):
  //    0x40..0x46 : route[0..6]
  //
  // CLI format: set <efx> <prm> <hex>     (all qualifiers 3 chars)
  //
  //   set trm rat <hex>       → 0x00     con <src> <dst>
  //   set trm dep <hex>       → 0x01       src/dst ∈ {adc,trm,pha,cho,dly,tub,fln,dac}
  //   set trm shp <hex>       → 0x02       e.g. "con adc trm" sets route[1]=0
  //   set pha spd <hex>       → 0x08
  //   set pha fbn <hex>       → 0x09     set byp trm <hex>   → 0x48
  //   set cho rat <hex>       → 0x10     set byp pha <hex>   → 0x49
  //   set cho dep <hex>       → 0x11     set byp cho <hex>   → 0x4A
  //   set cho efx <hex>       → 0x12
  //   set cho eqh <hex>       → 0x13     set <efx> on   → bypass[slot] = 0 (active)
  //   set cho eql <hex>       → 0x14     set <efx> off  → bypass[slot] = 1 (bypassed)
  //   set dly ton <hex>       → 0x18       <efx> ∈ {tub,trm,pha,cho,dly}
  //   set dly lvl <hex>       → 0x19
  //   set dly fdb <hex>       → 0x1A     wr <addr><data>  → raw write
  //   set dly tim <hex8>      → 0x1B..0x1C (2 writes, LE)
  //   set tub gai <hex>       → 0x20     ;                → status dump
  //   set tub bas <hex>       → 0x21
  //   set tub mid <hex>       → 0x22
  //   set tub tre <hex>       → 0x23
  //   set tub lvl <hex>       → 0x24
  //   set fln man <hex>       → 0x28     flanger manual (base delay)
  //   set fln wid <hex>       → 0x29     flanger width  (LFO depth)
  //   set fln spd <hex>       → 0x2A     flanger speed  (LFO rate)
  //   set fln reg <hex>       → 0x2B     flanger regen  (feedback, 0x80=center)
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
  localparam logic [7:0] ADDR_TUB_GAI = 8'h20;   // tube pre-gain
  localparam logic [7:0] ADDR_TUB_BAS = 8'h21;   // tube tone_bass
  localparam logic [7:0] ADDR_TUB_MID = 8'h22;   // tube tone_mid
  localparam logic [7:0] ADDR_TUB_TRE = 8'h23;   // tube tone_treble
  localparam logic [7:0] ADDR_TUB_LVL = 8'h24;   // tube output level
  localparam logic [7:0] ADDR_FLN_MAN = 8'h28;   // flanger manual (base delay)
  localparam logic [7:0] ADDR_FLN_WID = 8'h29;   // flanger width  (LFO depth)
  localparam logic [7:0] ADDR_FLN_SPD = 8'h2A;   // flanger speed  (LFO rate)
  localparam logic [7:0] ADDR_FLN_REG = 8'h2B;   // flanger regen  (feedback)
  localparam logic [7:0] ADDR_RTE_BASE = 8'h40;
  // Bypass is at cfg_mem[slot*8+7] - one address per slot
  localparam logic [7:0] ADDR_BYP_TRM = 8'h07;   // tremolo  bypass bit
  localparam logic [7:0] ADDR_BYP_PHA = 8'h0F;   // phaser   bypass bit
  localparam logic [7:0] ADDR_BYP_CHO = 8'h17;   // chorus   bypass bit
  localparam logic [7:0] ADDR_BYP_DLY = 8'h1F;   // dd3      bypass bit
  localparam logic [7:0] ADDR_BYP_TUB = 8'h27;   // tube     bypass bit
  localparam logic [7:0] ADDR_BYP_FLN = 8'h2F;   // flanger  bypass bit
  localparam logic [7:0] ADDR_RAW_SENTINEL = 8'hFF;

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
    S_ST_TUB,         // status: tube distortion slot
    S_ST_FLN,         // status: flanger slot
    S_TX_START,       // start transmitting next byte from tx_buf
    S_TX_WAIT,        // wait for tx_done
    S_LOAD_PROMPT,    // copy MSG_PROMPT into tx_buf then send
    S_LOAD_STRING     // copy selected canned message into tx_buf then send
  } state_t;

  state_t state        = S_BOOT;
  state_t return_state = S_IDLE;

  // ---------------------------------------------------------------------------
  // Message selector
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
  // RX latch
  // ---------------------------------------------------------------------------
  logic        rx_pending = '0;
  logic [7:0]  rx_latch   = '0;

  // ---------------------------------------------------------------------------
  // Multi-byte write helpers
  // ---------------------------------------------------------------------------
  logic        time_wr_idx = '0;
  logic [ 4:0] cdc_wait    = '0;

  // ---------------------------------------------------------------------------
  // Shadow registers
  //   shadow[0x00..0x2F] : slot registers (6 slots × 8 regs = 48 entries)
  //                        shadow[slot*8+7][0] = bypass bit for that slot
  //   shadow_route[0..6] : route[0..6]
  // ---------------------------------------------------------------------------
  logic [7:0] shadow       [0:47];
  logic [7:0] shadow_route [0:6];

  // Boot-time write sequencer
  // Total writes: 48 slot regs (includes bypass at [7]) + 7 route = 55
  // Total half-steps: 110 (0..109), done when init_idx == 110
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

  function automatic logic [7:0] n2h(input logic [3:0] n);
    return (n < 10) ? (8'(n) + CHAR_0) : (8'(n) - 10 + CHAR_A);
  endfunction

  function automatic logic m3(
    input int          start,
    input logic [7:0]  c0, c1, c2
  );
    return (cmd_buf[start]   == c0 &&
            cmd_buf[start+1] == c1 &&
            cmd_buf[start+2] == c2);
  endfunction

  function automatic logic str_match(input int start, input string s);
    for (int i = 0; i < s.len(); i++)
      if (cmd_buf[start+i] != s.getc(i)) return 0;
    return 1;
  endfunction

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
  // ===========================================================================

  task automatic st_hdr(
    input  logic [7:0] c0, c1, c2,
    inout  int         p
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

  task automatic emit_param(
    input logic [7:0] c0, c1, c2,
    input logic [7:0] val,
    inout int         p
  );
    st_hdr(c0, c1, c2, p);
    st_hex(val, p);
  endtask

  // Port source name - extended to include TUB (index 5)
  task automatic emit_port_src(input logic [2:0] src, inout int p);
    case (src)
      3'd0: begin tx_buf[p] <= "A"; p++; tx_buf[p] <= "D"; p++; tx_buf[p] <= "C"; p++; end
      3'd1: begin tx_buf[p] <= "T"; p++; tx_buf[p] <= "R"; p++; tx_buf[p] <= "M"; p++; end
      3'd2: begin tx_buf[p] <= "P"; p++; tx_buf[p] <= "H"; p++; tx_buf[p] <= "A"; p++; end
      3'd3: begin tx_buf[p] <= "C"; p++; tx_buf[p] <= "H"; p++; tx_buf[p] <= "O"; p++; end
      3'd4: begin tx_buf[p] <= "D"; p++; tx_buf[p] <= "L"; p++; tx_buf[p] <= "Y"; p++; end
      3'd5: begin tx_buf[p] <= "T"; p++; tx_buf[p] <= "U"; p++; tx_buf[p] <= "B"; p++; end
      3'd6: begin tx_buf[p] <= "F"; p++; tx_buf[p] <= "L"; p++; tx_buf[p] <= "N"; p++; end
      default: begin tx_buf[p] <= "?"; p++; tx_buf[p] <= "?"; p++; tx_buf[p] <= "?"; p++; end
    endcase
  endtask

  // ===========================================================================
  // Shadow update helper
  // ===========================================================================
  task automatic update_shadow(input logic [7:0] addr, input logic [7:0] data);
    if (addr < 8'h30)                                             // 6 slots × 8 = 0x30
      shadow[addr[5:0]] <= data;
    else if (addr >= ADDR_RTE_BASE && addr < ADDR_RTE_BASE + 8'd7)
      shadow_route[addr[2:0]] <= data;
    // bypass addrs (0x07,0x0F,0x17,0x1F,0x27,0x2F) fall within the slot range above
  endtask

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
      state        <= S_BOOT;
      cmd_idx      <= '0;
      tx_start     <= '0;
      rx_pending   <= '0;
      wr_en_reg    <= '0;

      for (int i = 0; i < 48; i++) shadow[i] <= '0;

      // Slot 0 (tremolo)
      shadow[0]  <= 8'h3C;
      shadow[1]  <= 8'hB4;

      // Slot 1 (phaser)
      shadow[8]  <= 8'h50;

      // Slot 2 (chorus)
      shadow[16] <= 8'h50; shadow[17] <= 8'h64; shadow[18] <= 8'h80;
      shadow[19] <= 8'hC8; shadow[20] <= 8'h80;

      // Slot 3 (dd3)
      shadow[24] <= 8'hFF; shadow[25] <= 8'h80; shadow[26] <= 8'h64;
      shadow[27] <= 8'h00; shadow[28] <= 8'h30;

      // Slot 4 (tube_distortion): gain=40 bass=80 mid=80 treble=80 level=A0
      shadow[32] <= 8'h40;   // gain
      shadow[33] <= 8'h80;   // tone_bass
      shadow[34] <= 8'h80;   // tone_mid
      shadow[35] <= 8'h80;   // tone_treble
      shadow[36] <= 8'hA0;   // level

      // Slot 5 (flanger): manual=80 width=C0 speed=30 regen=A0
      shadow[40] <= 8'h80;   // manual
      shadow[41] <= 8'hC0;   // width
      shadow[42] <= 8'h30;   // speed
      shadow[43] <= 8'hA0;   // regen

      // shadow[7/15/23/31/39/47] = bypass bits; default 0x01 (all bypassed)
      shadow[7]  <= 8'h01;
      shadow[15] <= 8'h01;
      shadow[23] <= 8'h01;
      shadow[31] <= 8'h01;
      shadow[39] <= 8'h01;
      shadow[47] <= 8'h01;

      // Default route: ADC→FLN→TUB→TRM→PHA→CHO→DLY→DAC
      //   route[0] = DAC source  ← DLY (4)
      //   route[1] = TRM source  ← TUB (5)
      //   route[2] = PHA source  ← TRM (1)
      //   route[3] = CHO source  ← PHA (2)
      //   route[4] = DLY source  ← CHO (3)
      //   route[5] = TUB source  ← FLN (6)
      //   route[6] = FLN source  ← ADC (0)
      shadow_route[0] <= 8'd4;
      shadow_route[1] <= 8'd5;
      shadow_route[2] <= 8'd1;
      shadow_route[3] <= 8'd2;
      shadow_route[4] <= 8'd3;
      shadow_route[5] <= 8'd6;
      shadow_route[6] <= 8'd0;

      init_idx   <= 7'd0;

    end else begin
      wr_en_reg <= '0;

      if (rx_valid) begin
        rx_pending <= 1;
        rx_latch   <= rx_byte;
      end

      // ================================================================
      // State machine
      // ================================================================
      case (state)

        // ------------------------------------------------------------
        // S_BOOT
        // ------------------------------------------------------------
        S_BOOT: begin
          tx_buf[0] <= CHAR_CR;
          tx_buf[1] <= CHAR_LF;
          tx_len    <= 2;
          tx_idx    <= '0;
          return_state <= S_INIT_WRITES;
          state <= S_TX_START;
        end

        // ------------------------------------------------------------
        // S_INIT_WRITES
        //
        // Writes: 48 slot regs (bypass at [7] included) + 7 route = 55
        // Half-steps: 110 (0..109); done when init_idx == 110
        //   write_num = init_idx[6:1]  (0..54)
        //   0..47  → shadow[write_num]           addr = write_num
        //   48..54 → shadow_route[write_num-48]  addr = ADDR_RTE_BASE + (write_num-48)
        // ------------------------------------------------------------
        S_INIT_WRITES: begin
          if (init_idx >= 7'd110) begin
            wr_en_reg <= 1'b0;
            state     <= S_LOAD_PROMPT;
          end else begin
            if (init_idx[0] == 1'b0) begin
              wr_en_reg <= 1'b0;
              if (init_idx[6:1] <= 7'd47) begin
                wr_addr_reg <= 8'(init_idx[6:1]);
                wr_data_reg <= shadow[init_idx[6:1]];
              end else begin
                wr_addr_reg <= ADDR_RTE_BASE + 8'(init_idx[6:1] - 7'd48);
                wr_data_reg <= shadow_route[3'(init_idx[6:1] - 7'd48)];
              end
            end else begin
              wr_en_reg <= 1'b1;
            end
            init_idx <= init_idx + 7'd1;
          end
        end

        // ------------------------------------------------------------
        // S_IDLE
        // ------------------------------------------------------------
        S_IDLE: begin
          tx_start <= '0;
          if (rx_pending) begin
            rx_pending <= '0;
            state <= S_RX_CHAR;
          end
        end

        // ------------------------------------------------------------
        // S_RX_CHAR
        // ------------------------------------------------------------
        S_RX_CHAR: begin
          if (rx_latch == CHAR_CR || rx_latch == CHAR_LF) begin
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
            cmd_buf[cmd_idx] <= rx_latch;
            cmd_idx          <= cmd_idx + 1;
            tx_buf[0]        <= rx_latch;
            tx_len           <= 1;
            tx_idx           <= '0;
            return_state     <= S_IDLE;
            state            <= S_TX_START;

          end else begin
            state <= S_IDLE;
          end
        end

        // ------------------------------------------------------------
        // S_BACKSPACE
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
        // S_CHECK_CMD
        // ============================================================
        S_CHECK_CMD: begin
          cmd_idx        <= '0;
          target_is_time <= '0;

          if (m3(0, "w","r"," ")) begin
            parse_ptr   <= 3;
            target_addr <= ADDR_RAW_SENTINEL;
            state       <= S_PARSE_INIT;

          end else if (m3(0, "c","o","n") && cmd_buf[3] == CHAR_SPC) begin
            // con <src> <dst>   - connect source master to destination slave
            //   e.g. "con adc trm" sets route[1]=0  (TRM input ← ADC output)
            begin
              logic [7:0] dst_addr;
              logic [7:0] src_val;
              logic       dst_ok, src_ok;

              src_ok = 1;
              if      (m3(4,"a","d","c")) src_val = 8'd0;
              else if (m3(4,"t","r","m")) src_val = 8'd1;
              else if (m3(4,"p","h","a")) src_val = 8'd2;
              else if (m3(4,"c","h","o")) src_val = 8'd3;
              else if (m3(4,"d","l","y")) src_val = 8'd4;
              else if (m3(4,"t","u","b")) src_val = 8'd5;
              else if (m3(4,"f","l","n")) src_val = 8'd6;
              else begin src_val = '0; src_ok = 0; end

              if (cmd_buf[7] != CHAR_SPC)
                dst_ok = 0;
              else begin
                dst_ok = 1;
                if      (m3(8,"d","a","c")) dst_addr = ADDR_RTE_BASE + 8'd0;
                else if (m3(8,"t","r","m")) dst_addr = ADDR_RTE_BASE + 8'd1;
                else if (m3(8,"p","h","a")) dst_addr = ADDR_RTE_BASE + 8'd2;
                else if (m3(8,"c","h","o")) dst_addr = ADDR_RTE_BASE + 8'd3;
                else if (m3(8,"d","l","y")) dst_addr = ADDR_RTE_BASE + 8'd4;
                else if (m3(8,"t","u","b")) dst_addr = ADDR_RTE_BASE + 8'd5;
                else if (m3(8,"f","l","n")) dst_addr = ADDR_RTE_BASE + 8'd6;
                else begin dst_addr = '0; dst_ok = 0; end
              end

              if (src_ok && dst_ok) begin
                wr_addr_reg <= dst_addr;
                wr_data_reg <= src_val;
                wr_en_reg   <= 1'b1;
                update_shadow(dst_addr, src_val);
                send_prompt();
              end else begin
                send_err();
              end
            end

          end else if (m3(0, "s","e","t") && cmd_buf[3] == CHAR_SPC) begin

            if (m3(4, "t","r","m") && cmd_buf[7] == CHAR_SPC) begin
              if      (m3(8,"r","a","t")) begin target_addr <= ADDR_TRM_RAT; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"d","e","p")) begin target_addr <= ADDR_TRM_DEP; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"s","h","p")) begin target_addr <= ADDR_TRM_SHP; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"o","n"," ") || (cmd_buf[8]=="o" && cmd_buf[9]=="n" && cmd_buf[10]==8'h00)) begin
                wr_addr_reg <= ADDR_BYP_TRM; wr_data_reg <= 8'h00; wr_en_reg <= 1'b1;
                update_shadow(ADDR_BYP_TRM, 8'h00); send_prompt();
              end
              else if (m3(8,"o","f","f")) begin
                wr_addr_reg <= ADDR_BYP_TRM; wr_data_reg <= 8'h01; wr_en_reg <= 1'b1;
                update_shadow(ADDR_BYP_TRM, 8'h01); send_prompt();
              end
              else                             send_err();

            end else if (m3(4, "p","h","a") && cmd_buf[7] == CHAR_SPC) begin
              if      (m3(8,"s","p","d")) begin target_addr <= ADDR_PHA_SPD; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"f","b","n")) begin target_addr <= ADDR_PHA_FBN; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"o","n"," ") || (cmd_buf[8]=="o" && cmd_buf[9]=="n" && cmd_buf[10]==8'h00)) begin
                wr_addr_reg <= ADDR_BYP_PHA; wr_data_reg <= 8'h00; wr_en_reg <= 1'b1;
                update_shadow(ADDR_BYP_PHA, 8'h00); send_prompt();
              end
              else if (m3(8,"o","f","f")) begin
                wr_addr_reg <= ADDR_BYP_PHA; wr_data_reg <= 8'h01; wr_en_reg <= 1'b1;
                update_shadow(ADDR_BYP_PHA, 8'h01); send_prompt();
              end
              else                             send_err();

            end else if (m3(4, "c","h","o") && cmd_buf[7] == CHAR_SPC) begin
              if      (m3(8,"r","a","t")) begin target_addr <= ADDR_CHO_RAT; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"d","e","p")) begin target_addr <= ADDR_CHO_DEP; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"e","f","x")) begin target_addr <= ADDR_CHO_EFX; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"e","q","h")) begin target_addr <= ADDR_CHO_EQH; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"e","q","l")) begin target_addr <= ADDR_CHO_EQL; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"o","n"," ") || (cmd_buf[8]=="o" && cmd_buf[9]=="n" && cmd_buf[10]==8'h00)) begin
                wr_addr_reg <= ADDR_BYP_CHO; wr_data_reg <= 8'h00; wr_en_reg <= 1'b1;
                update_shadow(ADDR_BYP_CHO, 8'h00); send_prompt();
              end
              else if (m3(8,"o","f","f")) begin
                wr_addr_reg <= ADDR_BYP_CHO; wr_data_reg <= 8'h01; wr_en_reg <= 1'b1;
                update_shadow(ADDR_BYP_CHO, 8'h01); send_prompt();
              end
              else                             send_err();

            end else if (m3(4, "d","l","y") && cmd_buf[7] == CHAR_SPC) begin
              if      (m3(8,"t","o","n")) begin target_addr <= ADDR_DLY_TON; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"l","v","l")) begin target_addr <= ADDR_DLY_LVL; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"f","d","b")) begin target_addr <= ADDR_DLY_FDB; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"t","i","m")) begin
                target_addr    <= ADDR_DLY_TIM;
                parse_ptr      <= 12;
                target_is_time <= 1;
                state          <= S_PARSE_INIT;
              end
              else if (m3(8,"o","n"," ") || (cmd_buf[8]=="o" && cmd_buf[9]=="n" && cmd_buf[10]==8'h00)) begin
                wr_addr_reg <= ADDR_BYP_DLY; wr_data_reg <= 8'h00; wr_en_reg <= 1'b1;
                update_shadow(ADDR_BYP_DLY, 8'h00); send_prompt();
              end
              else if (m3(8,"o","f","f")) begin
                wr_addr_reg <= ADDR_BYP_DLY; wr_data_reg <= 8'h01; wr_en_reg <= 1'b1;
                update_shadow(ADDR_BYP_DLY, 8'h01); send_prompt();
              end
              else                             send_err();

            end else if (m3(4, "t","u","b") && cmd_buf[7] == CHAR_SPC) begin
              // -- tube distortion --
              if      (m3(8,"g","a","i")) begin target_addr <= ADDR_TUB_GAI; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"b","a","s")) begin target_addr <= ADDR_TUB_BAS; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"m","i","d")) begin target_addr <= ADDR_TUB_MID; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"t","r","e")) begin target_addr <= ADDR_TUB_TRE; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"l","v","l")) begin target_addr <= ADDR_TUB_LVL; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"o","n"," ") || (cmd_buf[8]=="o" && cmd_buf[9]=="n" && cmd_buf[10]==8'h00)) begin
                wr_addr_reg <= ADDR_BYP_TUB; wr_data_reg <= 8'h00; wr_en_reg <= 1'b1;
                update_shadow(ADDR_BYP_TUB, 8'h00); send_prompt();
              end
              else if (m3(8,"o","f","f")) begin
                wr_addr_reg <= ADDR_BYP_TUB; wr_data_reg <= 8'h01; wr_en_reg <= 1'b1;
                update_shadow(ADDR_BYP_TUB, 8'h01); send_prompt();
              end
              else                             send_err();

            end else if (m3(4, "f","l","n") && cmd_buf[7] == CHAR_SPC) begin
              // -- flanger --
              if      (m3(8,"m","a","n")) begin target_addr <= ADDR_FLN_MAN; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"w","i","d")) begin target_addr <= ADDR_FLN_WID; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"s","p","d")) begin target_addr <= ADDR_FLN_SPD; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"r","e","g")) begin target_addr <= ADDR_FLN_REG; parse_ptr <= 12; state <= S_PARSE_INIT; end
              else if (m3(8,"o","n"," ") || (cmd_buf[8]=="o" && cmd_buf[9]=="n" && cmd_buf[10]==8'h00)) begin
                wr_addr_reg <= ADDR_BYP_FLN; wr_data_reg <= 8'h00; wr_en_reg <= 1'b1;
                update_shadow(ADDR_BYP_FLN, 8'h00); send_prompt();
              end
              else if (m3(8,"o","f","f")) begin
                wr_addr_reg <= ADDR_BYP_FLN; wr_data_reg <= 8'h01; wr_en_reg <= 1'b1;
                update_shadow(ADDR_BYP_FLN, 8'h01); send_prompt();
              end
              else                             send_err();

            end else begin
              send_err();
            end

          end else begin
            send_err();
          end
        end

        // ============================================================
        // Status display  (chain: ROUTE → TRM → PHA → CHO → DLY → TUB → FLN → PROMPT)
        // ============================================================

        // -- Routes --
        // Shows actual routing table: each port's source
        // Format: DAC<TRM TRM<ADC PHA<TRM CHO<PHA DLY<CHO TUB<FLN FLN<ADC
        S_ST_ROUTE: begin
          automatic int p = 0;
          automatic int j;

          // Port names: 0=DAC/ADC, 1=TRM, 2=PHA, 3=CHO, 4=DLY, 5=TUB, 6=FLN
          for (j = 0; j < 7; j++) begin
            // Destination (sink) name
            case (j)
              0: begin tx_buf[p]<="D";p++;tx_buf[p]<="A";p++;tx_buf[p]<="C";p++; end
              1: begin tx_buf[p]<="T";p++;tx_buf[p]<="R";p++;tx_buf[p]<="M";p++; end
              2: begin tx_buf[p]<="P";p++;tx_buf[p]<="H";p++;tx_buf[p]<="A";p++; end
              3: begin tx_buf[p]<="C";p++;tx_buf[p]<="H";p++;tx_buf[p]<="O";p++; end
              4: begin tx_buf[p]<="D";p++;tx_buf[p]<="L";p++;tx_buf[p]<="Y";p++; end
              5: begin tx_buf[p]<="T";p++;tx_buf[p]<="U";p++;tx_buf[p]<="B";p++; end
              6: begin tx_buf[p]<="F";p++;tx_buf[p]<="L";p++;tx_buf[p]<="N";p++; end
              default: ;
            endcase
            tx_buf[p] <= "<"; p++;
            // Source (master) name from shadow_route
            emit_port_src(shadow_route[j][2:0], p);
            if (j < 6) begin
              tx_buf[p] <= " "; p++;
            end
          end
          st_nl(p);

          tx_len       <= 6'(p);
          tx_idx       <= '0;
          return_state <= S_ST_TRM;
          state        <= S_TX_START;
        end

        // -- Tremolo (slot 0) --
        S_ST_TRM: begin
          if (!shadow[7][0]) begin
            automatic int p = 0;
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
          if (!shadow[15][0]) begin
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
          if (!shadow[23][0]) begin
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
          if (!shadow[31][0]) begin
            automatic int p = 0;
            tx_buf[p] <= "D"; p++; tx_buf[p] <= "L"; p++; tx_buf[p] <= "Y"; p++;
            tx_buf[p] <= " "; p++;
            emit_param("t","o","n", shadow[8'h18], p);
            emit_param("l","v","l", shadow[8'h19], p);
            emit_param("f","d","b", shadow[8'h1A], p);
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
            return_state <= S_ST_TUB;
            state        <= S_TX_START;
          end else begin
            state <= S_ST_TUB;
          end
        end

        // -- Tube distortion (slot 4) --
        S_ST_TUB: begin
          if (!shadow[39][0]) begin
            automatic int p = 0;
            tx_buf[p] <= "T"; p++; tx_buf[p] <= "U"; p++; tx_buf[p] <= "B"; p++;
            tx_buf[p] <= " "; p++;
            emit_param("g","a","i", shadow[8'h20], p);
            emit_param("b","a","s", shadow[8'h21], p);
            emit_param("m","i","d", shadow[8'h22], p);
            emit_param("t","r","e", shadow[8'h23], p);
            emit_param("l","v","l", shadow[8'h24], p);
            st_nl(p);
            tx_len       <= 6'(p);
            tx_idx       <= '0;
            return_state <= S_ST_FLN;
            state        <= S_TX_START;
          end else begin
            state <= S_ST_FLN;
          end
        end

        // -- Flanger (slot 5) --
        S_ST_FLN: begin
          if (!shadow[47][0]) begin
            automatic int p = 0;
            tx_buf[p] <= "F"; p++; tx_buf[p] <= "L"; p++; tx_buf[p] <= "N"; p++;
            tx_buf[p] <= " "; p++;
            emit_param("m","a","n", shadow[8'h28], p);
            emit_param("w","i","d", shadow[8'h29], p);
            emit_param("s","p","d", shadow[8'h2A], p);
            emit_param("r","e","g", shadow[8'h2B], p);
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
            parse_ptr <= parse_ptr + 1;

          end else if (nibble_valid) begin
            parse_acc <= {parse_acc[27:0], nibble};
            parse_ptr <= parse_ptr + 1;

          end else begin
            if (target_addr == ADDR_RAW_SENTINEL) begin
              wr_addr_reg <= parse_acc[15:8];
              wr_data_reg <= parse_acc[7:0];
              wr_en_reg   <= 1'b1;
              update_shadow(parse_acc[15:8], parse_acc[7:0]);
              send_prompt();

            end else if (target_is_time) begin
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
        // ============================================================
        S_WRITE_TIME: begin
          if (cdc_wait != '0) begin
            cdc_wait <= cdc_wait - 1;

          end else if (!wr_en_reg) begin
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

        S_TX_START: begin
          if (tx_idx < tx_len) begin
            if (!tx_busy) begin
              tx_byte  <= tx_buf[tx_idx];
              tx_start <= 1'b1;
              state    <= S_TX_WAIT;
            end
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

        default: begin
          state <= S_IDLE;
        end

      endcase
    end
  end

endmodule