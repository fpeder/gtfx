// =============================================================================
// cmd_proc.sv  -  Serial CLI command processor
//
// Conservative refactor of cmd_proc_v2:
//   - Module renamed: cmd_proc_v2 → cmd_proc
//   - Prompt changed to " gtfx>"
//   - set_param() / do_bypass() / is_on() / is_off() helper tasks/functions
//     eliminate the repetitive on/off/parse dispatch per effect
//   - Clearer section organisation with region comments
//   - All original FSM logic, timing, and state transitions preserved exactly
// =============================================================================

module cmd_proc (
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
  // ctrl_bus Address Map  (unchanged from v2 - see original header for details)
  // ===========================================================================

  // ─── ASCII constants ─────────────────────────────────────────────────────
  localparam logic [7:0] CHAR_CR  = 8'h0D;
  localparam logic [7:0] CHAR_LF  = 8'h0A;
  localparam logic [7:0] CHAR_BS  = 8'h08;
  localparam logic [7:0] CHAR_DEL = 8'h7F;
  localparam logic [7:0] CHAR_SPC = 8'h20;
  localparam logic [7:0] CHAR_0   = 8'h30;
  localparam logic [7:0] CHAR_A   = 8'h41;

  // ─── Register address map ────────────────────────────────────────────────
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
  localparam logic [7:0] ADDR_DLY_RPT = 8'h18;
  localparam logic [7:0] ADDR_DLY_MIX = 8'h19;
  localparam logic [7:0] ADDR_DLY_FLT = 8'h1A;
  localparam logic [7:0] ADDR_DLY_TIM = 8'h1B;
  localparam logic [7:0] ADDR_DLY_MOD = 8'h1D;
  localparam logic [7:0] ADDR_DLY_GRT = 8'h1E;
  localparam logic [7:0] ADDR_FLN_MAN = 8'h20;
  localparam logic [7:0] ADDR_FLN_WID = 8'h21;
  localparam logic [7:0] ADDR_FLN_SPD = 8'h22;
  localparam logic [7:0] ADDR_FLN_REG = 8'h23;
  localparam logic [7:0] ADDR_FLN_MIX = 8'h24;
  localparam logic [7:0] ADDR_REV_DEC = 8'h28;
  localparam logic [7:0] ADDR_REV_DMP = 8'h29;
  localparam logic [7:0] ADDR_REV_MIX = 8'h2A;
  localparam logic [7:0] ADDR_REV_PRE = 8'h2B;
  localparam logic [7:0] ADDR_REV_TON = 8'h2C;
  localparam logic [7:0] ADDR_REV_LVL = 8'h2D;
  localparam logic [7:0] ADDR_CMP_THR = 8'h30;
  localparam logic [7:0] ADDR_CMP_RAT = 8'h31;
  localparam logic [7:0] ADDR_CMP_ATK = 8'h32;
  localparam logic [7:0] ADDR_CMP_REL = 8'h33;
  localparam logic [7:0] ADDR_CMP_MAK = 8'h34;
  localparam logic [7:0] ADDR_WAH_FRQ = 8'h38;
  localparam logic [7:0] ADDR_WAH_RES = 8'h39;
  localparam logic [7:0] ADDR_WAH_DPT = 8'h3A;
  localparam logic [7:0] ADDR_WAH_MOD = 8'h3B;
  localparam logic [7:0] ADDR_WAH_MIX = 8'h3C;
  localparam logic [7:0] ADDR_RTE_BASE = 8'h60;

  localparam logic [7:0] ADDR_BYP_TRM = 8'h07;
  localparam logic [7:0] ADDR_BYP_PHA = 8'h0F;
  localparam logic [7:0] ADDR_BYP_CHO = 8'h17;
  localparam logic [7:0] ADDR_BYP_DLY = 8'h1F;
  localparam logic [7:0] ADDR_BYP_FLN = 8'h27;
  localparam logic [7:0] ADDR_BYP_REV = 8'h2F;
  localparam logic [7:0] ADDR_BYP_CMP = 8'h37;
  localparam logic [7:0] ADDR_BYP_WAH = 8'h3F;
  localparam logic [7:0] ADDR_RAW_SENTINEL = 8'hFF;

  localparam int CDC_WAIT_CYCLES = 31;

  // ─── Canned messages ─────────────────────────────────────────────────────
  localparam logic [7:0] MSG_PROMPT [0:5] = '{"g","t","f","x",">", " "};
  localparam int         LEN_PROMPT = 6;
  localparam logic [7:0] MSG_ERR [0:4]    = '{"E","r","r", CHAR_CR, CHAR_LF};
  localparam int         LEN_ERR  = 5;

  // ─── State encoding ──────────────────────────────────────────────────────
  typedef enum logic [5:0] {
    S_BOOT,
    S_INIT_WRITES,
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
    S_ST_FLN,
    S_ST_REV,
    S_ST_CMP,
    S_ST_WAH,
    S_TX_START,
    S_TX_WAIT,
    S_LOAD_PROMPT,
    S_LOAD_STRING,
    S_CHAIN_PARSE,
    S_CHAIN_WRITE
  } state_t;

  state_t state        = S_BOOT;
  state_t return_state = S_IDLE;

  // ─── Message selector ────────────────────────────────────────────────────
  typedef enum logic {
    MSG_SEL_PROMPT = 1'd0,
    MSG_SEL_ERR    = 1'd1
  } msg_sel_t;

  msg_sel_t tx_msg_sel = MSG_SEL_PROMPT;

  // ─── Buffers and indices ─────────────────────────────────────────────────
  logic [7:0] cmd_buf [0:47];
  logic [5:0] cmd_idx   = '0;

  logic [7:0] tx_buf [0:99];
  logic [6:0] tx_len    = '0;
  logic [6:0] tx_idx    = '0;

  // ─── Parser state ────────────────────────────────────────────────────────
  logic [31:0] parse_acc      = '0;
  logic [ 5:0] parse_ptr      = '0;
  logic [ 7:0] target_addr    = '0;
  logic        target_is_time = '0;

  // ─── RX latch ────────────────────────────────────────────────────────────
  logic        rx_pending = '0;
  logic [7:0]  rx_latch   = '0;

  // ─── Multi-byte write helpers ────────────────────────────────────────────
  logic        time_wr_idx = '0;
  logic [ 4:0] cdc_wait    = '0;

  // ─── Chain command state ─────────────────────────────────────────────
  logic [3:0] chain_ports [0:8];   // parsed port numbers (up to 9 ports)
  logic [3:0] chain_count;         // how many effects in chain
  logic [4:0] chain_step;          // write step counter (2 phases per write)
  logic [7:0] chain_enabled;       // 1 bit per slot: slot is in chain

  // ─── Shadow registers ────────────────────────────────────────────────────
  logic [7:0] shadow       [0:63];  // 8 slots × 8 regs = 64
  logic [7:0] shadow_route [0:8];   // 9 ports

  logic [7:0] init_idx;

  // ─── Write bus ───────────────────────────────────────────────────────────
  logic        wr_en_reg   = '0;
  logic [7:0]  wr_addr_reg = '0;
  logic [7:0]  wr_data_reg = '0;
  assign wr_en   = wr_en_reg;
  assign wr_addr = wr_addr_reg;
  assign wr_data = wr_data_reg;

  // =========================================================================
  // Helper functions
  // =========================================================================

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

  function automatic logic [3:0] hex_nibble(
    input  logic [7:0] c,
    output logic       valid
  );
    if      (c >= "0" && c <= "9") begin valid = 1; return c - "0";      end
    else if (c >= "A" && c <= "F") begin valid = 1; return c - "A" + 10; end
    else if (c >= "a" && c <= "f") begin valid = 1; return c - "a" + 10; end
    else                           begin valid = 0; return 4'd0;          end
  endfunction

  // ── tag → port mapping ──
  function automatic logic [3:0] tag_to_port(
    input logic [7:0] c0, c1, c2
  );
    if      (c0=="a" && c1=="d" && c2=="c") return 4'd0;
    else if (c0=="t" && c1=="r" && c2=="m") return 4'd1;
    else if (c0=="p" && c1=="h" && c2=="a") return 4'd2;
    else if (c0=="c" && c1=="h" && c2=="o") return 4'd3;
    else if (c0=="d" && c1=="l" && c2=="y") return 4'd4;
    else if (c0=="f" && c1=="l" && c2=="n") return 4'd5;
    else if (c0=="r" && c1=="e" && c2=="v") return 4'd6;
    else if (c0=="c" && c1=="m" && c2=="p") return 4'd7;
    else if (c0=="w" && c1=="a" && c2=="h") return 4'd8;
    else                                     return 4'd15; // invalid
  endfunction

  // ── on/off detection at cmd_buf position 8 ──
  function automatic logic is_on();
    return (m3(8,"o","n"," ") || (cmd_buf[8]=="o" && cmd_buf[9]=="n" && cmd_buf[10]==8'h00));
  endfunction

  function automatic logic is_off();
    return m3(8,"o","f","f");
  endfunction

  // =========================================================================
  // tx_buf builder tasks
  // =========================================================================

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

  task automatic emit_port_src(input logic [3:0] src, inout int p);
    case (src)
      4'd0:  begin tx_buf[p] <= "A"; p++; tx_buf[p] <= "D"; p++; tx_buf[p] <= "C"; p++; end
      4'd1:  begin tx_buf[p] <= "T"; p++; tx_buf[p] <= "R"; p++; tx_buf[p] <= "M"; p++; end
      4'd2:  begin tx_buf[p] <= "P"; p++; tx_buf[p] <= "H"; p++; tx_buf[p] <= "A"; p++; end
      4'd3:  begin tx_buf[p] <= "C"; p++; tx_buf[p] <= "H"; p++; tx_buf[p] <= "O"; p++; end
      4'd4:  begin tx_buf[p] <= "D"; p++; tx_buf[p] <= "L"; p++; tx_buf[p] <= "Y"; p++; end
      4'd5:  begin tx_buf[p] <= "F"; p++; tx_buf[p] <= "L"; p++; tx_buf[p] <= "N"; p++; end
      4'd6:  begin tx_buf[p] <= "R"; p++; tx_buf[p] <= "E"; p++; tx_buf[p] <= "V"; p++; end
      4'd7:  begin tx_buf[p] <= "C"; p++; tx_buf[p] <= "M"; p++; tx_buf[p] <= "P"; p++; end
      4'd8:  begin tx_buf[p] <= "W"; p++; tx_buf[p] <= "A"; p++; tx_buf[p] <= "H"; p++; end
      default: begin tx_buf[p] <= "?"; p++; tx_buf[p] <= "?"; p++; tx_buf[p] <= "?"; p++; end
    endcase
  endtask

  task automatic emit_disabled(
    input logic [7:0] c0, c1, c2,
    inout int         p
  );
    tx_buf[p] <= "-"; p++; tx_buf[p] <= "-"; p++; tx_buf[p] <= "-"; p++;
    tx_buf[p] <= " "; p++;
    tx_buf[p] <= c0; p++; tx_buf[p] <= c1; p++; tx_buf[p] <= c2; p++;
    st_nl(p);
  endtask

  // =========================================================================
  // Shadow update helper
  // =========================================================================

  task automatic update_shadow(input logic [7:0] addr, input logic [7:0] data);
    if (addr < 8'h40)
      shadow[addr[5:0]] <= data;
    else if (addr >= ADDR_RTE_BASE && addr < ADDR_RTE_BASE + 8'd9)
      shadow_route[addr[3:0]] <= data;
  endtask

  // =========================================================================
  // Message / prompt / error tasks
  // =========================================================================

  task automatic send_msg(
    input msg_sel_t  sel,
    input int        len,
    input state_t    ret
  );
    tx_msg_sel   <= sel;
    tx_len       <= 7'(len);
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

  // =========================================================================
  // Command dispatch helpers  (reduce per-effect copy-paste)
  // =========================================================================

  // Start hex parse for a parameter register
  task automatic set_param(input logic [7:0] addr);
    target_addr <= addr;
    parse_ptr   <= 12;
    state       <= S_PARSE_INIT;
  endtask

  // Write bypass register directly (on=0x00, off=0x01)
  task automatic do_bypass(input logic [7:0] byp_addr, input logic [7:0] val);
    wr_addr_reg <= byp_addr;
    wr_data_reg <= val;
    wr_en_reg   <= 1'b1;
    update_shadow(byp_addr, val);
    send_prompt();
  endtask

  // =========================================================================
  // Main always_ff block
  // =========================================================================
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      state        <= S_BOOT;
      cmd_idx      <= '0;
      tx_start     <= '0;
      rx_pending   <= '0;
      wr_en_reg    <= '0;

      for (int i = 0; i < 64; i++) shadow[i] <= '0;

      // Slot 0 (tremolo)
      shadow[0]  <= 8'h3C;
      shadow[1]  <= 8'hB4;

      // Slot 1 (phaser)
      shadow[8]  <= 8'h50;

      // Slot 2 (chorus)
      shadow[16] <= 8'h50; shadow[17] <= 8'h64; shadow[18] <= 8'h80;
      shadow[19] <= 8'hC8; shadow[20] <= 8'h80;

      // Slot 3 (timeline delay)
      shadow[24] <= 8'h64; shadow[25] <= 8'h80; shadow[26] <= 8'hFF;
      shadow[27] <= 8'h00; shadow[28] <= 8'h30;
      shadow[29] <= 8'h00; shadow[30] <= 8'h00;

      // Slot 4 (flanger)
      shadow[32] <= 8'h40;   // manual
      shadow[33] <= 8'h80;   // width
      shadow[34] <= 8'h30;   // speed
      shadow[35] <= 8'h90;   // regen
      shadow[36] <= 8'h60;   // mix

      // Slot 5 (reverb)
      shadow[40] <= 8'h80;   // decay
      shadow[41] <= 8'h60;   // damping
      shadow[42] <= 8'h60;   // mix
      shadow[43] <= 8'h20;   // pre-delay
      shadow[44] <= 8'h80;   // tone
      shadow[45] <= 8'h80;   // level

      // Slot 6 (compressor)
      shadow[48] <= 8'h60;   // threshold
      shadow[49] <= 8'h40;   // ratio
      shadow[50] <= 8'h20;   // attack
      shadow[51] <= 8'h40;   // release
      shadow[52] <= 8'h40;   // makeup

      // Slot 7 (wah)
      shadow[56] <= 8'h80;   // freq
      shadow[57] <= 8'h60;   // resonance
      shadow[58] <= 8'hFF;   // depth
      shadow[59] <= 8'h00;   // mode
      shadow[60] <= 8'hFF;   // mix
      shadow[61] <= 8'h60;   // decay

      // Bypass bits - all bypassed at boot
      shadow[7]  <= 8'h01;
      shadow[15] <= 8'h01;
      shadow[23] <= 8'h01;
      shadow[31] <= 8'h01;
      shadow[39] <= 8'h01;
      shadow[47] <= 8'h01;
      shadow[55] <= 8'h01;
      shadow[63] <= 8'h01;

      // Default route: ADC→WAH→CMP→PHA→FLN→CHO→TRM→DLY→REV→DAC
      shadow_route[0] <= 8'd6;    // DAC ← REV
      shadow_route[1] <= 8'd3;    // TRM ← CHO
      shadow_route[2] <= 8'd7;    // PHA ← CMP
      shadow_route[3] <= 8'd5;    // CHO ← FLN
      shadow_route[4] <= 8'd1;    // DLY ← TRM
      shadow_route[5] <= 8'd2;    // FLN ← PHA
      shadow_route[6] <= 8'd4;    // REV ← DLY
      shadow_route[7] <= 8'd8;    // CMP ← WAH
      shadow_route[8] <= 8'd0;    // WAH ← ADC

      chain_enabled <= 8'hFF;  // all effects in chain at boot
      init_idx      <= 8'd0;

    end else begin
      wr_en_reg <= '0;

      if (rx_valid) begin
        rx_pending <= 1;
        rx_latch   <= rx_byte;
      end

      // ================================================================
      //  State machine
      // ================================================================
      case (state)

        // ──────────────────────────────────────────────────────────────
        //  Boot & init
        // ──────────────────────────────────────────────────────────────

        S_BOOT: begin
          tx_buf[0] <= CHAR_CR;
          tx_buf[1] <= CHAR_LF;
          tx_len    <= 2;
          tx_idx    <= '0;
          return_state <= S_INIT_WRITES;
          state <= S_TX_START;
        end

        S_INIT_WRITES: begin
          if (init_idx >= 8'd146) begin
            wr_en_reg <= 1'b0;
            state     <= S_LOAD_PROMPT;
          end else if (cdc_wait != '0) begin
            // Wait for toggle to propagate through audio-domain CDC
            cdc_wait <= cdc_wait - 1;
          end else begin
            if (init_idx[0] == 1'b0) begin
              wr_en_reg <= 1'b0;
              if (init_idx[7:1] <= 8'd63) begin
                wr_addr_reg <= 8'(init_idx[7:1]);
                wr_data_reg <= shadow[init_idx[7:1]];
              end else begin
                wr_addr_reg <= ADDR_RTE_BASE + 8'(init_idx[7:1] - 8'd64);
                wr_data_reg <= shadow_route[4'(init_idx[7:1] - 8'd64)];
              end
            end else begin
              wr_en_reg <= 1'b1;
              cdc_wait  <= 5'(CDC_WAIT_CYCLES);
            end
            init_idx <= init_idx + 8'd1;
          end
        end

        // ──────────────────────────────────────────────────────────────
        //  Idle & RX character handling
        // ──────────────────────────────────────────────────────────────

        S_IDLE: begin
          tx_start <= '0;
          if (rx_pending) begin
            rx_pending <= '0;
            state <= S_RX_CHAR;
          end
        end

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

          end else if (cmd_idx < 47) begin
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

        // ──────────────────────────────────────────────────────────────
        //  Command dispatch
        // ──────────────────────────────────────────────────────────────

        S_CHECK_CMD: begin
          cmd_idx        <= '0;
          target_is_time <= '0;

          // ── "wr <addr><data>" ────────────────────────────────────
          if (m3(0, "w","r"," ")) begin
            parse_ptr   <= 3;
            target_addr <= ADDR_RAW_SENTINEL;
            state       <= S_PARSE_INIT;

          // ── "con <src> to <snk>" ─────────────────────────────────
          end else if (m3(0, "c","o","n") && cmd_buf[3] == CHAR_SPC) begin
            begin
              logic [7:0] snk_addr;
              logic [7:0] src_val;
              logic       snk_ok, src_ok;

              src_ok = 1;
              if      (m3(4,"a","d","c")) src_val = 8'd0;
              else if (m3(4,"t","r","m")) src_val = 8'd1;
              else if (m3(4,"p","h","a")) src_val = 8'd2;
              else if (m3(4,"c","h","o")) src_val = 8'd3;
              else if (m3(4,"d","l","y")) src_val = 8'd4;
              else if (m3(4,"f","l","n")) src_val = 8'd5;
              else if (m3(4,"r","e","v")) src_val = 8'd6;
              else if (m3(4,"c","m","p")) src_val = 8'd7;
              else if (m3(4,"w","a","h")) src_val = 8'd8;
              else begin src_val = '0; src_ok = 0; end

              if (cmd_buf[7] != CHAR_SPC || cmd_buf[8] != "t" || cmd_buf[9] != "o" || cmd_buf[10] != CHAR_SPC)
                snk_ok = 0;
              else begin
                snk_ok = 1;
                if      (m3(11,"d","a","c")) snk_addr = ADDR_RTE_BASE + 8'd0;
                else if (m3(11,"t","r","m")) snk_addr = ADDR_RTE_BASE + 8'd1;
                else if (m3(11,"p","h","a")) snk_addr = ADDR_RTE_BASE + 8'd2;
                else if (m3(11,"c","h","o")) snk_addr = ADDR_RTE_BASE + 8'd3;
                else if (m3(11,"d","l","y")) snk_addr = ADDR_RTE_BASE + 8'd4;
                else if (m3(11,"f","l","n")) snk_addr = ADDR_RTE_BASE + 8'd5;
                else if (m3(11,"r","e","v")) snk_addr = ADDR_RTE_BASE + 8'd6;
                else if (m3(11,"c","m","p")) snk_addr = ADDR_RTE_BASE + 8'd7;
                else if (m3(11,"w","a","h")) snk_addr = ADDR_RTE_BASE + 8'd8;
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

          // ── "set <efx> ..." ──────────────────────────────────────
          end else if (m3(0, "s","e","t") && cmd_buf[3] == CHAR_SPC) begin

            // ── tremolo ──
            if (m3(4, "t","r","m") && cmd_buf[7] == CHAR_SPC) begin
              if      (m3(8,"r","a","t")) set_param(ADDR_TRM_RAT);
              else if (m3(8,"d","e","p")) set_param(ADDR_TRM_DEP);
              else if (m3(8,"s","h","p")) set_param(ADDR_TRM_SHP);
              else if (is_on())           do_bypass(ADDR_BYP_TRM, 8'h00);
              else if (is_off())          do_bypass(ADDR_BYP_TRM, 8'h01);
              else                        send_err();

            // ── phaser ──
            end else if (m3(4, "p","h","a") && cmd_buf[7] == CHAR_SPC) begin
              if      (m3(8,"s","p","d")) set_param(ADDR_PHA_SPD);
              else if (m3(8,"f","b","n")) set_param(ADDR_PHA_FBN);
              else if (is_on())           do_bypass(ADDR_BYP_PHA, 8'h00);
              else if (is_off())          do_bypass(ADDR_BYP_PHA, 8'h01);
              else                        send_err();

            // ── chorus ──
            end else if (m3(4, "c","h","o") && cmd_buf[7] == CHAR_SPC) begin
              if      (m3(8,"r","a","t")) set_param(ADDR_CHO_RAT);
              else if (m3(8,"d","e","p")) set_param(ADDR_CHO_DEP);
              else if (m3(8,"e","f","x")) set_param(ADDR_CHO_EFX);
              else if (m3(8,"e","q","h")) set_param(ADDR_CHO_EQH);
              else if (m3(8,"e","q","l")) set_param(ADDR_CHO_EQL);
              else if (is_on())           do_bypass(ADDR_BYP_CHO, 8'h00);
              else if (is_off())          do_bypass(ADDR_BYP_CHO, 8'h01);
              else                        send_err();

            // ── delay ──
            end else if (m3(4, "d","l","y") && cmd_buf[7] == CHAR_SPC) begin
              if      (m3(8,"r","p","t")) set_param(ADDR_DLY_RPT);
              else if (m3(8,"m","i","x")) set_param(ADDR_DLY_MIX);
              else if (m3(8,"f","l","t")) set_param(ADDR_DLY_FLT);
              else if (m3(8,"t","i","m")) begin
                target_addr    <= ADDR_DLY_TIM;
                parse_ptr      <= 12;
                target_is_time <= 1;
                state          <= S_PARSE_INIT;
              end
              else if (m3(8,"m","o","d")) set_param(ADDR_DLY_MOD);
              else if (m3(8,"g","r","t")) set_param(ADDR_DLY_GRT);
              else if (is_on())           do_bypass(ADDR_BYP_DLY, 8'h00);
              else if (is_off())          do_bypass(ADDR_BYP_DLY, 8'h01);
              else                        send_err();

            // ── flanger ──
            end else if (m3(4, "f","l","n") && cmd_buf[7] == CHAR_SPC) begin
              if      (m3(8,"m","a","n")) set_param(ADDR_FLN_MAN);
              else if (m3(8,"w","i","d")) set_param(ADDR_FLN_WID);
              else if (m3(8,"s","p","d")) set_param(ADDR_FLN_SPD);
              else if (m3(8,"r","e","g")) set_param(ADDR_FLN_REG);
              else if (m3(8,"m","i","x")) set_param(ADDR_FLN_MIX);
              else if (is_on())           do_bypass(ADDR_BYP_FLN, 8'h00);
              else if (is_off())          do_bypass(ADDR_BYP_FLN, 8'h01);
              else                        send_err();

            // ── reverb ──
            end else if (m3(4, "r","e","v") && cmd_buf[7] == CHAR_SPC) begin
              if      (m3(8,"d","e","c")) set_param(ADDR_REV_DEC);
              else if (m3(8,"d","m","p")) set_param(ADDR_REV_DMP);
              else if (m3(8,"m","i","x")) set_param(ADDR_REV_MIX);
              else if (m3(8,"p","r","e")) set_param(ADDR_REV_PRE);
              else if (m3(8,"t","o","n")) set_param(ADDR_REV_TON);
              else if (m3(8,"l","v","l")) set_param(ADDR_REV_LVL);
              else if (is_on())           do_bypass(ADDR_BYP_REV, 8'h00);
              else if (is_off())          do_bypass(ADDR_BYP_REV, 8'h01);
              else                        send_err();

            // ── compressor ──
            end else if (m3(4, "c","m","p") && cmd_buf[7] == CHAR_SPC) begin
              if      (m3(8,"t","h","r")) set_param(ADDR_CMP_THR);
              else if (m3(8,"r","a","t")) set_param(ADDR_CMP_RAT);
              else if (m3(8,"a","t","k")) set_param(ADDR_CMP_ATK);
              else if (m3(8,"r","e","l")) set_param(ADDR_CMP_REL);
              else if (m3(8,"m","a","k")) set_param(ADDR_CMP_MAK);
              else if (is_on())           do_bypass(ADDR_BYP_CMP, 8'h00);
              else if (is_off())          do_bypass(ADDR_BYP_CMP, 8'h01);
              else                        send_err();

            // ── wah ──
            end else if (m3(4, "w","a","h") && cmd_buf[7] == CHAR_SPC) begin
              if      (m3(8,"f","r","q")) set_param(ADDR_WAH_FRQ);
              else if (m3(8,"r","e","s")) set_param(ADDR_WAH_RES);
              else if (m3(8,"d","p","t")) set_param(ADDR_WAH_DPT);
              else if (m3(8,"m","o","d")) set_param(ADDR_WAH_MOD);
              else if (m3(8,"m","i","x")) set_param(ADDR_WAH_MIX);
              else if (is_on())           do_bypass(ADDR_BYP_WAH, 8'h00);
              else if (is_off())          do_bypass(ADDR_BYP_WAH, 8'h01);
              else                        send_err();

            end else begin
              send_err();
            end

          // ── "chain <tag> ..." ─────────────────────────────────────
          end else if (m3(0, "c","h","a") && m3(3, "i","n"," ")) begin
            chain_count   <= 4'd0;
            chain_enabled <= '0;
            state         <= S_CHAIN_PARSE;

          end else begin
            send_err();
          end
        end


        S_ST_ROUTE: begin
          automatic int p = 0;
          tx_buf[p]<="D";p++;tx_buf[p]<="A";p++;tx_buf[p]<="C";p++;
          tx_buf[p]<="<";p++;
          emit_port_src(shadow_route[0][3:0], p);
          tx_buf[p]<=" ";p++;
          tx_buf[p]<="T";p++;tx_buf[p]<="R";p++;tx_buf[p]<="M";p++;
          tx_buf[p]<="<";p++;
          emit_port_src(shadow_route[1][3:0], p);
          tx_buf[p]<=" ";p++;
          tx_buf[p]<="P";p++;tx_buf[p]<="H";p++;tx_buf[p]<="A";p++;
          tx_buf[p]<="<";p++;
          emit_port_src(shadow_route[2][3:0], p);
          tx_buf[p]<=" ";p++;
          tx_buf[p]<="C";p++;tx_buf[p]<="H";p++;tx_buf[p]<="O";p++;
          tx_buf[p]<="<";p++;
          emit_port_src(shadow_route[3][3:0], p);
          tx_buf[p]<=" ";p++;
          tx_buf[p]<="D";p++;tx_buf[p]<="L";p++;tx_buf[p]<="Y";p++;
          tx_buf[p]<="<";p++;
          emit_port_src(shadow_route[4][3:0], p);
          tx_buf[p]<=" ";p++;
          tx_buf[p]<="F";p++;tx_buf[p]<="L";p++;tx_buf[p]<="N";p++;
          tx_buf[p]<="<";p++;
          emit_port_src(shadow_route[5][3:0], p);
          tx_buf[p]<=" ";p++;
          tx_buf[p]<="R";p++;tx_buf[p]<="E";p++;tx_buf[p]<="V";p++;
          tx_buf[p]<="<";p++;
          emit_port_src(shadow_route[6][3:0], p);
          tx_buf[p]<=" ";p++;
          tx_buf[p]<="C";p++;tx_buf[p]<="M";p++;tx_buf[p]<="P";p++;
          tx_buf[p]<="<";p++;
          emit_port_src(shadow_route[7][3:0], p);
          tx_buf[p]<=" ";p++;
          tx_buf[p]<="W";p++;tx_buf[p]<="A";p++;tx_buf[p]<="H";p++;
          tx_buf[p]<="<";p++;
          emit_port_src(shadow_route[8][3:0], p);
          st_nl(p);

          tx_len       <= 7'(p);
          tx_idx       <= '0;
          return_state <= S_ST_TRM;
          state        <= S_TX_START;
        end

        // ── Tremolo (slot 0) ──
        S_ST_TRM: begin
          if (!chain_enabled[0]) begin
            automatic int p = 0;
            emit_disabled("T", "R", "M", p);
            tx_len <= 7'(p); tx_idx <= '0;
            return_state <= S_ST_PHA;
            state <= S_TX_START;
          end else if (!shadow[7][0]) begin
            automatic int p = 0;
            tx_buf[p] <= "T"; p++; tx_buf[p] <= "R"; p++; tx_buf[p] <= "M"; p++;
            tx_buf[p] <= " "; p++;
            emit_param("r","a","t", shadow[0], p);
            emit_param("d","e","p", shadow[1], p);
            emit_param("s","h","p", shadow[2], p);
            st_nl(p);
            tx_len       <= 7'(p);
            tx_idx       <= '0;
            return_state <= S_ST_PHA;
            state        <= S_TX_START;
          end else begin
            state <= S_ST_PHA;
          end
        end

        // ── Phaser (slot 1) ──
        S_ST_PHA: begin
          if (!chain_enabled[1]) begin
            automatic int p = 0;
            emit_disabled("P", "H", "A", p);
            tx_len <= 7'(p); tx_idx <= '0;
            return_state <= S_ST_CHO;
            state <= S_TX_START;
          end else if (!shadow[15][0]) begin
            automatic int p = 0;
            tx_buf[p] <= "P"; p++; tx_buf[p] <= "H"; p++; tx_buf[p] <= "A"; p++;
            tx_buf[p] <= " "; p++;
            emit_param("s","p","d", shadow[8], p);
            emit_param("f","b","n", shadow[9], p);
            st_nl(p);
            tx_len       <= 7'(p);
            tx_idx       <= '0;
            return_state <= S_ST_CHO;
            state        <= S_TX_START;
          end else begin
            state <= S_ST_CHO;
          end
        end

        // ── Chorus (slot 2) ──
        S_ST_CHO: begin
          if (!chain_enabled[2]) begin
            automatic int p = 0;
            emit_disabled("C", "H", "O", p);
            tx_len <= 7'(p); tx_idx <= '0;
            return_state <= S_ST_DLY;
            state <= S_TX_START;
          end else if (!shadow[23][0]) begin
            automatic int p = 0;
            tx_buf[p] <= "C"; p++; tx_buf[p] <= "H"; p++; tx_buf[p] <= "O"; p++;
            tx_buf[p] <= " "; p++;
            emit_param("r","a","t", shadow[8'h10], p);
            emit_param("d","e","p", shadow[8'h11], p);
            emit_param("e","f","x", shadow[8'h12], p);
            emit_param("e","q","h", shadow[8'h13], p);
            emit_param("e","q","l", shadow[8'h14], p);
            st_nl(p);
            tx_len       <= 7'(p);
            tx_idx       <= '0;
            return_state <= S_ST_DLY;
            state        <= S_TX_START;
          end else begin
            state <= S_ST_DLY;
          end
        end

        // ── Delay (slot 3) ──
        S_ST_DLY: begin
          if (!chain_enabled[3]) begin
            automatic int p = 0;
            emit_disabled("D", "L", "Y", p);
            tx_len <= 7'(p); tx_idx <= '0;
            return_state <= S_ST_FLN;
            state <= S_TX_START;
          end else if (!shadow[31][0]) begin
            automatic int p = 0;
            tx_buf[p] <= "D"; p++; tx_buf[p] <= "L"; p++; tx_buf[p] <= "Y"; p++;
            tx_buf[p] <= " "; p++;
            emit_param("r","p","t", shadow[8'h18], p);
            emit_param("m","i","x", shadow[8'h19], p);
            emit_param("f","l","t", shadow[8'h1A], p);
            tx_buf[p] <= "t"; p++; tx_buf[p] <= "i"; p++; tx_buf[p] <= "m"; p++;
            tx_buf[p] <= ":"; p++;
            tx_buf[p] <= n2h(shadow[8'h1C][7:4]); p++;
            tx_buf[p] <= n2h(shadow[8'h1C][3:0]); p++;
            tx_buf[p] <= n2h(shadow[8'h1B][7:4]); p++;
            tx_buf[p] <= n2h(shadow[8'h1B][3:0]); p++;
            tx_buf[p] <= " "; p++;
            emit_param("m","o","d", shadow[8'h1D], p);
            emit_param("g","r","t", shadow[8'h1E], p);
            st_nl(p);
            tx_len       <= 7'(p);
            tx_idx       <= '0;
            return_state <= S_ST_FLN;
            state        <= S_TX_START;
          end else begin
            state <= S_ST_FLN;
          end
        end

        // ── Flanger (slot 4) ──
        S_ST_FLN: begin
          if (!chain_enabled[4]) begin
            automatic int p = 0;
            emit_disabled("F", "L", "N", p);
            tx_len <= 7'(p); tx_idx <= '0;
            return_state <= S_ST_REV;
            state <= S_TX_START;
          end else if (!shadow[39][0]) begin
            automatic int p = 0;
            tx_buf[p] <= "F"; p++; tx_buf[p] <= "L"; p++; tx_buf[p] <= "N"; p++;
            tx_buf[p] <= " "; p++;
            emit_param("m","a","n", shadow[8'h20], p);
            emit_param("w","i","d", shadow[8'h21], p);
            emit_param("s","p","d", shadow[8'h22], p);
            emit_param("r","e","g", shadow[8'h23], p);
            emit_param("m","i","x", shadow[8'h24], p);
            st_nl(p);
            tx_len       <= 7'(p);
            tx_idx       <= '0;
            return_state <= S_ST_REV;
            state        <= S_TX_START;
          end else begin
            state <= S_ST_REV;
          end
        end

        // ── Reverb (slot 5) ──
        S_ST_REV: begin
          if (!chain_enabled[5]) begin
            automatic int p = 0;
            emit_disabled("R", "E", "V", p);
            tx_len <= 7'(p); tx_idx <= '0;
            return_state <= S_ST_CMP;
            state <= S_TX_START;
          end else if (!shadow[47][0]) begin
            automatic int p = 0;
            tx_buf[p] <= "R"; p++; tx_buf[p] <= "E"; p++; tx_buf[p] <= "V"; p++;
            tx_buf[p] <= " "; p++;
            emit_param("d","e","c", shadow[8'h28], p);
            emit_param("d","m","p", shadow[8'h29], p);
            emit_param("m","i","x", shadow[8'h2A], p);
            emit_param("p","r","e", shadow[8'h2B], p);
            emit_param("t","o","n", shadow[8'h2C], p);
            emit_param("l","v","l", shadow[8'h2D], p);
            st_nl(p);
            tx_len       <= 7'(p);
            tx_idx       <= '0;
            return_state <= S_ST_CMP;
            state        <= S_TX_START;
          end else begin
            state <= S_ST_CMP;
          end
        end

        // ── Compressor (slot 6) ──
        S_ST_CMP: begin
          if (!chain_enabled[6]) begin
            automatic int p = 0;
            emit_disabled("C", "M", "P", p);
            tx_len <= 7'(p); tx_idx <= '0;
            return_state <= S_ST_WAH;
            state <= S_TX_START;
          end else if (!shadow[55][0]) begin
            automatic int p = 0;
            tx_buf[p] <= "C"; p++; tx_buf[p] <= "M"; p++; tx_buf[p] <= "P"; p++;
            tx_buf[p] <= " "; p++;
            emit_param("t","h","r", shadow[8'h30], p);
            emit_param("r","a","t", shadow[8'h31], p);
            emit_param("a","t","k", shadow[8'h32], p);
            emit_param("r","e","l", shadow[8'h33], p);
            emit_param("m","a","k", shadow[8'h34], p);
            st_nl(p);
            tx_len       <= 7'(p);
            tx_idx       <= '0;
            return_state <= S_ST_WAH;
            state        <= S_TX_START;
          end else begin
            state <= S_ST_WAH;
          end
        end

        // ── Wah (slot 7) ──
        S_ST_WAH: begin
          if (!chain_enabled[7]) begin
            automatic int p = 0;
            emit_disabled("W", "A", "H", p);
            tx_len <= 7'(p); tx_idx <= '0;
            return_state <= S_LOAD_PROMPT;
            state <= S_TX_START;
          end else if (!shadow[63][0]) begin
            automatic int p = 0;
            tx_buf[p] <= "W"; p++; tx_buf[p] <= "A"; p++; tx_buf[p] <= "H"; p++;
            tx_buf[p] <= " "; p++;
            emit_param("f","r","q", shadow[8'h38], p);
            emit_param("r","e","s", shadow[8'h39], p);
            emit_param("d","p","t", shadow[8'h3A], p);
            emit_param("m","o","d", shadow[8'h3B], p);
            emit_param("m","i","x", shadow[8'h3C], p);
            st_nl(p);
            tx_len       <= 7'(p);
            tx_idx       <= '0;
            return_state <= S_LOAD_PROMPT;
            state        <= S_TX_START;
          end else begin
            state <= S_LOAD_PROMPT;
          end
        end

        // ──────────────────────────────────────────────────────────────
        //  Chain command
        // ──────────────────────────────────────────────────────────────

        S_CHAIN_PARSE: begin
          logic [5:0] tag_pos;
          logic [7:0] c0;
          logic [3:0] parsed;

          tag_pos = 6 + {chain_count, 2'b00};  // 6 + chain_count*4
          c0 = cmd_buf[tag_pos];

          if (c0 >= "a" && c0 <= "z") begin
            parsed = tag_to_port(c0, cmd_buf[tag_pos + 1], cmd_buf[tag_pos + 2]);
            if (parsed == 4'd15 || parsed == 4'd0) begin
              send_err();
            end else begin
              chain_ports[chain_count] <= parsed;
              chain_enabled[parsed - 1] <= 1'b1;
              chain_count <= chain_count + 1;
              // stay in S_CHAIN_PARSE
            end
          end else begin
            if (chain_count == 0)
              send_err();
            else begin
              chain_step <= '0;
              state      <= S_CHAIN_WRITE;
            end
          end
        end

        S_CHAIN_WRITE: begin
          if (cdc_wait != '0) begin
            cdc_wait <= cdc_wait - 1;
          end else begin
            logic [3:0] cur_idx;
            cur_idx = chain_step[4:1];

            if (cur_idx > chain_count) begin
              send_prompt();
            end else if (chain_step[0] == 1'b0) begin
              // Phase 0: set up addr/data
              wr_en_reg <= 1'b0;
              if (cur_idx < chain_count) begin
                wr_addr_reg <= ADDR_RTE_BASE + {4'd0, chain_ports[cur_idx]};
                wr_data_reg <= (cur_idx == 4'd0)
                               ? 8'd0
                               : {4'd0, chain_ports[cur_idx - 1]};
              end else begin
                // DAC entry: route[0] = last effect
                wr_addr_reg <= ADDR_RTE_BASE;
                wr_data_reg <= {4'd0, chain_ports[chain_count - 1]};
              end
              chain_step <= chain_step + 1;
            end else begin
              // Phase 1: assert write + CDC wait
              wr_en_reg <= 1'b1;
              update_shadow(wr_addr_reg, wr_data_reg);
              chain_step <= chain_step + 1;
              cdc_wait   <= 5'(CDC_WAIT_CYCLES);
            end
          end
        end

        // ──────────────────────────────────────────────────────────────
        //  Hex parser
        // ──────────────────────────────────────────────────────────────

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

        // ──────────────────────────────────────────────────────────────
        //  Register writes
        // ──────────────────────────────────────────────────────────────

        S_WRITE_REG: begin
          wr_addr_reg <= target_addr;
          wr_data_reg <= parse_acc[7:0];
          wr_en_reg   <= 1'b1;
          update_shadow(target_addr, parse_acc[7:0]);
          send_prompt();
        end

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

        // ──────────────────────────────────────────────────────────────
        //  TX engine
        // ──────────────────────────────────────────────────────────────

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
              MSG_SEL_PROMPT: tx_buf[tx_idx] <= MSG_PROMPT[tx_idx[2:0]];
              MSG_SEL_ERR:    tx_buf[tx_idx] <= MSG_ERR   [tx_idx[2:0]];
              default:        tx_buf[tx_idx] <= MSG_ERR   [tx_idx[2:0]];
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