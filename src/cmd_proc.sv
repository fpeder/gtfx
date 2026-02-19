module cmd_proc (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        rx_valid,
    input  logic [ 7:0] rx_byte,
    input  logic        tx_done,
    input  logic        tx_busy,
    output logic        tx_start,
    output logic [ 7:0] tx_byte,
    
    // Switches from top module to identify active effects
    // sw[0]=trem, sw[1]=pha, sw[2]=cho, sw[3]=dly
    input  logic [ 3:0] sw_effect, 

    // DD3 delay controls
    output logic [ 7:0] tone_val,
    output logic [ 7:0] level_val,
    output logic [ 7:0] feedback_val,
    output logic [31:0] time_val,
    // CE-5 chorus controls
    output logic [ 7:0] chorus_rate_val,
    output logic [ 7:0] chorus_depth_val,
    output logic [ 7:0] chorus_efx_val,
    output logic [ 7:0] chorus_eqhi_val,
    output logic [ 7:0] chorus_eqlo_val,
    // Phaser controls
    output logic [ 7:0] phaser_speed_val,
    output logic        phaser_fben_val,
    // Tremolo controls
    output logic [ 7:0] trem_rate_val,
    output logic [ 7:0] trem_depth_val,
    output logic        trem_shape_val
);
  localparam logic [7:0] CHAR_CR  = 8'h0D;
  localparam logic [7:0] CHAR_LF  = 8'h0A;
  localparam logic [7:0] CHAR_BS  = 8'h08;
  localparam logic [7:0] CHAR_DEL = 8'h7F;
  localparam logic [7:0] CHAR_SPC = 8'h20;
  localparam logic [7:0] CHAR_0   = 8'h30;
  localparam logic [7:0] CHAR_A   = 8'h41;
  localparam logic [7:0] MSG_PROMPT[0:5] = '{"A", "r", "t", "y", ">", " "};
  localparam int LEN_PROMPT = 6;
  localparam logic [7:0] MSG_OK[0:3]  = '{"O", "K", 8'h0D, 8'h0A};
  localparam int LEN_OK = 4;
  localparam logic [7:0] MSG_ERR[0:4] = '{"E", "r", "r", 8'h0D, 8'h0A};
  localparam int LEN_ERR = 5;
  
  // Expanded enum to 5 bits to safely fit the broken-down STATUS states
  typedef enum logic [4:0] {
    S_BOOT,
    S_IDLE,
    S_RX_CHAR,
    S_BACKSPACE,
    S_CHECK_CMD,
    S_PREP_STATUS_BYP,
    S_PREP_STATUS_DLY,
    S_PREP_STATUS_CHO,
    S_PREP_STATUS_PHA,
    S_PREP_STATUS_TRM,
    S_PARSE_INIT,
    S_PARSE_LOOP,
    S_TX_START,
    S_TX_WAIT,
    S_LOAD_PROMPT,
    S_LOAD_STRING
  } state_t;
  state_t        state        = S_BOOT;
  state_t        return_state = S_IDLE;

  logic   [ 7:0] cmd_buf [0:31];
  logic   [ 5:0] cmd_idx = 0;
  logic   [ 7:0] tx_buf  [0:63];
  logic   [ 5:0] tx_len  = 0;
  logic   [ 5:0] tx_idx  = 0;
  // ---- Delay registers ----
  logic   [ 7:0] reg_tone  = 8'hFF;
  logic   [ 7:0] reg_level = 8'd128;
  logic   [ 7:0] reg_fb    = 8'd100;
  logic   [31:0] reg_time  = 32'd2000;

  // ---- Chorus registers ----
  logic   [ 7:0] reg_ch_rate  = 8'd80;
  logic   [ 7:0] reg_ch_depth = 8'd100;
  logic   [ 7:0] reg_ch_efx   = 8'd128;
  logic   [ 7:0] reg_ch_eqhi  = 8'd200;
  logic   [ 7:0] reg_ch_eqlo  = 8'd128;
  // ---- Phaser registers ----
  logic   [ 7:0] reg_pha_speed = 8'd80;
  logic          reg_pha_fben  = 1'b0;
  // ---- Tremolo registers ----
  logic   [ 7:0] reg_trem_rate  = 8'd60;
  logic   [ 7:0] reg_trem_depth = 8'd180;
  logic          reg_trem_shape = 1'b0;

  logic   [31:0] parse_acc = 0;
  logic   [ 5:0] parse_ptr = 0;
  logic   [ 3:0] target_reg = 0;
  logic          rx_pending = 0;
  logic   [ 7:0] rx_latch   = 0;
  assign tone_val         = reg_tone;
  assign level_val        = reg_level;
  assign feedback_val     = reg_fb;
  assign time_val         = reg_time;
  assign chorus_rate_val  = reg_ch_rate;
  assign chorus_depth_val = reg_ch_depth;
  assign chorus_efx_val   = reg_ch_efx;
  assign chorus_eqhi_val  = reg_ch_eqhi;
  assign chorus_eqlo_val  = reg_ch_eqlo;
  assign phaser_speed_val = reg_pha_speed;
  assign phaser_fben_val  = reg_pha_fben;
  assign trem_rate_val    = reg_trem_rate;
  assign trem_depth_val   = reg_trem_depth;
  assign trem_shape_val   = reg_trem_shape;

  function automatic logic [7:0] n2h(input [3:0] n);
    return (n < 10) ? (n + CHAR_0) : (n - 10 + CHAR_A);
  endfunction

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      state      <= S_BOOT;
      cmd_idx    <= 0;
      tx_start   <= 0;
      rx_pending <= 0;
      tx_byte    <= 0;
      rx_latch   <= 0;
    end else begin
      if (rx_valid) begin
        rx_pending <= 1;
        rx_latch   <= rx_byte;
      end

      case (state)
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

        S_CHECK_CMD: begin
          cmd_idx <= 0;

          if (cmd_buf[0] == "s" && cmd_buf[1] == "t"
              && (cmd_buf[2] == 8'h00 || cmd_buf[2] == " ")) begin
            // Route to appropriate status state machine phase
            if (sw_effect == 4'b0000) state <= S_PREP_STATUS_BYP;
            else                      state <= S_PREP_STATUS_DLY;
            
          end else if (cmd_buf[0]=="s" && cmd_buf[1]=="e" && cmd_buf[2]=="t"
                    && cmd_buf[3]==" "
                    && cmd_buf[4]=="d" && cmd_buf[5]=="l" && cmd_buf[6]=="y"
                    && cmd_buf[7]==" ") begin
            if      (cmd_buf[8]=="l")                                  begin parse_ptr<=14; target_reg<=1;  state<=S_PARSE_INIT; end
            else if (cmd_buf[8]=="f" && cmd_buf[9]=="e")               begin parse_ptr<=17; target_reg<=2;  state<=S_PARSE_INIT; end
            else if (cmd_buf[8]=="t" && cmd_buf[9]=="i")               begin parse_ptr<=13; target_reg<=3;  state<=S_PARSE_INIT; end
            else if (cmd_buf[8]=="t" && cmd_buf[9]=="o")               begin parse_ptr<=13; target_reg<=4;  state<=S_PARSE_INIT; end
            else begin tx_len<=LEN_ERR; tx_idx<=0; rx_latch<=3; state<=S_LOAD_STRING; return_state<=S_LOAD_PROMPT; end

          end else if (cmd_buf[0]=="s" && cmd_buf[1]=="e" && cmd_buf[2]=="t"
                    && cmd_buf[3]==" "
                    && cmd_buf[4]=="c" && cmd_buf[5]=="h" && cmd_buf[6]=="o"
                    && cmd_buf[7]==" ") begin
            if      (cmd_buf[8]=="r")                                        begin parse_ptr<=13; target_reg<=5;  state<=S_PARSE_INIT; end
            else if (cmd_buf[8]=="d")                                        begin parse_ptr<=14; target_reg<=6;  state<=S_PARSE_INIT; end
            else if (cmd_buf[8]=="e" && cmd_buf[9]=="f")                     begin parse_ptr<=12; target_reg<=7;  state<=S_PARSE_INIT; end
            else if (cmd_buf[8]=="e" && cmd_buf[9]=="q" && cmd_buf[10]=="h") begin parse_ptr<=13; target_reg<=8;  state<=S_PARSE_INIT; end
            else if (cmd_buf[8]=="e" && cmd_buf[9]=="q" && cmd_buf[10]=="l") begin parse_ptr<=13; target_reg<=9;  state<=S_PARSE_INIT; end
            else begin tx_len<=LEN_ERR; tx_idx<=0; rx_latch<=3; state<=S_LOAD_STRING; return_state<=S_LOAD_PROMPT; end

          end else if (cmd_buf[0]=="s" && cmd_buf[1]=="e" && cmd_buf[2]=="t"
                    && cmd_buf[3]==" "
                    && cmd_buf[4]=="p" && cmd_buf[5]=="h" && cmd_buf[6]=="a"
                    && cmd_buf[7]==" ") begin
            if      (cmd_buf[8]=="s") begin parse_ptr<=14; target_reg<=10; state<=S_PARSE_INIT; end
            else if (cmd_buf[8]=="f") begin parse_ptr<=13; target_reg<=11; state<=S_PARSE_INIT; end
            else begin tx_len<=LEN_ERR; tx_idx<=0; rx_latch<=3; state<=S_LOAD_STRING; return_state<=S_LOAD_PROMPT; end

          end else if (cmd_buf[0]=="s" && cmd_buf[1]=="e" && cmd_buf[2]=="t"
                    && cmd_buf[3]==" "
                    && cmd_buf[4]=="t" && cmd_buf[5]=="r" && cmd_buf[6]=="m"
                    && cmd_buf[7]==" ") begin
            if      (cmd_buf[8]=="r") begin parse_ptr<=13; target_reg<=12; state<=S_PARSE_INIT; end
            else if (cmd_buf[8]=="d") begin parse_ptr<=14; target_reg<=13; state<=S_PARSE_INIT; end
            else if (cmd_buf[8]=="s") begin parse_ptr<=14; target_reg<=14; state<=S_PARSE_INIT; end
            else begin tx_len<=LEN_ERR; tx_idx<=0; rx_latch<=3; state<=S_LOAD_STRING; return_state<=S_LOAD_PROMPT; end

          end else begin
            tx_len<=LEN_ERR; tx_idx<=0; rx_latch<=3; state<=S_LOAD_STRING; return_state<=S_LOAD_PROMPT;
          end
        end

        // ====================================================================
        // Status Output Chain 
        // Sequentially evaluates sw_effect and queues TX buffers
        // ====================================================================
        S_PREP_STATUS_BYP: begin
          tx_buf[0]<="B"; tx_buf[1]<="Y"; tx_buf[2]<="P"; tx_buf[3]<="A"; 
          tx_buf[4]<="S"; tx_buf[5]<="S"; tx_buf[6]<=CHAR_CR; tx_buf[7]<=CHAR_LF;
          tx_len <= 8;
          tx_idx <= 0;
          return_state <= S_LOAD_PROMPT;
          state <= S_TX_START;
        end

        S_PREP_STATUS_DLY: begin
          if (sw_effect[3]) begin
            tx_buf[0] <="D"; tx_buf[1] <="L"; tx_buf[2] <="Y"; tx_buf[3] <=" ";
            tx_buf[4] <="L"; tx_buf[5] <=":";
            tx_buf[6] <=n2h(reg_level[7:4]); tx_buf[7] <=n2h(reg_level[3:0]);
            tx_buf[8] <=" "; tx_buf[9] <="F"; tx_buf[10]<=":";
            tx_buf[11]<=n2h(reg_fb[7:4]); tx_buf[12]<=n2h(reg_fb[3:0]);
            tx_buf[13]<=" "; tx_buf[14]<="N"; tx_buf[15]<=":";
            tx_buf[16]<=n2h(reg_tone[7:4]); tx_buf[17]<=n2h(reg_tone[3:0]);
            tx_buf[18]<=" "; tx_buf[19]<="T"; tx_buf[20]<=":";
            tx_buf[21]<=n2h(reg_time[31:28]); tx_buf[22]<=n2h(reg_time[27:24]);
            tx_buf[23]<=n2h(reg_time[23:20]); tx_buf[24]<=n2h(reg_time[19:16]);
            tx_buf[25]<=n2h(reg_time[15:12]); tx_buf[26]<=n2h(reg_time[11:8]);
            tx_buf[27]<=n2h(reg_time[7:4]);   tx_buf[28]<=n2h(reg_time[3:0]);
            tx_buf[29]<=CHAR_CR; tx_buf[30]<=CHAR_LF;
            tx_len <= 31;
            tx_idx <= 0;
            return_state <= S_PREP_STATUS_CHO;
            state <= S_TX_START;
          end else begin
            state <= S_PREP_STATUS_CHO;
          end
        end

        S_PREP_STATUS_CHO: begin
          if (sw_effect[2]) begin
            tx_buf[0]<="C"; tx_buf[1]<="H"; tx_buf[2]<="O"; tx_buf[3]<=" ";
            tx_buf[4]<="R"; tx_buf[5]<=":";
            tx_buf[6]<=n2h(reg_ch_rate[7:4]);  tx_buf[7]<=n2h(reg_ch_rate[3:0]);
            tx_buf[8]<=" "; tx_buf[9]<="D"; tx_buf[10]<=":";
            tx_buf[11]<=n2h(reg_ch_depth[7:4]); tx_buf[12]<=n2h(reg_ch_depth[3:0]);
            tx_buf[13]<=" "; tx_buf[14]<="E"; tx_buf[15]<=":";
            tx_buf[16]<=n2h(reg_ch_efx[7:4]);   tx_buf[17]<=n2h(reg_ch_efx[3:0]);
            tx_buf[18]<=" "; tx_buf[19]<="H"; tx_buf[20]<=":";
            tx_buf[21]<=n2h(reg_ch_eqhi[7:4]);  tx_buf[22]<=n2h(reg_ch_eqhi[3:0]);
            tx_buf[23]<=" "; tx_buf[24]<="L"; tx_buf[25]<=":";
            tx_buf[26]<=n2h(reg_ch_eqlo[7:4]);  tx_buf[27]<=n2h(reg_ch_eqlo[3:0]);
            tx_buf[28]<=CHAR_CR; tx_buf[29]<=CHAR_LF;
            tx_len <= 30;
            tx_idx <= 0;
            return_state <= S_PREP_STATUS_PHA;
            state <= S_TX_START;
          end else begin
            state <= S_PREP_STATUS_PHA;
          end
        end

        S_PREP_STATUS_PHA: begin
          if (sw_effect[1]) begin
            tx_buf[0] <="P"; tx_buf[1] <="H"; tx_buf[2] <="A"; tx_buf[3] <=" ";
            tx_buf[4] <="S"; tx_buf[5] <=":";
            tx_buf[6] <=n2h(reg_pha_speed[7:4]); tx_buf[7] <=n2h(reg_pha_speed[3:0]);
            tx_buf[8] <=" "; tx_buf[9] <="B"; tx_buf[10]<=":";
            tx_buf[11]<= reg_pha_fben ? "1" : "0";
            tx_buf[12]<=CHAR_CR; tx_buf[13]<=CHAR_LF;
            tx_len <= 14;
            tx_idx <= 0;
            return_state <= S_PREP_STATUS_TRM;
            state <= S_TX_START;
          end else begin
            state <= S_PREP_STATUS_TRM;
          end
        end

        S_PREP_STATUS_TRM: begin
          if (sw_effect[0]) begin
            tx_buf[0]<="T"; tx_buf[1]<="R"; tx_buf[2]<="M"; tx_buf[3]<=" ";
            tx_buf[4]<="R"; tx_buf[5]<=":";
            tx_buf[6]<=n2h(reg_trem_rate[7:4]); tx_buf[7]<=n2h(reg_trem_rate[3:0]);
            tx_buf[8]<=" "; tx_buf[9]<="D"; tx_buf[10]<=":";
            tx_buf[11]<=n2h(reg_trem_depth[7:4]); tx_buf[12]<=n2h(reg_trem_depth[3:0]);
            tx_buf[13]<=" "; tx_buf[14]<="W"; tx_buf[15]<=":";
            tx_buf[16]<= reg_trem_shape ? "1" : "0";
            tx_buf[17]<=CHAR_CR; tx_buf[18]<=CHAR_LF;
            tx_len <= 19;
            tx_idx <= 0;
            return_state <= S_LOAD_PROMPT;
            state <= S_TX_START;
          end else begin
            state <= S_LOAD_PROMPT;
          end
        end

        // ====================================================================
        // Parse Engine
        // ====================================================================
        S_PARSE_INIT: begin
          parse_acc <= 0;
          state <= S_PARSE_LOOP;
        end

        S_PARSE_LOOP: begin
          logic [7:0] c;
          logic [3:0] nibble;
          c = cmd_buf[parse_ptr];
          if (c >= "0" && c <= "9") begin
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
            if      (target_reg == 1)  reg_level      <= parse_acc[7:0];
            else if (target_reg == 2)  reg_fb         <= parse_acc[7:0];
            else if (target_reg == 3)  reg_time       <= parse_acc;
            else if (target_reg == 4)  reg_tone       <= parse_acc[7:0];
            else if (target_reg == 5)  reg_ch_rate    <= parse_acc[7:0];
            else if (target_reg == 6)  reg_ch_depth   <= parse_acc[7:0];
            else if (target_reg == 7)  reg_ch_efx     <= parse_acc[7:0];
            else if (target_reg == 8)  reg_ch_eqhi    <= parse_acc[7:0];
            else if (target_reg == 9)  reg_ch_eqlo    <= parse_acc[7:0];
            else if (target_reg == 10) reg_pha_speed  <= parse_acc[7:0];
            else if (target_reg == 11) reg_pha_fben   <= parse_acc[0];
            else if (target_reg == 12) reg_trem_rate  <= parse_acc[7:0];
            else if (target_reg == 13) reg_trem_depth <= parse_acc[7:0];
            else if (target_reg == 14) reg_trem_shape <= parse_acc[0];
            tx_len <= LEN_OK; tx_idx <= 0; rx_latch <= 2;
            state <= S_LOAD_STRING;
            return_state <= S_LOAD_PROMPT;
          end
        end

        S_LOAD_PROMPT: begin
          tx_idx <= 0;
          tx_len <= LEN_PROMPT;
          rx_latch <= 1;
          state <= S_LOAD_STRING;
          return_state <= S_IDLE;
        end

        S_LOAD_STRING: begin
          if (tx_idx < tx_len) begin
            if      (rx_latch == 1) tx_buf[tx_idx] <= MSG_PROMPT[tx_idx];
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