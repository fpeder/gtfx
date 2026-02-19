module volume (
    input  logic               sys_clk,   // System Clock (e.g., 100MHz)
    input  logic               reset_n,   // Active High Reset    // User Interface
    input  logic               btn_up,    // Volume Up pulse
    input  logic               btn_down,  // Volume Down pulse
    input  logic signed [23:0] i_sample,  // Raw 24-bit sample from I2S
    output logic signed [23:0] o_sample   // Final adjusted sample to I2S
);
  // --- Internal Signals ---
  logic [3:0] vol_idx;
  logic [7:0] gain_q44;
  logic up_pulse, down_pulse;

  // --- 1. Debouncers for Buttons ---
  debouncer #(
      .CLK_FREQ_MHZ(100)
  ) db_u (
      .clk(sys_clk),
      .i_btn(btn_up),
      .o_pulse(up_pulse)
  );
  debouncer #(
      .CLK_FREQ_MHZ(100)
  ) db_d (
      .clk(sys_clk),
      .i_btn(btn_down),
      .o_pulse(down_pulse)
  );

  // --- 2. Volume Counter (Saturation at 0 and 15) ---
  always_ff @(posedge sys_clk or negedge reset_n) begin
    if (!reset_n) begin
      vol_idx <= 4'hC;  // Default to 0dB (Unity)
    end else begin
      if (up_pulse && vol_idx < 4'hF) vol_idx <= vol_idx + 1;
      else if (down_pulse && vol_idx > 4'h0) vol_idx <= vol_idx - 1;
    end
  end

  // --- 3. dB Look-Up Table (Mapping Index to Q4.4 Gain) ---
  // Mapping: 0=-inf, 12=0dB, 15=+6dB
  always_comb begin
    case (vol_idx)
      4'h0: gain_q44 = 8'h00;  // Mute
      4'h1: gain_q44 = 8'h01;  // -24 dB
      4'h2: gain_q44 = 8'h02;  // -18 dB
      4'h3: gain_q44 = 8'h03;
      4'h4: gain_q44 = 4'h04;  // -12 dB
      4'h5: gain_q44 = 8'h05;
      4'h6: gain_q44 = 8'h06;  // -8.5 dB
      4'h7: gain_q44 = 8'h07;
      4'h8: gain_q44 = 8'h08;  // -6 dB
      4'h9: gain_q44 = 8'h0A;  // -4 dB
      4'hA: gain_q44 = 8'h0C;  // -2.5 dB
      4'hB: gain_q44 = 8'h0E;  // -1 dB
      4'hC: gain_q44 = 8'h10;  //  0 dB (1.0x)
      4'hD: gain_q44 = 8'h14;  // +2 dB
      4'hE: gain_q44 = 8'h1A;  // +4 dB
      4'hF: gain_q44 = 8'h20;  // +6 dB (2.0x)
      default: gain_q44 = 8'h10;
    endcase
  end

  // --- 4. Multiplication and Clipping ---
  logic signed [31:0] full_product;
  logic signed [27:0] scaled_sample;

  always_comb begin
    // Multiply 24-bit signed by 8-bit unsigned (treated as signed positive)
    full_product  = i_sample * $signed({1'b0, gain_q44});
    // Q4.4 alignment: Shift right by 4 to get back to 24-bit range
    scaled_sample = full_product >>> 4;
    // Hard Clipping (Saturation) Logic
    if (scaled_sample > 28'sh7FFFFF) begin
      o_sample = 24'sh7FFFFF;  // Clip at max positive 24-bit
    end else if (scaled_sample < -28'sh800000) begin
      o_sample = 24'sh800000;  // Clip at max negative 24-bit
    end else begin
      o_sample = scaled_sample[23:0];
    end
  end

endmodule

