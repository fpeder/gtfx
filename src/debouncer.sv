module debouncer #(
    parameter int CLK_FREQ_MHZ = 100
) (
    input  logic clk,
    input  logic i_btn,
    output logic o_pulse
);
  localparam int WAIT = CLK_FREQ_MHZ * 10000;
  int   count = 0;
  logic stable_state = 0;
  logic sync_0, sync_1;

  always_ff @(posedge clk) begin
    sync_0 <= i_btn;
    sync_1 <= sync_0;
    if (sync_1 != stable_state) begin
      if (count < WAIT) count <= count + 1;
      else begin
        stable_state <= sync_1;
        count <= 0;
      end
    end else count <= 0;
  end

  logic prev_state;
  always_ff @(posedge clk) prev_state <= stable_state;
  assign o_pulse = stable_state && !prev_state;

endmodule

