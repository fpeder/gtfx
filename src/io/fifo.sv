`timescale 1ns / 1ps

module fifo #(
    parameter int DATA_WIDTH = 8,
    parameter int DEPTH      = 16  // must be power of 2
) (
    input logic clk,
    input logic rst_n,

    // Write interface
    input  logic                  wr_en,
    input  logic [DATA_WIDTH-1:0] wr_data,
    output logic                  full,

    // Read interface
    input  logic                  rd_en,
    output logic [DATA_WIDTH-1:0] rd_data,
    output logic                  empty,

    // Status
    output logic [$clog2(DEPTH):0] count
);

  localparam int PTR_W = $clog2(DEPTH);

  logic [DATA_WIDTH-1:0] mem[0:DEPTH-1];
  logic [PTR_W:0] wr_ptr, rd_ptr;

  assign count = wr_ptr - rd_ptr;
  assign full = (count == DEPTH[PTR_W:0]);
  assign empty = (wr_ptr == rd_ptr);

  assign rd_data = mem[rd_ptr[PTR_W-1:0]];

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      wr_ptr <= '0;
      rd_ptr <= '0;
    end else begin
      if (wr_en && !full) begin
        mem[wr_ptr[PTR_W-1:0]] <= wr_data;
        wr_ptr <= wr_ptr + 1;
      end
      if (rd_en && !empty) begin
        rd_ptr <= rd_ptr + 1;
      end
    end
  end

endmodule
