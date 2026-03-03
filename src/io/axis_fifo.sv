`timescale 1ns / 1ps

// ============================================================================
// axis_fifo.sv — AXI-Stream FIFO wrapper around the original fifo.sv core
//
// Instantiates fifo.sv internally and presents standard AXI-Stream
// slave (write) and master (read) interfaces.
//
// Transfer in:  s_axis beat  → FIFO write
// Transfer out: m_axis beat  → FIFO read
//
// tready is deasserted when FIFO is full (slave) or tvalid deasserted
// when FIFO is empty (master).
// ============================================================================

module axis_fifo #(
    parameter int DATA_WIDTH = 8,
    parameter int DEPTH      = 16
)(
    input  logic clk,
    input  logic rst_n,

    // AXI-Stream slave (write)
    input  logic [DATA_WIDTH-1:0] s_axis_tdata,
    input  logic                  s_axis_tvalid,
    output logic                  s_axis_tready,

    // AXI-Stream master (read)
    output logic [DATA_WIDTH-1:0] m_axis_tdata,
    output logic                  m_axis_tvalid,
    input  logic                  m_axis_tready
);

    // ---- Internal FIFO signals ----
    logic                  fifo_wr_en;
    logic [DATA_WIDTH-1:0] fifo_wr_data;
    logic                  fifo_full;
    logic                  fifo_rd_en;
    logic [DATA_WIDTH-1:0] fifo_rd_data;
    logic                  fifo_empty;

    // ---- Original FIFO core (UNCHANGED) ----
    fifo #(
        .DATA_WIDTH(DATA_WIDTH),
        .DEPTH     (DEPTH)
    ) core (
        .clk    (clk),
        .rst_n  (rst_n),
        .wr_en  (fifo_wr_en),
        .wr_data(fifo_wr_data),
        .full   (fifo_full),
        .rd_en  (fifo_rd_en),
        .rd_data(fifo_rd_data),
        .empty  (fifo_empty),
        .count  ()
    );

    // ---- Slave (write) side ----
    assign fifo_wr_en   = s_axis_tvalid & ~fifo_full;
    assign fifo_wr_data = s_axis_tdata;
    assign s_axis_tready = ~fifo_full;

    // ---- Master (read) side ----
    // The original fifo.sv has combinational (async) rd_data.
    // We present rd_data directly and pop on acceptance.
    //
    // m_axis_tvalid = !empty (data is available to read)
    // m_axis_tdata  = fifo_rd_data (combinational from fifo)
    // fifo_rd_en    = tvalid && tready (pop on beat)

    assign m_axis_tdata  = fifo_rd_data;
    assign m_axis_tvalid = ~fifo_empty;
    assign fifo_rd_en    = ~fifo_empty & m_axis_tready;

endmodule
