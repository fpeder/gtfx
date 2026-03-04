`timescale 1ns / 1ps
// ============================================================================
// ddr_delay_line.sv - DDR3-Backed Circular Buffer Delay Line
//
// Drop-in replacement for delay_line.sv when very long delays are needed
// (tens of seconds at 48 kHz).  Stores samples in external DDR3L SDRAM via
// an AXI4 master interface to Xilinx MIG 7 Series.
//
// Clock domains:
//   clk     (audio, ~12.288 MHz) — write pointer, request latching, results
//   ui_clk  (MIG,   ~83 MHz)    — AXI4 back-end FSM
//
// CDC uses toggle handshakes with 2-flop synchronizers, same pattern as
// top.sv.  Multi-bit data is held stable before the toggle flips.
//
// Each sample occupies 4 bytes in DDR3 (24-bit signed, sign-extended to 32).
// Byte address = BASE_ADDR + sample_index * 4.
//
// AXI4 transactions are single-beat (AxLEN=0, AxSIZE=4 bytes).  At 48 kHz
// the total DDR3 bandwidth is negligible (~0.4 MB/s per tap).
//
// Parameters:
//   DATA_W      Sample width (signed).  Default: 24.
//   DEPTH       Max samples in circular buffer.  Default: 2097152 (~43.7 s).
//   NUM_TAPS    Independent read taps.  Default: 1.
//   AXI_ADDR_W  MIG address width.  Default: 28.
//   AXI_DATA_W  MIG data width.  Default: 128.
//   AXI_ID_W    AXI ID width.  Default: 4.
//   BASE_ADDR   Byte offset in DDR3.  Default: 0.
// ============================================================================

module ddr_delay_line #(
    parameter int DATA_W     = 24,
    parameter int DEPTH      = 2097152,
    parameter int NUM_TAPS   = 1,
    parameter int AXI_ADDR_W = 28,
    parameter int AXI_DATA_W = 128,
    parameter int AXI_ID_W   = 4,
    parameter int BASE_ADDR  = 0
) (
    // Audio domain
    input  logic                          clk,
    input  logic                          rst_n,

    input  logic                          wr_en,
    input  logic signed [DATA_W-1:0]      wr_data,
    output logic        [ADDR_W-1:0]      wr_ptr_o,

    input  logic                          rd_en,
    input  logic        [ADDR_W-1:0]      rd_ptr    [NUM_TAPS],
    output logic signed [DATA_W-1:0]      rd_data   [NUM_TAPS],
    output logic                          rd_valid,

    // MIG domain
    input  logic                          ui_clk,
    input  logic                          ui_rst_n,
    input  logic                          init_calib_complete,

    // AXI4 Write Address
    output logic [AXI_ID_W-1:0]          m_axi_awid,
    output logic [AXI_ADDR_W-1:0]        m_axi_awaddr,
    output logic [7:0]                   m_axi_awlen,
    output logic [2:0]                   m_axi_awsize,
    output logic [1:0]                   m_axi_awburst,
    output logic                         m_axi_awvalid,
    input  logic                         m_axi_awready,

    // AXI4 Write Data
    output logic [AXI_DATA_W-1:0]        m_axi_wdata,
    output logic [AXI_DATA_W/8-1:0]      m_axi_wstrb,
    output logic                         m_axi_wlast,
    output logic                         m_axi_wvalid,
    input  logic                         m_axi_wready,

    // AXI4 Write Response
    input  logic [AXI_ID_W-1:0]          m_axi_bid,
    input  logic [1:0]                   m_axi_bresp,
    input  logic                         m_axi_bvalid,
    output logic                         m_axi_bready,

    // AXI4 Read Address
    output logic [AXI_ID_W-1:0]          m_axi_arid,
    output logic [AXI_ADDR_W-1:0]        m_axi_araddr,
    output logic [7:0]                   m_axi_arlen,
    output logic [2:0]                   m_axi_arsize,
    output logic [1:0]                   m_axi_arburst,
    output logic                         m_axi_arvalid,
    input  logic                         m_axi_arready,

    // AXI4 Read Data
    input  logic [AXI_ID_W-1:0]          m_axi_rid,
    input  logic [AXI_DATA_W-1:0]        m_axi_rdata,
    input  logic [1:0]                   m_axi_rresp,
    input  logic                         m_axi_rlast,
    input  logic                         m_axi_rvalid,
    output logic                         m_axi_rready
);

    // ========================================================================
    // Derived constants
    // ========================================================================
    localparam int ADDR_W = $clog2(DEPTH);
    localparam bit IS_POT = (DEPTH & (DEPTH - 1)) == 0;

    // Byte offset of the 32-bit sample within the AXI data word.
    // MIG with 128-bit data: low 32 bits are bytes 0-3.
    localparam int WSTRB_W = AXI_DATA_W / 8;

    // ========================================================================
    // A. Audio-domain write pointer (circular)
    // ========================================================================
    logic [ADDR_W-1:0] wr_ptr;
    logic [ADDR_W-1:0] wr_ptr_next;

    assign wr_ptr_o = wr_ptr;

    generate if (IS_POT) begin : g_pot_next
        assign wr_ptr_next = wr_ptr + ADDR_W'(1);
    end else begin : g_npot_next
        assign wr_ptr_next = (wr_ptr == ADDR_W'(DEPTH - 1)) ? '0
                                                              : wr_ptr + ADDR_W'(1);
    end endgenerate

    always_ff @(posedge clk) begin
        if (!rst_n)
            wr_ptr <= '0;
        else if (wr_en)
            wr_ptr <= wr_ptr_next;
    end

    // ========================================================================
    // B. Audio-domain request latching & toggle generation
    // ========================================================================

    // Write request channel (audio -> ui)
    logic                    wr_req_tog;
    logic [ADDR_W-1:0]      wr_req_addr;
    logic signed [DATA_W-1:0] wr_req_data;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            wr_req_tog  <= 1'b0;
            wr_req_addr <= '0;
            wr_req_data <= '0;
        end else if (wr_en) begin
            wr_req_addr <= wr_ptr;
            wr_req_data <= wr_data;
            wr_req_tog  <= ~wr_req_tog;
        end
    end

    // Read request channel (audio -> ui)
    logic               rd_req_tog;
    logic [ADDR_W-1:0]  rd_req_ptr [NUM_TAPS];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rd_req_tog <= 1'b0;
            for (int i = 0; i < NUM_TAPS; i++)
                rd_req_ptr[i] <= '0;
        end else if (rd_en) begin
            for (int i = 0; i < NUM_TAPS; i++)
                rd_req_ptr[i] <= rd_ptr[i];
            rd_req_tog <= ~rd_req_tog;
        end
    end

    // ========================================================================
    // C. Audio-domain response capture
    // ========================================================================

    // rd_done toggle (ui -> audio), synchronized
    logic rd_done_tog_ui;   // driven in ui_clk domain
    logic rd_done_s1, rd_done_s2, rd_done_prev;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rd_done_s1   <= 1'b0;
            rd_done_s2   <= 1'b0;
            rd_done_prev <= 1'b0;
        end else begin
            rd_done_s1   <= rd_done_tog_ui;
            rd_done_s2   <= rd_done_s1;
            rd_done_prev <= rd_done_s2;
        end
    end

    logic rd_done_edge;
    assign rd_done_edge = rd_done_s2 ^ rd_done_prev;

    // Read results held in ui_clk domain, captured here on rd_done edge
    logic signed [DATA_W-1:0] rd_result [NUM_TAPS];  // driven in ui_clk domain

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rd_valid <= 1'b0;
            for (int i = 0; i < NUM_TAPS; i++)
                rd_data[i] <= '0;
        end else begin
            rd_valid <= rd_done_edge;
            if (rd_done_edge) begin
                for (int i = 0; i < NUM_TAPS; i++)
                    rd_data[i] <= rd_result[i];
            end
        end
    end

    // ========================================================================
    // D. CDC: audio -> ui_clk (write request toggle)
    // ========================================================================
    logic wr_tog_s1, wr_tog_s2, wr_tog_prev;
    logic wr_tog_edge;

    always_ff @(posedge ui_clk) begin
        if (!ui_rst_n) begin
            wr_tog_s1   <= 1'b0;
            wr_tog_s2   <= 1'b0;
            wr_tog_prev <= 1'b0;
        end else begin
            wr_tog_s1   <= wr_req_tog;
            wr_tog_s2   <= wr_tog_s1;
            wr_tog_prev <= wr_tog_s2;
        end
    end

    assign wr_tog_edge = wr_tog_s2 ^ wr_tog_prev;

    // ========================================================================
    // E. CDC: audio -> ui_clk (read request toggle)
    // ========================================================================
    logic rd_tog_s1, rd_tog_s2, rd_tog_prev;
    logic rd_tog_edge;

    always_ff @(posedge ui_clk) begin
        if (!ui_rst_n) begin
            rd_tog_s1   <= 1'b0;
            rd_tog_s2   <= 1'b0;
            rd_tog_prev <= 1'b0;
        end else begin
            rd_tog_s1   <= rd_req_tog;
            rd_tog_s2   <= rd_tog_s1;
            rd_tog_prev <= rd_tog_s2;
        end
    end

    assign rd_tog_edge = rd_tog_s2 ^ rd_tog_prev;

    // ========================================================================
    // F. AXI4 back-end FSM (ui_clk domain)
    // ========================================================================

    typedef enum logic [2:0] {
        S_IDLE,
        S_WR_ADDR,
        S_WR_DATA,
        S_WR_RESP,
        S_RD_ADDR,
        S_RD_DATA,
        S_RD_NEXT,
        S_DONE
    } state_t;

    state_t state;

    // Pending flags — set on toggle edge, cleared when FSM processes them
    logic wr_pending;
    logic rd_pending;

    // Latched request data (ui_clk domain)
    logic [ADDR_W-1:0]        ui_wr_addr;
    logic signed [DATA_W-1:0] ui_wr_data;
    logic [ADDR_W-1:0]        ui_rd_ptr [NUM_TAPS];

    // Read tap counter
    logic [$clog2(NUM_TAPS > 1 ? NUM_TAPS : 2)-1:0] tap_idx;

    // Byte address computation
    function automatic logic [AXI_ADDR_W-1:0] sample_byte_addr(
        input logic [ADDR_W-1:0] idx
    );
        // BASE_ADDR + idx * 4
        return AXI_ADDR_W'(BASE_ADDR) + {AXI_ADDR_W'(idx), 2'b00};
    endfunction

    // Latch pending requests on toggle edges
    always_ff @(posedge ui_clk) begin
        if (!ui_rst_n) begin
            wr_pending <= 1'b0;
            ui_wr_addr <= '0;
            ui_wr_data <= '0;
        end else if (wr_tog_edge) begin
            wr_pending <= 1'b1;
            ui_wr_addr <= wr_req_addr;
            ui_wr_data <= wr_req_data;
        end else if (state == S_WR_ADDR && m_axi_awready) begin
            wr_pending <= 1'b0;
        end
    end

    always_ff @(posedge ui_clk) begin
        if (!ui_rst_n) begin
            rd_pending <= 1'b0;
            for (int i = 0; i < NUM_TAPS; i++)
                ui_rd_ptr[i] <= '0;
        end else if (rd_tog_edge) begin
            rd_pending <= 1'b1;
            for (int i = 0; i < NUM_TAPS; i++)
                ui_rd_ptr[i] <= rd_req_ptr[i];
        end else if (state == S_DONE) begin
            rd_pending <= 1'b0;
        end
    end

    // Main FSM
    always_ff @(posedge ui_clk) begin
        if (!ui_rst_n) begin
            state          <= S_IDLE;
            tap_idx        <= '0;
            rd_done_tog_ui <= 1'b0;
            for (int i = 0; i < NUM_TAPS; i++)
                rd_result[i] <= '0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (init_calib_complete) begin
                        if (wr_pending)
                            state <= S_WR_ADDR;
                        else if (rd_pending) begin
                            tap_idx <= '0;
                            state   <= S_RD_ADDR;
                        end
                    end
                end

                S_WR_ADDR: begin
                    if (m_axi_awready)
                        state <= S_WR_DATA;
                end

                S_WR_DATA: begin
                    if (m_axi_wready)
                        state <= S_WR_RESP;
                end

                S_WR_RESP: begin
                    if (m_axi_bvalid) begin
                        // After write, service pending read or go to DONE
                        if (rd_pending) begin
                            tap_idx <= '0;
                            state   <= S_RD_ADDR;
                        end else begin
                            state <= S_DONE;
                        end
                    end
                end

                S_RD_ADDR: begin
                    if (m_axi_arready)
                        state <= S_RD_DATA;
                end

                S_RD_DATA: begin
                    if (m_axi_rvalid) begin
                        // Capture result: low 32 bits, sign-extend to DATA_W
                        rd_result[tap_idx] <= DATA_W'($signed(m_axi_rdata[31:0]));
                        state <= S_RD_NEXT;
                    end
                end

                S_RD_NEXT: begin
                    if (tap_idx == NUM_TAPS - 1) begin
                        state          <= S_DONE;
                        rd_done_tog_ui <= ~rd_done_tog_ui;
                    end else begin
                        tap_idx <= tap_idx + 1;
                        state   <= S_RD_ADDR;
                    end
                end

                S_DONE: begin
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // ========================================================================
    // G. AXI4 output assignments
    // ========================================================================

    // Write address channel
    assign m_axi_awid    = '0;
    assign m_axi_awaddr  = sample_byte_addr(ui_wr_addr);
    assign m_axi_awlen   = 8'd0;                       // single beat
    assign m_axi_awsize  = 3'b010;                      // 4 bytes
    assign m_axi_awburst = 2'b01;                       // INCR
    assign m_axi_awvalid = (state == S_WR_ADDR);

    // Write data channel — 24-bit sample sign-extended to 32 bits in low word
    logic [AXI_DATA_W-1:0] wdata_packed;
    logic [WSTRB_W-1:0]    wstrb_packed;

    always_comb begin
        wdata_packed = '0;
        wdata_packed[31:0] = {{(32 - DATA_W){ui_wr_data[DATA_W-1]}}, ui_wr_data};
        wstrb_packed = '0;
        wstrb_packed[3:0] = 4'hF;   // bytes 0-3 valid
    end

    assign m_axi_wdata  = wdata_packed;
    assign m_axi_wstrb  = wstrb_packed;
    assign m_axi_wlast  = 1'b1;                        // single beat
    assign m_axi_wvalid = (state == S_WR_DATA);

    // Write response channel
    assign m_axi_bready = (state == S_WR_RESP);

    // Read address channel
    assign m_axi_arid    = '0;
    assign m_axi_araddr  = sample_byte_addr(ui_rd_ptr[tap_idx]);
    assign m_axi_arlen   = 8'd0;                       // single beat
    assign m_axi_arsize  = 3'b010;                      // 4 bytes
    assign m_axi_arburst = 2'b01;                       // INCR
    assign m_axi_arvalid = (state == S_RD_ADDR);

    // Read data channel
    assign m_axi_rready = (state == S_RD_DATA);

endmodule
