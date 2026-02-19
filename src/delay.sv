`timescale 1ns / 1ps

module delay #(
    parameter WIDTH     = 24,
    parameter RAM_DEPTH = 32000
)(
    input  logic             clk,
    input  logic             rst_n,
    input  logic             sample_en,

    // Controls (compatible with cmd_proc outputs)
    input  logic [7:0]       tone_val,        // 0x00 = dark (heavy LPF), 0xFF = bright (bypass)
    input  logic [7:0]       level_val,       // 0x00-0xFF wet/dry mix level
    input  logic [7:0]       feedback_val,    // 0x00-0xFF feedback gain
    input  logic [31:0]      time_val,        // delay time in sample counts

    // Audio
    input  logic signed [WIDTH-1:0] audio_in,
    output logic signed [WIDTH-1:0] audio_out
);

    // =========================================================================
    // 1. BLOCK RAM
    // =========================================================================
    (* ram_style = "block" *) logic signed [WIDTH-1:0] ram [0:RAM_DEPTH-1];

    logic signed [WIDTH-1:0] ram_read_data  = 0;
    logic signed [WIDTH-1:0] ram_write_data_next;

    localparam ADDR_WIDTH = $clog2(RAM_DEPTH);
    logic [ADDR_WIDTH-1:0] wr_ptr = 0;
    logic [ADDR_WIDTH-1:0] rd_ptr = 0;
    logic [ADDR_WIDTH-1:0] rd_ptr_next;

    logic [ADDR_WIDTH-1:0] delay_samples;

    initial begin
        for (int i = 0; i < RAM_DEPTH; i++) ram[i] = 0;
    end

    // =========================================================================
    // 2. GAIN & DELAY CALCULATIONS (Combinatorial)
    // =========================================================================
    // level_val / feedback_val are 8-bit hex values (0x00-0xFF).
    // Used as fixed-point gains with >>8 normalisation (0x00 = 0.0, 0xFF ~ 1.0).
    logic signed [9:0] fb_gain;
    logic signed [9:0] dry_gain;

    always_comb begin
        fb_gain  = {2'b00, feedback_val};
        dry_gain = {2'b00, level_val};
    end

    // Delay clamping logic
    always_comb begin
        // time_val = sample count, clamp to valid RAM range
        if (time_val >= RAM_DEPTH)    delay_samples = RAM_DEPTH - 1;
        else if (time_val == 0)       delay_samples = 1;
        else                          delay_samples = time_val[ADDR_WIDTH-1:0];
    end

    // =========================================================================
    // 3. TONE -- 1-POLE IIR LOW-PASS FILTER ON FEEDBACK PATH
    // =========================================================================
    //
    //   tone_val controls the cutoff:
    //     0x00  -> coeff = 1   -> heavy filtering  (very dark, almost DC)
    //     0x80  -> coeff = 129 -> moderate filtering
    //     0xFF  -> coeff = 256 -> no filtering      (fully bright / bypass)
    //
    //   Difference equation (fixed-point, >>8):
    //     lpf_state += coeff * (delayed_sample - lpf_state) >> 8
    //
    //   This is equivalent to:
    //     y[n] = (1-a)*y[n-1] + a*x[n]     where a = (tone_val + 1) / 256
    //
    // =========================================================================
    logic signed [9:0]       tone_coeff;       // tone_val + 1, range 1-256
    logic signed [WIDTH-1:0] lpf_state = 0;    // filter state register
    logic signed [WIDTH-1:0] lpf_state_next;

    always_comb begin
        tone_coeff = {2'b00, tone_val} + 1;    // 1 .. 256
    end

    // =========================================================================
    // 4. COMBINATORIAL LOGIC FOR NEXT-STATE CALCULATIONS
    // =========================================================================

    // 4A. Read pointer calculation
    always_comb begin
        if (wr_ptr >= delay_samples) 
            rd_ptr_next = wr_ptr - delay_samples;
        else                         
            rd_ptr_next = (RAM_DEPTH + wr_ptr) - delay_samples;
    end

    // 4B. Tone filter next state calculation
    logic signed [WIDTH+10:0] lpf_diff;
    logic signed [WIDTH+10:0] lpf_product;
    logic signed [WIDTH+10:0] lpf_increment;

    always_comb begin
        lpf_diff      = $signed(ram_read_data) - $signed(lpf_state);
        lpf_product   = lpf_diff * tone_coeff;
        lpf_increment = lpf_product[WIDTH+10:8];
        lpf_state_next = lpf_state + lpf_increment;
    end

    // 4C. Feedback path calculation (RAM write data)
    logic signed [WIDTH+10:0] fb_wet;
    logic signed [WIDTH+10:0] fb_calc;
    logic signed [WIDTH-1:0]  fb_saturated;

    always_comb begin
        fb_wet  = ($signed(lpf_state) * fb_gain) >>> 8;
        fb_calc = $signed(audio_in) + fb_wet;

        // Saturation
        if      (fb_calc >  (2**(WIDTH-1)-1))  fb_saturated = (2**(WIDTH-1)-1);
        else if (fb_calc < -(2**(WIDTH-1)))    fb_saturated = -(2**(WIDTH-1));
        else                                   fb_saturated = fb_calc[WIDTH-1:0];
        
        ram_write_data_next = fb_saturated;
    end

    // 4D. Output path calculation (audio out)
    logic signed [WIDTH+10:0] out_wet;
    logic signed [WIDTH+10:0] out_calc;
    logic signed [WIDTH-1:0]  out_saturated;

    always_comb begin
        out_wet  = ($signed(lpf_state) * dry_gain) >>> 8;
        out_calc = $signed(audio_in) + out_wet;

        // Saturation
        if      (out_calc >  (2**(WIDTH-1)-1))  out_saturated = (2**(WIDTH-1)-1);
        else if (out_calc < -(2**(WIDTH-1)))    out_saturated = -(2**(WIDTH-1));
        else                                    out_saturated = out_calc[WIDTH-1:0];
    end

    // =========================================================================
    // 5. REGISTERED OUTPUTS
    // =========================================================================
    logic signed [WIDTH-1:0] audio_out_reg = 0;
    assign audio_out = audio_out_reg;

    // =========================================================================
    // 6. SEQUENTIAL LOGIC (RAM + STATE UPDATES)
    // =========================================================================
    integer rst_idx;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            // Reset all pointers and state
            wr_ptr         <= 0;
            rd_ptr         <= 0;
            lpf_state      <= 0;
            audio_out_reg  <= 0;
            ram_read_data  <= 0;
            
            // Clear all RAM contents during reset
            for (rst_idx = 0; rst_idx < RAM_DEPTH; rst_idx = rst_idx + 1) begin
                ram[rst_idx] <= 0;
            end
            
        end else if (sample_en) begin

            // --- A. MEMORY READ ---
            // Read from current rd_ptr (calculated in previous cycle)
            ram_read_data <= ram[rd_ptr];

            // --- B. MEMORY WRITE ---
            // Write the feedback value calculated from previous cycle's state
            ram[wr_ptr] <= ram_write_data_next;

            // --- C. POINTER UPDATES ---
            // Advance write pointer
            if (wr_ptr == RAM_DEPTH - 1) 
                wr_ptr <= 0;
            else                         
                wr_ptr <= wr_ptr + 1;

            // Update read pointer using combinatorial calculation
            rd_ptr <= rd_ptr_next;

            // --- D. TONE FILTER STATE UPDATE ---
            // Update filter state using combinatorial calculation
            lpf_state <= lpf_state_next;

            // --- E. AUDIO OUTPUT UPDATE ---
            // Register the output using combinatorial calculation
            audio_out_reg <= out_saturated;

        end
    end

endmodule