`timescale 1ns / 1ps

module tb_uart_system_fast;

    // --- FAST SIMULATION SETTINGS ---
    parameter CLK_FREQ      = 100000000;
    
    // We increase Baud Rate to 10 MHz (High Speed) just for simulation!
    parameter BAUD_RATE     = 10000000; 
    
    // Recalculate bit period (100ns per bit instead of 8680ns)
    parameter BIT_PERIOD_NS = 1000000000 / BAUD_RATE; 

    // Signals
    logic       clk = 0;
    logic       ck_rst = 1;
    logic       uart_txd_in = 1;
    logic       uart_rxd_out;
    logic [3:0] led;

    // --- Instantiate with OVERRIDE ---
    uart_cmd_top #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(BAUD_RATE) // <--- Force 10Mbps here
    ) uut (
        .CLK100MHZ(clk),
        .ck_rst(ck_rst),
        .uart_txd_in(uart_txd_in),
        .uart_rxd_out(uart_rxd_out),
        .led(led)
    );

    always #5 clk = ~clk;

    // --- Same Tasks as before (Timing adapts automatically) ---
    task send_byte(input [7:0] data);
        integer i;
        begin
            uart_txd_in = 0;             #(BIT_PERIOD_NS); // Start
            for (i=0; i<8; i++) begin
                uart_txd_in = data[i];   #(BIT_PERIOD_NS); // Data
            end
            uart_txd_in = 1;             #(BIT_PERIOD_NS); // Stop
            #(BIT_PERIOD_NS * 2); 
        end
    endtask

    // --- Main Test ---
    initial begin
        $display("--- FAST Simulation Start ---");
        ck_rst = 0; #100; ck_rst = 1; #100;

        // Send "ON" + Enter
        send_byte("O");
        send_byte("N");
        send_byte(8'h0D);

        // Wait a short time (Since baud is fast, we don't need to wait long)
        #5000; 

        if (led == 4'b1111) $display("SUCCESS: LEDs ON");
        else $error("FAIL: LEDs %b", led);

        $display("--- FAST Simulation End ---");
        $finish;
    end
endmodule