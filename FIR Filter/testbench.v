`timescale 1ns / 10ps

module fir_tb;

    reg clk, rst;
    reg signed [15:0] input_sample;
    wire signed [15:0] output_sample;

    // Instantiate the FIR filter
    fir dut (
        .clk(clk),
        .rst(rst),
        .input_sample(input_sample),
        .output_sample(output_sample)
    );

    // Clock generation
    initial begin
        rst = 0;
        clk = 0;
    end

    always #5 clk = ~clk;

    // Test stimulus
    initial begin
        // Reset
        rst = 0;
        #20; // Keep reset active for a while
        rst = 1;

        // Apply input samples and simulate
        input_sample = 16'h1234; // Input sample 1
        #50;
        input_sample = 16'h5678; // Input sample 2
        #50;
        // Add more input samples as needed

        // Finish simulation
        #100;
        $finish;
    end
endmodule

