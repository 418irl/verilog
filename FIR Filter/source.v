module fir(
    input clk,
    input rst,
    input signed [15:0] input_sample,
    output reg signed [15:0] output_sample);

    // Coefficients for the filter as separate parameters
    parameter signed [15:0] coeff0 = 16'h1000;
    parameter signed [15:0] coeff1 = 16'h2000;
    parameter signed [15:0] coeff2 = 16'h3000;
    parameter signed [15:0] coeff3 = 16'h1000;

    // 4 registers for 4 samples of 16bit each
    reg signed [15:0] shift_reg [0:3]; 
    reg signed [31:0] accumulator;     // Output add

    integer i;

    always @(posedge clk or negedge rst) 
    begin
        if (!rst) 
        begin
            // Reset accumulator and shift register
            accumulator <= 0;
            for (i = 0; i < 4; i = i + 1)
                shift_reg[i] <= 0;
        end 
        else 
        begin
            shift_reg[0] <= input_sample; // input to shift0
            accumulator <= 0;

         // Multiply and add using individual coefficient parameters
            accumulator = accumulator + shift_reg[0] * coeff0;
            accumulator = accumulator + shift_reg[1] * coeff1;
            accumulator = accumulator + shift_reg[2] * coeff2;
            accumulator = accumulator + shift_reg[3] * coeff3;

         // Shift samples
            for (i = 3; i > 0; i = i - 1)
                shift_reg[i] <= shift_reg[i - 1];

         // Output result
            output_sample <= accumulator[31:16];
        end
    end
endmodule
