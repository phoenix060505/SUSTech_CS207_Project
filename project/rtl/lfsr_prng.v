`timescale 1ns / 1ps

module lfsr_prng (
    input wire clk,
    input wire rst,
    input wire enable,
    output wire [7:0] rand_out,
    output wire [7:0] rand_raw
);

    reg [15:0] lfsr_reg;

    // Polynomial: x^16 + x^14 + x^13 + x^11 + 1
    wire feedback = lfsr_reg[15] ^ lfsr_reg[13] ^ lfsr_reg[12] ^ lfsr_reg[10];

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            lfsr_reg <= 16'hACE1; // Non-zero seed
        end else if (enable) begin
            lfsr_reg <= {lfsr_reg[14:0], feedback};
        end
    end

    assign rand_raw = lfsr_reg[7:0];

    // Map to 0-9
    wire [3:0] raw = lfsr_reg[3:0];
    assign rand_out = (raw >= 10) ? (raw - 10) : raw;

endmodule
