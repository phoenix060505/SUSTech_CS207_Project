`timescale 1ns / 1ps

module input_image_rom (
    input wire clk,
    input wire [3:0] x, // Row 0-9
    input wire [3:0] y, // Col 0-11
    output reg [3:0] data_out
);

    // 10x12 Image
    // Simple pattern: data = (x + y) % 16
    always @(posedge clk) begin
        if (x < 10 && y < 12)
            data_out <= (x + y) & 4'hF;
        else
            data_out <= 0;
    end

endmodule
