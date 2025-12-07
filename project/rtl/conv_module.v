`timescale 1ns / 1ps

module conv_module (
    input wire clk,
    input wire rst,
    input wire start,
    input wire [199:0] kernel_flat, 
    input wire tx_busy,
    
    output wire tx_start,
    output wire [7:0] tx_data,
    output reg busy,                 
    output reg done,
    output reg [15:0] cycle_cnt
);

    //==============================================================
    // 1. Input Image ROM (Hardcoded as per requirements)
    //==============================================================
    wire [3:0] rom_x; // Row Address
    wire [3:0] rom_y; // Col Address
    wire [3:0] rom_data_out;

    input_image_rom img_rom (
        .clk(clk),
        .x(rom_x),
        .y(rom_y),
        .data_out(rom_data_out)
    );

    //==============================================================
    // 2. Convolution Logic
    //==============================================================
    // Input Image: 10x12, Kernel: 3x3, Stride: 1 => Output: (10-3+1)x(12-3+1) = 8x10
    
    // State Definition
    localparam S_IDLE = 0;
    localparam S_LOAD_KERNEL = 1; // Parse Kernel
    localparam S_CALC_PIXEL = 2;  // Calculate single pixel (Accumulate 9 mults)
    localparam S_SEND_RESULT = 3; // Launch ASCII output
    localparam S_WAIT_ASCII = 4;
    localparam S_NEXT_PIXEL = 5;  // Move to next pixel
    localparam S_DONE = 6;

    reg [3:0] state;
    
    // Kernel Storage
    reg [7:0] kernel [0:2][0:2];
    
    // Loop Counters
    reg [3:0] out_row; // 0-7
    reg [3:0] out_col; // 0-9
    reg [3:0] k_r;     // 0-2
    reg [3:0] k_c;     // 0-2
    
    // Accumulator
    reg [15:0] acc; 

    // ROM Address Control
    // Current Image Coord = (out_row + k_r, out_col + k_c)
    assign rom_x = out_row + k_r;
    assign rom_y = out_col + k_c;

    // ROM Latency Handling State Machine
    reg rom_wait;

    // ASCII output helper
    reg launch_ascii;
    reg [15:0] value_buf;
    reg [7:0] tail_char;
    wire ascii_done;

    value_ascii_tx printer (
        .clk(clk),
        .rst(rst),
        .start(launch_ascii),
        .value(value_buf),
        .tail_char_en(1'b1),
        .tail_char(tail_char),
        .tx_busy(tx_busy),
        .tx_start(tx_start),
        .tx_data(tx_data),
        .busy(),
        .done(ascii_done)
    );
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            out_row <= 4'd0;
            out_col <= 4'd0;
            k_r <= 4'd0;
            k_c <= 4'd0;
            acc <= 16'd0;
            rom_wait <= 1'b0;
            cycle_cnt <= 16'd0;
            launch_ascii <= 1'b0;
            value_buf <= 16'd0;
            tail_char <= 8'h20;
        end else begin
            if (busy) cycle_cnt <= cycle_cnt + 1'b1;
            launch_ascii <= 1'b0;
            done <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        busy <= 1'b1;
                        out_row <= 4'd0;
                        out_col <= 4'd0;
                        k_r <= 4'd0;
                        k_c <= 4'd0;
                        acc <= 16'd0;
                        rom_wait <= 1'b1;
                        state <= S_LOAD_KERNEL;
                    end
                end

                S_LOAD_KERNEL: begin
                    kernel[0][0] <= kernel_flat[7:0];
                    kernel[0][1] <= kernel_flat[15:8];
                    kernel[0][2] <= kernel_flat[23:16];
                    kernel[1][0] <= kernel_flat[31:24];
                    kernel[1][1] <= kernel_flat[39:32];
                    kernel[1][2] <= kernel_flat[47:40];
                    kernel[2][0] <= kernel_flat[55:48];
                    kernel[2][1] <= kernel_flat[63:56];
                    kernel[2][2] <= kernel_flat[71:64];
                    acc <= 16'd0;
                    k_r <= 4'd0;
                    k_c <= 4'd0;
                    rom_wait <= 1'b1;
                    state <= S_CALC_PIXEL;
                end

                S_CALC_PIXEL: begin
                    if (rom_wait) begin
                        rom_wait <= 1'b0;
                    end else begin
                        acc <= acc + rom_data_out * kernel[k_r][k_c];
                        rom_wait <= 1'b1;
                        if (k_c == 4'd2) begin
                            k_c <= 4'd0;
                            if (k_r == 4'd2) begin
                                state <= S_SEND_RESULT;
                            end else begin
                                k_r <= k_r + 1'b1;
                            end
                        end else begin
                            k_c <= k_c + 1'b1;
                        end
                    end
                end

                S_SEND_RESULT: begin
                    value_buf <= acc;
                    tail_char <= (out_col == 4'd9) ? 8'h0A : 8'h20;
                    launch_ascii <= 1'b1;
                    state <= S_WAIT_ASCII;
                end

                S_WAIT_ASCII: begin
                    if (ascii_done)
                        state <= S_NEXT_PIXEL;
                end

                S_NEXT_PIXEL: begin
                    if (out_col == 4'd9) begin
                        out_col <= 4'd0;
                        if (out_row == 4'd7) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            state <= S_DONE;
                        end else begin
                            out_row <= out_row + 1'b1;
                            acc <= 16'd0;
                            k_r <= 4'd0;
                            k_c <= 4'd0;
                            rom_wait <= 1'b1;
                            state <= S_CALC_PIXEL;
                        end
                    end else begin
                        out_col <= out_col + 1'b1;
                        acc <= 16'd0;
                        k_r <= 4'd0;
                        k_c <= 4'd0;
                        rom_wait <= 1'b1;
                        state <= S_CALC_PIXEL;
                    end
                end

                S_DONE: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule

