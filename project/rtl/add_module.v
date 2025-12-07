`timescale 1ns / 1ps

module add_module #(
    parameter ELEM_WIDTH = 8, 
    parameter MAX_DIM = 5
) (
    input wire clk,
    input wire rst,
    input wire start,
    input wire [3:0] m,
    input wire [3:0] n,
    input wire [199:0] in_a,
    input wire [199:0] in_b,
    input wire tx_busy,
    
    output wire tx_start,
    output wire [7:0] tx_data,
    output reg done
);

    reg [3:0] i, j;
    reg busy;
    reg launch_ascii;
    reg [15:0] value_buf;
    reg [7:0] tail_char;

    wire ascii_done;

    wire [7:0] mat_a [0:24];
    wire [7:0] mat_b [0:24];
    
    genvar k;
    generate
        for (k=0; k<25; k=k+1) begin : unpack
            assign mat_a[k] = in_a[k*8 +: 8];
            assign mat_b[k] = in_b[k*8 +: 8];
        end
    endgenerate

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

    localparam S_IDLE   = 3'd0;
    localparam S_PREP   = 3'd1;
    localparam S_ASCII  = 3'd2;
    localparam S_NEXT   = 3'd3;
    localparam S_FINISH = 3'd4;

    reg [2:0] state;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            i <= 4'd0;
            j <= 4'd0;
            launch_ascii <= 1'b0;
            value_buf <= 16'd0;
            tail_char <= 8'h20;
        end else begin
            launch_ascii <= 1'b0;
            done <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start && !busy) begin
                        busy <= 1'b1;
                        i <= 4'd0;
                        j <= 4'd0;
                        state <= S_PREP;
                    end
                end

                S_PREP: begin
                    value_buf <= mat_a[i * n + j] + mat_b[i * n + j];
                    tail_char <= ((n == 0) || (j == n - 1)) ? 8'h0A : 8'h20;
                    launch_ascii <= 1'b1;
                    state <= S_ASCII;
                end

                S_ASCII: begin
                    if (ascii_done)
                        state <= S_NEXT;
                end

                S_NEXT: begin
                    if (j == n - 1) begin
                        j <= 0;
                        if (i == m - 1) begin
                            state <= S_FINISH;
                        end else begin
                            i <= i + 1'b1;
                            state <= S_PREP;
                        end
                    end else begin
                        j <= j + 1'b1;
                        state <= S_PREP;
                    end
                end

                S_FINISH: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
