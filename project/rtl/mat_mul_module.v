`timescale 1ns / 1ps

module mat_mul_module #(
    parameter ELEM_WIDTH = 8, 
    parameter MAX_DIM = 5
) (
    input wire clk,
    input wire rst,
    input wire start,
    input wire [3:0] m,
    input wire [3:0] n,
    input wire [3:0] p,
    input wire [199:0] in_a,
    input wire [199:0] in_b,
    input wire tx_busy,
    
    output wire tx_start,
    output wire [7:0] tx_data,
    output reg done
);

    reg [3:0] i, j, k_idx;
    reg busy;
    reg [15:0] sum_temp;
    reg [7:0] ptr_a, ptr_b;
    reg [7:0] row_start_a;
    reg [7:0] col_start_b;
    reg [7:0] op_a, op_b;
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
    localparam S_LOAD   = 3'd2;
    localparam S_ACC    = 3'd3;
    localparam S_ASCII  = 3'd4;
    localparam S_NEXT   = 3'd5;

    reg [2:0] state;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            i <= 4'd0;
            j <= 4'd0;
            k_idx <= 4'd0;
            sum_temp <= 16'd0;
            ptr_a <= 8'd0;
            ptr_b <= 8'd0;
            row_start_a <= 8'd0;
            col_start_b <= 8'd0;
            op_a <= 8'd0;
            op_b <= 8'd0;
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
                        row_start_a <= 8'd0;
                        col_start_b <= 8'd0;
                        state <= S_PREP;
                    end
                end

                S_PREP: begin
                    ptr_a <= row_start_a;
                    ptr_b <= col_start_b;
                    k_idx <= 4'd0;
                    sum_temp <= 16'd0;
                    state <= S_LOAD;
                end

                S_LOAD: begin
                    op_a <= mat_a[ptr_a];
                    op_b <= mat_b[ptr_b];
                    state <= S_ACC;
                end

                S_ACC: begin
                    sum_temp <= sum_temp + op_a * op_b;
                    if (k_idx == n - 1) begin
                        value_buf <= sum_temp + op_a * op_b;
                        tail_char <= ((p == 0) || (j == p - 1)) ? 8'h0A : 8'h20;
                        launch_ascii <= 1'b1;
                        state <= S_ASCII;
                    end else begin
                        k_idx <= k_idx + 1'b1;
                        ptr_a <= ptr_a + 1'b1;
                        ptr_b <= ptr_b + p;
                        state <= S_LOAD;
                    end
                end

                S_ASCII: begin
                    if (ascii_done)
                        state <= S_NEXT;
                end

                S_NEXT: begin
                    if (j == p - 1) begin
                        j <= 0;
                        col_start_b <= 0;
                        if (i == m - 1) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            state <= S_IDLE;
                        end else begin
                            i <= i + 1'b1;
                            row_start_a <= row_start_a + n;
                            state <= S_PREP;
                        end
                    end else begin
                        j <= j + 1'b1;
                        col_start_b <= col_start_b + 1'b1;
                        state <= S_PREP;
                    end
                end
            endcase
        end
    end

endmodule
