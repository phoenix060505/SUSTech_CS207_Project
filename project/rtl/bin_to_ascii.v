`timescale 1ns / 1ps

module bin_to_ascii (
    input wire clk,
    input wire rst,
    input wire start,
    input wire [15:0] value,
    input wire is_signed,
    output reg tx_start,
    output reg [7:0] tx_data,
    input wire tx_busy,
    output reg done
);

    // FSM states
    localparam S_IDLE          = 5'd0;
    localparam S_PREP          = 5'd1;
    localparam S_TENK          = 5'd2;
    localparam S_THOUSANDS     = 5'd3;
    localparam S_HUNDREDS      = 5'd4;
    localparam S_TENS          = 5'd5;
    localparam S_ONES          = 5'd6;
    localparam S_DISPATCH      = 5'd7;
    localparam S_SEND_SIGN     = 5'd8;
    localparam S_POST_SIGN     = 5'd9;
    localparam S_NEXT_DIGIT    = 5'd10;
    localparam S_AFTER_DIGIT   = 5'd11;
    localparam S_WAIT_BUSY_HI  = 5'd12;
    localparam S_WAIT_BUSY_LO  = 5'd13;
    localparam S_DONE          = 5'd14;

    reg [4:0] state;
    reg [4:0] next_state;

    reg [15:0] work_val;
    reg [15:0] abs_val;
    reg sign_pending;

    reg [3:0] digit_10k;
    reg [3:0] digit_1k;
    reg [3:0] digit_100;
    reg [3:0] digit_10;
    reg [3:0] digit_1;

    reg [2:0] digit_phase;
    reg printed_digit;

    localparam DIG_10K = 3'd0;
    localparam DIG_1K  = 3'd1;
    localparam DIG_100 = 3'd2;
    localparam DIG_10  = 3'd3;
    localparam DIG_1   = 3'd4;

    function [3:0] current_digit;
        input [2:0] phase;
        begin
            case (phase)
                DIG_10K: current_digit = digit_10k;
                DIG_1K : current_digit = digit_1k;
                DIG_100: current_digit = digit_100;
                DIG_10 : current_digit = digit_10;
                default: current_digit = digit_1;
            endcase
        end
    endfunction

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            tx_start <= 1'b0;
            tx_data <= 8'h00;
            done <= 1'b0;
            work_val <= 16'h0000;
            abs_val <= 16'h0000;
            sign_pending <= 1'b0;
            digit_10k <= 4'd0;
            digit_1k <= 4'd0;
            digit_100 <= 4'd0;
            digit_10 <= 4'd0;
            digit_1 <= 4'd0;
            digit_phase <= DIG_10K;
            printed_digit <= 1'b0;
        end else begin
            tx_start <= 1'b0;
            done <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        if (is_signed && value[15]) begin
                            abs_val <= (~value) + 1'b1;
                            sign_pending <= 1'b1;
                        end else begin
                            abs_val <= value;
                            sign_pending <= 1'b0;
                        end
                        digit_10k <= 4'd0;
                        digit_1k <= 4'd0;
                        digit_100 <= 4'd0;
                        digit_10 <= 4'd0;
                        digit_1 <= 4'd0;
                        work_val <= 16'h0000;
                        printed_digit <= 1'b0;
                        state <= S_PREP;
                    end
                end

                S_PREP: begin
                    work_val <= abs_val;
                    state <= S_TENK;
                end

                S_TENK: begin
                    if (work_val >= 16'd10000) begin
                        work_val <= work_val - 16'd10000;
                        digit_10k <= digit_10k + 1'b1;
                    end else begin
                        state <= S_THOUSANDS;
                    end
                end

                S_THOUSANDS: begin
                    if (work_val >= 16'd1000) begin
                        work_val <= work_val - 16'd1000;
                        digit_1k <= digit_1k + 1'b1;
                    end else begin
                        state <= S_HUNDREDS;
                    end
                end

                S_HUNDREDS: begin
                    if (work_val >= 16'd100) begin
                        work_val <= work_val - 16'd100;
                        digit_100 <= digit_100 + 1'b1;
                    end else begin
                        state <= S_TENS;
                    end
                end

                S_TENS: begin
                    if (work_val >= 16'd10) begin
                        work_val <= work_val - 16'd10;
                        digit_10 <= digit_10 + 1'b1;
                    end else begin
                        state <= S_ONES;
                    end
                end

                S_ONES: begin
                    digit_1 <= work_val[3:0];
                    state <= S_DISPATCH;
                end

                S_DISPATCH: begin
                    digit_phase <= DIG_10K;
                    printed_digit <= 1'b0;
                    if (sign_pending)
                        state <= S_SEND_SIGN;
                    else
                        state <= S_NEXT_DIGIT;
                end

                S_SEND_SIGN: begin
                    if (!tx_busy) begin
                        tx_data <= 8'h2D; // '-'
                        tx_start <= 1'b1;
                        next_state <= S_POST_SIGN;
                        state <= S_WAIT_BUSY_HI;
                        sign_pending <= 1'b0;
                    end
                end

                S_POST_SIGN: begin
                    state <= S_NEXT_DIGIT;
                end

                S_NEXT_DIGIT: begin
                    if (digit_phase <= DIG_1) begin
                        if (!printed_digit && current_digit(digit_phase) == 0 && digit_phase != DIG_1) begin
                            digit_phase <= digit_phase + 1'b1;
                        end else if (!tx_busy) begin
                            tx_data <= 8'h30 | {4'b0000, current_digit(digit_phase)};
                            tx_start <= 1'b1;
                            next_state <= S_AFTER_DIGIT;
                            state <= S_WAIT_BUSY_HI;
                            printed_digit <= 1'b1;
                        end
                    end else begin
                        state <= S_DONE;
                    end
                end

                S_AFTER_DIGIT: begin
                    if (digit_phase == DIG_1)
                        state <= S_DONE;
                    else begin
                        digit_phase <= digit_phase + 1'b1;
                        state <= S_NEXT_DIGIT;
                    end
                end

                S_WAIT_BUSY_HI: begin
                    if (tx_busy)
                        state <= S_WAIT_BUSY_LO;
                end

                S_WAIT_BUSY_LO: begin
                    if (!tx_busy)
                        state <= next_state;
                end

                S_DONE: begin
                    done <= 1'b1;
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
