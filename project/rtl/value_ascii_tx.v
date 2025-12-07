`timescale 1ns / 1ps

module value_ascii_tx (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire [15:0] value,
    input  wire        tail_char_en,
    input  wire [7:0]  tail_char,
    input  wire        tx_busy,
    output wire        tx_start,
    output wire [7:0]  tx_data,
    output reg         busy,
    output reg         done
);

    // Internal signals to share UART line between bin_to_ascii and manual char
    reg        use_b2a;
    reg        manual_tx_start;
    reg [7:0]  manual_tx_data;

    wire       b2a_tx_start;
    wire [7:0] b2a_tx_data;
    reg        b2a_start;
    wire       b2a_done;

    assign tx_start = (use_b2a) ? b2a_tx_start : manual_tx_start;
    assign tx_data  = (use_b2a) ? b2a_tx_data  : manual_tx_data;

    bin_to_ascii b2a_inst (
        .clk(clk),
        .rst(rst),
        .start(b2a_start),
        .value(value),
        .is_signed(1'b0),
        .tx_start(b2a_tx_start),
        .tx_data(b2a_tx_data),
        .tx_busy(tx_busy),
        .done(b2a_done)
    );

    localparam S_IDLE        = 3'd0;
    localparam S_ASCII       = 3'd1;
    localparam S_TAIL        = 3'd2;
    localparam S_TAIL_WAIT_H = 3'd3;
    localparam S_TAIL_WAIT_L = 3'd4;
    localparam S_DONE        = 3'd5;

    reg [2:0] state;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state            <= S_IDLE;
            use_b2a          <= 1'b0;
            manual_tx_start  <= 1'b0;
            manual_tx_data   <= 8'h00;
            b2a_start        <= 1'b0;
            busy             <= 1'b0;
            done             <= 1'b0;
        end else begin
            manual_tx_start <= 1'b0;
            b2a_start       <= 1'b0;
            done            <= 1'b0;

            case (state)
                S_IDLE: begin
                    use_b2a <= 1'b0;
                    busy    <= 1'b0;
                    if (start) begin
                        busy      <= 1'b1;
                        use_b2a   <= 1'b1;
                        b2a_start <= 1'b1;
                        state     <= S_ASCII;
                    end
                end

                S_ASCII: begin
                    if (b2a_done) begin
                        use_b2a <= 1'b0;
                        if (tail_char_en)
                            state <= S_TAIL;
                        else
                            state <= S_DONE;
                    end
                end

                S_TAIL: begin
                    if (!tx_busy) begin
                        manual_tx_data  <= tail_char;
                        manual_tx_start <= 1'b1;
                        state           <= S_TAIL_WAIT_H;
                    end
                end

                S_TAIL_WAIT_H: begin
                    if (tx_busy)
                        state <= S_TAIL_WAIT_L;
                end

                S_TAIL_WAIT_L: begin
                    if (!tx_busy)
                        state <= S_DONE;
                end

                S_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
