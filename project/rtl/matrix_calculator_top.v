`timescale 1ns / 1ps
// Top Module: Matrix Calculator System Integration
// Includes FSM control, data storage, computation units, and IO interface drivers
module matrix_calculator_top #(
    parameter MAX_DIM = 5,           // Max matrix dimension
    parameter MAX_STORE = 2,         // Max storage per dimension
    parameter ELEM_WIDTH = 8,        // Element width
    parameter TIMEOUT_DEFAULT = 10,  // Default timeout (seconds)
    parameter CLK_FREQ = 100_000_000 // System clock frequency
) (
    input wire clk,
    input wire rst,                  // Active high reset
    input wire [7:0] sw,             // sw[7:4] reserved, sw[3:0] reused: Matrix Index/Scalar/Config
    input wire [3:0] btn,            // btn[0]: Confirm/Start
    input wire uart_rx,              // UART RX
    output reg [7:0] led,            // led[0]: Error, led[1]: Busy, Others: Status
    output reg [7:0] seg,            // 7-segment segment selection (Common Anode)
    output reg [3:0] an,             // 7-segment digit selection
    output wire uart_tx              // UART TX
);

    //==============================================================
    // 1. Internal Signal Definition
    //==============================================================
    
    wire rst_n = ~rst;

    // --- FSM Control Signals ---
    wire [1:0] mode;            // 00:Input, 01:Gen, 10:Display, 11:Calc
    wire [3:0] op_type;         // 0:Transpose, 1:Add, 2:Scalar Mul, 3:Mat Mul, 5:Conv
    wire timeout_en;            // Timeout enable
    wire wen_store;             // Storage write enable
    wire start_compute;         // Start computation pulse
    wire update_config;         // Bonus: Update config register enable (from FSM)

    // --- Status Feedback Signals ---
    wire uart_rx_done;
    wire [7:0] uart_rx_data;
    wire input_done;
    wire display_done;
    wire store_error;           // Storage error signal
    reg  compute_done;          // Aggregated computation done signal
    reg  operand_legal;         // Aggregated operand legality check
    reg  timeout_expired;       // Timeout expired flag
    reg  error_flag;            // Error flag

    // --- Debounce Signals ---
    wire btn_confirm_pulse;

    // --- Data Path: Matrix Data and Dimensions ---
    // Source matrices A and B read from storage
    // Flattened array for Verilog-2001 compatibility: [8*25-1 : 0] = [199:0]
    wire [199:0] mat_a_wire;
    wire [199:0] mat_b_wire;
    
    // Dimensions of source matrices
    wire [3:0] dim_m_a, dim_n_a; // Matrix A dimensions (Rows, Cols)
    wire [3:0] dim_m_b, dim_n_b; // Matrix B dimensions (Rows, Cols)
    
    // Source operand index / Scalar value / Config value
    wire [MAX_STORE-1:0] sel_idx_a = sw[3:2]; // A Matrix Index
    wire [MAX_STORE-1:0] sel_idx_b = sw[1:0]; // B Matrix Index
    wire [3:0] scalar_val = sw[3:0];          // Scalar multiplier

    // --- Dynamic Parameter Configuration (Bonus) ---
    reg [3:0] cfg_timeout_val;  // Configurable timeout duration

    // --- UART TX Arbitration ---
    reg [7:0] tx_data_mux;
    reg tx_start_mux;
    wire tx_busy;

    // Sub-module TX interfaces
    wire [7:0] tx_disp_data, tx_trans_data, tx_add_data, tx_smul_data, tx_mmul_data, tx_conv_data;
    wire tx_disp_start, tx_trans_start, tx_add_start, tx_smul_start, tx_mmul_start, tx_conv_start;
    
    // Sub-module Done signals
    wire done_trans, done_add, done_smul, done_mmul, done_conv;

    //==============================================================
    // 2. Basic Modules and Controller Instantiation
    //==============================================================

    // Auto-calculate debounce counter threshold: 20ms
    // If CLK_FREQ=100MHz -> 2,000,000
    // If CLK_FREQ=100Hz  -> 2
    localparam DEBOUNCE_CNT = CLK_FREQ / 50; 

    // Instantiate Debounce Module (for btn[0] Confirm)
    debounce #(.CNT_MAX(DEBOUNCE_CNT)) btn0_debounce (
        .clk(clk),
        .rst(rst),
        .btn_in(btn[0]),
        .btn_out(btn_confirm_pulse)
    );

    wire [1:0] calc_step;
    wire [4:0] fsm_state; // Added FSM state monitoring
    reg [7:0] id_a, id_b;
    reg [3:0] filter_m, filter_n;

    // Capture IDs and Filter Dimensions
    always @(posedge clk) begin
        if (rst) begin
            id_a <= 0;
            id_b <= 0;
            filter_m <= 0;
            filter_n <= 0;
        end else if (btn_confirm_pulse) begin
            // Capture Filter Dimensions
            if (fsm_state == 5'd10) filter_m <= sw[3:0]; // S_CALC_A_M
            if (fsm_state == 5'd11) filter_n <= sw[3:0]; // S_CALC_A_N
            if (fsm_state == 5'd14) filter_m <= sw[3:0]; // S_CALC_B_M
            if (fsm_state == 5'd15) filter_n <= sw[3:0]; // S_CALC_B_N
            
            // Capture IDs
            if (fsm_state == 5'd13) id_a <= sw; // S_CALC_A_ID
            if (fsm_state == 5'd17) id_b <= sw; // S_CALC_B_ID
        end
    end

    fsm_controller fsm_inst (
        .clk(clk), .rst(rst),
        .btn({btn[3:1], btn_confirm_pulse}), // Connect debounced pulse to btn[0]
        .sw(sw),
        .input_done(input_done),
        .display_done(display_done),
        .operand_legal(operand_legal),
        .compute_done(compute_done),
        .timeout_expired(timeout_expired),
        // Outputs
        .mode(mode),
        .op_type(op_type),
        .timeout_en(timeout_en),
        .wen_store(wen_store),
        .start_compute(start_compute),
        .update_config(update_config),
        .calc_step(calc_step),
        .fsm_state_out(fsm_state) // Connect new port
    );

    uart_rx #(.CLK_FREQ(CLK_FREQ), .UART_BPS(115200)) rx_inst (
        .clk(clk), .rst_n(rst_n),
        .uart_rxd(uart_rx),
        .uart_rx_data(uart_rx_data),
        .uart_rx_done(uart_rx_done)
    );

    uart_tx #(.CLK_FREQ(CLK_FREQ), .UART_BPS(115200)) tx_inst (
        .clk(clk), .rst_n(rst_n),
        .uart_tx_en(tx_start_mux),
        .uart_tx_data(tx_data_mux),
        .uart_txd(uart_tx),
        .uart_tx_busy(tx_busy)
    );

    matrix_storage #(.MAX_DIM(MAX_DIM), .MAX_STORE(MAX_STORE)) store_inst (
        .clk(clk), .rst(rst),
        .wen(wen_store && uart_rx_done),
        .flush(btn_confirm_pulse), // Added for manual finish
        .mode(mode), 
        .data_in(uart_rx_data),
        .read_id_a(id_a),
        .read_id_b(id_b),
        // Outputs
        .mat_a_out(mat_a_wire),
        .mat_b_out(mat_b_wire),
        .dim_a_m(dim_m_a), .dim_a_n(dim_n_a), // Output A dimensions
        .dim_b_m(dim_m_b), .dim_b_n(dim_n_b), // Output B dimensions
        .input_done(input_done),
        .error_flag(store_error), // Connect error signal
        // Display Interface
        .display_start(mode == 2'b10),
        .tx_busy(tx_busy), // Added
        .filter_en(fsm_state == 5'd12 || fsm_state == 5'd16), // Enable filter during LIST states
        .filter_m(filter_m),
        .filter_n(filter_n),
        .tx_start(tx_disp_start),
        .tx_data(tx_disp_data),
        .display_done(display_done)
    );

    //==============================================================
    // 3. Computation Modules Instantiation
    //==============================================================

    // 1. Transpose (Op: 0) - Only involves Matrix A
    transpose_module trans_inst (
        .clk(clk), .rst(rst),
        .start(start_compute && op_type == 4'd0),
        .m(dim_m_a), .n(dim_n_a),
        .in_matrix(mat_a_wire),
        .tx_busy(tx_busy),
        .tx_start(tx_trans_start),
        .tx_data(tx_trans_data),
        .done(done_trans)
    );

    // 2. Addition (Op: 1) - A + B
    add_module add_inst (
        .clk(clk), .rst(rst),
        .start(start_compute && op_type == 4'd1),
        .m(dim_m_a), .n(dim_n_a), // Use A's dimensions if legal
        .in_a(mat_a_wire),
        .in_b(mat_b_wire),
        .tx_busy(tx_busy),
        .tx_start(tx_add_start),
        .tx_data(tx_add_data),
        .done(done_add)
    );

    // 3. Scalar Multiplication (Op: 2) - A * Scalar
    scalar_mul_module smul_inst (
        .clk(clk), .rst(rst),
        .start(start_compute && op_type == 4'd2),
        .m(dim_m_a), .n(dim_n_a),
        .scalar(scalar_val),
        .in_matrix(mat_a_wire),
        .tx_busy(tx_busy),
        .tx_start(tx_smul_start),
        .tx_data(tx_smul_data),
        .done(done_smul)
    );

    // 4. Matrix Multiplication (Op: 3) - A(mxn) * B(nxp)
    mat_mul_module mmul_inst (
        .clk(clk), .rst(rst),
        .start(start_compute && op_type == 4'd3),
        .m(dim_m_a),       // Result rows = A rows
        .n(dim_n_a),       // Middle dim = A cols (must check == B rows)
        .p(dim_n_b),       // Result cols = B cols
        .in_a(mat_a_wire),
        .in_b(mat_b_wire),
        .tx_busy(tx_busy),
        .tx_start(tx_mmul_start),
        .tx_data(tx_mmul_data),
        .done(done_mmul)
    );

    // 5. Convolution (Op: 5) - Bonus
    wire [15:0] conv_cycle_cnt; // Cycle counter from conv module
    conv_module conv_inst (
        .clk(clk), .rst(rst),
        .start(start_compute && op_type == 4'd5),
        .kernel_flat(mat_a_wire), // Use Matrix A as kernel
        .tx_busy(tx_busy),
        .tx_start(tx_conv_start),
        .tx_data(tx_conv_data),
        .busy(),
        .done(done_conv),
        .cycle_cnt(conv_cycle_cnt)
    );

    //==============================================================
    // 4. Combinational Logic: UART Arbitration & Legality Check
    //==============================================================

    // --- UART TX Multiplexing ---
    always @(*) begin
        tx_data_mux  = 8'h00;
        tx_start_mux = 1'b0;
        
        if (mode == 2'b10) begin // Display Mode
            tx_data_mux  = tx_disp_data;
            tx_start_mux = tx_disp_start;
        end 
        else if (mode == 2'b11) begin // Calculation Mode
            case (op_type)
                4'd0: begin tx_data_mux = tx_trans_data; tx_start_mux = tx_trans_start; end
                4'd1: begin tx_data_mux = tx_add_data;   tx_start_mux = tx_add_start;   end
                4'd2: begin tx_data_mux = tx_smul_data;  tx_start_mux = tx_smul_start;  end
                4'd3: begin tx_data_mux = tx_mmul_data;  tx_start_mux = tx_mmul_start;  end
                4'd5: begin tx_data_mux = tx_conv_data;  tx_start_mux = tx_conv_start;  end
                default: ;
            endcase
        end
    end

    // --- Done Signal Aggregation & Legality Check ---
    always @(*) begin
        // 1. Aggregate Done Signals
        compute_done = 1'b0;
        if (mode == 2'b11) begin
            case (op_type)
                4'd0: compute_done = done_trans;
                4'd1: compute_done = done_add;
                4'd2: compute_done = done_smul;
                4'd3: compute_done = done_mmul;
                4'd5: compute_done = done_conv;
                default: compute_done = 1'b1;
            endcase
        end

        // 2. Legality Check Logic
        operand_legal = 1'b1;
        error_flag = 1'b0;
        
        if (mode == 2'b11) begin
            case (op_type)
                4'd1: begin // Addition: Must have same dimensions
                    if (dim_m_a != dim_m_b || dim_n_a != dim_n_b)
                        operand_legal = 1'b0;
                end
                4'd3: begin // Multiplication: A cols must equal B rows
                    if (dim_n_a != dim_m_b)
                        operand_legal = 1'b0;
                end
                default: operand_legal = 1'b1;
            endcase
        end
        
        // Illegal operation sets error flag
        if (!operand_legal) error_flag = 1'b1;
    end

    //==============================================================
    // 5. 7-Segment Decoding Function
    //==============================================================
    // Common Anode (0:On, 1:Off) - Display 0-F
    function [7:0] decode_7seg(input [3:0] num);
        case(num)
            4'h0: decode_7seg = 8'hC0; // 0
            4'h1: decode_7seg = 8'hF9; // 1
            4'h2: decode_7seg = 8'hA4; // 2
            4'h3: decode_7seg = 8'hB0; // 3
            4'h4: decode_7seg = 8'h99; // 4
            4'h5: decode_7seg = 8'h92; // 5
            4'h6: decode_7seg = 8'h82; // 6
            4'h7: decode_7seg = 8'hF8; // 7
            4'h8: decode_7seg = 8'h80; // 8
            4'h9: decode_7seg = 8'h90; // 9
            4'hA: decode_7seg = 8'h88; // A
            4'hB: decode_7seg = 8'h83; // b
            4'hC: decode_7seg = 8'hC6; // C
            4'hD: decode_7seg = 8'hA1; // d
            4'hE: decode_7seg = 8'h86; // E
            4'hF: decode_7seg = 8'h8E; // F
            default: decode_7seg = 8'hFF; // OFF
        endcase
    endfunction

    //==============================================================
    // 6. Sequential Logic: Timeout and Display Driver
    //==============================================================
    reg [31:0] timeout_cnt;
    reg [3:0] timeout_sec;
    reg [16:0] scan_cnt; // Scan counter for 7-segment display

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            timeout_cnt <= 0;
            cfg_timeout_val <= TIMEOUT_DEFAULT; // Reset to default config
            timeout_sec <= TIMEOUT_DEFAULT;
            timeout_expired <= 1'b0;
            led <= 8'b0;
            an <= 4'b1110;
            seg <= 8'hff;
            scan_cnt <= 0;
        end else begin
            // --- Bonus: Dynamic Parameter Update ---
            if (update_config) begin
                // Assume sw[3:0] is new time value in config mode
                cfg_timeout_val <= (sw[3:0] > 4 && sw[3:0] <= 15) ? sw[3:0] : TIMEOUT_DEFAULT;
            end

            // --- Timeout Logic ---
            if (timeout_en) begin
                if (timeout_cnt < CLK_FREQ - 1) begin
                    timeout_cnt <= timeout_cnt + 1;
                end else begin
                    timeout_cnt <= 0;
                    if (timeout_sec > 0)
                        timeout_sec <= timeout_sec - 1;
                    else
                        timeout_expired <= 1'b1;
                end
            end else begin
                timeout_cnt <= 0;
                timeout_sec <= cfg_timeout_val; // Reset using config value
                timeout_expired <= 1'b0;
            end

            // --- LED Display ---
            led[0] <= error_flag | store_error; // LED0: Error (includes storage error)
            led[1] <= tx_busy;        // LED1: UART Busy
            led[2] <= input_done;     // LED2: Input Done
            led[3] <= display_done;   // LED3: Display Done
            led[4] <= compute_done;   // LED4: Compute Done
            led[7:5] <= fsm_state[2:0]; // LED7-5: FSM State (Debug)

            // --- 7-Segment Dynamic Scan ---
            scan_cnt <= scan_cnt + 1;
            if (scan_cnt == 0) begin
                an <= {an[2:0], an[3]};   // Rotate digit selection
            end
            
            if (mode == 2'b11) begin
                if (op_type == 4'd5 && calc_step > 2) begin // Conv Exec
                    case (an)
                        4'b1110: seg <= decode_7seg(conv_cycle_cnt[15:12]);
                        4'b1101: seg <= decode_7seg(conv_cycle_cnt[11:8]);
                        4'b1011: seg <= decode_7seg(conv_cycle_cnt[7:4]);
                        4'b0111: seg <= decode_7seg(conv_cycle_cnt[3:0]);
                        default: seg <= 8'hff;
                    endcase
                end else begin
                    // Calc Selection Mode
                    case (an)
                        4'b1110: begin // Digit 0: Step/Mode
                            if (calc_step == 0) seg <= decode_op_type(sw[3:0]); // Show current Op selection
                            else if (calc_step == 1) seg <= 8'h88; // 'A'
                            else if (calc_step == 2) seg <= 8'h83; // 'b'
                            else seg <= decode_op_type(op_type); // Exec
                        end
                        4'b1101: begin // Digit 1: Value High
                            if (calc_step == 1 || calc_step == 2) seg <= decode_7seg(sw[7:4]);
                            else seg <= 8'hff;
                        end
                        4'b1011: begin // Digit 2: Value Low
                            if (calc_step == 1 || calc_step == 2) seg <= decode_7seg(sw[3:0]);
                            else seg <= 8'hff;
                        end
                        4'b0111: begin // Digit 3: Timeout
                            seg <= decode_7seg(timeout_sec);
                        end
                        default: seg <= 8'hff;
                    endcase
                end
            end else begin
                // Default Display Mode
                case (an)
                    4'b1110: seg <= decode_7seg({2'b00, mode}); // Mode
                    4'b1101: seg <= 8'hff;
                    4'b1011: seg <= 8'hff;
                    4'b0111: seg <= decode_7seg(timeout_sec); 
                    default: seg <= 8'hff; 
                endcase
            end
        end
    end

    // Helper Function: OpType Decoding
    function [7:0] decode_op_type(input [3:0] op);
        case(op)
            4'd0: decode_op_type = 8'h78; // T (t)
            4'd1: decode_op_type = 8'h88; // A
            4'd2: decode_op_type = 8'h83; // B (b)
            4'd3: decode_op_type = 8'hC6; // C
            4'd5: decode_op_type = 8'hE1; // J (approx)
            default: decode_op_type = 8'hFF;
        endcase
    endfunction

endmodule