`timescale 1ns / 1ps

module matrix_calculator_top #(
    parameter MAX_DIM = 5,
    parameter MAX_STORE = 2,
    parameter ELEM_WIDTH = 8,
    parameter TIMEOUT_DEFAULT = 10,
    parameter CLK_FREQ = 100_000_000
) (
    input wire clk,
    input wire rst,          // BTN_DOWN
    input wire [7:0] sw,
    input wire [3:0] btn,
    input wire uart_rx,
    output reg [7:0] led,
    output reg [7:0] seg,
    output reg [7:0] an,     // 8位位宽
    output wire uart_tx
);

    //==============================================================
    // 1. Definition of State Constants (Match FSM)
    //==============================================================
    localparam S_CALC_A_M    = 5'd10;
    localparam S_CALC_A_N    = 5'd11;
    localparam S_CALC_A_LIST = 5'd12;
    localparam S_CALC_A_ID   = 5'd13;
    localparam S_CALC_B_M    = 5'd14;
    localparam S_CALC_B_N    = 5'd15;
    localparam S_CALC_B_LIST = 5'd16;
    localparam S_CALC_B_ID   = 5'd17;

    //==============================================================
    // 2. Internal Signal Definition
    //==============================================================
    wire rst_n = ~rst;

    // FSM Signals
    wire [1:0] mode;
    wire [3:0] op_type;
    wire timeout_en;
    wire wen_store;
    wire start_compute;
    wire update_config;

    // Status Signals
    wire uart_rx_done;
    wire [7:0] uart_rx_data;
    wire input_done;
    wire display_done;
    wire store_error;
    reg  compute_done;
    reg  operand_legal;
    reg  timeout_expired;
    reg  error_flag;

    // Debounce Signals
    wire [3:0] btn_debounced; 

    // Matrix Data
    wire [199:0] mat_a_wire;
    wire [199:0] mat_b_wire;
    wire [3:0] dim_m_a, dim_n_a;
    wire [3:0] dim_m_b, dim_n_b;
    
    // Inputs
    wire [MAX_STORE-1:0] sel_idx_a = sw[3:2]; 
    wire [MAX_STORE-1:0] sel_idx_b = sw[1:0]; 
    wire [3:0] scalar_val = sw[3:0];
    reg [3:0] cfg_timeout_val;

    // UART TX Arbitration
    reg [7:0] tx_data_mux;
    reg tx_start_mux;
    wire tx_busy;
    
    // Sub-module interfaces
    wire [7:0] tx_disp_data, tx_trans_data, tx_add_data, tx_smul_data, tx_mmul_data, tx_conv_data;
    wire tx_disp_start, tx_trans_start, tx_add_start, tx_smul_start, tx_mmul_start, tx_conv_start;
    wire done_trans, done_add, done_smul, done_mmul, done_conv;

    //==============================================================
    // 3. Debounce Modules
    //==============================================================
    localparam DEBOUNCE_CNT = CLK_FREQ / 50; // 20ms

    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : gen_debounce
            debounce #(.CNT_MAX(DEBOUNCE_CNT)) btn_db (
                .clk(clk),
                .rst(rst),
                .btn_in(btn[i]),
                .btn_out(btn_debounced[i])
            );
        end
    endgenerate

    //==============================================================
    // 4. Capture Logic & FSM
    //==============================================================
    wire [1:0] calc_step;
    wire [4:0] fsm_state;
    reg [7:0] id_a, id_b;
    reg [3:0] filter_m, filter_n;

    always @(posedge clk) begin
        if (rst) begin
            id_a <= 0; id_b <= 0; filter_m <= 0; filter_n <= 0;
        end else if (btn_debounced[0]) begin 
            if (fsm_state == S_CALC_A_M) filter_m <= sw[3:0];
            if (fsm_state == S_CALC_A_N) filter_n <= sw[3:0];
            if (fsm_state == S_CALC_B_M) filter_m <= sw[3:0];
            if (fsm_state == S_CALC_B_N) filter_n <= sw[3:0];
            if (fsm_state == S_CALC_A_ID) id_a <= sw; 
            if (fsm_state == S_CALC_B_ID) id_b <= sw; 
        end
    end

    fsm_controller fsm_inst (
        .clk(clk), .rst(rst),
        .btn(btn_debounced), 
        .sw(sw),
        .input_done(input_done),
        .display_done(display_done),
        .operand_legal(operand_legal),
        .compute_done(compute_done),
        .timeout_expired(timeout_expired),
        .mode(mode),
        .op_type(op_type),
        .timeout_en(timeout_en),
        .wen_store(wen_store),
        .start_compute(start_compute),
        .update_config(update_config),
        .calc_step(calc_step),
        .fsm_state_out(fsm_state)
    );

    //==============================================================
    // 5. Interface & Storage Modules
    //==============================================================
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
        .flush(btn_debounced[0]), 
        .mode(mode), 
        .data_in(uart_rx_data),
        .read_id_a(id_a),
        .read_id_b(id_b),
        .mat_a_out(mat_a_wire),
        .mat_b_out(mat_b_wire),
        .dim_a_m(dim_m_a), .dim_a_n(dim_n_a), 
        .dim_b_m(dim_m_b), .dim_b_n(dim_n_b), 
        .input_done(input_done),
        .error_flag(store_error), 
        .display_start(mode == 2'b10),
        .tx_busy(tx_busy),
        .filter_en(fsm_state == S_CALC_A_LIST || fsm_state == S_CALC_B_LIST), 
        .filter_m(filter_m),
        .filter_n(filter_n),
        .tx_start(tx_disp_start),
        .tx_data(tx_disp_data),
        .display_done(display_done)
    );

    //==============================================================
    // 6. Computation Modules
    //==============================================================
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

    add_module add_inst (
        .clk(clk), .rst(rst),
        .start(start_compute && op_type == 4'd1),
        .m(dim_m_a), .n(dim_n_a), 
        .in_a(mat_a_wire),
        .in_b(mat_b_wire),
        .tx_busy(tx_busy),
        .tx_start(tx_add_start),
        .tx_data(tx_add_data),
        .done(done_add)
    );

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

    mat_mul_module mmul_inst (
        .clk(clk), .rst(rst),
        .start(start_compute && op_type == 4'd3),
        .m(dim_m_a), .n(dim_n_a), .p(dim_n_b), 
        .in_a(mat_a_wire),
        .in_b(mat_b_wire),
        .tx_busy(tx_busy),
        .tx_start(tx_mmul_start),
        .tx_data(tx_mmul_data),
        .done(done_mmul)
    );

    wire [15:0] conv_cycle_cnt; 
    conv_module conv_inst (
        .clk(clk), .rst(rst),
        .start(start_compute && op_type == 4'd5),
        .kernel_flat(mat_a_wire),
        .tx_busy(tx_busy),
        .tx_start(tx_conv_start),
        .tx_data(tx_conv_data),
        .busy(),
        .done(done_conv),
        .cycle_cnt(conv_cycle_cnt)
    );

    //==============================================================
    // 7. Combinational Logic: TX Mux & Legality
    //==============================================================
    always @(*) begin
        tx_data_mux  = 8'h00;
        tx_start_mux = 1'b0;
        
        if (mode == 2'b10) begin 
            tx_data_mux  = tx_disp_data;
            tx_start_mux = tx_disp_start;
        end 
        else if (mode == 2'b11) begin 
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

    always @(*) begin
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

        operand_legal = 1'b1;
        error_flag = 1'b0;
        
        if (mode == 2'b11) begin
            case (op_type)
                4'd1: begin // Add
                    if (dim_m_a != dim_m_b || dim_n_a != dim_n_b) operand_legal = 1'b0;
                end
                4'd3: begin // Mul
                    if (dim_n_a != dim_m_b) operand_legal = 1'b0;
                end
                default: operand_legal = 1'b1;
            endcase
        end
        if (!operand_legal) error_flag = 1'b1;
    end

    //==============================================================
    // 8. Display Driver (FINAL CORRECTED: ACTIVE HIGH)
    //==============================================================
    
    // 【关键修正】：使用高电平有效 (1=亮) 的段码表
    // 对应 EGO1 硬件: 1=Light ON, 0=Light OFF
    function [7:0] decode_7seg(input [3:0] num);
        case(num)
            4'h0: decode_7seg = 8'h3F; // 0011_1111
            4'h1: decode_7seg = 8'h06; // 0000_0110
            4'h2: decode_7seg = 8'h5B; // 0101_1011
            4'h3: decode_7seg = 8'h4F; // 0100_1111
            4'h4: decode_7seg = 8'h66; // 0110_0110
            4'h5: decode_7seg = 8'h6D; // 0110_1101
            4'h6: decode_7seg = 8'h7D; // 0111_1101
            4'h7: decode_7seg = 8'h07; // 0000_0111
            4'h8: decode_7seg = 8'h7F; // 0111_1111
            4'h9: decode_7seg = 8'h6F; // 0110_1111
            4'hA: decode_7seg = 8'h77; // A
            4'hB: decode_7seg = 8'h7C; // b
            4'hC: decode_7seg = 8'h39; // C
            4'hD: decode_7seg = 8'h5E; // d
            4'hE: decode_7seg = 8'h79; // E
            4'hF: decode_7seg = 8'h71; // F
            default: decode_7seg = 8'h00; // 全灭 (0 is OFF)
        endcase
    endfunction
    
    // 【关键修正】：操作符显示也改为 Active High
    function [7:0] decode_op_type(input [3:0] op);
        case(op)
            4'd0: decode_op_type = 8'h78; // t (0111_1000)
            4'd1: decode_op_type = 8'h77; // A (0111_0111)
            4'd2: decode_op_type = 8'h7C; // b (0111_1100)
            4'd3: decode_op_type = 8'h39; // C (0011_1001)
            4'd5: decode_op_type = 8'h1E; // J (0001_1110)
            default: decode_op_type = 8'h00; // 全灭
        endcase
    endfunction

    reg [31:0] timeout_cnt;
    reg [3:0] timeout_sec;
    reg [16:0] scan_cnt;
    reg [26:0] heartbeat;

    // 扫描选择信号
    wire [1:0] scan_sel = scan_cnt[16:15]; 

    wire [3:0] timeout_tens = (timeout_sec >= 4'd10) ? 4'd1 : 4'd0;
    wire [3:0] timeout_units = (timeout_sec >= 4'd10) ? (timeout_sec - 4'd10) : timeout_sec;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            timeout_cnt <= 0;
            cfg_timeout_val <= TIMEOUT_DEFAULT;
            timeout_sec <= TIMEOUT_DEFAULT;
            timeout_expired <= 1'b0;
            led <= 8'b0;
            
            // 【关键修正】复位时全输出 0 (Active High Logic: 0 = OFF)
            an <= 8'h00; 
            seg <= 8'h00; 
            
            scan_cnt <= 0;
            heartbeat <= 0;
        end else begin
            // Heartbeat Logic
            heartbeat <= heartbeat + 1;

            // Update Config
            if (update_config) begin
                cfg_timeout_val <= (sw[3:0] > 4 && sw[3:0] <= 15) ? sw[3:0] : TIMEOUT_DEFAULT;
            end

            // Timeout Logic
            if (timeout_en) begin
                if (timeout_cnt < CLK_FREQ - 1) begin
                    timeout_cnt <= timeout_cnt + 1;
                end else begin
                    timeout_cnt <= 0;
                    if (timeout_sec > 0) timeout_sec <= timeout_sec - 1;
                    else timeout_expired <= 1'b1;
                end
            end else begin
                timeout_cnt <= 0;
                timeout_sec <= cfg_timeout_val; 
                timeout_expired <= 1'b0;
            end

            // LED Status
            led[0] <= error_flag | store_error;
            led[1] <= tx_busy;      
            led[2] <= input_done;   
            led[3] <= display_done; 
            led[4] <= compute_done; 
            led[6:5] <= fsm_state[1:0]; 
            led[7] <= heartbeat[26];

            // ============================================
            // 修正后的数码管扫描逻辑 (Active High)
            // ============================================
            scan_cnt <= scan_cnt + 1;

            // 1. 控制位选 (Anode Control - Active High)
            // 扫描顺序：AN0 -> AN1 -> AN2 -> AN3 (只显示前4位)
            // 注意：现在用 1 来选通
            case (scan_sel)
                2'b00: an <= 8'b0000_0001; // AN0 Active (Rightmost)
                2'b01: an <= 8'b0000_0010; // AN1 Active
                2'b10: an <= 8'b0000_0100; // AN2 Active
                2'b11: an <= 8'b0000_1000; // AN3 Active (Leftmost of 4)
            endcase

            // 2. 控制段选 (Segment Control - Active High)
            // default 必须是 8'h00 (全灭)
            if (mode == 2'b11) begin
                if (op_type == 4'd5 && calc_step > 2) begin 
                    // 卷积结果显示
                    case (scan_sel)
                        2'b00: seg <= decode_7seg(conv_cycle_cnt[3:0]);   // AN0
                        2'b01: seg <= decode_7seg(conv_cycle_cnt[7:4]);   // AN1
                        2'b10: seg <= decode_7seg(conv_cycle_cnt[11:8]);  // AN2
                        2'b11: seg <= decode_7seg(conv_cycle_cnt[15:12]); // AN3
                        default: seg <= 8'h00;
                    endcase
                end else begin
                    // 普通运算结果显示
                    case (scan_sel)
                        2'b00: begin // AN0 (Rightmost)
                            if (calc_step == 0) seg <= decode_op_type(sw[3:0]); 
                            else if (calc_step == 1) seg <= 8'h77; // 'A' (Active High)
                            else if (calc_step == 2) seg <= 8'h7C; // 'b' (Active High)
                            else seg <= decode_op_type(op_type); 
                        end
                        2'b01: begin // AN1
                            if (calc_step == 1 || calc_step == 2) seg <= decode_7seg(sw[7:4]);
                            else seg <= 8'h00; // OFF
                        end
                        2'b10: begin // AN2
                            if (calc_step == 1 || calc_step == 2) seg <= decode_7seg(sw[3:0]);
                            else seg <= decode_7seg(timeout_units);
                        end
                        2'b11: begin // AN3
                            seg <= decode_7seg(timeout_tens);
                        end
                        default: seg <= 8'h00;
                    endcase
                end
            end else begin
                // Default Mode (Input/Idle/Gen)
                case (scan_sel)
                    2'b00: seg <= decode_7seg(timeout_units); // AN0
                    2'b01: seg <= decode_7seg(timeout_tens);  // AN1
                    2'b10: seg <= 8'h00;                      // AN2 (OFF)
                    2'b11: seg <= decode_7seg({2'b00, mode}); // AN3
                    default: seg <= 8'h00; 
                endcase
            end
        end
    end

endmodule