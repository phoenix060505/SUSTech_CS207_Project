module mat_mult #(
    parameter DIM_WIDTH  = 3,
    parameter DATA_WIDTH = 8
)(
    input  wire                   clk,
    input  wire                   rst_n,
    // --- 控制接口 ---
    input  wire                   start,
    input  wire [DIM_WIDTH-1:0]   m_a, n_a, m_b, n_b,
    input  wire                   slot_a_sel, slot_a_valid,
    input  wire                   slot_b_sel, slot_b_valid,
    // --- 状态输出 ---
    output reg                    ready, busy, done, error,
    // --- 存储读取接口 ---
    output reg                    rd_en,
    output reg                    rd_slot_idx,
    output wire [DIM_WIDTH-1:0]   rd_row_idx,
    output wire [DIM_WIDTH-1:0]   rd_col_idx,
    output reg [DIM_WIDTH-1:0]    rd_current_m,
    output reg [DIM_WIDTH-1:0]    rd_current_n,
    input  wire [DATA_WIDTH-1:0]  rd_elem,      
    input  wire                   rd_elem_valid,
    // --- 数据流输出 ---
    output reg                    out_valid,
    output reg  [DATA_WIDTH-1:0]  out_elem,
    output reg                    out_row_end,
    output reg                    out_last,
    output reg [2*DIM_WIDTH-1:0]  out_linear_idx
);

    // 状态定义
    localparam S_IDLE       = 4'd0;
    localparam S_CHECK      = 4'd1;
    localparam S_INIT_ROW   = 4'd2;
    localparam S_INIT_COL   = 4'd3;
    localparam S_PRE_A      = 4'd4; // 准备读 A
    localparam S_WAIT_A     = 4'd5; // 读 A
    localparam S_PRE_B      = 4'd6; // 准备读 B
    localparam S_WAIT_B     = 4'd7; // 读 B 并累加
    localparam S_OUTPUT     = 4'd8;
    localparam S_DONE       = 4'd9;
    localparam S_ERROR      = 4'd10;

    reg [3:0] state, next_state;
    
    // 寄存器
    reg [DIM_WIDTH-1:0] m_a_latched, n_a_latched, m_b_latched, n_b_latched;
    reg slot_a_latched, slot_b_latched;
    reg [DIM_WIDTH-1:0] i_cnt, j_cnt, k_cnt;
    reg [2*DATA_WIDTH-1:0] accumulator;
    reg [DATA_WIDTH-1:0]   a_temp;
    
    // 地址生成
    reg [DIM_WIDTH-1:0] addr_row, addr_col;
    assign rd_row_idx = addr_row;
    assign rd_col_idx = addr_col;

    // Next State Logic
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE:     if (start && ready) next_state = S_CHECK;
            S_CHECK:    if (slot_a_valid && slot_b_valid && n_a_latched == m_b_latched && m_a_latched != 0 && n_b_latched != 0)
                            next_state = S_INIT_ROW;
                        else next_state = S_ERROR;
            S_INIT_ROW: next_state = S_INIT_COL;
            S_INIT_COL: next_state = S_PRE_A; // Start with PRE
            
            S_PRE_A:    next_state = S_WAIT_A;
            S_WAIT_A:   if (rd_elem_valid) next_state = S_PRE_B;
                        else next_state = S_WAIT_A;
                        
            S_PRE_B:    next_state = S_WAIT_B;
            S_WAIT_B:   if (rd_elem_valid) begin
                            if (k_cnt == n_a_latched - 1) next_state = S_OUTPUT;
                            else next_state = S_PRE_A; // Back to PRE_A for next k
                        end else next_state = S_WAIT_B;
                        
            S_OUTPUT:   if (j_cnt == n_b_latched - 1 && i_cnt == m_a_latched - 1) next_state = S_DONE;
                        else if (j_cnt == n_b_latched - 1) next_state = S_INIT_ROW;
                        else next_state = S_INIT_COL;
            S_DONE:     next_state = S_IDLE;
            S_ERROR:    next_state = S_IDLE;
            default:    next_state = S_IDLE;
        endcase
    end

    // Sequential Logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            m_a_latched <= 0; n_a_latched <= 0; m_b_latched <= 0; n_b_latched <= 0;
            slot_a_latched <= 0; slot_b_latched <= 0;
            ready <= 1; busy <= 0; done <= 0; error <= 0;
            rd_en <= 0; rd_slot_idx <= 0; addr_row <= 0; addr_col <= 0;
            rd_current_m <= 0; rd_current_n <= 0;
            out_valid <= 0; out_elem <= 0; out_row_end <= 0; out_last <= 0; out_linear_idx <= 0;
            i_cnt <= 0; j_cnt <= 0; k_cnt <= 0; accumulator <= 0; a_temp <= 0;
        end else begin
            state <= next_state;
            rd_en <= 0; // Default off
            out_valid <= 0; out_row_end <= 0; out_last <= 0; done <= 0; error <= 0;
            
            case (state)
                S_IDLE: begin
                    ready <= 1; busy <= 0;
                    if (start && ready) begin
                        busy <= 1; ready <= 0;
                        m_a_latched <= m_a; n_a_latched <= n_a;
                        m_b_latched <= m_b; n_b_latched <= n_b;
                        slot_a_latched <= slot_a_sel; slot_b_latched <= slot_b_sel;
                        i_cnt <= 0; j_cnt <= 0; k_cnt <= 0; out_linear_idx <= 0;
                    end
                end
                S_INIT_ROW: j_cnt <= 0;
                S_INIT_COL: begin k_cnt <= 0; accumulator <= 0; end
                
                S_PRE_A: begin
                    rd_en <= 0; // 地址建立，使能拉低
                    rd_slot_idx <= slot_a_latched;
                    addr_row <= i_cnt; addr_col <= k_cnt;
                    rd_current_m <= m_a_latched; rd_current_n <= n_a_latched;
                end
                S_WAIT_A: begin
                    rd_en <= 1; // 保持使能
                    rd_slot_idx <= slot_a_latched;
                    if (rd_elem_valid) a_temp <= rd_elem;
                end
                
                S_PRE_B: begin
                    rd_en <= 0; // 地址建立
                    rd_slot_idx <= slot_b_latched;
                    addr_row <= k_cnt; addr_col <= j_cnt;
                    rd_current_m <= m_b_latched; rd_current_n <= n_b_latched;
                end
                S_WAIT_B: begin
                    rd_en <= 1; // 保持使能
                    rd_slot_idx <= slot_b_latched;
                    if (rd_elem_valid) begin
                        accumulator <= accumulator + (a_temp * rd_elem);
                        k_cnt <= k_cnt + 1;
                    end
                end
                // -----------------------------
                
                S_OUTPUT: begin
                    out_valid <= 1;
                    out_elem <= accumulator[DATA_WIDTH-1:0];
                    if (j_cnt == n_b_latched - 1) out_row_end <= 1;
                    if (j_cnt == n_b_latched - 1 && i_cnt == m_a_latched - 1) out_last <= 1;
                    out_linear_idx <= out_linear_idx + 1;
                    if (j_cnt == n_b_latched - 1) begin j_cnt <= 0; i_cnt <= i_cnt + 1; end
                    else j_cnt <= j_cnt + 1;
                end
                S_DONE:  begin done <= 1; busy <= 0; end
                S_ERROR: begin error <= 1; busy <= 0; end
            endcase
        end
    end
endmodule