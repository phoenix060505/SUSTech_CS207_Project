// 矩阵乘法模块
// 功能：计算两个矩阵的乘法 C = A × B
// 要求：A 的列数必须等于 B 的行数
module mat_mult #(
    parameter DIM_WIDTH  = 3,       // 维度位宽 (Max 7)
    parameter DATA_WIDTH = 8        // 数据位宽
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // --- 控制接口 ---
    input  wire                   start,
    input  wire [DIM_WIDTH-1:0]   m_a,         // 矩阵 A 的行数
    input  wire [DIM_WIDTH-1:0]   n_a,         // 矩阵 A 的列数
    input  wire [DIM_WIDTH-1:0]   m_b,         // 矩阵 B 的行数
    input  wire [DIM_WIDTH-1:0]   n_b,         // 矩阵 B 的列数
    
    input  wire                   slot_a_sel,  // 矩阵 A 的槽位
    input  wire                   slot_a_valid,
    input  wire                   slot_b_sel,  // 矩阵 B 的槽位
    input  wire                   slot_b_valid,
    
    // --- 状态输出 ---
    output reg                    ready,
    output reg                    busy,
    output reg                    done,
    output reg                    error,
    
    // --- 存储读取接口 ---
    output reg                    rd_en,
    output reg                    rd_slot_idx,
    output wire [DIM_WIDTH-1:0]   rd_row_idx,
    output wire [DIM_WIDTH-1:0]   rd_col_idx,
    input  wire [DATA_WIDTH-1:0]  rd_elem,      
    input  wire                   rd_elem_valid,

    // --- 数据流输出 ---
    output reg                    out_valid,
    output reg  [DATA_WIDTH-1:0]  out_elem,
    output reg                    out_row_end,
    output reg                    out_last,
    output reg [2*DIM_WIDTH-1:0]  out_linear_idx
);

    // ============================================================
    // 1. 状态定义
    // ============================================================
    localparam S_IDLE       = 4'd0;
    localparam S_CHECK      = 4'd1;
    localparam S_INIT_ROW   = 4'd2;  // 初始化结果矩阵的一行
    localparam S_INIT_COL   = 4'd3;  // 初始化累加器
    localparam S_FETCH_A    = 4'd4;  // 读取 A[i][k]
    localparam S_WAIT_A     = 4'd5;  // 等待 A 数据
    localparam S_FETCH_B    = 4'd6;  // 读取 B[k][j]
    localparam S_WAIT_B     = 4'd7;  // 等待 B 数据并累加
    localparam S_OUTPUT     = 4'd8;  // 输出一个元素
    localparam S_DONE       = 4'd9;
    localparam S_ERROR      = 4'd10;

    reg [3:0] state, next_state;

    // ============================================================
    // 2. 内部寄存器
    // ============================================================
    reg [DIM_WIDTH-1:0] m_a_latched;  // A: m_a × n_a
    reg [DIM_WIDTH-1:0] n_a_latched;
    reg [DIM_WIDTH-1:0] m_b_latched;  // B: m_b × n_b
    reg [DIM_WIDTH-1:0] n_b_latched;
    reg                 slot_a_latched;
    reg                 slot_b_latched;
    
    reg [DIM_WIDTH-1:0] i_cnt;        // 结果矩阵的行索引
    reg [DIM_WIDTH-1:0] j_cnt;        // 结果矩阵的列索引
    reg [DIM_WIDTH-1:0] k_cnt;        // 内积累加索引
    
    reg [2*DATA_WIDTH-1:0] accumulator;  // 累加器
    reg [DATA_WIDTH-1:0]   a_temp;       // 暂存 A[i][k]

    // 地址生成
    reg [DIM_WIDTH-1:0] addr_row;
    reg [DIM_WIDTH-1:0] addr_col;
    
    assign rd_row_idx = addr_row;
    assign rd_col_idx = addr_col;

    // ============================================================
    // 3. 状态机 - Next State Logic
    // ============================================================
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE: begin
                if (start && ready) next_state = S_CHECK;
            end
            S_CHECK: begin
                // 检查：A 的列数必须等于 B 的行数
                if (slot_a_valid && slot_b_valid && 
                    n_a_latched == m_b_latched && 
                    m_a_latched != 0 && n_b_latched != 0)
                    next_state = S_INIT_ROW;
                else
                    next_state = S_ERROR;
            end
            S_INIT_ROW: begin
                next_state = S_INIT_COL;
            end
            S_INIT_COL: begin
                next_state = S_FETCH_A;
            end
            S_FETCH_A: begin
                next_state = S_WAIT_A;
            end
            S_WAIT_A: begin
                if (rd_elem_valid) next_state = S_FETCH_B;
                else next_state = S_WAIT_A;
            end
            S_FETCH_B: begin
                next_state = S_WAIT_B;
            end
            S_WAIT_B: begin
                if (rd_elem_valid) begin
                    // 累加完成一个元素
                    if (k_cnt == n_a_latched - 1)
                        next_state = S_OUTPUT;
                    else
                        next_state = S_FETCH_A;
                end else begin
                    next_state = S_WAIT_B;
                end
            end
            S_OUTPUT: begin
                // 输出一个结果元素
                if (j_cnt == n_b_latched - 1 && i_cnt == m_a_latched - 1)
                    next_state = S_DONE;
                else if (j_cnt == n_b_latched - 1)
                    next_state = S_INIT_ROW;  // 下一行
                else
                    next_state = S_INIT_COL;  // 下一列
            end
            S_DONE:  next_state = S_IDLE;
            S_ERROR: next_state = S_IDLE;
            default: next_state = S_IDLE;
        endcase
    end

    // ============================================================
    // 4. 状态机 - 时序逻辑
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            m_a_latched     <= 0;
            n_a_latched     <= 0;
            m_b_latched     <= 0;
            n_b_latched     <= 0;
            slot_a_latched  <= 0;
            slot_b_latched  <= 0;
            
            ready           <= 1;
            busy            <= 0;
            done            <= 0;
            error           <= 0;
            
            rd_en           <= 0;
            rd_slot_idx     <= 0;
            addr_row        <= 0;
            addr_col        <= 0;
            
            out_valid       <= 0;
            out_elem        <= 0;
            out_row_end     <= 0;
            out_last        <= 0;
            out_linear_idx  <= 0;
            
            i_cnt           <= 0;
            j_cnt           <= 0;
            k_cnt           <= 0;
            accumulator     <= 0;
            a_temp          <= 0;
        end else begin
            state <= next_state;
            
            // 默认值
            rd_en       <= 0;
            done        <= 0;
            error       <= 0;
            out_valid   <= 0;
            out_row_end <= 0;
            out_last    <= 0;

            case (state)
                S_IDLE: begin
                    ready <= 1;
                    busy  <= 0;
                    if (start && ready) begin
                        busy  <= 1;
                        ready <= 0;
                        
                        m_a_latched     <= m_a;
                        n_a_latched     <= n_a;
                        m_b_latched     <= m_b;
                        n_b_latched     <= n_b;
                        slot_a_latched  <= slot_a_sel;
                        slot_b_latched  <= slot_b_sel;
                        
                        i_cnt           <= 0;
                        j_cnt           <= 0;
                        k_cnt           <= 0;
                        out_linear_idx  <= 0;
                    end
                end

                S_CHECK: begin
                    // 仅做检查
                end

                S_INIT_ROW: begin
                    // 准备计算新的一行
                    j_cnt <= 0;
                end

                S_INIT_COL: begin
                    // 准备计算新的一个元素 C[i][j]
                    k_cnt <= 0;
                    accumulator <= 0;
                end

                S_FETCH_A: begin
                    rd_en <= 1;
                    rd_slot_idx <= slot_a_latched;
                    addr_row <= i_cnt;
                    addr_col <= k_cnt;
                end

                S_WAIT_A: begin
                    if (rd_elem_valid) begin
                        a_temp <= rd_elem;
                    end
                end

                S_FETCH_B: begin
                    rd_en <= 1;
                    rd_slot_idx <= slot_b_latched;
                    addr_row <= k_cnt;
                    addr_col <= j_cnt;
                end

                S_WAIT_B: begin
                    if (rd_elem_valid) begin
                        // 累加：C[i][j] += A[i][k] * B[k][j]
                        accumulator <= accumulator + (a_temp * rd_elem);
                        k_cnt <= k_cnt + 1;
                    end
                end

                S_OUTPUT: begin
                    // 输出累加结果（截断）
                    out_valid <= 1;
                    out_elem  <= accumulator[DATA_WIDTH-1:0];
                    
                    // 标志位
                    if (j_cnt == n_b_latched - 1)
                        out_row_end <= 1;
                    if (j_cnt == n_b_latched - 1 && i_cnt == m_a_latched - 1)
                        out_last <= 1;

                    out_linear_idx <= out_linear_idx + 1;
                    
                    // 更新索引
                    if (j_cnt == n_b_latched - 1) begin
                        j_cnt <= 0;
                        i_cnt <= i_cnt + 1;
                    end else begin
                        j_cnt <= j_cnt + 1;
                    end
                end

                S_DONE: begin
                    done <= 1;
                    busy <= 0;
                end
                
                S_ERROR: begin
                    error <= 1;
                    busy  <= 0;
                end
            endcase
        end
    end

endmodule
