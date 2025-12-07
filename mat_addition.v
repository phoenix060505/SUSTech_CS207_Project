module mat_add #(
    parameter DIM_WIDTH  = 3,       // 维度位宽 (Max 7)
    parameter DATA_WIDTH = 8        // 数据位宽
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // --- 控制接口 ---
    input  wire                   start,
    input  wire [DIM_WIDTH-1:0]   m_sel,        // 矩阵行数
    input  wire [DIM_WIDTH-1:0]   n_sel,        // 矩阵列数
    
    // 双操作数选择
    input  wire                   slot_a_sel,   // 矩阵 A 的槽位
    input  wire                   slot_a_valid, // A 槽位是否有效
    input  wire                   slot_b_sel,   // 矩阵 B 的槽位
    input  wire                   slot_b_valid, // B 槽位是否有效
    
    // --- 状态输出 ---
    output reg                    ready,
    output reg                    busy,
    output reg                    done,
    output reg                    error,
    
    // 元数据输出
    output reg [2*DIM_WIDTH-1:0]  total_elements, 

    // --- 存储读取接口 (单端口分时复用) ---
    output reg                    rd_en,
    output reg                    rd_slot_idx,  // 【注意】这里要是 reg，因为要在 A/B 槽位间切换
    output wire [DIM_WIDTH-1:0]   rd_row_idx,
    output wire [DIM_WIDTH-1:0]   rd_col_idx,
    input  wire [DATA_WIDTH-1:0]  rd_elem,      
    input  wire                   rd_elem_valid,

    // --- 数据流输出 (Result = A + B) ---
    output reg                    out_valid,
    output reg  [DATA_WIDTH-1:0]  out_elem,     // 输出结果
    output reg                    out_row_end,
    output reg                    out_last,
    output reg [2*DIM_WIDTH-1:0]  out_linear_idx
);

    // ============================================================
    // 1. 状态定义
    // ============================================================
    localparam S_IDLE       = 4'd0;
    localparam S_CHECK      = 4'd1;
    // 读取 A 的流程
    localparam S_FETCH_A    = 4'd2; // 发地址 A
    localparam S_WAIT_A     = 4'd3; // 等数据 A 并暂存
    // 读取 B 并计算的流程
    localparam S_FETCH_B    = 4'd4; // 发地址 B
    localparam S_CALC_EMIT  = 4'd5; // 等数据 B，相加，输出
    
    localparam S_DONE       = 4'd6;
    localparam S_ERROR_DONE = 4'd7;

    reg [3:0] current_state, next_state; 

    // ============================================================
    // 2. 内部寄存器
    // ============================================================
    // 锁存输入
    reg [DIM_WIDTH-1:0] m_latched;
    reg [DIM_WIDTH-1:0] n_latched;
    reg                 slot_a_latched;
    reg                 slot_a_valid_latched;
    reg                 slot_b_latched;
    reg                 slot_b_valid_latched;
    
    // 计数器
    reg [DIM_WIDTH-1:0] row_cnt; 
    reg [DIM_WIDTH-1:0] col_cnt;
    
    // 暂存器：用于保存从存储器读出来的 Matrix A 的值
    reg signed [DATA_WIDTH-1:0] val_a_temp;

    // ============================================================
    // 3. 组合逻辑
    // ============================================================
    // 读地址由计数器直接驱动 (加法不需要转置，直接读 i,j)
    assign rd_row_idx  = row_cnt;
    assign rd_col_idx  = col_cnt;

    // Next State Logic
    always @(*) begin
        next_state = current_state; 
        case (current_state)
            S_IDLE: begin// 等待启动信号
                if (start && ready) next_state = S_CHECK;
            end
            
            S_CHECK: begin
                // 必须 A 和 B 两个槽位都有效，且维度非零
                if (slot_a_valid_latched && slot_b_valid_latched && 
                    m_latched != 0 && n_latched != 0) 
                    next_state = S_FETCH_A;
                else 
                    next_state = S_ERROR_DONE;
            end
            
            // --- Step 1: 读取 Matrix A ---
            S_FETCH_A: next_state = S_WAIT_A;
            
            S_WAIT_A: begin
                // 等待存储器返回 A 的数据
                if (rd_elem_valid) next_state = S_FETCH_B;
                else               next_state = S_WAIT_A;
            end
            
            // --- Step 2: 读取 Matrix B 并计算 ---
            S_FETCH_B: next_state = S_CALC_EMIT;
            
            S_CALC_EMIT: begin
                if (rd_elem_valid) begin
                    // 如果读到了 B，计算并判断是否结束
                    if (col_cnt == n_latched - 1 && row_cnt == m_latched - 1)
                        next_state = S_DONE;
                    else
                        next_state = S_FETCH_A; // 循环：去读下一个元素的 A
                end else begin
                    next_state = S_CALC_EMIT;
                end
            end
            
            S_DONE:         next_state = S_IDLE;
            S_ERROR_DONE:   next_state = S_IDLE;
            default:        next_state = S_IDLE;
        endcase
    end

    // ============================================================
    // 4. 时序逻辑
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state   <= S_IDLE;
            // 复位所有寄存器
            m_latched       <= 0;
            n_latched       <= 0;
            slot_a_latched  <= 0;
            slot_a_valid_latched <= 0;
            slot_b_latched  <= 0;
            slot_b_valid_latched <= 0;
            
            ready           <= 1;
            busy            <= 0;
            done            <= 0;
            error           <= 0;
            total_elements  <= 0;
            
            rd_en           <= 0;
            rd_slot_idx     <= 0;
            out_valid       <= 0;
            out_elem        <= 0;
            out_row_end     <= 0;
            out_last        <= 0;
            out_linear_idx  <= 0;
            
            row_cnt         <= 0;
            col_cnt         <= 0;
            val_a_temp      <= 0;
        end else begin
            current_state <= next_state;
            
            // 默认控制信号复位
            rd_en       <= 0; 
            done        <= 0;
            error       <= 0;
            out_valid   <= 0;
            out_row_end <= 0;
            out_last    <= 0;

            case (current_state)
                // ------------------------------------------------
                S_IDLE: begin
                    ready <= 1;
                    busy  <= 0;
                    if (start && ready) begin
                        busy  <= 1;
                        ready <= 0;
                        
                        // 锁存输入
                        m_latched            <= m_sel;
                        n_latched            <= n_sel;
                        slot_a_latched       <= slot_a_sel;
                        slot_a_valid_latched <= slot_a_valid;
                        slot_b_latched       <= slot_b_sel;
                        slot_b_valid_latched <= slot_b_valid;
                        
                        total_elements       <= m_sel * n_sel; 
                        
                        // 强制清零
                        row_cnt        <= 0;
                        col_cnt        <= 0;
                        out_linear_idx <= 0;
                    end
                end

                // ------------------------------------------------
                S_FETCH_A: begin
                    // 准备读 A
                    rd_slot_idx <= slot_a_latched; // 切换到 Slot A
                    rd_en       <= 1;
                end

                // ------------------------------------------------
                S_WAIT_A: begin
                    if (rd_elem_valid) begin
                        val_a_temp <= rd_elem; // 【关键】暂存 A 的值
                    end
                end

                // ------------------------------------------------
                S_FETCH_B: begin
                    // 准备读 B (此时地址 row_cnt/col_cnt 没变，还是当前坐标)
                    rd_slot_idx <= slot_b_latched; // 切换到 Slot B
                    rd_en       <= 1;
                end

                // ------------------------------------------------
                S_CALC_EMIT: begin
                    if (rd_elem_valid) begin
                        out_valid <= 1;
                        // 【核心计算】结果 = 暂存的A + 刚读到的B
                        // 注意：这里没有处理溢出，8bit+8bit直接截断保留低8bit
                        // 如果需要处理溢出，可以在这里加逻辑
                        out_elem  <= val_a_temp + rd_elem;
                        
                        // 标志位
                        if (col_cnt == n_latched - 1) 
                            out_row_end <= 1;
                        if (col_cnt == n_latched - 1 && row_cnt == m_latched - 1) 
                            out_last <= 1;

                        // 计数器更新
                        out_linear_idx <= out_linear_idx + 1;
                        
                        if (col_cnt == n_latched - 1) begin
                            col_cnt <= 0;
                            if (row_cnt == m_latched - 1) begin
                                // 结束
                            end else begin
                                row_cnt <= row_cnt + 1;
                            end
                        end else begin
                            col_cnt <= col_cnt + 1;
                        end
                    end
                end

                // ------------------------------------------------
                S_DONE: begin
                    done <= 1;
                    busy <= 0;
                end
                
                S_ERROR_DONE: begin
                    error <= 1;
                    busy  <= 0;
                end
            endcase
        end
    end

endmodule
