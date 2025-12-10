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
    input  wire                   slot_a_valid, 
    input  wire                   slot_b_sel,   // 矩阵 B 的槽位
    input  wire                   slot_b_valid, 
    
    // --- 状态输出 ---
    output reg                    ready,
    output reg                    busy,
    output reg                    done,
    output reg                    error,
    
    // 元数据输出
    output reg [2*DIM_WIDTH-1:0]  total_elements, 

    // --- 存储读取接口 ---
    output reg                    rd_en,
    output reg                    rd_slot_idx,
    output wire [DIM_WIDTH-1:0]   rd_row_idx,
    output wire [DIM_WIDTH-1:0]   rd_col_idx,
    input  wire [DATA_WIDTH-1:0]  rd_elem,   
    input  wire                   rd_elem_valid,

    // --- 数据流输出 (Result = A + B) ---
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
    localparam S_PRE_A      = 4'd2; // 准备读取 A (设置地址)
    localparam S_WAIT_A     = 4'd3; // 等待 A 数据
    localparam S_PRE_B      = 4'd4; // 准备读取 B (设置地址)
    localparam S_WAIT_B     = 4'd5; // 等待 B 数据并计算
    localparam S_DONE       = 4'd6;
    localparam S_ERROR      = 4'd7;

    reg [3:0] state, next_state;

    // ============================================================
    // 2. 内部寄存器
    // ============================================================
    reg [DIM_WIDTH-1:0] m_latched;
    reg [DIM_WIDTH-1:0] n_latched;
    reg                 slot_a_latched;
    reg                 slot_b_latched;
    reg [DIM_WIDTH-1:0] row_cnt; 
    reg [DIM_WIDTH-1:0] col_cnt;
    
    // 暂存器：用于保存 Matrix A 的值
    reg [DATA_WIDTH-1:0] val_a_temp;

    // 地址直接连接计数器
    assign rd_row_idx = row_cnt;
    assign rd_col_idx = col_cnt;

    // ============================================================
    // 3. 状态机逻辑
    // ============================================================
    
    // Next State Logic
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE: begin
                if (start) next_state = S_CHECK;
            end
            
            S_CHECK: begin
                // 简单的有效性检查
                if (m_sel != 0 && n_sel != 0 && slot_a_valid && slot_b_valid)
                    next_state = S_PRE_A;
                else
                    next_state = S_ERROR;
            end
            
            S_PRE_A: begin
                // 必须经过一个周期的 PRE 状态来稳定地址，并拉低 rd_en
                next_state = S_WAIT_A;
            end

            S_WAIT_A: begin
                // 等待存储模块返回有效数据
                if (rd_elem_valid) 
                    next_state = S_PRE_B;
                else 
                    next_state = S_WAIT_A;
            end
            
            S_PRE_B: begin
                next_state = S_WAIT_B;
            end

            S_WAIT_B: begin
                if (rd_elem_valid) begin
                    // 如果这是最后一个元素，去 DONE，否则回到 PRE_A 读下一个
                    if (col_cnt == n_latched - 1 && row_cnt == m_latched - 1)
                        next_state = S_DONE;
                    else
                        next_state = S_PRE_A;
                end else begin
                    next_state = S_WAIT_B;
                end
            end
            
            S_DONE:  next_state = S_IDLE;
            S_ERROR: next_state = S_IDLE;
            default: next_state = S_IDLE;
        endcase
    end

    // Output and Data Path Logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            m_latched       <= 0;
            n_latched       <= 0;
            slot_a_latched  <= 0;
            slot_b_latched  <= 0;
            row_cnt         <= 0;
            col_cnt         <= 0;
            val_a_temp      <= 0;
            ready           <= 1;
            busy            <= 0;
            done            <= 0;
            error           <= 0;
            rd_en           <= 0;
            rd_slot_idx     <= 0;
            out_valid       <= 0;
            out_elem        <= 0;
            out_row_end     <= 0;
            out_last        <= 0;
            out_linear_idx  <= 0;
            total_elements  <= 0;

        end else begin
            state <= next_state;
            
            // 默认信号复位 (Pulse signals)
            out_valid   <= 0;
            out_row_end <= 0;
            out_last    <= 0;
            done        <= 0;
            error       <= 0;
            // rd_en 默认由状态决定，不在默认里强制置0，而是在case里显式控制
            
            case (state)
                S_IDLE: begin
                    ready <= 1;
                    busy  <= 0;
                    rd_en <= 0;
                    
                    if (start) begin
                        ready <= 0;
                        busy  <= 1;
                        // 锁存参数
                        m_latched      <= m_sel;
                        n_latched      <= n_sel;
                        slot_a_latched <= slot_a_sel;
                        slot_b_latched <= slot_b_sel;
                        total_elements <= m_sel * n_sel;
                        // 复位计数器
                        row_cnt        <= 0;
                        col_cnt        <= 0;
                        out_linear_idx <= 0;
                    end
                end

                S_PRE_A: begin
                    // 【关键修改】在准备阶段拉低 rd_en，确保读时序的上升沿
                    rd_en       <= 0; 
                    rd_slot_idx <= slot_a_latched;
                end

                S_WAIT_A: begin
                    // 在等待阶段拉高 rd_en，一旦拿到数据立即拉低避免持续有效
                    rd_en       <= 1;
                    rd_slot_idx <= slot_a_latched;
                    
                    if (rd_elem_valid) begin
                        val_a_temp <= rd_elem; // 锁存 A
                        rd_en      <= 0;       // 防止 combinational rd_elem_valid 一直为高
                    end
                end

                S_PRE_B: begin
                    // 【关键修改】拉低 rd_en
                    rd_en       <= 0;
                    rd_slot_idx <= slot_b_latched;
                end

                S_WAIT_B: begin
                    // 拉高 rd_en 等待 B，一旦采到数据立即拉低以免持续触发
                    rd_en       <= 1;
                    rd_slot_idx <= slot_b_latched;

                    if (rd_elem_valid) begin
                        rd_en <= 0; // 防止 rd_elem_valid 持续为高导致重复输出

                        // 计算结果
                        out_valid <= 1;
                        out_elem  <= val_a_temp + rd_elem;
                        
                        // 更新线性索引
                        out_linear_idx <= out_linear_idx + 1;
                        
                        // 边界标志
                        if (col_cnt == n_latched - 1) begin
                            out_row_end <= 1;
                            if (row_cnt == m_latched - 1) 
                                out_last <= 1;
                        end

                        // 更新计数器 (为下一个元素做准备)
                        if (col_cnt == n_latched - 1) begin
                            col_cnt <= 0;
                            if (row_cnt != m_latched - 1)
                                row_cnt <= row_cnt + 1;
                        end else begin
                            col_cnt <= col_cnt + 1;
                        end
                    end
                end

                S_DONE: begin
                    busy  <= 0;
                    done  <= 1;
                    rd_en <= 0;
                end

                S_ERROR: begin
                    busy  <= 0;
                    error <= 1;
                    rd_en <= 0;
                end
            endcase
        end
    end

endmodule