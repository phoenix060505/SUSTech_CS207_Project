module mat_transposition_v4 #(
    parameter DIM_WIDTH  = 3,       // 维度位宽 (Max 2^3-1 = 7)
    parameter DATA_WIDTH = 8        // 数据位宽
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // --- 控制接口 (Control Interface) ---
    input  wire                   start,
    input  wire [DIM_WIDTH-1:0]   m_sel,
    input  wire [DIM_WIDTH-1:0]   n_sel,
    input  wire                   slot_sel,
    input  wire                   slot_valid,
    
    // --- 状态输出 (Status Output) ---
    output reg                    ready,
    output reg                    busy,
    output reg                    done,
    output reg                    error,
    
    // --- 元数据输出 (Metadata) ---
    // 自动计算位宽：例如 3-bit 维度最大 7x7=49，需要 6-bit (2*3)
    output reg [2*DIM_WIDTH-1:0]  total_elements, 

    // --- 存储读取接口 (To Matrix Storage) ---
    output reg                    rd_en,
    output wire                   rd_slot_idx,
    output wire [DIM_WIDTH-1:0]   rd_row_idx,
    output wire [DIM_WIDTH-1:0]   rd_col_idx,
    input  wire [DATA_WIDTH-1:0]  rd_elem,      
    input  wire                   rd_elem_valid,

    // --- 数据流输出 (To UART Formatter) ---
    output reg                    out_valid,
    output reg  [DATA_WIDTH-1:0]  out_elem,
    output reg                    out_row_end,
    output reg                    out_last,
    output reg [DIM_WIDTH-1:0]    out_row_idx,
    output reg [DIM_WIDTH-1:0]    out_col_idx,
    output reg [2*DIM_WIDTH-1:0]  out_linear_idx // 线性索引
);

    // ============================================================
    // 1. 状态定义
    // ============================================================
    localparam S_IDLE       = 3'd0;
    localparam S_CHECK      = 3'd1;
    localparam S_FETCH      = 3'd2; 
    localparam S_STREAM     = 3'd3; 
    localparam S_DONE       = 3'd4;
    localparam S_ERROR_DONE = 3'd5;

    reg [2:0] current_state, next_state; 

    // ============================================================
    // 2. 内部寄存器
    // ============================================================
    reg [DIM_WIDTH-1:0] m_latched;
    reg [DIM_WIDTH-1:0] n_latched;
    reg                 slot_latched;
    reg                 slot_valid_latched;
    
    reg [DIM_WIDTH-1:0] trans_row_cnt; 
    reg [DIM_WIDTH-1:0] trans_col_cnt;

    // ============================================================
    // 3. 组合逻辑：地址生成与 Next State
    // ============================================================
    assign rd_slot_idx = slot_latched;
    assign rd_row_idx  = trans_col_cnt; // 转置映射
    assign rd_col_idx  = trans_row_cnt;

    always @(*) begin
        next_state = current_state; 
        case (current_state)
            S_IDLE: begin
                // 【修复5】增加 ready 检查，防止重入
                if (start && ready) next_state = S_CHECK;
            end
            S_CHECK: begin
                if (slot_valid_latched && m_latched != 0 && n_latched != 0) 
                    next_state = S_FETCH;
                else 
                    next_state = S_ERROR_DONE;
            end
            S_FETCH: begin 
                next_state = S_STREAM; 
            end
            S_STREAM: begin
                if (rd_elem_valid) begin
                    if (trans_col_cnt == m_latched - 1 && trans_row_cnt == n_latched - 1)
                        next_state = S_DONE;
                    else
                        next_state = S_FETCH; 
                end else begin
                    next_state = S_STREAM; 
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
            // 锁存器复位
            m_latched       <= 0;
            n_latched       <= 0;
            slot_latched    <= 0;
            slot_valid_latched <= 0;
            
            // 状态输出复位
            ready           <= 1;
            busy            <= 0;
            done            <= 0;
            error           <= 0;
            total_elements  <= 0;
            
            // 接口复位
            rd_en           <= 0;
            out_valid       <= 0;
            out_elem        <= 0;
            out_row_end     <= 0;
            out_last        <= 0;
            out_row_idx     <= 0;
            out_col_idx     <= 0;
            out_linear_idx  <= 0;
            
            // 计数器复位
            trans_row_cnt   <= 0;
            trans_col_cnt   <= 0;
        end else begin
            // --- 默认值区域 (防止 latch 和未清零) ---
            current_state <= next_state;
            
            // 【修复2】rd_en 默认清零，仅在 S_FETCH 显式拉高
            rd_en       <= 0; 
            
            // 脉冲信号默认拉低
            done        <= 0;
            error       <= 0;
            out_valid   <= 0;
            out_row_end <= 0;
            out_last    <= 0;

            // --- 状态机行为 ---
            case (current_state)
                // ------------------------------------------------
                S_IDLE: begin
                    ready <= 1;
                    busy  <= 0; 
                    // 【修复5】增加 ready 条件
                    if (start && ready) begin
                        busy  <= 1;
                        ready <= 0;
                        
                        // 锁存输入
                        m_latched          <= m_sel;
                        n_latched          <= n_sel;
                        slot_latched       <= slot_sel;
                        slot_valid_latched <= slot_valid;
                        
                        // 【修复4】参数化计算总数
                        total_elements     <= m_sel * n_sel; 
                        
                        // 【修复1 & 3】强制清零所有计数器/索引
                        trans_row_cnt      <= 0;
                        trans_col_cnt      <= 0;
                        out_linear_idx     <= 0;
                        out_row_idx        <= 0;
                        out_col_idx        <= 0;
                    end
                end

                // ------------------------------------------------
                S_CHECK: begin
                    // 仅做检查跳转，无输出操作
                end

                // ------------------------------------------------
                S_FETCH: begin
                    rd_en <= 1; // 显式拉高
                    // 更新坐标输出以匹配当前请求的地址
                    out_row_idx <= trans_row_cnt; 
                    out_col_idx <= trans_col_cnt;
                end

                // ------------------------------------------------
                S_STREAM: begin
                    if (rd_elem_valid) begin
                        out_valid <= 1;
                        out_elem  <= rd_elem;
                        
                        // 标志位判断
                        if (trans_col_cnt == m_latched - 1) 
                            out_row_end <= 1;
                        if (trans_col_cnt == m_latched - 1 && trans_row_cnt == n_latched - 1) 
                            out_last <= 1;

                        // 计数器更新
                        out_linear_idx <= out_linear_idx + 1;
                        
                        if (trans_col_cnt == m_latched - 1) begin
                            trans_col_cnt <= 0;
                            if (trans_row_cnt == n_latched - 1) begin
                                // 结束，等待跳转 S_DONE
                            end else begin
                                trans_row_cnt <= trans_row_cnt + 1;
                            end
                        end else begin
                            trans_col_cnt <= trans_col_cnt + 1;
                        end
                    end
                end

                // ------------------------------------------------
                S_DONE: begin
                    done <= 1;
                    busy <= 0;
                end
                
                // ------------------------------------------------
                S_ERROR_DONE: begin
                    error <= 1;
                    busy  <= 0;
                end
            endcase
        end
    end

endmodule
