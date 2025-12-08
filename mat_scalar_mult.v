// 矩阵标量乘法模块
// 功能：将矩阵的每个元素乘以一个标量值
module mat_scalar_mult #(
    parameter DIM_WIDTH  = 3,       // 维度位宽 (Max 7)
    parameter DATA_WIDTH = 8        // 数据位宽
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // --- 控制接口 ---
    input  wire                   start,
    input  wire [DIM_WIDTH-1:0]   m_sel,        // 矩阵行数
    input  wire [DIM_WIDTH-1:0]   n_sel,        // 矩阵列数
    input  wire [DATA_WIDTH-1:0]  scalar,       // 标量值
    input  wire                   slot_sel,     // 矩阵槽位
    input  wire                   slot_valid,   // 槽位是否有效
    
    // --- 状态输出 ---
    output reg                    ready,
    output reg                    busy,
    output reg                    done,
    output reg                    error,
    
    // --- 存储读取接口 ---
    output reg                    rd_en,
    output wire                   rd_slot_idx,
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
    localparam S_IDLE       = 3'd0;
    localparam S_CHECK      = 3'd1;
    localparam S_FETCH      = 3'd2; 
    localparam S_COMPUTE    = 3'd3; 
    localparam S_DONE       = 3'd4;
    localparam S_ERROR      = 3'd5;

    reg [2:0] state, next_state;

    // ============================================================
    // 2. 内部寄存器
    // ============================================================
    reg [DIM_WIDTH-1:0] m_latched;
    reg [DIM_WIDTH-1:0] n_latched;
    reg [DATA_WIDTH-1:0] scalar_latched;
    reg                 slot_latched;
    
    reg [DIM_WIDTH-1:0] row_cnt;
    reg [DIM_WIDTH-1:0] col_cnt;
    
    reg [2*DATA_WIDTH-1:0] mult_result;  // 乘法结果（可能溢出）

    // ============================================================
    // 3. 地址生成
    // ============================================================
    assign rd_slot_idx = slot_latched;
    assign rd_row_idx  = row_cnt;
    assign rd_col_idx  = col_cnt;

    // ============================================================
    // 4. 状态机 - Next State Logic
    // ============================================================
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE: begin
                if (start && ready) next_state = S_CHECK;
            end
            S_CHECK: begin
                if (slot_valid && m_sel != 0 && n_sel != 0)
                    next_state = S_FETCH;
                else
                    next_state = S_ERROR;
            end
            S_FETCH: begin
                next_state = S_COMPUTE;
            end
            S_COMPUTE: begin
                if (rd_elem_valid) begin
                    if (col_cnt == n_latched - 1 && row_cnt == m_latched - 1)
                        next_state = S_DONE;
                    else
                        next_state = S_FETCH;
                end else begin
                    next_state = S_COMPUTE;
                end
            end
            S_DONE:  next_state = S_IDLE;
            S_ERROR: next_state = S_IDLE;
            default: next_state = S_IDLE;
        endcase
    end

    // ============================================================
    // 5. 状态机 - 时序逻辑
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            m_latched       <= 0;
            n_latched       <= 0;
            scalar_latched  <= 0;
            slot_latched    <= 0;
            
            ready           <= 1;
            busy            <= 0;
            done            <= 0;
            error           <= 0;
            
            rd_en           <= 0;
            out_valid       <= 0;
            out_elem        <= 0;
            out_row_end     <= 0;
            out_last        <= 0;
            out_linear_idx  <= 0;
            
            row_cnt         <= 0;
            col_cnt         <= 0;
            mult_result     <= 0;
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
                        
                        m_latched       <= m_sel;
                        n_latched       <= n_sel;
                        scalar_latched  <= scalar;
                        slot_latched    <= slot_sel;
                        
                        row_cnt         <= 0;
                        col_cnt         <= 0;
                        out_linear_idx  <= 0;
                    end
                end

                S_CHECK: begin
                    // 仅做检查
                end

                S_FETCH: begin
                    rd_en <= 1;
                end

                S_COMPUTE: begin
                    if (rd_elem_valid) begin
                        // 执行标量乘法
                        mult_result <= rd_elem * scalar_latched;
                        
                        // 输出结果（截断到 DATA_WIDTH）
                        out_valid <= 1;
                        out_elem  <= (rd_elem * scalar_latched) & {DATA_WIDTH{1'b1}};
                        
                        // 标志位
                        if (col_cnt == n_latched - 1)
                            out_row_end <= 1;
                        if (col_cnt == n_latched - 1 && row_cnt == m_latched - 1)
                            out_last <= 1;

                        // 计数器更新
                        out_linear_idx <= out_linear_idx + 1;
                        
                        if (col_cnt == n_latched - 1) begin
                            col_cnt <= 0;
                            if (row_cnt < m_latched - 1) begin
                                row_cnt <= row_cnt + 1;
                            end
                        end else begin
                            col_cnt <= col_cnt + 1;
                        end
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
