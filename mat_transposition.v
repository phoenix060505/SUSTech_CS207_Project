// 矩阵转置输出模块：从 matrix_storage 中选定槽位读取，按转置顺序逐元素输出
module mat_transposition #(
    parameter MAX_DIM    = 5,
    parameter MAX_STORE  = 2,
    parameter ELEM_WIDTH = 8,
    parameter SLOT_BITS  = (MAX_STORE <= 1) ? 1 : $clog2(MAX_STORE),
    parameter DIM_BITS   = (MAX_DIM   <= 1) ? 1 : $clog2(MAX_DIM)
)(
    input  wire                     clk,
    input  wire                     rst,          // 同步高有效复位
    // 控制接口
    input  wire                     start,        // 触发一次转置输出
    input  wire [SLOT_BITS-1:0]     slot_select,  // 目标存储槽
    output reg                      busy,         // 转置进行中
    output reg                      done,         // 本次完成脉冲（1周期）

    // 连接 matrix_storage 的状态展平信息
    input  wire [MAX_STORE*4-1:0]   stored_m_flat,
    input  wire [MAX_STORE*4-1:0]   stored_n_flat,
    input  wire [MAX_STORE-1:0]     slot_valid,

    // 连接 matrix_storage 的读端口
    output reg                      rd_en,
    output reg  [SLOT_BITS-1:0]     rd_slot_idx,
    output reg  [DIM_BITS-1:0]      rd_row_idx,
    output reg  [DIM_BITS-1:0]      rd_col_idx,
    input  wire [ELEM_WIDTH-1:0]    rd_elem,
    input  wire                     rd_elem_valid,

    // 转置后元素流输出 (A^T[row_t][col_t])
    output reg                      out_valid,
    output reg  [DIM_BITS-1:0]      out_row_t,    // 转置后行索引
    output reg  [DIM_BITS-1:0]      out_col_t,    // 转置后列索引
    output reg  [ELEM_WIDTH-1:0]    out_elem,

    // 错误指示
    output reg                      slot_error    // 非法槽或无效槽
);

    // 取得所选槽位的原矩阵尺寸
    wire [3:0] m_sel = stored_m_flat[slot_select*4 +: 4];
    wire [3:0] n_sel = stored_n_flat[slot_select*4 +: 4];

    // 计数器：遍历转置后 (row_t, col_t)
    reg [3:0] row_t_cnt;
    reg [3:0] col_t_cnt;

    // 读取延迟同步：由于 rd_elem_valid 为组合逻辑输出，这里假设稳定，
    // 为更稳健加入一拍寄存对齐输出
    reg [ELEM_WIDTH-1:0] elem_d;
    reg                  rd_valid_d;

    // 状态机
    localparam S_IDLE  = 3'd0;
    localparam S_CHECK = 3'd1;
    localparam S_READ  = 3'd2;
    localparam S_NEXT  = 3'd3;
    localparam S_DONE  = 3'd4;

    reg [2:0] state, state_n;

    // 状态转移
    always @(*) begin
        state_n = state;
        case (state)
            S_IDLE:  if (start) state_n = S_CHECK;
            S_CHECK: begin
                if (slot_error)       state_n = S_DONE;
                else if (m_sel == 0 || n_sel == 0) state_n = S_DONE;
                else                   state_n = S_READ;
            end
            S_READ:  state_n = S_NEXT;
            S_NEXT: begin
                if (row_t_cnt == n_sel-1 && col_t_cnt == m_sel-1)
                    state_n = S_DONE;
                else
                    state_n = S_READ;
            end
            S_DONE:  state_n = S_IDLE;
            default: state_n = S_IDLE;
        endcase
    end

    // 主时序
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state       <= S_IDLE;
            busy        <= 1'b0;
            done        <= 1'b0;
            rd_en       <= 1'b0;
            rd_slot_idx <= {SLOT_BITS{1'b0}};
            rd_row_idx  <= {DIM_BITS{1'b0}};
            rd_col_idx  <= {DIM_BITS{1'b0}};
            out_valid   <= 1'b0;
            out_row_t   <= {DIM_BITS{1'b0}};
            out_col_t   <= {DIM_BITS{1'b0}};
            out_elem    <= {ELEM_WIDTH{1'b0}};
            row_t_cnt   <= 4'd0;
            col_t_cnt   <= 4'd0;
            slot_error  <= 1'b0;
            elem_d      <= {ELEM_WIDTH{1'b0}};
            rd_valid_d  <= 1'b0;
        end else begin
            state <= state_n;
            done  <= 1'b0;
            out_valid <= 1'b0;
            rd_en <= 1'b0;
            rd_valid_d <= rd_elem_valid;
            elem_d <= rd_elem;

            case (state)
                S_IDLE: begin
                    busy       <= 1'b0;
                    slot_error <= 1'b0;
                    if (start) begin
                        rd_slot_idx <= slot_select;
                    end
                end
                S_CHECK: begin
                    busy <= 1'b1;
                    // 检查槽有效性与范围
                    if (slot_select >= MAX_STORE || !slot_valid[slot_select])
                        slot_error <= 1'b1;
                    // 初始化遍历计数器
                    row_t_cnt <= 4'd0;
                    col_t_cnt <= 4'd0;
                end
                S_READ: begin
                    // 对于转置后位置 (row_t_cnt, col_t_cnt) 需访问原矩阵元素 (col_t_cnt, row_t_cnt)
                    rd_en      <= 1'b1;
                    rd_row_idx <= col_t_cnt[DIM_BITS-1:0];
                    rd_col_idx <= row_t_cnt[DIM_BITS-1:0];
                end
                S_NEXT: begin
                    // 发出对应的转置元素
                    if (rd_valid_d) begin
                        out_valid <= 1'b1;
                        out_row_t <= row_t_cnt[DIM_BITS-1:0];
                        out_col_t <= col_t_cnt[DIM_BITS-1:0];
                        out_elem  <= elem_d;
                    end
                    // 迭代转置坐标：行优先遍历转置矩阵大小 = n_sel x m_sel
                    if (row_t_cnt == n_sel-1) begin
                        row_t_cnt <= 0;
                        if (col_t_cnt != m_sel-1)
                            col_t_cnt <= col_t_cnt + 1'b1;
                    end else begin
                        row_t_cnt <= row_t_cnt + 1'b1;
                    end
                end
                S_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end
            endcase
        end
    end

endmodule

// 简易测试平台示例（可选）
// 使用时请确认已例化 matrix_storage 并正确连接其读端口。
// 可以在顶层把本模块的 rd_* 直接连到 matrix_storage 的对应读接口。