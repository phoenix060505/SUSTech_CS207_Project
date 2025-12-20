module matrix_storage #(
    parameter MAX_DIM       = 5,
    parameter SLOTS_PER_DIM = 2,
    parameter ELEM_WIDTH    = 8,
    parameter NUM_DIM_COMBOS = MAX_DIM * MAX_DIM,
    parameter TOTAL_SLOTS    = NUM_DIM_COMBOS * SLOTS_PER_DIM,
    parameter DIM_BITS       = 3
) (
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     wen,
    input  wire [3:0]               m,
    input  wire [3:0]               n,
    input  wire [ELEM_WIDTH-1:0]    elem_in,
    input  wire                     elem_valid,
    input  wire                     rd_en,
    input  wire [3:0]               rd_m,
    input  wire [3:0]               rd_n,
    input  wire                     rd_slot_idx,
    input  wire [DIM_BITS-1:0]      rd_row_idx,
    input  wire [DIM_BITS-1:0]      rd_col_idx,
    output reg  [ELEM_WIDTH-1:0]    rd_elem,
    output reg                      rd_elem_valid,
    input  wire [3:0]               query_m,
    input  wire [3:0]               query_n,
    output wire [1:0]               query_count,
    output wire                     query_slot0_valid,
    output wire                     query_slot1_valid,
    output reg                      input_done
);

    (* ram_style = "block" *)
    reg [ELEM_WIDTH-1:0] matrix_mem [0:TOTAL_SLOTS-1][0:MAX_DIM-1][0:MAX_DIM-1];
    
    reg [SLOTS_PER_DIM-1:0] dim_slot_valid [0:NUM_DIM_COMBOS-1];
    reg fifo_ptr [0:NUM_DIM_COMBOS-1];
    
    reg [10:0]              elem_cnt;
    reg [5:0]               active_slot;
    reg [3:0]               active_m;
    reg [3:0]               active_n;
    reg                     active_valid;
    reg [3:0]               write_row;
    reg [3:0]               write_col;
    
    integer s;

    // 辅助函数：维度映射到索引 (0~24)
    function [4:0] get_dim_combo;
        input [3:0] dm, dn;
        begin
            if (dm >= 1 && dm <= MAX_DIM && dn >= 1 && dn <= MAX_DIM)
                get_dim_combo = (dm - 1) * MAX_DIM + (dn - 1);
            else
                get_dim_combo = 0;
        end
    endfunction

    wire [4:0] wen_dim_combo   = get_dim_combo(m, n);
    wire [4:0] query_dim_combo = get_dim_combo(query_m, query_n);
    wire [4:0] rd_dim_combo    = get_dim_combo(rd_m, rd_n);

    // 查询逻辑
    assign query_slot0_valid = dim_slot_valid[query_dim_combo][0];
    assign query_slot1_valid = dim_slot_valid[query_dim_combo][1];
    assign query_count = query_slot0_valid + query_slot1_valid;

    // ========== 写入逻辑 ==========
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            elem_cnt       <= 0;
            active_slot    <= 0;
            active_m       <= 0;
            active_n       <= 0;
            active_valid   <= 1'b0;
            input_done     <= 1'b0;
            write_row      <= 0;
            write_col      <= 0;
            
            // 复位有效位和指针
            for (s = 0; s < NUM_DIM_COMBOS; s = s + 1) begin
                dim_slot_valid[s] <= 2'b00;
                fifo_ptr[s] <= 1'b0;
            end
        end else begin
            input_done <= 1'b0; // 默认拉低

            // Start Write
            if (wen && !active_valid) begin
                if (m >= 1 && m <= MAX_DIM && n >= 1 && n <= MAX_DIM) begin
                    active_m     <= m;
                    active_n     <= n;
                    active_valid <= 1'b1;
                    elem_cnt     <= 0;
                    write_row    <= 0;
                    write_col    <= 0;
                    
                    // 计算目标 Slot
                    active_slot <= wen_dim_combo * SLOTS_PER_DIM + fifo_ptr[wen_dim_combo];
                    
                    // 更新有效位和指针
                    dim_slot_valid[wen_dim_combo][fifo_ptr[wen_dim_combo]] <= 1'b1;
                    fifo_ptr[wen_dim_combo] <= ~fifo_ptr[wen_dim_combo];
                    
                    // 这里千万不要用 for 循环清空 matrix_mem！
                    // 直接覆盖旧数据即可，dim_slot_valid 会保证读取安全。
                end
            end
            
            // Write Data
            if (active_valid && elem_valid) begin
                if (elem_cnt < active_m * active_n) begin
                    matrix_mem[active_slot][write_row][write_col] <= elem_in;
                    elem_cnt <= elem_cnt + 1;
                    
                    // 更新行列地址
                    if (write_col == active_n - 1) begin
                        write_col <= 0;
                        write_row <= write_row + 1;
                    end else begin
                        write_col <= write_col + 1;
                    end
                    
                    // 完成判断
                    if (elem_cnt + 1 == active_m * active_n) begin
                        input_done   <= 1'b1;
                        active_valid <= 1'b0;
                    end
                end else begin
                    // 输入数据个数超过维度乘积，忽略多余数据
                    // 不更新计数器，不写入内存，等待输入完成信号
                end
            end
            
            // 处理数据不足的情况：当UART输入完成但数据不足时，自动补0
            // 这里需要添加超时检测机制来判断输入是否完成
            // 暂时保持原逻辑，由FSM负责检测输入完成并触发补0
        end
    end
    
    // ========== 读取逻辑 ==========
    wire [5:0] rd_global_slot = rd_dim_combo * SLOTS_PER_DIM + rd_slot_idx;
    reg rd_en_d;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rd_en_d       <= 1'b0;
            rd_elem       <= 0;
            rd_elem_valid <= 1'b0;
        end else begin
            rd_en_d <= rd_en;
            rd_elem_valid <= 1'b0; // 默认脉冲低
            
            // 上升沿触发读取
            if (rd_en && !rd_en_d) begin
                // 只有当维度有效时才输出数据
                if (dim_slot_valid[rd_dim_combo][rd_slot_idx]) begin
                    rd_elem <= matrix_mem[rd_global_slot][rd_row_idx][rd_col_idx];
                    rd_elem_valid <= 1'b1;
                end else begin
                    rd_elem <= 8'h00; // 无效槽读出0
                    // rd_elem_valid 保持 0，或者你可以置 1 但数据为 0，看 FSM 怎么处理
                    // 这里保持 0，FSM 会打印 '?'，方便调试
                end
            end
        end
    end

endmodule