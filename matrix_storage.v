// 矩阵存储模块
module matrix_storage #(
    parameter MAX_DIM       = 5,        // 最大矩阵维度 (1~5)
    parameter SLOTS_PER_DIM = 2,        // 每个维度组合的存储槽位数
    parameter ELEM_WIDTH    = 8,        // 矩阵元素位宽 (8位，0~255)
    parameter NUM_DIM_COMBOS = MAX_DIM * MAX_DIM,  // 维度组合总数 (5x5=25)
    parameter TOTAL_SLOTS    = NUM_DIM_COMBOS * SLOTS_PER_DIM, // 总存储槽位 (25x2=50)
    parameter DIM_BITS       = 3        // 行列索引位宽 (最大支持7)
) (
    // 系统接口
    input  wire                     clk,        // 系统时钟 (100MHz)
    input  wire                     rst,        // 同步复位 (高电平有效)
    
    // 写入接口
    input  wire                     wen,        // 写入使能 (脉冲信号，启动新矩阵写入)
    input  wire [3:0]               m,          // 写入矩阵的行数 (1~5)
    input  wire [3:0]               n,          // 写入矩阵的列数 (1~5)
    input  wire [ELEM_WIDTH-1:0]    elem_in,    // 写入的矩阵元素数据
    input  wire                     elem_valid, // 元素数据有效信号
    input  wire                     store_abort, // 存储中止信号
    output reg                      input_done, // 矩阵写入完成标志
    
    // 读取接口
    input  wire                     rd_en,      // 读取使能 (上升沿触发)
    input  wire [3:0]               rd_m,       // 读取矩阵的行数
    input  wire [3:0]               rd_n,       // 读取矩阵的列数
    input  wire                     rd_slot_idx,// 读取的槽位索引 (0或1)
    input  wire [DIM_BITS-1:0]      rd_row_idx, // 读取的行索引 (0~4)
    input  wire [DIM_BITS-1:0]      rd_col_idx, // 读取的列索引 (0~4)
    output reg  [ELEM_WIDTH-1:0]    rd_elem,    // 读取的矩阵元素数据
    output reg                      rd_elem_valid, // 读取数据有效标志
    
    // 查询接口
    input  wire [3:0]               query_m,    // 查询的行数
    input  wire [3:0]               query_n,    // 查询的列数
    output wire [1:0]               query_count,// 该维度下有效矩阵数量 (0/1/2)
    output wire                     query_slot0_valid, // 槽位0是否有效
    output wire                     query_slot1_valid  // 槽位1是否有效
);

    // 存储结构定义
    // 矩阵数据存储为三维数组，索引为槽位, 行, 列
    (* ram_style = "block" *) //属性声明，将数组映射到块RAM
    reg [ELEM_WIDTH-1:0] matrix_mem [0:TOTAL_SLOTS-1][0:MAX_DIM-1][0:MAX_DIM-1];
    
    // 存储管理寄存器
    reg [SLOTS_PER_DIM-1:0] dim_slot_valid [0:NUM_DIM_COMBOS-1]; // 每种维度的槽有效标志
    reg fifo_ptr [0:NUM_DIM_COMBOS-1];                           // FIFO替换策略指针
    
    // 写入控制寄存器
    reg [10:0]              elem_cnt;       // 当前写入的元素计数 (最大支持1023个元素)
    reg [5:0]               active_slot;    // 当前激活的全局槽位索引 (0~49)
    reg [3:0]               active_m;       // 当前写入矩阵的行数
    reg [3:0]               active_n;       // 当前写入矩阵的列数
    reg                     active_valid;   // 当前写入会话有效标志
    reg [3:0]               write_row;      // 当前写入的行索引
    reg [3:0]               write_col;      // 当前写入的列索引
    
    integer s;  // 循环变量，用于复位初始化

    // 函数：将维度映射到索引
    // 将矩阵维度(m,n)映射到维度combo索引(0~24)
    // 公式：(m-1)*5 + (n-1)
    function [4:0] get_dim_combo;
        input [3:0] dm, dn;  // 输入维度
        begin
            if (dm >= 1 && dm <= MAX_DIM && dn >= 1 && dn <= MAX_DIM)
                get_dim_combo = (dm - 1) * MAX_DIM + (dn - 1);
            else
                get_dim_combo = 0;  // 无效维度返回0
        end
    endfunction

    // 维度组合指代双维度索引
    wire [4:0] wen_dim_combo   = get_dim_combo(m, n);       // 写入维度组合索引
    wire [4:0] query_dim_combo = get_dim_combo(query_m, query_n); // 查询维度组合索引
    wire [4:0] rd_dim_combo    = get_dim_combo(rd_m, rd_n); // 读取维度组合索引

    // 查询逻辑
    assign query_slot0_valid = dim_slot_valid[query_dim_combo][0]; // 槽位0有效性
    assign query_slot1_valid = dim_slot_valid[query_dim_combo][1]; // 槽位1有效性
    assign query_count = query_slot0_valid + query_slot1_valid;    // 有效矩阵总数

    // 写入逻辑
    // 处理矩阵数据的写入操作：复位与初始化-检测wen，开始写入-按顺序写入-检测写入是否完成
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // 复位初始化
            elem_cnt       <= 0;
            active_slot    <= 0;
            active_m       <= 0;
            active_n       <= 0;
            active_valid   <= 1'b0;
            input_done     <= 1'b0;
            write_row      <= 0;
            write_col      <= 0;
            for (s = 0; s < NUM_DIM_COMBOS; s = s + 1) begin
                dim_slot_valid[s] <= 2'b00;
                fifo_ptr[s] <= 1'b0;
            end
        end else begin
            input_done <= 1'b0; // 默认为写入完成

            // 开始新的写入会话
            if (wen && !active_valid) begin
                if (m >= 1 && m <= MAX_DIM && n >= 1 && n <= MAX_DIM) begin
                    active_m     <= m;          // 锁存行数
                    active_n     <= n;          // 锁存列数
                    active_valid <= 1'b1;       // 激活写入会话
                    elem_cnt     <= 0;          // 元素计数从0开始
                    write_row    <= 0;          // 行索引从0开始
                    write_col    <= 0;          // 列索引从0开始
                    
                    // 槽位分配
                    // 目标Slot：使用FIFO替换槽位
                    active_slot <= wen_dim_combo * SLOTS_PER_DIM + fifo_ptr[wen_dim_combo];
                    
                    // 更新有效位和指针：标记槽位有效，翻转FIFO指针
                    dim_slot_valid[wen_dim_combo][fifo_ptr[wen_dim_combo]] <= 1'b1;
                    fifo_ptr[wen_dim_combo] <= ~fifo_ptr[wen_dim_combo];
                    
                    // 重要：不要用for循环清空matrix_mem！
                    // 直接覆盖旧数据即可，dim_slot_valid会保证读取安全。
                end
            end
            
            // 处理存储中止信号：当收到中止信号时，将当前槽位标记为无效
            if (active_valid && store_abort) begin
                // 将当前激活的槽位标记为无效
                // 计算当前激活槽位对应的槽位索引 (0 或 1)
                // active_slot = wen_dim_combo * 2 + slot_index，所以 slot_index = active_slot[0]
                dim_slot_valid[wen_dim_combo][active_slot[0]] <= 1'b0;
                active_valid <= 1'b0;    // 结束写入会话
            end
            
            // 数据写入操作
            if (active_valid && elem_valid) begin
                if (elem_cnt < active_m * active_n) begin
                    // 写入数据到矩阵存储器
                    matrix_mem[active_slot][write_row][write_col] <= elem_in;
                    elem_cnt <= elem_cnt + 1;  // 递增元素计数
                    
                    // 行列索引更新：列优先遍历（从左到右，从上到下）
                    if (write_col == active_n - 1) begin
                        write_col <= 0;          // 列复位
                        write_row <= write_row + 1; // 行递增
                    end else begin
                        write_col <= write_col + 1; // 列递增
                    end
                    
                    // 写入完成检测
                    if (elem_cnt + 1 == active_m * active_n) begin
                        input_done   <= 1'b1;    // 产生完成脉冲
                        active_valid <= 1'b0;    // 结束写入会话
                    end
                end else begin
                    // 输入数据个数超过维度乘积，忽略多余数据
                    // 不更新计数器，不写入内存，等待输入完成信号
                end
            end
            
            // 处理数据不足的情况：当UART输入完成但数据不足时，自动补0
            // 由FSM负责检测输入完成和补0
        end
    end
    
    // 读取逻辑
    // 功能：提供矩阵数据的同步读取接口，支持按行列索引访问
    
    // 全局槽位计算：根据维度组合和槽位选择计算全局槽位索引
    // 公式：槽位索引=维度combo索引 × 2 + 槽位选择(0或1)
    wire [5:0] rd_global_slot = rd_dim_combo * SLOTS_PER_DIM + rd_slot_idx;
    
    // 有效数据判断：检查所选槽位是否包含有效数据
    // 防止读取无效或未初始化的矩阵数据
    wire rd_valid = dim_slot_valid[rd_dim_combo][rd_slot_idx];
    
    reg rd_en_d; // 读使能delay，用于在上升沿锁存前一周期的读使能，实现其上升沿检测

    // 在时钟上升沿检测读使能信号的上升沿
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // 复位
            rd_en_d       <= 1'b0;
            rd_elem       <= 0;
            rd_elem_valid <= 1'b0;
        end else begin
            rd_en_d <= rd_en; // 读使能delay：在每个上升沿，用<=锁存上一周期的读使能信号
            rd_elem_valid <= 1'b0; // 默认脉冲低
            
            // 上升沿触发读取操作：仅在读使能信号的上升沿才进入后续读取操作
            if (rd_en && !rd_en_d) begin
                // 检查读取有效性：槽位有效且行列索引在矩阵范围内
                if (rd_valid && rd_row_idx < rd_m && rd_col_idx < rd_n) begin
                    // 情况1：有效读取，输出矩阵元素数据
                    rd_elem <= matrix_mem[rd_global_slot][rd_row_idx][rd_col_idx];
                    rd_elem_valid <= 1'b1;  // 数据有效标志
                end else begin
                    // 情况2：无效读取，输出0并标记无效
                    rd_elem <= 8'h00; // 无效槽读出0
                    rd_elem_valid <= 1'b0;  // 数据无效标志
                end
            end
        end
    end

endmodule