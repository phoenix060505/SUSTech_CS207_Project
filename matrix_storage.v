// matrix_storage 模块负责管理多个矩阵的存储，支持写入新矩阵、读取已有矩阵元素
// 同时对输入数据进行后处理（如补零），并提供状态信息供外部模块查询
module matrix_storage #(
    // 模块参数配置
    parameter MAX_DIM    = 5,        // 支持的最大矩阵维度（例如5表示最大支持5x5矩阵）
    parameter MAX_STORE  = 2,        // 最大支持存储的矩阵数量
    parameter ELEM_WIDTH = 8,        // 矩阵元素的数据宽度（bit数）
    
    // 计算地址位宽参数
    parameter SLOT_BITS  = (MAX_STORE <= 1) ? 1 : $clog2(MAX_STORE),  // 存储槽索引所需位数
    parameter DIM_BITS   = (MAX_DIM   <= 1) ? 1 : $clog2(MAX_DIM)     // 行/列索引所需位数
) (
    // 时钟和复位信号
    input  wire                     clk,              // 主时钟
    input  wire                     rst,              // 高电平有效的同步复位信号
    
    // 写控制接口
    input  wire                     wen,              // 写使能信号，高电平时开始写入新的矩阵
    input  wire [3:0]               m,                // 当前写入矩阵的行数(1~MAX_DIM)
    input  wire [3:0]               n,                // 当前写入矩阵的列数(1~MAX_DIM)

    // 元素输入接口
    input  wire [ELEM_WIDTH-1:0]    elem_in,          // 输入的矩阵元素值
    input  wire                     elem_valid,       // 元素有效标志，每个元素占用一个时钟周期

    // 读接口：通过(slot,row,col)三元组读取单个元素
    input  wire                     rd_en,            // 读使能信号
    input  wire [SLOT_BITS-1:0]     rd_slot_idx,      // 目标存储槽索引
    input  wire [DIM_BITS-1:0]      rd_row_idx,       // 目标行索引
    input  wire [DIM_BITS-1:0]      rd_col_idx,       // 目标列索引
    output reg  [ELEM_WIDTH-1:0]    rd_elem,          // 读出的元素值
    output reg                      rd_elem_valid,    // 读出元素的有效标志

    // 状态输出接口（展平格式，便于外部模块解析）
    output wire [MAX_STORE*4-1:0]   stored_m_flat,    // 各存储槽中矩阵的行数信息（每4bit表示一个）
    output wire [MAX_STORE*4-1:0]   stored_n_flat,    // 各存储槽中矩阵的列数信息（每4bit表示一个）
    output reg  [MAX_STORE-1:0]     slot_valid,       // 各存储槽的有效性标志（1表示有数据）
    output reg                      input_done        // 当前矩阵完全写入完成时产生一个周期的脉冲
);

    // 写入状态寄存器
    reg [10:0]                elem_cnt;         // 已接收的元素计数器（足够大以容纳最大矩阵大小）
    reg [SLOT_BITS-1:0]       fifo_ptr;         // FIFO覆盖指针，指向下一个可写入的存储槽
    reg [SLOT_BITS-1:0]       active_slot;      // 正在写入的存储槽编号
    reg [3:0]                 active_m;         // 正在写入矩阵的行数
    reg [3:0]                 active_n;         // 正在写入矩阵的列数
    reg                       active_valid;     // 当前是否有正在进行的写操作

    // 内部存储结构
    reg [3:0]                 stored_m_arr [0:MAX_STORE-1];  // 各存储槽中矩阵的实际行数
    reg [3:0]                 stored_n_arr [0:MAX_STORE-1];  // 各存储槽中矩阵的实际列数
    reg [ELEM_WIDTH-1:0]      matrix_mem   [0:MAX_STORE-1][0:MAX_DIM-1][0:MAX_DIM-1];  // 矩阵存储内存

    // 循环变量声明
    integer s, i, j;

    // 将存储规格信息展平为一维向量输出，避免多维端口带来的综合问题
    genvar gv;
    generate
        for (gv = 0; gv < MAX_STORE; gv = gv + 1) begin : GEN_FLAT
            assign stored_m_flat[4*gv +: 4] = stored_m_arr[gv];  // 提取第gv个存储槽的行数
            assign stored_n_flat[4*gv +: 4] = stored_n_arr[gv];  // 提取第gv个存储槽的列数
        end
    endgenerate

    // 写入控制与时序逻辑（同步于时钟和复位）
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // 复位状态下初始化所有状态和存储内容
            elem_cnt    <= 0;
            fifo_ptr    <= 0;
            active_slot <= 0;
            active_m    <= 0;
            active_n    <= 0;
            active_valid<= 1'b0;
            slot_valid  <= {MAX_STORE{1'b0}};
            input_done  <= 1'b0;

            // 清空所有存储槽中的矩阵数据
            for (s = 0; s < MAX_STORE; s = s + 1) begin
                stored_m_arr[s] <= 0;
                stored_n_arr[s] <= 0;
                for (i = 0; i < MAX_DIM; i = i + 1) begin
                    for (j = 0; j < MAX_DIM; j = j + 1) begin
                        matrix_mem[s][i][j] <= {ELEM_WIDTH{1'b0}};  // 所有元素清零
                    end
                end
            end
        end else begin
            input_done <= 1'b0;  // 默认情况下input_done为低电平

            // 接收到写请求时准备写入新矩阵
            if (wen) begin
                active_slot  <= fifo_ptr;           // 设置当前活动存储槽
                active_m     <= m;                  // 保存目标矩阵行数
                active_n     <= n;                  // 保存目标矩阵列数
                active_valid <= 1'b1;               // 标记正在写入

                // 更新对应存储槽的状态信息
                stored_m_arr[fifo_ptr] <= m;
                stored_n_arr[fifo_ptr] <= n;
                slot_valid[fifo_ptr]   <= 1'b1;

                // 初始化目标存储槽的内存空间为0（自动补零）
                for (i = 0; i < MAX_DIM; i = i + 1) begin
                    for (j = 0; j < MAX_DIM; j = j + 1) begin
                        matrix_mem[fifo_ptr][i][j] <= {ELEM_WIDTH{1'b0}};
                    end
                end

                elem_cnt <= 0;  // 重置元素计数器

                // 更新FIFO指针（循环递增）
                if (fifo_ptr == MAX_STORE-1)
                    fifo_ptr <= 0;
                else
                    fifo_ptr <= fifo_ptr + 1'b1;
            end

            // 在有效写入阶段接收并存储元素
            if (active_valid && elem_valid && elem_cnt < active_m * active_n) begin
                // 按行优先顺序将元素写入内存
                matrix_mem[active_slot][elem_cnt / active_n][elem_cnt % active_n] <= elem_in;
                elem_cnt <= elem_cnt + 1'b1;

                // 如果已完成整个矩阵的写入，则标记完成并结束本次写入
                if (elem_cnt + 1 == active_m * active_n) begin
                    input_done   <= 1'b1;
                    active_valid <= 1'b0;
                end
            end
        end
    end

    // 读接口逻辑（组合逻辑实现即时响应）
    always @(*) begin
        rd_elem       = {ELEM_WIDTH{1'b0}};  // 默认输出为0
        rd_elem_valid = 1'b0;                // 默认无效

        // 若读使能且目标存储槽合法
        if (rd_en && rd_slot_idx < MAX_STORE) begin
            // 检查访问位置是否有效（槽有效且行列索引不越界）
            if (slot_valid[rd_slot_idx] &&
                rd_row_idx < stored_m_arr[rd_slot_idx] &&
                rd_col_idx < stored_n_arr[rd_slot_idx]) begin
                rd_elem       = matrix_mem[rd_slot_idx][rd_row_idx][rd_col_idx];  // 返回对应元素
                rd_elem_valid = 1'b1;  // 标记读取有效
            end
        end
    end

endmodule