// 负责矩阵数据的存储、覆盖与输入后处理（补零/忽略多余元素）
// 随机矩阵生成逻辑由外部 random_generator 模块提供

module matrix_storage #(
    parameter MAX_DIM    = 5,      // 最大矩阵维度大小
    parameter MAX_STORE  = 2,      // 最大可存储的矩阵数量
    parameter ELEM_WIDTH = 8       // 矩阵元素的数据宽度（bit）
) (
    input  wire clk,               // 主时钟信号
    input  wire rst,               // 高电平有效的复位信号

    // 写控制信号
    input  wire wen,                // 写使能：锁存新矩阵规格并清空对应存储槽
    input  wire [3:0] m,            // 矩阵行数，范围1~MAX_DIM
    input  wire [3:0] n,            // 矩阵列数，范围1~MAX_DIM

    // 矩阵元素输入
    input  wire [ELEM_WIDTH-1:0] elem_in,      // 输入的矩阵元素数据
    input  wire elem_valid,   // 元素有效信号，每个元素占用一个时钟周期

    // 状态输出信号
    output reg  [ELEM_WIDTH-1:0] matrix_store [0:MAX_STORE-1][0:MAX_DIM-1][0:MAX_DIM-1], // 矩阵存储器：[存储槽][行][列]
    output reg  [3:0] stored_m     [0:MAX_STORE-1], // 各存储槽中矩阵的实际行数
    output reg  [3:0] stored_n     [0:MAX_STORE-1], // 各存储槽中矩阵的实际列数
    output reg  [MAX_STORE-1:0]    slot_valid,      // 各存储槽的有效性的标志位
    output reg input_done                           // 当前矩阵写满 m*n 个元素时输出一个时钟周期的高电平脉冲
);

    // 计算存储槽地址所需的位数，存储为参数
    localparam SLOT_BITS = (MAX_STORE <= 1) ? 1 : $clog2(MAX_STORE);//理解成取对数

    // 写入状态控制信号
    reg [10:0] elem_cnt;         // 当前已写入的元素计数器
    reg [SLOT_BITS-1:0] fifo_ptr;         // FIFO覆盖指针，指向下一个可用的存储槽
    reg [SLOT_BITS-1:0] active_slot;      // 当前正在写入的存储槽索引
    reg [3:0] active_m;         // 当前正在写入的矩阵行数
    reg [3:0] active_n;         // 当前正在写入的矩阵列数
    reg active_valid;     // 当前存储槽是否处于有效写入状态
    
    // 循环变量，用于初始化存储器
    integer s, i, j;

    // 主要的状态机和控制逻辑
    always @(posedge clk or posedge rst) begin
        // 异步复位处理
        if (rst) begin
            elem_cnt    <= 0;              // 元素计数器清零
            fifo_ptr    <= 0;              // FIFO指针重置到第一个存储槽
            active_slot <= 0;              // 活动存储槽设为0
            active_m    <= 0;              // 活动矩阵行数清零
            active_n    <= 0;              // 活动矩阵列数清零
            active_valid<= 1'b0;           // 取消活动写入状态
            slot_valid  <= {MAX_STORE{1'b0}}; // 所有存储槽标记为无效，该语法意为创建一个长度为MAX_STORE的全为0的向量
            input_done  <= 1'b0;           // 写入完成信号设为0

            // 初始化所有存储矩阵为0
            for (s = 0; s < MAX_STORE; s = s + 1) begin
                stored_m[s] <= 0;          // 清除存储的行数信息
                stored_n[s] <= 0;          // 清除存储的列数信息
                for (i = 0; i < MAX_DIM; i = i + 1) begin
                    for (j = 0; j < MAX_DIM; j = j + 1) begin
                        matrix_store[s][i][j] <= {ELEM_WIDTH{1'b0}}; // 每个矩阵元素清零
                    end
                end
            end
        end else begin
            // 默认情况下清除写入完成信号
            input_done <= 1'b0;

            // 当收到写使能信号时，准备一个新的矩阵写入操作
            if (wen) begin
                active_slot  <= fifo_ptr;        // 设置当前活动存储槽
                active_m     <= m;               // 保存矩阵行数
                active_n     <= n;               // 保存矩阵列数
                active_valid <= 1'b1;            // 标记进入有效写入状态

                // 在对应存储槽中记录矩阵规格信息
                stored_m[fifo_ptr]   <= m;       // 保存行数到存储槽
                stored_n[fifo_ptr]   <= n;       // 保存列数到存储槽
                slot_valid[fifo_ptr] <= 1'b1;    // 标记该存储槽为有效

                // 清空目标存储槽中的旧数据
                for (i = 0; i < MAX_DIM; i = i + 1) begin
                    for (j = 0; j < MAX_DIM; j = j + 1) begin
                        matrix_store[fifo_ptr][i][j] <= {ELEM_WIDTH{1'b0}};
                    end
                end

                elem_cnt <= 0;                   // 元素计数器重置

                // 更新FIFO指针，实现循环存储（FIFO覆盖策略）
                if (fifo_ptr == MAX_STORE-1)
                    fifo_ptr <= 0;               // 回绕到第一个存储槽
                else
                    fifo_ptr <= fifo_ptr + 1'b1; // 指向下一个存储槽
            end

            // 当处于有效写入状态且有新元素输入时，存储矩阵元素
            if (active_valid && elem_valid && elem_cnt < active_m * active_n) begin
                // 按行优先顺序存储元素
                // elem_cnt / active_n 计算元素应存放的行号
                // elem_cnt % active_n 计算元素应存放的列号
                matrix_store[active_slot][elem_cnt / active_n][elem_cnt % active_n]
                    <= elem_in;
                elem_cnt <= elem_cnt + 1'b1;     // 元素计数器递增

                // 当所有元素都写入完毕时
                if (elem_cnt + 1 == active_m * active_n) begin
                    input_done <= 1'b1;          // 置位写入完成信号
                    active_valid <= 1'b0;        // 取消活动写入状态
                end
            end
        end
    end

endmodule