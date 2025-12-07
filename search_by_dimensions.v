// 维度筛选模块：根据请求的 (req_m, req_n) 输出所有匹配矩阵的槽位掩码及数量
// 该模块作为 matrix_storage 与后端运算/展示模块之间的过渡层

module search_by_dimensions #(
    // 模块参数配置
    parameter MAX_DIM    = 5,        // 支持的最大矩阵维度（例如5表示最大支持5x5矩阵）
    parameter MAX_STORE  = 2,        // 最大支持存储的矩阵数量
    parameter COUNT_BITS = (MAX_STORE <= 1) ? 1 : $clog2(MAX_STORE + 1)
) (
    // 时钟和复位信号（当前版本未使用，为未来扩展保留）
    input  wire                       clk,          
    input  wire                       rst,          

    // 查询请求接口
    // 一次仅查询一组维度，上层需保证 req_m/req_n 在整个查询过程中保持稳定
    input  wire [3:0]                 req_m,        // 请求匹配的矩阵行数
    input  wire [3:0]                 req_n,        // 请求匹配的矩阵列数

    // 来自 matrix_storage 的状态信息（展平后的行/列信息）
    // stored_m_flat 和 stored_n_flat 是将 matrix_storage 中的 stored_m_arr 和 stored_n_arr 
    // 展平为一维向量的结果，每4位表示一个存储槽的行数或列数
    input  wire [MAX_STORE*4-1:0]     stored_m_flat, // 各存储槽中矩阵的行数信息
    input  wire [MAX_STORE*4-1:0]     stored_n_flat, // 各存储槽中矩阵的列数信息
    input  wire [MAX_STORE-1:0]       slot_valid,    // 各存储槽的有效性标志（1表示有数据）

    // 输出：匹配结果
    output reg  [MAX_STORE-1:0]       match_mask,    // 逐槽匹配掩码，每一位对应一个存储槽，
                                                    // 1表示该槽中的矩阵与请求维度匹配
    output reg                        match_exists,  // 是否存在至少一个匹配矩阵（1表示存在）
    output reg  [COUNT_BITS-1:0]      match_count    // 匹配的矩阵数量（计数值）
);

    // 循环变量和临时存储变量
    integer idx;                    // 循环索引，用于遍历所有存储槽
    reg [3:0] stored_m_val;         // 从 stored_m_flat 中提取的当前存储槽的行数
    reg [3:0] stored_n_val;         // 从 stored_n_flat 中提取的当前存储槽的列数

    // 组合逻辑过程块：根据输入请求和存储状态计算匹配结果
    always @(*) begin
        // 初始化输出信号
        match_mask   = {MAX_STORE{1'b0}};    // 默认所有存储槽都不匹配
        match_count  = {COUNT_BITS{1'b0}};   // 默认匹配计数为0
        match_exists = 1'b0;                 // 默认不存在匹配矩阵

        // 遍历所有存储槽，检查每个槽中的矩阵是否与请求维度匹配
        for (idx = 0; idx < MAX_STORE; idx = idx + 1) begin
            // 从展平的向量中提取当前存储槽的行数和列数信息
            // idx*4 +: 4 表示从第 idx*4 位开始，提取连续的4位
            stored_m_val = stored_m_flat[idx*4 +: 4];
            stored_n_val = stored_n_flat[idx*4 +: 4];

            // 检查当前存储槽是否满足以下条件：
            // 1. 存储槽有效（slot_valid[idx] == 1）
            // 2. 矩阵行数与请求匹配（stored_m_val == req_m）
            // 3. 矩阵列数与请求匹配（stored_n_val == req_n）
            if (slot_valid[idx] &&
                stored_m_val == req_m &&
                stored_n_val == req_n) begin
                
                // 如果匹配，则设置对应的掩码位为1
                match_mask[idx] = 1'b1;
                // 增加匹配计数
                match_count     = match_count + 1'b1;
            end
        end

        // 如果匹配计数不为0，说明存在至少一个匹配矩阵
        if (match_count != 0)
            match_exists = 1'b1;
    end

endmodule

// 设计说明：
// 放置位置：将 search_by_dimensions.v 置于与 matrix_storage.v 同一层
// （例如 local/ 或 rtl/storage/），因为它直接消费存储模块导出的
// stored_m/stored_n/slot_valid，属于存储子系统的辅助逻辑。
// 后续在顶层或控制模块中统一例化 matrix_storage、search_by_dimensions 以及具体运算单元。
// 实例化关系：该模块不实例化 matrix_storage。
// 它只是读取 matrix_storage 已公开的输出（矩阵规格与有效位）
// 根据输入的单一维度对槽位进行筛选并给出掩码与数量。
// 顶层应先例化 matrix_storage，再将其 stored_m/stored_n/slot_valid
// 连接到 search_by_dimensions 的对应端口，然后上层 FSM/运算模块根据 match_mask
// 等结果决定如何取用具体矩阵数据。