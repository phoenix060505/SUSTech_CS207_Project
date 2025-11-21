// 维度筛选模块：根据请求的 (req_m, req_n) 输出所有匹配矩阵的槽位码及数量
// 该模块作为 matrix_storage 与后端运算/展示模块之间的过渡层

module search_by_dimensions #(
    parameter MAX_DIM    = 5,       // 最大矩阵维度（与matrix_storage保持一致）
    parameter MAX_STORE  = 2,       // 最大存储矩阵数量（与matrix_storage保持一致）
    parameter ELEM_WIDTH = 8        // 元素宽度（与matrix_storage保持一致，便于系统一致性）
) (
    input  wire                       clk,          // 时钟信号
    input  wire                       rst,          // 复位信号

    // 查询请求（一次仅查询一组维度，上层需保证 req_m/req_n 稳定）
    input  wire [3:0]                 req_m,        // 请求匹配的矩阵行数
    input  wire [3:0]                 req_n,        // 请求匹配的矩阵列数

    // 来自 matrix_storage 的状态信息
    input  wire [3:0]                 stored_m   [0:MAX_STORE-1],  // 各存储槽中矩阵的实际行数
    input  wire [3:0]                 stored_n   [0:MAX_STORE-1],  // 各存储槽中矩阵的实际列数
    input  wire [MAX_STORE-1:0]       slot_valid,   // 各存储槽的有效性标志位

    // 输出：匹配结果
    output reg  [MAX_STORE-1:0]       match_mask,   // 逐槽匹配掩码，每一位代表对应槽位是否匹配
    output reg                        match_exists, // 是否存在至少一个匹配矩阵的标志
    output reg  [COUNT_BITS-1:0]      match_count   // 匹配的槽位数量计数
);

    // 计算匹配计数所需位数（最多可计数MAX_STORE个匹配项，所以需要log2(MAX_STORE+1)位）
    localparam COUNT_BITS = (MAX_STORE <= 1) ? 1 : $clog2(MAX_STORE + 1);

    // 循环索引变量
    integer idx;

    // 组合逻辑过程块：根据输入的维度请求，在所有存储槽中查找匹配项
    always @(*) begin
        // 初始化输出信号
        match_mask   = {MAX_STORE{1'b0}};     // 默认所有槽位都不匹配
        match_count  = {COUNT_BITS{1'b0}};    // 匹配计数器清零
        match_exists = 1'b0;                  // 默认不存在匹配项

        // 遍历所有存储槽，查找维度匹配的矩阵
        for (idx = 0; idx < MAX_STORE; idx = idx + 1) begin
            // 判断条件：存储槽有效 且 行数匹配 且 列数匹配
            if (slot_valid[idx] &&           
                stored_m[idx] == req_m &&    
                stored_n[idx] == req_n)
                begin
                    match_mask[idx] = 1'b1;      // 标记该槽位为匹配项
                    match_count     = match_count + 1'b1;  // 增加匹配计数
                end
        end

        // 如果匹配计数不为零，则表示存在匹配项
        if (match_count != 0)
            match_exists = 1'b1;
    end

endmodule

//放置位置：将search_by_dimensions.v 置于与 matrix_storage.v 同一层
//（例如 local/ 或 rtl/storage/），因为它直接消费存储模块导出的
//stored_m/stored_n/slot_valid，属于存储子系统的辅助逻辑。
//后续在顶层或控制模块中统一例化 matrix_storage、search_by_dimensions以及具体运算单元。
//实例化关系：该模块不实例化 matrix_storage。
//它只是读取 matrix_storage 已公开的输出（矩阵规格与有效位）
//根据输入的单一维度对槽位进行筛选并给出掩码与数量。
//顶层应先例化 matrix_storage，再将其 stored_m/stored_n/slot_valid
//连接到 search_by_dimensions 的对应端口，然后上层 FSM/运算模块根据 match_mask
//等结果决定如何取用具体矩阵数据。