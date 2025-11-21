// 顶层模块：矩阵计算器整体控制
// 支持矩阵维度MAX_DIM=5, 存储上限MAX_STORE=2
// 输入：时钟、复位、拨码开关、按键、UART接收
// 输出：LED错误指示、7段数码管、UART发送
module matrix_calculator_top #(     //模块化参数，括号前加#
    parameter MAX_DIM = 5,          // 矩阵最大维度（1~5）
    parameter MAX_STORE = 2,        // 每规格矩阵存储上限（默认2，可配置）
    parameter ELEM_WIDTH = 8,       // 矩阵元素位宽（0~9整数）
    parameter TIMEOUT_DEFAULT = 10  // 默认倒计时秒数（5~15可配置）
) (
    input wire clk,                 // 系统时钟（50MHz或100MHz）
    input wire rst,                 // 异步复位，高有效
    input wire [7:0] sw,            // 拨码开关：模式选择或运算类型
    input wire [3:0] btn,           // 按键：btn[0]确认键
    input wire uart_rx,             // UART接收线：矩阵数据输入
    output reg [7:0] led,           // LED输出：led[0]错误指示
    output reg [7:0] seg,           // 7段数码管段码输出
    output reg [3:0] an,            // 7段数码管阳极选位（动态扫描）
    output wire uart_tx             // UART发送线：结果/矩阵展示输出
);

// 内部信号定义总体注释：声明wire/reg信号，用于模块间连接和临时存储
// 原理：wire用于组合连接，reg用于时序保持
// 目的：路由FSM输出到子模块，存储矩阵数据，管理倒计时和合法检查
wire [1:0] mode;                    // FSM输出的系统模式
wire [3:0] op_type;                 // FSM输出的运算类型
wire timeout_en;                    // FSM输出的倒计时使能
wire wen_store;                     // FSM输出的存储写使能
wire start_compute;                 // FSM输出的计算启动信号
wire uart_valid;                    // UART接收有效信号
wire [7:0] uart_data_out;           // UART接收到的字节数据
wire input_done;                    // 输入/生成完成信号（简化假设1）
wire display_done;                  // 展示完成信号（简化假设1）
wire operand_legal;                 // 运算数合法信号（从检查逻辑）
wire compute_done;                  // 计算完成信号（从运算模块）
wire timeout_expired;               // 超时到期信号（cnt==0）
wire rst_n = ~rst;
wire tx_busy;

reg [ELEM_WIDTH-1:0] matrix_store[MAX_STORE-1:0][MAX_DIM-1:0][MAX_DIM-1:0]; // 矩阵存储数组（reg实现，覆盖逻辑FIFO）
reg [3:0] m, n;                     // 当前处理的矩阵维度
reg [31:0] timeout_cnt;             // 倒计时计数器（clk分频到秒级）
reg [3:0] timeout_sec = TIMEOUT_DEFAULT; // 当前倒计时秒值
reg error_flag;                     // 错误标志，用于led[0]

// 子模块实例化总体注释：连接所有功能模块，形成完整数据路径
// 原理：顶层wire连接子模块端口，实现控制流（FSM）和数据流（存储/运算）
// 目的：整合IO、控制、通信、存储和计算
fsm_controller fsm_inst (           // FSM核心控制器
    .clk(clk),
    .rst(rst),
    .sw(sw),
    .btn(btn),
    .uart_valid(uart_valid),
    .input_done(input_done),
    .display_done(display_done),
    .operand_legal(operand_legal),
    .compute_done(compute_done),
    .timeout_expired(timeout_expired),
    .mode(mode),
    .op_type(op_type),
    .timeout_en(timeout_en),
    .wen_store(wen_store),
    .start_compute(start_compute)
);

uart_rx #(.CLK_FREQ(100_000_000), .BAUD_RATE(115200)) uart_rx_inst (
    .clk(clk),
    .rst_n(rst_n),
    .rx(uart_rx),
    .rx_data(uart_data_out),
    .rx_done(uart_valid)
);

uart_tx #(.CLK_FREQ(100_000_000), .BAUD_RATE(115200)) uart_tx_inst (
    .clk(clk),
    .rst_n(rst_n),
    .tx_start(uart_start),
    .tx_data(uart_data_in),
    .tx(uart_tx),
    .tx_busy(tx_busy)
);

matrix_storage matrix_store_inst (  // 矩阵存储模块：支持输入/生成写入，覆盖旧矩阵
    .clk(clk),
    .rst(rst),
    .wen(wen_store),                // 写使能输入
    .m(m),
    .n(n),
    .elem_in(uart_data_out),        // 元素输入（从UART）
    .matrix_out(matrix_store),      // 输出所有存储矩阵
    .input_done(input_done)         // 完成信号输出
);

transpose_module transpose_inst (   // 转置模块（仅实例化）
    .in_matrix(/*从存储*/),
    .m(m), .n(n),
    .out_matrix(/*到UART*/)
);

add_module add_inst (               // 加法模块（仅实例化）
    .in_a(/*矩阵A*/), .in_b(/*矩阵B*/),
    .m(m), .n(n),
    .out_matrix(/*结果*/)
);

scalar_mul_module scalar_mul_inst ( // 标量乘模块（仅实例化）
    .in_matrix(/*矩阵*/),
    .scalar(/*从sw或随机*/),
    .m(m), .n(n),
    .out_matrix(/*结果*/)
);

mat_mul_module mat_mul_inst (       // 矩阵乘模块（仅实例化）
    .in_a(/*A*/), .in_b(/*B*/),
    .m(m), .n(/*中间维度*/), .p(/*列*/),
    .out_matrix(/*结果*/)
);

conv_module conv_inst (             // bonus卷积模块（仅实例化）
    .clk(clk),
    .kernel(/*3x3从UART*/),
    .out_matrix(/*8x10结果*/),
    .cycle_count(/*到数码管*/)
);

input_image_rom rom_inst (          // bonus图像ROM（仅实例化）
    .clk(clk),
    .x(/*行地址*/), .y(/*列地址*/),
    .data_out(/*4bit元素到卷积*/)
);

// 时钟域逻辑块总体注释：处理倒计时、错误LED和数码管显示
// 原理：同步时序逻辑，分频clk实现秒级倒计时，动态扫描数码管
// 目的：实现项目要求的错误超时（5~15s）、LED指示、数码管显示类型/倒计时
// 使用端口/信号：timeout_en（启动）、op_type/timeout_sec（显示内容）
always @(posedge clk or posedge rst) begin
    if (rst) begin
        timeout_cnt <= 0;
        timeout_sec <= TIMEOUT_DEFAULT;
        led <= 8'b0;
        error_flag <= 1'b0;
        an <= 4'b1110;                  // 初始选首位
    end else begin
        // 倒计时分频逻辑（假设50MHz clk，50000000周期=1s）
        if (timeout_cnt < 50000000) timeout_cnt <= timeout_cnt + 1;
        else begin
            timeout_cnt <= 0;
            if (timeout_en && timeout_sec > 0) begin
                timeout_sec <= timeout_sec - 1;  // 秒递减
            end
        end
        if (timeout_sec == 0 && timeout_en) timeout_expired = 1'b1;  // 超时信号（wire需reg驱动）
        
        // 错误LED：任何非法设置error_flag，点亮led[0]
        if (error_flag) led[0] <= 1'b1;
        
        // 数码管动态扫描（4位轮询）
        an <= {an[2:0], an[3]};         // 移位选位
        case (an)
            4'b1110: seg <= op_type;    // 显示运算类型（简化码表）
            4'b1101: seg <= timeout_sec[3:0];  // 显示倒计时
            // 其他位扩展
        endcase
    end
end

// 组合逻辑块总体注释：根据FSM模式路由数据和合法检查
// 原理：纯组合逻辑，避免时序环路
// 目的：解析输入、检查合法性（维度1~5、元素0~9、运算匹配）、设置错误标志
// 使用信号：mode/op_type（路由）、uart_data_out（解析）、operand_legal（输出）
always @(*) begin
    operand_legal = 1'b1;               // 默认合法
    error_flag = 1'b0;
    
    case (mode)
        2'b00: begin                    // 输入模式：解析UART数据到m/n/元素
            // 示例解析：假设uart_data_out连续字节，先m后n后元素
            if (/*维度解析*/ m > MAX_DIM || n > MAX_DIM) begin
                error_flag = 1'b1;      // 非法维度设置错误
                operand_legal = 1'b0;
            end
            if (/*元素解析*/ uart_data_out > 9) error_flag = 1'b1;
        end
        2'b11: begin                    // 运算模式：检查运算数合法
            case (op_type)
                4'b0010: if (/*加法维度不匹配*/) operand_legal = 1'b0;  // 加法要求同维度
                4'b0100: if (/*乘法列行不匹配*/) operand_legal = 1'b0;  // An == Bm
                // 其他运算类似
            endcase
        end
    endcase
end

endmodule