module fsm (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [1:0] op_mode,         // 运算类型选择 (SW0, SW1)
    input  wire [1:0] func_sel,        // 功能选择 (SW6, SW7): 00=输入, 01=生成, 10=展示, 11=运算
    input  wire       uart_rx_done,
    input  wire [7:0] uart_rx_data,
    input  wire       btn_start,       // 确认/启动按钮 (S0)
    input  wire       btn_back,        // 返回主菜单按钮 (S2)
    
    output reg        store_wen,       
    output reg  [3:0] store_m,         
    output reg  [3:0] store_n,         
    output reg  [7:0] store_elem_in,   
    output reg        store_elem_valid,
    input  wire       storage_input_done, 
    
    output reg        add_start,
    input  wire       add_done,
    input  wire       add_busy,

    output reg        trans_start,
    input  wire       trans_done,
    input  wire       trans_busy,

    output reg        scalar_start,
    input  wire       scalar_done,
    input  wire       scalar_busy,

    output reg        matmul_start,
    input  wire       matmul_done,
    input  wire       matmul_busy,
    
    output reg [7:0]  scalar_value,
    output reg        current_slot,
    output reg [1:0]  led_status,
    output reg [1:0]  main_state_out   // 当前主状态输出 (用于LED指示)
);

    // 主状态定义 (顶层菜单)
    localparam MAIN_MENU     = 2'd0;  // 主菜单等待
    localparam MAIN_INPUT    = 2'd1;  // 矩阵输入模式 (func_sel=00)
    localparam MAIN_GENERATE = 2'd2;  // 矩阵生成模式 (func_sel=01)
    localparam MAIN_DISPLAY  = 2'd3;  // 矩阵展示模式 (func_sel=10) / 矩阵运算模式 (func_sel=11)

    // 子状态定义 (运算/输入子流程)
    localparam S_IDLE       = 4'd0;
    localparam S_GET_N      = 4'd1;
    localparam S_START_STORE= 4'd2; 
    localparam S_RX_DATA    = 4'd3;
    localparam S_WAIT_START = 4'd4;
    localparam S_CALC       = 4'd5;
    localparam S_DONE       = 4'd6;
    localparam S_GET_SCALAR = 4'd7;

    reg [1:0] main_state;   // 主状态寄存器
    reg [3:0] sub_state;    // 子状态寄存器
    reg [1:0] mat_count;
    reg [7:0] scalar_input;
    
    // 目标矩阵数量: 00(加法):2, 01(转置):1, 10(标量):1, 11(乘法):2
    wire [1:0] target_mat_count = (op_mode == 2'b01 || op_mode == 2'b10) ? 2'd1 : 2'd2;
    wire       need_scalar = (op_mode == 2'b10);

    // 主状态输出
    always @(*) begin
        main_state_out = main_state;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            main_state <= MAIN_MENU;
            sub_state <= S_IDLE;
            store_wen <= 0;
            store_elem_valid <= 0;
            add_start <= 0;
            trans_start <= 0;
            scalar_start <= 0;
            matmul_start <= 0;
            mat_count <= 0;
            current_slot <= 0;
            scalar_value <= 0;
            scalar_input <= 0;
            led_status <= 2'b00;
        end else begin
            // 脉冲信号复位
            store_wen <= 0;
            store_elem_valid <= 0;
            add_start <= 0;
            trans_start <= 0;
            scalar_start <= 0;
            matmul_start <= 0;

            // 返回按钮处理 - 任何状态按下返回都回到主菜单
            if (btn_back && main_state != MAIN_MENU) begin
                main_state <= MAIN_MENU;
                sub_state <= S_IDLE;
                mat_count <= 0;
                current_slot <= 0;
                led_status <= 2'b00;
            end else begin
                case (main_state)
                    MAIN_MENU: begin
                        led_status <= 2'b00;  // 主菜单状态灯
                        // 按下确认按钮进入对应功能
                        if (btn_start) begin
                            case (func_sel)
                                2'b00: main_state <= MAIN_INPUT;    // 矩阵输入
                                2'b01: main_state <= MAIN_GENERATE; // 矩阵生成
                                2'b10: main_state <= MAIN_DISPLAY;  // 矩阵展示
                                2'b11: main_state <= MAIN_DISPLAY;  // 矩阵运算 (复用DISPLAY状态)
                            endcase
                            sub_state <= S_IDLE;
                            mat_count <= 0;
                        end
                    end

                    MAIN_INPUT: begin
                        // 矩阵输入模式 - 通过UART接收矩阵数据
                        case (sub_state)
                            S_IDLE: begin
                                led_status <= 2'b01;  // 输入模式指示
                                if (uart_rx_done) begin
                                    store_m <= uart_rx_data[3:0];
                                    sub_state <= S_GET_N;
                                end
                            end

                            S_GET_N: begin 
                                if (uart_rx_done) begin
                                    store_n <= uart_rx_data[3:0];
                                    sub_state <= S_START_STORE;
                                end
                            end

                            S_START_STORE: begin 
                                store_wen <= 1;
                                current_slot <= mat_count[0]; 
                                sub_state <= S_RX_DATA;
                            end

                            S_RX_DATA: begin 
                                if (uart_rx_done) begin
                                    store_elem_in <= uart_rx_data;
                                    store_elem_valid <= 1;
                                end
                                
                                if (storage_input_done) begin
                                    mat_count <= mat_count + 1'b1;
                                    sub_state <= S_IDLE; // 继续等待下一个矩阵或返回
                                    led_status <= 2'b11; // 输入完成指示
                                end
                            end
                        endcase
                    end

                    MAIN_GENERATE: begin
                        // 矩阵生成模式 - 预留功能
                        led_status <= 2'b10;  // 生成模式指示
                        // TODO: 实现矩阵随机生成逻辑
                    end

                    MAIN_DISPLAY: begin
                        // 矩阵展示/运算模式
                        if (func_sel == 2'b10) begin
                            // 展示模式 - 预留功能
                            led_status <= 2'b01;
                            // TODO: 实现矩阵展示逻辑
                        end else begin
                            // 运算模式 (func_sel == 2'b11)
                            case (sub_state)
                                S_IDLE: begin
                                    if (uart_rx_done) begin
                                        store_m <= uart_rx_data[3:0];
                                        sub_state <= S_GET_N;
                                    end
                                end

                                S_GET_N: begin 
                                    if (uart_rx_done) begin
                                        store_n <= uart_rx_data[3:0];
                                        sub_state <= S_START_STORE;
                                    end
                                end

                                S_START_STORE: begin 
                                    store_wen <= 1;
                                    current_slot <= mat_count[0]; 
                                    sub_state <= S_RX_DATA;
                                end

                                S_RX_DATA: begin 
                                    if (uart_rx_done) begin
                                        store_elem_in <= uart_rx_data;
                                        store_elem_valid <= 1;
                                    end
                                    
                                    if (storage_input_done) begin
                                        if (mat_count + 1'b1 >= target_mat_count) begin
                                            mat_count <= mat_count + 1'b1;
                                            if (need_scalar)
                                                sub_state <= S_GET_SCALAR;
                                            else
                                                sub_state <= S_WAIT_START;
                                        end else begin
                                            mat_count <= mat_count + 1'b1;
                                            sub_state <= S_IDLE;
                                        end
                                    end
                                end

                                S_GET_SCALAR: begin
                                    if (uart_rx_done) begin
                                        scalar_input <= uart_rx_data;
                                        sub_state <= S_WAIT_START;
                                    end
                                end

                                S_WAIT_START: begin 
                                    led_status <= 2'b10;
                                    if (btn_start) begin
                                        case (op_mode)
                                            2'b00: add_start <= 1;
                                            2'b01: trans_start <= 1;
                                            2'b10: begin
                                                scalar_start <= 1;
                                                scalar_value <= scalar_input;
                                            end
                                            2'b11: matmul_start <= 1;
                                        endcase
                                        sub_state <= S_CALC;
                                    end
                                end

                                S_CALC: begin
                                    led_status <= 2'b11;
                                    if (add_done || trans_done || scalar_done || matmul_done) begin
                                        sub_state <= S_DONE;
                                    end
                                end
                                
                                S_DONE: begin
                                    led_status <= 2'b01;
                                    // 按返回键回主菜单，或等待复位
                                end
                            endcase
                        end
                    end
                endcase
            end
        end
    end
endmodule