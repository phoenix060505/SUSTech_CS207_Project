module fsm_full (
    input  wire        clk,
    input  wire        rst_n,
    
    // 控制输入
    input  wire [1:0]  op_mode,          // 运算类型: 00=加法, 01=转置, 10=标量乘, 11=矩阵乘
    input  wire [1:0]  func_sel,         // 功能选择: 00=输入, 01=生成, 10=展示, 11=运算
    input  wire        btn_start,        // 确认按钮
    input  wire        btn_back,         // 返回按钮
    input  wire [3:0]  scalar_input,     // 标量值输入 (0-15)
    input  wire        sw8_conv_mode,    // 卷积模式选择
    
    // UART 接口
    input  wire        uart_rx_done,
    input  wire [7:0]  uart_rx_data,
    
    // 存储控制接口 - 写入时只需提供维度，存储模块自动FIFO分配
    output reg         store_wen,
    output reg  [3:0]  store_m,
    output reg  [3:0]  store_n,
    output reg  [7:0]  store_elem_in,
    output reg         store_elem_valid,
    input  wire        storage_input_done,
    
    // 存储查询接口 - 查询指定维度下有多少矩阵
    output reg  [3:0]  query_m,
    output reg  [3:0]  query_n,
    input  wire [1:0]  query_count,       // 该维度下有效矩阵数量(0/1/2)
    input  wire        query_slot0_valid, // 槽0是否有效
    input  wire        query_slot1_valid, // 槽1是否有效
    
    // 存储读取接口 - 需要指定维度
    output reg         disp_rd_en,
    output reg         disp_rd_slot,
    output reg  [2:0]  disp_rd_row,
    output reg  [2:0]  disp_rd_col,
    output reg  [3:0]  disp_rd_m,         // 展示时的目标维度m
    output reg  [3:0]  disp_rd_n,         // 展示时的目标维度n
    input  wire [7:0]  disp_rd_elem,
    input  wire        disp_rd_valid,
    
    // UART 发送接口
    output reg         tx_data_valid,
    output reg  [7:0]  tx_data,
    input  wire        tx_busy,
    
    // 运算模块启动信号
    output reg         add_start,
    output reg         trans_start,
    output reg         scalar_start,
    output reg         matmul_start,
    input  wire        add_done,
    input  wire        trans_done,
    input  wire        scalar_done,
    input  wire        matmul_done,
    input  wire        add_busy,
    input  wire        trans_busy,
    input  wire        scalar_busy,
    input  wire        matmul_busy,
    
    // 卷积模块接口
    output reg         conv_start,
    output reg         conv_kernel_valid,
    output reg  [3:0]  conv_kernel_in,
    input  wire        conv_busy,
    input  wire        conv_done,

    // 运算参数输出
    output reg  [7:0]  scalar_value,
    output reg         sel_slot_a,       // 选中的第一个矩阵槽位(在当前维度下0或1)
    output reg         sel_slot_b,       // 选中的第二个矩阵槽位
    output reg  [2:0]  sel_m_a,          // 第一个矩阵的m
    output reg  [2:0]  sel_n_a,          // 第一个矩阵的n
    output reg  [2:0]  sel_m_b,          // 第二个矩阵的m
    output reg  [2:0]  sel_n_b,          // 第二个矩阵的n
    
    // 随机数接口
    output reg         rand_enable,
    input  wire [3:0]  rand_out,
    
    // 状态输出
    output reg  [1:0]  main_state_out,
    output reg  [1:0]  led_status,
    output reg         error_led,        // 错误指示LED
    output reg  [4:0]  countdown_val,    // 倒计时值 (0~31)
    output reg         countdown_active  // 倒计时激活
);

    // =========================================================
    // 主状态定义
    // =========================================================
    localparam MAIN_MENU     = 2'd0;
    localparam MAIN_INPUT    = 2'd1;
    localparam MAIN_GENERATE = 2'd2;
    localparam MAIN_DISPLAY  = 2'd3;
    
    // =========================================================
    // 子状态定义
    // =========================================================
    // 通用
    localparam S_IDLE          = 6'd0;
    
    // 输入模式子状态
    localparam S_IN_GET_M      = 6'd1;
    localparam S_IN_GET_N      = 6'd2;
    localparam S_IN_SET_DIM    = 6'd3;   // 先设置维度
    localparam S_IN_START_STORE= 6'd4;   // 再发送wen脉冲
    localparam S_IN_RX_DATA    = 6'd5;
    localparam S_IN_DONE       = 6'd6;
    localparam S_IN_FILL_ZEROS = 6'd7;   // 数据不足时自动补0
    
    // 生成模式子状态
    localparam S_GEN_GET_M     = 6'd8;
    localparam S_GEN_GET_N     = 6'd9;
    localparam S_GEN_GET_CNT   = 6'd10;
    localparam S_GEN_SET_DIM   = 6'd11;  // 先设置维度
    localparam S_GEN_START     = 6'd12;  // 再发送wen脉冲
    localparam S_GEN_FILL      = 6'd13;
    localparam S_GEN_DONE      = 6'd14;
    
    // 展示模式子状态
    localparam S_DISP_START    = 6'd15;
    localparam S_DISP_SEND_INFO= 6'd16;
    localparam S_DISP_SEND_MAT = 6'd17;
    localparam S_DISP_DONE     = 6'd18;
    
    // 运算模式子状态
    localparam S_OP_SHOW_INFO  = 6'd19;
    localparam S_OP_SEL_DIM_M  = 6'd20;
    localparam S_OP_SEL_DIM_N  = 6'd21;
    localparam S_OP_SHOW_MATS  = 6'd22;
    localparam S_OP_SEL_MAT    = 6'd23;
    localparam S_OP_GET_SCALAR = 6'd24;
    localparam S_OP_CHECK      = 6'd25;
    localparam S_OP_COUNTDOWN  = 6'd26;
    localparam S_OP_CALC       = 6'd27;
    localparam S_OP_OUTPUT     = 6'd28;
    localparam S_OP_DONE       = 6'd29;
        
    // 发送子状态
    localparam S_TX_WAIT       = 6'd30;

    // Bonus Convolution States
    localparam S_CALC_CONV_INPUT = 6'd31;
    localparam S_CALC_CONV_RUN   = 6'd32;
    localparam S_CALC_CONV_DONE  = 6'd33;

    localparam S_RAND_SEARCH_A = 6'd34; // 随机搜索 A
    localparam S_RAND_SEARCH_B = 6'd35; // 随机搜索 B
    localparam S_ECHO_PREP     = 6'd36; // 准备回显
    localparam S_ECHO_PRINT    = 6'd37; // 执行回显打印
    localparam S_ECHO_SCALAR   = 6'd38; // 回显标量值
    // =========================================================
    // 寄存器定义
    // =========================================================
    reg [1:0]  main_state;
    reg [5:0]  sub_state;
    reg [5:0]  next_sub_state;  // 发送完成后返回的状态
    
    // 输入/生成参数
    reg [3:0]  temp_m, temp_n;
    reg [1:0]  gen_mat_count;   // 要生成的矩阵数量
    reg [1:0]  current_mat_idx; // 当前处理的矩阵索引
    reg [4:0]  elem_count;      // 元素计数
    reg [4:0]  total_elems;     // 总元素数
    
    // 运算参数
    reg        selecting_second;  // 正在选择第二个运算数
    reg        op_sel_a_done;     // 已锁定第一个矩阵
    reg [3:0]  sel_dim_m, sel_dim_n;  // 用户选择的维度
    reg        op_dim_ready;          // 防止重复触发显示
    reg        op_listed_once;        // 当前维度已输出过一次
    
    // 展示/发送控制
    reg [2:0]  send_row, send_col;
    reg [1:0]  send_slot;         // 当前发送的槽号(在当前维度下)
    reg [7:0]  send_phase;        // 发送阶段
    reg [7:0]  tx_buffer;
    reg [3:0]  disp_m, disp_n;    // 当前展示的维度
    
    // 概览扫描用（遍历所有维度组合）
    reg [2:0]  scan_m, scan_n;    // 当前扫描的维度 (1~5)
    reg [5:0]  total_mat_count;   // 总矩阵数
    
    // 倒计时
    reg [26:0] countdown_timer; // 1秒计数器 (100MHz)
    reg [4:0]  countdown_sec;   // 倒计时秒数
    reg [4:0]  countdown_cfg;   // 配置的倒计时时间
    
    // 输入超时检测
    reg [31:0] input_timeout_timer; // 输入超时计数器 (100MHz)
    reg        input_timeout_active; // 输入超时检测激活标志

    // 两位数输入处理
    reg [3:0]  digit_buffer;        // 数字缓冲区 (0-9)
    reg        digit_buffer_valid;  // 缓冲区有效标志
    reg [7:0]  temp_elem;           // 临时存储计算后的元素值
    reg [7:0]  rand_retry_cnt;   // 随机重试计数器
    reg [3:0]  echo_m, echo_n;   // 待回显矩阵的维度
    reg        echo_slot;        // 待回显矩阵的槽位
    // check临时变量
    reg check_pass;
    // 主状态输出
    always @(*) begin
        main_state_out = main_state;
    end
    
    // =========================================================
    // 主状态机
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            main_state <= MAIN_MENU;
            sub_state <= S_IDLE;
            next_sub_state <= S_IDLE;
            
            store_wen <= 0;
            store_elem_valid <= 0;
            store_m <= 0;
            store_n <= 0;
            store_elem_in <= 0;
            
            query_m <= 0;
            query_n <= 0;
            
            disp_rd_en <= 0;
            disp_rd_slot <= 0;
            disp_rd_row <= 0;
            disp_rd_col <= 0;
            disp_rd_m <= 0;
            disp_rd_n <= 0;
            disp_m <= 0;
            disp_n <= 0;
            
            tx_data_valid <= 0;
            tx_data <= 0;
            
            add_start <= 0;
            trans_start <= 0;
            scalar_start <= 0;
            matmul_start <= 0;
            
            scalar_value <= 0;
            sel_slot_a <= 0;
            sel_slot_b <= 0;
            sel_m_a <= 0;
            sel_n_a <= 0;
            sel_m_b <= 0;
            sel_n_b <= 0;
            
            rand_enable <= 0;
            
            led_status <= 2'b00;
            error_led <= 0;
            countdown_val <= 0;
            countdown_active <= 0;
            countdown_cfg <= 5'd30;
            countdown_timer <= 0;
            countdown_sec <= 0;
            input_timeout_timer <= 0;
            input_timeout_active <= 0;
            
            temp_m <= 0;
            temp_n <= 0;
            gen_mat_count <= 0;
            current_mat_idx <= 0;
            elem_count <= 0;
            total_elems <= 0;
            selecting_second <= 0;
            op_sel_a_done <= 0;
            sel_dim_m <= 0;
            sel_dim_n <= 0;
            send_row <= 0;
            send_col <= 0;
            send_slot <= 0;
            send_phase <= 0;
            op_dim_ready <= 0;
            op_listed_once <= 0;
            scan_m <= 1;
            scan_n <= 1;
            total_mat_count <= 0;
            
            digit_buffer <= 0;
            digit_buffer_valid <= 0;
            rand_retry_cnt <= 0;
            echo_m <= 0; echo_n <= 0; echo_slot <= 0;
        end else begin
            // 脉冲信号复位
            store_wen <= 0;
            store_elem_valid <= 0;
            tx_data_valid <= 0;
            add_start <= 0;
            trans_start <= 0;
            scalar_start <= 0;
            matmul_start <= 0;
            rand_enable <= 0;
            disp_rd_en <= 0;
            
            // 返回按钮处理
            if (btn_back && main_state != MAIN_MENU) begin
                main_state <= MAIN_MENU;
                sub_state <= S_IDLE;
                error_led <= 0;
                countdown_active <= 0;
                selecting_second <= 0;
                op_sel_a_done <= 0;
                current_mat_idx <= 0;
                op_dim_ready <= 0;
                op_listed_once <= 0;
            end else begin
                
               case (main_state)
                    // =============================================
                    // 主菜单
                    // =============================================
                    MAIN_MENU: begin
                        led_status <= 2'b00;
                        error_led <= 0;
                        countdown_active <= 0;
                        
                        if (btn_start) begin
                            case (func_sel)
                                2'b00: begin
                                    main_state <= MAIN_INPUT;
                                    sub_state <= S_IN_GET_M;
                                end
                                2'b01: begin
                                    main_state <= MAIN_GENERATE;
                                    sub_state <= S_GEN_GET_M;
                                end
                                2'b10: begin
                                    main_state <= MAIN_DISPLAY;
                                    sub_state <= S_DISP_START;
                                    send_phase <= 100;
                                end
                                2'b11: begin
                                    main_state <= MAIN_DISPLAY;
                                    // 卷积模式特殊处理
                                    if (op_mode == 2'b11 && sw8_conv_mode) begin
                                        sub_state <= S_CALC_CONV_INPUT;
                                        send_phase <= 0;
                                        elem_count <= 0; // 卷积输入元素计数清零
                                        conv_start <= 1; // 启动卷积模块
                                    end else begin
                                        sub_state <= S_OP_SHOW_INFO;
                                        send_phase <= 0; 
                                    end
                                end
                            endcase
                            current_mat_idx <= 0;
                            selecting_second <= 0;
                        end
                    end
                    
                    // =============================================
                    // 矩阵输入模式
                    // =============================================
                    MAIN_INPUT: begin
                        led_status <= 2'b01;
                        
                        case (sub_state)
                            S_IN_GET_M: begin
                                error_led <= 0;
                                if (uart_rx_done) begin
                                    // 接受ASCII '1'-'5' (0x31-0x35) 或空格 (0x20)
                                    if (uart_rx_data >= 8'h31 && uart_rx_data <= 8'h35) begin
                                        temp_m <= uart_rx_data - 8'h30;
                                        sub_state <= S_IN_GET_N;
                                    end else if (uart_rx_data == 8'h20) begin
                                        // 忽略维度输入中的空格
                                    end else begin
                                        error_led <= 1;  // 违规操作，LED开始亮起
                                        input_timeout_timer <= 0;
                                        input_timeout_active <= 1;
                                    end
                                end else if (input_timeout_active) begin
                                    // 超时计数器递增
                                    if (input_timeout_timer < 26'd5000000) begin  // 50ms超时 (100MHz时钟)
                                        input_timeout_timer <= input_timeout_timer + 1;
                                    end else begin
                                        // 超时发生，检查是否有错误
                                        input_timeout_active <= 0;
                                        if (error_led) begin
                                            // 有错误，重新开始输入，LED保持亮起
                                            sub_state <= S_IN_GET_M;
                                        end
                                    end
                                end
                            end
                            
                            S_IN_GET_N: begin
                                if (uart_rx_done) begin
                                    // 接受ASCII '1'-'5' (0x31-0x35) 或空格 (0x20)
                                    if (uart_rx_data >= 8'h31 && uart_rx_data <= 8'h35) begin
                                        temp_n <= uart_rx_data - 8'h30;
                                        sub_state <= S_IN_SET_DIM;  // 先设置维度
                                        error_led <= 0;  // 输入正确，LED熄灭
                                    end else if (uart_rx_data == 8'h20) begin
                                        // 忽略维度输入中的空格
                                    end else begin
                                        error_led <= 1;  // 违规操作，LED亮起
                                        input_timeout_timer <= 0;
                                        input_timeout_active <= 1;
                                    end
                                end else if (input_timeout_active) begin
                                    // 超时计数器递增
                                    if (input_timeout_timer < 26'd5000000) begin  // 50ms超时 (100MHz时钟)
                                        input_timeout_timer <= input_timeout_timer + 1;
                                    end else begin
                                        // 超时发生，检查是否有错误
                                        input_timeout_active <= 0;
                                        if (error_led) begin
                                            // 有错误，重新开始输入，LED保持亮起
                                            sub_state <= S_IN_GET_M;
                                        end
                                    end
                                end
                            end
                            
                            //设置维度
                            S_IN_SET_DIM: begin
                                // 检查是否有维度错误
                                if (error_led) begin
                                    // 有错误，不设置维度，直接回到输入开始状态
                                    sub_state <= S_IN_GET_M;
                                end else begin
                                    // 无错误，设置维度并继续
                                    store_m <= temp_m;
                                    store_n <= temp_n;
                                    elem_count <= 0;
                                    total_elems <= temp_m * temp_n;
                                    sub_state <= S_IN_START_STORE;
                                end
                            end
                            
                            // 发送wen脉冲（此时m、n已经稳定）
                            S_IN_START_STORE: begin
                                // 再次检查是否有错误
                                if (error_led) begin
                                    // 有错误，不发送wen脉冲，直接回到输入开始状态
                                    sub_state <= S_IN_GET_M;
                                end else begin
                                    // 无错误，发送wen脉冲并继续
                                    store_wen <= 1;
                                    sub_state <= S_IN_RX_DATA;
                                end
                            end
                            
                            S_IN_RX_DATA: begin
                                // --------------------------------------------------------
                                // 1. 全局计时器逻辑：无数据接收时一直计数，有数据则清零
                                // --------------------------------------------------------
                                if (uart_rx_done) begin
                                    input_timeout_timer <= 0; // 收到数据，重置“发呆”计时器
                                end else begin
                                    // 防止溢出
                                    if (input_timeout_timer < 32'd300_000_000) 
                                        input_timeout_timer <= input_timeout_timer + 1;
                                end

                                // --------------------------------------------------------
                                // 2. 数据接收处理
                                // --------------------------------------------------------
                                if (uart_rx_done) begin
                                    // --- A. 数字字符处理 (0-9) ---
                                    if (uart_rx_data >= 8'h30 && uart_rx_data <= 8'h39) begin
                                        if (!digit_buffer_valid) begin
                                            // 存入缓冲区
                                            digit_buffer <= uart_rx_data - 8'h30;
                                            digit_buffer_valid <= 1;
                                        end else begin
                                            // 缓冲区已满，组合成两位数
                                            temp_elem <= (digit_buffer * 10) + (uart_rx_data - 8'h30);
                                            // 合法性检查与存储
                                            if (((digit_buffer * 10) + (uart_rx_data - 8'h30)) <= 9) begin 
                                                store_elem_in <= (digit_buffer * 10) + (uart_rx_data - 8'h30);
                                                store_elem_valid <= 1;
                                                elem_count <= elem_count + 1;
                                                error_led <= 0;
                                            end else begin
                                                error_led <= 1; // 越界报错
                                            end
                                            digit_buffer_valid <= 0; // 清空缓冲区
                                        end
                                    end
                                    // --- B. 分隔符处理 (空格/回车/换行) ---
                                    else if (uart_rx_data == 8'h20 || uart_rx_data == 8'h0D || uart_rx_data == 8'h0A) begin
                                        // 遇到分隔符，如果缓冲区有数字，先把它存进去
                                        if (digit_buffer_valid && elem_count < total_elems) begin
                                            store_elem_in <= digit_buffer;
                                            store_elem_valid <= 1;
                                            elem_count <= elem_count + 1;
                                            error_led <= 0;
                                            digit_buffer_valid <= 0;
                                        end
                                    end
                                    // --- C. 非法字符 ---
                                    else begin
                                        error_led <= 1;
                                    end
                                end

                                // --------------------------------------------------------
                                // 3. 超时处理逻辑 (双重模式)
                                // --------------------------------------------------------
                                
                                // 模式A：错误恢复超时 (0.5秒 = 50,000,000)
                                // 如果当前处于报错状态，0.5秒后自动熄灭LED，允许继续输入
                                if (error_led && input_timeout_timer > 32'd50_000_000) begin
                                    error_led <= 0;
                                    input_timeout_timer <= 0; // 重置计时，重新开始检测空闲
                                end

                                // 模式B：空闲补零超时 (2.0秒 = 200,000,000)
                                // 如果没有报错，且用户发呆超过2秒，认为输入结束
                                else if (!error_led && input_timeout_timer > 32'd200_000_000) begin
                                    input_timeout_timer <= 0;
                                    
                                    // 1. 如果缓冲区里还有个落单的数字，先存进去
                                    if (digit_buffer_valid && elem_count < total_elems) begin
                                        store_elem_in <= digit_buffer;
                                        store_elem_valid <= 1;
                                        elem_count <= elem_count + 1;
                                        digit_buffer_valid <= 0;
                                    end
                                    
                                    // 2. 状态跳转
                                    // 注意：这里利用下一时钟周期的状态判断，
                                    // 如果上面的 store_elem_valid 导致 elem_count 满了，
                                    // 下面的状态跳转会在下下个周期被 S_IN_DONE 捕获（或者在这里直接预判）
                                    
                                    // 为简化逻辑，直接跳到补零状态，由 S_IN_FILL_ZEROS 里的逻辑判断是否真的需要补
                                    sub_state <= S_IN_FILL_ZEROS;
                                end
                                
                                // --------------------------------------------------------
                                // 4. 完成检测
                                // --------------------------------------------------------
                                if (elem_count >= total_elems) begin
                                    sub_state <= S_IN_DONE;
                                end
                            end
                            
                            S_IN_FILL_ZEROS: begin
                                // 数据不足时自动补0
                                if (elem_count < total_elems) begin
                                    store_elem_in <= 8'h0;  // 发送0值
                                    store_elem_valid <= 1;
                                    elem_count <= elem_count + 1;
                                end else begin
                                    // 补0完成，等待存储模块完成
                                    if (storage_input_done) begin
                                        current_mat_idx <= current_mat_idx + 1;
                                        sub_state <= S_IN_DONE;
                                    end
                                end
                            end
                            
                            S_IN_DONE: begin
                                // 检查是否有错误发生
                                if (error_led) begin
                                    // 有错误，不存储矩阵，直接回到输入开始状态，LED保持亮起
                                    led_status <= 2'b00;
                                    sub_state <= S_IN_GET_M;
                                end else begin
                                    // 无错误，存储矩阵并继续
                                    led_status <= 2'b11;
                                    // 自动进入下一次输入，无需再次按键
                                    sub_state <= S_IN_GET_M;
                                end
                            end
                        endcase
                    end
                    
                    // =============================================
                    // 矩阵生成模式
                    // =============================================
                    MAIN_GENERATE: begin
                        led_status <= 2'b10;
                        
                        case (sub_state)
                            S_GEN_GET_M: begin
                                error_led <= 0;
                                if (uart_rx_done) begin
                                    // 接受ASCII '1'-'5' (0x31-0x35)
                                    if (uart_rx_data >= 8'h31 && uart_rx_data <= 8'h35) begin
                                        temp_m <= uart_rx_data - 8'h30;
                                        sub_state <= S_GEN_GET_N;
                                    end else begin
                                        error_led <= 1;
                                        input_timeout_timer <= 0;
                                        input_timeout_active <= 1;
                                    end
                                end else if (input_timeout_active) begin
                                    // 超时计数器递增
                                    if (input_timeout_timer < 26'd5000000) begin  // 50ms超时 (100MHz时钟)
                                        input_timeout_timer <= input_timeout_timer + 1;
                                    end else begin
                                        // 超时发生，检查是否有错误
                                        input_timeout_active <= 0;
                                        if (error_led) begin
                                            // 有错误，重新开始输入
                                            sub_state <= S_GEN_GET_M;
                                        end
                                    end
                                end
                            end
                            
                            S_GEN_GET_N: begin
                                if (uart_rx_done) begin
                                    // 接受ASCII '1'-'5' (0x31-0x35)
                                    if (uart_rx_data >= 8'h31 && uart_rx_data <= 8'h35) begin
                                        temp_n <= uart_rx_data - 8'h30;
                                        sub_state <= S_GEN_GET_CNT;
                                        error_led <= 0;
                                    end else begin
                                        error_led <= 1;
                                        input_timeout_timer <= 0;
                                        input_timeout_active <= 1;
                                    end
                                end else if (input_timeout_active) begin
                                    // 超时计数器递增
                                    if (input_timeout_timer < 26'd5000000) begin  // 50ms超时 (100MHz时钟)
                                        input_timeout_timer <= input_timeout_timer + 1;
                                    end else begin
                                        // 超时发生，检查是否有错误
                                        input_timeout_active <= 0;
                                        if (error_led) begin
                                            // 有错误，重新开始输入
                                            sub_state <= S_GEN_GET_M;
                                        end
                                    end
                                end
                            end
                            
                            S_GEN_GET_CNT: begin
                                if (uart_rx_done) begin
                                    // 接受ASCII '1'-'2' (0x31-0x32)
                                    if (uart_rx_data >= 8'h31 && uart_rx_data <= 8'h32) begin
                                        gen_mat_count <= uart_rx_data - 8'h30; // '1'->1, '2'->2
                                        current_mat_idx <= 0;
                                        sub_state <= S_GEN_SET_DIM;  // 先设置维度
                                        error_led <= 0;  // 输入正确，LED熄灭
                                    end else begin
                                        error_led <= 1;  // 违规操作，LED亮起
                                        input_timeout_timer <= 0;
                                        input_timeout_active <= 1;
                                    end
                                end else if (input_timeout_active) begin
                                    // 超时计数器递增
                                    if (input_timeout_timer < 26'd5000000) begin  // 50ms超时 (100MHz时钟)
                                        input_timeout_timer <= input_timeout_timer + 1;
                                    end else begin
                                        // 超时发生，检查是否有错误
                                        input_timeout_active <= 0;
                                        if (error_led) begin
                                            // 有错误，重新开始输入，LED保持亮起
                                            sub_state <= S_GEN_GET_M;
                                        end
                                    end
                                end
                            end
                            
                            // 先设置维度
                            S_GEN_SET_DIM: begin
                                // 检查是否有错误
                                if (error_led) begin
                                    // 有错误，不设置维度，直接回到开始状态
                                    sub_state <= S_GEN_GET_M;
                                end else begin
                                    // 无错误，设置维度并继续
                                    store_m <= temp_m;
                                    store_n <= temp_n;
                                    elem_count <= 0;
                                    total_elems <= temp_m * temp_n;
                                    sub_state <= S_GEN_START;
                                end
                            end
                            
                            // 发送wen脉冲（此时m、n已经稳定）
                            S_GEN_START: begin
                                // 再次检查是否有错误
                                if (error_led) begin
                                    // 有错误，不发送wen脉冲，直接回到开始状态
                                    sub_state <= S_GEN_GET_M;
                                end else begin
                                    // 无错误，发送wen脉冲并继续
                                    store_wen <= 1;
                                    sub_state <= S_GEN_FILL;
                                end
                            end
                            
                            S_GEN_FILL: begin
                                if (elem_count < total_elems) begin
                                    rand_enable <= 1;
                                    store_elem_in <= {4'b0, rand_out[3:0]};  // 直接使用低4位作为0~9
                                    store_elem_valid <= 1;
                                    elem_count <= elem_count + 1;
                                end else if (storage_input_done) begin
                                    current_mat_idx <= current_mat_idx + 1;
                                    if (current_mat_idx + 1 < gen_mat_count) begin
                                        sub_state <= S_GEN_SET_DIM;  // 重新设置维度
                                    end else begin
                                        sub_state <= S_GEN_DONE;
                                    end
                                end
                            end
                            
                            S_GEN_DONE: begin
                                led_status <= 2'b11; // 完成状态指示
                                
                                // 按下确认键，重新开始生成流程
                                if (btn_start) begin
                                    error_led <= 0;
                                    input_timeout_timer <= 0;
                                    input_timeout_active <= 0;
                                    led_status <= 2'b10; // 切回生成模式状态灯
                                    sub_state <= S_GEN_GET_M; // 跳回获取维度 m 的状态
                                end
                            end
                        endcase
                    end
                    
                    // =============================================
                    // 展示/运算模式
                    // =============================================
                    MAIN_DISPLAY: begin
                        if (countdown_active) begin
                            // 实时同步数值到输出端口
                            countdown_val <= countdown_sec; 

                            // 1. 计数器递减
                            if (countdown_timer >= 100_000_000 - 1) begin
                                countdown_timer <= 0;
                                if (countdown_sec > 0) begin
                                    countdown_sec <= countdown_sec - 1;
                                end else begin
                                    // === 超时处理 ===
                                    sub_state <= S_OP_SHOW_INFO; 
                                    send_phase <= 0;

                                    // 重置所有选择标志
                                    op_sel_a_done <= 0;
                                    selecting_second <= 0;
                                    op_dim_ready <= 0;
                                    op_listed_once <= 0;
                                    
                                    countdown_active <= 0; 
                                    error_led <= 0;
                                    
                                    // 重置时间以便下次使用
                                    countdown_sec <= countdown_cfg;
                                    countdown_val <= countdown_cfg; // 同步更新显示值
                                end
                            end else begin
                                countdown_timer <= countdown_timer + 1;
                            end
                        end
                        if (func_sel == 2'b10) begin
                            // ========== 展示模式 ==========
                            led_status <= 2'b01;
                            case (sub_state)
                                S_DISP_START: begin
                                    // 进入展示模式时先提示输入 m
                                    disp_m <= 0;
                                    disp_n <= 0;                                    
                                    if (!tx_busy) begin
                                        case (send_phase)
                                            100: begin tx_data <= "I"; tx_data_valid <= 1; send_phase <= 101; end
                                            101: begin tx_data <= "N"; tx_data_valid <= 1; send_phase <= 102; end
                                            102: begin tx_data <= "P"; tx_data_valid <= 1; send_phase <= 103; end
                                            103: begin tx_data <= "U"; tx_data_valid <= 1; send_phase <= 104; end
                                            104: begin tx_data <= "T"; tx_data_valid <= 1; send_phase <= 105; end
                                            105: begin tx_data <= 8'h20; tx_data_valid <= 1; send_phase <= 106; end
                                            106: begin tx_data <= "M"; tx_data_valid <= 1; send_phase <= 107; end
                                            107: begin tx_data <= 8'h20; tx_data_valid <= 1; send_phase <= 108; end
                                            108: begin tx_data <= "("; tx_data_valid <= 1; send_phase <= 109; end
                                            109: begin tx_data <= "1"; tx_data_valid <= 1; send_phase <= 110; end
                                            110: begin tx_data <= "-"; tx_data_valid <= 1; send_phase <= 111; end
                                            111: begin tx_data <= "5"; tx_data_valid <= 1; send_phase <= 112; end
                                            112: begin tx_data <= ")"; tx_data_valid <= 1; send_phase <= 113; end
                                            113: begin tx_data <= 8'h0D; tx_data_valid <= 1; send_phase <= 114; end
                                            114: begin tx_data <= 8'h0A; tx_data_valid <= 1; send_phase <= 0; sub_state <= S_DISP_SEND_INFO; end
                                            default: send_phase <= 100;
                                        endcase
                                    end
                                end
                                
                                S_DISP_SEND_INFO: begin
                                    // 等待用户通过UART输入维度 m
                                    if (uart_rx_done) begin
                                    // ASCII '1'~'5'
                                        if (uart_rx_data >= 8'h31 && uart_rx_data <= 8'h35) begin
                                            disp_m <= uart_rx_data - 8'h30;
                                            sub_state <= S_DISP_SEND_MAT;
                                            send_phase <= 200; // 200~214: 提示输入 n
                                        end
                                    end
                                end
                                
                                S_DISP_SEND_MAT: begin
                                    // 等待用户输入维度 n，然后查询并显示
                                    case (send_phase)
                                        // 提示输入 n
                                        200: if (!tx_busy) begin tx_data <= "I"; tx_data_valid <= 1; send_phase <= 201; end
                                        201: if (!tx_busy) begin tx_data <= "N"; tx_data_valid <= 1; send_phase <= 202; end
                                        202: if (!tx_busy) begin tx_data <= "P"; tx_data_valid <= 1; send_phase <= 203; end
                                        203: if (!tx_busy) begin tx_data <= "U"; tx_data_valid <= 1; send_phase <= 204; end
                                        204: if (!tx_busy) begin tx_data <= "T"; tx_data_valid <= 1; send_phase <= 205; end
                                        205: if (!tx_busy) begin tx_data <= 8'h20; tx_data_valid <= 1; send_phase <= 206; end
                                        206: if (!tx_busy) begin tx_data <= "N"; tx_data_valid <= 1; send_phase <= 207; end
                                        207: if (!tx_busy) begin tx_data <= 8'h20; tx_data_valid <= 1; send_phase <= 208; end
                                        208: if (!tx_busy) begin tx_data <= "("; tx_data_valid <= 1; send_phase <= 209; end
                                        209: if (!tx_busy) begin tx_data <= "1"; tx_data_valid <= 1; send_phase <= 210; end
                                        210: if (!tx_busy) begin tx_data <= "-"; tx_data_valid <= 1; send_phase <= 211; end
                                        211: if (!tx_busy) begin tx_data <= "5"; tx_data_valid <= 1; send_phase <= 212; end
                                        212: if (!tx_busy) begin tx_data <= ")"; tx_data_valid <= 1; send_phase <= 213; end
                                        213: if (!tx_busy) begin tx_data <= 8'h0D; tx_data_valid <= 1; send_phase <= 214; end
                                        214: if (!tx_busy) begin tx_data <= 8'h0A; tx_data_valid <= 1; send_phase <= 0; end

                                        // 等待 n 输入
                                        0: begin
                                            if (uart_rx_done) begin
                                                if (uart_rx_data >= 8'h31 && uart_rx_data <= 8'h35) begin
                                                    disp_n <= uart_rx_data - 8'h30;
                                                    // 设置查询维度
                                                    query_m <= disp_m;
                                                    query_n <= uart_rx_data - 8'h30;
                                                    send_phase <= 1;
                                                end
                                            end
                                        end
                                        
                                        1: begin
                                            // 查询结果已就绪，开始发送
                                            disp_rd_m <= disp_m;
                                            disp_rd_n <= disp_n;
                                            send_slot <= 0;
                                            send_row <= 0;
                                            send_col <= 0;
                                            op_listed_once <= 0;  // 复用此标志：跟踪是否已显示过矩阵
                                            send_phase <= 2;
                                        end
                                        
                                        2: begin  // 检查当前槽是否有效
                                            if (!tx_busy) begin
                                                // 使用查询结果判断槽有效性
                                                if (send_slot == 0 && query_slot0_valid) begin
                                                    disp_rd_en <= 1;
                                                    disp_rd_slot <= 0;
                                                    disp_rd_row <= send_row;
                                                    disp_rd_col <= send_col;
                                                    op_listed_once <= 1;  // 标记已显示过矩阵
                                                    send_phase <= 3; // 两拍握手：3 拉低，4 等待 valid
                                                end else if (send_slot == 1 && query_slot1_valid) begin
                                                    disp_rd_en <= 1;
                                                    disp_rd_slot <= 1;
                                                    disp_rd_row <= send_row;
                                                    disp_rd_col <= send_col;
                                                    op_listed_once <= 1;  // 标记已显示过矩阵
                                                    send_phase <= 3;
                                                end else if (send_slot < 2) begin
                                                    send_slot <= send_slot + 1;
                                                end else begin
                                                    // 遍历完毕，检查是否显示过矩阵
                                                    if (op_listed_once) begin
                                                        // 已显示过矩阵，直接结束
                                                        sub_state <= S_DISP_DONE;
                                                    end else begin
                                                        // 两个槽都无效，输出 EMPTY
                                                        send_phase <= 50; // 50~57 输出 "EMPTY\r\n"
                                                    end
                                                end
                                            end
                                        end
                                        // 两拍握手
                                        3: begin
                                            disp_rd_en <= 0; // 第二拍拉低
                                            send_phase <= 4;
                                        end
                                        4: begin  // 等待数据脉冲
                                            if (disp_rd_valid) begin
                                                // 确保元素值在0-9范围内，避免乱码
                                                if (disp_rd_elem <= 9) begin
                                                    tx_data <= disp_rd_elem + 8'h30;  // 转换为ASCII数字
                                                end else begin
                                                    tx_data <= "?";  // 超出范围显示问号
                                                end
                                                tx_data_valid <= 1;
                                                send_phase <= 5;
                                            end
                                        end
                                        
                                        5: begin  // 发送空格或换行
                                            if (!tx_busy) begin
                                                if (send_col + 1 < disp_n) begin
                                                    tx_data <= 8'h20;  // 空格
                                                    tx_data_valid <= 1;
                                                    send_col <= send_col + 1;
                                                    send_phase <= 2;
                                                end else begin
                                                    tx_data <= 8'h0D;  // CR
                                                    tx_data_valid <= 1;
                                                    send_phase <= 6;
                                                end
                                            end
                                        end
                                        
                                        6: begin  // 发送LF
                                            if (!tx_busy) begin
                                                tx_data <= 8'h0A;  // LF
                                                tx_data_valid <= 1;
                                                send_col <= 0;
                                                if (send_row + 1 < disp_m) begin
                                                    send_row <= send_row + 1;
                                                    send_phase <= 2;
                                                end else begin
                                                    // 当前矩阵完成，准备下一个
                                                    send_row <= 0;
                                                    send_slot <= send_slot + 1;
                                                    // 检查是否还有更多矩阵需要显示
                                                    if (send_slot + 1 < 2) begin
                                                        send_phase <= 7;  // 发送空行后检查下一个槽
                                                    end else begin
                                                        // 所有槽已处理完毕，直接结束
                                                        sub_state <= S_DISP_DONE;
                                                    end
                                                end
                                            end
                                        end
                                        
                                        7: begin  // 矩阵间空行
                                            if (!tx_busy) begin
                                                tx_data <= 8'h0D;
                                                tx_data_valid <= 1;
                                                send_phase <= 8;
                                            end
                                        end
                                        
                                        8: begin
                                            if (!tx_busy) begin
                                                tx_data <= 8'h0A;
                                                tx_data_valid <= 1;
                                                // 重置行列，准备读取下一个矩阵
                                                send_row <= 0;
                                                send_col <= 0;
                                                send_phase <= 2;
                                            end
                                        end
                                        
                                        // EMPTY 输出
                                        50: if (!tx_busy) begin tx_data <= "E"; tx_data_valid <= 1; send_phase <= 51; end
                                        51: if (!tx_busy) begin tx_data <= "M"; tx_data_valid <= 1; send_phase <= 52; end
                                        52: if (!tx_busy) begin tx_data <= "P"; tx_data_valid <= 1; send_phase <= 53; end
                                        53: if (!tx_busy) begin tx_data <= "T"; tx_data_valid <= 1; send_phase <= 54; end
                                        54: if (!tx_busy) begin tx_data <= "Y"; tx_data_valid <= 1; send_phase <= 55; end
                                        55: if (!tx_busy) begin tx_data <= 8'h0D; tx_data_valid <= 1; send_phase <= 56; end
                                        56: if (!tx_busy) begin tx_data <= 8'h0A; tx_data_valid <= 1; send_phase <= 57; end
                                        57: begin sub_state <= S_DISP_DONE; end
                                    endcase
                                end
                                
                                S_DISP_DONE: begin
                                    led_status <= 2'b11; // 完成状态指示
                                    
                                    if (btn_start) begin
                                        led_status <= 2'b01; // 切回展示模式状态灯
                                        send_phase <= 100;   // 重置发送阶段为初始值(打印提示语)
                                        sub_state <= S_DISP_START; // 跳回展示开始状态
                                    end
                                end
                            endcase
                            
                        end else begin
                            // ========== 运算模式 (func_sel == 2'b11) ==========
                            led_status <= 2'b10;
                            
                            case (sub_state)
                                S_OP_SHOW_INFO: begin
                                    // 显示存储矩阵概览: "总数 m*n*x m*n*x ..."
                                    // 需要遍历所有维度组合统计
                                    if (!tx_busy) begin
                                        case (send_phase)
                                            // === 阶段 0-2: 统计总数 ===
                                            0: begin
                                                // 初始化扫描
                                                scan_m <= 1;
                                                scan_n <= 1;
                                                total_mat_count <= 0;
                                                send_phase <= 1;
                                            end
                                            1: begin
                                                // 设置查询维度
                                                query_m <= {1'b0, scan_m};
                                                query_n <= {1'b0, scan_n};
                                                send_phase <= 2;
                                            end
                                            2: begin
                                                // 累加当前维度的矩阵数
                                                total_mat_count <= total_mat_count + query_count;
                                                // 移动到下一个维度
                                                if (scan_n < 5) begin
                                                    scan_n <= scan_n + 1;
                                                    send_phase <= 1;
                                                end else if (scan_m < 5) begin
                                                    scan_m <= scan_m + 1;
                                                    scan_n <= 1;
                                                    send_phase <= 1;
                                                end else begin
                                                    // 扫描完成，开始发送
                                                    send_phase <= 3;
                                                end
                                            end
                                            
                                            // === 阶段 3-4: 发送总数 ===
                                            3: begin
                                                // 发送总数（简化：只发个位数，最多50）
                                                if (total_mat_count >= 10) begin
                                                    tx_data <= 8'h30 + (total_mat_count / 10);
                                                    tx_data_valid <= 1;
                                                    send_phase <= 31;
                                                end else begin
                                                    tx_data <= 8'h30 + total_mat_count;
                                                    tx_data_valid <= 1;
                                                    send_phase <= 4;
                                                end
                                            end
                                            31: begin  // 发送个位
                                                tx_data <= 8'h30 + (total_mat_count % 10);
                                                tx_data_valid <= 1;
                                                send_phase <= 4;
                                            end
                                            4: begin  // 空格
                                                tx_data <= 8'h20;
                                                tx_data_valid <= 1;
                                                // 重新开始扫描以发送详细信息
                                                scan_m <= 1;
                                                scan_n <= 1;
                                                send_phase <= 5;
                                            end
                                            
                                            // === 阶段 5-12: 发送每种维度的 m*n*x ===
                                            5: begin
                                                // 设置查询维度
                                                query_m <= {1'b0, scan_m};
                                                query_n <= {1'b0, scan_n};
                                                send_phase <= 6;
                                            end
                                            6: begin
                                                // 检查该维度是否有矩阵
                                                if (query_count > 0) begin
                                                    // 发送 m
                                                    tx_data <= 8'h30 + scan_m;
                                                    tx_data_valid <= 1;
                                                    send_phase <= 7;
                                                end else begin
                                                    // 跳过这个维度
                                                    send_phase <= 12;
                                                end
                                            end
                                            7: begin  // *
                                                tx_data <= 8'h2A;
                                                tx_data_valid <= 1;
                                                send_phase <= 8;
                                            end
                                            8: begin  // n
                                                tx_data <= 8'h30 + scan_n;
                                                tx_data_valid <= 1;
                                                send_phase <= 9;
                                            end
                                            9: begin  // *
                                                tx_data <= 8'h2A;
                                                tx_data_valid <= 1;
                                                send_phase <= 10;
                                            end
                                            10: begin  // x (数量)
                                                tx_data <= 8'h30 + query_count;
                                                tx_data_valid <= 1;
                                                send_phase <= 11;
                                            end
                                            11: begin  // 空格
                                                tx_data <= 8'h20;
                                                tx_data_valid <= 1;
                                                send_phase <= 12;
                                            end
                                            12: begin
                                                // 移动到下一个维度
                                                if (scan_n < 5) begin
                                                    scan_n <= scan_n + 1;
                                                    send_phase <= 5;
                                                end else if (scan_m < 5) begin
                                                    scan_m <= scan_m + 1;
                                                    scan_n <= 1;
                                                    send_phase <= 5;
                                                end else begin
                                                    // 发送完成，换行
                                                    send_phase <= 13;
                                                end
                                            end
                                            
                                            // === 阶段 13-14: 换行并进入维度选择 ===
                                            13: begin
                                                tx_data <= 8'h0D;  // CR
                                                tx_data_valid <= 1;
                                                send_phase <= 14;
                                            end
                                            14: begin
                                                tx_data <= 8'h0A;  // LF
                                                tx_data_valid <= 1;
                                                send_phase <= 0;
                                                sub_state <= S_OP_SEL_DIM_M;
                                                selecting_second <= 0;
                                            end
                                        endcase
                                    end
                                end
                                
                                S_OP_SEL_DIM_M: begin
                                    // 保持 Error LED 状态（如果之前报错了，进入这里后等待新输入或超时清除）
                                    if (uart_rx_done) begin
                                        // 检测 'R' 键触发随机模式
                                        if (uart_rx_data == "R" || uart_rx_data == "r") begin
                                            rand_retry_cnt <= 0;          // 搜索次数计数器清零
                                            rand_enable <= 1;             // 开启随机数
                                            op_sel_a_done <= 0;           // 重置 A 选择标志
                                            selecting_second <= 0;        // 重置 B 选择标志
                                            send_phase <= 0;              // 相位清零
                                            error_led <= 0;               // 先熄灭错误灯
                                            sub_state <= S_RAND_SEARCH_A; // 开始搜寻 A
                                        end
                                        // 正常数字输入处理...
                                        else if (uart_rx_data >= 8'h31 && uart_rx_data <= 8'h35) begin
                                            sel_dim_m <= uart_rx_data - 8'h30;
                                            op_dim_ready <= 0; 
                                            op_listed_once <= 0;
                                            // 如果是选 A，重置标志；如果是乘法回头选 B，保持标志
                                            if (!op_sel_a_done) begin
                                                selecting_second <= 0;
                                                op_sel_a_done <= 0; 
                                            end
                                            error_led <= 0; // 输入正确，灭灯
                                            sub_state <= S_OP_SEL_DIM_N;
                                        end else begin
                                            error_led <= 1; // 输入非法字符
                                        end
                                    end
                                end
                                
                                S_OP_SEL_DIM_N: begin
                                    if (uart_rx_done && !op_dim_ready) begin
                                        // 接受ASCII '1'-'5' (0x31-0x35)
                                        if (uart_rx_data >= 8'h31 && uart_rx_data <= 8'h35) begin
                                            sel_dim_n <= uart_rx_data - 8'h30;
                                            // 设置查询维度
                                            query_m <= sel_dim_m;
                                            query_n <= uart_rx_data - 8'h30;
                                            error_led <= 0;
                                            op_dim_ready <= 1; // 已锁定本次维度，忽略后续 CR/LF
                                            // 行列都收到后立即列出匹配矩阵，无需再次按键
                                            if (!op_listed_once) begin
                                                sub_state <= S_OP_SHOW_MATS;
                                                send_phase <= 0;
                                            end else begin
                                                error_led <= 1;
                                            end
                                        end else begin
                                            error_led <= 1;
                                        end
                                    end
                                end
                                
                                S_OP_SHOW_MATS: begin
                                    op_listed_once <= 1; // 只显示一次，除非重新输入维度
                                    // 显示该维度下的所有矩阵及编号
                                    // 使用查询接口: query_slot0_valid, query_slot1_valid
                                    
                                    // 读取相关的状态不需要等待 tx_busy，可以独立运行
                                    case (send_phase)
                                        35: begin
                                            disp_rd_m <= sel_dim_m;
                                            disp_rd_n <= sel_dim_n;
                                            disp_rd_slot <= send_slot[0];
                                            disp_rd_row <= send_row;
                                            disp_rd_col <= send_col;
                                            send_phase <= 4;
                                        end

                                        // === 读取数据 ===
                                        4: begin  
                                            disp_rd_en <= 1;
                                            send_phase <= 45;
                                        end

                                        // === 停止读请求 ===
                                        45: begin
                                            send_phase <= 46; 
                                        end

                                        // === 捕获数据并准备发送 ===
                                        46: begin
                                            if (disp_rd_valid) begin
                                                tx_buffer <= disp_rd_elem + 8'h30;
                                                send_phase <= 47; // 等待 tx 空闲再发送
                                            end else begin
                                                tx_buffer <= 8'h3F; // '?'
                                                send_phase <= 47;
                                            end
                                        end
                                        
                                        // === 等待 tx_busy 后发送捕获的数据 ===
                                        47: begin
                                            if (!tx_busy) begin
                                                tx_data <= tx_buffer;
                                                tx_data_valid <= 1;
                                                send_phase <= 6;
                                            end
                                        end
                                        
                                        default: begin
                                            // 其他需要 tx 的状态
                                            if (!tx_busy) begin
                                                case (send_phase)
                                                    // === 初始化阶段 ===
                                                    0: begin
                                                        send_slot <= 0;
                                                        disp_rd_m <= sel_dim_m;
                                                        disp_rd_n <= sel_dim_n;
                                                        disp_rd_row <= 0;
                                                        disp_rd_col <= 0;
                                                        send_phase <= 1;
                                                    end
                                                    
                                                    // === 检查槽位有效性并显示ID ===
                                                    1: begin
                                                        if (send_slot < 2) begin
                                                            if ((send_slot == 0 && query_slot0_valid) ||
                                                                (send_slot == 1 && query_slot1_valid)) begin
                                                                tx_data <= 8'h30 + send_slot + 1;
                                                                tx_data_valid <= 1;
                                                                send_phase <= 2;
                                                            end else begin
                                                                send_slot <= send_slot + 1;
                                                            end
                                                        end else begin
                                                            tx_data <= 8'h0D;
                                                            tx_data_valid <= 1;
                                                            send_phase <= 20;
                                                        end
                                                    end
                                                    
                                                    2: begin
                                                        tx_data <= 8'h0D;
                                                        tx_data_valid <= 1;
                                                        send_phase <= 3;
                                                    end
                                                    
                                                    3: begin
                                                        tx_data <= 8'h0A;
                                                        tx_data_valid <= 1;
                                                        send_row <= 0;
                                                        send_col <= 0;
                                                        send_phase <= 35;
                                                    end
                                                    
                                                    // === 发送后的处理：空格或换行 ===
                                                    6: begin
                                                        if (send_col + 1 < sel_dim_n) begin
                                                            tx_data <= 8'h20;
                                                            tx_data_valid <= 1;
                                                            send_col <= send_col + 1;
                                                            send_phase <= 35;
                                                        end else begin
                                                            tx_data <= 8'h0D;
                                                            tx_data_valid <= 1;
                                                            send_phase <= 7;
                                                        end
                                                    end
                                                    
                                                    7: begin
                                                        tx_data <= 8'h0A;
                                                        tx_data_valid <= 1;
                                                        send_col <= 0;
                                                        if (send_row + 1 < sel_dim_m) begin
                                                            send_row <= send_row + 1;
                                                            send_phase <= 35;
                                                        end else begin
                                                            send_row <= 0;
                                                            send_slot <= send_slot + 1;
                                                            send_phase <= 60;
                                                        end
                                                    end

                                                    // === 矩阵间空行处理 ===
                                                    60: begin
                                                        tx_data <= 8'h0D;
                                                        tx_data_valid <= 1;
                                                        send_phase <= 61;
                                                    end
                                                    61: begin
                                                        tx_data <= 8'h0A;
                                                        tx_data_valid <= 1;
                                                        send_phase <= 1;
                                                    end
                                                    
                                                    // === 结束 ===
                                                    20: begin
                                                        tx_data <= 8'h0A;
                                                        tx_data_valid <= 1;
                                                        send_phase <= 0;
                                                        sub_state <= S_OP_SEL_MAT;
                                                    end
                                                    
                                                    default: send_phase <= 0;
                                                endcase
                                            end
                                        end
                                    endcase
                                end
                                

                                S_OP_SEL_MAT: begin
                                    if (uart_rx_done) begin
                                        if (uart_rx_data >= 8'h31 && uart_rx_data <= 8'h32) begin
                                            if (!op_sel_a_done) begin
                                                // === 选中 A ===
                                                sel_slot_a <= uart_rx_data[0] - 1;
                                                sel_m_a   <= sel_dim_m[2:0];
                                                sel_n_a   <= sel_dim_n[2:0];
                                                op_sel_a_done <= 1;
                                                error_led <= 0;
                                                
                                                // 配置回显参数
                                                echo_m <= sel_dim_m;
                                                echo_n <= sel_dim_n;
                                                echo_slot <= uart_rx_data[0] - 1;
                                                
                                                if (op_mode == 2'b01) next_sub_state <= S_OP_CHECK;      // 转置 -> 检查
                                                else if (op_mode == 2'b10) next_sub_state <= S_OP_GET_SCALAR; // 标量 -> 输标量
                                                else if (op_mode == 2'b11) next_sub_state <= S_OP_SEL_DIM_M;  // 乘法 -> 选B维度
                                                
                                                // 加法模式(00)，直接去选B (S_OP_SEL_MAT)，不要再展示冗余列表
                                                else next_sub_state <= S_OP_SEL_MAT; 

                                                if (op_mode == 2'b00 || op_mode == 2'b11) selecting_second <= 1;
                                                
                                                send_phase <= 0;
                                                sub_state <= S_ECHO_PREP; // 【跳转】去回显

                                            end else if (selecting_second) begin
                                                // === 选中 B ===
                                                sel_slot_b <= uart_rx_data[0] - 1;
                                                sel_m_b   <= sel_dim_m[2:0]; 
                                                sel_n_b   <= sel_dim_n[2:0];
                                                selecting_second <= 0;
                                                error_led <= 0;
                                                
                                                // 配置回显 B
                                                echo_m <= sel_dim_m;
                                                echo_n <= sel_dim_n;
                                                echo_slot <= uart_rx_data[0] - 1;
                                                
                                                next_sub_state <= S_OP_CHECK; // 回显完去检查/计算
                                                send_phase <= 0;
                                                sub_state <= S_ECHO_PREP; // 【跳转】去回显
                                            end
                                        end else begin
                                            error_led <= 1;
                                        end
                                    end
                                end
                                // ========================================================
                                // 随机搜索 A 逻辑 (修复：随机起点+线性扫描，确保必中)
                                // ========================================================
                                S_RAND_SEARCH_A: begin
                                    rand_enable <= 1; // 保持随机数生成
                                    
                                    // 全局超时保护 (防止死循环)
                                    if (rand_retry_cnt > 200) begin
                                        error_led <= 1;
                                        send_phase <= 0;
                                        sub_state <= S_OP_SHOW_INFO;
                                    end else begin
                                        
                                        case (send_phase)
                                            0: begin // === 步骤1: 确定随机扫描起点 ===
                                                // rand_out 是 4位 (0-15)，取模5得到 0-4，+1得到 1-5
                                                temp_m <= (rand_out % 5) + 1;
                                                send_phase <= 1;
                                            end

                                            1: begin // 错开一拍再取 n，增加随机性
                                                temp_n <= (rand_out % 5) + 1;
                                                // elem_count 用作“本次扫描过的组合数”，防止无限绕圈
                                                elem_count <= 0; 
                                                send_phase <= 2;
                                            end
                                            
                                            2: begin // === 步骤2: 查询当前维度 ===
                                                query_m <= temp_m;
                                                query_n <= temp_n; 
                                                send_phase <= 3;
                                            end
                                            
                                            3: begin // === 步骤3: 检查结果 ===
                                                if (query_count > 0) begin
                                                    // >>> 找到了存在的矩阵 A <<<
                                                    sel_m_a <= temp_m[2:0];
                                                    sel_n_a <= temp_n[2:0];
                                                    
                                                    // 随机选槽位
                                                    if (query_count == 1) sel_slot_a <= query_slot0_valid ? 0 : 1;
                                                    else sel_slot_a <= rand_out[0];
                                                    
                                                    // 准备 A 的回显信息
                                                    echo_m <= temp_m;
                                                    echo_n <= temp_n;
                                                    echo_slot <= (query_count == 1) ? (query_slot0_valid ? 0 : 1) : rand_out[0];

                                                    // >>> 决定打印完 A 之后去哪 <<<
                                                    if (op_mode == 2'b01) begin // [转置]
                                                        op_sel_a_done <= 1;
                                                        next_sub_state <= S_OP_CHECK; // 打印完A -> 去检查
                                                    end
                                                    else if (op_mode == 2'b10) begin // [标量乘]
                                                        op_sel_a_done <= 1;
                                                        scalar_value <= {4'b0, rand_out}; // 随机标量
                                                        next_sub_state <= S_ECHO_SCALAR; // 【关键】打印完A -> 去打印标量
                                                    end
                                                    else begin // [加法/乘法]
                                                        // 还没结束，不要置 done
                                                        next_sub_state <= S_RAND_SEARCH_B; // 【关键】打印完A -> 去搜B
                                                    end
                                                    
                                                    // >>> 立即出发去打印 A <<<
                                                    send_phase <= 0;
                                                    sub_state <= S_ECHO_PREP;

                                                end else begin
                                                    // >>> 没找到，扫描下一个维度 <<<
                                                    // 逻辑：N+1，如果溢出则M+1，遍历所有 5x5=25 种组合
                                                    if (temp_n < 5) temp_n <= temp_n + 1;
                                                    else begin
                                                        temp_n <= 1;
                                                        if (temp_m < 5) temp_m <= temp_m + 1;
                                                        else temp_m <= 1;
                                                    end
                                                    
                                                    // 计数保护
                                                    elem_count <= elem_count + 1;
                                                    if (elem_count > 26) begin
                                                        // 扫描了一整圈还没找到？说明存储器是空的，或者运气极差
                                                        // 增加全局重试计数，重新随机个起点再试
                                                        rand_retry_cnt <= rand_retry_cnt + 20; 
                                                        send_phase <= 0; 
                                                    end else begin
                                                        send_phase <= 2; // 继续查下一个
                                                    end
                                                end
                                            end
                                        endcase
                                    end
                                end

                                // ========================================================
                                // 随机搜索 B 逻辑 (依赖 A，同样支持扫描)
                                // ========================================================
                                S_RAND_SEARCH_B: begin
                                    rand_enable <= 1;
                                    
                                    if (rand_retry_cnt > 200) begin
                                        error_led <= 1;
                                        sub_state <= S_OP_SHOW_INFO;
                                    end else begin
                                    
                                        case (send_phase)
                                            0: begin // === 步骤1: 设定 B 的目标 ===
                                                if (op_mode == 2'b00) begin // [加法] B 维度必须等于 A
                                                    temp_m <= sel_m_a;
                                                    temp_n <= sel_n_a;
                                                    // 加法不需要扫描，直接去查是否存在
                                                    send_phase <= 4; 
                                                end 
                                                else begin // [乘法] B行=A列，B列随机扫描
                                                    temp_m <= sel_n_a; // 固定行
                                                    temp_n <= (rand_out % 5) + 1; // 随机列起点
                                                    elem_count <= 0; // 本地扫描计数
                                                    send_phase <= 1;
                                                end
                                            end
                
                                            // --- 乘法扫描逻辑 ---
                                            1: begin // 查询
                                                query_m <= temp_m;
                                                query_n <= temp_n; 
                                                send_phase <= 2; 
                                            end
                                            
                                            2: begin // 检查结果
                                                if (query_count > 0) begin
                                                    // >> 找到 B 了 <<
                                                    sel_m_b <= temp_m[2:0];
                                                    sel_n_b <= temp_n[2:0];
                                                    if (query_count == 1) sel_slot_b <= query_slot0_valid ? 0 : 1;
                                                    else sel_slot_b <= rand_out[0];

                                                    op_sel_a_done <= 1;
                                                    selecting_second <= 0; 
                                                    
                                                    // 准备 B 的回显信息
                                                    echo_m <= temp_m;
                                                    echo_n <= temp_n;
                                                    echo_slot <= (query_count == 1) ? (query_slot0_valid ? 0 : 1) : rand_out[0];
                                                    
                                                    // >>> 决定打印完 B 之后去哪 <<<
                                                    next_sub_state <= S_OP_CHECK; // 打印完B -> 去检查计算
                                                    
                                                    // >>> 立即出发去打印 B <<<
                                                    send_phase <= 0;
                                                    sub_state <= S_ECHO_PREP;
                                                end else begin
                                                    // >> 没找到，换下一列 <<
                                                    if (temp_n < 5) temp_n <= temp_n + 1;
                                                    else temp_n <= 1;
                                                    
                                                    elem_count <= elem_count + 1;
                                                    if (elem_count > 6) begin
                                                        // 这一行的所有列都试过了，都没有矩阵
                                                        // 说明这个 A 是个“死胡同”，没法做乘法
                                                        // >> 回退到 A 状态，重新选一个 A <<
                                                        rand_retry_cnt <= rand_retry_cnt + 5;
                                                        send_phase <= 0;
                                                        sub_state <= S_RAND_SEARCH_A; 
                                                    end else begin
                                                        send_phase <= 1; // 查下列
                                                    end
                                                end
                                            end

                                            // --- 加法检查逻辑 (不扫描) ---
                                            4: begin
                                                query_m <= temp_m;
                                                query_n <= temp_n;
                                                send_phase <= 5;
                                            end
                                            
                                            5: begin
                                                if (query_count > 0) begin
                                                    // 找到 B
                                                    sel_m_b <= temp_m[2:0];
                                                    sel_n_b <= temp_n[2:0];
                                                    if (query_count == 1) sel_slot_b <= query_slot0_valid ? 0 : 1;
                                                    else sel_slot_b <= rand_out[0];

                                                    op_sel_a_done <= 1;
                                                    selecting_second <= 0;
                                                    
                                                    echo_m <= temp_m;
                                                    echo_n <= temp_n;
                                                    echo_slot <= (query_count == 1) ? (query_slot0_valid ? 0 : 1) : rand_out[0];
                                                    
                                                    next_sub_state <= S_OP_CHECK;
                                                    send_phase <= 0;
                                                    sub_state <= S_ECHO_PREP;
                                                end else begin
                                                    error_led <= 1;
                                                    sub_state <= S_OP_SHOW_INFO;
                                                end
                                            end
                                        endcase
                                    end
                                end
                                // ========================================================
                                // 通用回显打印逻辑
                                // ========================================================
                                S_ECHO_PREP: begin
                                    disp_rd_m <= echo_m; disp_rd_n <= echo_n;
                                    disp_rd_slot <= echo_slot;
                                    disp_rd_row <= 0; disp_rd_col <= 0;
                                    send_phase <= 0; sub_state <= S_ECHO_PRINT;
                                end
                                
                                S_ECHO_PRINT: begin
                                    //移除外层的 if (!tx_busy)，防止因等待忙信号而错过 RAM 的 valid 脉冲
                                    case (send_phase)
                                        0: begin 
                                            // 发送初始换行，需等待空闲
                                            if (!tx_busy) begin 
                                                tx_data <= 8'h0A;
                                                tx_data_valid <= 1; 
                                                send_phase <= 1; 
                                            end 
                                        end
                                        
                                        1: begin 
                                            // 发起读请求（不依赖 tx_busy）
                                            disp_rd_en <= 1;
                                            send_phase <= 2; 
                                        end 
                                        
                                        2: begin 
                                            // 结束读请求
                                            disp_rd_en <= 0;
                                            send_phase <= 3; 
                                        end 
                                        
                                        3: begin 
                                            if (disp_rd_valid) begin 
                                                // 将数据暂存到 buffer
                                                if (disp_rd_elem > 9) tx_buffer <= "?"; // 简单保护
                                                else tx_buffer <= disp_rd_elem + 8'h30;
                                                
                                                send_phase <= 4; // 捕获成功，去发送
                                            end 
                                        end
                                        
                                        4: begin 
                                            // 将暂存的数据发送出去（此时才检查忙信号）
                                            if (!tx_busy) begin
                                                tx_data <= tx_buffer;
                                                tx_data_valid <= 1; 
                                                send_phase <= 5; 
                                            end
                                        end
                                        
                                        5: begin // 打印空格或换行 CR
                                            if (!tx_busy) begin
                                                if (disp_rd_col + 1 < disp_rd_n) begin
                                                    tx_data <= 8'h20; // 空格
                                                    tx_data_valid <= 1;
                                                    disp_rd_col <= disp_rd_col + 1; 
                                                    send_phase <= 1; // 回去读下一个列
                                                end else begin
                                                    tx_data <= 8'h0D; // CR
                                                    tx_data_valid <= 1; 
                                                    send_phase <= 6;
                                                end
                                            end
                                        end
        
                                        6: begin // 行末 LF
                                            if (!tx_busy) begin
                                                tx_data <= 8'h0A; // LF
                                                tx_data_valid <= 1;
                                                disp_rd_col <= 0;
                                                if (disp_rd_row + 1 < disp_rd_m) begin
                                                    disp_rd_row <= disp_rd_row + 1;
                                                    send_phase <= 1; // 下一行
                                                end else begin
                                                    send_phase <= 7; // 矩阵打印完成
                                                end
                                            end
                                        end
                                        
                                        7: begin 
                                            send_phase <= 0; 
                                            sub_state <= next_sub_state; 
                                        end
                                    endcase
                                end     
                                // ========================================================
                                // 打印标量值 (0-15)
                                // ========================================================
                                S_ECHO_SCALAR: begin
                                    case (send_phase)
                                        0: begin // 发送数字
                                             if (!tx_busy) begin
                                                 if (scalar_value >= 10) begin
                                                     tx_data <= "1"; // 十位 '1'
                                                     tx_data_valid <= 1;
                                                     send_phase <= 1;
                                                 end else begin
                                                     tx_data <= scalar_value + 8'h30; // 个位
                                                     tx_data_valid <= 1;
                                                     send_phase <= 2; // 跳过第二位
                                                 end
                                             end
                                        end
                                        
                                        1: begin // 发送第二位 (如果 >= 10)
                                             if (!tx_busy) begin
                                                 tx_data <= (scalar_value - 10) + 8'h30;
                                                 tx_data_valid <= 1;
                                                 send_phase <= 2;
                                             end
                                        end
                                        
                                        2: begin // 发送回车 CR
                                             if (!tx_busy) begin
                                                 tx_data <= 8'h0D;
                                                 tx_data_valid <= 1;
                                                 send_phase <= 3;
                                             end
                                        end
                                        
                                        3: begin // 发送换行 LF
                                             if (!tx_busy) begin
                                                 tx_data <= 8'h0A;
                                                 tx_data_valid <= 1;
                                                 send_phase <= 4;
                                             end
                                        end
                                        
                                        4: begin // 完成，去检查
                                             sub_state <= S_OP_CHECK;
                                             send_phase <= 0;
                                        end
                                    endcase
                                end                           
                                S_OP_GET_SCALAR: begin
                                    // 直接使用拨码开关输入标量值 (0-15)
                                    scalar_value <= scalar_input;
                                    error_led <= 0;
                                    sub_state <= S_OP_CHECK; // 立即进入检查/运算
                                end
                                
                                S_OP_CHECK: begin
                                    // 默认进入计算状态的标志
                                    check_pass = 0;

                                    // 1. 检查逻辑
                                    case (op_mode)
                                        2'b00: if (sel_m_a == sel_m_b && sel_n_a == sel_n_b) check_pass = 1; // 加法
                                        2'b01: check_pass = 1; // 转置
                                        2'b10: check_pass = 1; // 标量
                                        2'b11: if (sel_n_a == sel_m_b) check_pass = 1; // 乘法
                                    endcase

                                    // 2. 结果处理
                                    if (check_pass) begin
                                        // === 成功 ===
                                        // 无论之前是否在倒计时，现在都算成功，解除所有警报
                                        sub_state <= S_OP_CALC;
                                        error_led <= 0;
                                        countdown_active <= 0; // 【关键】成功后停止倒计时
                                    end else begin
                                        // === 失败 ===
                                        // 维度不匹配，激活倒计时模式，让用户限时重选
                                        error_led <= 1;
                                        // 无论当前是否已经在倒计时，只要输错，就强制重置时间为30秒
                                        // 去掉了 if (!countdown_active) 的判断
                                        countdown_sec <= countdown_cfg;   // 重置秒数
                                        countdown_val <= countdown_cfg;   // 【关键】同步更新显示值，让用户立刻看到变回30
                                        countdown_timer <= 0;             // 重置毫秒计数器
                                        countdown_active <= 1;            // 确保倒计时激活
                                        
                                        // 彻底清空当前选择，让用户从选维度 M 开始
                                        op_sel_a_done <= 0;
                                        selecting_second <= 0;
                                        op_dim_ready <= 0;
                                        op_listed_once <= 0;
                                        
                                        // 直接跳回维度选择，而不是去 S_OP_COUNTDOWN
                                        sub_state <= S_OP_SEL_DIM_M; 
                                    end
                                end
                                
                                S_OP_COUNTDOWN: begin
                                    sub_state <= S_OP_SEL_DIM_M;
                                end
                                
                                S_OP_CALC: begin
                                    led_status <= 2'b11;
                                    case (op_mode)
                                        2'b00: add_start <= 1;
                                        2'b01: trans_start <= 1;
                                        2'b10: scalar_start <= 1;
                                        2'b11: matmul_start <= 1;
                                    endcase
                                    sub_state <= S_OP_OUTPUT;
                                end
                                
                                S_OP_OUTPUT: begin
                                    if (add_done || trans_done || scalar_done || matmul_done) begin
                                        sub_state <= S_OP_DONE;
                                    end
                                end
                                
                                S_OP_DONE: begin
                                    // 运算结束，LED状态切回 "等待输入" (01)
                                    led_status <= 2'b01; 
                                    countdown_active <= 0;
                                    
                                    // 支持按确认键(btn_start)继续当前运算
                                    if (btn_start) begin
                                        // 1. 重置所有运算相关的标志位
                                        selecting_second <= 0; 
                                        op_sel_a_done    <= 0; 
                                        op_dim_ready     <= 0; 
                                        op_listed_once   <= 0; 
                                        error_led        <= 0; 
                                        
                                        // 2. 重置发送阶段计数器
                                        send_phase <= 0; 

                                        // 3. 跳转回 "展示矩阵概览" 状态
                                        // 这样流程就是：概览 -> 选维度 -> 选矩阵 -> 计算
                                        sub_state <= S_OP_SHOW_INFO;
                                    end
                                end
                                
                                // =============================================
                                // Bonus Convolution States
                                // =============================================
                                S_CALC_CONV_INPUT: begin
                                    conv_start <= 0; // Clear start pulse
                                    led_status <= 2'b01; // Input Mode
                                    conv_kernel_valid <= 0;
                                    
                                    if (!tx_busy) begin
                                        case (send_phase)
                                            // 0. Print CR LF
                                            0: begin tx_data <= 8'h0D; tx_data_valid <= 1; send_phase <= 1; end
                                            1: begin tx_data <= 8'h0A; tx_data_valid <= 1; send_phase <= 2; end

                                            // 1. Print Prompt "K:"
                                            2: begin tx_data <= "K"; tx_data_valid <= 1; send_phase <= 3; end
                                            3: begin tx_data <= ":"; tx_data_valid <= 1; send_phase <= 4; end
                                            
                                            // 2. Wait for Input
                                            4: begin
                                                if (uart_rx_done && uart_rx_data >= "0" && uart_rx_data <= "9") begin
                                                    // Send to Kernel Module
                                                    conv_kernel_in <= uart_rx_data[3:0]; // 0-9
                                                    conv_kernel_valid <= 1;
                                                    
                                                    // Echo Character
                                                    tx_data <= uart_rx_data;
                                                    tx_data_valid <= 1;
                                                    
                                                    send_phase <= 5; // Go to send space
                                                end
                                            end
                                            
                                            // 3. Send Space
                                            5: begin
                                                tx_data <= " ";
                                                tx_data_valid <= 1;
                                                
                                                if (elem_count < 8) begin
                                                    elem_count <= elem_count + 1;
                                                    send_phase <= 4; // Back to wait for next digit
                                                end else begin
                                                    // All 9 digits received
                                                    sub_state <= S_CALC_CONV_RUN;
                                                    send_phase <= 0;
                                                end
                                            end
                                        endcase
                                    end
                                end
                                
                                S_CALC_CONV_RUN: begin
                                    conv_kernel_valid <= 0;
                                    // Print CRLF before result
                                    if (!tx_busy) begin
                                        case (send_phase)
                                            0: begin tx_data <= 8'h0D; tx_data_valid <= 1; send_phase <= 1; end
                                            1: begin tx_data <= 8'h0A; tx_data_valid <= 1; send_phase <= 2; end
                                            2: begin
                                                // Module auto-starts after 9th input, just wait
                                                sub_state <= S_CALC_CONV_DONE;
                                            end
                                        endcase
                                    end
                                end
                                
                                S_CALC_CONV_DONE: begin
                                    conv_start <= 0;
                                    led_status <= 2'b11; // Busy/Done
                                    
                                    if (conv_done) begin
                                        // Wait for restart
                                        if (btn_start) begin
                                            sub_state <= S_CALC_CONV_INPUT;
                                            send_phase <= 0;
                                            elem_count <= 0;
                                        end
                                    end
                                end
                            endcase
                        end
                    end
                endcase
            end
        end
    end

endmodule