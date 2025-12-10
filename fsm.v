// 完整FSM状态机 - 支持矩阵输入/生成/展示/运算的全流程
// 更新：支持按维度分组的矩阵存储（每种维度最多2个）
module fsm_full (
    input  wire        clk,
    input  wire        rst_n,
    
    // 控制输入
    input  wire [1:0]  op_mode,          // 运算类型: 00=加法, 01=转置, 10=标量乘, 11=矩阵乘
    input  wire [1:0]  func_sel,         // 功能选择: 00=输入, 01=生成, 10=展示, 11=运算
    input  wire        btn_start,        // 确认按钮
    input  wire        btn_back,         // 返回按钮
    
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
    
    // 存储读取接口 (用于展示) - 需要指定维度
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
    localparam MAIN_DISPLAY  = 2'd3;  // 也用于运算模式
    
    // =========================================================
    // 子状态定义
    // =========================================================
    // 通用
    localparam S_IDLE          = 5'd0;
    
    // 输入模式子状态
    localparam S_IN_GET_M      = 5'd1;
    localparam S_IN_GET_N      = 5'd2;
    localparam S_IN_SET_DIM    = 5'd3;   // 先设置维度
    localparam S_IN_START_STORE= 5'd4;   // 再发送wen脉冲
    localparam S_IN_RX_DATA    = 5'd5;
    localparam S_IN_DONE       = 5'd6;
    
    // 生成模式子状态
    localparam S_GEN_GET_M     = 5'd7;
    localparam S_GEN_GET_N     = 5'd8;
    localparam S_GEN_GET_CNT   = 5'd9;
    localparam S_GEN_SET_DIM   = 5'd10;  // 先设置维度
    localparam S_GEN_START     = 5'd11;  // 再发送wen脉冲
    localparam S_GEN_FILL      = 5'd12;
    localparam S_GEN_DONE      = 5'd13;
    
    // 展示模式子状态
    localparam S_DISP_START    = 5'd14;
    localparam S_DISP_SEND_INFO= 5'd15;
    localparam S_DISP_SEND_MAT = 5'd16;
    localparam S_DISP_DONE     = 5'd17;
    
    // 运算模式子状态
    localparam S_OP_SHOW_INFO  = 5'd18;
    localparam S_OP_SEL_DIM_M  = 5'd19;
    localparam S_OP_SEL_DIM_N  = 5'd20;
    localparam S_OP_SHOW_MATS  = 5'd21;
    localparam S_OP_SEL_MAT    = 5'd22;
    localparam S_OP_GET_SCALAR = 5'd23;
    localparam S_OP_CHECK      = 5'd24;
    localparam S_OP_COUNTDOWN  = 5'd25;
    localparam S_OP_CALC       = 5'd26;
    localparam S_OP_OUTPUT     = 5'd27;
    localparam S_OP_DONE       = 5'd28;
    
    // 发送子状态
    localparam S_TX_WAIT       = 5'd29;
    
    // 错误消息发送状态
    localparam S_OP_SEND_ERROR = 5'd30;
    
    // =========================================================
    // 寄存器定义
    // =========================================================
    reg [1:0]  main_state;
    reg [4:0]  sub_state;
    reg [4:0]  next_sub_state;  // 发送完成后返回的状态
    
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
    reg [4:0]  countdown_cfg;   // 配置的倒计时时间 (默认10秒)
    
    // 错误类型寄存器
    reg [1:0]  error_type;      // 0=无错误, 1=加法维度不同, 2=矩阵乘法维度不匹配, 3=无效维度
    reg [7:0]  err_msg_phase;   // 错误消息发送阶段
    
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
            countdown_cfg <= 5'd10;  // 默认10秒
            countdown_timer <= 0;
            countdown_sec <= 0;
            
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
            error_type <= 0;
            err_msg_phase <= 0;
            
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
            disp_rd_en <= 0;  // disp_rd_en 作为脉冲信号每周期复位，在需要时设为1
            
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
                                    send_phase <= 100; // 【修改点1】在此处初始化
                                end
                                2'b11: begin
                                    main_state <= MAIN_DISPLAY;
                                    sub_state <= S_OP_SHOW_INFO;
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
                                    // 接受ASCII '1'-'5' (0x31-0x35)
                                    if (uart_rx_data >= 8'h31 && uart_rx_data <= 8'h35) begin
                                        temp_m <= uart_rx_data - 8'h30; // '1'->1, '5'->5
                                        sub_state <= S_IN_GET_N;
                                    end else begin
                                        error_led <= 1;  // 维度超范围
                                    end
                                end
                            end
                            
                            S_IN_GET_N: begin
                                if (uart_rx_done) begin
                                    // 接受ASCII '1'-'5' (0x31-0x35)
                                    if (uart_rx_data >= 8'h31 && uart_rx_data <= 8'h35) begin
                                        temp_n <= uart_rx_data - 8'h30;
                                        sub_state <= S_IN_SET_DIM;  // 先设置维度
                                        error_led <= 0;
                                    end else begin
                                        error_led <= 1;
                                    end
                                end
                            end
                            
                            // 新增：先设置维度，让存储模块计算好地址
                            S_IN_SET_DIM: begin
                                store_m <= temp_m;
                                store_n <= temp_n;
                                elem_count <= 0;
                                total_elems <= temp_m * temp_n;
                                sub_state <= S_IN_START_STORE;
                            end
                            
                            // 发送wen脉冲（此时m、n已经稳定）
                            S_IN_START_STORE: begin
                                store_wen <= 1;
                                sub_state <= S_IN_RX_DATA;
                            end
                            
                            S_IN_RX_DATA: begin
                                if (uart_rx_done) begin
                                    // 接受ASCII '0'-'9' (0x30-0x39)
                                    if (uart_rx_data >= 8'h30 && uart_rx_data <= 8'h39) begin
                                        if (elem_count < total_elems) begin
                                            store_elem_in <= uart_rx_data - 8'h30;
                                            store_elem_valid <= 1;
                                            elem_count <= elem_count + 1;
                                            error_led <= 0;
                                        end
                                        // 超出个数的忽略
                                    end else begin
                                        error_led <= 1;  // 元素超范围，需重新输入该元素
                                    end
                                end
                                
                                if (storage_input_done) begin
                                    current_mat_idx <= current_mat_idx + 1;
                                    sub_state <= S_IN_DONE;
                                end
                            end
                            
                            S_IN_DONE: begin
                                led_status <= 2'b11;
                                // 自动进入下一次输入，无需再次按键
                                sub_state <= S_IN_GET_M;
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
                                        error_led <= 0;
                                    end else begin
                                        error_led <= 1;
                                    end
                                end
                            end
                            
                            // 新增：先设置维度
                            S_GEN_SET_DIM: begin
                                store_m <= temp_m;
                                store_n <= temp_n;
                                elem_count <= 0;
                                total_elems <= temp_m * temp_n;
                                sub_state <= S_GEN_START;
                            end
                            
                            // 发送wen脉冲（此时m、n已经稳定）
                            S_GEN_START: begin
                                store_wen <= 1;
                                sub_state <= S_GEN_FILL;
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
                                led_status <= 2'b11;
                            end
                        endcase
                    end
                    
                    // =============================================
                    // 展示/运算模式
                    // =============================================
                    MAIN_DISPLAY: begin
                        if (func_sel == 2'b10) begin
                            // ========== 展示模式 ==========
                            led_status <= 2'b01;
                            case (sub_state)
                                S_DISP_START: begin
                                    // 进入展示模式时先提示输入 m
                                    disp_m <= 0;
                                    disp_n <= 0;
                                    // 【修改点2】删除了这里的 send_phase <= 100;
                                    
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
                                                tx_data <= disp_rd_elem + 8'h30;
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
                                    led_status <= 2'b11;
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
                                                // 先设置第一个查询
                                                query_m <= 4'd1;
                                                query_n <= 4'd1;
                                                send_phase <= 1;
                                            end
                                            1: begin
                                                // 等待一个周期让查询结果稳定
                                                send_phase <= 2;
                                            end
                                            2: begin
                                                // 累加当前维度的矩阵数
                                                total_mat_count <= total_mat_count + query_count;
                                                // 移动到下一个维度
                                                if (scan_n < 5) begin
                                                    scan_n <= scan_n + 1;
                                                    query_m <= {1'b0, scan_m};
                                                    query_n <= {1'b0, scan_n} + 1;
                                                    send_phase <= 1;
                                                end else if (scan_m < 5) begin
                                                    scan_m <= scan_m + 1;
                                                    scan_n <= 1;
                                                    query_m <= {1'b0, scan_m} + 1;
                                                    query_n <= 4'd1;
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
                                                query_m <= 4'd1;
                                                query_n <= 4'd1;
                                                send_phase <= 5;
                                            end
                                            
                                            // === 阶段 5-12: 发送每种维度的 m*n*x ===
                                            5: begin
                                                // 等待查询结果稳定
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
                                                // 移动到下一个维度，同时更新查询
                                                if (scan_n < 5) begin
                                                    scan_n <= scan_n + 1;
                                                    query_m <= {1'b0, scan_m};
                                                    query_n <= {1'b0, scan_n} + 1;
                                                    send_phase <= 5;
                                                end else if (scan_m < 5) begin
                                                    scan_m <= scan_m + 1;
                                                    scan_n <= 1;
                                                    query_m <= {1'b0, scan_m} + 1;
                                                    query_n <= 4'd1;
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
                                    error_led <= 0;
                                    if (uart_rx_done) begin
                                        // 接受ASCII '1'-'5' (0x31-0x35)
                                        if (uart_rx_data >= 8'h31 && uart_rx_data <= 8'h35) begin
                                            sel_dim_m <= uart_rx_data - 8'h30;
                                            op_dim_ready <= 0; // 重新开始收集维度
                                            op_listed_once <= 0; // 允许新维度重新输出
                                            
                                            // 【修改点】增加了条件判断！
                                            // 如果 op_sel_a_done 为 1，说明是矩阵乘法回头选 B，不要清零！
                                            // 只有在选 A (op_sel_a_done == 0) 时，才重置这些标志。
                                            if (!op_sel_a_done) begin
                                                selecting_second <= 0;
                                                op_sel_a_done <= 0; 
                                            end
                                            
                                            sub_state <= S_OP_SEL_DIM_N;
                                        end else begin
                                            error_led <= 1;
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
                                        // === 【新增状态】地址稳定缓冲 ===
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
                                    // 用户输入矩阵编号 (ASCII '1' 或 '2')
                                    if (uart_rx_done) begin
                                        if (uart_rx_data >= 8'h31 && uart_rx_data <= 8'h32) begin
                                            if (!op_sel_a_done) begin
                                                sel_slot_a <= uart_rx_data[0] - 1; // '1'->0, '2'->1
                                                sel_m_a   <= sel_dim_m[2:0];
                                                sel_n_a   <= sel_dim_n[2:0];
                                                op_sel_a_done <= 1;
                                                error_led <= 0;
                                                // 根据运算类型决定是否需要第二个矩阵
                                                case (op_mode)
                                                    2'b01: begin  // 转置：只需1个矩阵，立即进入检查
                                                        sub_state <= S_OP_CHECK;
                                                    end
                                                    2'b10: begin  // 标量乘：1个矩阵，再输入标量
                                                        sub_state <= S_OP_GET_SCALAR;
                                                    end
                                                    default: begin  // 加法、矩阵乘：需要第二个矩阵
                                                        selecting_second <= 1; // 标记正在选第二个
                                                        
                                                        if (op_mode == 2'b11) begin 
                                                            // 【新增逻辑】如果是矩阵乘法，A和B维度不同
                                                            // 必须跳回前面，让用户输入矩阵 B 的维度
                                                            sub_state <= S_OP_SEL_DIM_M; 
                                                            
                                                            // 重置维度输入相关的标志位，以便重新接收 UART 输入
                                                            op_dim_ready <= 0;
                                                            op_listed_once <= 0;
                                                            
                                                            // 注意：此时 sel_m_a, sel_n_a 已经被上面的代码锁存好了，
                                                            // 所以我们重新使用 sel_dim_m/n 变量来存 B 的维度是安全的。
                                                        end 
                                                        // 如果是加法(00)，保持原逻辑（维度相同，直接去选矩阵B）
                                                    end
                                                endcase
                                            end else if (selecting_second) begin
                                                sel_slot_b <= uart_rx_data[0] - 1; // '1'->0, '2'->1
                                                sel_m_b   <= sel_dim_m[2:0];
                                                sel_n_b   <= sel_dim_n[2:0];
                                                selecting_second <= 0;
                                                error_led <= 0;
                                                sub_state <= S_OP_CHECK; // 收到第二个后立即检查/运算
                                            end
                                        end else begin
                                            error_led <= 1;
                                        end
                                    end
                                end
                                
                                S_OP_GET_SCALAR: begin
                                    // UART输入标量 (ASCII '0'-'9')
                                    if (uart_rx_done) begin
                                        if (uart_rx_data >= 8'h30 && uart_rx_data <= 8'h39) begin
                                            scalar_value <= uart_rx_data - 8'h30;
                                            error_led <= 0;
                                            sub_state <= S_OP_CHECK; // 收到标量后立即进入检查/运算
                                        end else begin
                                            error_led <= 1;
                                        end
                                    end
                                end
                                
                                S_OP_CHECK: begin
                                    // 检查运算数合法性
                                    case (op_mode)
                                        2'b00: begin  // 加法：维度必须相同
                                            if (sel_m_a == sel_m_b && sel_n_a == sel_n_b) begin
                                                sub_state <= S_OP_CALC;
                                                error_led <= 0;
                                            end else begin
                                                error_led <= 1;
                                                error_type <= 2'd1;  // 加法维度错误
                                                err_msg_phase <= 0;
                                                sub_state <= S_OP_SEND_ERROR;
                                            end
                                        end
                                        2'b01: begin  // 转置：总是合法
                                            sub_state <= S_OP_CALC;
                                        end
                                        2'b10: begin  // 标量乘：总是合法
                                            sub_state <= S_OP_CALC;
                                        end
                                        2'b11: begin  // 矩阵乘：A的列数=B的行数
                                            if (sel_n_a == sel_m_b) begin
                                                sub_state <= S_OP_CALC;
                                                error_led <= 0;
                                            end else begin
                                                error_led <= 1;
                                                error_type <= 2'd2;  // 矩阵乘维度错误
                                                err_msg_phase <= 0;
                                                sub_state <= S_OP_SEND_ERROR;
                                            end
                                        end
                                    endcase
                                end
                                
                                // 发送错误消息状态
                                S_OP_SEND_ERROR: begin
                                    if (!tx_busy) begin
                                        case (error_type)
                                            2'd1: begin  // 加法维度错误: "ERR:ADD DIM MISMATCH\r\n"
                                                case (err_msg_phase)
                                                    0:  begin tx_data <= "E"; tx_data_valid <= 1; err_msg_phase <= 1; end
                                                    1:  begin tx_data <= "R"; tx_data_valid <= 1; err_msg_phase <= 2; end
                                                    2:  begin tx_data <= "R"; tx_data_valid <= 1; err_msg_phase <= 3; end
                                                    3:  begin tx_data <= ":"; tx_data_valid <= 1; err_msg_phase <= 4; end
                                                    4:  begin tx_data <= "A"; tx_data_valid <= 1; err_msg_phase <= 5; end
                                                    5:  begin tx_data <= "D"; tx_data_valid <= 1; err_msg_phase <= 6; end
                                                    6:  begin tx_data <= "D"; tx_data_valid <= 1; err_msg_phase <= 7; end
                                                    7:  begin tx_data <= " "; tx_data_valid <= 1; err_msg_phase <= 8; end
                                                    8:  begin tx_data <= "D"; tx_data_valid <= 1; err_msg_phase <= 9; end
                                                    9:  begin tx_data <= "I"; tx_data_valid <= 1; err_msg_phase <= 10; end
                                                    10: begin tx_data <= "M"; tx_data_valid <= 1; err_msg_phase <= 11; end
                                                    11: begin tx_data <= " "; tx_data_valid <= 1; err_msg_phase <= 12; end
                                                    12: begin tx_data <= "M"; tx_data_valid <= 1; err_msg_phase <= 13; end
                                                    13: begin tx_data <= "I"; tx_data_valid <= 1; err_msg_phase <= 14; end
                                                    14: begin tx_data <= "S"; tx_data_valid <= 1; err_msg_phase <= 15; end
                                                    15: begin tx_data <= "M"; tx_data_valid <= 1; err_msg_phase <= 16; end
                                                    16: begin tx_data <= "A"; tx_data_valid <= 1; err_msg_phase <= 17; end
                                                    17: begin tx_data <= "T"; tx_data_valid <= 1; err_msg_phase <= 18; end
                                                    18: begin tx_data <= "C"; tx_data_valid <= 1; err_msg_phase <= 19; end
                                                    19: begin tx_data <= "H"; tx_data_valid <= 1; err_msg_phase <= 200; end
                                                    200: begin tx_data <= 8'h0D; tx_data_valid <= 1; err_msg_phase <= 201; end
                                                    201: begin tx_data <= 8'h0A; tx_data_valid <= 1; err_msg_phase <= 202; end
                                                    202: begin
                                                        // 启动倒计时
                                                        countdown_sec <= countdown_cfg;
                                                        countdown_timer <= 0;
                                                        countdown_active <= 1;
                                                        countdown_val <= countdown_cfg;
                                                        // 重置选择状态
                                                        selecting_second <= 0;
                                                        op_sel_a_done <= 0;
                                                        op_dim_ready <= 0;
                                                        op_listed_once <= 0;
                                                        sub_state <= S_OP_COUNTDOWN;
                                                    end
                                                    default: err_msg_phase <= 0;
                                                endcase
                                            end
                                            
                                            2'd2: begin  // 矩阵乘维度错误: "ERR:MULT An!=Bm\r\n"
                                                case (err_msg_phase)
                                                    0:  begin tx_data <= "E"; tx_data_valid <= 1; err_msg_phase <= 1; end
                                                    1:  begin tx_data <= "R"; tx_data_valid <= 1; err_msg_phase <= 2; end
                                                    2:  begin tx_data <= "R"; tx_data_valid <= 1; err_msg_phase <= 3; end
                                                    3:  begin tx_data <= ":"; tx_data_valid <= 1; err_msg_phase <= 4; end
                                                    4:  begin tx_data <= "M"; tx_data_valid <= 1; err_msg_phase <= 5; end
                                                    5:  begin tx_data <= "U"; tx_data_valid <= 1; err_msg_phase <= 6; end
                                                    6:  begin tx_data <= "L"; tx_data_valid <= 1; err_msg_phase <= 7; end
                                                    7:  begin tx_data <= "T"; tx_data_valid <= 1; err_msg_phase <= 8; end
                                                    8:  begin tx_data <= " "; tx_data_valid <= 1; err_msg_phase <= 9; end
                                                    9:  begin tx_data <= "A"; tx_data_valid <= 1; err_msg_phase <= 10; end
                                                    10: begin tx_data <= "n"; tx_data_valid <= 1; err_msg_phase <= 11; end
                                                    11: begin tx_data <= "!"; tx_data_valid <= 1; err_msg_phase <= 12; end
                                                    12: begin tx_data <= "="; tx_data_valid <= 1; err_msg_phase <= 13; end
                                                    13: begin tx_data <= "B"; tx_data_valid <= 1; err_msg_phase <= 14; end
                                                    14: begin tx_data <= "m"; tx_data_valid <= 1; err_msg_phase <= 200; end
                                                    200: begin tx_data <= 8'h0D; tx_data_valid <= 1; err_msg_phase <= 201; end
                                                    201: begin tx_data <= 8'h0A; tx_data_valid <= 1; err_msg_phase <= 202; end
                                                    202: begin
                                                        // 启动倒计时
                                                        countdown_sec <= countdown_cfg;
                                                        countdown_timer <= 0;
                                                        countdown_active <= 1;
                                                        countdown_val <= countdown_cfg;
                                                        // 重置选择状态
                                                        selecting_second <= 0;
                                                        op_sel_a_done <= 0;
                                                        op_dim_ready <= 0;
                                                        op_listed_once <= 0;
                                                        sub_state <= S_OP_COUNTDOWN;
                                                    end
                                                    default: err_msg_phase <= 0;
                                                endcase
                                            end
                                            
                                            default: begin
                                                // 无效错误类型，直接进入倒计时
                                                countdown_sec <= countdown_cfg;
                                                countdown_timer <= 0;
                                                countdown_active <= 1;
                                                countdown_val <= countdown_cfg;
                                                selecting_second <= 0;
                                                op_sel_a_done <= 0;
                                                op_dim_ready <= 0;
                                                op_listed_once <= 0;
                                                sub_state <= S_OP_COUNTDOWN;
                                            end
                                        endcase
                                    end
                                end
                                
                                S_OP_COUNTDOWN: begin
                                    countdown_val <= countdown_sec;
                                    
                                    // 1秒计数
                                    if (countdown_timer >= 100_000_000 - 1) begin
                                        countdown_timer <= 0;
                                        if (countdown_sec > 0) begin
                                            countdown_sec <= countdown_sec - 1;
                                        end else begin
                                            // 超时，返回重选运算数起始阶段
                                            countdown_active <= 0;
                                            error_led <= 0;
                                            error_type <= 0;
                                            selecting_second <= 0;
                                            op_sel_a_done <= 0;
                                            op_dim_ready <= 0;
                                            op_listed_once <= 0;
                                            send_phase <= 0;  // 重新发送概览信息
                                            sub_state <= S_OP_SHOW_INFO;
                                        end
                                    end else begin
                                        countdown_timer <= countdown_timer + 1;
                                    end
                                    
                                    // 倒计时期间允许通过UART重新输入
                                    if (uart_rx_done) begin
                                        // 配置倒计时时间 ASCII '5'-'9' -> 5~9秒，'0' -> 10秒
                                        // 格式: 'T' + 数字 配置时间，或直接输入数字选矩阵
                                        if (uart_rx_data >= 8'h31 && uart_rx_data <= 8'h35) begin
                                            // ASCII '1'-'5' 可能是维度输入
                                            if (!op_sel_a_done) begin
                                                // 正在输入第一个矩阵的维度m
                                                sel_dim_m <= uart_rx_data - 8'h30;
                                                op_dim_ready <= 0;
                                                op_listed_once <= 0;
                                                // 继续等待维度n，但不退出倒计时
                                            end else if (selecting_second && !op_dim_ready) begin
                                                // 正在输入第二个矩阵的维度m
                                                sel_dim_m <= uart_rx_data - 8'h30;
                                                op_dim_ready <= 0;
                                                op_listed_once <= 0;
                                            end
                                        end
                                        
                                        // 处理维度n输入和矩阵槽选择
                                        if (uart_rx_data >= 8'h31 && uart_rx_data <= 8'h32) begin
                                            // '1' 或 '2' - 选择矩阵槽
                                            if (op_dim_ready) begin
                                                if (!op_sel_a_done) begin
                                                    sel_slot_a <= uart_rx_data[0] - 1;
                                                    sel_m_a <= sel_dim_m[2:0];
                                                    sel_n_a <= sel_dim_n[2:0];
                                                    op_sel_a_done <= 1;
                                                    // 根据运算类型决定下一步
                                                    case (op_mode)
                                                        2'b01: begin  // 转置：只需1个
                                                            sub_state <= S_OP_CHECK;
                                                            countdown_active <= 0;
                                                            error_led <= 0;
                                                        end
                                                        2'b10: begin  // 标量乘：需要标量
                                                            sub_state <= S_OP_GET_SCALAR;
                                                            countdown_active <= 0;
                                                            error_led <= 0;
                                                        end
                                                        default: begin  // 加法/矩阵乘：需要第二个
                                                            selecting_second <= 1;
                                                            if (op_mode == 2'b11) begin
                                                                // 矩阵乘需要重新输入B的维度
                                                                op_dim_ready <= 0;
                                                                op_listed_once <= 0;
                                                            end
                                                        end
                                                    endcase
                                                end else if (selecting_second && op_dim_ready) begin
                                                    sel_slot_b <= uart_rx_data[0] - 1;
                                                    sel_m_b <= sel_dim_m[2:0];
                                                    sel_n_b <= sel_dim_n[2:0];
                                                    selecting_second <= 0;
                                                    // 收齐后重新检查
                                                    sub_state <= S_OP_CHECK;
                                                    countdown_active <= 0;
                                                    error_led <= 0;
                                                end
                                            end
                                        end
                                        
                                        // 处理维度n输入 (可以是1-5)
                                        if (uart_rx_data >= 8'h31 && uart_rx_data <= 8'h35 && 
                                            sel_dim_m != 0 && !op_dim_ready) begin
                                            // 如果已输入m且n尚未确定
                                            sel_dim_n <= uart_rx_data - 8'h30;
                                            query_m <= sel_dim_m;
                                            query_n <= uart_rx_data - 8'h30;
                                            op_dim_ready <= 1;
                                        end
                                        
                                        // 配置倒计时: 'C' + 数字 (5-15)
                                        // 或直接输入数字 '5'-'9' 配置 5-9秒
                                        // 输入 'A' 配置 10秒, 'B' 配置 11秒, ... 'F' 配置 15秒
                                        if (uart_rx_data >= 8'h41 && uart_rx_data <= 8'h46) begin
                                            // 'A'-'F' -> 10-15秒
                                            countdown_cfg <= uart_rx_data - 8'h41 + 5'd10;
                                        end
                                    end
                                    
                                    // 按确认键时，如果已完成所有输入则立即检查
                                    if (btn_start) begin
                                        if (op_mode == 2'b01 || op_mode == 2'b10) begin
                                            // 转置或标量乘：只需一个矩阵
                                            if (op_sel_a_done) begin
                                                sub_state <= S_OP_CHECK;
                                                countdown_active <= 0;
                                                error_led <= 0;
                                            end
                                        end else begin
                                            // 加法或矩阵乘：需要两个矩阵
                                            if (op_sel_a_done && !selecting_second) begin
                                                sub_state <= S_OP_CHECK;
                                                countdown_active <= 0;
                                                error_led <= 0;
                                            end
                                        end
                                    end
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
                                    led_status <= 2'b01;
                                    countdown_active <= 0;
                                    // 按返回键回主菜单
                                end
                            endcase
                        end
                    end
                endcase
            end
        end
    end

endmodule
