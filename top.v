module top (
    input  wire clk,          // 100MHz System Clock
    input  wire rst_n_in,     // S4 (U4) 复位按键
    input  wire uart_rx,      // EGO1 N5
    output wire uart_tx,      // EGO1 T4
    input  wire btn_s0,       // EGO1 R11 (确认)
    input  wire btn_s2,       // EGO1 R15 (返回)
    input  wire sw0_op,       // 运算类型 bit0
    input  wire sw1_op,       // 运算类型 bit1
    input  wire sw2,          // 标量值 bit0 (0-15)
    input  wire sw3,          // 标量值 bit1
    input  wire sw4,          // 标量值 bit2
    input  wire sw5,          // 标量值 bit3
    input  wire sw6_func,     // 功能选择 bit0
    input  wire sw7_func,     // 功能选择 bit1
    input  wire sw8,          // 卷积模式选择 (Bonus)
    output wire [7:0] led,    // LED状态显示
    output wire error_led,    // 错误LED指示
    
    // 7段数码管输出 (8个数码管)
    output wire [7:0] seg0,   // DN0段选
    output wire [7:0] seg1,   // DN1段选
    output wire [7:0] dig_sel // 8位片选 {DN1_K4..K1, DN0_K4..K1}
);

    // =========================================================
    // 系统信号
    // =========================================================
    wire sys_rst_n = ~rst_n_in;
    
    wire [1:0] op_mode_sel  = {sw1_op, sw0_op};
    wire [1:0] func_sel     = {sw7_func, sw6_func};
    wire [3:0] scalar_input = {sw5, sw4, sw3, sw2};  // 标量值输入 (0-15)
    
    // =========================================================
    // UART 信号
    // =========================================================
    wire [7:0] rx_data;
    wire       rx_done;
    wire       tx_busy;
    
    // =========================================================
    // 按钮消抖
    // =========================================================
    wire btn_start_pulse;
    wire btn_back_pulse;
    
    // =========================================================
    // FSM 信号
    // =========================================================
    wire        stor_wen;
    wire [3:0]  stor_m, stor_n;
    wire [7:0]  stor_elem_in;
    wire        stor_elem_valid;
    wire        stor_input_done;
    
    // 存储查询接口（给定维度，查询有多少匹配的矩阵）
    wire [3:0]  query_m, query_n;
    wire [1:0]  query_count;
    wire        query_slot0_valid, query_slot1_valid;
    
    wire        disp_rd_en;
    wire        disp_rd_slot;
    wire [2:0]  disp_rd_row, disp_rd_col;
    wire [3:0]  disp_rd_m, disp_rd_n;   // 展示时的维度
    wire [7:0]  disp_rd_elem;
    wire        disp_rd_valid;
    
    wire        fsm_tx_valid;
    wire [7:0]  fsm_tx_data;
    
    wire        add_start, trans_start, scalar_start, matmul_start;
    wire        add_done, trans_done, scalar_done, matmul_done;
    wire        add_busy, trans_busy, scalar_busy, matmul_busy;
    
    wire [7:0]  scalar_value;
    wire        sel_slot_a, sel_slot_b;
    wire [2:0]  sel_m_a, sel_n_a, sel_m_b, sel_n_b;
    
    wire        rand_enable;
    wire [3:0]  rand_out;
    
    wire [1:0]  main_state_out;
    wire [1:0]  fsm_led;
    wire [4:0]  countdown_val;
    wire        countdown_active;
    
    // =========================================================
    // 运算模块信号
    // =========================================================
    wire       add_rd_en, trans_rd_en, scalar_rd_en, matmul_rd_en;
    wire       add_rd_slot, trans_rd_slot, scalar_rd_slot, matmul_rd_slot;
    wire [2:0] add_rd_row, trans_rd_row, scalar_rd_row, matmul_rd_row;
    wire [2:0] add_rd_col, trans_rd_col, scalar_rd_col, matmul_rd_col;
    wire       add_res_valid, trans_res_valid, scalar_res_valid, matmul_res_valid;
    wire [7:0] add_res_elem, trans_res_elem, scalar_res_elem, matmul_res_elem;
    wire       add_row_end, trans_row_end, scalar_row_end, matmul_row_end;
    wire [2:0] matmul_rd_current_m, matmul_rd_current_n;
    // 存储读取信号
    wire [7:0] rd_elem;
    wire       rd_elem_valid;

    // Bonus Convolution Signals
    wire        conv_start;
    wire        conv_kernel_valid;
    wire [3:0]  conv_kernel_in;
    wire        conv_busy;
    wire        conv_done;
    wire        conv_out_valid;
    wire [11:0] conv_out_elem;
    wire        conv_row_end;
    wire        conv_last;
    wire [15:0] conv_cycle_count;
    reg  [15:0] conv_cycle_latched;
    reg         conv_done_latched;
    
    // 总线仲裁（包含维度信息）
    reg        mux_rd_en;
    reg        mux_rd_slot;
    reg [2:0]  mux_rd_row, mux_rd_col;
    reg [3:0]  mux_rd_m, mux_rd_n;   // 读取时的目标维度
    
    // =========================================================
    // LED 显示
    // =========================================================
    reg rx_activity_toggle;
    reg store_done_toggle;    // 写入完成反馈
    always @(posedge clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            rx_activity_toggle   <= 0;
            store_done_toggle    <= 0;
        end else begin
            if (rx_done)            rx_activity_toggle <= ~rx_activity_toggle;
            if (stor_input_done)    store_done_toggle  <= ~store_done_toggle; // 成功写入一次矩阵时翻转
        end
    end
    
    assign led[0] = sys_rst_n;
    assign led[1] = store_done_toggle; // 每写完一矩阵翻转，便于用户确认
    assign led[3:2] = fsm_led;
    assign led[5:4] = main_state_out;
    assign led[6] = 1'b0; // 错误LED指示，由FSM控制
    assign led[7] = tx_busy;

    // 卷积周期数锁存（在 conv_done 时捕获，显示用）
    always @(posedge clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            conv_cycle_latched <= 0;
            conv_done_latched  <= 0;        end else if (main_state_out == 0) begin
            // 返回主菜单时清除显示
            conv_cycle_latched <= 0;
            conv_done_latched  <= 0;        end else if (conv_start) begin
            // 新一次卷积开始时清零计数显示
            conv_cycle_latched <= 0;
            conv_done_latched  <= 0;
        end else if (conv_done) begin
            conv_cycle_latched <= conv_cycle_count;
            conv_done_latched  <= 1;
        end
    end
    
        always @(*) begin
        // 1. 硬件计算模块拥有最高优先级 (一旦开始计算，地址线必须锁定在计算模块上)
        if (matmul_busy) begin
            mux_rd_en   = matmul_rd_en;
            mux_rd_slot = matmul_rd_slot;
            mux_rd_row  = matmul_rd_row;
            mux_rd_col  = matmul_rd_col;
            mux_rd_m    = {1'b0, matmul_rd_current_m};
            mux_rd_n    = {1'b0, matmul_rd_current_n};
        end else if (scalar_busy) begin
            mux_rd_en   = scalar_rd_en;
            mux_rd_slot = scalar_rd_slot;
            mux_rd_row  = scalar_rd_row;
            mux_rd_col  = scalar_rd_col;
            mux_rd_m    = {1'b0, sel_m_a};
            mux_rd_n    = {1'b0, sel_n_a};
        end else if (trans_busy) begin
            mux_rd_en   = trans_rd_en;
            mux_rd_slot = trans_rd_slot;
            mux_rd_row  = trans_rd_row;
            mux_rd_col  = trans_rd_col;
            mux_rd_m    = {1'b0, sel_m_a};
            mux_rd_n    = {1'b0, sel_n_a};
        end else if (add_busy) begin
            mux_rd_en   = add_rd_en;
            mux_rd_slot = add_rd_slot;
            mux_rd_row  = add_rd_row;
            mux_rd_col  = add_rd_col;
            mux_rd_m    = {1'b0, sel_m_a};
            mux_rd_n    = {1'b0, sel_n_a};
        
        // 2. 如果没有计算在忙，且 FSM 处于 DISPLAY 状态 (包含纯展示 和 计算预览)，则允许 FSM 控制总线
        // 2'd3 对应 MAIN_DISPLAY 状态
        end else if (main_state_out == 2'd3) begin
            mux_rd_en   = disp_rd_en;
            mux_rd_slot = disp_rd_slot;
            mux_rd_row  = disp_rd_row;
            mux_rd_col  = disp_rd_col;
            mux_rd_m    = disp_rd_m;
            mux_rd_n    = disp_rd_n;
            
        // 3. 默认/空闲情况
        end else begin
            mux_rd_en   = 0;
            mux_rd_slot = 0;
            mux_rd_row  = 0;
            mux_rd_col  = 0;
            mux_rd_m    = 0; 
            mux_rd_n    = 0;
        end
    end
    
    // 展示模式读取结果
    assign disp_rd_elem  = rd_elem;
    assign disp_rd_valid = rd_elem_valid;
    
    // =========================================================
    // 结果输出和FIFO
    // =========================================================
    wire       calc_res_valid = add_res_valid | trans_res_valid | scalar_res_valid | matmul_res_valid | conv_out_valid;
    wire [15:0] calc_res_elem  = add_res_valid ? {8'b0, add_res_elem} : 
                                trans_res_valid ? {8'b0, trans_res_elem} :
                                scalar_res_valid ? {8'b0, scalar_res_elem} :
                                matmul_res_valid ? {8'b0, matmul_res_elem} :
                                conv_out_valid ? {4'b0, conv_out_elem} : 16'd0;
    wire       calc_row_end   = add_res_valid ? add_row_end :
                                trans_res_valid ? trans_row_end :
                                scalar_res_valid ? scalar_row_end :
                                matmul_res_valid ? matmul_row_end :
                                conv_row_end;

    // 扩大 FIFO 深度到 128，防止 8x10 矩阵结果溢出 (80 > 64)
    // 强制使用寄存器 (Registers) 以确保绝对零延迟读取，彻底消除BRAM/LUTRAM潜在时序问题
    (* ram_style = "registers" *) reg [15:0] res_fifo [0:127];          
    (* ram_style = "registers" *) reg        res_row_end_fifo [0:127];  
    
    // 增加指针位宽 (2^7 = 128)
    reg [6:0]  res_wr_ptr, res_rd_ptr;   
    
    // 增加计数器位宽 (需能存下 128)
    reg [7:0]  res_count;                
    
    wire       res_fifo_empty = (res_count == 0);
    
    reg        dec_pending;
    reg [15:0] dec_val;    // Upgrade to 16-bit
    reg        dec_row_end;    // 当前元素是否是行末
    reg [2:0]  dec_stage;      // 需3-bit: 0~7 states
    reg [2:0]  dec_start;      // 起始位：0=千位，1=百位，2=十位，3=个位
    
    // FIFO for TX - Increased to 512 to handle full burst without backpressure issues
    reg [7:0]  tx_fifo [0:511];
    reg [8:0]  tx_wr_ptr, tx_rd_ptr; // 9-bit for 512
    reg [9:0]  tx_count;             // 10-bit for count
    reg [7:0]  tx_data_hold;     // latch head before advancing pointer
    wire       tx_fifo_empty = (tx_count == 0);
    
    reg        tx_fifo_rd_en;
    reg        tx_en_reg;
    reg [1:0]  tx_state;
    
    always @(posedge clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            tx_wr_ptr    <= 0;
            tx_rd_ptr    <= 0;
            tx_count     <= 0;
            tx_data_hold <= 0;
            dec_pending  <= 0;
            dec_val      <= 0;
            dec_row_end  <= 0;
            dec_stage    <= 0;
            dec_start    <= 0;
            res_wr_ptr   <= 0;
            res_rd_ptr   <= 0;
            res_count    <= 0;
        end else begin
            // 结果缓冲队列的写入（独立处理）- 同时保存元素和行末标志
            // 将上限判断从 64 改为 128
            if (calc_res_valid && res_count < 128) begin
                res_fifo[res_wr_ptr] <= calc_res_elem;
                res_row_end_fifo[res_wr_ptr] <= calc_row_end;
                res_wr_ptr <= res_wr_ptr + 1;
            end
            
            // 从结果队列取数据进行十进制展开（独立处理）
            if (!dec_pending && res_count > 0) begin
                dec_val     <= res_fifo[res_rd_ptr];
                dec_row_end <= res_row_end_fifo[res_rd_ptr];
                dec_pending <= 1;
                res_rd_ptr  <= res_rd_ptr + 1;
                dec_stage   <= 0;
                // 计算起始位，跳过前导零 - 使用当前读指针的值
                case (1'b1)
                    (res_fifo[res_rd_ptr] >= 16'd1000): dec_start <= 3'd0; // 千位起
                    (res_fifo[res_rd_ptr] >= 16'd100):  dec_start <= 3'd1; // 百位起
                    (res_fifo[res_rd_ptr] >= 16'd10):   dec_start <= 3'd2; // 十位起
                    default:                            dec_start <= 3'd3; // 个位起
                endcase
            end
            
            // 结果队列计数：精确处理同时读写
            case ({calc_res_valid && res_count < 128, !dec_pending && res_count > 0})
                2'b10: res_count <= res_count + 1;
                2'b01: res_count <= res_count - 1;
                // 2'b11: 同时读写，计数不变
                // 2'b00: 无操作，计数不变
                default: res_count <= res_count;
            endcase
            // 写入 - 优先FSM单字节，其次结果十进制展开
            if (fsm_tx_valid && tx_count < 512) begin
                tx_fifo[tx_wr_ptr] <= fsm_tx_data;
                tx_wr_ptr <= tx_wr_ptr + 1;
            end else if (dec_pending && tx_count < 512) begin
                // 按 dec_start 起始输出，跳过前导零
                // stage 0~3: 千/百/十/个位；stage 4: 空格；stage 5: 换行CR；stage 6: 换行LF
                case (dec_stage + dec_start)
                    3'd0: tx_fifo[tx_wr_ptr] <= 8'h30 + (dec_val / 1000);             // 千位
                    3'd1: tx_fifo[tx_wr_ptr] <= 8'h30 + ((dec_val % 1000) / 100);     // 百位
                    3'd2: tx_fifo[tx_wr_ptr] <= 8'h30 + ((dec_val % 100) / 10);       // 十位
                    3'd3: tx_fifo[tx_wr_ptr] <= 8'h30 + (dec_val % 10);               // 个位
                    3'd4: tx_fifo[tx_wr_ptr] <= 8'h20;                                // 空格
                    3'd5: tx_fifo[tx_wr_ptr] <= 8'h0D;                                // CR
                    default: tx_fifo[tx_wr_ptr] <= 8'h0A;                             // LF
                endcase
                tx_wr_ptr <= tx_wr_ptr + 1;
                // 完成判断：非行末输出到空格后结束；行末输出到LF后结束
                if (dec_row_end) begin
                    // 行末：输出完 CR+LF 后结束
                    if (dec_stage + dec_start >= 3'd6) begin
                        dec_stage   <= 0;
                        dec_pending <= 0;
                    end else begin
                        dec_stage   <= dec_stage + 1;
                    end
                end else begin
                    // 非行末：输出完空格后结束
                    if (dec_stage + dec_start >= 3'd4) begin
                        dec_stage   <= 0;
                        dec_pending <= 0;
                    end else begin
                        dec_stage   <= dec_stage + 1;
                    end
                end
            end
            
            // 读取：在出队时提前锁存要发送的数据
            if (tx_fifo_rd_en && tx_count > 0) begin
                tx_data_hold <= tx_fifo[tx_rd_ptr];
                tx_rd_ptr <= tx_rd_ptr + 1;
            end
            
            // 计数：精确处理同时读写（写触发=FSM有效或十进制展开一次）
            case ({(fsm_tx_valid || (dec_pending && tx_count < 512)) , tx_fifo_rd_en && tx_count > 0})
                2'b10: tx_count <= tx_count + 1;
                2'b01: tx_count <= tx_count - 1;
                default: tx_count <= tx_count;
            endcase
        end
    end
    
    // TX 状态机
    always @(posedge clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            tx_fifo_rd_en <= 0;
            tx_en_reg <= 0;
            tx_state <= 0;
        end else begin
            case (tx_state)
                0: begin
                    tx_en_reg <= 0;
                    tx_fifo_rd_en <= 0;
                    if (!tx_fifo_empty && !tx_busy) begin
                        tx_fifo_rd_en <= 1; // 出队并锁存数据
                        tx_en_reg <= 1;     // 触发一次发送
                        tx_state <= 1;
                    end
                end
                1: begin
                    tx_en_reg <= 0;
                    tx_fifo_rd_en <= 0;
                    tx_state <= 2;
                end
                2: begin
                    if (tx_busy) tx_state <= 3;
                    else tx_state <= 3;
                end
                3: begin
                    if (!tx_busy) tx_state <= 0;
                end
            endcase
        end
    end
    
    // =========================================================
    // 模块实例化
    // =========================================================
    
    uart_rx #(
        .CLK_FREQ(100_000_000),
        .UART_BPS(9600)
    ) u_rx (
        .clk(clk),
        .rst_n(sys_rst_n),
        .uart_rxd(uart_rx),
        .uart_rx_done(rx_done),
        .uart_rx_data(rx_data)
    );
    
    uart_tx #(
        .CLK_FREQ(100_000_000),
        .UART_BPS(9600)
    ) u_tx (
        .clk(clk),
        .rst_n(sys_rst_n),
        .uart_tx_en(tx_en_reg),
        .uart_tx_data(tx_data_hold),
        .uart_txd(uart_tx),
        .uart_tx_busy(tx_busy)
    );
    
    debounce u_btn_start (
        .clk(clk),
        .rst(!sys_rst_n),
        .btn_in(btn_s0),
        .btn_out(btn_start_pulse)
    );
    
    debounce u_btn_back (
        .clk(clk),
        .rst(!sys_rst_n),
        .btn_in(btn_s2),
        .btn_out(btn_back_pulse)
    );
    
    lfsr_random u_rand (
        .clk(clk),
        .rst_n(sys_rst_n),
        .enable(rand_enable),
        .random_out(rand_out)
    );
    
    fsm_full u_fsm (
        .clk(clk),
        .rst_n(sys_rst_n),
        .op_mode(op_mode_sel),
        .func_sel(func_sel),
        .scalar_input(scalar_input),
        .sw8_conv_mode(sw8), // New input
        .btn_start(btn_start_pulse),
        .btn_back(btn_back_pulse),
        .uart_rx_done(rx_done),
        .uart_rx_data(rx_data),
        
        .store_wen(stor_wen),
        .store_m(stor_m),
        .store_n(stor_n),
        .store_elem_in(stor_elem_in),
        .store_elem_valid(stor_elem_valid),
        .storage_input_done(stor_input_done),
        
        // 存储查询接口
        .query_m(query_m),
        .query_n(query_n),
        .query_count(query_count),
        .query_slot0_valid(query_slot0_valid),
        .query_slot1_valid(query_slot1_valid),
        
        .disp_rd_en(disp_rd_en),
        .disp_rd_slot(disp_rd_slot),
        .disp_rd_row(disp_rd_row),
        .disp_rd_col(disp_rd_col),
        .disp_rd_m(disp_rd_m),
        .disp_rd_n(disp_rd_n),
        .disp_rd_elem(disp_rd_elem),
        .disp_rd_valid(disp_rd_valid),
        
        .tx_data_valid(fsm_tx_valid),
        .tx_data(fsm_tx_data),
        .tx_busy(tx_busy),
        
        .add_start(add_start),
        .trans_start(trans_start),
        .scalar_start(scalar_start),
        .matmul_start(matmul_start),
        .add_done(add_done),
        .trans_done(trans_done),
        .scalar_done(scalar_done),
        .matmul_done(matmul_done),
        .add_busy(add_busy),
        .trans_busy(trans_busy),
        .scalar_busy(scalar_busy),
        .matmul_busy(matmul_busy),
        
        .conv_start(conv_start),
        .conv_kernel_valid(conv_kernel_valid),
        .conv_kernel_in(conv_kernel_in),
        .conv_busy(conv_busy),
        .conv_done(conv_done),

        .scalar_value(scalar_value),
        .sel_slot_a(sel_slot_a),
        .sel_slot_b(sel_slot_b),
        .sel_m_a(sel_m_a),
        .sel_n_a(sel_n_a),
        .sel_m_b(sel_m_b),
        .sel_n_b(sel_n_b),
        
        .rand_enable(rand_enable),
        .rand_out(rand_out),
        
        .main_state_out(main_state_out),
        .led_status(fsm_led),
        .error_led(error_led),
        .countdown_val(countdown_val),
        .countdown_active(countdown_active)
    );
    
    matrix_storage #(
        .MAX_DIM(5),
        .SLOTS_PER_DIM(2),
        .ELEM_WIDTH(8)
    ) u_storage (
        .clk(clk),
        .rst(!sys_rst_n),
        .wen(stor_wen),
        .m(stor_m),
        .n(stor_n),
        .elem_in(stor_elem_in),
        .elem_valid(stor_elem_valid),
        .input_done(stor_input_done),
        // 读接口（带维度）
        .rd_en(mux_rd_en),
        .rd_m(mux_rd_m),
        .rd_n(mux_rd_n),
        .rd_slot_idx(mux_rd_slot),
        .rd_row_idx(mux_rd_row),
        .rd_col_idx(mux_rd_col),
        .rd_elem(rd_elem),
        .rd_elem_valid(rd_elem_valid),
        // 查询接口
        .query_m(query_m),
        .query_n(query_n),
        .query_count(query_count),
        .query_slot0_valid(query_slot0_valid),
        .query_slot1_valid(query_slot1_valid)
    );
    
    mat_add #(
        .DIM_WIDTH(3),
        .DATA_WIDTH(8)
    ) u_add (
        .clk(clk),
        .rst_n(sys_rst_n),
        .start(add_start),
        .m_sel(sel_m_a),
        .n_sel(sel_n_a),
        .slot_a_sel(sel_slot_a),
        .slot_a_valid(1'b1),
        .slot_b_sel(sel_slot_b),
        .slot_b_valid(1'b1),
        .ready(),
        .busy(add_busy),
        .done(add_done),
        .error(),
        .rd_en(add_rd_en),
        .rd_slot_idx(add_rd_slot),
        .rd_row_idx(add_rd_row),
        .rd_col_idx(add_rd_col),
        .rd_elem(rd_elem),
        .rd_elem_valid(rd_elem_valid),
        .out_valid(add_res_valid),
        .out_elem(add_res_elem),
        .out_row_end(add_row_end),
        .out_last(),
        .out_linear_idx(),
        .total_elements()
    );
    
    mat_transposition_v4 #(
        .DIM_WIDTH(3),
        .DATA_WIDTH(8)
    ) u_trans (
        .clk(clk),
        .rst_n(sys_rst_n),
        .start(trans_start),
        .m_sel(sel_m_a),
        .n_sel(sel_n_a),
        .slot_sel(sel_slot_a),
        .slot_valid(1'b1),
        .ready(),
        .busy(trans_busy),
        .done(trans_done),
        .error(),
        .total_elements(),
        .rd_en(trans_rd_en),
        .rd_slot_idx(trans_rd_slot),
        .rd_row_idx(trans_rd_row),
        .rd_col_idx(trans_rd_col),
        .rd_elem(rd_elem),
        .rd_elem_valid(rd_elem_valid),
        .out_valid(trans_res_valid),
        .out_elem(trans_res_elem),
        .out_row_end(trans_row_end),
        .out_last(),
        .out_row_idx(),
        .out_col_idx(),
        .out_linear_idx()
    );
    
    mat_scalar_mult #(
        .DIM_WIDTH(3),
        .DATA_WIDTH(8)
    ) u_scalar (
        .clk(clk),
        .rst_n(sys_rst_n),
        .start(scalar_start),
        .m_sel(sel_m_a),
        .n_sel(sel_n_a),
        .scalar(scalar_value),
        .slot_sel(sel_slot_a),
        .slot_valid(1'b1),
        .ready(),
        .busy(scalar_busy),
        .done(scalar_done),
        .error(),
        .rd_en(scalar_rd_en),
        .rd_slot_idx(scalar_rd_slot),
        .rd_row_idx(scalar_rd_row),
        .rd_col_idx(scalar_rd_col),
        .rd_elem(rd_elem),
        .rd_elem_valid(rd_elem_valid),
        .out_valid(scalar_res_valid),
        .out_elem(scalar_res_elem),
        .out_row_end(scalar_row_end),
        .out_last(),
        .out_linear_idx()
    );
    
    mat_mult #(
        .DIM_WIDTH(3),
        .DATA_WIDTH(8)
    ) u_matmul (
        .clk(clk),
        .rst_n(sys_rst_n),
        .start(matmul_start),
        .m_a(sel_m_a),
        .n_a(sel_n_a),
        .m_b(sel_m_b),
        .n_b(sel_n_b),
        .slot_a_sel(sel_slot_a),
        .slot_a_valid(1'b1),
        .slot_b_sel(sel_slot_b),
        .slot_b_valid(1'b1),
        .ready(),
        .busy(matmul_busy),
        .done(matmul_done),
        .error(),
        .rd_en(matmul_rd_en),
        .rd_slot_idx(matmul_rd_slot),
        .rd_row_idx(matmul_rd_row),
        .rd_col_idx(matmul_rd_col),
        .rd_elem(rd_elem),
        .rd_elem_valid(rd_elem_valid),
        .rd_current_m(matmul_rd_current_m),
        .rd_current_n(matmul_rd_current_n),
        .out_valid(matmul_res_valid),
        .out_elem(matmul_res_elem),
        .out_row_end(matmul_row_end),
        .out_last(),
        .out_linear_idx()
    );
    
    seg7_display u_seg7 (
        .clk(clk),
        .rst_n(sys_rst_n),
        .main_state(main_state_out),
        .func_sel(func_sel),
        .op_mode(op_mode_sel),
        .countdown_val(countdown_val),
        .countdown_active(countdown_active),
        .conv_mode(sw8),
        .conv_done(conv_done_latched),
        .conv_cycle(conv_cycle_latched),
        .seg0(seg0),
        .seg1(seg1),
        .dig_sel(dig_sel)
    );

    mat_conv_bonus #(
        .DATA_WIDTH(4),
        .KERNEL_WIDTH(4),
        .ACC_WIDTH(12)
    ) u_conv (
        .clk(clk),
        .rst_n(sys_rst_n),
        .start(conv_start),
        .kernel_in(conv_kernel_in),
        .kernel_valid(conv_kernel_valid),
        .kernel_ready(), 
        .busy(conv_busy),
        .done(conv_done),
        .out_valid(conv_out_valid),
        .out_elem(conv_out_elem),
        .out_row_end(conv_row_end),
        .out_last(conv_last),
        .out_row_idx(),
        .out_col_idx(),
        .cycle_count(conv_cycle_count)
    );

endmodule