module top (
    input  wire clk,      // 100MHz System Clock
    input  wire rst_n_in, // 改用 S4 (P2) 作为复位，按下为高
    input  wire uart_rx,  // EGO1 N5
    output wire uart_tx,  // EGO1 T4
    input  wire btn_s0,   // EGO1 R11 (启动计算/确认)
    input  wire btn_s2,   // EGO1 R15 (返回主菜单)
    input  wire sw0_op,   // 操作模式选择 bit0 (运算类型)
    input  wire sw1_op,   // 操作模式选择 bit1 (运算类型)
    input  wire sw6_func, // 功能选择 bit0 (主菜单)
    input  wire sw7_func, // 功能选择 bit1 (主菜单)
    output wire [7:0] led, // LED显示状态
    
    // 7段数码管输出
    output wire [7:0] seg0,   // DN0段选 {DP0, G0, F0, E0, D0, C0, B0, A0}
    output wire [7:0] seg1,   // DN1段选 {DP1, G1, F1, E1, D1, C1, B1, A1}
    output wire [3:0] dig_sel // 片选 {DN0_K4, DN0_K3, DN0_K2, DN0_K1}
);

    // 1. 系统复位处理 (适配 EGO1 普通按键：按下为高，松开为低)
    // 我们需要低电平复位，所以按下(1)时产生复位(0)
    wire sys_rst_n;
    assign sys_rst_n = ~rst_n_in;
    
    // 操作模式选择 (2bit: 00=加法, 01=转置, 10=标量乘, 11=矩阵乘)
    wire [1:0] op_mode_sel;
    assign op_mode_sel = {sw1_op, sw0_op}; 

    // 功能选择 (2bit: 00=输入, 01=生成, 10=展示, 11=运算)
    wire [1:0] func_sel;
    assign func_sel = {sw7_func, sw6_func}; 

    // 2. 模块信号
    wire [7:0] rx_data;
    wire       rx_done;
    wire btn_start_pulse;
    wire btn_back_pulse;  // 返回按钮消抖后脉冲
    
    // FSM <-> Storage
    wire       stor_wen;
    wire [3:0] stor_m;
    wire [3:0] stor_n;
    wire [7:0] stor_elem_in;
    wire       stor_elem_valid;
    wire       stor_input_done;
    
    // Storage 读取信号
    wire [7:0] rd_elem;
    wire       rd_elem_valid;
    
    // 加法器信号
    wire       add_start;
    wire       add_done;
    wire       add_busy;
    wire       add_rd_en;
    wire       add_rd_slot;
    wire [2:0] add_rd_row;
    wire [2:0] add_rd_col;
    wire       add_res_valid;
    wire [7:0] add_res_elem;
    
    // 转置器信号
    wire       trans_start;
    wire       trans_done;
    wire       trans_busy;
    wire       trans_rd_en;
    wire       trans_rd_slot;
    wire [2:0] trans_rd_row;
    wire [2:0] trans_rd_col;
    wire       trans_res_valid;
    wire [7:0] trans_res_elem;
    
    // 标量乘器信号
    wire       scalar_start;
    wire       scalar_done;
    wire       scalar_busy;
    wire       scalar_rd_en;
    wire       scalar_rd_slot;
    wire [2:0] scalar_rd_row;
    wire [2:0] scalar_rd_col;
    wire       scalar_res_valid;
    wire [7:0] scalar_res_elem;
    wire [7:0] scalar_value;
    
    // 矩阵乘器信号
    wire       matmul_start;
    wire       matmul_done;
    wire       matmul_busy;
    wire       matmul_rd_en;
    wire       matmul_rd_slot;
    wire [2:0] matmul_rd_row;
    wire [2:0] matmul_rd_col;
    wire       matmul_res_valid;
    wire [7:0] matmul_res_elem;
    
    // 总线仲裁后的信号
    reg        mux_rd_en;
    reg        mux_rd_slot;
    reg [2:0]  mux_rd_row;
    reg [2:0]  mux_rd_col;
    
    // 结果合并信号
    wire       final_res_valid;
    wire [7:0] final_res_elem;
    wire       fifo_empty;
    wire [7:0] fifo_dout;
    wire       tx_busy;
    reg        fifo_rd_en;
    reg        tx_en_reg;

    // FSM 状态灯和槽位追踪
    wire [1:0] fsm_led;
    wire       current_mat_slot;  // 当前操作的矩阵槽位
    wire [1:0] main_state_out;    // 主状态输出

    // =========================================================
    // LED 调试逻辑
    // =========================================================
    reg rx_activity_toggle; // 用于指示接收到数据的翻转信号
    always @(posedge clk or negedge sys_rst_n) begin
        if(!sys_rst_n) rx_activity_toggle <= 0;
        else if(rx_done) rx_activity_toggle <= ~rx_activity_toggle;
    end

    // LED0: 系统运行指示 (亮=运行, 灭=复位)
    assign led[0] = sys_rst_n; 
    // LED1: 数据接收指示 (接收数据时翻转/闪烁)
    assign led[1] = rx_activity_toggle;
    // LED2-3: FSM 子状态 (亮起代表 Ready 或 Computing)
    assign led[3:2] = fsm_led; 
    // LED4-5: 主状态指示 (00=菜单, 01=输入, 10=生成, 11=展示/运算)
    assign led[5:4] = main_state_out;
    // LED7: 调试 - 串口发送忙状态
    assign led[7] = tx_busy;
    assign led[6] = 0;

    // =========================================================
    // 总线仲裁逻辑 - 根据谁在忙决定谁控制存储器读取
    // =========================================================
    always @(*) begin
        if (matmul_busy) begin
            mux_rd_en   = matmul_rd_en;
            mux_rd_slot = matmul_rd_slot;
            mux_rd_row  = matmul_rd_row;
            mux_rd_col  = matmul_rd_col;
        end else if (scalar_busy) begin
            mux_rd_en   = scalar_rd_en;
            mux_rd_slot = scalar_rd_slot;
            mux_rd_row  = scalar_rd_row;
            mux_rd_col  = scalar_rd_col;
        end else if (trans_busy) begin
            mux_rd_en   = trans_rd_en;
            mux_rd_slot = trans_rd_slot;
            mux_rd_row  = trans_rd_row;
            mux_rd_col  = trans_rd_col;
        end else begin
            mux_rd_en   = add_rd_en;
            mux_rd_slot = add_rd_slot;
            mux_rd_row  = add_rd_row;
            mux_rd_col  = add_rd_col;
        end
    end

    // =========================================================
    // 结果输出合并逻辑
    // =========================================================
    assign final_res_valid = add_res_valid | trans_res_valid | scalar_res_valid | matmul_res_valid;
    assign final_res_elem  = add_res_valid ? add_res_elem : 
                             trans_res_valid ? trans_res_elem :
                             scalar_res_valid ? scalar_res_elem :
                             matmul_res_elem;

    // =========================================================
    // 模块实例化
    // =========================================================

    uart_rx #(
        .CLK_FREQ(100_000_000),
        .UART_BPS(9600) // 确保串口助手也是 9600
    ) u_rx (
        .clk(clk),
        .rst_n(sys_rst_n),
        .uart_rxd(uart_rx),
        .uart_rx_done(rx_done),
        .uart_rx_data(rx_data)
    );

    debounce u_btn (
        .clk(clk),
        .rst(!sys_rst_n),
        .btn_in(btn_s0),
        .btn_out(btn_start_pulse)
    );

    // 返回按钮消抖
    debounce u_btn_back (
        .clk(clk),
        .rst(!sys_rst_n),
        .btn_in(btn_s2),
        .btn_out(btn_back_pulse)
    );

    fsm u_fsm (
        .clk(clk),
        .rst_n(sys_rst_n),
        .op_mode(op_mode_sel),
        .func_sel(func_sel),
        .uart_rx_done(rx_done),
        .uart_rx_data(rx_data),
        .btn_start(btn_start_pulse),
        .btn_back(btn_back_pulse),
        .store_wen(stor_wen),
        .store_m(stor_m),
        .store_n(stor_n),
        .store_elem_in(stor_elem_in),
        .store_elem_valid(stor_elem_valid),
        .storage_input_done(stor_input_done),
        .add_start(add_start),
        .add_done(add_done),
        .add_busy(add_busy),
        .trans_start(trans_start),
        .trans_done(trans_done),
        .trans_busy(trans_busy),
        .scalar_start(scalar_start),
        .scalar_done(scalar_done),
        .scalar_busy(scalar_busy),
        .matmul_start(matmul_start),
        .matmul_done(matmul_done),
        .matmul_busy(matmul_busy),
        .scalar_value(scalar_value),
        .current_slot(current_mat_slot),
        .led_status(fsm_led),
        .main_state_out(main_state_out)
    );

    matrix_storage #(
        .MAX_DIM(5), .MAX_STORE(2), .ELEM_WIDTH(8)
    ) u_storage (
        .clk(clk),
        .rst(!sys_rst_n),
        .wen(stor_wen),
        .m(stor_m),
        .n(stor_n),
        .elem_in(stor_elem_in),
        .elem_valid(stor_elem_valid),
        .input_done(stor_input_done),
        .rd_en(mux_rd_en),
        .rd_slot_idx(mux_rd_slot),
        .rd_row_idx(mux_rd_row),
        .rd_col_idx(mux_rd_col),
        .rd_elem(rd_elem),
        .rd_elem_valid(rd_elem_valid),
        .stored_m_flat(), .stored_n_flat(), .slot_valid()
    );

    mat_add #(
        .DIM_WIDTH(3), .DATA_WIDTH(8)
    ) u_add (
        .clk(clk),
        .rst_n(sys_rst_n),
        .start(add_start),
        .m_sel(stor_m[2:0]),
        .n_sel(stor_n[2:0]),
        .slot_a_sel(1'b0), .slot_a_valid(1'b1),
        .slot_b_sel(1'b1), .slot_b_valid(1'b1),
        .ready(), .busy(add_busy), .done(add_done), .error(),
        .rd_en(add_rd_en),
        .rd_slot_idx(add_rd_slot),
        .rd_row_idx(add_rd_row),
        .rd_col_idx(add_rd_col),
        .rd_elem(rd_elem),
        .rd_elem_valid(rd_elem_valid),
        .out_valid(add_res_valid),
        .out_elem(add_res_elem),
        .out_row_end(), .out_last(), .out_linear_idx()
    );

    // 转置模块实例化
    mat_transposition_v4 #(
        .DIM_WIDTH(3), .DATA_WIDTH(8)
    ) u_trans (
        .clk(clk),
        .rst_n(sys_rst_n),
        .start(trans_start),
        .m_sel(stor_m[2:0]),
        .n_sel(stor_n[2:0]),
        .slot_sel(current_mat_slot),  // 使用FSM追踪的槽位
        .slot_valid(1'b1),
        .ready(), .busy(trans_busy), .done(trans_done), .error(),
        .total_elements(),
        .rd_en(trans_rd_en),
        .rd_slot_idx(trans_rd_slot),
        .rd_row_idx(trans_rd_row),
        .rd_col_idx(trans_rd_col),
        .rd_elem(rd_elem),
        .rd_elem_valid(rd_elem_valid),
        .out_valid(trans_res_valid),
        .out_elem(trans_res_elem),
        .out_row_end(), .out_last(), .out_row_idx(), .out_col_idx(), .out_linear_idx()
    );

    // 标量乘模块实例化
    mat_scalar_mult #(
        .DIM_WIDTH(3), .DATA_WIDTH(8)
    ) u_scalar (
        .clk(clk),
        .rst_n(sys_rst_n),
        .start(scalar_start),
        .m_sel(stor_m[2:0]),
        .n_sel(stor_n[2:0]),
        .scalar(scalar_value),
        .slot_sel(current_mat_slot),
        .slot_valid(1'b1),
        .ready(), .busy(scalar_busy), .done(scalar_done), .error(),
        .rd_en(scalar_rd_en),
        .rd_slot_idx(scalar_rd_slot),
        .rd_row_idx(scalar_rd_row),
        .rd_col_idx(scalar_rd_col),
        .rd_elem(rd_elem),
        .rd_elem_valid(rd_elem_valid),
        .out_valid(scalar_res_valid),
        .out_elem(scalar_res_elem),
        .out_row_end(), .out_last(), .out_linear_idx()
    );

    // 矩阵乘模块实例化
    mat_mult #(
        .DIM_WIDTH(3), .DATA_WIDTH(8)
    ) u_matmul (
        .clk(clk),
        .rst_n(sys_rst_n),
        .start(matmul_start),
        .m_a(stor_m[2:0]),
        .n_a(stor_n[2:0]),
        .m_b(stor_m[2:0]),  // 假设两个矩阵维度信息都用stor_m和stor_n传递
        .n_b(stor_n[2:0]),
        .slot_a_sel(1'b0),
        .slot_a_valid(1'b1),
        .slot_b_sel(1'b1),
        .slot_b_valid(1'b1),
        .ready(), .busy(matmul_busy), .done(matmul_done), .error(),
        .rd_en(matmul_rd_en),
        .rd_slot_idx(matmul_rd_slot),
        .rd_row_idx(matmul_rd_row),
        .rd_col_idx(matmul_rd_col),
        .rd_elem(rd_elem),
        .rd_elem_valid(rd_elem_valid),
        .out_valid(matmul_res_valid),
        .out_elem(matmul_res_elem),
        .out_row_end(), .out_last(), .out_linear_idx()
    );

// --- 修复后的 FIFO 和 发送逻辑 ---
    reg [7:0] fifo_mem [0:31];
    reg [4:0] wr_ptr, rd_ptr;
    reg [5:0] count;
    
    assign fifo_empty = (count == 0);
    assign fifo_dout = fifo_mem[rd_ptr]; // 组合逻辑读出当前指针的数据

    always @(posedge clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
            count <= 0;
        end else begin
            // 写入
            if (final_res_valid && count < 32) begin
                fifo_mem[wr_ptr] <= final_res_elem;
                wr_ptr <= wr_ptr + 1;
            end
            
            // 读取 (指针移动)
            if (fifo_rd_en) begin
                rd_ptr <= rd_ptr + 1;
            end
            
            // 计数
            if (final_res_valid && !fifo_rd_en) count <= count + 1;
            else if (!final_res_valid && fifo_rd_en) count <= count - 1;
        end
    end

    // 状态机控制发送 (修复时序)
    reg [1:0] tx_state;
    always @(posedge clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            fifo_rd_en <= 0;
            tx_en_reg <= 0;
            tx_state <= 0;
        end else begin
            case(tx_state)
                0: begin // IDLE
                    // 默认拉低
                    tx_en_reg <= 0;
                    fifo_rd_en <= 0;
                    
                    if (!fifo_empty && !tx_busy) begin
                        // 【关键修复】
                        // 1. 立即触发串口发送当前 fifo_dout (对应当前的 rd_ptr)
                        tx_en_reg <= 1; 
                        // 2. 同时拉高读使能，让 rd_ptr 在下一个时钟沿加1，指向下一个数据
                        fifo_rd_en <= 1; 
                        
                        tx_state <= 1;
                    end
                end
                
                1: begin // LATCHED
                    // 握手信号只需保持一个周期，立刻拉低
                    tx_en_reg <= 0;
                    fifo_rd_en <= 0;
                    // 等待 UART 模块变忙 (UART 模块接收到 tx_en 后由于是同步的，busy 可能会滞后一拍)
                    tx_state <= 2;
                end
                
                2: begin // WAIT BUSY
                     // 简单延时或等待 busy 信号稳定
                     if (tx_busy) tx_state <= 3;
                     else tx_state <= 3; // 容错，直接去等待结束
                end
                
                3: begin // WAIT DONE
                    if (!tx_busy) tx_state <= 0; // 发送完毕，回到 IDLE 发送下一个
                end
            endcase
        end
    end

    uart_tx #(
        .CLK_FREQ(100_000_000),
        .UART_BPS(9600)
    ) u_tx (
        .clk(clk),
        .rst_n(sys_rst_n),
        .uart_tx_en(tx_en_reg),
        .uart_tx_data(fifo_dout),
        .uart_txd(uart_tx),
        .uart_tx_busy(tx_busy)
    );

    // 7段数码管显示模块
    seg7_display u_seg7 (
        .main_state(main_state_out),
        .func_sel(func_sel),
        .op_mode(op_mode_sel),
        .seg0(seg0),
        .seg1(seg1),
        .dig_sel(dig_sel)
    );

endmodule