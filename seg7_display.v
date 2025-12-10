// 7段数码管显示模块 - 支持8个数码管
// DN0组 (右侧4个): 显示功能/运算类型
// DN1组 (左侧4个): K3/K4用于显示倒计时 (十位和个位)
module seg7_display (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [1:0]  main_state,     // 主状态
    input  wire [1:0]  func_sel,       // 功能选择
    input  wire [1:0]  op_mode,        // 运算模式
    input  wire [4:0]  countdown_val,  // 倒计时值 (0~31)
    input  wire        countdown_active,// 倒计时激活
    
    // DN0组段选信号
    output reg [7:0] seg0,
    // DN1组段选信号
    output reg [7:0] seg1,
    // 片选信号 - 需要动态扫描
    output reg [7:0] dig_sel           // {DN1_K4, DN1_K3, DN1_K2, DN1_K1, DN0_K4, DN0_K3, DN0_K2, DN0_K1}
);

    // 7段数码管编码 (共阴极，高电平有效)
    // 段位排列：{DP, G, F, E, D, C, B, A}
    localparam SEG_0 = 8'b0011_1111;
    localparam SEG_1 = 8'b0000_0110;
    localparam SEG_2 = 8'b0101_1011;
    localparam SEG_3 = 8'b0100_1111;
    localparam SEG_4 = 8'b0110_0110;
    localparam SEG_5 = 8'b0110_1101;
    localparam SEG_6 = 8'b0111_1101;
    localparam SEG_7 = 8'b0000_0111;
    localparam SEG_8 = 8'b0111_1111;
    localparam SEG_9 = 8'b0110_1111;
    
    // 字母编码
    localparam SEG_A = 8'b0111_0111;
    localparam SEG_T = 8'b0111_1000;
    localparam SEG_B = 8'b0111_1100;
    localparam SEG_C = 8'b0011_1001;
    localparam SEG_OFF = 8'b0000_0000;
    
    // 数字查找表
    function [7:0] digit_to_seg;
        input [3:0] digit;
        begin
            case (digit)
                4'd0: digit_to_seg = SEG_0;
                4'd1: digit_to_seg = SEG_1;
                4'd2: digit_to_seg = SEG_2;
                4'd3: digit_to_seg = SEG_3;
                4'd4: digit_to_seg = SEG_4;
                4'd5: digit_to_seg = SEG_5;
                4'd6: digit_to_seg = SEG_6;
                4'd7: digit_to_seg = SEG_7;
                4'd8: digit_to_seg = SEG_8;
                4'd9: digit_to_seg = SEG_9;
                default: digit_to_seg = SEG_OFF;
            endcase
        end
    endfunction
    
    // 扫描计数器 (用于动态扫描8个数码管)
    reg [16:0] scan_cnt;
    reg [2:0]  scan_idx;
    
    // 倒计时十位和个位
    wire [3:0] countdown_tens = countdown_val / 10;
    wire [3:0] countdown_ones = countdown_val % 10;
    
    // DN0组要显示的内容
    reg [7:0] dn0_display;
    
    always @(*) begin
        case (main_state)
            2'b00: dn0_display = SEG_OFF;  // 主菜单
            2'b01: dn0_display = SEG_1;    // 输入模式
            2'b10: dn0_display = SEG_2;    // 生成模式
            2'b11: begin
                if (func_sel == 2'b10) begin
                    dn0_display = SEG_3;   // 展示模式
                end else begin
                    // 运算模式 - 显示运算类型
                    case (op_mode)
                        2'b00: dn0_display = SEG_A;
                        2'b01: dn0_display = SEG_T;
                        2'b10: dn0_display = SEG_B;
                        2'b11: dn0_display = SEG_C;
                        default: dn0_display = SEG_OFF;
                    endcase
                end
            end
            default: dn0_display = SEG_OFF;
        endcase
    end
    
    // 扫描计数器
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scan_cnt <= 0;
            scan_idx <= 0;
        end else begin
            if (scan_cnt >= 100000 - 1) begin  // 1kHz扫描频率
                scan_cnt <= 0;
                scan_idx <= scan_idx + 1;
            end else begin
                scan_cnt <= scan_cnt + 1;
            end
        end
    end
    
    // 动态扫描输出
    always @(*) begin
        // 默认关闭
        dig_sel = 8'b0000_0000;
        seg0 = SEG_OFF;
        seg1 = SEG_OFF;
        
        case (scan_idx)
            3'd0: begin  // DN0_K1 - 显示功能/运算类型
                if (main_state != 2'b00) begin
                    dig_sel = 8'b0000_0001;
                    seg0 = dn0_display;
                end
            end
            
            3'd1: begin  // DN0_K2 - 不使用
                dig_sel = 8'b0000_0000;
            end
            
            3'd2: begin  // DN0_K3 - 不使用
                dig_sel = 8'b0000_0000;
            end
            
            3'd3: begin  // DN0_K4 - 不使用
                dig_sel = 8'b0000_0000;
            end
            
            3'd4: begin  // DN1_K1 - 不使用
                dig_sel = 8'b0000_0000;
            end
            
            3'd5: begin  // DN1_K2 - 不使用
                dig_sel = 8'b0000_0000;
            end
            
            3'd6: begin  // DN1_K3 - 倒计时个位
                if (countdown_active) begin
                    dig_sel = 8'b0100_0000;
                    seg1 = digit_to_seg(countdown_ones);
                end
            end
            
            3'd7: begin  // DN1_K4 - 倒计时十位
                if (countdown_active && countdown_tens > 0) begin
                    dig_sel = 8'b1000_0000;
                    seg1 = digit_to_seg(countdown_tens);
                end
            end
        endcase
    end

endmodule

