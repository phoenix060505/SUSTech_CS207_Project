// 7段数码管显示模块
// 主菜单：熄灭
// 功能模式：显示 1/2/3/4
// 运算模式(func=11)下：显示运算类型 A/T/b/C
module seg7_display (
    input  wire [1:0] main_state,  // 主状态: 00=菜单, 01=输入, 10=生成, 11=展示/运算
    input  wire [1:0] func_sel,    // 功能选择
    input  wire [1:0] op_mode,     // 运算模式选择
    
    // DN0组段选信号 (A0~G0, DP0)
    output reg [7:0] seg0,         // {DP0, G0, F0, E0, D0, C0, B0, A0}
    
    // DN1组段选信号 (A1~G1, DP1) 
    output reg [7:0] seg1,         // {DP1, G1, F1, E1, D1, C1, B1, A1}
    
    // 片选信号 (使用DN0_K1，只显示一个数码管)
    output reg [3:0] dig_sel       // {DN0_K4, DN0_K3, DN0_K2, DN0_K1}
);

    // 7段数码管编码 (共阴极，高电平有效)
    // 段位排列：{DP, G, F, E, D, C, B, A}
    //     A
    //   F   B
    //     G
    //   E   C
    //     D   DP
    
    // 数字编码
    localparam SEG_1 = 8'b0000_0110;  // 显示 "1"  (B, C)
    localparam SEG_2 = 8'b0101_1011;  // 显示 "2"  (A, B, G, E, D)
    localparam SEG_3 = 8'b0100_1111;  // 显示 "3"  (A, B, G, C, D)
    localparam SEG_4 = 8'b0110_0110;  // 显示 "4"  (F, G, B, C)
    
    // 字母编码
    localparam SEG_T = 8'b0111_1000;  // 显示 "T"  (F, G, E, D) - 小写t形式
    localparam SEG_A = 8'b0111_0111;  // 显示 "A"  (A, B, C, E, F, G)
    localparam SEG_B = 8'b0111_1100;  // 显示 "b"  (C, D, E, F, G)
    localparam SEG_C = 8'b0011_1001;  // 显示 "C"  (A, D, E, F)
    localparam SEG_OFF = 8'b0000_0000; // 熄灭
    
    always @(*) begin
        // DN1组不使用，全部关闭
        seg1 = SEG_OFF;
        
        case (main_state)
            2'b00: begin
                // 主菜单状态 - 数码管熄灭
                dig_sel = 4'b0000;  // 关闭所有片选
                seg0 = SEG_OFF;
            end
            
            2'b01: begin
                // 矩阵输入模式 - 显示 "1"
                dig_sel = 4'b0001;
                seg0 = SEG_1;
            end
            
            2'b10: begin
                // 矩阵生成模式 - 显示 "2"
                dig_sel = 4'b0001;
                seg0 = SEG_2;
            end
            
            2'b11: begin
                // 展示/运算模式
                dig_sel = 4'b0001;
                if (func_sel == 2'b10) begin
                    // 展示模式 - 显示 "3"
                    seg0 = SEG_3;
                end else begin
                    // 运算模式 (func_sel == 2'b11) - 显示运算类型
                    case (op_mode)
                        2'b00: seg0 = SEG_A;    // 加法显示 "A"
                        2'b01: seg0 = SEG_T;    // 转置显示 "T"
                        2'b10: seg0 = SEG_B;    // 标量乘显示 "b"
                        2'b11: seg0 = SEG_C;    // 矩阵乘显示 "C"
                        default: seg0 = SEG_OFF;
                    endcase
                end
            end
            
            default: begin
                dig_sel = 4'b0000;
                seg0 = SEG_OFF;
            end
        endcase
    end

endmodule
