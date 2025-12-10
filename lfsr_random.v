// LFSR 伪随机数生成器
// 用于矩阵生成和随机选择运算数
module lfsr_random (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       enable,      // 使能信号，每次enable产生一个新随机数
    output wire [3:0] random_out   // 输出0~15的随机数
);

    // 16-bit LFSR，使用多项式 x^16 + x^14 + x^13 + x^11 + 1
    reg [15:0] lfsr;
    
    wire feedback = lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10];
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lfsr <= 16'hACE1;  // 非零初始种子
        end else if (enable) begin
            lfsr <= {lfsr[14:0], feedback};
        end else begin
            // 即使不enable也持续移位，增加随机性
            lfsr <= {lfsr[14:0], feedback};
        end
    end
    
    // 输出低4位作为0~15的随机数
    assign random_out = lfsr[3:0];

endmodule
