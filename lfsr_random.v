// LFSR 伪随机数生成器 - 范围限制为 0~9
module lfsr_random (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       enable,      // 保持端口兼容
    output wire [3:0] random_out   // 输出 0~9 的随机数
);
    // 16-bit LFSR，使用多项式 x^16 + x^14 + x^13 + x^11 + 1
    reg [15:0] lfsr;
    wire feedback = lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lfsr <= 16'hACE1; // 非零种子
        end else begin
            // 无论 enable 是否有效都持续移位，利用 100MHz 时钟的高频特性增加随机性
            lfsr <= {lfsr[14:0], feedback};
        end
    end
    
    // 对 16 位状态空间 (1~65535) 取模 10
    assign random_out = lfsr % 10;

endmodule