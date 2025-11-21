// matrix_storage.v
// 负责矩阵数据的存储、覆盖与输入后处理（补零/忽略多余元素）
// 随机矩阵生成逻辑由外部 random_generator 模块提供

module matrix_storage #(
    parameter MAX_DIM    = 5,
    parameter MAX_STORE  = 2,
    parameter ELEM_WIDTH = 8
) (
    input  wire                     clk,
    input  wire                     rst,          // 高有效复位

    // 写控制
    input  wire                     wen,          // 写开始：锁存新矩阵规格并清空槽
    input  wire [3:0]               m,            // 行数 1~MAX_DIM
    input  wire [3:0]               n,            // 列数 1~MAX_DIM

    // 元素输入
    input  wire [ELEM_WIDTH-1:0]    elem_in,
    input  wire                     elem_valid,   // 每个元素一个周期脉冲

    // 状态输出
    output reg  [ELEM_WIDTH-1:0]    matrix_store [0:MAX_STORE-1][0:MAX_DIM-1][0:MAX_DIM-1],
    output reg  [3:0]               stored_m     [0:MAX_STORE-1],
    output reg  [3:0]               stored_n     [0:MAX_STORE-1],
    output reg  [MAX_STORE-1:0]     slot_valid,
    output reg                      input_done    // 当前矩阵写满 m*n 时输出一个周期脉冲
);

    localparam SLOT_BITS = (MAX_STORE <= 1) ? 1 : $clog2(MAX_STORE);

    // 写入状态
    reg [10:0]                elem_cnt;
    reg [SLOT_BITS-1:0]       fifo_ptr;      // FIFO 覆盖指针
    reg [SLOT_BITS-1:0]       active_slot;
    reg [3:0]                 active_m;
    reg [3:0]                 active_n;
    reg                       active_valid;

    integer s, i, j;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            elem_cnt    <= 0;
            fifo_ptr    <= 0;
            active_slot <= 0;
            active_m    <= 0;
            active_n    <= 0;
            active_valid<= 1'b0;
            slot_valid  <= {MAX_STORE{1'b0}};
            input_done  <= 1'b0;

            for (s = 0; s < MAX_STORE; s = s + 1) begin
                stored_m[s] <= 0;
                stored_n[s] <= 0;
                for (i = 0; i < MAX_DIM; i = i + 1) begin
                    for (j = 0; j < MAX_DIM; j = j + 1) begin
                        matrix_store[s][i][j] <= {ELEM_WIDTH{1'b0}};
                    end
                end
            end
        end else begin
            input_done <= 1'b0;

            if (wen) begin
                active_slot  <= fifo_ptr;
                active_m     <= m;
                active_n     <= n;
                active_valid <= 1'b1;

                stored_m[fifo_ptr]   <= m;
                stored_n[fifo_ptr]   <= n;
                slot_valid[fifo_ptr] <= 1'b1;

                for (i = 0; i < MAX_DIM; i = i + 1) begin
                    for (j = 0; j < MAX_DIM; j = j + 1) begin
                        matrix_store[fifo_ptr][i][j] <= {ELEM_WIDTH{1'b0}};
                    end
                end

                elem_cnt <= 0;

                if (fifo_ptr == MAX_STORE-1)
                    fifo_ptr <= 0;
                else
                    fifo_ptr <= fifo_ptr + 1'b1;
            end

            if (active_valid && elem_valid && elem_cnt < active_m * active_n) begin
                matrix_store[active_slot][elem_cnt / active_n][elem_cnt % active_n]
                    <= elem_in;
                elem_cnt <= elem_cnt + 1'b1;

                if (elem_cnt + 1 == active_m * active_n) begin
                    input_done <= 1'b1;
                    active_valid <= 1'b0;
                end
            end
        end
    end

endmodule
