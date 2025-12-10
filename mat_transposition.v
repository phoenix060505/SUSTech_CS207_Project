module mat_transposition_v4 #(
    parameter DIM_WIDTH  = 3,
    parameter DATA_WIDTH = 8
)(
    input  wire                   clk, rst_n, start,
    input  wire [DIM_WIDTH-1:0]   m_sel, n_sel,
    input  wire                   slot_sel, slot_valid,
    output reg                    ready, busy, done, error,
    output reg [2*DIM_WIDTH-1:0]  total_elements, 
    output reg                    rd_en,
    output wire                   rd_slot_idx,
    output wire [DIM_WIDTH-1:0]   rd_row_idx, rd_col_idx,
    input  wire [DATA_WIDTH-1:0]  rd_elem, 
    input  wire                   rd_elem_valid,
    output reg                    out_valid,
    output reg  [DATA_WIDTH-1:0]  out_elem,
    output reg                    out_row_end, out_last,
    output reg [DIM_WIDTH-1:0]    out_row_idx, out_col_idx,
    output reg [2*DIM_WIDTH-1:0]  out_linear_idx
);

    localparam S_IDLE       = 3'd0;
    localparam S_CHECK      = 3'd1;
    localparam S_PRE        = 3'd2; // 新增 PRE
    localparam S_WAIT       = 3'd3; // Stream/Wait
    localparam S_DONE       = 3'd4;
    localparam S_ERROR      = 3'd5;

    reg [2:0] state, next_state; 
    reg [DIM_WIDTH-1:0] m_latched, n_latched;
    reg slot_latched, slot_valid_latched;
    reg [DIM_WIDTH-1:0] trans_row_cnt, trans_col_cnt; // 控制转置后的输出顺序

    assign rd_slot_idx = slot_latched;
    // 转置逻辑：当我们要输出转置后的(i,j)时，实际上要去读原矩阵的(j,i)
    // 这里 trans_row_cnt 是输出矩阵的行，trans_col_cnt 是输出矩阵的列
    assign rd_row_idx  = trans_col_cnt; 
    assign rd_col_idx  = trans_row_cnt;

    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE:  if (start && ready) next_state = S_CHECK;
            S_CHECK: if (slot_valid_latched && m_latched != 0 && n_latched != 0) next_state = S_PRE;
                     else next_state = S_ERROR;
                     
            S_PRE:   next_state = S_WAIT;
            S_WAIT:  if (rd_elem_valid) begin
                        if (trans_col_cnt == m_latched - 1 && trans_row_cnt == n_latched - 1) next_state = S_DONE;
                        else next_state = S_PRE;
                     end else next_state = S_WAIT;
            
            S_DONE:  next_state = S_IDLE;
            S_ERROR: next_state = S_IDLE;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            m_latched <= 0; n_latched <= 0; slot_latched <= 0; slot_valid_latched <= 0;
            ready <= 1; busy <= 0; done <= 0; error <= 0; total_elements <= 0;
            rd_en <= 0; out_valid <= 0; out_elem <= 0; out_row_end <= 0; out_last <= 0;
            out_row_idx <= 0; out_col_idx <= 0; out_linear_idx <= 0;
            trans_row_cnt <= 0; trans_col_cnt <= 0;
        end else begin
            state <= next_state;
            rd_en <= 0; // Default off
            out_valid <= 0; out_row_end <= 0; out_last <= 0; done <= 0; error <= 0;

            case (state)
                S_IDLE: begin
                    ready <= 1; busy <= 0;
                    if (start && ready) begin
                        busy <= 1; ready <= 0;
                        m_latched <= m_sel; n_latched <= n_sel;
                        slot_latched <= slot_sel; slot_valid_latched <= slot_valid;
                        total_elements <= m_sel * n_sel;
                        trans_row_cnt <= 0; trans_col_cnt <= 0; out_linear_idx <= 0;
                        out_row_idx <= 0; out_col_idx <= 0;
                    end
                end
                
                S_PRE: begin
                    rd_en <= 0; // Setup addr
                    out_row_idx <= trans_row_cnt; // Metadata output
                    out_col_idx <= trans_col_cnt;
                end
                
                S_WAIT: begin
                    rd_en <= 1; // Read
                    if (rd_elem_valid) begin
                        out_valid <= 1;
                        out_elem <= rd_elem;
                        if (trans_col_cnt == m_latched - 1) out_row_end <= 1;
                        if (trans_col_cnt == m_latched - 1 && trans_row_cnt == n_latched - 1) out_last <= 1;
                        out_linear_idx <= out_linear_idx + 1;
                        
                        if (trans_col_cnt == m_latched - 1) begin
                            trans_col_cnt <= 0;
                            if (trans_row_cnt < n_latched - 1) trans_row_cnt <= trans_row_cnt + 1;
                        end else begin
                            trans_col_cnt <= trans_col_cnt + 1;
                        end
                    end
                end
                
                S_DONE: begin done <= 1; busy <= 0; end
                S_ERROR: begin error <= 1; busy <= 0; end
            endcase
        end
    end
endmodule