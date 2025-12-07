`timescale 1ns / 1ps

module matrix_storage #(
    parameter MAX_DIM = 5,
    parameter MAX_STORE = 2,
    parameter ELEM_WIDTH = 8
) (
    input wire clk,
    input wire rst,
    
    // Write Interface
    input wire wen,                  
    input wire flush,                // Added for manual finish (padding)
    input wire [1:0] mode,           
    input wire [7:0] data_in,        
    
    // Read Interface (ID based)
    input wire [7:0] read_id_a, 
    input wire [7:0] read_id_b, 
    
    // Output to computation modules
    output reg [199:0] mat_a_out, 
    output reg [199:0] mat_b_out,
    
    // Output matrix dimensions
    output wire [3:0] dim_a_m, output wire [3:0] dim_a_n,
    output wire [3:0] dim_b_m, output wire [3:0] dim_b_n,
    
    // Status signals
    output reg input_done,
    output reg error_flag,
    
    // Display Interface
    input wire display_start,        
    input wire tx_busy,              // Added for ASCII conversion handshake
    input wire filter_en,            // Filter enable
    input wire [3:0] filter_m,       // Filter Rows
    input wire [3:0] filter_n,       // Filter Cols
    output wire tx_start,            // Changed to wire for Mux
    output wire [7:0] tx_data,       // Changed to wire for Mux
    output reg display_done          
);

    // Internal regs for dimensions
    reg [3:0] dim_a_m_int, dim_a_n_int;
    reg [3:0] dim_b_m_int, dim_b_n_int;
    
    assign dim_a_m = dim_a_m_int;
    assign dim_a_n = dim_a_n_int;
    assign dim_b_m = dim_b_m_int;
    assign dim_b_n = dim_b_n_int;

    // --- ASCII Conversion & UART Mux ---
    reg manual_tx_start;
    reg [7:0] manual_tx_data;
    reg use_b2a;
    
    wire b2a_tx_start;
    wire [7:0] b2a_tx_data;
    wire b2a_done;
    reg b2a_start;
    reg [15:0] b2a_value;
    
    assign tx_start = (use_b2a) ? b2a_tx_start : manual_tx_start;
    assign tx_data = (use_b2a) ? b2a_tx_data : manual_tx_data;

    bin_to_ascii b2a_inst (
        .clk(clk), .rst(rst),
        .start(b2a_start),
        .value(b2a_value),
        .is_signed(1'b0),
        .tx_start(b2a_tx_start),
        .tx_data(b2a_tx_data),
        .tx_busy(tx_busy),
        .done(b2a_done)
    );

    //==============================================================
    // 1. Optimized Storage Structure Definition (Modified for Requirements)
    //==============================================================
    // Requirement: Max x matrices per dimension type (e.g., 2x3).
    // If full, overwrite oldest.
    // We need to track count per dimension type.
    // Since dimension combinations are limited (5x5 = 25 types), we can use a lookup or just search.
    // To simplify, we keep the linear pool but implement "Find Slot by Dimension" logic.
    
    parameter MAX_MATRICES = 16;
    parameter MAX_PER_TYPE = 2; // Default x=2
    
    reg [7:0] storage_pool [0:15][0:24]; 
    reg [7:0] matrix_ids [0:15];         
    reg [3:0] matrix_rows [0:15];        
    reg [3:0] matrix_cols [0:15];        
    reg       slot_used [0:15];          
    reg [1:0] type_age [0:15];           // Age for LRU replacement (0=Newest, 1=Oldest)
    reg [7:0] next_matrix_id;
    reg [3:0] global_replace_ptr;

    // Helper function to find slot for new matrix (m, n)
    // Returns: Free slot OR Slot to overwrite
    function [3:0] find_write_slot;
        input [3:0] m, n;
        integer i;
        reg [3:0] count;
        reg [3:0] oldest_slot;
        begin
            find_write_slot = 4'hF;
            count = 0;
            oldest_slot = 4'hF;
            
            // 1. Count existing matrices of same type and find oldest
            for(i=0; i<MAX_MATRICES; i=i+1) begin
                if (slot_used[i] && matrix_rows[i] == m && matrix_cols[i] == n) begin
                    count = count + 1;
                    if (type_age[i] == MAX_PER_TYPE - 1) oldest_slot = i[3:0];
                end
            end
            
            // 2. Decision
            if (count < MAX_PER_TYPE) begin
                // Find any free slot
                for(i=0; i<MAX_MATRICES; i=i+1) begin
                    if (!slot_used[i] && find_write_slot == 4'hF) find_write_slot = i[3:0];
                end
            end else begin
                // Overwrite oldest
                find_write_slot = oldest_slot;
            end
        end
    endfunction

    // Update ages after writing to target_slot
    task update_ages;
        input [3:0] target_slot;
        input [3:0] m, n;
        integer i;
        begin
            for(i=0; i<MAX_MATRICES; i=i+1) begin
                if (slot_used[i] && matrix_rows[i] == m && matrix_cols[i] == n) begin
                    if (i == target_slot) type_age[i] <= 0; // Newest
                    else if (type_age[i] < MAX_PER_TYPE - 1) type_age[i] <= type_age[i] + 1;
                end
            end
        end
    endtask

    //==============================================================
    // 2. Random Number Generator (LFSR)
    //==============================================================
    wire [7:0] rand_val;
    wire [7:0] rand_raw;
    reg lfsr_en;
    lfsr_prng lfsr_inst (
        .clk(clk), .rst(rst), .enable(lfsr_en), .rand_out(rand_val), .rand_raw(rand_raw)
    );

    //==============================================================
    // 3. Input/Generation State Machine
    //==============================================================
    localparam S_IDLE = 0;
    localparam S_IN_M = 1, S_IN_N = 2, S_IN_DATA = 3;
    localparam S_GEN_M = 4, S_GEN_N = 5, S_GEN_COUNT = 6, S_GEN_EXEC = 7;
    localparam S_DONE = 8, S_ERR = 9;
    localparam S_CALC_SLOT_IN = 10, S_CALC_SLOT_GEN = 11;

    reg [3:0] cur_m, cur_n;
    reg [1:0] cur_count_target, cur_count_done;
    reg [3:0] state;
    reg [4:0] elem_cnt;
    reg [7:0] temp_buf [0:24];

    integer i, j, k;

    reg [3:0] target_slot; // Temp storage for calculated slot
    reg [3:0] gen_slot;    // State register for generation slot
    reg [3:0] active_slot; // Registered slot to break timing path

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            input_done <= 0;
            error_flag <= 0;
            elem_cnt <= 0;
            lfsr_en <= 0;
            next_matrix_id <= 8'h01;
            global_replace_ptr <= 4'd0;
            for(i=0; i<MAX_MATRICES; i=i+1) begin
                slot_used[i] <= 0;
                matrix_ids[i] <= 0;
                matrix_rows[i] <= 0;
                matrix_cols[i] <= 0;
                type_age[i] <= 0;
            end
        end else begin
            lfsr_en <= 0;
            input_done <= 0;
            
            case (state)
                S_IDLE: begin
                    error_flag <= 0;
                    elem_cnt <= 0;
                    if (mode == 2'b00 && wen) begin // Input Mode
                        if (data_in >= 1 && data_in <= 5) begin
                            cur_m <= data_in;
                            state <= S_IN_N;
                        end else begin
                            error_flag <= 1;
                            state <= S_ERR;
                        end
                    end else if (mode == 2'b01 && wen) begin // Gen Mode
                        if (data_in >= 1 && data_in <= 5) begin
                            cur_m <= data_in;
                            state <= S_GEN_N;
                        end else begin
                            error_flag <= 1;
                            state <= S_ERR;
                        end
                    end
                end

                S_IN_N: begin
                    if (wen) begin
                        if (data_in >= 1 && data_in <= 5) begin
                            cur_n <= data_in;
                            state <= S_CALC_SLOT_IN;
                        end else begin
                            error_flag <= 1;
                            state <= S_ERR;
                        end
                    end
                end

                S_CALC_SLOT_IN: begin
                    if (find_write_slot(cur_m, cur_n) == 4'hF) begin
                        active_slot <= global_replace_ptr;
                        global_replace_ptr <= global_replace_ptr + 1'b1;
                    end else begin
                        active_slot <= find_write_slot(cur_m, cur_n);
                    end
                    state <= S_IN_DATA;
                    elem_cnt <= 0;
                end

                S_IN_DATA: begin
                    if (flush) begin
                        // Pad with 0 and save
                        target_slot = active_slot;
                        for (k=0; k<25; k=k+1) begin
                            if (k < elem_cnt) begin
                                storage_pool[target_slot][k] <= temp_buf[k];
                            end else if (k < cur_m * cur_n) begin
                                storage_pool[target_slot][k] <= 0; // Pad
                            end
                        end
                        lfsr_en <= 1; 
                        matrix_ids[target_slot] <= next_matrix_id;
                        matrix_rows[target_slot] <= cur_m;
                        matrix_cols[target_slot] <= cur_n;
                        slot_used[target_slot] <= 1;
                        update_ages(target_slot, cur_m, cur_n);
                        next_matrix_id <= (next_matrix_id == 8'hFF) ? 8'h01 : next_matrix_id + 1'b1;
                        state <= S_DONE;
                    end else if (wen) begin
                        if (data_in > 9) begin 
                            error_flag <= 1;
                            state <= S_ERR;
                        end else begin
                            temp_buf[elem_cnt] <= data_in;
                            if (elem_cnt == (cur_m * cur_n) - 1) begin
                                // Find slot based on requirements
                                target_slot = active_slot;
                                
                                if (target_slot != 4'hF) begin
                                    // Write data
                                    for (k=0; k<25; k=k+1) begin
                                        if (k < elem_cnt) begin
                                            storage_pool[target_slot][k] <= temp_buf[k];
                                        end else if (k == elem_cnt) begin
                                            storage_pool[target_slot][k] <= data_in;
                                        end
                                    end
                                    lfsr_en <= 1; 
                                    matrix_ids[target_slot] <= next_matrix_id;
                                    matrix_rows[target_slot] <= cur_m;
                                    matrix_cols[target_slot] <= cur_n;
                                    slot_used[target_slot] <= 1;
                                    update_ages(target_slot, cur_m, cur_n);
                                    next_matrix_id <= (next_matrix_id == 8'hFF) ? 8'h01 : next_matrix_id + 1'b1;
                                end
                                
                                state <= S_DONE;
                            end else begin
                                elem_cnt <= elem_cnt + 1;
                            end
                        end
                    end
                end

                S_GEN_N: begin
                    if (wen) begin
                        if (data_in >= 1 && data_in <= 5) begin
                            cur_n <= data_in;
                            state <= S_GEN_COUNT;
                        end else begin
                            error_flag <= 1;
                            state <= S_ERR;
                        end
                    end
                end

                S_GEN_COUNT: begin
                    if (wen) begin
                        if (data_in >= 1 && data_in <= MAX_STORE) begin
                            cur_count_target <= data_in[1:0];
                            cur_count_done <= 0;
                            state <= S_CALC_SLOT_GEN;
                        end else begin
                            error_flag <= 1;
                            state <= S_ERR;
                        end
                    end
                end

                S_CALC_SLOT_GEN: begin
                    if (find_write_slot(cur_m, cur_n) == 4'hF) begin
                        active_slot <= global_replace_ptr;
                        global_replace_ptr <= global_replace_ptr + 1'b1;
                    end else begin
                        active_slot <= find_write_slot(cur_m, cur_n);
                    end
                    state <= S_GEN_EXEC;
                    elem_cnt <= 0;
                end

                S_GEN_EXEC: begin
                    lfsr_en <= 1;
                    
                    target_slot = active_slot;
                    
                    storage_pool[target_slot][elem_cnt] <= rand_val;
                    if (elem_cnt == (cur_m * cur_n) - 1) begin
                        matrix_ids[target_slot] <= next_matrix_id;
                        matrix_rows[target_slot] <= cur_m;
                        matrix_cols[target_slot] <= cur_n;
                        slot_used[target_slot] <= 1;
                        update_ages(target_slot, cur_m, cur_n);
                        next_matrix_id <= (next_matrix_id == 8'hFF) ? 8'h01 : next_matrix_id + 1'b1;
                        if (cur_count_done + 1 == cur_count_target) begin
                            state <= S_DONE;
                        end else begin
                            cur_count_done <= cur_count_done + 1;
                            state <= S_CALC_SLOT_GEN;
                        end
                    end else begin
                        elem_cnt <= elem_cnt + 1;
                    end
                end

                S_DONE: begin
                    input_done <= 1;
                    if (mode == 2'b10 || mode == 2'b11) state <= S_IDLE;
                end

                S_ERR: begin
                    if (mode == 2'b10 || mode == 2'b11) state <= S_IDLE;
                end
            endcase
        end
    end

    //==============================================================
    // 4. Optimized Read Logic (Registered Output for Timing)
    //==============================================================
    integer search_i;
    
    // Pipeline Stage 1: Search Logic (Combinatorial + Register)
    reg [3:0] next_slot_a, next_slot_b;
    reg next_found_a, next_found_b;
    reg [3:0] p1_slot_a, p1_slot_b;
    reg p1_found_a, p1_found_b;

    always @(*) begin
        next_found_a = 0; next_slot_a = 0;
        for(search_i = 0; search_i < MAX_MATRICES; search_i = search_i + 1) begin
            if (slot_used[search_i] && matrix_ids[search_i] == read_id_a) begin
                next_found_a = 1;
                next_slot_a = search_i[3:0];
            end
        end
        
        next_found_b = 0; next_slot_b = 0;
        for(search_i = 0; search_i < MAX_MATRICES; search_i = search_i + 1) begin
            if (slot_used[search_i] && matrix_ids[search_i] == read_id_b) begin
                next_found_b = 1;
                next_slot_b = search_i[3:0];
            end
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            p1_slot_a <= 0; p1_found_a <= 0;
            p1_slot_b <= 0; p1_found_b <= 0;
        end else begin
            p1_slot_a <= next_slot_a;
            p1_found_a <= next_found_a;
            p1_slot_b <= next_slot_b;
            p1_found_b <= next_found_b;
        end
    end

    // Pipeline Stage 2: Read Data (using p1_slot_a/b)
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            mat_a_out <= 0;
            dim_a_m_int <= 0;
            dim_a_n_int <= 0;
            mat_b_out <= 0;
            dim_b_m_int <= 0;
            dim_b_n_int <= 0;
        end else begin
            // Output Matrix A
            if (p1_found_a) begin
                dim_a_m_int <= matrix_rows[p1_slot_a];
                dim_a_n_int <= matrix_cols[p1_slot_a];
                // Pack matrix data into 200-bit output
                mat_a_out <= {storage_pool[p1_slot_a][24], storage_pool[p1_slot_a][23],
                            storage_pool[p1_slot_a][22], storage_pool[p1_slot_a][21],
                            storage_pool[p1_slot_a][20], storage_pool[p1_slot_a][19],
                            storage_pool[p1_slot_a][18], storage_pool[p1_slot_a][17],
                            storage_pool[p1_slot_a][16], storage_pool[p1_slot_a][15],
                            storage_pool[p1_slot_a][14], storage_pool[p1_slot_a][13],
                            storage_pool[p1_slot_a][12], storage_pool[p1_slot_a][11],
                            storage_pool[p1_slot_a][10], storage_pool[p1_slot_a][9],
                            storage_pool[p1_slot_a][8], storage_pool[p1_slot_a][7],
                            storage_pool[p1_slot_a][6], storage_pool[p1_slot_a][5],
                            storage_pool[p1_slot_a][4], storage_pool[p1_slot_a][3],
                            storage_pool[p1_slot_a][2], storage_pool[p1_slot_a][1],
                            storage_pool[p1_slot_a][0]};
            end else begin
                dim_a_m_int <= 0; dim_a_n_int <= 0;
                mat_a_out <= 0;
            end

            // Output Matrix B
            if (p1_found_b) begin
                dim_b_m_int <= matrix_rows[p1_slot_b];
                dim_b_n_int <= matrix_cols[p1_slot_b];
                mat_b_out <= {storage_pool[p1_slot_b][24], storage_pool[p1_slot_b][23],
                            storage_pool[p1_slot_b][22], storage_pool[p1_slot_b][21],
                            storage_pool[p1_slot_b][20], storage_pool[p1_slot_b][19],
                            storage_pool[p1_slot_b][18], storage_pool[p1_slot_b][17],
                            storage_pool[p1_slot_b][16], storage_pool[p1_slot_b][15],
                            storage_pool[p1_slot_b][14], storage_pool[p1_slot_b][13],
                            storage_pool[p1_slot_b][12], storage_pool[p1_slot_b][11],
                            storage_pool[p1_slot_b][10], storage_pool[p1_slot_b][9],
                            storage_pool[p1_slot_b][8], storage_pool[p1_slot_b][7],
                            storage_pool[p1_slot_b][6], storage_pool[p1_slot_b][5],
                            storage_pool[p1_slot_b][4], storage_pool[p1_slot_b][3],
                            storage_pool[p1_slot_b][2], storage_pool[p1_slot_b][1],
                            storage_pool[p1_slot_b][0]};
            end else begin
                dim_b_m_int <= 0; dim_b_n_int <= 0;
                mat_b_out <= 0;
            end
        end
    end

    //==============================================================
    // 5. Optimized Display Logic (ASCII)
    //==============================================================
    localparam D_IDLE = 0, D_CHECK = 1, D_NEXT = 2, D_DONE_STATE = 3;
    localparam D_SEQ_ID = 4, D_SEQ_SPACE1 = 5, D_SEQ_M = 6, D_SEQ_SPACE2 = 7, D_SEQ_N = 8, D_SEQ_NL1 = 9;
    localparam D_SEQ_DATA = 10, D_SEQ_SPACE3 = 11, D_SEQ_CHECK_LOOP = 12, D_SEQ_NL2 = 13;
    localparam D_DO_B2A = 14, D_WAIT_B2A = 15, D_DO_MANUAL = 16, D_WAIT_MANUAL = 17;

    reg [4:0] d_state;
    reg [4:0] next_seq_state;
    reg [3:0] current_slot;
    reg [4:0] de;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            d_state <= D_IDLE;
            display_done <= 0;
            manual_tx_start <= 0;
            manual_tx_data <= 0;
            use_b2a <= 0;
            b2a_start <= 0;
            b2a_value <= 0;
            current_slot <= 0; de <= 0;
            next_seq_state <= D_IDLE;
        end else begin
            // Default pulses
            manual_tx_start <= 0;
            b2a_start <= 0;
            
            case (d_state)
                D_IDLE: begin
                    display_done <= 0;
                    if (display_start) begin
                        d_state <= D_CHECK;
                        current_slot <= 0;
                    end
                end

                D_CHECK: begin
                    if (current_slot < MAX_MATRICES) begin
                        if (slot_used[current_slot]) begin
                            // Check filter if enabled
                            if (!filter_en || (matrix_rows[current_slot] == filter_m && matrix_cols[current_slot] == filter_n)) begin
                                d_state <= D_SEQ_ID;
                                de <= 0;
                            end else begin
                                current_slot <= current_slot + 1;
                            end
                        end else begin
                            current_slot <= current_slot + 1;
                        end
                    end else begin
                        d_state <= D_DONE_STATE;
                    end
                end

                // Sequence: ID -> Space -> M -> Space -> N -> Newline
                D_SEQ_ID: begin
                    b2a_value <= matrix_ids[current_slot];
                    d_state <= D_DO_B2A;
                    next_seq_state <= D_SEQ_SPACE1;
                end
                D_SEQ_SPACE1: begin
                    manual_tx_data <= 8'h20; // Space
                    d_state <= D_DO_MANUAL;
                    next_seq_state <= D_SEQ_M;
                end
                D_SEQ_M: begin
                    b2a_value <= matrix_rows[current_slot];
                    d_state <= D_DO_B2A;
                    next_seq_state <= D_SEQ_SPACE2;
                end
                D_SEQ_SPACE2: begin
                    manual_tx_data <= 8'h20; // Space
                    d_state <= D_DO_MANUAL;
                    next_seq_state <= D_SEQ_N;
                end
                D_SEQ_N: begin
                    b2a_value <= matrix_cols[current_slot];
                    d_state <= D_DO_B2A;
                    next_seq_state <= D_SEQ_NL1;
                end
                D_SEQ_NL1: begin
                    manual_tx_data <= 8'h0A; // Newline
                    d_state <= D_DO_MANUAL;
                    next_seq_state <= D_SEQ_DATA;
                end

                // Data Loop
                D_SEQ_DATA: begin
                    b2a_value <= storage_pool[current_slot][de];
                    d_state <= D_DO_B2A;
                    next_seq_state <= D_SEQ_SPACE3;
                end
                D_SEQ_SPACE3: begin
                    manual_tx_data <= 8'h20; // Space
                    d_state <= D_DO_MANUAL;
                    next_seq_state <= D_SEQ_CHECK_LOOP;
                end
                D_SEQ_CHECK_LOOP: begin
                    if (de == (matrix_rows[current_slot] * matrix_cols[current_slot]) - 1) begin
                        d_state <= D_SEQ_NL2;
                    end else begin
                        de <= de + 1;
                        d_state <= D_SEQ_DATA;
                    end
                end
                D_SEQ_NL2: begin
                    manual_tx_data <= 8'h0A; // Newline
                    d_state <= D_DO_MANUAL;
                    next_seq_state <= D_NEXT;
                end

                // Helpers
                D_DO_B2A: begin
                    use_b2a <= 1;
                    b2a_start <= 1;
                    d_state <= D_WAIT_B2A;
                end
                D_WAIT_B2A: begin
                    b2a_start <= 0;
                    if (b2a_done) begin
                        use_b2a <= 0;
                        d_state <= next_seq_state;
                    end
                end
                
                D_DO_MANUAL: begin
                    if (!tx_busy) begin
                        manual_tx_start <= 1;
                        d_state <= D_WAIT_MANUAL;
                    end
                end
                D_WAIT_MANUAL: begin
                    manual_tx_start <= 0;
                    d_state <= next_seq_state;
                end

                D_NEXT: begin
                    current_slot <= current_slot + 1;
                    d_state <= D_CHECK;
                end

                D_DONE_STATE: begin
                    display_done <= 1;
                    if (!display_start) d_state <= D_IDLE;
                end
            endcase
        end
    end

endmodule
