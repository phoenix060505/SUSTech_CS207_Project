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
    input wire flush,                
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
    input wire tx_busy,              
    input wire filter_en,            
    input wire show_specific_en, // New input
    input wire [3:0] filter_m,       
    input wire [3:0] filter_n,       
    output wire tx_start,            
    output wire [7:0] tx_data,       
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

    // 假设你有 bin_to_ascii 模块，这里保持实例化
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
    // 1. Storage Structure
    //==============================================================
    parameter MAX_MATRICES = 16;
    parameter MAX_PER_TYPE = 2; 
    
    reg [7:0] storage_pool [0:15][0:24]; 
    reg [7:0] matrix_ids [0:15];         
    reg [3:0] matrix_rows [0:15];        
    reg [3:0] matrix_cols [0:15];        
    reg       slot_used [0:15];          
    reg [1:0] type_age [0:15];           
    reg [7:0] next_matrix_id;
    reg [3:0] global_replace_ptr;

    // Helper function: Pure Combinational Logic for finding slot
    function [3:0] find_write_slot;
        input [3:0] m, n;
        input [3:0] current_replace_ptr; 
        // Note: passing current_replace_ptr to make function pure
        integer i;
        reg [3:0] count;
        reg [3:0] oldest_slot;
        reg found_oldest;
        begin
            find_write_slot = 4'hF;
            count = 0;
            oldest_slot = 4'hF;
            found_oldest = 0;
            
            // 1. Count existing matrices of same type and find oldest
            for(i=0; i<MAX_MATRICES; i=i+1) begin
                if (slot_used[i] && matrix_rows[i] == m && matrix_cols[i] == n) begin
                    count = count + 1;
                    if (type_age[i] == MAX_PER_TYPE - 1) begin
                        oldest_slot = i[3:0];
                        found_oldest = 1;
                    end
                end
            end
            
            // 2. Decision
            if (count < MAX_PER_TYPE) begin
                // Find any free slot
                for(i=0; i<MAX_MATRICES; i=i+1) begin
                    if (!slot_used[i] && find_write_slot == 4'hF) find_write_slot = i[3:0];
                end
                // If no free slot but we haven't reached MAX_PER_TYPE (weird edge case), use replace ptr
                if (find_write_slot == 4'hF) find_write_slot = current_replace_ptr; 
            end else begin
                // Overwrite oldest
                if (found_oldest) find_write_slot = oldest_slot;
                else find_write_slot = current_replace_ptr; // Fallback
            end
        end
    endfunction

    // Update ages task
    task update_ages;
        input [3:0] target_slot_idx;
        input [3:0] m, n;
        integer i;
        begin
            for(i=0; i<MAX_MATRICES; i=i+1) begin
                if (slot_used[i] && matrix_rows[i] == m && matrix_cols[i] == n) begin
                    if (i == target_slot_idx) type_age[i] <= 0; // Newest
                    else if (type_age[i] < MAX_PER_TYPE - 1) type_age[i] <= type_age[i] + 1;
                end
            end
        end
    endtask

    //==============================================================
    // 2. Random Number Generator
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
    reg [5:0] cur_size; 
    reg [1:0] cur_count_target, cur_count_done;
    reg [3:0] state;
    reg [4:0] elem_cnt;
    reg [7:0] temp_buf [0:24];

    integer i, k;
    reg [3:0] active_slot; 

    // Helper signals for ASCII processing
    wire is_digit = (data_in >= 8'h30 && data_in <= 8'h39);
    wire is_whitespace = (data_in == 8'h20 || data_in == 8'h0A || data_in == 8'h0D || data_in == 8'h09);
    wire [3:0] val_digit = data_in[3:0]; // ASCII '0' is 0x30, so lower 4 bits are 0-9

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            input_done <= 0;
            error_flag <= 0;
            elem_cnt <= 0;
            cur_size <= 0;
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
                    if (wen) begin
                        if (is_whitespace) begin
                            // Ignore whitespace
                        end else if (is_digit) begin
                            if (mode == 2'b00) begin // Input Mode
                                if (val_digit >= 1 && val_digit <= 5) begin
                                    cur_m <= val_digit;
                                    state <= S_IN_N;
                                end else begin
                                    error_flag <= 1;
                                    state <= S_ERR;
                                end
                            end else if (mode == 2'b01) begin // Gen Mode
                                if (val_digit >= 1 && val_digit <= 5) begin
                                    cur_m <= val_digit;
                                    state <= S_GEN_N;
                                end else begin
                                    error_flag <= 1;
                                    state <= S_ERR;
                                end
                            end
                        end else begin
                            // Invalid char
                            error_flag <= 1;
                            state <= S_ERR;
                        end
                    end
                end

                S_IN_N: begin
                    if (wen) begin
                        if (is_whitespace) begin
                            // Ignore
                        end else if (is_digit) begin
                            if (val_digit >= 1 && val_digit <= 5) begin
                                cur_n <= val_digit;
                                cur_size <= cur_m * val_digit;
                                state <= S_CALC_SLOT_IN;
                            end else begin
                                error_flag <= 1;
                                state <= S_ERR;
                            end
                        end else begin
                            error_flag <= 1;
                            state <= S_ERR;
                        end
                    end
                end

                S_CALC_SLOT_IN: begin
                    // 使用非阻塞赋值计算 Slot
                    if (find_write_slot(cur_m, cur_n, global_replace_ptr) == 4'hF) begin
                        // This case should ideally be handled by the function falling back
                        active_slot <= global_replace_ptr;
                        global_replace_ptr <= global_replace_ptr + 1'b1;
                    end else begin
                        active_slot <= find_write_slot(cur_m, cur_n, global_replace_ptr);
                    end
                    state <= S_IN_DATA;
                    elem_cnt <= 0;
                end

                S_IN_DATA: begin
                    if (flush) begin
                        // Write Buffer to Storage (Use Non-blocking <=)
                        for (k=0; k<25; k=k+1) begin
                            if (k < elem_cnt) 
                                storage_pool[active_slot][k] <= temp_buf[k];
                            else if (k < cur_size) 
                                storage_pool[active_slot][k] <= 8'd0; // Pad
                        end
                        
                        lfsr_en <= 1; // Just to tick LFSR
                        matrix_ids[active_slot] <= next_matrix_id;
                        matrix_rows[active_slot] <= cur_m;
                        matrix_cols[active_slot] <= cur_n;
                        slot_used[active_slot] <= 1;
                        update_ages(active_slot, cur_m, cur_n);
                        next_matrix_id <= (next_matrix_id == 8'hFF) ? 8'h01 : next_matrix_id + 1'b1;
                        state <= S_DONE;
                        
                    end else if (wen) begin
                        if (is_whitespace) begin
                            // Ignore
                        end else if (is_digit) begin
                            // val_digit is 0-9, which is exactly what we want for matrix elements
                            temp_buf[elem_cnt] <= {4'b0, val_digit}; // Store as 8-bit value
                            if (elem_cnt == (cur_size) - 1) begin
                                // Full matrix received
                                for (k=0; k<25; k=k+1) begin
                                    if (k <= elem_cnt) // Write all accumulated
                                        storage_pool[active_slot][k] <= (k==elem_cnt) ? {4'b0, val_digit} : temp_buf[k];
                                end
                                matrix_ids[active_slot] <= next_matrix_id;
                                matrix_rows[active_slot] <= cur_m;
                                matrix_cols[active_slot] <= cur_n;
                                slot_used[active_slot] <= 1;
                                update_ages(active_slot, cur_m, cur_n);
                                next_matrix_id <= (next_matrix_id == 8'hFF) ? 8'h01 : next_matrix_id + 1'b1;
                                state <= S_DONE;
                            end else begin
                                elem_cnt <= elem_cnt + 1;
                            end
                        end else begin
                            error_flag <= 1;
                            state <= S_ERR;
                        end
                    end
                end

                S_GEN_N: begin
                    if (wen) begin
                        if (is_whitespace) begin
                            // Ignore
                        end else if (is_digit) begin
                            if (val_digit >= 1 && val_digit <= 5) begin
                                cur_n <= val_digit;
                                cur_size <= cur_m * val_digit;
                                state <= S_GEN_COUNT;
                            end else begin
                                error_flag <= 1;
                                state <= S_ERR;
                            end
                        end else begin
                            error_flag <= 1;
                            state <= S_ERR;
                        end
                    end
                end

                S_GEN_COUNT: begin
                    if (wen) begin
                        if (is_whitespace) begin
                            // Ignore
                        end else if (is_digit) begin
                            if (val_digit >= 1 && val_digit <= MAX_STORE) begin
                                cur_count_target <= val_digit[1:0];
                                cur_count_done <= 0;
                                state <= S_CALC_SLOT_GEN;
                            end else begin
                                error_flag <= 1;
                                state <= S_ERR;
                            end
                        end else begin
                            error_flag <= 1;
                            state <= S_ERR;
                        end
                    end
                end

                S_CALC_SLOT_GEN: begin
                    if (find_write_slot(cur_m, cur_n, global_replace_ptr) == 4'hF) begin
                        active_slot <= global_replace_ptr;
                        global_replace_ptr <= global_replace_ptr + 1'b1;
                    end else begin
                        active_slot <= find_write_slot(cur_m, cur_n, global_replace_ptr);
                    end
                    state <= S_GEN_EXEC;
                    elem_cnt <= 0;
                end

                S_GEN_EXEC: begin
                    lfsr_en <= 1; // Enable LFSR
                    // Write Random Data
                    storage_pool[active_slot][elem_cnt] <= rand_val;
                    
                    if (elem_cnt == (cur_size) - 1) begin
                        matrix_ids[active_slot] <= next_matrix_id;
                        matrix_rows[active_slot] <= cur_m;
                        matrix_cols[active_slot] <= cur_n;
                        slot_used[active_slot] <= 1;
                        update_ages(active_slot, cur_m, cur_n);
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
    // 4. Robust Read Logic (2-Stage Pipeline)
    //==============================================================
    // Note: It takes 2 cycles from read_id change to mat_out valid.
    // Cycle 1: Search -> p1_slot
    // Cycle 2: Fetch -> mat_out
    
    integer search_i;
    reg [3:0] next_slot_a, next_slot_b;
    reg next_found_a, next_found_b;
    reg [3:0] p1_slot_a, p1_slot_b;
    reg p1_found_a, p1_found_b;

    // Stage 1: Search (Combinational)
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

    // Stage 1: Register
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

    // Stage 2: Fetch Data
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            mat_a_out <= 0; dim_a_m_int <= 0; dim_a_n_int <= 0;
            mat_b_out <= 0; dim_b_m_int <= 0; dim_b_n_int <= 0;
        end else begin
            // Matrix A
            if (p1_found_a) begin
                dim_a_m_int <= matrix_rows[p1_slot_a];
                dim_a_n_int <= matrix_cols[p1_slot_a];
                // Flatten 2D array to 1D output vector
                mat_a_out <= {
                    storage_pool[p1_slot_a][24], storage_pool[p1_slot_a][23], storage_pool[p1_slot_a][22], storage_pool[p1_slot_a][21], storage_pool[p1_slot_a][20],
                    storage_pool[p1_slot_a][19], storage_pool[p1_slot_a][18], storage_pool[p1_slot_a][17], storage_pool[p1_slot_a][16], storage_pool[p1_slot_a][15],
                    storage_pool[p1_slot_a][14], storage_pool[p1_slot_a][13], storage_pool[p1_slot_a][12], storage_pool[p1_slot_a][11], storage_pool[p1_slot_a][10],
                    storage_pool[p1_slot_a][9],  storage_pool[p1_slot_a][8],  storage_pool[p1_slot_a][7],  storage_pool[p1_slot_a][6],  storage_pool[p1_slot_a][5],
                    storage_pool[p1_slot_a][4],  storage_pool[p1_slot_a][3],  storage_pool[p1_slot_a][2],  storage_pool[p1_slot_a][1],  storage_pool[p1_slot_a][0]
                };
            end else begin
                dim_a_m_int <= 0; dim_a_n_int <= 0;
                mat_a_out <= 0;
            end

            // Matrix B
            if (p1_found_b) begin
                dim_b_m_int <= matrix_rows[p1_slot_b];
                dim_b_n_int <= matrix_cols[p1_slot_b];
                mat_b_out <= {
                    storage_pool[p1_slot_b][24], storage_pool[p1_slot_b][23], storage_pool[p1_slot_b][22], storage_pool[p1_slot_b][21], storage_pool[p1_slot_b][20],
                    storage_pool[p1_slot_b][19], storage_pool[p1_slot_b][18], storage_pool[p1_slot_b][17], storage_pool[p1_slot_b][16], storage_pool[p1_slot_b][15],
                    storage_pool[p1_slot_b][14], storage_pool[p1_slot_b][13], storage_pool[p1_slot_b][12], storage_pool[p1_slot_b][11], storage_pool[p1_slot_b][10],
                    storage_pool[p1_slot_b][9],  storage_pool[p1_slot_b][8],  storage_pool[p1_slot_b][7],  storage_pool[p1_slot_b][6],  storage_pool[p1_slot_b][5],
                    storage_pool[p1_slot_b][4],  storage_pool[p1_slot_b][3],  storage_pool[p1_slot_b][2],  storage_pool[p1_slot_b][1],  storage_pool[p1_slot_b][0]
                };
            end else begin
                dim_b_m_int <= 0; dim_b_n_int <= 0;
                mat_b_out <= 0;
            end
        end
    end

    //==============================================================
    // 5. Improved Display Logic (Fixed Handshake)
    //==============================================================
    localparam D_IDLE = 0, D_CHECK = 1, D_NEXT = 2, D_DONE_STATE = 3;
    localparam D_SEQ_ID = 4, D_SEQ_SPACE1 = 5, D_SEQ_M = 6, D_SEQ_SPACE2 = 7, D_SEQ_N = 8, D_SEQ_NL1 = 9;
    localparam D_SEQ_DATA = 10, D_SEQ_SPACE3 = 11, D_SEQ_CHECK_LOOP = 12, D_SEQ_NL2 = 13;
    localparam D_DO_B2A = 14, D_WAIT_B2A = 15, D_DO_MANUAL = 16, D_WAIT_MANUAL_BUSY = 17, D_WAIT_MANUAL_IDLE = 18;

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
            manual_tx_start <= 0; // Default off
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
                            // Filter Logic
                            if (show_specific_en) begin
                                // Only show if ID matches read_id_a or read_id_b
                                // We need to know which one. 
                                // Hack: We can infer from filter_en being false? No.
                                // Better: Pass the target ID to filter.
                                // But wait, read_id_a/b are inputs.
                                // If we are in S_CALC_A_SHOW, we want to show read_id_a.
                                // If we are in S_CALC_B_SHOW, we want to show read_id_b.
                                // Since we don't have state info here, we can rely on the fact that
                                // top module sets read_id_a/b correctly before this state.
                                // But wait, both are always connected.
                                // Let's assume if show_specific_en is high, we check against read_id_a OR read_id_b?
                                // Or better, let's just check if ID matches EITHER.
                                // Since in A_SHOW, only A ID is set (or B is irrelevant), and vice versa.
                                // Actually, in Top, id_a is set in S_CALC_A_ID.
                                if (matrix_ids[current_slot] == read_id_a || matrix_ids[current_slot] == read_id_b) begin
                                     d_state <= D_SEQ_ID;
                                     de <= 0;
                                end else begin
                                     current_slot <= current_slot + 1;
                                end
                            end else if (!filter_en || (matrix_rows[current_slot] == filter_m && matrix_cols[current_slot] == filter_n)) begin
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

                // --- Sequence Logic Same as before, just state flow changes ---
                D_SEQ_ID: begin
                    b2a_value <= {8'b0, matrix_ids[current_slot]};
                    d_state <= D_DO_B2A;
                    next_seq_state <= D_SEQ_SPACE1;
                end
                D_SEQ_SPACE1: begin
                    manual_tx_data <= 8'h20; // Space
                    d_state <= D_DO_MANUAL;
                    next_seq_state <= D_SEQ_M;
                end
                D_SEQ_M: begin
                    b2a_value <= {12'b0, matrix_rows[current_slot]};
                    d_state <= D_DO_B2A;
                    next_seq_state <= D_SEQ_SPACE2;
                end
                D_SEQ_SPACE2: begin
                    manual_tx_data <= 8'h20; 
                    d_state <= D_DO_MANUAL;
                    next_seq_state <= D_SEQ_N;
                end
                D_SEQ_N: begin
                    b2a_value <= {12'b0, matrix_cols[current_slot]};
                    d_state <= D_DO_B2A;
                    next_seq_state <= D_SEQ_NL1;
                end
                D_SEQ_NL1: begin
                    manual_tx_data <= 8'h0A; // Newline
                    d_state <= D_DO_MANUAL;
                    next_seq_state <= D_SEQ_DATA;
                end

                D_SEQ_DATA: begin
                    b2a_value <= {8'b0, storage_pool[current_slot][de]};
                    d_state <= D_DO_B2A;
                    next_seq_state <= D_SEQ_SPACE3;
                end
                D_SEQ_SPACE3: begin
                    manual_tx_data <= 8'h20;
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
                    manual_tx_data <= 8'h0A;
                    d_state <= D_DO_MANUAL;
                    next_seq_state <= D_NEXT;
                end

                // --- Helper States ---
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
                
                // === FIXED MANUAL TX HANDSHAKE ===
                D_DO_MANUAL: begin
                    if (!tx_busy) begin
                        manual_tx_start <= 1;
                        d_state <= D_WAIT_MANUAL_BUSY; // New state
                    end
                end
                // Wait for Busy to go High (Acknowledgment)
                D_WAIT_MANUAL_BUSY: begin
                    manual_tx_start <= 0;
                    // Usually takes 1 cycle for UART IP to raise busy.
                    // If it's already high, good. If not, wait.
                    // Risk: if UART is super fast, it might have finished already?
                    // Safe approach: Wait for busy to be 1, OR if it's already 0 but we just started, 
                    // we assume we need to wait for it to become 0 AFTER being 1.
                    // For typical UART IPs, just waiting for !busy again is often safer if we add a small delay.
                    // Here we simply wait for !busy (Idle) but ensure we passed through a busy state? 
                    // Simplest safe way: Just wait for busy to drop.
                    if (tx_busy) d_state <= D_WAIT_MANUAL_IDLE;
                    else d_state <= D_WAIT_MANUAL_IDLE; // Assume we might have missed the rising edge, wait for idle anyway
                end
                D_WAIT_MANUAL_IDLE: begin
                   if (!tx_busy) begin
                       d_state <= next_seq_state;
                   end
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