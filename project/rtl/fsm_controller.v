module fsm_controller (
    input wire clk,
    input wire rst,
    input wire [3:0] btn,        // btn[0]: Confirm/Start
    input wire [7:0] sw,         // Mode selection and configuration
    
    // Status feedback from sub-modules
    input wire input_done,       // Matrix input/generation done
    input wire display_done,     // Matrix display done
    input wire operand_legal,    // Operand legality check
    input wire compute_done,     // Computation done
    input wire timeout_expired,  // Timeout expired

    // Control signals to Top Module
    output reg [1:0] mode,       // 00:Input, 01:Gen, 10:Display, 11:Calc
    output reg [3:0] op_type,    // Operation type
    output reg timeout_en,       // Enable timeout
    output reg wen_store,        // Storage write enable
    output reg start_compute,    // Start computation pulse
    output reg update_config,    // Update configuration
    output reg [1:0] calc_step,  // 0: Op, 1: A, 2: B (Legacy, can be used for LED)
    output wire [4:0] fsm_state_out // Expose state for Top control
);

    assign fsm_state_out = current_state;

    //==============================================================
    // 1. State Definition
    //==============================================================
    localparam S_IDLE        = 4'd0;  
    localparam S_MODE_DECIDE = 4'd1;  
    localparam S_INPUT       = 4'd2;  
    localparam S_GEN         = 4'd3;  
    localparam S_DISPLAY     = 4'd4;  
    localparam S_CALC_OP     = 4'd5;  // Select Op
    localparam S_CALC_CHECK  = 4'd6;  
    localparam S_CALC_EXEC   = 4'd7;  
    localparam S_ERROR       = 4'd8;  
    localparam S_CONFIG      = 4'd9;  
    
    // New States for "Dim -> List -> ID" flow
    localparam S_CALC_A_M    = 5'd10; 
    localparam S_CALC_A_N    = 5'd11;
    localparam S_CALC_A_LIST = 5'd12;
    localparam S_CALC_A_ID   = 5'd13;
    
    localparam S_CALC_B_M    = 5'd14;
    localparam S_CALC_B_N    = 5'd15;
    localparam S_CALC_B_LIST = 5'd16;
    localparam S_CALC_B_ID   = 5'd17;
    localparam S_CALC_WAIT   = 5'd18; // Wait for storage read

    reg [4:0] current_state, next_state; // Expanded to 5 bits

    wire confirm_pressed = btn[0]; 
    
    // Helper to check if op needs B operand (Binary Ops: 1=Add, 3=Mul)
    // Op 0=Trans, 2=Scalar, 5=Conv are Unary
    function is_binary_op;
        input [3:0] op;
        begin
            is_binary_op = (op == 4'd1 || op == 4'd3);
        end
    endfunction

    //==============================================================
    // 2. State Transition Logic
    //==============================================================
    always @(*) begin
        next_state = current_state; 

        case (current_state)
            S_IDLE: begin
                if (confirm_pressed) next_state = S_MODE_DECIDE;
            end

            S_MODE_DECIDE: begin
                if (timeout_expired) begin
                    next_state = S_IDLE;
                end else begin
                case (sw[7:6])
                    2'b00: next_state = S_INPUT;       
                    2'b01: next_state = S_GEN;         
                    2'b10: next_state = S_DISPLAY;     
                    2'b11: begin
                        if (sw[5]) next_state = S_CONFIG; 
                        else next_state = S_CALC_OP;  
                    end
                endcase
                end
            end

            S_INPUT: begin
                if (timeout_expired) next_state = S_IDLE;
                else if (input_done) next_state = S_IDLE;
            end

            S_GEN: begin
                if (timeout_expired) next_state = S_IDLE;
                else if (input_done) next_state = S_IDLE;
            end

            S_DISPLAY: begin
                if (timeout_expired) next_state = S_IDLE;
                else if (display_done) next_state = S_IDLE;
            end

            // --- Calculation Flow ---
            S_CALC_OP: begin
                if (timeout_expired) next_state = S_IDLE;
                else if (confirm_pressed) next_state = S_CALC_A_M;
            end

            // Operand A Selection
            S_CALC_A_M: begin
                if (timeout_expired) next_state = S_IDLE;
                else if (confirm_pressed) next_state = S_CALC_A_N;
            end
            S_CALC_A_N: begin
                if (timeout_expired) next_state = S_IDLE;
                else if (confirm_pressed) next_state = S_CALC_A_LIST;
            end
            S_CALC_A_LIST: begin
                if (display_done) next_state = S_CALC_A_ID;
            end
            S_CALC_A_ID: begin
                if (timeout_expired) next_state = S_IDLE;
                else if (confirm_pressed) begin
                    if (is_binary_op(op_type)) next_state = S_CALC_B_M;
                    else next_state = S_CALC_WAIT;
                end
            end

            // Operand B Selection (Only for Binary Ops)
            S_CALC_B_M: begin
                if (timeout_expired) next_state = S_IDLE;
                else if (confirm_pressed) next_state = S_CALC_B_N;
            end
            S_CALC_B_N: begin
                if (timeout_expired) next_state = S_IDLE;
                else if (confirm_pressed) next_state = S_CALC_B_LIST;
            end
            S_CALC_B_LIST: begin
                if (display_done) next_state = S_CALC_B_ID;
            end
            S_CALC_B_ID: begin
                if (timeout_expired) next_state = S_IDLE;
                else if (confirm_pressed) next_state = S_CALC_WAIT;
            end

            S_CALC_WAIT: begin
                next_state = S_CALC_CHECK;
            end

            S_CALC_CHECK: begin
                if (operand_legal) next_state = S_CALC_EXEC;
                else next_state = S_ERROR; 
            end

            S_CALC_EXEC: begin
                if (compute_done) next_state = S_IDLE; 
            end

            S_ERROR: begin
                if (timeout_expired) next_state = S_IDLE;
            end

            S_CONFIG: begin
                if (timeout_expired) next_state = S_IDLE;
                else if (confirm_pressed) next_state = S_IDLE;
            end

            default: next_state = S_IDLE;
        endcase
    end

    //==============================================================
    // 3. State Update Logic
    //==============================================================
    always @(posedge clk or posedge rst) begin
        if (rst) 
            current_state <= S_IDLE;
        else 
            current_state <= next_state;
    end

    function is_wait_state;
        input [4:0] st;
        begin
            case (st)
                S_MODE_DECIDE,
                S_INPUT,
                S_GEN,
                S_DISPLAY,
                S_CALC_OP,
                S_CALC_A_M,
                S_CALC_A_N,
                S_CALC_A_ID,
                S_CALC_B_M,
                S_CALC_B_N,
                S_CALC_B_ID,
                S_CONFIG,
                S_ERROR: is_wait_state = 1'b1;
                default: is_wait_state = 1'b0;
            endcase
        end
    endfunction

    //==============================================================
    // 4. Output Control Logic
    //==============================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            mode <= 2'b00;
            op_type <= 4'd0;
            wen_store <= 1'b0;
            start_compute <= 1'b0;
            update_config <= 1'b0;
            calc_step <= 0;
            timeout_en <= 1'b0;
        end else begin
            wen_store <= 1'b0;
            start_compute <= 1'b0;
            update_config <= 1'b0;
            timeout_en <= is_wait_state(next_state);
            
            case (next_state) 
                S_INPUT: begin
                    mode <= 2'b00;
                    wen_store <= 1'b1; 
                end
                S_GEN: begin
                    mode <= 2'b01;
                    wen_store <= 1'b1; 
                end
                S_DISPLAY: begin
                    mode <= 2'b10;
                end
                S_CALC_OP: begin
                    mode <= 2'b11;
                    calc_step <= 0;
                end
                
                // A Selection Flow
                S_CALC_A_M, S_CALC_A_N: begin
                    mode <= 2'b11;
                    calc_step <= 1;
                    if (current_state == S_CALC_OP) op_type <= sw[3:0];
                end
                S_CALC_A_LIST: begin
                    mode <= 2'b10; // Display Mode for List
                    calc_step <= 1;
                end
                S_CALC_A_ID: begin
                    mode <= 2'b11;
                    calc_step <= 1;
                end

                // B Selection Flow
                S_CALC_B_M, S_CALC_B_N: begin
                    mode <= 2'b11;
                    calc_step <= 2;
                end
                S_CALC_B_LIST: begin
                    mode <= 2'b10; // Display Mode for List
                    calc_step <= 2;
                end
                S_CALC_B_ID: begin
                    mode <= 2'b11;
                    calc_step <= 2;
                end

                S_CALC_WAIT: begin
                    mode <= 2'b11;
                end

                S_CALC_CHECK: begin
                    mode <= 2'b11;
                end
                S_CALC_EXEC: begin
                    mode <= 2'b11;
                    if (current_state == S_CALC_CHECK) 
                        start_compute <= 1'b1; 
                end
                S_CONFIG: begin
                    if (confirm_pressed) update_config <= 1'b1;
                end
            endcase
        end
    end

endmodule
