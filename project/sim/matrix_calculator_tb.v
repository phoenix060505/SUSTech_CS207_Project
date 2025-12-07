`timescale 1ns / 1ps

module matrix_calculator_tb;

    // Parameters
    parameter MAX_DIM = 5;
    parameter MAX_STORE = 2;
    parameter ELEM_WIDTH = 8;
    parameter TIMEOUT_DEFAULT = 10;
    parameter CLK_FREQ = 100_000_000;
    parameter UART_BIT_PERIOD = 1000000000 / 115200; // ns per bit

    // Inputs
    reg clk;
    reg rst;
    reg [7:0] sw;
    reg [3:0] btn;
    reg uart_rx;

    // Outputs
    wire [7:0] led;
    wire [7:0] seg;
    wire [3:0] an;
    wire uart_tx;

    // Instantiate the Unit Under Test (UUT)
    matrix_calculator_top #(
        .MAX_DIM(MAX_DIM),
        .MAX_STORE(MAX_STORE),
        .ELEM_WIDTH(ELEM_WIDTH),
        .TIMEOUT_DEFAULT(TIMEOUT_DEFAULT),
        .CLK_FREQ(CLK_FREQ)
    ) uut (
        .clk(clk),
        .rst(rst),
        .sw(sw),
        .btn(btn),
        .uart_rx(uart_rx),
        .led(led),
        .seg(seg),
        .an(an),
        .uart_tx(uart_tx)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz
    end

    // UART Task
    task send_byte;
        input [7:0] data;
        integer i;
        begin
            // Start bit
            uart_rx = 0;
            #(UART_BIT_PERIOD);
            
            // Data bits
            for (i = 0; i < 8; i = i + 1) begin
                uart_rx = data[i];
                #(UART_BIT_PERIOD);
            end
            
            // Stop bit
            uart_rx = 1;
            #(UART_BIT_PERIOD);
            
            // Inter-byte delay
            #(UART_BIT_PERIOD * 2);
        end
    endtask

    // Button Press Task
    task press_confirm;
        begin
            btn[0] = 1;
            #25000000; // 25ms > 20ms debounce
            btn[0] = 0;
            #2000000; // Wait a bit
        end
    endtask

    // Test Sequence
    initial begin
        // Initialize Inputs
        rst = 1;
        sw = 0;
        btn = 0;
        uart_rx = 1;

        // Wait for global reset
        #100;
        rst = 0;
        #100;

        $display("Starting Simulation...");

        // ============================================================
        // 1. Input Matrix A (ID 1, 2x2)
        // ============================================================
        $display("Step 1: Input Matrix A (2x2)");
        sw = 8'b00000000; // Mode: Input
        press_confirm();
        
        // Send Dimensions
        send_byte(2); // M=2
        send_byte(2); // N=2
        
        // Send Data (1, 2, 3, 4)
        send_byte(1);
        send_byte(2);
        send_byte(3);
        send_byte(4);
        
        // Wait for Input Done (LED[2])
        wait(led[2] == 1);
        $display("Matrix A Input Done");
        #1000000;

        // ============================================================
        // 2. Input Matrix B (ID 2, 2x2)
        // ============================================================
        $display("Step 2: Input Matrix B (2x2)");
        sw = 8'b00000000; // Mode: Input
        press_confirm();
        
        // Send Dimensions
        send_byte(2); // M=2
        send_byte(2); // N=2
        
        // Send Data (5, 6, 7, 8)
        send_byte(5);
        send_byte(6);
        send_byte(7);
        send_byte(8);
        
        wait(led[2] == 1);
        $display("Matrix B Input Done");
        #1000000;

        // ============================================================
        // 3. Input Matrix C (ID 3, 3x3) for Convolution
        // ============================================================
        $display("Step 3: Input Matrix C (3x3)");
        sw = 8'b00000000; // Mode: Input
        press_confirm();
        
        // Send Dimensions
        send_byte(3); // M=3
        send_byte(3); // N=3
        
        // Send Data (1..9)
        send_byte(1); send_byte(2); send_byte(3);
        send_byte(4); send_byte(5); send_byte(6);
        send_byte(7); send_byte(8); send_byte(9);
        
        wait(led[2] == 1);
        $display("Matrix C Input Done");
        #1000000;

        // ============================================================
        // 4. Test Addition (A + B)
        // ============================================================
        $display("Step 4: Test Addition (A + B)");
        
        sw = 8'b11000000; // Mode: Calc
        press_confirm(); // -> S_CALC_OP
        
        sw = 8'b00000001; // Op 1 (Add)
        press_confirm(); // -> S_CALC_A_M
        
        // Select A (2x2, ID 1)
        sw = 8'b00000010; // M=2
        press_confirm();
        sw = 8'b00000010; // N=2
        press_confirm();
        wait(led[3] == 1); // List
        sw = 8'b00000001; // ID=1
        press_confirm(); // -> S_CALC_B_M
        
        // Select B (2x2, ID 2)
        sw = 8'b00000010; // M=2
        press_confirm();
        sw = 8'b00000010; // N=2
        press_confirm();
        wait(led[3] == 1); // List
        sw = 8'b00000010; // ID=2
        press_confirm(); // -> EXEC
        
        wait(led[4] == 1);
        $display("Addition Done");
        #2000000;

        // ============================================================
        // 5. Test Multiplication (A * B)
        // ============================================================
        $display("Step 5: Test Multiplication (A * B)");
        
        sw = 8'b11000000; 
        press_confirm(); // -> S_CALC_OP
        
        sw = 8'b00000011; // Op 3 (Mul)
        press_confirm(); // -> S_CALC_A_M
        
        // Select A (2x2, ID 1)
        sw = 8'b00000010; // M=2
        press_confirm();
        sw = 8'b00000010; // N=2
        press_confirm();
        wait(led[3] == 1); // List
        sw = 8'b00000001; // ID=1
        press_confirm(); // -> S_CALC_B_M
        
        // Select B (2x2, ID 2)
        sw = 8'b00000010; // M=2
        press_confirm();
        sw = 8'b00000010; // N=2
        press_confirm();
        wait(led[3] == 1); // List
        sw = 8'b00000010; // ID=2
        press_confirm(); // -> EXEC
        
        wait(led[4] == 1);
        $display("Multiplication Done");
        #2000000;

        // ============================================================
        // 6. Test Transpose (A^T)
        // ============================================================
        $display("Step 6: Test Transpose (A^T)");
        
        sw = 8'b11000000; 
        press_confirm(); 
        
        sw = 8'b00000000; // Op 0 (Transpose)
        press_confirm(); // -> S_CALC_A_M
        
        // Select A (2x2, ID 1)
        sw = 8'b00000010; // M=2
        press_confirm();
        sw = 8'b00000010; // N=2
        press_confirm();
        wait(led[3] == 1); // List
        sw = 8'b00000001; // ID=1
        press_confirm(); // -> EXEC (Unary)
        
        wait(led[4] == 1);
        $display("Transpose Done");
        #2000000;

        // ============================================================
        // 7. Test Scalar Mul (B * 2)
        // ============================================================
        $display("Step 7: Test Scalar Mul (B * 2)");
        
        sw = 8'b11000000; 
        press_confirm(); 
        
        sw = 8'b00000010; // Op 2 (Scalar Mul)
        press_confirm(); // -> S_CALC_A_M
        
        // Select B (2x2, ID 2)
        sw = 8'b00000010; // M=2
        press_confirm();
        sw = 8'b00000010; // N=2
        press_confirm();
        wait(led[3] == 1); // List
        sw = 8'b00000010; // ID=2 (Also Scalar=2)
        press_confirm(); // -> EXEC (Unary)
        
        wait(led[4] == 1);
        $display("Scalar Mul Done");
        #2000000;

        // ============================================================
        // 8. Test Convolution (Kernel C)
        // ============================================================
        $display("Step 8: Test Convolution (Kernel C)");
        
        sw = 8'b11000000; 
        press_confirm(); 
        
        sw = 8'b00000101; // Op 5 (Conv)
        press_confirm(); // -> S_CALC_A_M
        
        // Select C (3x3, ID 3)
        sw = 8'b00000011; // M=3
        press_confirm();
        sw = 8'b00000011; // N=3
        press_confirm();
        wait(led[3] == 1); // List
        sw = 8'b00000011; // ID=3
        press_confirm(); // -> EXEC (Unary)
        
        wait(led[4] == 1);
        $display("Convolution Done");
        #2000000;

        $display("All Tests Completed Successfully");
        $stop;
    end

endmodule
