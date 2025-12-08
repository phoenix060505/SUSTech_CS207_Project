`timescale 1ns / 1ps
module debounce (
    input wire clk,      // System Clock
    input wire rst,
    input wire btn_in,   // Physical Button Input
    output reg btn_out   // Debounced Stable Signal (Single Cycle Pulse)
);
    // Parameter Calculation: 20ms @ 100MHz = 2,000,000 cycles
    parameter CNT_MAX = 2_000_000; 
    
    reg [20:0] cnt;
    reg btn_stable;
    reg btn_sync_0, btn_sync_1; // Synchronization registers to prevent metastability

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cnt <= 0;
            btn_out <= 0;
            btn_stable <= 0;
            btn_sync_0 <= 0;
            btn_sync_1 <= 0;
        end else begin
            // 1. Two-stage synchronization
            btn_sync_0 <= btn_in;
            btn_sync_1 <= btn_sync_0;

            // 2. Counter Logic
            if (btn_sync_1 != btn_stable) begin
                if (cnt < CNT_MAX) begin
                    cnt <= cnt + 1;
                end else begin
                    btn_stable <= btn_sync_1; // Update stable state
                    cnt <= 0;
                    // 3. Generate single cycle pulse on press (Rising Edge Detect)
                    if (btn_sync_1 == 1'b1) btn_out <= 1'b1; 
                end
            end else begin
                cnt <= 0;
                btn_out <= 1'b0; // Reset pulse
            end
        end
    end
endmodule