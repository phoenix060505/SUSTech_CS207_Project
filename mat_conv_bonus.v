// Bonus: 3x3 convolution on fixed 10x12 image (stride=1, valid padding)
// - Image is hard-coded in on-chip ROM (values 0-9, 4-bit)
// - Kernel is provided at run time as 9 successive 4-bit values (kernel_valid pulses)
// - Start pulse resets and begins accepting kernel; computation starts after 9 values captured
// - Outputs stream 8x10 results with out_valid/out_row_end/out_last strobes
// - cycle_count counts the cycles spent in CALC/EMIT for the current run

module mat_conv_bonus #(
    parameter DATA_WIDTH   = 4,   // image pixel width (0-9)
    parameter KERNEL_WIDTH = 4,   // kernel element width (0-9)
    parameter ACC_WIDTH    = 12   // accumulator width (covers 9*9*9=729 < 2^10)
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  start,          // pulse to start a new run
    input  wire [KERNEL_WIDTH-1:0] kernel_in,    // kernel element (0-9)
    input  wire                  kernel_valid,   // strobe for kernel_in
    output reg                   kernel_ready,   // high while waiting for 9 values
    output reg                   busy,
    output reg                   done,
    output reg                   out_valid,
    output reg  [ACC_WIDTH-1:0]  out_elem,
    output reg                   out_row_end,
    output reg                   out_last,
    output reg  [3:0]            out_row_idx,
    output reg  [3:0]            out_col_idx,
    output reg  [15:0]           cycle_count
);
    localparam IMG_M  = 10;
    localparam IMG_N  = 12;
    localparam K_SIZE = 3;
    localparam OUT_M  = IMG_M - K_SIZE + 1; // 8
    localparam OUT_N  = IMG_N - K_SIZE + 1; // 10

    // Hard-coded image ROM
    wire [DATA_WIDTH-1:0] rom_data;
    reg  [3:0] rom_x, rom_y;
    input_image_rom u_image (
        .clk(clk),
        .x(rom_x),
        .y(rom_y),
        .data_out(rom_data)
    );

    // Kernel storage
    reg [KERNEL_WIDTH-1:0] kernel_mem [0:8];
    reg [3:0] kernel_count;

    // FSM
    localparam ST_IDLE = 3'd0;
    localparam ST_LOAD = 3'd1;
    localparam ST_CALC = 3'd2;
    localparam ST_EMIT = 3'd3;
    localparam ST_DONE = 3'd4;

    reg [2:0] state;

    // Loop counters
    reg [3:0] base_row;
    reg [3:0] base_col;

    // Accumulation pipeline
    reg [ACC_WIDTH-1:0] acc;
    reg [ACC_WIDTH-1:0] acc_hold;
    reg [3:0] k_ptr_issue;
    reg [3:0] k_ptr_issue_q;
    reg [3:0] accum_count;
    reg       addr_valid;
    reg       addr_valid_q;

    // Use rom_data directly. With pre-calculation logic:
    // Cycle T: rom_x updates to Addr(k+1).
    // Cycle T+1: Data(k) (from previous rom_x) is valid. k_ptr_issue_q is k.
    wire [ACC_WIDTH-1:0] mul_prod = rom_data * kernel_mem[k_ptr_issue_q];

    // Sequential logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_IDLE;
            kernel_ready  <= 0;
            kernel_count  <= 0;
            busy          <= 0;
            done          <= 0;
            out_valid     <= 0;
            out_elem      <= 0;
            out_row_end   <= 0;
            out_last      <= 0;
            out_row_idx   <= 0;
            out_col_idx   <= 0;
            cycle_count   <= 0;
            base_row      <= 0;
            base_col      <= 0;
            acc           <= 0;
            acc_hold      <= 0;
            k_ptr_issue   <= 0;
            k_ptr_issue_q <= 0;
            accum_count   <= 0;
            addr_valid    <= 0;
            addr_valid_q  <= 0;
            rom_x         <= 0;
            rom_y         <= 0;
        end else begin
            // defaults
            out_valid  <= 0;
            out_row_end<= 0;
            out_last   <= 0;
            kernel_ready <= 0;

            // pipeline capture
            addr_valid_q  <= addr_valid;
            k_ptr_issue_q <= k_ptr_issue;

            case (state)
                ST_IDLE: begin
                    busy         <= 0;
                    done         <= 0;
                    cycle_count  <= 0;
                    if (start) begin
                        kernel_count <= 0;
                        state        <= ST_LOAD;
                    end
                end

                ST_LOAD: begin
                    kernel_ready <= 1;
                    if (kernel_valid && kernel_count < 9) begin
                        kernel_mem[kernel_count] <= kernel_in;
                        kernel_count <= kernel_count + 1;
                        if (kernel_count == 8) begin
                            // Pre-calculate Address 0 for the first pixel
                            base_row    <= 0;
                            base_col    <= 0;
                            rom_x       <= 0; // base_row + 0
                            rom_y       <= 0; // base_col + 0
                            
                            acc         <= 0;
                            accum_count <= 0;
                            k_ptr_issue <= 0;
                            addr_valid  <= 1; // Enable read for k=0 immediately
                            busy        <= 1;
                            state       <= ST_CALC;
                        end
                    end
                end

                ST_CALC: begin
                    busy        <= 1;
                    cycle_count <= cycle_count + 1;

                    // Issue next address (k+1)
                    if (k_ptr_issue < 9) begin
                        // Calculate address for NEXT kernel index (k+1)
                        // Current rom_x (set in prev cycle) corresponds to k_ptr_issue
                        rom_x       <= base_row + ((k_ptr_issue + 1) / K_SIZE);
                        rom_y       <= base_col + ((k_ptr_issue + 1) % K_SIZE);
                        addr_valid  <= 1;
                        k_ptr_issue <= k_ptr_issue + 1;
                    end else begin
                        addr_valid <= 0;
                    end

                    // Accumulate when data returns from ROM (1 cycle latency from rom_x update)
                    if (addr_valid_q) begin
                        acc <= acc + mul_prod;
                        accum_count <= accum_count + 1;
                        if (accum_count == 4'd8) begin
                            acc_hold <= acc + mul_prod; // final sum
                            state    <= ST_EMIT;
                        end
                    end
                end

                ST_EMIT: begin
                    busy        <= 1;
                    cycle_count <= cycle_count + 1;

                    out_valid   <= 1;
                    out_elem    <= acc_hold;
                    out_row_idx <= base_row;
                    out_col_idx <= base_col;
                    out_row_end <= (base_col == OUT_N-1);
                    out_last    <= (base_row == OUT_M-1) && (base_col == OUT_N-1);

                    // Advance output position
                    if (base_col == OUT_N-1) begin
                        base_col <= 0;
                        base_row <= base_row + 1;
                        
                        // Pre-calculate Address 0 for NEXT pixel
                        if (base_row < OUT_M-1) begin
                            rom_x <= (base_row + 1); // + 0/3
                            rom_y <= 0;              // + 0%3
                        end
                    end else begin
                        base_col <= base_col + 1;
                        
                        // Pre-calculate Address 0 for NEXT pixel
                        rom_x <= base_row;           // + 0/3
                        rom_y <= (base_col + 1);     // + 0%3
                    end

                    // Prepare next accumulation or finish
                    if ((base_row == OUT_M-1) && (base_col == OUT_N-1)) begin
                        busy        <= 0;
                        done        <= 1;
                        state       <= ST_DONE;
                    end else begin
                        acc         <= 0;
                        accum_count <= 0;
                        k_ptr_issue <= 0;
                        addr_valid  <= 1; // Enable read for k=0 of next pixel
                        state       <= ST_CALC;
                    end
                end

                ST_DONE: begin
                    // Hold cycle_count for external display until next start
                    if (start) begin
                        done         <= 0;
                        kernel_count <= 0;
                        cycle_count  <= 0;
                        state        <= ST_LOAD;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule

// Hard-coded 10x12 image ROM (values 0-9). Uses synchronous read.
module input_image_rom(
    input  wire       clk,
    input  wire [3:0] x, // row address (0-9)
    input  wire [3:0] y, // col address (0-11)
    output reg  [3:0] data_out
);
    reg [3:0] rom [0:119];

    initial begin
        // Row 0: 372905184632
        rom[0]=4'd3; rom[1]=4'd7; rom[2]=4'd2; rom[3]=4'd9; rom[4]=4'd0; rom[5]=4'd5; rom[6]=4'd1; rom[7]=4'd8; rom[8]=4'd4; rom[9]=4'd6; rom[10]=4'd3; rom[11]=4'd2;
        // Row 1: 816473905281
        rom[12]=4'd8; rom[13]=4'd1; rom[14]=4'd6; rom[15]=4'd4; rom[16]=4'd7; rom[17]=4'd3; rom[18]=4'd9; rom[19]=4'd0; rom[20]=4'd5; rom[21]=4'd2; rom[22]=4'd8; rom[23]=4'd1;
        // Row 2: 490268357149
        rom[24]=4'd4; rom[25]=4'd9; rom[26]=4'd0; rom[27]=4'd2; rom[28]=4'd6; rom[29]=4'd8; rom[30]=4'd3; rom[31]=4'd5; rom[32]=4'd7; rom[33]=4'd1; rom[34]=4'd4; rom[35]=4'd9;
        // Row 3: 738514920673
        rom[36]=4'd7; rom[37]=4'd3; rom[38]=4'd8; rom[39]=4'd5; rom[40]=4'd1; rom[41]=4'd4; rom[42]=4'd9; rom[43]=4'd2; rom[44]=4'd0; rom[45]=4'd6; rom[46]=4'd7; rom[47]=4'd3;
        // Row 4: 264087531924
        rom[48]=4'd2; rom[49]=4'd6; rom[50]=4'd4; rom[51]=4'd0; rom[52]=4'd8; rom[53]=4'd7; rom[54]=4'd5; rom[55]=4'd3; rom[56]=4'd1; rom[57]=4'd9; rom[58]=4'd2; rom[59]=4'd4;
        // Row 5: 907352864190
        rom[60]=4'd9; rom[61]=4'd0; rom[62]=4'd7; rom[63]=4'd3; rom[64]=4'd5; rom[65]=4'd2; rom[66]=4'd8; rom[67]=4'd6; rom[68]=4'd4; rom[69]=4'd1; rom[70]=4'd9; rom[71]=4'd0;
        // Row 6: 581649273058
        rom[72]=4'd5; rom[73]=4'd8; rom[74]=4'd1; rom[75]=4'd6; rom[76]=4'd4; rom[77]=4'd9; rom[78]=4'd2; rom[79]=4'd7; rom[80]=4'd3; rom[81]=4'd0; rom[82]=4'd5; rom[83]=4'd8;
        // Row 7: 149270685314
        rom[84]=4'd1; rom[85]=4'd4; rom[86]=4'd9; rom[87]=4'd2; rom[88]=4'd7; rom[89]=4'd0; rom[90]=4'd6; rom[91]=4'd8; rom[92]=4'd5; rom[93]=4'd3; rom[94]=4'd1; rom[95]=4'd4;
        // Row 8: 625831749062
        rom[96]=4'd6; rom[97]=4'd2; rom[98]=4'd5; rom[99]=4'd8; rom[100]=4'd3; rom[101]=4'd1; rom[102]=4'd7; rom[103]=4'd4; rom[104]=4'd9; rom[105]=4'd0; rom[106]=4'd6; rom[107]=4'd2;
        // Row 9: 073956418207
        rom[108]=4'd0; rom[109]=4'd7; rom[110]=4'd3; rom[111]=4'd9; rom[112]=4'd5; rom[113]=4'd6; rom[114]=4'd4; rom[115]=4'd1; rom[116]=4'd8; rom[117]=4'd2; rom[118]=4'd0; rom[119]=4'd7;
    end

    wire [6:0] addr = x * 7'd12 + y;

    always @(posedge clk) begin
        data_out <= rom[addr];
    end
endmodule
