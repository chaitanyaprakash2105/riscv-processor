`default_nettype none

/**
 * Program Counter (PC) Register (Modern Verilog-2001 Style)
 *
 * This version uses an ANSI-style port header to fix the
 * `default_nettype none` compilation errors.
 */
module PC_Module (
    // Inputs
    input  wire        clk,
    input  wire        rst,       // Active-high reset
    input  wire [31:0] PC_Next, // The next PC value
    // Parameterize reset value to match hart RESET_ADDR
    input  wire [31:0] PC_Reset_Value,

    // Output
    // 'PC' must be 'output reg' because it is assigned
    // inside an always @(posedge clk) block.
    output reg  [31:0] PC        // The current PC value
);

    // Asynchronous reset to align with other pipeline regs using async reset
    always @(posedge clk or posedge rst) begin
        if (rst == 1'b1)
            PC <= PC_Reset_Value;
        else
            PC <= PC_Next;
    end
    
endmodule

`default_nettype wire
