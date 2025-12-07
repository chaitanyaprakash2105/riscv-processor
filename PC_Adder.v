`default_nettype none

/**
 * 32-bit Adder (Modern Verilog-2001 Style)
 *
 * This version uses an ANSI-style port header to fix the
 * `default_nettype none` compilation errors.
 */
module PC_Adder (
    // Inputs
    input  wire [31:0] a,
    input  wire [31:0] b,

    // Output
    output wire [31:0] c
);

    // Combinational assignment
    assign c = a + b;
    
endmodule

`default_nettype wire