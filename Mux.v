`default_nettype none

/**
 * 2-to-1 32-bit Mux (Modern Verilog-2001 Style)
 */
module Mux (
    input  wire [31:0] a,
    input  wire [31:0] b,
    input  wire        s,
    output wire [31:0] c
);

    assign c = (~s) ? a : b ;
    
endmodule

/**
 * 3-to-1 32-bit Mux (Modern Verilog-2001 Style)
 * Note: A 4-to-1 mux would be more standard.
 */
module Mux_3_by_1 (
    input  wire [31:0] a,
    input  wire [31:0] b,
    input  wire [31:0] c,
    input  wire [1:0]  s,
    output wire [31:0] d
);

    // This combinational logic requires 'd' to be a 'wire'
    assign d = (s == 2'b00) ? a : 
               (s == 2'b01) ? b : 
               (s == 2'b10) ? c : 
               32'h00000000; // Default case for s == 2'b11
    
endmodule

`default_nettype wire