`default_nettype none

module writeback_cycle (
    // Clock and Reset (not used by this combinational logic,
    // but good practice to include if other stages do)
    input  wire        clk,
    input  wire        rst,
    
    // Inputs from MEM/WB Pipeline Register (via memory_cycle)
    input  wire [1:0]  ResultSrcW,   // <-- CHANGED: Now 2 bits
    input  wire        JumpW,        // <-- ADDED: Matches hart.v
    input  wire [31:0] PCPlus4W,
    input  wire [31:0] ALU_ResultW,
    input  wire [31:0] ReadDataW,
    
    // Output to Register File (in Decode Stage)
    output reg  [31:0] ResultW       // <-- CHANGED: Now a 'reg'
);

    // This combinational block is the 3-to-1 MUX
    // It selects the final data to be written to the register file.
    always @(*) begin
        case (ResultSrcW)
            // 2'b00: Write the result from the ALU
            // (Used for R-type, I-type, LUI)
            2'b00: ResultW = ALU_ResultW;
            
            // 2'b01: Write the data from memory
            // (Used for Load instructions)
            2'b01: ResultW = ReadDataW;
            
            // 2'b10: Write the PC+4 value
            // (Used for JAL, JALR)
            2'b10: ResultW = PCPlus4W;
            
            // Default: Fallback to ALU result (prevents latches)
            default: ResultW = ALU_ResultW; 
        endcase
    end

endmodule

`default_nettype wire
