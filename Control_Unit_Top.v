`default_nettype none

module Control_Unit_Top(
    input  wire [6:0] Op,
    input  wire [2:0] funct3,
    input  wire [6:0] funct7, // funct7 is 7 bits, not [6:0]
    
    output wire       RegWrite,
    output wire       ALUSrc,
    output wire       MemWrite,
    output wire       Branch,
    output wire       Jump,       // <-- NEW: For JAL/JALR
    output wire [2:0] ImmSrc,     // <-- CHANGED: Now 3 bits
    output wire [1:0] ResultSrc,  // <-- CHANGED: Now 2 bits
    output wire [2:0] ALUControl
);

    wire [1:0] ALUOp;

    Main_Decoder Main_Decoder (
        .Op(Op),
        .RegWrite(RegWrite),
        .ImmSrc(ImmSrc),
        .MemWrite(MemWrite),
        .ResultSrc(ResultSrc),
        .Branch(Branch),
        .Jump(Jump),         // <-- NEW
        .ALUSrc(ALUSrc),
        .ALUOp(ALUOp)
    );

    ALU_Decoder ALU_Decoder (
        .ALUOp(ALUOp),
        .funct3(funct3),
        .funct7(funct7),
        .op(Op),             // This input seems redundant but keeping it
        .ALUControl(ALUControl)
    );

endmodule

`default_nettype wire