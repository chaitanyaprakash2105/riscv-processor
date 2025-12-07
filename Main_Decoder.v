`default_nettype none

module Main_Decoder (
    input  wire [6:0] Op,
    
    output reg        RegWrite,
    output reg [2:0]  ImmSrc,
    output reg        ALUSrc,
    output reg        MemWrite,
    output reg [1:0]  ResultSrc,
    output reg        Branch,
    output reg        Jump,
    output reg [1:0]  ALUOp
);

    always @(*) begin
        
        // --- STEP 1: Default all signals to a 'nop' ---
        RegWrite  = 1'b0;
        ImmSrc    = 3'b000; // I-type
        ALUSrc    = 1'b0;   // Reg (rs2)
        MemWrite  = 1'b0;
        ResultSrc = 2'b00;  // ALU Result
        Branch    = 1'b0;
        Jump      = 1'b0;
        ALUOp     = 2'b00;  // ADD

        // --- STEP 2: Override defaults based on opcode ---
        case (Op)
            
            7'b0110011: begin // R-type
                RegWrite  = 1'b1;
                ALUOp     = 2'b10;
            end

            7'b0010011: begin // I-type (addi, slti, andi, etc.)
                RegWrite  = 1'b1;
                ALUSrc    = 1'b1;
                ALUOp     = 2'b10; // <-- *** THIS WAS THE BUG ***
                ImmSrc    = 3'b000; // I-type immediate
            end

            7'b0000011: begin // Load (lw)
                RegWrite  = 1'b1;
                ALUSrc    = 1'b1;
                ResultSrc = 2'b01; // Data from memory
                ALUOp     = 2'b00; // ADD for address
                ImmSrc    = 3'b000; // I-type immediate
            end

            7'b0100011: begin // Store (sw)
                ALUSrc    = 1'b1;
                MemWrite  = 1'b1;
                ImmSrc    = 3'b001; // S-type immediate
                ALUOp     = 2'b00; // ADD for address
            end
            
            7'b1100011: begin // Branch (beq, bne, etc.)
                Branch    = 1'b1;
                ImmSrc    = 3'b010; // B-type immediate
                ALUOp     = 2'b01; // SUB for compare
            end

            7'b0110111: begin // LUI
                RegWrite  = 1'b1;
                ALUSrc    = 1'b1;
                ImmSrc    = 3'b011; // U-type immediate
                ALUOp     = 2'b00; // ADD (will be 0 + imm)
            end

            7'b1101111: begin // JAL
                RegWrite  = 1'b1;
                ResultSrc = 2'b10; // PC + 4
                Jump      = 1'b1;
                ImmSrc    = 3'b100; // J-type immediate
            end

            7'b1100111: begin // JALR
                RegWrite  = 1'b1;
                ALUSrc    = 1'b1;
                ResultSrc = 2'b10; // PC + 4
                Jump      = 1'b1;
                ImmSrc    = 3'b000; // I-type immediate
            end

            7'b0010111: begin // AUIPC
                RegWrite  = 1'b1;
                ALUSrc    = 1'b1;
                ImmSrc    = 3'b011; // U-type immediate
                ALUOp     = 2'b00;  // ADD (will be PC + imm)
            end
            
            // 7'b1110011: // SYSTEM (ebreak)
            // No signals set, will be handled by 'ValidD'
            
            default: begin
                // All signals remain at their default 'nop' values
            end

        endcase
    end
endmodule

`default_nettype wire