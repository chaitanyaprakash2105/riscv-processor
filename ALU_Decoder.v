`default_nettype none

/**
 * ALU Decoder
 *
 * This module decodes the ALUOp, funct3, and funct7 fields
 * to generate the final 3-bit ALUControl signal for the ALU.
 *
 * ALUControl Map:
 * 3'b000: ADD
 * 3'b001: SUB
 * 3'b010: AND
 * 3'b011: OR
 * 3'b100: XOR
 * 3'b101: SLL (Shift Left)
 * 3'b110: SRL / SRA (Shift Right)
 * 3'b111: SLT / SLTU (Set Less Than)
 */
module ALU_Decoder (
    input  wire [1:0] ALUOp,
    input  wire [2:0] funct3,
    input  wire [6:0] funct7,
    input  wire [6:0] op,
    
    output reg [2:0] ALUControl
);

    // Wires to check for specific instruction bits
    wire is_R_type = (op[6:0] == 7'b0110011);
    wire is_I_type = (op[6:0] == 7'b0010011);
    wire funct7_5  = funct7[5]; // Distinguishes SUB/SRA
    wire imm_10    = funct7[5]; // (funct7 is just instr[31:25])

    always @(*) begin
        case (ALUOp)
            
            2'b00: ALUControl = 3'b000; // Load/Store/LUI: ADD
            
            2'b01: ALUControl = 3'b001; // Branch: SUB
            
            2'b10: begin // R-type or I-type
                case (funct3)
                    // ADD, SUB, ADDI
                    3'b000: begin
                        // Check for SUB (R-type) or ADDI (I-type)
                        if ((is_R_type && funct7_5))
                            ALUControl = 3'b001; // SUB
                        else
                            ALUControl = 3'b000; // ADD or ADDI
                    end
                    
                    // SLL, SLLI
                    3'b001: ALUControl = 3'b101;
                    
                    // SLT, SLTI
                    3'b010: ALUControl = 3'b111;
                    
                    // SLTU, SLTIU
                    3'b011: ALUControl = 3'B111; // Share with SLT, ALU will differentiate
                    
                    // XOR, XORI
                    3'b100: ALUControl = 3'b100;
                    
                    // SRL, SRLI, SRA, SRAI
                    3'b101: ALUControl = 3'b110;
                    
                    // OR, ORI
                    3'b110: ALUControl = 3'b011;
                    
                    // AND, ANDI
                    3'b111: ALUControl = 3'b010;
                    
                    default: ALUControl = 3'b000; // Default to ADD
                endcase
            end
            
            default: ALUControl = 3'b000; // Default to ADD
            
        endcase
    end

endmodule

`default_nettype wire