`default_nettype none

module decode_cycle(
    // --- Inputs ---
    input  wire        clk,
    input  wire        rst,
    input  wire        RegWriteW,
    input  wire [4:0]  RDW,
    input  wire [31:0] InstrD,
    input  wire [31:0] PCD,
    input  wire [31:0] PCPlus4D,
    input  wire [31:0] ResultW,
    input  wire        PCSrcE,
    input wire    Stall,
    output wire        ValidE,

    // --- Outputs ---
    output wire        RegWriteE,
    output wire        ALUSrcE,
    output wire        MemWriteE,
    output wire        BranchE,
    output wire        JumpE,
    output wire [1:0]  ResultSrcE,
    output wire [2:0]  ALUControlE,
    output wire [31:0] RD1_E,
    output wire [31:0] RD2_E,
    output wire [31:0] Imm_Ext_E,
    output wire [4:0]  RS1_E,
    output wire [4:0]  RS2_E,
    output wire [4:0]  RD_E,
    output wire [31:0] PCE,
    output wire [31:0] PCPlus4E,
    output wire [31:0] InstrE,
    // Early JAL redirect (Decode stage)
    output wire        PCSrcD,
    output wire [31:0] PCTargetD
);

    // Declare Interim Wires
    wire        RegWriteD, ALUSrcD, MemWriteD, BranchD, JumpD;
    wire        ValidD; // <-- ***FIX 1: Declared ValidD wire***
    wire [1:0]  ResultSrcD;
    wire [2:0]  ImmSrcD;
    wire [2:0]  ALUControlD;
    wire [31:0] RD1_D, RD2_D, Imm_Ext_D;

    // Declaration of Interim Register (ID/EX Register)
    reg         RegWriteD_r, ALUSrcD_r, MemWriteD_r, BranchD_r, JumpD_r;
    reg  [1:0]  ResultSrcD_r;
    reg  [2:0]  ALUControlD_r;
    reg  [31:0] RD1_D_r, RD2_D_r, Imm_Ext_D_r;
    reg  [4:0]  RD_D_r, RS1_D_r, RS2_D_r;
    reg  [31:0] PCD_r, PCPlus4D_r;
    reg  [31:0] InstrD_r;
    reg         ValidD_r;

    wire [2:0] funct3;
    assign funct3 = InstrD[14:12];

    // Initiate the modules
    // Control Unit
    Control_Unit_Top control (
        .Op(InstrD[6:0]),
        .funct3(InstrD[14:12]),
        .funct7(InstrD[31:25]),
        .RegWrite(RegWriteD),
        .ImmSrc(ImmSrcD),
        .ALUSrc(ALUSrcD),
        .MemWrite(MemWriteD),
        .ResultSrc(ResultSrcD),
        .Branch(BranchD),
        .Jump(JumpD),
        .ALUControl(ALUControlD)
    );

    // This wire is 1 if the instruction is not a NOP (i.e., it does something)
    assign ValidD = RegWriteD | MemWriteD | BranchD | JumpD | (InstrD == 32'h00100073); // Valid if ebreak

    // Register File
    Register_File rf (
        .clk(clk),
        .rst(rst),
        .WE3(RegWriteW),
        .WD3(ResultW),
        .A1(InstrD[19:15]),
        .A2(InstrD[24:20]),
        .A3(RDW),
        .RD1(RD1_D),
        .RD2(RD2_D)
    );

    // Sign Extension
    Sign_Extend extension (
        .In(InstrD[31:0]),
        .Imm_Ext(Imm_Ext_D),
        .ImmSrc(ImmSrcD)
    );


    // Declaring Register Logic (ID/EX Pipeline Register)
    always @(posedge clk or posedge rst) begin
        if (rst == 1'b1) begin
            RegWriteD_r  <= 1'b0;
            ALUSrcD_r    <= 1'b0;
            MemWriteD_r  <= 1'b0;
            ResultSrcD_r <= 2'b00;
            BranchD_r    <= 1'b0;
            JumpD_r      <= 1'b0;
            ALUControlD_r <= 3'b000;
            RD1_D_r      <= 32'h00000000; 
            RD2_D_r      <= 32'h00000000; 
            Imm_Ext_D_r  <= 32'h00000000;
            RD_D_r       <= 5'h00;
            PCD_r        <= 32'h00000000; 
            PCPlus4D_r   <= 32'h00000000;
            RS1_D_r      <= 5'h00;
            RS2_D_r      <= 5'h00;
            InstrD_r     <= 32'h00000000;
            ValidD_r     <= 1'b0; // Reset to 0 (invalid)
        end
        // *** FLUSH LOGIC HAS PRIORITY OVER STALL ***
        else if (PCSrcE) begin
            // Insert a NOP bubble
            RegWriteD_r  <= 1'b0;
            ALUSrcD_r    <= 1'b0;
            MemWriteD_r  <= 1'b0;
            ResultSrcD_r <= 2'b00;
            BranchD_r    <= 1'b0;
            JumpD_r      <= 1'b0;
            ALUControlD_r <= 3'b000;
            RD1_D_r      <= 32'h00000000; 
            RD2_D_r      <= 32'h00000000; 
            Imm_Ext_D_r  <= 32'h00000000;
            RD_D_r       <= 5'h00;
            PCD_r        <= 32'h00000000; 
            PCPlus4D_r   <= 32'h00000000;
            RS1_D_r      <= 5'h00;
            RS2_D_r      <= 5'h00;
            InstrD_r     <= 32'h0; // NOP
            ValidD_r     <= 1'b0;         // This is an invalid bubble
        end
        else if (Stall) begin
            // Freeze all registers (Keep previous value)
            RegWriteD_r   <= RegWriteD_r;
            ALUSrcD_r     <= ALUSrcD_r;
            MemWriteD_r   <= MemWriteD_r;
            ResultSrcD_r  <= ResultSrcD_r;
            BranchD_r     <= BranchD_r;
            JumpD_r       <= JumpD_r;
            ALUControlD_r <= ALUControlD_r;
            RD1_D_r       <= RD1_D_r;
            RD2_D_r       <= RD2_D_r;
            Imm_Ext_D_r   <= Imm_Ext_D_r;
            RD_D_r        <= RD_D_r;
            PCD_r         <= PCD_r;
            PCPlus4D_r    <= PCPlus4D_r;
            RS1_D_r       <= RS1_D_r;
            RS2_D_r       <= RS2_D_r;
            InstrD_r      <= InstrD_r;
            ValidD_r      <= ValidD_r;
        end
        else begin
            RegWriteD_r  <= RegWriteD;
            ALUSrcD_r    <= ALUSrcD;
            MemWriteD_r  <= MemWriteD;
            ResultSrcD_r <= ResultSrcD;
            BranchD_r    <= BranchD;
            JumpD_r      <= JumpD;
            ALUControlD_r <= ALUControlD;
            RD1_D_r      <= RD1_D; 
            RD2_D_r      <= RD2_D; 
            Imm_Ext_D_r  <= Imm_Ext_D;
            RD_D_r       <= InstrD[11:7];
            PCD_r        <= PCD; 
            PCPlus4D_r   <= PCPlus4D;
            RS1_D_r      <= InstrD[19:15];
            RS2_D_r      <= InstrD[24:20];
            InstrD_r     <= InstrD;
            ValidD_r     <= ValidD; // <-- ***FIX 2: Use the calculated ValidD***
        end
    end

    // Early redirect generation (Decode) for JAL and unconditional BEQ x0,x0
    wire [6:0] opcode_D = InstrD[6:0];
    wire        is_jal_D     = (opcode_D == 7'b1101111);
    wire        is_branch_D  = (opcode_D == 7'b1100011);
    wire        is_beq_x0x0  = is_branch_D & (InstrD[14:12] == 3'b000) &
                               (InstrD[19:15] == 5'd0) & (InstrD[24:20] == 5'd0);
    assign PCSrcD    = ValidD & ((JumpD & is_jal_D) | is_beq_x0x0);
    assign PCTargetD = PCD + Imm_Ext_D;       // Target = PCD + imm

    // Output assign statements
    assign RegWriteE   = RegWriteD_r;
    assign ALUSrcE     = ALUSrcD_r;
    assign MemWriteE   = MemWriteD_r;
    assign ResultSrcE  = ResultSrcD_r;
    assign BranchE     = BranchD_r;
    assign JumpE       = JumpD_r;
    assign ALUControlE = ALUControlD_r;
    assign RD1_E       = RD1_D_r;
    assign RD2_E       = RD2_D_r;
    assign Imm_Ext_E   = Imm_Ext_D_r;
    assign RD_E        = RD_D_r;
    assign PCE         = PCD_r;
    assign PCPlus4E    = PCPlus4D_r;
    assign RS1_E       = RS1_D_r;
    assign RS2_E       = RS2_D_r;
    assign InstrE      = InstrD_r;
    assign ValidE      = ValidD_r;
endmodule

`default_nettype wire
