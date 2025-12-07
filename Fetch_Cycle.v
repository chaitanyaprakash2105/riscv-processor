`default_nettype none

module fetch_cycle #(
    parameter RESET_ADDR = 32'h00000000
) (
    input  wire        clk,
    input  wire        rst,
    // Branch / jump decision from EX stage
    input  wire        PCSrcE,
    input  wire [31:0] PCTargetE,
    // Pipeline stall controls
    input  wire        StallF,   // Stall PC update (load-use or cache stall)
    input  wire        StallD,   // Stall IF/ID register
    // Instruction memory data (synchronous)
    input  wire [31:0] i_imem_rdata,

    // Instruction memory address
    output wire [31:0] o_imem_raddr,
    // Outputs to Decode stage
    output wire [31:0] InstrD,
    output wire [31:0] PCD,
    output wire [31:0] PCPlus4D
);
    // Program counter
    wire [31:0] PCF;
    wire [31:0] PCPlus4F = PCF + 4;

    // When a branch/jump resolves while the front end is stalled (typically
    // on an I-cache miss), remember its target and apply it once the stall
    // clears. This ensures we don't lose taken branches like
    // `bne x0, gp, pass` in 08ori, while still keeping the request address
    // stable during the stall.
    reg        pending_branch;
    reg [31:0] pending_target;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pending_branch <= 1'b0;
            pending_target <= RESET_ADDR;
        end else begin
            // Latch a pending branch target if one resolves while stalled.
            if (PCSrcE && StallF)
                {pending_branch, pending_target} <= {1'b1, PCTargetE};
            // Once stall clears, the pending target will be consumed in
            // the next PC update, so we can drop the flag.
            else if (!StallF && pending_branch)
                pending_branch <= 1'b0;
        end
    end

    wire use_pending_target = pending_branch && !StallF;

    // PC update logic:
    //  - If stalled, hold PC (keep request address stable).
    //  - Else, if a pending branch exists, jump to that target.
    //  - Else, if a branch/jump was just resolved in EX, use its target.
    //  - Otherwise, fall through to PC+4.
    wire [31:0] PC_Next =
        StallF           ? PCF :
        use_pending_target ? pending_target :
        PCSrcE           ? PCTargetE :
                           PCPlus4F;

    PC_Module Program_Counter (
        .clk            (clk),
        .rst            (rst),
        .PC             (PCF),
        .PC_Next        (PC_Next),
        .PC_Reset_Value (RESET_ADDR)
    );

    assign o_imem_raddr = PCF;

    // IF/ID pipeline registers
    reg [31:0] InstrD_reg;
    reg [31:0] PCD_reg;
    reg [31:0] PCPlus4D_reg;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            InstrD_reg   <= 32'h00000000;
            PCD_reg      <= RESET_ADDR;
            PCPlus4D_reg <= RESET_ADDR + 4;
        end else begin
            // On a taken branch / jump, insert a bubble into Decode.
            // This flushes the wrong-path instruction currently in IF/ID.
            if (PCSrcE) begin
                InstrD_reg   <= 32'h00000000;
                PCD_reg      <= 32'h00000000;
                PCPlus4D_reg <= 32'h00000000;
            end
            // Load-use hazard or cache stall: hold IF/ID stable.
            else if (StallD) begin
                InstrD_reg   <= InstrD_reg;
                PCD_reg      <= PCD_reg;
                PCPlus4D_reg <= PCPlus4D_reg;
            end
            // Normal flow: latch freshly fetched instruction and PC.
            else begin
                InstrD_reg   <= i_imem_rdata;
                PCD_reg      <= PCF;
                PCPlus4D_reg <= PCPlus4F;
            end
        end
    end

    assign InstrD   = InstrD_reg;
    assign PCD      = PCD_reg;
    assign PCPlus4D = PCPlus4D_reg;

endmodule

`default_nettype wire
