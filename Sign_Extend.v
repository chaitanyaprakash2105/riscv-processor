`default_nettype none

module Sign_Extend (
    input  wire [31:0] In,
    input  wire [2:0]  ImmSrc,  // <-- CHANGED: Now 3 bits
    output reg  [31:0] Imm_Ext   // <-- CHANGED: Now a 'reg'
);

    // Use an always_comb block to generate the immediate value
    always @(*) begin
        case (ImmSrc)
            
            // 3'b000: I-type (addi, jalr, lw)
            3'b000: Imm_Ext = {{20{In[31]}}, In[31:20]};
            
            // 3'b001: S-type (sw)
            3'b001: Imm_Ext = {{20{In[31]}}, In[31:25], In[11:7]};
            
            // 3'b010: B-type (beq)
            3'b010: Imm_Ext = {{20{In[31]}}, In[7], In[30:25], In[11:8], 1'b0};
            
            // 3'b011: U-type (lui)
            3'b011: Imm_Ext = {In[31:12], 12'b0};
            
            // 3'b100: J-type (jal)
            3'b100: Imm_Ext = {{12{In[31]}}, In[19:12], In[20], In[30:21], 1'b0};
            
            // Default to 0 for any other (invalid) ImmSrc value
            default: Imm_Ext = 32'h00000000;
            
        endcase
    end

endmodule

`default_nettype wire