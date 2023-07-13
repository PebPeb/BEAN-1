

module BEAN_1(clk, reset);

  input wire  clk, reset;
  output wire [31:0]  mem_addrs
data_mem_READ
data_mem_WRITE
clk
mem_WE
mem_MODE

  control_logic control_unit(
    .inst(inst), 
    .jump(jump), 
    .clk(clk), 
    .reset(reset), 
    .reg_WE(reg_WE), 
    .rs1_SEL(rs1_SEL),
    .rs2_SEL(rs2_SEL), 
    .reg_SEL(reg_SEL), 
    .pc_SEL(pc_SEL), 
    .imm_SEL(imm_SEL),
    .ALU_MODE(ALU_MODE), 
    .addrs_SEL(addrs_SEL), 
    .pc_EN(pc_EN), 
    .instr_EN(instr_EN),
    .ALU_mem_EN(ALU_mem_EN), 
    .mem_in_EN(mem_in_EN), 
    .mem_WE(mem_WE), 
    .mem_MODE(mem_MODE)
    );

  datapath data_unit(
    .clk(), 
    .reset(), 
    .reg_WE(), 
    .rs1_SEL(), 
    .rs2_SEL(), 
    .reg_SEL(), 
    .pc_SEL(), 
    .imm_SEL(), 
    .ALU_MODE(),
    .addrs_SEL(),
    .pc_EN(), 
    .instr_EN(), 
    .ALU_mem_EN(), 
    .mem_in_EN(), 
    .data_mem_IN(),
    .data_mem_OUT(),
    .mem_addrs()
    );

endmodule


