

module BEAN_1(clk, reset, data_mem_IN, data_mem_OUT, mem_addrs, mem_MODE, mem_WE);

  input wire  clk, reset;
  input wire [31:0]   data_mem_IN;

  output wire         mem_WE;
  output wire [2:0]   mem_MODE;
  output wire [31:0]  mem_addrs, data_mem_OUT;

  wire          jump;
  wire          reg_WE;
  wire          rs1_SEL;
  wire          rs2_SEL;
  wire          addrs_SEL;
  wire          pc_EN;
  wire          instr_EN;
  wire          ALU_mem_EN;
  wire          mem_in_EN;
  wire [1:0]    reg_SEL,  pc_SEL;
  wire [2:0]    imm_SEL;
  wire [3:0]    ALU_MODE;
  wire [31:0]   inst;

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
    .data_mem_IN(data_mem_IN),
    .data_mem_OUT(data_mem_OUT),
    .mem_addrs(mem_addrs),
    .instr(inst),
    .jump(jump)
    );

endmodule


