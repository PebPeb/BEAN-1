

module control_logic_tb();
  reg clk, reset = 0, jump = 0;

  reg [31:0]    inst;

  wire          reg_WE,       rs1_SEL,      rs2_SEL;
  wire          addrs_SEL,    pc_EN,        instr_EN;
  wire          ALU_mem_EN,   mem_in_EN,    mem_WE;
  wire [1:0]    reg_SEL,      pc_SEL;
  wire [2:0]    imm_SEL,      mem_MODE;
  wire [3:0]    ALU_MODE;


  initial clk <= 0;
	always #1 clk <= ~clk;
	
  control_logic UUT(
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
    .mem_MODE(mem_MODE));



  initial begin
    inst <= 32'h00000037;
    #4
    $finish;
  end

  initial begin
    $dumpfile("control_logic_tb.vcd");
    //$dumpvars(0, datapath_tb);
    $dumpvars(0, control_logic_tb);


  end

endmodule

