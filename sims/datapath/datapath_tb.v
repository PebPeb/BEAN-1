

module control_logic_tb();
  reg clk;
  reg reset;

  reg           reg_WE, rs1_SEL, rs2_SEL;
  reg           addrs_SEL, pc_EN, instr_EN;
  reg           ALU_mem_EN, mem_in_EN, mem_WE;
  reg [1:0]     reg_SEL, pc_SEL;
  reg [2:0]     imm_SEL, mem_MODE;
  reg [3:0]     ALU_MODE;

  wire [31:0]   data_mem_READ, data_mem_WRITE, mem_addrs;

  initial clk <= 0;
	always #1 clk <= ~clk;
	
  datapath UUT(
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
    .data_mem_IN(data_mem_READ),
    .data_mem_OUT(data_mem_WRITE),
    .mem_addrs(mem_addrs) 
    );

  mem memory_Unit(
    .a(mem_addrs), 
    .rd(data_mem_READ), 
    .wd(data_mem_WRITE), 
    .clk(clk), 
    .we(mem_WE), 
    .mode(mem_MODE) 
  );


  initial begin
    reset <= 1'b1;
    pc_SEL <= 2'b00;

    #2    // Updating Control logic on the negative edge
  
    // Instruction 1 
    // addi       x10,x0,8
    // 0x00800513

    // Fetch 
    reset <= 1'b0;
    reg_WE <= 1'b0;
    rs1_SEL <= 1'bX;
    rs2_SEL <= 1'bX;
    reg_SEL <= 2'bXX;
    pc_SEL <= 2'bXX;
    imm_SEL <= 3'bXXX;
    ALU_MODE <= 4'bXXXX;
    addrs_SEL <= 1'b1;
    pc_EN <= 1'b0;
    instr_EN <= 1'b1;
    ALU_mem_EN <= 1'b0;
    mem_in_EN <= 1'b1;
    mem_WE <= 1'b0;
    mem_MODE <= 3'b000;

    #2

    // Execute
    reset <= 1'b0;
    reg_WE <= 1'b1;
    rs1_SEL <= 1'b0;
    rs2_SEL <= 1'b1;
    reg_SEL <= 2'b01;
    pc_SEL <= 2'b00;
    imm_SEL <= 3'b011;
    ALU_MODE <= 4'b0000;
    addrs_SEL <= 1'b0;
    pc_EN <= 1'b1;
    instr_EN <= 1'b0;
    ALU_mem_EN <= 1'b0;
    mem_in_EN <= 1'b0;
    mem_WE <= 1'b0;
    mem_MODE <= 3'bXXX;

    #2

    // Instruction 2 
    // addi       x11,x0,16
    // 0x01000593       

    // Fetch 
    reset <= 1'b0;
    reg_WE <= 1'b0;
    rs1_SEL <= 1'bX;
    rs2_SEL <= 1'bX;
    reg_SEL <= 2'bXX;
    pc_SEL <= 2'bXX;
    imm_SEL <= 3'bXXX;
    ALU_MODE <= 4'bXXXX;
    addrs_SEL <= 1'b1;
    pc_EN <= 1'b0;
    instr_EN <= 1'b1;
    ALU_mem_EN <= 1'b0;
    mem_in_EN <= 1'b1;
    mem_WE <= 1'b0;
    mem_MODE <= 3'b000;

    #2

    // Execute
    reset <= 1'b0;
    reg_WE <= 1'b1;
    rs1_SEL <= 1'b0;
    rs2_SEL <= 1'b1;
    reg_SEL <= 2'b01;
    pc_SEL <= 2'b00;
    imm_SEL <= 3'b011;
    ALU_MODE <= 4'b0000;
    addrs_SEL <= 1'b0;
    pc_EN <= 1'b1;
    instr_EN <= 1'b0;
    ALU_mem_EN <= 1'b0;
    mem_in_EN <= 1'b0;
    mem_WE <= 1'b0;
    mem_MODE <= 3'bXXX;

    #2

    // Instruction 3
    // add        x11,x10,x11
    // 0x00b505b3       

    // Fetch
    reset <= 1'b0;
    reg_WE <= 1'b0;
    rs1_SEL <= 1'bX;
    rs2_SEL <= 1'bX;
    reg_SEL <= 2'bXX;
    pc_SEL <= 2'bXX;
    imm_SEL <= 3'bXXX;
    ALU_MODE <= 4'bXXXX;
    addrs_SEL <= 1'b1;
    pc_EN <= 1'b0;
    instr_EN <= 1'b1;
    ALU_mem_EN <= 1'b0;
    mem_in_EN <= 1'b1;
    mem_WE <= 1'b0;
    mem_MODE <= 3'b000;

    #2

    // Execute
    reset <= 1'b0;
    reg_WE <= 1'b1;
    rs1_SEL <= 1'b0;
    rs2_SEL <= 1'b0;
    reg_SEL <= 2'b01;
    pc_SEL <= 2'b00;
    imm_SEL <= 3'bXXX;
    ALU_MODE <= 4'b0000;
    addrs_SEL <= 1'b0;
    pc_EN <= 1'b1;
    instr_EN <= 1'b0;
    ALU_mem_EN <= 1'b0;
    mem_in_EN <= 1'b0;
    mem_WE <= 1'b0;
    mem_MODE <= 3'bXXX;

    #2


    // Instruction 4
    // sw         x11,0(x10)     
    // 0x00b52023       

    // Fetch
    reset <= 1'b0;
    reg_WE <= 1'b0;
    rs1_SEL <= 1'bX;
    rs2_SEL <= 1'bX;
    reg_SEL <= 2'bXX;
    pc_SEL <= 2'bXX;
    imm_SEL <= 3'bXXX;
    ALU_MODE <= 4'bXXXX;
    addrs_SEL <= 1'b1;
    pc_EN <= 1'b0;
    instr_EN <= 1'b1;
    ALU_mem_EN <= 1'b0;
    mem_in_EN <= 1'b1;
    mem_WE <= 1'b0;
    mem_MODE <= 3'b000;

    #2

    // Execute
    reset <= 1'b0;
    reg_WE <= 1'b0;
    rs1_SEL <= 1'b0;
    rs2_SEL <= 1'b1;
    reg_SEL <= 2'bXX;
    pc_SEL <= 2'b00;
    imm_SEL <= 3'b001;
    ALU_MODE <= 4'b0000;
    addrs_SEL <= 1'b0;
    pc_EN <= 1'b1;
    instr_EN <= 1'b0;
    ALU_mem_EN <= 1'b1;
    mem_in_EN <= 1'b0;
    mem_WE <= 1'b1;
    mem_MODE <= 3'b000;

    #2

    // Fetch
    reset <= 1'b0;
    reg_WE <= 1'b0;
    rs1_SEL <= 1'bX;
    rs2_SEL <= 1'bX;
    reg_SEL <= 2'bXX;
    pc_SEL <= 2'bXX;
    imm_SEL <= 3'bXXX;
    ALU_MODE <= 4'bXXXX;
    addrs_SEL <= 1'b1;
    pc_EN <= 1'b0;
    instr_EN <= 1'b1;
    ALU_mem_EN <= 1'b0;
    mem_in_EN <= 1'b1;
    mem_WE <= 1'b0;
    mem_MODE <= 3'b000;

    #2

    $finish;
  end

  integer idx;
  initial begin
    $dumpfile("datapath_tb.vcd");
    //$dumpvars(0, datapath_tb);
    $dumpvars(0, datapath_tb);
    for (idx = 0; idx < 32; idx = idx + 1) begin
      $dumpvars(0, UUT.regFILE.x[idx]);
    end
    for (idx = 0; idx < 256; idx = idx + 1) begin
      $dumpvars(0, memory_Unit.mem[idx]);
    end

  end

endmodule

