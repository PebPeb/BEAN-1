
//
//	datapath.v
//		RV32I datapath
//    This datapath is for the BEAN-1.
//

// -------------------------------- //
//	By: Bryce Keen	
//	Created: 07/08/2023
// -------------------------------- //
//	Last Modified: 07/09/2023


module datapath(clk, 
                reset, 
                
                reg_WE, 
                rs1_SEL, 
                rs2_SEL, 
                reg_SEL, 
                pc_SEL, 
                imm_SEL, 
                ALU_MODE,
                addrs_SEL,

                pc_EN, 
                instr_EN, 
                ALU_mem_EN, 
                mem_in_EN, 
                
                data_mem_IN,
                data_mem_OUT,
                mem_addrs
                );

    input clk, reset;

    input reg_WE;
    input rs1_SEL, rs2_SEL;
    input addrs_SEL;
    input [1:0] reg_SEL, pc_SEL;
    input [2:0] imm_SEL;
    input [3:0] ALU_MODE;
    input [31:0] data_mem_IN;

    output [31:0]   data_mem_OUT, mem_addrs; 

    wire [31:0]     data_bus;

    wire [31:0]     Instr;
    wire [31:0]     PC_now, PC_next, pc;
    wire [31:0]     rdout1, rdout2, wrs3;
    wire [31:0]     ExtImm, muxrs1, muxrs2;
    wire [31:0]     ALUResults;
    wire [31:0]     pcplus4, pcPlusImm;


    flopren #(.WIDTH(32)) REG_instr (
        .d(data_bus), 
        .q(Instr), 
        .clk(clk), 
        .enable(instr_EN),
        .reset(reset));

    regfile regFILE (
        .rs1(Instr[19:15]),
        .rs2(Instr[24:20]),
        .wrs3(wrs3),
        .rd(Instr[11:7]),
        .we(reg_WE),
        .clk(clk),
        .reset(reset),
        .rdout1(rdout1),
        .rdout2(rdout2));

    flopren #(.WIDTH(32)) REG_pc (
        .d(PC_now), 
        .q(PC_next), 
        .clk(clk), 
        .enable(pc_EN),
        .reset(reset));
    assign pc = PC_next;

    extend EXTND_Imm(
        .Instr(Instr[31:7]), 
        .ImmSrc(imm_SEL), 
        .ExtImm(ExtImm));

    mux2 #(.WIDTH(32)) MUX_rs1 (
        .a(rdout1), 
        .b(pc), 
        .sel(rs1_SEL), 
        .y(muxrs1));

    mux2 #(.WIDTH(32)) MUX_rs2 (
        .a(rdout2), 
        .b(ExtImm), 
        .sel(rs2_SEL), 
        .y(muxrs2));

    alu32 ALU (
        .a(muxrs1), 
        .b(muxrs2), 
        .ALUControl(ALU_MODE), 
        .result(ALUResults));

    adder #(.WIDTH(32)) ADDER_plus4 (
        .a(4), 
        .b(pc), 
        .y(pcplus4));

    mux4 #(.WIDTH(32)) MUX_regfile (
        .a(data_bus),
        .b(ALUResults),
        .c(ExtImm),
        .d(pcplus4),
        .sel(reg_SEL), 
        .y(wrs3));

    mux4 #(.WIDTH(32)) MUX_pc (
        .a(pcplus4),
        .b(ALUResults),
        .c(pcPlusImm),
        .d(32'hXXXXXXXX),
        .sel(pc_SEL), 
        .y(PC_now));

    adder #(.WIDTH(32)) ADDER_Imm (
        .a(pc), 
        .b(ExtImm), 
        .y(pcPlusImm));


    mux2 #(.WIDTH(32)) MUX_mem_addrs (
        .a(rdout2), 
        .b(pc), 
        .sel(addrs_SEL), 
        .y(mem_addrs));

    assign data_mem_OUT = data_bus;

    tri_buf #(.WIDTH(32)) TRIBUF_mem (
      .data_in(data_mem_IN), 
      .data_out(data_bus), 
      .enable(mem_in_EN)
    )

    tri_buf #(.WIDTH(32)) TRIBUF_ALU (
      .data_in(ALUResults), 
      .data_out(data_bus), 
      .enable(ALU_mem_EN)
    )

endmodule





