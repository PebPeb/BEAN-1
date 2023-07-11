
// -------------------------------- //
//	By: Bryce Keen	
//	Created: 07/10/2023
// -------------------------------- //
//	Last Modified: 07/10/2023

//
//	control_logic.v
//		RV32I Control Logic
//

module control_logic(
  inst, jump, clk, reset, reg_WE, rs1_SEL,
  rs2_SEL, reg_SEL, pc_SEL, imm_SEL,
  ALU_MODE, addrs_SEL, pc_EN, instr_EN,
  ALU_mem_EN, mem_in_EN, mem_WE, mem_MODE);

  // Input & Outputs
  input wire           jump, clk, reset;
  input wire [31:0]    inst;

  output wire          reg_WE,       rs1_SEL,      rs2_SEL;
  output wire          addrs_SEL,    pc_EN,        instr_EN;
  output wire          ALU_mem_EN,   mem_in_EN,    mem_WE;
  output wire [1:0]    reg_SEL,      pc_SEL;
  output wire [2:0]    imm_SEL,      mem_MODE;
  output wire [3:0]    ALU_MODE;

  // Internal
  wire [6:0]          opcode, funct7;
  wire [2:0]          funct3;
  reg [19:0]          control_bus;
  reg [2:0]           state_control;

  reg state;
  localparam 
    Fetch = 1'b0,
    Execute = 1'b1;


  assign opcode = inst[6:0];
  assign funct3 = inst[14:12];
  assign funct7 = inst[31:25];

  initial begin
    state <= Fetch;
    control_bus <= 20'b00000000000000000000;
  end

  assign {
    reg_WE, 
    rs1_SEL, 
    rs2_SEL, 
    reg_SEL, 
    pc_SEL, 
    imm_SEL, 
    ALU_MODE, 
    ALU_mem_EN, 
    mem_in_EN, 
    mem_WE, 
    mem_MODE} = control_bus;


  assign { addrs_SEL, pc_EN, instr_EN} = state_control;

  // Need to account for reset
  always @(posedge reset, posedge clk) begin
    if (reset)
      state <= Fetch;
    else
      case (state)    // State Machine
        Fetch:      state <= Execute;
        Execute:    state <= Fetch;
      endcase
  end


  always @(*) begin
    case (state)
      Fetch:
        begin
          state_control <= 3'b101;
          control_bus <= 20'b00000000000000010000;
        end
      Execute:
        begin
          state_control <= 3'b010;
          case (opcode)
            7'b0110111:   control_bus <= 20'b10010000000000000000;      // LUI
            7'b0010111:   control_bus <= 20'b11101000000000000000;      // AUIPC
            7'b1101111:   control_bus <= 20'b10011101000000000000;      // JAL
            7'b1100111:   control_bus <= 20'b10111010111101000000;      // JALR
            7'b1100011:
              begin
                case (funct3)
                  3'b000: control_bus <= {5'b00000, (jump == 1'b1) ? 2'b10 : 2'b00, 13'b0101000000000};  // BEQ
                  3'b001: control_bus <= {5'b00000, (jump == 1'b0) ? 2'b10 : 2'b00, 13'b0101000000000};  // BNE
                endcase
              end
          endcase
        end
    endcase


      //                 3'b100:     // BLT
      //                     /*  BLT take the branch if rs1 is less than rs2, using
      //                     signed comparison */
      //                     begin
      //                         mem_MODE    <= 3'b000;
      //                         dmemWE      <= 1'b0;
      //                         regWE       <= 1'b0;
      //                         rs1SEL      <= 1'b0;
      //                         rs2SEL      <= 1'b0;
      //                         regSEL      <= 2'b00;
      //                         pcSEL       <= (jump == 1'b1) ? 2'b10 : 2'b00;
      //                         immSEL      <= 3'b010;
      //                         ALU_MODE  <= 4'b1010;
      //                     end
      //                 3'b101:     // BGE
      //                     /* BGE take the branch if rs1 is greater than or equal to rs2, 
      //                     using signed comparison */
      //                     begin
      //                         mem_MODE    <= 3'b000;
      //                         dmemWE      <= 1'b0;
      //                         regWE       <= 1'b0;
      //                         rs1SEL      <= 1'b0;
      //                         rs2SEL      <= 1'b0;
      //                         regSEL      <= 2'b00;
      //                         pcSEL       <= (jump == 1'b1) ? 2'b10 : 2'b00;
      //                         immSEL      <= 3'b010;
      //                         ALU_MODE  <= 4'b1100;
      //                     end
      //                 3'b110:     // BLTU
      //                     /*  BLTU take the branch if rs1 is less than rs2, using
      //                     unsigned comparison */
      //                     begin
      //                         mem_MODE    <= 3'b000;
      //                         dmemWE      <= 1'b0;
      //                         regWE       <= 1'b0;
      //                         rs1SEL      <= 1'b0;
      //                         rs2SEL      <= 1'b0;
      //                         regSEL      <= 2'b00;
      //                         pcSEL       <= (jump == 1'b1) ? 2'b10 : 2'b00;
      //                         immSEL      <= 3'b010;
      //                         ALU_MODE  <= 4'b1001;
      //                     end
      //                 3'b111:     // BGEU
      //                     /* BGEU take the branch if rs1 is greater than or equal to rs2, 
      //                     using unsigned comparison */
      //                     begin
      //                         mem_MODE    <= 3'b000;
      //                         dmemWE      <= 1'b0;
      //                         regWE       <= 1'b0;
      //                         rs1SEL      <= 1'b0;
      //                         rs2SEL      <= 1'b0;
      //                         regSEL      <= 2'b00;
      //                         pcSEL       <= (jump == 1'b1) ? 2'b10 : 2'b00;
      //                         immSEL      <= 3'b010;
      //                         ALU_MODE  <= 4'b1011;
      //                     end
      //             endcase
      //         end
      //     7'b0000011:
      //         begin
      //             case (funct3)
      //                 3'b000:     // LB
      //                     begin
      //                         mem_MODE    <= 3'b110;
      //                         dmemWE      <= 1'b0;
      //                         regWE       <= 1'b1;
      //                         rs1SEL      <= 1'b0;
      //                         rs2SEL      <= 1'b1;
      //                         regSEL      <= 2'b00;
      //                         pcSEL       <= 2'b00;
      //                         immSEL      <= 3'b011;
      //                         ALU_MODE  <= 4'b0000;
      //                     end
      //                 3'b001:     // LH
      //                     begin
      //                         mem_MODE    <= 3'b101;
      //                         dmemWE      <= 1'b0;
      //                         regWE       <= 1'b1;
      //                         rs1SEL      <= 1'b0;
      //                         rs2SEL      <= 1'b1;
      //                         regSEL      <= 2'b00;
      //                         pcSEL       <= 2'b00;
      //                         immSEL      <= 3'b011;
      //                         ALU_MODE  <= 4'b0000;
      //                     end
      //                 3'b010:     // LW
      //                     begin
      //                         mem_MODE    <= 3'b000;
      //                         dmemWE      <= 1'b0;
      //                         regWE       <= 1'b1;
      //                         rs1SEL      <= 1'b0;
      //                         rs2SEL      <= 1'b1;
      //                         regSEL      <= 2'b00;
      //                         pcSEL       <= 2'b00;
      //                         immSEL      <= 3'b011;
      //                         ALU_MODE  <= 4'b0000;
      //                     end
      //                 3'b100:     // LBU
      //                     begin
      //                         mem_MODE    <= 3'b010;
      //                         dmemWE      <= 1'b0;
      //                         regWE       <= 1'b1;
      //                         rs1SEL      <= 1'b0;
      //                         rs2SEL      <= 1'b1;
      //                         regSEL      <= 2'b00;
      //                         pcSEL       <= 2'b00;
      //                         immSEL      <= 3'b011;
      //                         ALU_MODE  <= 4'b0000;
      //                     end
      //                 3'b101:     // LHU
      //                     begin
      //                         mem_MODE    <= 3'b001;
      //                         dmemWE      <= 1'b0;
      //                         regWE       <= 1'b1;
      //                         rs1SEL      <= 1'b0;
      //                         rs2SEL      <= 1'b1;
      //                         regSEL      <= 2'b00;
      //                         pcSEL       <= 2'b00;
      //                         immSEL      <= 3'b011;
      //                         ALU_MODE  <= 4'b0000;
      //                     end
      //             endcase
      //         end
      //     7'b0100011:
      //         begin
      //             case (funct3)
      //                 3'b000:     // SB
      //                     begin
      //                         mem_MODE    <= 3'b010;
      //                         dmemWE      <= 1'b1;
      //                         regWE       <= 1'b0;
      //                         rs1SEL      <= 1'b0;
      //                         rs2SEL      <= 1'b1;
      //                         regSEL      <= 2'b00;
      //                         pcSEL       <= 2'b00;
      //                         immSEL      <= 3'b001;
      //                         ALU_MODE  <= 4'b0000;
      //                     end
      //                 3'b001:     // SH
      //                     begin
      //                         mem_MODE    <= 3'b001;
      //                         dmemWE      <= 1'b1;
      //                         regWE       <= 1'b0;
      //                         rs1SEL      <= 1'b0;
      //                         rs2SEL      <= 1'b1;
      //                         regSEL      <= 2'b00;
      //                         pcSEL       <= 2'b00;
      //                         immSEL      <= 3'b001;
      //                         ALU_MODE  <= 4'b0000;
      //                     end
      //                 3'b010:     // SW
      //                     begin
      //                         mem_MODE    <= 3'b000;
      //                         dmemWE      <= 1'b1;
      //                         regWE       <= 1'b0;
      //                         rs1SEL      <= 1'b0;
      //                         rs2SEL      <= 1'b1;
      //                         regSEL      <= 2'b00;
      //                         pcSEL       <= 2'b00;
      //                         immSEL      <= 3'b001;
      //                         ALU_MODE  <= 4'b0000;
      //                     end
      //             endcase
      //         end
      //     7'b0010011:
      //         begin
      //             case (funct3)
      //                 3'b000:     // ADDI
      //                     begin
      //                         mem_MODE    <= 3'b000;
      //                         dmemWE      <= 1'b0;
      //                         regWE       <= 1'b1;
      //                         rs1SEL      <= 1'b0;
      //                         rs2SEL      <= 1'b1;
      //                         regSEL      <= 2'b01;
      //                         pcSEL       <= 2'b00;
      //                         immSEL      <= 3'b011;
      //                         ALU_MODE  <= 4'b0000;
      //                     end
      //                 3'b010:     // SLTI
      //                     begin
      //                         mem_MODE    <= 3'b000;
      //                         dmemWE      <= 1'b0;
      //                         regWE       <= 1'b1;
      //                         rs1SEL      <= 1'b0;
      //                         rs2SEL      <= 1'b1;
      //                         regSEL      <= 2'b01;
      //                         pcSEL       <= 2'b00;
      //                         immSEL      <= 3'b011;
      //                         ALU_MODE  <= 4'b1010;
      //                     end
      //                 3'b011:     // SLTIU
      //                     begin
      //                         mem_MODE    <= 3'b000;
      //                         dmemWE      <= 1'b0;
      //                         regWE       <= 1'b1;
      //                         rs1SEL      <= 1'b0;
      //                         rs2SEL      <= 1'b1;
      //                         regSEL      <= 2'b01;
      //                         pcSEL       <= 2'b00;
      //                         immSEL      <= 3'b011;
      //                         ALU_MODE  <= 4'b1001;
      //                     end
      //                 3'b100:     // XORI
      //                     begin
      //                         mem_MODE    <= 3'b000;
      //                         dmemWE      <= 1'b0;
      //                         regWE       <= 1'b1;
      //                         rs1SEL      <= 1'b0;
      //                         rs2SEL      <= 1'b1;
      //                         regSEL      <= 2'b01;
      //                         pcSEL       <= 2'b00;
      //                         immSEL      <= 3'b011;
      //                         ALU_MODE  <= 4'b0100;
      //                     end
      //                 3'b110:     // ORI
      //                     begin
      //                         mem_MODE    <= 3'b000;
      //                         dmemWE      <= 1'b0;
      //                         regWE       <= 1'b1;
      //                         rs1SEL      <= 1'b0;
      //                         rs2SEL      <= 1'b1;
      //                         regSEL      <= 2'b01;
      //                         pcSEL       <= 2'b00;
      //                         immSEL      <= 3'b011;
      //                         ALU_MODE  <= 4'b0011;
      //                     end
      //                 3'b111:     // ANDI
      //                     begin
      //                         mem_MODE    <= 3'b000;
      //                         dmemWE      <= 1'b0;
      //                         regWE       <= 1'b1;
      //                         rs1SEL      <= 1'b0;
      //                         rs2SEL      <= 1'b1;
      //                         regSEL      <= 2'b01;
      //                         pcSEL       <= 2'b00;
      //                         immSEL      <= 3'b011;
      //                         ALU_MODE  <= 4'b0010;
      //                     end
      //                 3'b001:
      //                     case (funct7)
      //                         7'b0000000:     // SLLI
      //                             begin
      //                                 mem_MODE    <= 3'b000;
      //                                 dmemWE      <= 1'b0;
      //                                 regWE       <= 1'b1;
      //                                 rs1SEL      <= 1'b0;
      //                                 rs2SEL      <= 1'b1;
      //                                 regSEL      <= 2'b01;
      //                                 pcSEL       <= 2'b00;
      //                                 immSEL      <= 3'b011;
      //                                 ALU_MODE  <= 4'b0101;
      //                             end
      //                     endcase
      //                 3'b101:     
      //                     case (funct7)
      //                         7'b0000000:     // SRLI
      //                             begin
      //                                 mem_MODE    <= 3'b000;
      //                                 dmemWE      <= 1'b0;
      //                                 regWE       <= 1'b1;
      //                                 rs1SEL      <= 1'b0;
      //                                 rs2SEL      <= 1'b1;
      //                                 regSEL      <= 2'b01;
      //                                 pcSEL       <= 2'b00;
      //                                 immSEL      <= 3'b011;
      //                                 ALU_MODE  <= 4'b0110;
      //                             end
      //                         7'b0100000:     // SRAI
      //                             begin
      //                                 mem_MODE    <= 3'b000;
      //                                 dmemWE      <= 1'b0;
      //                                 regWE       <= 1'b1;
      //                                 rs1SEL      <= 1'b0;
      //                                 rs2SEL      <= 1'b1;
      //                                 regSEL      <= 2'b01;
      //                                 pcSEL       <= 2'b00;
      //                                 immSEL      <= 3'b011;
      //                                 ALU_MODE  <= 4'b0111;
      //                             end
      //                     endcase
      //             endcase
      //         end
      //     7'b0110011:
      //         case (funct3)
      //             3'b000:
      //                 case (funct7)
      //                     7'b0000000:     // ADD
      //                         begin
      //                             mem_MODE    <= 3'b000;
      //                             dmemWE      <= 1'b0;
      //                             regWE       <= 1'b1;
      //                             rs1SEL      <= 1'b0;
      //                             rs2SEL      <= 1'b0;
      //                             regSEL      <= 2'b01;
      //                             pcSEL       <= 2'b00;
      //                             immSEL      <= 3'b000;
      //                             ALU_MODE  <= 4'b0000;
      //                         end
      //                     7'b0100000:     // SUB
      //                         begin
      //                             mem_MODE    <= 3'b000;
      //                             dmemWE      <= 1'b0;
      //                             regWE       <= 1'b1;
      //                             rs1SEL      <= 1'b0;
      //                             rs2SEL      <= 1'b0;
      //                             regSEL      <= 2'b01;
      //                             pcSEL       <= 2'b00;
      //                             immSEL      <= 3'b000;
      //                             ALU_MODE  <= 4'b0001;
      //                         end
      //                 endcase
      //             3'b001:
      //                 case (funct7)
      //                     7'b0000000:     // SLL
      //                         begin
      //                             mem_MODE    <= 3'b000;
      //                             dmemWE      <= 1'b0;
      //                             regWE       <= 1'b1;
      //                             rs1SEL      <= 1'b0;
      //                             rs2SEL      <= 1'b0;
      //                             regSEL      <= 2'b01;
      //                             pcSEL       <= 2'b00;
      //                             immSEL      <= 3'b000;
      //                             ALU_MODE  <= 4'b0101;
      //                         end
      //                 endcase
      //             3'b010:
      //                 case (funct7)
      //                     7'b0000000:     // SLT
      //                         begin
      //                             mem_MODE    <= 3'b000;
      //                             dmemWE      <= 1'b0;
      //                             regWE       <= 1'b1;
      //                             rs1SEL      <= 1'b0;
      //                             rs2SEL      <= 1'b0;
      //                             regSEL      <= 2'b01;
      //                             pcSEL       <= 2'b00;
      //                             immSEL      <= 3'b000;
      //                             ALU_MODE  <= 4'b1010;
      //                         end
      //                 endcase
      //             3'b011:
      //                 case (funct7)
      //                     7'b0000000:     // SLTU
      //                         begin
      //                             mem_MODE    <= 3'b000;
      //                             dmemWE      <= 1'b0;
      //                             regWE       <= 1'b1;
      //                             rs1SEL      <= 1'b0;
      //                             rs2SEL      <= 1'b0;
      //                             regSEL      <= 2'b01;
      //                             pcSEL       <= 2'b00;
      //                             immSEL      <= 3'b000;
      //                             ALU_MODE  <= 4'b1001;
      //                         end
      //                 endcase
      //             3'b100:
      //                 case (funct7)
      //                     7'b0000000:     // XOR
      //                         begin
      //                             mem_MODE    <= 3'b000;
      //                             dmemWE      <= 1'b0;
      //                             regWE       <= 1'b1;
      //                             rs1SEL      <= 1'b0;
      //                             rs2SEL      <= 1'b0;
      //                             regSEL      <= 2'b01;
      //                             pcSEL       <= 2'b00;
      //                             immSEL      <= 3'b000;
      //                             ALU_MODE  <= 4'b0100;
      //                         end
      //                 endcase
      //             3'b101:
      //                 case (funct7)
      //                     7'b0000000:     // SRL
      //                         begin
      //                             mem_MODE    <= 3'b000;
      //                             dmemWE      <= 1'b0;
      //                             regWE       <= 1'b1;
      //                             rs1SEL      <= 1'b0;
      //                             rs2SEL      <= 1'b0;
      //                             regSEL      <= 2'b01;
      //                             pcSEL       <= 2'b00;
      //                             immSEL      <= 3'b000;
      //                             ALU_MODE  <= 4'b0110;
      //                         end
      //                     7'b0100000:     // SRA
      //                         begin
      //                             mem_MODE    <= 3'b000;
      //                             dmemWE      <= 1'b0;
      //                             regWE       <= 1'b1;
      //                             rs1SEL      <= 1'b0;
      //                             rs2SEL      <= 1'b0;
      //                             regSEL      <= 2'b01;
      //                             pcSEL       <= 2'b00;
      //                             immSEL      <= 3'b000;
      //                             ALU_MODE  <= 4'b0111;
      //                         end
      //                 endcase
      //             3'b110:
      //                 case (funct7)
      //                     7'b0000000:     // OR
      //                         begin
      //                             mem_MODE    <= 3'b000;
      //                             dmemWE      <= 1'b0;
      //                             regWE       <= 1'b1;
      //                             rs1SEL      <= 1'b0;
      //                             rs2SEL      <= 1'b0;
      //                             regSEL      <= 2'b01;
      //                             pcSEL       <= 2'b00;
      //                             immSEL      <= 3'b000;
      //                             ALU_MODE  <= 4'b0011;
      //                         end
      //                 endcase
      //             3'b111:
      //                 case (funct7)
      //                     7'b0000000:     // AND
      //                         begin
      //                             mem_MODE    <= 3'b000;
      //                             dmemWE      <= 1'b0;
      //                             regWE       <= 1'b1;
      //                             rs1SEL      <= 1'b0;
      //                             rs2SEL      <= 1'b0;
      //                             regSEL      <= 2'b01;
      //                             pcSEL       <= 2'b00;
      //                             immSEL      <= 3'b000;
      //                             ALU_MODE  <= 4'b0010;
      //                         end
      //                 endcase
      //         endcase
      //     /*
      //         Instructions are not necessary for Single Cycle Implementations

      //     7'b0001111:
      //         if (inst[31:28] == 0 and inst[19:7] == 0) begin
      //             // FENCELL
      //         end 
      //         else if (inst[31:7] == 25'h0000020) begin
      //             // FENCE.I
      //         end
      //     7'b1110011:
      // case (funct3)
      //   3'b000:
      //     if (inst[11:7] == 0 and inst[19:15] == 0 and inst[31:20] == 0) begin
      //               // ECALL
      //     end
      //     else if (inst[11:7] == 0 and inst[19:15] == 0 and inst[31:20] == 1) begin
      //                         // EBREAK
      //     end
      //   3'b001:					// CSRRW
      //   3'b010:					// CSRRS
      //   3'b011:					// CSRRC
      //   3'b101:					// CSRRWI
      //   3'b110:					// CSRRSI
      //   3'b111:					// CSRRCI
      // endcase
      //     */    
      //     default:
      //         begin
      //             mem_MODE    <= 3'b000;
      //             dmemWE      <= 1'b0;
      //             regWE       <= 1'b0;
      //             rs1SEL      <= 1'b0;
      //             rs2SEL      <= 1'b0;
      //             regSEL      <= 2'b00;
      //             pcSEL       <= 2'b00;
      //             immSEL      <= 3'b000;
      //             ALU_MODE  <= 4'b0000;
      //         end
      
  end
endmodule
