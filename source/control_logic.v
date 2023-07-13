
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
              case (funct3)
                3'b000: control_bus <= {5'b00000, (jump == 1'b1) ? 2'b10 : 2'b00, 13'b0101000000000};  // BEQ
                3'b001: control_bus <= {5'b00000, (jump == 1'b0) ? 2'b10 : 2'b00, 13'b0101000000000};  // BNE
                3'b100: control_bus <= {5'b00000, (jump == 1'b1) ? 2'b10 : 2'b00, 13'b0101010000000};  // BLT
                3'b101: control_bus <= {5'b00000, (jump == 1'b1) ? 2'b10 : 2'b00, 13'b0101100000000};  // BGE
                3'b110: control_bus <= {5'b00000, (jump == 1'b1) ? 2'b10 : 2'b00, 13'b0101001000000};  // BLTU
                3'b111: control_bus <= {5'b00000, (jump == 1'b1) ? 2'b10 : 2'b00, 13'b0101011000000};  // BGEU
              endcase
            7'b0000011:
              case (funct3)
                3'b000:  control_bus <= 20'b10100000110000010110;      // LB
                3'b001:  control_bus <= 20'b10100000110000010101;      // LH
                3'b010:  control_bus <= 20'b10100000110000010000;      // LW
                3'b100:  control_bus <= 20'b10100000110000010010;      // LBU
                3'b101:  control_bus <= 20'b10100000110000010001;      // LHU
              endcase
            7'b0100011:
              case (funct3)
                3'b000:  control_bus <= 20'b00100000010000101010;      // SB
                3'b001:  control_bus <= 20'b00100000010000101001;      // SH
                3'b010:  control_bus <= 20'b00100000010000101000;      // SW
              endcase
            7'b0010011:
              begin
                control_bus[19:10] <= 10'b1010100011;
                control_bus[5:0] <= 6'b000000; 
                case (funct3)
                  3'b000:   control_bus[9:6] <= 4'b0000;   // ADDI
                  3'b010:   control_bus[9:6] <= 4'b1010;   // SLTI
                  3'b011:   control_bus[9:6] <= 4'b1001;   // SLTIU
                  3'b100:   control_bus[9:6] <= 4'b0100;   // XORI
                  3'b110:   control_bus[9:6] <= 4'b0011;   // ORI
                  3'b111:   control_bus[9:6] <= 4'b0010;   // ANDI
                  3'b001:
                    case (funct7)
                      7'b0000000:   control_bus[9:6] <= 4'b0101;   // SLLI
                    endcase
                  3'b101:
                    case (funct7)
                      7'b0000000:   control_bus[9:6] <= 4'b0110;   // SRLI
                      7'b0100000:   control_bus[9:6] <= 4'b0111;   // SRAI
                    endcase
                endcase
              end
              7'b0110011:
                begin
                  control_bus[19:10] <= 10'b1000100011;
                  control_bus[5:0] <= 6'b000000; 
                  case (funct3)
                    3'b000:
                      case (funct7)
                        7'b0000000:   control_bus[9:6] <= 4'b0000;   // ADD
                        7'b0100000:   control_bus[9:6] <= 4'b0001;   // SUB
                      endcase
                    3'b001:
                      case (funct7)
                        7'b0000000:   control_bus[9:6] <= 4'b0101;   // SLL
                      endcase
                    3'b010:
                      case (funct7)
                        7'b0000000:   control_bus[9:6] <= 4'b1010;   // SLT
                      endcase   
                    3'b011:
                      case (funct7)
                        7'b0000000:   control_bus[9:6] <= 4'b1001;   // SLTU
                      endcase
                    3'b100:   
                      case (funct7)
                        7'b0000000:   control_bus[9:6] <= 4'b0100;   // XOR
                      endcase   
                    3'b101:
                      case (funct7)
                        7'b0000000:   control_bus[9:6] <= 4'b0110;   // SRL
                        7'b0100000:   control_bus[9:6] <= 4'b0111;   // SRA
                      endcase  
                    3'b110:
                      case (funct7)
                        7'b0000000:   control_bus[9:6] <= 4'b0011;   // OR
                      endcase 
                    3'b111:
                      case (funct7)
                        7'b0000000:   control_bus[9:6] <= 4'b0010;   // AND
                      endcase                    
                  endcase
                end
          endcase
        end
    endcase

			



      //     7'b0010011:
      //         begin
      //             case (funct3)
      //                 3'b000:     // ADDI
      //                 3'b010:     // SLTI
      //                 3'b011:     // SLTIU
      //                 3'b100:     // XORI
      //                 3'b110:     // ORI
      //                 3'b111:     // ANDI
      //                 3'b001:
      //                     case (funct7)
      //                         7'b0000000:     // SLLI
      //                     endcase
      //                 3'b101:     
      //                     case (funct7)
      //                         7'b0000000:     // SRLI
      //                         7'b0100000:     // SRAI
      //                     endcase
      //             endcase
      //         end
      //     7'b0110011:
      //         case (funct3)
      //             3'b000:
      //                     7'b0000000:     // ADD
      //                     7'b0100000:     // SUB
      //             3'b001:
      //                     7'b0000000:     // SLL
      //             3'b010:
      //                 case (funct7)
      //                     7'b0000000:     // SLT
      //             3'b011:
      //                 case (funct7)
      //                     7'b0000000:     // SLTU
      //             3'b100:
      //                 case (funct7)
      //                     7'b0000000:     // XOR
      //             3'b101:
      //                 case (funct7)
      //                     7'b0000000:     // SRL
      //                     7'b0100000:     // SRA
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
