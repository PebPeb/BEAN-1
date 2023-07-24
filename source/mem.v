
//
//	mem.v
//		1, 2, and 4 byte addressable memory 
//

// -------------------------------- //
//	By: Bryce Keen	
//	Created: 10/08/2022
// -------------------------------- //
//	Last Modified: 07/09/2023

// Change Log:
//		11/01/2022 - Added Reset, modes, changed memory to byte addressable
//    07/09/2023 - Added Data initialization & removed reset

module mem(a, rd, wd, clk, we, mode);
	input wire 			clk, we;
	input wire [2:0]	mode;
	input wire [31:0]	a, wd;
	output reg [31:0]	rd = 0;
	
	parameter SIZE_BYTES = 1024;

	reg [7:0] mem [0:SIZE_BYTES - 1];
	
	parameter INITIAL_DATA_PATH = "../../source/mem.dat";
	parameter LOAD_DATA = 1;

	integer i;
	initial begin
		if (LOAD_DATA) begin
      $readmemh(INITIAL_DATA_PATH, mem);
		end
		else begin
			for (i = 0; i < SIZE_BYTES; i = i + 1) begin
				mem[i] <= 8'h00;
			end		
		end

	end
		
  // Write
	always @(posedge clk) begin
		if (we) begin
			case (mode)
				3'b000:	{mem[a], mem[a + 1], mem[a + 2], mem[a + 3]} <= wd;	    // 4 byte mode (32 bit)
				3'b001:	{mem[a], mem[a + 1]} <= wd[15:0];					              // 2 byte mode (16 bit)
				3'b101:	{mem[a], mem[a + 1]} <= wd[15:0];					              // 2 byte mode (16 bit)
				3'b010: mem[a] <= wd[7:0];									                    // 1 byte mode (8 bit)
				3'b110: mem[a] <= wd[7:0];									                    // 1 byte mode (8 bit)
				default:{mem[a], mem[a + 1], mem[a + 2], mem[a + 3]} <= wd;	    // 4 byte mode (32 bit)
			endcase
		end
	end
	
  // Read
	always @(posedge clk, a, mode) begin
		case (mode)
			3'b000: rd <= {mem[a], mem[a + 1], mem[a + 2], mem[a + 3]};	      // 4 byte mode (32 bit)
			3'b001: rd <= {{16{1'b0}}, mem[a], mem[a + 1]};				            // 2 byte not signextended
			3'b101:	rd <= {{16{mem[a][7]}}, mem[a], mem[a + 1]};		          // 2 byte signextended
			3'b010: rd <= {{24{1'b0}}, mem[a]};							                  // 1 byte not signextended
			3'b110: rd <= {{24{mem[a][7]}}, mem[a]};					                // 1 byte signextended
			default:rd <= {mem[a], mem[a + 1], mem[a + 2], mem[a + 3]};	      // 4 byte mode (32 bit)
		endcase
	end
	
	// always @(posedge clk, reset) begin
	// 	if (reset) begin
	// 		for (i = 0; i < 256; i = i + 1) begin
	// 			mem[i] <= 8'h00;
	// 		end	
	// 	end
	// end	
	
endmodule
