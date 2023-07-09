
//
//	tri_buf.v
//		Adjustable data width Tri State Buffer
//

// -------------------------------- //
//	By: Bryce Keen	
//	Created: 06/30/2023
// -------------------------------- //
//	Last Modified: 06/30/2023



module tri_buf(data_in, data_out, enable);
	parameter WIDTH = 32;
	
	input wire [WIDTH - 1:0]	data_in;
  input wire      			    enable;
  inout wire [WIDTH - 1:0]	data_out;

  assign data_out = enable ? data_in : {WIDTH{1'bZ}};

endmodule
