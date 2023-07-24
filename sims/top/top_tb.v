

module top_tb();
  reg clk, reset = 0;

  initial clk <= 0;
	always #1 clk <= ~clk;
	
  top DUT (
    .clk(clk), 
    .reset(reset)
  );

  initial begin
    #650
    $finish;
  end

  integer i;
  initial begin
    $dumpfile("top_tb.vcd");
    $dumpvars(0, top_tb);
    for (i = 0; i < 32; i = i + 1)
      $dumpvars(1, DUT.CPU.data_unit.regFILE.x[i]);
    for (i = 0; i < 1024; i = i + 1)
      $dumpvars(1, DUT.memory.mem[i]);
  end
endmodule

