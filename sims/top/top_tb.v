

module top_tb();
  reg clk, reset = 0;


  initial clk <= 0;
	always #1 clk <= ~clk;
	




  initial begin

    $finish;
  end

  initial begin
    $dumpfile("top_tb.vcd");
    $dumpvars(0, top_tb);
  end
endmodule

