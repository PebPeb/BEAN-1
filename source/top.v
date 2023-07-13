

module top(clk, reset);
  input wire   clk, reset;  

  wire         mem_WE;
  wire [2:0]   mem_MODE;
  wire [31:0]  data_mem_READ, data_mem_WRITE, mem_addrs;


  BEAN_1 CPU (
    .clk(clk), 
    .reset(reset), 
    .data_mem_IN(data_mem_READ), 
    .data_mem_OUT(data_mem_WRITE), 
    .mem_addrs(mem_addrs),
    .mem_MODE(mem_MODE), 
    .mem_WE(mem_WE)
    );


  mem memory(
    .a(mem_addrs), 
    .rd(data_mem_READ), 
    .wd(data_mem_WRITE), 
    .clk(clk), 
    .we(mem_WE), 
    .mode(mem_MODE) 
    );


endmodule



