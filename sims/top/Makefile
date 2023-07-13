
all: build launch


build:
	iverilog -o top_tb.out top_tb.v ../../source/*.v
	vvp top_tb.out

launch:
	gtkwave top_tb.vcd

clean:
	rm -f top_tb.out
	rm -f top_tb.vcd


