module mux32(SW,LEDR,LEDG,KEY);
	input [17:0] SW;
    input [3:0] KEY;
	output [17:0] LEDR;
	output [7:0] LEDG;

	assign LEDR=SW;

	Mux_32_bit DUT(.in0(SW[15:8]), .in1(SW[7:0]), .mux_out(LEDG[7:0]), .select(SW[17]));
endmodule


module Mux_32_bit (in0, in1, mux_out, select);
	input [31:0] in0, in1;
	output [31:0] mux_out;
	input select;
	assign mux_out = select ? in1 : in0;
endmodule