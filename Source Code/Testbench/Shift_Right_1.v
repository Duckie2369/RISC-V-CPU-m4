module Shift_Right_1(SW,LEDR,LEDG,KEY);
	input [17:0] SW;
    input [3:0] KEY;
	output [17:0] LEDR;
	output [7:0] LEDG;
	
	assign LEDR=SW;

	Shift_Right_1_Bit DUT(.in(SW[7:0]), .out(LEDG[7:0]));
endmodule

module Shift_Right_1_Bit(in, out);
	input [31:0] in;
	output [31:0] out;
	assign out = in >> 1;
endmodule