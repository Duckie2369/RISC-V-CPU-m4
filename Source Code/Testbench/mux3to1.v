module mux3to1(SW,LEDR,LEDG,KEY);
	input [17:0] SW;
    input [3:0] KEY;
	output [17:0] LEDR;
	output [7:0] LEDG;

	assign LEDR=SW;

	mux3to1_main DUT(.select(SW[17:16]), .in1(SW[11:8]), .in2(SW[7:4]), .in3(SW[3:0]), .out(LEDG[7:0]));
endmodule
	
module mux3to1_main(select, in1, in2, in3, out);
	input [31:0] in1, in2, in3;
	input [1:0] select;
	output [31:0] out;
	assign out = select[1] ? (select[0] ? 32'b0 : in3) : (select[0] ? in2 : in1);
endmodule