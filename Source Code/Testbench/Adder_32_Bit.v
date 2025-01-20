module Adder_32_Bit(SW,LEDR,LEDG,KEY, HEX7, HEX6, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0);
	input [17:0] SW;
    input [3:0] KEY;
	output [0:6] HEX7, HEX6, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0;
	output [17:0] LEDR;
	output [7:0] LEDG;
	
	assign LEDR=SW;
	wire [31:0] W_out;
	hex_ssd C0(W_out[3:0], HEX0);
	hex_ssd C1(W_out[7:4], HEX1);
	hex_ssd C2(W_out[11:8], HEX2);
	hex_ssd C3(W_out[15:12], HEX3);
	hex_ssd C4(SW[3:0], HEX4);
	hex_ssd C5(SW[7:4], HEX5);
	hex_ssd C6(SW[11:8], HEX6);
	hex_ssd C7(SW[15:12], HEX7);

	Adder32Bit DUT(.input1(SW[15:8]), .input2(SW[7:0]), .out(W_out)); 
endmodule

module Adder32Bit(input1, input2, out);
	input [31:0] input1, input2;
	output [31:0] out;
	reg [31:0] out;
	always @(input1 or input2)
		begin
			out <= input1 + input2;
		end
endmodule