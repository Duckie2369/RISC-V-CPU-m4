module Imem(SW,LEDR,LEDG,KEY, HEX7, HEX6, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0);
	input [17:0] SW;
    input [3:0] KEY;
	output [0:6] HEX7, HEX6, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0;
	output [17:0] LEDR;
	output [7:0] LEDG;
	
	assign LEDR=SW;
	wire [31:0] W_read_data;
	hex_ssd C0(W_read_data[3:0], HEX0);
	hex_ssd C1(W_read_data[7:4], HEX1);
	hex_ssd C2(W_read_data[11:8], HEX2);
	hex_ssd C3(W_read_data[15:12], HEX3);

	Instruction_Memory DUT(.read_address(SW[7:0]), .read_data(W_read_data), .reset(SW[17])); 
endmodule

module Instruction_Memory (read_address, read_data, reset);
	input reset;
	input [31:0] read_address;
	output [31:0] read_data;
	reg [31:0] Imemory [63:0];
	integer k;
	// I-MEM in this case is addressed by word, not by byte
	assign read_data = Imemory[read_address];
	always @(posedge reset)
	begin
	for (k=0; k<64; k=k+1) 
		begin  
		  Imemory[k] = 32'b0;
		end
    Imemory[0] = 32'h3;
    Imemory[1] = 32'h8;
    Imemory[8] = 32'ha;
    Imemory[15] = 32'hc;
	end
  	
endmodule