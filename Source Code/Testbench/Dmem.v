module Dmem(SW,LEDR,LEDG,KEY);
	input [17:0] SW;
    input [3:0] KEY;
	output [17:0] LEDR;
	output [7:0] LEDG;

	assign LEDR=SW;

	Data_Memory DUT(.clk(SW[17]), .addr(SW[16:12]), .wr_en(SW[11]), .wr_data(SW[10:7]), .rd_en(SW[6]), .rd_data(LEDG[7:0]));
endmodule

		
module Data_Memory (clk, addr, wr_en, wr_data, rd_en, rd_data);
	input clk, wr_en, rd_en;
	input [4:0] addr;
	input [31:0] wr_data;
	
	output [31:0] rd_data;
	
	reg [31:0] DMemory [31:0];
	
	integer k;
	
	assign rd_data = (rd_en) ? DMemory[addr] : 32'b0;
	
	initial begin
		for (k=0; k<32; k=k+1) 
            begin
		           DMemory[k] = 32'b0;
			end
		//DMemory[11] = 99;
	end
		
	always @(posedge clk)
		begin
			if (wr_en) DMemory[addr] = wr_data;
		end
endmodule