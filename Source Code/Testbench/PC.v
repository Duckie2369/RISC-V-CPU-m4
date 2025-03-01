module testbench(SW,LEDR,LEDG,KEY, HEX7, HEX6, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0);
	input [17:0] SW;
    input [3:0] KEY;
    output [0:6] HEX7, HEX6, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0;
	output [17:0] LEDR;
	output [7:0] LEDG;
	
	assign LEDR=SW;
	wire [31:0] W_imm, W_rs1_data, W_PC_in, W_PC_out;
	
	hex_ssd C0(W_PC_out[3:0], HEX0);
	hex_ssd C1(W_PC_out[7:4], HEX1);
	
	hex_ssd C2(SW[3:0], HEX2);
	hex_ssd C3(W_PC_in[7:4], HEX3);
	
	hex_ssd C4(SW[7:4], HEX4);
	hex_ssd C5(W_rs1_data[7:4], HEX5);
	
	hex_ssd C6(SW[11:8], HEX6);
	hex_ssd C7(W_imm[7:4], HEX7);

	Program_Counter DUT(.clk(SW[17]), .reset(SW[16]), .taken_br(SW[15]), .is_jal(SW[14]), .is_jalr(SW[13]), .imm(SW[11:8]), .rs1_data(SW[7:4]), .PC_in(SW[3:0]), .PC_out(W_PC_out));
endmodule

module hex_ssd (BIN, SSD);
  input [3:0] BIN;
  output reg [0:6] SSD;

  always@(*) begin
    case(BIN)
      0:SSD=7'b0000001;
      1:SSD=7'b1001111;
      2:SSD=7'b0010010;
      3:SSD=7'b0000110;
      4:SSD=7'b1001100;
      5:SSD=7'b0100100;
      6:SSD=7'b0100000;
      7:SSD=7'b0001111;
      8:SSD=7'b0000000;
      9:SSD=7'b0001100;
      10:SSD=7'b0001000;
      11:SSD=7'b1100000;
      12:SSD=7'b0110001;
      13:SSD=7'b1000010;
      14:SSD=7'b0110000;
      15:SSD=7'b0111000;
    endcase
  end
endmodule

module Program_Counter (clk, reset, taken_br, is_jal, is_jalr, imm, rs1_data, PC_in, PC_out);
	input clk, reset, taken_br, is_jal, is_jalr;
	input [31:0] PC_in, imm, rs1_data;
	output [31:0] PC_out;
	reg [31:0] PC_out;
	always @(posedge clk or posedge reset)
	begin
      	if(reset)
        	PC_out <= 32'b0;
		else if(taken_br == 1'b1)
			PC_out <= PC_in + imm;
		else if(is_jal == 1'b1)
			PC_out <= PC_in + imm;
      else if(is_jalr == 1'b1)
			PC_out <= rs1_data + imm;
		else
			PC_out <= PC_in;
	end
endmodule