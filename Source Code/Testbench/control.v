module control(SW,LEDR,LEDG,KEY);
	input [17:0] SW;
    input [3:0] KEY;
	output [17:0] LEDR;
	output [7:0] LEDG;

	assign LEDR=SW;

	Control DUT(.instruction(SW[17:0]), .rd_valid(LEDG[4]), .rs1_valid(LEDG[3]), 
	.rs2_valid(LEDG[2]), .is_s_instr(LEDG[1]), .is_load(LEDG[0]));
endmodule
	
module Control(instruction, rd_valid, rs1_valid, rs2_valid, is_s_instr, is_load);
	input [31:0] instruction;
	output rd_valid, rs1_valid, rs2_valid, is_s_instr, is_load;

	wire unsigned is_u_instr, is_b_instr, is_j_instr, is_r_instr, is_i_instr, empty_rd;
 
	assign empty_rd = (instruction[11:7] == 5'b00000) ? 1'b1 : 1'b0;
	assign is_load = ((instruction[6:0] == 7'b0000011) || (instruction[6:0] == 7'b0100011)) ? 1'b1 : 1'b0;
	assign is_u_instr = ((instruction[6:2] == 5'b00101) || (instruction[6:2] == 5'b01101)) ? 1'b1 : 1'b0;
	assign is_b_instr = (instruction[6:2] == 5'b11000) ? 1'b1 : 1'b0;
	assign is_s_instr = ((instruction[6:2] == 5'b01000) || (instruction[6:2] == 5'b01001)) ? 1'b1 : 1'b0;
	assign is_j_instr = (instruction[6:2] == 5'b11011) ? 1'b1 : 1'b0;
	assign is_r_instr = ((instruction[6:2] == 5'b00101) || (instruction[6:2] == 5'b01101) || 
	(instruction[6:2] == 5'b10100)) ? 1'b1 : 1'b0;
	assign is_i_instr = ((instruction[6:2] == 5'b00000) || (instruction[6:2] == 5'b00001) || 
	(instruction[6:2] == 5'b00100) || (instruction[6:2] == 5'b00110) || (instruction[6:2] == 5'b11001)) ? 1'b1 : 1'b0;
	assign rd_valid = ~(is_s_instr | is_b_instr | empty_rd);
	assign rs1_valid = ~(is_u_instr | is_j_instr);
	assign rs2_valid = (is_r_instr | is_s_instr | is_b_instr);
endmodule