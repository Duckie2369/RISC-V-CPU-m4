module ALU(SW,LEDR,LEDG,KEY, HEX7, HEX6, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0);
	input [17:0] SW;
    input [3:0] KEY;
    output [0:6] HEX7, HEX6, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0;
	output [17:0] LEDR;
	output [7:0] LEDG;

	assign LEDR=SW;
	wire W_funct7;
	wire [2:0] W_funct3;
	wire [4:0] W_Dmem;
	wire [31:0] W_ra, W_rb, W_pc, W_imm, W_alu_out;
	assign W_ra[31] = SW[8];
	assign W_rb[31] = SW[7];
	//assign W_imm[31] = SW[7];
	
	assign W_ra[1:0] = SW[3:2];
	assign W_rb[1:0] = SW[1:0];
	//assign W_imm[1:0] = SW[1:0];
	
	hex_ssd C0(W_alu_out[3:0], HEX0);
	hex_ssd C1(W_alu_out[7:4], HEX1);
	
	hex_ssd C2(W_alu_out[11:8], HEX2);
	hex_ssd C3(W_alu_out[15:12], HEX3);
	
	hex_ssd C4(SW[1:0], HEX4);
	hex_ssd C5(SW[17], HEX5);
	
	hex_ssd C6(SW[3:2], HEX6);
	hex_ssd C7(SW[6:4], HEX7);

	//alu DUT(.ra(SW[7:4]), .rb(SW[3:0]), .imm(SW[3:0]), .pc(SW[7:4]), .opcode(SW[13:9]), .funct3(SW[16:14]), .funct7(SW[17]), .taken_branch(LEDG[7]), .is_jal(LEDG[6]), .is_jalr(LEDG[5]), .DMem_addr(LEDG[4:0]), .alu_out(W_alu_out));

	// LUI: alu DUT(.ra(W_ra), .rb(W_rb), .imm(SW[17:2]), .pc(W_pc), .opcode(SW[4:0]), .funct3(W_funct3), .funct7(W_funct7), .taken_branch(LEDG[7]), .is_jal(LEDG[6]), .is_jalr(LEDG[5]), .DMem_addr(LEDG[4:0]), .alu_out(W_alu_out));
	// AUIPC, JALR, JAL: alu DUT(.ra(W_ra), .rb(W_rb), .imm(SW[17:2]), .pc(SW[6:5]), .opcode(SW[4:0]), .funct3(SW[7:5]), .funct7(W_funct7), .taken_branch(LEDG[7]), .is_jal(LEDG[6]), .is_jalr(LEDG[5]), .DMem_addr(LEDG[4:0]), .alu_out(W_alu_out));

	// B-type: alu DUT(.ra(W_ra), .rb(SW[1:0]), .imm(W_imm), .pc(W_pc), .opcode(SW[17:13]), .funct3(SW[12:10]), .funct7(W_funct7), .taken_branch(LEDG[7]), .is_jal(LEDG[6]), .is_jalr(LEDG[5]), .DMem_addr(LEDG[4:0]), .alu_out(W_alu_out));

	// Store and Load: alu DUT(.ra(W_ra), .rb(SW[9:0]), .imm(W_imm), .pc(W_pc), .opcode(SW[17:13]), .funct3(SW[12:10]), .funct7(W_funct7), .taken_branch(LEDG[7]), .is_jal(LEDG[6]), .is_jalr(LEDG[5]), .DMem_addr(W_Dmem), .alu_out(W_alu_out));

	// I-type: alu DUT(.ra(W_ra), .rb(W_rb), .imm(W_imm), .pc(W_pc), .opcode(SW[17:13]), .funct3(SW[12:10]), .funct7(SW[9]), .taken_branch(LEDG[7]), .is_jal(LEDG[6]), .is_jalr(LEDG[5]), .DMem_addr(W_Dmem), .alu_out(W_alu_out));

	alu DUT(.ra(W_ra), .rb(W_rb), .imm(W_imm), .pc(W_pc), .opcode(SW[17:13]), .funct3(SW[12:10]), .funct7(SW[9]), .taken_branch(LEDG[7]), .is_jal(LEDG[6]), .is_jalr(LEDG[5]), .DMem_addr(W_Dmem), .alu_out(W_alu_out));
endmodule

module alu(
	input [31:0] ra,
	input [31:0] rb,
	input [31:0] imm, pc,
	input [4:0] opcode,
	input [2:0] funct3,
	input funct7,
	output reg taken_branch, is_jal, is_jalr,
	output [4:0] DMem_addr,
	output reg [31:0] alu_out);
	
	parameter	
					OP_LUI			= 5'b01101,
					OP_AUIPC			= 5'b00101,
					OP_JAL_JALR			= 5'b11011,
					OP_JALR			= 3'b000,
					
					OP_BRANCH		= 5'b11000,
					
					OP_LOAD			= 5'b00000,
					
					OP_STORE			= 5'b01000,
					
					OP_OPIMM			= 5'b00100,
					
					OP_OP				= 5'b01100,
					
					FUNC_BEQ	 		= 3'b000,
					FUNC_BNE	 		= 3'b001,
					FUNC_BLT	 		= 3'b100,
					FUNC_BGE  		= 3'b101,
					FUNC_BLTU 		= 3'b110,
					FUNC_BGEU 		= 3'b111,
					
					FUNC_LB			= 3'b000,
					FUNC_LH			= 3'b001,
					FUNC_LW			= 3'b010,
					FUNC_LBU			= 3'b100,
					FUNC_LHU			= 3'b101,
					
					FUNC_SB			= 3'b000,
					FUNC_SH			= 3'b001,
					FUNC_SW			= 3'b010,
					
					FUNC_ADDI		= 3'b000,
					FUNC_SLTI		= 3'b010,
					FUNC_SLTIU		= 3'b011,
					FUNC_XORI		= 3'b100,
					FUNC_ORI			= 3'b110,
					FUNC_ANDI		= 3'b111,
					FUNC_SLLI		= 3'b001,
					FUNC_SRLI_SRAI	= 3'b101,
					FUNC_SRLI		= 1'b0,
					FUNC_SRAI		= 1'b1,
					
					ALU_OP_ADD_SUB	= 3'b000,
					ALU_OP_ADD		= 1'b0,
					ALU_OP_SUB	   = 1'b1,
					ALU_OP_SLL	   = 3'b001,
					ALU_OP_SLT	   = 3'b010,
					ALU_OP_SLTU		= 3'b011,
					ALU_OP_XOR	   = 3'b100,
					ALU_OP_SRL_SRA = 3'b101,
					ALU_OP_SRL		= 1'b0,
					ALU_OP_SRA	   = 1'b1,
					ALU_OP_OR		= 3'b110,
					ALU_OP_AND	   = 3'b111;
	
  reg [31:0] r_temp; 
  	reg[31:0]sltu_rslt, sltiu_rslt;
	reg[63:0] srai_rslt, sra_rslt,sext_ra;
	
	always @(*) 
		begin
          	taken_branch=1'b0;
          	is_jal=1'b0;
          	is_jalr=1'b0;
          	alu_out=32'b0;
          	r_temp = (r_temp!==32'bx) ? r_temp : (ra+imm) ;
			sltu_rslt 	= {{31{1'b0}},ra<rb};
			sltiu_rslt 	= {{31{1'b0}},ra<imm};
			sext_ra		= {{32{ra[31]}}, ra[31:0]};
			srai_rslt 	= sext_ra >> imm[4:0];
			sra_rslt 	= sext_ra >> rb[4:0];
			
			case(opcode)
				OP_LUI:
					alu_out = {imm[31:12], 12'b0};
				OP_AUIPC:
					alu_out = pc + {imm[31:12], 12'b0};
				OP_JAL_JALR:
					case(funct3)
						OP_JALR:	is_jalr = 1'b1; 
						default:	is_jal = 1'b1;
                    endcase
				OP_BRANCH:
					case(funct3)
						FUNC_BEQ:  taken_branch = (ra == rb) ? 1'b1 : 1'b0;
						FUNC_BNE:  taken_branch = (ra !== rb) ? 1'b1 : 1'b0;
						FUNC_BLT:  taken_branch = ((ra < rb) ^ (ra[31] !== rb[31])) ? 1'b1 : 1'b0;
						FUNC_BGE:  taken_branch = ((ra >= rb) ^ (ra[31] !== rb[31])) ? 1'b1 : 1'b0;
						FUNC_BLTU: taken_branch = (ra < rb) ? 1'b1 : 1'b0;
						FUNC_BGEU: taken_branch = (ra >= rb) ? 1'b1 : 1'b0;
						default:   taken_branch = 1'b0;
					endcase
				
				OP_LOAD:
					case(funct3)
						FUNC_LB:		alu_out = {{24{r_temp[7]}}, r_temp[7:0]};
						FUNC_LH:		alu_out = {{16{r_temp[15]}}, r_temp[15:0]};		
						FUNC_LW:		alu_out = {r_temp[31:0]};		
						FUNC_LBU:	alu_out = {{24{1'b0}}, r_temp[7:0]};
						FUNC_LHU:	alu_out = {{16{1'b0}}, r_temp[15:0]};
						default:	  	alu_out = 32'b0;
					endcase
				
				OP_STORE:
					case(funct3)
						FUNC_SB:		r_temp[7:0] = rb[7:0];
						FUNC_SH:		r_temp[15:0] = rb[15:0];
						FUNC_SW:		r_temp[31:0] = rb[31:0];
					endcase
				
				OP_OPIMM:
					case(funct3)
						FUNC_ADDI:			alu_out = ra + imm;
						FUNC_SLTI:			alu_out = (ra[31] == imm[31]) ? sltiu_rslt : {{31{1'b0}}, ra[31]};
						FUNC_SLTIU:			alu_out = sltiu_rslt;
						FUNC_XORI:			alu_out = ra ^ imm;
						FUNC_ORI:			alu_out = ra | imm;
						FUNC_ANDI:			alu_out = ra & imm;
						FUNC_SLLI:			alu_out = ra << imm[5:0];
						FUNC_SRLI_SRAI:	
							case(funct7)
								FUNC_SRLI:	alu_out = ra >> imm[5:0];
								FUNC_SRAI:	alu_out = srai_rslt[31:0];
							endcase
						default:	  			alu_out = 32'b0;
					endcase
				
				OP_OP:
					case(funct3)
						ALU_OP_ADD_SUB: 
							case(funct7)
								ALU_OP_ADD:	alu_out = ra + rb;
								ALU_OP_SUB: alu_out = ra - rb;
							endcase
						ALU_OP_SLL:			alu_out = ra << rb;
						ALU_OP_SLT:			alu_out = (ra[31] == rb[31]) ? sltu_rslt : {{31{1'b0}}, ra[31]};
						ALU_OP_SLTU:		alu_out = sltu_rslt;
						ALU_OP_XOR:			alu_out = ra ^ rb;
						ALU_OP_SRL_SRA:
							case(funct7)
								ALU_OP_SRL:			alu_out = ra >> rb;
								ALU_OP_SRA:			alu_out = sra_rslt[31:0];
							endcase
						ALU_OP_OR:			alu_out = ra | rb;
						ALU_OP_AND:			alu_out = ra & rb;
						default:	  			alu_out = 32'b0;
					endcase
				
			endcase;
		end
		
		assign DMem_addr = alu_out[6:2];
endmodule