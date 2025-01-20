//Test bench


module CPU_Core(clk, reset, W_PC_in, W_PC_out, W_instruction, W_rs1, W_rd_data1, W_rs2, W_rd_data2, W_imm, W_rd, W_alu_out);
	input clk, reset;
  output [31:0] W_PC_in, W_rs1, W_rs2, W_rd, W_rd_data1, W_rd_data2, W_imm;
  output [31:0] W_PC_out, W_alu_out, W_instruction;
	
	wire  [31:0] W_addr, W_rd_data1, W_rd_data2, W_result_write_rf, W_ld_data;
	wire  [6:0] W_funct7;
	wire  [4:0] W_opcode, W_DMem_addr;
	wire  [2:0] W_funct3;
	wire	taken_br, is_jal, is_jalr, W_rd_valid, W_rs1_valid, W_rs2_valid, W_is_s_instr, W_is_load;
	
  // PC increases by 1 (4 bytes)
  Adder32Bit C3(.clk(clk), .reset(reset), .input1(W_PC_in), .input2(32'd4), .out(W_PC_out));
	// Program Counter
	Program_Counter C1(.clk(clk), .reset(reset), .taken_br(taken_br), .is_jal(is_jal), .is_jalr(is_jalr), .imm(W_imm), .rs1_data(W_rd_data1), .PC_in(W_PC_out), .PC_out(W_PC_in));
  
	// Instruction Memory
	Instruction_Memory C4(.read_address(W_PC_out), .read_data(W_instruction), .reset(reset));
	// Decoder
	Decoder C5(.clk(clk), .instr(W_instruction), .rs1(W_rs1), .rs2(W_rs2), .rd(W_rd), .opcode(W_opcode), .funct3(W_funct3), .funct7(W_funct7), .imm(W_imm));
	// Control for enable
	Control C6(.instruction(W_instruction), .rd_valid(W_rd_valid), .rs1_valid(W_rs1_valid), .rs2_valid(W_rs2_valid), .is_s_instr(W_is_s_instr), .is_load(W_is_load));
	// Register File
	Register_File C7(.clk(clk), .wr_en(W_rd_valid), .wr_addr(W_rd), .wr_data(W_result_write_rf), .rd_en1(W_rs1_valid), .rd_addr1(W_rs1), .rd_data1(W_rd_data1), .rd_en2(W_rs2_valid), .rd_addr2(W_rs2), .rd_data2(W_rd_data2));
	// ALU
	alu C8(.ra(W_rd_data1), .rb(W_rd_data2), .imm(W_imm), .pc(W_PC_in), .opcode(W_opcode), .funct3(W_funct3), .funct7(W_funct7), .taken_branch(taken_br), .is_jal(is_jal), .is_jalr(is_jalr), .DMem_addr(W_DMem_addr), .alu_out(W_alu_out));
	// Data Memory
	Data_Memory C9(.clk(clk), .addr(W_DMem_addr), .wr_en(W_is_s_instr), .wr_data(W_rd_data2), .rd_en(W_is_load), .rd_data(W_ld_data));
	// Select write data for Register File
	Mux_32_bit C10(.in0(W_alu_out), .in1(W_rd_data2), .mux_out(W_result_write_rf), .select(W_is_load));                                                                                                                              
	
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

module Adder32Bit(clk, reset, input1, input2, out);
  input clk, reset;
	input [31:0] input1, input2;
	output [31:0] out;
	reg [31:0] out;
  always @(input1 or input2 or posedge reset or posedge clk)
    if(reset) begin
      out <= input1;
    end
     else
		begin
			out <= input1 + input2;
		end
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
  
    //add $t3 $t0 $t1	    //	$t3 = $t0 + $t1 (3 = 1 + 2)
      Imemory[0] = 32'b00000000_0110_00101_000_11100_01100_11;       
        
      //beq $t2 $t3 0	    //	if $t2 = $t3 (3=3) then pc = pc + 0
      Imemory[4] = 32'b0000000_11100_00111_000_00000_11000_11;
      
      //jal $s0 8	    	//	$s0 = pc + 8 (12 = 4 + 8), pc = $s0 = 12
      Imemory[8] = 32'b00000000100_000000000_01000_11011_11;
      
      //jalr $s0 14($t1)	//	$s0 = pc + 4 (12 = 8 + 4), pc = $t1 + 14 (16 = 2 + 14)
      Imemory[12] = 32'b000000001110_00110_000_01000_11001_11;
      
      //sb $t3 0($a0)  		//	$a0[7:0] = $t3[7:0]
      Imemory[16] = 32'b0000000_11100_01010_000_00000_01000_11;
      
      //lb $a1 0($a0)		//	$a1[7:0] = $a0[7:0]
      Imemory[20] = 32'b000000000000_01010_000_01011_00000_11;
      
      //jal $s0 0	//	$s0 = pc + 4 (4 = 0 + 4), pc = zero + 0 (0 = 0 + 0)
      Imemory[24] = 32'b00000000000000000000_01000_11011_11; 
	end
endmodule

module Decoder(
	input clk,
	input [31:0] instr,
	
	output [4:0] rs1,rs2,
	output [4:0] rd,
  output [4:0] opcode,
	output [2:0] funct3,
	output [6:0] funct7,
	output reg [31:0] imm
	);
	
	parameter	
					OP_LUI			= 5'b01101,
					OP_AUIPC			= 5'b00101,
					OP_JAL			= 5'b11011,
					OP_JALR			= 5'b11001,
					
					OP_BRANCH		= 5'b11000,
					
					OP_STORE			= 5'b01000,
					
					OP_LOAD			= 5'b00000;

	//Non-immediate fields
	assign rs1 = instr[19:15];
	assign rs2 = instr[24:20];
  assign opcode = instr[6:2];
	assign funct3 = instr[14:12];
	assign funct7 = instr[31:25];
    assign rd = instr[11:7];
  always @(*) begin
		// Extracting the immediate field from the instruction
      case(opcode)
			OP_LUI, OP_AUIPC: imm = {instr[31:12], {12{1'b0}}}; // U-type
			OP_JAL: imm = {{11{instr[31]}}, instr[19:12], {2{instr[20]}}, instr[30:21], 1'b0}; // J-type
			OP_BRANCH: imm = {{19{instr[31]}}, {2{instr[7]}}, instr[30:25], instr[11:8], 1'b0}; // B-type
         OP_STORE: imm = {{21{instr[31]}}, instr[30:25], instr[11:8], instr[7]}; // S-type
         OP_JALR: imm = {{21{instr[31]}}, instr[30:20]}; // I-type
			default: imm = {{21{instr[31]}}, instr[30:20]}; // I-type
      endcase
  end
endmodule

module alu(
	input [31:0] ra,
	input [31:0] rb,
	input [31:0] imm, pc,
  input [4:0] opcode,
	input [2:0] funct3,
	input [6:0] funct7,
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
  	reg [31:0] sltu_rslt, sltiu_rslt;
	reg [63:0] srai_rslt, sra_rslt,sext_ra;
	
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
						FUNC_SLTI:			alu_out = (ra[31] == imm[31]) ? sltu_rslt : {{31{1'b0}}, ra[31]};
						FUNC_SLTIU:			alu_out = sltiu_rslt;
						FUNC_XORI:			alu_out = ra ^ imm;
						FUNC_ORI:			alu_out = ra | imm;
						FUNC_ANDI:			alu_out = ra & imm;
						FUNC_SLLI:			alu_out = ra << imm[5:0];
						FUNC_SRLI_SRAI:	
							case(funct7[5])
								FUNC_SRLI:	alu_out = ra >> imm[5:0];
								FUNC_SRAI:	alu_out = srai_rslt[31:0];
							endcase
						default:	  			alu_out = 32'b0;
					endcase
				
				OP_OP:
					case(funct3)
						ALU_OP_ADD_SUB: 
							case(funct7[5])
								ALU_OP_ADD:	alu_out = ra + rb;
								ALU_OP_SUB: alu_out = ra - rb;
							endcase
						ALU_OP_SLL:			alu_out = ra << rb;
						ALU_OP_SLT:			alu_out = (ra[31] == rb[31]) ? sltu_rslt : {{31{1'b0}}, ra[31]};
						ALU_OP_SLTU:		alu_out = sltu_rslt;
						ALU_OP_XOR:			alu_out = ra ^ rb;
						ALU_OP_SRL_SRA:
							case(funct7[5])
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


module Register_File (clk, wr_en, wr_addr, wr_data, rd_en1, rd_addr1, rd_data1, rd_en2, rd_addr2 , rd_data2);
	input  clk,wr_en,rd_en1,rd_en2;
	input [4:0] rd_addr1, rd_addr2, wr_addr;
	input [31:0] wr_data;

	output reg [31:0] rd_data1, rd_data2;
	
	reg [31:0] Regfile [31:0];
	
	integer k;
	
	initial begin
	for (k=0; k<32; k=k+1) 
		begin
			Regfile[k] = 32'b0;
		end
      Regfile[5]=32'd1;// $t0 = 1
      Regfile[6]=32'd2;// $t1 = 10
      Regfile[7]=32'd3;// $t2 = 11
	end
	
	//Assign data1 to address1 register
	always @(rd_data1 or Regfile[rd_addr1])
		begin
			if (rd_addr1 == 5'b0) 
				rd_data1 = 32'b0;
			else 
				begin
					if (rd_en1) begin
						rd_data1 = Regfile[rd_addr1];
					end
				end
		end
		
	//Assign data2 to address2 register
	always @(rd_data2 or Regfile[rd_addr2])
		begin
			if (rd_addr2 == 5'b0) 
				rd_data2 = 32'b0;
			else 
				begin
					if (rd_en2) begin
						rd_data2 = Regfile[rd_addr2];
					end
				end
		end
		
	//Assign write data to write address
	always @(posedge clk)
		begin
			if (wr_en == 1'b1)
				begin 
					Regfile[wr_addr] = wr_data;
                /*for (k=0; k<32; k=k+1) 
            		begin
              		$display(,,"addr=%d   data=%d",k,Regfile[k]);
					end*/
				end
		end
endmodule

module Mux_32_bit (in0, in1, mux_out, select);
	input [31:0] in0, in1;
	output [31:0] mux_out;
	input select;
	assign mux_out = select ? in1 : in0;
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
	end
		
	always @(posedge clk)
		begin
			if (wr_en) DMemory[addr] = wr_data;
          /*for (k=0; k<32; k=k+1) 
            begin
              $display(,,"addr=%d   data=%d",k,DMemory[k]);
			end
          */

		end
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
	assign is_r_instr = ((instruction[6:2] == 5'b01000) || (instruction[6:2] == 5'b01001) || (instruction[6:2] == 5'b01010) || 
	(instruction[6:2] == 5'b01011) || (instruction[6:2] == 5'b01100) || (instruction[6:2] == 5'b01101) || 
	(instruction[6:2] == 5'b01110) || (instruction[6:2] == 5'b01111) || (instruction[6:2] == 5'b10100)) ? 1'b1 : 1'b0;
	assign is_i_instr = ((instruction[6:2] == 5'b00000) || (instruction[6:2] == 5'b00001) || 
	(instruction[6:2] == 5'b00100) || (instruction[6:2] == 5'b00110) || (instruction[6:2] == 5'b11001)) ? 1'b1 : 1'b0;
	assign rd_valid = ~(is_s_instr | is_b_instr | empty_rd);
	assign rs1_valid = ~(is_u_instr | is_j_instr);
	assign rs2_valid = (is_r_instr | is_s_instr | is_b_instr);
endmodule

