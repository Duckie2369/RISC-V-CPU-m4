//Test bench


module Lab6_Task2(clk, reset, taken_br, is_jal, is_jalr, W_PC_out, W_rd_data1, W_rd_data2, W_alu_out);
	input clk, reset, taken_br, is_jal, is_jalr;
	output [31:0] W_PC_out;
	output [31:0] W_rd_data1, W_rd_data2, W_alu_out;
	
	wire  [31:0] W_PC_in, W_shifted_PC_out, W_instruction, W_addr, W_result_write_rf, imm, W_mux_out, W_ld_data;
	wire  [4:0] W_rs1, W_rs2, W_rd, is_load, is_imm;
	
	assign is_load = 5'b0x000;
	assign is_imm = 5'b00100;
	
	// Program Counter
	Program_Counter C1(.clk(clk), .reset(reset), .taken_br(taken_br), .is_jal(is_jal), .is_jalr(is_jalr), .imm(imm), .rs1_data(W_rd_data1), .PC_in(W_PC_in), .PC_out(W_PC_out));
	// Shift right the next pc by 1 bit
	Shift_Right_1_Bit C2(.in(W_PC_out), .out(W_shifted_PC_out));
	// PC increases by 1
   Adder32Bit C3(.input1(W_shifted_PC_out), .input2(32'b1), .out(W_PC_in));
	// Instruction Memory
	Instruction_Memory C4(.read_address(W_shifted_PC_out), .read_data(W_instruction), .reset(reset));
	// Decoder
	Decoder C5(.clk(clk), .instr(W_instruction), .rs1(W_rs1), .rs2(W_rs2), .rd(W_rd), .imm(imm));
	// Register File
	Register_File C6(.clk(clk), .reset(reset), .wr_en(     ), .wr_addr(W_rd), .wr_data(W_result_write_rf), .rd_en1(    ), .rd_addr1(W_rs1), .rd_data1(W_rd_data1), .rd_en2(    ), .rd_addr2(W_rs2), .rd_data2(W_rd_data2));
	// ALU
	alu C7(.ra(W_mux_out), .rb(W_rd_data2), .imm(imm), .pc(W_PC_in), .instr(W_instruction), .taken_branch(taken_br), .is_jal(is_jal), .is_jalr(is_jalr), alu_out(W_alu_out));
	// Data Memory
	Data_Memory C8(.reset(reset), .addr(W_addr), .wr_en(    ), .wr_data(W_rd_data2), .rd_en(     ), .rd_data(W_ld_data));
	
	Mux_32_bit C9(.in0(W_alu_out), .in1(W_rd_data2), .mux_out(W_result_write_rf), .select(is_load));                                                                                                                              
	
	//Control C12(.clk(clk),.Op_intstruct(W_read_data[31:26]),.ints_function(W_read_data[5:0]),.RegDst(RegDst),.PcScrc(PCSrc),.MemRead(MemRead),.MemtoReg(MemtoReg),.ALUOp(ALUop),.MemWrite(MemWrite),.ALUSrc(ALUScr),.RegWrite(RegWrite),.Zero(Zero));
endmodule

module Program_Counter (clk, reset, taken_br, is_jal, is_jalr, imm, rs1_data, PC_in, PC_out);
	input clk, reset, taken_br, is_jal, is_jalr;
	input [31:0] PC_in, imm, rs1_data;
	output [31:0] PC_out;
	reg [31:0] PC_out;
	reg [31:0] rs1_data, imm;
	always @(posedge clk or posedge reset)
	begin
		if(reset == 1'b1)
			PC_out <= 32'b0;
		else if(taken_br == 1'b1)
			PC_out <= PC_in + imm;
		else if(is_jal == 1'b1)
			PC_out <= PC_in + imm;
		else if(is_jalr == 1'b1)
			PC_out <= rs1_data + imm;
		else
			PC_out <= PC_in + 32'd4;
	end
endmodule

module Shift_Right_1_Bit(in, out);
	input [31:0] in;
	output[31:0] out;
	always @(*)
		begin
			out <= in >> 1;
		end
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
  
    //add $s1 $s2 $s4	// do the sum
    //Imemory[0] = 32'b00000010010101001000100000100000;
	 
    //sub  $s3, $s5, $6	// do the sum
    //Imemory[1] = 32'b00000010101101101001100000100010;    
	end
endmodule

module Decoder(
	input clk,
	input [31:0] instr,
	
	output [4:0] rs1,rs2,
	output reg [4:0] rd,
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
					
	reg[6:0] opcode;

	//Non-immediate fields
	assign rs1 = instr[19:15];
   assign rs2 = instr[24:20];
	
	always @(*) begin
		//Non-immediate fields
		opcode = instr[6:2];
      rd = instr[11:7];
		
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
	input reg [31:0] imm, pc,
	input reg [31:0] instr,
	output taken_branch, is_jal, is_jalr,
	output reg [31:0] alu_out);
	
	parameter	
					OP_LUI			= 5'b01101,
					OP_AUIPC			= 5'b00101,
					OP_JAL			= 5'b11011,
					OP_JALR			= 5'b11001,
					
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
	
	reg[2:0] funct3;
	reg[4:0] opcpde;
	reg[5:0] rd;
	reg[6:0] funct7;
	reg[31:0] r_temp, sltu_rslt, sltiu_rslt;
	reg[63:0] srai_rslt, sra_rslt,sext_ra;
	
	assign is_jal = 1'b0;
	assign is_jalr = 1'b0;
	assign taken_branch = 1'b0;
	
	always @(*) 
		begin
			opcode 		= instr[6:2];
			funct3 		= instr[14:12];
			funct7 		= instr[31:25];
			rd 			= instr[11:7];
			r_temp 		= ra+imm;
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
				OP_JAL:
					is_jal = 1'b1;
					alu_out = pc + 32'd4;
				OP_JALR:
					is_jalr = 1'b1;
					alu_out = pc + 32'd4;
					
				OP_BRANCH:
					case(funct3)
						FUNC_BEQ:  taken_branch = (ra == rb) ? 1'b1 : 1'b0;
						FUNC_BNE:  taken_branch = (ra !== rb) ? 1'b1 : 1'b0;
						FUNC_BLT:  taken_branch = ((ra < rb) ^ (ra[31] !== rb[31])) ? 1'b1 : 1'b0;
						FUNC_BGE:  taken_branch = ((ra >= rb) ^ (ra[31] !== rb[31])) ? 1'b1 : 1'b0;
						FUNC_BLTU: taken_branch = (ra < rb) ? 1'b1 : 1'b0;
						FUNC_BGEU: taken_branch = (ra >= rs2) ? 1'b1 : 1'b0;
						default:	  taken_branch = 1'b0;
					endcase
				;
				
				OP_LOAD:
					case(funct3)
						FUNC_LB:		alu_out = {{24{r_temp[7]}, r_temp[7:0]};
						FUNC_LH:		alu_out = {{16{r_temp[15]}, r_temp[15:0]};		
						FUNC_LW:		alu_out = {r_temp[31:0]};		
						FUNC_LBU:	alu_out = {{24{1'b0}, r_temp[7:0]};
						FUNC_LHU:	alu_out = {{16{1'b0}, r_temp[15:0]};
						default:	  	alu_out = 32'b0;
					endcase
				;
				
				OP_STORE:
					case(funct3)
						FUNC_SB:		r_temp[7:0] = rb[7:0];
						FUNC_SH:		r_temp[15:0] = rb[15:0];
						FUNC_SW		r_temp[31:0] = rb[31:0];
					endcase
				;
				
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
				;	
				
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
								ALU_OP_SRA::		alu_out = sra_rslt[31:0];
							endcase
						ALU_OP_OR:			alu_out = ra | rb;
						ALU_OP_AND:			alu_out = ra & rb;
						default:	  			alu_out = 32'b0;
					endcase
				;
				
			endcase
		end
endmodule


module Register_File (clk,reset, wr_en, wr_addr, wr_data, rd_en1, rd_addr1, rd_data1, rd_en2, rd_addr2 , rd_data2);
	input  clk,reset,wr_en,rd_en1,rd_en2;
	input [4:0] rd_addr1, rd_addr2, wr_addr;
	input [31:0] wr_data;

	output reg [31:0] rd_data1, rd_data2;
	
	reg [31:0] Regfile [31:0];
	
	integer k;
	
	always @(posedge reset)
		begin
			for (k=0; k<32; k=k+1) 
				begin
					Regfile[k] = 32'b0;
				end 
		end
	
	//Assign data1 to address1 register
	always @(rd_data1 or Regfile[rd_addr1])
		begin
			if (rd_addr1 == 5'b0) 
				rd_data1 = 32b'0;
			else 
				begin
					rd_data1 = Regfile[rd_addr1];
					//$display("read_addr_1=%d,read_data_1=%h",read_addr_1,read_data_1);
				end
		end
		
	//Assign data2 to address2 register
	always @(rd_data2 or Regfile[rd_addr2])
		begin
			if (rd_addr2 == 0) 
				rd_data2 = 0;
			else 
				begin
					rd_data2 = Regfile[rd_addr2];
					//$display("read_addr_2=%d,read_data_2=%h",read_addr_2,read_data_2);
				end
		end
		
	//Assign write data to write address
	always @(posedge clk)
		begin
			if (wr_en == 1'b1)
				begin 
					Regfile[wr_addr] = wr_data;
					//$display("write_addr=%d write_data=%d",write_addr,write_data);
				end
		end
endmodule

module Mux_32_bit (in0, in1, mux_out, select);
	input [31:0] in0, in1;
	output [31:0] mux_out;
	input select;
	assign mux_out = select ? in1 : in0;
endmodule

module Data_Memory (reset, addr, wr_en, wr_data, rd_en, rd_data);
	input reset, wr_en, rd_en;
	input [4:0] addr;
	input [31:0] wr_data;
	
	output [31:0] rd_data;
	
	reg [31:0] DMemory [31:0];
	
	integer k;
	
	assign rd_data = (rd_en) ? DMemory[addr] : 32'b0;
	
	always @(posedge reset)
		begin
			for (k=0; k<32; k=k+1)
				begin
					DMemory[k] = 32'b0;
				end
				//DMemory[11] = 99;
		end
		
	always @(*)
		begin
			if (wr_en) DMemory[addr] = wr_data;
		end
endmodule

module Control(clk, wr_en_Reg, rd_en1_Reg, rd_en2_Reg, wr_en_DMem, rd_en_DMem);
    input clk,Zero;
    input [5:0] ints_function;
    input [5:0] Op_intstruct;
    output reg RegDst,PcScrc,MemRead,MemtoReg;
    output reg [2:0] ALUOp;
    output reg MemWrite,ALUSrc,RegWrite;
    always @(posedge clk)
    begin
                    RegDst=1;
                    PcScrc=0;
                    MemRead=0;
                    MemtoReg=0;
                    MemWrite=0;
                    ALUSrc=0;
                    RegWrite=1;
                    ALUOp =3'b000;
                    if(ints_function==6'b100000) // add
                            ALUOp =3'b000;
                                  
                    if(ints_function==6'b100010) // sub 
                            ALUOp =3'b001;
                                   
                    if(ints_function==6'b100100) // and
                            ALUOp =3'b010;
                                   
                    if(ints_function==6'b100101) // or
                            ALUOp =3'b010;
    end
endmodule
