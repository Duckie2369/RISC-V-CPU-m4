module regfile(SW,LEDR,LEDG,KEY, HEX7, HEX6, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0);
	input [17:0] SW;
    input [3:0] KEY;
    output [0:6] HEX7, HEX6, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0;
	output [17:0] LEDR;
	output [7:0] LEDG;

	assign LEDR=SW;
	wire [31:0] W_rd_data1, W_rd_data2;
	
	hex_ssd C0(W_rd_data2[3:0], HEX0);
	hex_ssd C1(W_rd_data2[7:4], HEX1);
	
	hex_ssd C2(SW[4:2], HEX2);
	hex_ssd C3(SW[0], HEX3);
	
	hex_ssd C4(W_rd_data1[3:0], HEX4);
	hex_ssd C5(W_rd_data1[7:4], HEX5);
	
	hex_ssd C6(SW[8:6], HEX6);
	hex_ssd C7(SW[0], HEX7);

	Register_File DUT(.clk(SW[17]), .wr_en(SW[16]), .wr_addr(SW[15:13]), .wr_data(SW[12:10]), .rd_en1(SW[9]), .rd_addr1(SW[8:6]), .rd_data1(W_rd_data1), .rd_en2(SW[5]), .rd_addr2(SW[4:2]), .rd_data2(W_rd_data2));
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
			if (rd_addr2 == 0) 
				rd_data2 = 0;
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
				end
		end
endmodule