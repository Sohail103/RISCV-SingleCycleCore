// Program counter
module Program_Counter(clk, reset, PC_in, PC_out);

  input clk, reset;
  input [31:0] PC_in;
  output reg [31:0] PC_out;

  always @(posedge clk or posedge reset)
    begin
      if(reset)
        PC_out <= 32'b0;
      else
        PC_out <= PC_in;
    end
    
endmodule

// PC + 4
module PCplus4(fromPC, NexttoPC);

  input [31:0] fromPC;
  output [31:0] NexttoPC;

  assign NexttoPC = 4 + fromPC;

endmodule

// Instruction Memory

module Instruction_Mem(clk, reset, read_address, instruction_out);

  input clk, reset;
  input [31:0] read_address;
  output reg [31:0] instruction_out;
  integer k;

  reg [31:0] I_Mem[63:0];

  always @(posedge clk or posedge reset)
    begin
    
    if(reset)
    begin
      
      for(k=0; k<64; k=k+1)
      begin
        I_Mem[k] <= 32'b0;
      end
    end
    else begin
      instruction_out <= I_Mem[read_address];
    end

    end
    
endmodule

// Regsiter File
module Reg_File(clk, reset, Regwrite, Rs1, Rs2, Rd, Write_data, read_data1, read_data2);

  input clk, reset, Regwrite;
  input [4:0] Rs1, Rs2, Rd;
  input [31:0] Write_data;
  output [31:0] read_data1, read_data2;

  reg [31:0] Registers[31:0];

  always @(posedge clk or posedge reset)
  begin
    if(reset) begin
      for(k=0; k<32; k=k+1) begin
        Registers[k] <= 32'b0;
      end

    end
endmodule
