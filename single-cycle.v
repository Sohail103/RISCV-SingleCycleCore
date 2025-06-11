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

// Register File
module Reg_File(clk, reset, Regwrite, Rs1, Rs2, Rd, Write_data, read_data1, read_data2);

  input clk, reset, Regwrite;
  input [4:0] Rs1, Rs2, Rd;
  input [31:0] Write_data;
  output [31:0] read_data1, read_data2;
  integer k;
  reg [31:0] Registers[31:0];

  always @(posedge clk or posedge reset)
  begin
    if(reset) begin
      for(k=0; k<32; k=k+1) begin
        Registers[k] <= 32'b0;
      end
    end
    else if (Regwrite) begin
      Registers[Rd] <= Write_data;
    end

    assign read_data1 = Registers[Rs1];
    assign read_data2 = Registers[Rs2];

endmodule

// Immediate Generator
module ImmGen(Opcode, instruction, Immext);

  input [6:0] Opcode;
  input [31:0] instruction;
  output [31:0] ImmExt;

  always @(*)
  begin
    case (Opcode)
      7'b0000011: ImmExt = {20{instruction[31]}, instruction[31:20]};
      7'b0100011: ImmExt = {20{instruction[31]}, instruction[31:25], instruction[11:7]};
      7'b1100011: ImmExt = {19{instruction[31]}, instruction[31], instruction[30:25], instruction[11:8], 1'b0};
    endcase
  end

endmodule

//Control unit
module Control_Unit(instruction, Branch, MemRead, MemtoReg, ALUop, MemWrite, ALUSrc, RegWrite);

  input [6:0] instruction;
  output Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite;
  output [1:0]  ALUOp;

  always @(*)
  begin
    case(instruction)
      7'b0110011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} <= 8'b001000_01;
      7'b0000011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} <= 8'b111100_00;
      7'b0100011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} <= 8'b100010_00;
      7'b1100011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} <= 8'b000001_01;
    endcase




endmodule
