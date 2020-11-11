/***********************************************************************************************/
/*********************************  MIPS 5-stage pipeline implementation ***********************/
/***********************************************************************************************/
`include "constants.h"

module cpu(input clock, input reset);
 reg [31:0] PC; 
 reg [31:0] IFID_PCplus4;
 reg [31:0] IFID_instr;
 reg [31:0] IDEX_rdA, IDEX_rdB, IDEX_signExtend, IDEX_SignExtendSll, IDEX_PCplus4;
 reg [4:0]  IDEX_instr_rt, IDEX_instr_rs, IDEX_instr_rd;                            
 reg        IDEX_RegDst, IDEX_ALUSrc;
 reg [1:0]  IDEX_ALUcntrl;
 reg        IDEX_Branch,IDEX_Bne, IDEX_MemRead, IDEX_MemWrite; 
 reg        IDEX_MemToReg, IDEX_RegWrite;                
 reg [4:0]  EXMEM_RegWriteAddr, EXMEM_instr_rd; 
 reg [31:0] EXMEM_ALUOut;
 reg        EXMEM_Zero;
 reg [31:0] EXMEM_MemWriteData;
 reg        EXMEM_Branch,EXMEM_Bne , EXMEM_MemRead, EXMEM_MemWrite, EXMEM_RegWrite, EXMEM_MemToReg;
 reg [31:0] MEMWB_DMemOut;
 reg [4:0]  MEMWB_RegWriteAddr, MEMWB_instr_rd; 
 reg [31:0] MEMWB_ALUOut;
 reg        MEMWB_MemToReg, MEMWB_RegWrite;               
 wire [31:0] instr, ALUInA, ALUInB, ALUOut, rdA, rdB, signExtend, DMemOut, wRegData, PCIncr;
 wire Zero, RegDst, MemRead, MemWrite, MemToReg, ALUSrc, RegWrite, Branch,Bne;
 wire [5:0] opcode, func;
 wire [4:0] instr_rs, instr_rt, instr_rd, RegWriteAddr;
 wire [3:0] ALUOp;
 wire ALUsll;
 reg IDEX_ALUsll;
 wire [1:0] ALUcntrl;
 wire [15:0] imm;
 wire [31:0] signExtendsll;
 wire [31:0] ALUInBSll;
 wire JumpSignal;
 
 wire PCSrc,PCSrc_Bne;
 reg [31:0] new_pc;
 reg [31:0] EXMEM_new_pc;
 
 wire bubble_idex;
 wire PCWrite;
 wire IFID_Write;
 wire STALL_select;
 
 wire [31:0] bubble;
 wire [5:0] sllExtend;
 
assign bubble = `NOP;
/***************** Instruction Fetch Unit (IF)  ****************/
 always @(posedge clock or negedge reset)
  begin 
    if (reset == 1'b0)     
       PC <= -1;     
	
    else if (PC == -1)
       PC <= 0;
	else if(PCWrite==1'b1)
		PC <= PC;
	else if ( JumpSignal==1'b1)
		PC <= {IFID_PCplus4[31:26],IFID_instr[25:0]<<2};
    else 
       PC <= (PCSrc==1) ? EXMEM_new_pc : 
			 (PCSrc_Bne==1)? EXMEM_new_pc : PC + 4;
  end
  
  // IFID pipeline register
 always @(posedge clock or negedge reset)
  begin 
    if (reset == 1'b0)     
      begin
       IFID_PCplus4 <= 32'b0;    
       IFID_instr <= 32'b0;
    end 
    else 
      begin
		if(IFID_Write!=1)begin
			
			IFID_PCplus4 <= (JumpSignal || PCSrc || PCSrc_Bne) ? 32'b0 : PC + 32'd4; //ayto allaksa
			IFID_instr <= (JumpSignal || PCSrc || PCSrc_Bne) ? 32'b0 : instr;
		end
		
    end
  end
  
// Instruction memory 1KB
Memory cpu_IMem(clock, reset, 1'b1, 1'b0, PC>>2, 32'b0, instr );//PC
   

  
  
/***************** Instruction Decode Unit (ID)  ****************/




assign bubble_idex = (STALL_select || PCSrc || PCSrc_Bne);

assign opcode = (JumpSignal || STALL_select==0) ? IFID_instr[31:26] : bubble[31:26];
assign func = (JumpSignal || STALL_select==0)? IFID_instr[5:0] : bubble[5:0];
assign instr_rs = (JumpSignal || STALL_select==0) ? IFID_instr[25:21] : bubble[25:21];
assign instr_rt = (JumpSignal || STALL_select==0) ? IFID_instr[20:16] : bubble[20:16];
assign instr_rd = (JumpSignal || STALL_select==0 ) ? IFID_instr[15:11] : bubble[15:11];
assign imm = (JumpSignal || STALL_select==0) ? IFID_instr[15:0] :  bubble[15:0];
assign sllExtend =  IFID_instr [10:6] ;
assign signExtend = {{16{imm[15]}}, imm};

assign signExtendsll = {{25{sllExtend[5]}},sllExtend};
// Register file
RegFile cpu_regs(clock, reset, instr_rs, instr_rt, MEMWB_RegWriteAddr, MEMWB_RegWrite, wRegData, rdA, rdB);

  // IDEX pipeline register
 always @(posedge clock or negedge reset)
  begin 
    if (reset == 1'b0 || bubble_idex)
      begin
       IDEX_rdA <= 32'b0;    
       IDEX_rdB <= 32'b0;
       IDEX_signExtend <= 32'b0;
       IDEX_instr_rd <= 5'b0;
       IDEX_instr_rs <= 5'b0;
       IDEX_instr_rt <= 5'b0;
       IDEX_RegDst <= 1'b0;
       IDEX_ALUcntrl <= 2'b0;
       IDEX_ALUSrc <= 1'b0;
       IDEX_Branch <= 1'b0;
       IDEX_MemRead <= 1'b0;
       IDEX_MemWrite <= 1'b0;
       IDEX_MemToReg <= 1'b0;                  
       IDEX_RegWrite <= 1'b0;
       IDEX_SignExtendSll <= 32'b0;
       IDEX_PCplus4 <= 32'b0;
       IDEX_Bne <= 1'b0;
       //IDEX_ALUsll <= 1'b0;
    end 
    else 
      begin

       IDEX_rdA <= rdA;
       IDEX_rdB <= rdB;
       IDEX_signExtend <= signExtend;
       IDEX_instr_rd <= instr_rd;
       IDEX_instr_rs <= instr_rs ;
       IDEX_instr_rt <= instr_rt;
       IDEX_RegDst <= RegDst  ;//********EX
       IDEX_ALUcntrl <=  ALUcntrl ;
       IDEX_ALUSrc <= ALUSrc ;//********EX
       IDEX_Branch <= Branch ;//den to xrhshmopoioume
       IDEX_MemRead <= MemRead ;//******M
       IDEX_MemWrite <=  MemWrite ;//****M
       IDEX_MemToReg <=  MemToReg ;//****wb                
       IDEX_RegWrite <=  RegWrite ;//****wb
       IDEX_SignExtendSll <= signExtendsll;
       IDEX_ALUsll <= ALUsll;
       IDEX_PCplus4 <= IFID_PCplus4;
       IDEX_Bne <= Bne;
    end
  end

// Main Control Unit 
control_main control_main (RegDst,
                  Branch,
                  MemRead,
                  MemWrite,
                  MemToReg,
                  ALUSrc,
                  RegWrite,
                  ALUcntrl,
                  Bne,
                  JumpSignal,
                  opcode);
                  
// Instantiation of Control Unit that generates stalls goes here

hazard_unit HazardUnit(.ifid_rs(IFID_instr[25:21]),
					   .ifid_rt(IFID_instr[20:16]),
					   .idex_mem_read(IDEX_MemRead),
					   .idex_rt(IDEX_instr_rt),
					   .pc_write(PCWrite),
					   .ifid_write(IFID_Write),
					   .select(STALL_select));

/***************** Execution Unit (EX)  ****************/

	
	wire [1:0] ForwardA;
    wire [1:0] ForwardB;
    
    wire [31:0] temp;
    
always @( IDEX_PCplus4 or IDEX_signExtend) 
	begin
	new_pc <= IDEX_PCplus4 + (IDEX_signExtend <<2) ;
	end

assign ALUInA = (ALUsll==1'b1 ) ? temp : (ForwardA==2'b00) ? IDEX_rdA : (ForwardA==2'b01) ? wRegData : EXMEM_ALUOut ;
assign temp = (ForwardB == 2'b00) ? IDEX_rdB : (ForwardB==2'b01) ? wRegData : EXMEM_ALUOut;
assign ALUInB = (IDEX_ALUSrc == 1'b0) ? temp : IDEX_signExtend;
assign ALUInBSll = (ALUsll == 1'b1) ? IDEX_SignExtendSll : ALUInB ;
//  ALU
ALU  #(32) cpu_alu(ALUOut, Zero, ALUInA, ALUInBSll, ALUOp);

assign RegWriteAddr = (IDEX_RegDst==1'b0) ? IDEX_instr_rt : IDEX_instr_rd;

 // EXMEM pipeline register
 always @(posedge clock or negedge reset)
  begin 
    if (reset == 1'b0)     
      begin
       EXMEM_ALUOut <= 32'b0;    
       EXMEM_RegWriteAddr <= 5'b0;
       EXMEM_MemWriteData <= 32'b0;
       EXMEM_Zero <= 1'b0;
       EXMEM_Branch <= 1'b0;
       EXMEM_MemRead <= 1'b0;
       EXMEM_MemWrite <= 1'b0;
       EXMEM_MemToReg <= 1'b0;                  
       EXMEM_RegWrite <= 1'b0;
       EXMEM_new_pc <= 32'b0;
       EXMEM_Bne <= 1'b0;
      end 
    else 
      begin
       EXMEM_ALUOut <= (PCSrc || PCSrc_Bne) ? 'b0 : ALUOut;    
       EXMEM_RegWriteAddr <= (PCSrc || PCSrc_Bne ) ? 'b0 :RegWriteAddr;
       EXMEM_MemWriteData <= (PCSrc || PCSrc_Bne ) ? 'b0 :temp;
       EXMEM_Zero <= (PCSrc || PCSrc_Bne ) ? 'b0 :Zero;
       EXMEM_Branch <= (PCSrc || PCSrc_Bne ) ? 'b0 :IDEX_Branch;
       EXMEM_MemRead <= (PCSrc || PCSrc_Bne ) ? 'b0 :IDEX_MemRead;
       EXMEM_MemWrite <= (PCSrc || PCSrc_Bne ) ? 'b0 :IDEX_MemWrite;
       EXMEM_MemToReg <= (PCSrc || PCSrc_Bne ) ? 'b0 :IDEX_MemToReg;                  
       EXMEM_RegWrite <= (PCSrc || PCSrc_Bne ) ? 'b0 :IDEX_RegWrite;
       EXMEM_instr_rd <= (PCSrc || PCSrc_Bne ) ? 'b0 :IDEX_instr_rd; //*****
       EXMEM_new_pc <= (PCSrc || PCSrc_Bne) ? 'b0 :new_pc;
       EXMEM_Bne <= (PCSrc || PCSrc_Bne ) ? 'b0 :IDEX_Bne;
      end
  end
  
  
  
  
  assign PCSrc = EXMEM_Branch & EXMEM_Zero;
  assign PCSrc_Bne = EXMEM_Bne & !EXMEM_Zero ;
  
  // ALU control
  control_alu control_alu(ALUOp,ALUsll, IDEX_ALUcntrl, IDEX_signExtend[5:0]);
  
  //assign
  
    // Instantiation of control logic for Forwarding goes here
    
    control_bypass_ex ForwardUnit(.bypassA(ForwardA),
								  .bypassB(ForwardB),
								  .sll(ALUsll),
								  .idex_rs(IDEX_instr_rs),
								  .idex_rt(IDEX_instr_rt),
								  .exmem_rd(EXMEM_RegWriteAddr),
								  .memwb_rd(MEMWB_RegWriteAddr),
								  .exmem_regwrite(EXMEM_RegWrite),
								  .memwb_regwrite(MEMWB_RegWrite) );

  
/***************** Memory Unit (MEM)  ****************/  

// Data memory 1KB
Memory cpu_DMem(clock, reset, EXMEM_MemRead, EXMEM_MemWrite, EXMEM_ALUOut, EXMEM_MemWriteData, DMemOut);

// MEMWB pipeline register
 always @(posedge clock or negedge reset)
  begin 
    if (reset == 1'b0)     
      begin
       MEMWB_DMemOut <= 32'b0;    
       MEMWB_ALUOut <= 32'b0;
       MEMWB_RegWriteAddr <= 5'b0;
       MEMWB_MemToReg <= 1'b0;                  
       MEMWB_RegWrite <= 1'b0;
      end 
    else 
      begin
       MEMWB_DMemOut <= DMemOut;
       MEMWB_ALUOut <= EXMEM_ALUOut;
       MEMWB_RegWriteAddr <= EXMEM_RegWriteAddr;
       MEMWB_MemToReg <= EXMEM_MemToReg;                  
       MEMWB_RegWrite <= EXMEM_RegWrite;
       MEMWB_instr_rd <= EXMEM_instr_rd;
      end
  end

  
  
  

/***************** WriteBack Unit (WB)  ****************/  
assign wRegData = (MEMWB_MemToReg == 1'b0) ? MEMWB_ALUOut : MEMWB_DMemOut;


endmodule
