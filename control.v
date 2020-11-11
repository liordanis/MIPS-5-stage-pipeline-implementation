`include "constants.h"

/************** Main control in ID pipe stage  *************/
module control_main(output reg RegDst,
                output reg Branch,  
                output reg MemRead,
                output reg MemWrite,  
                output reg MemToReg,  
                output reg ALUSrc,  
                output reg RegWrite,  
                output reg [1:0] ALUcntrl,
                output reg Bne,
                output reg Jump,
                input [5:0] opcode);

  always @(*) 
   begin
     case (opcode)
      `R_FORMAT: 
          begin 
            RegDst = 1'b1;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            MemToReg = 1'b0;
            ALUSrc = 1'b0;
            RegWrite = 1'b1;
            Branch = 1'b0; 
            Bne=1'b0;
            ALUcntrl  = 2'b10; // R   
            Jump=1'b0;
          end
       `LW :   
           begin 
            RegDst = 1'b0;
            MemRead = 1'b1;
            MemWrite = 1'b0;
            MemToReg = 1'b1;
            ALUSrc = 1'b1;
            RegWrite = 1'b1;
            Branch = 1'b0;
            Bne=1'b0;
            ALUcntrl  = 2'b00; // add
            Jump=1'b0;
           end
        `SW :   
           begin 
            RegDst = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b1;
            MemToReg = 1'b0;
            ALUSrc = 1'b1;
            RegWrite = 1'b0;
            Branch = 1'b0;
            Bne=1'b0;
            ALUcntrl  = 2'b00; // add
            Jump=1'b0;
           end
       `BEQ:  
           begin 
            RegDst = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            MemToReg = 1'b0;
            ALUSrc = 1'b0;
            RegWrite = 1'b0;
            Branch = 1'b1;
            Bne=1'b0;
            ALUcntrl = 2'b01; // sub
            Jump=1'b0;
           end
		`BNE:
			begin
			RegDst = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            MemToReg = 1'b0;
            ALUSrc = 1'b0;
            RegWrite = 1'b0;
            Branch = 1'b0;
            Bne=1'b1;
            ALUcntrl = 2'b01; // sub
            Jump=1'b0;
			end
		`ADDI:
			begin
				RegDst = 1'b0;
				MemToReg=1'b0;
				MemRead=1'b0;
				MemWrite=1'b0;
				MemToReg=1'b0;
				ALUSrc=1'b1;
				RegWrite=1'b1;
				Branch=1'b0;
				Bne=1'b0;
				ALUcntrl=2'b00; // add
				Jump=1'b0;
			end
		6'b000010:   //jump
			begin
				RegDst = 1'b0;
				MemToReg=1'b0;
				MemRead=1'b0;
				MemWrite=1'b0;
				MemToReg=1'b0;
				ALUSrc=1'b0;
				RegWrite=1'b0;
				Branch=1'b0;
				Bne=1'b0;
				ALUcntrl=2'b00;
				Jump=1'b1;
			end
				
				
		
       default:
           begin
            RegDst = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            MemToReg = 1'b0;
            ALUSrc = 1'b0;
            RegWrite = 1'b0;
            ALUcntrl = 2'b00; 
            Jump = 1'b0;
         end
      endcase
    end // always
endmodule


/**************** Module for Bypass Detection in EX pipe stage goes here  *********/
 module  control_bypass_ex(output reg [1:0] bypassA,
                       output reg [1:0] bypassB,
                       input sll,
                       input [4:0] idex_rs,
                       input [4:0] idex_rt,
                       input [4:0] exmem_rd,
                       input [4:0] memwb_rd,
                       input       exmem_regwrite,
                       input       memwb_regwrite);
      /* Fill in module details */
	always @(*)
		begin
		bypassA=0; 
		bypassB=0;
			if(exmem_regwrite && (exmem_rd!=0) && (exmem_rd == idex_rs))
				bypassA = 2'b10;//o prwtos telesteos einai to apotelesma ths proigoumenis ALU
			else if(memwb_regwrite && (memwb_rd!=0) && (exmem_regwrite==0  || exmem_rd != idex_rs) && (memwb_rd == idex_rs))
				bypassA = 2'b01;//o prwtos apo mnhmh h apo ena proigoumeno apotelesma ALU
			else if(sll && memwb_regwrite && (exmem_regwrite==0  || exmem_rd == idex_rt) && (memwb_rd == idex_rt))
				bypassA = 2'b01;
			else
				bypassA = 2'b00;
			
			if(exmem_regwrite && (exmem_rd!=0) && (exmem_rd == idex_rt))
				bypassB = 2'b10;//o deyteros telesteos einai to apotelesma ths proigoumenis ALU
			else if(memwb_regwrite && (memwb_rd!=0) && (exmem_regwrite ==0 || exmem_rd != idex_rt) && (memwb_rd == idex_rt))//*************hennesy me aperol
				bypassB = 2'b01;//o deyteros apo mnhmh h apo ena proigoumeno apotelesma ALU
			else
				bypassB = 2'b00;
	end

endmodule          
                       

/**************** Module for Stall Detection in ID pipe stage goes here  *********/
module hazard_unit (input [4:0] ifid_rs,
					input [4:0] ifid_rt,
					input idex_mem_read,
					input [4:0] idex_rt,
					output reg  pc_write,
					output reg ifid_write,
					output reg select);
	always @(*)begin
		if( idex_mem_read==1 && ((idex_rt==ifid_rs) || (idex_rt==ifid_rt))) begin 
			select = 1'b1;
			pc_write=1'b1;
			ifid_write=1'b1;
		end
		else begin
			select = 1'b0;
			pc_write=1'b0;
			ifid_write=1'b0;
		end
	end
endmodule
                       
/************** control for ALU control in EX pipe stage  *************/
module control_alu(output reg [3:0] ALUOp,
				output reg ALUsll,
               input [1:0] ALUcntrl,
               input [5:0] func);

  always @(ALUcntrl or func)  
    begin
      case (ALUcntrl)
        2'b10: 
           begin
             case (func)
              6'b100000: begin ALUOp = 4'b0010; ALUsll=1'b0; end // add
              6'b100010: begin ALUOp = 4'b0110; ALUsll=1'b0; end// sub
              6'b100100: begin ALUOp = 4'b0000; ALUsll=1'b0; end// and
              6'b100101: begin ALUOp = 4'b0001; ALUsll=1'b0; end// or
              6'b100111: begin ALUOp = 4'b1100; ALUsll=1'b0; end// nor
              6'b101010: begin ALUOp = 4'b0111; ALUsll=1'b0; end// slt
              6'b100110: begin ALUOp = 4'b1000; ALUsll=1'b0; end //xor
              6'b000100: begin ALUOp = 4'b1110; ALUsll=1'b0; end // shift left(sllv)
              6'b000000: begin ALUOp = 4'b1111; ALUsll=1'b1; end// shift left(sll)
              default: begin ALUOp = 4'b0000; ALUsll=1'b0; end       
             endcase 
          end   
        2'b00: begin 
              ALUOp  = 4'b0010; ALUsll=1'b0; end // add
        2'b01: begin
              ALUOp = 4'b0110; ALUsll=1'b0; end// sub
        default: begin
              ALUOp = 4'b0000; ALUsll=1'b0; end
     endcase
    end
endmodule
