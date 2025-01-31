`timescale 1ns / 1ps

module all_modules(
   
    );
endmodule

// Define pipes that exist in the PipelinedDatapath. 
// The pipe between Writeback (W) and Fetch (F), as well as Fetch (F) and Decode (D) is given to you.
// However, you can change them, if you want.
// Create the rest of the pipes where inputs follow the naming conventions in the book.


module PipeFtoD(input logic[31:0] instr, PcPlus4F,
                input logic EN, clk, clr,		// StallD will be connected as this EN
                output logic[31:0] instrD, PcPlus4D);

                always_ff @(posedge clk)
                    if (clr) begin
                        instrD<=32'b0;
                        PcPlus4D<=32'b0;
                    end
                    else if(EN)
                        begin
                        instrD<=instr;
                        PcPlus4D<=PcPlus4F;
                        end
                
endmodule

// Similarly, the pipe between Writeback (W) and Fetch (F) is given as follows.

module PipeWtoF(input logic[31:0] PC,
                input logic EN, clk, clr,		// StallF will be connected as this EN
                output logic[31:0] PCF);

                always_ff @(posedge clk)
                    if (clr)
                        PCF <= 32'b0;
                    else if(EN)
                        begin
                        PCF<=PC;
                        end
                
endmodule

// *******************************************************************************
// Below, write the modules for the pipes PipeDtoE, PipeEtoM, PipeMtoW yourselves.
// Don't forget to connect Control signals in these pipes as well.
// *******************************************************************************


module PipeDtoE(input logic clk,
                input logic regWriteD, MemtoRegD, MemWriteD, ALUSrcD, RegDstD,
                input logic [2:0] ALUControlD,
                input logic [31:0] rs_valD, rt_valD,
                input logic [4:0] rsD, rtD, rdD,
                input logic [31:0] SignImmD,
                input logic CLR, 
                output logic regWriteE, MemtoRegE, MemWriteE, ALUSrcE, RegDstE,
                output logic [2:0] ALUControlE,
                output logic [31:0] rs_valE, rt_valE,
                output logic [4:0] rsE, rtE, rdE,
                output logic [31:0] SignImmE
                );
                
                
                always_ff @(posedge clk)
                    if (CLR) begin
                        regWriteE <= 0;
                        MemtoRegE <= 0;
                        MemWriteE <= 0;
                        ALUSrcE <= 0;
                        RegDstE <= 0;
                        ALUControlE <= 3'b00;
                        rs_valE <= 32'b0;
                        rt_valE <= 32'b0;
                        rsE <= 5'b0;
                        rtE <= 5'b0;
                        rdE <= 5'b0;
                        SignImmE <= 16'b0;
                    end
                    else begin
                        regWriteE <= regWriteD;
                        MemtoRegE <= MemtoRegD;
                        MemWriteE <= MemWriteD;
                        ALUSrcE <= ALUSrcD;
                        RegDstE <= RegDstD;
                        ALUControlE <= ALUControlD;
                        rs_valE <= rs_valD;
                        rt_valE <= rt_valD;
                        rsE <= rsD;
                        rtE <= rtD;
                        rdE <= rdD;
                        SignImmE <= SignImmD;
                    end
                        
                        
                        
                        
                
                
             
endmodule

module PipeEtoM(input logic clk, clr,
                input logic RegWriteE, MemtoRegE, MemWriteE,
                input logic [31:0] ALUOutE, WriteDataE,
                input logic [4:0] WriteRegE, 
                
                output logic RegWriteM, MemtoRegM, MemWriteM,
                output logic [31:0] ALUOutM, WriteDataM,
                output logic [4:0] WriteRegM
                );
                
                
                always_ff @(posedge clk)
                
                    if (clr)
                        begin
                        RegWriteM   <= 1'b0;          // 1-bit logic
                        MemtoRegM   <= 1'b0;          // 1-bit logic
                        MemWriteM   <= 1'b0;          // 1-bit logic
                        ALUOutM     <= 32'b0;         // 32-bit logic
                        WriteDataM  <= 32'b0;         // 32-bit logic
                        WriteRegM   <= 5'b0;          // 5-bit logic
                        end
                    else
                        begin
                        RegWriteM <= RegWriteE;
                        MemtoRegM <= MemtoRegE;
                        MemWriteM <= MemWriteE;
                        ALUOutM <= ALUOutE;
                        WriteDataM <= WriteDataE;
                        WriteRegM <= WriteRegE;
                        end
		        
endmodule

module PipeMtoW(input logic clk, clr,
                input logic RegWriteM, MemtoRegM,
                input logic [31:0] ReadDataM,
                input logic [31:0] ALUOutM,
                input logic [4:0] WriteRegM,
                
                output logic RegWriteW, MemtoRegW,
                output logic [31:0] ReadDataW,
                output logic [31:0] ALUOutW,
                output logic [4:0] WriteRegW
                );
                
                
                always_ff @(posedge clk) 
                    
                    if (clr)
                        begin
                        RegWriteW     <= 0;
                        MemtoRegW     <= 0;
                        ReadDataW     <= 32'b0;  // 32-bit zero value
                        ALUOutW       <= 32'b0;  // 32-bit zero value
                        WriteRegW     <= 5'b0;
                        end
                
                    else
                
                        begin
                        RegWriteW <= RegWriteM;
                        MemtoRegW <= MemtoRegM;
                        ReadDataW <= ReadDataM;
                        ALUOutW <= ALUOutM;
                        WriteRegW <= WriteRegM;
                        end 
              
endmodule



// *******************************************************************************
// End of the individual pipe definitions.
// ******************************************************************************

// *******************************************************************************
// Below is the definition of the datapath.
// The signature of the module is given. The datapath will include (not limited to) the following items:
//  (1) Adder that adds 4 to PC
//  (2) Shifter that shifts SignImmE to left by 2
//  (3) Sign extender and Register file
//  (4) PipeFtoD
//  (5) PipeDtoE and ALU
//  (5) Adder for PCBranchM
//  (6) PipeEtoM and Data Memory
//  (7) PipeMtoW
//  (8) Many muxes
//  (9) Hazard unit
//  ...?
// *******************************************************************************


module mips (input  logic        clk, reset,
             
             output logic [31:0] writeDataM_test, AluoutM_test, 
             output logic RegWriteE
             
             );
           
           
   logic regwrite_test, MemToRegE, MemWriteE, memWrite_test, stallf_test, MemToRegE_test, lwstall;
   logic [31:0] WriteDataE, resultW_test;
   logic [31:0] instrF_test, pcf_test;
   logic [1:0] ForwardAe, ForwardBe;
   logic [4:0] rtE, rdE, WriteRegE, WriteRegM;
   logic [4:0] rsD, rtD, rdD;
   //logic [4:0] WriteRegE;
   
   
   
   /*logic [31:0] RD1_test, RD2_test;
   logic [4:0] rs, rt, rd;*/
  

  
   datapath dp (clk, reset,
		    instrF_test, writeDataM_test, AluoutM_test, resultW_test, pcf_test,
		    memWrite_test, regwrite_test, stallf_test,
		    ForwardAe, ForwardBe,
		    
		    rtE, rdE, WriteRegM,
		    /*RD1_test, RD2_test,
		    rs, rt, rd,*/
		    
		    rsD, rtD, rdD,
		    MemToRegE_test,
		    
		    RegWriteE,MemToRegE,MemWriteE,
		    WriteDataE,
		    WriteRegE,
		    lwstall
               	 );
	
endmodule

module datapath (input  logic clk, reset,
		    
		         		         // Change input-outputs according to your datapath design
		         
		         output logic [31:0] instrF_test, writeDataM_test, AluoutM_test, resultw_test, pcf_test,
		         output logic memWrite_test, regwrite_test, stallf_test,
		         
		         output logic [1:0] ForwardAe_test, ForwardBe_test,
		         
		         output logic [4:0] rtE_test, rdE_test, WriteRegM_test,
		         
		         output logic [4:0] rsD_test1, rtD_test1, rdD_test1,
		         output logic MemToRegE_test,
		   
		         
		         output logic RegWriteE,MemToRegE,MemWriteE,                 
                 output logic[31:0] WriteDataE,
                 output logic [4:0] WriteRegE,
                 output logic lwstall
               	 ); 

	// ********************************************************************
	// Here, define the wires that are needed inside this pipelined datapath module
	// ********************************************************************
	
	logic clear_decoder, clear_execute;
	
	assign clear_decoder = 0;
	assign clear_execute = 0;
	
	/*int i = 0;
	
	always_ff @(posedge clk)begin
	   i = i + 1;
    end
    
    always_comb begin
        if (i < 1) clear_decoder = 1;
        else clear_decoder = 0;
        if (i < 2) clear_execute = 1;
        else clear_execute = 0;
    end*/
	    
	
	

	logic stallF, stallD,  ForwardAD, ForwardBD,  FlushE;
	logic [1:0] ForwardAE, ForwardBE;		// Wires for connecting Hazard Unit
	logic PcSrcD, MemToRegW, RegWriteW;											// Add the rest of the wires whenever necessary.
    logic [31:0] PC, PCF, instrF, instrD, PcSrcA, PcSrcB, PcPlus4F, PcPlus4D;
    logic [31:0] PcBranchD, ALUOutW, ReadDataW, ResultW, RD1, RD2;
    logic [4:0] WriteRegW;
    
    
    
    logic [4:0] rsE, rtE, rdE;
    logic RegWriteM, MemToRegM;
    logic [4:0] WriteRegM;
    
    logic [31:0] SignImmD;
    
    logic [4:0] rsD,rtD,rdD;	
    
    
    logic MemToRegD, MemWriteD, ALUSrcD;
    logic RegDstD, RegWriteD, BranchD;
    logic [2:0] ALUControlD;
    logic jump;
     
    
     
    HazardUnit hazard_unit (
                jump,
                RegWriteW,
                WriteRegW, WriteRegE,
                RegWriteM,MemToRegM,
                WriteRegM,
                RegWriteE,MemToRegE,
                rsE,rtE,
                rsD,rtD,
                BranchD,
                ForwardAE,ForwardBE, 
                ForwardAD, ForwardBD,
                FlushE,stallD, stallF,
                lwstall
    );
    
    
    
    controller controller_unit(instrD[31:26], instrD[5:0],
               MemToRegD, MemWriteD,
               ALUSrcD,
               RegDstD, RegWriteD,
               jump,
               ALUControlD,
               BranchD);
	
	// ********************************************************************
	// Instantiate the required modules below in the order of the datapath flow.
	// ********************************************************************
	
  // Connections for the writeback stage and the fetch stage is written for you.
  // You can change them if you want.

	mux2 #(32) result_mux( ALUOutW, ReadDataW, MemToRegW, ResultW);
	
	
	PipeWtoF pWtoF(PC, ~stallF, clk, reset, PCF);							// Writeback stage pipe

    assign PcPlus4F = PCF + 4;       
    
    logic [31:0] pc_prev, jump_pc;
                                   // Here PCF is from fetch stage
  	mux2 #(32) pc_mux(PcPlus4F, PcBranchD, PcSrcD, pc_prev);             // Here PcBranchD is from decode stage
    mux2 #(32) jump_mux (pc_prev, {PcPlus4F[31:28], jump_pc[27:0]}, jump, PC);
    
    
    // Note that normally whole PCF should be driven to
    // instruction memory. However for our instruction 
    // memory this is not necessary
	imem im1(PCF[7:2], instrF);								        // Instantiated instruction memory

	PipeFtoD pFtoD(instrF, PcPlus4F, ~stallD, clk, jump | clear_decoder, instrD, PcPlus4D);			    // Fetch stage pipe

	regfile rf(clk, reset, RegWriteW, instrD[25:21], instrD[20:16],
	            WriteRegW, ResultW, RD1, RD2);
	
	assign rsE_top = rsE;   
	            
	      
	            
	always_comb begin
	   rsD = instrD[25:21];
	   rtD = instrD[20:16];
	   rdD = instrD[15:11];
	end	
	
	
	signext signExtensionImm(instrD[15:0], SignImmD);
	
	logic [31:0] shifted_imm;
	sl2 shift_left(SignImmD, shifted_imm);
	
	sl2 jump_shift({6'b0, instrD[25:0]}, jump_pc);
	
	
	assign PcBranchD = shifted_imm + PcPlus4D;
	
	logic [31:0] rs_branch, rt_branch;
	logic EqualD;
	logic [31:0] ALUOutM;
	mux2 #32 rsD_branch_selector (RD1, ALUOutM, ForwardAD, rs_branch);
	mux2 #32 rtD_branch_selector (RD2, ALUOutM, ForwardBD, rt_branch);
	
	always_comb begin
	   if (rs_branch == rt_branch)
	       EqualD = 1;
	   else
	       EqualD = 0;
	end
	
	always_comb begin
	   PcSrcD = BranchD & EqualD;
    end
    
    
    logic [2:0] ALUControlE;
    logic ALUSrcE, RegDstE;
    
    logic [31:0] rsE_val, rtE_val;
    logic [31:0] SignImmE;
    
    
    
    PipeDtoE pipeDtoE(clk,
                RegWriteD, MemToRegD, MemWriteD, ALUSrcD, RegDstD,
                ALUControlD,
                RD1, RD2,
                rsD, rtD, rdD,
                SignImmD,
                FlushE | clear_execute, 
                RegWriteE, MemToRegE, MemWriteE, ALUSrcE, RegDstE,
                ALUControlE,
                rsE_val, rtE_val,
                rsE, rtE, rdE,
                SignImmE
                );
                
                
    mux2 #5 write_reg_selector (rtE, rdE, RegDstE, WriteRegE);
    
    logic [31:0] SrcAE, prev_SrcBE, SrcBE;
    mux3 #32 srcae_selector (rsE_val, ResultW, ALUOutM, ForwardAE, SrcAE);
    mux3 #32 srcbe_selector (rtE_val, ResultW, ALUOutM, ForwardBE, prev_SrcBE);
    
    assign WriteDataE = prev_SrcBE;
    
    
    mux2 #32 source_selector (prev_SrcBE, SignImmE, ALUSrcE, SrcBE);
    
    logic zero;
    
    logic [31:0] ALUresult;
    
    alu mh_alu( SrcAE, SrcBE, 
                ALUControlE, 
                ALUresult,
                zero);
    
    logic MemWriteM;
    logic [31:0] WriteDataM;
    
    PipeEtoM pEtoM(clk, reset,
                RegWriteE, MemToRegE, MemWriteE,
                ALUresult, WriteDataE,
                WriteRegE, 
                
                RegWriteM, MemToRegM, MemWriteM,
                ALUOutM, WriteDataM,
                WriteRegM
                );
                
                
    
    
    logic [31:0] ReadDataM;
    
    dmem my_dmem(clk, MemWriteM,
             ALUOutM, WriteDataM,
             ReadDataM);            
         
         
    PipeMtoW pMtoW(clk, reset,
                RegWriteM, MemToRegM,
                ReadDataM,
                ALUOutM,
                WriteRegM,
                
                RegWriteW, MemToRegW,
                ReadDataW,
                ALUOutW,
                WriteRegW
                );
                
                
    assign instrF_test = instrF;
	assign writeDataM_test = WriteDataM;
	assign AluoutM_test = ALUOutM;
	assign resultw_test = ResultW;
	assign memWrite_test = MemWriteM;
	assign regwrite_test = PcSrcD;
	assign pcf_test = PCF;
	assign stallf_test = stallF;
	
	/*assign RD1_test = SrcAE;
	assign RD2_test = prev_SrcBE;
	
	assign rsD_test = rsE;
    assign rtD_test = rtE;
    assign rdD_test = rdE;*/
    
    assign ForwardAe_test[1] = ForwardAD;
    assign ForwardAe_test[0] = ForwardBD;
    assign ForwardBe_test = ForwardBE;
    
    assign rtE_test = rtE;
    assign rdE_test = rdE;
    assign WriteRegM_test = WriteRegM;
    
    assign rsD_test1 = rsD;
    assign rtD_test1 = rtD;
    assign rdD_test1 = rdD;
    
    assign MemToRegE_test = MemToRegE;
		         
					          

endmodule



// Hazard Unit with inputs and outputs named
// according to the convention that is followed on the book.

module HazardUnit( 
                input logic jump,
                input logic RegWriteW,
                input logic [4:0] WriteRegW, WriteRegE,
                input logic RegWriteM,MemToRegM,
                input logic [4:0] WriteRegM,
                input logic RegWriteE,MemToRegE,
                input logic [4:0] rsE,rtE,
                input logic [4:0] rsD,rtD,
                input logic BranchD,
                output logic [1:0] ForwardAE,ForwardBE, 
                output logic ForwardAD, ForwardBD,
                output logic FlushE,StallD,StallF,
                output logic lwstall
         
               
    );
    
    
    
    
    //logic lwstall;
    logic branchstall;
    
    
    
    always_comb begin
    
        if (MemToRegE == 1'b1) begin
            lwstall = ((rsD === rtE) || (rtD === rtE));
        end else begin
            lwstall = 1'b0; // Directly assign 0 when MemToRegE is 0
        end
                  
        branchstall = (BranchD & RegWriteE & (WriteRegE == rsD | WriteRegE == rtD)) | (BranchD & MemToRegM & (WriteRegM == rsD || WriteRegM == rtD));
        
        ForwardAD = (rsD != 5'b0) && (rsD == WriteRegM) && (RegWriteM == 1'b1);
        ForwardBD = (rtD != 5'b0) && (rtD == WriteRegM) && (RegWriteM == 1'b1);
        
        StallF = lwstall | branchstall;
        StallD = lwstall | branchstall | jump;
        FlushE = lwstall | branchstall | jump;
    
    
	// ********************************************************************
	// Here, write equations for the Hazard Logic.
	// If you have troubles, please study pages ~420-430 in your book.
	// ********************************************************************
	
	   // forwarding data
	
        if ((rsE != 5'b0) && (rsE == WriteRegM) && (RegWriteM == 1))
            ForwardAE = 2'b10;
        else if ((rsE != 5'b0) && (rsE == WriteRegW) && (RegWriteW == 1))
               ForwardAE = 2'b01;
        else 
            ForwardAE = 2'b00;    
             
        if ((rtE != 5'b0) && (rtE == WriteRegM) && (RegWriteM == 1))
            ForwardBE = 2'b10;
        else if ((rtE != 5'b0) && (rtE == WriteRegW) && (RegWriteW == 1))
            ForwardBE = 2'b01;
        else 
            ForwardBE = 2'b00;
              
    
         
                     
    end
endmodule





// External instruction memory used by MIPS single-cycle
// processor. It models instruction memory as a stored-program 
// ROM, with address as input, and instruction as output
// Modify it to test your own programs.

module imem ( input logic [5:0] addr, output logic [31:0] instr);

// imem is modeled as a lookup table, a stored-program byte-addressable ROM
	always_comb
	   case ({addr,2'b00})		   	// word-aligned fetch
//
// 	***************************************************************************
//	Here, you can paste your own test cases that you prepared for the part 1-g.
//	Below is a program from the single-cycle lab.
//	***************************************************************************
//
//		address		instruction
//		-------		-----------
        
        // for jump
        
        /*8'h00: instr = 32'h012a4020;
        8'h04: instr = 32'h012a5820;
        8'h08: instr = 32'h08000005;
        8'h0C: instr = 32'h00008020;
        8'h10: instr = 32'h00008020;
        8'h14: instr = 32'h01098020;*/
        


        // for beq 
        /*8'h00: instr = 32'h012a4020;
        8'h04: instr = 32'h012a5820;
        8'h08: instr = 32'h110b0002;
        8'h0C: instr = 32'h00008020;
        8'h10: instr = 32'h00008020;
        8'h14: instr = 32'h01098020;*/
        

        // for lw
        
        /*8'h00: instr = 32'h8C100000;
        8'h04: instr = 32'h02114024;
        8'h08: instr = 32'h02904825;
        8'h0c: instr = 32'h02155022;
        
		8'h10: instr = 32'h8C100000;  	
		8'h14: instr = 32'h02114024;  	
		8'h18: instr = 32'h02904825;  	
		8'h1c: instr = 32'h02155022;  */
		
		
		// for compute use 
		
		//8'h00: instr = 32'h02538020;  
		8'h00: instr = 32'h20100005;
		8'h04: instr = 32'h02008820;  	
		8'h08: instr = 32'h02309020;  	
		 
		
	
		
		8'h20: instr = 32'h10800001;
		8'h24: instr = 32'h20050000;
		8'h28: instr = 32'h00e2202a;
		8'h2c: instr = 32'h00853820;
		8'h30: instr = 32'h00e23822;
		8'h34: instr = 32'hac670044;
		8'h38: instr = 32'h8c020050;
		8'h3c: instr = 32'h08000011;
		8'h40: instr = 32'h20020001;
		8'h44: instr = 32'hac020054;
		8'h48: instr = 32'h08000012;	// j 48, so it will loop here
	     default:  instr = {32{1'bx}};	// unknown address
	   endcase
endmodule


// 	***************************************************************************
//	Below are the modules that you shouldn't need to modify at all..
//	***************************************************************************

module controller(input  logic[5:0] op, funct,
                  output logic     memtoreg, memwrite,
                  output logic     alusrc,
                  output logic     regdst, regwrite,
                  output logic     jump,
                  output logic[2:0] alucontrol,
                  output logic branch);

   logic [1:0] aluop;

   maindec md (op, memtoreg, memwrite, branch, alusrc, regdst, regwrite, 
         jump, aluop);

   aludec  ad (funct, aluop, alucontrol);

endmodule

// External data memory used by MIPS single-cycle processor

module dmem (input  logic        clk, we,
             input  logic[31:0]  a, wd,
             output logic[31:0]  rd);

   logic  [31:0] RAM[63:0];
   
   /*always_comb begin
        RAM[6'b0] = 5;
   end*/
  
   assign rd = RAM[a[31:2]];    // word-aligned  read (for lw)

   always_ff @(posedge clk)
     if (we)
       RAM[a[31:2]] <= wd;      // word-aligned write (for sw)

endmodule

module maindec (input logic[5:0] op, 
	              output logic memtoreg, memwrite, branch,
	              output logic alusrc, regdst, regwrite, jump,
	              output logic[1:0] aluop );
   logic [8:0] controls;

   assign {regwrite, regdst, alusrc, branch, memwrite,
                memtoreg,  aluop, jump} = controls;

  always_comb
    case(op)
      6'b000000: controls <= 9'b110000100; // R-type
      6'b100011: controls <= 9'b101001000; // LW
      6'b101011: controls <= 9'b001010000; // SW
      6'b000100: controls <= 9'b000100010; // BEQ
      6'b001000: controls <= 9'b101000000; // ADDI
      6'b000010: controls <= 9'b000000001; // J
      default:   controls <= 9'b000000000; // illegal op
    endcase
endmodule

module aludec (input    logic[5:0] funct,
               input    logic[1:0] aluop,
               output   logic[2:0] alucontrol);
  always_comb
    case(aluop)
      2'b00: alucontrol  = 3'b010;  // add  (for lw/sw/addi)
      2'b01: alucontrol  = 3'b110;  // sub   (for beq)
      default: case(funct)          // R-TYPE instructions
          6'b100000: alucontrol  = 3'b010; // ADD
          6'b100010: alucontrol  = 3'b110; // SUB
          6'b100100: alucontrol  = 3'b000; // AND
          6'b100101: alucontrol  = 3'b001; // OR
          6'b101010: alucontrol  = 3'b111; // SLT
          default:   alucontrol  = 3'bxxx; // ???
        endcase
    endcase
endmodule

module regfile (input    logic clk, reset, we3, 
                input    logic[4:0]  ra1, ra2, wa3, 
                input    logic[31:0] wd3, 
                output   logic[31:0] rd1, rd2);

  logic [31:0] rf [31:0];

  // three ported register file: read two ports combinationally
  // write third port on rising edge of clock. Register0 hardwired to 0.
  
  

  always_ff @(negedge clk)
	 if (reset)
		for (int i=0; i<32; i++) rf[i] = 32'b0;
     else if (we3) 
         rf [wa3] <= wd3;	

  assign rd1 = (ra1 != 0) ? rf [ra1] : 0;
  assign rd2 = (ra2 != 0) ? rf[ ra2] : 0;

endmodule

module alu(input  logic [31:0] a, b, 
           input  logic [2:0]  alucont, 
           output logic [31:0] result,
           output logic zero);
    
    always_comb
    
        case(alucont)
             3'b010: result = a + b;
             3'b110: result = a - b;
             3'b000: result = a & b;
             3'b001: result = a | b;
             3'b111: result = (a < b) ? 1 : 0;
             default: result = {32{1'bx}};
        endcase
    
    assign zero = (result == 0) ? 1'b1 : 1'b0;
    
endmodule

module adder (input  logic[31:0] a, b,
              output logic[31:0] y);
     
     assign y = a + b;
endmodule

module sl2 (input  logic[31:0] a,
            output logic[31:0] y);
     
     assign y = {a[29:0], 2'b00}; // shifts left by 2
endmodule

module signext (input  logic[15:0] a,
                output logic[31:0] y);
              
  assign y = {{16{a[15]}}, a};    // sign-extends 16-bit a
endmodule

// parameterized register
module flopr #(parameter WIDTH = 8)
              (input logic clk, reset, 
	       input logic[WIDTH-1:0] d, 
               output logic[WIDTH-1:0] q);

  always_ff@(posedge clk, posedge reset)
    if (reset) q <= 0; 
    else       q <= d;
endmodule


// paramaterized 2-to-1 MUX
module mux2 #(parameter WIDTH = 8)
             (input  logic[WIDTH-1:0] d0, d1,  
              input  logic s, 
              output logic[WIDTH-1:0] y);
  
   assign y = s ? d1 : d0; 
endmodule



module mux3 #(parameter WIDTH = 8)
            (input logic[WIDTH - 1:0] d0, d1, d2,
            input logic [1:0] s,
            output logic [WIDTH - 1:0] y);
            
   always_comb begin
    
        if (s == 2'b00)
            y = d0;
        else if (s == 2'b01)
            y = d1;
        else if (s == 2'b10)
            y = d2;
   end
         
endmodule 

