//-----------------------------------------------------------------------------
//
// Title       : sinhron_reg
// Design      : sinh1
// Author      : Microsoft
// Company     : Microsoft
//
//-----------------------------------------------------------------------------
//
// File        : sinhron_reg.v
// Generated   : Thu Jan 22 18:38:44 2015
// From        : interface description file
// By          : Itf2Vhdl ver. 1.22
//
//-----------------------------------------------------------------------------
//
// Description : 
//
//-----------------------------------------------------------------------------
`timescale 1 ns / 1 ps

//{{ Section below this comment is automatically maintained
//   and may be overwritten
//{module {sinhron_reg}}
module sinhron_reg ( clk, clk5, data_in, dta_from_bus, wr, rd, a1, a2, a3, a4, adr, adr_out, dto, reset, TNO_mk, TNC_mk, TOBM_mk, TNI_mk, TKI_mk, TNP_mk, TKP_mk, TNO, TNC, TOBM, TNI, TKI, TNP, TKP, wr_bus, rd_bus, ale1, ale2, ale3, ale4);

output TNO ;
wire   TNO ;
output TNC ;
wire   TNC ;
output TOBM ;
wire   TOBM ;
output TNI ;
wire   TNI ;
output TKI ;
wire   TKI ;
output TNP ;
wire   TNP ;
output TKP ;
wire   TKP ;

output ale1;
wire   ale1;
output ale2;
wire   ale2;
output ale3;
wire   ale3;
output ale4;
wire   ale4;

output wr_bus;
wire   wr_bus;
output rd_bus;
wire   rd_bus;

output TNO_mk ;
wire   TNO_mk ;
output TNC_mk ;
wire   TNC_mk ;
output TOBM_mk ;
wire   TOBM_mk ;
output TNI_mk ;
wire   TNI_mk ;
output TKI_mk ;
wire   TKI_mk ;
output TNP_mk ;
wire   TNP_mk ;
output TKP_mk ;
wire   TKP_Mk ;

output [7:0] dto ;
wire [7:0] dto ; 
output [7:0] adr_out ;
wire [7:0] adr_out ;

input a1;
wire a1; 
input a2;
wire a2;
input a3;
wire a3; 
input a4;
wire a4;

input clk ;
wire clk ; 
input clk5 ;
wire clk5 ;
input [15:0] dta_from_bus ;
wire [15:0] dta_from_bus ;
input [15:0] data_in ;
wire [15:0] data_in ;
input wr;
wire wr;
input rd;
wire rd;
input [7:0] adr ;
wire [7:0] adr ;   

input reset ;
wire reset ;

reg [31:0] main_timer;

reg [3:0] ale_reg;
reg reg_wr;
reg reg_rd;

reg [31:0] reg_TNO;
reg [31:0] reg_TNC;
reg [31:0] reg_TOBM;
reg [31:0] reg_TNI;
reg [31:0] reg_TKI;
reg [31:0] reg_TNP;
reg [31:0] reg_TKP;	   	



reg [31:0] temp1;
reg [15:0] temp_indata;

reg flag_TNO_mk;
reg flag_TNC_mk;
reg flag_TOBM_mk;
reg flag_TNI_mk;
reg flag_TKI_mk;
reg flag_TNP_mk;
reg flag_TKP_mk;  

reg flag_TNO;
reg flag_TNC;
reg flag_TOBM;
reg flag_TNI;
reg flag_TKI;
reg flag_TNP;
reg flag_TKP; 

reg[7:0] d_TNO;
reg[7:0] d_TNC;
reg[7:0] d_TOBM;
reg[7:0] d_TNI;
reg[7:0] d_TKI;
reg[7:0] d_TNP;
reg[7:0] d_TKP; 

reg [7:0] reg_dto;	   
reg [7:0] reg_adr_out;
reg [7:0] reg_adr;	
reg [7:0] sch_1us;	 

reg flag_T1us;

reg [2:0] front_t1us; 
reg [2:0] front_wr;	
reg [2:0] front_rd;	  
reg [2:0] front_clk5;	

 
always @(posedge clk)
		begin
	  	   //контролируем по€вление фронта сигнала
			front_wr  <={  front_wr  [1:0],wr  };	   
			front_rd  <={  front_rd  [1:0],rd  };
			front_clk5<={  front_clk5[1:0],clk5};	
			
			reg_adr_out<=adr;
			reg_wr<=wr;
			reg_rd<=rd;
			ale_reg<={a1,a2,a3,a4};
			
		end
		
always @(posedge clk)	  	//формируем сигнал T1us 1мкс , пока из 20 ћ√ц тактовой
	begin
	   
		if (sch_1us<20) 
			begin
				sch_1us<=sch_1us+1;
				flag_T1us<=0;
			end 
		else
			begin
			  sch_1us  <=0;
			  flag_T1us<=1;
			end
			
	end
 

always @(posedge clk)
	
	if (reset)
		
		begin
		   
			reg_TNO <=32'h0;
			reg_TNC <=32'h0;
			reg_TOBM<=32'h0;
			reg_TNI <=32'h0;
			reg_TKI <=32'h0;
			reg_TNP <=32'h0;
			reg_TKP <=32'h0;
			
			main_timer<=1;
			
			flag_TNO <=0;
			flag_TNC <=0;
			flag_TOBM<=0;
			flag_TNI <=0;
			flag_TKI <=0;
			flag_TNP <=0;
			flag_TKP <=0; 
			
			flag_TNO_mk <=0;
			flag_TNC_mk <=0;
			flag_TOBM_mk<=0;
			flag_TNI_mk <=0;
			flag_TKI_mk <=0;
			flag_TNP_mk <=0;
			flag_TKP_mk <=0; 
			
			d_TNO <=10;
 			d_TNC <=10;
  			d_TOBM<=10;
  			d_TNI <=10;
  			d_TKI <=10;
  			d_TNP <=10;
  			d_TKP <=10; 
			
			temp1      <=32'h00000000;
			temp_indata<=16'h0000; 
			
			reg_dto<=0;	 
			
		end	
	else
		
		begin
			
			if (flag_T1us==1)  main_timer<=main_timer+1;  //основной счЄтчик времени	   
			
			if (front_wr==3'b011)   		 //запись данных в €чейки пам€ти
						begin  
							case (adr) 
															
								8'd 0:begin  reg_TNO [31:16]<=data_in;  flag_TNO <=0; d_TNO <=10; end
								8'd 1:		 reg_TNO [15: 0]<=data_in;
								8'd 2:begin  reg_TNC [31:16]<=data_in;  flag_TNC <=0; d_TNC <=10; end	
								8'd 3: 		 reg_TNC [15: 0]<=data_in;	
								8'd 4:begin  reg_TOBM[31:16]<=data_in;  flag_TOBM<=0; d_TOBM<=10; end	 
								8'd 5: 		 reg_TOBM[15: 0]<=data_in;	
								8'd 6:begin  reg_TNI [31:16]<=data_in;  flag_TNI <=0; d_TNI <=10; end	
								8'd 7: 		 reg_TNI [15: 0]<=data_in;	
								8'd 8:begin  reg_TKI [31:16]<=data_in;  flag_TKI <=0; d_TKI <=10; end		   
								8'd 9: 		 reg_TKI [15: 0]<=data_in;
								8'd10:begin  reg_TNP [31:16]<=data_in;  flag_TNP <=0; d_TNP <=10; end	
								8'd11: 		 reg_TNP [15: 0]<=data_in;	
								8'd12:begin  reg_TKP [31:16]<=data_in;  flag_TKP <=0; d_TKP <=10; end	   
								8'd13: 		 reg_TKP [15: 0]<=data_in;	   
								
								default: ;
							 endcase
						end	
						
			 else
					begin	 //проверка событий на срабатывание по времени
					
						if (main_timer==reg_TNO )  flag_TNO_mk <=1;
						if (main_timer==reg_TNC )  flag_TNC_mk <=1;
						if (main_timer==reg_TOBM)  flag_TOBM_mk<=1;	
						if (main_timer==reg_TNI )  flag_TNI_mk <=1;	
						if (main_timer==reg_TKI )  flag_TKI_mk <=1;  
						if (main_timer==reg_TNP )  flag_TNP_mk <=1;  
						if (main_timer==reg_TKP )  flag_TKP_mk <=1; 	
							
						if (flag_TNO ) if (!(d_TNO ==0)) begin d_TNO <=d_TNO -1; flag_TNO <=1; end else  flag_TNO <=0;	  
						if (flag_TNC ) if (!(d_TNC ==0)) begin d_TNC <=d_TNC -1; flag_TNC <=1; end else  flag_TNC <=0; 
						if (flag_TOBM) if (!(d_TOBM==0)) begin d_TOBM<=d_TOBM-1; flag_TOBM<=1; end else  flag_TOBM<=0; 
						if (flag_TNI ) if (!(d_TNI ==0)) begin d_TNI <=d_TNI -1; flag_TNI <=1; end else  flag_TNI <=0; 	
						if (flag_TKI ) if (!(d_TKI ==0)) begin d_TKI <=d_TKI -1; flag_TKI <=1; end else  flag_TKI <=0;
						if (flag_TNP ) if (!(d_TNP ==0)) begin d_TNP <=d_TNP -1; flag_TNP <=1; end else  flag_TNP <=0; 
						if (flag_TKP ) if (!(d_TKP ==0)) begin d_TKP <=d_TKP -1; flag_TKP <=1; end else  flag_TKP <=0; 
							
					 end
						
						
						
						
				if (front_rd==3'b011)   //чтение данных из €чеек пам€ти
					begin 
							case (adr) 
								
								8'd 0: reg_dto<=reg_TNO [31:16];
								8'd 1: reg_dto<=reg_TNO [15: 0];
								8'd 2: reg_dto<=reg_TNC [31:16]; 
								8'd 3: reg_dto<=reg_TNC [15: 0];	
								8'd 4: reg_dto<=reg_TOBM[31:16]; 
								8'd 5: reg_dto<=reg_TOBM[15: 0];	
								8'd 6: reg_dto<=reg_TNI [31:16];  
								8'd 7: reg_dto<=reg_TNI [15: 0];	
								8'd 8: reg_dto<=reg_TKI [31:16]; 		   
								8'd 9: reg_dto<=reg_TKI [15: 0];
								8'd10: reg_dto<=reg_TNP [31:16]; 
								8'd11: reg_dto<=reg_TNP [15: 0];	
								8'd12: reg_dto<=reg_TKP [31:16];  
								8'd13: reg_dto<=reg_TKP [15: 0];	   
								
								default:reg_dto<=dta_from_bus; //если адреса не соответствуют синхронизатору - то чтение данных с шины обмена
							 endcase
					end 
					
		end
		
	assign 	 TNO_mk =flag_TNO_mk ;
	assign	 TNC_mk =flag_TNC_mk ;
	assign 	 TOBM_mk=flag_TOBM_mk;
	assign	 TNI_mk =flag_TNI_mk ;
	assign	 TKI_mk =flag_TKI_mk ;
	assign	 TNP_mk =flag_TNP_mk ;
	assign   TKP_mk =flag_TKP_mk ;
	
	assign 	 TNO =flag_TNO ;
	assign	 TNC =flag_TNC ;
	assign 	 TOBM=flag_TOBM;
	assign	 TNI =flag_TNI ;
	assign	 TKI =flag_TKI ;
	assign	 TNP =flag_TNP ;
	assign   TKP =flag_TKP ; 
	
	assign    dto=reg_dto; 
	
	assign    adr_out=reg_adr_out;
	
	assign  wr_bus=reg_wr;
	assign  rd_bus=reg_rd;
	
	assign  ale1=ale_reg[3];
	assign  ale2=ale_reg[2];
	assign  ale3=ale_reg[1];
	assign  ale4=ale_reg[0];

	
endmodule



































