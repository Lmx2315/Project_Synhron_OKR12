//-----------------------------------------------------------------------------
//
// Title       : timer_1us
// Design      : sinh1
// Author      : Microsoft
// Company     : Microsoft
//
//-----------------------------------------------------------------------------
//
// File        : timer_1us.v
// Generated   : Thu Jan 22 19:50:36 2015
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
//{module {timer_1us}}
module timer_1us ( t1us ,adr ,data ,df ,de ,dd ,dc ,db ,da ,d9 ,d8 ,d7 ,d6 ,d5 ,d4 ,d3 ,d2 ,d1 ,d0, ad2 ,ad1 ,ad0 ,clk5mhz );

output t1us ;
wire t1us ;
output [2:0] adr ;
wire [2:0] adr ;
output [15:0] data ;
wire [15:0] data ;

input df ;
wire df ;
input de ;
wire de ;
input dd ;
wire dd ;
input dc ;
wire dc ;
input db ;
wire db ;
input da ;
wire da ;
input d9 ;
wire d9 ;
input d8 ;
wire d8 ;
input d7 ;
wire d7 ;
input d6 ;
wire d6 ;
input d5 ;
wire d5 ;
input d4 ;
wire d4 ;
input d3 ;
wire d3 ;
input d2 ;
wire d2 ;
input d1 ;
wire d1 ;
input d0 ;
wire d0 ;
input ad2 ;
wire ad2 ;
input ad1 ;
wire ad1 ;
input ad0 ;
wire ad0 ;
input clk5mhz ;
wire clk5mhz ;

reg [7:0] tick;
reg flag_t1us;

always@(posedge clk5mhz)
	
		begin
		if (tick<5) 
				begin 
					tick<=tick+1; 
					flag_t1us<=0; 
				end 
			else 
				begin 
				tick<=0; 
				flag_t1us<=1;
				end 
		end
		
		assign data[0]=d0;
		assign data[1]=d1;
		assign data[2]=d2;
		assign data[3]=d3;
		assign data[4]=d4;
		assign data[5]=d5;
		assign data[6]=d6;
		assign data[7]=d7;
		assign data[8]=d8;
		assign data[9]=d9;
		assign data[10]=da;
		assign data[11]=db;
		assign data[12]=dc;	
		assign data[13]=dd;
		assign data[14]=de;
		assign data[15]=df;
		
		assign adr[0]=ad0;
		assign adr[1]=ad1;
		assign adr[2]=ad2;	 
		
		assign t1us=flag_t1us;
		
	
	
endmodule	 






















