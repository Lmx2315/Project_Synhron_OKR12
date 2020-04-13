//-----------------------------------------------------------------------------
//
// Title       : tst2
// Design      : sinh1
// Author      : Microsoft
// Company     : Microsoft
//
//-----------------------------------------------------------------------------
//
// File        : tst2.v
// Generated   : Wed Jan 21 19:18:16 2015
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
//{module {tst2}}
module tst2 ( clk ,o1 ,o2 ,o3 ,o4 ,o5 ,o6 ,o7 ,o0 );

output o1 ;
wire o1 ;
output o2 ;
wire o2 ;
output o3 ;
wire o3 ;
output o4 ;
wire o4 ;
output o5 ;
wire o5 ;
output o6 ;
wire o6 ;
output o7 ;
wire o7 ;
output o0 ;
wire o0 ;

input clk ;
wire clk ;

reg [15:0] a;
reg [15:0] b;

always @ (posedge clk)	  
	
	begin
	
		b<=b+1;
		a<={b[7:0],b[15:8]};

		
	end	
	
	assign o0=a[0];
	assign o1=a[1];
	assign o2=a[2];
	
	assign o3=a[3];
	assign o4=a[4];
	assign o5=a[5];	
	
	
	assign o6=a[6];
	assign o7=a[7];
	

endmodule

