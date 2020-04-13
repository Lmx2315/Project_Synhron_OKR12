//-----------------------------------------------------------------------------
//
// Title       : bus_line
// Design      : sinh1
// Author      : Microsoft
// Company     : Microsoft
//
//-----------------------------------------------------------------------------
//
// File        : bus_line.v
// Generated   : Fri Jan 23 17:46:50 2015
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
//{module {bus_line}}
module bus_line ( clk ,in,i0 ,i1 ,i2 ,i3 ,i4 ,i5 ,i6  ,i7 );

output i1 ;
wire i1 ;
output i2 ;
wire i2 ;
output i3 ;
wire i3 ;
output i4 ;
wire i4 ;
output i5 ;
wire i5 ;
output i6 ;
wire i6 ;
output i0 ;
wire i0 ;
output i7 ;
wire i7 ;

input clk ;
wire clk ;
input [7:0] in ;
wire [7:0] in ;


reg [7:0] a;

always@(posedge clk) a<=in;
	 	
	assign i0=a[0];
	assign i1=a[1];
	assign i2=a[2];
	assign i3=a[3];	  
	
	assign i4=a[4];
	assign i5=a[5];
	assign i6=a[6];
	assign i7=a[7];

endmodule

