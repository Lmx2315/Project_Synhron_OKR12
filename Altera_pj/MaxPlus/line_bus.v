//-----------------------------------------------------------------------------
//
// Title       : line_bus
// Design      : sinh1
// Author      : Microsoft
// Company     : Microsoft
//
//-----------------------------------------------------------------------------
//
// File        : line_bus.v
// Generated   : Fri Jan 23 18:18:50 2015
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
//{module {line_bus}}
module line_bus ( clk ,i0 ,i1 ,i2 ,i3 ,i4 ,i5 ,i6 ,i7 ,out );

output [7:0] out ;
wire [7:0] out ;

input clk ;
wire clk ;
input i0 ;
wire i0 ;
input i1 ;
wire i1 ;
input i2 ;
wire i2 ;
input i3 ;
wire i3 ;
input i4 ;
wire i4 ;
input i5 ;
wire i5 ;
input i6 ;
wire i6 ;
input i7 ;
wire i7 ;

reg [7:0] a;

always @(posedge clk) a<={i7,i6,i5,i4,i3,i2,i1,i0};
	
	assign out=a;

endmodule

