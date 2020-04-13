//-----------------------------------------------------------------------------
//
// Title       : led_test
// Design      : sinh1
// Author      : Microsoft
// Company     : Microsoft
//
//-----------------------------------------------------------------------------
//
// File        : led_test.v
// Generated   : Mon Jan 19 15:26:05 2015
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
//{module {led_test}}
module led_test ( clk,RX1_led, RX2_led ,led1 ,led2 ,led3 ,led4 ,led5 ,led6 ,led7 ,led8,led_POWER ,led_Avariya  );
	
	
input RX1_led ;
wire RX1_led ;
input RX2_led ;
wire RX2_led ;	
	
input led_POWER ;
wire led_POWER ;
input led_Avariya ;
wire led_Avariya ;	

output led1 ;
wire led1 ;
output led2 ;
wire led2 ;
output led3 ;
wire led3 ;
output led4 ;
wire led4 ;
output led5 ;
wire led5 ;
output led6 ;
wire led6 ;
output led7 ;
wire led7 ;
output led8 ;
wire led8 ;

input clk ;
wire clk ;

reg [31:0] a;	
reg [25:0] b;	

reg Avariya_flag;
reg POWER_flag;

reg tk1;
reg tk2;	  

reg flag1;
reg flag2;

reg  [7:0] sch1;
reg  [7:0] sch2; 	  
reg [31:0] sch3; 

reg [2:0] front1;
reg [2:0] front2;

always @(posedge clk)
	
	begin 
		
		front1<={front1[1:0],RX1_led};
	    front2<={front2[1:0],RX2_led};
		
		a<=a+1;	 
		b<=b+a;	 
		
	if (front1==3'b001) flag1<=1;	else if (sch1==50) flag1<=0;			  
	if (front2==3'b001) flag2<=1;	else if (sch1==50) flag2<=0;
		
		
	if ((flag1)&&(front1==3'b001)) sch1<=sch1+1;  else if (!flag1) sch1<=0;
	if ((flag2)&&(front2==3'b001)) sch2<=sch2+1;  else if (!flag2) sch2<=0;	  
		
	if (sch1==25) tk1<=0; else if (sch1==50) tk1<=1;
	if (sch2==25) tk2<=0; else if (sch2==50) tk2<=1;		 
		
	Avariya_flag <= led_Avariya;
	POWER_flag	 <=led_POWER;
	 	
	
	end	  
	
	assign led1=a[25];
	assign led2=a[24];
	assign led3=a[23];
	assign led4=a[22];	  	
	
	assign led5=Avariya_flag; //светодиод аварии на лицевой панели
	assign led6=POWER_flag;	  //светодиод питания на лицевой панели
	assign led7=tk1;
	assign led8=tk2;	
	

endmodule

