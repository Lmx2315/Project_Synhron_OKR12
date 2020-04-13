#include "spec_1508pl10.h"
#include "MDR32F9Qx_port.h"

   

void Write_FOCH1 (reg_1508pl10 a)
{
	unsigned short temp1=0xffff;
	unsigned short temp2=0xffff;
	unsigned short temp3=0xffff;


	temp3  = a.Kosn00_15;

	temp2  = (a.FD		  <<14)
			|(a.Kb		  <<13)
			|(a.Kp		  <<12)
			|(a.Kok		  <<11)
			|(a.Kpo		  <<10)
			|(a.Klt		  <<9)
			|(a.KREF4	  <<8)
			|(a.KREF3	  <<7)
			|(a.Kosn16_19 <<3)
			|(a.KREF2	  <<2)
			|(a.KREF1     <<1)
			|(a.KREF0	  <<0);

	temp1  = (a.T_Amp_pres<<5)
			|(a.T_Fop_o	  <<4)
			|(a.T_vr      <<3)
			|(a.T_Pd      <<2)
			|(a.T_Del_m   <<1)
			|(a.Kz        <<0);


	
    //CS_SPI1 (1);
	SPI1_send(~temp1);
	SPI1_send(~temp2);
	SPI1_send(~temp3);
	CS_SPI1 (0);
	CS_SPI1 (1);

}


void Write_FOCH2 (reg_1508pl10 a)
{
	unsigned short temp1=0xffff;
	unsigned short temp2=0xffff;
	unsigned short temp3=0xffff;

	temp3  = a.Kosn00_15;

	temp2  = (a.FD		  <<14)
			|(a.Kb		  <<13)
			|(a.Kp		  <<12)
			|(a.Kok		  <<11)
			|(a.Kpo		  <<10)
			|(a.Klt		  <<9)
			|(a.KREF4	  <<8)
			|(a.KREF3	  <<7)
			|(a.Kosn16_19 <<3)
			|(a.KREF2	  <<2)
			|(a.KREF1     <<1)
			|(a.KREF0	  <<0);

	temp1  = (a.T_Amp_pres<<5)
			|(a.T_Fop_o	  <<4)
			|(a.T_vr      <<3)
			|(a.T_Pd      <<2)
			|(a.T_Del_m   <<1)
			|(a.Kz        <<0);
			
	
   // CS_SPI2 (1);
	SPI2_send(~temp1);
	SPI2_send(~temp2);
	SPI2_send(~temp3);
	CS_SPI2 (0);
	CS_SPI2 (1);
}