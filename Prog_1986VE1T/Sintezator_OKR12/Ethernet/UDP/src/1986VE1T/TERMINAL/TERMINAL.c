#ifndef __TERMINAL_c__
#define __TERMINAL_c__

#include "opora.h"
#include "TERMINAL.h"


//--- Terminal configuration ---
void Menu()
{

}


void Menu1(char a)
 
 {
//***************************************************************************

    int i;
	
 
 for (i=0;i<20;i++) sendT("\r\n");    // ������� ���������
	for (i=0; i<64; i++) sendT ("*");  // ����� �����������
	sendT("\r\n");
	sendT("\r\n");
	sendT("\r\n");
	sendT("\r\n");
	sendT("......Terminal B600-1....\r\n");
	sendT("\r\n");
	sendT("\r\n");
	sendT("\r\n");
	sendT("MENU :\r\n");
	sendT("-------\r\n");
	sendT("~ - ��������� ����\r\n");
	sendT("1 - ����� ��������\r\n");
	sendT("start_TCP -  �������\r\n");
	sendT("=30000000 - ������ �������\r\n");
	sendT(";- ����� ����� \r\n");
	sendT(".............. \r\n");
	sendT("������: \r\n");
	sendT("1 - 1�-�6xx   , ������������� ������: \r\n");
	sendT("2 - 1�-�6xx-1 , ������������� ������\r\n");
	sendT("3 - 1�-�6xx , ���������� ������� ���������  \r\n");
	sendT("4 - 1�-�6xx , ���������� ������� ����������  \r\n");
	sendT("6 - 1�-�6xx , ���  \r\n");
	sendT("+++++++++++++++++++\r\n");
	sendT("~1 watch_reg_dds;\r\n");
	sendT("~1 id_dds;\r\n");
	sendT("~1 wrn_dds_1;\r\n");
	sendT("~1 wrn_dds_0;\r\n");
	sendT("~1 init_dds;\r\n");
	sendT("~1 rd_dds;\r\n");
	sendT("~1 wr_dds=1;\r\n");
	sendT("~1 rd_tst;\r\n");
	sendT("~1 wr_tst=1;\r\n");
	sendT("~1 help;\r\n");
	sendT("~1 TEST_BUS_OUT;\r\n");
	/*
	sendT("~1 config_Ethernet;\r\n");
	sendT("~1 ���������;\r\n");
	sendT("~1 ��������;\r\n");
	sendT("~1 ���������;\r\n");
	sendT("~1 UDP_TCP_test;\r\n");
	sendT("~1 �������������;\r\n");
	sendT("~1 PPI_�����_�612;\r\n");
	sendT("~1 PPI_�����_�613;\r\n");
	*/
	sendT("~1 ��������_����������;\r\n");

	sendT("~1 ���������_���������_�����������;\r\n");
	sendT("~1 ��������_���_UP;\r\n");
	sendT("~1 ��������_���_DOWN;\r\n");
	sendT("~1 ����������_���������=0;\r\n");
	sendT("~1 ����������_F0=60000000;\r\n");
	sendT("~1 ����������_���;\r\n");
	sendT("~1 �������_����������_F���=100;\r\n");
	sendT("~1 ������������_��������_T���=100;\r\n");
	sendT("~1 ����������_�������_F_low=430000000;\r\n");
	sendT("~1 ����������_�������_F_high=440000000;\r\n");
	/*
	sendT("~3 ���������_�������_�����=100000;\r\n");
	sendT("~3 ���������_�������_����=100000;\r\n");
	sendT("~3 ���������_����_�����;\r\n");
	sendT("~3 ���������_����_����;\r\n");
	sendT("~3 ���������_���������_�����;\r\n");
	sendT("~3 ���������_���������_����;\r\n");
	sendT("~1 ������_������_ON;\r\n");
	sendT("~1 ������_������_OFF;\r\n");
	sendT("~1 SPORT_data;\r\n");
	sendT("~3 ��������_������;\r\n");
    sendT("~3 ����_������;\r\n");
 */
 /*
    sendT("~6 F64(on);\r\n");
    sendT("~6 F64(off);\r\n");
    sendT("~6 F160(on);\r\n");
    sendT("~6 F160(off);\r\n"); 
    sendT("~6 F360(on);\r\n");
    sendT("~6 F360(off);\r\n");
    sendT("~6 F400(on);\r\n");
    sendT("~6 F400(off);\r\n");   
  */
  
    sendT("~1 ADC=1;  �������� ADC ����������������\r\n");
  /*sendT("~6 �������_��������;\r\n");  
    sendT("~6 ��������_�������;\r\n");
    sendT("~6 ������_���;\r\n");
	sendT("~6 init_K611;\r\n");
	sendT("~1 K611;\r\n");
	sendT("~5 INPORT:xxxxxx;\r\n");
	sendT("~5 OUTPORT:xxxxxx;\r\n");
	*/
	sendT("\r\n");
	sendT("\r\n");
	sendT("++++++++++++++++++++\r\n");
	sendT("\r\n");
	sendT("\r\n");
	//for (i=0; i<64; i++) zputs ("*",1);  // ����� �����������
	//for (i=0;i<10;i++) puts("\r",1);  // ������� ���������
	sendT("\r\n");
	//*******************************************************************************


	
	}



#endif // __TERMINAL_c__
