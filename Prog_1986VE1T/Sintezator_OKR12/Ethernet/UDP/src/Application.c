#ifndef __Application_c__
#define __Application_c__

#include "Application.h"
#include "opora.h"
#include "CAN1.h"
#include "RST_CLK.h"
#include "ETHERNET.h"
#include "PORTA.h"
#include "PORTB.h"
#include "PORTC.h"
#include "PORTD.h"
#include "PORTE.h"
#include "PORTF.h"
#include "TERMINAL.h"
#include "DDS1508pl1t.h"
#include "adc.h"
#include "math.h"

#include "MAC.h"
#include "ARP.h"
#include "IP.h"
#include "UDP.h"
#include "ICMP.h"


#define PA0_0 PORTA->CLRTX = (1<<0) //������������� ����    PA[0] , ���������� 
#define PA0_1 PORTA->SETTX = (1<<0) //������������� ������� PA[0],  ����������

#define PA1_0 PORTA->CLRTX = (1<<1) //������������� ����    PA[1] , ���������� 
#define PA1_1 PORTA->SETTX = (1<<1) //������������� ������� PA[1],  ����������

#define PA2_0 PORTA->CLRTX = (1<<2) //������������� ����    PA[2] , ���������� 
#define PA2_1 PORTA->SETTX = (1<<2) //������������� ������� PA[2],  ����������

#define PA3_0 PORTA->CLRTX = (1<<3) //������������� ����    PA[3] , ���������� 
#define PA3_1 PORTA->SETTX = (1<<3) //������������� ������� PA[3],  ����������

#define PA4_0 PORTA->CLRTX = (1<<4) //������������� ����    PA[4] , ���������� 
#define PA4_1 PORTA->SETTX = (1<<4) //������������� ������� PA[4],  ����������

#define PA5_0 PORTA->CLRTX = (1<<5) //������������� ����    PA[5] , ���������� 
#define PA5_1 PORTA->SETTX = (1<<5) //������������� ������� PA[5],  ����������

#define PA6_0 PORTA->CLRTX = (1<<6) //������������� ����    PA[6] , ���������� 
#define PA6_1 PORTA->SETTX = (1<<6) //������������� ������� PA[6],  ����������

#define PA7_0 PORTA->CLRTX = (1<<7) //������������� ����    PA[7] , ���������� 
#define PA7_1 PORTA->SETTX = (1<<7) //������������� ������� PA[7],  ����������

#define PA8_0 PORTA->CLRTX = (1<<8) //������������� ����    PA[8] , ���������� 
#define PA8_1 PORTA->SETTX = (1<<8) //������������� ������� PA[8],  ����������

#define PA9_0 PORTA->CLRTX = (1<<9) //������������� ����    PA[9] , ���������� 
#define PA9_1 PORTA->SETTX = (1<<9) //������������� ������� PA[9],  ����������


#define PA10_0 PORTA->CLRTX = (1<<10) //������������� ����    PA[10] , ���������� ������� DD2 ������ DE_485 - ��������� �� ��������
#define PA10_1 PORTA->SETTX = (1<<10) //������������� ������� PA[10], ���������� ������� DD2 ������ DE_485 - ��������� �� ���� 

#define PA11_0 PORTA->CLRTX = (1<<11) //������������� ����    PA[10] , ���������� ������� DD2 ������ DE_485 - ��������� �� ��������
#define PA11_1 PORTA->SETTX = (1<<11) //������������� ������� PA[10], ���������� ������� DD2 ������ DE_485 - ��������� �� ���� 

#define POWER_ON PA1_1
#define POWER_OFF PA1_0


#define PF2_0 PORTF->CLRTX = (1<<2) //������������� ����    PF[2] , ���������� 
#define PF2_1 PORTF->SETTX = (1<<2) //������������� ������� PF[2],  ����������

#define PF3_0 PORTF->CLRTX = (1<<3) //������������� ����    PF[3] , ���������� 
#define PF3_1 PORTF->SETTX = (1<<3) //������������� ������� PF[3],  ����������

#define PF4_0 PORTF->CLRTX = (1<<4) //������������� ����    PF[4] , ���������� 
#define PF4_1 PORTF->SETTX = (1<<4) //������������� ������� PF[4],  ����������

#define PF5_0 PORTF->CLRTX = (1<<5) //������������� ����    PF[5] , ���������� 
#define PF5_1 PORTF->SETTX = (1<<5) //������������� ������� PF[5],  ����������

#define PF6_0 PORTF->CLRTX = (1<<6) //������������� ����    PF[6] , ���������� 
#define PF6_1 PORTF->SETTX = (1<<6) //������������� ������� PF[6],  ����������

#define PF7_0 PORTF->CLRTX = (1<<7) //������������� ����    PF[7] , ���������� 
#define PF7_1 PORTF->SETTX = (1<<7) //������������� ������� PF[7],  ����������


#define htons(A) ((((uint16_t)(A) & 0xff00) >> 8) | (((uint16_t)(A) & 0x00ff) << 8))

IP_Address MyIP   = { 192, 168, 1, 163 };
IP_Address HostIP = { 192, 168, 1, 1 };

UDP_Port p2054 = { 0x08, 0x06 };
UDP_Port pCB99 = { 0xCB, 0x99 };

#pragma pack ( push, 2)
const MAC_Address MyMAC = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB };
#pragma pack (pop)

_Rec_Frame Rec;

//--------------------------------------------------------------------------------------------------
// DDS_1508pl1t

  ShiftReg pins_dds=     {PORTE, 0, //���� 16 ��� ������
                          PORTC,15, //���� 16 ��� ������ ������
                          PORTD,15, //����� ������
                          PORTD, 6, //����� ������
                          PORTD, 0, //����� ����������
                          PORTD, 5, //����� SCSN
                          PORTD, 1  //����� ������
                          };    //����� ���������� ������������ ������ DDS		

    
  DDS_param	 DDS_data= {
								1,
						435000000, //������� ����������������� � ��
						430000000, // ������� ������ � ��
						440000000, // ������� ������� � ��
							  100, // ������������ �������� � ���
						    10000, // ������� ���������� ��������� � ��
							    1, // �������� ��������� � ��
							    1  // ����������� ��� 1 ��� -1
						};
//-------------------------------------------------------------------------------------------------------

// UART

 unsigned char DataTr_flag=0;
 unsigned char Data_in_port=0;
 unsigned char DontConnect_flag=0;      // ���������� � PC ����������
 unsigned      CntDown = 0;             // ������������ ��� ������. ������� ��� ����. ������.-
 unsigned char CyclEnd_flag=0;          // ���� pRcv_Buf �������� ����� RcvBuf
                                        // ��� BufIsRead_flag=0
 unsigned char RcvBufOverflow_flag=0;   // ����� �� ������ ���� � ������ ��������     
                                        // 
 unsigned char BufIsRead_flag=0;        // ���������, ����� ������� ��� ����. ������
 unsigned char Data4TrAbsent_flag=1;    // ������ ��� �������� ��������.
#define buf_size 256
 unsigned char TrBuf[buf_size];
 unsigned char RcvBuf[buf_size];
 unsigned char sch_buf=0;

                                         // ���. ��� ����������� ���������. �������
                                        // ������ � ��������
 unsigned char *pTr_Buf;                // ����. �� ����. �����.-�� � ����
 unsigned char *pcur_Tr;                // ����. �� ����. ������������� � �����.���
 unsigned char *pRcv_Buf;           
 unsigned char *pcur_Rcv;

 void fillBuf  (unsigned char* ptrBuf, unsigned short lengthBuf, unsigned char symb );
 void clearBuf (unsigned char* ptrBuf, unsigned short lengthBuf );
 void zputs    (unsigned char* s, unsigned  s_size );
 int getStr    (unsigned char* s, unsigned char* s_size );

  //  UART



#define Bufer_size  128     //16384

volatile  unsigned int  text_lengh;


unsigned int j_pack=0;
unsigned char packet[4][32];
unsigned char text_buffer[Bufer_size];
unsigned char flag_clear=0;
unsigned char ok[5];
unsigned int led_tick;
int *pointer1;
unsigned adspChipVersion;
unsigned nn=0;
unsigned tt;
unsigned tested;
short sch_uart;
unsigned char  sch_plis_ppi=0;
unsigned char  flag_test_sync=0;
unsigned char  sch_packet_UDP_reciv=0;


unsigned char lsr=0;
unsigned char lk;


//-------------------------------------
uint32_t Temp, CurrentLed = 0;
uint32_t i=0;

unsigned short flag_rsv=0;
//-------------------------------------

#define dT_kalibrovka 304
#define dT_pomeha 512   
#define Test_delay 5
#define  u16 unsigned short
#define  u32 unsigned int   
#define  u8  unsigned char
#define  CPUacceler          1     //������ BF533



//*************************************************************************

#define BUF_RX 64

   	   unsigned char  k;

  unsigned  	char lsym;
  unsigned  	char  sym;
  unsigned  	char flag;
  unsigned  	char packet_flag;
  unsigned      char	NB;

  unsigned    char Adress=0x34;      // ����� �������� Adress=0x33 - ������� �����������
  unsigned    char Master_flag=0x0;  // ���� ������������ ������ �������-1,0-��������������� �������������  --�������� ������ ����� ���� ��� ������

  unsigned    char packet_sum;
  volatile   unsigned  	char crc,comanda=1;
      char     InOut[BUF_RX];
 
      char      Word [BUF_RX];    //������ ���������� �����
      char DATA_Word [BUF_RX];    //������ ����� - ������
      char DATA_Word2[BUF_RX];    //������ ����� - ������
	  
			char s1[BUF_RX];
			char s2[BUF_RX];
			char s3[BUF_RX];
			char sr[BUF_RX];
			
  unsigned    char crc_ok;
  unsigned    char packet_ok;
  unsigned    char ink1; // ������� ���� � ������
  unsigned    char data_in;
  unsigned	  char index_word=0;
  unsigned    char index_data_word =0;
  unsigned    char index_data_word2=0;
  unsigned    char data_flag=0;
  unsigned    char data_flag2=0;
  unsigned    char crc_input;	
  unsigned    char crc_comp;	
  unsigned    char sch_obmen=0;
  unsigned    char   Process_code=0;
  unsigned    char   Test_f_mono=0;

volatile  unsigned int   sch_avariya=0;
  
      		     
 unsigned volatile int  time_uart; // ���������� �������� �������
 unsigned volatile int  tick_wait;
			
		unsigned  char UDP_TCP_flag=0;
		
                     unsigned char CRC_m[3][6]; //������ �������
                     unsigned char K615_indik=0;	
              volatile       unsigned char K615_crc_sch=0;   
              volatile       unsigned char K615_crc_sch2=0;  
			
                    unsigned  char Error_ethernet_obmen=0; //���������� �������� ������ ������
 volatile unsigned  char flag_pachka_sinhron; //���� ���������� ������� ����� ����� ��������� � ������ � �������������

 volatile  unsigned  char flag_pachka_TXT; //���� ������������ ��� ��� �������� ����� ������
 volatile  unsigned  char flag_pachka_TXT2; //���� ������������ ��� ��� �������� ����� ������

 volatile  unsigned  int sch_pachek_test=0; //������� ����������� �������  � �������� ������������ ��� ��� �������� ����� ������


  unsigned volatile char          flag_K615_event;  //event �� �������� � ������� 615

 volatile   unsigned char flag_cikl_ON=0;
 volatile   unsigned char flag_Ethernet_packet_rcv=0;
        volatile              int sch_UDP_pakets=0;

        unsigned char flag_process=1;
        unsigned char flag_event_K615_run=0;
        unsigned int  delay_process=400;

        unsigned char flag_uart_trcv=0;

        unsigned char Pachka_START=0;
        unsigned char   sync_flag=0;

      volatile  unsigned char   sync_sch_flag=3;
      unsigned volatile char tick_us;

    unsigned char sinc_type=0;

   volatile unsigned char  flag_MASTER_pachka=0;
   volatile unsigned int   TNC_number_run=0;
            unsigned char  flag_form_packet_SDRAM=0;
            unsigned char Test_bus_out_FLAG=0;


 //*************************************************************************    
 
 unsigned  char index1   =0;
 unsigned  char lsym1    =0;
 unsigned  char pack_ok1 =0;
 unsigned  char pack_sum1=0;   
 unsigned  char sym1     =0;  


unsigned int Alfa =0;  //���������� ������� ��� ������������ ����� ���������������
unsigned int Beta =0;
unsigned int Gamma=0;

unsigned short TNC_actual  =0; //������� ���	
unsigned short TNO_actual  =0; //������� ���
unsigned int   Time_Preset =0; //����������������� ����� � ���


unsigned int   TNI_a[32];   //������� ���������
unsigned int   TKI_a[32];
unsigned int   TNP_a[32];
unsigned int   TKP_a[32];
unsigned int   TNO_a[32];
unsigned int   TNC_a[32];
unsigned int  TOBM_a[32];
unsigned int  TDDS_a[32];
/*
unsigned int   TNI_b[32];   //������� ���������
unsigned int   TKI_b[32];
unsigned int   TNP_b[32];
unsigned int   TKP_b[32];
unsigned int   TNO_b[32];
unsigned int   TNC_b[32];
unsigned int  TOBM_b[32];
*/

unsigned int   tin_k=0;  //�������� ���������
unsigned int   tik_k=0;  //�������� ���������
unsigned int   tpn_k=0;
unsigned int   tpk_k=0;
unsigned int tdds_k=0;
unsigned int  tno_k=0;
unsigned int  tnc_k=0;

unsigned int  TNO_work1=0;
unsigned int  TNO_work2=0;
unsigned char flag_TNO_work2=0; //���� ��������������� � ������ ������ ������ ����� ���������� ������� ���
unsigned int dFo1;

 
 //****************************DSP***********************************************   
  char   dma_buf;		

   unsigned char A[256];  //������ ��������� ��� ��������������
   unsigned char SintezI[256];  //������   ��� c����������1
   unsigned char SintezG[256];  //������   ��� c����������2

  unsigned int  Inf_A1[1]; // ������ ������ ���������� ���������� �� ��� 3  1�07��
  unsigned char Inf_A2[15];  // ������ ������ ���������� ���������  �� ��� 3  1�07��
  //unsigned int  Inf_A3[31]; // ������ ������ ���������� ������������ ������ �� ��� 3  1�07��
  
  unsigned char Code_error=0;       //��� ������ D15?D12 � � 1�08�� �� ������������.
  unsigned char Norm_ohlagdeniya=0; //����� ���������� D11 � ����� ���������� - ���. �1� .

  unsigned char PriemNi=0;          //���. �2  �� D10 � ���������� � �������� ������ � �������,
// ��� ���. "0" � ������� D8 - ���������� ��� ���� ������, ��� ���.  �1�- ��� ������.
// D9 � ���������� � �������� ������ �� �2, ��� ���.
// "0" � ������� D8 -���������� ��� ���� ��-����, ��� ���. �1�- ��� ������.

  unsigned char Vikl=0;             //���.���. 
  unsigned char GBR=0;              //���.���. 
  unsigned char BR=0;               //���.���. 
  unsigned char CU_MU=0;            //���.���. 
  unsigned char T_norma=0;          // ����� ����������� ����������� ������������ 1�07�� (���. �1�).
  unsigned char Pred_avariya=0;     //���������� ����������� ������������ 1�07�� (���.  �1�).
  
  unsigned char Norma_amplifer=0;   //D13?D10 � ������� ������� � ����� �� �������� D10-D13,
// ������� � ���, ��� ������ ���� �� ��������� �� ������� ������. 
// ������ D10 ������������� 1 ������, ����������� ���.2, �����-���� �� � ���.6 (��. ������� 3.8).
//  � ������ ������������������� ��������� �� (� ���������-������ ���� ���������� ����) �� ������,
//   1�07�� ������������� �� ��������� ��� 1 ������.

  unsigned char Norma_pitaniyaFT=0;  //D9?D6 � ������� ������� ������� � ����� ������� ��������� ��� � ��������������� ����-��.
 //��� ��������� ��������� (���������� ������� �� �� ����� �� ���������� ��) �����-�� ����� ���� �� ���� ��������.
 
  unsigned char Avariya_blokov=0;  // �600  �605  �604  �603  �602  �601,������� ������ ����� 1�07�� (���.  �1�) .
  
  unsigned char Rezerv1=0;          //
  
  unsigned char Norma_IVE=0;       //����� ��� ����� 1�07�� - (���.  �1�).
  
  unsigned char Rezerv2=0;          //
  
  unsigned char Nomer_rezer_amplifer_FT=0;     //D3?D0 � ������ D0 ������������� ������, ����������� ���.2,
// ����.1 ��� � ���.6 (��. ����-��� 3.8).
// ������� ���� � ������ ������� ������� � ���, ��� �������� ������ ��������� �� ���-���� ������,
// �.�. �� ����� �� ����� � ����� 1 �������� �� ��������� ��������� ���. �����-����� � ��� ������ ��������.

//������:
//?	���� � ������� D10 ����� 02h �������, � � ������� D0 ����� 04h ����, �� ������ �������� ������� � ���, ��� �������� ��������� �� 1 ������. 
//?	���� � ������� D10 ����� 02h ����, � � ������� D0 ����� 04h �������, �� ������ �������� ������� � ���, ��� �������� ��������� ��� (��������� �� � ����������) 1 ������. 
//?	���� � ������� D10 ����� 02h ����, � � ������� D0 ����� 04h ����, �� ������ �������� ������� � ���, ��� �������� ��������� �� � ���. � ����� ������ ���� � ������� D6 ����� 02h ����, �� ��������� ����������� ��� 1 ������. ���� � ������� D6 ����� 02h �������, �� ��������� ������������ ����������� 1 ������.

//..................��� � �������-���������� ��� ������� ������ ����� � �������� 5 ����� (��. ������� 3.4)........................................

 unsigned char Knc_min=0;    //  ��� �����
 
 unsigned char Knc_10sec=0;  //  ��� �������� ������

 unsigned char Knc_sec=0;    //  ���  ������

 unsigned char Knc_100ms=0;  //  ��� ����� ��
 
 unsigned char Knc_10ms=0;   //  ��� �������� ��
 
 unsigned char Knc_ms=0;     //  ���  ��
 
 unsigned char Knc_100us=0;  //  ��� ����� ���
 
 unsigned char Knc_10us=0;   //  ��� �������� ���
 
 unsigned char Knc_us=0;     //  ���  ���

/* 
��� ����� � ��� ���������� ����� � ������� �����������.
��� �������� ������ � ��� ���������� �������� ������ � ������� ������.
��� ������ � ��� ���������� ������ � ������� �������.
��� ����� �� - ��� ���������� ������ �� � ������� �������.
��� �������� �� - ��� ���������� �������� �� � ������� �����.
��� ������ �� - ��� ���������� �� � ������� �������.
��� ����� ��� - ��� ���������� ������ �� � ������� ��.
��� �������� ��� - ��� ���������� �������� ��� � ������� �����.
��� ������ ���- ��� ���������� ��� � ������� �������.
  */

  //�������������� ��� �������� ���������� � ���������� ��� �������� ������� (������, ����, �����), ������������� �������.
  
 unsigned char Mypr_min=0;    //  ������
 
 unsigned char Mypr_hour=0;   //  ����
 
 unsigned char Mypr_day=0;    //  �����
 
 //..................................................................
 
 
  unsigned char NNI=0;        			 //������� ������� ���������� ���;
  
  unsigned char RC_KTV_T5M=0; 			 //��� 2 � ��������������� ����� ��� ������������ �5�; 
  
  unsigned char Tc_2sec=0;    			 //��� 3 � ��   2 ���. � ������������ ����� ������ 2 ���.;
  
  unsigned char SintV_nesootv_zadan=0;   //��� 4 � �������������� ��������� ������� �� ������������� ��������;
  
  unsigned char In_sys_Obmen_error=0;   //����� � 1��203 �������, �� �� ������������� �������� ���������;
  
  unsigned char Sboy_form_interval=0;   //��� 6 � ��������� ���� ������������ ����������;
  
  unsigned char Sboy_obmena_inform=0;   //��� 7� ��������� ���� ������ �����������.
  
  unsigned char FAPCH_1=0;   			//��� 0 - �������� ���� ���������� f� (1�-�605);
  
  unsigned char FAPCH_2=0;   			//��� 1 - �������� ���� ���������� f� (1�-�603);
  
  unsigned char Zakon_modulac_kontrol=0;//�������� ������� �� �������������� ������� ��������� (1�-�605, 1�-�606, 1�-�608);
  
  unsigned char FAPCH_3=0;   			//��� 3 - �������� ���� ���������� f� (1�-�606);
  
  unsigned char LCHM_osnv=0;   		    	//���0 � �������/���������� ������ ��� ��������� �������
  
  unsigned char Dop_interval=0;   		//���1 � �������/���������� ��������������� ���������
  
  unsigned char Type_pachki=0;   		//���2 1 � ����������� / 0 - �� ����������� (��� N>0) ����� ��������
  
  unsigned char LCHM_pomeha=0;   		//���3 � �������/���������� ��� � ������� �������
  
  unsigned char SYNC_T0Tnc=0;   		//���7 � �������/���������� �������������   ��  

  unsigned char LCHM_osnv2=0;   		    	//���0 � �������/���������� ������ ��� ��������� �������
  
  unsigned char Dop_interval2=0;   		//���1 � �������/���������� ��������������� ���������
  
  unsigned char Type_pachki2=0;   		//���2 1 � ����������� / 0 - �� ����������� (��� N>0) ����� ��������
  
  unsigned char LCHM_pomeha2=0;   		//���3 � �������/���������� ��� � ������� �������
  
  unsigned char SYNC_T0Tnc2=0;   		//���7 � �������/���������� �������������   ��  

  unsigned char LCHM_osnv3=0;   		    	//���0 � �������/���������� ������ ��� ��������� �������
  
  unsigned char Dop_interval3=0;   		//���1 � �������/���������� ��������������� ���������
  
  unsigned char Type_pachki3=0;   		//���2 1 � ����������� / 0 - �� ����������� (��� N>0) ����� ��������
  
  unsigned char LCHM_pomeha3=0;   		//���3 � �������/���������� ��� � ������� �������
  
  unsigned char SYNC_T0Tnc3=0;   		//���7 � �������/���������� �������������   ��  

  unsigned char TNO_reg=1;              //������� ������������ ������ ���
  
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  unsigned char P0=0;  //������������ ������ (�������), ���������� �������� ������� ����� ��������� ���������� ��� , Do = ��-1
  unsigned int Pniu=0;  //�������� �������, 
//  ��������� ��� ���������� ����������� (DDS) � ������ 
//  (������������� ��������� �� ���������� ���������� �� ��������� ����������). 
//  ������ ��������� ������� ��� �� �����, ��� �� 8 ��� D��� = ����/8, D���<D��

  unsigned int Pni=0;  //�������� ������� ������������� ������������ ���������� �������� ���, 
//  ���� �� �������� ��� (��� ������� ��������� ��������� � ������ ������������ ��������������� ���������)
//  �� �������� ���, ������������ ������ ������ ��������� D�� = ���/8, D��>D���

 unsigned int Pii=0;  //�������� ������� �� ��� �� ���, � ������� �������� ���������� ��������� �������, D��� = ����/8
 
 unsigned int Pnpu=0;  //�������� �������, 
//  ��������� ��� ���������� ����������� (DDS) � ������ 
//  (������������� ��������� �� ���������� ���������� �� ��������� ����������). 
//  ������ ��������� ������� ��� �� �����, ��� �� 8 ��� D��� = ����/8, D���<D��

 unsigned int    Pnp=0;  //�������� ������� ����� ��� � ���, D�� = ���/8, D��>D���
 
 unsigned int    Pip=0;  //�������� ������, ������������ ������������ ������. ������ �������� ������� ����� ���������� ��� � ��� D�� = ���/8 - 16
 
 unsigned int Pdop=848;  //������������ ��������������� ���������
 
 unsigned int  Pnpk=16;  //�������� ����� ���������� ��� � ��� ����������
 
 unsigned int   Pk=304;   //������������ ��������� ����������
 
 unsigned int  Pnpp=16;  //�������� ����� ���������� ��� ���������� � ��� ������ ������
 
 unsigned int   Pp=512;  //������������ ��������� ������ ������
 
 unsigned char  Ntp=1;  //����� ������ �  ����� D�� = N�� , ������������ �������� = 8
 
 unsigned char   Nt=1;  //����� ����� � ����� D� = N�-1 , ������������ �������� = 3

 unsigned char  Ntc=1;  //����� ������ � �����

unsigned char  n1,n2,n3=1;  //����� n ��� ������� ������ ��� � ���������
 //  �� (��� ���(0) ������ ���� �����  �1�) 
 //  ������������ ��������� ��������� ������ ������������� �������:

//--------------------������� 2 -------------------------------------------------


  unsigned int Pniu2=0;  //�������� �������, 
//  ��������� ��� ���������� ����������� (DDS) � ������ 
//  (������������� ��������� �� ���������� ���������� �� ��������� ����������). 
//  ������ ��������� ������� ��� �� �����, ��� �� 8 ��� D��� = ����/8, D���<D��

  unsigned int Pni2=0;  //�������� ������� ������������� ������������ ���������� �������� ���, 
//  ���� �� �������� ��� (��� ������� ��������� ��������� � ������ ������������ ��������������� ���������)
//  �� �������� ���, ������������ ������ ������ ��������� D�� = ���/8, D��>D���

 unsigned int Pii2=0;  //�������� ������� �� ��� �� ���, � ������� �������� ���������� ��������� �������, D��� = ����/8
 
 unsigned int Pnpu2=0;  //�������� �������, 
//  ��������� ��� ���������� ����������� (DDS) � ������ 
//  (������������� ��������� �� ���������� ���������� �� ��������� ����������). 
//  ������ ��������� ������� ��� �� �����, ��� �� 8 ��� D��� = ����/8, D���<D��

 unsigned int    Pnp2=0;  //�������� ������� ����� ��� � ���, D�� = ���/8, D��>D���
 
 unsigned int    Pip2=0;  //�������� ������, ������������ ������������ ������. ������ �������� ������� ����� ���������� ��� � ��� D�� = ���/8 - 16
 
 unsigned int Pdop2=848;  //������������ ��������������� ���������
 
 unsigned int  Pnpk2=16;  //�������� ����� ���������� ��� � ��� ����������
 
 unsigned int   Pk2=304;   //������������ ��������� ����������
 
 unsigned int  Pnpp2=16;  //�������� ����� ���������� ��� ���������� � ��� ������ ������
 
 unsigned int   Pp2=512;  //������������ ��������� ������ ������
 
 unsigned char  Ntp2=1;  //����� ������ �  ����� D�� = N�� , ������������ �������� = 8
 
 unsigned char   Nt2=1;  //����� ����� � ����� D� = N�-1 , ������������ �������� = 3

 unsigned char  Ntc2=1;  //����� ������ � �����


//----------------------������� 3-----------------------------------

 
  unsigned int Pniu3=0;  //�������� �������, 
//  ��������� ��� ���������� ����������� (DDS) � ������ 
//  (������������� ��������� �� ���������� ���������� �� ��������� ����������). 
//  ������ ��������� ������� ��� �� �����, ��� �� 8 ��� D��� = ����/8, D���<D��

  unsigned int Pni3=0;  //�������� ������� ������������� ������������ ���������� �������� ���, 
//  ���� �� �������� ��� (��� ������� ��������� ��������� � ������ ������������ ��������������� ���������)
//  �� �������� ���, ������������ ������ ������ ��������� D�� = ���/8, D��>D���

 unsigned int Pii3=0;  //�������� ������� �� ��� �� ���, � ������� �������� ���������� ��������� �������, D��� = ����/8
 
 unsigned int Pnpu3=0;  //�������� �������, 
//  ��������� ��� ���������� ����������� (DDS) � ������ 
//  (������������� ��������� �� ���������� ���������� �� ��������� ����������). 
//  ������ ��������� ������� ��� �� �����, ��� �� 8 ��� D��� = ����/8, D���<D��

 unsigned int    Pnp3=0;  //�������� ������� ����� ��� � ���, D�� = ���/8, D��>D���
 
 unsigned int    Pip3=0;  //�������� ������, ������������ ������������ ������. ������ �������� ������� ����� ���������� ��� � ��� D�� = ���/8 - 16
 
 unsigned int Pdop3=848;  //������������ ��������������� ���������
 
 unsigned int  Pnpk3=16;  //�������� ����� ���������� ��� � ��� ����������
 
 unsigned int   Pk3=304;   //������������ ��������� ����������
 
 unsigned int  Pnpp3=16;  //�������� ����� ���������� ��� ���������� � ��� ������ ������
 
 unsigned int   Pp3=512;  //������������ ��������� ������ ������
 
 unsigned char  Ntp3=1;  //����� ������ �  ����� D�� = N�� , ������������ �������� = 8
 
 unsigned char   Nt3=1;  //����� ����� � ����� D� = N�-1 , ������������ �������� = 3

 unsigned char  Ntc3=1;  //����� ������ � �����

 
  //*****************����� ���������***********************************
  
 volatile unsigned char               Flag_K611=0;
 volatile unsigned char          Flag_init_K611=0;
 volatile unsigned char       Flag_control_K611=0;
 volatile unsigned char   Flag_control_sig_K611=0;
 volatile unsigned char   Flag_control_end_K611=0;
 volatile unsigned char    Flag_zahvat_sig_K611=0;
 volatile  unsigned char   Flag_zahvat_end_K611=0;
 volatile unsigned char              Qwant_K611=0;
 volatile unsigned char     Flag_zahvat_OK_K611=0;//����� ���� ������� �������� �������
 volatile unsigned char     Flag_signal_OK_K611=0;// ����� ���� ��� ��� ������� � �����
  
 volatile unsigned char                 Flag_K612=0;
 volatile unsigned char               Flag_init_K612=0;
 volatile unsigned char            Flag_control_K612=0;
 /*
  unsigned char         Flag_vkl_sintez_K612=0;
  unsigned char        Flag_vkl_LCHM_UP_K612=0;
  unsigned char      Flag_vkl_LCHM_DOWN_K612=0;
  unsigned char           Flag_SETUP_F0_K612=0;
  unsigned char        Flag_SETUP_F_low_K612=0;
  unsigned char       Flag_SETUP_F_high_K612=0;
  unsigned char         Flag_SDVIG_F_UP_K612=0;
  unsigned char       Flag_SDVIG_F_DOWN_K612=0;
  unsigned char      Flag_SDVIG_Faza_UP_K612=0;
  unsigned char    Flag_SDVIG_Faza_DOWN_K612=0;
  unsigned char         Flag_SDVIG_A_UP_K612=0;
  unsigned char       Flag_SDVIG_A_DOWN_K612=0;
  unsigned char          Flag_FFT_start_K612=0;
  unsigned char           Flag_FFT_stop_K612=0;
  unsigned char                   Qwant_K612=0;
  */
  
 volatile unsigned char                    Flag_K613=0;
 volatile unsigned char               Flag_init_K613=0;
 volatile unsigned char            Flag_control_K613=0;
 /*
  unsigned char         Flag_vkl_sintez_K613=0;
  unsigned char        Flag_vkl_LCHM_UP_K613=0;
  unsigned char      Flag_vkl_LCHM_DOWN_K613=0;
  unsigned char           Flag_SETUP_F0_K613=0;
  unsigned char        Flag_SETUP_F_low_K613=0;
  unsigned char       Flag_SETUP_F_high_K613=0;
  unsigned char         Flag_SDVIG_F_UP_K613=0;
  unsigned char       Flag_SDVIG_F_DOWN_K613=0;
  unsigned char      Flag_SDVIG_Faza_UP_K613=0;
  unsigned char    Flag_SDVIG_Faza_DOWN_K613=0;
  unsigned char         Flag_SDVIG_A_UP_K613=0;
  unsigned char       Flag_SDVIG_A_DOWN_K613=0;
  unsigned char          Flag_FFT_start_K613=0;
  unsigned char           Flag_FFT_stop_K613=0;
  unsigned char                   Qwant_K613=0;
  */
  
 volatile unsigned char                    Flag_K615=0;
 volatile unsigned char               Flag_init_K615=0;
 volatile unsigned char            Flag_control_K615=0;
 volatile unsigned char                   Qwant_K615=0;

 volatile unsigned char                    Flag_K614=0;
 volatile unsigned char               Flag_init_K614=0;
 volatile unsigned char            Flag_control_K614=0;
 volatile unsigned char                   Qwant_K614=0;
 volatile unsigned char              flag_event_K614=0;
  
 volatile unsigned char                  PROCESS=0;

  unsigned char   test1=0;
  unsigned char   test2=0;

 volatile unsigned char sys_life_k612=0;
 volatile unsigned char sys_life_k613=0;

volatile  unsigned char AVARIYA_flag=0;

 unsigned volatile int  tick_process;
 unsigned volatile int  tick_process_K611;
 unsigned volatile int  tick_process_K615;
 unsigned volatile int  tick_process_K612;
 unsigned volatile int  tick_process_K613;
 unsigned volatile int  tick_process_OK;
 unsigned volatile int  tick_TCP;
 unsigned volatile int  tick_UDP;

  
 unsigned volatile char flag_Ethernet;  //���� ���������� ��� ������ ����� �� ��������
       unsigned volatile char flag_Ethernet_Terminal=0;  //���� ���������� ��� �������� ����� � ��������� , �������.

     
   volatile    unsigned char flag_PPI_sz1 =0;
   volatile    unsigned char flag_PPI_sz2 =0;
   volatile    unsigned char flag_PPI_sinc=0;
   volatile    unsigned char flag_contr_TNC_TNO=0;
   volatile    unsigned char flag_Packet_form=0;

   unsigned char flag_PPI_START=0;


       unsigned char label_PPI=0;
  	   unsigned char Test_PPI_flag1=0;
  	   unsigned char Test_PPI_flag2=0;
  	   unsigned char Test_PPI_flag3=0;

       unsigned char RESET_SINTEZ_flag=0;

   volatile    unsigned int time_TNO=0;
   volatile    unsigned int time_TNC=0;

   volatile    unsigned int time_TNO_min=0;
   volatile    unsigned int time_TNC_min=0;

   volatile    unsigned int time_TNO_max=0;
   volatile    unsigned int time_TNC_max=0;

   volatile    unsigned char flag_1HZ_sync=0;
               unsigned char flag_START_packa_SINTEZ=0;
			          double Fget_define;
  //------------------------------------------------------------------

  
   unsigned char  strng[32];
  
  /**
 * The 8-bit signed data type.
 */
typedef char int8;
/**
 * The volatile 8-bit signed data type.
 */
typedef volatile char vint8;
/**
 * The 8-bit unsigned data type.
 */
typedef unsigned char uint8;
/**
 * The volatile 8-bit unsigned data type.
 */
typedef volatile unsigned char vuint8;

/**
 * The 16-bit signed data type.
 */
typedef int int16;
/**
 * The volatile 16-bit signed data type.
 */
//typedef volatile int vint16;
/**
 * The 16-bit unsigned data type.
 */
typedef unsigned int uint16;
/**
 * The volatile 16-bit unsigned data type.
 */
//typedef volatile unsigned int vuint16;
/**
 * The 32-bit signed data type.
 */
typedef short int32;
/**
 * The volatile 32-bit signed data type.
 */
//typedef volatile long vint32;
/**
 * The 32-bit unsigned data type.
 */
typedef unsigned short uint32;
/**
 * The volatile 32-bit unsigned data type.
 */
//typedef volatile unsigned long vuint32;
    
typedef uint8			u_char;		/**< 8-bit value */
typedef uint8 			SOCKET;
typedef uint16			u_short;	/**< 16-bit value */
typedef uint16			u_int;		/**< 16-bit value */
typedef uint32			u_long;		/**< 32-bit value */
typedef uint8 			SOCKET;	
  
  
  typedef struct                //flags
{
  u8 Kod_zameny     : 4;        //  ��� ������  4-�� ����
  u8 KCH_OZU        : 4;        //  ����������� ������ ��� 4-�� ����
  u8 Kol_error      : 2;        //  ���������� ������ 2-�� ����
  u8 Nomer_prov_cepi: 4;        //  ����� ����������� ���� 4-�� ����
  u8 Kod_error      : 4;        //  ��� ������ 2-�� ����

  u16    Nomer_TNC     :16;        // ����� ��������� ���
  u16    Nomer_TNO     :16;        // ����� ��������� ���
 // int32  Time_SEV      :32;        // ����� ��� , � ��������.



/*
�������� ���������� - �������� ������ � ���������.
______________________________________________________________________________
D15 D14 D13 D12    D11 D10 D9 D8    D7  D6    D5  D4  D3  D2        D1  D0
  ��� ������    |    ������      |   ���.   | ����� �����������  | ��� ������
                                    ������         ����
��  2�  1�  0�     ��  2�  1�  0�   1�  0�    ��  2�  1�  0�        1�  0�
------------------------------------------------------------------------------

D15-D12- �������� ������ ������� ��� ������� �� �2 ��� ������ .
D7, D6 - ������� ������ � ���������� �� �������� ��� ������: 
��� 00 - ��� ������, ��� 01- ������ �� �2 � ������ ������� ������,
��� 10 - ������ �� �2 � ������ � ������ �������� ������,
��� 11 - ������ �� �2 � ���� �������� ������.
D5-D0 � � ������ 1�08�� �� ������������.


*/

  u8 Zadano_off     : 4;        //  ������ ����  , 4-�� ����
  u8 Zadano_GBR     : 4;        //  ������ ���   , 4-�� ����
  u8 Zadano_BR      : 4;        //  ������ ��    , 4-�� ����
  u8 Zadano_CU_MU   : 4;        //  ������ ��/�� , 4-�� ����
  
/*
�������� ���������� - ����� ������� � ����� ���������� ����� 1�07��.

D15 D14 D13 D12  D11 D10 D9  D8   D7  D6  D5  D4     D3  D2  D1  D0
����           |      ���       |       ��        |     ��/��
3�  2�  1�  0�  3�  2�  1�  0�    3�  2�  1�  0�      3� 2�  1�  0�

D15-D12, Dl 1-D8, D7-D4 - ������� ������� - ��� 0101,
 ������ ������� - ��� 1100, 
 ��� ����-����� - ��� 0000.
  D3-D0-��-��� 0101,
   ��-��� 1100.
   ��������� ���������� - ������ �������.

*/

 u16 Kod_Dniu        : 16;       //  ��� D���
 
 u16 Kod_Dniu2       : 16;       //  ��� D���
 
 u16 Kod_Dniu3       : 16;       //  ��� D���

/*�������� ������� D��� ,
 ��������� ��� ���������� ����������� (DDS)
  � ������ (������������� ��������� �� ���������� ���������� �� ��������� ����������).
 16-��������� ��� ������������ ������������ ��������� ����� ������� ���������.
   ������ ��������� ������� ��� �� �����, ��� �� 8 ��� 
   ��������� �������:   D��� = ����/8, D���<D��


      */


 u16 Kod_Dii         : 16;       //  ��� D��
 
 u16 Kod_Dii2        : 16;       //  ��� D��
 
 u16 Kod_Dii3        : 16;       //  ��� D��
 
 /*
 16 ��������� ��� ������������ ��������� ���������.
�������� ������� �� ��� �� ���, � ������� �������� ���������� ��������� �������.
������� : D��� = ����/8;  ���, ��� ; ����������� �������� 128 ��� , ������� 8 ���.

0A  |  ��� D��-���� 1 (����� 7) � ���.1  ��� D��-���� 2 (����� 6) � ���.1
    |  7�  6�  5�  4�  3�  2�  1�  0�    7�  6�  5�  4�  3�  2�  1�  0�


 */

u16 Kod_Dni         : 16;       //  ��� D��

u16 Kod_Dni2        : 16;       //  ��� D��

u16 Kod_Dni3        : 16;       //  ��� D��

 /*

�������� ������� ���, ���  (������� ��������� 8 ���)
 ������������� 
 ������������ ���������� �������� ���,
  ���� �� �������� ��� (��� ������� ��������� ��������� � ������ ������������ ��������������� ���������)
   �� �������� ���, ������������ ������ ������ ���������

 */

 u16 Kod_Dnp         : 16;       //  ��� D��
 
 u16 Kod_Dnp2        : 16;       //  ��� D�� 
 
 u16 Kod_Dnp3        : 16;       //  ��� D�� 

/*
16 ��������� ��� ������������ ��������� ����� ������� ������
���, ��� ����������� �������� 16 (������� ��������� 8 ���)
�������� ������� ����� ��� � ���
�������:D�� = ���/8, D��>D���
*/

 u16 Kod_Dnpu        : 16;       //  ��� D���
 
 u16 Kod_Dnpu2       : 16;       //  ��� D���
 
 u16 Kod_Dnpu3       : 16;       //  ��� D���

/*
����, ���  (������� ��������� 8 ���)
�������� �������,
 ��������� ��� ���������� ����������� (DDS)
  � ������ 
  (������������� ��������� �� ���������� ���������� �� ��������� ����������). 
  ������ ��������� ������� ��� �� �����, ��� �� 8 ���

  �������: D��� = ����/8, D���<D��

*/

 u16 Kod_DN         : 16;       //  ��� Dn
 
/*

8 ��������� ��� ����� ���������� �� ������������ ��������� � �����.
���������� ��������� � ����� ����� N+1 ;

  12              ������              ��� DN-���� 2 (����� 16)
  7�  6�  5�  4�  3�  2�  1�  0�  |  7�  6�  5�  4�  3�  2�  1�  0�



*/

 u16 Kod_Dip         : 16;       //  ��� D��
 
 u16 Kod_Dip2        : 16;       //  ��� D��
 
 u16 Kod_Dip3        : 16;       //  ��� D��
 
/*
���, ���  (������� ��������� 8 ���), ����������� �������� 128
16 ��������� ��� ������������ ��������� ������.
 ������������ ������������ ������.
  ������ �������� ������� ����� ���������� ��� � ���
  �������: D�� = ���/8 - 16


*/

  u8 Kod_Kp_hour    : 8;        //  ��� �� - ����  (����� 21)  
  u8 Kod_Kp_day     : 8;        //  ��� �� - ����� (����� 20) 

/*
������������ � ������ ��, ������� ���� ���� ������ � ������ �� ������ ���� ��������� �0�.
*/

  u8 Kod_Dm         : 8;        //  ��� D�         (����� 19) 
  
/*
8 ��������� ��� ���������� ������ � �����.
��� ������ ���������� ������� �������, ��������� �� ���� ����������:
- ���������, �����������  N+1 ���������� ����������, ������������ ������ ���, ���, ���, ���, �N;
- ���������������, ������� ����� ���� ����� ���� ����, ���� 848 ���.
 ��������� ����� ��������� ������������ ���������� � � ���� ���.

 15         ��� D� (����� 19)
      7�  6�  5�  4�  3�  2�  1�  0�



*/

 u8 Kod_Do         : 8;        //  ��� Do         (����� 18) -

  /*
    ����������� �������� = 1 ���, ������������ 16 ���
    4 ��������� ��� ������������ ������ (���� 0, 1, 2, 3). ���� 4, 5, 6, 7 ��������.
   ������������ ������, ���������� �������� ������� ����� ��������� ���������� ���,
   (�������� ����������� ���������) - �� (� ��������), Do = ��-1 - ������� ������� 

  14        ��� D� (����� 18)
      7�  6�  5�  4�  3�  2�  1�  0�


   */

  u8 Kod_Dpp1        : 8;        //  ��� D��        (����� 29) � ���.1
  u8 Kod_Dpp2        : 8;        //  ��� D��        (����� 29) � ���.2
  u8 Kod_Dpp3        : 8;        //  ��� D��        (����� 29) � ���.3 
  
/*
8 ��������� ��� ������ ����� (1/0) (��� ���� ��������� ��������� � �����)

1B       ��� Dpp (����� 29)
      7�  6�  5�  4�  3�  2�  1�  0�
47       ��� Dpp (����� 29)
      7�  6�  5�  4�  3�  2�  1�  0�
67       ��� Dpp (����� 29)
      7�  6�  5�  4�  3�  2�  1�  0�

  ���0 � �������/���������� ������ ��� ��������� �������
  ���1 � �������/���������� ��������������� ���������
  ���2 � ����������� /�� ����������� (��� N>0) ����� ��������
  ���3 � �������/���������� ��� � ������� �������
  ���7 � �������/���������� �������������  T0 ��  Ti0
  ���� 4,5,6,7 ����� ���� (������������ � ������� ����������)


*/

  u8 Kod_Dn1         : 8;        //  ��� D�         (����� 28)
  u8 Kod_Dn2         : 8;        //  ��� D�         (����� 28)
  u8 Kod_Dn3         : 8;        //  ��� D�         (����� 28) 

/*
�������, ������������ ��������� ��������� ��������.
1A      ��� Dn (����� 28) � ��� 1
      7�  6�  5�  4�  3�  2�  1�  0�
46      ��� Dn (����� 28) � ��� 2
      7�  6�  5�  4�  3�  2�  1�  0�
66      ��� Dn (����� 28) � ��� 3
      7�  6�  5�  4�  3�  2�  1�  0�

8 ��������� ��� �����  n (��� ���� ��������� ��������� � �����).
��� ������� ������ ��� � ���������   �� (��� ���(0) ������ ���� �����  �1�)
 ������������ ��������� ��������� ������ ������������� �������:

 ��� ������� ������ ��� � ���������  2-10**6 �� (��� ���(0) ������ ���� �����  �1�)
  ������������ ��������� ��������� ������ ������������� �������:
 Tm=dT*2**i*n (���),
��� n=2,3,...255
 .
����� i  � n  ������������ ...n-1
 .



*/

  u8  Kod_Krp        : 8;        //  ��� ���        (����� 23) 
  u32 Kod_Kp         : 24;        //  ��� ��- ������ (����� 22) 
  
/*
16    ��� ��-���� 2 (����� 21)       ��� ��-���� 3 (����� 20)
      7�  6�  5�  4�  3�  2�  1�    0�  7�  6�  5�  4�  3�  2�  1�  0�
18    ��� ��� (����� 23)             ��� ��-���� 1 (����� 22)
      7�  6�  5�  4�  3�  2�  1�    0�  7�  6�  5�  4�  3�  2�  1�  0�


������������ � ������ ��, ������� ���� ���� ������ � ������ �� ������ ���� ��������� �0�.
*/

unsigned long Kod_Ddi_48      : 32;       //  ��� D�� ����� 2 - 5 (����� 35)(����� 34)(����� 33)(����� 32)   
  u16 Kod_Ddi_16     : 16;       //  ��� D�� ����� 0,1   (����� 37)(����� 36)  
 
unsigned long Kod_Ddi_48_2     :32;       //  ��� D�� ����� 2 - 5 (����� 35)(����� 34)(����� 33)(����� 32)   
u16 Kod_Ddi_16_2     : 16;       //  ��� D�� ����� 0,1   (����� 37)(����� 36)
 
unsigned long Kod_Ddi_48_3    : 32;       //  ��� D�� ����� 2 - 5 (����� 35)(����� 34)(����� 33)(����� 32)   
u16 Kod_Ddi_16_3     : 16;       //  ��� D�� ����� 0,1   (����� 37)(����� 36) 

unsigned int Kod_Ddi_32      : 32;       //  ��� D�� ����� 2 - 5 (����� 35)(����� 34)(����� 33)(����� 32)   

unsigned int Kod_Ddi_32_2     :32;       //  ��� D�� ����� 2 - 5 (����� 35)(����� 34)(����� 33)(����� 32)   

unsigned int Kod_Ddi_32_3    : 32;       //  ��� D�� ����� 2 - 5 (����� 35)(����� 34)(����� 33)(����� 32)   


/*
48 ��������� ��� �������� ������� ���������.
���   ������ ���� ����� 1 (������� ���), � ��������� ������ ��� 0 .

����� ���� ��̻ ��������� � ������,
 ����� ��������� ����� ������ ������������ �������, �.�. ��� ������.
  � ���� ������ ���   ������ ���� ��� ����� �����  �1�.
����� ������ �������������� �����,
 ������� ������ ������ �� �������� ������� ������� ��������� ������������ �����,
  �������������� ���������� �� (�������� ��������� ����-��).

  1C      ��� D��-���� 5 (����� 33) � ���.1       ��� D��-���� 6 (����� 32) � ���.1
         7�  6�  5�  4�  3�  2�  1�              0�  7�  6�  5�  4�  3�  2�  1�  0�
  1E      ��� D��-���� 3 (����� 35) � ���.1       ��� D��-���� 4 (����� 34) � ���.1
         7�  6�  5�  4�  3�  2�  1�              0�  7�  6�  5�  4�  3�  2�  1�  0�
  20      ��� D��-���� 1 (����� 37) � ���.1       ��� D��-���� 2 (����� 36) � ���.1
         7�  6�  5�  4�  3�  2�  1�              0�  7�  6�  5�  4�  3�  2�  1�  0�


*/

 u32 Kod_Dchi        : 24;       //  ��� D�� (����� 39)(����� 38)(����� 40)
 
 u32 Kod_Dchi2       : 24;       //  ��� D�� (����� 39)(����� 38)(����� 40) 
 
 u32 Kod_Dchi3       : 24;       //  ��� D�� (����� 39)(����� 38)(����� 40) 
 
/*
24 ��������� ��� ��������� ������� ���������.
��������� ������� ��������� ������������ ����������:
 (430*10**6 + b*df)��,
���  df=10^7/2^10 ��. �������� 9765,625 ��.
����� �������, ��� ������������ ����   ���������� ���������� �����  :
 ,
��� ����  .


*/

unsigned long Kod_Ddg_48        : 32;       //  ��� D�� (����� 53)(����� 52)(����� 51)       
 u16 Kod_Ddg_16        : 16;       //  ��� D�� (����� 50)(����� 49)(����� 48)  
 
unsigned long Kod_Ddg_48_2      : 32;       //  ��� D�� (����� 53)(����� 52)(����� 51)       
 u16 Kod_Ddg_16_2      : 16;       //  ��� D�� (����� 50)(����� 49)(����� 48)  
 
unsigned long Kod_Ddg_48_3      : 32;       //  ��� D�� (����� 53)(����� 52)(����� 51)       
 u16 Kod_Ddg_16_3      : 16;       //  ��� D�� (����� 50)(����� 49)(����� 48)  
 
unsigned int Kod_Ddg_32        : 32;       //  ��� D�� (����� 53)(����� 52)(����� 51)       
 
unsigned int Kod_Ddg_32_2      : 32;       //  ��� D�� (����� 53)(����� 52)(����� 51)       
 
unsigned int Kod_Ddg_32_3      : 32;       //  ��� D�� (����� 53)(����� 52)(����� 51)       
 

/*
48 ��������� ��� �������� ������� ���������� �� ��������� ������.
���   ������ ���� ����� 1 (������� ���).


*/

 u32 Kod_Dchg        : 24;       //  ��� D�� (����� 56)(����� 55)(����� 54)
 
 u32 Kod_Dchg2       : 24;       //  ��� D�� (����� 56)(����� 55)(����� 54)
 
 u32 Kod_Dchg3       : 24;       //  ��� D�� (����� 56)(����� 55)(����� 54)
 

/*
24 ��������� ��� ��������� ������� ������������� ������� �� ��������� ������.
������� ������������� ������� �� ��������� ������ ������������ ����������:
  (��), 

2A       ��� D��-���� 3 (����� 54) � ���.1 
         7�  6�  5�  4�  3�  2�  1�  0�                
2C       ��� D��-���� 1 (����� 56) � ���.1    ��� D��-���� 2 (����� 55) � ���.1
         7�  6�  5�  4�  3�  2�  1�  0�         7�  6�  5�  4�  3�  2�  1�  0�
*/

 u32 Kod_Dchp       : 24;       //  ��� D�� (����� 58)(����� 57)  
 
/*
24 ��������� ��� ������� ������������� ������� �� ��������� ������� 
���   ������ ���� ��������� ������  �1� (������� ��������������� ���������).
������� ������������� ������� �� ��������� ������� ������������ ����������:
*/

 u32 Kod_Dchk       : 24;       //  ��� D�� (����� 62)(����� 61)(����� 60)
 
/*
24 ��������� ��� ������� ������������� ������� �� ��������� �����������.
���   ������ ���� ��������� �  �1� (������� ��������������� ���������).

30       ���  D��-���� 3 (����� 60)  
       7�  6�  5�  4�  3�  2�  1�  0�                
32       ���  D��-���� 1 (����� 62)       ���  D��-���� 2 (����� 61)
       7�  6�  5�  4�  3�  2�  1�  0�    7�  6�  5�  4�  3�  2�  1�  0�


*/

 unsigned long Kod_Ddp_48      : 32;       //  ��� D�� (����� 66)(����� 65)(����� 64) 
 u16 Kod_Ddp_16      : 16;       //  ��� D�� (����� 69)(����� 68)(����� 67) 
 
 unsigned int Kod_Ddp_32      : 32;       //  ��� D�� (����� 66)(����� 65)(����� 64) 

 /*

48 ��������� ��� �������� ������� ������������� ������� �� ��������� �������.
��� ����   ���3 ������ ���� ��������� �  �1� � �������/���������� ��� � ������� �������.

 */


} Inf_packet;

   
  typedef struct                //���������� ��������� ��������� ������ � ��������� �����
{
  u16 code_00h     : 16;  
/*
�������� ���������� - ����������� ��������� ���������� ����� 1�07��.

D15 D14 D13 D12        D11        D10 D9  D8       D7  D6      D5  D4      D3  D2      D1  D0
��� ������             ����.
                       ���.      ����� (��)         ����         ���        ��         ��/��
3�  2�  1�  0�                   ���. �2  ��       ���.���.    ���.���.    ���.���.    ���.���.

D15?D12 � � 1�08�� �� ������������.
D11 � ����� ���������� - ���. �1� .
D10 � ���������� � �������� ������ � �������,
 ��� ���. "0" � ������� D8 - ���������� ��� ���� ������, ��� ���.  �1�- ��� ������.
D9 � ���������� � �������� ������ �� �2, ��� ���.
 "0" � ������� D8 -���������� ��� ���� ��-����, ��� ���. �1�- ��� ������.
D7?D2 � ��������� ���������: ��� 11 - ������� ������ � ���������, 
��� 00 - ��� ������� � ��-�������� �������,
 ��� 01 - ��� ���������� �������, ��� 10 - ��� ������� �������.
Dl?D0 - ��� 11 - �� ����� � ��������,
 ��� 00 - �� ����� � ��������, ��� 01 - �� ��������,
  �� �����, ��� 10 - �� ��������, �� �����.


*/

 u16 code_02h   :16;
 
 /*
�������� ���������� - ����������� ��������� ����������

D15     D14           D13 D12 D11 D10        D9  D8  D7  D6         D5    D4    D3   D2    D1  DO
����.��              ����� ���. ������.     ����� ������� ��                 ������ ������
���� t  ��             4   3   2   1          4   3   2  1         �600  �605  �604  �603  �602  �601

D15 � ����� ����������� ����������� ������������ 1�07�� (���. �1�).
D14 � ���������� ����������� ������������ 1�07�� (���.  �1�).
D13?D10 � ������� ������� � ����� �� �������� D10-D13,
 ������� � ���, ��� ������ ���� �� ��������� �� ������� ������. 
 ������ D10 ������������� 1 ������, ����������� ���.2, �����-���� �� � ���.6 (��. ������� 3.8).
  � ������ ������������������� ��������� �� (� ���������-������ ���� ���������� ����) �� ������,
   1�07�� ������������� �� ��������� ��� 1 ������.
D9?D6 � ������� ������� ������� � ����� ������� ��������� ��� � ��������������� ����-��.
 ��� ��������� ��������� (���������� ������� �� �� ����� �� ���������� ��) �����-�� ����� ���� �� ���� ��������.
D5?D0 � ������� ������ ����� 1�07�� (���.  �1�) .


 */ 
 
  u16 code_04h   :16;
 
 /*
�������� ���������� - ����������� ��������� ����������

   D15 - D9        D8       D7 - D5                D3 - D0	
   ������      ���� ���    ������        ����� ��������� �����-����� ��
 

D8 � ����� ��� ����� 1�07�� - (���.  �1�).
D3?D0 � ������ D0 ������������� ������, ����������� ���.2,
 ����.1 ��� � ���.6 (��. ����-��� 3.8).
 ������� ���� � ������ ������� ������� � ���, ��� �������� ������ ��������� �� ���-���� ������,
 �.�. �� ����� �� ����� � ����� 1 �������� �� ��������� ��������� ���. �����-����� � ��� ������ ��������.

������:
?	���� � ������� D10 ����� 02h �������, � � ������� D0 ����� 04h ����, �� ������ �������� ������� � ���, ��� �������� ��������� �� 1 ������. 
?	���� � ������� D10 ����� 02h ����, � � ������� D0 ����� 04h �������, �� ������ �������� ������� � ���, ��� �������� ��������� ��� (��������� �� � ����������) 1 ������. 
?	���� � ������� D10 ����� 02h ����, � � ������� D0 ����� 04h ����, �� ������ �������� ������� � ���, ��� �������� ��������� �� � ���. � ����� ������ ���� � ������� D6 ����� 02h ����, �� ��������� ����������� ��� 1 ������. ���� � ������� D6 ����� 02h �������, �� ��������� ������������ ����������� 1 ������.

 */ 

 u8 code_05h   :8;
 u8 code_06h   :8;
 u8 code_07h   :8;
 u8 code_08h   :8;
 u8 code_09h   :8;

 
  /*
��� � �������-���������� ��� ������� ������ ����� � �������� 5 ����� (��. ������� 3.4).

7�	6�	5�	4�	3�	2�	1�	0�
��� �����
X	X	X	X	X	2	1	0
7�	6�	5�	4�	3�	2�	1�	0�
��� �������� ������	��� ������
3	2	1	0	3	2	1	0

7�	6�	5�	4�	3�	2�	1�	0�
��� ����� ��	��� �������� ��
3	2	1	0	3	2	1	0
7�	6�	5�	4�	3�	2�	1�	0�
��� ������ ��	��� ����� ���
3	2	1	0	3	2	1	0
7�	6�	5�	4�	3�	2�	1�	0�
��� �������� ���	��� ������ ���
3	2	1	0	3	2	1	0

��� ����� � ��� ���������� ����� � ������� �����������.
��� �������� ������ � ��� ���������� �������� ������ � ������� ������.
��� ������ � ��� ���������� ������ � ������� �������.
��� ����� �� - ��� ���������� ������ �� � ������� �������.
��� �������� �� - ��� ���������� �������� �� � ������� �����.
��� ������ �� - ��� ���������� �� � ������� �������.
��� ����� ��� - ��� ���������� ������ �� � ������� ��.
��� �������� ��� - ��� ���������� �������� ��� � ������� �����.
��� ������ ���- ��� ���������� ��� � ������� �������.

 */ 

   u8 code_0Ah   :8;
   u8 code_0Bh   :8;
   u8 code_0Ch   :8;
 /*  
   �������������� ��� �������� ���������� � ���������� ��� �������� ������� (������, ����, �����), ������������� �������.
 */
 
   u8 code_0Dh   :8;
   u8 code_0Eh   :8;
   
   /*
   ��� � ��� ��������������� ��������
7�	6�	5�	4�	3�	2�	1�	0�
��� � (����� 88)
7	6	5	4	3	2	1	X

0Dh


1 ���� 88 ������ ����������� (�� 1�07�� �� 1�09�):
��� 0 � ��������� ;
���1- ������� ������� ���������� ���;
��� 2 � ��������������� ����� ��� ������������ �5�; 
��� 3 � ��   2 ���. � ������������ ����� ������ 2 ���.;
��� 4 � �������������� ��������� ������� �� ������������� ��������;
��� 5 � ����� � 1��203 �������, �� �� ������������� �������� ���������;
��� 6 � ��������� ���� ������������ ����������;
��� 7� ��������� ���� ������ �����������.
7�	6�	5�	4�	3�	2�	1�	0�
��� � (����� 89)
X	X	X	X	3	2	1	0

0Eh

1 ���� 89 ������ ����������� (�� 1�07�� �� 1�09�):
��� 0 - �������� ���� ���������� f� (1�-�605);
��� 1 � �������� ���� ���������� f��. (1�-�603);
��� 2 � �������� ������� �� �������������� ������� ��������� (1�-�605, 1�-�606, 1�-�608);
��� 3 � �������� ���� ���������� f� (1�-�606);
���� 4, 5, 6, 7 ��������.


   
   */

} Inf_sys_packet;

 
  typedef struct                //flags
{
  u8 byte1      : 8;        //  ��� ������  4-�� ����
  u8 byte2      : 8;        //  ����������� ������ ��� 4-�� ����
  u8 byte3      : 8;        //  ���������� ������ 2-�� ����
  u8 byte4      : 8;        //  ����� ����������� ���� 4-�� ����
  u8 byte5      : 8;        //  ��� ������ 2-�� ����
  u8 byte6      : 8; 

  u8  VCU:1; // ������� ���
  u8   MU:1; // ������� �� 
  
  u8  GBR:1; // ������� ���
  u8   BR:1; // ������� �� 
  u8 VYKL:2; // ������� ����
  u8 Ohl :1; // ��������� ����������
 
  
  
} Init_K615_struct;

 Inf_packet         Mem1 ; // �������� ��������� Mem1
 Inf_sys_packet     Mem_sys;
 Init_K615_struct   Mem_K615;  // �������� ��������� ��� ������� �615
 
      	
  unsigned char DSP_work[256];
  short triger1;
  char dma_buf;
  char work=0;
 
 unsigned int dsp_index;
 unsigned int dsp_chanal;
volatile unsigned char flag_dsp_ok;
 unsigned int chanal;
 unsigned int flag_data_transfer; 
 
 unsigned int flag_data_transfer_r; 
 unsigned int flag_fs3; 
 unsigned int schet_control; 
 
 char vspom;  

//***********************���������� �������*********************************

unsigned char  C_t_day =0;  //������� 
unsigned char  C_t_hour=0;
unsigned char  C_t_min =0;
unsigned char  C_t_sec =0;

unsigned short C_t_ms  =0;
unsigned int   C_t_us  =0;


unsigned char  C_t_dayX =0; //����� ������������
unsigned char  C_t_hourX=0;
unsigned char  C_t_minX =0;
unsigned char  C_t_secX =0;

unsigned short C_t_msX  =0;
unsigned int   C_t_usX  =450;

//***************************************************************************

void delay(int inc)
{
//---------------------------------------------------------------
   inc <<= 3;
   while(inc !=0) inc--;
//---------------------------------------------------------------
}


  // reverse:  �������������� ������ s �� ����� 
 void reverse(char s[])
 {
     int i, j;
     char c;
 
     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
 }

 // itoa_t:  ������������ n � ������� � s 
 void itoa_t(int n, char s[])
 {
     int i, sign;
 
     if ((sign = n) < 0)  // ���������� ���� 
         n = -n;          // ������ n ������������� ������ 
     i = 0;
     do {       // ���������� ����� � �������� ������� 
         s[i++] = n % 10 + '0';   // ����� ��������� ����� 
     } while ((n /= 10) > 0);     // ������� 
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
 }

unsigned int leng (unsigned char *s)

  {
		unsigned  char i=0;
		 while ((s[i]!='\0')&&(i<120)) { i++;}
		return i;
  }


//----------------------------------------



void zputc (unsigned long c) 
{
	while((UART1->FR&(1<<5)) == (1<<5));	//wait until FIFO Tx full
	UART1->DR = c; //�������� ������
	while((UART1->FR&(1<<3)) == (1<<3));	//wait until Tx transmit data
}

 void zputs(unsigned char *s, unsigned l)
{
      unsigned i;
      for (i=0;i<l;i++) {zputc ((s[i]));}
}


void sendT (unsigned char * s)

 {
       zputs(s,leng(s));

 }




void zputc2 (unsigned long c) 
{
	while((UART2->FR&(1<<5)) == (1<<5));	//wait until FIFO Tx full
	UART2->DR = c; //�������� ������
	while((UART2->FR&(1<<3)) == (1<<3));	//wait until Tx transmit data
}

 void zputs2(unsigned char *s, unsigned l)
{
      unsigned i;
      for (i=0;i<l;i++) {zputc2 ((s[i]));}
}

void sendT2 (unsigned char * s)

 {
       zputs2(s,leng(s));

 }


 void Transf( char* s)  // ��������� �������� ������ �������� � ���� 
   {
       unsigned  short l=0;
       unsigned  short i=0;
         
	  // TX_485;
        sendT(s);	//uart1
       sendT2(s);   //uart2
	  // RX_485;
  }
   

 void ZTransf( char* s,unsigned char a )  // ��������� �������� ������ �������� � ���� 
   {
      unsigned  char i;
	 
	//  i=leng(s);
   
      zputs(s,a);
      zputs2(s,a);
      
  }
   
void TX_485 (void)
{
	PA10_1;
	PA11_1;
}

void RX_485 (void)
{
	PA10_0;
	PA11_0;
}



void DDS_setup_ind(void)

{
 Transf ("-----------------------------------");
 Transf ("\r\n");
 Transf ("��������� �����������:\r\n" );
 
 Transf ("����� ������ �����������:" );
 
 if (DDS_data.regim==1) { Transf ("������ ���\r\n"); }
 if (DDS_data.regim==0) { Transf ("������ ���������\r\n"); }
  
 Transf ("\r\n");
  
 Transf ("������� ����������� F0:" );
 sprintf(strng,"%d",DDS_data.F0);
 Transf(strng);
 Transf ("\r\n");
 
 Transf ("������� ������   F_low:" );
 sprintf(strng,"%d",DDS_data.F_low);
 Transf(strng);
 Transf ("\r\n");
 
 Transf ("������� ������� F_high:" );
 sprintf(strng,"%d",DDS_data.F_high);
 Transf(strng);
 Transf ("\r\n");
 Transf ("\r\n");
 
 Transf ("������������ ��������� � ���:" );
 sprintf(strng,"%d",DDS_data.dImp);
 Transf(strng);
 Transf ("\r\n");
 Transf ("\r\n");
 
 Transf ("������� ���������� ��������� � ��:" );
 sprintf(strng,"%d",DDS_data.FImp);
 Transf(strng);
 Transf ("\r\n");
 Transf ("\r\n");
 
 Transf ("���������, ���������� � ��:" );
 sprintf(strng,"%d",DDS_data.A);
 Transf(strng);
 Transf ("\r\n");
 Transf ("\r\n");
 
 Transf ("����������� ���� ���:" );
 sprintf(strng,"%d",DDS_data.znak_LCHM);
 Transf(strng);
 Transf ("\r\n");
 Transf ("-----------------------------");
 Transf ("\r\n");

}

void ADC_test (unsigned char ch)

{
	unsigned int a;
	float z;
	
	a=ADC_m(ch);
	
	 Transf ("ADC:");	
	 sprintf(strng,"%d",a);
            Transf(strng);
          Transf ("\r\n");
		  
		  z=(3.3/4096)*a;
		  
		   Transf ("U(2):");	
	 sprintf(strng,"%f",z);
            Transf(strng);
          Transf ("\r\n");


}

unsigned int DB_INT(unsigned int db)
{
  unsigned int A=0;
  float x=0;

  x = (float) db;
  x = x/10;
  x = pow (10,x);

  A = (unsigned int) x;

  A = 1024 * A;

  if (A>32767) A = 32767;

  return A;
}



void IO (char* sr_io)      // ������� ��������� ��������� ������
 
 {
       
 unsigned char s[6];   
 unsigned char tst[1];  
 unsigned char i=0;
 unsigned char index=0;
 unsigned char dlsr=0;
  char z;
  char p[1];
  unsigned char  z_k611=0;
  char sys_char=0;
  int l=0;

  unsigned short PF1_var=0;
  unsigned short PF2_var=0;
  unsigned short PF3_var=0;
  unsigned short PF4_var=0;
  unsigned int Z_sum=0;
  unsigned short temp_short=0;
  
  i=leng(sr_io);
     
   sym1=sr_io[0];
   
   p[0]=sym1;   
   
   
if ((sym1==0x7e)||(time_uart>100)) 
	{
		time_uart=0;  //��������� �������� ���� ����
		packet_flag=1; 
		index1=0; 
		crc=0; 
		crc_ok=0; 
		packet_ok=0; 
		index_word=0; 
		index_data_word =1;
        index_data_word2=1;
		data_flag =0;
        data_flag2=0;
		DATA_Word [0]=' ';
        DATA_Word2[0]=' ';
		
	} // ���������� ������ ������

if (packet_flag==1)

{

	while (i>0)   //���������� ��������� ������ � ������ ���������
	
	{

	   InOut[index1]=sr_io[index];
	   sr_io[index]=0x0;
	  
	  if  (InOut[index1]==' ')  ink1=ink1+1;
	  
	  if  (InOut[index1]==';')  packet_ok=1;
	  
	  if ((InOut[index1]=='=')||(InOut[index1]==':')) data_flag=1;

      if  (InOut[index1]=='.') data_flag2=1;
	  

        	  if ((index1>2)&&(InOut[2]==' '))  
        		{
        		
                		 if  ( (InOut[index1]!=' ')&&
                               (InOut[index1]!=';')&&
                               (data_flag!=1))	       Word[index_word]=InOut[index1]; // ��������� ��������� �����
                		
                   
                        	   if  ((data_flag==1)&&
                                 (InOut[index1]!=' ')&&
                                 (InOut[index1]!=';')&&
                                 (InOut[index1]!='=')&&
                                 (InOut[index1]!=':')&&(data_flag2==0))    	DATA_Word[index_data_word]=InOut[index1]; // ���������  ����� ������1
                      
                   
                             if  ((data_flag==1)&&
                                 (InOut[index1]!=' ')&&
                                 (InOut[index1]!=';')&&
                                 (InOut[index1]!='=')&&
                                 (InOut[index1]!=':')&&(data_flag2==1))     DATA_Word2[index_data_word2]=InOut[index1]; // ���������  ����� ������2
                		      
                	   
                             if ((data_flag!=1)&&
                                 (InOut[index1]!='=')&&
                                 (InOut[index1]!=':'))      index_word=index_word+1; 
                            		   else 
                                            {
                                             
                                             if ((data_flag==1)&&
                                                 (InOut[index1]!='=')&&
                                                 (InOut[index1]!=':')&&
                                                 (InOut[index1]!='.')&&
                                                 (data_flag2==0))      index_data_word=index_data_word+1;
                                            
                                             if ((data_flag==1)&&
                                                 (InOut[index1]!='=')&&
                                                 (InOut[index1]!=':')&&
                                                 (InOut[index1]!='.')&&
                                                 (data_flag2==1))      index_data_word2=index_data_word2+1;
                                            }
        	  	           
        		}
				
        		index1=index1+1;
        		index =index +1;
        		i=i-1;
	
	   }
	
	i=index1;
	
if (packet_ok==1) 
	{
	
	    if (InOut[0]==0x7e)   crc_ok=crc_ok|0x1;   // �������� ������� ������� ������ - ������ ������
	  	if (InOut[1]==Adress) crc_ok=crc_ok|0x2;   // �������� ������� ������� ������ - ������� ����������
	
	if (crc_ok==0x3)  //��������� ������ ��������� ������� �������� ������ 
    	{
		
					if (strcmp(Word,"help")==0)        				      

					  { Transf ("������ help\r"    ); 
						  Menu1('t');	   }
						  
					if (strcmp(Word,"help2")==0)        				      

					  { Transf ("������ help2\r"    ); 
						  Menu1('t');	   }
                  		
                  //		if (strcmp(Word,  "������_������_ON" )==0)      { Transf ("������ ������_������_ON\r"  ); AVARIYA_on ;}
                  //		if (strcmp(Word,  "������_������_OFF")==0)      { Transf ("������ ������_������_OFF\r"  ); AVARIYA_off ;}

                   	  if (strcmp(Word,"TEST_BUS_OUT"  )==0)        { Transf ("������ TEST_BUS_OUT\r"  );  Test_bus_out_FLAG=1;       }//�������� ������ ������������ ���� ������	
                      if (strcmp(Word,"watch_reg_dds"  )==0)       { Transf ("������ watch_reg_dds\r"  ); reg_DDS_watch (&pins_dds); }//������ � ����� �� ����� ��������� DDS 
					 					 
					  if (strcmp(Word,"ADC"      )==0) 	    //�������� ���������� � ������ //�������� ADC ����������������
                  				{ 	dFo1=atoi(DATA_Word);
            									Transf ("������ ADC:" );
                              					Transf("DATA_Word:");
                              					ZTransf (DATA_Word,index_data_word-1);
            									Transf("\r");
            									ADC_test(dFo1);
            								 }
									 
                      if (strcmp(Word,"wrn_dds_1"      )==0)       { Transf ("������ wrn_dds_1\r"      ); PORTD->RXTX|= (1<<15); } // 
                      if (strcmp(Word,"wrn_dds_0"      )==0)       { Transf ("������ wrn_dds_0\r"      ); PORTD->RXTX&=~(1<<15); } // 
                      if (strcmp(Word,"init_dds"       )==0)       { Transf ("������ init_dds \r"       ); init_DDS (&DDS_data,&pins_dds); }         //������������� DDS 
                      if (strcmp(Word,"id_dds"         )==0)       {   //������ ������ � dds
                                                                            Transf ("������ id_dds:" );
                                                                     temp_short=Read_DDS(0x0001,&pins_dds); 
                                                                                itoa_t(temp_short,strng);
                                                                                         Transf(strng);
                                                                                         Transf ("\r");
                                                                                                       }
                    
                      if (strcmp(Word,"rd_dds"    )==0)      {   //������ ������ � dds
                                                                        Transf ("������ rd_dds:" );
                                                                 temp_short=Read_DDS(0x1020,&pins_dds); 
                                                                            itoa_t(temp_short,strng);
                                                                                     Transf(strng);
                                                                                     Transf ("\r");
                                                                                                   }
                    
                      if (strcmp(Word,"wr_dds"      )==0)      //���������� ������ �� dds
                          { temp_short=atoi(DATA_Word);
                            Transf ("������ wr_dds: \r" );
                            Transf("DATA_Word:");
                            ZTransf (DATA_Word,index_data_word-1);
                            Transf("\r");
                            Write_DDS(temp_short,0x1020,&pins_dds);
                             }

                      if (strcmp(Word,"rd_tst"    )==0)       {   //������ ������ � dds
                                                      Transf ("������ rd_tst:" );
                                               temp_short=rd_tst(0x1020,&pins_dds); 
                                                          itoa_t(temp_short,strng);
                                                                   Transf(strng);
                                                                   Transf ("\r");
                                                                                 }
                    
                      if (strcmp(Word,"wr_tst"      )==0)      //���������� ������ �� dds
                          { temp_short=atoi(DATA_Word);
                            Transf ("������ wr_tst: \r" );
                            Transf("DATA_Word:");
                            ZTransf (DATA_Word,index_data_word-1);
                            Transf("\r");
                            wr_tst(temp_short,0x1020,&pins_dds);
                             }
							 
                      if (strcmp(Word,"����������_���������"      )==0) 	    //���������� ������� F0 �����������
                  				{ 	dFo1=atoi(DATA_Word);
								              Transf ("������ ����������_���������: \r" );
										          DDS_data.A= dFo1;
										            sprintf(strng,"%d",dFo1);
													Transf(strng);
													Transf("\r");
              							     amplitud(&DDS_data,&pins_dds);  //����� ������� ������� � ���������������� DDS
										              TX_485();
										            Transf("~1 COMMAND_OK;");
                  					 }    
                 	    if (strcmp(Word,"���������_���������_�����������")==0)      { Transf ("������ ���������_���������_�����������\r\n"   );   DDS_setup_ind();	     }		   
                      if (strcmp(Word,"��������_����������"		     )==0)      { Transf ("������ ��������_����������\r"        		 );         			     }
                  		if (strcmp(Word,"��������_���_UP"    			 )==0)      { Transf ("������ ��������_���_UP    \r"       		     );   DDS_data.znak_LCHM= 1;   }
                  		if (strcmp(Word,"��������_���_DOWN"  			 )==0)      { Transf ("������ ��������_���_DOWN  \r"         		 );   DDS_data.znak_LCHM=-1;   }
                  		if (strcmp(Word,"����������_F0"     			 )==0) 	    //���������� ������� F0 �����������
                  				{ 	dFo1=atoi(DATA_Word);
								            	Transf ("������ ����������_F0: \r" );
                  					   Transf("DATA_Word:");
                  					   ZTransf (DATA_Word,index_data_word-1);
              									Transf("\r");
              									DDS_data.F0=dFo1;
              									DDS_data.regim=0;
												init_DDS (&DDS_data,&pins_dds); //������������� DDS - ����� ��������� � ����
              									Fs_calc(&DDS_data,&pins_dds);  //����� ������� ������� � ���������������� DDS
            										TX_485();
            										Transf("~1 COMMAND_OK;");
                  					 }
                  		
						if (strcmp(Word,"����������_���"      )==0) 	    //���������� ��� 
                  				{ 	dFo1=atoi(DATA_Word);
              									Transf ("������ ����������_���: \r" );
                                					Transf("DATA_Word:");
                                					ZTransf (DATA_Word,index_data_word-1);
              									Transf("\r");
              									DDS_data.regim=1;
												init_DDS (&DDS_data,&pins_dds); //������������� DDS - ����� ��������� � ����
              									Set_LCHM(&DDS_data,&pins_dds);  //����� ������� ������� � ���������������� DDS
												TX_485();
										Transf("~1 COMMAND_OK;");
                  					 }
						if (strcmp(Word,"����������_��������_���" )==0) 	   //���������� ������� F ������ ��� ��� ����������� � ��
                  				{ dFo1=atoi(DATA_Word);
                  					Transf ("������ ����������_��������_���: \r" );
                  					Transf("DATA_Word:");
                  					ZTransf (DATA_Word,index_data_word-1);
                  					Transf("\r");
            									DDS_data.F_low =DDS_data.F0-dFo1/2;
            									DDS_data.F_high=DDS_data.F0+dFo1/2;
												TX_485();
											Transf("~1 COMMAND_OK;");
        						}	
						
                      if (strcmp(Word,"����������_�������_F_low" )==0) 	   //���������� ������� F ������ ��� ��� ����������� � ��
                  				{ dFo1=atoi(DATA_Word);
                  					Transf ("������ ����������_�������_F_low: \r" );
                  					Transf("DATA_Word:");
                  					ZTransf (DATA_Word,index_data_word-1);
                  					Transf("\r");
								          	DDS_data.F_low=dFo1;
											TX_485();
										Transf("~1 COMMAND_OK;");
									}	
                  		
                      if (strcmp(Word,"����������_�������_F_high" )==0) 	   //���������� ������� F ������� ��� ��� ����������� � ��
                  				{ dFo1=atoi(DATA_Word);
                  					Transf ("������ ����������_�������_F_high: \r" );
                  					Transf("DATA_Word:");
                  					ZTransf (DATA_Word,index_data_word-1);
                  					Transf("\r");
									          DDS_data.F_high=dFo1;
									TX_485();
									Transf("~1 COMMAND_OK;");
								 }		
                   if (strcmp(Word,"������������_��������_T���" )==0) 	   //���������� ������������ �������� � ���
                  				{ dFo1=atoi(DATA_Word);
                  					Transf ("������ ������������_��������_T���: \r" );
                  					Transf("DATA_Word:");
                  					ZTransf (DATA_Word,index_data_word-1);
                  					Transf("\r");
									DDS_data.dImp=dFo1;
									TX_485();
									Transf("~1 COMMAND_OK;");
									}	
                  		
					if (strcmp(Word,"�������_����������_F���" )==0) 	   //���������� ������������ �������� � ��
                  				{ dFo1=atoi(DATA_Word);
                  					Transf ("������ �������_����������_F���: \r" );
                  					Transf("DATA_Word:");
                  					ZTransf (DATA_Word,index_data_word-1);
                  					Transf("\r");
									DDS_data.FImp=dFo1;
									TX_485();
									Transf("~1 COMMAND_OK;");
									}	
					
                  		
                  //--------------------------------------------------K612----------------------------------------------------------------				
                  		
                  		if (strcmp(Word,"���_������_K612")==0)
                  		{ 
                    	if (Master_flag==1)	Transf ("������ ��������_���������� K612  \r"  );
                  		Flag_init_K612=1;
                  		Flag_K612=0;
                  		tick_process=0;
                  		sch_obmen=0;
                  		tick_process_K612=0;
                  		} //��������� ������������� �� ����� ���������� ������� �������������
                  		
                  			
                  		if (strcmp(Word,"���������_K612")==0)
                  		{ 
                                             
                             sys_char=atoi(DATA_Word);
                     
                  	    
                        if (Master_flag==1)  
                          {
                              Transf ("������ ��������� K612:"  );
                              itoa_t(sys_char,s3);
                              Transf(s3);
                              Transf("\r\n");

                              if ((sys_char&0x4)==0x4)  Transf ("TEMP-OK\r"  ); 
                              if ((sys_char&0x2)==0x2)  Transf ("Voltag-OK\r"); 
                              if ((sys_char&0x1)==0x1)  Transf ("Level_F-OK\r"); 
                          }
                                               

                      //  Transf ("������ ��������� K612\r\n"  );

                        sys_life_k612=sys_char;    //���������� ������� ��������� ������� 612 0�1 - �������� ��������� ������� 0�2 - �������� ���������� ���������� 0�4 - ����������� �������

                       if (sys_life_k612&0x4) Mem_K615.Ohl=1; else Mem_K615.Ohl=0; // ��������� ���������� ������

                  		Flag_control_K612=1;
                  		Flag_K612=0;
                  		tick_process=0;
                  		sch_obmen=0;
                  	    tick_process_K612=0;

                  		} //��������� ������������� �� ����� ���������� ������� �������������
                  		
                  		
                  //--------------------------------------------------K613----------------------------------------------------------------				
                  		
                  		if (strcmp(Word,"���_������_K613")==0)
                  		{ 
                      
                      if (Master_flag==1) 	Transf ("������ ��������_���������� K613\r"  );

                  		Flag_init_K613=1;
                  		Flag_K613=0;
                  		tick_process=0;
                  		sch_obmen=0;
                        tick_process_K613=0;

                  		} //��������� ������������� �� ����� ���������� ������� �������������
                  		
                  				
                  		if (strcmp(Word,"���������_K613")==0)
                  		{ 

                      	  sys_char=atoi(DATA_Word);

                      // Transf ("������ ��������� K613\r"  );

                       
                        if (Master_flag==1) 
                                {
                                          Transf ("������ ��������� K613:"  );       
                                          itoa_t(sys_char,s3);
                                          Transf(s3);
                                          Transf("\r");
                                }  

                          sys_life_k613=sys_char;   //���������� ������� ��������� ������� 613 

                  		Flag_control_K613=1;
                  		Flag_K613=0;
                  		tick_process=0;
                  		sch_obmen=0;
                  	    tick_process_K613=0;
                  		} //��������� ������������� �� ����� ���������� ������� �������������


                  //------------------------------------------------------------------------------------------------------------------		
		} 
	
	}
	
	if ((packet_ok==1)&&(crc_ok==0x1))     //��������� ������ ��������� ������� �������� �����

	{
	  if (Master_flag==0)
      {
				/*
				  TX_485();
        	Transf("\r");
        	ZTransf (InOut,index1);
        	Transf("\r");
				*/
      }
	}
	
	
	if ( packet_ok==1) 
		
		{			
			for (i=0;i<BUF_RX;i++)        Word[i]     =0x0;
			for (i=0;i<BUF_RX;i++)   DATA_Word[i]     =0x0;
			for (i=0;i<BUF_RX;i++)  DATA_Word2[i]     =0x0;  
			for (i=0;i<BUF_RX;i++)       InOut[i]     =0x0;
			for (i=0;i<BUF_RX;i++)       sr_io[i]     =0x0;
			for (i=0;i<BUF_RX;i++)          sr[i]     =0x0;
			
			time_uart=0;  //��������� �������� ���� ����
			packet_flag=0; 
			index1=0; 
			crc=0; 
			crc_ok=0; 
			i=0;
			packet_ok=0; 
			index_word=0; 
			index_data_word=0;
			data_flag=0;
			index_data_word2=0;
			data_flag2=0;
		};

	}
         
 } 


	void led2(char a)
	{
	 if (a==0) PA1_0;
   if (a==1) PA1_1;
	}
	
void fillBuf (unsigned char* ptrBuf, unsigned short lengthBuf, unsigned char symb )
   { unsigned short i;

     for ( i=0; i < lengthBuf; i++ )
      {
        *ptrBuf=symb; ptrBuf++;
      }
   }

  void clearBuf (unsigned char* ptrBuf, unsigned short lengthBuf )
   {  fillBuf( ptrBuf, lengthBuf, ' ' );
   }
      

   int getStr ( unsigned char* s, unsigned char*ps_size )
    { unsigned char* temp;
      temp=pRcv_Buf;
      *ps_size=0;
      if ( pcur_Rcv <  temp ){
      	
   
        //*ps_size=temp-pcur_Rcv;
        *ps_size=1;
        while(pcur_Rcv <  temp)
          {*s=*pcur_Rcv; s++; pcur_Rcv++;}
         
        }
       else if  ( temp < pcur_Rcv )
              {  
             	
              	if  ( CyclEnd_flag )
                    { while ( pcur_Rcv != temp )
                        {*s = *pcur_Rcv; s++;
                         *ps_size++;
                         if ( pcur_Rcv == (RcvBuf+ buf_size-1) )
                           { pcur_Rcv= RcvBuf;
                             CyclEnd_flag=0;  
                           }
                          else
                            pcur_Rcv++;
                        }
                    }
                  else
                   {; // �� ������ �� ����� ����
                    return 1;
                   }
              }
  
       else  
       	                    return 1;       
   
      return 0;
    }


void UART_control (void)

{

	if ((BufIsRead_flag==1)) // �������� ������ �� ����� �� ����� � ��� ��������� &&(flag_pachka_sinhron==0)
           
     {  
      getStr(sr,&lsr);
      BufIsRead_flag=0;
      IO(sr);
	  RX_485();
	 }	
}


void BUS_wr (unsigned short a)
{

	if (a&(1<<0)) PORTF->SETTX = (1<<8); //������������� ������� PF[8], ��� 0 
    else		  PORTF->CLRTX = (1<<8); //������������� ���� PF[8], ��� 0 

    if (a&(1<<1)) PORTF->SETTX = (1<<9); //������������� ������� PF[8], ��� 0 
    else		  PORTF->CLRTX = (1<<9); //������������� ���� PF[8], ��� 0 

    if (a&(1<<2)) PORTF->SETTX = (1<<10); //������������� ������� PF[8], ��� 0 
    else		  PORTF->CLRTX = (1<<10); //������������� ���� PF[8], ��� 0 

    if (a&(1<<3)) PORTF->SETTX = (1<<11); //������������� ������� PF[8], ��� 0 
    else		  PORTF->CLRTX = (1<<11); //������������� ���� PF[8], ��� 0 

    if (a&(1<<4)) PORTF->SETTX = (1<<12); //������������� ������� PF[8], ��� 0 
    else		  PORTF->CLRTX = (1<<12); //������������� ���� PF[8], ��� 0

    if (a&(1<<5)) PORTF->SETTX = (1<<13); //������������� ������� PF[8], ��� 0 
    else		  PORTF->CLRTX = (1<<13); //������������� ���� PF[8], ��� 0  

    if (a&(1<<6)) PORTF->SETTX = (1<<14); //������������� ������� PF[8], ��� 0 
    else		  PORTF->CLRTX = (1<<14); //������������� ���� PF[8], ��� 0 

    if (a&(1<<7)) PORTF->SETTX = (1<<15); //������������� ������� PF[8], ��� 0 
    else		  PORTF->CLRTX = (1<<15); //������������� ���� PF[8], ��� 0 

    if (a&(1<<0x9)) PORTD->SETTX = (1<<9); //������������� ������� PF[8], ��� 0 
    else		    PORTD->CLRTX = (1<<9); //������������� ���� PF[8], ��� 0 

    if (a&(1<<0xa)) PORTC->SETTX = (1<<2); //������������� ������� PF[8], ��� 0 
    else		    PORTC->CLRTX = (1<<2); //������������� ���� PF[8], ��� 0 

    if (a&(1<<0xb)) PORTD->SETTX = (1<<10); //������������� ������� PF[8], ��� 0 
    else		    PORTD->CLRTX = (1<<10); //������������� ���� PF[8], ��� 0 

    if (a&(1<<0xc)) PORTD->SETTX = (1<<11); //������������� ������� PF[8], ��� 0 
    else		    PORTD->CLRTX = (1<<11); //������������� ���� PF[8], ��� 0 

    if (a&(1<<0xd)) PORTD->SETTX = (1<<12); //������������� ������� PF[8], ��� 0 
    else		    PORTD->CLRTX = (1<<12); //������������� ���� PF[8], ��� 0 

    if (a&(1<<0xe)) PORTD->SETTX = (1<<13); //������������� ������� PF[8], ��� 0 
    else		    PORTD->CLRTX = (1<<13); //������������� ���� PF[8], ��� 0 

    if (a&(1<<0xf)) PORTD->SETTX = (1<<14); //������������� ������� PF[8], ��� 0 
    else		    PORTD->CLRTX = (1<<14); //������������� ���� PF[8], ��� 0 

}


void UART1_Handler()
{
	unsigned stat;
    unsigned t;
	uint32_t value;
	unsigned char letter[1];

	UART1->ICR=0x10;		//reset Rx interrupt	
	letter[0]=(UART1->DR);
	//BufIsRead_flag=1; //���� ���������� �����
	
	                    //  Receive data ready.
            { 
                                                      //  �������� �����: ��������. �����. ������
             t= letter[0];
             if ( /*t != 0*/1 ) 
               {
                 *pRcv_Buf=t;  
                 if ( pRcv_Buf == pcur_Rcv )         //
                   { if ( CyclEnd_flag )
                       { 
                        RcvBufOverflow_flag=1;
                        CyclEnd_flag = 0;
                       }
                      //else
                     BufIsRead_flag=1;      // ��� ����. ������ ��� �������
                   }
                 if ( pRcv_Buf == (RcvBuf+buf_size-1))
                   {
                    pRcv_Buf = RcvBuf;
                    CyclEnd_flag = 0x1;
                   }
                  else
                   { 
                    pRcv_Buf++;
                   }
                  
               }
            }
	
}


void UART2_Handler()
{
	unsigned stat;
    unsigned t;
	uint32_t value;
	unsigned char letter[1];

	UART2->ICR=0x10;		//reset Rx interrupt	
	letter[0]=(UART2->DR);
	//BufIsRead_flag=1; //���� ���������� �����
	
	                    //  Receive data ready.
            { 
                                                      //  �������� �����: ��������. �����. ������
             t= letter[0];
             if ( /*t != 0*/1 ) 
               {
                 *pRcv_Buf=t;  
                 if ( pRcv_Buf == pcur_Rcv )         //
                   { if ( CyclEnd_flag )
                       { 
                        RcvBufOverflow_flag=1;
                        CyclEnd_flag = 0;
                       }
                      //else
                     BufIsRead_flag=1;      // ��� ����. ������ ��� �������
                   }
                 if ( pRcv_Buf == (RcvBuf+buf_size-1))
                   {
                    pRcv_Buf = RcvBuf;
                    CyclEnd_flag = 0x1;
                   }
                  else
                   { 
                    pRcv_Buf++;
                   }
                  
               }
            }
	
}


void ETHERNET_Handler()
{
	uint16_t Status;
	uint16_t Size[2];
	void* Buf;
	Status=ETHERNET->IFR;
	ETHERNET->IFR=Status;

	if((Status&0x01)==0x01)
	{
		// ReadDataFromR_Buffer (&Size, 4); 
		// Buf = MallocEthInPacket (Size[0]);
		// ReadDataFromR_Buffer (Buf, Size[0]);
		// ReadEthInPacket (Buf, Size[0]);
		Rec.Counter = ReadPacket (&Rec);
		ReadEthInPacket ( Rec.Data, Rec.Counter );
	}
}

// *** �������, ���������� ���������� ���������� ��� ��������� ������� ���������.
// ����� ������������ ������ OnUDPReceive, ��� ����, ����� �������� ������, ���������� ��� �� ��������� UDP.
// ��� ������� ����� ����� ������������� ������ ������ �������, �� ��� �� ����������, �.�. ����� �������� �������
// � �������� � ���� ������� ��������� ������� ���������� ����������.
void OnEthReceive ( void* pIPData, uint16_t Size) {}
void OnIPReceive ( void* pIPData, uint16_t Size) {}
void OnARPReceive (void* pARPData, uint16_t Size) {}
	
void OnUDPReceive (IP_Address SrcIP, uint16_t SrcPort, uint16_t DstPort, void* pUDPData, uint16_t Size) { 
  uint16_t SendMe, i;
	uint32_t counter;
	//PORTD->CLRTX = 0xFFFF;
  //PORTD->SETTX = *((uint8_t*)pUDPData) << 7;
	SendViaUDP (SrcIP, 0x0608 , 0x0608, pUDPData, Size);
	zputs(pUDPData,Size);
	sendT("\r\n");
}

void init_SEL (void)

{
	PA3_1;//SEL1[0]
	PA4_0;//SEL1[1]
	PA5_1;//SEL1[2]
	PA6_1;//SEL1[3]
	PA7_1;//SEL1[4]
	PA8_1;//SEL1[5]
	
	PF2_1;//SEL2[0]
	PF3_1;//SEL2[1]
	PF4_1;//SEL2[2]
	PF5_1;//SEL2[3]
	PF6_1;//SEL2[4]
	PF7_1;//SEL2[5]
}

void INIT_sintez1(void)
{

	if (Adress==0x33)
	{
		IO("~3 ����������_F0=435000000;");
	
		IO("~3 ����������_��������_���=10450000;");
	
		IO("~3 �������_����������_F���=1000;");
	
		IO("~3 ������������_��������_T���=128;");
	
		IO("~3 ����������_���;");

		IO("~3 ����������_���������:3;");
	
		//IO("~3 watch_reg_dds;");
	}
	
	if (Adress==0x34)
	{
		IO("~4 ����������_F0=389000000;");
	
		IO("~4 ����������_��������_���=10450000;");
	
		IO("~4 �������_����������_F���=1000;");
	
		IO("~4 ������������_��������_T���=128;");
	
		IO("~4 ����������_���;");

		IO("~4 ����������_���������:8;");
	
		//IO("~4 watch_reg_dds;");
	}
	RX_485();
}

void INIT_sintez2(void)
{

	if (Adress==0x33)
	{
		IO("~3 ����������_F0=435000000;");
	
		IO("~3 ����������_��������_���=10250000;");
	
		IO("~3 �������_����������_F���=1;");
	
		IO("~3 ������������_��������_T���=600000;");
	
		IO("~3 ����������_���;");

		IO("~3 ����������_���������:4;");
	
		//IO("~3 watch_reg_dds;");
	}
	
	if (Adress==0x34)
	{
		IO("~4 ����������_F0=389000000;");
	
		IO("~4 ����������_��������_���=10250000;");
	
		IO("~4 �������_����������_F���=1;");
	
		IO("~4 ������������_��������_T���=600000;");
	
		IO("~4 ����������_���;");

		IO("~4 ����������_���������:8;");
	
		//IO("~4 watch_reg_dds;");
	}
	RX_485();
}

void OnICMPReceive (void* pICMPData, uint16_t Size) {}
void SendEthPacket (void* pHeader, uint16_t Size) {}

 unsigned int FS = 1000000000; //������� DDS Hz
 
 unsigned int INIT_SINTEZ_delay = 150000u;
 #define led_delay 70000u
 #define DELAY_SHOW 180000000u  // 20000000u - 20 ������

int main ()
{
	
	unsigned char letter[1];
	unsigned int i=0;
	unsigned int j=0;
	unsigned int sch_test_bus=0;
	char INIT_FLAG1=0;
	char INIT_FLAG2=1;
	unsigned int TIME_show=0;
	
  //  __disable_irq();
  	 RST_CLKConfig ();
     // CAN1Config ();
       UART1Config ();
       UART2Config ();
	//ETHERNETConfig ();
	   PORTAConfig ();
	   PORTBConfig ();
	   PORTDConfig ();
	   PORTCConfig ();
	   PORTEConfig ();
	   PORTFConfig ();
	        ADCInit();
	
	//NVIC_EnableIRQ (ETHERNET_IRQn);
	NVIC_EnableIRQ (   UART1_IRQn);
	NVIC_EnableIRQ (   UART2_IRQn);

  //��������������� ���� ��� DDS ����������� ��� � ���
     PORTC->RXTX|=(1<<9)|(1<<11); //���������� �������
  //--------------------------------------------------

	pcur_Tr= TrBuf;
    pTr_Buf= TrBuf;

    pcur_Rcv= RcvBuf;
    pRcv_Buf= RcvBuf;
	
	letter[0]='A';

	for (i=0;i<64;i++) sr[i]=0x0;

	Menu1();

    POWER_ON;

	init_SEL ();
	
	if (Adress==0x34) 
	{
		INIT_SINTEZ_delay = 12000000u;
		Fget_define = 450000000; 
		FS =  900000000;
	} //������� ���������� � ������� ����������
	
	if (Adress==0x33) 
	{
		INIT_SINTEZ_delay = 10000000u;
		Fget_define = 500000000;
		FS = 1000000000;
	} //������� ���������� � ������� �����������
	

			
	init_DDS (&DDS_data,&pins_dds); //������������� DDS
				
	
	while(1)
	{
		UART_control ();

		if (i<INIT_SINTEZ_delay) i=i+1; else i=0;
		if (j<led_delay)         j=j+1; else j=0;
		

		if ((i==INIT_SINTEZ_delay)&&(INIT_FLAG2==0))   {INIT_sintez2();INIT_FLAG2=1;INIT_FLAG1=0;i=0;	INIT_SINTEZ_delay = DELAY_SHOW;}
		if ((i==INIT_SINTEZ_delay)&&(INIT_FLAG1==0))   {INIT_sintez1();INIT_FLAG1=1;INIT_FLAG2=0;i=0;	INIT_SINTEZ_delay = DELAY_SHOW;}
		

		/*	
		if ((ETHERNET->PHY_STATUS&0x02) == 0x00) ;//PORTB->SETTX = 1<<15;
		else ;//PORTB->CLRTX = 1<<15;
		if ((ETHERNET->PHY_STATUS&0x08) == 0x00) ;//PORTB->SETTX = 1<<14;
		else ;//PORTB->CLRTX = 1<<14;
		*/
	}
}



#endif // __Application_c__
