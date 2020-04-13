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
//#include "TERMINAL.h"

#include "MAC.h"
#include "ARP.h"
#include "IP.h"
#include "UDP.h"
#include "ICMP.h"
#include <stdlib.h>





#define PA7_0 PORTA->CLRTX = (1<<7) //������������� ����    PA[7] , ���������� ������� DD6 
#define PA7_1 PORTA->SETTX = (1<<7) //������������� ������� PA[7],  ���������� ������� DD6 

#define PE15_0 PORTE->CLRTX = (1<<15) //������������� ����    PE[15] , ���������� ����������� ������ ����� ����
#define PE15_1 PORTE->SETTX = (1<<15) //������������� ������� PE[15],  ���������� ����������� ������ ����� ����

#define Zahvat_OFF  PE15_1
#define Zahvat_ON PE15_0 

#define PC12_0 PORTC->CLRTX = (1<<12) //������������� ����    PC[12] , ���������� ����������� ������� ����� ����
#define PC12_1 PORTC->SETTX = (1<<12) //������������� ������� PC[12],  ���������� ����������� ������� ����� ����

#define POWER_OFF  PC12_1
#define POWER_ON PC12_0 


#define RX_485 PORTA->CLRTX = (1<<10) //������������� ����    PA[10] , ���������� ������� DD2 ������ DE_485 - ��������� �� ��������
#define TX_485 PORTA->SETTX = (1<<10) //������������� ������� PA[10], ���������� ������� DD2 ������ DE_485 - ��������� �� ���� 

#define htons(A) ((((uint16_t)(A) & 0xff00) >> 8) | (((uint16_t)(A) & 0x00ff) << 8))

IP_Address MyIP   = { 192, 168, 1, 163 };
IP_Address HostIP = { 192, 168, 1, 1 };

UDP_Port p2054 = { 0x08, 0x06 };
UDP_Port pCB99 = { 0xCB, 0x99 };

#pragma pack ( push, 2)
const MAC_Address MyMAC = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB };
#pragma pack (pop)

_Rec_Frame Rec;

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



  //  UART



#define Bufer_size  64     //16384


volatile  unsigned int  text_lengh;


unsigned int j_pack=0;
unsigned char packet[4][256];
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

unsigned char s1[Bufer_size];
unsigned l1= 5;
unsigned char s2[Bufer_size];
unsigned l2=6;
unsigned char s3[Bufer_size];
unsigned l3=7;
unsigned char sr[Bufer_size];
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
   	   unsigned char  k;

  unsigned  	char lsym;
  unsigned  	char  sym;
  unsigned  	char flag;
  unsigned  	char packet_flag;
  unsigned      char	NB;

  unsigned    char Adress=0x31;      // ����� �������� Adress=0x37;
  unsigned    char Master_flag=0x0;  // ���� ������������ ������ �������-1,0-��������������� �������������  --�������� ������ ����� ���� ��� ������

  unsigned    char packet_sum;
  volatile   unsigned  	char crc,comanda=1;
  unsigned    char     InOut[Bufer_size];
 
  unsigned    char      Word [Bufer_size];    //������ ���������� �����
  unsigned    char DATA_Word [Bufer_size];    //������ ����� - ������
  unsigned    char DATA_Word2[Bufer_size];    //������ ����� - ������
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
  unsigned char* p;
   short* p2; // !!!
  unsigned char   DSP_in1[256];
  //short  DSP_in2[256];


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
			   unsigned char flag_status_ZAHVAT=0;
			   unsigned char FLAG_INIT_FOCH=0;	
			   unsigned char FLAG_INIT_MENU=0;	
			   unsigned char FLAG_INIT_n_FOCH=0; //���� ����� ������������� ���	
			   unsigned char FLAG_LED_n=0;      //���� ������ ��������� �����������	
			    unsigned char Control_FOCH_FLAG=0;
			   
			   
  //------------------------------------------------------------------

  
   unsigned char  strng[64];
  
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

 //-------------------------------------------------------------------------
    typedef struct    
   {
     unsigned char    Level_F; //������� �������� ��������,1/0
     unsigned char    Level_U; //������� ���������� ������� 1/0
     unsigned char       Temp; //����������� ������� 1/0
     unsigned char    TimeOUT; //������� ������� 0/1  ������� ������ �������� "0" ������� ����������� ��������, ��� 4 ��� �� ��������� �������� � ���������� ���������.
     unsigned char  Condition; //��������� �������: 1/0
   } Life_Blok;

 Life_Blok K611;
 Life_Blok K612;   
 Life_Blok K613;
 Life_Blok K614;
 Life_Blok K615;
 Life_Blok B610;
 
  unsigned int  M [256];

  
     	
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
// ��������� ������� � �������� 
   
char init_K611(char );
char control_K611(char);
char  SYS_control (Life_Blok *,uint16 );

void reverse(char *);
void itoa(int , char*);
char init_K615(char );
char control_K615(char );
char init_K612(char );
char control_K612(char );
char init_K613(char );  
char control_K613(char );
void IO ( char* );

void fillBuf (char* , unsigned short , unsigned char  );
void zputs(char *, unsigned );
 int getStr (char* , unsigned char* );
 void clearBuf ( char* , unsigned short  );

unsigned int leng ( char *) ;
void sendT ( char* );
void Transf( char* );
void Transf_485( char* );
void ZTransf( char* ,unsigned char ) ;


void SendViaUDP (IP_Address , uint16_t , uint16_t , void* , uint16_t );
 
 IP_Address SrcIP_m;
 uint16_t SrcPort_m;
 uint16_t DstPort_m; 




//***************************************************************************


//-----------------------------------------



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

 // itoa:  ������������ n � ������� � s 
 void itoa(int n, char s[])
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


//----------------------------------------

 //----------------------------------------


  char control_K615(char a)

{
  if (Mem_K615.VYKL==2) Mem_K615.VYKL=0;

  if (RESET_SINTEZ_flag==1) Mem_K615.VYKL=2;

//TX_485;

  Transf("\r\n");
  Transf("~5 INPORT");
  //ZTransf(s,6);
  Transf(";");
  Transf("\r\n");

 //�������� �� ����� 485 ����
 return 1;
}
//-------------------------------------
char  SYS_control (Life_Blok *a,uint16 t)
{
  a->Level_F   = (t&0x01);
  a->Level_U   = (t&0x02)>>1;
  a->Temp      = (t&0x04)>>2;
  a->TimeOUT   =       0;
  a->Condition = (t&0x08)>>3;

  if (a->Condition==0) AVARIYA_flag++;
  
  return 1;
  
}

char SYS_monitor(void)
{
	if (K611.Level_F  ==1)  Transf("5�-�611:������� �������� ��������                - �����\r"); else  Transf("5�-�611:������� �������� �������� 			     - ������\r");
    if (K611.Level_U  ==1)  Transf("5�-�611:������� ���������� ���������� ���������� - �����\r"); else  Transf("5�-�611:������� ���������� ���������� ���������� - ������\r");
	if (K611.Condition==1)  Transf("5�-�611:����� ���������							 - �����\r"); else  Transf("5�-�611:����� ���������							 - ������\r");
	
	if (K612.Level_F  ==1)  Transf("5�-�612:������� �������� ��������                - �����\r"); else  Transf("5�-�612:������� �������� �������� 			     - ������\r");
    if (K612.Level_U  ==1)  Transf("5�-�612:������� ���������� ���������� ���������� - �����\r"); else  Transf("5�-�612:������� ���������� ���������� ���������� - ������\r");
	if (K612.Condition==1)  Transf("5�-�612:����� ���������							 - �����\r"); else  Transf("5�-�612:����� ���������							 - ������\r");
	
	if (K613.Level_F  ==1)  Transf("5�-�612-1:������� �������� ��������                - �����\r"); else  Transf("5�-�612-1:������� �������� �������� 			     - ������\r");
    if (K613.Level_U  ==1)  Transf("5�-�612-1:������� ���������� ���������� ���������� - �����\r"); else  Transf("5�-�612-1:������� ���������� ���������� ���������� - ������\r");
	if (K613.Condition==1)  Transf("5�-�612-1:����� ���������	  					   - �����\r"); else  Transf("5�-�612-1:����� ���������							 - ������\r");
	
	if (K614.Level_F  ==1)  Transf("5�-�614:������� �������� ��������                - �����\r"); else  Transf("5�-�614:������� �������� �������� 			     - ������\r");
    if (K614.Level_U  ==1)  Transf("5�-�614:������� ���������� ���������� ���������� - �����\r"); else  Transf("5�-�614:������� ���������� ���������� ���������� - ������\r");
	if (K614.Condition==1)  Transf("5�-�614:����� ���������							 - �����\r"); else  Transf("5�-�614:����� ���������							 - ������\r");

	return 1;
}

char init_K611(char a)

{ 

  TX_485;
  Transf("\r");
  Transf("~6 init_K611;");
  Transf("\r");
	RX_485; 
  return 1; 
}

char control_K611(char a)

{ 
  TX_485;
  Transf("\r");
  Transf("~6 control_K611;");
  Transf("\r");
  RX_485;   
 
return 1; 
}

//---------------------------------------
 char init_K612(char a)

{ 
  TX_485;
  Transf("\r");  
  Transf("~3 ���_������;");
  Transf("\r");
  RX_485; 
  return 1;

} 

  char control_K612(char a)

{
  TX_485;
  Transf("\r"); 
  Transf("~3 ���������;");
  Transf("\r");
  RX_485; 
  return 1;
}

 char init_K613(char a)

{  
  TX_485;
  Transf("\r");  
  Transf("~4 ���_������;");
  Transf("\r");
  RX_485; 
  return 1;
} 

  char control_K613(char a)

{ 
  TX_485;
  Transf("\r");  
  Transf("~4 ���������;");
  Transf("\r");
  RX_485; 
  return 1;
} 


 void Transf( char* s)  // ��������� �������� ������ �������� � ���� 
   {
       unsigned  short l=0;
       unsigned  short i=0;
 
       sendT(s);	  //uart1
       sendT2(s);     //uart2

       SendViaUDP (SrcIP_m, SrcPort_m , DstPort_m, s, leng(s)); //�������� ��������� ������ ������� �� UDP
	
  }
  
  

 void Transf_485( char* s)  // ��������� �������� ������ �������� � ���� �� 485 ����
   {
       unsigned  short l=0;
       unsigned  short i=0;
	   TX_485;
       sendT2(s);     //uart2
	   RX_485; 
  }
   
   

 void ZTransf( char* s,unsigned char a)  // ��������� �������� ������ �������� � ���� 
   {
     unsigned  short l;
     unsigned  short i;

        zputs(s,a);
	    zputs2(s,a);
      
  }
   


u16 SPITxInProgress(void){
  //return (*pSPI_STAT) & SPIF;
}

u16 SPITxFull(void){
 // return (*pSPI_STAT) & TXS;
}

u16 SPIRxReady(void){
  //return (*pSPI_STAT) & RXS;
}

void SPIPut(u16 data){
  //*pSPI_TDBR=data;
}

u16 SPIGet(void){


 // return *pSPI_RDBR;
}

void SPIInit (u16 sclkDiv, u16 send16Bit){
 // *pSPI_BAUD=sclkDiv;
 // *pSPI_CTL=TDBR_CORE | SZ | (send16Bit ? SIZE : 0) | MSTR | SPE;
}

  

   void spi_cs(u8 cs)   //��������� ������������  ���������� ��� ���� SPI
{
   switch (cs) {

case 0:    break;//������� ���� �� PF3  
case 1:    break;//������� ���� �� PF3  

default: break;
   }

}
  
	
void spi_transf(u8* command,u8 l)    
 
 {     
 // spi ��������
 unsigned char crc,a;
  unsigned i; 
  
   	  for (i=0;i<l;i++)
      
 	 {
	 
	 //a=SPI(command[i]);
	 
	 }
 	 
   }
   
    
  u8 SPI(u8 spi)
{
//*pSPI_TDBR=spi;

//while(!(*pSPI_STAT&0x0001));
//while(!(*pSPI_STAT&0x0020));
//return (u8)*pSPI_RDBR;
}
  

    
void IO ( char* sr)      // ������� ��������� ��������� ������
 
 {
       
 unsigned char s[6];   
 unsigned char i=0;
 unsigned char index=0;

  unsigned char  z_k611=0;
  char sys_char=0;
  int l=0;
  
    i=leng(sr);
     
   sym1=sr[0];
   
   p[0]=sym1;   
   
//ZTransf (sr,i);   

if ((sym1==0x7e)||(time_uart>100)) 
  {
    time_uart=0;  //��������� �������� ���� ����
    packet_flag=1; 
    index1=0; 
    crc=0; 
    crc_ok=0; 
    packet_ok=0; 
    index_word=0; 
    index_data_word =0;
    index_data_word2=0;
       data_flag =0;
       data_flag2=0;
    DATA_Word [0]=' ';
    DATA_Word2[0]=' ';
    
  } // ���������� ������ ������

if (packet_flag==1)

{
  while (i>0)   //���������� ��������� ������ � ������ ���������
  
  {

     InOut[index1]=sr[index];
     sr[index]=0x0;
    
    if  (InOut[index1]==' ')  ink1=ink1+1;
    
    if  (InOut[index1]==';')  packet_ok=1;
    
    if ((InOut[index1]=='=')||(InOut[index1]==':')) data_flag=1;

      if  (InOut[index1]=='.') data_flag2=1;
    

            if ((index1>2)&&(InOut[2]==' '))  
            {
            
                             if  ( (InOut[index1]!=' ')&&
                                   (InOut[index1]!=';')&&
                                   (data_flag!=1))         Word[index_word]=InOut[index1]; // ��������� ��������� �����
                    
                   
                             if  ((data_flag==1)&&
                                 ((InOut[index1]!=' ')||
                                  (InOut[index1]!=';')||
                                  (InOut[index1]!='=')||
                                  (InOut[index1]!=':'))&&(data_flag2==0))     DATA_Word[index_data_word]=InOut[index1]; // ���������  ����� ������1
                      
                   
                             if  ((data_flag==1)&&
                                 ((InOut[index1]!=' ')||
                                  (InOut[index1]!=';')||
                                  (InOut[index1]!='=')||
                                  (InOut[index1]!=':'))&&(data_flag2==1))     DATA_Word2[index_data_word2]=InOut[index1]; // ���������  ����� ������2
                          
                     
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

              {
               Transf ("������ help\r"    );
               
               Menu1 ();
       
              }
            
                    
            if (strcmp(Word,  "�������������1"    )==0)      
                              
                              {  Transf ("������ �������������1\r"  ); 
                                            
                               }     
							   
			 if (strcmp(Word,  "COMMAND_OK"    )==0)      
                              
                              {  Transf ("OK! ������ �������������!\r"  ); 
                                            
                               }   

           if (strcmp(Word,  "sync_1"    )==0)      
                              
                              {  Transf ("������ sync_1\r"  ); 
                                
                               }    
           if (strcmp(Word,  "UDP_control"      )==0)      
                                        { 
                                          crc_comp=atoi(DATA_Word);
                                          Transf ("������ UDP_control\r" );
                                          
                                        }
           if (strcmp(Word,  "Pachka_control"      )==0)      
                                        { 
                                          crc_comp=atoi(DATA_Word);
                                          Transf ("������ Pachka_control\r" );
                                         }


           if (strcmp(Word,"init_K611")==0) //��������� ������������� �� ����� ���������� ������� �������������
                      { 
                        Transf ("\r ������ init_K611  \r"  );
                        Flag_init_K611=1;
                        Flag_K611=0;
                        tick_process=0;
                        sch_obmen=0;
                        tick_process_K611=0;
                      } 
                      
           if (strcmp(Word,"���������_�611")==0) //��������� ������������� �� ����� ���������� ������� �������� ���������
                      {
                                  crc_comp =atoi(DATA_Word);
                                  crc_input=atoi(DATA_Word2);  

                                 if (crc_comp==crc_input)
                                     {
										flag_status_ZAHVAT=crc_comp;
										FLAG_LED_n=250;//��������� ������� ���������� �������!!!
										
                                        SYS_control(&K611,crc_comp);
                                        if (Control_FOCH_FLAG==0) //�������� �������: ������ ��� ��������������, �������������� �� ���������� �� ������!!!
											{
												Transf ("������ ��������� �611:");
												sprintf(strng,"%u",crc_comp);
												Transf(strng);
												Transf("\r");
											}	
											else Control_FOCH_FLAG=0;
                                       																	
                                        Flag_control_K611=1;
                                        Flag_K611=0;
                                        tick_process=0;
                                        sch_obmen=0;
                                        tick_process_K611=0;
                                     }
                       }

          if (strcmp(Word,"���������")==0)//������� ������ ���������� � ��������� �����
                      { 
                        Transf ("\r ������ ���������  \r"  );
                         SYS_monitor();
                        Flag_init_K614=1;
                        Flag_K614=0;
                        tick_process=0;
                        sch_obmen=0;
                       
                      } 
                       
                      
         if (strcmp(Word,"���_612")==0)//��������� ������������� �� ����� ���������� ������� �������������
                      { 
                        Transf ("\r ������ ���_612  \r"  );
                        Flag_init_K612=1;
                        Flag_K612=0;
                        tick_process=0;
                        sch_obmen=0;
                        tick_process_K612=0;
                      } 
                      
                        
                    if (strcmp(Word,"���������_�612")==0)//��������� ������������� �� ����� ���������� ������� ���������
                      {
                                      crc_comp =atoi(DATA_Word);
                                      crc_input=atoi(DATA_Word2);  

                                  if (crc_comp==crc_input)
                                             {
                                                  SYS_control(&K612,crc_comp);
                                                  Transf ("\r ������ ��������� �612:");
                                                  sprintf(strng,"%u",crc_comp);
                                                  Transf(strng);
                                                  Transf("\r");
                                                  Flag_control_K612=1;
                                                  Flag_K612=0;
                                                  tick_process=0;
                                                  sch_obmen=0;
                                                  tick_process_K612=0;
                                              }
                        } 

                      
                    if (strcmp(Word,"���_613")==0)
                      { 
                        Transf ("\r ������ ���_613  \r"  );
                        Flag_init_K613=1;
                        Flag_K613=0;
                        tick_process=0;
                        sch_obmen=0;
                        tick_process_K613=0;
                      } //��������� ������������� �� ����� ���������� ������� �������������
                      
                        
                    if (strcmp(Word,"���������_�613")==0)//��������� ������������� �� ����� ���������� ������� ���������
                      {
                              crc_comp =atoi(DATA_Word);
                              crc_input=atoi(DATA_Word2);  

                          if (crc_comp==crc_input)
                                     {
                                            SYS_control(&K613,crc_comp);
                                            Transf ("\r ������ ��������� �613:"  );
                                            sprintf(strng,"%u",crc_comp);
                                            Transf(strng);
                                            Transf("\r");
                                              Flag_control_K613=1;
                                              Flag_K613=0;
                                              tick_process=0;
                                              sch_obmen=0;
                                              tick_process_K613=0;
                                     }
              } 
      } 
  
  }
  
  if ((packet_ok==1)&&(crc_ok==0x1))     //��������� ������ ��������� ������� �������� �����

  {
    if ((Master_flag==0)&&(Adress=='1'))

      { 
          TX_485; 
          Transf("\r");
          ZTransf (InOut,index1);
		  Transf("\r");
          RX_485;        
      }

    
  }
  
  
  if ( packet_ok==1) 
    
    {
  
      for (i=0;i<Bufer_size;i++)        Word[i]     =0x0;
      for (i=0;i<Bufer_size;i++)   DATA_Word[i]     =0x0;
      for (i=0;i<Bufer_size;i++)  DATA_Word2[i]     =0x0;  
      for (i=0;i<Bufer_size;i++)   	   InOut[i]     =0x0;
    
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

	void led_high(void)
	{
	 PORTE->SETTX = 0x2;
	}
	
	
		void led_low(void)
	{
		PORTE->CLRTX = 0x2;
	}


void fillBuf ( char* ptrBuf, unsigned short lengthBuf, unsigned char symb )
   { unsigned short i;

     for ( i=0; i < lengthBuf; i++ )
      {
        *ptrBuf=symb; ptrBuf++;
      }
   }

  void clearBuf ( char* ptrBuf, unsigned short lengthBuf )
   {  fillBuf( ptrBuf, lengthBuf, ' ' );
   }
      

   int getStr (  char* s, unsigned char*ps_size )
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
			  // ZTransf(sr,lsr);
			  // Transf("\r");
                  IO(sr);
                  RX_485;
            
            }	
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
		Rec.Counter = ReadPacket (&Rec);
		ReadEthInPacket ( Rec.Data, Rec.Counter );
	}
}

// *** �������, ���������� ���������� ���������� ��� ��������� ������� ���������.
// ����� ������������ ������ OnUDPReceive, ��� ����, ����� �������� ������, ���������� ��� �� ��������� UDP.
// ��� ������� ����� ����� ������������� ������ ������ �������, �� ��� �� ����������, �.�. ����� �������� �������
// � �������� � ���� ������� ��������� ������� ���������� ����������.
void OnEthReceive (void* pIPData , uint16_t Size) {}
void OnIPReceive  (void* pIPData , uint16_t Size) {}
void OnARPReceive (void* pARPData, uint16_t Size) {}


	
void OnUDPReceive (IP_Address SrcIP, uint16_t SrcPort, uint16_t DstPort, void* pUDPData, uint16_t Size) { 
    uint16_t SendMe, i;
	uint32_t counter;
	
	SrcIP_m   = SrcIP;
	SrcPort_m = SrcPort;
	DstPort_m = DstPort;
	
	IO(pUDPData); //��������� �������� ������

	SendViaUDP (SrcIP, SrcPort , DstPort, pUDPData, Size); //�������� ��������� ������ ������� �� UDP
	
	//zputs(pUDPData,Size);
	//sendT("\r\n");
}



void Led_Zahvat (unsigned char a)
{
	static unsigned char l;
	
	  if (a==0x00) l=0;
      if (a==0x01) l=~l;
      if (a==0x02) l=~l;
      if (a==0x03) l=0xff;
	  if (l>0u) Zahvat_ON; else Zahvat_OFF;

}


unsigned char POWER_LED (unsigned char i)

{
  if (i==1) POWER_OFF;  
 
  if (i==2) POWER_ON;

  if (i==3) i=0; else if (i<10) i++;
  
  if (i>200)  POWER_ON;
   
  return i;
 }

void OnICMPReceive (void* pICMPData, uint16_t Size) {}
void SendEthPacket (void* pHeader, uint16_t Size) {}

#define INIT_FOCH_delay 500000u
#define led_delay 70000u

int main ()
{
	int f=0;

	unsigned char letter[14]="Hello World!\r\n";
	
	unsigned int i=0;
	unsigned int j=0;

	
  //  __disable_irq();
  	 RST_CLKConfig ();
     // CAN1Config ();
       UART1Config ();
       UART2Config ();
	ETHERNETConfig ();
	   PORTAConfig ();
	   PORTBConfig ();
	   PORTDConfig ();
	   PORTCConfig ();
	   PORTEConfig ();
	
	NVIC_EnableIRQ (ETHERNET_IRQn);
	NVIC_EnableIRQ (   UART1_IRQn);
	NVIC_EnableIRQ (   UART2_IRQn);

	pcur_Tr= TrBuf;
    pTr_Buf= TrBuf;

    pcur_Rcv= RcvBuf;
    pRcv_Buf= RcvBuf;

	PA7_0; //������������� ����    PA[7] , ���������� ������� DD6
	
	RX_485;

	Zahvat_OFF;
	POWER_ON;
		
	Menu1();
	
	while(1)
	{
			if (i<INIT_FOCH_delay) i=i+1; else i=0;
			if (j<led_delay)       j=j+1; else j=0;
			
		UART_control ();
		
	
		//if (j==led_delay)  FLAG_LED_n=POWER_LED (FLAG_LED_n);
		
		
		if (i==INIT_FOCH_delay) {Transf_485("~6 LD_control;");Control_FOCH_FLAG=1;} 
		if (j==led_delay)     	Led_Zahvat(flag_status_ZAHVAT); //��������� �������� ������� �� ������� ������

		//SendViaUDP (HostIP, 0xCB99 , 2054, letter, 14);f=0;
				
		if ((ETHERNET->PHY_STATUS&0x02) == 0x00) ;//PORTB->SETTX = 1<<15;
		else ;//PORTB->CLRTX = 1<<15;
		if ((ETHERNET->PHY_STATUS&0x08) == 0x00) ;//PORTB->SETTX = 1<<14;
		else ;//PORTB->CLRTX = 1<<14;
	}
}

 



#endif // __Application_c__
