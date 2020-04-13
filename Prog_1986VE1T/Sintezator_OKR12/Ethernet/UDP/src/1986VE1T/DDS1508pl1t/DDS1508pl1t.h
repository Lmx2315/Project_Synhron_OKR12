
#include "opora.h"

typedef struct ShiftReg   //flags //Init_pins_for_DDS   pins_DDS;  // �������� ��������� 
{
  volatile PORT_TypeDef *port_data;  //���� ������ � ������
  unsigned int      	  pin_data;  //����� ������ � ������

  volatile PORT_TypeDef  *adr;  //������ ������ ����� ������� ��������� � ������� ���������
  unsigned char       pin_adr;  //����� ����� ������� ��������� � ������� ���������

  volatile PORT_TypeDef  *wrn;  // ������ ������
  unsigned char   wrn_DDS;  //������ ������

  volatile PORT_TypeDef  *rdn;  // ������ ������
  unsigned char   rdn_DDS;  //������ ������

  volatile PORT_TypeDef  *csn;  // ������ ������ ����������
  unsigned char   csn_DDS;  //������ ������ ����������

  volatile PORT_TypeDef *SCSN;  // ������ ������ ������������� ����������
  unsigned char  SCSN_DDS;  //������ ������ ������������� ���������� 

  volatile PORT_TypeDef *RSTN;  // ������ ������ ������������� ����������
  unsigned char  RSTN_DDS;  //������ ������ ������������� ���������� 
   
} ShiftReg;


typedef struct reg_1508pl1t   //flags //Init_pins_for_DDS   pins_DDS;  // �������� ��������� 
{
  unsigned short       CH1_LS_CTR; //���������� �������� ���.
  unsigned short          CH1_TSW; //���������� �������� ������������ ���������� �������.

  unsigned short    T_SEL_STATE;   //���������� ������: ������� ��������� ������� SEL
  unsigned short        T_E_SEL;   //���������� �������: ����������� SEL

  unsigned short        ROUTE;   //���������� ������� ������ � �������������

  unsigned short        TC_L;    //�������� ������� ��������, ���� [15: 0]
  unsigned short        TC_H;    //�������� ������� ��������, ���� [31:16]

  unsigned short    CH1_LS_TPH1_L; //������� ������������ 1-�� ���� ���-������� [15:0]
  unsigned short    CH1_LS_TPH1_M; //������� ������������ 1-�� ���� ���-������� [31:16]
  unsigned short    CH1_LS_TPH1_H; //������� ������������ 1-�� ���� ���-������� [45:32]

  unsigned short    CH1_LS_TPH2_L; //������� ������������ 2-�� ���� ���-������� [15:0]
  unsigned short    CH1_LS_TPH2_M; //������� ������������ 2-�� ���� ���-������� [31:16]
  unsigned short    CH1_LS_TPH2_H; //������� ������������ 2-�� ���� ���-������� [45:32]

  unsigned short    CH1_LS_TPH3_L; //������� ������������ 3-�� ���� ���-������� [15:0]
  unsigned short    CH1_LS_TPH3_M; //������� ������������ 3-�� ���� ���-������� [31:16]
  unsigned short    CH1_LS_TPH3_H; //������� ������������ 3-�� ���� ���-������� [45:32]

  unsigned short    CH1_LS_TPH4_L; //������� ������������ 4-�� ���� ���-������� [15:0]
  unsigned short    CH1_LS_TPH4_M; //������� ������������ 4-�� ���� ���-������� [31:16]
  unsigned short    CH1_LS_TPH4_H; //������� ������������ 4-�� ���� ���-������� [45:32]

  unsigned short      CH1_LS_F1_L; //������� ��������� ������� ��� 1 [15:0]
  unsigned short      CH1_LS_F1_M; //������� ��������� ������� ��� 1 [31:16]
  unsigned short      CH1_LS_F1_H; //������� ��������� ������� ��� 1 [47:32]

  unsigned short      CH1_LS_F2_L; //������� ��������� ������� ��� 2 [15:0]
  unsigned short      CH1_LS_F2_M; //������� ��������� ������� ��� 2 [31:16]
  unsigned short      CH1_LS_F2_H; //������� ��������� ������� ��� 2 [47:32]

  unsigned short       CH1_LS_Ph1; //������� ��������� ���� ��� 1
  unsigned short       CH1_LS_Ph2; //������� ��������� ���� ��� 2

  unsigned short     CH1_LS_dF1_L; //������� ���������� ������� 1 [15: 0]
  unsigned short     CH1_LS_dF1_M; //������� ���������� ������� 1 [31:16]
  unsigned short     CH1_LS_dF1_H; //������� ���������� ������� 1 [47:32]

  unsigned short     CH1_LS_dF2_L; //������� ���������� ������� 2 [15: 0]
  unsigned short     CH1_LS_dF2_M; //������� ���������� ������� 2 [31:16]
  unsigned short     CH1_LS_dF2_H; //������� ���������� ������� 2 [47:32]

  unsigned short     CH1_dPh_all_L; //������ ���������� ���� [15:0] �� ��� �������
  unsigned short     CH1_dPh_all_M; //������ ���������� ���� [31:16] �� ��� �������
  unsigned short     CH1_dPh_all_H; //������ ���������� ���� [47:32] �� ��� �������

  unsigned short         CH1_P_all; //������ �������� ���� �� ��� �������
  unsigned short       CH1_Mul_all; //������ ������������ �������� �� ��� �������

  unsigned short    CH1_Offset_all; //������ ����������� �������� �� ��� �������

  unsigned short        CH1_dPh0_L; //������� ���������� ���� [15: 0], ������� 0
  unsigned short        CH1_dPh0_M; //������� ���������� ���� [31:16], ������� 0
  unsigned short        CH1_dPh0_H; //������� ���������� ���� [47:32], ������� 0
  
  unsigned short        CH1_dPh1_L; //������� ���������� ���� [15: 0], ������� 0
  unsigned short        CH1_dPh1_M; //������� ���������� ���� [31:16], ������� 0
  unsigned short        CH1_dPh1_H; //������� ���������� ���� [47:32], ������� 0
  
  unsigned short        CH1_dPh2_L; //������� ���������� ���� [15: 0], ������� 0
  unsigned short        CH1_dPh2_M; //������� ���������� ���� [31:16], ������� 0
  unsigned short        CH1_dPh2_H; //������� ���������� ���� [47:32], ������� 0
  
  unsigned short        CH1_dPh3_L; //������� ���������� ���� [15: 0], ������� 0
  unsigned short        CH1_dPh3_M; //������� ���������� ���� [31:16], ������� 0
  unsigned short        CH1_dPh3_H; //������� ���������� ���� [47:32], ������� 0

  unsigned short            CH1_P0; //������� ���������� �����, ������� 0
  unsigned short            CH1_P1; //������� ���������� �����, ������� 1
  unsigned short            CH1_P2; //������� ���������� �����, ������� 2
  unsigned short            CH1_P3; //������� ���������� �����, ������� 3
  
  unsigned short          CH1_Mul0; //������� ���������� ����������, ������� 0 ���������� �������� ��������� � �����������
  unsigned short          CH1_Mul1; //������� ���������� ����������, ������� 1 ���������� �������� ��������� � ���
  unsigned short          CH1_Mul2; //������� ���������� ����������, ������� 2 ���������� �������� ��������� � ���
  unsigned short          CH1_Mul3; //������� ���������� ����������, ������� 3 ���������� �������� ��������� � ���

  unsigned short       CH1_Offset0; //������� ���. ��������� ��������� �������, ������� 0  ���������� ������������ ��������� � �����������
  unsigned short       CH1_Offset1; //������� ���. ��������� ��������� �������, ������� 0  ���������� �������� ��������� � ���
  unsigned short       CH1_Offset2; //������� ���. ��������� ��������� �������, ������� 0  ���������� �������� ��������� � ���
  unsigned short       CH1_Offset3; //������� ���. ��������� ��������� �������, ������� 0  ���������� �������� ��������� � ���

  unsigned short               CLR; //������� ������������� ����, ������ � ��������� ���
  unsigned short              SYNC; //���������� ��������������
  unsigned short               CTR; //������� ����������

  unsigned short           SEL_REG; //����� ��������� ������� �������
  unsigned short             DEVID; //������������� ����������, ������ ������
  unsigned short             SWRST; //������� ������������ ������
  
  //-------------------------------------------------------------------------

  unsigned short             LINK;  //���������� LINK-�����������
 
  unsigned short             CH1_LS_CRFMIN;  //������ ������� ��������������� ��������� ������
  
  unsigned short             CH1_T_SEL; 	 //����� ��������� �������
  
  
  
  
  


} reg_1508pl1t;

typedef struct DDS_param   //flags //Init_pins_for_DDS   pins_DDS;  // �������� ��������� 
{
  unsigned char       regim;// ����� ������ ����������� 1 - ��� , 0 - �����������
  unsigned int   	   F0  ;// ������� �����������������
  unsigned int  	  F_low;// ������� ������ � ��
  unsigned int 	     F_high;// ������� ������� � ��
  unsigned int   	   dImp;// ������������ �������� � ���
  unsigned int   	   FImp;// ������� ���������� ��������� � ��
  unsigned int    		  A;// �������� ��������� � ��
          char   znak_LCHM;// ����������� ��� 1 ��� -1
    
} DDS_param;


 // itoa_t:  ������������ n � ������� � s 
 void itoa_t(int n, char s[]);
