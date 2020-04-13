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


#define PA0_0 PORTA->CLRTX = (1<<0) //устанавливаем ноль    PA[0] , управление 
#define PA0_1 PORTA->SETTX = (1<<0) //устанавливаем еденицу PA[0],  управление

#define PA1_0 PORTA->CLRTX = (1<<1) //устанавливаем ноль    PA[1] , управление 
#define PA1_1 PORTA->SETTX = (1<<1) //устанавливаем еденицу PA[1],  управление

#define PA2_0 PORTA->CLRTX = (1<<2) //устанавливаем ноль    PA[2] , управление 
#define PA2_1 PORTA->SETTX = (1<<2) //устанавливаем еденицу PA[2],  управление

#define PA3_0 PORTA->CLRTX = (1<<3) //устанавливаем ноль    PA[3] , управление 
#define PA3_1 PORTA->SETTX = (1<<3) //устанавливаем еденицу PA[3],  управление

#define PA4_0 PORTA->CLRTX = (1<<4) //устанавливаем ноль    PA[4] , управление 
#define PA4_1 PORTA->SETTX = (1<<4) //устанавливаем еденицу PA[4],  управление

#define PA5_0 PORTA->CLRTX = (1<<5) //устанавливаем ноль    PA[5] , управление 
#define PA5_1 PORTA->SETTX = (1<<5) //устанавливаем еденицу PA[5],  управление

#define PA6_0 PORTA->CLRTX = (1<<6) //устанавливаем ноль    PA[6] , управление 
#define PA6_1 PORTA->SETTX = (1<<6) //устанавливаем еденицу PA[6],  управление

#define PA7_0 PORTA->CLRTX = (1<<7) //устанавливаем ноль    PA[7] , управление 
#define PA7_1 PORTA->SETTX = (1<<7) //устанавливаем еденицу PA[7],  управление

#define PA8_0 PORTA->CLRTX = (1<<8) //устанавливаем ноль    PA[8] , управление 
#define PA8_1 PORTA->SETTX = (1<<8) //устанавливаем еденицу PA[8],  управление

#define PA9_0 PORTA->CLRTX = (1<<9) //устанавливаем ноль    PA[9] , управление 
#define PA9_1 PORTA->SETTX = (1<<9) //устанавливаем еденицу PA[9],  управление


#define PA10_0 PORTA->CLRTX = (1<<10) //устанавливаем ноль    PA[10] , управление буфером DD2 сигнал DE_485 - переводим на передачу
#define PA10_1 PORTA->SETTX = (1<<10) //устанавливаем еденицу PA[10], управление буфером DD2 сигнал DE_485 - переводим на приём 

#define PA11_0 PORTA->CLRTX = (1<<11) //устанавливаем ноль    PA[10] , управление буфером DD2 сигнал DE_485 - переводим на передачу
#define PA11_1 PORTA->SETTX = (1<<11) //устанавливаем еденицу PA[10], управление буфером DD2 сигнал DE_485 - переводим на приём 

#define POWER_ON PA1_1
#define POWER_OFF PA1_0


#define PF2_0 PORTF->CLRTX = (1<<2) //устанавливаем ноль    PF[2] , управление 
#define PF2_1 PORTF->SETTX = (1<<2) //устанавливаем еденицу PF[2],  управление

#define PF3_0 PORTF->CLRTX = (1<<3) //устанавливаем ноль    PF[3] , управление 
#define PF3_1 PORTF->SETTX = (1<<3) //устанавливаем еденицу PF[3],  управление

#define PF4_0 PORTF->CLRTX = (1<<4) //устанавливаем ноль    PF[4] , управление 
#define PF4_1 PORTF->SETTX = (1<<4) //устанавливаем еденицу PF[4],  управление

#define PF5_0 PORTF->CLRTX = (1<<5) //устанавливаем ноль    PF[5] , управление 
#define PF5_1 PORTF->SETTX = (1<<5) //устанавливаем еденицу PF[5],  управление

#define PF6_0 PORTF->CLRTX = (1<<6) //устанавливаем ноль    PF[6] , управление 
#define PF6_1 PORTF->SETTX = (1<<6) //устанавливаем еденицу PF[6],  управление

#define PF7_0 PORTF->CLRTX = (1<<7) //устанавливаем ноль    PF[7] , управление 
#define PF7_1 PORTF->SETTX = (1<<7) //устанавливаем еденицу PF[7],  управление


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

  ShiftReg pins_dds=     {PORTE, 0, //порт 16 бит данных
                          PORTC,15, //порт 16 бит выбора адреса
                          PORTD,15, //ножка записи
                          PORTD, 6, //ножка чтения
                          PORTD, 0, //ножка чипселекта
                          PORTD, 5, //ножка SCSN
                          PORTD, 1  //ножка ресета
                          };    //ножка управления параллельным портом DDS		

    
  DDS_param	 DDS_data= {
								1,
						435000000, //частота моногармоническая в Гц
						430000000, // частота нижняя в Гц
						440000000, // частота верхняя в Гц
							  100, // длительность импульса в мкс
						    10000, // частота следования импульсов в Гц
							    1, // аплитуда импульсов в Дб
							    1  // направление ЛЧМ 1 или -1
						};
//-------------------------------------------------------------------------------------------------------

// UART

 unsigned char DataTr_flag=0;
 unsigned char Data_in_port=0;
 unsigned char DontConnect_flag=0;      // соединение с PC невозможно
 unsigned      CntDown = 0;             // используется для обратн. отсчета при пров. соедин.-
 unsigned char CyclEnd_flag=0;          // если pRcv_Buf достигал конца RcvBuf
                                        // при BufIsRead_flag=0
 unsigned char RcvBufOverflow_flag=0;   // когда не читаем ввод и данные теряются     
                                        // 
 unsigned char BufIsRead_flag=0;        // установим, когда считаны все введ. данные
 unsigned char Data4TrAbsent_flag=1;    // данные для передачи отсутств.
#define buf_size 256
 unsigned char TrBuf[buf_size];
 unsigned char RcvBuf[buf_size];
 unsigned char sch_buf=0;

                                         // исп. для организации циклическ. буферов
                                        // приема и передачи
 unsigned char *pTr_Buf;                // указ. на симв. вывод.-ся в порт
 unsigned char *pcur_Tr;                // указ. на симв. записываеемый в кольц.буф
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
#define  CPUacceler          1     //разгон BF533



//*************************************************************************

#define BUF_RX 64

   	   unsigned char  k;

  unsigned  	char lsym;
  unsigned  	char  sym;
  unsigned  	char flag;
  unsigned  	char packet_flag;
  unsigned      char	NB;

  unsigned    char Adress=0x34;      // адрес блекфина Adress=0x33 - кассета синтезатора
  unsigned    char Master_flag=0x0;  // флаг обозначающий мастер кассету-1,0-вспомогательный синхронизатор  --отменено всегда стоит ноль как мастер

  unsigned    char packet_sum;
  volatile   unsigned  	char crc,comanda=1;
      char     InOut[BUF_RX];
 
      char      Word [BUF_RX];    //массив командного слова
      char DATA_Word [BUF_RX];    //массив слова - данных
      char DATA_Word2[BUF_RX];    //массив слова - данных
	  
			char s1[BUF_RX];
			char s2[BUF_RX];
			char s3[BUF_RX];
			char sr[BUF_RX];
			
  unsigned    char crc_ok;
  unsigned    char packet_ok;
  unsigned    char ink1; // счётчик байт в пакете
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
  
      		     
 unsigned volatile int  time_uart; // переменная подсчёта времени
 unsigned volatile int  tick_wait;
			
		unsigned  char UDP_TCP_flag=0;
		
                     unsigned char CRC_m[3][6]; //массив выборов
                     unsigned char K615_indik=0;	
              volatile       unsigned char K615_crc_sch=0;   
              volatile       unsigned char K615_crc_sch2=0;  
			
                    unsigned  char Error_ethernet_obmen=0; //переменная контроля ошибок обмена
 volatile unsigned  char flag_pachka_sinhron; //флаг готовности принять новую пачку импульсов к выводу в синхронизатор

 volatile  unsigned  char flag_pachka_TXT; //флаг показывающий что идёт передача пачки данных
 volatile  unsigned  char flag_pachka_TXT2; //флаг показывающий что идёт передача пачки данных

 volatile  unsigned  int sch_pachek_test=0; //счётчик контрольных пакетов  в терминал показывающий что идёт передача пачки данных


  unsigned volatile char          flag_K615_event;  //event по событиям в кассете 615

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


unsigned int Alfa =0;  //переменные времени для формирования пачек синхронгизатора
unsigned int Beta =0;
unsigned int Gamma=0;

unsigned short TNC_actual  =0; //текущий ТНЦ	
unsigned short TNO_actual  =0; //текущий ТНО
unsigned int   Time_Preset =0; //предустановленное время в мкс


unsigned int   TNI_a[32];   //массивы импульсов
unsigned int   TKI_a[32];
unsigned int   TNP_a[32];
unsigned int   TKP_a[32];
unsigned int   TNO_a[32];
unsigned int   TNC_a[32];
unsigned int  TOBM_a[32];
unsigned int  TDDS_a[32];
/*
unsigned int   TNI_b[32];   //массивы импульсов
unsigned int   TKI_b[32];
unsigned int   TNP_b[32];
unsigned int   TKP_b[32];
unsigned int   TNO_b[32];
unsigned int   TNC_b[32];
unsigned int  TOBM_b[32];
*/

unsigned int   tin_k=0;  //счётчики импульсов
unsigned int   tik_k=0;  //счётчики импульсов
unsigned int   tpn_k=0;
unsigned int   tpk_k=0;
unsigned int tdds_k=0;
unsigned int  tno_k=0;
unsigned int  tnc_k=0;

unsigned int  TNO_work1=0;
unsigned int  TNO_work2=0;
unsigned char flag_TNO_work2=0; //флаг устанавливаемый в случае обрыва пакета перед установкой сигнала ТНО
unsigned int dFo1;

 
 //****************************DSP***********************************************   
  char   dma_buf;		

   unsigned char A[256];  //массив импульсов для синхронизатора
   unsigned char SintezI[256];  //массив   для cинтезатора1
   unsigned char SintezG[256];  //массив   для cинтезатора2

  unsigned int  Inf_A1[1]; // массив Состав информации управления на РЭМ 3  1У07ФВ
  unsigned char Inf_A2[15];  // массив Состав информации состояния  от РЭМ 3  1У07ФВ
  //unsigned int  Inf_A3[31]; // массив Состав информации контрольного чтения от РЭМ 3  1У07ФВ
  
  unsigned char Code_error=0;       //Код ошибки D15?D12 – в 1У08ПП не используются.
  unsigned char Norm_ohlagdeniya=0; //Норма охлаждения D11 – норма охлаждение - лог. «1» .

  unsigned char PriemNi=0;          //Ком. М2  Ош D10 – информация о принятой ошибке в команде,
// при лог. "0" в разряде D8 - трактуется как сбой обмена, при лог.  «1»- как ошибка.
// D9 – информация о принятой ошибке по М2, при лог.
// "0" в разряде D8 -трактуется как сбой об-мена, при лог. «1»- как ошибка.

  unsigned char Vikl=0;             //Исп.Зад. 
  unsigned char GBR=0;              //Исп.Зад. 
  unsigned char BR=0;               //Исп.Зад. 
  unsigned char CU_MU=0;            //Исп.Зад. 
  unsigned char T_norma=0;          // норма температуры инженерного оборудования 1У07ФИ (лог. «1»).
  unsigned char Pred_avariya=0;     //предавария инженерного оборудования 1У07ФИ (лог.  «1»).
  
  unsigned char Norma_amplifer=0;   //D13?D10 – наличие единицы в одном из разрядов D10-D13,
// говорит о том, что работа идет по усилителю БР данного канала. 
// Разряд D10 соответствует 1 каналу, содержащему Ком.2, усили-тель БР и Ком.6 (см. рисунок 3.8).
//  В случае неработоспособности усилителя БР (в соответст-вующем бите логический нуль) из канала,
//   1У07ФТ переключается на усилитель ГБР 1 канала.

  unsigned char Norma_pitaniyaFT=0;  //D9?D6 – наличие единицы говорит о норме питания источника ВИП в соответствующем кана-ле.
 //При начальном включение (исполнение команды БР на одном из комплектов ФВ) какое-то время нули во всех разрядах.
 
  unsigned char Avariya_blokov=0;  // Б600  Б605  Б604  Б603  Б602  Б601,авариии блоков шкафа 1У07ФВ (лог.  «1») .
  
  unsigned char Rezerv1=0;          //
  
  unsigned char Norma_IVE=0;       //норма ИВЭ шкафа 1У07ФВ - (лог.  «1»).
  
  unsigned char Rezerv2=0;          //
  
  unsigned char Nomer_rezer_amplifer_FT=0;     //D3?D0 – разряд D0 соответствует каналу, содержащему Ком.2,
// Усил.1 ГБР и Ком.6 (см. рису-нок 3.8).
// Наличие нуля в данном разряде говорит о том, что возникла авария усилителя по дан-ному каналу,
// т.е. он вышел из строя и канал 1 работает на резервном усилителе ГБР. Анало-гично и для других разрядов.

//Пример:
//?	Если в разряде D10 слова 02h единица, а в разряде D0 слова 04h нуль, то данная ситуация говорит о том, что работает усилитель БР 1 канала. 
//?	Если в разряде D10 слова 02h нуль, а в разряде D0 слова 04h единица, то данная ситуация говорит о том, что работает усилитель ГБР (усилитель БР – неисправен) 1 канала. 
//?	Если в разряде D10 слова 02h нуль, а в разряде D0 слова 04h нуль, то данная ситуация говорит о том, что работают усилители БР и ГБР. В таком случае если в разряде D6 слова 02h нуль, то считается неисправным ВИП 1 канала. Если в разряде D6 слова 02h единица, то считаются неисправными коммутаторы 1 канала.

//..................Кнц – двоично-десятичный код времени начала цикла в пределах 5 минут (см. Таблицу 3.4)........................................

 unsigned char Knc_min=0;    //  Кнц минут
 
 unsigned char Knc_10sec=0;  //  Кнц десятков секунд

 unsigned char Knc_sec=0;    //  Кнц  секунд

 unsigned char Knc_100ms=0;  //  Кнц сотен мс
 
 unsigned char Knc_10ms=0;   //  Кнц десятков мс
 
 unsigned char Knc_ms=0;     //  Кнц  мс
 
 unsigned char Knc_100us=0;  //  Кнц сотен мкс
 
 unsigned char Knc_10us=0;   //  Кнц десятков мкс
 
 unsigned char Knc_us=0;     //  Кнц  мкс

/* 
Кнц минут – код количества минут в текущей пятиминутке.
Кнц десятков секунд – код количества десятков секунд в текущей минуте.
Кнц секунд – код количества секунд в текущей десятке.
Кнц сотен мс - код количества сотней мс в текущей секунде.
Кнц десятков мс - код количества десятков мс в текущей сотне.
Кнц единиц мс - код количества мс в текущей десятке.
Кнц сотен мкс - код количества сотней мс в текущей мс.
Кнц десятков мкс - код количества десятков мкс в текущей сотне.
Кнц единиц мкс- код количества мкс в текущей десятке.
  */

  //Зарезервирован для местного управления и определяют код текущего времени (минуты, часы, сутки), установленный вручную.
  
 unsigned char Mypr_min=0;    //  минуты
 
 unsigned char Mypr_hour=0;   //  часы
 
 unsigned char Mypr_day=0;    //  сутки
 
 //..................................................................
 
 
  unsigned char NNI=0;        			 //наличие нулевых интервалов Пии;
  
  unsigned char RC_KTV_T5M=0; 			 //бит 2 – рассогласование сетки КТВ относительно Т5М; 
  
  unsigned char Tc_2sec=0;    			 //бит 3 – Тц   2 сек. – длительность цикла больше 2 сек.;
  
  unsigned char SintV_nesootv_zadan=0;   //бит 4 – сформированные интервалы времени не соответствуют заданным;
  
  unsigned char In_sys_Obmen_error=0;   //обмен с 1УЯ203 начался, но не удовлетворяет заданным критериям;
  
  unsigned char Sboy_form_interval=0;   //бит 6 – единичный сбой формирования интервалов;
  
  unsigned char Sboy_obmena_inform=0;   //бит 7– единичный сбой обмена информацией.
  
  unsigned char FAPCH_1=0;   			//бит 0 - контроль ФАПЧ генератора fи (1У-К605);
  
  unsigned char FAPCH_2=0;   			//бит 1 - контроль ФАПЧ генератора fи (1У-К603);
  
  unsigned char Zakon_modulac_kontrol=0;//контроль сигнала СО сформированных законов модуляции (1У-К605, 1У-К606, 1У-К608);
  
  unsigned char FAPCH_3=0;   			//бит 3 - контроль ФАПЧ генератора fи (1У-К606);
  
  unsigned char LCHM_osnv=0;   		    	//бит0 – наличие/отсутствие режима ЛЧМ основного сигнала
  
  unsigned char Dop_interval=0;   		//бит1 – наличие/отсутствие дополнительного интервала
  
  unsigned char Type_pachki=0;   		//бит2 1 – когерентная / 0 - не когерентная (при N>0) пачка сигналов
  
  unsigned char LCHM_pomeha=0;   		//бит3 – наличие/отсутствие ЛЧМ в сигнале “помехи”
  
  unsigned char SYNC_T0Tnc=0;   		//бит7 – наличие/отсутствие синхронизации   по  

  unsigned char LCHM_osnv2=0;   		    	//бит0 – наличие/отсутствие режима ЛЧМ основного сигнала
  
  unsigned char Dop_interval2=0;   		//бит1 – наличие/отсутствие дополнительного интервала
  
  unsigned char Type_pachki2=0;   		//бит2 1 – когерентная / 0 - не когерентная (при N>0) пачка сигналов
  
  unsigned char LCHM_pomeha2=0;   		//бит3 – наличие/отсутствие ЛЧМ в сигнале “помехи”
  
  unsigned char SYNC_T0Tnc2=0;   		//бит7 – наличие/отсутствие синхронизации   по  

  unsigned char LCHM_osnv3=0;   		    	//бит0 – наличие/отсутствие режима ЛЧМ основного сигнала
  
  unsigned char Dop_interval3=0;   		//бит1 – наличие/отсутствие дополнительного интервала
  
  unsigned char Type_pachki3=0;   		//бит2 1 – когерентная / 0 - не когерентная (при N>0) пачка сигналов
  
  unsigned char LCHM_pomeha3=0;   		//бит3 – наличие/отсутствие ЛЧМ в сигнале “помехи”
  
  unsigned char SYNC_T0Tnc3=0;   		//бит7 – наличие/отсутствие синхронизации   по  

  unsigned char TNO_reg=1;              //регистр определяющий начало ТНО
  
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  unsigned char P0=0;  //Длительность обзора (секунды), определяет интервал времени между соседними импульсами Тно , Do = По-1
  unsigned int Pniu=0;  //Интервал времени, 
//  требуемый для подготовки синтезатора (DDS) к работе 
//  (инициализация регистров по информации полученной от алгоритма управления). 
//  Должен упреждать импульс Тни не менее, чем на 8 мкс Dниу = Пниу/8, Dниу<Dни

  unsigned int Pni=0;  //Интервал времени отсчитываемый относительно последнего импульса Ткп, 
//  либо от импульса Тнц (для первого интервала излучения в случае выключенного дополнительного интервала)
//  до импульса Тни, определяющий момент начала излучения Dни = Пни/8, Dни>Dниу

 unsigned int Pii=0;  //Интервал времени от Тни до Тки, в течение которого проводится излучение сигнала, Dниу = Пниу/8
 
 unsigned int Pnpu=0;  //Интервал времени, 
//  требуемый для подготовки синтезатора (DDS) к работе 
//  (инициализация регистров по информации полученной от алгоритма управления). 
//  Должен упреждать импульс Тни не менее, чем на 8 мкс Dнпу = Пнпу/8, Dнпу<Dнп

 unsigned int    Pnp=0;  //Интервал времени между Тки и Тнп, Dнп = Пнп/8, Dнп>Dнпу
 
 unsigned int    Pip=0;  //Интервал приема, определяющий длительность приема. Задает интервал времени между импульсами Тнп и Ткп Dип = Пип/8 - 16
 
 unsigned int Pdop=848;  //Длительность дополнительного интервала
 
 unsigned int  Pnpk=16;  //Интервал между импульсами Тнц и Тнп калибровки
 
 unsigned int   Pk=304;   //Длительность интервала калибровки
 
 unsigned int  Pnpp=16;  //Интервал между импульсами Ткп калибровки и Тнп приема помехи
 
 unsigned int   Pp=512;  //Длительность интервала приема помехи
 
 unsigned char  Ntp=1;  //Число темпов в  пачке Dтп = Nтп , Максимальное значение = 8
 
 unsigned char   Nt=1;  //Число пачек в цикле Dт = Nт-1 , Максимальное значение = 3

 unsigned char  Ntc=1;  //Число темпов в цикле

unsigned char  n1,n2,n3=1;  //Число n При задании режима ЛЧМ с девиацией
 //  Гц (код Дрр(0) должен быть равен  «1») 
 //  длительность интервала модуляции должна удовлетворять условию:

//--------------------импульс 2 -------------------------------------------------


  unsigned int Pniu2=0;  //Интервал времени, 
//  требуемый для подготовки синтезатора (DDS) к работе 
//  (инициализация регистров по информации полученной от алгоритма управления). 
//  Должен упреждать импульс Тни не менее, чем на 8 мкс Dниу = Пниу/8, Dниу<Dни

  unsigned int Pni2=0;  //Интервал времени отсчитываемый относительно последнего импульса Ткп, 
//  либо от импульса Тнц (для первого интервала излучения в случае выключенного дополнительного интервала)
//  до импульса Тни, определяющий момент начала излучения Dни = Пни/8, Dни>Dниу

 unsigned int Pii2=0;  //Интервал времени от Тни до Тки, в течение которого проводится излучение сигнала, Dниу = Пниу/8
 
 unsigned int Pnpu2=0;  //Интервал времени, 
//  требуемый для подготовки синтезатора (DDS) к работе 
//  (инициализация регистров по информации полученной от алгоритма управления). 
//  Должен упреждать импульс Тни не менее, чем на 8 мкс Dнпу = Пнпу/8, Dнпу<Dнп

 unsigned int    Pnp2=0;  //Интервал времени между Тки и Тнп, Dнп = Пнп/8, Dнп>Dнпу
 
 unsigned int    Pip2=0;  //Интервал приема, определяющий длительность приема. Задает интервал времени между импульсами Тнп и Ткп Dип = Пип/8 - 16
 
 unsigned int Pdop2=848;  //Длительность дополнительного интервала
 
 unsigned int  Pnpk2=16;  //Интервал между импульсами Тнц и Тнп калибровки
 
 unsigned int   Pk2=304;   //Длительность интервала калибровки
 
 unsigned int  Pnpp2=16;  //Интервал между импульсами Ткп калибровки и Тнп приема помехи
 
 unsigned int   Pp2=512;  //Длительность интервала приема помехи
 
 unsigned char  Ntp2=1;  //Число темпов в  пачке Dтп = Nтп , Максимальное значение = 8
 
 unsigned char   Nt2=1;  //Число пачек в цикле Dт = Nт-1 , Максимальное значение = 3

 unsigned char  Ntc2=1;  //Число темпов в цикле


//----------------------импульс 3-----------------------------------

 
  unsigned int Pniu3=0;  //Интервал времени, 
//  требуемый для подготовки синтезатора (DDS) к работе 
//  (инициализация регистров по информации полученной от алгоритма управления). 
//  Должен упреждать импульс Тни не менее, чем на 8 мкс Dниу = Пниу/8, Dниу<Dни

  unsigned int Pni3=0;  //Интервал времени отсчитываемый относительно последнего импульса Ткп, 
//  либо от импульса Тнц (для первого интервала излучения в случае выключенного дополнительного интервала)
//  до импульса Тни, определяющий момент начала излучения Dни = Пни/8, Dни>Dниу

 unsigned int Pii3=0;  //Интервал времени от Тни до Тки, в течение которого проводится излучение сигнала, Dниу = Пниу/8
 
 unsigned int Pnpu3=0;  //Интервал времени, 
//  требуемый для подготовки синтезатора (DDS) к работе 
//  (инициализация регистров по информации полученной от алгоритма управления). 
//  Должен упреждать импульс Тни не менее, чем на 8 мкс Dнпу = Пнпу/8, Dнпу<Dнп

 unsigned int    Pnp3=0;  //Интервал времени между Тки и Тнп, Dнп = Пнп/8, Dнп>Dнпу
 
 unsigned int    Pip3=0;  //Интервал приема, определяющий длительность приема. Задает интервал времени между импульсами Тнп и Ткп Dип = Пип/8 - 16
 
 unsigned int Pdop3=848;  //Длительность дополнительного интервала
 
 unsigned int  Pnpk3=16;  //Интервал между импульсами Тнц и Тнп калибровки
 
 unsigned int   Pk3=304;   //Длительность интервала калибровки
 
 unsigned int  Pnpp3=16;  //Интервал между импульсами Ткп калибровки и Тнп приема помехи
 
 unsigned int   Pp3=512;  //Длительность интервала приема помехи
 
 unsigned char  Ntp3=1;  //Число темпов в  пачке Dтп = Nтп , Максимальное значение = 8
 
 unsigned char   Nt3=1;  //Число пачек в цикле Dт = Nт-1 , Максимальное значение = 3

 unsigned char  Ntc3=1;  //Число темпов в цикле

 
  //*****************флаги процессов***********************************
  
 volatile unsigned char               Flag_K611=0;
 volatile unsigned char          Flag_init_K611=0;
 volatile unsigned char       Flag_control_K611=0;
 volatile unsigned char   Flag_control_sig_K611=0;
 volatile unsigned char   Flag_control_end_K611=0;
 volatile unsigned char    Flag_zahvat_sig_K611=0;
 volatile  unsigned char   Flag_zahvat_end_K611=0;
 volatile unsigned char              Qwant_K611=0;
 volatile unsigned char     Flag_zahvat_OK_K611=0;//общий флаг наличия сигналов захвата
 volatile unsigned char     Flag_signal_OK_K611=0;// общий флаг что все сигналы в норме
  
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

  
 unsigned volatile char flag_Ethernet;  //флаг означающий что пришёл пакет по эзернету
       unsigned volatile char flag_Ethernet_Terminal=0;  //флаг означающий что эзернету пакет в обработке , выводим.

     
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
  u8 Kod_zameny     : 4;        //  код замены  4-ре бита
  u8 KCH_OZU        : 4;        //  контрольное чтение ОЗУ 4-ре бита
  u8 Kol_error      : 2;        //  количество ошибок 2-ва бита
  u8 Nomer_prov_cepi: 4;        //  номер проверяемой цепи 4-ре бита
  u8 Kod_error      : 4;        //  код ошибки 2-ва бита

  u16    Nomer_TNC     :16;        // номер принятого ТНЦ
  u16    Nomer_TNO     :16;        // номер принятого ТНЦ
 // int32  Time_SEV      :32;        // время СЕВ , в секундах.



/*
Основное назначение - контроль обмена с абонентом.
______________________________________________________________________________
D15 D14 D13 D12    D11 D10 D9 D8    D7  D6    D5  D4  D3  D2        D1  D0
  Код замены    |    Резерв      |   Кол.   | Номер проверяемой  | Тип ошибки
                                    ошибок         цепи
Зр  2р  1р  0р     Зр  2р  1р  0р   1р  0р    Зр  2р  1р  0р        1р  0р
------------------------------------------------------------------------------

D15-D12- имитация ошибки команды или свертки по М2 при записи .
D7, D6 - задание ошибок в информации от абонента при чтении: 
код 00 - нет ошибок, код 01- ошибки по М2 в первой посылке обмена,
код 10 - ошибки по М2 в первой и второй посылках обмена,
код 11 - ошибки по М2 в трех посылках обмена.
D5-D0 – в секции 1У08ФВ не используются.


*/

  u8 Zadano_off     : 4;        //  Задано ВЫКЛ  , 4-ре бита
  u8 Zadano_GBR     : 4;        //  Задано ГБР   , 4-ре бита
  u8 Zadano_BR      : 4;        //  Задано БР    , 4-ре бита
  u8 Zadano_CU_MU   : 4;        //  Задано ЦУ/МУ , 4-ре бита
  
/*
Основное назначение - выбор режимов и видов управления шкафа 1У07ФВ.

D15 D14 D13 D12  D11 D10 D9  D8   D7  D6  D5  D4     D3  D2  D1  D0
ВЫКЛ           |      ГБР       |       БР        |     ЦУ/МУ
3р  2р  1р  0р  3р  2р  1р  0р    3р  2р  1р  0р      3р 2р  1р  0р

D15-D12, Dl 1-D8, D7-D4 - задание команды - код 0101,
 отмена команды - код 1100, 
 нет изме-нений - код 0000.
  D3-D0-ЦУ-код 0101,
   МУ-код 1100.
   Остальные комбинации - ошибка команды.

*/

 u16 Kod_Dniu        : 16;       //  Код Dниу
 
 u16 Kod_Dniu2       : 16;       //  Код Dниу
 
 u16 Kod_Dniu3       : 16;       //  Код Dниу

/*Интервал времени Dниу ,
 требуемый для подготовки синтезатора (DDS)
  к работе (инициализация регистров по информации полученной от алгоритма управления).
 16-разрядный код длительности упрежденного интервала перед началом излучения.
   Должен упреждать импульс Тни не менее, чем на 8 мкс 
   Расчетная формула:   Dниу = Пниу/8, Dниу<Dни


      */


 u16 Kod_Dii         : 16;       //  Код Dии
 
 u16 Kod_Dii2        : 16;       //  Код Dии
 
 u16 Kod_Dii3        : 16;       //  Код Dии
 
 /*
 16 разрядный код длительности интервала излучения.
Интервал времени от Тни до Тки, в течение которого проводится излучение сигнала.
Формула : Dниу = Пниу/8;  Пии, мкс ; Минимальное значение 128 мкс , дискрет 8 мкс.

0A  |  Код Dии-байт 1 (адрес 7) – имп.1  Код Dии-байт 2 (адрес 6) – имп.1
    |  7р  6р  5р  4р  3р  2р  1р  0р    7р  6р  5р  4р  3р  2р  1р  0р


 */

u16 Kod_Dni         : 16;       //  Код Dни

u16 Kod_Dni2        : 16;       //  Код Dни

u16 Kod_Dni3        : 16;       //  Код Dни

 /*

Интервал времени Пни, мкс  (дискрет изменения 8 мкс)
 отсчитываемый 
 относительно последнего импульса Ткп,
  либо от импульса Тнц (для первого интервала излучения в случае выключенного дополнительного интервала)
   до импульса Тни, определяющий момент начала излучения

 */

 u16 Kod_Dnp         : 16;       //  Код Dнп
 
 u16 Kod_Dnp2        : 16;       //  Код Dнп 
 
 u16 Kod_Dnp3        : 16;       //  Код Dнп 

/*
16 разрядный код длительности интервала перед началом приема
Пнп, мкс минимальное значение 16 (дискрет изменения 8 мкс)
Интервал времени между Тки и Тнп
формула:Dнп = Пнп/8, Dнп>Dнпу
*/

 u16 Kod_Dnpu        : 16;       //  Код Dнпу
 
 u16 Kod_Dnpu2       : 16;       //  Код Dнпу
 
 u16 Kod_Dnpu3       : 16;       //  Код Dнпу

/*
Пнпу, мкс  (дискрет изменения 8 мкс)
Интервал времени,
 требуемый для подготовки синтезатора (DDS)
  к работе 
  (инициализация регистров по информации полученной от алгоритма управления). 
  Должен упреждать импульс Тнп не менее, чем на 8 мкс

  формула: Dнпу = Пнпу/8, Dнпу<Dнп

*/

 u16 Kod_DN         : 16;       //  Код Dn
 
/*

8 разрядный код числа одинаковых по длительности импульсов в пачке.
Количество импульсов в пачке равно N+1 ;

  12              Резерв              Код DN-байт 2 (адрес 16)
  7р  6р  5р  4р  3р  2р  1р  0р  |  7р  6р  5р  4р  3р  2р  1р  0р



*/

 u16 Kod_Dip         : 16;       //  Код Dип
 
 u16 Kod_Dip2        : 16;       //  Код Dип
 
 u16 Kod_Dip3        : 16;       //  Код Dип
 
/*
Пип, мкс  (дискрет изменения 8 мкс), минимальное значение 128
16 разрядный код длительности интервала приема.
 определяющий длительность приема.
  Задает интервал времени между импульсами Тнп и Ткп
  формула: Dип = Пип/8 - 16


*/

  u8 Kod_Kp_hour    : 8;        //  Код Кп - часы  (адрес 21)  
  u8 Kod_Kp_day     : 8;        //  Код Кп - сутки (адрес 20) 

/*
Используются в режиме МУ, поэтому коды этих команд в режиме ЦУ должны быть заполнены «0».
*/

  u8 Kod_Dm         : 8;        //  Код Dм         (адрес 19) 
  
/*
8 разрядный код количества темпов в цикле.
Под темпом понимается отрезок времени, состоящий из двух интервалов:
- основного, включающего  N+1 одинаковых интервалов, определяемых кодами Дни, Дии, Днп, Дип, ДN;
- дополнительного, который может быть равен либо нулю, либо 848 мкс.
 Состояние этого интервала определяется параметром Д в коде Дрр.

 15         Код Dм (адрес 19)
      7р  6р  5р  4р  3р  2р  1р  0р



*/

 u8 Kod_Do         : 8;        //  Код Do         (адрес 18) -

  /*
    минимальное значение = 1 сек, максимальное 16 сек
    4 разрядный код длительности обзора (биты 0, 1, 2, 3). Биты 4, 5, 6, 7 свободны.
   Длительность обзора, определяет интервал времени между соседними импульсами Тно,
   (Условное обозначение интервала) - По (в секундах), Do = По-1 - формула расчёта 

  14        Код Dо (адрес 18)
      7р  6р  5р  4р  3р  2р  1р  0р


   */

  u8 Kod_Dpp1        : 8;        //  Код Dрр        (адрес 29) – имп.1
  u8 Kod_Dpp2        : 8;        //  Код Dрр        (адрес 29) – имп.2
  u8 Kod_Dpp3        : 8;        //  Код Dрр        (адрес 29) – имп.3 
  
/*
8 разрядный код режима работ (1/0) (для трех импульсов излучения в цикле)

1B       Код Dpp (адрес 29)
      7р  6р  5р  4р  3р  2р  1р  0р
47       Код Dpp (адрес 29)
      7р  6р  5р  4р  3р  2р  1р  0р
67       Код Dpp (адрес 29)
      7р  6р  5р  4р  3р  2р  1р  0р

  бит0 – наличие/отсутствие режима ЛЧМ основного сигнала
  бит1 – наличие/отсутствие дополнительного интервала
  бит2 – когерентная /не когерентная (при N>0) пачка сигналов
  бит3 – наличие/отсутствие ЛЧМ в сигнале “помехи”
  бит7 – наличие/отсутствие синхронизации  T0 по  Ti0
  биты 4,5,6,7 равны нулю (используются в местном управлении)


*/

  u8 Kod_Dn1         : 8;        //  Код Dн         (адрес 28)
  u8 Kod_Dn2         : 8;        //  Код Dн         (адрес 28)
  u8 Kod_Dn3         : 8;        //  Код Dн         (адрес 28) 

/*
Команды, определяющие параметры модуляции сигналов.
1A      Код Dn (адрес 28) – имп 1
      7р  6р  5р  4р  3р  2р  1р  0р
46      Код Dn (адрес 28) – имп 2
      7р  6р  5р  4р  3р  2р  1р  0р
66      Код Dn (адрес 28) – имп 3
      7р  6р  5р  4р  3р  2р  1р  0р

8 разрядный код числа  n (для трех импульсов излучения в цикле).
При задании режима ЛЧМ с девиацией   Гц (код Дрр(0) должен быть равен  «1»)
 длительность интервала модуляции должна удовлетворять условию:

 При задании режима ЛЧМ с девиацией  2-10**6 Гц (код Дрр(0) должен быть равен  «1»)
  длительность интервала модуляции должна удовлетворять условию:
 Tm=dT*2**i*n (мкс),
где n=2,3,...255
 .
Числа i  и n  определяются ...n-1
 .



*/

  u8  Kod_Krp        : 8;        //  Код Крп        (адрес 23) 
  u32 Kod_Kp         : 24;        //  Код Кп- минуты (адрес 22) 
  
/*
16    Код Кп-байт 2 (адрес 21)       Код Кп-байт 3 (адрес 20)
      7р  6р  5р  4р  3р  2р  1р    0р  7р  6р  5р  4р  3р  2р  1р  0р
18    Код Крп (адрес 23)             Код Кп-байт 1 (адрес 22)
      7р  6р  5р  4р  3р  2р  1р    0р  7р  6р  5р  4р  3р  2р  1р  0р


Используются в режиме МУ, поэтому коды этих команд в режиме ЦУ должны быть заполнены «0».
*/

unsigned long Kod_Ddi_48      : 32;       //  Код Dди байты 2 - 5 (адрес 35)(адрес 34)(адрес 33)(адрес 32)   
  u16 Kod_Ddi_16     : 16;       //  Код Dди байты 0,1   (адрес 37)(адрес 36)  
 
unsigned long Kod_Ddi_48_2     :32;       //  Код Dди байты 2 - 5 (адрес 35)(адрес 34)(адрес 33)(адрес 32)   
u16 Kod_Ddi_16_2     : 16;       //  Код Dди байты 0,1   (адрес 37)(адрес 36)
 
unsigned long Kod_Ddi_48_3    : 32;       //  Код Dди байты 2 - 5 (адрес 35)(адрес 34)(адрес 33)(адрес 32)   
u16 Kod_Ddi_16_3     : 16;       //  Код Dди байты 0,1   (адрес 37)(адрес 36) 

unsigned int Kod_Ddi_32      : 32;       //  Код Dди байты 2 - 5 (адрес 35)(адрес 34)(адрес 33)(адрес 32)   

unsigned int Kod_Ddi_32_2     :32;       //  Код Dди байты 2 - 5 (адрес 35)(адрес 34)(адрес 33)(адрес 32)   

unsigned int Kod_Ddi_32_3    : 32;       //  Код Dди байты 2 - 5 (адрес 35)(адрес 34)(адрес 33)(адрес 32)   


/*
48 разрядный код девиации частоты излучения.
Код   должен быть равен 1 (наличие ЛЧМ), в противном случае код 0 .

Режим «без ЛЧМ» необходим в случае,
 когда модуляция нужна только гетеродинной частоты, т.е. при приеме.
  В этом случае код   должен быть все равно равен  «1».
Выбор режима осуществляется извне,
 поэтому внутри секции ФВ девиация частоты сигнала полностью определяется кодом,
  сформированным программой ФУ (согласно заданному режи-му).

  1C      Код Dди-байт 5 (адрес 33) – имп.1       Код Dди-байт 6 (адрес 32) – имп.1
         7р  6р  5р  4р  3р  2р  1р              0р  7р  6р  5р  4р  3р  2р  1р  0р
  1E      Код Dди-байт 3 (адрес 35) – имп.1       Код Dди-байт 4 (адрес 34) – имп.1
         7р  6р  5р  4р  3р  2р  1р              0р  7р  6р  5р  4р  3р  2р  1р  0р
  20      Код Dди-байт 1 (адрес 37) – имп.1       Код Dди-байт 2 (адрес 36) – имп.1
         7р  6р  5р  4р  3р  2р  1р              0р  7р  6р  5р  4р  3р  2р  1р  0р


*/

 u32 Kod_Dchi        : 24;       //  Код Dчи (адрес 39)(адрес 38)(адрес 40)
 
 u32 Kod_Dchi2       : 24;       //  Код Dчи (адрес 39)(адрес 38)(адрес 40) 
 
 u32 Kod_Dchi3       : 24;       //  Код Dчи (адрес 39)(адрес 38)(адрес 40) 
 
/*
24 разрядный код начальной частоты излучения.
Начальная частота излучения определяется выражением:
 (430*10**6 + b*df)Гц,
где  df=10^7/2^10 Гц. примерно 9765,625 Гц.
Таким образом, для формирования кода   необходимо определить число  :
 ,
при этом  .


*/

unsigned long Kod_Ddg_48        : 32;       //  Код Dдг (адрес 53)(адрес 52)(адрес 51)       
 u16 Kod_Ddg_16        : 16;       //  Код Dдг (адрес 50)(адрес 49)(адрес 48)  
 
unsigned long Kod_Ddg_48_2      : 32;       //  Код Dдг (адрес 53)(адрес 52)(адрес 51)       
 u16 Kod_Ddg_16_2      : 16;       //  Код Dдг (адрес 50)(адрес 49)(адрес 48)  
 
unsigned long Kod_Ddg_48_3      : 32;       //  Код Dдг (адрес 53)(адрес 52)(адрес 51)       
 u16 Kod_Ddg_16_3      : 16;       //  Код Dдг (адрес 50)(адрес 49)(адрес 48)  
 
unsigned int Kod_Ddg_32        : 32;       //  Код Dдг (адрес 53)(адрес 52)(адрес 51)       
 
unsigned int Kod_Ddg_32_2      : 32;       //  Код Dдг (адрес 53)(адрес 52)(адрес 51)       
 
unsigned int Kod_Ddg_32_3      : 32;       //  Код Dдг (адрес 53)(адрес 52)(адрес 51)       
 

/*
48 разрядный код девиации частоты гетеродина на интервале приема.
Код   должен быть равен 1 (наличие ЛЧМ).


*/

 u32 Kod_Dchg        : 24;       //  Код Dчг (адрес 56)(адрес 55)(адрес 54)
 
 u32 Kod_Dchg2       : 24;       //  Код Dчг (адрес 56)(адрес 55)(адрес 54)
 
 u32 Kod_Dchg3       : 24;       //  Код Dчг (адрес 56)(адрес 55)(адрес 54)
 

/*
24 разрядный код начальной частоты гетеродинного сигнала на интервале приема.
Частота гетеродинного сигнала на интервале приема определяется выражением:
  (Гц), 

2A       Код Dчг-байт 3 (адрес 54) – имп.1 
         7р  6р  5р  4р  3р  2р  1р  0р                
2C       Код Dчг-байт 1 (адрес 56) – имп.1    Код Dчг-байт 2 (адрес 55) – имп.1
         7р  6р  5р  4р  3р  2р  1р  0р         7р  6р  5р  4р  3р  2р  1р  0р
*/

 u32 Kod_Dchp       : 24;       //  Код Dчп (адрес 58)(адрес 57)  
 
/*
24 разрядный код частоты гетеродинного сигнала на интервале “помехи” 
Код   должен быть выставлен равным  «1» (наличие дополнительного интервала).
Частота гетеродинного сигнала на интервале “помехи” определяется выражением:
*/

 u32 Kod_Dchk       : 24;       //  Код Dчк (адрес 62)(адрес 61)(адрес 60)
 
/*
24 разрядный код частоты гетеродинного сигнала на интервале «калибровка».
Код   должен быть выставлен в  «1» (наличие дополнительного интервала).

30       Код  Dчк-байт 3 (адрес 60)  
       7р  6р  5р  4р  3р  2р  1р  0р                
32       Код  Dчк-байт 1 (адрес 62)       Код  Dчк-байт 2 (адрес 61)
       7р  6р  5р  4р  3р  2р  1р  0р    7р  6р  5р  4р  3р  2р  1р  0р


*/

 unsigned long Kod_Ddp_48      : 32;       //  Код Dдп (адрес 66)(адрес 65)(адрес 64) 
 u16 Kod_Ddp_16      : 16;       //  Код Dдп (адрес 69)(адрес 68)(адрес 67) 
 
 unsigned int Kod_Ddp_32      : 32;       //  Код Dдп (адрес 66)(адрес 65)(адрес 64) 

 /*

48 разрядный код девиации частоты гетеродинного сигнала на интервале “помехи”.
При этом   бит3 должен быть выставлен в  «1» – наличие/отсутствие ЛЧМ в сигнале «помехи».

 */


} Inf_packet;

   
  typedef struct                //Объявление структуры ответного пакета о состоянии блока
{
  u16 code_00h     : 16;  
/*
Основное назначение - техническое состояние аппаратуры шкафа 1У07ФВ.

D15 D14 D13 D12        D11        D10 D9  D8       D7  D6      D5  D4      D3  D2      D1  D0
Код ошибки             Норм.
                       Охл.      Прием (НИ)         ВЫКЛ         ГБР        БР         ЦУ/МУ
3р  2р  1р  0р                   Ком. М2  Ош       Исп.Зад.    Исп.Зад.    Исп.Зад.    Исп.Зад.

D15?D12 – в 1У08ПП не используются.
D11 – норма охлаждение - лог. «1» .
D10 – информация о принятой ошибке в команде,
 при лог. "0" в разряде D8 - трактуется как сбой обмена, при лог.  «1»- как ошибка.
D9 – информация о принятой ошибке по М2, при лог.
 "0" в разряде D8 -трактуется как сбой об-мена, при лог. «1»- как ошибка.
D7?D2 – квитанция состояния: код 11 - команда задана и исполнена, 
код 00 - нет задания и ис-полнения команды,
 код 01 - нет исполнения команды, код 10 - нет задания команды.
Dl?D0 - код 11 - ЦУ задан и исполнен,
 код 00 - МУ задан и исполнен, код 01 - МУ исполнен,
  ЦУ задан, код 10 - ЦУ исполнен, МУ задан.


*/

 u16 code_02h   :16;
 
 /*
Основное назначение - техническое состояние аппаратуры

D15     D14           D13 D12 D11 D10        D9  D8  D7  D6         D5    D4    D3   D2    D1  DO
Сост.ПИ              Норма осн. усилит.     Норма питания ФТ                 Авария блоков
Норм t  ПА             4   3   2   1          4   3   2  1         Б600  Б605  Б604  Б603  Б602  Б601

D15 – норма температуры инженерного оборудования 1У07ФИ (лог. «1»).
D14 – предавария инженерного оборудования 1У07ФИ (лог.  «1»).
D13?D10 – наличие единицы в одном из разрядов D10-D13,
 говорит о том, что работа идет по усилителю БР данного канала. 
 Разряд D10 соответствует 1 каналу, содержащему Ком.2, усили-тель БР и Ком.6 (см. рисунок 3.8).
  В случае неработоспособности усилителя БР (в соответст-вующем бите логический нуль) из канала,
   1У07ФТ переключается на усилитель ГБР 1 канала.
D9?D6 – наличие единицы говорит о норме питания источника ВИП в соответствующем кана-ле.
 При начальном включение (исполнение команды БР на одном из комплектов ФВ) какое-то время нули во всех разрядах.
D5?D0 – авариии блоков шкафа 1У07ФВ (лог.  «1») .


 */ 
 
  u16 code_04h   :16;
 
 /*
Основное назначение - техническое состояние аппаратуры

   D15 - D9        D8       D7 - D5                D3 - D0	
   Резерв      Норм ИВЭ    Резерв        Норма резервных усили-телей ФТ
 

D8 – норма ИВЭ шкафа 1У07ФВ - (лог.  «1»).
D3?D0 – разряд D0 соответствует каналу, содержащему Ком.2,
 Усил.1 ГБР и Ком.6 (см. рису-нок 3.8).
 Наличие нуля в данном разряде говорит о том, что возникла авария усилителя по дан-ному каналу,
 т.е. он вышел из строя и канал 1 работает на резервном усилителе ГБР. Анало-гично и для других разрядов.

Пример:
?	Если в разряде D10 слова 02h единица, а в разряде D0 слова 04h нуль, то данная ситуация говорит о том, что работает усилитель БР 1 канала. 
?	Если в разряде D10 слова 02h нуль, а в разряде D0 слова 04h единица, то данная ситуация говорит о том, что работает усилитель ГБР (усилитель БР – неисправен) 1 канала. 
?	Если в разряде D10 слова 02h нуль, а в разряде D0 слова 04h нуль, то данная ситуация говорит о том, что работают усилители БР и ГБР. В таком случае если в разряде D6 слова 02h нуль, то считается неисправным ВИП 1 канала. Если в разряде D6 слова 02h единица, то считаются неисправными коммутаторы 1 канала.

 */ 

 u8 code_05h   :8;
 u8 code_06h   :8;
 u8 code_07h   :8;
 u8 code_08h   :8;
 u8 code_09h   :8;

 
  /*
Кнц – двоично-десятичный код времени начала цикла в пределах 5 минут (см. Таблицу 3.4).

7р	6р	5р	4р	3р	2р	1р	0р
Кнц минут
X	X	X	X	X	2	1	0
7р	6р	5р	4р	3р	2р	1р	0р
Кнц десятков секунд	Кнц секунд
3	2	1	0	3	2	1	0

7р	6р	5р	4р	3р	2р	1р	0р
Кнц сотен мс	Кнц десятков мс
3	2	1	0	3	2	1	0
7р	6р	5р	4р	3р	2р	1р	0р
Кнц единиц мс	Кнц сотен мкс
3	2	1	0	3	2	1	0
7р	6р	5р	4р	3р	2р	1р	0р
Кнц десятков мкс	Кнц единиц мкс
3	2	1	0	3	2	1	0

Кнц минут – код количества минут в текущей пятиминутке.
Кнц десятков секунд – код количества десятков секунд в текущей минуте.
Кнц секунд – код количества секунд в текущей десятке.
Кнц сотен мс - код количества сотней мс в текущей секунде.
Кнц десятков мс - код количества десятков мс в текущей сотне.
Кнц единиц мс - код количества мс в текущей десятке.
Кнц сотен мкс - код количества сотней мс в текущей мс.
Кнц десятков мкс - код количества десятков мкс в текущей сотне.
Кнц единиц мкс- код количества мкс в текущей десятке.

 */ 

   u8 code_0Ah   :8;
   u8 code_0Bh   :8;
   u8 code_0Ch   :8;
 /*  
   Зарезервирован для местного управления и определяют код текущего времени (минуты, часы, сутки), установленный вручную.
 */
 
   u8 code_0Dh   :8;
   u8 code_0Eh   :8;
   
   /*
   Кфк – код функционального контроля
7р	6р	5р	4р	3р	2р	1р	0р
Кфк – (адрес 88)
7	6	5	4	3	2	1	X

0Dh


1 байт 88 обмена информацией (от 1У07ФВ на 1У09К):
Бит 0 – свободный ;
бит1- наличие нулевых интервалов Пии;
бит 2 – рассогласование сетки КТВ относительно Т5М; 
бит 3 – Тц   2 сек. – длительность цикла больше 2 сек.;
бит 4 – сформированные интервалы времени не соответствуют заданным;
бит 5 – обмен с 1УЯ203 начался, но не удовлетворяет заданным критериям;
бит 6 – единичный сбой формирования интервалов;
бит 7– единичный сбой обмена информацией.
7р	6р	5р	4р	3р	2р	1р	0р
Кфк – (адрес 89)
X	X	X	X	3	2	1	0

0Eh

1 байт 89 обмена информацией (от 1У07ФВ на 1У09К):
бит 0 - контроль ФАПЧ генератора fи (1У-К605);
бит 1 – контроль ФАПЧ генератора fоп. (1У-К603);
бит 2 – контроль сигнала СО сформированных законов модуляции (1У-К605, 1У-К606, 1У-К608);
бит 3 – контроль ФАПЧ генератора fг (1У-К606);
биты 4, 5, 6, 7 свободны.


   
   */

} Inf_sys_packet;

 
  typedef struct                //flags
{
  u8 byte1      : 8;        //  код замены  4-ре бита
  u8 byte2      : 8;        //  контрольное чтение ОЗУ 4-ре бита
  u8 byte3      : 8;        //  количество ошибок 2-ва бита
  u8 byte4      : 8;        //  номер проверяемой цепи 4-ре бита
  u8 byte5      : 8;        //  код ошибки 2-ва бита
  u8 byte6      : 8; 

  u8  VCU:1; // команда ВЦУ
  u8   MU:1; // команда МУ 
  
  u8  GBR:1; // команда ГБР
  u8   BR:1; // команда БР 
  u8 VYKL:2; // команда ВЫКЛ
  u8 Ohl :1; // индикатор Охлаждения
 
  
  
} Init_K615_struct;

 Inf_packet         Mem1 ; // объявляю структуру Mem1
 Inf_sys_packet     Mem_sys;
 Init_K615_struct   Mem_K615;  // объявляю структуру для кассеты К615
 
      	
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

//***********************переменные времени*********************************

unsigned char  C_t_day =0;  //текущие 
unsigned char  C_t_hour=0;
unsigned char  C_t_min =0;
unsigned char  C_t_sec =0;

unsigned short C_t_ms  =0;
unsigned int   C_t_us  =0;


unsigned char  C_t_dayX =0; //время стабатывания
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


  // reverse:  переворачиваем строку s на месте 
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

 // itoa_t:  конвертируем n в символы в s 
 void itoa_t(int n, char s[])
 {
     int i, sign;
 
     if ((sign = n) < 0)  // записываем знак 
         n = -n;          // делаем n положительным числом 
     i = 0;
     do {       // генерируем цифры в обратном порядке 
         s[i++] = n % 10 + '0';   // берем следующую цифру 
     } while ((n /= 10) > 0);     // удаляем 
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
	UART1->DR = c; //Передача данных
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
	UART2->DR = c; //Передача данных
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


 void Transf( char* s)  // процедура отправки строки символов в порт 
   {
       unsigned  short l=0;
       unsigned  short i=0;
         
	  // TX_485;
        sendT(s);	//uart1
       sendT2(s);   //uart2
	  // RX_485;
  }
   

 void ZTransf( char* s,unsigned char a )  // процедура отправки строки символов в порт 
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
 Transf ("Состояние синтезатора:\r\n" );
 
 Transf ("Режим работы синтезатора:" );
 
 if (DDS_data.regim==1) { Transf ("синтез ЛЧМ\r\n"); }
 if (DDS_data.regim==0) { Transf ("синтез гармоники\r\n"); }
  
 Transf ("\r\n");
  
 Transf ("частота центральная F0:" );
 sprintf(strng,"%d",DDS_data.F0);
 Transf(strng);
 Transf ("\r\n");
 
 Transf ("частота нижняя   F_low:" );
 sprintf(strng,"%d",DDS_data.F_low);
 Transf(strng);
 Transf ("\r\n");
 
 Transf ("частота верхняя F_high:" );
 sprintf(strng,"%d",DDS_data.F_high);
 Transf(strng);
 Transf ("\r\n");
 Transf ("\r\n");
 
 Transf ("Длительность импульсов в мкс:" );
 sprintf(strng,"%d",DDS_data.dImp);
 Transf(strng);
 Transf ("\r\n");
 Transf ("\r\n");
 
 Transf ("Частота следования импульсов в Гц:" );
 sprintf(strng,"%d",DDS_data.FImp);
 Transf(strng);
 Transf ("\r\n");
 Transf ("\r\n");
 
 Transf ("Амплитуда, подавление в Дб:" );
 sprintf(strng,"%d",DDS_data.A);
 Transf(strng);
 Transf ("\r\n");
 Transf ("\r\n");
 
 Transf ("направление пилы ЛЧМ:" );
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



void IO (char* sr_io)      // функция обработки протокола обмена
 
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
		time_uart=0;  //обнуление счётчика тайм аута
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
		
	} // обнаружено начало пакета

if (packet_flag==1)

{

	while (i>0)   //перегрузка принятого пакета в массив обработки
	
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
                               (data_flag!=1))	       Word[index_word]=InOut[index1]; // заполняем командное слово
                		
                   
                        	   if  ((data_flag==1)&&
                                 (InOut[index1]!=' ')&&
                                 (InOut[index1]!=';')&&
                                 (InOut[index1]!='=')&&
                                 (InOut[index1]!=':')&&(data_flag2==0))    	DATA_Word[index_data_word]=InOut[index1]; // заполняем  слово данных1
                      
                   
                             if  ((data_flag==1)&&
                                 (InOut[index1]!=' ')&&
                                 (InOut[index1]!=';')&&
                                 (InOut[index1]!='=')&&
                                 (InOut[index1]!=':')&&(data_flag2==1))     DATA_Word2[index_data_word2]=InOut[index1]; // заполняем  слово данных2
                		      
                	   
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
	
	    if (InOut[0]==0x7e)   crc_ok=crc_ok|0x1;   // проверка первого условия пакета - начало пакета
	  	if (InOut[1]==Adress) crc_ok=crc_ok|0x2;   // проверка второго условия пакета - адресат назначения
	
	if (crc_ok==0x3)  //обработка команд адресатом которых является хозяин 
    	{
		
					if (strcmp(Word,"help")==0)        				      

					  { Transf ("принял help\r"    ); 
						  Menu1('t');	   }
						  
					if (strcmp(Word,"help2")==0)        				      

					  { Transf ("принял help2\r"    ); 
						  Menu1('t');	   }
                  		
                  //		if (strcmp(Word,  "сигнал_авария_ON" )==0)      { Transf ("принял сигнал_авария_ON\r"  ); AVARIYA_on ;}
                  //		if (strcmp(Word,  "сигнал_авария_OFF")==0)      { Transf ("принял сигнал_авария_OFF\r"  ); AVARIYA_off ;}

                   	  if (strcmp(Word,"TEST_BUS_OUT"  )==0)        { Transf ("принял TEST_BUS_OUT\r"  );  Test_bus_out_FLAG=1;       }//проверка выхода параллельной шины данных	
                      if (strcmp(Word,"watch_reg_dds"  )==0)       { Transf ("принял watch_reg_dds\r"  ); reg_DDS_watch (&pins_dds); }//чтение и вывод на экран регистров DDS 
					 					 
					  if (strcmp(Word,"ADC"      )==0) 	    //измерить напряжение в канале //проверка ADC микроконтроллера
                  				{ 	dFo1=atoi(DATA_Word);
            									Transf ("принял ADC:" );
                              					Transf("DATA_Word:");
                              					ZTransf (DATA_Word,index_data_word-1);
            									Transf("\r");
            									ADC_test(dFo1);
            								 }
									 
                      if (strcmp(Word,"wrn_dds_1"      )==0)       { Transf ("принял wrn_dds_1\r"      ); PORTD->RXTX|= (1<<15); } // 
                      if (strcmp(Word,"wrn_dds_0"      )==0)       { Transf ("принял wrn_dds_0\r"      ); PORTD->RXTX&=~(1<<15); } // 
                      if (strcmp(Word,"init_dds"       )==0)       { Transf ("принял init_dds \r"       ); init_DDS (&DDS_data,&pins_dds); }         //инициализация DDS 
                      if (strcmp(Word,"id_dds"         )==0)       {   //чтение данных с dds
                                                                            Transf ("принял id_dds:" );
                                                                     temp_short=Read_DDS(0x0001,&pins_dds); 
                                                                                itoa_t(temp_short,strng);
                                                                                         Transf(strng);
                                                                                         Transf ("\r");
                                                                                                       }
                    
                      if (strcmp(Word,"rd_dds"    )==0)      {   //чтение данных с dds
                                                                        Transf ("принял rd_dds:" );
                                                                 temp_short=Read_DDS(0x1020,&pins_dds); 
                                                                            itoa_t(temp_short,strng);
                                                                                     Transf(strng);
                                                                                     Transf ("\r");
                                                                                                   }
                    
                      if (strcmp(Word,"wr_dds"      )==0)      //установить данные на dds
                          { temp_short=atoi(DATA_Word);
                            Transf ("принял wr_dds: \r" );
                            Transf("DATA_Word:");
                            ZTransf (DATA_Word,index_data_word-1);
                            Transf("\r");
                            Write_DDS(temp_short,0x1020,&pins_dds);
                             }

                      if (strcmp(Word,"rd_tst"    )==0)       {   //чтение данных с dds
                                                      Transf ("принял rd_tst:" );
                                               temp_short=rd_tst(0x1020,&pins_dds); 
                                                          itoa_t(temp_short,strng);
                                                                   Transf(strng);
                                                                   Transf ("\r");
                                                                                 }
                    
                      if (strcmp(Word,"wr_tst"      )==0)      //установить данные на dds
                          { temp_short=atoi(DATA_Word);
                            Transf ("принял wr_tst: \r" );
                            Transf("DATA_Word:");
                            ZTransf (DATA_Word,index_data_word-1);
                            Transf("\r");
                            wr_tst(temp_short,0x1020,&pins_dds);
                             }
							 
                      if (strcmp(Word,"Установить_Амплитуду"      )==0) 	    //установить частоту F0 синтезатора
                  				{ 	dFo1=atoi(DATA_Word);
								              Transf ("принял Установить_Амплитуду: \r" );
										          DDS_data.A= dFo1;
										            sprintf(strng,"%d",dFo1);
													Transf(strng);
													Transf("\r");
              							     amplitud(&DDS_data,&pins_dds);  //вызов функции расчёта и программирования DDS
										              TX_485();
										            Transf("~1 COMMAND_OK;");
                  					 }    
                 	    if (strcmp(Word,"Индикация_состояния_синтезатора")==0)      { Transf ("принял Индикация_состояния_синтезатора\r\n"   );   DDS_setup_ind();	     }		   
                      if (strcmp(Word,"Включить_синтезатор"		     )==0)      { Transf ("принял Включить_синтезатор\r"        		 );         			     }
                  		if (strcmp(Word,"Включить_ЛЧМ_UP"    			 )==0)      { Transf ("принял Включить_ЛЧМ_UP    \r"       		     );   DDS_data.znak_LCHM= 1;   }
                  		if (strcmp(Word,"Включить_ЛЧМ_DOWN"  			 )==0)      { Transf ("принял Включить_ЛЧМ_DOWN  \r"         		 );   DDS_data.znak_LCHM=-1;   }
                  		if (strcmp(Word,"Установить_F0"     			 )==0) 	    //установить частоту F0 синтезатора
                  				{ 	dFo1=atoi(DATA_Word);
								            	Transf ("принял Установить_F0: \r" );
                  					   Transf("DATA_Word:");
                  					   ZTransf (DATA_Word,index_data_word-1);
              									Transf("\r");
              									DDS_data.F0=dFo1;
              									DDS_data.regim=0;
												init_DDS (&DDS_data,&pins_dds); //инициализация DDS - сброс регистров в НОЛЬ
              									Fs_calc(&DDS_data,&pins_dds);  //вызов функции расчёта и программирования DDS
            										TX_485();
            										Transf("~1 COMMAND_OK;");
                  					 }
                  		
						if (strcmp(Word,"Установить_ЛЧМ"      )==0) 	    //установить ЛЧМ 
                  				{ 	dFo1=atoi(DATA_Word);
              									Transf ("принял Установить_ЛЧМ: \r" );
                                					Transf("DATA_Word:");
                                					ZTransf (DATA_Word,index_data_word-1);
              									Transf("\r");
              									DDS_data.regim=1;
												init_DDS (&DDS_data,&pins_dds); //инициализация DDS - сброс регистров в НОЛЬ
              									Set_LCHM(&DDS_data,&pins_dds);  //вызов функции расчёта и программирования DDS
												TX_485();
										Transf("~1 COMMAND_OK;");
                  					 }
						if (strcmp(Word,"Установить_девиацию_ЛЧМ" )==0) 	   //установить частоту F нижнее для ЛЧМ синтезатора в Гц
                  				{ dFo1=atoi(DATA_Word);
                  					Transf ("принял Установить_девиацию_ЛЧМ: \r" );
                  					Transf("DATA_Word:");
                  					ZTransf (DATA_Word,index_data_word-1);
                  					Transf("\r");
            									DDS_data.F_low =DDS_data.F0-dFo1/2;
            									DDS_data.F_high=DDS_data.F0+dFo1/2;
												TX_485();
											Transf("~1 COMMAND_OK;");
        						}	
						
                      if (strcmp(Word,"Установить_частоту_F_low" )==0) 	   //установить частоту F нижнее для ЛЧМ синтезатора в Гц
                  				{ dFo1=atoi(DATA_Word);
                  					Transf ("принял Установить_частоту_F_low: \r" );
                  					Transf("DATA_Word:");
                  					ZTransf (DATA_Word,index_data_word-1);
                  					Transf("\r");
								          	DDS_data.F_low=dFo1;
											TX_485();
										Transf("~1 COMMAND_OK;");
									}	
                  		
                      if (strcmp(Word,"Установить_частоту_F_high" )==0) 	   //установить частоту F верхнее для ЛЧМ синтезатора в Гц
                  				{ dFo1=atoi(DATA_Word);
                  					Transf ("принял Установить_частоту_F_high: \r" );
                  					Transf("DATA_Word:");
                  					ZTransf (DATA_Word,index_data_word-1);
                  					Transf("\r");
									          DDS_data.F_high=dFo1;
									TX_485();
									Transf("~1 COMMAND_OK;");
								 }		
                   if (strcmp(Word,"Длительность_импульса_Tимп" )==0) 	   //установить длительность импульса в мкс
                  				{ dFo1=atoi(DATA_Word);
                  					Transf ("принял Длительность_импульса_Tимп: \r" );
                  					Transf("DATA_Word:");
                  					ZTransf (DATA_Word,index_data_word-1);
                  					Transf("\r");
									DDS_data.dImp=dFo1;
									TX_485();
									Transf("~1 COMMAND_OK;");
									}	
                  		
					if (strcmp(Word,"Частота_повторения_Fимп" )==0) 	   //установить длительность импульса в Гц
                  				{ dFo1=atoi(DATA_Word);
                  					Transf ("принял Частота_повторения_Fимп: \r" );
                  					Transf("DATA_Word:");
                  					ZTransf (DATA_Word,index_data_word-1);
                  					Transf("\r");
									DDS_data.FImp=dFo1;
									TX_485();
									Transf("~1 COMMAND_OK;");
									}	
					
                  		
                  //--------------------------------------------------K612----------------------------------------------------------------				
                  		
                  		if (strcmp(Word,"Вкл_синтез_K612")==0)
                  		{ 
                    	if (Master_flag==1)	Transf ("принял Включить_синтезатор K612  \r"  );
                  		Flag_init_K612=1;
                  		Flag_K612=0;
                  		tick_process=0;
                  		sch_obmen=0;
                  		tick_process_K612=0;
                  		} //принимаем подтверждение на ранее отосланную команду инициализации
                  		
                  			
                  		if (strcmp(Word,"состояние_K612")==0)
                  		{ 
                                             
                             sys_char=atoi(DATA_Word);
                     
                  	    
                        if (Master_flag==1)  
                          {
                              Transf ("принял состояние K612:"  );
                              itoa_t(sys_char,s3);
                              Transf(s3);
                              Transf("\r\n");

                              if ((sys_char&0x4)==0x4)  Transf ("TEMP-OK\r"  ); 
                              if ((sys_char&0x2)==0x2)  Transf ("Voltag-OK\r"); 
                              if ((sys_char&0x1)==0x1)  Transf ("Level_F-OK\r"); 
                          }
                                               

                      //  Transf ("принял состояние K612\r\n"  );

                        sys_life_k612=sys_char;    //переменная хранить состояние кассеты 612 0х1 - мощность выходного сигнала 0х2 - значения внутренних напряжений 0х4 - температура кассеты

                       if (sys_life_k612&0x4) Mem_K615.Ohl=1; else Mem_K615.Ohl=0; // индикация охлаждения блоков

                  		Flag_control_K612=1;
                  		Flag_K612=0;
                  		tick_process=0;
                  		sch_obmen=0;
                  	    tick_process_K612=0;

                  		} //принимаем подтверждение на ранее отосланную команду инициализации
                  		
                  		
                  //--------------------------------------------------K613----------------------------------------------------------------				
                  		
                  		if (strcmp(Word,"Вкл_синтез_K613")==0)
                  		{ 
                      
                      if (Master_flag==1) 	Transf ("принял Включить_синтезатор K613\r"  );

                  		Flag_init_K613=1;
                  		Flag_K613=0;
                  		tick_process=0;
                  		sch_obmen=0;
                        tick_process_K613=0;

                  		} //принимаем подтверждение на ранее отосланную команду инициализации
                  		
                  				
                  		if (strcmp(Word,"состояние_K613")==0)
                  		{ 

                      	  sys_char=atoi(DATA_Word);

                      // Transf ("принял состояние K613\r"  );

                       
                        if (Master_flag==1) 
                                {
                                          Transf ("принял состояние K613:"  );       
                                          itoa_t(sys_char,s3);
                                          Transf(s3);
                                          Transf("\r");
                                }  

                          sys_life_k613=sys_char;   //переменная хранить состояние кассеты 613 

                  		Flag_control_K613=1;
                  		Flag_K613=0;
                  		tick_process=0;
                  		sch_obmen=0;
                  	    tick_process_K613=0;
                  		} //принимаем подтверждение на ранее отосланную команду инициализации


                  //------------------------------------------------------------------------------------------------------------------		
		} 
	
	}
	
	if ((packet_ok==1)&&(crc_ok==0x1))     //обработка команд адресатом которых является слейв

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
			
			time_uart=0;  //обнуление счётчика тайм аута
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
                   {; // не должно бы такое быть
                    return 1;
                   }
              }
  
       else  
       	                    return 1;       
   
      return 0;
    }


void UART_control (void)

{

	if ((BufIsRead_flag==1)) // проверка пришёл ли пакет по уарту и его обработка &&(flag_pachka_sinhron==0)
           
     {  
      getStr(sr,&lsr);
      BufIsRead_flag=0;
      IO(sr);
	  RX_485();
	 }	
}


void BUS_wr (unsigned short a)
{

	if (a&(1<<0)) PORTF->SETTX = (1<<8); //устанавливаем еденицу PF[8], бит 0 
    else		  PORTF->CLRTX = (1<<8); //устанавливаем ноль PF[8], бит 0 

    if (a&(1<<1)) PORTF->SETTX = (1<<9); //устанавливаем еденицу PF[8], бит 0 
    else		  PORTF->CLRTX = (1<<9); //устанавливаем ноль PF[8], бит 0 

    if (a&(1<<2)) PORTF->SETTX = (1<<10); //устанавливаем еденицу PF[8], бит 0 
    else		  PORTF->CLRTX = (1<<10); //устанавливаем ноль PF[8], бит 0 

    if (a&(1<<3)) PORTF->SETTX = (1<<11); //устанавливаем еденицу PF[8], бит 0 
    else		  PORTF->CLRTX = (1<<11); //устанавливаем ноль PF[8], бит 0 

    if (a&(1<<4)) PORTF->SETTX = (1<<12); //устанавливаем еденицу PF[8], бит 0 
    else		  PORTF->CLRTX = (1<<12); //устанавливаем ноль PF[8], бит 0

    if (a&(1<<5)) PORTF->SETTX = (1<<13); //устанавливаем еденицу PF[8], бит 0 
    else		  PORTF->CLRTX = (1<<13); //устанавливаем ноль PF[8], бит 0  

    if (a&(1<<6)) PORTF->SETTX = (1<<14); //устанавливаем еденицу PF[8], бит 0 
    else		  PORTF->CLRTX = (1<<14); //устанавливаем ноль PF[8], бит 0 

    if (a&(1<<7)) PORTF->SETTX = (1<<15); //устанавливаем еденицу PF[8], бит 0 
    else		  PORTF->CLRTX = (1<<15); //устанавливаем ноль PF[8], бит 0 

    if (a&(1<<0x9)) PORTD->SETTX = (1<<9); //устанавливаем еденицу PF[8], бит 0 
    else		    PORTD->CLRTX = (1<<9); //устанавливаем ноль PF[8], бит 0 

    if (a&(1<<0xa)) PORTC->SETTX = (1<<2); //устанавливаем еденицу PF[8], бит 0 
    else		    PORTC->CLRTX = (1<<2); //устанавливаем ноль PF[8], бит 0 

    if (a&(1<<0xb)) PORTD->SETTX = (1<<10); //устанавливаем еденицу PF[8], бит 0 
    else		    PORTD->CLRTX = (1<<10); //устанавливаем ноль PF[8], бит 0 

    if (a&(1<<0xc)) PORTD->SETTX = (1<<11); //устанавливаем еденицу PF[8], бит 0 
    else		    PORTD->CLRTX = (1<<11); //устанавливаем ноль PF[8], бит 0 

    if (a&(1<<0xd)) PORTD->SETTX = (1<<12); //устанавливаем еденицу PF[8], бит 0 
    else		    PORTD->CLRTX = (1<<12); //устанавливаем ноль PF[8], бит 0 

    if (a&(1<<0xe)) PORTD->SETTX = (1<<13); //устанавливаем еденицу PF[8], бит 0 
    else		    PORTD->CLRTX = (1<<13); //устанавливаем ноль PF[8], бит 0 

    if (a&(1<<0xf)) PORTD->SETTX = (1<<14); //устанавливаем еденицу PF[8], бит 0 
    else		    PORTD->CLRTX = (1<<14); //устанавливаем ноль PF[8], бит 0 

}


void UART1_Handler()
{
	unsigned stat;
    unsigned t;
	uint32_t value;
	unsigned char letter[1];

	UART1->ICR=0x10;		//reset Rx interrupt	
	letter[0]=(UART1->DR);
	//BufIsRead_flag=1; //флаг пришедшего байта
	
	                    //  Receive data ready.
            { 
                                                      //  добавить здесь: переполн. входн. буфера
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
                     BufIsRead_flag=1;      // все введ. данные уже считаны
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
	//BufIsRead_flag=1; //флаг пришедшего байта
	
	                    //  Receive data ready.
            { 
                                                      //  добавить здесь: переполн. входн. буфера
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
                     BufIsRead_flag=1;      // все введ. данные уже считаны
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

// *** Функции, вызываемые драйверами протоколов при получении пакетов протокола.
// Можно использовать только OnUDPReceive, для того, чтобы получить данные, переданные нам по протоколу UDP.
// При желании также можно анализировать данные других пакетов, но это не желательно, т.к. может нарушить даннные
// и привести к сбою штатной обработки пакетов драйверами протоколов.
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
		IO("~3 Установить_F0=435000000;");
	
		IO("~3 Установить_девиацию_ЛЧМ=10450000;");
	
		IO("~3 Частота_повторения_Fимп=1000;");
	
		IO("~3 Длительность_импульса_Tимп=128;");
	
		IO("~3 Установить_ЛЧМ;");

		IO("~3 Установить_Амплитуду:3;");
	
		//IO("~3 watch_reg_dds;");
	}
	
	if (Adress==0x34)
	{
		IO("~4 Установить_F0=389000000;");
	
		IO("~4 Установить_девиацию_ЛЧМ=10450000;");
	
		IO("~4 Частота_повторения_Fимп=1000;");
	
		IO("~4 Длительность_импульса_Tимп=128;");
	
		IO("~4 Установить_ЛЧМ;");

		IO("~4 Установить_Амплитуду:8;");
	
		//IO("~4 watch_reg_dds;");
	}
	RX_485();
}

void INIT_sintez2(void)
{

	if (Adress==0x33)
	{
		IO("~3 Установить_F0=435000000;");
	
		IO("~3 Установить_девиацию_ЛЧМ=10250000;");
	
		IO("~3 Частота_повторения_Fимп=1;");
	
		IO("~3 Длительность_импульса_Tимп=600000;");
	
		IO("~3 Установить_ЛЧМ;");

		IO("~3 Установить_Амплитуду:4;");
	
		//IO("~3 watch_reg_dds;");
	}
	
	if (Adress==0x34)
	{
		IO("~4 Установить_F0=389000000;");
	
		IO("~4 Установить_девиацию_ЛЧМ=10250000;");
	
		IO("~4 Частота_повторения_Fимп=1;");
	
		IO("~4 Длительность_импульса_Tимп=600000;");
	
		IO("~4 Установить_ЛЧМ;");

		IO("~4 Установить_Амплитуду:8;");
	
		//IO("~4 watch_reg_dds;");
	}
	RX_485();
}

void OnICMPReceive (void* pICMPData, uint16_t Size) {}
void SendEthPacket (void* pHeader, uint16_t Size) {}

 unsigned int FS = 1000000000; //частота DDS Hz
 
 unsigned int INIT_SINTEZ_delay = 150000u;
 #define led_delay 70000u
 #define DELAY_SHOW 180000000u  // 20000000u - 20 секунд

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

  //вспомогательные ноги для DDS синтезатора ТНИ и ТНП
     PORTC->RXTX|=(1<<9)|(1<<11); //выставляем еденицы
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
	} //частота гетеродина в кассете гетеродина
	
	if (Adress==0x33) 
	{
		INIT_SINTEZ_delay = 10000000u;
		Fget_define = 500000000;
		FS = 1000000000;
	} //частота гетеродина в кассете синтезатора
	

			
	init_DDS (&DDS_data,&pins_dds); //инициализация DDS
				
	
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
