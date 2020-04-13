#include "MDR32F9Qx_port.h"
#include <MDR32Fx.h>
#include "MDR32F9Qx_config.h"
#include "MDR32F9Qx_uart.h"
#include "MDR32F9Qx_rst_clk.h"
#include "MDR32F9Qx_it.h"
#include "MDR32F9Qx_ssp.h"


#include <math.h>
#include <stdio.h>

#include "spec_1508pl10.h"

#define PA2_1 PORT_SetBits  (MDR_PORTA, PORT_Pin_2)
#define PA2_0 PORT_ResetBits(MDR_PORTA, PORT_Pin_2)

#define RE_485_1 PA2_1 
#define RE_485_0 PA2_0 

#define PC0_1 PORT_SetBits  (MDR_PORTC, PORT_Pin_0)
#define PC0_0 PORT_ResetBits(MDR_PORTC, PORT_Pin_0)

#define DE_485_1 PC0_1 
#define DE_485_0 PC0_0 

#define PB8_1 PORT_SetBits  (MDR_PORTB, PORT_Pin_8)
#define PB8_0 PORT_ResetBits(MDR_PORTB, PORT_Pin_8)

#define PWR_OFF PB8_1 
#define PWR_ON  PB8_0 

#define PC1_1 PORT_SetBits  (MDR_PORTC, PORT_Pin_1)
#define PC1_0 PORT_ResetBits(MDR_PORTC, PORT_Pin_1)

#define ZAHVAT_OFF PC1_1 
#define ZAHVAT_ON  PC1_0 

#define PA6_1  PORT_SetBits  (MDR_PORTA, PORT_Pin_6)
#define PA6_0  PORT_ResetBits(MDR_PORTA, PORT_Pin_6)

#define LED1_OFF PA6_1 
#define LED1_ON  PA6_0 

#define PA7_1  PORT_SetBits  (MDR_PORTA, PORT_Pin_7)
#define PA7_0  PORT_ResetBits(MDR_PORTA, PORT_Pin_7)

#define LED2_OFF PA7_1 
#define LED2_ON  PA7_0 


//===========================================================================================================

static PORT_InitTypeDef PortInit;
static UART_InitTypeDef UART_InitStructure;
static SSP_InitTypeDef sSSP;

//SPI


uint8_t TxIdx = 0, RxIdx = 0;

// UART

 unsigned char DataTr_flag=0;
 unsigned char Data_in_port=0;
 unsigned char DontConnect_flag=0;      
 unsigned      CntDown = 0;             
 unsigned char CyclEnd_flag=0;          
                                        
 unsigned char RcvBufOverflow_flag=0;    
                                        
 unsigned char BufIsRead_flag=0;       
 unsigned char Data4TrAbsent_flag=1;    
#define buf_size 256
 unsigned char TrBuf[buf_size];
 unsigned char RcvBuf[buf_size];
 unsigned char sch_buf=0;

                                         
                                       
 unsigned char *pTr_Buf;                
 unsigned char *pcur_Tr;                
 unsigned char *pRcv_Buf;           
 unsigned char *pcur_Rcv;

 void fillBuf  (         char* ptrBuf, unsigned short lengthBuf, unsigned char symb );
 void clearBuf (         char* ptrBuf, unsigned short lengthBuf );
 void zputs    (              char* s, unsigned char s_size );
  int getStr   (              char* s, unsigned char* s_size );

  //  UART

  uint32_t uart1_IT_TX_flag=RESET;
  uint32_t uart1_IT_RX_flag=RESET;

  uint32_t uart2_IT_TX_flag=RESET;
  uint32_t uart2_IT_RX_flag=RESET;


#define Bufer_size  64     //16384

volatile  unsigned int  text_lengh;


unsigned int j_pack=0;
unsigned char packet[4][32];
unsigned char flag_clear=0;
 char ok[5];
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

 char s1[Bufer_size];
unsigned l1= 5;
 char s2[Bufer_size];
unsigned l2=6;
 char s3[Bufer_size];
unsigned l3=7;
 char sr[Bufer_size];
static  char lsr=0;
unsigned char lk;
unsigned int dFo1;

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
#define  CPUacceler          1     


//*************************************************************************
       unsigned char  k;
        char  strng[Bufer_size];

 volatile unsigned    char lsym;
 volatile unsigned    char  sym;
 volatile unsigned    char flag;
 volatile unsigned    char packet_flag;
 volatile unsigned      char  NB;

  unsigned    char Adress=0x36;       
  unsigned    char Master_flag=0x0;   

  static volatile   unsigned    char packet_sum;
  static volatile   unsigned    char crc,comanda=1;
  static volatile   unsigned    char     InOut[Bufer_size];
 
  static volatile    char      Word [Bufer_size];     
  static volatile    char DATA_Word [Bufer_size];    
  static volatile    char DATA_Word2[Bufer_size];     
  static volatile unsigned    char crc_ok;
  static volatile unsigned    char packet_ok;
  unsigned    char ink1;  
  unsigned    char data_in;
  static volatile unsigned    char index_word=0;
  static volatile unsigned    char index_data_word =0;
  static volatile unsigned    char index_data_word2=0;
  static volatile unsigned    char data_flag=0;
  static volatile unsigned    char data_flag2=0;
  unsigned    char crc_input; 
  unsigned    char crc_comp;  
  unsigned    char sch_obmen=0;
  unsigned    char   Process_code=0;
  unsigned    char   Test_f_mono=0;

volatile  unsigned int   sch_avariya=0;
  
               
 unsigned volatile int  time_uart;  
 unsigned volatile int  tick_wait;
      
    unsigned  char UDP_TCP_flag=0;
    
                     unsigned char CRC_m[3][6];  
                     unsigned char K615_indik=0;  
              volatile       unsigned char K615_crc_sch=0;   
              volatile       unsigned char K615_crc_sch2=0;  
      
          unsigned  char Error_ethernet_obmen=0;  
 volatile unsigned  char flag_pachka_sinhron;  

 volatile  unsigned  char flag_pachka_TXT; // 
 volatile  unsigned  char flag_pachka_TXT2; // 

 volatile  unsigned  int sch_pachek_test=0;


  unsigned volatile char          flag_K615_event; 

 volatile   unsigned char flag_cikl_ON=0;
 volatile   unsigned char flag_Ethernet_packet_rcv=0;
 volatile            int sch_UDP_pakets=0;

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
			unsigned char  otvet_flag=0;
			unsigned char  SYS_LIFE=0;
			


 //*************************************************************************    
 
 unsigned volatile  char index1  =0;
 unsigned volatile char lsym1    =0;
 unsigned volatile char pack_ok1 =0;
 unsigned volatile char pack_sum1=0;   
 unsigned volatile char sym1     =0;  
  
 volatile unsigned char               Flag_K611=0;
 volatile unsigned char          Flag_init_K611=0;
 volatile unsigned char       Flag_control_K611=0;
 volatile unsigned char   Flag_control_sig_K611=0;
 volatile unsigned char   Flag_control_end_K611=0;
 volatile unsigned char    Flag_zahvat_sig_K611=0;
 volatile  unsigned char   Flag_zahvat_end_K611=0;
 volatile unsigned char              Qwant_K611=0;
 volatile unsigned char     Flag_zahvat_OK_K611=0;
 volatile unsigned char     Flag_signal_OK_K611=0;


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

  
 unsigned volatile char flag_Ethernet;  
       unsigned volatile char flag_Ethernet_Terminal=0; 

     
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
			   unsigned char FLAG_INIT_n_FOCH=0; //флаг этапа инициализации ФОЧ	
			   unsigned char FLAG_LED_n=0; //флаг стадии зажигания светодиодов			   
  //------------------------------------------------------------------

 



reg_1508pl10 reg_FAPCH1={ 
              {'Ф','А','П','Ч','1'},  
              0,//Тестовый режим.Включение режима при применении не допускается.
							0,//Обход усилителя кварцевого осциллятора 0 – стандартный режим 1 – обход
							0,//Тестовый режим.Включение режима при применении не допускается.
							0,//Тестовый режим.Включение режима при применении не допускается.
							0,//Тестовый режим.Включение режима при применении не допускается.
							0,//Разряд управления «Z–состоянием» фазового детектора. 0 – режим выключен 1 – режим включен
						  0x0,//Соответствие кода FD1, FD0 и ширины импульса «мертвой зоны»: 00 - 17нс|01 - 29 | 10 - 52| 11 - 100 нс
						    0,//Разряд управления функцией вывода Fdiv2(14) 0 – вывод работает как выход 1 – вывод работает как вход
						    0,// Разряд управления полярностью выводов ФД. 0 – прямая полярность 1 – обратная полярность
						    0,// Разряд управления типом выводов. 0 – открытый сток (см. назначение выводов) 1 – КМОП выход
						    0,// Разряд управления функцией вывода Fdiv3(9). 0 – вывод работает как выход 1 – вывод работает как вход
						    1,// Разряд управления источником тока. 0 – источник тока на выводах ФД выключен 1 – источник тока на выводах ФД включен. В этом режиме ФД имеет один выход, а к выходу Fd2 (Down) должен быть подключен внешний токозадающий резистор
						    0,//Коэффициенты деления опорного канала 00 - Kdiv1 = 10 ; 01 - Kdiv1 = 80 ; 10 - Kdiv1 = 100; 11 - Kdiv1 = 125   
						    0,//
						    0,// 000 - Kdiv2 = 160; 001 - Kdiv2 = 80; 010 - Kdiv2 = 40; 011 - Kdiv2 = 20; 100 - Kdiv2 = 200; 101 - Kdiv2 = 100;    
						    0,//
						    0,//
						    0,//  Коэффициенты деления основного канала старшие биты
						  240 //  Коэффициенты деления основного канала младшие биты

                          }; 

 reg_1508pl10 reg_FAPCH2={ 
              {'Ф','А','П','Ч','2'},     
              0,//Тестовый режим.Включение режима при применении не допускается.
							0,//Обход усилителя кварцевого осциллятора 0 – стандартный режим 1 – обход
							0,//Тестовый режим.Включение режима при применении не допускается.
							0,//Тестовый режим.Включение режима при применении не допускается.
							0,//Тестовый режим.Включение режима при применении не допускается.
							0,//Разряд управления «Z–состоянием» фазового детектора. 0 – режим выключен 1 – режим включен
						  0x0,//Соответствие кода FD1, FD0 и ширины импульса «мертвой зоны»: 00 - 17нс|01 - 29 | 10 - 52| 11 - 100 нс
						    0,//Разряд управления функцией вывода Fdiv2(14) 0 – вывод работает как выход 1 – вывод работает как вход
						    0,// Разряд управления полярностью выводов ФД. 0 – прямая полярность 1 – обратная полярность
						    0,// Разряд управления типом выводов. 0 – открытый сток (см. назначение выводов) 1 – КМОП выход
						    0,// Разряд управления функцией вывода Fdiv3(9). 0 – вывод работает как выход 1 – вывод работает как вход
						    1,// Разряд управления источником тока. 0 – источник тока на выводах ФД выключен 1 – источник тока на выводах ФД включен. В этом режиме ФД имеет один выход, а к выходу Fd2 (Down) должен быть подключен внешний токозадающий резистор
						    0,//Коэффициенты деления опорного канала 00 - Kdiv1 = 10 ; 01 - Kdiv1 = 80 ; 10 - Kdiv1 = 100; 11 - Kdiv1 = 125   
						    0,//
						    0,// 000 - Kdiv2 = 160; 001 - Kdiv2 = 80; 010 - Kdiv2 = 40; 011 - Kdiv2 = 20; 100 - Kdiv2 = 200; 101 - Kdiv2 = 100;    
						    0,//
						    0,//
						    0,//  Коэффициенты деления основного канала старшие биты
						  240 //  Коэффициенты деления основного канала младшие биты

                          };    

//------------------------------------
void Uart1PinCfg(void)
{
  /* Fill PortInit structure*/
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_DRIVER;
    PortInit.PORT_GFEN = PORT_GFEN_OFF;
    PortInit.PORT_FUNC = PORT_FUNC_ALTER;
    PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
    PortInit.PORT_MODE = PORT_MODE_DIGITAL;
    /* Configure PORTB pins 6 (UART1_RX) as input */
    PortInit.PORT_OE = PORT_OE_IN;
    PortInit.PORT_Pin = PORT_Pin_6;
    PORT_Init(MDR_PORTB, &PortInit);
    /* Configure PORTB pins 5 (UART1_TX) as output */
    PortInit.PORT_OE = PORT_OE_OUT;
    PortInit.PORT_Pin = PORT_Pin_5;
    PORT_Init(MDR_PORTB, &PortInit);  
}

void Uart2PinCfg(void)
{
	
	  /* Fill PortInit structure*/
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_DRIVER;
    PortInit.PORT_GFEN = PORT_GFEN_OFF;
    PortInit.PORT_FUNC = PORT_FUNC_ALTER;
    PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
    PortInit.PORT_MODE = PORT_MODE_DIGITAL;
    /* Configure PORTD pins 0 (UART2_RX) as input */
    PortInit.PORT_OE = PORT_OE_IN;
    PortInit.PORT_Pin = PORT_Pin_0;
    PORT_Init(MDR_PORTD, &PortInit);
    /* Configure PORTD pins 1 (UART2_TX) as output */
    PortInit.PORT_OE = PORT_OE_OUT;
    PortInit.PORT_Pin = PORT_Pin_1;
    PORT_Init(MDR_PORTD, &PortInit); 


}
void MltPinCfg (void)
{


  /* Fill PortInit structure*/
    PortInit.PORT_PULL_UP   = PORT_PULL_UP_ON;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
    PortInit.PORT_PD_SHM    = PORT_PD_SHM_OFF;
    PortInit.PORT_PD        = PORT_PD_DRIVER;
    PortInit.PORT_GFEN      = PORT_GFEN_OFF;
	
  /* Configure PORTA pins 2,5 for mlt out data  */
  PortInit.PORT_Pin   = (PORT_Pin_2 |  PORT_Pin_5);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
  PORT_Init(MDR_PORTA, &PortInit);
 /* Configure PORTA pins 3 for mlt input  */
  PortInit.PORT_Pin   = (PORT_Pin_3);
  PortInit.PORT_OE    = PORT_OE_IN;
  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
  PORT_Init(MDR_PORTA, &PortInit);

  /* Configure PORTB pins 9,10 for mlt output  */
  PortInit.PORT_Pin   = (PORT_Pin_8|PORT_Pin_9 | PORT_Pin_10);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
  PORT_Init(MDR_PORTB, &PortInit);
   /* Configure PORTB pins 7 for mlt input */
  PortInit.PORT_Pin   = (PORT_Pin_7 );
  PortInit.PORT_OE    = PORT_OE_IN;
  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
    PORT_Init(MDR_PORTB, &PortInit);

  /* Configure PORTC pins 0,2 for mlt output */
  PortInit.PORT_Pin   = (PORT_Pin_0|PORT_Pin_1);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
  PORT_Init(MDR_PORTC, &PortInit);
  
   /* Configure PORTC pins 2 for mlt input */
  PortInit.PORT_Pin   = (PORT_Pin_2);
  PortInit.PORT_OE    = PORT_OE_IN;
  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
  PORT_Init(MDR_PORTC, &PortInit);
 
 
    /* Configure PORTE pins 6 for mlt output */
  PortInit.PORT_Pin   = (PORT_Pin_6);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
  PORT_Init(MDR_PORTE, &PortInit);
   /* Configure PORTE pins 0,1,2,3,7 for mlt input */
  PortInit.PORT_Pin   = (PORT_Pin_0|PORT_Pin_1|PORT_Pin_2|PORT_Pin_3|PORT_Pin_7);
  PortInit.PORT_OE    = PORT_OE_IN;
  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
  PORT_Init(MDR_PORTE, &PortInit);

}

void LedPinGfg (void)
{
  /* Configure PORTA pins 6,7 for output to switch LEDs on/off */
  PortInit.PORT_Pin   = (PORT_Pin_6 | PORT_Pin_7);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_SLOW;

  PORT_Init(MDR_PORTA, &PortInit);

    /* Configure PORTB pins 8 for output to switch LEDs on/off */
  PortInit.PORT_Pin   = (PORT_Pin_8);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_SLOW;

  PORT_Init(MDR_PORTB, &PortInit);

      /* Configure PORTC pins 1 for output to switch LEDs on/off */
  PortInit.PORT_Pin   = (PORT_Pin_1);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_SLOW;

  PORT_Init(MDR_PORTC, &PortInit);
}


void Uart2Setup(void)
{
	unsigned int temp;
 
    UART_InitStructure.UART_BaudRate                = 115200;
    UART_InitStructure.UART_WordLength              = UART_WordLength8b;
    UART_InitStructure.UART_StopBits                = UART_StopBits1;
    UART_InitStructure.UART_Parity                  = UART_Parity_No;
    UART_InitStructure.UART_FIFOMode                = UART_FIFO_OFF;
    UART_InitStructure.UART_HardwareFlowControl     = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;

    /* Configure UART2 parameters*/
    UART_Init (MDR_UART2,&UART_InitStructure);
      /* Enable Receiver interrupt */
 //   UART_ITConfig (MDR_UART2, UART_IT_RX, ENABLE);
    /* Enables UART2 peripheral */
    UART_Cmd(MDR_UART2,ENABLE); 

      /* Enable transmitter interrupt (UARTTXINTR) */
 // UART_ITConfig (MDR_UART2, UART_IT_TX, ENABLE);

   /* Enable Receiver interrupt */
    UART_ITConfig (MDR_UART2, UART_IT_RX, ENABLE);

}

void Uart1Setup(void)
{
	unsigned int temp;
 
    UART_InitStructure.UART_BaudRate                = 115200;
    UART_InitStructure.UART_WordLength              = UART_WordLength8b;
    UART_InitStructure.UART_StopBits                = UART_StopBits1;
    UART_InitStructure.UART_Parity                  = UART_Parity_No;
    UART_InitStructure.UART_FIFOMode                = UART_FIFO_OFF;
    UART_InitStructure.UART_HardwareFlowControl     = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;

    /* Configure UART1 parameters*/
    UART_Init (MDR_UART1,&UART_InitStructure);
      /* Enable Receiver interrupt */
 //   UART_ITConfig (MDR_UART1, UART_IT_RX, ENABLE);
    /* Enables UART1 peripheral */
    UART_Cmd(MDR_UART1,ENABLE); 

         /* Enable transmitter interrupt (UARTTXINTR) */
 // UART_ITConfig (MDR_UART1, UART_IT_TX, ENABLE);

   /* Enable Receiver interrupt */
  UART_ITConfig (MDR_UART1, UART_IT_RX, ENABLE);

}

void SPI_init(void)

{
	/* Configure SSP2 pins: FSS, CLK, RXD, TXD */
  /* Configure PORTD pins  3 как ручной чипселект*/ 
  PortInit.PORT_Pin   = (PORT_Pin_3);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_FAST;
  PortInit.PORT_PD    = PORT_PD_DRIVER;
  PORT_Init(MDR_PORTD, &PortInit);	

  /* Configure PORTD pins   5, 6 */
  PortInit.PORT_Pin   = (PORT_Pin_6 |  PORT_Pin_5);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PortInit.PORT_FUNC  = PORT_FUNC_ALTER;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_FAST;
  PortInit.PORT_PD    = PORT_PD_DRIVER;
  PORT_Init(MDR_PORTD, &PortInit);

  /* Configure SSP1 pins: FSS, CLK, RXD, TXD */

  /* Configure PORTF pins 0, 1,  3 */
  PortInit.PORT_Pin   = (PORT_Pin_0 | PORT_Pin_1);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PORT_Init(MDR_PORTF, &PortInit);

    /* Configure PORTF pins  2 ,  как ручной чипселект */

  PortInit.PORT_Pin   = (PORT_Pin_2);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_FAST;
  PortInit.PORT_PD    = PORT_PD_DRIVER;
  PORT_Init(MDR_PORTF, &PortInit);	

  SSP_BRGInit(MDR_SSP1,SSP_HCLKdiv16);
  SSP_BRGInit(MDR_SSP2,SSP_HCLKdiv16);

  /* SSP1 MASTER configuration ------------------------------------------------*/
  SSP_StructInit (&sSSP);

  sSSP.SSP_SCR  = 0xff; /*!< This member configures the SSP communication speed.
                                                This parameter is number from 0 to 255.
                                                The information rate is computed using the following formula:
                                                F_SSPCLK / ( CPSDVR * (1 + SCR) ) */
  sSSP.SSP_CPSDVSR = 16;/*!< This member configures the SSP clock divider.
                                                This parameter is an even number from 2 to 254 */
  sSSP.SSP_Mode = SSP_ModeMaster;
  sSSP.SSP_WordLength = SSP_WordLength16b;
  sSSP.SSP_SPH = SSP_SPH_2Edge;
  sSSP.SSP_SPO = SSP_SPO_High;
  sSSP.SSP_FRF = SSP_FRF_SPI_Motorola;
  sSSP.SSP_HardwareFlowControl = SSP_HardwareFlowControl_None;
  SSP_Init (MDR_SSP1,&sSSP);

  /* SSP2 MASTER configuration ------------------------------------------------*/
  
  sSSP.SSP_SCR  = 0xff;
  sSSP.SSP_CPSDVSR = 16;
  sSSP.SSP_Mode = SSP_ModeMaster;
  sSSP.SSP_WordLength = SSP_WordLength16b;
  sSSP.SSP_SPH = SSP_SPH_2Edge;
  sSSP.SSP_SPO = SSP_SPO_High;
  sSSP.SSP_FRF = SSP_FRF_SPI_Motorola;
  sSSP.SSP_HardwareFlowControl = SSP_HardwareFlowControl_None;
  SSP_Init (MDR_SSP2,&sSSP);

  /* Enable SSP1 */
  SSP_Cmd(MDR_SSP1, ENABLE);
  /* Enable SSP2 */
  SSP_Cmd(MDR_SSP2, ENABLE);

}


//===============================================================
void delay(int inc)
{
//---------------------------------------------------------------
   inc <<= 3;
   while(inc !=0) inc--;
//---------------------------------------------------------------
}



void CS_SPI1 (char a)
{
   while (SSP_GetFlagStatus(MDR_SSP1, SSP_FLAG_BSY) == SET) {};

  if (a==1)
  {
    delay(300);
    PORT_SetBits  (MDR_PORTF, PORT_Pin_2);
    delay(300);
  }

  if (a==0)
  {
    delay(300);
    PORT_ResetBits (MDR_PORTF, PORT_Pin_2);
    delay(300);
  }

}     

void CS_SPI2 (char a)
{

  while (SSP_GetFlagStatus(MDR_SSP2, SSP_FLAG_BSY) == SET) {};

  if (a==1)
  {
    delay(300);
    PORT_SetBits  (MDR_PORTD, PORT_Pin_3);
    delay(300);
  }

  if (a==0)
  {
    delay(300);
    PORT_ResetBits (MDR_PORTD, PORT_Pin_3);
    delay(300);
  }

}                 

void SPI1_send(u16 a)
{
	/* Wait for SPI1 Tx buffer empty */
    while (SSP_GetFlagStatus(MDR_SSP1, SSP_FLAG_TFE) == RESET)
    {
    }
    /* Send SPI1 data */
    SSP_SendData(MDR_SSP1,a);
}

void SPI2_send(u16 a)
{
	/* Wait for SPI2 Tx buffer empty */
    while (SSP_GetFlagStatus(MDR_SSP2, SSP_FLAG_TFE) == RESET)
    {
    }
    /* Send SPI2 data */
    SSP_SendData(MDR_SSP2,a);
}

u16 SPI1_read(void)
{
	u16 a;
	/* Wait for SPI1 data reception */
    while (SSP_GetFlagStatus(MDR_SSP1, SSP_FLAG_RNE) == RESET)
    {
    }
    /* Read SPI1 received data */
    a = SSP_ReceiveData(MDR_SSP1);
    return a;
}

u16 SPI2_read(void)
{
	u16 a;
	/* Wait for SPI2 data reception */
    while (SSP_GetFlagStatus(MDR_SSP2, SSP_FLAG_RNE) == RESET)
    {
    }
    /* Read SPI2 received data */
    a = SSP_ReceiveData(MDR_SSP2);
    return a;
}


//===============================================================

void RegisterInits () 
{ 
  /* Set RST_CLK to default */
 //   RST_CLK_DeInit();
   
 RST_CLK_PCLKcmd(RST_CLK_PCLK_BKP,ENABLE);

// 3. CPU_CLK = 7*HSE/2 clock 
   
    //Enable HSE clock source 
     RST_CLK_HSEconfig(RST_CLK_HSE_ON);

     while (RST_CLK_HSEstatus() != SUCCESS)  {}; // Good HSE clock 

     // Select HSE clock as CPU_PLL input clock source
     // Set PLL multiplier to 7                       
     RST_CLK_CPU_PLLconfig(RST_CLK_CPU_PLLsrcHSEdiv1, 5);

      // Enable CPU_PLL 
     RST_CLK_CPU_PLLcmd(ENABLE);

     while (RST_CLK_HSEstatus() != SUCCESS)    {}; //Good CPU PLL 

       // Set CPU_C3_prescaler to 2 
        RST_CLK_CPUclkPrescaler(RST_CLK_CPUclkDIV2);
        // Set CPU_C2_SEL to CPU_PLL output instead of CPU_C1 clock 
        RST_CLK_CPU_PLLuse(ENABLE);
        // Select CPU_C3 clock on the CPU clock MUX 
        RST_CLK_CPUclkSelection(RST_CLK_CPUclkCPU_C3);

     RST_CLK_PCLKcmd(RST_CLK_PCLK_RST_CLK,ENABLE);
	 RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTA,ENABLE);
	 RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTB,ENABLE);
	 RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTC,ENABLE);
	 RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTD,ENABLE); 
	 RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTE,ENABLE);
     RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTF,ENABLE);

//Enables SPI
	RST_CLK_PCLKcmd(RST_CLK_PCLK_SSP1,ENABLE);
	RST_CLK_PCLKcmd(RST_CLK_PCLK_SSP2,ENABLE);

 /* Enables the CPU_CLK clock on UART1,UART2 */
     RST_CLK_PCLKcmd(RST_CLK_PCLK_UART1, ENABLE);
     RST_CLK_PCLKcmd(RST_CLK_PCLK_UART2, ENABLE);

// Set the HCLK division factor = 1 for UART1,UART2 
    UART_BRGInit(MDR_UART1, UART_HCLKdiv1);
    UART_BRGInit(MDR_UART2, UART_HCLKdiv1);

  NVIC_EnableIRQ(UART1_IRQn);
  NVIC_EnableIRQ(UART2_IRQn);


}

 void TX_485 (void)
{
	RE_485_1;
	DE_485_1;
}

void RX_485 (void)
{
	RE_485_0;
	DE_485_0;
}


void zputc ( uint8_t c) 
{
  uint8_t DataByte;

  DataByte=c;

    /* Check TXFE flag */
    while (UART_GetFlagStatus (MDR_UART1, UART_FLAG_TXFE)!= SET)   {};
   /* Send Data from UART1 */
    UART_SendData (MDR_UART1,DataByte);

}


void zputc2 ( uint8_t c) 
{
  uint8_t DataByte;

  DataByte=c;

    /* Check TXFE flag */
    while (UART_GetFlagStatus (MDR_UART2, UART_FLAG_TXFE)!= SET)   {};
   /* Send Data from UART2 */
    UART_SendData (MDR_UART2,DataByte);

}


 void zputs( char s[], unsigned char l)
{
    unsigned char i=0;
         char c[64];
	
      for (i=0;i<l;i++) 
        {
            c[i]=s[i];
         zputc (c[i]);
        }

}

 void zputs2( char s[], unsigned char l)
{
      unsigned char i=0;
               char c[64];

      for (i=0;i<l;i++) 
        {
             c[i]=s[i];
         zputc2 (c[i]);
        }
 }

unsigned int leng ( char s[])

  {
    unsigned  char i=0;
     while ((s[i]!='\0')&&(i<120)) { i++;}
    return i;
  }

void sendT ( char  s[])

 { 
   zputs(s,leng(s));
 }

void sendT2 ( char s[])

 {
    zputs2(s,leng(s));

 }




  // reverse: 
 void reverse( char s[])
 {
     int i, j;
     char c;
 
     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
 }


 void itoa(int n, char s[])
 {
     int i, sign;
 
     if ((sign = n) < 0)  
         n = -n;         
     i = 0;
     do {       
         s[i++] = n % 10 + '0';   
     } while ((n /= 10) > 0);    
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
 }


//----------------------------------------

 void Transf( char s[]) 
   {
         sendT(s);    //uart1
        sendT2(s);   //uart2
    }
   

 void ZTransf( char s[],unsigned char a) 
   {
      zputs (s,a);
      zputs2(s,a);
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
                   {; 
                    return 1;
                   }
              }
  
       else  
                            return 1;       
   
      return 0;
    }


//--------------------------------


//--- Terminal configuration ---
void Menu()

{

}


void Menu1()
 
 {
//***************************************************************************

    int i;
	
 
    for (i=0; i< 3; i++) Transf("\r");    // очистка терминала
	for (i=0; i<15; i++) Transf ("*");  // вывод приветствия
	
	Transf("\r");
	Transf("\r");
	Transf("\r");
	Transf("\r");
	Transf("......Terminal ФОЧ-1 Комплекция-12....\r");
	Transf("\r");
	Transf("\r");
	Transf("\r");
	Transf("MENU :\r");
	Transf("-------\r");
	Transf("~ - стартовый байт\r");
	Transf("1 - адрес абонента\r");
	Transf("start_TCP -  команда\r");
	Transf("=30000000 - данные команды\r");
	Transf(";- конец пачки \r");
	Transf(".............. \r");
	Transf("Адреса: \r");
	Transf("1 - 1У-К6xx   , синхронизатор мастер: \r");
	Transf("2 - 1У-К6xx-1 , синхронизатор резерв\r");
	Transf("3 - 1У-К6xx , синтезатор частоты излучения  \r");
	Transf("4 - 1У-К6xx , синтезатор частоты гетеродина  \r");
	Transf("6 - 1У-К6xx , ФОЧ  \r");
	Transf("+++++++++++++++++++\r");
	Transf("~1 watch_reg_ФАПЧ1;\r");
	Transf("~1 watch_reg_ФАПЧ2;\r");
	/*
	sendT("~1 id_dds;\r\n");
	sendT("~1 wrn_dds_1;\r\n");
	sendT("~1 wrn_dds_0;\r\n");
	sendT("~1 init_dds;\r\n");
	sendT("~1 rd_dds;\r\n");
	sendT("~1 wr_dds=1;\r\n");
	sendT("~1 rd_tst;\r\n");
	sendT("~1 wr_tst=1;\r\n");
	*/
  
	Transf("~1 ФАПЧ1_Kdiv1();\r");
	Transf("~1 ФАПЧ2_Kdiv1();\r");

	Transf("~1 ФАПЧ1_Kdiv2();\r");
	Transf("~1 ФАПЧ2_Kdiv2();\r");

	Transf("~1 help;\r");
	Transf("~1 SPI_send():0x55;\r");
	
	/*
	sendT("~1 config_Ethernet;\r\n");
	sendT("~1 состояние;\r\n");
	sendT("~1 включить;\r\n");
	sendT("~1 выключить;\r\n");
	sendT("~1 UDP_TCP_test;\r\n");
	sendT("~1 синхросигналы;\r\n");
	sendT("~1 PPI_пачка_К612;\r\n");
	sendT("~1 PPI_пачка_К613;\r\n");
	*/
	
	Transf("~1 ФАПЧ1_init;\r");
	Transf("~1 ФАПЧ2_init;\r");
	Transf("~1 Включить_синтезатор;\r");

	Transf("~1 Индикация_состояния_синтезатора;\r");
	/*
	sendT("~1 Включить_ЛЧМ_UP;\r\n");
	sendT("~1 Включить_ЛЧМ_DOWN;\r\n");
	*/
	Transf("~1 Установить_Амплитуду=0;\r");
	/*
	sendT("~1 Установить_F0=60000000;\r\n");
	sendT("~1 Установить_ЛЧМ;\r\n");
	sendT("~1 Частота_повторения_Fимп=100;\r\n");
	sendT("~1 Длительность_импульса_Tимп=100;\r\n");
	sendT("~1 Установить_частоту_F_low=430000000;\r\n");
	sendT("~1 Установить_частоту_F_high=440000000;\r\n");
	*/
	/*
	sendT("~3 Подвинуть_частоту_вверх=100000;\r\n");
	sendT("~3 Подвинуть_частоту_вниз=100000;\r\n");
	sendT("~3 Подвинуть_фазу_вверх;\r\n");
	sendT("~3 Подвинуть_фазу_вниз;\r\n");
	sendT("~3 Подвинуть_амплитуду_вверх;\r\n");
	sendT("~3 Подвинуть_амплитуду_вниз;\r\n");
	sendT("~1 сигнал_авария_ON;\r\n");
	sendT("~1 сигнал_авария_OFF;\r\n");
	sendT("~1 SPORT_data;\r\n");
	sendT("~3 показать_спектр;\r\n");
    sendT("~3 стоп_спектр;\r\n");
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
  
    Transf("~1 ADC=1;  проверка ADC микроконтроллера\r");
  /*sendT("~6 сигналы_контроля;\r\n");  
    sendT("~6 контроль_уровней;\r\n");
    sendT("~6 массив_ДБм;\r\n");
	sendT("~6 init_K611;\r\n");
	sendT("~1 K611;\r\n");
	sendT("~5 INPORT:xxxxxx;\r\n");
	sendT("~5 OUTPORT:xxxxxx;\r\n");
	*/
	Transf("\r");
	Transf("\r");
	Transf("++++++++++++++++++++\r\n");
	Transf("\r");
	Transf("\r");
	//for (i=0; i<64; i++) zputs ("*",1);  // вывод приветствия
	//for (i=0;i<10;i++) puts("\r",1);  // очистка терминала
	sendT("\r");
	//*******************************************************************************
	
	}

void reg_FAPCH_watch (reg_1508pl10 a)
{
  Transf ("Индикация структуры бит ФАПЧ:");

  if (a.Name[4]=='1') Transf ("ФАПЧ1\r");
  if (a.Name[4]=='2') Transf ("ФАПЧ2\r");
  Transf("\r");

}

void Kdiv1(unsigned int dFo1,reg_1508pl10 *a)
{

   Transf("\r");
   Transf ("..установка кода Kdiv1:");
   sprintf(strng,"%d",dFo1);
   Transf(strng);
   Transf("\r");

  if (a->Name[4]=='1') Transf ("ФАПЧ1\r");
  if (a->Name[4]=='2') Transf ("ФАПЧ2\r");
  Transf("\r");

       if (dFo1==10)  {
                        a->KREF4=0;
                        a->KREF3=0;
                      } else
                                
       if (dFo1==80)  {
                        a->KREF4=0;
                        a->KREF3=1;
                      } else
       if (dFo1==100) {
                        a->KREF4=1;
                        a->KREF3=0;
                      } else
       if (dFo1==125) {
                        a->KREF4=1;
                        a->KREF3=1;
                      } else Transf ("..Kdiv1 - НЕТ ТАКОГО КОДА!!!...\r");
                                           
 Transf("\r");

}

void Kdiv2(unsigned int dFo1,reg_1508pl10 *a)
{
   Transf("\r");
   Transf ("..установка кода Kdiv2:");
   sprintf(strng,"%d",dFo1);
   Transf(strng);
   Transf("\r");
   
  if (a->Name[4]=='1') Transf ("ФАПЧ1\r");
  if (a->Name[4]=='2') Transf ("ФАПЧ2\r");
  Transf("\r");


       if (dFo1==160)  {
                        a->KREF2=0;
                        a->KREF1=0;
                        a->KREF0=0;
                                           } else
                                
       if (dFo1== 80)  {
                        a->KREF2=0;
                        a->KREF1=0;
                        a->KREF0=1;
                                           } else
       if (dFo1== 40) {
                        a->KREF2=0;
                        a->KREF1=1;
                        a->KREF0=0;
                                           } else
       if (dFo1== 20) {
                        a->KREF2=0;
                        a->KREF1=1;
                        a->KREF0=1;
                                           } else
       if (dFo1==200) {
                        a->KREF2=1;
                        a->KREF1=0;
                        a->KREF0=0;
                                           } else

       if (dFo1==100) {
                        a->KREF2=1;
                        a->KREF1=0;
                        a->KREF0=1;
                                           } else
       if (dFo1==50) {
                        a->KREF2=1;
                        a->KREF1=1;
                        a->KREF0=0;
                                           } else
       if (dFo1==10) {
                        a->KREF2=1;
                        a->KREF1=1;
                        a->KREF0=1;
                                           } else Transf ("..Kdiv2 - НЕТ ТАКОГО КОДА!!!...\r");

        // Kdiv1(10,a); 
//------------для других dFo1>200 --------------------- 


}


void Kdiv3(unsigned int dFo1,reg_1508pl10 *a)
{
   Transf("\r");
   Transf ("..установка кода Kdiv3:");
   sprintf(strng,"%d",dFo1);
   Transf(strng);
   Transf("\r");
   
  if (a->Name[4]=='1') Transf ("ФАПЧ1\r");
  if (a->Name[4]=='2') Transf ("ФАПЧ2\r");
  Transf("\r");

   a->Kosn00_15 =  dFo1;     //младшие 16 бит кода деления основного канала
   a->Kosn16_19 = (dFo1>>16);//старшие 4-ре бита кода деления основного канала

}


char Sys_indikate(void)

{
  char a=0;
  unsigned char cond=0;


}

void Otvet_3(void)
{
   otvet_flag=0;
				 
   TX_485();
   Transf("\r");
   Transf ("~1 состояние_К611:");
   sprintf(s3,"%d",flag_status_ZAHVAT);
   Transf(s3);
   Transf(".");
   sprintf(s3,"%d",flag_status_ZAHVAT);
   Transf(s3);
   Transf(";");
   Transf("\r");
   RX_485();
 
}


void IO ( char* sr)      // функция обработки протокола обмена
 
 {
       
 unsigned char s[6];   
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
  
  
   i=leng (sr);
     
   sym1=sr[0];
   
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


	   InOut[index1]=sr[index];
	   sr[index]=0x0;
	  
	  if  (InOut[index1]==' ')  ink1=ink1+1;
	  
	  if  (InOut[index1]==';')  packet_ok=1;
	  
	  if ((InOut[index1]=='=')||(InOut[index1]==':')) data_flag=1;

    if  (InOut[index1]=='.') data_flag2=1;


        	  if ((index1>2)&&(InOut[2]==' '))  
        		{
        		
                		         if  ((InOut[index1]!=' ')&&
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

                    			          {
                    			             Transf ("принял help\r"    ); 
                    			              Menu1('t');
                    			             flag_uart_trcv=0;
                    			             sch_pachek_test=1;
                    			           }

   
                              if (strcmp(Word,"watch_reg_ФАПЧ1"  )==0)       { Transf ("принял watch_reg_ФАПЧ1\r"  ); reg_FAPCH_watch (reg_FAPCH1); }//чтение и вывод на экран регистров ФАПЧ 
					 		  if (strcmp(Word,"watch_reg_ФАПЧ2"  )==0)       { Transf ("принял watch_reg_ФАПЧ2\r"  ); reg_FAPCH_watch (reg_FAPCH2); }//чтение и вывод на экран регистров ФАПЧ 

                  			  if (strcmp(Word,"ADC"      )==0) 	    //измерить напряжение в канале //проверка ADC микроконтроллера
                                    				{ 	dFo1=atoi(DATA_Word);
                  								           	Transf ("принял ADC:" );
                                    					    sprintf(strng,"%x",dFo1);
															Transf(strng);
															Transf("\r\n");
                  									             //ADC_test(dFo1);
                  									
                                    					 }
	
                            if (strcmp(Word,"Установить_Амплитуду"      )==0) 	    //установить частоту F0 синтезатора
                        				{ 	dFo1=atoi(DATA_Word);
      									        Transf ("принял Установить_Амплитуду:" );
                        				        sprintf(strng,"%x",dFo1);
												Transf(strng);
												Transf("\r");
												sendT(strng);
												Transf("\r");
												sendT2(strng);
                        					    Transf("\r");
      									
                        				}   

                        		if (strcmp(Word,"SPI_send()"      )==0) 	    //установить частоту F0 синтезатора
                        				{ 	dFo1=atoi(DATA_Word);
      								          Transf ("принял SPI_send():" );
                        				      sprintf(strng,"%x",dFo1);
                        					  Transf(strng);
                        					  Transf("\r");

                        					SPI1_send(dFo1);
                        					SPI2_send(dFo1);
      									
                        				}   

                             if (strcmp(Word,"ФАПЧ1_Kdiv1()"      )==0)       //установить частоту F0 синтезатора
                                  {   dFo1=atoi(DATA_Word);
                                      Transf ("принял ФАПЧ1_Kdiv1():" );
                                      sprintf(strng,"%d",dFo1);
                                      Transf(strng);
                                      Transf("\r");

                                      Kdiv1(dFo1,&reg_FAPCH1);
                                      Write_FOCH1 (reg_FAPCH1);
                          
                                     } 
                             if (strcmp(Word,"ФАПЧ2_Kdiv1()"      )==0)       //установить частоту F0 синтезатора
                                  {   dFo1=atoi(DATA_Word);
                                      Transf ("принял ФАПЧ2_Kdiv1():" );
                                      sprintf(strng,"%d",dFo1);
                                      Transf(strng);
                                      Transf("\r");

                                      Kdiv1(dFo1,&reg_FAPCH2);
                                      Write_FOCH2 (reg_FAPCH2);
                          
                                     } 
                          if (strcmp(Word,"ФАПЧ1_Kdiv2()"      )==0)       //установить частоту F0 синтезатора
                                  {   dFo1=atoi(DATA_Word);
                                      Transf ("принял ФАПЧ1_Kdiv2():" );
                                      sprintf(strng,"%d",dFo1);
                                      Transf(strng);
                                      Transf("\r");

                                      Kdiv2(dFo1,&reg_FAPCH1);
                                      Write_FOCH1 (reg_FAPCH1);
                          
                                     } 
                        if (strcmp(Word,"ФАПЧ2_Kdiv2()"      )==0)       //установить частоту F0 синтезатора
                                  {   dFo1=atoi(DATA_Word);
                                      Transf ("принял ФАПЧ2_Kdiv2():" );
                                      sprintf(strng,"%d",dFo1);
                                      Transf(strng);
                                      Transf("\r");

                                      Kdiv2(dFo1,&reg_FAPCH2);
                                      Write_FOCH2 (reg_FAPCH2);
                          
                                     } 

                        if (strcmp(Word,"ФАПЧ1_Kdiv3()"      )==0)       //установить частоту F0 синтезатора
                                  {   dFo1=atoi(DATA_Word);
                                      Transf ("принял ФАПЧ1_Kdiv3():" );
                                      sprintf(strng,"%d",dFo1);
                                      Transf(strng);
                                      Transf("\r");

                                      Kdiv3(dFo1,&reg_FAPCH1);
                                      Write_FOCH1 (reg_FAPCH1);
                          
                                     } 
                        if (strcmp(Word,"ФАПЧ2_Kdiv3()"      )==0)       //установить частоту F0 синтезатора
                                  {   dFo1=atoi(DATA_Word);
                                      Transf ("принял ФАПЧ2_Kdiv3():" );
                                      sprintf(strng,"%d",dFo1);
                                      Transf(strng);
                                      Transf("\r");

                                      Kdiv3(dFo1,&reg_FAPCH2);
                                      Write_FOCH2 (reg_FAPCH2);
                          
                                     }  
						if (strcmp(Word,"LD_control")==0)      
                                    { 
                                      Transf ("принял LD_control\r"   );

                                      if (flag_status_ZAHVAT==0x00) Transf ("Захватов нет\r" );
                                      if (flag_status_ZAHVAT==0x01) Transf ("Захват Гетеродина\r" );
                                      if (flag_status_ZAHVAT==0x02) Transf ("Захват Синтезатор\r" );
                                      if (flag_status_ZAHVAT==0x03) Transf ("Оба в захвате\r" );

                                      Transf ("LD_control():" );
                                      sprintf(strng,"%d",flag_status_ZAHVAT);
                                      Transf(strng);
                                      Transf("\r");
									  
									                      Otvet_3();

                                    }; 
									 

                  	    if (strcmp(Word,"ФАПЧ1_init")==0)      { Transf ("принял ФАПЧ1_init\r"   ); Write_FOCH1 (reg_FAPCH1);    };	
                  	    if (strcmp(Word,"ФАПЧ2_init")==0)      { Transf ("принял ФАПЧ2_init\r"   ); Write_FOCH2 (reg_FAPCH2);    };
					    if (strcmp(Word,"ФОЧ_init")==0)        { Transf ("принял ФОЧ_init\r"       ); FLAG_INIT_FOCH=0;  };
						

                 	    if (strcmp(Word,"Индикация_состояния_синтезатора")==0)      { Transf ("принял Индикация_состояния_синтезатора\r"   );     }		   
                        if (strcmp(Word,"Включить_синтезатор"		     )==0)      { Transf ("принял Включить_синтезатор\r"        		 );     }
             
						
                        if (strcmp(Word,"сигналы_захвата_1У-К611" )==0) 	   //контроль сигналов захвата кассеты 611
                  				{ 
                                  crc_comp =atoi(DATA_Word);
                                  crc_input=atoi(DATA_Word2);  

                             if (crc_comp==crc_input)
                                 {
                                        z_k611=atoi(DATA_Word);

                                   //     Transf ("принял сигналы_захвата_1У-К611:" );
                                  //      ZTransf (DATA_Word,index_data_word-1);
                                  //      Transf("\r\n");

                                        //control_zahvat_K611(z_k611);
                                         Flag_zahvat_end_K611=1;
                                         tick_process=0;
                                        // Flag_K611=0;
                                         sch_obmen=0;

                                 }

                          
                  					}	
                  					
                  		if (strcmp(Word,"значение_уровней_1У-К611" )==0) 	   //контроль уровне сигналов кассеты 611
                  				{ 

                                  crc_comp =atoi(DATA_Word);
                                  crc_input=atoi(DATA_Word2); 

                             if (crc_comp==crc_input)
                                 {
                                       z_k611=atoi(DATA_Word);
                            			//	   Transf ("принял значение_уровней_1У-К611:" );
                            			//  	 ZTransf (DATA_Word,index_data_word-1);
                                 //      Transf("\r\n");
                                      // control_level_K611(z_k611);
                            				   Flag_control_end_K611=1;
                            				   tick_process=0;
                            				  // Flag_K611=0;
                            				   sch_obmen=0;
                                  }
                  					}

                            
                  	
                  		if (strcmp(Word,"init_K611")==0) 
                  		{ 
                  	//	Transf ("принял init_K611  \r"  );
                  		Flag_init_K611=1;
                  		Flag_K611=0;
                  		tick_process=0;
                  		sch_obmen=0;
                  	    tick_process_K611=0;
                  		} //принимаем подтверждение на ранее отосланную команду инициализации
                  		
                  		if (strcmp(Word,"состояние_1У-К611")==0)
                  		{ 
                  //		Transf ("принял состояние_1У-К611  \r"  );
                  		Flag_control_K611=1;
                  		//Flag_K611=0;
                  		tick_process=0;
                  		sch_obmen=0;
                  	    tick_process_K611=0;
                  				} //принимаем подтверждение на ранее отосланную команду контроля состояния
                  		
                  		if (strcmp(Word,"сигналы_контроля_1У-К611")==0)
                  		{ 
                  	//	Transf ("принял сигналы_контроля_1У-К611  \r"  );
                  		Flag_zahvat_sig_K611=1;
                  		//Flag_K611=0;
                  		tick_process=0;
                  		sch_obmen=0;
                  	    tick_process_K611=0;
                  		} //принимаем подтверждение на ранее отосланную команду контроля сигнала захвата
                  		
                  		if (strcmp(Word,"контроль_уровней_1У-К611")==0)
                  		{ 
                  	//	Transf ("принял контроль_уровней_1У-К611  \r"  );
                  		Flag_control_sig_K611=1;
                  		//Flag_K611=0;
                  		tick_process=0;
                  		sch_obmen=0;
                  	    tick_process_K611=0;
                  		} //принимаем подтверждение на ранее отосланную команду контроля уровней
                  		
                  //--------------------------------------------------K612----------------------------------------------------------------				
              
                      
		
	    } 
	
	}
	
	if ((packet_ok==1)&&(crc_ok==0x1))     //обработка команд адресатом которых является слейв

	{
		
	  if (Master_flag==0)

      {/*
    
        	Transf("\r\n");
        	ZTransf (InOut,index1);
       // 	Transf("\r\n");
       // 	Transf("\r\n");
          //    TX_485;	
		*/		  
      }

		
	}
	
	
	if ( packet_ok==1) 
		
		{
		
			for (i=0;i<Bufer_size;i++)        Word[i]     =0x0;
			for (i=0;i<Bufer_size;i++)   DATA_Word[i]     =0x0;
			for (i=0;i<Bufer_size;i++)  DATA_Word2[i]     =0x0;  
			for (i=0;i<Bufer_size;i++)       InOut[i]     =0x0;
		
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



void UART1_IRQHandler(void)
{ 
  unsigned stat;
    unsigned t;
  uint32_t value;
  uint8_t ReciveByte;
  unsigned char letter[1];

if (UART_GetITStatusMasked(MDR_UART1, UART_IT_RX) == SET)
  {
    UART_ClearITPendingBit(MDR_UART1, UART_IT_RX);
    uart1_IT_RX_flag = SET;
  }

  if (uart1_IT_RX_flag == SET)
  {

    /* Clear uart2_IT_RX_flag */
       uart1_IT_RX_flag = RESET;

     /* Recive data */
    ReciveByte = UART_ReceiveData (MDR_UART1);

  letter[0]=ReciveByte;
  
  BufIsRead_flag=1; //флаг пришедшего байта
  
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


}




/*******************************************************************************
* Function Name  : UART2_IRQHandler
* Description    : This function handles UART2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART2_IRQHandler(void)
{
  unsigned stat;
    unsigned t;
  uint32_t value;
  uint8_t ReciveByte;
  unsigned char letter[1];

if (UART_GetITStatusMasked(MDR_UART2, UART_IT_RX) == SET)
  {
    UART_ClearITPendingBit(MDR_UART2, UART_IT_RX);
    uart2_IT_RX_flag = SET;
  }

  if (uart2_IT_RX_flag == SET)
  {

    /* Clear uart2_IT_RX_flag */
       uart2_IT_RX_flag = RESET;

     /* Recive data */
    ReciveByte = UART_ReceiveData (MDR_UART2);
  

  
  letter[0]=ReciveByte;

  BufIsRead_flag=1;   //флаг пришедшего байта
  
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
}

//===============================================================


void UART_control (void)

{
     if ((BufIsRead_flag==1)) // 
           
            {            	
               getStr(sr,&lsr);
               BufIsRead_flag=0;
			   //ZTransf(sr,lsr);
               IO(sr);
			   
		  	   RX_485();
                    
            } 
}




uint8_t LD_control (void) // если 0 - нет захватов, 1 - захват гетеродина, 2 - захват излучения, 3 - оба в захвате.
{
  uint8_t a=0;
 
  a =      a|PORT_ReadInputDataBit(MDR_PORTE,PORT_Pin_1); //чтение состояния сигнала Lock Detect мезанина Излучения
  a = (a<<1)|PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_2); //чтение состояния сигнала Lock Detect мезанина Гетеродина

  return a;

}

unsigned char INIT_FOCH (unsigned char z)
{
  
	//--------инициализация ФАПЧ-1--------------
	if (z==5)  IO("~6 ФАПЧ2_init;");

	if (z==8)  IO("~6 ФАПЧ2_Kdiv1()=10;");

	if (z==7)  IO("~6 ФАПЧ2_Kdiv2():20;");
	   
	if (z==6)  IO("~6 ФАПЧ2_Kdiv3():3600;");

	//--------инициализация ФАПЧ-2--------------
	 
	if (z==1)  IO("~6 ФАПЧ1_init;");

	if (z==4)  IO("~6 ФАПЧ1_Kdiv1()=10;");

	if (z==3)  IO("~6 ФАПЧ1_Kdiv2():10;");

	if (z==2)  IO("~6 ФАПЧ1_Kdiv3():2000;");

	//-------контроль захватов-----------------

	 if (z==9) {IO("~6 LD_control;");FLAG_INIT_FOCH=1;z=0;}
	 
	 z = z+1;
	 
	return z;
}

void Led_Zahvat (unsigned char a)
{
	static unsigned char l;
	
	  if (a==0x00) l=0;
      if (a==0x01) l=~l;
      if (a==0x02) l=~l;
      if (a==0x03) l=0xff;
	  if (l>0u) ZAHVAT_ON; else ZAHVAT_OFF;

}


unsigned char Watch_DOG (unsigned char i)

{
  if (i==1) LED2_OFF;

  if (i==2) LED2_ON;           

  if (i==3) LED1_OFF;          

  if (i==4) LED1_ON;
  
  if (i==5) i=0;
  
  i++;
  
  return i;
 }
 




#define INIT_FOCH_delay 100000
#define led_delay 70000

int main()
{
	unsigned int i=0;
	unsigned int j=0;


    RegisterInits();

    MltPinCfg ();
    LedPinGfg ();

    Uart1PinCfg();
    Uart1Setup();
    

    Uart2PinCfg();
    Uart2Setup();

    SPI_init();

    pcur_Tr= TrBuf;
    pTr_Buf= TrBuf;

    pcur_Rcv= RcvBuf;
    pRcv_Buf= RcvBuf;

   for (i=0;i<Bufer_size;i++)       strng[i]     =0x0;
   for (i=0;i<Bufer_size;i++)          sr[i]     =0x0;
   for (i=0;i<Bufer_size;i++)        Word[i]     =0x0;
   for (i=0;i<Bufer_size;i++)   DATA_Word[i]     =0x0;
   for (i=0;i<Bufer_size;i++)  DATA_Word2[i]     =0x0;  
   for (i=0;i<Bufer_size;i++)       InOut[i]     =0x0;

   	i=0;

  // 	Menu1();

	RX_485();

    PWR_ON; //включаем светодиод питания на лицевой панели


 while(1)
 {
			if (i<INIT_FOCH_delay) i=i+1; else i=0;
			if (j<led_delay)       j=j+1; else j=0;

             UART_control ();

             flag_status_ZAHVAT=LD_control (); //контроль сигналов захвата

            if (j==led_delay)  FLAG_LED_n=Watch_DOG (FLAG_LED_n);
    
			      if (j==led_delay) Led_Zahvat (flag_status_ZAHVAT); //индикация сигналов захвата на лицевой панели
	
            if ((i==INIT_FOCH_delay)&&(FLAG_INIT_FOCH==0)) {FLAG_INIT_n_FOCH=INIT_FOCH (FLAG_INIT_n_FOCH);} //инициализация ФАПЧей
				
            if ((i==INIT_FOCH_delay)&&(FLAG_INIT_MENU==0)) {Menu1();FLAG_INIT_MENU=1;}

      
 }
 
}


