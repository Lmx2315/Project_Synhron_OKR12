
#include "opora.h"
#include "DDS1508pl1t.h"
#include "TERMINAL.h"


#include <math.h>
#include <stdint.h>

#define  u16 unsigned short
#define  u32 unsigned int   
#define  u8  unsigned char

extern char strng [64];

unsigned char _PolSync      =0;        // полярность импульсов синхронизации - 1
unsigned char _Mode         =0;        // номер профиля - 2
unsigned int _Poz_Freq     =2000;      // позиция разряда изменения частоты (конец свопирования) -6
unsigned char _Poz_Amplitude  =1;      // позиция разряда изменения амплитуды
unsigned int  _Amplitude    =0x1fff9;  // значение текущей амплитуды
unsigned int  _Amplitude_0dB  =1;      // значение опорной амплитуды для пересчета в dB
unsigned int  _Period_Burst   =1000;   // период следования пачки
unsigned char _Poz_Period_Burst =1;    // позиция разряда период следования пачки
unsigned int  _Delay_Burst      =1;    // задержка пачки
unsigned char _Poz_Delay_Burst  =1;    // позиция разряда задержка пачки
unsigned int  _Nperiod_Burst    =1;    // количество периодов в пачке
unsigned char _Poz_Nperiod_Burst=1;    // позиция разряда количество периодов в пачке

extern unsigned int FS ;//частота DDS Hz
extern double dFo1;
extern double Fget_define;

//#define dds_data PORTA->RXTX; //устанавливаем  данные


reg_1508pl1t reg_DDS={      0x1000,//Управление синтезом ЛЧМ.
                            0x1002,//Управление временем переключения параметров синтеза.

                            0x00E1,//Управление синтезом ЛЧМ.
                            0x00E2,//Управление временем переключения параметров синтеза.

                            0x0007,//Управление потоком данных и рандомизацией

                            0x0008,//Делитель чиповой скорости, биты [15: 0]
 							0x0009,//Делитель чиповой скорости, биты [31:16]

                            0x1010, //Регистр длительности 1-ой фазы ЛЧМ-сигнала [15:0]
                            0x1011, //Регистр длительности 1-ой фазы ЛЧМ-сигнала [31:16]
                            0x1012, //Регистр длительности 1-ой фазы ЛЧМ-сигнала [45:32]

                            0x1014, //Регистр длительности 2-ой фазы ЛЧМ-сигнала [15:0]
                            0x1015, //Регистр длительности 2-ой фазы ЛЧМ-сигнала [31:16]
                            0x1016, //Регистр длительности 2-ой фазы ЛЧМ-сигнала [45:32]

                            0x1018, //Регистр длительности 3-ой фазы ЛЧМ-сигнала [15:0]
                            0x1019, //Регистр длительности 3-ой фазы ЛЧМ-сигнала [31:16]
                            0x101A, //Регистр длительности 3-ой фазы ЛЧМ-сигнала [45:32]

                            0x101C,//Регистр длительности 4-ой фазы ЛЧМ-сигнала [15:0]
                            0x101D,//Регистр длительности 4-ой фазы ЛЧМ-сигнала [31:16]
                            0x101E,//Регистр длительности 4-ой фазы ЛЧМ-сигнала [45:32]

                            0x1020,//Регистр начальной частоты ЛЧМ 1 [15:0]
                            0x1021,//Регистр начальной частоты ЛЧМ 1 [31:16]
                            0x1022,//Регистр начальной частоты ЛЧМ 1 [47:32]

                            0x1024,//Регистр начальной частоты ЛЧМ 2 [15:0]
                            0x1025,//Регистр начальной частоты ЛЧМ 2 [31:16]
                            0x1026,//Регистр начальной частоты ЛЧМ 2 [47:32]

                            0x1030,//Регистр начальной фазы ЛЧМ 1
                            0x1031,//Регистр начальной фазы ЛЧМ 2

                            0x1040, //Регистр приращения частоты 1 [15: 0]
                            0x1041, //Регистр приращения частоты 1 [31:16]
                            0x1042, //Регистр приращения частоты 1 [47:32]

                            0x1044, //Регистр приращения частоты 2 [15: 0]
                            0x1045, //Регистр приращения частоты 2 [31:16]
                            0x1046, //Регистр приращения частоты 2 [47:32]

                            0x1300,//Запись приращения фазы [15:0] во все профили
                            0x1301,//Запись приращения фазы [31:16] во все профили
                            0x1302,//Запись приращения фазы [47:32] во все профили

                            0x1304, //Запись смещения фазы во все профили
                            0x1305, //Запись коэффициента усиления во все профили

                            0x1306,//Запись постоянного смещения во все профили

                            0x1400, //Регистр приращения фазы [15: 0], профиль 0
                            0x1401, //Регистр приращения фазы [31:16], профиль 0
                            0x1402, //Регистр приращения фазы [47:32], профиль 0

                            0x1410, //Регистр приращения фазы [15: 0], профиль 1
                            0x1411, //Регистр приращения фазы [31:16], профиль 1
                            0x1412, //Регистр приращения фазы [47:32], профиль 1

                            0x1420, //Регистр приращения фазы [15: 0], профиль 2
                            0x1421, //Регистр приращения фазы [31:16], профиль 2
                            0x1422, //Регистр приращения фазы [47:32], профиль 2

                            0x1430, //Регистр приращения фазы [15: 0], профиль 3
                            0x1431, //Регистр приращения фазы [31:16], профиль 3
                            0x1432, //Регистр приращения фазы [47:32], профиль 3

                            0x1404, //Регистр управления фазой, профиль 0
                            0x1414, //Регистр управления фазой, профиль 1
                            0x1424, //Регистр управления фазой, профиль 2
                            0x1434, //Регистр управления фазой, профиль 3

                            0x1405, //Регистр управления амплитудой, профиль 0
						    0x1415, //Регистр управления амплитудой, профиль 1
							0x1425, //Регистр управления амплитудой, профиль 2
							0x1435, //Регистр управления амплитудой, профиль 3

                            0x1406, //Регистр упр. смещением выходного сигнала, профиль 0
							0x1416, //Регистр упр. смещением выходного сигнала, профиль 1
							0x1426, //Регистр упр. смещением выходного сигнала, профиль 2
							0x1436, //Регистр упр. смещением выходного сигнала, профиль 3

                            0x0005, //Очистка аккумуляторов фазы, запуск и остановка ЛЧМ
                            0x0004, //Управление синхронизацией
                            0x0003, //Регистр управления
                            0x0002, //Выбор активного профиля синтеза
                            0x0001, //Идентификатор устройства, только чтение
                            0x0000, //Регистр программного сброса
							
							0x0006, //Управление LINK-интерфейсом - выключен
							0x1001, //Установка нижней границы корректируемого диапазона частот, старшие 16 бит
							0x1808  //Номер активного профиля

                          }; 



unsigned int delay_us(unsigned int a) //функция задержки
{
	unsigned int t,l;
	unsigned int k;

	k=100*a;

	for (t=0;t<k;t++) l=l+1;

		return l;
}


void Write_DDS(unsigned short data, unsigned short port_adr,ShiftReg *pins)
{
	//запись адреса

	delay_us(1);

	pins->rdn->RXTX|=(1<<(pins->rdn_DDS)); //устанавливаем еденицу на сигнал rdn

	delay_us(1);

	pins->port_data->OE=0xffff; //переводим порт на выход
  
    delay_us(1);
    
    pins->adr->RXTX&=~(1<<(pins->pin_adr)); //устанавливаем ноль на сигнал adr, Сигал ADR выбирает доступ к адресному регистру (ADR=0), либо к данным (ADR=1).
    
    delay_us(1);	
	
	pins->csn->RXTX&=~(1<<(pins->csn_DDS)); //устанавливаем ноль на сигнал csn
    
    delay_us(1);	
	
	pins->port_data->RXTX=port_adr; //устанавливаем значение адреса на шину данных
	
	delay_us(1);

    pins->wrn->RXTX&=~(1<<(pins->wrn_DDS)); //устанавливаем ноль на сигнал wrn

	delay_us(1);

	pins->wrn->RXTX|=(1<<(pins->wrn_DDS)); //устанавливаем еденицу на сигнал wrn

	delay_us(1);

	pins->csn->RXTX|=(1<<(pins->csn_DDS)); //устанавливаем еденицу на сигнал csn

	delay_us(1);

	//запись данных по установленному ранее адресу
    
    pins->adr->RXTX|=(1<<(pins->pin_adr)); //устанавливаем еденицу на сигнал adr, Сигал ADR выбирает доступ к адресному регистру (ADR=0), либо к данным (ADR=1).
	
	delay_us(1);

	pins->csn->RXTX&=~(1<<(pins->csn_DDS)); //устанавливаем ноль на сигнал csn

	delay_us(1);
    
    pins->port_data->RXTX=data; //устанавливаем значение данных на шину данных
   
    delay_us(1);
   
    pins->wrn->RXTX&=~(1<<(pins->wrn_DDS)); //устанавливаем ноль на сигнал wrn
  
	delay_us(1);

	pins->wrn->RXTX|=(1<<(pins->wrn_DDS)); //устанавливаем еденицу на сигнал wrn
	
	delay_us(1);
	
	pins->csn->RXTX|=(1<<(pins->csn_DDS)); //устанавливаем еденицу на сигнал csn

	delay_us(1);

	pins->port_data->RXTX=0xffff; //устанавливаем значение данных на шину данных

}


unsigned short Read_DDS(unsigned short port_adr,ShiftReg *pins)
{

	unsigned short a=0;
	unsigned short b=0;

	//запись адреса

	pins->rdn->RXTX|=(1<<(pins->rdn_DDS)); //устанавливаем еденицу на сигнал rdn

	delay_us(1);

	pins->port_data->OE=0xffff; //переводим порт на выход
    
    pins->csn->RXTX&=~(1<<(pins->csn_DDS)); //устанавливаем ноль на сигнал csn
	
	delay_us(1);

    pins->wrn->RXTX&=~(1<<(pins->wrn_DDS)); //устанавливаем ноль на сигнал wrn

    delay_us(1);
    
    pins->adr->RXTX&=~(1<<(pins->pin_adr)); //устанавливаем ноль на сигнал adr, Сигал ADR выбирает доступ к адресному регистру (ADR=0), либо к данным (ADR=1).
	
    pins->port_data->RXTX=port_adr; //устанавливаем значение адреса на шину данных
         
	delay_us(1);
    
  	pins->wrn->RXTX|=(1<<(pins->wrn_DDS)); //устанавливаем еденицу на сигнал wrn

	delay_us(1);

	pins->csn->RXTX|=(1<<(pins->csn_DDS)); //устанавливаем еденицу на сигнал csn

	pins->adr->RXTX|=(1<<(pins->pin_adr)); //устанавливаем еденицу на сигнал adr, Сигал ADR выбирает доступ к адресному регистру (ADR=0), либо к данным (ADR=1).

	delay_us(1);

	//чтение данных по установленному ранее адресу

	pins->port_data->OE=0x0000; //переводим порт на вход
    
    delay_us(1);
    
    pins->adr->RXTX|=(1<<(pins->pin_adr)); //устанавливаем еденицу на сигнал adr, Сигал ADR выбирает доступ к адресному регистру (ADR=0), либо к данным (ADR=1).
	
	pins->csn->RXTX&=~(1<<(pins->csn_DDS)); //устанавливаем ноль на сигнал csn
	
	pins->rdn->RXTX&=~(1<<(pins->rdn_DDS)); //устанавливаем ноль на сигнал rdn
	
	delay_us(1);

	pins->wrn->RXTX&=~(1<<(pins->wrn_DDS)); //устанавливаем ноль на сигнал wrn

	delay_us(1);

    b=pins->port_data->RXTX; //считываем значение данных с шины данных

	delay_us(1);

	pins->wrn->RXTX|=(1<<(pins->wrn_DDS)); //устанавливаем еденицу на сигнал wrn

	delay_us(1);
    
	pins->rdn->RXTX|=(1<<(pins->rdn_DDS)); //устанавливаем еденицу на сигнал rdn
	
    delay_us(1);
	
	pins->csn->RXTX|=(1<<(pins->csn_DDS)); //устанавливаем еденицу на сигнал csn

	delay_us(1);

	return (a+b);

}


unsigned char init_reg_DDS (unsigned short data, unsigned short port_adr,ShiftReg *pins)
{
	unsigned char a=0;

	Write_DDS(data,port_adr,pins);

 if (data==Read_DDS(port_adr,pins)) a=1; else a=0; //проверка что всё хорошо

 return a;

}


void init_DDS (DDS_param *dds,ShiftReg *pins)
{

	 unsigned char failure=0;
    

     //инициализация ног DDS    

	pins->RSTN->RXTX|=(1<<(pins->RSTN_DDS)); //устанавливаем еденицу на сигнал RSTN

	pins->SCSN->RXTX|=(1<<(pins->SCSN_DDS)); //устанавливаем еденицу на сигнал SCSN - выбор параллельного порта

	pins->wrn ->RXTX|=(1<<(pins->wrn_DDS)); //устанавливаем еденицу на сигнал wrn

	pins->rdn->RXTX|=(1<<(pins->rdn_DDS)); //устанавливаем еденицу на сигнал rdn

	pins->csn->RXTX|=(1<<(pins->csn_DDS)); //устанавливаем еденицу на сигнал csn

	delay_us(1);
	
	//инициализация регистров DDS

  //  if (init_reg_DDS (0x0078,reg_DDS.SWRST,    pins)==0) failure=failure+1; 

    delay_us(10000);

	if (init_reg_DDS (0x1000,reg_DDS.CTR,    pins)==0) failure=failure+1; //Регистр управления
	if (init_reg_DDS (0x0000,reg_DDS.SYNC,   pins)==0) failure=failure+1; //Управление синхронизацией
	if (init_reg_DDS (0x0,reg_DDS.CLR,    pins)==0) failure=failure+1; //Очистка аккумуляторов фазы, запуск и остановка ЛЧМ
	if (init_reg_DDS (0x0000,reg_DDS.SEL_REG,pins)==0) failure=failure+1; //Выбор активного профиля синтеза

 //   if (init_reg_DDS (0x7000,reg_DDS.CH1_Mul_all,  pins)==0) failure=failure+1;   //Запись коэффициента усиления во все профили
	if (init_reg_DDS (0x6000,reg_DDS.CH1_Mul0,     pins)==0) failure=failure+1;     //амплитуда сигнала
	if (init_reg_DDS (0x6000,reg_DDS.CH1_Mul1,     pins)==0) failure=failure+1;     //амплитуда сигнала
	if (init_reg_DDS (0x6000,reg_DDS.CH1_Mul2,     pins)==0) failure=failure+1;     //амплитуда сигнала
	if (init_reg_DDS (0x6000,reg_DDS.CH1_Mul3,     pins)==0) failure=failure+1;     //амплитуда сигнала
 

	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_CTR,   pins)==0) failure=failure+1;     //Управление синтезом ЛЧМ.

	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_TPH1_L,   pins)==0) failure=failure+1;  //Регистр длительности 1-ой фазы ЛЧМ-сигнала [15:0]
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_TPH1_M,   pins)==0) failure=failure+1; 
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_TPH1_H,   pins)==0) failure=failure+1; 

	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_TPH2_L,   pins)==0) failure=failure+1;  //Регистр длительности 2-ой фазы ЛЧМ-сигнала [15:0]
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_TPH2_M,   pins)==0) failure=failure+1; 
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_TPH2_H,   pins)==0) failure=failure+1; 

	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_TPH3_L,   pins)==0) failure=failure+1; //Регистр длительности 3-ой фазы ЛЧМ-сигнала [15:0]
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_TPH3_M,   pins)==0) failure=failure+1; 
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_TPH3_H,   pins)==0) failure=failure+1; 

	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_TPH4_L,   pins)==0) failure=failure+1; //Регистр длительности 4-ой фазы ЛЧМ-сигнала [15:0]
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_TPH4_M,   pins)==0) failure=failure+1; 
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_TPH4_H,   pins)==0) failure=failure+1; 



    if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_F1_L,    pins)==0) failure=failure+1; //Регистр начальной частоты ЛЧМ 1 [15:0]
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_F1_M,    pins)==0) failure=failure+1; 
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_F1_H,    pins)==0) failure=failure+1; 
    
    if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_F2_L,    pins)==0) failure=failure+1; //Регистр начальной частоты ЛЧМ 2 [15:0]
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_F2_M,    pins)==0) failure=failure+1; 
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_F2_H,    pins)==0) failure=failure+1; 

	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_Ph1,     pins)==0) failure=failure+1; //Регистр начальной фазы ЛЧМ 1
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_Ph2,     pins)==0) failure=failure+1; 

    if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_dF1_L,   pins)==0) failure=failure+1; //Регистр приращения частоты 1 [15: 0]
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_dF1_M,   pins)==0) failure=failure+1; 
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_dF1_H,   pins)==0) failure=failure+1; 

	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_dF2_L,   pins)==0) failure=failure+1; //Регистр приращения частоты 2 [15: 0]
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_dF2_M,   pins)==0) failure=failure+1; 
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_dF2_H,   pins)==0) failure=failure+1; 
/*	
	if (init_reg_DDS (0x0000,reg_DDS.CH1_dPh_all_L,   pins)==0) failure=failure+1;  //Запись приращения фазы [15:0] во все профили
	if (init_reg_DDS (0x0000,reg_DDS.CH1_dPh_all_M,   pins)==0) failure=failure+1; 
	if (init_reg_DDS (0x0000,reg_DDS.CH1_dPh_all_H,   pins)==0) failure=failure+1; 
*/
	if (init_reg_DDS (0x0000,reg_DDS.CH1_dPh0_L,   pins)==0) failure=failure+1;     //Регистр приращения фазы [15: 0], профиль 0
	if (init_reg_DDS (0x0000,reg_DDS.CH1_dPh0_M,   pins)==0) failure=failure+1; 
	if (init_reg_DDS (0x0000,reg_DDS.CH1_dPh0_H,   pins)==0) failure=failure+1; 
	
	if (init_reg_DDS (0x0000,reg_DDS.CH1_dPh1_L,   pins)==0) failure=failure+1;     //Регистр приращения фазы [15: 0], профиль 0
	if (init_reg_DDS (0x0000,reg_DDS.CH1_dPh1_M,   pins)==0) failure=failure+1; 
	if (init_reg_DDS (0x0000,reg_DDS.CH1_dPh1_H,   pins)==0) failure=failure+1; 

	if (init_reg_DDS (0x0000,reg_DDS.CH1_dPh2_L,   pins)==0) failure=failure+1;     //Регистр приращения фазы [15: 0], профиль 0
	if (init_reg_DDS (0x0000,reg_DDS.CH1_dPh2_M,   pins)==0) failure=failure+1; 
	if (init_reg_DDS (0x0000,reg_DDS.CH1_dPh2_H,   pins)==0) failure=failure+1; 

	if (init_reg_DDS (0x0000,reg_DDS.CH1_dPh3_L,   pins)==0) failure=failure+1;     //Регистр приращения фазы [15: 0], профиль 0
	if (init_reg_DDS (0x0000,reg_DDS.CH1_dPh3_M,   pins)==0) failure=failure+1; 
	if (init_reg_DDS (0x0000,reg_DDS.CH1_dPh3_H,   pins)==0) failure=failure+1; 

	if (init_reg_DDS (0x0000,reg_DDS.CH1_P0,   pins)==0) failure=failure+1;        //Регистр управления фазой, профиль 0
	if (init_reg_DDS (0x0000,reg_DDS.CH1_P1,   pins)==0) failure=failure+1;        //Регистр управления фазой, профиль 0
	if (init_reg_DDS (0x0000,reg_DDS.CH1_P2,   pins)==0) failure=failure+1;        //Регистр управления фазой, профиль 0
	if (init_reg_DDS (0x0000,reg_DDS.CH1_P3,   pins)==0) failure=failure+1;        //Регистр управления фазой, профиль 0

//	if (init_reg_DDS (0x0000,reg_DDS.CH1_Offset_all,   pins)==0) failure=failure+1;     //Регистр Запись постоянного смещения во все профили
	if (init_reg_DDS (0x0000,reg_DDS.CH1_Offset0,      pins)==0) failure=failure+1;     //Регистр упр. смещением выходного сигнала, профиль 0
	if (init_reg_DDS (0x0000,reg_DDS.CH1_Offset1,      pins)==0) failure=failure+1;     //Регистр упр. смещением выходного сигнала, профиль 0
	if (init_reg_DDS (0x0000,reg_DDS.CH1_Offset2,      pins)==0) failure=failure+1;     //Регистр упр. смещением выходного сигнала, профиль 0
	if (init_reg_DDS (0x0000,reg_DDS.CH1_Offset3,      pins)==0) failure=failure+1;     //Регистр упр. смещением выходного сигнала, профиль 0

	if (init_reg_DDS (0x0000,reg_DDS.LINK,         pins)==0) failure=failure+1;     //Управление LINK-интерфейсом
//	if (init_reg_DDS (0xffff,reg_DDS.CH1_LS_CRFMIN,pins)==0) failure=failure+1;     //Нижняя граница корректируемого диапазона частот
	if (init_reg_DDS (0x0007,reg_DDS.CH1_TSW,pins)==0) failure=failure+1;           //Управление временем переключения параметров синтеза.
    if (init_reg_DDS (0x00D4,reg_DDS.ROUTE,         pins)==0) failure=failure+1;     //Управление LINK-интерфейсом


	 	Transf ("---------------\r\n" );
        Transf ("failure:" );
        itoa_t(failure,strng);
        Transf(strng);
        Transf("\r\n----------------");
        Transf ("\r\n");

}



void reg_DDS_watch(ShiftReg *pins)
{
	unsigned short temp_short=0;
	//   unsigned char  strng[64];

Transf ("//------------------------------\r\n" );

	

        Transf ("DEVID:" );
 temp_short=Read_DDS(reg_DDS.DEVID,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("	//Идентификатор устройства, только чтение");
                         Transf ("\r\n");
                         Transf ("\r\n");

    Transf ("T_SEL_STATE:" );
 temp_short=Read_DDS(reg_DDS.T_SEL_STATE,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("	//Отладочный регистр: текущее состояние выводов SEL");
                         Transf ("\r\n");

  Transf ("T_E_SEL:" );
 temp_short=Read_DDS(reg_DDS.T_E_SEL,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("	//Отладочный регистр: эффективный SEL");
                         Transf ("\r\n");
                         Transf ("\r\n");

        Transf ("CTR:" );
 temp_short=Read_DDS(reg_DDS.CTR,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("	//Регистр управления");
                         Transf ("\r\n");

         Transf ("SEL_REG:" );
 temp_short=Read_DDS(reg_DDS.SEL_REG,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("	//Выбор активного профиля синтеза");
                         Transf ("\r\n");  


         Transf ("SYNC:" );
 temp_short=Read_DDS(reg_DDS.SYNC,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("	//Управление синхронизацией");
                         Transf ("\r\n");  
                         Transf ("\r\n");  

     Transf ("CLR:" );
 temp_short=Read_DDS(reg_DDS.CLR,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("		//Очистка аккумуляторов фазы, запуск и остановка ЛЧМ");
                         Transf ("\r\n");          
                         Transf ("\r\n");               
                       
       Transf ("TC_L:" );
 temp_short=Read_DDS(reg_DDS.TC_L,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("			//Делитель чиповой скорости, биты [15: 0]");
                         Transf ("\r\n");     
                       
       Transf ("TC_H:" );
 temp_short=Read_DDS(reg_DDS.TC_H,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("			//Делитель чиповой скорости, биты [31:16]");
                         Transf ("\r\n");  
                         Transf ("\r\n");   

       Transf ("ROUTE:" );
 temp_short=Read_DDS(reg_DDS.ROUTE,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("			//Управление потоком данных и рандомизацией");
                         Transf ("\r\n");  
                         Transf ("\r\n");     




         Transf ("CH1_Offset0:" );
 temp_short=Read_DDS(reg_DDS.CH1_Offset0,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("	//Регистр упр. смещением выходного сигнала, профиль 0");
                         Transf ("\r\n"); 
						 Transf ("\r\n");  


         Transf ("CH1_Mul0:" );
 temp_short=Read_DDS(reg_DDS.CH1_Mul0,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("		//Регистр управления амплитудой, профиль 0");
                         Transf ("\r\n");          
                         Transf ("\r\n");       


          Transf ("CH1_Mul_all:" );
 temp_short=Read_DDS(reg_DDS.CH1_Mul_all,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("//Регистр управления амплитудой, профиль ALL");
                         Transf ("\r\n");
                         Transf ("\r\n");  



     
         Transf ("CH1_LS_CTR:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_CTR,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("		//Управление синтезом ЛЧМ.");
                         Transf ("\r\n");          
                        
//----------------------------------------------------
          Transf ("CH1_dPh0_L:" );
 temp_short=Read_DDS(reg_DDS.CH1_dPh0_L,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("	//Регистр приращения фазы [15: 0], профиль 0");
                         Transf ("\r\n");  
                       
             Transf ("CH1_dPh0_M:" );
 temp_short=Read_DDS(reg_DDS.CH1_dPh0_M,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf ("\r\n");  
                       
          Transf ("CH1_dPh0_H:" );
 temp_short=Read_DDS(reg_DDS.CH1_dPh0_H,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf ("\r\n");
                         Transf ("\r\n"); 
//---------------------------------------------------
           Transf ("CH1_dPh_all_L:" );
 temp_short=Read_DDS(reg_DDS.CH1_dPh_all_L,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("	//Запись приращения фазы [15:0] во все профили");
                         Transf ("\r\n");  
                       
             Transf ("CH1_dPh_all_M:" );
 temp_short=Read_DDS(reg_DDS.CH1_dPh_all_M,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf ("\r\n");  
                       
          Transf ("CH1_dPh_all_H:" );
 temp_short=Read_DDS(reg_DDS.CH1_dPh_all_H,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf ("\r\n");
                         Transf ("\r\n"); 
//------------------------------------------------
           Transf ("CH1_LS_dF1_L:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_dF1_L,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf(" //Регистр приращения частоты 1 [15: 0]");
                         Transf ("\r\n");  
                       
             Transf ("CH1_LS_dF1_M:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_dF1_M,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf ("\r\n");  
                       
          Transf ("CH1_LS_dF1_H:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_dF1_H,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf ("\r\n");
                         Transf ("\r\n"); 
//-----------------------------------------------
            Transf ("CH1_LS_dF2_L:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_dF2_L,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("//Регистр начальной частоты ЛЧМ 2 [15:0]");
                         Transf ("\r\n");  
                       
             Transf ("CH1_LS_dF2_M:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_dF2_M,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf ("\r\n");  
                       
          Transf ("CH1_LS_dF2_H:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_dF2_H,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf ("\r\n");
                         Transf ("\r\n"); 
//-----------------------------------------------     


            Transf ("CH1_LS_F1_L:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_F1_L,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("//Регистр начальной частоты ЛЧМ 1 [15:0]");
                         Transf ("\r\n");  
                       
             Transf ("CH1_LS_F1_M:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_F1_M,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf ("\r\n");  
                       
          Transf ("CH1_LS_F1_H:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_F1_H,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf ("\r\n"); 
                         Transf ("\r\n");                     
//-----------------------------------------------


            Transf ("CH1_LS_F2_L:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_F2_L,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("//Регистр начальной частоты ЛЧМ 2 [15:0]");
                         Transf ("\r\n");  
                       
             Transf ("CH1_LS_F2_M:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_F2_M,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf ("\r\n");  
                       
          Transf ("CH1_LS_F2_H:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_F2_H,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf ("\r\n"); 
                         Transf ("\r\n");                     
//-----------------------------------------------                       

            Transf ("СH1_LS_TPH1_L:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_TPH1_L,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("//Регистр длительности 1-ой фазы ЛЧМ-сигнала [15:0]");
                         Transf ("\r\n");  
                       
             Transf ("СH1_LS_TPH1_M:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_TPH1_M,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf ("\r\n");  
                       
          Transf ("СH1_LS_TPH1_H:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_TPH1_H,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf ("\r\n"); 
                         Transf ("\r\n");                     

//----------------------------------------------


            Transf ("СH1_LS_TPH2_L:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_TPH2_L,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("//Регистр длительности 2-ой фазы ЛЧМ-сигнала [15:0]");
                         Transf ("\r\n");  
                       
             Transf ("СH1_LS_TPH2_M:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_TPH2_M,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf ("\r\n");  
                       
          Transf ("СH1_LS_TPH2_H:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_TPH2_H,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf ("\r\n");
                         Transf ("\r\n");                      

//----------------------------------------------




            Transf ("СH1_LS_TPH3_L:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_TPH3_L,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("//Регистр длительности 3-ой фазы ЛЧМ-сигнала [15:0]");
                         Transf ("\r\n");  
                       
             Transf ("СH1_LS_TPH3_M:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_TPH3_M,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf ("\r\n");  
                       
          Transf ("СH1_LS_TPH3_H:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_TPH3_H,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf ("\r\n"); 
                         Transf ("\r\n");                     

//----------------------------------------------

            Transf ("СH1_LS_TPH4_L:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_TPH4_L,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("//Регистр длительности 4-ой фазы ЛЧМ-сигнала [15:0]");
                         Transf ("\r\n");  
                       
             Transf ("СH1_LS_TPH4_M:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_TPH4_M,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf ("\r\n");  
                       
          Transf ("СH1_LS_TPH4_H:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_TPH4_H,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf ("\r\n");
                         Transf ("\r\n");                      

//----------------------------------------------

                 
             Transf ("CH1_LS_Ph1:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_Ph1,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("//Регистр начальной фазы ЛЧМ 1");
                         Transf ("\r\n");  
                       
          Transf ("CH1_LS_Ph2:" );
 temp_short=Read_DDS(reg_DDS.CH1_LS_Ph2,pins); 
                itoa_t(temp_short,strng);
                         Transf(strng);
                         Transf("//Регистр начальной фазы ЛЧМ 2");
                         Transf ("\r\n");
                         Transf ("\r\n");                      

//----------------------------------------------



Transf ("//------------------------------\r\n" );


}





void wr_tst (unsigned short data, unsigned short port_adr,ShiftReg *pins)

{

	Write_DDS(data,port_adr,pins);

}

unsigned short rd_tst(unsigned short port_adr,ShiftReg *pins)

{
	unsigned short a;

	a=Read_DDS(port_adr,pins);

	return a;
	
}



void Fs_calc(DDS_param *dds,ShiftReg *pins) // запись текущей частоты (Fs) для профиля 0

{ 
  unsigned long long f;
  unsigned char failure=0;
  double dF,dFs;
  unsigned short x1,x2,x3;
   
   
  
  Transf ("work..\r\n" );
  dFs =    FS;
  dF  = Fget_define - (dds->F0);
  dF = (double)((dF/dFs)* 0xFFFFFFFFFFFF); //Вычисление коеффициента для установки частоты в DDS
  f =  dF;
  
  Transf ("расчёт произведён:\r\n" );
  Transf ("f:" );
  sprintf(strng,"%d",f);
            Transf(strng);
          Transf ("\r\n");			  
		  
  
  x1=(f>> 0)&0xFFFF;
    Transf("L:");
   itoa_t(x1,strng);
    Transf(strng);
  Transf ("\r\n");
		  
  x2=(f>>16)&0xFFFF;
      Transf("M:");
    itoa_t(x2,strng);
      Transf(strng);
   Transf ("\r\n");
  
  x3=(f>>32)&0xFFFF;
      Transf("H:");
    itoa_t(x3,strng);
     Transf(strng);
   Transf ("\r\n");
  
  Transf ("конвертация в 16 битный формат\r\n" );
  
    	
   	if (init_reg_DDS (x1,reg_DDS.CH1_dPh0_L,   pins)==0) failure=failure+1;     //Регистр приращения фазы [15: 0], профиль 0
	if (init_reg_DDS (x2,reg_DDS.CH1_dPh0_M,   pins)==0) failure=failure+1; 
	if (init_reg_DDS (x3,reg_DDS.CH1_dPh0_H,   pins)==0) failure=failure+1; 
	
	//if (init_reg_DDS (0x7000,reg_DDS.CH1_Mul0,     pins)==0) failure=failure+1;     //амплитуда сигнала
	
	    Transf ("--------\r\n" );
        Transf ("failure:" );
        itoa_t(failure,strng);
        Transf(strng);
        Transf("\r\n---------");
        Transf ("\r\n");
     
 }
 
 
void amplitud(DDS_param *dds,ShiftReg *pins)
{
	//----------------------запись амплитуды-------------------------------------------------

    
	unsigned char failure=0;
	float ratio,dbf;
	unsigned short a=0x7ff8;
	unsigned short amplitud;
	
	dbf=dds->A;
	
	ratio= powf (10,dbf/20);
	
Transf ("расчёт произведён:\r\n" );
             Transf ("amplitud:" );
         sprintf(strng,"%f",ratio);
                     Transf(strng);
                   Transf ("\r\n");

	amplitud=a/ratio;	

	 Transf("amplitud:");
    itoa_t(amplitud,strng);
           Transf(strng);
         Transf ("\r\n");
  				   
	//if (init_reg_DDS (amplitud,reg_DDS.CH1_Mul0,     pins)==0) failure=failure+1;     //амплитуда сигнала
	if (init_reg_DDS (amplitud,reg_DDS.CH1_Mul_all,  pins)==0) failure=failure+1;   //Запись коэффициента усиления во все профили

	if (init_reg_DDS (amplitud,reg_DDS.CH1_Mul0,     pins)==0) failure=failure+1;     //амплитуда сигнала  Определяет значение амплитуды в моносигнале
	if (init_reg_DDS (amplitud,reg_DDS.CH1_Mul1,     pins)==0) failure=failure+1;     //амплитуда сигнала  Определяет значение амплитуды в ЛЧМ
	if (init_reg_DDS (amplitud,reg_DDS.CH1_Mul2,     pins)==0) failure=failure+1;     //амплитуда сигнала  Определяет значение амплитуды в ЛЧМ
	if (init_reg_DDS (amplitud,reg_DDS.CH1_Mul3,     pins)==0) failure=failure+1;     //амплитуда сигнала  Определяет значение амплитуды в ЛЧМ
	
	 Transf ("--------\r\n" );
        Transf ("failure:" );
        itoa_t(failure,strng);
        Transf(strng);
        Transf("\r\n---------");
        Transf ("\r\n");
	
	
   
}



void Set_LCHM(DDS_param *dds,ShiftReg *pins){// Calc & Set Positive Linear Sweep Control Word
  
  unsigned long long fstart,fend,time_l;
  long long  step1,step2;
  unsigned char failure=0;
  double dF,dF1,dF2,dF3,dFdelta,dFs,dF_step;
  unsigned short x1,x2,x3;
  unsigned short t1,t2,t3;
  u16  CLR_reg=0;
  u16  LS_CTR_reg=0;
  
  Transf ("work..\r\n" );
  
  dFs =    FS;
  
  dF  = Fget_define-(dds->F_low);
  dF1 = ((dF/dFs)* 0xFFFFFFFFFFFF); //Вычисление коеффициента для установки частоты 1 в DDS
  fstart =  dF1;
  
  Transf ("расчёт произведён:\r\n" );
    
  Transf ("dF1:" );
  sprintf(strng,"%f",dF1);
             Transf(strng);
           Transf ("\r\n");
  
		  
//--------------этап начальных частот--------------------------	  
  x1=(fstart>> 0)&0xFFFF;
     Transf("L:");
  sprintf(strng,"%X",x1);
    Transf(strng);
  Transf ("\r\n");
		  
  x2=(fstart>>16)&0xFFFF;
      Transf("M:");
	    sprintf(strng,"%X",x2);
       Transf(strng);
   Transf ("\r\n");
  
  x3=(fstart>>32)&0xFFFF;
      Transf("H:");
	    sprintf(strng,"%X",x3);
      Transf(strng);
   Transf ("\r\n");
     

    if (init_reg_DDS (x1,reg_DDS.CH1_LS_F1_L,    pins)==0) failure=failure+1; //Регистр начальной частоты ЛЧМ 1 [15:0]
	if (init_reg_DDS (x2,reg_DDS.CH1_LS_F1_M,    pins)==0) failure=failure+1; 
	if (init_reg_DDS (x3,reg_DDS.CH1_LS_F1_H,    pins)==0) failure=failure+1; 		  
  
  Transf ("...\r\n" );

  dF  =Fget_define-(dds->F_high);
  dFs = FS;
  dF2 = ((dF/dFs)* 0xFFFFFFFFFFFF); //Вычисление коеффициента для установки частоты 2 в DDS
  
  fend=dF2;
  
  Transf ("dF2:" );
  sprintf(strng,"%f",dF2);
             Transf(strng);
           Transf ("\r\n");
		  
  /*
	x1=(fend>> 0)&0xFFFF;
    Transf("L:");
	  sprintf(strng,"%X",x1);
     Transf(strng);
  Transf ("\r\n");
		  
  x2=(fend>>16)&0xFFFF;
      Transf("M:");
	    sprintf(strng,"%X",x2);
       Transf(strng);
   Transf ("\r\n");
  
  x3=(fend>>32)&0xFFFF;
      Transf("H:");
	    sprintf(strng,"%X",x3);
       Transf(strng);
   Transf ("\r\n");
   */
   
	if (init_reg_DDS (x1,reg_DDS.CH1_LS_F2_L,    pins)==0) failure=failure+1; //Регистр начальной частоты ЛЧМ 2 [15:0]
	if (init_reg_DDS (x2,reg_DDS.CH1_LS_F2_M,    pins)==0) failure=failure+1; 
	if (init_reg_DDS (x3,reg_DDS.CH1_LS_F2_H,    pins)==0) failure=failure+1; 	   
	
  Transf ("...\r\n" );	
//--------------------------------------------------------------------		   
//--------------этап длительности--------------------------	

        dFs = FS;	   
    dFdelta = dF1 - dF2; // разница частот
        dF3 = (dds->dImp)*(dFs / 4)/(1000000);// количество шагов свопирования _Period_Burst - в мкс
       time_l = dF3; //переводим из дробного в целый
   dF_step  = dFdelta/ time_l;
  	 
	  
 Transf("double шаг dF_step:");
 sprintf(strng,"%f",dF_step);
            Transf(strng);
          Transf ("\r\n");	  
  	
    Transf("длительность time:");
 sprintf(strng,"%f",dF3);
            Transf(strng);
          Transf ("\r\n");
		  
 Transf("разница частот:");
 sprintf(strng,"%f",dFdelta);
            Transf(strng);
          Transf ("\r\n");

		t1=(time_l>> 0)&0xFFFF;
		Transf("L:");
		sprintf(strng,"%X",x1);
		Transf(strng);
	  Transf ("\r\n");
			  
		t2=(time_l>>16)&0xFFFF;
		  Transf("M:");
		 sprintf(strng,"%X",x2);
		  Transf(strng);
	   Transf ("\r\n");
	  
		t3=(time_l>>32)&0xFFFF;
		  Transf("H:");
		 sprintf(strng,"%X",x3);
		 Transf(strng);
		   Transf ("\r\n");
					

		if (init_reg_DDS (t1,reg_DDS.CH1_LS_TPH1_L,   pins)==0) failure=failure+1;//Регистр длительности 1-ой фазы ЛЧМ-сигнала [15:0]
		if (init_reg_DDS (t2,reg_DDS.CH1_LS_TPH1_M,   pins)==0) failure=failure+1; 
		if (init_reg_DDS (t3,reg_DDS.CH1_LS_TPH1_H,   pins)==0) failure=failure+1; 
		
		Transf ("CH1_LS_TPH1...\r\n" );		

  
	
	//расчёт частоты следования импульсов
	
	  Transf("частота следования импульсов:");
			   sprintf(strng,"%u",dds->FImp);
               Transf(strng);
               Transf ("\r\n");
	
	   dF=1000000/(dds->FImp);  // период следования импульсов в мкс
	   dF3 =dF*(dFs / 4)/(1000000);// количество шагов тактовой частоты
	   time_l = dF3-time_l; 	// переводим из дробного в целый
      	
		  	
    Transf("период следования импульсов:");
 sprintf(strng,"%f",dF3);
            Transf(strng);
          Transf ("\r\n");
		
    	x1=(time_l>> 0)&0xFFFF;
		Transf("L:");
		sprintf(strng,"%X",x1);
		Transf(strng);
	    Transf ("\r\n");
			  
		x2=(time_l>>16)&0xFFFF;
		Transf("M:");
		sprintf(strng,"%X",x2);
		Transf(strng);
	    Transf ("\r\n");
	  
		x3=(time_l>>32)&0xFFFF;
		Transf("H:");
		sprintf(strng,"%X",x3);
		Transf(strng);
		Transf ("\r\n");
					
	
	if (init_reg_DDS (x1,reg_DDS.CH1_LS_TPH2_L,   pins)==0) failure=failure+1;//Регистр длительности 2-ой фазы ЛЧМ-сигнала [15:0]
	if (init_reg_DDS (x2,reg_DDS.CH1_LS_TPH2_M,   pins)==0) failure=failure+1; 
	if (init_reg_DDS (x3,reg_DDS.CH1_LS_TPH2_H,   pins)==0) failure=failure+1; 


	Transf ("CH1_LS_TPH2...\r\n" );
	
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_TPH3_L,   pins)==0) failure=failure+1;//Регистр длительности 3-ой фазы ЛЧМ-сигнала [15:0]
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_TPH3_M,   pins)==0) failure=failure+1; 
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_TPH3_H,   pins)==0) failure=failure+1; 

	
    
	Transf ("CH1_LS_TPH3...\r\n" );
	
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_TPH4_L,   pins)==0) failure=failure+1;//Регистр длительности 4-ой фазы ЛЧМ-сигнала [15:0]
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_TPH4_M,   pins)==0) failure=failure+1; 
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_TPH4_H,   pins)==0) failure=failure+1; 

	Transf ("CH1_LS_TPH4...\r\n" );
	

         if ((dds->znak_LCHM)== 1) {step1 = -1*dF_step;}   //условие - если ЛЧМ положительная //шаг прироста частоты
	else if ((dds->znak_LCHM)==-1) {step1 =    dF_step;}   //условие - если ЛЧМ отрицательная //шаг уменьшения частоты

	
	
	 Transf("double шаг dF_step:");
 sprintf(strng,"%f",dF_step);
            Transf(strng);
          Transf ("\r\n");	 
	
	Transf("Направление ЛЧМ принятое:");
     sprintf(strng,"%d",dds->znak_LCHM);
            	 Transf(strng);
               Transf ("\r\n");
	
	          Transf("шаг ЛЧМ :");
     sprintf(strng,"%d",step1);
            	 Transf(strng);
               Transf ("\r\n");
		   
		x1=(step1>> 0)&0xFFFF;
		Transf("L:");
		sprintf(strng,"%X",x1);
		Transf(strng);
	    Transf ("\r\n");
			  
	    x2=(step1>>16)&0xFFFF;
		Transf("M:");
	    sprintf(strng,"%X",x2);
		Transf(strng);
	    Transf ("\r\n");
	  
	    x3=(step1>>32)&0xFFFF;
		Transf("H:");
		sprintf(strng,"%X",x3);
		Transf(strng);
	    Transf ("\r\n");
		
		
		if (init_reg_DDS (x1,reg_DDS.CH1_LS_dF1_L,   pins)==0) failure=failure+1; //Регистр приращения частоты 1 [15: 0]
		if (init_reg_DDS (x2,reg_DDS.CH1_LS_dF1_M,   pins)==0) failure=failure+1; 
		if (init_reg_DDS (x3,reg_DDS.CH1_LS_dF1_H,   pins)==0) failure=failure+1;

		Transf ("CH1_LS_dF1...\r\n" );
	  /*  
		//это блок используется при одновременной работе обоих направлений ЛЧМ, второй набор частот
		
		step2 =  - step1;
	
		Transf("step2:");
		sprintf(strng,"%d",step2);
		Transf(strng);
		Transf ("\r\n");
		
	    x1=(step2>> 0)&0xFFFF;
		Transf("L:");
	    itoa_t(x1,strng);
		Transf(strng);
	    Transf ("\r\n");
			  
	    x2=(step2>>16)&0xFFFF;
		Transf("M:");
		itoa_t(x2,strng);
		Transf(strng);
	    Transf ("\r\n");
	  
	    x3=(step2>>32)&0xFFFF;
		Transf("H:");
		itoa_t(x3,strng);
		Transf(strng);
	    Transf ("\r\n");	

		if (init_reg_DDS (x1,reg_DDS.CH1_LS_dF2_L,   pins)==0) failure=failure+1; //Регистр приращения частоты 2 [15: 0]
		if (init_reg_DDS (x2,reg_DDS.CH1_LS_dF2_M,   pins)==0) failure=failure+1; 
		if (init_reg_DDS (x3,reg_DDS.CH1_LS_dF2_H,   pins)==0) failure=failure+1; 
		
		Transf ("CH1_LS_dF2...\r\n" );
	*/

	
//---------------управление------------------------	

    
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_Ph1,     pins)==0) failure=failure+1; //Регистр начальной фазы ЛЧМ 1
	if (init_reg_DDS (0x0000,reg_DDS.CH1_LS_Ph2,     pins)==0) failure=failure+1; 

    if (init_reg_DDS (0x4000,reg_DDS.SYNC,   pins)==0) failure=failure+1; 
		
	LS_CTR_reg|= (1<<15)  // 1 : включение режима ЛЧМ.
		    //    |(1<<13)  //“1”: установка частоты в начале стадии 3 в значение CHx_LS_F2.
			    |(1<<12)  //“1”: установка частоты в начале стадии 1 в значение CHx_LS_F1.
		//		|(1<<11)  //Сброс фазы в CHx_LS_Ph2 в начале стадии 3
				|(1<<10)  //Сброс фазы в CHx_LS_Ph1 в начале стадии 1.
			//	|(1<< 9)  //“0” выключение сигнала во 2-ой стадии.
			//	|(1<< 9)  //“0” выключение сигнала в 4-ой стадии.
			//    &(~(1<< 7))   //“1”: в ЛЧМ режиме включение кусочно-линейной коррекции сигнала.
			    |(1<< 6)  //1: нулевое приращение фазы в стадии 2
			    |(1<< 5)  //1: нулевое приращение фазы в стадии 4
				|(1<< 4); // Автоповтор ЛЧМ последовательности (переход к стадии 1 по окончании стадии 4).
	
    if (init_reg_DDS (LS_CTR_reg,reg_DDS.CH1_LS_CTR,      pins)==0) failure=failure+1;//Управление синтезом ЛЧМ.
	
		CLR_reg|= (1<<4)  //Запуск (переход к стадии 1) ЛЧМ последовательности в 1-ом канале
			     |(1<<2)  //Установка аккумулятора частоты 1-канала в значение CH1_LS_F1
			     |(1<<0); //Очистка аккумулятора фазы 1-канала
	
	if (init_reg_DDS (CLR_reg  ,reg_DDS.CLR,    pins)==0) failure=failure+1; 
	
	    Transf ("--------\r\n" );
        Transf ("failure:" );
        itoa_t(failure,strng);
        Transf(strng);
        Transf("\r\n---------");
        Transf ("\r\n");
}

