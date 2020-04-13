#ifndef __PORTF_c__
#define __PORTF_c__

#include "opora.h"
#include "PORTF.h"

//--- Ports configuration ---
void PORTFConfig()
{
// 
	PORTF->FUNC = 0x00000000;	// Указание режима работы порта. По 2 бита на каждый вывод. 00 - порт. 01 - основная функция. 10 - альтернативная функция. 11 - переопределённая функция.
//   Таблица 1 - Назначение выводов порта PORTD в цифровом режиме в зависимости от значения регистра PORTD->FUNC.
// ╔═══════════════════╦════════╦════════╦════════╦════════╦════════╦════════╦════════╦════════╦════════╦════════╦════════╦════════╦════════╦════════╦════════╦════════╗
// ║ 	Режим 	\	 Вывод ║  PF15  ║  PF14  ║  PF13  ║  PF12  ║  PF11  ║  PF10  ║  PF9   ║  PF8   ║  PF7   ║  PF6   ║  PF5   ║  PF4   ║  PF3   ║  PF2   ║  PF1   ║  PF0   ║
// ╠═══════════════════╬════════╬════════╬════════╬════════╬════════╬════════╬════════╬════════╬════════╬════════╬════════╬════════╬════════╬════════╬════════╬════════╣
// ║ 00 (порт)         ║  -|-   ║  -|-   ║  -|-   ║  -|-   ║  -|-   ║  -|-   ║  -|-   ║  -|-   ║  -|-   ║  -|-   ║  -|-   ║   -|-  ║  -|-   ║  -|-   ║  -|-   ║  -|-   ║
// ╟───────────────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╢
// ║ 01 (осн. ф-я)     ║SSP3_RXD║  OUT2- ║  OUT2+ ║PRD_PRMD║PRD_PRMC║ PRDD-  ║ PRDD+  ║  PRDC- ║ PRDC+  ║ PRMD-  ║ PRMD+  ║ PRMC-  ║ PRMC+  ║PRD_PRMC║PRD_PRMB║PRD_PRMA║
// ╟───────────────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╢
// ║ 10 (альт. ф-я)    ║  A12   ║   A11  ║   A10  ║   A9   ║   A8   ║   A7   ║   A6   ║   A5   ║   A4   ║   A3   ║   A2   ║   A1   ║  A0    ║  A31   ║  A30   ║ READY  ║
// ╟───────────────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╢
// ║ 11 (переопр. ф-я) ║SSP3_TXD║SSP3_FCK║SSP3_FSS║  OUT2- ║  OUT2+ ║ OUT3-  ║ OUT3+  ║ OUT4-  ║ OUT4+  ║TMR1_CH4║TMR1_CH3║TMR1_CH2║TMR1_CH1║  COL   ║  CRS   ║ RXER   ║
// ╚═══════════════════╩════════╩════════╩════════╩════════╩════════╩════════╩════════╩════════╩════════╩════════╩════════╩════════╩════════╩════════╩════════╩════════╝
	PORTF->ANALOG=0xFFFF;			// Аналоговый/цифровой режим работы порта. 0 - аналоговый. 1 - цифровой.
// ╔═══════════════════╦════════╦════════╦════════╦════════╦════════╦════════╦════════╦════════╦════════╦════════╦════════╦════════╦════════╦════════╦═════════╦════════╗
// ║ 	Режим 	\	 Вывод ║  PF15  ║  PF14  ║  PF13  ║  PF12  ║  PF11  ║  PF10  ║  PF9   ║  PF8   ║  PF7   ║  PF6   ║  PF5   ║  PF4   ║  PF3   ║  PF2   ║   PF1   ║  PF0   ║
// ╠═══════════════════╬════════╬════════╬════════╬════════╬════════╬════════╬════════╬════════╬════════╬════════╬════════╬════════╬════════╬════════╬═════════╬════════╣
// ║ 0 (аналоговый)    ║  нет   ║  нет   ║  нет   ║  нет   ║  нет   ║  нет   ║  нет   ║  нет   ║  нет   ║  нет   ║  нет   ║  нет   ║  нет   ║  нет   ║OSC_OUT25║OSC_IN25║
// ╟───────────────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫─────────╫────────╢
// ║ 1 (цифровой)      ║                                                               см. регистр PORTD->FUNC                                                          ║
// ╚═══════════════════╩════════╩════════╩════════╩════════╩════════╩════════╩════════╩════════╩════════╩════════╩════════╩════════╩════════╩════════╩═════════╩════════╝

	PORTF->RXTX=0;						// Данные порта.
	PORTF->OE|=0x7F80;				// Направление порта. 1 - выход, 0 - вход.
	PORTF->PWR|=0x3FFFC000;		// Скорость изменения фронта выходного сигнала порта. 2 бита на вывод. 11 - максимально быстрый, 10 - быстрый, 01 - медленный, 00 - зарезервировано.
//	PORTF->PD->PORTSHM = 			// Режим работы входа:  0 - триггер Шмитта выключен, гистерезис 200 мВ. 1 - триггер Шмитта включен, гистерезис 400 мВ.
//	PORTF->PD->PORTPD = 			// Режим работы выхода: 0 - управляемый драйвер, 1 - открытый сток.
}


#endif // __PORTF_c__
