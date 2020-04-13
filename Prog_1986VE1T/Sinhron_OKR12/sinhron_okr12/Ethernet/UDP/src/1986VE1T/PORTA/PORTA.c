#ifndef __PORTA_c__
#define __PORTA_c__

#include "opora.h"
#include "PORTA.h"

//--- Ports configuration ---
void PORTAConfig()
{
// 
	PORTA->FUNC = 0x00000000;	// Указание режима работы порта. По 2 бита на каждый вывод. 00 - порт. 01 - основная функция. 10 - альтернативная функция. 11 - переопределённая функция.
//   Таблица 1 - Назначение выводов порта PORTD в цифровом режиме в зависимости от значения регистра PORTD->FUNC.
// ╔═══════════════════╦═══════╦═══════╦═════════╦════════╦═════════╦════════╦═════════╦═════════╦═════════╦═════════╦═════════╦═════════╦════════╦═════════╦═════════╦═══════╗
// ║ 	Режим 	\	 Вывод ║ PA15  ║ PA14  ║  PA13   ║  PA12  ║  PA11   ║  PA10  ║   PA9   ║  PA8    ║   PA7   ║   PA6   ║   PA5   ║   PA4   ║  PA3   ║   PA2   ║   PA1   ║  PA0  ║
// ╠═══════════════════╬═══════╬═══════╬═════════╬════════╬═════════╬════════╬═════════╬═════════╬═════════╬═════════╬═════════╬═════════╬════════╬═════════╬═════════╬═══════╣
// ║ 00 (порт)         ║  -|-  ║  -|-  ║  -|-    ║  -|-   ║  -|-    ║  -|-   ║   -|-   ║   -|-   ║   -|-   ║   -|-   ║   -|-   ║   -|-   ║  -|-   ║   -|-   ║   -|-   ║  -|-  ║
// ╟───────────────────╫───────╫───────╫─────────╫────────╫─────────╫────────╫─────────╫─────────╫─────────╫─────────╫─────────╫─────────╫────────╫─────────╫─────────╫───────╢
// ║ 01 (осн. ф-я)		 ║  D16  ║  D15  ║  D14    ║  D13   ║  D12    ║  D10   ║   D9    ║   D8    ║   D7    ║   D6    ║   D5    ║   D4    ║   D3   ║   D2    ║   D1    ║  D0   ║
// ╟───────────────────╫───────╫───────╫─────────╫────────╫─────────╫────────╫─────────╫─────────╫─────────╫─────────╫─────────╫─────────╫────────╫─────────╫─────────╫───────╢
// ║ 10 (альт. ф-я)		 ║ ETR4  ║ BRK4  ║TMR4_CH4N║TMR4_CH4║TMR4_CH3N║TMR4_CH3║TMR4_CH2N║TMR4_CH2 ║TMR4_CH1N║TMR4_CH1 ║  BRK3   ║  BRK2   ║EXTINT4 ║ EXTINT3 ║EXTINT2  ║EXTINT1║
// ╟───────────────────╫───────╫───────╫─────────╫────────╫─────────╫────────╫─────────╫─────────╫─────────╫─────────╫─────────╫─────────╫────────╫─────────╫─────────╫───────╢
// ║ 11 (переопр. ф-я) ║ PRDD- ║ PRDD+ ║  PRDC-  ║ PRDC+  ║  PRMD-  ║  PRMD+ ║  PRMC-  ║  PRMC+  ║   FTX   ║  FXEN   ║   FSD   ║   FRX   ║  BRK1  ║  ETR3   ║  ETR2   ║ ETR1  ║
// ╚═══════════════════╩═══════╩═══════╩═════════╩════════╩═════════╩════════╩═════════╩═════════╩═════════╩═════════╩═════════╩═════════╩════════╩═════════╩═════════╩═══════╝
	PORTA->ANALOG=0xFFFF;			// Аналоговый/цифровой режим работы порта. 0 - аналоговый. 1 - цифровой.
// У порта A судя по описанию, аналогового режима нет.

	PORTA->RXTX=0;						// Данные порта.
	PORTA->OE =0x0000;
	PORTA->OE|=(1<<7)|(1<<10);	// PA[7] , PA[10] Направление порта. 1 - выход, 0 - вход.
	PORTA->PWR|=0xffffffff;		// Скорость изменения фронта выходного сигнала порта. 2 бита на вывод. 11 - максимально быстрый, 10 - быстрый, 01 - медленный, 00 - зарезервировано.
	// PORTA->PD->PORTSHM = 			// Режим работы входа:  0 - триггер Шмитта выключен, гистерезис 200 мВ. 1 - триггер Шмитта включен, гистерезис 400 мВ.
	// PORTA->PD->PORTPD = 			// Режим работы выхода: 0 - управляемый драйвер, 1 - открытый сток.
}

#endif // __PORTA_c__