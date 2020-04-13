#ifndef __PORTC_c__
#define __PORTC_c__

#include "opora.h"
#include "PORTC.h"

//--- Ports configuration ---
void PORTCConfig()
{
// 
	PORTC->FUNC = 0x00000000|(0x5<<6)|(0xf<<26); //основная функция порта С - уарт1 и переопределённая функция уарт2

	PORTC->RXTX = 0x0000;
	//PORTC->ANALOG = 0x0000|(0x3<<3)|(0x3<<13); //режим работы - цифровой , ставим еденички для уартов 1 и 2
	PORTC->ANALOG = 0xFFFF; //режим работы - цифровой , ставим еденички для уартов 1 и 2
	PORTC->OE =0x0000;
	PORTC->OE|=0x1|(0x1<<1)|(0x1<<2)|(0x1<<12);				// Направление порта. 1 - выход, 0 - вход. выход 
	PORTC->PWR = 0xffffffff;

	//	По 2 бита на каждый вывод. 00 - порт. 01 - основная функция. 10 - альтернативная функция. 11 - переопределённая функция.
//   Таблица 1 - Назначение выводов порта PORTC в цифровом режиме в зависимости от значения регистра PORTC->FUNC.
// ╔═══════════════════╦════════╦═════════╦═════════╦════════╦════════╦════════╦════════╦════════╦════════╦════════╦════════╦═════════╦═════════╦═════════╦════════╦════════╗
// ║  Режим  \  Вывод  ║  PC15  ║  PC14   ║  PC13   ║  PC12  ║  PC11  ║  PC10  ║  PC9   ║  PC8   ║  PC7   ║  PC6   ║  PC5   ║   PC4   ║   PC3   ║   PC2   ║  PC1   ║  PC0   ║
// ╠═══════════════════╬════════╬═════════╬═════════╬════════╬════════╬════════╬════════╬════════╬════════╬════════╬════════╬═════════╬═════════╬═════════╬════════╬════════╣
// ║ 00 (порт)	 			 ║	-|-   ║	  -|-   ║	  -|-   ║	  -|-  ║  -|-   ║	 -|-   ║	-|-   ║	 -|-   ║	-|-   ║	 -|-   ║	-|-   ║   -|-   ║	 -|-    ║	  -|-   ║	 -|-   ║	 -|-  ║
// ╟───────────────────╫────────╫─────────╫─────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫─────────╫─────────╫─────────╫────────╫────────╢
// ║ 01 (осн. ф-я)		 ║  PRMB+ ║  PRMA-  ║  PRMA+  ║SSP2_FSS║SSP2_SCK║SSP2_RXD║SSP2_TXD║EXTINT4 ║EXTINT3 ║EXTINT2 ║EXTINT1 ║UART_RXD1║UART_TXD1║   ALE   ║  nRD   ║  nWR   ║
// ╟───────────────────╫────────╫─────────╫─────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫─────────╫─────────╫─────────╫────────╫────────╢
// ║ 10 (альт. ф-я)		 ║  BUSY  ║   A31   ║   A30   ║  BE3   ║  BE2   ║  BE1   ║  BE0   ║SSP1_FSS║SSP1_SCK║SSP1_RXD║SSP1_TXD║  BUSY   ║   CLE   ║  CLKO   ║  ETR2  ║  ETR1  ║
// ╟───────────────────╫────────╫─────────╫─────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫────────╫─────────╫─────────╫─────────╫────────╫────────╢
// ║ 11 (переопр. ф-я) ║TMR2_CH1║UART_RXD2║UART_TXD2║CAN_TX2 ║CAN_RX2 ║CAN_TX1 ║ CAN_RX1║  FTX   ║ FXEN   ║SSP1_TXD║SSP1_RXD║ SIR_IN1 ║SIR_OUT1 ║  BRK3   ║  BRK2  ║  BRK1  ║
// ╚═══════════════════╩════════╩═════════╩═════════╩════════╩════════╩════════╩════════╩════════╩════════╩════════╩════════╩═════════╩═════════╩═════════╩════════╩════════╝

	
}


#endif // __PORTC_c__