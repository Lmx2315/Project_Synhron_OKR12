
#ifndef __CONFIG_H
#define __CONFIG_H

#define REVISION_2

#define HSE2_OSCILLATOR

#define SPEED_100M

void ClkConfig(void);
void PortConfig(void);
void SysTickInit(void);

#endif	//__CONFIG_H

