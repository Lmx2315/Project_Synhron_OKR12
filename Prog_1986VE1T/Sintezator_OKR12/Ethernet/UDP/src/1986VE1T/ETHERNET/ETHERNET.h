
#ifndef __ETHERNET_h__
#define __ETHERNET_h__

#include "opora.h"
#include "config.h"

typedef struct
{
    unsigned short Data[288];
	  unsigned short Counter;
		unsigned short Status;
} _Rec_Frame;

void SetPHYReg(uint8_t,uint8_t,uint16_t);
uint16_t GetPHYReg(uint8_t,uint8_t);
void PHYInit(uint8_t,uint8_t);
void EthernetConfig(void);
void MACReset(void);
void ClearMemory(void);
uint32_t ReadPacket(_Rec_Frame*);
int	SendPacket(void*, int);

void SendViaETHERNET (void* pBuf, uint16_t Size);
void ReadDataFromR_Buffer (void* pDst, uint16_t Size);
void WriteDataToX_Buffer (void* pSrc, uint16_t Size);
void ShiftX_BufferTail (uint16_t Num);
void ShiftR_BufferHead (uint16_t Num);

typedef volatile union
{
	uint8_t AsBytes[6];
	uint16_t AsShorts[3];
} MAC_Address;

#endif	//__ETHERNET_h__

