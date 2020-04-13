#ifndef __MAC_h__
#define __MAC_h__

#include "opora.h"
#include "ETHERNET.h"

#pragma pack(push, 1)
typedef volatile union {
	uint8_t AsBytes[2];
	uint16_t AsShort;
} Prot_ID_Eth;
#pragma pack (pop)

#pragma pack(push, 1)
typedef volatile struct 
{
	MAC_Address DstMAC;
	MAC_Address SrcMAC;
	Prot_ID_Eth ProtID;
} Ethernet_Header;
#pragma pack (pop)

#pragma pack(push, 1)
typedef volatile union
{
	uint8_t AsBytes[4];
	uint16_t AsShorts[2];
	uint32_t AsInt;
} Ethernet_Tail;
#pragma pack (pop)

int	SendPacket(void* buffer, int size);
void ETHERNETConfig(void);

void* MallocEthInPacket (uint16_t Size);
void* MallocEthOutData (uint16_t Size);
void* MallocEthOutDataARP (uint16_t Size);
void ReadEthInPacket (void* pHeader, uint16_t Size);
void ReadDataFromR_Buffer( void* pDst, uint16_t Size);
void SendViaEthernet (void* pHeader, uint16_t Size, MAC_Address* SrcMAC, MAC_Address* DstMAC, Prot_ID_Eth ProtID);
void FillEthHeader ( void* pHeader, MAC_Address* SrcMAC, MAC_Address* DstMAC, Prot_ID_Eth ProtID);

#endif // __MAC_h__
