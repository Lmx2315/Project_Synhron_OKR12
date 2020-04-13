#ifndef __ICMP_h__
#define __ICMP_h__

#include "opora.h"
#include "IP.h"

#pragma pack(push, 1)
typedef volatile struct {
	uint8_t TYPE;
	uint8_t CODE;
	uint16_t CHECKSUM;
} ICMP_Header;
#pragma pack (pop)

#pragma pack(push, 1)
typedef volatile struct {
	uint8_t TYPE;
	uint8_t CODE;
	uint16_t CHECKSUM;	
	uint16_t ID;
	uint16_t Num;
} ICMP_Echo_Header;
#pragma pack (pop)

void ReadICMPInPacket (IP_Address SrcIP, void* pPacket, uint16_t Size);
void ReadICMPEchoInPacket (IP_Address SrcIP, void* pPacket, uint16_t Size);
void* MallocICMPOutData (uint16_t Size);
void SendViaICMP (IP_Address DstIP, uint8_t TYPE, uint8_t CODE, void* Data, uint16_t DataSize);
void FillICMPHeader (void* pPacket);
void FillICMPCheckSum (void* pHeader, uint16_t Size);

#endif // __ICMP_h__
