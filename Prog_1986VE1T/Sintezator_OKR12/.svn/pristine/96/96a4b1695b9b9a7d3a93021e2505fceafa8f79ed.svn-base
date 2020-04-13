#ifndef __UDP_h__
#define __UDP_h__

#include "opora.h"
#include "IP.h"

#pragma pack(push, 2)
typedef struct {
	uint16_t SrcPort;
	uint16_t DstPort;
	uint16_t Length;
	uint16_t Checksum;
} UDP_Header;
#pragma pack (pop)

#pragma pack(push, 1)
typedef struct {
	IP_Address SrcIP;
	IP_Address DstIP;
	uint8_t Zero;
	uint8_t Protocol;
	uint16_t UDPLength;
} UDP_Pseudo_Header;
#pragma pack (pop)

#pragma pack(push, 1)
typedef union {
	uint8_t AsBytes[2];
	uint16_t AsShort;
} UDP_Port;
#pragma pack (pop)

void ReadUDPInPacket (IP_Address SrcIP, void* pPacket, uint16_t Size);
void* MallocUDPOutData (uint16_t Size);
void SendViaUDP (IP_Address DstIP, uint16_t DstPort, uint16_t SrcPort, void* Data, uint16_t DataSize);
void FillUDPHeader (void* pPacket);
void FillUDPCheckSum (void* pPseudoHeader, void* pHeader, uint16_t Size);

#endif // __UDP_h__
