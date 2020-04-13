#ifndef __IP_h__
#define __IP_h__

#include "opora.h"

#pragma pack(push, 1)
typedef volatile union { 
	uint8_t AsBytes[4];
	uint16_t AsShorts[2];
	uint32_t AsInt;
} IP_Address;
#pragma pack (pop)

#pragma pack(push, 1)
 typedef volatile struct  {
	unsigned IHL : 4; 
	unsigned Version : 4;
	uint8_t ServType;
	uint16_t PacketLength;
	uint16_t ID;
	unsigned Offset : 13;
	unsigned MF : 1;	  
	unsigned DF : 1;	 
	unsigned Zero : 1;	 
	uint8_t TTL;
	uint8_t Protocol;
	uint16_t CheckSum;
	IP_Address SrcIP;
	IP_Address DstIP;
} IP_Header;
#pragma pack (pop)

void ReadIPInPacket ( void* pHeader, uint16_t Size );
void SendViaIP ( void* pData, uint16_t Size, IP_Address DstIP, uint8_t ProtID );
void* MallocIPOutData ( uint16_t Size );
void FillIPHeader ( void* pHeader, IP_Address DstIP, uint8_t ProtID );


#endif // __IP_h__
