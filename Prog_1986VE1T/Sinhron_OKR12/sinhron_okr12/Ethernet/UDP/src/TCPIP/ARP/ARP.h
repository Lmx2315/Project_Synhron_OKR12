#ifndef __ARPPROTOCOL_C__
#define __ARPPROTOCOL_C__

#include "MAC.h"
#include "IP.h"

#pragma pack(push, 1)
typedef volatile struct {
	IP_Address IP;
	MAC_Address MAC;
} IP_MAC_Match;
#pragma pack (pop)

#pragma pack(push, 1)
typedef volatile union {
	uint8_t AsBytes[2];
	uint16_t AsShort;
} ARP_Operation;
#pragma pack (pop)

#pragma pack(push, 1)
typedef volatile struct {
	Prot_ID_Eth HTYPE;
	Prot_ID_Eth PTYPE;
	uint8_t HLEN;
	uint8_t PLEN;
	ARP_Operation OPER;
	MAC_Address SHA;
	IP_Address SPA;
	MAC_Address THA;
	IP_Address TPA;
} ARP_Packet;
#pragma pack (pop)

void ReadARPInPacket ( void* pHeader, uint16_t Size, MAC_Address* SrcMAC );
void SendARPResponse ( IP_Address* IP, MAC_Address* MAC );
void SetMACToIPMatch ( MAC_Address* MAC, IP_Address* IP);
MAC_Address* GetMACFromIP ( IP_Address* IP );

#endif // __ARPPROTOCOL_C__
