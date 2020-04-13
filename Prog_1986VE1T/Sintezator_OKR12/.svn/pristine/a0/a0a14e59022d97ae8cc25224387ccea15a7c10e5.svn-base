#ifndef __UDP_c__
#define __UDP_c__

#include "opora.h"
#include "UDP.h"
#include "Application.h"
#include "MemAPI.h"

#define htons(A) ((((uint16_t)(A) & 0xff00) >> 8) | (((uint16_t)(A) & 0x00ff) << 8))

void* pUDPInPacket;
void* pUDPInData;
uint16_t UDPInPacketSize;
uint16_t UDPInDataSize;
void* pUDPOutPacket;
void* pUDPOutData;
uint16_t UDPOutPacketSize;
uint16_t UDPOutDataSize;

extern const IP_Address MyIP;
extern const uint8_t UDPID;

UDP_Pseudo_Header InUDPFullHeader, OutUDPFullHeader;

void ReadUDPInPacket (IP_Address SrcIP, void* pPacket, uint16_t Size)
{
	//PORTD->SETTX = 1 << 12;
	pUDPInPacket = pPacket;
	pUDPInData = (void*)((uint32_t)pPacket + sizeof(UDP_Header));
	UDPInPacketSize = Size;
	UDPInDataSize = UDPInPacketSize - sizeof(UDP_Header);
	OnUDPReceive ( SrcIP, ((UDP_Header*)pPacket)->SrcPort, ((UDP_Header*)pPacket)->DstPort, pUDPInData, UDPInDataSize );
	//PORTD->CLRTX = 1 << 12;
}

void* MallocUDPOutData (uint16_t Size)
{
	UDPOutDataSize = Size;
  UDPOutPacketSize = UDPOutDataSize + sizeof (UDP_Header);
	pUDPOutPacket = MallocIPOutData ( UDPOutPacketSize );
  pUDPOutData = (void*)((uint32_t)pUDPOutPacket + sizeof(UDP_Header));
	return pUDPOutPacket;
}

void SendViaUDP (IP_Address DstIP, uint16_t DstPort, uint16_t SrcPort, void* Data, uint16_t DataSize)
{
	uint16_t tmpUDPOutPacketSize;
	UDP_Pseudo_Header tmpUDPPseudoHeader;	
	uint16_t CS;
	uint32_t PreCS;
	if ( Data != pUDPOutData ) {
		MallocUDPOutData ( DataSize );
		MemCpy ( pUDPOutData, Data, DataSize );
	}
	tmpUDPOutPacketSize = htons ( UDPOutPacketSize );
	MemCpy( (void*) &(((UDP_Header*)pUDPOutPacket)->DstPort), (void*) &DstPort, sizeof(UDP_Port));
	MemCpy( (void*) &(((UDP_Header*)pUDPOutPacket)->SrcPort), (void*) &SrcPort, sizeof(UDP_Port));
	MemCpy( (void*) &(((UDP_Header*)pUDPOutPacket)->Length), (void*) &tmpUDPOutPacketSize, sizeof(uint16_t)) ;
	((UDP_Header*)pUDPOutPacket)->Checksum = 0x0000;

	MemCpy ( (void*) &(tmpUDPPseudoHeader.SrcIP), (void*) &MyIP, sizeof (IP_Address));
	MemCpy ( (void*) &(tmpUDPPseudoHeader.DstIP), (void*) &DstIP, sizeof (IP_Address));
	tmpUDPPseudoHeader.Zero = 0x00;
	tmpUDPPseudoHeader.Protocol = UDPID;
	tmpUDPPseudoHeader.UDPLength = tmpUDPOutPacketSize;
	
	PreCS = PreCheckSum( 0, (void*)&tmpUDPPseudoHeader, sizeof(UDP_Pseudo_Header));
	CS = CheckSum ( PreCS, pUDPOutPacket, UDPOutPacketSize );
	MemCpy ( (void*) &(((UDP_Header*)pUDPOutPacket)->Checksum), (void*) &CS, sizeof (uint16_t) );
	// ((UDP_Header*)pUDPOutPacket)->Checksum = ChS;
	SendViaIP ( pUDPOutPacket, UDPOutPacketSize, DstIP, UDPID);
}

void FillUDPHeader (void* pPacket)
{
}

void FillUDPCheckSum (void* pPseudoHeader, void* pHeader, uint16_t Size)
{
}

#endif // __UDP_c__
