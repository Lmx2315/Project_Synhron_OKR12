#ifndef __IP_c__
#define __IP_c__

#include "opora.h"
#include "MAC.h"
#include "ARP.h"
#include "UDP.h"
#include "ICMP.h"
#include "MemAPI.h"

#define htons(A) ((((uint16_t)(A) & 0xff00) >> 8) | (((uint16_t)(A) & 0x00ff) << 8))

void* pIPInPacket;
void* pIPInData;
uint16_t IPInPacketSize;
uint16_t IPInDataSize;
void* pIPOutPacket;
void* pIPOutData;
uint16_t IPOutPacketSize;
uint16_t IPOutDataSize;

extern IP_Address MyIP;
extern MAC_Address MyMAC;
extern Prot_ID_Eth IPIDEth;

const volatile uint8_t ICMPID = 0x01;
const volatile uint8_t UDPID = 17;
const IP_Address VoidIP = { 0, 0, 0, 0 };
const IP_Address WideIP = { 255, 255, 255, 255};

//*** Функция для подсчета контрольной суммы заголовка пакета IP-протокола
void FillIPCheckSum ( void* pIPHeader )
{
				// uint16_t cs = 0;
        uint32_t a, cs=0;
        for(a=0; a<10; a++)
        {
                if(a == 5) continue;
                else cs = cs + ((uint16_t*)pIPHeader)[a];
        }
				cs = (cs >> 16) + (cs & 0xFFFF);
        ((uint16_t*)pIPHeader)[5] = (uint16_t)(~cs);
}

void* MallocIPOutData ( uint16_t Size )
{
	IPOutDataSize = Size;
	IPOutPacketSize = IPOutDataSize + sizeof ( IP_Header );
	pIPOutPacket = MallocEthOutData ( IPOutPacketSize );
	pIPOutData = (void*) ( (uint32_t) pIPOutPacket + sizeof ( IP_Header ) );
	return pIPOutData;
}

void ReadIPInPacket ( void* pIPHeader, uint16_t Size )
{
	if ( !MemCmp ((void*) &(((IP_Header*)pIPHeader)->DstIP), (void*) &MyIP, sizeof(IP_Address)) ) {
		pIPInPacket = pIPHeader;
		pIPInData = (void*) ((uint32_t)pIPInPacket + sizeof ( IP_Header));
		MemCpy( (void*) &IPInPacketSize, (void*) &(((IP_Header*)pIPHeader)->PacketLength), sizeof(uint16_t));
		IPInPacketSize = htons(IPInPacketSize);
		IPInDataSize = IPInPacketSize - sizeof(IP_Header);
		if ( ((IP_Header*)pIPHeader)->Protocol == ICMPID ) {
			ReadICMPInPacket ( ((IP_Header*)pIPHeader)->SrcIP, pIPInData, IPInDataSize );
		}
		if ( ((IP_Header*)pIPHeader)->Protocol == UDPID ) {
			ReadUDPInPacket ( ((IP_Header*)pIPHeader)->SrcIP, pIPInData, IPInDataSize );
	  }
  }
}

void FillIPHeader ( void* pHeader, IP_Address DstIP, uint8_t ProtID )
{
	uint16_t tmpPackSize = htons ( IPOutPacketSize);
	((IP_Header*)pHeader)->Version = 4;
	((IP_Header*)pHeader)->IHL = 5;
	((IP_Header*)pHeader)->ServType = 0;
	((IP_Header*)pHeader)->PacketLength = htons (IPOutPacketSize);
	((IP_Header*)pHeader)->ID = 0;
	((IP_Header*)pHeader)->Zero = 0;
	((IP_Header*)pHeader)->DF = 0;
	((IP_Header*)pHeader)->MF = 0;
	((IP_Header*)pHeader)->Offset = 0;
	((IP_Header*)pHeader)->TTL = 128;
	((IP_Header*)pHeader)->Protocol = ProtID;
	((IP_Header*)pHeader)->CheckSum = 0;
	((IP_Header*)pHeader)->SrcIP = MyIP;
	((IP_Header*)pHeader)->DstIP = DstIP;	
}

void SendViaIP ( void* pData, uint16_t Size, IP_Address DstIP, uint8_t ProtID )
{
	MAC_Address* DstMAC;
	if (pData != pIPOutData) {
		MallocIPOutData ( Size );
		MemCpy ( pIPOutData, pData, Size );
	}
	FillIPHeader ( pIPOutPacket, DstIP, ProtID );
	
	((IP_Header*)pIPOutPacket)->CheckSum = CheckSum ( 0, pIPOutPacket, sizeof(IP_Header));
	// FillIPCheckSum ( pIPOutPacket );
	// ((IP_Header*)pIPOutPacket)->CheckSum = 0x0000;
  DstMAC = GetMACFromIP ( &DstIP );
	SendViaEthernet ( pIPOutPacket, IPOutPacketSize, &MyMAC, DstMAC, IPIDEth );
}

#endif // __IP_c__
