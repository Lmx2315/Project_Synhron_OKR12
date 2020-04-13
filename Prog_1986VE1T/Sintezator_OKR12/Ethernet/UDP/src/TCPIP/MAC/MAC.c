#ifndef __MAC_c__
#define __MAC_c__

#include "opora.h"
#include "MAC.h"
#include "ETHERNET.h"
#include "IP.h"
#include "ARP.h"
#include "ICMP.h"
#include "MemAPI.h"

const Prot_ID_Eth IPIDEth = { 0x08, 0x00 };
const Prot_ID_Eth EthIDEth = { 0x00, 0x01 };
const Prot_ID_Eth ARPIDEth = { 0x08, 0x06 };

extern const MAC_Address MyMAC;


uint32_t EthOutPacket [128];
uint32_t EthInPacket [128];
uint32_t EthOutPacketARP [128];

void* pEthOutPacket;
void* pEthOutData;
uint16_t EthOutPacketSize;
uint16_t EthOutDataSize;
void* pEthInPacket;
void* pEthInData;
uint16_t EthInPacketSize;
uint16_t EthInDataSize;
void* pEthOutPacketARP;
void* pEthOutDataARP;
uint16_t EthOutPacketSizeARP;
uint16_t EthOutDataSizeARP;

void* MallocEthInPacket (uint16_t Size)
{
	pEthInPacket = EthInPacket;
	pEthInData = (void*) ((uint32_t) pEthInData + sizeof (Ethernet_Header) );
	EthInPacketSize = Size;
	return pEthInPacket;
}

void* MallocEthOutData ( uint16_t DataSize )
{
	pEthOutPacket = EthOutPacket;
	pEthOutData = (void*) ( ((uint32_t)pEthOutPacket) + sizeof ( Ethernet_Header ) );
	EthOutDataSize = DataSize;
	EthOutPacketSize = sizeof ( Ethernet_Header ) + EthOutDataSize;
	return pEthOutData;
}

void* MallocEthOutDataARP ( uint16_t DataSize )
{
	pEthOutPacketARP = EthOutPacketARP;
	pEthOutDataARP = (void*) ( (uint32_t) pEthOutPacketARP + sizeof ( Ethernet_Header ) );
	EthOutDataSizeARP = DataSize;
	EthOutPacketSizeARP = sizeof ( Ethernet_Header ) + EthOutDataSizeARP;
	return pEthOutDataARP;
}

void FillEthHeader ( void* pHeader, MAC_Address* SrcMAC, MAC_Address* DstMAC, Prot_ID_Eth ProtID)
{
	MemCpy ( (void*) &(((Ethernet_Header*)pHeader)->SrcMAC), (void*) SrcMAC, sizeof (MAC_Address));
	MemCpy ( (void*) &(((Ethernet_Header*)pHeader)->DstMAC), (void*) DstMAC, sizeof (MAC_Address));
	MemCpy ( (void*) &(((Ethernet_Header*)pHeader)->ProtID), (void*) &ProtID, sizeof (Prot_ID_Eth));
}

void SendEthernetPacket ( void* pHeader, uint16_t Size, MAC_Address* SrcMAC, MAC_Address* DstMAC, Prot_ID_Eth ProtID )
{
	FillEthHeader ( pHeader, SrcMAC, DstMAC, ProtID );
}

void SendViaEthernet (void* pHeader, uint16_t Size, MAC_Address* SrcMAC, MAC_Address* DstMAC, Prot_ID_Eth ProtID )
{
	//PORTD->SETTX = 1 << 10;
	if ( pHeader == pEthOutData )
	{
		FillEthHeader (pEthOutPacket, SrcMAC, DstMAC, ProtID);
		SendViaETHERNET (pEthOutPacket, EthOutPacketSize);
	}
	else if ( pHeader == pEthOutDataARP )
	{
		FillEthHeader (pEthOutPacketARP, SrcMAC, DstMAC, ProtID);
		SendViaETHERNET (pEthOutPacketARP, EthOutPacketSizeARP);
	}
	else
	{
		MemCpy ( MallocEthOutData ( Size ), pHeader, Size );
		FillEthHeader ( pEthOutPacket, SrcMAC, DstMAC, ProtID );
		SendViaETHERNET ( pEthOutPacket, EthOutPacketSize );
	}
	//PORTD->CLRTX = 1 << 10;
}

void ReadEthInPacket (void* pHeader, uint16_t Size)
{
	//PORTD->SETTX = 1 << 7;
	pEthInPacket = pHeader;
	pEthInData = (void*)((uint32_t)pEthInPacket + sizeof(Ethernet_Header));
	EthInPacketSize = Size;
	EthInDataSize = EthInPacketSize - sizeof(Ethernet_Header) - sizeof(Ethernet_Tail);
	if ( !MemCmp( (void*)&(((Ethernet_Header*)pHeader)->ProtID.AsShort), (void*)&(IPIDEth.AsShort), sizeof(uint16_t) )) { 
	   ReadIPInPacket(pEthInData, EthInDataSize); 
	}
	if ( !MemCmp( (void*)&(((Ethernet_Header*)pHeader)->ProtID.AsShort), (void*)&(ARPIDEth.AsShort), sizeof(uint16_t) )) {
	   ReadARPInPacket (pEthInData, EthInDataSize, &(((Ethernet_Header*)pHeader)->SrcMAC) ); 
	}
	//PORTD->CLRTX = 1 << 7;
}

#endif // __MAC_c__
