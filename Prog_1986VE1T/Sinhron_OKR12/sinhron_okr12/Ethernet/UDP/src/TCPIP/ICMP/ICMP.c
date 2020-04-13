#ifndef __ICMP_c__
#define __ICMP_c__

#include "ICMP.h"
#include "IP.h"
#include "ARP.h"
#include "MemAPI.h"

void* pICMPInPacket;
void* pICMPInData;
uint16_t ICMPInDataSize;
uint16_t ICMPInPacketSize;
void* pICMPOutPacket;
void* pICMPOutData;
uint16_t ICMPOutDataSize;
uint16_t ICMPOutPacketSize;

extern const uint8_t ICMPID;

const volatile uint8_t EchoRequest = 8;
const volatile uint8_t EchoReply = 0;

void* MallocICMPOutData ( uint16_t DataSize )
{
	ICMPOutDataSize = DataSize;
	ICMPOutPacketSize = ICMPOutDataSize + sizeof (ICMP_Header);
	pICMPOutPacket = (void*) MallocEthOutData ( ICMPOutPacketSize );
	pICMPOutData = (void*)((uint32_t)pICMPOutPacket + sizeof (ICMP_Header));
	return pICMPOutData;
}

//*** Функция для подсчета контрольной суммы заголовка пакета ICMP-протокола
void FillICMPCheckSum (void* pHeader, uint16_t Size)
{
        unsigned long a, cs=0;
        for(a=0; a<Size; a++)
        {
                if(a==1) continue;
                else cs+= ((uint16_t*)pHeader)[a];
        }
        cs=(cs>>16)+(cs&0xFFFF);
        ((uint16_t*)pHeader)[1] = (uint16_t)(~cs);
}

void ReadICMPInPacket ( IP_Address SrcIP, void* pPacket, uint16_t Size )
{
	//PORTD->SETTX = 1 << 11;
	pICMPInPacket = pPacket;
	pICMPInData = (void*) ( (uint32_t) pPacket + sizeof (ICMP_Header));
	ICMPInPacketSize = Size;
	ICMPInDataSize = ICMPInPacketSize - sizeof (ICMP_Header);
	if (((ICMP_Header* )pPacket)->TYPE == EchoRequest ) ReadICMPEchoInPacket ( SrcIP, pPacket, Size );
	//PORTD->CLRTX = 1 << 11;
}

void ReadICMPEchoInPacket ( IP_Address SrcIP, void* pPacket, uint16_t Size ) {
	void* pICMPEchoInData = (void*)((uint32_t)pICMPInPacket + sizeof( ICMP_Echo_Header) );
	uint16_t ICMPEchoInDataSize = ICMPInPacketSize - sizeof(ICMP_Echo_Header);
	pICMPOutPacket = MallocIPOutData ( Size );
	pICMPOutData = (void*)((uint32_t)pICMPOutPacket + sizeof (ICMP_Echo_Header));
	ICMPOutPacketSize = Size;
	ICMPOutDataSize = ICMPOutPacketSize - sizeof ( ICMP_Echo_Header);
	((ICMP_Echo_Header*)pICMPOutPacket)->TYPE = EchoReply;
	((ICMP_Echo_Header*)pICMPOutPacket)->CODE = 0x00;
	((ICMP_Echo_Header*)pICMPOutPacket)->CHECKSUM = 0x0000;
	((ICMP_Echo_Header*)pICMPOutPacket)->ID = ((ICMP_Echo_Header*)pICMPInPacket)->ID;
	((ICMP_Echo_Header*)pICMPOutPacket)->Num = ((ICMP_Echo_Header*)pICMPInPacket)->Num;
	MemCpy(pICMPOutData, pICMPEchoInData, ICMPEchoInDataSize);
	((ICMP_Echo_Header*)pICMPOutPacket)->CHECKSUM = CheckSum ( 0, pICMPOutPacket, ICMPOutPacketSize );
	// FillICMPCheckSum (pICMPOutPacket, ICMPOutPacketSize);
	SendViaIP ( pICMPOutPacket, ICMPOutPacketSize, SrcIP, ICMPID);
}

void SendViaICMP ( IP_Address DstIP, uint8_t TYPE, uint8_t CODE, void* Data, uint16_t DataSize )
{
	if ( Data != pICMPOutData ) {
		MallocICMPOutData ( DataSize );
		MemCpy ( pICMPOutData, Data, DataSize );
	}
	((ICMP_Header*)pICMPOutPacket)->TYPE = TYPE;
	((ICMP_Header*)pICMPOutPacket)->CODE = CODE;
	FillICMPCheckSum (pICMPOutPacket, ICMPOutPacketSize);
}


#endif // __ICMP_c__
