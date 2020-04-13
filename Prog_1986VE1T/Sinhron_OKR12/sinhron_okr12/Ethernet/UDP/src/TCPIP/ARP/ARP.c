#ifndef __ARP_c__
#define __ARP_c__

#include "opora.h"
#include "MAC.h"
#include "ARP.h"
#include "MemAPI.h"

IP_MAC_Match IPToMACMatches[10];
ARP_Packet* Response;

extern MAC_Address MyMAC;
extern IP_Address MyIP;

const uint16_t Echo_Request = 0x0100;
const uint16_t Echo_Response = 0x0200;

extern Prot_ID_Eth IPIDEth;
extern Prot_ID_Eth ARPIDEth;
extern Prot_ID_Eth EthIDEth;

void* OutARPPocket;
void* InARPPocket;

MAC_Address* GetMACFromIP ( IP_Address* pIP ) {
	uint8_t i;
	for (i=0; i<10; i++) {
		if (!MemCmp ( (void*)&(IPToMACMatches[i]).IP, (void*)pIP, sizeof(IP_Address))) return &(IPToMACMatches[i].MAC);
	}
	return (MAC_Address*) 0;
}

void SetMACToIPMatch ( MAC_Address* pMAC, IP_Address* pIP)
{
	uint8_t FreeRecExists = 0;
	uint8_t ThisIPExists = 0;
	uint8_t MinFreeRec = 0;
	uint8_t i;
	for (i=0; i<10; i++) {
		if ( ( IPToMACMatches[i].IP.AsInt != 0 ) && ( !FreeRecExists ) ) { 
			FreeRecExists = 1;
			MinFreeRec = i;
		}
		if ( IPToMACMatches[i].IP.AsInt == pIP->AsInt ) {
			MemCpy ( (void*)&(IPToMACMatches[i].MAC), (void*)pMAC, sizeof (MAC_Address));
			ThisIPExists = 1;
		}
	}
	if ( !ThisIPExists ) {
		MemCpy ( (void*)&(IPToMACMatches[MinFreeRec].IP), (void*)pIP, sizeof (IP_Address));
		MemCpy ( (void*)&(IPToMACMatches[MinFreeRec].MAC), (void*)pMAC, sizeof (MAC_Address));
	}
}

void SendARPResponse ( IP_Address* pIP, MAC_Address* pDstMAC)
{
	//PORTD->SETTX = 1 << 9;
	Response = ( ARP_Packet* ) MallocEthOutDataARP ( sizeof (ARP_Packet) );
	Response->HLEN = 6;
	Response->PLEN = 4;
	MemCpy ((void*) &(Response->HTYPE), (void*) &EthIDEth, sizeof (Prot_ID_Eth));
	MemCpy ((void*) &(Response->PTYPE), (void*) &IPIDEth, sizeof (Prot_ID_Eth));
	MemCpy ((void*) &(Response->SHA), (void*) &MyMAC, sizeof (MAC_Address));
	MemCpy ((void*) &(Response->SPA), (void*) &MyIP, sizeof (IP_Address));
	MemCpy ((void*) &(Response->THA), (void*) pDstMAC, sizeof (MAC_Address));
	MemCpy ((void*) &(Response->TPA), (void*) pIP, sizeof (IP_Address));
	SendViaEthernet ( (void*) Response, sizeof ( ARP_Packet ), &MyMAC, pDstMAC, ARPIDEth );
	//PORTD->CLRTX = 1 << 9;
}

void ReadARPInPacket ( void* pHeader, uint16_t Size, MAC_Address* SrcMAC )
{
	
	void* Buf = pHeader;
	Prot_ID_Eth HTYPE;
	Prot_ID_Eth PTYPE;
	uint8_t HLEN;
	uint8_t PLEN;
	ARP_Operation OPER;
	MAC_Address SHA;
	IP_Address SPA;
	MAC_Address THA;
	IP_Address TPA;
	
	//PORTD->SETTX = 1 << 8;
	MemCpy ( (void*) &HTYPE, (void*) &(((ARP_Packet*)Buf)->HTYPE), sizeof (Prot_ID_Eth) );
	MemCpy ( (void*) &PTYPE, (void*) &(((ARP_Packet*)Buf)->PTYPE), sizeof (Prot_ID_Eth) );
	MemCpy ( (void*) &HLEN, (void*) &(((ARP_Packet*)Buf)->HLEN), sizeof (HLEN) );
	MemCpy ( (void*) &PLEN, (void*) &(((ARP_Packet*)Buf)->PLEN), sizeof (PLEN) );
	MemCpy ( (void*) &OPER, (void*) &(((ARP_Packet*)Buf)->OPER), sizeof (OPER) );
	MemCpy ( (void*) &SHA, (void*) &(((ARP_Packet*)Buf)->SHA), sizeof (SHA) );
	MemCpy ( (void*) &SPA, (void*) &(((ARP_Packet*)Buf)->SPA), sizeof (SPA) );
	MemCpy ( (void*) &THA, (void*) &(((ARP_Packet*)Buf)->THA), sizeof (THA) );
	MemCpy ( (void*) &TPA, (void*) &(((ARP_Packet*)Buf)->TPA), sizeof (TPA) );

  Buf = pHeader;
	
	if ( !MemCmp ( (void*) &(((ARP_Packet*)Buf)->TPA), (void*) &MyIP, sizeof(IP_Address))) {
		SetMACToIPMatch ( &(((ARP_Packet*)Buf)->SHA), &(((ARP_Packet*)Buf)->SPA) );
		Buf = MallocEthOutDataARP ( sizeof(ARP_Packet));
		MemCpy( (void*) &(((ARP_Packet*)Buf)->HTYPE), (void*) &EthIDEth, sizeof (Prot_ID_Eth));
	  MemCpy( (void*) &(((ARP_Packet*)Buf)->PTYPE), (void*) &IPIDEth, sizeof (Prot_ID_Eth));
	  ((ARP_Packet*)Buf)->HLEN = sizeof (MAC_Address);
	  ((ARP_Packet*)Buf)->PLEN = sizeof (IP_Address);
  	MemCpy( (void*) &(((ARP_Packet*)Buf)->OPER), (void*) &Echo_Response, sizeof (ARP_Operation));
	  MemCpy( (void*) &(((ARP_Packet*)Buf)->SHA), (void*) &MyMAC, sizeof (MAC_Address) );
    MemCpy( (void*) &(((ARP_Packet*)Buf)->SPA), (void*) &MyIP, sizeof (IP_Address) );
	  MemCpy( (void*) &(((ARP_Packet*)Buf)->THA), (void*) &(((ARP_Packet*)pHeader)->SHA), sizeof(MAC_Address));
	  MemCpy( (void*) &(((ARP_Packet*)Buf)->TPA), (void*) &(((ARP_Packet*)pHeader)->SPA), sizeof(MAC_Address));
	  SendViaEthernet ( Buf, sizeof(ARP_Packet), &MyMAC, &(((ARP_Packet*)pHeader)->SHA), ARPIDEth);
	}
	//PORTD->CLRTX = 1 << 8;
}

#endif // __ARP_c__
