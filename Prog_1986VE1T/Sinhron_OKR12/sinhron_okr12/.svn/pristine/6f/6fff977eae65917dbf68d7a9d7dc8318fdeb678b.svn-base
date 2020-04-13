#ifndef __Application_h__
#define __Application_h__

#include "opora.h"
#include "IP.h"
#include "UDP.h"

#pragma pack (push, 1)
typedef struct {
	uint8_t Byte1;
	uint8_t Byte2;
} Remote_Control_Request;
#pragma pack (pop)

#pragma pack(push, 1)
typedef struct {
	uint8_t LastCommand;
	unsigned LowResist400Hz220V : 1;
	unsigned LowResist50Hz380V : 1;
	unsigned FireAlarm : 1;
	unsigned LocalManage : 1;
	unsigned Reserved : 4;
	unsigned RemoteEngineReady : 1;
	unsigned HasFaults : 1;
	unsigned EngineModeOn : 1;
	unsigned EngineModeOff : 1;
	unsigned RemotePowsupReady : 1;
} Remote_Control_Reply;
#pragma pack (pop)

void OnEthReceive ( void* pIPData, uint16_t Size);
void OnIPReceive ( void* pIPData, uint16_t Size);
void OnARPReceive (void* pARPData, uint16_t Size);
void OnUDPReceive (IP_Address SrcIP, uint16_t SrcPort, uint16_t DstPort, void* pUDPData, uint16_t Size);
void OnICMPReceive (void* pICMPData, uint16_t Size);
void SendEthPacket (void* pHeader, uint16_t Size);

#endif // __Application_h__
