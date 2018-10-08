/* 
 * File:   wireshark.c
 * Author: Erik Friesen
 * MIT license
 * Permission is hereby granted, free of charge, to any person obtaining a 
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation 
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, 
 * and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Created on September 30, 2017, 5:24 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "system_config.h"
#include "xmem.h"
#include "system/common/sys_module.h"
#include "system/fs/sys_fs.h"
#include "wireshark.h"
#include "uart.h"
#include "osal/osal.h"
#include "tcpip/tcpip_mac.h"
#include "driver/ethmac/src/dynamic/_eth_dcpt_lists.h"
#include "FreeRtos.h"
#include "FreeRTOS_CLI.h"
#include "eeprom.h"
#include "app_tasks.h"
#include "logs.h"
#include "system/tmr/sys_tmr.h"

//#include "driver/sqi_flash/sst26/drv_sst26.h"
//#include "flashaddress.h"
 
/*
 * 
 */
#ifdef STACK_USE_WIRESHARK

enum {
    WS_CaptureIdle, WS_OpenUSB, WS_EraseFlash1, WS_EraseFlash2, WS_StartCapture, WS_EndCapture
};

typedef struct {
    unsigned char * data;
    unsigned long len;
} _Packet;

volatile int ActiveCapture = 0;
unsigned long TimeStamper;
static volatile char * PacketBuffer = NULL;
static int Init = 0;
static volatile unsigned int HeadPtr = 0;
static time_t StartTime = 0;
static volatile char PcapFileName[256];
static uint32_t FileSize = 0;
static QueueHandle_t xQueuePacket = NULL;

void WireSharkTasks(void) {
    static char buffer[256];
    static pcap_hdr_t GlobalHeader;
    SYS_FS_HANDLE FH = SYS_FS_HANDLE_INVALID;
    size_t res;
    _Packet pkt;
    uint32_t len, PinnedHead, FilePtr;
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ClearWatchDog(WDT_WIRESHARK);
    SetWatchDogTimeout(WDT_WIRESHARK, 5000, "Wireshark");
    xQueuePacket = xQueueCreate(20, sizeof (_Packet));
    configASSERT(xQueuePacket != NULL);
    while (1) {
	ClearWatchDog(WDT_WIRESHARK);
	switch (ActiveCapture) {
	    case WS_CaptureIdle:
		vTaskDelay(50 / portTICK_PERIOD_MS);
		break;
	    case WS_OpenUSB:
		TimeStamper = SYS_TMR_TickCountGet();
		StartTime = time(NULL);
		HeadPtr = 0;
		FileSize = 0;
		GlobalHeader.magic_number = 0xA1B2C3D4;
		GlobalHeader.version_major = 2;
		GlobalHeader.version_minor = 4;
		GlobalHeader.thiszone = 0;
		GlobalHeader.sigfigs = 0;
		GlobalHeader.snaplen = 65535;
		GlobalHeader.network = 1; //LINKTYPE_ETHERNET
		memcpy((void*) buffer, &GlobalHeader, sizeof (GlobalHeader));
		FH = SYS_FS_FileOpen((const char*) PcapFileName, (SYS_FS_FILE_OPEN_WRITE));
		if (SYS_FS_HANDLE_INVALID != FH) {
		    ActiveCapture = WS_StartCapture;
		    res = SYS_FS_FileWrite(FH, buffer, sizeof (GlobalHeader) );
		    if (res != sizeof (GlobalHeader)) {
			SysLog(LOG_DEBUG, "WS: Unable to write pcap header\r\n");
			ActiveCapture = WS_EndCapture;
		    }
		    FileSize = sizeof (GlobalHeader);
		} else {
		    SysLog(LOG_DEBUG, "WS: Unable to open file for writing\r\n");
		    ActiveCapture = WS_EndCapture;
		}
		break;
	    case WS_StartCapture:

		if (xQueueReceive(xQueuePacket, &pkt, 25 / portTICK_PERIOD_MS)) {
		    if (pkt.data) {
			if (pkt.len > 1680) {
			    SysLog(LOG_DEBUG, "WS: Internal error pkt.len %i\r\n", pkt.len);
			    ActiveCapture = WS_EndCapture;
			} else {
			    
			    res = SYS_FS_FileWrite(FH, pkt.data, pkt.len);
			    if (res != pkt.len) {
				ActiveCapture = WS_EndCapture;
				SysLog(LOG_DEBUG, "WS: USB write error %i != %i\r\n", res, pkt.len);
			    } else {
				FileSize += pkt.len;
			    }
			}
			x_free(pkt.data);
		    }
		}
		break;
	    case WS_EndCapture:
		if (FH != DRV_HANDLE_INVALID) {
		    while (xQueueReceive(xQueuePacket, &pkt, 25 / portTICK_PERIOD_MS)) {
			if (pkt.data) {
			    if (pkt.len > 1680) {
				SysLog(LOG_DEBUG, "WS: Internal error pkt.len %i\r\n", pkt.len);
			    } else {
				res = SYS_FS_FileWrite(FH, pkt.data, pkt.len);
				if (res != pkt.len) {
				    ActiveCapture = WS_EndCapture;
				    SysLog(LOG_DEBUG, "WS: USB write error %i != %i\r\n", res, pkt.len);
				} else {
				    FileSize += pkt.len;
				}
			    }
			    x_free(pkt.data);
			}
		    }
		    (void) SYS_FS_FileClose(FH);
		    FH = SYS_FS_HANDLE_INVALID;
		} else {
		    while (xQueueReceive(xQueuePacket, &pkt, 25 / portTICK_PERIOD_MS)) {
			if (pkt.data) {
			    x_free(pkt.data);
			}
		    }
		}
		SysLog(LOG_NORMAL, "Wireshark File save length %i\r\n", FileSize);
		ActiveCapture = WS_CaptureIdle;
		break;
	}
    }
}

void WireSharkCallBack(const void *pBuff, unsigned short int nBytes) {
    if (ActiveCapture == WS_StartCapture) {
	unsigned int hp;
        pcaprec_hdr_t Hdr;
	unsigned long TotalTime = SYS_TMR_TickCountGet() - TimeStamper;
	Hdr.ts_sec = StartTime + (TotalTime / SYS_TMR_TickCounterFrequencyGet());
	TotalTime %= SYS_TMR_TickCounterFrequencyGet();
	long long T = (long long) TotalTime * 1000000 / SYS_TMR_TickCounterFrequencyGet();
	Hdr.ts_usec = T;
	Hdr.orig_len = nBytes;
	Hdr.incl_len = nBytes;
	_Packet pkt = {0};
	pkt.len = nBytes + sizeof (Hdr);
	pkt.data = x_malloc(pkt.len, XMEM_HEAP_DDR);
	memcpy(pkt.data, &Hdr, sizeof (Hdr));
	memcpy(&pkt.data[sizeof (Hdr)], pBuff, nBytes);
	if (xQueueSendToBack(xQueuePacket, (void *) &pkt, 5 / portTICK_PERIOD_MS) != pdPASS) {
	    /* Failed to post the message, even after 5 ticks. */
	    x_free(pkt.data);
	} 
    }
}

int GetCaptureStats(char* out, int MaxLen) {
    snprintf(out, MaxLen, "\r%bytes captured : %i", FileSize);
    return ActiveCapture > 0;
}

void EndCapture(char* out, int MaxLen) {
    OSAL_CRITSECT_DATA_TYPE status = OSAL_CRIT_Enter(OSAL_CRIT_TYPE_HIGH);
    if (!ActiveCapture)
	return;
    ActiveCapture = WS_EndCapture;
    snprintf(out, MaxLen, "\rWrote %s len %i\r\n", PcapFileName, FileSize);
    OSAL_CRIT_Leave(OSAL_CRIT_TYPE_HIGH, status);
}

void StartCapture(char * FileName, char* out, int MaxLen) {
    OSAL_CRITSECT_DATA_TYPE status = OSAL_CRIT_Enter(OSAL_CRIT_TYPE_HIGH);
    if (ActiveCapture) {
	snprintf(out, MaxLen, "Capture in progress");
	return;
    }
    strcpy((char*)PcapFileName, FileName);
    snprintf(out, MaxLen, "PCAP capture started\r\n");
    ActiveCapture = WS_OpenUSB;
    OSAL_CRIT_Leave(OSAL_CRIT_TYPE_HIGH, status);
}

#endif
