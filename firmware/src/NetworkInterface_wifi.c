/*******************************************************************************
 *  Network Interface file
 *
 *  Summary:
 *   Network Interface file for FreeRTOS-Plus-TCP stack
 *
 *  Description:
 *   - Interfaces PIC32 to the FreeRTOS TCP/IP stack
 *******************************************************************************/

/*******************************************************************************
 *  File Name:  pic32_NetworkInterface.c
 *  Copyright 2017 Microchip Technology Incorporated and its subsidiaries.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of
 *  this software and associated documentation files (the "Software"), to deal in
 *  the Software without restriction, including without limitation the rights to
 *  use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 *  of the Software, and to permit persons to whom the Software is furnished to do
 *  so, subject to the following conditions:
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE
 *******************************************************************************/

#include <sys/kmem.h>
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "FreeRTOS_DHCP.h"

#include "NetworkInterface.h"
#include "NetworkInterface_wifi.h"
#include "NetworkBufferManagement.h"

#include "system_config.h"

#include "driver/include/m2m_types.h"
#include "driver/include/m2m_wifi.h"

#include "NetworkConfig.h"
#include "common/include/nm_common.h"

#include "system/wdt/sys_wdt.h"
#include "system/tmr/sys_tmr.h"
#include "app_tasks.h"
#include "logs.h"
#include "rnd.h"

#include "GenericTypeDefs.h"
#include "global.h"

#define PACKET_BUFFER_SIZE 1600
#define HOSTNAME "EASYLISTEN"

#define WIFI_BSP_INIT	    (0)
#define WIFI_INIT_START	    (1)
#define WIFI_INIT_READY	    (2)
#define WIFI_INIT_ERR	    (-1)
#define WIFI_INIT_MACERR    (-2)

#define WIFI_RSSI_FREQ (2)//Seconds
#define WIFI_RETRY_INT (30)//Seconds
#define WIFI_AUTO_SCAN_FREQ (60)

#define WIFI_CMD_CONNECT_TO_AP 1
#define WIFI_CMD_RESET_MODULE  2

static uint8_t s_ethRxBuf[PACKET_BUFFER_SIZE] __attribute__((coherent, aligned(16)));
static uint8_t SSIDCON[M2M_MAX_SSID_LEN + 1];

SemaphoreHandle_t HifMutex = NULL;

static void wifi_cb(uint8_t u8MsgType, void *pvMsg);
static void eth_cb(uint8 u8MsgType, void * pvMsg, void * pvCtrlBuf);
void xNetworkFrameReceived(uint32_t len, uint8_t const * const frame);

static volatile int WifiModuleInited = 0;
static volatile int ConnectedToAP = false;
static volatile int ConnectedToAPErr = M2M_ERR_NONE;
static volatile int8_t wifiRSSI = -100;
//Scan variables
static volatile int WifiScanStatus = 0;
static volatile int ScanResultReady = FALSE;
static tstrM2mWifiscanResult * PrivateScanResults;
static tstrM2mWifiscanResult * PublicScanResults;
static SemaphoreHandle_t ScanMutex = NULL;
//static SemaphoreHandle_t ScanMutex;
static volatile int ScanResultCount = 0;
static volatile int ScanResultIndex = 0;
static volatile int DHCPready = 0;
//static volatile int ScanSuccess = FALSE;
extern void WireSharkCallBack(const void *pBuff, unsigned short int nBytes);

static volatile struct {
    int CommandReq;
    int NetworkID;
} WifiConnectRequest;

enum {
    WFC_None, WFC_Connect, WFC_Scan
};

uint32_t ulApplicationGetNextSequenceNumber(uint32_t ulSourceAddress, uint16_t usSourcePort, uint32_t ulDestinationAddress, uint16_t usDestinationPort) {
    (void) ulSourceAddress;
    (void) usSourcePort;
    (void) ulDestinationAddress;
    (void) usDestinationPort;

    return ulRand();
}

static void UpdateWifiState(const char* format, ...) {
    va_list argptr = 0;
    va_start(argptr, format);
    xSemaphoreTake(ConfigMutex, portMAX_DELAY);
    {
	vsnprintf((char*) WifiSetup.Status, sizeof (WifiSetup.Status) - 1, format, argptr);
    }
    xSemaphoreGive(ConfigMutex);
    va_end(argptr);
}

static void UpdateWifiConStatus(const char* format, ...) {
    va_list argptr = 0;
    va_start(argptr, format);
    xSemaphoreTake(ConfigMutex, portMAX_DELAY);
    {
	vsnprintf((char*) WifiSetup.ConStatus, sizeof (WifiSetup.ConStatus) - 1, format, argptr);
    }
    xSemaphoreGive(ConfigMutex);
    va_end(argptr);
}

void NetworkIFinit(void) {
    HifMutex = xSemaphoreCreateMutex();
    configASSERT(HifMutex != NULL);
    ScanMutex = xSemaphoreCreateMutex();
    configASSERT(ScanMutex != NULL);
#ifndef __PRODUCTION
    vTraceSetMutexName(HifMutex, "HifMutex");
#endif
    PrivateScanResults = x_calloc(sizeof (tstrM2mWifiscanResult) * MAX_SCAN_COUNT, 1, XMEM_HEAP_DDR);
    PublicScanResults = x_calloc(sizeof (tstrM2mWifiscanResult) * MAX_SCAN_COUNT, 1, XMEM_HEAP_DDR);
}

void NetworkTasks(void) {
    while (1) {
	ulTaskNotifyTake(pdTRUE, 10 / portTICK_PERIOD_MS);
	if (WifiModuleInited >= WIFI_INIT_START) {
	    if (xSemaphoreTake(HifMutex, 100 / portTICK_PERIOD_MS) == pdTRUE) {
		m2m_wifi_handle_events(NULL);
		(void) xSemaphoreGive(HifMutex);
	    }
	}
    }
}

void WifiManagerTasks(void) {

    enum {
	WMT_INIT, WMT_CONNECT, WMT_CONNECTED, WMT_TRYNEXT, WMT_WAITTRYNEXT, WMT_WAIT_DISCON,
	WMT_CONNERR, WMT_START_SCAN, WMT_WAIT_SCAN, WMT_READ_SCAN,
	WMT_WAIT_READ, WMT_CHECK_CMD, WMT_RESETWILC, WMT_WAITRESET, WMT_ERROR
    };
    int8_t ret;
    int a, val;
    static uint8_t mac0[6];
    static uint8_t mac1[6];
    static int task = 0;
    static _Timer rssi_t = 0;
    static _Timer err_t = 0;
    static _Timer dly_t = 0;
    static _Timer cmd_t = 0;
    static _Timer scan_t = 0;
    static int NetworkIndex = 0;
    static int PreScanTask = 0;
    static int PostDisconnectAction = 0;
    static int PreCommandCheckTask = 0;
    static uint32_t GatewayIP = 0;
    tuniM2MWifiAuth gstrCred1x;
    _Wifi * wificon;
    scan_t = 0 - (SYS_TMR_TickCounterFrequencyGet() * WIFI_AUTO_SCAN_FREQ);
    ClearWatchDog(WDT_WIFIMANAGER);
    SetWatchDogTimeout(WDT_WIFIMANAGER, 1000, "WifiManager");
    while (1) {
	vTaskDelay(10 / portTICK_PERIOD_MS);
	if (xSemaphoreTake(HifMutex, 20 / portTICK_PERIOD_MS) == pdTRUE) {
	    ClearTraceWatchDog(WDT_WIFIMANAGER, task);
	    switch (task) {
		case WMT_INIT:
		    UpdateWifiState("Wifi init");
		    nm_bsp_init();
		    tstrWifiInitParam param;
		    /* Initialize Wi-Fi parameters structure. */
		    memset((uint8_t *) & param, 0, sizeof (tstrWifiInitParam));
		    /* Initialize Wi-Fi driver with data and status callback. */
		    param.pfAppWifiCb = wifi_cb;
		    param.strEthInitParam.pfAppEthCb = eth_cb;
		    param.strEthInitParam.au8ethRcvBuf = s_ethRxBuf;
		    param.strEthInitParam.u16ethRcvBufSize = sizeof (s_ethRxBuf);
		    WifiModuleInited = WIFI_INIT_START;
		    ret = m2m_wifi_init(&param);

		    if (M2M_SUCCESS != ret) {
			SysLog(LOG_DEBUG, "WIFI: m2m_wifi_init error %i\r\n", ret);
			WifiModuleInited = WIFI_INIT_ERR;
			task = WMT_ERROR;
			UpdateWifiState("Wifi init fail");
			break;
		    }
		    ret = m2m_wifi_get_mac_address(mac0, mac1);
		    if (M2M_SUCCESS != ret) {
			SysLog(LOG_DEBUG, "WIFI: m2m_wifi_get_mac_address error %i\r\n", ret);
			WifiModuleInited = WIFI_INIT_MACERR;
			task = WMT_ERROR;
			UpdateWifiState("Wifi mac fail");
			break;
		    }
		    FreeRTOS_UpdateMACAddress(mac0);
		    WifiModuleInited = WIFI_INIT_READY;
		    //Clear any previous auth failures
		    (void) xSemaphoreTake(ConfigMutex, portMAX_DELAY);
		    for (a = 0; a < WifiConfigCount; a++) {
			WifiSetup.Network[a].SecurityFailure = 0;
		    }
		    (void) xSemaphoreGive(ConfigMutex);
		    task = WMT_CONNECT;
		    break;
		case WMT_CONNECT:
		    wificon = (_Wifi*) & WifiSetup.Network[NetworkIndex];
		    DHCPready = GatewayIP = 0;
		    (void) xSemaphoreTake(ConfigMutex, portMAX_DELAY);
		    memset(&gstrCred1x, 0, sizeof (tuniM2MWifiAuth));
		    if (strlen(wificon->SSID) < 1 || wificon->SecurityFailure) {
			task = WMT_TRYNEXT;
			(void) xSemaphoreGive(ConfigMutex);
			break;
		    } else if (strlen(wificon->SSID) > M2M_MAX_SSID_LEN) {
			SysLog(LOG_DEBUG, "WIFI: SSID length > %i\r\n", M2M_MAX_SSID_LEN);
			task = WMT_TRYNEXT;
			(void) xSemaphoreGive(ConfigMutex);
			break;
		    }
		    switch (wificon->Security) {
			case M2M_WIFI_SEC_OPEN:
			    break;
			case M2M_WIFI_SEC_WPA_PSK:
			    if (strlen(wificon->Password) > M2M_MAX_PSK_LEN - 1) {
				SysLog(LOG_DEBUG, "WIFI: WPA PSK password too long\r\n");
				task = WMT_TRYNEXT;
				break;
			    } else if (strlen(wificon->Password) < 1) {
				SysLog(LOG_DEBUG, "WIFI: WPA PSK password too short\r\n");
				task = WMT_TRYNEXT;
				break;
			    }
			    strcpy(gstrCred1x.au8PSK, wificon->Password);
			    break;
			case M2M_WIFI_SEC_WEP:
			    val = strlen(wificon->Password);
			    gstrCred1x.strWepInfo.u8KeyIndx = 1;
			    if (val == WEP_104_KEY_STRING_SIZE) {
				memcpy(gstrCred1x.strWepInfo.au8WepKey, wificon->Password, WEP_104_KEY_STRING_SIZE + 1);
				gstrCred1x.strWepInfo.u8KeySz = WEP_104_KEY_STRING_SIZE + 1;
				break;
			    } else if (val == WEP_40_KEY_STRING_SIZE) {
				memcpy(gstrCred1x.strWepInfo.au8WepKey, wificon->Password, WEP_40_KEY_STRING_SIZE + 1);
				gstrCred1x.strWepInfo.u8KeySz = WEP_40_KEY_STRING_SIZE + 1;
			    } else {
				SysLog(LOG_DEBUG, "WIFI: WEP password must be 10 or 24 length\r\n");
				task = WMT_TRYNEXT;
				break;
			    }
			    break;
			case M2M_WIFI_SEC_802_1X:
			    if (strlen(wificon->Username) > M2M_1X_USR_NAME_MAX - 1) {
				SysLog(LOG_DEBUG, "WIFI: 802.1x username too long\r\n");
				task = WMT_TRYNEXT;
				break;
			    } else if (strlen(wificon->Username) < 1) {
				SysLog(LOG_DEBUG, "WIFI: 802.1x username too short\r\n");
				task = WMT_TRYNEXT;
				break;
			    }
			    strcpy(gstrCred1x.strCred1x.au8UserName, wificon->Username);
			    if (strlen(wificon->Password) > M2M_1X_PWD_MAX - 1) {
				SysLog(LOG_DEBUG, "WIFI: 802.1x password too long\r\n");
				task = WMT_TRYNEXT;
				break;
			    } else if (strlen(wificon->Password) < 1) {
				SysLog(LOG_DEBUG, "WIFI: 802.1x password too short\r\n");
				task = WMT_TRYNEXT;
				break;
			    }
			    strcpy(gstrCred1x.strCred1x.au8Passwd, wificon->Password);
			    break;
			default:
			    SysLog(LOG_DEBUG, "WIFI: Invalid Security type %i\r\n", wificon->Security);
			    task = WMT_TRYNEXT;
			    break;
		    }
		    if (task != WMT_TRYNEXT) {
			//Pre validated information in wificon
			ConnectedToAP = true;
			ret = m2m_wifi_connect(wificon->SSID, strlen(wificon->SSID), wificon->Security, & gstrCred1x, M2M_WIFI_CH_ALL);
			if (M2M_SUCCESS != ret) {
			    strcpy(SSIDCON, "No connection");
			    SysLog(LOG_DEBUG, "WIFI: m2m_wifi_connect error %i\r\n", ret);
			    task = WMT_RESETWILC;
			    PostDisconnectAction = WMT_CONNECT;
			    ConnectedToAP = false;
			} else {
			    strcpy(SSIDCON, wificon->SSID);
			    SysLog(LOG_DEBUG, "WIFI: m2m_wifi_connect success\r\n");
			    task = WMT_CONNECTED;
			}
		    }
		    (void) xSemaphoreGive(ConfigMutex);
		    if (task == WMT_CONNECTED) {
			UpdateWifiState("Wifi net #%i", NetworkIndex + 1);
		    }
		    break;
		case WMT_CONNECTED:
		    if (ConnectedToAP == false) {
			(void) xSemaphoreTake(ConfigMutex, portMAX_DELAY);
			{
			    if (ConnectedToAPErr == M2M_ERR_AUTH_FAIL) {
				WifiSetup.Network[NetworkIndex].SecurityFailure = TRUE;
				SysLog(LOG_DEBUG, "WIFI: SSID %s auth failure\r\n", WifiSetup.Network[NetworkIndex].SSID);

			    } else if (ConnectedToAPErr == M2M_ERR_SEC_CNTRMSR) {
				WifiSetup.Network[NetworkIndex].SecurityFailure = TRUE;
				SysLog(LOG_DEBUG, "WIFI: SSID %s auth type failure\r\n", WifiSetup.Network[NetworkIndex].SSID);
			    } else if (ConnectedToAPErr == M2M_ERR_AP_NOT_FOUND) {
				SysLog(LOG_DEBUG, "WIFI: SSID %s AP not found\r\n", WifiSetup.Network[NetworkIndex].SSID);
			    } else {
				SysLog(LOG_DEBUG, "WIFI: SSID %s disconnected err %i\r\n", WifiSetup.Network[NetworkIndex].SSID, ConnectedToAPErr);
			    }
			}
			(void) xSemaphoreGive(ConfigMutex);
			//UpdateWifiState handles the mutex itself
			if (ConnectedToAPErr == M2M_ERR_AUTH_FAIL) {
			    UpdateWifiState("Wifi #%i auth err", NetworkIndex + 1);
			} else if (ConnectedToAPErr == M2M_ERR_SEC_CNTRMSR) {
			    UpdateWifiState("Wifi #%i auth err", NetworkIndex + 1);
			} else {
			    UpdateWifiState("Wifi #%i offline", NetworkIndex + 1);
			}
			task = WMT_TRYNEXT;
			break;
		    }
		    if (GatewayIP == 0 && DHCPready) {
			GatewayIP = FreeRTOS_GetGatewayAddress();
			if (GatewayIP) {
			    UpdateWifiState("AP #%i", NetworkIndex + 1);
			} else {
			    UpdateWifiState("DHCP error");
			}
		    }
		    //  RSSI request timer
		    if (SYS_TMR_TickCountGet() - rssi_t > SYS_TMR_TickCounterFrequencyGet() * WIFI_RSSI_FREQ) {
			rssi_t = SYS_TMR_TickCountGet();
			m2m_wifi_req_curr_rssi();
			break;
		    }
		    //Other commands
		    if (SYS_TMR_TickCountGet() - cmd_t > SYS_TMR_TickCounterFrequencyGet() / 10) {
			cmd_t = SYS_TMR_TickCountGet();
			PreCommandCheckTask = WMT_CONNECTED;
			task = WMT_CHECK_CMD;
			break;
		    }
		    break;
		case WMT_TRYNEXT:
		    wifiRSSI = -100;
		    dly_t = SYS_TMR_TickCountGet();
		    task = WMT_WAITTRYNEXT;
		    break;
		case WMT_WAITTRYNEXT:
		    if (SYS_TMR_TickCountGet() - dly_t > SYS_TMR_TickCounterFrequencyGet() * 2) {
			if (NetworkIndex == WifiConfigCount - 1) {
			    NetworkIndex = 0;
			    task = WMT_CONNERR;
			    SysLog(LOG_DEBUG, "WIFI: Tried all active connections\r\n");
			    UpdateWifiState("No Wifi");
			    err_t = SYS_TMR_TickCountGet();
			} else {
			    NetworkIndex++;
			    (void) xSemaphoreTake(ConfigMutex, portMAX_DELAY);
			    {
				wificon = (_Wifi*) & WifiSetup.Network[NetworkIndex];
				if (strlen(wificon->SSID) > 0 && !wificon->SecurityFailure) {
				    task = WMT_CONNECT;
				}
			    }
			    (void) xSemaphoreGive(ConfigMutex);
			}
		    }
		    break;
		case WMT_WAIT_DISCON:
		    if (ConnectedToAP == false) {
			task = PostDisconnectAction;
		    }
		    break;
		case WMT_CONNERR:
		    if (SYS_TMR_TickCountGet() - cmd_t > SYS_TMR_TickCounterFrequencyGet() / 10) {
			cmd_t = SYS_TMR_TickCountGet();
			PreCommandCheckTask = WMT_CONNERR;
			task = WMT_CHECK_CMD;
			break;
		    }
		    if (SYS_TMR_TickCountGet() - err_t > SYS_TMR_TickCounterFrequencyGet() * WIFI_RETRY_INT) {
			task = WMT_CONNECT;
		    }
		    break;
		case WMT_START_SCAN:
		    ScanResultReady = FALSE;
		    ret = m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
		    memset(PrivateScanResults, 0, sizeof (tstrM2mWifiscanResult) * MAX_SCAN_COUNT);
		    if (M2M_SUCCESS != ret) {
			SysLog(LOG_DEBUG, "WIFI: Scan request failed err %i\r\n", ret);
			task = WMT_RESETWILC;
			(void) xSemaphoreTake(ScanMutex, portMAX_DELAY);
			{
			    ScanResultCount = 0;
			}
			(void) xSemaphoreGive(ScanMutex);
		    } else {
			task = WMT_WAIT_SCAN;
			scan_t = SYS_TMR_TickCountGet();
		    }
		    break;
		case WMT_WAIT_SCAN:
		    if (ScanResultReady) {
			(void) xSemaphoreTake(ScanMutex, portMAX_DELAY);
			{
			    ScanResultCount = ScanResultIndex;
			    if (ScanResultIndex > 0) {
				memcpy(PublicScanResults, PrivateScanResults, sizeof (tstrM2mWifiscanResult) * ScanResultCount);
			    }
			}
			(void) xSemaphoreGive(ScanMutex);
			task = PreScanTask;
		    } else {
			if (SYS_TMR_TickCountGet() - scan_t > SYS_TMR_TickCounterFrequencyGet()) {
			    task = PreScanTask; //Give up, probably some unknown firmware issue on the wilc1000
			    (void) xSemaphoreTake(ScanMutex, portMAX_DELAY);
			    {
				ScanResultCount = 0;
			    }
			    (void) xSemaphoreGive(ScanMutex);
			}
		    }
		    break;
		case WMT_CHECK_CMD:
		    task = PreCommandCheckTask;
		    if (WifiConnectRequest.CommandReq == WIFI_CMD_CONNECT_TO_AP) {
			WifiConnectRequest.CommandReq = 0;
			if (WifiConnectRequest.NetworkID != NetworkIndex) {
			    if (WifiConnectRequest.NetworkID >= WifiConfigCount) {
				SysLog(LOG_DEBUG, "WIFI: Param > WifiConfigCount\r\n");
				break;
			    }
			    NetworkIndex = WifiConnectRequest.NetworkID;
			    (void) xSemaphoreTake(ConfigMutex, portMAX_DELAY);
			    {
				WifiSetup.Network[NetworkIndex].SecurityFailure = FALSE; //Clear this
				SysLog(LOG_DEBUG, "WIFI: SSID connecting to %s\r\n", WifiSetup.Network[NetworkIndex].SSID);
			    }
			    (void) xSemaphoreGive(ConfigMutex);
			    ret = m2m_wifi_disconnect();
			    if (M2M_SUCCESS != ret) {
				SysLog(LOG_DEBUG, "WIFI: m2m_wifi_disconnect err %i\r\n", ret);
			    }
			    task = WMT_WAIT_DISCON;
			    PostDisconnectAction = WMT_CONNECT;
			} else {
			    (void) xSemaphoreTake(ConfigMutex, portMAX_DELAY);
			    {
				SysLog(LOG_DEBUG, "WIFI: SSID %s already connect\r\n", WifiSetup.Network[NetworkIndex].SSID);
			    }
			    (void) xSemaphoreGive(ConfigMutex);
			}
			break;
		    } else if (WifiConnectRequest.CommandReq == WIFI_CMD_RESET_MODULE) {
			WifiConnectRequest.CommandReq = 0;
			task = WMT_RESETWILC;
			PostDisconnectAction = WMT_CONNECT;
		    }
		    if (SYS_TMR_TickCountGet() - scan_t > SYS_TMR_TickCounterFrequencyGet() * WIFI_AUTO_SCAN_FREQ) {
			scan_t = SYS_TMR_TickCountGet();
			(void) xSemaphoreTake(ConfigMutex, portMAX_DELAY);
			if (WifiSetup.AutoScan && !PlayerSetup.Ctrl_StreamIn) {
			    PreScanTask = PreCommandCheckTask;
			    task = WMT_START_SCAN;
			}
			(void) xSemaphoreGive(ConfigMutex);
		    }
		    break;
		case WMT_RESETWILC:
		    wifiRSSI = -100;
		    SysLog(LOG_DEBUG, "WIFI: Resetting wifi module\r\n", ret);
		    task = WMT_WAITRESET;
		    ret = m2m_wifi_disconnect();
		    err_t = SYS_TMR_TickCountGet();
		    if (M2M_SUCCESS != ret) {
			SysLog(LOG_DEBUG, "WIFI: m2m_wifi_disconnect err %i\r\n", ret);
		    }
		    ret = m2m_wifi_deinit(NULL);
		    if (ret != M2M_SUCCESS) {
			task = PostDisconnectAction;
			SysLog(LOG_DEBUG, "WIFI: m2m_wifi_deinit err %i\r\n", ret);
			break;
		    }
		    WifiModuleInited = WIFI_BSP_INIT;
		    FreeRTOS_NetworkDown();
		    ret = nm_bsp_deinit();
		    if (ret != M2M_SUCCESS) {
			task = WMT_ERROR;
			SysLog(LOG_DEBUG, "WIFI: nm_bsp_deinit err %i\r\n", ret);
			UpdateWifiState("Wifi rst fail");
			break;
		    }
		    break;
		case WMT_WAITRESET:
		    if (SYS_TMR_TickCountGet() - err_t > SYS_TMR_TickCounterFrequencyGet() * 2) {
			task = WMT_INIT;
		    }
		    break;
		case WMT_ERROR:
		    //End of the road, wifi hardware has an issue.
		    break;
	    }
	    (void) xSemaphoreGive(HifMutex);
	}
    }
}

static void wifi_cb(uint8_t u8MsgType, void *pvMsg) {
    int8_t ret;
    tstrM2MConnInfo s_connInfo;
    switch (u8MsgType) {
	case M2M_WIFI_RESP_SCAN_DONE:
	{
	    tstrM2mScanDone *pstrInfo = (tstrM2mScanDone*) pvMsg;
	    ScanResultIndex = 0;
	    if (pstrInfo->s8ScanState == M2M_SUCCESS) {
		if (pstrInfo->u8NumofCh >= 1) {
		    m2m_wifi_req_scan_result(ScanResultIndex);
		} else {
		    ScanResultReady = TRUE;
		}
	    } else {
		SysLog(LOG_DEBUG, "WIFI: Scan failed err %i\r\n", pstrInfo->s8ScanState);
		ScanResultReady = TRUE;
	    }
	}
	    break;
	case M2M_WIFI_RESP_SCAN_RESULT:
	{
	    tstrM2mWifiscanResult *pstrScanResult = (tstrM2mWifiscanResult*) pvMsg;
	    uint8 u8NumFoundAPs = m2m_wifi_get_num_ap_found();
	    memcpy((void *) &PrivateScanResults[ScanResultIndex], (void *) pvMsg, sizeof (tstrM2mWifiscanResult));
	    ScanResultIndex++;
	    if (ScanResultIndex < u8NumFoundAPs && ScanResultIndex < MAX_SCAN_COUNT - 1) {
		// Read the next scan result
		if(m2m_wifi_req_scan_result(ScanResultIndex)!= M2M_SUCCESS){
		    ScanResultReady = TRUE;
		    ScanResultIndex = 0;
		}	
	    } else {
		ScanResultReady = TRUE;
	    }
	}
	    break;
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
	    tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *) pvMsg;
	    if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
		ConnectedToAP = true;
		ConnectedToAPErr = pstrWifiState->u8ErrCode;
		FreeRTOS_NetworkDown();
	    } else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
		ConnectedToAPErr = pstrWifiState->u8ErrCode;
		FreeRTOS_NetworkDown();
		ConnectedToAP = false;
		SysLog(LOG_DEBUG, "WIFI: Disconnected\r\n");
	    }
	    break;
	}
	case M2M_WIFI_RESP_CONN_INFO:
	    memcpy((void *) &s_connInfo, (void *) pvMsg, sizeof (tstrM2MConnInfo));
	    wifiRSSI = s_connInfo.s8RSSI;
	    SysLog(LOG_DEBUG, "WIFI: Connected to SSID:%s\r\n", s_connInfo.acSSID);
	    break;
	case M2M_WIFI_RESP_CURRENT_RSSI:
	    wifiRSSI = *(int8_t *) pvMsg;	    
	    break;

	case M2M_WIFI_RESP_FIRMWARE_STRTED:
	    SysLog(LOG_DEBUG, "WIFI: WILC firmware loaded ver: %u.%u.%u\r\n", M2M_DRIVER_VERSION_MAJOR_NO, M2M_DRIVER_VERSION_MINOR_NO, M2M_DRIVER_VERSION_PATCH_NO);
	    break;

	default:
	    SysLog(LOG_DEBUG, "WIFI: Invalid Callback ID");
	    break;
    }
}

static void eth_cb(uint8 u8MsgType, void * pvMsg, void * pvCtrlBuf) {
    if (u8MsgType == M2M_WIFI_RESP_ETHERNET_RX_PACKET) {
	tstrM2MDataBufCtrl *ctrl = (tstrM2MDataBufCtrl *) pvCtrlBuf;
	xNetworkFrameReceived(ctrl->u16DataSize, (uint8_t *) pvMsg);
	WireSharkCallBack((uint8_t *) pvMsg, ctrl->u16DataSize);
	m2m_wifi_set_receive_buffer(s_ethRxBuf, sizeof (s_ethRxBuf));
    }
}

BaseType_t xNetworkIFReady(void) {
    return ConnectedToAP && FreeRTOS_GetGatewayAddress();
}

BaseType_t xWifiRSSI(void) {
    //Convert to % signal from dBm
    if (wifiRSSI<-90) {
	return 0;
    } else if (wifiRSSI<-80) {
	return 25;
    } else if (wifiRSSI<-70) {
	return 50;
    } else if (wifiRSSI<-60) {
	return 75;
    } else  {
	return 100;
    }
}

BaseType_t xRawWifiRSSI(void) {
    return wifiRSSI;
}

int xGetWifiScanResults(tstrM2mWifiscanResult * data) {
    int ScanCount = 0;
    if (ScanMutex == NULL) {
	return 0;
    }
    (void) xSemaphoreTake(ScanMutex, portMAX_DELAY);
    {
	ScanCount = ScanResultCount;
	memcpy(data, PublicScanResults, sizeof (tstrM2mWifiscanResult) * ScanResultCount);
    }
    (void) xSemaphoreGive(ScanMutex);
    return ScanCount;
}

/*
 * Attempt to connect to an access point
 */
BaseType_t xConnectToSpecificAP(BaseType_t index){
    WifiConnectRequest.CommandReq = WIFI_CMD_CONNECT_TO_AP;
    WifiConnectRequest.NetworkID = index;
}

/*
 * Reset the wifi module
 */
BaseType_t xResetWifiModule(void){
    WifiConnectRequest.CommandReq = WIFI_CMD_RESET_MODULE;
    WifiConnectRequest.NetworkID = 0;
}

/*
 * Null terminated SSID string
 */
char * SSIDINFO(void){
    return SSIDCON;
}

/* local definitions and data */

/* FreeRTOS implementation functions */
BaseType_t xNetworkInterfaceInitialise(void) {

    if (WifiModuleInited != WIFI_INIT_READY || !ConnectedToAP) {
	return pdFAIL;
    }
    return pdPASS;
}

/*-----------------------------------------------------------*/

BaseType_t xNetworkInterfaceOutput(NetworkBufferDescriptor_t * const pxDescriptor, BaseType_t xReleaseAfterSend) {
    BaseType_t retRes = pdFALSE;
    if (WifiModuleInited != WIFI_INIT_READY) {
	//Don't send anything until ready
    } else if ((pxDescriptor != 0) && (pxDescriptor->pucEthernetBuffer != 0) && (pxDescriptor->xDataLength != 0)) {
	/* There you go */
	(void) xSemaphoreTake(HifMutex, portMAX_DELAY);
	if (m2m_wifi_send_ethernet_pkt(pxDescriptor->pucEthernetBuffer, pxDescriptor->xDataLength, STATION_INTERFACE) == M2M_SUCCESS) {
	    retRes = pdTRUE;
	}
	(void) xSemaphoreGive(HifMutex);
	if (retRes) {
	    WireSharkCallBack(pxDescriptor->pucEthernetBuffer, pxDescriptor->xDataLength);
	}
	/* The buffer has been sent so can be released. */
	if (xReleaseAfterSend != pdFALSE) {
	    vReleaseNetworkBufferAndDescriptor(pxDescriptor);
	}
    }

    return retRes;
}


/************************************* Section: helper functions ************************************************** */
/* */



/************************************* Section: worker code ************************************************** */

/* */

void xNetworkFrameReceived
(uint32_t len, uint8_t const * const frame) {
    bool pktSuccess, pktLost;
    NetworkBufferDescriptor_t * pxNetworkBuffer = NULL;
    IPStackEvent_t xRxEvent = {eNetworkRxEvent, NULL};

    pktSuccess = pktLost = false;

    while (true) {
	if (eConsiderFrameForProcessing(frame) != eProcessBuffer) {
	    break;
	}

	/* get the network descriptor (no data buffer) to hold this packet */
	pxNetworkBuffer = pxGetNetworkBufferWithDescriptor(len, 0);

	if (pxNetworkBuffer == NULL) {
	    pktLost = true;
	    break;
	}

	memcpy(pxNetworkBuffer->pucEthernetBuffer, frame, len);

	xRxEvent.pvData = (void *) pxNetworkBuffer;

	/* Send the data to the TCP/IP stack */
	if (xSendEventStructToIPTask(&xRxEvent, 0) == pdFALSE) { /* failed */
	    pktLost = true;
	} else { /* success */
	    pktSuccess = true;
	    iptraceNETWORK_INTERFACE_RECEIVE();
	}

	break;
    }

    if (!pktSuccess) { /* smth went wrong; nothing sent to the */
	if (pxNetworkBuffer != NULL) {
	    pxNetworkBuffer->pucEthernetBuffer = 0;
	    vReleaseNetworkBufferAndDescriptor(pxNetworkBuffer);
	}

	if (pktLost) {
	    iptraceETHERNET_RX_EVENT_LOST();
	}
    }
}

const char *pcApplicationHostnameHook(void) {
    return HOSTNAME;
}

BaseType_t xApplicationDNSQueryHook(const char *pcName) {
    if (memcmp(pcName, HOSTNAME, sizeof (HOSTNAME) - 1) == 0) {
	return pdTRUE;
    } else {
	return pdFALSE;
    }
}

void vApplicationIPNetworkEventHook(eIPCallbackEvent_t eNetworkEvent) {
    static BaseType_t xTasksAlreadyCreated = pdFALSE;

    /* Both eNetworkUp and eNetworkDown events can be processed here. */
    if (eNetworkEvent == eNetworkUp) {
	/* Create the tasks that use the TCP/IP stack if they have not already
	been created. */
	DHCPready = TRUE;
	if (xTasksAlreadyCreated == pdFALSE) {
	    /*
	     * For convenience, tasks that use FreeRTOS+TCP can be created here
	     * to ensure they are not created before the network is usable.
	     */

	    xTasksAlreadyCreated = pdTRUE;
	}
    }
}

eDHCPCallbackAnswer_t xApplicationDHCPHook(eDHCPCallbackPhase_t eDHCPPhase, uint32_t ulIPAddress) {
    eDHCPCallbackAnswer_t eReturn;
    uint32_t ulStaticIPAddress, ulStaticNetMask;

    /* This hook is called in a couple of places during the DHCP process, as
    identified by the eDHCPPhase parameter. */
    switch (eDHCPPhase) {
	case eDHCPPhasePreDiscover:
	    /* A DHCP discovery is about to be sent out.  eDHCPContinue is
	    returned to allow the discovery to go out.

	    If eDHCPUseDefaults had been returned instead then the DHCP process
	    would be stopped and the statically configured IP address would be
	    used.

	    If eDHCPStopNoChanges had been returned instead then the DHCP
	    process would be stopped and whatever the current network
	    configuration was would continue to be used. */
	    eReturn = eDHCPContinue;
	    break;

	case eDHCPPhasePreRequest:
	    /* An offer has been received from the DHCP server, and the offered
	    IP address is passed in the ulIPAddress parameter.  Convert the
	    offered and statically allocated IP addresses to 32-bit values. */
	    eReturn = eDHCPContinue;

	    break;
	case eDHCPPhaseFailed:
	    SysLog(LOG_NORMAL, "WIFI: DHCP failure\r\n");
	    eReturn = eDHCPContinue;
	    break;

	default:
	    /* Cannot be reached, but set eReturn to prevent compiler warnings
	    where compilers are disposed to generating one. */
	    eReturn = eDHCPContinue;
	    break;
    }

    return eReturn;
}

