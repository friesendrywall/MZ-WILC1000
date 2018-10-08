/* 
 * File:   NetworkInterface_wifi.h
 * Author: Erik
 *
 * Created on June 20, 2018, 11:32 AM
 */

#ifndef NETWORKINTERFACE_WIFI_H
#define	NETWORKINTERFACE_WIFI_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include "driver/include/m2m_types.h"

    #define MAX_SCAN_COUNT	10

    char * SSIDINFO(void);
    BaseType_t xConnectToSpecificAP(BaseType_t index);
    BaseType_t xResetWifiModule(void);
    int xGetWifiScanResults(tstrM2mWifiscanResult * data);
    BaseType_t xWifiRSSI(void);
    BaseType_t xRawWifiRSSI(void);
    void NetworkIFinit(void);


#ifdef	__cplusplus
}
#endif

#endif	/* NETWORKINTERFACE_WIFI_H */

