/**
 *
 * \file
 *
 * \brief This module contains NMC1000 bus wrapper APIs implementation.
 *
 * Copyright (c) 2016-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#include <stdio.h>
#include "conf_wilc.h"
#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"
#include "xc.h"
#include "system_config.h"
#include "sys/kmem.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#define MIN_DMA_SPI_SIZE 256
#define MAX_DMA_SPI_TRANSFER 2048

#define NM_BUS_MAX_TRX_SZ 4096

SemaphoreHandle_t SpiMutex = NULL;
static TaskHandle_t xTaskToNotify = NULL;

tstrNmBusCapabilities egstrNmBusCapabilities ={
    NM_BUS_MAX_TRX_SZ
};

/** Fast CS macro. */
#define SPI_ASSERT_CS()		WILC_CS_LO
#define SPI_DEASSERT_CS()	WILC_CS_HI

//static volatile int SpiRxCount = 0;
static volatile int SpiRxCount = 0;
static unsigned char EmptyRxBuff[MAX_DMA_SPI_TRANSFER] __attribute__((coherent, aligned(16))) = {0};
static unsigned char EmptyTxBuff[MAX_DMA_SPI_TRANSFER] __attribute__((coherent, aligned(16))) = {0};

void DmaHandler2(void) {
    //SpiTxCount -= DCH2SSIZ;
    DCH2INTCLR = _DCH2INT_CHBCIF_MASK;
    IFS4CLR = _IFS4_DMA2IF_MASK;
    /*if (SpiTxCount) {
	if (SpiTxCount < DumpBuffSize) {
	    DCH2SSIZ = SpiTxCount;
	}
	DCH2CONSET = _DCH2CON_CHEN_MASK; //Re enable, we have more
    } */
}

void DmaHandler3(void) {
    BaseType_t xHigherPriorityTaskWoken = pdTRUE;
    DCH3INTCLR = _DCH3INT_CHBCIF_MASK;
    IFS4CLR = _IFS4_DMA3IF_MASK;
    SpiRxCount = 0;
    configASSERT(xTaskToNotify != NULL);
    vTaskNotifyGiveFromISR(xTaskToNotify, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

static sint8 spi_rw_dma(uint8 *pu8Mosi, uint8 *pu8Miso, uint16 u16Sz) {

    int TxIndex, TxSize, Remaining;
    configASSERT(xTaskToNotify == NULL);
    xTaskToNotify = xTaskGetCurrentTaskHandle();
    SPI_ASSERT_CS();
    for (TxIndex = 0; TxIndex < u16Sz;) {
	Remaining = u16Sz - TxIndex;
	if (Remaining > MAX_DMA_SPI_TRANSFER) {
	    TxSize = MAX_DMA_SPI_TRANSFER;
	} else {
	    TxSize = Remaining;
	}
	//Set up transmit
	DCH2SSIZ = TxSize;
	if (pu8Mosi == NULL) {
	    //Dump data
	    DCH2SSA = KVA_TO_PA(EmptyTxBuff); //Source
	} else {
	    DCH2SSA = KVA_TO_PA(&pu8Mosi[TxIndex]); //Source
	}

	//Set up to receive
	DCH3DSIZ = TxSize;
	if (pu8Miso == NULL) {
	    configASSERT(pu8Mosi != NULL);
	    //Dump data
	    DCH3DSA = KVA_TO_PA(EmptyRxBuff); //Destination
	} else {
	    DCH3DSA = KVA_TO_PA(&pu8Miso[TxIndex]); //Destination
	}
	SpiRxCount = TxSize;
	//Kick it off
	DCH3CONSET = _DCH3CON_CHEN_MASK; //RX
	DCH2CONSET = _DCH2CON_CHEN_MASK; //TX
	//Wait for finish
	while (SpiRxCount) {
	    ulTaskNotifyTake(pdTRUE, 1 / portTICK_PERIOD_MS);
	}
	TxIndex += TxSize;
    }
    xTaskToNotify = NULL;
    SPI_DEASSERT_CS();

    return M2M_SUCCESS;
}

static sint8 spi_rw(uint8 *pu8Mosi, uint8 *pu8Miso, uint16 u16Sz) {
    uint8_t data;
    uint32_t i;
    uint32_t in = 0;
    /* Assert chip select. */
    SPI_ASSERT_CS();
    for (i = 0; i < u16Sz; i++) {
	/* Transmit byte to SPI. */
	while (!SPI4STATbits.SPITBE) {
	    if (!SPI4STATbits.SPIRBE) {
		data = SPI4BUF;
		if (pu8Miso) {
		    pu8Miso[in] = data;
		}
		in++;
	    }
	}
	if (pu8Mosi) {
	    SPI4BUF = pu8Mosi[i];
	} else {
	    SPI4BUF = 0;
	}

	/* Receive byte from SPI if available*/
	if (!SPI4STATbits.SPIRBE) {
	    data = SPI4BUF;
	    if (pu8Miso) {
		pu8Miso[in] = data;
	    }
	    in++;
	}
    }
    //Clean up the FIFO
    while (in < u16Sz) {
	if (!SPI4STATbits.SPIRBE) {
	    data = SPI4BUF;
	    if (pu8Miso) {
		pu8Miso[in] = data;
	    }
	    in++;
	}
    }

    /* De-assert chip select. */
    SPI_DEASSERT_CS();

    return M2M_SUCCESS;
}

/*
 *	@fn		nm_bus_init
 *	@brief	Initialize the bus wrapper
 *	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
 */
#ifndef __PRODUCTION
//static traceString channel;
#endif

sint8 nm_bus_init(void *pvinit) {
    if (SpiMutex == NULL) {
	SpiMutex = xSemaphoreCreateMutex();
	configASSERT(SpiMutex != NULL);
#ifndef __PRODUCTION
	vTraceSetMutexName(SpiMutex, "SpiMutex");
#endif
    }
    sint8 result = M2M_ERR_BUS_FAIL;
    /* Configure SPI pins. */
    //Harmony already did it
    WILC_WAKE_HI;
    SPI_DEASSERT_CS();

    /* Configure SPI module. */
    SPI4STAT = 0;
    SPI4CON = 0;
    SPI4CONbits.MSTEN = 1;
    SPI4CONbits.CKP = CONF_WILC_SPI_POL;
    SPI4CONbits.CKE = 1;
    SPI4CONbits.SMP = 1;
    SPI4CONbits.ENHBUF = 1;
    SPI4CONbits.SRXISEL = 1;//Interrupt when RX buffer is not empty
    SPI4CONbits.STXISEL = 1;//Interrupt when TX buffer is not full
    SPI4BRG = SYS_CLK_BUS_PERIPHERAL_2 / (2 * CONF_WILC_SPI_CLOCK) - 1;
    SPI4CONbits.ON = 1;
    SPI4CONSET = _SPI4CON_ON_MASK;
    //Set up dma 2 and 3
    
    //TX setup
    DCH2CON = 0;
    DCH2ECON = 0;
    DCH2INT = 0;
    DCH2ECONbits.CHSIRQ = _SPI4_TX_VECTOR;
    DCH2ECONbits.SIRQEN = 1;
    DCH2INTbits.CHBCIE = 1; //Channel block transfer complete interrupt enable
    //DCH2SSA = KVA_TO_PA(&screenbuff); //Source
    DCH2SSIZ = 1;
    DCH2DSA = KVA_TO_PA(&SPI4BUF); //Destination
    DCH2DSIZ = 1;
    DCH2CSIZ = 1;
    //IEC4SET = _IEC4_DMA2IE_MASK;
    IPC34SET = (2 << _IPC34_DMA2IP_POSITION);

    //RX setup
    DCH3CON = 0;
    DCH3ECON = 0;
    DCH3INT = 0;
    DCH3CONbits.CHPRI = 1;
    DCH3ECONbits.CHSIRQ = _SPI4_RX_VECTOR;
    DCH3ECONbits.SIRQEN = 1;
    DCH3INTbits.CHBCIE = 1; //Channel block transfer complete interrupt enable
    //DCH3SSA = KVA_TO_PA(&screenbuff); //Source
    DCH3SSIZ = 1;
    DCH3SSA = KVA_TO_PA(&SPI4BUF); //Destination
    DCH3DSIZ = 1;
    DCH3CSIZ = 1;
    IEC4SET = _IEC4_DMA3IE_MASK;
    IPC34SET = (2 << _IPC34_DMA3IP_POSITION);

    nm_bsp_reset();
    SPI_DEASSERT_CS();
    result = M2M_SUCCESS;
    return result;
}

/*
 *	@fn		nm_bus_ioctl
 *	@brief	send/receive from the bus
 *	@param[IN]	u8Cmd
 *					IOCTL command for the operation
 *	@param[IN]	pvParameter
 *					Arbitrary parameter depenging on IOCTL
 *	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
 *	@note	For SPI only, it's important to be able to send/receive at the same time
 */
sint8 nm_bus_ioctl(uint8 u8Cmd, void* pvParameter) {
    sint8 s8Ret = 0;
   /* static void * exttrack = (void*) 0;
    extern void *pxCurrentTCB;
    if (exttrack != pxCurrentTCB) {
	PrintUart("nm_bus_ioctl = 0x%X\r\n", pxCurrentTCB);
	exttrack = pxCurrentTCB;
    }*/
    //(void) xSemaphoreTake(SpiMutex, portMAX_DELAY);//Last ditch safety
    {
	switch (u8Cmd) {
	    case NM_BUS_IOCTL_RW:
	    {
		tstrNmSpiRw *pstrParam = (tstrNmSpiRw *) pvParameter;
		if (pstrParam->u16Sz > MIN_DMA_SPI_SIZE && (
			(IS_KVA1(pstrParam->pu8InBuf) && pstrParam->pu8InBuf) ||
			(IS_KVA1(pstrParam->pu8OutBuf)) && pstrParam->pu8OutBuf)
			) {
		    s8Ret = spi_rw_dma(pstrParam->pu8InBuf, pstrParam->pu8OutBuf, pstrParam->u16Sz);
		    //s8Ret = spi_rw(pstrParam->pu8InBuf, pstrParam->pu8OutBuf, pstrParam->u16Sz);
		} else {
		    s8Ret = spi_rw(pstrParam->pu8InBuf, pstrParam->pu8OutBuf, pstrParam->u16Sz);
		}

	    }
		break;
	    default:
		s8Ret = -1;
		M2M_ERR("Invalid IOCTL command!\n");
		break;
	}
    }
    //(void) xSemaphoreGive(SpiMutex);

    return s8Ret;
}

/*
 *	@fn		nm_bus_deinit
 *	@brief	De-initialize the bus wrapper
 */
sint8 nm_bus_deinit(void) {
    DCH2CON = 0;
    DCH2ECON = 0;
    DCH2INT = 0;
    DCH3CON = 0;
    DCH3ECON = 0;
    DCH3INT = 0;
    SPI4STAT = 0;
    SPI4CON = 0;
    IEC4CLR = _IEC4_DMA3IE_MASK;
    return M2M_SUCCESS;
}
