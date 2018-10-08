/**
 *
 * \file
 *
 * \brief This module contains SAMV71 BSP APIs implementation.
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
#include "xc.h"
#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#include "conf_wilc.h"
#include "app_tasks.h"
#include "system/int/sys_int.h"

static tpfNmBspIsr gpfIsr;

void chip_isr_ext4(void) {

    BaseType_t xHigherPriorityTaskWoken = pdTRUE;
    if (gpfIsr) {
	gpfIsr();
    }
    IFS0CLR = _IFS0_INT4IF_MASK;
    if (gpfIsr) {
	vTaskNotifyGiveFromISR(xNetworkTaskHandle, &xHigherPriorityTaskWoken);
	
    }
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/*
 *	@fn		init_chip_pins
 *	@brief	Initialize reset, chip enable and wake pin
 */
static void init_chip_pins(void)
{
	WILC_CS_HI;
}

/*
*	@fn		nm_bsp_init
*	@brief	Initialize BSP
*	@return	0 in case of success and -1 in case of failure
*/
sint8 nm_bsp_init(void)
{
	gpfIsr = NULL;

	/* Initialize chip IOs. */
	init_chip_pins();

    /* Make sure a 1ms Systick is configured. */
   /* if (!(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk && SysTick->CTRL & SysTick_CTRL_TICKINT_Msk)) {
	    delay_init();
    }*/

	/* Perform chip reset. */
	nm_bsp_reset();

	return 0;
}

sint8 nm_bsp_deinit(void) {
    IEC0CLR = _IEC0_INT4IE_MASK;
    WILC_CHIPEN_LO;
    WILC_RESET_LO;
    return 0;
}

/**
 *	@fn		nm_bsp_reset
 *	@brief	Reset NMC1500 SoC by setting CHIP_EN and RESET_N signals low,
 *           CHIP_EN high then RESET_N high
 */
void nm_bsp_reset(void)
{
    
	WILC_CHIPEN_LO;
	WILC_RESET_LO;
	nm_bsp_sleep(100);
	WILC_CHIPEN_HI;
	nm_bsp_sleep(100);
	WILC_RESET_HI;
	nm_bsp_sleep(100);
}

/*
*	@fn		nm_bsp_sleep
*	@brief	Sleep in units of mSec
*	@param[IN]	u32TimeMsec
*				Time in milliseconds
*/
void nm_bsp_sleep(uint32 u32TimeMsec)
{
	vTaskDelay(u32TimeMsec);
}

/*
*	@fn		nm_bsp_register_isr
*	@brief	Register interrupt service routine
*	@param[IN]	pfIsr
*				Pointer to ISR handler
*/
void nm_bsp_register_isr(tpfNmBspIsr pfIsr)
{
	gpfIsr = pfIsr;
	INTCONCLR = _INTCON_INT4EP_MASK;
	//IFS0CLR = _IFS0_INT4IF_MASK;//We need to catch the first interrupt for this to work
	IEC0SET = _IEC0_INT4IE_MASK;
	IPC5SET = CONF_WILC_SPI_INT_PRIORITY << _IPC5_INT4IP_POSITION;
}

/*
*	@fn		nm_bsp_interrupt_ctrl
*	@brief	Enable/Disable interrupts
*	@param[IN]	u8Enable
*				'0' disable interrupts. '1' enable interrupts
*/
void nm_bsp_interrupt_ctrl(uint8 u8Enable) {

    SYS_INT_Disable();//We have to prevent a race condition on this.
    // If INT line is low, we might have missed an interrupt, hence force an interrupt to
    // clear the status
    if (!PORTFbits.RF5) {
	IFS0SET = _IFS0_INT4IF_MASK;
    }
    if (u8Enable) {
	/* The status register of the PIO controller is cleared prior to enabling the interrupt */
	//IFS0CLR = _IFS0_INT4IF_MASK;
	IEC0SET = _IEC0_INT4IE_MASK;
    } else {
	IEC0CLR = _IEC0_INT4IE_MASK;
    }
    SYS_INT_Enable();
}
