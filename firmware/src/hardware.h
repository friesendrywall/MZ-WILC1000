/* 
 * File:   hardware.h
 * Author: Erik
 *
 * Created on August 5, 2017, 6:16 PM
 */

#ifndef HARDWARE_H
#define	HARDWARE_H

#ifdef	__cplusplus
extern "C" {
#endif
#include "xc.h"
    
//#define EthResetOn (LATASET = (1<<14))
    
#define DAC_FMT_I2S (LATBCLR = (1<<12))
#define DAC_FMT_LFJ (LATBSET = (1<<12))

#define DAC_XSMT_MUTE (LATBCLR = (1<<4))
#define DAC_XSMT_UNMUTE (LATBSET = (1<<4))

#define POWERAMP_MUTE (LATJCLR = (1<<14))
#define POWERAMP_UNMUTE (LATJSET = (1<<14))
/*
#define ADC_GAIN_12DB_ON (LATGSET = (1<<0))
#define ADC_GAIN_12DB_OFF (LATGCLR = (1<<0))

#define ADC_GAIN_24DB_ON (LATFSET = (1<<4))
#define ADC_GAIN_24DB_OFF (LATFCLR = (1<<4))

#define ADC_CTRL_CS_HI (LATDSET = (1<<13))
#define ADC_CTRL_CS_LO (LATDCLR = (1<<13))
*/
#define CLOCK_CS_HI (LATGSET = (1<<15))
#define CLOCK_CS_LO (LATGCLR = (1<<15))

#define SPI_32K_CS_HI (LATHSET = (1<<4))
#define SPI_32K_CS_LO (LATHCLR = (1<<4))

#define DISP_CS_HI (LATJSET = (1<<4))
#define DISP_CS_LO (LATJCLR = (1<<4))

#define DISP_DC_HI (LATJSET = (1<<3))
#define DISP_DC_LO (LATJCLR = (1<<3))

#define DISP_RESET_HI (LATJSET = (1<<2))
#define DISP_RESET_LO (LATJCLR = (1<<2))

#define WILC_CS_HI (LATDSET = (1<<12))
#define WILC_CS_LO (LATDCLR = (1<<12))

#define WILC_RESET_HI (LATGSET = (1<<0))
#define WILC_RESET_LO (LATGCLR = (1<<0))

#define WILC_CHIPEN_HI (LATDSET = (1<<13))
#define WILC_CHIPEN_LO (LATDCLR = (1<<13))

#define WILC_WAKE_HI (LATHSET = (1<<15))
#define WILC_WAKE_LO (LATHCLR = (1<<15))

#define TOUCH_0_IN (PORTKbits.RK0)
#define TOUCH_1_IN (PORTKbits.RK1)
#define TOUCH_2_IN (PORTKbits.RK2)
#define TOUCH_3_IN (PORTKbits.RK3)
    
#define TOUCH_UP    (!TOUCH_0_IN)
#define TOUCH_DOWN  (!TOUCH_1_IN)
#define TOUCH_PLAY  (!TOUCH_2_IN)
#define TOUCH_STOP  (!TOUCH_3_IN)

#define CsMem1Set   SPI_32K_CS_HI
#define CsMem1Clr   SPI_32K_CS_LO
    
#define Led_Status_Mask (1<<3)
#define Led_Status_OFF (LATCSET = Led_Status_Mask)
#define Led_Status_ON (LATCCLR = Led_Status_Mask)

//#define ClearLeds Led_Play_Green_OFF;Led_Play_Red_OFF;Led_Signal_Green_OFF;Led_Signal_Red_OFF;Led_Stream_Green_OFF;Led_Stream_Red_OFF;Led_Status_Green_OFF;Led_Status_Red_OFF;Led_Power_Red_OFF

#ifdef	__cplusplus
}
#endif

#endif	/* HARDWARE_H */

