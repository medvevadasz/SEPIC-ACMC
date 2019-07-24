/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   ext_reference.h
 * Author: M91406
 * Comments: Configuration and execution of the external reference handler
 * Revision history: 
 *      07/24/2019   initial version
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef EXTERNAL_REFERENCE_HANDLER_H
#define	EXTERNAL_REFERENCE_HANDLER_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>
#include <stdbool.h>
#include <math.h>


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#define V_REF_MINIMUM       9.0  // lower output voltage limit in [V]
#define V_REF_MAXIMUM       22.0 // upper output voltage limit in [V]
    
#define V_SOURCE_R1         (2.0 * 2.87)    // Upper voltage divider resistor in kOhm
#define V_SOURCE_R2         (1.0)           // Lower voltage divider resistor in kOhm
#define V_FB_GAIN           (float)((V_SOURCE_R2) / (V_SOURCE_R1 + V_SOURCE_R2))
    
//#define V_REF_MINIMUM       1657 // reference value for 9V output voltage
//#define V_REF_MAXIMUM       4051 // reference value for 22V output voltage
    
#define ADCREF              3.300 // ADC reference voltage in V
#define ADCRES              12.0  // ADC resolution in [bit]
#define ADCGRAN             (float)(ADCREF / pow(2.0, ADCRES))
    
#define V_REF_MIN           (uint16_t)(V_REF_MINIMUM * V_FB_GAIN / ADCGRAN)
#define V_REF_MAX           (uint16_t)(V_REF_MAXIMUM * V_FB_GAIN / ADCGRAN)
#define V_REF_DIFF          (V_REF_MAX - V_REF_MIN)
    
    
extern volatile uint16_t ext_reference_init(void);


#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* EXTERNAL_REFERENCE_HANDLER_H */

