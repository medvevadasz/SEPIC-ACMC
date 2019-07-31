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
 * File:   init_acmp.h
 * Author: M91406
 * Comments: Header file for DAC initialization routines
 * Revision history: 
 * 07/24/2019   initial version
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef INITIALIZE_ANALOG_COMPARATOR_H
#define	INITIALIZE_ANALOG_COMPARATOR_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>
#include <stdbool.h>

#include "globals.h"


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */


#define INIT_DACDATH    0       // DAC value for the buck the slope starts from
#define INIT_DACDATL    0       // Set this to minimum in Slope mode
    
extern volatile uint16_t init_acmp_module(void);
extern volatile uint16_t init_sepic_acmp(void);
extern volatile uint16_t launch_sepic_acmp(void);


#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* INITIALIZE_ANALOG_COMPARATOR_H */

