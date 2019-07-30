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
 * File:   task_fault_handler.h
 * Author: M91406
 * Comments: Global definitions of fault objects
 * Revision history: 
 *  07/30/2019  initial version
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef TASK_FAULT_HANDLER_H
#define	TASK_FAULT_HANDLER_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>
#include <stdbool.h>

#include "globals.h"

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct
{
    volatile uint16_t* obj; // Pointer to value to monitor
    volatile uint16_t counter; // Fault hit counter
    volatile uint16_t reset_level; // Input signal fault reset level/fault reset point
    volatile uint16_t reset_cnt_threshold; // Fault counter threshold resetting fault exception
    volatile uint16_t trip_level; // Input signal fault trip level/fault trip point
    volatile uint16_t trip_cnt_threshold; // Fault counter threshold triggering fault exception
}__attribute__((packed))FAULT_OBJECT_t;


extern volatile uint16_t fault_check_exec(void);
extern volatile uint16_t fault_check_init(void);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* TASK_FAULT_HANDLER_H */

