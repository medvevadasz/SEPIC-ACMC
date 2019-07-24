/*
 * File:   external_reference.c
 * Author: M91406
 *
 * Created on July 24, 2019, 12:45 PM
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "main.h"

volatile uint16_t ext_reference_init(void) {

    sepic.data.v_ref = 0;   // Reset SEPIC reference
    init_pot_adc();         // Initialize 
 
    return(1);
}


void __attribute__((__interrupt__, auto_psv, context)) _ADCAN6Interrupt(void)
{
    volatile uint16_t samp=0;
    volatile uint32_t res=0;
    
    samp = ADCBUF6; // read latest sample
    samp <<= 3;     // normalize to Q15
    res = (volatile uint32_t)samp * V_REF_DIFF;
    res >>= 15;     // normalize back into 16-bit
    sepic.data.v_ref = (V_REF_MINIMUM + (volatile uint16_t)res);

    _ADCAN6IF = 0;  // Clear the ADCANx interrupt flag 
    
}

