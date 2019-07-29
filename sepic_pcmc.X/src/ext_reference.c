/*
 * File:   external_reference.c
 * Author: M91406
 *
 * Created on July 24, 2019, 12:45 PM
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "globals.h"
#include "ext_reference.h"

volatile uint32_t vref_avg=0;
volatile uint16_t avg_cnt = 0;

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
    
    vref_avg += (V_REF_MIN + (volatile uint16_t)res);   // Add most recent value to averaging buffer
    
    if(!(++avg_cnt & 0x00FF)) {
        sepic.data.v_ref= (vref_avg >> 8); // Copy averaged value into reference
        vref_avg = 0;                       // Reset averaging buffer
    }
    
    _ADCAN6IF = 0;  // Clear the ADCANx interrupt flag 
    
}

