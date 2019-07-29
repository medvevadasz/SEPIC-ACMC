/*
 * File:   main.c
 * Author: M91406
 *
 * Created on July 8, 2019, 1:52 PM
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "main.h"

volatile uint16_t tgl_cnt = 0;  // local counter of LED toggle loops
#define TGL_INTERVAL    2999    // LED toggle interval of (2999 + 1) x 100usec = 100ms
#define TMR1_TIMEOUT    30000   // Timeout protection for Timer1 interrupt flag bit

volatile uint32_t dac_cnt = 0;  // local counter of LED toggle loops
#define DACMOD_COUNT    1999    // DAC increment interval of (1999 + 1) x 100usec = 200ms


int main(void) {

    volatile uint16_t timeout = 0;
     
    init_fosc();        // Set up system oscillator for 100 MIPS operation
    init_aclk();        // Set up Auxiliary PLL for 500 MHz (source clock to PWM module)
    init_timer1();      // Set up Timer1 as scheduler time base
    init_gpio();        // Initialize common device GPIOs
    
    // Basic setup of common power controller peripheral modules
    init_pwm_module();  // Set up PWM module (basic module configuration)
    init_acmp_module(); // Set up analog comparator/DAC module
    init_adc_module();  // Set up Analog-To-Digital converter module
    init_vin_adc();     // Initialize ADC Channel to measure input voltage
    
    ext_reference_init();   // initialize external reference input
    
    // Reset Soft-Start Phase to Initialization
    sepic.soft_start.phase = SEPIC_SS_INIT;   
    sepic.status.flags.auto_start = true;
    
// ===========================================
 // ToDo: FOR DEBUGGING ONLY! REMOVE WHEN DONE
// ===========================================
    // Comparator output = RB11(RP43) = TP41 on DP DevBoard
    __builtin_write_RPCON(0x0000);
    _TRISB11 = 0;
    _LATB11 = 0;
    RPOR5bits.RP43R = 23;      // Assign COMP1 (=23) output to pin RB11 = RP43  (DSP_GPIO3)
__builtin_write_RPCON(0x0800);
// ===========================================
    
    
    // Enable Timer1
    T1CONbits.TON = 1; 
    
    DBGPIN_2_CLEAR;
    DBGPIN_3_CLEAR;
    
    while (1) {

        // wait for timer1 to overrun
        while ((!_T1IF) && (timeout++ < TMR1_TIMEOUT));
        timeout = 0;    // Reset timeout counter
        _T1IF = 0; // reset Timer1 interrupt flag bit
        DBGPIN_1_TOGGLE; // Toggle DEBUG-PIN

        exec_sepic_pwr_control();
               
        if (tgl_cnt++ > TGL_INTERVAL) // Count 100usec loops until LED toggle interval is exceeded
        {
            DBGLED_TOGGLE;
            tgl_cnt = 0;
        } // Toggle LED and reset toggle counter
        Nop();
        
        // Starting at a slope of 100mV/usec (=8), the slew rate is incremented in single steps
        // up to 1V/usec (=80) and then resets to 100mV/usec (=8)
        if (dac_cnt++ > DACMOD_COUNT)
        {
            if(SLP1DAT < 80)
            { SLP1DAT++; }
            else
            { SLP1DAT = DAC_SLOPE_RATE; }
            
            dac_cnt = 0;
        }
        
        
    }


    return (0);
}
