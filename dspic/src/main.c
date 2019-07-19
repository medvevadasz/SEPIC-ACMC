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

volatile MY_DATA_POINTS_t data;

volatile uint16_t tgl_cnt = 0;  // local counter of LED toggle loops
#define TGL_INTERVAL    2999     // LED toggle interval of (2999 + 1) x 100usec = 100ms
#define TMR1_TIMEOUT    30000   // Timeout protection for Timer1 interrupt flag bit

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
    
    // Initialize peripheral modules of individual power controllers
//    init_boost_pwr_control();   // Initialize all peripherals and data structures of the boost controller

//    launch_boost_pwr_control(); // Start Buck Power Controller

    // Enable Timer1
    T1CONbits.TON = 1; 
    
    // Reset Soft-Start Phase to Initialization
    buck_soft_start.phase = BUCK_SS_INIT;   
    
    while (1) {

        // wait for timer1 to overrun
        while ((!_T1IF) && (timeout++ < TMR1_TIMEOUT));
        timeout = 0;    // Reset timeout counter
        _T1IF = 0; // reset Timer1 interrupt flag bit
        DBGPIN_1_TOGGLE; // Toggle DEBUG-PIN

        exec_buck_pwr_control();
        //exec_boos_pwr_control();
        
        if (tgl_cnt++ > TGL_INTERVAL) // Count 100usec loops until LED toggle interval is exceeded
        {
            DBGLED_TOGGLE;
            tgl_cnt = 0;
        } // Toggle LED and reset toggle counter
        Nop();
    }


    return (0);
}
