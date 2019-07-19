/*
 * File:   pwr_control.c
 * Author: M91406
 *
 * Created on July 9, 2019, 1:10 PM
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "npnz16b.h"


volatile bool adc_active = false;
volatile BUCK_SOFT_START_t buck_soft_start;

volatile uint16_t init_buck_pwr_control(void) {
    
    init_buck_trig_pwm();   // Set up auxiliary PWM for buck converter
    init_buck_pwm();        // Set up buck converter PWM
    init_buck_acmp();       // Set up buck converter peak current comparator/DAC
    init_buck_adc();        // Set up buck converter ADC (voltage feedback only)
    
    buck_soft_start.counter = 0;                // Reset Soft-Start Counter
    buck_soft_start.pwr_on_delay = 999;         // Soft-Start Power-On Delay = 100 ms
//    buck_soft_start.precharge_delay = 9;      // Soft-Start Boost Capacitor pre-charge delay = 1 ms
    buck_soft_start.ramp_period = 499;          // Soft-Start Ramp Period = 50 ms
    buck_soft_start.pwr_good_delay = 1999;      // Soft-Start Power Good Delay = 200 ms
    buck_soft_start.reference = 2047;           // Soft-Start Target Reference = 3.3V
    
    c2p2z_buck_Init();
    
    c2p2z_buck.ADCTriggerOffset = 80;
    c2p2z_buck.ptrADCTriggerRegister = &PG3TRIGA;
    c2p2z_buck.InputOffset = 0;
    c2p2z_buck.ptrControlReference = &data.buck_vref;
    c2p2z_buck.ptrSource = &ADCBUF13;
    c2p2z_buck.ptrTarget = &DAC1DATH;
    c2p2z_buck.MaxOutput = 3600;
    c2p2z_buck.MinOutput = 10;
    c2p2z_buck.status.flag.enable = 0;
    
    data.buck_vref = 0;
    
    return(1);
}

volatile uint16_t launch_buck_pwr_control(void) {

    // Run enable-sequence of all peripherals used by this power controller
    launch_adc();           // Start ADC Module
    launch_buck_acmp();     // Start analog comparator/DAC module
    launch_buck_trig_pwm(); // Start auxiliary PWM 
    launch_buck_pwm();      // Start PWM
    
    return(1);
}

volatile uint16_t exec_buck_pwr_control(void) {
        
    switch (buck_soft_start.phase) {
        
        case BUCK_SS_INIT: // basic PWM, ADC, CMP, DAC configuration
            
            init_buck_pwr_control();    // Initialize all peripherals and data structures of the buck controller
            buck_soft_start.phase = BUCK_SS_LAUNCH_PER;
            
            break;

        case BUCK_SS_LAUNCH_PER: // Enabling PWM, ADC, CMP, DAC 
            
            launch_buck_pwr_control(); 
            buck_soft_start.phase = BUCK_SS_PWR_ON_DELAY;
            
            break;
        
        /* In this step the soft-start procedure continues with counting up 
         * until the defined power-on delay period has expired. PWM1H is kept low,
         * while PWM1L is kept high to pre-charge the half-bridge bootstrap cap.
         * At the end of this phase, PWM1 output user overrides are disabled 
         * and the control is enabled. */     
        case BUCK_SS_PWR_ON_DELAY:  
            
            if(buck_soft_start.counter++ > buck_soft_start.pwr_on_delay)
            {
                PG1IOCONLbits.OVRENH = 0;  // User override disabled for PWMxH Pin
                PG1IOCONLbits.OVRENL = 0;  // User override disabled for PWMxL Pin
               
                while (!adc_active); 
                c2p2z_buck.status.flag.enable = 1; // Start the control loop for buck
                
                buck_soft_start.counter = 0;
                buck_soft_start.phase   = BUCK_SS_RAMP_UP;
            }
            break;    
                 
        case BUCK_SS_RAMP_UP: // Increasing reference for buck by 4 every scheduler cycle
            
            data.buck_vref += 4; 
            
            // check if ramp is complete
            if (data.buck_vref >= buck_soft_start.reference)
            {
               buck_soft_start.counter = 0;
               buck_soft_start.phase   = BUCK_SS_PWR_GOOD_DELAY;
            }
            break; 
            
        case BUCK_SS_PWR_GOOD_DELAY:
            
            if(buck_soft_start.counter++ > buck_soft_start.pwr_good_delay)
            {
             buck_soft_start.counter = 0; 
             buck_soft_start.phase   = BUCK_SS_COMPLETE;
            }
            break;
                
        case BUCK_SS_COMPLETE: // Soft start is complete, system is running
            Nop();
            break;

        default: // If something is going wrong, reset entire PWR controller
            buck_soft_start.phase = BUCK_SS_INIT;
            break;
            
    }
        
    
    return(1);
}

void __attribute__((__interrupt__, auto_psv)) _ADCAN13Interrupt(void)
{
    volatile uint16_t dummy=0;
    
    adc_active = true;
    
//    dummy = ADCBUF13;

    data.vout_buck = ADCBUF13;
    c2p2z_buck_Update(&c2p2z_buck);

    _ADCAN13IF = 0;  // Clear the ADCANx interrupt flag 
    
}

