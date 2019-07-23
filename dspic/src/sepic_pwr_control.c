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
volatile SEPIC_SOFT_START_t sepic_soft_start;

volatile uint16_t init_sepic_pwr_control(void) {
    
    init_sepic_trig_pwm();   // Set up auxiliary PWM for sepic converter
    init_sepic_pwm();        // Set up sepic converter PWM
    init_sepic_acmp();       // Set up sepic converter peak current comparator/DAC
    init_sepic_adc();        // Set up sepic converter ADC (voltage feedback only)
    init_pot_adc();          // Set up ADC for sampling reference provided by external voltage divider        
    
    sepic_soft_start.counter = 0;                // Reset Soft-Start Counter
    sepic_soft_start.pwr_on_delay = 999;         // Soft-Start Power-On Delay = 100 ms
    sepic_soft_start.ramp_period = 499;          // Soft-Start Ramp Period = 50 ms
    sepic_soft_start.pwr_good_delay = 1999;      // Soft-Start Power Good Delay = 200 ms
    sepic_soft_start.reference = 2204;           // Soft-Start Target Reference = 12V
    
    c2p2z_sepic_Init();
    
    c2p2z_sepic.ADCTriggerOffset = VOUT_ADC_TRIGGER_DELAY;
    c2p2z_sepic.ptrADCTriggerRegister = &PG3TRIGA;
    c2p2z_sepic.InputOffset = 0;
    c2p2z_sepic.ptrControlReference = &data.sepic_vref;
    c2p2z_sepic.ptrSource = &ADCBUF16;
    c2p2z_sepic.ptrTarget = &DAC3DATH;
    c2p2z_sepic.MaxOutput = 3600;
    c2p2z_sepic.MinOutput = 10;
    c2p2z_sepic.status.flag.enable = 0;
    
    data.sepic_vref     = 0;
    data.manual_vref    = sepic_soft_start.reference;   // This is the initial reference for the controller after soft-start finished
    
    return(1);
}

volatile uint16_t launch_sepic_pwr_control(void) {

    // Run enable-sequence of all peripherals used by this power controller
    launch_adc();               // Start ADC Module
    launch_sepic_acmp();        // Start analog comparator/DAC module
    launch_sepic_trig_pwm();    // Start auxiliary PWM 
    launch_sepic_pwm();         // Start PWM
    
    return(1);
}

volatile uint16_t exec_sepic_pwr_control(void) {
        
    switch (sepic_soft_start.phase) {
        
        case SEPIC_SS_INIT: // basic PWM, ADC, CMP, DAC configuration
            
            init_sepic_pwr_control();    // Initialize all peripherals and data structures of the controller
            sepic_soft_start.phase = SEPIC_SS_LAUNCH_PER;
            DBGPIN_2_SET;
            DBGPIN_3_SET;
        break;

        case SEPIC_SS_LAUNCH_PER: // Enabling PWM, ADC, CMP, DAC 
            
            launch_sepic_pwr_control(); 
            sepic_soft_start.phase = SEPIC_SS_PWR_ON_DELAY;
            DBGPIN_2_CLEAR;
        break;
        
        /* In this step the soft-start procedure continues with counting up 
         * until the defined power-on delay period has expired. PWM1H is kept low.
         * At the end of this phase, PWM1H output user override is disabled 
         * and the control is enabled. */     
        case SEPIC_SS_PWR_ON_DELAY:  
            
            if(sepic_soft_start.counter++ > sepic_soft_start.pwr_on_delay)
            {
                PG1IOCONLbits.OVRENH = 0;  // User override disabled for PWMxH Pin
                PG1IOCONLbits.OVRENL = 0;  // User override disabled for PWMxL Pin; Although not used, disabling override for this pin is necessary for being able to use PWMxH!!
                               
                while (!adc_active); 
                c2p2z_sepic.status.flag.enable = 1; // Start the control loop 
//               c2p2z_sepic.status.flag.enable = 0; // Start the control loop 
                
               sepic_soft_start.counter = 0;
               sepic_soft_start.phase   = SEPIC_SS_RAMP_UP;
               DBGPIN_2_SET;
            }
            break;    
                 
        case SEPIC_SS_RAMP_UP: // Increasing reference by 4 every scheduler cycle
            
            data.sepic_vref += 4; 
            
            // check if ramp is complete
            if (data.sepic_vref >= sepic_soft_start.reference)
            {
               sepic_soft_start.counter = 0;
               sepic_soft_start.phase   = SEPIC_SS_PWR_GOOD_DELAY;
               DBGPIN_2_CLEAR;
            }
            break; 
            
        case SEPIC_SS_PWR_GOOD_DELAY:
            
            if(sepic_soft_start.counter++ > sepic_soft_start.pwr_good_delay)
            {
             sepic_soft_start.counter = 0; 
             sepic_soft_start.phase   = SEPIC_SS_COMPLETE;
             DBGPIN_2_SET;
            }
            break;
                
        case SEPIC_SS_COMPLETE: // Soft start is complete, system is running, output voltage reference is taken from external potentiometer
            c2p2z_sepic.ptrControlReference = &data.manual_vref;
           
            _ADCAN6IE = 1;    // Enable ADCAN6 Interrupt to sample potentiometer defined reference voltage
            
            DBGPIN_2_TOGGLE;
            break;

        default: // If something is going wrong, reset entire PWR controller
            sepic_soft_start.phase = SEPIC_SS_INIT;
            break;
            
    }
        
    
    return(1);
}

void __attribute__((__interrupt__, auto_psv)) _ADCAN16Interrupt(void)
{
    
    adc_active = true;
    
//    dummy = ADCBUF13;

    data.vout_sepic = ADCBUF16;
    c2p2z_sepic_Update(&c2p2z_sepic);

    _ADCAN16IF = 0;  // Clear the ADCANx interrupt flag 
    
}

void __attribute__((__interrupt__, auto_psv)) _ADCAN6Interrupt(void)
{
        
//    dummy = ADCBUF13;

    data.manual_vref = ADCBUF6;
    c2p2z_sepic_Update(&c2p2z_sepic);

    _ADCAN6IF = 0;  // Clear the ADCANx interrupt flag 
    
}

