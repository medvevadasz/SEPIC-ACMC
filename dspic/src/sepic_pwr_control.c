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

volatile SEPIC_POWER_CONTROLLER_t sepic;

// Remove: volatile bool adc_active = false;
// Remove: soft start settings are now part of the SEPIC data structure
//volatile sepic.soft_start_t sepic.soft_start;

volatile uint16_t init_sepic_pwr_control(void) {
    
    init_sepic_trig_pwm();   // Set up auxiliary PWM for sepic converter
    init_sepic_pwm();        // Set up sepic converter PWM
    init_sepic_acmp();       // Set up sepic converter peak current comparator/DAC
    init_sepic_adc();        // Set up sepic converter ADC (voltage feedback only)
    init_pot_adc();          // Set up ADC for sampling reference provided by external voltage divider        
    
    sepic.soft_start.counter = 0;                // Reset Soft-Start Counter
    sepic.soft_start.pwr_on_delay = 4999;        // Soft-Start Power-On Delay = 500 ms
    sepic.soft_start.ramp_period = 499;          // Soft-Start Ramp Period = 50 ms
    sepic.soft_start.pwr_good_delay = 1999;      // Soft-Start Power Good Delay = 200 ms
    sepic.soft_start.reference = 2204;           // Soft-Start Target Reference = 12V
    
    c2p2z_sepic_Init();
    
    c2p2z_sepic.ADCTriggerOffset = VOUT_ADC_TRIGGER_DELAY;
    c2p2z_sepic.ptrADCTriggerRegister = &PG3TRIGA;
    c2p2z_sepic.InputOffset = 0;
    c2p2z_sepic.ptrControlReference = &sepic.data.v_ref;
    c2p2z_sepic.ptrSource = &ADCBUF16;
    c2p2z_sepic.ptrTarget = &DAC1DATH;
    c2p2z_sepic.MaxOutput = DAC_MAXIMUM;
    c2p2z_sepic.MinOutput = DAC_MINIMUM;
    c2p2z_sepic.status.flag.enable = 0;
    
    sepic.data.v_ref    = 0; // Reset SEPIC reference value (will be set via external potentiometer)
//Remove:    data.manual_vref    = sepic.soft_start.reference;   // This is the initial reference for the controller after soft-start finished
    
    return(1);
}

volatile uint16_t launch_sepic_pwr_control(void) {

    // Run enable-sequence of all peripherals used by this power controller
    launch_adc();               // Start ADC Module
    launch_sepic_acmp();        // Start analog comparator/DAC module
    launch_sepic_trig_pwm();    // Start auxiliary PWM 
    launch_sepic_pwm();         // Start PWM
    
    c2p2z_sepic_Reset(&c2p2z_sepic);    // Reset control loop histories
    
    return(1);
}

volatile uint16_t exec_sepic_pwr_control(void) {
        
    switch (sepic.soft_start.phase) {
        
        /*!SEPIC_SS_INIT
         * When SEPIC converter is in initialization mode, the basic 
         * peripherals needed for the operation of the power converter
         * will be set up. This step is only executed once. */
        case SEPIC_SS_INIT: // basic PWM, ADC, CMP, DAC configuration
            
            init_sepic_pwr_control();    // Initialize all peripherals and data structures of the controller
            
            sepic.status.flags.op_status = SEPIC_STAT_OFF; // Set SEPIC status to OFF
            sepic.soft_start.phase = SEPIC_SS_LAUNCH_PER;
            DBGPIN_2_SET;
            DBGPIN_3_SET;
        break;

        /*!SEPIC_SS_LAUNCH_PER
         * Once all required peripherals have been initialized, the peripheral blocks and
         * control related software modules are enabled in a dedicated sequence. The PWM 
         * outputs and control loop execution, however, are still disabled and the power
         * supply will not start up yet. This step is only executed once and will complete by 
         * switching into STANDBY  */
        case SEPIC_SS_LAUNCH_PER: // Enabling PWM, ADC, CMP, DAC 
            
            launch_sepic_pwr_control(); 
            
            sepic.status.flags.op_status = SEPIC_STAT_OFF; // Set SEPIC status to OFF
            sepic.soft_start.phase = SEPIC_SS_STANDBY;
            DBGPIN_2_CLEAR;
            
        break;
        
        /*!SEPIC_SS_STANDBY
         * This state is entered once all required peripherals have been launched the 
         * power supply is waiting to be launched. This is also the standard fall-back 
         * state after a fault condition.
         * To get the power supply to start, all faults status bits need to be cleared
         * and the status bit "sepic.status.flags" has to be set to "SEPIC_STAT_START"
        */
        case SEPIC_SS_STANDBY: // Enabling PWM, ADC, CMP, DAC 

            sepic.status.flags.op_status = SEPIC_STAT_STANDBY;  // Set SEPIC status to STANDBY
            
            // Force PWM output and controller to OFF state
            PG1IOCONLbits.OVRENH = 1;           // Disable PWMxH output
            c2p2z_sepic.status.flag.enable = 0; // Disable the control loop
            sepic.status.flags.pwm_active = false;   // Clear PWM_ACTIVE flag bit

            // wait for fault to be cleared, adc to run and the GO bit to be set
            if( (sepic.status.flags.enabled == 1) && 
                (sepic.status.flags.adc_active) &&
                (sepic.status.flags.GO) )
            {
                sepic.soft_start.counter = 0;                   // Reset soft-start counter
                sepic.soft_start.phase = SEPIC_SS_PWR_ON_DELAY; // Switch to Power On Delay mode
                DBGPIN_2_CLEAR;
            }
            break;

        /*!SEPIC_SS_PWR_ON_DELAY
         * In this step the soft-start procedure continues with counting up 
         * until the defined power-on delay period has expired. PWM1H is kept low.
         * At the end of this phase, PWM1H output user override is disabled 
         * and the control is enabled. */     
        case SEPIC_SS_PWR_ON_DELAY:  

            sepic.status.flags.op_status = SEPIC_STAT_START; // Set SEPIC status to START-UP
            
            if(sepic.soft_start.counter++ > sepic.soft_start.pwr_on_delay)
            {
                sepic.soft_start.reference = 0;  // Reset soft-start reference to minimum
                c2p2z_sepic.ptrControlReference = &sepic.soft_start.reference; // Hijack controller reference

                sepic.soft_start.counter = 0;                   // Reset soft-start counter
                sepic.soft_start.phase   = SEPIC_SS_RAMP_UP;    // Switch to ramp-up mode
                DBGPIN_2_SET;
            }
            break;    
                 
        case SEPIC_SS_RAMP_UP: // Increasing reference by 4 every scheduler cycle
            
            sepic.status.flags.op_status = SEPIC_STAT_START; // Set SEPIC status to START-UP

            // Force PWM output and controller to be active 
            PG1IOCONLbits.OVRENH = 0;           // User override disabled for PWMxH Pin =< PWM signal output starts
            c2p2z_sepic.status.flag.enable = 1; // Start the control loop 

            sepic.soft_start.reference += 4;  // increment reference
            
            // check if ramp is complete
            if (sepic.soft_start.reference >= sepic.data.v_ref)
            {
                sepic.soft_start.counter = 0;                       // Reset soft-start counter
                sepic.soft_start.phase   = SEPIC_SS_PWR_GOOD_DELAY; // switch to Power Good Delay mode
                DBGPIN_2_CLEAR;
            }
            break; 
            
        case SEPIC_SS_PWR_GOOD_DELAY:
            
            sepic.status.flags.op_status = SEPIC_STAT_START; // Set SEPIC status to START-UP
            
            if(sepic.soft_start.counter++ > sepic.soft_start.pwr_good_delay)
            {
                sepic.soft_start.counter = 0;                 // Reset soft-start counter
                sepic.soft_start.phase   = SEPIC_SS_COMPLETE; // switch to SOFT-START COMPLETE mode
                DBGPIN_2_SET;
            }
            break;
                
        case SEPIC_SS_COMPLETE: // Soft start is complete, system is running, output voltage reference is taken from external potentiometer
            
            sepic.status.flags.op_status = SEPIC_STAT_ON; // Set SEPIC status to ON mode

            c2p2z_sepic.ptrControlReference = &sepic.data.v_ref; // hand reference control back
            
            DBGPIN_2_TOGGLE;
            break;

        default: // If something is going wrong, reset entire PWR controller

            sepic.status.flags.op_status = SEPIC_STAT_FAULT; // Set SEPIC status to FAULT mode
            sepic.status.flags.fault_active = true;          // Set FAULT flag bit

            sepic.soft_start.phase = SEPIC_SS_STANDBY;
            break;
            
    }
        
    // Power converter Auto-Start function
    if (sepic.status.flags.auto_start == true) {
        sepic.status.flags.enabled = true;  // Auto-Enable power converter
        sepic.status.flags.GO = true;       // Auto-Kick-off power converter
    }
    else { 
        sepic.status.flags.GO = false; // Always Auto-Clear GO bit
    }
        
    return(1);
}

void __attribute__((__interrupt__, auto_psv, context)) _ADCAN16Interrupt(void)
{
    sepic.status.flags.adc_active = true;
    sepic.data.v_out = ADCBUF16;

    c2p2z_sepic_Update(&c2p2z_sepic);

    _ADCAN16IF = 0;  // Clear the ADCANx interrupt flag 
    
}

