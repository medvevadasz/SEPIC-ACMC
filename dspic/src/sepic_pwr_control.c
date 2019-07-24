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
    
    sepic.soft_start.counter = 0;                             // Reset Soft-Start Counter
    sepic.soft_start.pwr_on_delay = SEPIC_POWER_ON_DELAY;     // Soft-Start Power-On Delay = 500 ms
    sepic.soft_start.ramp_period = SEPIC_RAMP_PERIOD;         // Soft-Start Ramp Period = 50 ms
    sepic.soft_start.pwr_good_delay = SEPIC_POWER_GOOD_DELAY; // Soft-Start Power Good Delay = 200 ms
    sepic.soft_start.reference = SEPIC_V_OUT_REF;             // Soft-Start Target Reference = 12V
    sepic.soft_start.ramp_ref_increment = SEPIC_REF_STEP;     // Soft-Start Single Step Increment of Reference
    
    c2p2z_sepic_Init();
    
    c2p2z_sepic.ADCTriggerOffset = VOUT_ADC_TRIGGER_DELAY;
    c2p2z_sepic.ptrADCTriggerRegister = &ADCTRIG_VOUT;
    c2p2z_sepic.InputOffset = ADC_INPUT_OFFSET;
    c2p2z_sepic.ptrControlReference = &sepic.data.v_ref;
    c2p2z_sepic.ptrSource = &ADCBUF_VOUT;
    c2p2z_sepic.ptrTarget = &DAC_PCMC;
    c2p2z_sepic.MaxOutput = DAC_MAXIMUM;
    c2p2z_sepic.MinOutput = DAC_MINIMUM;
    c2p2z_sepic.status.flag.enable = 0;
    
    sepic.data.v_ref    = 0; // Reset SEPIC reference value (will be set via external potentiometer)
    
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
            
        break;
        
        /*!SEPIC_SS_STANDBY
         * This state is entered once all required peripherals have been launched and the 
         * power supply is waiting to be powered up. This is also the standard fall-back 
         * state after a fault/restart condition.
         * To get the power supply to start, all faults status bits need to be cleared,
         * the ADC has to run and produce data, the power controller has to be enabled 
         * and the status bit "sepic.status.flags.GO" has to be set.
         * 
         * Please note:
         * The data structure sepic.status.flags also offers a setting called auto_start.
         * When this bit is set, the 'enable' and 'GO' bits are set automatically and only
         * the 'adc_active' and 'fault_active' bits are checked.
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
                (!sepic.status.flags.fault_active) && 
                (sepic.status.flags.GO) )
            {
                sepic.soft_start.counter = 0;                   // Reset soft-start counter
                sepic.soft_start.phase = SEPIC_SS_PWR_ON_DELAY; // Switch to Power On Delay mode
            }
            break;

        /*!SEPIC_SS_PWR_ON_DELAY
         * In this step the soft-start procedure is counting up call intervals until
         * the defined power-on delay period has expired. PWM and control loop are disabled.
         * At the end of this phase, the state automatically switches to RAMP_UP mode */     
        case SEPIC_SS_PWR_ON_DELAY:  

            sepic.status.flags.op_status = SEPIC_STAT_START; // Set SEPIC status to START-UP
            
            if(sepic.soft_start.counter++ > sepic.soft_start.pwr_on_delay)
            {
                sepic.soft_start.reference = 0;  // Reset soft-start reference to minimum
                c2p2z_sepic.ptrControlReference = &sepic.soft_start.reference; // Hijack controller reference

                sepic.soft_start.counter = 0;                   // Reset soft-start counter
                sepic.soft_start.phase   = SEPIC_SS_RAMP_UP;    // Switch to ramp-up mode
            }
            break;    
                 
        /*!SEPIC_SS_RAMP_UP
         * During ramp up, the PWM and control loop are forced ON while the control reference is 
         * incremented. Once the 'private' reference of the soft-start data structure equals the
         * reference level set in sepic.data.v_ref, the ramp-up period ends and the state machine 
         * automatically switches to POWER GOOD DELAY mode */     
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
            }
            break; 
            
        /*!SEPIC_SS_PWR_GOOD_DELAY
         * POWER GOOD DELAY is just like POWER ON DELAY a state in which the soft-start counter
         * is counting call intervals until the user defined period has expired. Then the state 
         * machine automatically switches to COMPLETE mode */     
        case SEPIC_SS_PWR_GOOD_DELAY:
            
            sepic.status.flags.op_status = SEPIC_STAT_START; // Set SEPIC status to START-UP
            
            if(sepic.soft_start.counter++ > sepic.soft_start.pwr_good_delay)
            {
                sepic.soft_start.counter = 0;                 // Reset soft-start counter
                sepic.soft_start.phase   = SEPIC_SS_COMPLETE; // switch to SOFT-START COMPLETE mode
            }
            break;
                
        /*!SEPIC_SS_COMPLETE
         * The COMPLETE phase is the default state of the power controller. Once entered, only a FAULT
         * condition or external modifications of the soft-start phase can trigger a change of state. */     
        case SEPIC_SS_COMPLETE: // Soft start is complete, system is running, output voltage reference is taken from external potentiometer
            
            sepic.status.flags.op_status = SEPIC_STAT_ON; // Set SEPIC status to ON mode
            c2p2z_sepic.ptrControlReference = &sepic.data.v_ref; // hand reference control back
            break;

        /*!SEPIC_SS_FAULT or undefined state
         * If any controller state is set, different from the previous ones (e.g. FAULT),
         * the power controller sets the FAULT flag bit, enforces detection of the ADC activity 
         * by clearing the adc_active bit and switches the state machine into STANDBY, from
         * which the power controller may recover as soon as all startup conditions are met again. */
        default: // If something is going wrong, reset PWR controller to STANDBY

            sepic.status.flags.op_status = SEPIC_STAT_FAULT; // Set SEPIC status to FAULT mode
            sepic.status.flags.fault_active = true;          // Set FAULT flag bit
            sepic.status.flags.adc_active = false;           // Clear ADC_READY flag bit

            sepic.soft_start.phase = SEPIC_SS_STANDBY;
            break;
            
    }
        
    /*!Power Converter Auto-Start Function
     * When the control bit sepic.status.flags.auto_start is set, the status bits 'enabled' 
     * and 'GO' are automatically set and continuously enforced to ensure the power supply
     * will enter RAMP UP from STANDBY without the need for user code intervention. */
    // 
    if (sepic.status.flags.auto_start == true) {
        sepic.status.flags.enabled = true;  // Auto-Enable power converter
        sepic.status.flags.GO = true;       // Auto-Kick-off power converter
    }
    else { 
        sepic.status.flags.GO = false; // Always Auto-Clear GO bit
    }
        
    return(1);
}

/*!Power Converter Auto-Start Function
 * **************************************************************************************************
 * 
 * **************************************************************************************************/

void __attribute__((__interrupt__, auto_psv, context))_VOUT_ADCInterrupt(void)
{
    sepic.status.flags.adc_active = true;
    sepic.data.v_out = ADCBUF_VOUT;

    c2p2z_sepic_Update(&c2p2z_sepic);

    _ADCAN16IF = 0;  // Clear the ADCANx interrupt flag 
    
}

