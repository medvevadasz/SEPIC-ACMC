/*
 * File:   pwr_control.c
 * Author: M91406
 *
 * Created on July 9, 2019, 1:10 PM
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "globals.h"

volatile SEPIC_POWER_CONTROLLER_t sepic;

volatile uint16_t init_sepic_pwr_control(void) {
    
//    init_sepic_trig_pwm();   // Set up auxiliary PWM for sepic converter
    init_sepic_pwm();       // Set up sepic converter PWM
    init_iin_adc();         // Set up ADC for input current feedback
    init_iout_adc();        // Set up ADC for output current feedback
//    init_vout_adc();        // Set up sepic converter ADC (voltage feedback only)
//    init_vref_adc();        // Set up ADC for sampling reference provided by external voltage divider 
    init_vin_adc();         // Initialize ADC Channel to measure input voltage
    
    sepic.soft_start.iin_acc        = 0;            // Reset accumulator for averaging to calculate input current offset
    sepic.soft_start.avg_counter    = 0;            // Reset counter for input current offset calculation
    sepic.soft_start.counter        = 0;            // Reset Soft-Start Counter
    sepic.soft_start.cal_counter    = 0;
    
    sepic.soft_start.pwr_on_delay   = SEPIC_PODLY;  // Soft-Start Power-On Delay = 500 ms
    sepic.soft_start.ramp_period_1  = SEPIC_RPER_1; // Soft-Start 1st Phase Ramp Period
    sepic.soft_start.ramp_period_2  = SEPIC_RPER_2; // Soft-Start 2nd Phase Ramp Period 
    sepic.soft_start.pwr_good_delay = SEPIC_PGDLY;  // Soft-Start Power Good Delay = 200 ms
    
    sepic.soft_start.iout_reference = IOUT_SS_INIT_REF;  // Soft-Start Initial Output Current Reference
                
    sepic.soft_start.ramp_iin_ref_increment    = IIN_SS_STEP;    // Soft-Start Single Step Increment of Input Current Reference
    sepic.soft_start.ramp_iout_ref_increment   = IOUT_SS_STEP;   // Soft-Start Single Step Increment of Output Current Reference

    if(sepic.soft_start.ramp_iout_ref_increment == 0) {          // Protecting startup settings against 
        sepic.soft_start.ramp_iout_ref_increment = 1;            // ZERO settings
    }
    if(sepic.soft_start.ramp_iin_ref_increment == 0) {          // Protecting startup settings against 
        sepic.soft_start.ramp_iin_ref_increment = 1;            // ZERO settings
    }
    
    c2p2z_iin_Init();
    
    c2p2z_iin.ADCTriggerOffset = 0;      // Automatic Trigger Placement is selected in DCLD so trigger will automatically be placed at 50% ON time
                                         // ToDo: Propagation delay to be added here
    c2p2z_iin.ptrADCTriggerRegister = &SEPIC_IIN_ADCTRIG;
    c2p2z_iin.InputOffset = SEPIC_IIN_FEEDBACK_OFFSET;
    c2p2z_iin.ptrControlReference = &sepic.data.i_in_ref;
    c2p2z_iin.ptrSource = &sepic.data.i_in;
    c2p2z_iin.ptrTarget = &SEPIC_DUTY_CYCLE;
    c2p2z_iin.MaxOutput = MAX_DUTY_CYCLE;
    c2p2z_iin.MinOutput = MIN_DUTY_CYCLE;
    c2p2z_iin.status.flag.enable = false;
    
    sepic.data.i_in_ref = 0; // Reset inner current loop reference value 
    
    c2p2z_iout_Init();
    
    c2p2z_iout.ptrControlReference = &sepic.soft_start.iout_reference;
    c2p2z_iout.ptrSource = &sepic.data.i_out;
    c2p2z_iout.ptrTarget = &sepic.data.i_in_ref;
    c2p2z_iout.MaxOutput = IIN_SS_INIT_REF;
    c2p2z_iout.MinOutput = SEPIC_MIN_IIN;
    c2p2z_iout.status.flag.enable = false;
    
    sepic.data.i_out_ref = SEPIC_IOUT_NOM; // Set outer current loop reference value
    
    return(1);
}

volatile uint16_t launch_sepic_pwr_control(void) {

    // Run enable-sequence of all peripherals used by this power controller
    launch_adc();               // Start ADC Module
    launch_sepic_pwm();         // Start PWM
    
    c2p2z_iin_Reset(&c2p2z_iin);   // Reset input current control loop histories
    c2p2z_iout_Reset(&c2p2z_iout); // Reset output current control loop histories
    
    enable_adc_interrupts();
    
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
        case SEPIC_SS_STANDBY: 
DBGPIN_2_SET;
            sepic.status.flags.op_status = SEPIC_STAT_STANDBY;  // Set SEPIC status to STANDBY
            
            // Force PWM output and controller to OFF state
            PG1IOCONLbits.OVRENH = 1;                   // Disable PWMxH output
            c2p2z_iin.status.flag.enable  = false;      // Disable the inner current control loop
            c2p2z_iout.status.flag.enable = false;      // Disable the outer current control loop
            sepic.status.flags.pwm_active = false;      // Clear PWM_ACTIVE flag bit

            // wait for fault to be cleared, adc to run and the GO bit to be set
            if( (sepic.status.flags.enabled == 1) && 
                (sepic.status.flags.adc_active) &&
                (!sepic.status.flags.fault_active) && 
                (sepic.status.flags.GO) )
            {
                // Reset soft-start counters and accumulators
                sepic.soft_start.counter     = 0;                  
                sepic.soft_start.cal_counter = 0;
                sepic.soft_start.avg_counter = 0;
                sepic.soft_start.iin_acc     = 0;
                
                // Hijack output current controller reference
                c2p2z_iout.ptrControlReference = &sepic.soft_start.iout_reference; 
                // Set initial reference for the outer current loop
                sepic.soft_start.iout_reference = IOUT_SS_INIT_REF; 
                
                // Clamp output current controller output 
                c2p2z_iout.MaxOutput  = IIN_SS_INIT_REF;
                // Protecting startup settings against ZERO settings
                if(c2p2z_iout.MaxOutput == 0) {           
                    c2p2z_iout.MaxOutput = 10;             
                }
                
                // Reset input and output current control loop histories
                c2p2z_iin_Reset(&c2p2z_iin);                
                c2p2z_iout_Reset(&c2p2z_iout);  
                
                // Switch to next phase
                sepic.soft_start.phase = SEPIC_SS_PWR_ON_DELAY; 
            }
            break;
        
        /*!SEPIC_SS_PWR_ON_DELAY
         * In this step the soft-start procedure is counting up call intervals until
         * the defined power-on delay period has expired. PWM and control loop are disabled.
         * At the end of this phase, the state automatically switches to RAMP_UP mode */     
        case SEPIC_SS_PWR_ON_DELAY:  
//DBGPIN_2_CLEAR;
            sepic.status.flags.op_status = SEPIC_STAT_START; // Set SEPIC status to START-UP
            
            if(sepic.soft_start.counter++ > sepic.soft_start.pwr_on_delay)
            {
                // Reset soft-start counter
                sepic.soft_start.counter = 0;                
                // Switch to next phase
                sepic.soft_start.phase   = SEPIC_SS_CAL_IIN_OFFSET; 
            }
            break;    
            
        /*!SEPIC_SS_CAL_IIN_OFFSET
         *
         */    
        case SEPIC_SS_CAL_IIN_OFFSET:  
//DBGPIN_2_SET;
            sepic.status.flags.op_status = SEPIC_STAT_START; // Set SEPIC status to START-UP
            
            // While PWM is off
            // every 100us read the input current to establish offset value 
            // at the input current sensor output
            sepic.soft_start.iin_acc += sepic.data.i_in;
            
            if (++sepic.soft_start.avg_counter == 8) {
                
                c2p2z_iin.InputOffset = sepic.soft_start.iin_acc >> 3;
                
                // Force PWM output and controller to be active 
                // User override disabled for PWMxH Pin =< PWM signal output starts
                PG1IOCONLbits.OVRENH = 0;          

                // Set PWM_ACTIVE flag bit
                sepic.status.flags.pwm_active = true;          

                // Start both inner and outer current loops
                c2p2z_iin.status.flag.enable = true;         
                c2p2z_iout.status.flag.enable = true; 
                
                // Switch to next phase
                sepic.soft_start.phase   = SEPIC_SS_WAIT_IOUT_SENSE;
            }
            break;    
                 
        /*!SEPIC_SS_WAIT_IOUT_SENSE
         *
         */    
        case  SEPIC_SS_WAIT_IOUT_SENSE:
//DBGPIN_2_CLEAR;            
            sepic.status.flags.op_status = SEPIC_STAT_START; // Set SEPIC status to START-UP
            
            // Wait until 8 consecutive measured samples of the output current are larger than "10"
            if (sepic.data.i_out > 10) {
                sepic.soft_start.cal_counter++;
                if (sepic.soft_start.cal_counter == 8) {
                    // Switch to next phase
                    sepic.soft_start.phase   = SEPIC_SS_INCR_CLAMP;
                }
            } 
            else if (sepic.soft_start.cal_counter != 0) {
                sepic.soft_start.cal_counter--;
            }
            break;
        
        /*!SEPIC_SS_INCR_CLAMP
         *
         */  
        case SEPIC_SS_INCR_CLAMP:
//DBGPIN_2_SET;            
            sepic.status.flags.op_status = SEPIC_STAT_START; // Set SEPIC status to START-UP
            
            // Gradually increase upper output clamping value of the outer loop to the nominal maximum 
            c2p2z_iout.MaxOutput += sepic.soft_start.ramp_iin_ref_increment;  
            
            if (c2p2z_iout.MaxOutput >= SEPIC_MAX_IIN) {
                c2p2z_iout.MaxOutput = SEPIC_MAX_IIN;
                // Switch to next phase
                sepic.soft_start.phase   = SEPIC_SS_RAMP_UP;
            }
            break;
                    
        /*!SEPIC_SS_RAMP_UP
         * During ramp up, the PWM and control loop are forced ON while the control reference is 
         * incremented. Once the 'private' reference of the soft-start data structure equals the
         * reference level set in sepic.data.v_ref, the ramp-up period ends and the state machine 
         * automatically switches to POWER GOOD DELAY mode */     
        case SEPIC_SS_RAMP_UP: // Increasing reference every scheduler cycle
//DBGPIN_2_CLEAR;
            sepic.status.flags.op_status = SEPIC_STAT_START; // Set SEPIC status to START-UP
            // increment reference
            sepic.soft_start.iout_reference += sepic.soft_start.ramp_iout_ref_increment;  
           
            // Check if ramp is complete
            if (sepic.soft_start.iout_reference >= SEPIC_IOUT_NOM)
            {
                // Restore original source for outer current loop reference
                c2p2z_iout.ptrControlReference = &sepic.data.i_out_ref;
                // Switch to next phase
                sepic.soft_start.phase   = SEPIC_SS_PWR_GOOD_DELAY; 
            }
            break; 
            
        /*!SEPIC_SS_PWR_GOOD_DELAY
         * POWER GOOD DELAY is just like POWER ON DELAY a state in which the soft-start counter
         * is counting call intervals until the user defined period has expired. Then the state 
         * machine automatically switches to COMPLETE mode */     
        case SEPIC_SS_PWR_GOOD_DELAY:
//DBGPIN_2_SET;
            
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
//DBGPIN_2_CLEAR;

            sepic.status.flags.op_status = SEPIC_STAT_ON; // Set SEPIC status to ON mode
            break;

        /*!SEPIC_SS_FAULT or undefined state
         * If any controller state is set, different from the previous ones (e.g. FAULT),
         * the power controller sets the FAULT flag bit, enforces detection of the ADC activity 
         * by clearing the adc_active bit and switches the state machine into STANDBY, from
         * which the power controller may recover as soon as all startup conditions are met again. */
        default: // If something went wrong, reset PWR controller to STANDBY
DBGPIN_2_CLEAR;
            sepic.status.flags.op_status = SEPIC_STAT_FAULT; // Set SEPIC status to FAULT mode
            sepic.status.flags.fault_active = true;         // Set FAULT flag bit

            PG1IOCONLbits.OVRENH = 1;                           // Disable PWMxH output
            c2p2z_iin.status.flag.enable = false;           // Disable the inner current control loop
            c2p2z_iout.status.flag.enable = false;   // Disable the outer current control loop
            sepic.status.flags.enabled = false;                 // Disable power controller

            sepic.status.flags.adc_active = false;          // Clear ADC_READY flag bit
            sepic.status.flags.pwm_active = false;          // Clear PWM_ACTIVE flag bit
            sepic.status.flags.GO = false;                  // Reset power supply start trigger GO bit
            
            sepic.soft_start.phase = SEPIC_SS_STANDBY;
            break;
            
    }
        
    /*!Power Converter Auto-Start Function
     * When the control bit sepic.status.flags.auto_start is set, the status bits 'enabled' 
     * and 'GO' are automatically set and continuously enforced to ensure the power supply
     * will enter RAMP UP from STANDBY without the need for user code intervention. */
    // 
    if( (sepic.status.flags.auto_start == true) && (sepic.status.flags.fault_active == false)) {
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

//void __attribute__((__interrupt__, auto_psv, context))_SEPIC_VOUT_ADCInterrupt(void)
//{
//    DBGPIN_1_SET;
//    
//    sepic.status.flags.adc_active = true;
//    sepic.data.v_out =  SEPIC_VOUT_ADCBUF;
//    sepic.data.v_in =   SEPIC_VIN_ADCBUF;
//    
//    sepic.data.i_in_ref =  SEPIC_VREF_ADCBUF;  
//
//    c2p2z_voltage_Update(&c2p2z_voltage);
//
//    _ADCAN16IF = 0;  // Clear the ADCANx interrupt flag 
//    DBGPIN_1_CLEAR;
//    
//}

void __attribute__((__interrupt__, auto_psv, context))_SEPIC_IIN_ADCInterrupt(void)
{
DBGPIN_3_SET;
    
    sepic.status.flags.adc_active = true;
    sepic.data.i_in = SEPIC_IIN_ADCBUF;
//DBGPIN_2_SET;    
    c2p2z_iin_Update(&c2p2z_iin);
//DBGPIN_2_CLEAR;    

    
    // Wait until ADC conversion for output current finishes
//    while(!_SEPIC_IOUT_ADC_IF); 
        
    sepic.data.i_out = SEPIC_IOUT_ADCBUF;
//DBGPIN_2_SET;     
   c2p2z_iout_Update(&c2p2z_iout);
//DBGPIN_2_CLEAR; 
    SEPIC_VOUT_ADCTRIG = SEPIC_IIN_ADCTRIG + (PWM_PERIOD >> 1);
    
    _SEPIC_IOUT_ADC_IF = 0;  // Clearing the ADCANx interrupt flag
    _SEPIC_IIN_ADC_IF  = 0;  // Clearing the ADCANx interrupt flag
    
DBGPIN_3_CLEAR;
    
}

