/*
 * File:   tas_fault_handler_dummy.c
 * Author: M91281
 *
 * Created on August 6, 2019, 1:45 PM
 */


#include "xc.h"
#include <stdint.h>
#include <stdbool.h>

#include "globals.h"


volatile FAULT_OBJECT_t ovp;   // Output Over Voltage Protection (OVP)

volatile uint16_t release_counter=0;    // Counter for Recover From Fault Release


volatile uint16_t fault_check_dummy_exec(void) { 
    
    // Check if output voltage is above maximum
    if(*ovp.obj > ovp.trip_level) {
        if(ovp.counter++ > ovp.trip_cnt_threshold) {
            ovp.counter = ovp.trip_cnt_threshold + 1;
        }
    }
    else if(*ovp.obj < ovp.reset_level) {
        ovp.counter = 0;
    }
    
    // Check if any FAULT condition is persistent and the power supply needs to be shut down
    if (!sepic.status.flags.fault_active) {
    // if no FAULT condition has been detected, check if any fault condition is overdue
    
        // Check fault counters for fault trip conditions
        if(ovp.counter > ovp.trip_cnt_threshold) {
                sepic.status.flags.fault_active = true;    // Set Global FAULT flag (causes the power controller to shut down)
                sepic.soft_start.phase = SEPIC_SS_STANDBY; // Reset power controller to STANDBY waiting for recovery
        }
    }
    // Check if persistent FAULT condition has been resolved and the power supply can start up again
    else {
    // if a FAULT condition is active, wait for restart opportunity
        
        // If all fault counters have been reset to ZERO
        if(ovp.counter == 0) {

            release_counter++;  // Increment RELEASE DELAY counter

            // If release delay counter is longer than all fault release counter thresholds
            if(release_counter > ovp.reset_cnt_threshold)  {
                
                    sepic.status.flags.fault_active = false;    // Reset global FAULT flag
                    release_counter = 0;                        // Reset REALEASE counter
            }
        }
    } // else
    
    return 0;
}

volatile uint16_t fault_check_init(void) {

    // Over Voltage Lock-Out (OVLO)
    ovp.counter = 0;                            // Reset FAULT occurrence counter
    ovp.obj = &sepic.data.v_out;                // Monitor output voltage
    ovp.reset_level = (SEPIC_VOUT_OVP - SEPIC_VOUT_HYST);    // Level at which power supply recovers
    ovp.reset_cnt_threshold = SEPIC_RCVRY_DLY;  // wait for 1000 ms before restart attempt
    ovp.trip_level = SEPIC_VOUT_OVP;            // Level at which power supply shuts down
    ovp.trip_cnt_threshold = SEPIC_FLTTRP_DLY;  // wait for 1 ms before shutting down
    
    return(1);
}
