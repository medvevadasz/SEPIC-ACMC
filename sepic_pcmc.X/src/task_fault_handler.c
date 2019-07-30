/*
 * File:   fault_handler.c
 * Author: M91406
 *
 * Created on July 30, 2019, 10:11 PM
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "task_fault_handler.h"

volatile FAULT_OBJECT_t uvlo;  // Input Under Voltage Lock Out (UVLO)
volatile FAULT_OBJECT_t ovlo;  // Input Over Voltage Lock Out (OVLO)
volatile FAULT_OBJECT_t ovp;   // Output Over Voltage Protection (OVP)
volatile FAULT_OBJECT_t reg;   // Regulation tolerance (VOUT vs. VREF)

volatile uint16_t release_counter=0;    // Counter for Recover From Fault Release

// Private function prototypes
volatile uint16_t fault_check_test(void);   // Test for FAULT threshold violations
volatile uint16_t fault_check_trip(void) ;  // Test for FAULT threshold trip violations
volatile uint16_t fault_check_release(void); // Test for FAULT RECOVERY conditions

volatile uint16_t fault_check_exec(void) {

    // Test for FAULT threshold violations
    if (!fault_check_test()) return(0);

    // Check if any FAULT condition is persistent and the power supply needs to be shut down
    if (!sepic.status.flags.fault_active) {
    // if no FAULT condition has been detected, check if any fault condition is overdue
        fault_check_trip();
    }
    // Check if persistent FAULT condition has been resolved and the power supply can start up again
    else {
    // if a FAULT condition is active, wait for restart opportunity
        fault_check_release();
    }
    
    return(1);
}

inline volatile uint16_t fault_check_test(void) {

    volatile int16_t sign_buf = 0;
    
    // Return if fault check is not possible
    if (!sepic.status.flags.adc_active) {
        return(0);  
    }
    
    // Check if input voltage is below minimum
    if(*uvlo.obj < uvlo.trip_level) {
        if(uvlo.counter++ > uvlo.trip_cnt_threshold) {
            uvlo.counter = uvlo.trip_cnt_threshold + 1;
        }
    }
    else if(*uvlo.obj > uvlo.reset_level) {
        uvlo.counter = 0;
    }

    // Check if input voltage is above maximum
    if(*ovlo.obj > ovlo.trip_level) {
        if(ovlo.counter++ > ovlo.trip_cnt_threshold) {
            ovlo.counter = ovlo.trip_cnt_threshold + 1;
        }
    }
    else if(*ovlo.obj < ovlo.reset_level) {
        ovlo.counter = 0;
    }

    // Check if output voltage is above maximum
    if(*ovp.obj > ovp.trip_level) {
        if(ovp.counter++ > ovp.trip_cnt_threshold) {
            ovp.counter = ovp.trip_cnt_threshold + 1;
        }
    }
    else if(*ovp.obj < ovp.reset_level) {
        ovp.counter = 0;
    }

    // Check if output voltage is in regulation
    if(c2p2z_sepic.status.flag.enable) {

        // Determine deviation between controller reference and output voltage
        sign_buf = abs((int16_t)*reg.obj - (int16_t)*c2p2z_sepic.ptrControlReference);

        // Check deviation for defined limits
        if(sign_buf > reg.trip_level) {
            if(reg.counter++ > reg.trip_cnt_threshold) {
                reg.counter = reg.trip_cnt_threshold + 1;
            }
        }
        else if(sign_buf < reg.trip_level) {
            reg.counter = 0;
        }

    }
    else {
        reg.counter = 0;
    }
    
    return(1);
    
}

inline volatile uint16_t fault_check_trip(void) {

    // Check fault counters for fault trip conditions
    if( (uvlo.counter > uvlo.trip_cnt_threshold) ||
        (ovlo.counter > ovlo.trip_cnt_threshold) ||
        (ovp.counter > ovp.trip_cnt_threshold)   ||
        (reg.counter > reg.trip_cnt_threshold)
      ) {
            sepic.status.flags.fault_active = true;    // Set Global FAULT flag (causes the power controller to shut down)
            sepic.soft_start.phase = SEPIC_SS_STANDBY; // Reset power controller to STANDBY waiting for recovery
    }
    
    return(1);
    
}

inline volatile uint16_t fault_check_release(void) {

    // If all fault counters have been reset to ZERO
    if( (uvlo.counter == 0) && (ovlo.counter == 0) && (ovp.counter == 0) ) {
        
        release_counter++;  // Increment RELEASE DELAY counter

        // If release delay counter is longer than all fault release counter thresholds
        if( (release_counter > uvlo.reset_cnt_threshold) && 
            (release_counter > ovlo.reset_cnt_threshold) && 
            (release_counter > ovp.reset_cnt_threshold)  &&
            (release_counter > reg.reset_cnt_threshold) 
            ) {
                sepic.status.flags.fault_active = false;    // Reset global FAULT flag
                release_counter = 0;                        // Reset REALEASE counter
        }
    }
    
    return(1);
    
}

volatile uint16_t fault_check_init(void) {

    // Under Voltage Lock-Out (UVLO)
    uvlo.counter = 0;                  // Reset FAULT occurrence counter
    uvlo.obj = &sepic.data.v_in;       // Monitor input voltage
    uvlo.reset_level = (SEPIC_VIN_UVLO + SEPIC_VIN_HYST);    // wait for 1000 ms before restart attempt
    uvlo.reset_cnt_threshold = SEPIC_RCVRY_DLY;    // wait for 1000 ms before restart attempt
    uvlo.trip_level = SEPIC_VIN_UVLO;    // wait for 1000 ms before restart attempt
    uvlo.trip_cnt_threshold = SEPIC_FLTTRP_DLY;    // wait for 1 ms before shutting down

    // Over Voltage Lock-Out (OVLO)
    ovlo.counter = 0;                  // Reset FAULT occurrence counter
    ovlo.obj = &sepic.data.v_in;       // Monitor input voltage
    ovlo.reset_level = (SEPIC_VIN_OVLO - SEPIC_VIN_HYST);    // wait for 1000 ms before restart attempt
    ovlo.reset_cnt_threshold = SEPIC_RCVRY_DLY;    // wait for 1000 ms before restart attempt
    ovlo.trip_level = SEPIC_VIN_OVLO;    // wait for 1000 ms before restart attempt
    ovlo.trip_cnt_threshold = SEPIC_FLTTRP_DLY;    // wait for 1 ms before shutting down

    // Over Voltage Lock-Out (OVLO)
    ovp.counter = 0;                            // Reset FAULT occurrence counter
    ovp.obj = &sepic.data.v_out;                // Monitor output voltage
    ovp.reset_level = (SEPIC_VOUT_OVP - SEPIC_VOUT_HYST);    // Level at which power supply recovers
    ovp.reset_cnt_threshold = SEPIC_RCVRY_DLY;  // wait for 1000 ms before restart attempt
    ovp.trip_level = SEPIC_VOUT_OVP;            // Level at which power supply shuts down
    ovp.trip_cnt_threshold = SEPIC_FLTTRP_DLY;  // wait for 1 ms before shutting down
    
    // Regulation Deviation Monitoring (VOUT vs. VREF)
    reg.counter = 0;                            // Reset FAULT occurrence counter
    reg.obj = &sepic.data.v_out;                // Monitor output voltage
    reg.reset_level = SEPIC_VOUT_LDEVI;         // Lower allowed deviation before FAULT trips
    reg.reset_cnt_threshold = SEPIC_RCVRY_DLY;  // wait for 1000 ms before restart attempt
    reg.trip_level = SEPIC_VOUT_UDEVI;          // Upper allowed deviation before FAULT trips
    reg.trip_cnt_threshold = SEPIC_FLTTRP_DLY;  // wait for 1 ms before shutting down
    
    return(1);
}
