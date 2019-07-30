/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   globals.h
 * Author: M91406
 * Comments: global defines of this application
 * Revision history: 
 * v1.0 initial version
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef APPLICATION_GLOBALS_HEADER_H
#define	APPLICATION_GLOBALS_HEADER_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// List of user included header files
#include "init/init_fosc.h"
#include "init/init_timer1.h"
#include "init/init_gpio.h"

#include "init/init_acmp.h"
#include "init/init_adc.h"
#include "init/init_pwm.h"

#include "pwr_control_sepic.h"
#include "ext_reference.h"


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

/*!Microcontroller and Hardware Abstraction
 * *************************************************************************************************
 * Summary:
 * Global defines for device specific parameters
 * 
 * Description:
 * This section is used to define device specific parameters like clock settings, analog 
 * reference and resolution of ADC or DAC. 
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers 
 * to be written to SFRs
 * 
 * *************************************************************************************************/

    
/*!Device Clock Settings
 * *************************************************************************************************
 * Summary:
 * Global defines for device clock settings
 * 
 * Description:
 * This section is used to define device specific parameters related to the core clock and 
 * auxiliary clock used to drive PWM, ADC and DAC.
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers 
 * to be written to SFRs
 * 
 * *************************************************************************************************/

#define CPU_CLK_FREQUENCY       100000000   // CPU frequency in [Hz]
#define AUX_CLK_FREQUENCY       500000000   // Auxiliary Clock Frequency in [Hz]
#define PWM_CLK_FREQUENCY       500000000   // PWM Generator Base Clock Frequency in [Hz]
    
/*!State Machine Settings
 * *************************************************************************************************
 * Summary:
 * Global defines for state-machine specific parameters
 * 
 * Description:
 * This section is used to define state-machine settings such as the main execution call interval. 
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers to be 
 * written to SFRs and variables.
 * 
 * *************************************************************************************************/
    
#define MAIN_EXECUTION_PERIOD    100e-6     // main state machine pace period in [sec]
#define MAIN_EXEC_PER           (uint16_t)((CPU_CLK_FREQUENCY * MAIN_EXECUTION_PERIOD)-1.0)

/*!ADC Settings
 * *************************************************************************************************
 * Summary:
 * Global defines for specific parameters of the device ADC
 * 
 * Description:
 * This section is used to define device specific parameters of ADC reference, resolution
 * and granularity to calculate register values representing physical voltages.
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers 
 * to be written to SFRs
 * 
 * *************************************************************************************************/

#define ADC_REF              3.300 // ADC reference voltage in V
#define ADC_RES              12.0  // ADC resolution in [bit]
#define ADC_GRAN             (float)(ADC_REF / pow(2.0, ADC_RES)) // ADC granularity in [V/tick]

/*!ADC Settings
 * *************************************************************************************************
 * Summary:
 * Global defines for specific parameters of the device DAC
 * 
 * Description:
 * This section is used to define device specific parameters of ADC reference, resolution,
 * granularity and slope timer frequency to calculate register values representing physical voltages.
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers 
 * to be written to SFRs
 * 
 * *************************************************************************************************/

// Feedback Loop Output Settings
#define DAC_MINIMUM      0.650   // Minimum DAC voltage in [V]
#define DAC_MAXIMUM      3.100   // Maximum DAC voltage in [V]
#define SLEW_RATE        0.120   // Compensation ramp in [V/usec] (SLPxDAT is calculated below)
                                 // (slope resolution is ~12.4 mV/tick @ 4ns tick rate)

//-------    
#define DAC_REF         (double)3.300           // DAC reference voltage (usually AVDD)
#define DAC_RES         (double)12.00           // DAC resolution in [bit]
#define DAC_GRAN        (double)(DAC_REF / pow(2, DAC_RES))  // DAC granularity in [V/tick]
#define FDAC            (double)AUX_CLK_FREQUENCY   // DAC input clock in Hz
#define DACCLK          (double)(2.0/FDAC)      // DAC input clock (period) selected in [sec]

//-------    
#define DAC_CBLANK_TIME 300e-9  // Comparator Blanking Period in [ns] applied when DAC reference changes 
#define DAC_T_RESET     300e-9  // Transition Mode Duration
#define DAC_T_SETTLING  340e-9  // Time from Start of Transition Mode until Steady-State Filter is Enabled

// Device-specific DAC settings
#define DAC_MIN         (uint16_t)(DAC_MINIMUM / DAC_GRAN)
#define DAC_MAX         (uint16_t)(DAC_MAXIMUM / DAC_GRAN)
#define DAC_SLOPE_RATE  (uint16_t)((16.0 * (SLEW_RATE / DAC_GRAN) / (1.0e-6/DACCLK)) + 1.0) // SLOPE DATA in [DAC-ticks/CLK-tick]
#define DAC_TMCB        (uint16_t)((DAC_CBLANK_TIME * FDAC)/2.0)    // Leading edge period for the comparator when slope re-settles to its initial value
#define DAC_TMODTIME    (uint16_t)((DAC_T_RESET * FDAC)/2.0)            // Transition Mode Duration
#define DAC_SSTIME      (uint16_t)((DAC_T_SETTLING * FDAC)/2.0)         // Time from Start of Transition Mode until Steady-State Filter is Enabled


/*!PWM Settings
 * *************************************************************************************************
 * Summary:
 * Global defines for specific parameters of the device PWM
 * 
 * Description:
 * This section is used to define device specific parameters of PWM frequency, may duty ratio, 
 * leading edge blanking, slope compensation and ADC triggers.
 * granularity and slope timer frequency to calculate register values representing physical voltages.
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers 
 * to be written to SFRs
 * 
 * *************************************************************************************************/
    
#define SWITCHING_FREQUENCY         350e+3      // Power Supply Switching Frequency in [Hz]
//------ macros
#define SWITCHING_PERIOD            (1.0/SWITCHING_FREQUENCY)   // Power Supply Switching Period in [sec]
#define PWM_RES                     (1.0/AUX_CLK_FREQUENCY)         // PWM Resolution
#define PWM_PERIOD                  (uint16_t)(SWITCHING_PERIOD / PWM_RES)      // Measured in [tick = 2ns]
//------ 

#define MAXIMUM_DUTY_RATIO          0.80    // Maximum Duty Ratio in [%]
#define LEB_PERIOD                  200e-9  // Leading Edge Blanking period in [sec]
#define SLOPE_START_DELAY           150e-9  // Delay in {sec] until the slope compensation ramp starts
#define SLOPE_STOP_DELAY            0.85    // Delay in {sec] until the slope compensation ramp stops
#define VOUT_ADC_TRIGGER_DELAY      ((0.80 * SWITCHING_PERIOD) - 1000e-9) // ADC trigger delay in [sec] used to sample output voltage
#define PWM_MAIN_PHASE_SHIFT        50e-9   // Switching frequency phase shift in [sec]
#define PWM_AUX_PHASE_SHIFT         150e-9  // Switching frequency phase shift in [sec]
    
//------ macros
#define MAX_DUTY_CYCLE              (uint16_t)(PWM_PERIOD * MAXIMUM_DUTY_RATIO)     // This sets the maximum duty cycle
#define PWM_LEB_PERIOD              (uint16_t)(LEB_PERIOD / PWM_RES)  // Leading Edge Blanking = n x PWM resolution (here: 50 x 2ns = 100ns)
#define PWM_PHASE_SHIFT             (uint16_t)(PWM_MAIN_PHASE_SHIFT / PWM_RES)   // Leading Edge Blanking = n x PWM resolution (here: 50 x 2ns = 100ns)
    
#define VOUT_ADCTRIG                (uint16_t)(VOUT_ADC_TRIGGER_DELAY / PWM_RES)    // ADC trigger delay in [ticks] used to sample output voltage
#define SLP_TRIG_START              (uint16_t)(SLOPE_START_DELAY / PWM_RES)         // Delay in {sec] until the slope compensation ramp starts
#define SLP_TRIG_STOP               (uint16_t)(PWM_PERIOD * SLOPE_STOP_DELAY) // Delay in {sec] until the slope compensation ramp stops

#define PWM_DEAD_TIME_RISING        0   // Rising edge dead time [2ns]
#define PWM_DEAD_TIME_FALLING       0   // Falling edge dead time [2ns]

    
/*!Hardware Abstraction
 * *************************************************************************************************
 * Summary:
 * Global defines for hardware specific parameters
 * 
 * Description:
 * This section is used to define hardware specific parameters such as output voltage dividers,
 * reference levels or feedback gains. Pre-compiler macros are used to translate physical  
 * values into binary (integer) numbers to be written to SFRs
 * 
 * *************************************************************************************************/
    
#define SEPIC_VOUT_NOMINAL  15.0            // Nominal output voltage

#define SEPIC_VOUT_R1       (2.0 * 2.87)    // Upper voltage divider resistor in kOhm
#define SEPIC_VOUT_R2       (1.0)           // Lower voltage divider resistor in kOhm

#define SEPIC_VOUT_FB_GAIN  (float)((SEPIC_VOUT_R2) / (SEPIC_VOUT_R1 + SEPIC_VOUT_R2))
#define SEPIC_V_OUT_REF     (uint16_t)(SEPIC_VOUT_NOMINAL * SEPIC_VOUT_FB_GAIN / ADC_GRAN)

/*!Startup Behavior
 * *************************************************************************************************
 * Summary:
 * Global defines for soft-start specific parameters
 * 
 * Description:
 * This section is used to define power supply startup timing setting. The soft-start sequence 
 * is part of the power controller. It allows to program specific timings for Power On Delay,
 * Ramp Period and Power Good Delay. After the startup has passed these three timing periods,
 * the power supply is ending up in "normal" operation, continuously regulating the output until 
 * a fault is detected or the operating state is changed for any other reason.
 * 
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers to 
 * be written to SFRs and variables.
 * 
 * *************************************************************************************************/

#define SEPIC_POWER_ON_DELAY          500e-3      // power on delay in [sec]
#define SEPIC_RAMP_PERIOD             50e-3         // ramp period in [sec]
#define SEPIC_POWER_GOOD_DELAY        100e-3        // power good in [sec]

#define SEPIC_PODLY     (uint16_t)((SEPIC_POWER_ON_DELAY / MAIN_EXECUTION_PERIOD)-1.0)
#define SEPIC_RPER      (uint16_t)((SEPIC_RAMP_PERIOD / MAIN_EXECUTION_PERIOD)-1.0)
#define SEPIC_PGDLY     (uint16_t)((SEPIC_POWER_GOOD_DELAY / MAIN_EXECUTION_PERIOD)-1.0)
#define SEPIC_REF_STEP  (uint16_t)((SEPIC_V_OUT_REF / (SEPIC_RPER + 1.0)))

/*!SEPIC_POWER_CONTROLLER_t data structure sepic
 * *************************************************************************************************
 * Summary:
 * Global SEPIC Signal Mapping
 * 
 * Description:
 * The SEPIC needs one PWM output, one ADC input to sample output voltage and one analog
 * feedback signal for the peak current feedback signal. In addition, the following internal 
 * peripheral instances need to be defined:
 * 
 *     - Main PWM Generator Instance
 *     - Auxiliary PWM Generator Instance
 *     - ADC trigger register
 *     - ADC input number
 *     - Comparator/DAC Instance
 *     - Comparator Input Selection
 *  
 * *************************************************************************************************/

#define _SEPIC_VOUT_ADCInterrupt        _ADCAN16Interrupt   
#define SEPIC_VOUT_ADCBUF               ADCBUF16
#define SEPIC_VOUT_ADCTRIG              PG2TRIGA
#define SEPIC_VOUT_FEEDBACK_OFFSET      0
#define SEPIC_DAC_VREF_REGISTER         DAC1DATH

/*!External Reference Voltage Input
 * *************************************************************************************************
 * Summary:
 * Global option to enable/disable the external reference voltage input of the SEPIC board
 * 
 * Description:
 * The SEPIC board offers an external reference voltage input. The input voltage between 0 to 3.3V
 * is read from this pin being interpreted as adjustment range between 0 to 100%.
 * The effective reference range needs to be specified using the defines below, where
 * 
 *    - V_REF_MINIMUM defines the reference value when the external reference voltage input 
 *                    reads 0V (=0 ticks)
 *    - V_REF_MAXIMUM defines the reference value when the external reference voltage input
 *                    reads 3.3V (=4095 ticks).
 * 
 * Both values need to be specified as SEPIC output voltage level in [V]. The macros will 
 * calculate the effective integer numbers based on the ADC and voltage divider settings specified 
 * in the hardware- and microcontroller abstraction sections of this file
 * 
 * *************************************************************************************************/

#define USE_EXTERNAL_REFERENCE  true    // Enable/disable external reference voltage input
    
#define V_REF_MINIMUM       9.0  // lower output voltage limit in [V]
#define V_REF_MAXIMUM       18.0 // upper output voltage limit in [V]
    
#define V_REF_MIN           (uint16_t)(V_REF_MINIMUM * SEPIC_VOUT_FB_GAIN / ADC_GRAN)
#define V_REF_MAX           (uint16_t)(V_REF_MAXIMUM * SEPIC_VOUT_FB_GAIN / ADC_GRAN)
#define V_REF_DIFF          (V_REF_MAX - V_REF_MIN)
    
/*!SEPIC_POWER_CONTROLLER_t data structure sepic
 * *************************************************************************************************
 * Summary:
 * Global data objct for the SEPIC 
 * 
 * Description:
 * the 'sepic' data object holds all status, control and monitoring values of the SEPIC power 
 * controller. The SEPIC_POWER_CONTROLLER_t data structure is defined in sepic_pwr_contro.h.
 * Please refer to the comments on top of this file for further information.
 *  
 * *************************************************************************************************/

extern volatile SEPIC_POWER_CONTROLLER_t sepic;

    
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* APPLICATION_GLOBALS_HEADER_H */

