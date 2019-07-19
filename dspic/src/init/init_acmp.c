/*
 * File:   init_acmp.c
 * Author: M91406
 *
 * Created on July 9, 2019, 11:12 AM
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "init_acmp.h"

#define DACDATH_BUCK    0       // DAC value for the buck the slope starts from
#define DACDATH_BOOST   0       // DAC value for the boost the slope starts from
#define DACDATL_BUCK    205     // Set this to minimum in Slope mode
#define DACDATL_BOOST   205     // Set this to minimum in Slope mode

#define LEB_PER_COMP    50      // Leading edge period for the comparator when slope re-settles to its initial value

#define TMOD_DURATION   75      // Transition Mode Duration
#define SS_DURATION     85      // Time from Start of Transition Mode until Steady-State Filter is Enabled

#define SLOPE_RATE      43      // Slope Ramp Rate Value

volatile uint16_t init_acmp_module(void) {

    // Make sure power is turned on to comparator module #1 & #2
    PMD7bits.CMP1MD = 0; // Comparator 1 Module Powetr Disable: Comparator 1 module is enabled
    PMD7bits.CMP2MD = 0; // Comparator 1 Module Powetr Disable: Comparator 1 module is enabled
    
    // Turn off all Comparator/DAC modules during configuration
    DACCTRL1Lbits.DACON = 0; // Common DAC Module Enable: Disables all DAC modules
    DAC1CONLbits.DACEN = 0; // Individual DAC1 Module Enable: Disables DAC1 module during configuration
    DAC2CONLbits.DACEN = 0; // Individual DAC2 Module Enable: Disables DAC2 module during configuration
    DAC3CONLbits.DACEN = 0; // Individual DAC3 Module Enable: Disables DAC3 module during configuration

    // VREGCON: VOLTAGE REGULATOR CONTROL REGISTER
    VREGCONbits.LPWREN = 0; // Low-Power Mode Enable: Voltage regulators are in Full Power mode
    VREGCONbits.VREG1OV = 0b00; // Regulator #1 Low-Power Mode Enable: VOUT = 1.5 * VBG = 1.2V
    VREGCONbits.VREG2OV = 0b00; // Regulator #2 Low-Power Mode Enable: VOUT = 1.5 * VBG = 1.2V
    VREGCONbits.VREG3OV = 0b00; // Regulator #3 Low-Power Mode Enable: VOUT = 1.5 * VBG = 1.2V

    // DACCTRL1L: DAC CONTROL 1 LOW REGISTER
    DACCTRL1Lbits.DACSIDL = 0; // DAC Stop in Idle Mode: Continues module operation in Idle mode
    DACCTRL1Lbits.CLKSEL = 0b10; // DAC Clock Source Selection: AFPLLO
    DACCTRL1Lbits.CLKDIV = 0b00; // DAC Clock Divider: Divider = 1:1
    DACCTRL1Lbits.FCLKDIV = 0b000; // Comparator Filter Clock Divider: Divider = 1:1
    
    // DACCTRL2H/L: DAC CONTROL 2 HIGH and DAC CONTROL 2 LOW REGISTER
    // Settings = 2 x 
    DACCTRL2Lbits.TMODTIME = (TMOD_DURATION  & 0x03FF); // Transition Mode Duration (default 0x55 = 340ns @ 500 MHz)
    DACCTRL2Hbits.SSTIME = (SS_DURATION & 0x0FFF); // Time from Start of Transition Mode until Steady-State Filter is Enabled (default 0x8A = 552ns @ 500 MHz)
    
    return(1);
}

volatile uint16_t init_buck_acmp(void) {

    // DACxCONL: DACx CONTROL LOW REGISTER
    DAC1CONLbits.DACEN = 0; // Individual DACx Module Enable: Disables DACx module during configuration
    DAC1CONLbits.IRQM = 0b00; // Interrupt Mode Selection: Interrupts are disabled
    DAC1CONLbits.CBE = 1; // Comparator Blank Enable: Enables the analog comparator output to be blanked (gated off) during the recovery transition following the completion of a slope operation
    DAC1CONLbits.DACOEN = 1; // DACx Output Buffer Enable: DACx analog voltage is connected to the DACOUT1 pin (RA3/TP35 on DPSK3)
    DAC1CONLbits.FLTREN = 0; // Comparator Digital Filter Enable: Digital filter is disabled
    // DAC1CONLbits.CMPSTAT (read only bit)
    DAC1CONLbits.CMPPOL = 0; // Comparator Output Polarity Control: Output is non-inverted
    DAC1CONLbits.INSEL = 0b000; // Comparator Input Source Select: feedback is connected to CMPxA input pin
    DAC1CONLbits.HYSPOL = 0; // Comparator Hysteresis Polarity Selection: Hysteresis is applied to the rising edge of the comparator output
    DAC1CONLbits.HYSSEL = 0b11; // Comparator Hysteresis Selection: 45 mv hysteresis (0 = 0mV, 1 = 15mV, 2 = 30mV, 3 = 45mV)
    
    // DACxCONH: DACx CONTROL HIGH REGISTER
    
    // ***********************************************
    // ToDo: CHECK DAC LEB PERIOD TO BE CORRECT AND DOESN'T CREATE CONFLICTS
    DAC1CONHbits.TMCB = LEB_PER_COMP; // DACx Leading-Edge Blanking: period for the comparator
    // ***********************************************
        
    // DACxDATH: DACx DATA HIGH REGISTER
    DAC1DATH = (DACDATH_BUCK & 0x0FFF); // DACx Data: This register specifies the high DACx data value. Valid values are from 205 to 3890.
    DAC1DATL = (DACDATL_BUCK & 0x0FFF); // DACx Low Data
        
    // SLPxCONH: DACx SLOPE CONTROL HIGH REGISTER
    SLP1CONHbits.SLOPEN = 1; // Slope Function Enable/On: Enables slope function
    SLP1CONHbits.HME = 0; // Hysteretic Mode Enable: Disables Hysteretic mode for DACx
    SLP1CONHbits.TWME = 0; // Triangle Wave Mode Enable: Disables Triangle Wave mode for DACx
    SLP1CONHbits.PSE = 0; // Positive Slope Mode Enable: Slope mode is negative (decreasing)
    
    // SLPxCONL: DACx SLOPE CONTROL LOW REGISTER
    SLP1CONLbits.HCFSEL = 0b0000; // Hysteretic Comparator Function Input Selection: (none)
    SLP1CONLbits.SLPSTOPA = 0b0001; // Slope Stop A Signal Selection: PWM1 Trigger 2
    SLP1CONLbits.SLPSTOPB = 0b0001; // Slope Stop B Signal Selection: CMP1 Out
//    SLP1CONLbits.SLPSTOPB = 0b0000; // Slope Stop B Signal Selection: 0
    SLP1CONLbits.SLPSTRT = 0b0001; // Slope Start Signal Selection: PWM1 Trigger 1
    
    // ToDo: CHECK SLP1DAT in conjunction with DAC1DATH and DAC1DATL
    // DAC1DATL should be reserved/valid only in hysteretic and triangular mode
    // So for normal slope compensation the valid registers should be DAC1DATH as reference level
    // and SLP1DAT for the slew rate (V/usec translated in DAC-ticks/time-ticks)
    // Previous configurations have shown that this might not be true, so please revisit this setting.
    
    // SLPxDAT: DACx SLOPE DATA REGISTER
//    SLP1DAT = 500; // Slope Ramp Rate Value
    SLP1DAT = SLOPE_RATE; // Slope Ramp Rate Value
            
        
    return(1);
}

volatile uint16_t launch_buck_acmp(void) {
    
    DAC1CONLbits.DACEN = 1; // Individual DACx Module Enable: Enables DAC1 module 
    DACCTRL1Lbits.DACON = 1; // Common DAC Module Enable: Enables all enabled DAC modules
    
    return(1);
}

volatile uint16_t init_boost_acmp(void) {
    

    // DACxCONL: DACx CONTROL LOW REGISTER
    DAC2CONLbits.DACEN = 0; // Individual DACx Module Enable: Disables DACx module during configuration
    DAC2CONLbits.IRQM = 0b00; // Interrupt Mode Selection: Interrupts are disabled
    DAC2CONLbits.CBE = 1; // Comparator Blank Enable: Enables the analog comparator output to be blanked (gated off) during the recovery transition following the completion of a slope operation
    DAC2CONLbits.DACOEN = 1; // DACx Output Buffer Enable: disabled for this module
    DAC2CONLbits.FLTREN = 0; // Comparator Digital Filter Enable: Digital filter is disabled
    // DAC2CONLbits.CMPSTAT (read only bit)
    DAC2CONLbits.CMPPOL = 0; // Comparator Output Polarity Control: Output is non-inverted
    DAC2CONLbits.INSEL = 0b011; // Comparator Input Source Select: feedback is connected to CMPxD input pin
    DAC2CONLbits.HYSPOL = 0; // Comparator Hysteresis Polarity Selection: Hysteresis is applied to the rising edge of the comparator output
    DAC2CONLbits.HYSSEL = 0b11; // Comparator Hysteresis Selection: 45 mv hysteresis (0 = 0mV, 1 = 15mV, 2 = 30mV, 3 = 45mV)
    
    // DACxCONH: DACx CONTROL HIGH REGISTER
    
    // ***********************************************
    // ToDo: CHECK DAC LEB PERIOD TO BE CORRECT AND DOESN'T CREATE CONFLICTS
    DAC2CONHbits.TMCB = LEB_PER_COMP; // DACx Leading-Edge Blanking: period for the comparator
    // ***********************************************
        
    // DACxDATH: DACx DATA HIGH REGISTER
    DAC2DATH = (DACDATH_BOOST & 0x0FFF); // DACx Data: This register specifies the high DACx data value. Valid values are from 205 to 3890.
    DAC2DATL = (DACDATL_BOOST & 0x0FFF); // DACx Low Data
        
    // SLPxCONH: DACx SLOPE CONTROL HIGH REGISTER
    SLP2CONHbits.SLOPEN = 1; // Slope Function Enable/On: Enables slope function
    SLP2CONHbits.HME = 0; // Hysteretic Mode Enable: Disables Hysteretic mode for DACx
    SLP2CONHbits.TWME = 0; // Triangle Wave Mode Enable: Disables Triangle Wave mode for DACx
    SLP2CONHbits.PSE = 0; // Positive Slope Mode Enable: Slope mode is negative (decreasing)
    
    // SLPxCONL: DACx SLOPE CONTROL LOW REGISTER
    SLP2CONLbits.HCFSEL = 0b0000; // Hysteretic Comparator Function Input Selection: (none)
    SLP2CONLbits.SLPSTOPA = 0b0010; // Slope Stop A Signal Selection: PWM2 Trigger 2
    SLP2CONLbits.SLPSTOPB = 0b0010; // Slope Stop B Signal Selection: CMP2 Out
    SLP2CONLbits.SLPSTRT = 0b0010; // Slope Start Signal Selection: PWM2 Trigger 1
    
    // ToDo: CHECK SLP2DAT in conjunction with DAC2DATH and DAC2DATL
    // DAC2DATL should be reserved/valid only in hysteretic and triangular mode
    // So for normal slope compensation the valid registers should be DAC2DATH as reference level
    // and SLP2DAT for the slew rate (V/usec translated in DAC-ticks/time-ticks)
    // Previous configurations have shown that this might not be true, so please revisit this setting.
    
    // SLPxDAT: DACx SLOPE DATA REGISTER
    SLP2DAT = SLOPE_RATE; // Slope Ramp Rate Value
            
        
    
    return(1);
}

volatile uint16_t launch_boost_acmp(void) {
    
    DAC2CONLbits.DACEN = 1; // Individual DACx Module Enable: Enables DAC2 module 
    DACCTRL1Lbits.DACON = 1; // Common DAC Module Enable: Enables all enabled DAC modules
    
    return(1);
}

