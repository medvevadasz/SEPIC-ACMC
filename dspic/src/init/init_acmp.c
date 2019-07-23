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



volatile uint16_t init_acmp_module(void) {

    // Make sure power is turned on to comparator module #1 & #2
//    PMD7bits.CMP1MD = 0; // Comparator 1 Module Powetr Disable: Comparator 1 module is enabled
//    PMD7bits.CMP2MD = 0; // Comparator 2 Module Powetr Disable: Comparator 2 module is enabled
    PMD7bits.CMP3MD = 0; // Comparator 3 Module Powetr Disable: Comparator 3 module is enabled
    
    
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

volatile uint16_t init_sepic_acmp(void) {

    // DACxCONL: DACx CONTROL LOW REGISTER
    DAC3CONLbits.DACEN = 0; // Individual DACx Module Enable: Disables DACx module during configuration
    DAC3CONLbits.IRQM = 0b00; // Interrupt Mode Selection: Interrupts are disabled
    DAC3CONLbits.CBE = 1; // Comparator Blank Enable: Enables the analog comparator output to be blanked (gated off) during the recovery transition following the completion of a slope operation
    DAC3CONLbits.DACOEN = 1; // DACx Output Buffer Enable: DACx analog voltage is connected to the DACOUT1 pin (RA3/TP35 on DPSK3)
    DAC3CONLbits.FLTREN = 0; // Comparator Digital Filter Enable: Digital filter is disabled
    // DAC3CONLbits.CMPSTAT (read only bit)
    DAC3CONLbits.CMPPOL = 0; // Comparator Output Polarity Control: Output is non-inverted
    DAC3CONLbits.INSEL = 0b001; // Comparator Input Source Select: feedback is connected to CMPxB input pin
    DAC3CONLbits.HYSPOL = 0; // Comparator Hysteresis Polarity Selection: Hysteresis is applied to the rising edge of the comparator output
    DAC3CONLbits.HYSSEL = 0b11; // Comparator Hysteresis Selection: 45 mv hysteresis (0 = 0mV, 1 = 15mV, 2 = 30mV, 3 = 45mV)
    
    // DACxCONH: DACx CONTROL HIGH REGISTER
    
    // ***********************************************
    // ToDo: CHECK DAC LEB PERIOD TO BE CORRECT AND DOESN'T CREATE CONFLICTS
    DAC3CONHbits.TMCB = LEB_PER_COMP; // DACx Leading-Edge Blanking: period for the comparator
    // ***********************************************
        
    // DACxDATH: DACx DATA HIGH REGISTER
    DAC3DATH = (DACDATH & 0x0FFF); // DACx Data: This register specifies the high DACx data value. Valid values are from 205 to 3890.
    DAC3DATL = (DACDATL & 0x0FFF); // DACx Low Data
        
    // SLPxCONH: DACx SLOPE CONTROL HIGH REGISTER
    SLP3CONHbits.SLOPEN = 1; // Slope Function Enable/On: Enables slope function
    SLP3CONHbits.HME = 0; // Hysteretic Mode Enable: Disables Hysteretic mode for DACx
    SLP3CONHbits.TWME = 0; // Triangle Wave Mode Enable: Disables Triangle Wave mode for DACx
    SLP3CONHbits.PSE = 0; // Positive Slope Mode Enable: Slope mode is negative (decreasing)
    
    // SLPxCONL: DACx SLOPE CONTROL LOW REGISTER
    SLP3CONLbits.HCFSEL = 0b0000; // Hysteretic Comparator Function Input Selection: (none)
    SLP3CONLbits.SLPSTOPA = 0b0001; // Slope Stop A Signal Selection: PWM1 Trigger 2
    SLP3CONLbits.SLPSTOPB = 0b0001; // Slope Stop B Signal Selection: CMP1 Out
//    SLP1CONLbits.SLPSTOPB = 0b0000; // Slope Stop B Signal Selection: 0
    SLP3CONLbits.SLPSTRT = 0b0001; // Slope Start Signal Selection: PWM1 Trigger 1
    
    // ToDo: CHECK SLP3DAT in conjunction with DAC3DATH and DAC3DATL
    // DAC3DATL should be reserved/valid only in hysteretic and triangular mode
    // So for normal slope compensation the valid registers should be DAC3DATH as reference level
    // and SLP1DAT for the slew rate (V/usec translated in DAC-ticks/time-ticks)
    // Previous configurations have shown that this might not be true, so please revisit this setting.
    
    // SLPxDAT: DACx SLOPE DATA REGISTER
//    SLP3DAT = 500; // Slope Ramp Rate Value
    SLP3DAT = SLOPE_RATE; // Slope Ramp Rate Value
            
        
    return(1);
}

volatile uint16_t launch_sepic_acmp(void) {
    
    DAC3CONLbits.DACEN = 1; // Individual DACx Module Enable: Enables DAC3 module 
    DACCTRL1Lbits.DACON = 1; // Common DAC Module Enable: Enables all enabled DAC modules
    
    return(1);
}

