/*
 * File:   init_gpio.c
 * Author: M91406
 *
 * Created on July 8, 2019, 6:26 PM
 */


#include <xc.h>
#include "init_gpio.h"

volatile uint16_t init_gpio(void) {
    
    ANSELA = 0x0000;
    ANSELB = 0x0000;
    ANSELC = 0x0000;
    ANSELD = 0x0000;
    
    DBGLED_INIT;
    DBGLED_GN_INIT;
    DBGLED_RD_INIT;
    
    DBGPIN_1_INIT;
    DBGPIN_2_INIT;
    DBGPIN_3_INIT;
    
    return(1);
}
