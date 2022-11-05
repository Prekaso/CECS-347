// SysTickInterrupt.c
// Documentation
// Description: Initialize SysTick timer for 204ms delay with
//         interrupt enabled and priority 1 assuming 80MHz clock

#include "tm4c123gh6pm.h"

// Initialize SysTick timer for 0.034s delay with interrupt enabled
void SysTickInterrupt_Init(void){
    NVIC_ST_CTRL_R = 0;                                    // disable SysTick during setup
    NVIC_ST_RELOAD_R = (16777216) - 1;            // number of counts to wait 204ms (assuming 80MHz clock)
    //NVIC_ST_RELOAD_R = (2720000*6) - 1;            // number of counts to wait 34ms (assuming 80MHz clock)
    NVIC_ST_CURRENT_R = 0;                            // any write to CURRENT clears
    NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x1FFFFFFF)|0x20000000;    // priority 1
    NVIC_ST_CTRL_R = 0x07;                            // enable SysTick with core and interrupts
}