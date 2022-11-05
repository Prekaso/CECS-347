#include "tm4c123gh6pm.h"
#include "SysTick.h"
#include "PLL.h"

#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))
#define NVIC_ST_CTRL_COUNT      0x00010000  // Count flag
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_M        0x00FFFFFF  // Counter load value


// Initialize SysTick with busy wait running at bus clock.
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_RELOAD_R = 0x0098967F;  // maximum reload value
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it
	//NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0x1FFFFFF) | 0x50000000;
                                        // enable SysTick with core clock
  NVIC_ST_CTRL_R |= NVIC_ST_CTRL_INTEN+NVIC_ST_CTRL_CLK_SRC;
}

void Systick_On(void) {
	NVIC_ST_CTRL_R |=NVIC_ST_CTRL_ENABLE;
}

void Systick_Off(void){
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;
}


