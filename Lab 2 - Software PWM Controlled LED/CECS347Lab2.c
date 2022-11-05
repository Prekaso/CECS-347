#include "tm4c123gh6pm.h"

// Constants
#define PERIOD 			1600           	// stay on, change brightness
//#define PERIOD 			16000           	// stay on, change brightness
//#define PERIOD 			160000           	// stay on, Change brightness
//#define PERIOD 			1600000           	// faster blink
//#define PERIOD 			16000000           	// Slow blink
#define MIN_DUTY    PERIOD/10							// minimum duty cycle 10%
#define MAX_DUTY    PERIOD*0.9						// maximum duty cycle 90%
#define DUTY_STEP		PERIOD/10							// duty cycle change for each button press
#define NVIC_EN0_PORTF 0x40000000

// Function prototypes
// External functions for interrupt control defined in startup.s
extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // low power mode

// This function initialize PA5 to output PWM signal and 
// Systick to generate required timing for PWM signal
void PWM_Output_Init(void);

// This function initilizes port F and arm PF4, PF0 for falling edge interrupts
void Switch_Init(void);

// Global variables: 
// H: number of clocks cycles for duty cycle
// L: number of clock cycles for non-duty cycle
unsigned long H,L;

int main(void){
  DisableInterrupts();  // disable interrupts to allow initializations
  PWM_Output_Init();    // output from PF2, SysTick interrupts
  Switch_Init();        // arm PF4, PF0 for falling edge interrupts
  EnableInterrupts();   // enable after initializations are done
  while(1){
    // main program is free to perform other tasks
    WaitForInterrupt(); // low power mode
  }
}

// This function initialize PA5 to output PWM signal and 
// Systick to generate required timing for PWM signal
void PWM_Output_Init(void){
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate clock for port F
  H = L = PERIOD/2;            // 50% duty cycle, assume system clock 16MHz, 
  GPIO_PORTF_AMSEL_R &= ~0x04;  // disable analog functionality on PF2
  GPIO_PORTF_PCTL_R &= ~0x00000F00; // configure PF2 as GPIO
  GPIO_PORTF_DIR_R |= 0x04;     // make PF2 out
//  GPIO_PORTF_DR8R_R |= 0x04;    // Add this line if the PWM is used to drive a DC Motor to enable 8 mA drive on PF2
  GPIO_PORTF_AFSEL_R &= ~0x04;  // disable alt funct on PF2
  GPIO_PORTF_DEN_R |= 0x04;     // enable digital I/O on PF2
  GPIO_PORTF_DATA_R &= ~0x04;   // make PF2 low
  NVIC_ST_CTRL_R = 0;           // disable SysTick during setup
  NVIC_ST_RELOAD_R = L-1;       // reload value for 50% duty cycle
  NVIC_ST_CURRENT_R = 0;        // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x1FFFFFFF)|0x40000000; // bit 31-29 for SysTick, set priority to 2
  NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE|NVIC_ST_CTRL_INTEN|NVIC_ST_CTRL_CLK_SRC;  // enable with core clock and interrupts, start systick timer
}

// Initilize port F and arm PF4, PF0 for falling edge interrupts
void Switch_Init(void){  unsigned long volatile delay;
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // (a) activate clock for port F
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // unlock GPIO Port F
  GPIO_PORTF_CR_R |= 0x11;         // allow changes to PF4,0
  GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4,0 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x11;  //     disable alt funct on PF4,0
  GPIO_PORTF_DEN_R |= 0x11;     //     enable digital I/O on PF4,0
  GPIO_PORTF_PCTL_R &= ~0x000F000F; //  configure PF4,0 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x11;  //     disable analog functionality on PF4,0
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4,0
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4,PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4,PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4,PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flags 4,0
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4,PF0
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF1FFFFF)|0x00400000; // (g) bits:23-21 for PORTF, set priority to 2
  NVIC_EN0_R |= NVIC_EN0_PORTF;      // (h) enable interrupt 30 in NVIC
}

// SysTick ISR:
// 1. Implement timing control for duty cycle and non-duty cycle
// 2. utput a waveform based on current duty cycle
void SysTick_Handler(void){
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;// turn off SysTick to reset reload value
  if(GPIO_PORTF_DATA_R&0x04){   // toggle PF2:previous one is a duty cycle
    GPIO_PORTF_DATA_R &= ~0x04; // make PF2 low
    NVIC_ST_RELOAD_R = L-1;     // reload value for low phase
  } else{ // come from non-duty, going to duty cycle
    GPIO_PORTF_DATA_R |= 0x04;  // make PF2 high
    NVIC_ST_RELOAD_R = H-1;     // reload value for high phase
  }
	NVIC_ST_CURRENT_R = 0;
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; // turn on systick to continue
}

// PORTF ISR:
// Change delivered power based on switch press: 
// sw1: increase 10% until reach 90%
// sw2: decrease 10% until reach 10%
void GPIOPortF_Handler(void){ // called on touch of either SW1 or SW2
  if(GPIO_PORTF_RIS_R&0x01){  // SW2 touched 
    GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag0
    if(H>MIN_DUTY) H = H-DUTY_STEP;    // reduce delivered power
  }
  if(GPIO_PORTF_RIS_R&0x10){  // SW1 touch
    GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag4
    if(H<MAX_DUTY) H = H+DUTY_STEP;   // increase delivered power
  }
  L = PERIOD-H; // constant period, variable duty cycle
}
