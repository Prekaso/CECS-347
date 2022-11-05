#include <stdint.h>
#include "tm4c123gh6pm.h"

#define PERIOD 			10000           	// stay on, Change brightness
#define MIN_DUTY    PERIOD/10							// minimum duty cycle 10%
#define MAX_DUTY    PERIOD*0.9						// maximum duty cycle 90%
#define DUTY_STEP		PERIOD/10							// duty cycle change for each button press
#define HALF_DUTY		PERIOD/2

// Function prototypes
extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // low power mode

// This function initilizes port F and arm PF4, PF0 for falling edge interrupts
void Switch_Init(void);
void Delay_20ms(void);
void PWM1G3A_PWM1G2B_Init(uint32_t period);
void PWM1A_PWM1B_Duty(uint16_t duty);

// Global Variables
unsigned long duty;

int main(void){
	DisableInterrupts();  // disable interrupts to allow initializations
	Switch_Init(); 				// arm PF4, PF0 for falling edge interrupts
	PWM1G3A_PWM1G2B_Init(PERIOD);
	EnableInterrupts();		// enable after initializations are done
	
	duty = PERIOD / 2;
	PWM1A_PWM1B_Duty(duty);
		
	while(1){
		// main program is free to perform other tasks
		WaitForInterrupt();	// Low Power mode
	}
}

// Initilize port F and arm PF4, PF0 for falling edge interrupts
void Switch_Init(void){  unsigned long volatile delay;
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;// (a) activate clock for port F
  delay = SYSCTL_RCGC2_R;
  //GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO Port F
	GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
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
  NVIC_EN0_R |= 0x40000000;      // (h) enable interrupt 30 in NVIC
}

void PWM1G3A_PWM1G2B_Init(uint32_t period){
	SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
	if((SYSCTL_RCGC2_R &= SYSCTL_RCGC2_GPIOF) != SYSCTL_RCGC2_GPIOF){
		SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;
		while((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOF) != SYSCTL_RCGC2_GPIOF){}
		}
		GPIO_PORTF_AFSEL_R |= 0x06;
		GPIO_PORTF_PCTL_R &= ~0x00000FF0;
		GPIO_PORTF_PCTL_R |= 0x00000550;
		GPIO_PORTF_AMSEL_R &= ~0x06;
		GPIO_PORTF_DEN_R |= 0x06;
		
		SYSCTL_RCC_R = 0x00100000|(SYSCTL_RCC_R &(~0x001E0000));
		PWM1_3_CTL_R = 0;
		PWM1_2_CTL_R = 0;
		PWM1_3_GENA_R = 0xC8;
		PWM1_2_GENB_R = 0xC08;
		PWM1_3_LOAD_R = period - 1;
		PWM1_2_LOAD_R = period - 1;
		PWM1_3_CMPA_R = 0;
		PWM1_2_CMPB_R = 0;
		PWM1_3_CTL_R |= 0x00000001;
		PWM1_2_CTL_R |= 0x00000001;
		PWM1_ENABLE_R |= 0x00000060;
	}
// change duty cycle of PF1
// duty is number of PWM clock cycles output is high
void PWM1A_PWM1B_Duty(uint16_t duty){
	PWM1_3_CMPA_R = duty-1;
	PWM1_2_CMPB_R = duty-1;
}


// Subroutine to wait 0.02sec
// Inputs: None
// Outputs: None
//
void Delay_20ms(void){
	unsigned long volatile time;
	time = 727240*2/91; // 0.02sec
	while(time){
		time--;
	}
	for(time=0; time<1000; time=time+10){}
	}

// PORTF ISR:
// Change delivered power based on switch press: 
// sw1: increase 10% until reach 90%
// sw2: decrease 10% until reach 10%
void GPIOPortF_Handler(void){ // called on touch of either SW1 or SW2
	Delay_20ms();
  if(GPIO_PORTF_RIS_R&0x01){  // SW2 touched 
    GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag0
    if(duty>MIN_DUTY) duty = duty-DUTY_STEP; 		// reduce delivered power
		PWM1A_PWM1B_Duty(duty);
  }
  if(GPIO_PORTF_RIS_R&0x10){  // SW1 touch
    GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag4
    if(duty<MAX_DUTY) duty = duty+DUTY_STEP;   // increase delivered power
		PWM1A_PWM1B_Duty(duty);
  }
}