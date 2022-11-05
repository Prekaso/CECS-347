#include "ADCSWTrigger.h"
#include "tm4c123gh6pm.h"
#include "PLL.h"
#include <stdint.h>


// function prototypes
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode

unsigned char eq_calculation(unsigned int ADC_Value);
unsigned char tb_estimation(unsigned int ADC_Value);
void SysTick_Init(unsigned long period);
void PORTF_Init(void);
void PWM_Init(uint16_t period);
void PWM_Duty(uint16_t duty);


// Constant definitions
// constants used in euqarion y=A+B/x
#define A 453 
#define B 24790 
#define NUM_SAMPLES 50

// array of ADC Values for distance 10cm to 70cm, with a maximum of 5cm step
uint16_t dist[] = {2800,2400,1800,1600,1400,1350,1300,1250,1200,1100,1050,1000,900};
		

volatile unsigned long ADCvalue;
// The digital number ADCvalue is a representation of the voltage on PE3
// voltage  ADCvalue
// 0.00V     0
// 0.75V    1024
// 1.50V    2048
// 2.25V    3072
// 3.00V    4095


// Global variables
unsigned char sample=0; // variable to indicate time to do the ADC sampling
unsigned char too_close=0; // variable to indicate the obstacle is too close to the sensor
unsigned char tabledist, eqdist; // distance estimation for tabel lookup and equation calculation.
unsigned char count=0;

int main(void){
	DisableInterrupts();
  PLL_Init();                          
  ADC0_InitSWTriggerSeq3_Ch1();         // ADC initialization PE2/AIN1
	PORTF_Init();
	PWM_Init(16000);										
	SysTick_Init(400000);
	EnableInterrupts();
  while(1){
			ADCvalue /= NUM_SAMPLES;   				// Find the average ADCvalue
			eqdist = eq_calculation(ADCvalue);
			tabledist = tb_estimation(ADCvalue);
		  count = 0;
			if(eqdist >= 15){  
				PWM1_3_CMPA_R = (16000 - (eqdist * 228)); //Set time for PWM to be high
			}
			if(eqdist >= 15){ 
				too_close = 0;
			}
			else{
				too_close = 1;
				PWM1_ENABLE_R ^= 0x40;
			}
  }
}

// Initialize SysTick timer with interrupt enabled
void SysTick_Init(unsigned long period){ 
	NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_RELOAD_R = period-1;// reload value
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x1FFFFFFF)|0x60000000; // priority 3
                              // enable SysTick with core clock and interrupts
  NVIC_ST_CTRL_R = 0x07;
  EnableInterrupts();
}


// SysTick ISR, do the following tasks:
// 1. set a flag to signal sample next input
// 2. blink red LED if distance is < 15cm
void SysTick_Handler(void){
	//collect data 50 times
	int j = 0;
	for( j = 0; j < NUM_SAMPLES; j++)
	{
		ADCvalue += ADC0_InSeq3(); //add sample to temp
	}
		
	if(too_close)
	{
			for(j = 0; j < 29900;j++)
			{}
	}
}

// Calculates distance based on ADC value passed in
unsigned char eq_calculation(unsigned int ADC_Value){
	unsigned char dist = 0;
	dist = B / (ADCvalue - A); 
	return dist;
}

unsigned char tb_estimation(unsigned int ADC_Value){
	unsigned char dist=0;
	if(ADC_Value > 2240){
		dist = 10;
	}
	else if(ADC_Value > 1920){
		dist = 15;
	}
	else if(ADC_Value > 1440){
		dist = 20;
	}
	else if(ADC_Value > 1280){
		dist = 25;
	}
	else if(ADC_Value > 1120){
		dist = 30;
	}
	else if(ADC_Value > 1080){
		dist = 35;
	}
	else{
		dist = 40;
	}
	return dist;
}


void PORTF_Init(void)
{
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;	// Activate F clocks
	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOF)==0){};
		
	GPIO_PORTF_AMSEL_R &= ~0x0E;      // 3) disable analog function
  GPIO_PORTF_PCTL_R &= ~0x0000FFF0; // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R |= 0x0E;         // 6) PF1-PF3 output
  GPIO_PORTF_AFSEL_R &= ~0x0E;      // 7) no alternate function     
  GPIO_PORTF_DEN_R |= 0x0E;         // 8) enable digital pins PF3-PF1
}


void PWM_Init(uint16_t period){
  SYSCTL_RCGCPWM_R |= 0x02;             // 1) activate PWM1
   SYSCTL_RCGC2_R |= 0x20;            // 2) activate port F: 0010 0000
  while(( SYSCTL_RCGC2_R&0x20) == 0){};
	GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO Port F
	GPIO_PORTF_CR_R = 0x04;         // allow changes to PF2
	GPIO_PORTF_AFSEL_R |= 0x04;           // enable alt funct on PF2: 0010 0000
  GPIO_PORTF_PCTL_R &= ~0x00000F00;     // configure PF2 as PWM1
  GPIO_PORTF_PCTL_R |= 0x00000500;
  GPIO_PORTF_AMSEL_R &= ~0x04;          // disable analog functionality on PF2
  GPIO_PORTF_DEN_R |= 0x04;             // enable digital I/O on PF2
  SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV; // 3) use PWM divider
  SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M; //    clear PWM divider field
  SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;  //    configure for /2 divider
	
  PWM1_3_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM1_3_GENA_R = 0xC8;	// low on LOAD, high on CMPA down
  // PF2 goes low on LOAD
  // PF2 goes high on CMPA down
  PWM1_3_LOAD_R = period - 1;           // 5) cycles needed to count down to 0
  PWM1_3_CMPA_R = 0;             // 6) count value when output rises
  PWM1_3_CTL_R |= 0x00000001;           // 7) start PWM1
  PWM1_ENABLE_R |= 0x00000040;          // enable PF2/M1PWM6 0100 0000
}
 
void PWM_Duty(uint16_t duty){
  PWM1_3_CMPA_R = duty - 1;             // 6) count value when output rises
}
