#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "PLL.h"

#define PERIOD 50000      

#define DUTY0 0
#define DUTY30 15000
#define DUTY60 30000
#define DUTY80 40000
#define DUTY98 49000

#define GREEN 0x08
#define BLUE  0x04
#define RED		0x02
#define LED GPIO_PORTF_DATA_R

// Function prototypes
extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // low power mode

// Global Variables
unsigned int dutyA, dutyB;
unsigned long speed[] = {DUTY30, DUTY60, DUTY80, DUTY98, DUTY0}; // 98%, 80%, 60%, 30%, 0%
unsigned long duty = 0;
unsigned long direction = 0;

void Switch_Init(void);
void LED_Init(void);
void Delay(void);
void PWM0A_Init(unsigned int period);
void PWM0A_Duty(unsigned int duty);
void PWM0B_Init(unsigned int period);
void PWM0B_Duty(unsigned int duty);
void Motors_Init(void);
void Debounce(void);

int main(void){
	DisableInterrupts();  // disable interrupts to allow initializations
	PLL_Init();
	Switch_Init(); 				// arm PF4, PF0 for falling edge interrupts
	LED_Init();
	
	PWM0A_Init(PERIOD); // PB6 motor
	PWM0B_Init(PERIOD); // PB7 motor
	dutyA = DUTY0; // 0% speed
	dutyB = DUTY0;
	Motors_Init(); // PA4-7 output
	
	EnableInterrupts();		// enable after initializations are done		
	while(1){
		// main program is free to perform other tasks
		WaitForInterrupt();	// Low Power mode
	}
}

// Initilize port F and arm PF4, PF0 for falling edge interrupts
void Switch_Init(void) {
	unsigned long volatile delay;
	SYSCTL_RCGC2_R |= 0x00000020;
	delay = SYSCTL_RCGC2_R;
	GPIO_PORTF_LOCK_R = 0x4C4F434B;
  GPIO_PORTF_CR_R |= 0x11;         // allow changes to PF4,0
  GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4,0 in (built-in button)
	GPIO_PORTF_DIR_R |= 0x0E;
  GPIO_PORTF_AFSEL_R &= ~0x1F;  //     disable alt funct on PF4,0
  GPIO_PORTF_DEN_R |= 0x1F;     //     enable digital I/O on PF4,0
  GPIO_PORTF_PCTL_R &= ~0x000F000F; //  configure PF4,0 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x1F;  //     disable analog functionality on PF4,0
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4,0
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4,PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4,PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4,PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flags 4,0
	LED |= 0x02;
	GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4,PF0
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00400000; // (g) bits:23-21 for PORTF, set priority to 2
  NVIC_EN0_R |= 0x40000000;      // (h) enable interrupt 30 in NVIC
}

void LED_Init(void){ 
	SYSCTL_RCGC2_R |= 0x00000020;	// Activate F clocks
	while ((SYSCTL_RCGC2_R & 0x00000020) != 0x00000020){}
	GPIO_PORTF_LOCK_R = 0x4C4F434B;
  GPIO_PORTF_AMSEL_R &= ~0x0E;      // 3) disable analog function
  GPIO_PORTF_PCTL_R &= ~0x0000FFF0; // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R |= 0x0E;         // 6) PF1-PF3 output
  GPIO_PORTF_AFSEL_R &= ~0x0E;      // 7) no alternate function     
  GPIO_PORTF_DEN_R |= 0x0E;         // 8) enable digital pins PF3-PF1
}


void PWM0A_Init(unsigned int period) { // PB6
	SYSCTL_RCGCPWM_R |= 0x01;
	SYSCTL_RCGCGPIO_R |= 0x02;
	while ((SYSCTL_RCGCGPIO_R&0x02) == 0) {};
	GPIO_PORTB_AFSEL_R |= 0x40;
	GPIO_PORTB_PCTL_R &= ~0x0F000000;
	GPIO_PORTB_PCTL_R |= 0x04000000;
	GPIO_PORTB_AMSEL_R &= ~0x40;
	GPIO_PORTB_DEN_R |= 0x40;
	//GPIO_PORTB_DR8R_R |= 0xC0;
	SYSCTL_RCC_R = 0x00100000 | (SYSCTL_RCC_R & (~0x001E0000));
	PWM0_0_CTL_R = 0;
	PWM0_0_GENA_R = 0xC8;
	PWM0_0_LOAD_R = period - 1;
	PWM0_0_CMPA_R = 0;
	PWM0_0_CTL_R |= 0x00000001;
	PWM0_ENABLE_R |= 0x00000001;
}

void PWM0A_Duty(unsigned int duty) { // changing duty cycle
	PWM0_0_CMPA_R = duty - 1;
}

void PWM0B_Init(unsigned int period) { // PB7
	volatile unsigned long delay;
	SYSCTL_RCGCPWM_R |= 0x01;
	SYSCTL_RCGCGPIO_R |= 0x02;
	delay = SYSCTL_RCGCGPIO_R;
	GPIO_PORTB_AFSEL_R |= 0x80;
	GPIO_PORTB_PCTL_R &= ~ 0xF0000000;
	GPIO_PORTB_PCTL_R |= 0x40000000;
	GPIO_PORTB_AMSEL_R &= ~0x80;
	GPIO_PORTB_DEN_R |= 0x80;
	SYSCTL_RCC_R = SYSCTL_RCC_USEPWMDIV;
	SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M;
	SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;
	PWM0_0_CTL_R = 0;
	PWM0_0_GENB_R = 0xC08; 
	PWM0_0_LOAD_R = period - 1;
	PWM0_0_CMPB_R = 0;
	PWM0_0_CTL_R |= 0x00000001;
	PWM0_ENABLE_R |= 0x00000002;
}

void PWM0B_Duty(unsigned int duty) { // changing duty cycle
	PWM0_0_CMPB_R = duty - 1;	
}

void Motors_Init(void) { // controls direction
	SYSCTL_RCGC2_R |= 0x00000001;
	GPIO_PORTA_AMSEL_R &= ~0xF0;
	GPIO_PORTA_PCTL_R &= ~0x0FF00000;
	GPIO_PORTA_DIR_R |= 0xF0;
	GPIO_PORTA_DR8R_R |= 0xF0;
	GPIO_PORTA_AFSEL_R &= ~0xF0;
	GPIO_PORTA_DEN_R |= 0xF0;
	GPIO_PORTA_DATA_R |= 0x60;
}

void GPIOPortF_Handler(void) {
	Debounce();
	if (GPIO_PORTF_RIS_R&0x10) { //SW1, increase speed
		Delay();
		GPIO_PORTF_ICR_R = 0x10; // acknowledge flag
		dutyA = speed[duty];
		dutyB = speed[duty];
		PWM0A_Duty(dutyA);
		PWM0B_Duty(dutyB);
		
		// if forwards, output GREEN
		if (direction % 2 == 0) {
			LED &= ~0x0E;
			LED |= GREEN;
		}
		
		// if backwards, output BLUE
		if (direction % 2 == 1) {
			LED &= ~0x0E;
			LED |= BLUE;
		}
		
		// initial state, output RED
		if (speed[duty] == 0) {
			LED &= ~0x0E;
			LED |= RED;
		}
		
		// increase speed when SW1 is pressed
		duty = duty + 1;
		
		// restart cycle after 98% speed
		if (duty == 5) {
			duty = 0;
		}
		Debounce();
	}
	
	if (GPIO_PORTF_RIS_R&0x01) { // SW2, change directions
		Debounce();
		Delay();
		
		GPIO_PORTF_ICR_R = 0x01;
		
		if (duty != DUTY0) { // if car is in motion, output blue or green depending on direction
			Delay();
			LED &= ~0x0E;
			direction = direction + 1;
			
			if (direction % 2 == 0) { // forwards
				LED |= GREEN;
				GPIO_PORTA_DATA_R &= ~0xF0;
				GPIO_PORTA_DATA_R |= 0x60;
			}
			
			if (direction % 2 == 1) { // backwards
				LED |= BLUE;
				GPIO_PORTA_DATA_R &= ~0xF0;
				GPIO_PORTA_DATA_R |= 0x90;
			}
		}
		
		if (duty == DUTY0) { // if car is not in motion, output RED even if SW2 is pressed
			Delay();
			LED &= ~0x0E;
			direction = direction + 1;
			
			if (direction % 2 == 0) { // forwards
				LED |= RED;
				GPIO_PORTA_DATA_R &= ~0xF0;
				GPIO_PORTA_DATA_R |= 0x60;
			}
			
			if (direction % 2 == 1) { // backwards
				LED |= RED;
				GPIO_PORTA_DATA_R &= ~0xF0;
				GPIO_PORTA_DATA_R |= 0x90;
			}
			
		}
	}
	Debounce();
}

void Delay(void){
	unsigned long volatile time;
	time = 727240*2/91; // 1 sec
	while(time){
		time--;
	}
	for(time=0; time<1000; time=time+10){
	}
}

void Debounce(void) {
	unsigned long j;
	for (j = 0; j < 199999; j++) {
	}
}

