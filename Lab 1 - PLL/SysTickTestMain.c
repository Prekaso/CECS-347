#include "SysTick.h"
#include "PLL.h"
#include "tm4c123gh6pm.h"

int SW;
int LED;
int PRESSED;

void PORTF_Init(void);
void EdgeCounter_Init(void);
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // Go to low power mode while waiting for the next interrupt
void SysTick_Handler(void);	
void GPIOPortF_Handler(void);
void SysTick_Wait10ms(unsigned long delay);
void Debounce(void);						// debounce module for button
void Systick_On(void);
void Systick_Off(void);

int main(void){
	PORTF_Init();
  PLL_Init();               // set system clock to 50 MHz
  SysTick_Init();           // initialize SysTick timer
	EdgeCounter_Init();
	EnableInterrupts();
 
  while(1){
		WaitForInterrupt();
  }
}

void PORTF_Init(void) {
	if ((SYSCTL_RCGC2_R & 0x00000020) != 0x00000020) {
		SYSCTL_RCGC2_R |= 0x00000020;    // 1) F clock
	}
	while ((SYSCTL_RCGC2_R & 0x00000020) != 0x00000020) {}	
  GPIO_PORTF_AMSEL_R &= ~0x1F;       // 3) disable analog function 
  GPIO_PORTF_PCTL_R &= ~0x000F000F;  // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R |= 0x1F;          // 5) PF0 input & PF3,PF1 output  
  GPIO_PORTF_AFSEL_R &= ~0x1F;       // 6) no alternate function 
  GPIO_PORTF_DEN_R |= 0x1F;          // 7) enable digital pins PF3,PF1 
}

void EdgeCounter_Init(void) {  
	
	if ((SYSCTL_RCGC2_R & 0x00000020) != 0x00000020) {  
		SYSCTL_RCGC2_R |= 0x00000020; 	// activate clock for port F
	}
	while ((SYSCTL_RCGC2_R & 0x00000020) != 0x00000020) {}
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // unlock PortF PF0
  GPIO_PORTF_CR_R = 0x1F;          // allow changes to PF4-1
  GPIO_PORTF_DIR_R &= ~0x11;    		//  make PF4,1 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x1F;  		//  disable alt funct on PF0
  GPIO_PORTF_DEN_R |= 0x1F;     		//  enable digital I/O on PF4-1
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; //  configure PF4-1 as GPIO
  GPIO_PORTF_AMSEL_R = 0;      			//  disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x11;     		//  enable weak pull-up on PF4,1
	
	GPIO_PORTF_IS_R &= ~0x11;     		//  PF4,1 is edge-sensitive, 0 = edge, 1 = level
  GPIO_PORTF_IBE_R &= ~0x11;    			//  PF4,1 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    			//  PF4,1 rising edge event, 0 = falling, 1 = rising
  GPIO_PORTF_ICR_R |= 0x11;     		//  clear flag
  GPIO_PORTF_IM_R |= 0x11;      		//  arm interrupt on PF4,1
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // priority 5
  NVIC_EN0_R |= 0x40000000;    		 	//  enable interrupt 30 in NVIC  
}

// Handle GPIO Port F interrupts. 
void GPIOPortF_Handler(void) {
	
	GPIO_PORTF_ICR_R = 0x11; 		 		
	
	SW = GPIO_PORTF_DATA_R & 0x11;
	
	Debounce();
	
	if (SW == 0x01 && LED == 0) { // when sw1 is pressed
		GPIO_PORTF_DATA_R = 0x02; // red
		PRESSED = 0; // flag for sw2
		LED = 1; // there is light
		if (LED == 1) { // if LED is on, blink
			Systick_On();
		} else {
			Systick_Off();
		}
	} 
	else if (SW == 0x01 && LED == 1) {
		GPIO_PORTF_DATA_R = 0x00; // off
		LED = 0; // no light
	}
	
	Debounce();
	
	if (SW == 0x10) { // sw2
		PRESSED = PRESSED + 1;
		if (PRESSED == 1) {
			GPIO_PORTF_DATA_R = 0x08; // green
		}
		else if (PRESSED == 2) {
			GPIO_PORTF_DATA_R = 0x04; // blue
		}
		else if (PRESSED > 2) {
			PRESSED = 0;
			GPIO_PORTF_DATA_R = 0x02; // red
		}
	}
}

void SysTick_Handler(void) {
	if (LED == 1 && PRESSED == 0) {
		GPIO_PORTF_DATA_R ^= 0x02; // red
	}
	else if (LED == 1 && PRESSED == 1) { // green , sw2 pressed 1x
		GPIO_PORTF_DATA_R ^= 0x08;
	}
	else if (LED == 1 && PRESSED == 2) { // blue , sw2 pressed 2x
		GPIO_PORTF_DATA_R ^= 0x04;
	}
}

void Debounce(void) {
	unsigned long j;
	for (j = 0; j < 199999; j++) {
	}
}
