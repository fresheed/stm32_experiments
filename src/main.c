//
// 	from https://github.com/AndoniV/STM32_Discovery_HelloWorld/blob/master/main.c
//
#include "stm32f0xx.h"
#include "stm32f0_discovery.h"
#include "headers/system_clock.h"
			

//============================================================================================
//	Global Variables
//============================================================================================
volatile uint16_t ticks;
uint16_t ledToggleMoment;
uint16_t buttonCheckMoment;
uint8_t buttonLogicalState;

int pwmToggle=0;

//============================================================================================
//	Interrupt services
//============================================================================================
void SysTick_Handler(void)
{
	static uint8_t previousPhysicalState = 0;
	static uint8_t currentPhysicalState = 0;

	ticks++;

	if (ticks >= buttonCheckMoment) {
		currentPhysicalState = GPIOA->IDR & 0x0001;					// Sets current state of button
		if ((currentPhysicalState & previousPhysicalState) == 0x0001) {		// Checks if current state is equal to previousPhysicalState
			if (buttonLogicalState == 0) {
				buttonLogicalState=1;
				GPIOC->ODR ^= (1 << 9);					// Toggles PC9
				GPIOC->ODR ^= (1 << 8);					// Toggles PC8
				if (pwmToggle==0){
					TIM1->CCR1=2500;					// compare value
				} else {
					TIM1->CCR1=5;					// compare value
				}
				pwmToggle ^= 1;
			}
		} else {
			buttonLogicalState=0;
		}

		previousPhysicalState = currentPhysicalState;

		buttonCheckMoment = ticks + 20;
	}

	if (ticks >= ledToggleMoment){
		GPIOC->ODR ^= (1 << 9);					// Toggles PC9
		GPIOC->ODR ^= (1 << 0);					// Toggles PC9
		ledToggleMoment=ticks+10;
	}
}


void setupPwm(){
	RCC->AHBENR |= (1 << 17);		// Enable clock for Port A: TIM1_CH1 maps to PA8
	GPIOA->MODER |= (2 << 16); 		// PA8: alternating function
	GPIOA->AFR[1] |= (2);		// AF2: TIM1_CH1 for PA8; shift register since we need to access higher one
	GPIOA->OTYPER &= ~(1 << 8);		// push-pull; don't know what is it, but it mentioned too http://stm32.chrns.com/post/150532351494/ledpwm
	GPIOA->OSPEEDR |= (3 << 16); 	// high speed; originally 50mhz is used
	//int a=RCC_APB2ENR_TIM1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;

	RCC->APB2ENR |= (1 << 11);		// enable TIM1
	TIM1->PSC=72; 					// prescaler - should discover more about it
	TIM1->ARR=5000;					// auto-reload value
	TIM1->CCR1=500;					// compare value
	TIM1->CCER |= 3;				// enable output to pin, active level is low
	TIM1->BDTR |= (1 << 15);		// main output enable, whatever it means
	TIM1->CCMR1 &= ~3;				// compare/capture is compare (which means output)

	// maybe also setup OC1PE?
	TIM1->CCMR1 |= (1 << 3);

	TIM1->CCMR1 |= (6 << 4);		// pwm mode 1 (usual)
	TIM1->CR1 &= ~(1 << 4);			// count up
	TIM1->CR1 &= ~(3 << 5);			// count as usually - other options are related to "center-aligned count"

	TIM1->CR1 |= 1; 				// enable !

}


int main(void){
	ticks = 0;
	ledToggleMoment = ticks + 1000;					// Will toggle PC8 each 1000 ms
	buttonCheckMoment = ticks + 20;					// Will check button each 20 ms
	buttonLogicalState = 0;

	__enable_irq();								// Enable Global Interrupts

	// GPIO ports clock enable
	// RCC: reset&clock control
	// AHBENR: peripheral clock enable register
	// same value: RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= (1 << 17) | (1 << 19);		// Enable clocks for Ports A and C

	// GPIO ports configuration
	// leds are connected to port GPIO C, bits 8 and 9
	// two bits per pin: 01 means general-purpose output
	// button input is connected to PA0 (see docs), input mode is 00 (by default)
	GPIOC->MODER |= (1 << 16) | (1 << 18) /*| (1 << 0)*/;		// Port C bits 8 and 9 as outputs, PA0 is already an input
	// 10 means pull-up
	GPIOA->PUPDR |= (1 << 1);					// Enable pull-down resistor in Port A bit 0

	setupSystemClockRepeatingTimer(1);

	// ODR is output data for corresponding port
	GPIOC->ODR |= (1 << 9);					// SETS PC9
	//GPIOC->ODR ^= (1 << 8);
	GPIOC->ODR &= ~(1 << 8);				// CLEARS PC8

	setupPwm();

    while(1)
    {
    	__WFI();								// Wait for interrupts
    }
}
