#include "system_clock.h"
#include "stm32f0xx.h"

#define MS_TICKS ((SystemCoreClock-1)/1000)


void setupSystemClockRepeatingTimer(int period_ms){
	// SysTick configuration
	// info of somewhat similar is in prog ref
	// current value register; any write sets value to 0
	SysTick->VAL = 0;							// Resets SysTick VAL
	// reload value; SystemCoreClock is clock frequency, so setting it as reload value gives us 1 second interval
	SysTick->LOAD = MS_TICKS*period_ms;
	// three control bits (0,1,2): enable counter, enable SysTick interruptions, use internal processor clock
	SysTick->CTRL = 0x07;						// Enable SysTick interrupt
}
