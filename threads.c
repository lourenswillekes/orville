
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "rit128x96x4.h"
#include "scheduler.h"
#include <stdio.h>


void threadUART(void)
{
	while (1)
	{  
		if (lock_acquire(&uartlock))
		{
			iprintf("HELLO FRO");
			iprintf("M THREAD ");
			iprintf("NUMBER %d ", getCurrThread());
			iprintf("%d %d\r\n", getPrivLevel(), getContextSwitch());

			lock_release(&uartlock);
		}
		yield();
	}
}


void threadOLED(void)
{
	int i;
	while (1)
	{
		for (i = 0; i < 500000; i++){
			asm volatile ("nop");
		}
		RIT128x96x4StringDraw("  ", 20, 60, 15);
		RIT128x96x4StringDraw("HI", 20, 20, 15);
		for (i = 0; i < 500000; i++){
			asm volatile ("nop");
		}
		RIT128x96x4StringDraw("  ", 20, 20, 15);
		RIT128x96x4StringDraw("HI", 80, 20, 15);
		for (i = 0; i < 500000; i++){
			asm volatile ("nop");
		}
		RIT128x96x4StringDraw("  ", 80, 20, 15);
		RIT128x96x4StringDraw("HI", 80, 60, 15);
		for (i = 0; i < 500000; i++){
			asm volatile ("nop");
		}
		RIT128x96x4StringDraw("  ", 80, 60, 15);
		RIT128x96x4StringDraw("HI", 20, 60, 15);
	}

}


void threadLED(void)
{
	int i, state = 0;
	while (1)
	{
		state ^= 1;
		for (i = 0; i < 200000; i++){
			asm volatile ("nop");
		}
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, state);
	}
}
