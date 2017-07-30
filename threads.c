
#include "rit128x96x4.h"
#include "scheduler.h"
#include <stdio.h>


/*void uart_lock(unsigned *count)
{
	(*count)++;
}

void uart_unlock(unsigned *lock, unsigned *count)
{
	(*count)--;
	if (0 == (*count))
	{
		lock_release(lock);
	}
}*/


void threadUART(void)
{
	while (1)
	{  
		if (lock_acquire(&uartlock))
		{
			iprintf("THIS IS T");
			yield();
			iprintf("HREAD NU");
			yield();
			iprintf("MBER %d\r\n", currThread);
			
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
		for (i = 0; i < 250000; i++);
		RIT128x96x4StringDraw("SWITCHING",       12, 16, 15);
        yield();
		for (i = 0; i < 250000; i++);
		RIT128x96x4StringDraw("THINGS   ",       12, 16, 15);
        yield();
	}

}

void threadLED(void)
{
	while (1)
	{
		// nepis
	}
	
}
