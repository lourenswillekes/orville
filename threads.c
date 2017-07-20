#include <stdio.h>
#include "scheduler.h"

// These are the user-space threads. Note that they are completely oblivious
// to the technical concerns of the scheduler. The only interface to the
// scheduler is the single function yield() and the global variable
// currThread which indicates the number of the thread currently
// running.

void threadUART(void)
{
  unsigned count;
  
  while(1)
    for (count = 0; count < 10000; count++) {
      iprintf("Thread %u: UART\r\n", currThread);
	}
  }
  
}

void threadOLED(void)
{
  unsigned count;
  
  while(1)
    for (count = 0; count < 10000; count++) {
      iprintf("Thread %u: OLED\r\n", currThread);
	}
  }
  
}

void threadLED(void)
{
  unsigned count;

  for (count = 0; count < 10; count++) {
    iprintf("Thread %u: LED -- yield %d\r\n", currThread, count);
    yield();
  }
  
}
