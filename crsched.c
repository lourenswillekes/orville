#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/lm3s6965.h"
#include "rit128x96x4.h"
#include "scheduler.h"
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driverlib/interrupt.h"


#define STACK_SIZE 4096


unsigned uartlock;
extern unsigned lock_acquire(unsigned *lock);
extern void lock_release(unsigned *lock);
void scheduler(void);
void threadStarter(void);
void lock_init(unsigned *lock)
{
  lock_release(lock);
}

typedef struct {
  int active;       // non-zero means thread is allowed to run
  char *stack;      // pointer to TOP of stack (highest memory location)
  unsigned savedRegs[10]; // Array holding r4 - r11 and PSP
} threadStruct_t;

// thread_t is a pointer to function with no parameters and
// no return value...i.e., a user-space thread.
typedef void (*thread_t)(void);

extern void threadUART(void);
extern void threadOLED(void);
extern void threadLED(void);

// This array replaces STACK_SIZE macro for variable stack sizes
static int stackSize[] = {
  STACK_SIZE,
  STACK_SIZE,
  STACK_SIZE,
  STACK_SIZE
};

static thread_t threadTable[] = {
  threadUART,
  threadUART,
  threadOLED,
  threadLED,
};

#define NUM_THREADS (sizeof(threadTable)/sizeof(threadTable[0]))

// These static global variables are used in scheduler(), in yield(), and in threadStarter()
static threadStruct_t threads[NUM_THREADS];
unsigned currThread;
unsigned firstTime = 1;



// This function is implemented in assembly. It sets up the scheduler
// with default values in the threads stack along with its saved Registers
// buffer.
extern void createThread(unsigned *savedRegs, char *stack);

void main(void)
{
  unsigned i;

  // Set the clocking to run directly from the crystal.
  SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                 SYSCTL_XTAL_8MHZ);

  // Initialize the OLED display and write status.
  RIT128x96x4Init(1000000);
  RIT128x96x4StringDraw("Scheduler Demo",       20,  0, 15);

  // Enable the peripherals used by this example.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

  // Set GPIO A0 and A1 as UART pins.
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  // Configure the UART for 115,200, 8-N-1 operation.
  UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));

  // Init LED
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);

  // Init lock function
  lock_init(&uartlock);

  // Set Systick to be a 1ms timer with interrupt
  NVIC_ST_CTRL_R = 0;
  NVIC_ST_RELOAD_R = 8000;
  NVIC_ST_CURRENT_R = 0;
  NVIC_ST_CTRL_R = 0x7;

  // Create all the threads and allocate a stack for each one
  for (i=0; i < NUM_THREADS; i++) {
    // Mark thread as runnable
    threads[i].active = 1;

    // Allocate stack
    threads[i].stack = (char *)malloc(stackSize[i]) + stackSize[i];
    if (threads[i].stack == 0) {
      iprintf("Out of memory\r\n");
      exit(1);
    }

    // After createThread() executes, we can execute a restore stack
    // and the thread will begin execution at threadStarter() with
    // its own stack.
    createThread(threads[i].savedRegs, threads[i].stack);
  }

  // Enable interrupts
  IntMasterEnable();

  // Starts scheduler in privileged handler mode
  yield();

  // If scheduler() returns, all coroutines are inactive and we return
  // from main() hence exit() should be called implicitly (according to
  // ANSI C). However, TI's startup_gcc.c code (ResetISR) does not
  // call exit() so we do it manually.
  exit(0);
}

void scheduler(void){
  // First time scheduler is called the correct value is put into psp for save state
  if(firstTime){
      asm volatile ("MSR PSP, %0" : : "r" (threads[currThread].savedRegs[8]));
      firstTime = 0;
  }

  // Save thread state
  asm volatile("MRS r12, PSP");
  asm volatile("stm %0, {r4-r12}" : : "r" (threads[currThread].savedRegs));

  if (++currThread == NUM_THREADS) {
    currThread = 0;
  }

  //iprintf(".");
  NVIC_ST_CURRENT_R = 0;

  // Restore thread state
  asm volatile("ldm %0, {r4-r12}" : : "r" (threads[currThread].savedRegs));
  asm volatile("MSR PSP, r12");

  // Fake return to unprivileged thread mode
  asm volatile ("movw lr, #0xfffd");
  asm volatile ("movt lr, #0xffff");
  asm volatile ("bx lr");
}

// This function causes and svc exception to execute scheduler in privileged handler mode
void yield(void)
{
  asm volatile ("svc #0");
}

// This is the starting point for all threads. It runs in user thread
// context using the thread-specific stack. The address of this function
// is saved by createThread() in the savedRegs buffer.
void threadStarter(void)
{
  // Call the entry point for this thread. The next line returns
  // only when the thread exits.
  (*(threadTable[currThread]))();

  // Do thread-specific cleanup tasks. Currently, this just means marking
  // the thread as inactive. Do NOT free the stack here because we're
  // still using it! Remember, this function runs in user thread context.
  threads[currThread].active = 0;

  // This yield returns to the scheduler and never returns back since
  // the scheduler identifies the thread as inactive.
  yield();
}

int getCurrThread(void){
  return currThread;
}
