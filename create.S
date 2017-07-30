/*
 * Store default values into r0-r3, r12, r14, A2R2, XPSR
 * Store r4-r11, SP into array
 */

    .syntax unified
    .text
    .align 2
    .thumb
    .thumb_func
    .type createThread,function
    .global createThread
createThread:  

    // Save random stuff in r0-r3, r12
    subs    r1, #32
    stmea   r1!, { r0-r3, r12 }

    // Save default start values for r14, a2r2, xpsr
    //mov     r2, #13
    ldr     r2, .L0
    ldr     r3, .L0
    mov     r4, #1
    lsls    r4, r4, #24
    stmea   r1!, { r2-r4 }
  
    // Save r4-r11 and updated stack pointer
    subs    r12, r1, #32
    stmea   r0, { r4-r12 }

    bx      lr
    
.L0:
    .word   threadStarter
