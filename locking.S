    .syntax unified
    .text
    .align 2
    .thumb
    .thumb_func
	
	
    .type lock_acquire,function
    .global lock_acquire
lock_acquire:  

	mov		r1, #0
	
	ldrex	r2, [r0]
	cmp		r2, r1
	itt		ne
	strexne	r2, r1, [r0]
	
	cmpne	r2, #1
	beq		1f
	
	mov		r0, #1
	bx		lr
	
1:
	clrex
	mov		r0, #0
	bx		lr
  

    .type lock_release,function
    .global lock_release
lock_release:

	mov 	r1, #1
	str 	r1, [r0]
	bx		lr