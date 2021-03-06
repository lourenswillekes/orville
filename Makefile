# Modify as appropriate
STELLARISWARE=C:/StellarisWare

CC=arm-none-eabi-gcc -g -Wall -Os -march=armv7-m -mcpu=cortex-m3 -mthumb -mfix-cortex-m3-ldrd -Wl,--gc-sections
	
# This creates the .BIN file once the .ELF file is created
crsched.bin: crsched.elf
	arm-none-eabi-objcopy -O binary crsched.elf crsched.bin

crsched.elf: crsched.c threads.c startup_gcc.c
	${CC} -o $@ -I${STELLARISWARE} -L${STELLARISWARE}/driverlib/gcc-cm3 -Tlinkscript.x -Wl,-Map,crsched.map -Wl,--entry,ResetISR crsched.c create.S locking.S threads.c startup_gcc.c syscalls.c rit128x96x4.c -ldriver-cm3

.PHONY: clean
clean:
	rm -f *.elf *.map *.bin

# vim: noexpandtab  
