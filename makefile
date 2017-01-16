# Name: Makefile
# Project: LogoBot 
# Author: Sixto Diaz 
# Creation Date: 2015-03-15 
# Tabsize: 4
# License: GNU GPL v2 (see License.txt) or proprietary (CommercialLicense.txt)

DEVICE = atmega328p
#DEVICE=atmega8
AVRDUDE = avrdude -c usbasp -P avrdoper -p $(DEVICE)
# Choose your favorite programmer and interface above.
#-include pfatfs/subdir.mk


CCDF="-DF_CPU=16000000 -DSERIALDEBUG"
#SERIALDEBUG to print debug messages to uart

COMPILE = avr-gcc -Wall -fshort-enums -mcall-prologues -Os  -I. -mmcu=$(DEVICE) ${CCDEF}
#COMPILE = avr-gcc -Wall -fshort-enums -mcall-prologues -Os  -I. -mmcu=$(DEVICE) -DF_CPU=16000000
# NEVER compile the final product with debugging! Any debug output will
# distort timing so that the specs can't be met.


OBJECTS = main.o I2C_slave.o uart.o


# symbolic targets:
all:	main.hex main.c

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:main.hex:i



clean:
	rm -f  main.o main.hex main.lst main.obj main.cof main.list main.map main.eep.hex main.bin  *.o uart/*.o 

# file targets:
main.bin:	$(OBJECTS)
	$(COMPILE) -o main.bin $(OBJECTS)

main.hex:	main.bin
	rm -f main.hex main.eep.hex
	avr-objcopy -j .text -j .data -O ihex main.bin main.hex
	./checksize main.bin

disasm:	main.bin
	avr-objdump -d main.bin

cpp:
	$(COMPILE) -E main.c
