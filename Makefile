CC=arm-none-eabi-gcc
MTYPE=cortex-m3
OPTFLAG=-O0
CFLAGS= -Wall -Wextra -mthumb -mcpu=${MTYPE} ${OPTFLAG} 
# -mthumb means ARM instruction encoding is in Thumb mode (16 bit)
# else it would be in -marm which is 32 bits but all ARM Cortex-M operates on Thumb instruction encoding
# https://github.com/JayHeng/cortex-m-spec/blob/master/ARM%20Cortex-M%20for%20Beginners.pdf
LDFLAGS= -nostdlib -T linker.ld -Wl,-Map=main.map
# -nostdlib means don't use the standard startup file when linking
OBJFILES=main.o
ELFFILES=main.elf
BINFILES=build.bin
CFILES=main.c

all: ${ELFFILES}

${ELFFILES}: ${OBJFILES}
	${CC} ${LDFLAGS} -o $@ $^
	
${OBJFILES}: ${CFILES}
	${CC} ${CFLAGS} -c -o $@ $^

clean:
	rm -f *.o *.exe *.elf *.map