CC=arm-none-eabi-gcc
MTYPE=cortex-m3
OPTFLAG=-O0
CFLAGS= -Wall -mcpu=${MTYPE} ${OPTFLAG}
LDFLAGS= -nostdlib -T linker.ld -Wl,-Map=main.map

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