CC=arm-none-eabi-gcc
MTYPE=cortex-m3
OPTFLAG=-O0
ARMFLAGS= -mcpu=${MTYPE} -mthumb -mfloat-abi=soft
CFLAGS= -Wall -Wextra ${OPTFLAG} ${ARMFLAGS}
# -mthumb means ARM instruction encoding is in Thumb mode (16 bit)
# else it would be in -marm which is 32 bits but all ARM Cortex-M operates on Thumb instruction encoding
# https://github.com/JayHeng/cortex-m-spec/blob/master/ARM%20Cortex-M%20for%20Beginners.pdf
LDFLAGS= ${ARMFLAGS} -Tlinker.ld -nostartfiles -Wl,-Map=main.map
# -nostdlib means don't use the standard startup file and libraries when linking
OBJFILES=startup.o main.o
ELFFILES=main.elf
BINFILES=build.bin

all: ${ELFFILES}

${ELFFILES}: ${OBJFILES}
	${CC} ${LDFLAGS} -o $@ $^
	
%.o: %.c
	${CC} ${CFLAGS} -c -o $@ $^

build: ${BINFILES}
	st-flash --reset write $< 0x08000000

${BINFILES}: ${ELFFILES}
	arm-none-eabi-objcopy -O binary $< $@

clean:
	rm -f *.o *.elf *.map *.bin *.txt