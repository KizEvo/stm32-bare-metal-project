# Bare metal programming with STM32
Practice setting up firmware for the STM32 Cortex M3 without any IDE, built-in HAL or third party libraries.

If you wish to use this repo, prepare the following hardwares/softwares:
  - STM32F103C8T6 (Blue pill)
  - ST-Link V2 (Programmer)
  - CP2102 module (UART to USB)
  - A potentiometer (For ADC)
  - stlink toolset (Flashing firmware)
  - ARM GNU Toolchain (Compiling)
  - Makefiles (Build automation)

Current all of the functions, peripheral initialization happens in a single **main.c** files, in the future I will try to organize them to different files for better code management.
# Finished
- [x] Writing linker script
- [x] Build automation using Makefiles
- [x] Setup blinky LED
- [x] Setup SysTick timer
- [x] Setup UART to send data to PC
- [x] Setup ADC to sample potentiometer value
