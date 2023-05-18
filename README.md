# stm32f103-nucleo64-serial2USB
Simple program for connecting a device with UART communication to PC using USB using nucleo64 board with stm32f103 MCU. Tested with Rockpro64.

## IMPORTANT
Pass <ARM_TOOLCHAIN_PATH>(compiler and objcopy) and <ST_TOOLCHAIN_PATH>(st-flash tool) arguments with make command in order to compile and flash.

## Debugging
To debug follow the steps:
* run `st-util` (included in st toolchain)
* run `gdb`
* (in gdb)`target extended:4242`
* (in gdb)`file <path_to_exe>.elf`



