# stm32f103rb-template-project
stm32f103rb template project with basic CMSIS libs setup using Unix Makefiles.

## IMPORTANT
Pass <ARM_TOOLCHAIN_PATH>(compiler and objcopy) and <ST_TOOLCHAIN_PATH>(st-flash tool) arguments with make command in order to compile and flash.

## Debugging
To debug follow the steps:
* run `st-util` (included in st toolchain)
* run `gdb`
* (in gdb)`target extended:4242`
* (in gdb)`file <path_to_exe>.elf`



