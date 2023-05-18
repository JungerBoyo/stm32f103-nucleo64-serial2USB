SOURCE_FILES = 						  \
	src/main.c 								\
	cmsis/system_stm32f1xx.c  \
	cmsis/startup_stm32f103xb.s

FLASH_SCRIPT = cmsis/STM32F103XB_FLASH.ld

PROJ_NAME = template
BUILD_FOLDER = build

CC = $(ARM_TOOLCHAIN_PATH)arm-none-eabi-gcc #-x assembler-with-cpp
OBJCOPY = $(ARM_TOOLCHAIN_PATH)arm-none-eabi-objcopy

CFLAGS  = -g -ggdb3 -O0 -I. -Wall -T$(FLASH_SCRIPT) --specs=nosys.specs 		  		#	general flags and linker script
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m3 -mfloat-abi=soft -DSTM32F103xB  # mcpu specific flags and defs

OBJECT_FILES = $(SOURCE_FILES:.c=.o)

# commands
all: proj

proj: $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SOURCE_FILES) $(BUILD_FOLDER)
	$(CC) $(CFLAGS) $(SOURCE_FILES) -o $(BUILD_FOLDER)/$@
	$(OBJCOPY) -O ihex $(BUILD_FOLDER)/$(PROJ_NAME).elf $(BUILD_FOLDER)/$(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(BUILD_FOLDER)/$(PROJ_NAME).elf $(BUILD_FOLDER)/$(PROJ_NAME).bin

$(BUILD_FOLDER):
	mkdir $(BUILD_FOLDER)

flash: proj
	$(ST_TOOLCHAIN_PATH)st-flash write $(BUILD_FOLDER)/$(PROJ_NAME).bin 0x08000000

clean: proj
	rm -f $(BUILD_FOLDER)/*.o







