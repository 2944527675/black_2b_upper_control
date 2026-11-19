##############################################################################
# upper_control - STM32G474RET6 CAN bus send/receive test
#
#   make            build build/upper_control.elf + .bin + .hex
#   make flash      flash through CMSIS-DAP (DAP-Link) / OpenOCD
#   make probe      check that OpenOCD can open the DAP probe
#   make reset      reset the MCU (prints the boot banner again)
#   make pc         halt and print PC/LR/SP
#   make size       print section sizes
#   make clean
#
# Toolchain: arm-none-eabi-gcc.  On this machine run `source scripts/env.sh`
# from the workspace root first (it adds the ARM toolchain and OpenOCD to PATH).
##############################################################################

TARGET    = upper_control
BUILD_DIR = build

DEBUG = 1
OPT   = -Og

C_SOURCES = $(wildcard *.c) \
            $(wildcard Drivers/STM32G4xx_HAL_Driver/Src/*.c) \
            Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/system_stm32g4xx.c

ASM_SOURCES = Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/gcc/startup_stm32g474xx.s

PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size

HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

C_DEFS = -DUSE_HAL_DRIVER -DSTM32G474xx

C_INCLUDES = -I. \
             -IDrivers/STM32G4xx_HAL_Driver/Inc \
             -IDrivers/STM32G4xx_HAL_Driver/Inc/Legacy \
             -IDrivers/CMSIS/Device/ST/STM32G4xx/Include \
             -IDrivers/CMSIS/Include

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -Wextra \
         -Wno-unused-parameter -fdata-sections -ffunction-sections -MMD -MP

LDSCRIPT = STM32G474RETX_FLASH.ld
LIBS = -lc -lm -lnosys
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBS) \
          -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections \
          -Wl,--print-memory-usage -Wl,--no-warn-rwx-segments \
          -u _printf_float -u _scanf_float

OPENOCD ?= openocd
# DAP-Link (CMSIS-DAP) is the default probe on this board.
# For ST-Link use: make flash OOCD_INTERFACE=stlink.cfg
OOCD_INTERFACE ?= cmsis-dap.cfg
OOCD_TARGET ?= stm32g4x.cfg

# ST-Link uses the HLA transport, CMSIS-DAP uses plain SWD.
ifeq ($(findstring stlink,$(OOCD_INTERFACE)),stlink)
OOCD_TRANSPORT := hla_swd
else
OOCD_TRANSPORT := swd
endif

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

.PHONY: all clean flash probe reset pc size

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin \
     $(BUILD_DIR)/$(TARGET).hex size

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

size: $(BUILD_DIR)/$(TARGET).elf
	$(SZ) $<

probe:
	$(OPENOCD) -f interface/$(OOCD_INTERFACE) -c "transport select $(OOCD_TRANSPORT)" \
	  -f target/$(OOCD_TARGET) -c "init; shutdown"

reset:
	$(OPENOCD) -f interface/$(OOCD_INTERFACE) -c "transport select $(OOCD_TRANSPORT)" \
	  -f target/$(OOCD_TARGET) -c "init; reset run; shutdown"

pc:
	$(OPENOCD) -f interface/$(OOCD_INTERFACE) -c "transport select $(OOCD_TRANSPORT)" \
	  -f target/$(OOCD_TARGET) \
	  -c "init; halt; reg pc; reg lr; reg sp; shutdown"

flash: $(BUILD_DIR)/$(TARGET).elf
	$(OPENOCD) -f interface/$(OOCD_INTERFACE) -c "transport select $(OOCD_TRANSPORT)" \
	  -f target/$(OOCD_TARGET) \
	  -c "program $< verify reset exit"

clean:
	rm -rf $(BUILD_DIR)

-include $(wildcard $(BUILD_DIR)/*.d)
