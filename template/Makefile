TOOLCHAIN_PREFIX ?= arm-none-eabi

AS = $(TOOLCHAIN_PATH)$(TOOLCHAIN_PREFIX)-as
CC = $(TOOLCHAIN_PATH)$(TOOLCHAIN_PREFIX)-gcc
LD = $(TOOLCHAIN_PATH)$(TOOLCHAIN_PREFIX)-gcc
OBJCOPY = $(TOOLCHAIN_PATH)$(TOOLCHAIN_PREFIX)-objcopy
OBJDUMP = $(TOOLCHAIN_PATH)$(TOOLCHAIN_PREFIX)-objdump
SIZE = $(TOOLCHAIN_PATH)$(TOOLCHAIN_PREFIX)-size
GDB = $(TOOLCHAIN_PATH)$(TOOLCHAIN_PREFIX)-gdb

START_CODE ?= startup_nrf51.s
SYSTEM_FILE ?= system_nrf51.c

SDK_INCLUDE_PATH += $(SDK_PATH)Include/
SDK_SOURCE_PATH += $(SDK_PATH)Source/
CMSIS_INCLUDE_PATH += $(SDK_PATH)Include/gcc

LIBRARY_PATHS += ../ $(SDK_INCLUDE_PATH)
SOURCE_PATHS += ../ $(SDK_SOURCE_PATH) $(TEMPLATE_PATH) $(wildcard $(SDK_SOURCE_PATH)*/)

ifeq ($(USE_SOFTDEVICE), s110)
    USE_BLE = 1
endif

ifeq ($(USE_SOFTDEVICE), s120)
    USE_BLE = 1
endif

ifeq ($(USE_SOFTDEVICE), s210)
    USE_ANT = 1
endif

ifeq ($(USE_SOFTDEVICE), s310)
    USE_BLE = 1
    USE_ANT = 1
endif

ifdef USE_BLE
    LIBRARY_PATHS += $(SDK_INCLUDE_PATH)ble/
    LIBRARY_PATHS += $(SDK_INCLUDE_PATH)ble/ble_services/
    SOURCE_PATHS += $(SDK_SOURCE_PATH)ble/ble_services/
    CFLAGS += -DBLE_STACK_SUPPORT_REQD
endif

ifdef USE_ANT
    CFLAGS += -DANT_STACK_SUPPORT_REQD
endif

ifdef USE_SOFTDEVICE
    LIBRARY_PATHS += $(SDK_INCLUDE_PATH)$(USE_SOFTDEVICE)
    LIBRARY_PATHS += $(SDK_INCLUDE_PATH)app_common/
    LIBRARY_PATHS += $(SDK_INCLUDE_PATH)sd_common/
else
    USE_SOFTDEVICE = blank
endif

LINKER_SCRIPT ?= gcc_nrf51_$(USE_SOFTDEVICE).ld
OUTPUT_NAME ?= $(addsuffix _$(USE_SOFTDEVICE), $(PROJECT_NAME))

ifdef USE_EXT_SENSORS
LIBRARY_PATHS += $(SDK_INCLUDE_PATH)ext_sensors/
SOURCE_PATHS += $(wildcard $(SDK_SOURCE_PATH)ext_sensors/*/)
endif

LIBRARY_INCLUDES = $(addprefix -I,$(LIBRARY_PATHS))
CMSIS_INCLUDE = $(addprefix -I,$(CMSIS_INCLUDE_PATH))

VPATH = $(SOURCE_PATHS)

OUTPUT_PATH = _build/

CPUFLAGS = -mthumb -mcpu=cortex-m0 -march=armv6-m
CFLAGS += -std=gnu99 -c $(CPUFLAGS) -Wall -D$(DEVICE) -D$(BOARD) $(CMSIS_INCLUDE) $(LIBRARY_INCLUDES) -MD
LDFLAGS += $(CPUFLAGS) -L $(TEMPLATE_PATH) -T $(LINKER_SCRIPT) -Wl,-Map=$(OUTPUT_PATH)$(OUTPUT_NAME).Map

HEX = $(OUTPUT_PATH)$(OUTPUT_NAME).hex
ELF = $(OUTPUT_PATH)$(OUTPUT_NAME).elf
BIN = $(OUTPUT_PATH)$(OUTPUT_NAME).bin

SRCS = $(SYSTEM_FILE) $(notdir $(APPLICATION_SRCS))
OBJS = $(addprefix $(OUTPUT_PATH), $(SRCS:.c=.o)) $(addprefix $(LIBS_PATH),$(APPLICATION_LIBS))
DEPS = $(addprefix $(OUTPUT_PATH), $(SRCS:.c=.d))
SRCS_AS = $(START_CODE)
OBJS_AS = $(addprefix $(OUTPUT_PATH), $(SRCS_AS:.s=.os))

JLINK_OPTIONS = -device nrf51822 -if swd -speed 1000

all: $(OBJS) $(OBJS_AS) $(HEX)

rebuild: clean all

ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)Makefile.windows
else
include $(TEMPLATE_PATH)Makefile.posix
endif

$(HEX): $(OBJS) $(OBJS_AS)
	$(LD) $(LDFLAGS) $(OBJS_AS) $(OBJS) -o $(ELF)
	$(OBJCOPY) -Oihex $(ELF) $(HEX)
	$(OBJCOPY) -Obinary $(ELF) $(BIN)
	$(SIZE) $(ELF)

size: $(ELF)
	$(SIZE) $(ELF)

$(OUTPUT_PATH)%.o: %.c
	$(MAKE_BUILD_FOLDER)
	$(CC) $(LDFLAGS) $(CFLAGS) $< -o $@

$(OUTPUT_PATH)%.os: %.s
	$(MAKE_BUILD_FOLDER)
	$(AS) $< -o $@

-include $(DEPS)

.PHONY: all clean rebuild size
