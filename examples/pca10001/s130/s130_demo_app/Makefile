# List all source files the application uses.
APPLICATION_SRCS = $(notdir $(wildcard ../src/*.c))

PROJECT_NAME = $(shell basename "$(realpath ../)")

DEVICE = NRF51
BOARD = BOARD_PCA10001
#SEGGER_SERIAL =

USE_SOFTDEVICE = s130

SDK_PATH = $(HOME)/Projects/nrf51-sdk-5.2.0/nrf51822/
TEMPLATE_PATH = $(HOME)/Projects/nrf51-pure-gcc-setup/template/

LIBRARY_PATHS += ../inc/
LIBRARY_PATHS += ../../include/

SOURCE_PATHS += ../src/

CFLAGS = -Os
LDFLAGS = --specs=nano.specs

GDB_PORT_NUMBER = 2331

include $(TEMPLATE_PATH)Makefile
