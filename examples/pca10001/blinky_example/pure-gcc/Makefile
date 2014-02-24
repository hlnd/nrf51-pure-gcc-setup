# List all source files the application uses.
APPLICATION_SRCS = $(notdir $(wildcard ../*.c))  nrf_delay.c
# Use shell to find name of root folder. Possible but horrible to do in make.
PROJECT_NAME = $(shell basename "$(realpath ../)")

DEVICE = NRF51
BOARD = BOARD_PCA10001
#SEGGER_SERIAL =

#USE_SOFTDEVICE = s110

SDK_PATH = $(HOME)/Projects/nrf51/nrf51822/
TEMPLATE_PATH = $(HOME)/Projects/nrf51-pure-gcc-setup/template/

CFLAGS = -Os

GDB_PORT_NUMBER = 2331

include $(TEMPLATE_PATH)Makefile
