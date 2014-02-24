# List all source files the application uses.
APPLICATION_SRCS = $(notdir $(wildcard ../*.c))
APPLICATION_SRCS += app_gpiote.c
APPLICATION_SRCS += app_button.c
APPLICATION_SRCS += app_timer.c
APPLICATION_SRCS += ble_advdata.c
APPLICATION_SRCS += ble_conn_params.c
APPLICATION_SRCS += pstorage.c
APPLICATION_SRCS += ble_bondmngr.c
APPLICATION_SRCS += ble_bas.c
APPLICATION_SRCS += ble_dis.c
APPLICATION_SRCS += ble_hrs.c
APPLICATION_SRCS += ble_srv_common.c
APPLICATION_SRCS += crc16.c
APPLICATION_SRCS += softdevice_handler.c

PROJECT_NAME = $(shell basename "$(realpath ../)")

DEVICE = NRF51
BOARD = BOARD_PCA10001
#SEGGER_SERIAL =

USE_SOFTDEVICE = s110

SDK_PATH = $(HOME)/Projects/nrf51-sdk-5.1.0/nrf51822/
TEMPLATE_PATH = $(HOME)/Projects/nrf51-pure-gcc-setup/template/

CFLAGS = -Os

GDB_PORT_NUMBER = 2331

include $(TEMPLATE_PATH)Makefile
