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
APPLICATION_SRCS += ble_ias.c
APPLICATION_SRCS += ble_ias_c.c
APPLICATION_SRCS += ble_lls.c
APPLICATION_SRCS += ble_tps.c
APPLICATION_SRCS += ble_srv_common.c
APPLICATION_SRCS += crc16.c
APPLICATION_SRCS += softdevice_handler.c
APPLICATION_SRCS += ble_debug_assert_handler.c
APPLICATION_SRCS += ble_error_log.c

PROJECT_NAME = ble_app_proximity

DEVICE = NRF51
BOARD = BOARD_PCA10001
#SEGGER_SERIAL =

USE_SOFTDEVICE = s110

SDK_PATH = ../../../../../
TEMPLATE_PATH = c:/temp/nrf51-pure-gcc-setup/template/

CFLAGS = -Os

GDB_PORT_NUMBER = 2331

include $(TEMPLATE_PATH)Makefile
