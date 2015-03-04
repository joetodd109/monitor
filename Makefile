
SRCS = main.c uart.c iox.c timer.c utl.c \
		dht22.c nrf24l01.c \
		system_stm32f4xx.c \

PROJ_NAME=monitor

###################################################

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
LD=arm-none-eabi-ld
AR=arm-none-eabi-ar

CFLAGS  = -g -O2 -Wall -Tstm32_flash.ld 
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=softfp -mfpu=fpv4-sp-d16

###################################################

vpath %.c src

ROOT=$(shell pwd)

CFLAGS += -Iinc 
CFLAGS += -Iinc/core

SRCS += src/startup_stm32f4xx.s \

OBJS = $(SRCS:.c=.o)

###################################################

.PHONY: proj

all: proj

proj: 	$(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ 
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin
