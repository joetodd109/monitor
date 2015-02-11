CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

###########################################

vpath %.c src src/peripherals
vpath %.c src/core

CFLAGS  = -g -O2 -Wall
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=softfp -mfpu=fpv4-sp-d16
CFLAGS += -ffreestanding -nostdlib
CFLAGS += -Iinc -Iinc/core -Iinc/peripherals

SRCS = misc.c stm32f4xx_rcc.c \
	stm32f4xx_rtc.c \
	stm32f4xx_gpio.c \
	stm32f4xx_syscfg.c \
	stm32f4xx_tim.c \
	stm32f4xx_usart.c \

OBJS = $(SRCS:.c=.o)

.PHONY: libstm32f4.a

all: libstm32f4.a

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $^

libstm32f4.a: $(OBJS)
	$(AR) -r $@ $(OBJS)

clean:
	rm -f $(OBJS) libstm32f4.a
