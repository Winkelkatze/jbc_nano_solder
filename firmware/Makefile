PROJECT=NanoSolder
BUILDDIR = build
LIBNAME	= opencm3_stm32f0
DEFS += -DSTM32F0
LDSCRIPT = stm32f030f4.ld
OOCD_TARGET	= stm32f0x
FP_FLAGS	= -msoft-float
ARCH_FLAGS	= -mcpu=cortex-m0 -mthumb $(FP_FLAGS)
OPT ?= -Os

TGT_CFLAGS += -Iinclude

include source/Makefile.include

OOCD = openocd
OOCD_INTERFACE = stlink-v2
BINARY = build
$(shell mkdir -p $(BUILDDIR))
include ./tinyprintf.mk
include ./ssd1306/ssd1306.mk
include ./rules.mk

#$(shell arm-none-eabi-size $(BUILDDIR)/*.o)
#$(shell arm-none-eabi-size $(BINARY).elf)

all: elf size

size: elf
	arm-none-eabi-size $(BUILDDIR)/*.o
	arm-none-eabi-size $(BINARY).elf
