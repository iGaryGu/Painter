PROJECT = Painter

EXECUTABLE = $(PROJECT).elf
BIN_IMAGE = $(PROJECT).bin
HEX_IMAGE = $(PROJECT).hex

# set the path to STM32F429I-Discovery firmware package
STDP ?= STM32F429I-Discovery_FW_V1.0.1

# Toolchain configurations
CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
SIZE = $(CROSS_COMPILE)size

# Cortex-M4 implements the ARMv7E-M architecture
CPU = cortex-m4
CFLAGS = -mcpu=$(CPU) -march=armv7e-m -mtune=cortex-m4
CFLAGS += -mlittle-endian -mthumb
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

LDFLAGS =
define get_library_path
    $(shell dirname $(shell $(CC) $(CFLAGS) -print-file-name=$(1)))
endef
LDFLAGS += -L $(call get_library_path,libc.a)
LDFLAGS += -L $(call get_library_path,libgcc.a)

# Basic configurations
CFLAGS += -g -std=c99
CFLAGS += -Wall

# Optimizations
CFLAGS += -g -std=c99 -O3 -ffast-math
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -Wl,--gc-sections
CFLAGS += -fno-common
CFLAGS += --param max-inline-insns-single=1000


# specify STM32F429
CFLAGS += -DSTM32F429_439xx

# to run from FLASH
CFLAGS += -DVECT_TAB_FLASH
LDFLAGS += -T stm32f429zi_flash.ld

# PROJECT SOURCE
CFLAGS += -I./inc
OBJS = \
    ./src/main.o \
    ./src/stm32f4xx_it.o \
    ./src/system_stm32f4xx.o \
	./src/i2c_ops.o \
	./src/ov7670.o \
	./src/i2c_routine.o \
	./src/usb_bsp.o \
	./src/usbh_usr.o \
	./src/command.o \
	./src/fattime.o \
	./src/flash_if.o

# STARTUP FILE
OBJS += startup_stm32f429_439xx.o

# CMSIS
CFLAGS += -I$(STDP)/Libraries/CMSIS/Device/ST/STM32F4xx/Include
CFLAGS += -I$(STDP)/Libraries/CMSIS/Include
CFLAGS += -I$(STDP)/Utilities/Third_Party/fat_fs/inc
CFLAGS += -I$(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/inc
CFLAGS += -I$(STDP)/Libraries/STM32_USB_HOST_Library/Core/inc
CFLAGS += -I$(STDP)/Libraries/STM32_USB_HOST_Library/Class/MSC/inc
CFLAGS += -I$(STDP)/Libraries/STM32_USB_OTG_Driver/inc

# STM32F4xx_StdPeriph_Driver
CFLAGS += -DUSE_STDPERIPH_DRIVER
CFLAGS += -DUSE_USB_OTG_HS
CFLAGS += -DUSE_EMBEDDED_PHY

#CFLAGS += -D"assert_param(expr)=((void)0)"
OBJS += \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/misc.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dcmi.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.o \
	$(STDP)/Utilities/Third_Party/fat_fs/src/ff.o \
	$(STDP)/Libraries/STM32_USB_HOST_Library/Class/MSC/src/usbh_msc_bot.o \
	$(STDP)/Libraries/STM32_USB_HOST_Library/Class/MSC/src/usbh_msc_core.o \
	$(STDP)/Libraries/STM32_USB_HOST_Library/Class/MSC/src/usbh_msc_fatfs.o \
	$(STDP)/Libraries/STM32_USB_HOST_Library/Class/MSC/src/usbh_msc_scsi.o \
	$(STDP)/Libraries/STM32_USB_HOST_Library/Core/src/usbh_core.o \
	$(STDP)/Libraries/STM32_USB_HOST_Library/Core/src/usbh_hcs.o \
	$(STDP)/Libraries/STM32_USB_HOST_Library/Core/src/usbh_ioreq.o \
	$(STDP)/Libraries/STM32_USB_HOST_Library/Core/src/usbh_stdreq.o \
	$(STDP)/Libraries/STM32_USB_OTG_Driver/src/usb_core.o \
	$(STDP)/Libraries/STM32_USB_OTG_Driver/src/usb_hcd.o \
	$(STDP)/Libraries/STM32_USB_OTG_Driver/src/usb_hcd_int.o \
	$(STDP)/Utilities/STM32F429I-Discovery/stm32f429i_discovery.o

# STM32F429I-Discovery Utilities
CFLAGS += -I$(STDP)/Utilities/STM32F429I-Discovery


all: $(BIN_IMAGE)

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@
	$(OBJCOPY) -O ihex $^ $(HEX_IMAGE)
	$(OBJDUMP) -h -S -D $(EXECUTABLE) > $(PROJECT).lst
	$(SIZE) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJS)
	$(LD) -o $@ $(OBJS) \
		--start-group $(LIBS) --end-group \
		$(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.S
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)
	rm -rf $(HEX_IMAGE)
	rm -f $(OBJS)
	rm -f $(PROJECT).lst

flash:
	st-flash write $(BIN_IMAGE) 0x8000000

.PHONY: clean
