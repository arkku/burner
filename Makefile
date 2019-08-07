# Makefile for the EEPROM burner.
# Copyright (c) 2019 Kimmo Kulovesi, https://arkku.com/
###############################################################################
# Put your local settings in "local.mk", it is ignored by Git.
-include local.mk
### AVR MCU ###################################################################
# Only tested with ATmega328P.

MCU ?= atmega328p
F_CPU ?= 16000000UL
BAUD ?= 19200UL

LFUSE ?= FF
HFUSE ?= D7
EFUSE ?= FD

#### BURNER ###################################################################
# Specify the burner on the command-line if you wish, e.g.,
#	make burn BURNER=avrisp2 PORT=/dev/ttyUSB0 BPS=115200

# Burner device
BURNER ?= dragon_isp
# Burner port
#PORT ?= /dev/ttyUSB0
# Burner speed
#BPS ?= 115200
###############################################################################

CC=avr-gcc
CFLAGS=-Wall -std=c11 -pedantic -Wextra -Wno-unused-parameter -Os $(AVR_FLAGS)
LDFLAGS=-Os $(AVR_FLAGS)
AR=avr-ar
ARFLAGS=rcs
OBJCOPY=avr-objcopy
AVRDUDE=avrdude

IHEX_FLAGS=-DIHEX_DISABLE_SEGMENTS=1 -DIHEX_MAX_OUTPUT_LINE_LENGTH=64 -DIHEX_LINE_MAX_LENGTH=64 -DIHEX_EXTERNAL_WRITE_BUFFER=1 -Iihex
UART_FLAGS=-DKK_UART_CONVERT_CRLF_IN_TO_LF=1 -DKK_UART_CONVERT_LF_OUT_TO_CRLF=1 -DKK_UART_RECEIVE_BUFFER_SIZE=256

AVR_FLAGS=-mmcu=$(MCU) -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) $(IHEX_FLAGS) $(UART_FLAGS)


HEX=burner.hex
BIN=burner.elf
OBJS=burner.o kk_ihex_read.o kk_ihex_write.o kk_uart.o
BOOTLOADER=optiboot_atmega328.hex

all: $(HEX)

burner.o: ihex/kk_ihex.h ihex/kk_ihex_read.h ihex/kk_ihex_write.h kk_uart.h
kk_uart.o: kk_uart.h
ihex/kk_ihex_read.h: ihex/kk_ihex.h
ihex/kk_ihex_write.h: ihex/kk_ihex.h
$(OBJS): Makefile

$(HEX): $(BIN)

%.o: ihex/%.c ihex/%.h
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.c %.h
	$(CC) $(CFLAGS) -c $< -o $@

$(BIN): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $+

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

burn: $(HEX)
	$(AVRDUDE) -c $(BURNER) $(if $(PORT),-P $(PORT) ,)$(if $(BPS),-b $(BPS) ,)-p $(MCU) -U flash:w:$< -v

upload: $(HEX)
	$(AVRDUDE) -c arduino $(if $(PORT),-P $(PORT),-P /dev/ttyUSB0) $(if $(BPS),-b $(BPS),-b $(subst L,,$(subst U,,$(BAUD)))) -p $(MCU) -U flash:w:$< -v

fuses:
	$(AVRDUDE) -c $(BURNER) $(if $(PORT),-P $(PORT) ,)$(if $(BPS),-b (BPS) ,)-p $(MCU) -U efuse:w:0x$(EFUSE):m -U hfuse:w:0x$(HFUSE):m -U lfuse:w:0x$(LFUSE):m

bootloader: $(BOOTLOADER)
	$(AVRDUDE) -c $(BURNER) $(if $(PORT),-P $(PORT) ,)$(if $(BPS),-b $(BPS) ,)-p $(MCU) -e -U flash:w:$< -v

unlock:
	$(AVRDUDE) -c $(BURNER) $(if $(PORT),-P $(PORT) ,)$(if $(BPS),-b $(BPS) ,)-p $(MCU) -U lock:w:0x3F:m -v

lock:
	$(AVRDUDE) -c $(BURNER) $(if $(PORT),-P $(PORT) ,)$(if $(BPS),-b $(BPS) ,)-p $(MCU) -U lock:w:0x0F:m -v

clean:
	rm -f $(OBJS)

distclean: | clean
	rm -f $(HEX) $(BIN)

.PHONY: all clean distclean burn fuses upload lock unlock bootloader
