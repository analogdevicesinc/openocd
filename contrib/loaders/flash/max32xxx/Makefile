# SPDX-License-Identifier: GPL-2.0-or-later

TARGET=max32xxx_write
ENTRY=algo_write

BIN2C = ../../../../src/helper/bin2char.sh

ifeq "$(RISCV)" ""
PREFIX=arm-none-eabi
CFLAGS=-mthumb -mcpu=cortex-m4 -Wa,-mimplicit-it=thumb
else
PREFIX=riscv-none-embed
CFLAGS=-march=rv32imc -mabi=ilp32 -Wall -g3 -ggdb -DDEBUG -O0
endif

all: $(TARGET).inc

%.o: %.c
	$(PREFIX)-gcc $(CFLAGS) -Os -Wall -c ${<} -o ${@}

%.elf: %.o
	$(PREFIX)-ld -nostdlib --entry $(ENTRY) ${<} -o ${@}
	$(PREFIX)-size ${@}

%.bin: %.elf
	$(PREFIX)-objcopy -O binary ${<} ${@}

%.inc: %.bin
	$(BIN2C) < $< > $@

%.dasm: %.o
	$(PREFIX)-objdump -S ${<} > ${TARGET}.dasm

clean:
	rm -rf $(TARGET).bin $(TARGET).elf $(TARGET).o $(TARGET).dasm $(TARGET).inc
