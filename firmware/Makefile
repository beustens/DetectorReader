TARGET = project
MCU = attiny814
DEVICE = tiny814
# clock must be configured in main.c accordingly
CLOCK = 20000000
SOURCES = main.c
OBJECTS = $(SOURCES:.c=.o)
COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(MCU)

# serial port for UPDI programmer
ifeq ($(OS),Windows_NT)
	PROGRAMMER = COM10
else
	PROGRAMMER = /dev/tty.usbserial-A50285BI
endif

# compiling and linking, target is the finished hex file
all: $(TARGET).hex

# compile source files to object files
.c.o:
	$(COMPILE) -c $< -o $@

# link the object files together
$(TARGET).elf: $(OBJECTS)
	$(COMPILE) $(OBJECTS) -o $(TARGET).elf

# convert elf file to hex file
$(TARGET).hex: $(TARGET).elf
	avr-objcopy -j .text -j .data -j .rodata -O ihex $(TARGET).elf $(TARGET).hex

# flash (call make flash), requires pyupdi installed
flash: $(TARGET).hex
ifeq ($(OS),Windows_NT)
	python -m updi.pyupdi -d $(DEVICE) -c $(PROGRAMMER) -f $(TARGET).hex
else
	python3 -m updi.pyupdi -d $(DEVICE) -c $(PROGRAMMER) -f "$(shell pwd)/$(TARGET).hex"
endif

clean:
ifeq ($(OS),Windows_NT)
	del -f $(TARGET).hex $(TARGET).elf $(OBJECTS)
else
	rm -f $(TARGET).hex $(TARGET).elf $(OBJECTS)
endif