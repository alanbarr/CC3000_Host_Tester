CC=msp430-gcc
CCFLAGS=-Wall -g
CPPFLAGS=-D TIME430_CLOCK_FREQ=$(FREQ)
TARGET=msp430g2553
OUTPUT=cc3000HostStartTester.elf
FREQ=16
all: clean $(OUTPUT)

$(OUTPUT):
	$(CC) $(CCFLAGS) $(CPPFLAGS) -mmcu=$(TARGET) -o $(OUTPUT) main.c
	@echo "Made with Frequency $(FREQ) MHz"

preprocess:
	$(CC) $(CCFLAGS) $(CPPFLAGS) -mmcu=$(TARGET) -E main.c
	@echo "Made with Frequency $(FREQ) MHz"

compile:
	$(CC) $(CCFLAGS) $(CPPFLAGS) -mmcu=$(TARGET) -S main.c
	@echo "Made with Frequency $(FREQ) MHz"

clean:
	-rm $(OUTPUT)
