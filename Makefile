DEVICE      = attiny85
ifdef CLOCK16
EXTRA_FLAGS = -DSIXTEEN
CLOCK       = 16000000
else
EXTRA_FLAGS =
CLOCK       =  8000000
endif
ifdef DEBUG
EXTRA_FLAGS += -DDEBUG
endif

ifdef RESET_ACTIVE
EXTRA_FLAGS += -DRESET_ACTIVE
endif
EXTRA_FLAGS += -DF_CPU=8000000L -DCLOCK_SOURCE=6 -DARDUINO=10819 -DARDUINO_AVR_ATTINYX5 -DARDUINO_ARCH_AVR   -DNEOPIXELPORT=PORTB   -DBOOTTUNED165

COMPILE    = avr-gcc -save-temps=obj -Wall  -mmcu=$(DEVICE) -O3
OBJS       = ctcss-n3ryb.o
OUTNAME    := $(notdir $(patsubst %/,%,$(dir $(realpath $(firstword $(MAKEFILE_LIST))))))


all: $(OUTNAME).hex Makefile

install: $(OUTNAME).hex
	micronucleus $^
# fuse.txt: Makefile
ifdef CLOCK16 
	echo "fuses_lo = 0x00f1" > fuse.txt
else
	echo "fuses_lo = 0x00e2" > fuse.txt
endif

ifdef RESET_ACTIVE
	echo "fuses_hi = 0x00D7" >> fuse.txt
else
	echo "fuses_hi = 0x0057" >> fuse.txt
endif
	echo "lock_byte = 0x00ff" >> fuse.txt


$(OUTNAME).bin: $(OUTNAME).elf
	rm -f $@ $(basename $@).eep
	avr-objcopy -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0  $^ $(basename $@).eep
	avr-objcopy -O binary -R .eeprom  $^ $@

$(OUTNAME).hex: $(OUTNAME).elf
	rm -f $@
	avr-objcopy -j .text -j .data -O ihex $^ $@

clean:
	rm -f *.o *.s *.i
	rm -f *.elf *.bin *.hex *.map *.eep
	rm -f fuse.txt

$(OUTNAME).elf: $(OBJS)
	$(COMPILE) -Wall -Wextra  -Wl,--gc-sections -Wl,-Map,$(basename $@).map -o $@ $^
	avr-size $@ -C --mcu=$(DEVICE)


#flashfuse: fuse.txt
#	minipro -c config -p $(DEVICE) -w $^

%.o : %.c
	$(COMPILE) $(EXTRA_FLAGS) -c $< -o $@
