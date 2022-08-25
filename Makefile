#  avrdude -c usbtiny -p t85 -U lfuse:w:0xde:m -U hfuse:w:0xd7:m



#DEVICE      = attiny85
CLOCK16 = 1
DEVICE = atmega32u4
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
# EXTRA_FLAGS += -DF_CPU=8000000L -DCLOCK_SOURCE=6 

COMPILE    = avr-gcc -save-temps=obj -Wall -Wextra -Wno-unused-parameter -mmcu=$(DEVICE) -Os -fno-unroll-loops  -finline-functions   -fverbose-asm -std=gnu11 -DF_CPU=16000000L -DBAUD_TOL=3 -DBAUD=115200 -DREF_FREQ=25000000 # -flto

OBJS       = ctcss-n3ryb.o event.o tools.o pwm-sine.o
OUTNAME    := $(notdir $(patsubst %/,%,$(dir $(realpath $(firstword $(MAKEFILE_LIST))))))


all: $(OUTNAME).hex Makefile

# ctcss-n3ryb: 

all-1: test-serial.hex


#test-serial: test-serial.elf

#test-serial.hex: test-serial

%.eep: %.elf
	avr-objcopy -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0  $^ $(basename $@).eep
	

#install: $(OUTNAME).hex
#	micronucleus $^

install: $(OUTNAME).hex $(OUTNAME).eep
	pkill -TSTP minicom || true
	avrdude -c arduino -p m32u4 -P /dev/ttyUSB1 -v -b 115200  -U flash:w:$^
	#  -U eeprom:w:$(OUTNAME).eep


install-avr: $(OUTNAME).hex
	avrdude -p m32u4 -c usbtiny -v -U flash:w:$^

install-micro: $(OUTNAME).hex
	 avrdude -c avr109 -p m32u4 -P /dev/ttyACM0 -v -U flash:w:$^

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

ctcss-n3ryb.o: ctcss-n3ryb.c 

# event.o: event.c


%.bin: %.elf
	rm -f $@ $(basename $@).eep
	avr-objcopy -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0  $^ $(basename $@).eep
	avr-objcopy -O binary -R .eeprom  $^ $@

%.hex: %.elf
	rm -f $@
	avr-objcopy -j .text -j .data -O ihex $^ $@

clean:
	rm -f *.o *.s *.i
	rm -f *.elf *.bin *.hex *.map *.eep
	rm -f fuse.txt

ctcss-n3ryb.elf: $(OBJS)
	$(COMPILE) -Wall -Wextra -std=gnu11 -Wl,--gc-sections -Wl,-Map,$(basename $@).map -o $@ $^
	avr-size $@ -C --mcu=$(DEVICE)


#flashfuse: fuse.txt
#	minipro -c config -p $(DEVICE) -w $^

%.o : %.c
	$(COMPILE) $(EXTRA_FLAGS) -c $< -o $@
