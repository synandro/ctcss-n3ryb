#define F_CPU 16000000UL
#define BAUD 9600


#include <stdint.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <stdbool.h>
#include <string.h>
#include <avr/io.h>
#include <util/setbaud.h>
#include <stdio.h>

#define DEBUG 1
#ifdef DEBUG
#define dprintf(...) printf(__VA_ARGS__)
#else
#define dprintf(...)
#endif

#include "tools.h"


buf_head_t uart_rx_buf;
uint32_t vfo_a_freq;

/* the different states we can be in.  start in idle mode  */
enum {
	MODE_IDLE,
	MODE_HALFWAY,
	MODE_PROGRAM,
};



/* these are (Hz * 8) but experimentally checked to be on frequency */
enum {
	/* these are (Hz * 8) but experimentally checked to be on frequency */

	MULT_77 = 618,
	MULT_100 = 800,
	MULT_103_5 = 828,
	MULT_118_8 = 949,
	MULT_123 = 984,
	MULT_127_3 = 1017,
	MULT_131_8 = 1052,
	MULT_141_3 = 1129, 
	MULT_146_2 = 1168,
	MULT_167_9 = 1342,
	MULT_179_9 = 1439,
	MULT_203_5 = 1624,
	MULT_NONE = 0,
	
	/* these all need tested */
	MULT_67 = 536,
	MULT_69_3 = 554,
	MULT_71_9 = 575,
	MULT_74_4 = 595,
	MULT_79_7 = 638,
	MULT_82_5 = 660,
	MULT_85_4 = 683,
	MULT_88_5 = 708,
	MULT_91_5 = 732,

	MULT_94_8 = 758,
	MULT_97_4 = 779,
	MULT_107_2 = 858,
	MULT_110_9 = 887,
	MULT_114_8 = 918,
	MULT_136_5 = 1092,
	MULT_151_4 = 1211,
	MULT_156_7 = 1254,
	MULT_162_2 = 1298,
	MULT_173_8 = 1390,
	MULT_186_2 = 1490,
	MULT_192_8 = 1542,
	MULT_206_5 = 1652,
	MULT_210_7 = 1686,
	MULT_218_1 = 1745,
	MULT_225_7 = 1806,
	MULT_229_1 = 1833,
	MULT_233_6 = 1869,
	MULT_241_8 = 1934,
	MULT_250_3 = 2002,
	MULT_254_1 = 2033,



};

/* these are the multipliers that were calculated, they need verified ***
	MULT_67 = 536,
	MULT_69_3 = 554,
	MULT_71_9 = 575,
	MULT_74_4 = 595,
	MULT_77 = 616,
	MULT_79_7 = 638,
	MULT_82_5 = 660,
	MULT_85_4 = 683,
	MULT_88_5 = 708,
	MULT_91_5 = 732,
	MULT_94_8 = 758,
	MULT_97_4 = 779,
	MULT_100 = 800,
	MULT_103_5 = 828,
	MULT_107_2 = 858,
	MULT_110_9 = 887,
	MULT_114_8 = 918,
	MULT_118_8 = 950,
	MULT_123 = 984,
	MULT_127_3 = 1018,
	MULT_131_8 = 1054,
	MULT_136_5 = 1092,
	MULT_141_3 = 1130,
	MULT_146_2 = 1170,
	MULT_151_4 = 1211,
	MULT_156_7 = 1254,
	MULT_162_2 = 1298,
	MULT_167_9 = 1343,
	MULT_173_8 = 1390,
	MULT_179_9 = 1439,
	MULT_186_2 = 1490,
	MULT_192_8 = 1542,
	MULT_203_5 = 1628,
	MULT_206_5 = 1652,
	MULT_210_7 = 1686,
	MULT_218_1 = 1745,
	MULT_225_7 = 1806,
	MULT_229_1 = 1833,
	MULT_233_6 = 1869,
	MULT_241_8 = 1934,
	MULT_250_3 = 2002,
	MULT_254_1 = 2033,
*/



struct frequencies 
{
	const char *tone;
	const uint16_t mult;
	const uint16_t start;
	const uint16_t end;
	const uint16_t channel;
};


enum  {
	HZ_100 = 0,
	HZ_103_5 = 1,
	HZ_118_8 = 2,
	HZ_123 = 3,
	HZ_127_3 = 4,
	HZ_131_8 = 5,
	HZ_141_3 = 6,
	HZ_146_2 = 7,
	HZ_167_9 = 8,
	HZ_179_9 = 9,
	HZ_NONE = 10,
};

/*

	HZ_67
	HZ_69_3
	HZ_71_9
	HZ_74_4
	HZ_77
	HZ_79_7
	HZ_82_5
	HZ_85_4
	HZ_88_5
	HZ_91_5
	HZ_94_8
	HZ_97_4
	HZ_100
	HZ_103_5
	HZ_107_2
	HZ_110_9
	HZ_114_8
	HZ_118_8
	HZ_123
	HZ_127_3
	HZ_131_8
	HZ_136_5
	HZ_141_3
	HZ_146_2
	HZ_151_4
	HZ_156_7
	HZ_162_2
	HZ_167_9
	HZ_173_8
	HZ_179_9
	HZ_186_2
	HZ_192_8
	HZ_203_5
	HZ_206_5
	HZ_210_7
	HZ_218_1
	HZ_225_7
	HZ_229_1
	HZ_233_6
	HZ_241_8
	HZ_250_3
	HZ_254_1
*/

/*
INDEX=994 R2=51000 R1=33000 vout = 4.85714285714286 index= 994
INDEX=852 R2=51000 R1=47000 vout = 4.16326530612245 index= 852
INDEX=819 R2=51000 R1=51000 vout = 4 index= 819
INDEX=702 R2=51000 R1=68000 vout = 3.42857142857143 index= 702
INDEX=553 R2=51000 R1=100000 vout = 2.70198675496689 index= 553
INDEX=415 R2=51000 R1=150000 vout = 2.02985074626866 index= 415
INDEX=332 R2=51000 R1=200000 vout = 1.62549800796813 index= 332
INDEX=308 R2=51000 R1=220000 vout = 1.50553505535055 index= 308
INDEX=238 R2=51000 R1=300000 vout = 1.16239316239316 index= 238
INDEX=219 R2=51000 R1=330000 vout = 1.07086614173228 index= 219
INDEX=189 R2=51000 R1=390000 vout = 0.925170068027211 index= 189
INDEX=160 R2=51000 R1=470000 vout = 0.783109404990403 index= 160
INDEX=114 R2=51000 R1=680000 vout = 0.558139534883721 index= 114


*/

static const struct frequencies freq_table[] = {
	{ .tone = "100",	.mult = MULT_100, 	.start = 950,	.end = 1024,	.channel = 1	},
	{ .tone = "103.5",	.mult = MULT_103_5, 	.start = 860,	.end = 920,	.channel = 2	},
	{ .tone = "118.5",	.mult = MULT_118_8, 	.start = 751,	.end = 860,	.channel = 3	},
	{ .tone = "123",  	.mult = MULT_123, 	.start = 651,	.end = 750,	.channel = 4	},
	{ .tone = "127.3",	.mult = MULT_127_3, 	.start = 550,	.end = 650,	.channel = 5 	},
	{ .tone = "131.8",	.mult = MULT_131_8, 	.start = 450,	.end = 550,	.channel = 6 	},
	{ .tone = "141.3",	.mult = MULT_141_3, 	.start = 330,	.end = 450,	.channel = 7	},
	{ .tone = "146.2",	.mult = MULT_146_2, 	.start = 300,	.end = 329,	.channel = 8	},
	{ .tone = "167.9",	.mult = MULT_167_9, 	.start = 200,	.end = 299,	.channel = 9 	},
	{ .tone = "179.9",	.mult = MULT_179_9, 	.start = 130,	.end = 200,	.channel = 10	},
	{ .tone = "0", 		.mult = MULT_NONE, 	.start = 80,	.end = 129,	.channel = 11	},
};


#if 0
static const struct frequencies freq_table_two[] = {
	{ .tone = "67", 	.mult = MULT_67, 	.start = 950,	.end = 1024,	.channel = 1	},
	{ .tone = "69.3",	.mult = MULT_69_3, 	.start = 860,	.end = 920,	.channel = 2	},
	{ .tone = "71.9",	.mult = MULT_71_9, 	.start = 751,	.end = 860,	.channel = 3	},
	{ .tone = "74.4",	.mult = MULT_74_4, 	.start = 651,	.end = 750,	.channel = 4	},
	{ .tone = "77",		.mult = MULT_77, 	.start = 550,	.end = 650,	.channel = 5 	},
	{ .tone = "79.7",	.mult = MULT_79_7, 	.start = 450,	.end = 550,	.channel = 6 	},
	{ .tone = "82.5",	.mult = MULT_82_5, 	.start = 330,	.end = 450,	.channel = 7	},
	{ .tone = "85.4",	.mult = MULT_85_4, 	.start = 300,	.end = 329,	.channel = 8	},
	{ .tone = "88.5",	.mult = MULT_88_5, 	.start = 200,	.end = 299,	.channel = 9 	},
	{ .tone = "91.5",	.mult = MULT_91_5, 	.start = 130,	.end = 200,	.channel = 10	},
	{ .tone = "0",	.mult = MULT_NONE, 	.start = 80,	.end = 129,	.channel = 11	},
};

static const struct frequencies freq_table_three[] = {
	{ .tone = "94.8", 	.mult = MULT_94_8, 	.start = 950,	.end = 1024,	.channel = 1	},
	{ .tone = "97.4",	.mult = MULT_97_4, 	.start = 860,	.end = 920,	.channel = 2	},
	{ .tone = "100",	.mult = MULT_100, 	.start = 751,	.end = 860,	.channel = 3	},
	{ .tone = "103.5",	.mult = MULT_103_5, 	.start = 651,	.end = 750,	.channel = 4	},
	{ .tone = "107.2",	.mult = MULT_107_2, 	.start = 550,	.end = 650,	.channel = 5 	},
	{ .tone = "110.9",	.mult = MULT_110_9, 	.start = 450,	.end = 550,	.channel = 6 	},
	{ .tone = "114.8",	.mult = MULT_114_8, 	.start = 330,	.end = 450,	.channel = 7	},
	{ .tone = "118.8",	.mult = MULT_118_8, 	.start = 300,	.end = 329,	.channel = 8	},
	{ .tone = "123",	.mult = MULT_123, 	.start = 200,	.end = 299,	.channel = 9 	},
	{ .tone = "127.3",	.mult = MULT_127_3, 	.start = 130,	.end = 200,	.channel = 10	},
	{ .tone = "0", 	.mult = MULT_NONE, 	.start = 80,	.end = 129,	.channel = 11	},
};

static const struct frequencies freq_table_four[] = {
	{ .tone = "131.8", 	.mult = MULT_131_8, 	.start = 950,	.end = 1024,	.channel = 1	},
	{ .tone = "136.5",	.mult = MULT_136_5, 	.start = 860,	.end = 920,	.channel = 2	},
	{ .tone = "141.3",	.mult = MULT_141_3, 	.start = 751,	.end = 860,	.channel = 3	},
	{ .tone = "146.2",	.mult = MULT_146_2, 	.start = 651,	.end = 750,	.channel = 4	},
	{ .tone = "151.4",	.mult = MULT_151_4, 	.start = 550,	.end = 650,	.channel = 5 	},
	{ .tone = "156.7",	.mult = MULT_156_7, 	.start = 450,	.end = 550,	.channel = 6 	},
	{ .tone = "162.2",	.mult = MULT_162_2, 	.start = 330,	.end = 450,	.channel = 7	},
	{ .tone = "167.9",	.mult = MULT_167_9, 	.start = 300,	.end = 329,	.channel = 8	},
	{ .tone = "173.8",	.mult = MULT_173_8, 	.start = 200,	.end = 299,	.channel = 9 	},
	{ .tone = "179.9",	.mult = MULT_179_9, 	.start = 130,	.end = 200,	.channel = 10	},
	{ .tone = "0", 	.mult = MULT_NONE, 	.start = 80,	.end = 129,	.channel = 11	},
};

#endif


static const uint8_t sine_wave[256] = {
	0x80,0x83,0x86,0x89,0x8c,0x8f,0x92,0x95,
	0x98,0x9c,0x9f,0xa2,0xa5,0xa8,0xab,0xae,
	0xb0,0xb3,0xb6,0xb9,0xbc,0xbf,0xc1,0xc4,
	0xc7,0xc9,0xcc,0xce,0xd1,0xd3,0xd5,0xd8,
	0xda,0xdc,0xde,0xe0,0xe2,0xe4,0xe6,0xe8,
	0xea,0xeb,0xed,0xef,0xf0,0xf2,0xf3,0xf4,
	0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfb,0xfc,
	0xfd,0xfd,0xfe,0xfe,0xfe,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,0xfe,0xfe,0xfd,0xfd,
	0xfc,0xfc,0xfb,0xfa,0xf9,0xf8,0xf7,0xf6,
	0xf5,0xf4,0xf2,0xf1,0xef,0xee,0xec,0xeb,
	0xe9,0xe7,0xe5,0xe3,0xe1,0xdf,0xdd,0xdb,
	0xd9,0xd7,0xd4,0xd2,0xcf,0xcd,0xca,0xc8,
	0xc5,0xc3,0xc0,0xbd,0xba,0xb8,0xb5,0xb2,
	0xaf,0xac,0xa9,0xa6,0xa3,0xa0,0x9d,0x9a,
	0x97,0x94,0x91,0x8e,0x8a,0x87,0x84,0x81,
	0x7e,0x7b,0x78,0x75,0x71,0x6e,0x6b,0x68,
	0x65,0x62,0x5f,0x5c,0x59,0x56,0x53,0x50,
	0x4d,0x4a,0x47,0x45,0x42,0x3f,0x3c,0x3a,
	0x37,0x35,0x32,0x30,0x2d,0x2b,0x28,0x26,
	0x24,0x22,0x20,0x1e,0x1c,0x1a,0x18,0x16,
	0x14,0x13,0x11,0x10,0x0e,0x0d,0x0b,0x0a,
	0x09,0x08,0x07,0x06,0x05,0x04,0x03,0x03,
	0x02,0x02,0x01,0x01,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x01,0x01,0x01,0x02,0x02,
	0x03,0x04,0x04,0x05,0x06,0x07,0x08,0x09,
	0x0b,0x0c,0x0d,0x0f,0x10,0x12,0x14,0x15,
	0x17,0x19,0x1b,0x1d,0x1f,0x21,0x23,0x25,
	0x27,0x2a,0x2c,0x2e,0x31,0x33,0x36,0x38,
	0x3b,0x3e,0x40,0x43,0x46,0x49,0x4c,0x4f,
	0x51,0x54,0x57,0x5a,0x5d,0x60,0x63,0x67,
	0x6a,0x6d,0x70,0x73,0x76,0x79,0x7c,0x80
};



static uint16_t cur_mult;
static uint16_t counter;
static uint8_t cur_freq;

static uint8_t EEMEM saved_frequency = HZ_123;


static uint32_t atoul(const char *num)
{
	uint32_t val = 0;
        while (*num && (*num >= '0' && *num <= '9'))
        {
	    	val = (val * 10) + *num - '0';
	    	num++;
        }
        return val;
}

static uint8_t atous(const char *num)
{
	uint8_t val = 0;
        while (*num && (*num >= '0' && *num <= '9'))
        {
	    	val = (val * 10) + *num - '0';
	    	num++;
        }
        return val;
}



static inline  __attribute__((always_inline)) void led_off(void)
{
	PORTB &= ~(1 << PB0);
}

static inline __attribute__((always_inline)) void led_on(void)
{
	PORTB |= (1 << PB0);
}


static void __attribute__((noinline)) fast_blink(const uint8_t count)
{
	led_off();
	_delay_ms(1000);
	for(uint8_t i = 0; i < count; i++)
	{
		led_on();
		_delay_ms(100);
		led_off();
		_delay_ms(50);
	}
	led_on();

}

static void timeout_blink(void)
{
	fast_blink(2);
	_delay_ms(200);
	fast_blink(3);
	_delay_ms(200);
	fast_blink(2);
	_delay_ms(500);
}


static void do_blink(uint8_t count)
{
	led_off();
	_delay_ms(1000);
	for(uint8_t i = 0; i < count; i++)
	{
		led_on();
		_delay_ms(200);
		led_off();
		_delay_ms(200);
	}
	_delay_ms(1000);
	led_on();
}



static void change_frequency(const uint8_t freq, bool save)
{
	cur_freq = freq;
	cur_mult = freq_table[freq].mult;
	if(cur_mult == MULT_NONE)
	{
		fast_blink(4);
	} else {
		do_blink(freq_table[freq].channel);
	}
	if(save == false)
		return;

	/* save some wear if its the same frequency */
	if(!(eeprom_read_byte(&saved_frequency) == cur_freq))	
		eeprom_write_byte(&saved_frequency, cur_freq);
}

static void load_saved_frequency(void)
{
	uint8_t f;
	bool save = false;

	f = eeprom_read_byte(&saved_frequency);
		
	/* corrupt eeprom? just turn ourselves off */
	if(f > HZ_NONE) {
		f = HZ_NONE;
		save = true;
	}
	f = HZ_103_5;
	change_frequency(f, save);
}

#ifdef TESTING

static void loop_all(void)
{
	for(uint8_t x = 0; x < sizeof(freq_table)/sizeof(struct frequencies); x++)
	{
		change_frequency(x, false);
		_delay_ms(10000);		
	}
}
#endif

#ifdef DEBUG

static int uart_putchar(char c, FILE *stream) 
{
    loop_until_bit_is_set(UCSR1A, UDRE1);
    UDR1 = c;
    return c;
}

int uart_getchar(FILE *stream) 
{
    loop_until_bit_is_set(UCSR1A, RXC1); /* Wait until data exists. */
    return UDR1;
}

volatile bool usart1_int;

ISR(USART1_RX_vect)
{
	usart1_int = true;
}




FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);

static void uart_init(void) 
{
	UBRR1H = UBRRH_VALUE;
	UBRR1L = UBRRL_VALUE;

#if USE_2X
	UCSR1A |= _BV(U2X1);
#else
	UCSR1A &= ~(_BV(U2X1));
#endif

	UCSR1C = _BV(UCSZ11) | _BV(UCSZ10); /* 8-bit data */
	UCSR1B = _BV(RXEN1) | _BV(TXEN1) | _BV(RXCIE1);   /* Enable RX and TX */
	stdout = &uart_output;
	stdin  = &uart_input;
//	cmdstacklen = 0;
}
#endif 

static void setup_pwm(void)
{
	TIMSK0 = 0;

	OCR4B = 120;

	TCCR4A = _BV(WGM40) | _BV(WGM41) | _BV(COM4B0);
	TCCR4B = _BV(CS40);
	

	TCCR0A = _BV(COM1B0) | _BV(WGM00) | _BV(WGM01);
	TCCR0B = _BV(WGM02) | _BV(CS01);
	TIMSK0 = _BV(OCIE0A);		
	OCR0A = 243;

	//DDRB |= (1 << PB7); 
	//DDRB |= (1 << PB5); 
	DDRB |= (1 << PB6); 
}




static __attribute__((noinline)) void SPI_write16 (const uint16_t data)          
{
	uint8_t upper = (data >> 8) & 0xFF;
	uint8_t lower = data & 0xFF; 

//        dprintf("Writing spi_write16: %x\r\n", data);
        SPDR = upper;
	asm volatile( "nop" );	
        while (!(SPSR & (1<<SPIF)));                    
        SPDR = lower;                                                 
	asm volatile( "nop" );
        while (!(SPSR & (1<<SPIF)));                    
}

// AD9833 Control Register helpers
#define CR_B28_COMBINED      0x2000
#define CR_FSELECT_0         0x0000
#define CR_PSELECT_0         0x0000
#define CR_RESET             0x0100
#define CR_SLEEP1            0x0080
#define CR_SLEEP12           0x0040
#define CR_OPBITEN           0x0020
#define CR_DIV2              0x0008
#define CR_MODE_D1_TRIANGLE  0x0002
#define CR_MODE_D1_SINE      0x0000

// Mnemonics for wave forms
#define SINE                 (CR_B28_COMBINED | CR_MODE_D1_SINE)
#define SQUARE               (CR_B28_COMBINED | CR_OPBITEN)
#define FAST_SQUARE          (SQUARE | CR_DIV2)
#define TRIANGLE             (CR_B28_COMBINED | CR_MODE_D1_TRIANGLE)

#define FREQ0                0x4000
#define PHASE0               0xC000
#define REF_FREQ             25000000
#define SPI_CLOCK_SPEED      12000000




void spi_init(void)
{
	// PB1 = SCLK, PB2 = MOSI, PB3 = MISO, PE6 = slave select
	DDRB |= _BV(PB1) | _BV(PB2);
	DDRE |= _BV(PE6); // slave select 
	PORTE |= (1 << PE6); 
	PORTB |= (1 << PB1);
	SPCR  = _BV(SPE)| _BV(MSTR)| _BV(SPR0) | _BV(CPOL);
}


static void __attribute__((noinline)) setWave(int waveform, uint32_t frequency) 
{
	uint32_t freq_data = ((uint64_t)frequency << 28) / REF_FREQ;
	uint16_t freq_MSB = (freq_data >> 14) | FREQ0;
	uint16_t freq_LSB = (freq_data & 0x3FFF) | FREQ0;
	dprintf("Setting with freq_data: %lu\r\n", freq_data);
	PORTE &= ~_BV(PE6);                                             //      Fsync Low --> begin frame
	_delay_ms(10);
	SPI_write16(CR_B28_COMBINED | CR_FSELECT_0 | CR_PSELECT_0 | CR_RESET);
	SPI_write16(freq_LSB);
	SPI_write16(freq_MSB);
	SPI_write16(PHASE0);
	SPI_write16(waveform);
	_delay_ms(10);
	PORTE |= _BV(PE6);                                              //      Fsync High --> End of frame
}

typedef void CMDCB(char **parv, uint8_t parc);


struct command_struct 
{
	const char *cmd;
	CMDCB *handler;
	bool iscat; /* a cat command, only match the first two characters */
};


static void cmd_vfo_a(char **argv, uint8_t argc)
{
	uint32_t tfreq;

	if(argv[0][1] == ';') 
	{
		dprintf("FA;%lu;\r\n", vfo_a_freq);
	}
		

	vfo_a_freq = tfreq = atoul(&argv[0][2]);
	setWave(SINE, tfreq);
	dprintf("Setting wave to tfreq: %lu\r\n", vfo_a_freq);
	dprintf("FA;%lu;\r\n", tfreq);

}
#if 0
static void cmd_vfo_b(const char *buf, size_t len)
{
	dprintf("Got cmd_vfo_a: %s %d\r\n", buf, len);


}


static void cmd_hi(const char *cmd, size_t len)
{
	dprintf("HI!\n");
	if(len > 0)
	{
		
	}

}
#endif

#define AD_MUX_ADC0 0x0 // F0
#define AD_MUX_ADC1 (_BV(MUX0)) // F1
#define AD_MUX_ADC4 (_BV(MUX2))  // F4
#define AD_MUX_ADC5 (_BV(MUX0) | _BV(MUX2))  // F5
#define AD_MUX_ADC6 (_BV(MUX1) | _BV(MUX2)) // 20 // F6
#define AD_MUX_ADC7 (_BV(MUX0) | _BV(MUX1) | _BV(MUX2)) // 21 // F7



#define AD_MUX_ADC8 (_BV(MUX5)) // 22 // D4
#define AD_MUX_ADC9 (_BV(MUX5) | _BV(MUX0)) //100 // D6
#define AD_MUX_ADC10 (_BV(MUX5) | _BV(MUX1)) // 101 // D7
#define AD_MUX_ADC11 (_BV(MUX5) | _BV(MUX2) | _BV(MUX1)) // 102 // B4
#define AD_MUX_ADC12 (_BV(MUX5) | _BV(MUX2)) // 110 // B5
#define AD_MUX_ADC13 (_BV(MUX5) | _BV(MUX2) | _BV(MUX0)) // 111 // B6
#define AD_MUX_TEMP (_BV(MUX1) | _BV(MUX2) | _BV(MUX3))


#define AD_SRB_ADC0 0
#define AD_SRB_ADC1 0
#define AD_SRB_ADC2 0
#define AD_SRB_ADC3 0
#define AD_SRB_ADC4 0
#define AD_SRB_ADC5 0
#define AD_SRB_ADC6 0 
#define AD_SRB_ADC7 0
#define AD_SRB_ADC8 (_BV(MUX5))
#define AD_SRB_ADC9 (_BV(MUX5))
#define AD_SRB_ADC10 (_BV(MUX5))
#define AD_SRB_ADC11 (_BV(MUX5))
#define AD_SRB_ADC12 (_BV(MUX5))
#define AD_SRB_ADC13 (_BV(MUX5));
#define AD_SRB_TEMP (_BV(MUX5));





static void cmd_adc(char **argv, uint8_t argc)
{
	uint8_t admux_val;
	uint8_t adc = 120; // the temp sensor
	uint8_t adc_samples = 16;
	uint16_t adc_result = 0;	
	if(argc > 1)
		adc = atous(argv[1]);
	
	switch(adc)
	{
		case 0:
			ADMUX = AD_MUX_ADC0;
			ADCSRB = AD_SRB_ADC0;
			break;
		case 1:
			ADMUX = AD_MUX_ADC1;
			ADCSRB = AD_SRB_ADC1;
			
			break;
		case 4:
			ADMUX = AD_MUX_ADC4;
			ADCSRB = AD_SRB_ADC4;

			break;
		case 5:
			ADMUX = AD_MUX_ADC5;
			ADCSRB = AD_SRB_ADC5;
			
			break;
		case 6:
			ADMUX = AD_MUX_ADC6;
			ADCSRB = AD_SRB_ADC6;
			break;
		case 7:
			ADMUX = AD_MUX_ADC7;
			ADCSRB = AD_SRB_ADC7;
			break;
		case 8:
			ADMUX = AD_MUX_ADC8;
			ADCSRB = AD_SRB_ADC8;

			break;
			
		case 9:
			ADMUX = AD_MUX_ADC9;
			ADCSRB = AD_SRB_ADC9;

			break;
		case 10:
			ADMUX = AD_MUX_ADC10;
			ADCSRB = AD_SRB_ADC10;
			break;
		case 11:
			ADMUX = AD_MUX_ADC11;
			ADCSRB = AD_SRB_ADC11;
			
			break;
		case 12:
			ADMUX = AD_MUX_ADC12;
			ADCSRB = AD_SRB_ADC12;

			break;
		case 13:
			ADMUX = AD_MUX_ADC13;
			ADCSRB = AD_SRB_ADC13;
			break;
		case 120:
			ADMUX = AD_MUX_TEMP;
			ADCSRB = AD_SRB_TEMP;
			break;
		default:
			ADMUX = AD_MUX_TEMP;
			ADCSRB = AD_SRB_TEMP;
			break;
	
	}


	ADCSRA = _BV(ADEN) | _BV(ADATE);      //Enable ADC module


	//ADMUX = _BV(MUX0) | _BV(MUX1) | _BV(MUX2);
//        ADMUX= 0x01;            // configuring PB2 to take input
        ADCSRB= 0x00;                   //Configuring free running mode
        ADCSRA |= (1<<ADSC)|(1<<ADATE);   //Start ADC conversion and enabling Auto trigger


	ADCSRA |= _BV(ADEN); // | _BV(ADATE);	/* turn on the ADC and auto update */ 

	ADMUX |= _BV(REFS0); 
	ADCSRA |= _BV(ADSC);			/* lets start a conversion */
	_delay_ms(10);
	
//        uint16_t adc_val = 0;

	while(1)
	{
//        while(1)
		while(!bit_is_set(ADCSRA,ADSC));

        	unsigned int adc_val = ADCW;
        	
        	for(uint8_t i = 0; i < adc_samples; i++)
        	{
        		while(!bit_is_set(ADCSRA,ADSC));
        		adc_result += ADCW;
        	}
        	
        	dprintf("ADC: adc_val: %u average: %u\r\n", adc_val, adc_result / adc_samples);
        	adc_result = 0;
        	_delay_ms(5000);
	}
#if 0
        for(uint8_t i = 0; i < adc_samples; i++)
        {
        	uint8_t l = ADCL;
        	adc_val += (ADCH << 8) | l;
		_delay_ms(10);
        }
        dprintf("ADC: %u\r\n", adc_val / adc_samples);
#endif
//        ADCSRA &= _BV(ADSC); /* turn the ADC back off */

}



struct command_struct commands[] = {
	{ .cmd = "FA", .handler = cmd_vfo_a, .iscat = 1 },
//	{ .cmd = "FB", .handler = cmd_vfo_b },
//	{ .cmd = "HI", .handler = cmd_hi },
	{ .cmd = "ADC", .handler = cmd_adc, .iscat = 0 },
	{ .cmd = NULL, .handler = NULL }, 

};

#define MAX_PARAMS 4
static char *para[MAX_PARAMS];

static void process_commands(void)
{
	char buf[BUF_DATA_SIZE+1];

	int result; 
	uint8_t parc;


	while( (result = rb_linebuf_get(&uart_rx_buf, buf, sizeof(buf)-1, false, false)) > 0)
	{	
		parc = rb_string_to_array(buf, para, MAX_PARAMS);
		for(int i = 0; commands[i].cmd != NULL; i++)
		{
			if(commands[i].iscat == true)
			{
				if(para[0][0] == commands[i].cmd[0] && para[0][1] == commands[i].cmd[1])
				{
					commands[i].handler(para, parc);
					break;;
				}
			} else {
				if(strcmp(commands[i].cmd, para[0]) == 0)
				{
					commands[i].handler(para, parc);
					break;
				}		
			}
		}
		/* we just drop crap we don't understand */
	} 
	
}

static void process_uart(void)
{
	char buf[BUF_DATA_SIZE];
	char *p;
	
	cli();
	if(usart1_int == false)
	{
		sei();
		return;
	} 

	usart1_int = false;
	sei();

	p = &buf[0];	
	
	while(bit_is_set(UCSR1A, RXC1))
	{
		*p++ = UDR1;
		if((p - buf) == BUF_DATA_SIZE)
		{
			if(rb_linebuf_parse(&uart_rx_buf, buf, p - &buf[0], true) == 0)
			{
				process_commands(); /* process the queue */
			}
			p = &buf[0];
		}
	}

	if(p > buf) {
		rb_linebuf_parse(&uart_rx_buf, buf, (p - buf), true);
	}
	return;
}
	




static void adc_init(void)
{
	ADCSRA |= (1<<ADEN);      //Enable ADC module
	ADMUX= 0x01;            // configuring PB2 to take input
	ADCSRB= 0x00;                   //Configuring free running mode
	ADCSRA |= (1<<ADSC)|(1<<ADATE);   //Start ADC conversion and enabling Auto trigger
}


// buf_head_t uart_tx_buf;

static void setup_linebuf(void)
{
	rb_linebuf_newbuf(&uart_rx_buf);
}





static void setup() 
{
	vfo_a_freq = 8300000;
	uint32_t loops = 0;
	cli();
#ifdef DEBUG
	uart_init();
#endif
	dprintf("Starting up and enabling PLL\r\n");
	PLLFRQ = _BV(PLLTM1) | _BV(PDIV3) | _BV(PDIV1) ;
	PLLCSR = _BV(PINDIV) | _BV(PLLE);
	/* shut off the USB controller before we enable interrupts again - we'll go splat otherwise */
	USBCON = 0;

	while((PLLCSR & (1<<PLOCK)) == 0)
	{
		dprintf("ctcss-n3ryb - waiting for PLL lock\r\n");	
	}

	setup_linebuf();	
	dprintf("setting up pwm timers\r\n");
	setup_pwm();
	
	dprintf("timers have been setup\r\n");	

	dprintf("doing spi_init()\r\n");
	spi_init();
	dprintf("survived spi_init()\r\n");

	dprintf("loading saved frequencies\r\n");
	load_saved_frequency();
	dprintf("Enabling interrupts\r\n");
  	sei();
  	dprintf("Enabled interrupts successfully\r\n");
  	dprintf("Setting DDS frequency to: %lu cycles\r\n", vfo_a_freq);
  	setWave(SINE, vfo_a_freq);

  	while(1)
  	{
  		loops++;
  		
  		if(loops % 100)
  		{
	  		process_uart();
	  		process_commands();
		}
  	}
  	
#ifdef TESTING	
//	loop_all();
#endif

}

static uint16_t adc_avg(void)
{
	const uint8_t adc_samples = 8; /* keep this powers of 2 otherwise the division is no longer a bit shift */
	uint16_t adc_val = 0;

	for(uint8_t i = 0; i < adc_samples; i++)
	{
		adc_val += (ADCL|(ADCH<<8));
		_delay_ms(5);
	}
	return (adc_val / adc_samples);
}


/* minus 1 since the table is 0 zero based */
#define FIRST_TOGGLE (7-1)
#define SECOND_TOGGLE (3-1)


static inline bool compare_adc(const uint16_t adc_val, const uint16_t adc2_val, const uint16_t start, const uint16_t end)
{
	if((adc_val >= start && adc_val <= end) && (adc2_val >= start && adc2_val <= end))
		return true;
	else
		return false;
}


static void loop() 
{
	uint16_t adc_val;
	uint16_t adc2_val;
	uint8_t mode = MODE_IDLE; 
	uint8_t halfway_count = 0;

//	process_commands();
	while(1)
	{
		process_commands();		

		adc_val = adc_avg();
		_delay_ms(500);
		adc2_val = adc_avg();
		halfway_count++;		
		switch(mode)
		{
			case MODE_IDLE:
			{
				/* toggle to channel 73  */
				if(compare_adc(adc_val, adc2_val,  freq_table[FIRST_TOGGLE].start, freq_table[FIRST_TOGGLE].end))
				{
					/* got the first toggle, turn off the lights */
					mode = MODE_HALFWAY;
					halfway_count = 1;
					fast_blink(3);
					led_off();
				}
				break;
			}
			case MODE_HALFWAY: 
			{
				if(compare_adc(adc_val, adc2_val,  freq_table[SECOND_TOGGLE].start, freq_table[SECOND_TOGGLE].end))
				{
					mode = MODE_PROGRAM;
					halfway_count = 0;
					fast_blink(3);
					led_off();
					_delay_ms(6000);
					break;
				}
				break;
				
			} 
			case MODE_PROGRAM:
			{
				for(uint8_t x = 0; x < sizeof(freq_table)/sizeof(struct frequencies); x++)
				{
					/* do both samples agree */
					if(compare_adc(adc_val, adc2_val,  freq_table[x].start, freq_table[x].end))
					{
						if(cur_freq == x)
						{
							_delay_ms(200);
						} else {
							change_frequency(x, true);
							mode = MODE_IDLE;
							led_on();
							_delay_ms(2000); /* don't make any more changes for a wee bit */
							break;
						}
					}
				}
				break;
			}
			default:
			{
				break;
			}
			
		}

		if(halfway_count == 8 && mode != MODE_IDLE)
		{
			mode = MODE_IDLE;
			halfway_count = 0;
			timeout_blink();
		}
	}
}


ISR(TIMER0_COMPA_vect) 
{
	OCR4B = sine_wave[((counter += cur_mult) >> 8)];
}

int main(void)
{
	setup();
	loop();	
}

