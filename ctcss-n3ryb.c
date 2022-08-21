#define BAUD 115200
#define BAUD_TOL 3
#define REF_FREQ             25000000



#include <stdint.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <util/delay.h>
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
#include "ad9833.h"
#include "event.h"
#include "pwm-sine.h"

/* globals - we have globals.  it's embedded. whatever */


//volatile uint32_t tick;

buf_head_t uart_rx_buf;
buf_head_t uart_tx_buf;

// uint32_t vfo_a_freq;




uint16_t current_band;
uint16_t current_channel;


struct ev_entry *read_channel_ev; 
static void read_channel(void *data);

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


/* these are (Hz * 8) but experimentally checked to be on frequency */
enum {
	/* these are (Hz * 8) but experimentally checked to be on frequency */

	MULT_77_0 = 618,
	MULT_100_0 = 800,
	MULT_103_5 = 828,
	MULT_118_8 = 949,
	MULT_123_0 = 984,
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

//static uint8_t EEMEM saved_frequency = HZ_123;

struct frequencies 
{
	const char *tone;
	const uint16_t mult;
	const uint16_t start;
	const uint16_t end;
	const uint16_t channel;
};

/* So we are using a precomputed frequency table for the LSB and MSB half of the frequency
 * this all assumes that the clock on the ad9833 is running at 25MHz
 * 
        uint32_t freq_data = ((uint64_t)frequency << 28) / REF_FREQ;
        uint16_t freq_MSB = (freq_data >> 14);  //| FREQ0;
        uint16_t freq_LSB = (freq_data & 0x3FFF); //| FREQ0;

 */
struct memory_entry {
	uint16_t freq_msb;
	uint16_t freq_lsb;
//	const uint16_t band;
//	const uint16_t channel;	
	uint16_t tone_mult;
};

/* band 0 - 144
 * band 1 - 144.5
 * band 2 - 145.0
 * band 3 - 145.5
 * band 4 - 146.0
 * band 5 - 146.5
 * band 6 - 147.0
 * band 7 - 147.5

*/


#if 1
struct memory_entry EEMEM band_1[11] = {};
struct memory_entry EEMEM band_2[11] = {};

struct memory_entry EEMEM band_3[11] = {
	{},
	{ .tone_mult = MULT_107_2, .freq_msb = 5328, .freq_lsb = 1258 }, /* 145.13000 Haymarket  N3KL */
	{ .tone_mult = MULT_179_9, .freq_msb = 5341, .freq_lsb = 3014 }, /* 145.15000 Martinsburg  W8ORS */
	{ .tone_mult = MULT_118_8, .freq_msb = 5367, .freq_lsb = 6527 }, /* 145.19000 Moorefield  N8VAA */
	{ .tone_mult = MULT_141_3, .freq_msb = 5380, .freq_lsb = 8283 }, /* 145.21000 Front Royal High Knob Mountain K4QJZ */
	{ .tone_mult = MULT_123_0, .freq_msb = 5419, .freq_lsb = 13552 }, /* 145.27000 Meyersdale Hays Mill Fire Tower KQ3M */
	{ .tone_mult = MULT_146_2, .freq_msb = 5498, .freq_lsb = 7707 }, /* 145.39000 Winchester North Mountain KG4Y */
	{ .tone_mult = MULT_123_0, .freq_msb = 5537, .freq_lsb = 12976 }, /* 145.45000 Oldtown Warrior Mountain WMA W3YMW */
	{ .tone_mult = MULT_123_0, .freq_msb = 5564, .freq_lsb = 104 }, /* 145.49000 Bedford  K3NQT */
};

struct memory_entry EEMEM band_4[11] = {};

struct memory_entry EEMEM band_5[11] = {
	{},
	{ .tone_mult = MULT_131_8, .freq_msb = 5324, .freq_lsb = 13107 }, /* 146.62500 New Market Luray Caverns N4YSA */
	{ .tone_mult = MULT_123_0, .freq_msb = 5403, .freq_lsb = 7261 }, /* 146.74500 Berkeley Springs Cacapon Mountain KK3L */
	{ .tone_mult = MULT_123_0, .freq_msb = 5432, .freq_lsb = 15309 }, /* 146.79000 Clearville Martin Hill K3NQT */
	{ .tone_mult = MULT_123_0, .freq_msb = 5442, .freq_lsb = 12530 }, /* 146.80500 Oakland  KB8NUF */
	{ .tone_mult = MULT_146_2, .freq_msb = 5452, .freq_lsb = 9751 }, /* 146.82000 Winchester Great North Mountain W4RKC */
	{ .tone_mult = MULT_77_0, .freq_msb = 5472, .freq_lsb = 4194 }, /* 146.85000 Charles Town  WA4TXE */
	{ .tone_mult = MULT_123_0, .freq_msb = 5491, .freq_lsb = 15020 }, /* 146.88000 Midland Dan's Mountain W3YMW */
	{ .tone_mult = MULT_100_0, .freq_msb = 5531, .freq_lsb = 3905 }, /* 146.94000 Clear Spring Fairview Mountain W3CWC */
};

//struct memory_entry EEMEM band_6[11] = {};

struct memory_entry EEMEM band_6[11] = {
	{},
	{ .tone_mult = MULT_100_0, .freq_msb = 5321, .freq_lsb = 8572 }, /* 147.12000 Chambersburg Clark's Knob W3ACH */
	{ .tone_mult = MULT_167_9, .freq_msb = 5341, .freq_lsb = 3014 }, /* 147.15000 Blue Knob Ski Resort KB3KWD */
	{ .tone_mult = MULT_167_9, .freq_msb = 5351, .freq_lsb = 235 }, /* 147.16500 Warrenton  W4VA */
	{ .tone_mult = MULT_123_0, .freq_msb = 5409, .freq_lsb = 16331 }, /* 147.25500 Martinsburg  WB8YZV */
	{ .tone_mult = MULT_203_5, .freq_msb = 5419, .freq_lsb = 13552 }, /* 147.27000 Gum Spring  WB4IKL */
	{ .tone_mult = MULT_103_5, .freq_msb = 5429, .freq_lsb = 10774 }, /* 147.28500 Circleville Spruce Knob N8HON */
	{ .tone_mult = MULT_146_2, .freq_msb = 5439, .freq_lsb = 7995 }, /* 147.30000 Bluemont Blue Ridge WA4TSC */
	{ .tone_mult = MULT_131_8, .freq_msb = 5449, .freq_lsb = 5216 }, /* 147.31500 Basye Great North Mountain K4MRA */
	{ .tone_mult = MULT_123_0, .freq_msb = 5468, .freq_lsb = 16043 }, /* 147.34500 Clear Spring  K3MAD */
	{ .tone_mult = MULT_127_3, .freq_msb = 5478, .freq_lsb = 13264 }, /* 147.36000 Skyline  K7SOB */
};


struct memory_entry EEMEM band_7[11] = {
	{},
	{ .tone_mult = MULT_100_0, .freq_msb = 5321, .freq_lsb = 8572 }, /* 147.12000 Chambersburg Clark's Knob W3ACH */
	{ .tone_mult = MULT_167_9, .freq_msb = 5341, .freq_lsb = 3014 }, /* 147.15000 Blue Knob Ski Resort KB3KWD */
	{ .tone_mult = MULT_167_9, .freq_msb = 5351, .freq_lsb = 235 }, /* 147.16500 Warrenton  W4VA */
	{ .tone_mult = MULT_123_0, .freq_msb = 5409, .freq_lsb = 16331 }, /* 147.25500 Martinsburg  WB8YZV */
	{ .tone_mult = MULT_203_5, .freq_msb = 5419, .freq_lsb = 13552 }, /* 147.27000 Gum Spring  WB4IKL */
	{ .tone_mult = MULT_103_5, .freq_msb = 5429, .freq_lsb = 10774 }, /* 147.28500 Circleville Spruce Knob N8HON */
	{ .tone_mult = MULT_146_2, .freq_msb = 5439, .freq_lsb = 7995 }, /* 147.30000 Bluemont Blue Ridge WA4TSC */
	{ .tone_mult = MULT_131_8, .freq_msb = 5449, .freq_lsb = 5216 }, /* 147.31500 Basye Great North Mountain K4MRA */
	{ .tone_mult = MULT_123_0, .freq_msb = 5468, .freq_lsb = 16043 }, /* 147.34500 Clear Spring  K3MAD */
	{ .tone_mult = MULT_127_3, .freq_msb = 5478, .freq_lsb = 13264 }, /* 147.36000 Skyline  K7SOB */
};

struct memory_entry EEMEM band_8[11];



#endif

/*
91k  - 990 5V     1
100k - 960 - 4.8  2
120k - 896 - 4.45 3
180k - 768 - 3.66 4
300k - 543 - 2.65 5
360k - 496 - 2.35 6
430k - 448 - 2.08 7
470k - 414 - 1.94 8
510k - 385 - 1.81 9
680k - 319 - 1.46 10
820k - 255 - 1.23 11
1.5m - 191 - 0.754 12
*/

// adc values for the band/channel switch, indexed by 1, assume a +30/-30 on these
static const uint16_t channel_switch[] = {
	0, 990, 960, 896, 768, 543, 496, 448, 414, 385, 319, 225, 191
};


#if 0
static const struct frequencies freq_table[] = {
	{ .tone = "100",	.mult = MULT_100_0, 	.start = 950,	.end = 1024,	.channel = 1	},
	{ .tone = "103.5",	.mult = MULT_103_5, 	.start = 860,	.end = 920,	.channel = 2	},
	{ .tone = "118.5",	.mult = MULT_118_8, 	.start = 751,	.end = 860,	.channel = 3	},
	{ .tone = "123",  	.mult = MULT_123_0, 	.start = 651,	.end = 750,	.channel = 4	},
	{ .tone = "127.3",	.mult = MULT_127_3, 	.start = 550,	.end = 650,	.channel = 5 	},
	{ .tone = "131.8",	.mult = MULT_131_8, 	.start = 450,	.end = 550,	.channel = 6 	},
	{ .tone = "141.3",	.mult = MULT_141_3, 	.start = 330,	.end = 450,	.channel = 7	},
	{ .tone = "146.2",	.mult = MULT_146_2, 	.start = 300,	.end = 329,	.channel = 8	},
	{ .tone = "167.9",	.mult = MULT_167_9, 	.start = 200,	.end = 299,	.channel = 9 	},
	{ .tone = "179.9",	.mult = MULT_179_9, 	.start = 130,	.end = 200,	.channel = 10	},
	{ .tone = "0", 		.mult = MULT_NONE, 	.start = 80,	.end = 129,	.channel = 11	},
};
#endif

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

static  uint8_t atous(const char *num)
{
	uint8_t val = 0;
        while (*num && (*num >= '0' && *num <= '9'))
        {
	    	val = (val * 10) + *num - '0';
	    	num++;
        }
        return val;
}


static inline __attribute__((always_inline)) void led_off(void)
{
	PORTD &= ~_BV(PD1);
}

static inline  __attribute__((always_inline)) void led_on(void)
{
	PORTD |= _BV(PD1);
}

static inline __attribute__((always_inline)) void led_toggle(void)
{
	PORTD ^= _BV(PD1); 
}


#if 0
static int16_t rb_linebuf_flush(buf_head_t * bufhead)
{
	buf_line_t *bufline;
	int16_t retval = 0;

	if(bufhead->list.head == NULL)
		return -1;

	bufline = bufhead->list.head->data;
	if(!bufline->terminated)
                return -1;
	
	if(!bit_is_set(UCSR1A, UDRE1))
		return -1;
	while(bit_is_set(UCSR1A, UDRE1))
	{
		UDR1 = *(bufline->buf + bufhead->writeofs);
		retval++;
		bufhead->writeofs++;
	}
	
	if(bufhead->writeofs == bufline->len)
	{
		bufhead->writeofs = 0;
		rb_linebuf_done_line(bufhead, bufline, bufhead->list.head);
	}
	return retval;
}
#endif

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



static inline  __attribute__((always_inline)) void SPI_write(const uint8_t data)
{
	SPDR = data;
	asm volatile( "nop" );
	loop_until_bit_is_set(SPSR, SPIF);
}



static __attribute__((noinline)) void SPI_write16 (const uint16_t data)          
{
	SPI_write((data >> 8) & 0xFF);
	SPI_write(data & 0xFF);
}

void spi_init(void)
{
	// PB1 = SCLK, PB2 = MOSI, PB3 = MISO
	
	DDRB |= _BV(PB1) | _BV(PB2);
	PORTB &= ~_BV(PB1);
	PORTB &= ~_BV(PB2);
	SPCR  = _BV(SPE)| _BV(MSTR)| _BV(SPR0) | _BV(CPOL);
}


inline static void ad9833_sselect_high(void)
{
        PORTF |= _BV(PF7);
        _delay_us(40);
}

inline static void ad9833_sselect_low(void)
{
        PORTF &= ~_BV(PF7);
        _delay_us(10);
}


static bool dds_power;

static void dds_amp_on(void)
{
	PORTF |= _BV(PF4); 
}
static void dds_amp_off(void)
{
	PORTF &= ~_BV(PF4);
}

static void ad9833_off(void)
{
	PORTF &= ~_BV(PF5); 
	dds_power = 0;
}

static void ad9833_on(void)
{
	DDRF |= _BV(PF5);
	PORTF |= _BV(PF5);
	dds_power = 1;
}

static void ad9833_init(void)
{
	DDRF |= _BV(PF4);
	dds_amp_on();
	spi_init();

	ad9833_on();
	_delay_ms(10);

	DDRF |= _BV(PF7); // slave select

	ad9833_sselect_low();
	SPI_write16(AD9833_RESET);
	ad9833_sselect_high();
}


static void ad9833_shutdown(void)
{
	ad9833_sselect_low();
	SPI_write16(AD9833_RESET);
	ad9833_sselect_high();
	
	PORTF &= ~_BV(PF5); 

	ad9833_sselect_low();
	dds_power = 0;
	SPCR = 0;
}


static void ad9833_setvfo(uint16_t freq_msb, uint16_t freq_lsb)
{
	if(dds_power == false)
		ad9833_init();
	ad9833_sselect_low();

	SPI_write16(AD9833_B28);
	SPI_write16(freq_lsb | AD9833_D14); /* D14 is freq0 */
	SPI_write16(freq_msb | AD9833_D14);
	SPI_write16(AD9833_B28);
	ad9833_sselect_high();
}


static void __attribute__((noinline)) setWave(uint32_t frequency) 
{
	uint32_t freq_data = ((uint64_t)frequency << 28) / REF_FREQ;
	uint16_t freq_MSB = (freq_data >> 14);  //| FREQ0;
	uint16_t freq_LSB = (freq_data & 0x3FFF); //| FREQ0;

	if(dds_power == false)
		ad9833_init();

	ad9833_sselect_low();

	SPI_write16(AD9833_B28);
	SPI_write16(freq_LSB | AD9833_D14); /* D14 is freq0 */
	SPI_write16(freq_MSB | AD9833_D14);
	SPI_write16(AD9833_B28);
	freq_data = ((uint64_t)(frequency + 1200) << 28) / REF_FREQ;
	freq_MSB = (freq_data >> 14);
	freq_LSB = (freq_data & 0x3FFF);
	SPI_write16(freq_LSB | AD9833_D15); /* D15 is freq1 */
	SPI_write16(freq_MSB | AD9833_D15);
	ad9833_sselect_high();
}


typedef void CMDCB(char **parv, uint8_t parc);


struct command_struct 
{
	const char *cmd;
	CMDCB *handler;
	bool iscat; /* a cat command, only match the first two characters */
};


static void cmd_t(char **argv, uint8_t argc)
{
	for(uint16_t i = 0;i < 128;i++)
	{
		SPI_write16(AD9833_FSELECT1);
		_delay_ms(5);
		SPI_write16(AD9833_FSELECT0);
		_delay_ms(5);
	}
}

static void cmd_r(char **argv, uint8_t argc)
{
	SPI_write16(AD9833_FSELECT0);
}

static void cmd_f(char **argv, uint8_t argc)
{
	uint32_t vfo_a_freq = atoul(argv[1]);
	if(vfo_a_freq > 14000000)
	{
		dprintf("\r\nF: frequency too high\r\n");
		return;
	}
	dprintf("\r\nF: Setting VFO A to %lu\r\n", vfo_a_freq);
	setWave(vfo_a_freq);

}

static void cmd_tone(char **argv, uint8_t argc)
{
	if(argc != 1)
		return;
		
	set_ctcss(atoul(argv[1]));
	dprintf("\r\nF: Set frequency to (%u / 8)Hz \r\n", cur_mult);


}

static void adc_avg_channel(uint16_t *channel, uint16_t *band);

static void cmd_adc(char **argv, uint8_t argc)
{
	uint16_t channel, band;
	
	if(argc == 1)
	
	adc_avg_channel(&channel, &band);
	dprintf("\r\nCMD_ADC: %u %u\r\n", channel, band);
	
}

static void cmd_adcoff(char **argv, uint8_t argc)
{
	dprintf("\r\nADC OFF\r\n");
	rb_event_delete(read_channel_ev);
}

static void cmd_adcon(char **argv, uint8_t argc)
{
	dprintf("\r\nADC ON\r\n");
	read_channel_ev = rb_event_add(read_channel, NULL, 5000, 0);
}


#if 0
static void cmd_vfo_a(char **argv, uint8_t argc)
{
	uint32_t tfreq;

	if(argv[0][1] == ';') 
	{
		dprintf("FA;%lu;\r\n", vfo_a_freq);
	}
		

	vfo_a_freq = tfreq = atoul(&argv[0][2]);
	setWave(tfreq);
	dprintf("\r\nSetting wave to tfreq: %lu\r\n", vfo_a_freq);
	dprintf("FA;%lu;\r\n", tfreq);

}

static void cmd_vfo_b(const char *buf, size_t len)
{
	dprintf("Got cmd_vfo_a: %s %d\r\n", buf, len);


}

#endif

static void cmd_o(char **argv, uint8_t argc)
{
	dprintf("\r\nShutting off ad9833\r\n");
	ad9833_shutdown();	

}

static void cmd_d(char **argv, uint8_t argc)
{
	dprintf("\r\nShutting off dds amp\r\n");
	dds_amp_off();
}

static void cmd_D(char **argv, uint8_t argc)
{
	dprintf("\r\nTurning on dds amp\r\n");
	dds_amp_on();
}

static void cmd_chan(char **argv, uint8_t argc)
{
	uint8_t band, channel;
	
	band = atous(argv[1]);
	channel = atous(argv[2]);

	dprintf("\r\nCMD_CHAN band: %u chan %u\r\n", band, channel);
		

}

static void cmd_reboot(char **argv, uint8_t argc)
{
        typedef void (*do_reboot_t)(void);
        const do_reboot_t do_reboot = (do_reboot_t)((FLASHEND - 511) >> 1);
        
        dprintf("\r\n\r\nAttempting to reboot in 5 seconds\r\n");
        ad9833_shutdown();
        dds_amp_off();
        cli();
	MCUSR = 0; 
        TCCR0A = TCCR4A = 0; // make sure interrupts are off and timers are reset.
        _delay_ms(5000);
        do_reboot();
}

static struct command_struct commands[] = {
//	{ .cmd = "FA", .handler = cmd_vfo_a, .iscat = 1 },
	{ .cmd = "f", .handler = cmd_f   },
	{ .cmd = "o", .handler = cmd_o },
//	{ .cmd = "FB", .handler = cmd_vfo_b },
//	{ .cmd = "HI", .handler = cmd_hi },
	{ .cmd = "T", .handler = cmd_t },
	{ .cmd = "R", .handler = cmd_r },
	{ .cmd = "adc", .handler = cmd_adc },
	{ .cmd = "adcoff", .handler = cmd_adcoff },
	{ .cmd = "d", .handler = cmd_d },
	{ .cmd = "D", .handler = cmd_D },
	{ .cmd = "reboot", .handler = cmd_reboot },
	{ .cmd = "tone", .handler = cmd_tone },
	{ .cmd = "chan", .handler = cmd_chan },
	{ .cmd = NULL, .handler = NULL }, 

};

#define MAX_PARAMS 4

static void process_commands(void *unused)
{
	static char *para[MAX_PARAMS];
	static char buf[BUF_DATA_SIZE+1];

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

static void process_uart(void *unused)
{
	static char buf[BUF_DATA_SIZE];
	char *p;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if(usart1_int == false)
		{
			return;
		} 
		usart1_int = false;
	}

	p = buf;

	while(bit_is_set(UCSR1A, RXC1))
	{
		*p++ = UDR1;
		if((p - buf) == BUF_DATA_SIZE)
		{
			if(rb_linebuf_parse(&uart_rx_buf, buf, p - buf, true) == 0)
			{
				process_commands(NULL); /* process the queue */
			}
			p = buf;
		}

	}

	if(p > buf) {
		rb_linebuf_parse(&uart_rx_buf, buf, (p - buf), true);
	}
	return;
}
	



/* we are using two adc channels ADC11 and ADC12
 * ADC11 = PB4 - A8/D8  also PCINT4 -  This is our channel selector
 * ADC12 = PB5 - A9/D9  also PCINT5 -  This is our band selector 
 
 D8           A8              PB4                                     ADC11/PCINT4
 D9#          A9              PB5             PWM16           OC1A/#OC4B/ADC12/PCINT5

 */

//#define ADMUX_ADC11  = _BV(MUX0) | _BV(MUX1) |_BV(REFS0);

#define ADMUX_ADC11 (_BV(MUX0) | _BV(MUX1) |_BV(REFS0))
#define ADCSRB_ADC11 _BV(MUX5)

#define ADMUX_ADC12 (_BV(MUX2) | _BV(REFS0))
#define ADCSRB_ADC12 _BV(MUX5)


static void adc_init(void)
{
	ADCSRA = 0;

	DDRB &= ~(_BV(PB4) | _BV(PB5)); 
	PORTB &= ~(_BV(PB4) | _BV(PB5));	
	DIDR2 |= _BV(ADC11D) | _BV(ADC12D);


	ADMUX = ADMUX_ADC11;
	ADCSRB = ADCSRB_ADC11;
	

	ADCSRA = _BV(ADEN);
	 
	ADCSRA |= _BV(ADSC) | _BV(ADATE) | _BV(ADEN);

}

static uint16_t adc_avg(void)
{
	const uint8_t adc_samples = 16; /* keep this powers of 2 otherwise the division is no longer a bit shift */
	uint16_t adc_val = 0;
	
	for(uint8_t i = 0; i < adc_samples; i++)
	{
		while(!bit_is_set(ADCSRA,ADSC));
		adc_val += ADCW;
	}
	return (adc_val / adc_samples);
}


static void adc_avg_channel(uint16_t *channel, uint16_t *band)
{

	ADMUX = ADMUX_ADC11;
	ADCSRB = ADCSRB_ADC11;

	*channel = adc_avg();
	*channel = adc_avg();
#if 1
	_delay_ms(10);
	ADMUX = ADMUX_ADC12;
	ADCSRB = ADCSRB_ADC12;
	ADCSRA |= _BV(ADSC); 
	_delay_us(30);
	*band = adc_avg(); /* throw away the first sample */
	*band = adc_avg();
	_delay_us(30);
#endif
//	dprintf("Got adc_avg for band %u\r\n");
}






bool lookup_channel(uint16_t adc_val, uint16_t *channel)
{
	for(uint16_t i = 1; i < sizeof(channel_switch) / sizeof(uint16_t) ; i++)
	{
		if((adc_val >= (channel_switch[i] - 20)) &&  (adc_val <= (channel_switch[i] + 25))   )
		{
			*channel = i;
			return true;
		}
	}
	return false;
}

static void set_channel(uint16_t band, uint16_t channel)
{
	struct memory_entry m;
	void *bptr;

	switch(band)
	{
		case 1:
			bptr = &band_1[channel];
			break;
		case 2:
			bptr = &band_2[channel];
			break;
		case 3:
			bptr = &band_3[channel];
			break;
		case 4:
			bptr = &band_4[channel];
			break;
		case 5:
			bptr = &band_5[channel];
			break;
		case 6:
			bptr = &band_6[channel];
			break;
		case 7:
			bptr = &band_7[channel];
			break;
		case 8:
			bptr = &band_8[channel];
			break;
		default: 
			bptr = &band_1[channel];
			break;


	}
	eeprom_read_block(&m, bptr, sizeof(m));
	dprintf("set_channel: band: %u channel: %u - %u %u - ctcss frequency: %u\r\n",  band, channel, m.freq_msb, m.freq_lsb, m.tone_mult);
	ad9833_setvfo(m.freq_msb, m.freq_lsb);
	cur_mult = m.tone_mult;	
}

static void read_channel(void *data)
{
	uint16_t c, b;
	uint16_t new_channel = 0, new_band = 0;
	adc_avg_channel(&c, &b);			
//	dprintf("%lu: ADC average: channel: %u band: %u\r\n", tick, c, b);

	new_band = 6;
	new_channel = 5;

	if(lookup_channel(c, &new_channel) == false)
	{
		dprintf("ADC out of range for channel: %u\r\n", c);
//		return;
	}

	if(lookup_channel(b, &new_band) == false)
	{
		dprintf("ADC out of range for band: %u\r\n", b);
//		return;
	}

	if((current_channel == new_channel) && (current_band == new_band))
	{
		dprintf("NO changes to make\r\n");
//		return;
	}
	current_band = new_band;
	current_channel = new_channel;
	set_channel(new_band, new_channel);

}



// buf_head_t uart_tx_buf;

static void setup_linebuf(void)
{
	rb_linebuf_newbuf(&uart_rx_buf);
	rb_linebuf_newbuf(&uart_tx_buf);
}


static uint16_t adc_avg(void);


static void blink_cb(void *data)
{
	dprintf("\r\n* - FAKE BLINK\r\n");
	led_toggle();
}


static void event_blink(uint8_t count)
{
	rb_event_add(blink_cb, NULL, 250, count);
}


static void setup() 
{
	MCUSR = ~(1 << WDRF);
	wdt_disable();

	led_on();
	
	cli();
	
	rb_event_init();
	setup_linebuf();	
	uart_init();

	dprintf("\r\nFT-221 DDS/CTCSS encoder thingie\r\nHi N3RYB!\r\n");
	PLLFRQ = _BV(PLLTM1) | _BV(PDIV3) | _BV(PDIV1) | _BV(PLLUSB); /* run at 96MHz */
	PLLCSR = _BV(PINDIV) | _BV(PLLE);

	/* shut off the USB controller before we enable interrupts again - we'll go splat otherwise */
	USBCON = 0;

	while((PLLCSR & _BV(PLOCK)) == 0)

	DDRD |= _BV(PD1);


	PRR0 |= _BV(PRTWI); // turn off two wire
	PRR0 |= _BV(PRUSB); // and usb too
	

	setup_pwm();
	spi_init();

	dprintf("Enabling interrupts;\r\n");
  	sei();
  	dprintf("Enabled interrupts successfully\r\n");


 	ad9833_init();

 	adc_init();
 	dprintf("Out of adc_init()\r\n");
 	
	read_channel_ev = rb_event_add(read_channel, NULL, 5000, 0);
	rb_event_add(process_uart, NULL, 50, 0);
	rb_event_add(process_commands, NULL, 20, 0);
	
	event_blink(4);
	dprintf("setup finished\r\n");	
}


int main(void)
{
	setup();
	while(1)
	{
		rb_event_run();
	}
}

