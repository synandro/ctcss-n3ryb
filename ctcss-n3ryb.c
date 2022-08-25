#define BAUD 115200
#define BAUD_TOL 3

// #define AD9833_FREQ 25000000

#define CHAN_VFO 12
#define CHAN_MAX 12
#define BAND_MAX 8 

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
#define dprintf(...) printf_P(__VA_ARGS__)
#else
#define dprintf(...)
#endif

#include "tools.h"
#include "ad9833.h"
#include "event.h"
#include "pwm-sine.h"

/* globals - we have globals.  it's embedded. whatever */

buf_head_t uart_rx_buf;
buf_head_t uart_tx_buf;

uint16_t current_band;
uint16_t current_channel;


struct ev_entry *read_channel_ev; 
static void read_channel(void *data);
static void set_channel(uint16_t band, uint16_t channel);

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

struct memory_entry {
	uint16_t freq_msb;
	uint16_t freq_lsb;
	uint16_t tone_mult;
};


/* So we are using a precomputed frequency table for the LSB and MSB half of the frequency
 * this all assumes that the clock on the ad9833 is running at 25MHz
 * 
        uint32_t freq_data = ((uint64_t)frequency << 28) / AD9833_FREQ;
        uint16_t freq_MSB = (freq_data >> 14);  //| FREQ0;
        uint16_t freq_LSB = (freq_data & 0x3FFF); //| FREQ0;

 */

/* band 0 - 144
 * band 1 - 144.5
 * band 2 - 145.0
 * band 3 - 145.5
 * band 4 - 146.0
 * band 5 - 146.5
 * band 6 - 147.0
 * band 7 - 147.5

*/

struct memory_entry bands[BAND_MAX][CHAN_MAX - 1] EEMEM = {
	
	{
		/* band 0 - 144-144.49 */
		{ .tone_mult = 0, .freq_msb = 5373, .freq_lsb = 15597 }, /* 144.200 */
		{ .tone_mult = 0, .freq_msb = 5498, .freq_lsb = 7707 }, /* 144.390 */
	},	
	{
		/* band 1 - 144.5 - 145.0 */
	},	
	{
		/* band 2 - 145.0 - 145.49 */ 	
		{ .tone_mult = MULT_107_2, .freq_msb = 5328, .freq_lsb = 1258 }, /* 145.13000 Haymarket  N3KL */
		{ .tone_mult = MULT_179_9, .freq_msb = 5341, .freq_lsb = 3014 }, /* 145.15000 Martinsburg  W8ORS */
		{ .tone_mult = MULT_118_8, .freq_msb = 5367, .freq_lsb = 6527 }, /* 145.19000 Moorefield  N8VAA */
		{ .tone_mult = MULT_141_3, .freq_msb = 5380, .freq_lsb = 8283 }, /* 145.21000 Front Royal High Knob Mountain K4QJZ */
		{ .tone_mult = MULT_123_0, .freq_msb = 5419, .freq_lsb = 13552 }, /* 145.27000 Meyersdale Hays Mill Fire Tower KQ3M */
		{ .tone_mult = MULT_146_2, .freq_msb = 5498, .freq_lsb = 7707 }, /* 145.39000 Winchester North Mountain KG4Y */
		{ .tone_mult = MULT_123_0, .freq_msb = 5537, .freq_lsb = 12976 }, /* 145.45000 Oldtown Warrior Mountain WMA W3YMW */
		{ .tone_mult = MULT_123_0, .freq_msb = 5564, .freq_lsb = 104 }, /* 145.49000 Bedford  K3NQT */
	},
	{	
		/* band 3 - 145.5 - 145.99 */
	},
	{	
		/* band 4 - 146.0 - 146.6 */
		{ .tone_mult = 0, .freq_msb = 5583, .freq_lsb = 10931 }, /* 146.520 */
		{ .tone_mult = 0, .freq_msb = 5596, .freq_lsb = 12687 }, /* 146.540 */
		{ .tone_mult = 0, .freq_msb = 5609, .freq_lsb = 14444 }, /* 146.560 */
		{ .tone_mult = 0, .freq_msb = 5622, .freq_lsb = 16200 }, /* 146.580 */
		
	},

	{
		/* band 5 - 146.5 - 146.99 */
		{ .tone_mult = MULT_131_8, .freq_msb = 5324, .freq_lsb = 13107 }, /* 146.62500 New Market Luray Caverns N4YSA */
		{ .tone_mult = MULT_123_0, .freq_msb = 5403, .freq_lsb = 7261 }, /* 146.74500 Berkeley Springs Cacapon Mountain KK3L */
		{ .tone_mult = MULT_123_0, .freq_msb = 5432, .freq_lsb = 15309 }, /* 146.79000 Clearville Martin Hill K3NQT */
		{ .tone_mult = MULT_123_0, .freq_msb = 5442, .freq_lsb = 12530 }, /* 146.80500 Oakland  KB8NUF */
		{ .tone_mult = MULT_146_2, .freq_msb = 5452, .freq_lsb = 9751 }, /* 146.82000 Winchester Great North Mountain W4RKC */
		{ .tone_mult = MULT_77_0, .freq_msb = 5472, .freq_lsb = 4194 }, /* 146.85000 Charles Town  WA4TXE */
		{ .tone_mult = MULT_123_0, .freq_msb = 5491, .freq_lsb = 15020 }, /* 146.88000 Midland Dan's Mountain W3YMW */
		{ .tone_mult = MULT_100_0, .freq_msb = 5531, .freq_lsb = 3905 }, /* 146.94000 Clear Spring Fairview Mountain W3CWC */
	},
	{
		/* band 6 */
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
	}, 
	{} 	/* band 7 */
};




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


#if 0
// adc values for the band/channel switch, indexed by 1, assume a +30/-30 on these
static const uint16_t channel_switch[] = {
	0, 990, 960, 896, 768, 543, 496, 448, 414, 385, 319, 225, 191
};
#endif

/* min/max range for each band switch */
static uint16_t band_switch_range[BAND_MAX][2] EEMEM  = {
	{ 1024,990 }, 
	{ 980,950 },
	{ 900,850 },
	{ 800,700 },
	{ 600,520 }, 
	{ 510,480 },
	{ 470,430 }, 
	{ 420,400 }, 
//	{ 390,350 },
//	{ 330,280 },
//	{ 240,210 },
//	{ 200,150 },
};

/* min/max range for each band switch */
static uint16_t channel_switch_range[CHAN_MAX][2] EEMEM = {
	{ 1024,990 }, 
	{ 980,950 },
	{ 900,850 },
	{ 800,700 },
	{ 600,520 }, 
	{ 510,480 },
	{ 470,430 }, 
	{ 420,400 }, 
	{ 390,350 },
	{ 330,280 },
	{ 240,210 },
	{ 200,150 },
};




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

static uint16_t atoui(const char *num)
{
	uint16_t val = 0;
        while (*num && (*num >= '0' && *num <= '9'))
        {
	    	val = (val * 10) + *num - '0';
	    	num++;
        }
        return val;
}


static inline __attribute__((always_inline)) void led_off(void)
{
	DDRD |= _BV(PD1) | _BV(PD5);
	PORTD &= ~(_BV(PD1) | _BV(PD5));

	DDRC |= _BV(PC7);
	PORTC &= ~_BV(PC7);
}

static inline  __attribute__((always_inline)) void led_on(void)
{
	DDRD |= _BV(PD1) |  _BV(PD5);
	PORTD |= _BV(PD1) | _BV(PD5);
}

static inline __attribute__((always_inline)) void led_toggle(void)
{
	PORTD ^= _BV(PD1) | _BV(PD5); 
//	PORTD ^= _BV(PD5);
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
}



static inline  __attribute__((always_inline)) void SPI_write(const uint8_t data)
{
	SPDR = data;
	asm volatile( "nop" );
	loop_until_bit_is_set(SPSR, SPIF);
}



static void SPI_write16 (const uint16_t data)          
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


static inline void ad9833_sselect_high(void)
{
        PORTF |= _BV(PF7);
        _delay_us(40);
}

static inline void ad9833_sselect_low(void)
{
        PORTF &= ~_BV(PF7);
        _delay_us(10);
}


static bool dds_power;

static void dds_amp_on(void)
{
	DDRF |= _BV(PF4);
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
	dds_amp_on();
	spi_init();

	DDRF |= _BV(PF7); // slave select
	ad9833_sselect_high();

	ad9833_on();

	_delay_ms(10);

	ad9833_sselect_low();
	SPI_write16(AD9833_RESET);
	ad9833_sselect_high();
}


static void ad9833_shutdown(void)
{
	dds_amp_off();
	ad9833_sselect_low();
	SPI_write16(AD9833_RESET);
	ad9833_sselect_high();
	
	PORTF &= ~_BV(PF5); 

	ad9833_off();
	ad9833_sselect_low();
	dds_power = 0;
	SPCR = 0;
}


static void ad9833_setvfo(uint16_t freq_msb, uint16_t freq_lsb)
{
	if(dds_power == false)
		ad9833_init();

	dds_amp_on();
	ad9833_sselect_low();

	SPI_write16(AD9833_B28);
	SPI_write16(freq_lsb | AD9833_D14); /* D14 is freq0 */
	SPI_write16(freq_msb | AD9833_D14);
	SPI_write16(AD9833_B28);
	ad9833_sselect_high();
}


static void __attribute__((noinline)) setWave(uint32_t frequency) 
{
#if 1

	uint32_t freq_data = ((uint64_t)frequency << 28) / AD9833_FREQ;
	uint16_t freq_msb = (freq_data >> 14);  //| FREQ0;
	uint16_t freq_lsb = (freq_data & 0x3FFF); //| FREQ0;
#endif
#if 0
	uint32_t freq_data = (double)(frequency * pow(2, 28)) / (double)AD9833_FREQ;
	uint32_t freq_msb = (freq_data >> 14);
	uint32_t freq_lsb = (freq_data & 0x3FFF); 
#endif


	if(dds_power == false)
		ad9833_init();

	ad9833_sselect_low();

	SPI_write16(AD9833_B28);
	SPI_write16(freq_lsb | AD9833_D14); /* D14 is freq0 */
	SPI_write16(freq_msb | AD9833_D14);
	SPI_write16(AD9833_B28);
	freq_data = ((uint64_t)(frequency + 1200) << 28) / AD9833_FREQ;
	freq_msb = (freq_data >> 14);
	freq_lsb = (freq_data & 0x3FFF);
	SPI_write16(freq_lsb | AD9833_D15); /* D15 is freq1 */
	SPI_write16(freq_msb | AD9833_D15);
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
		dprintf(PSTR("\r\nF: frequency too high\r\n"));
		return;
	}
	dprintf(PSTR("\r\nF: Setting VFO A to %lu\r\n"), vfo_a_freq);
	setWave(vfo_a_freq);

}

static void cmd_tone(char **argv, uint8_t argc)
{
	if(argc != 1)
		return;
		
	set_ctcss(atoul(argv[1]));
	dprintf(PSTR("\r\nF: Set frequency to (%u / 8)Hz \r\n"), cur_mult);
}

static void adc_avg_channel(uint16_t *channel, uint16_t *band);

static void cmd_adc(char **argv, uint8_t argc)
{
	uint16_t channel, band;
	adc_avg_channel(&channel, &band);
	dprintf(PSTR("\r\nCMD_ADC: %u %u\r\n"), channel, band);
}

static void cmd_adcoff(char **argv, uint8_t argc)
{
	dprintf(PSTR("\r\nADC OFF\r\n"));
	rb_event_delete(read_channel_ev);
}

static void cmd_adcon(char **argv, uint8_t argc)
{
	dprintf(PSTR("\r\nADC ON\r\n"));
	read_channel_ev = rb_event_add(read_channel, NULL, 5000, 0);
}


static void cmd_o(char **argv, uint8_t argc)
{
	dprintf(PSTR("\r\nShutting off ad9833\r\n"));
	ad9833_shutdown();	
}

static void cmd_ddsampoff(char **argv, uint8_t argc)
{
	dprintf(PSTR("\r\nShutting off dds amp\r\n"));
	dds_amp_off();
}

static void cmd_ddsampon(char **argv, uint8_t argc)
{
	dprintf(PSTR("\r\nTurning on dds amp\r\n"));
	dds_amp_on();
}

static void cmd_chan(char **argv, uint8_t argc)
{
	uint8_t band, channel;
	
	band = atous(argv[1]);
	channel = atous(argv[2]);

	dprintf(PSTR("\r\nCMD_CHAN band: %u chan %u\r\n"), band, channel);
	set_channel(band, channel);
}

static void cmd_savechan(char **argv, uint8_t argc)
{
	struct memory_entry m;
	uint8_t band, channel;
	const char *status_str;
	/* savechan band channel msb lsb tone */
	if(argc != 6)
	{
		dprintf(PSTR("\r\nSAVECHAN BAND CHANNEL MSB LSB TONE\r\n"));
		return;
	}

	band = atous(argv[1]);
	channel = atous(argv[2]);

	if(band > BAND_MAX - 1 || channel > CHAN_MAX - 1)
	{
		dprintf(PSTR("\r\nSAVECHAN: Max 8 bands / 11 channels\r\n"));
		return;
	}

	/* i don't care what garbage you give me */
	m.freq_msb = atoui(argv[3]);
	m.freq_lsb = atoui(argv[4]);
	m.tone_mult = atoui(argv[5]);
	

	status_str = PSTR("\r\nSAVECHAN: %u: %u:%u: %u %u %u\r\n");

	dprintf(status_str, 1,  band, channel, m.freq_msb, m.freq_lsb, m.tone_mult);
	eeprom_write_block(&m, &bands[band][channel], sizeof(m));
	eeprom_busy_wait();
	eeprom_read_block(&m, &bands[band][channel], sizeof(m));
	dprintf(status_str, 0,  band, channel, m.freq_msb, m.freq_lsb, m.tone_mult);
}

static void cmd_saveadc_c(char **argv, uint8_t argc)
{
	uint8_t channel;
	uint16_t range[2];
	
	if(argc != 3)
	{
		dprintf(PSTR("\r\nSAVEADC_C channel start end\r\n"));
		return;
	}
	
	channel = atoui(argv[1]);
	range[0] = atoui(argv[2]);	
	range[1] = atoui(argv[3]);

	if(channel > 11)
	{
		dprintf(PSTR("\r\nSAVEADC_C max channel is 11\r\n"));
		return;
	}
	dprintf(PSTR("\r\nSAVEADC_C: channel:%u range:%u-%u\r\n"), channel, range[0], range[1]);
	eeprom_write_block(&range, &channel_switch_range[channel], sizeof(range));
	eeprom_busy_wait();
	eeprom_read_block(&range, &channel_switch_range[channel], sizeof(range));
	dprintf(PSTR("\r\nSAVEADC_C: Read: channel:%u range:%u-%u\r\n"), channel, range[0], range[1]);
	return;
	
}

static void cmd_saveadc_b(char **argv, uint8_t argc)
{
	uint8_t band;
	uint16_t range[2];

	if(argc != 3)
	{
		dprintf(PSTR("\r\nSAVEADC_B band start end\r\n"));
		return;
	}

	band = atoui(argv[1]);
	range[0] = atoui(argv[2]);	
	range[1] = atoui(argv[3]);

	if(band > 8)
	{
		dprintf(PSTR("\r\nSAVEADC_B max band is 8\n"));
		return;
	}

	dprintf(PSTR("\r\nSAVEADC_B: channel:%u range:%u-%u\r\n"), band, range[0], range[1]);
	eeprom_write_block(&range, &band_switch_range[band], sizeof(range));
	eeprom_busy_wait();
	eeprom_read_block(&range, &band_switch_range[band], sizeof(range));
	dprintf(PSTR("\r\nSAVEADC_B: Read: channel:%u range:%u-%u\r\n"), band, range[0], range[1]);
	
}

static void cmd_reboot(char **argv, uint8_t argc)
{
        typedef void (*do_reboot_t)(void);
        const do_reboot_t do_reboot = (do_reboot_t)((FLASHEND - 511) >> 1);
        
        dprintf(PSTR("\r\n\r\nAttempting to reboot in 5 seconds\r\n"));
        ad9833_shutdown();
        dds_amp_off();
        cli();
	MCUSR = 0; 
        TCCR0A = TCCR4A = 0; // make sure interrupts are off and timers are reset.
        _delay_ms(5000);
        do_reboot();
}

static void cmd_help(char **argv, uint8_t argc);

/* not sure this would be worth stuffing in PROGMEM for the complexity it would 
 * add to the command parser 
 */
static const struct command_struct commands[] = {
//	{ .cmd = "FA", .handler = cmd_vfo_a, .iscat = 1 },
	{ .cmd = "f", .handler = cmd_f   },
	{ .cmd = "o", .handler = cmd_o },
//	{ .cmd = "FB", .handler = cmd_vfo_b },
//	{ .cmd = "HI", .handler = cmd_hi },
	{ .cmd = "T", .handler = cmd_t },
	{ .cmd = "R", .handler = cmd_r },
	{ .cmd = "adc", .handler = cmd_adc },
	{ .cmd = "adcoff", .handler = cmd_adcoff },
	{ .cmd = "adcon", .handler = cmd_adcon },
	{ .cmd = "ddsampoff", .handler = cmd_ddsampoff },
	{ .cmd = "ddsampon", .handler = cmd_ddsampon },
	{ .cmd = "reboot", .handler = cmd_reboot },
	{ .cmd = "tone", .handler = cmd_tone },
	{ .cmd = "chan", .handler = cmd_chan },
	{ .cmd = "savechan", .handler = cmd_savechan }, 
	{ .cmd = "saveadc_b", .handler = cmd_saveadc_b },
	{ .cmd = "saveadc_c", .handler = cmd_saveadc_c },
	{ .cmd = "help", .handler = cmd_help },
	{ .cmd = NULL, .handler = NULL }, 

};

static void cmd_help(char **argv, uint8_t argc)
{
	dprintf(PSTR("\r\nHELP - List of commands for encoder/DDS\r\n"));
	for(uint8_t x = 0; commands[x].cmd != NULL; x++)
	{
		dprintf(PSTR("HELP: %s\r\n"), commands[x].cmd);
	}
}


#define MAX_PARAMS 8

static void process_commands(void *unused)
{
	static char *para[MAX_PARAMS];
	static char buf[BUF_DATA_SIZE+1];

	int result; 
	uint8_t parc;

	while( (result = rb_linebuf_get(&uart_rx_buf, buf, sizeof(buf)-1, false, false)) > 0)
	{	
		parc = rb_string_to_array(buf, para, MAX_PARAMS);
		dprintf(PSTR("command with parc: %u\r\n"), parc);
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
				if(strcasecmp(commands[i].cmd, para[0]) == 0)
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
			return;
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
	*channel = adc_avg();
	*channel = adc_avg();

	ADMUX = ADMUX_ADC12;
	ADCSRB = ADCSRB_ADC12;
	ADCSRA |= _BV(ADSC); 
	_delay_ms(30);
	*band = adc_avg(); /* throw away the first sample */
	*band = adc_avg();
	*band = adc_avg(); /* throw away the first sample */
	*band = adc_avg();
	_delay_us(30);
}


static bool lookup_channel(uint16_t adc_val, uint16_t *channel)
{
	uint16_t channel_range[CHAN_MAX][2];
	uint8_t i; 

	eeprom_read_block(&channel_range, &channel_switch_range, sizeof(channel_range));
	for(i = 0; i < CHAN_MAX; i++)
	{
		if(adc_val >= channel_range[i][0] && adc_val <= channel_range[i][1])
		{
			*channel = i;
			return true;
		}
		
	}
	return false;
}

static bool lookup_band(uint16_t adc_val, uint16_t *band)
{
	uint16_t band_range[8][2];
	uint8_t i;
	
	eeprom_read_block(&band_range, &band_switch_range, sizeof(band_range));
	for(i = 0; i < BAND_MAX; i++)
	{
		if(adc_val >= band_range[i][0] && adc_val <= band_range[i][1])
		{
			*band = i;
			return true;
		}
		
	}
	return false;
}


static void set_channel(uint16_t band, uint16_t channel)
{
	struct memory_entry m;
	
	if(band > BAND_MAX - 1 || channel > CHAN_MAX - 1)
	{
		dprintf(PSTR("\r\nset_channel: band or channel out of range\r\n"));
		return;
	}
	
	if(channel == CHAN_VFO)
	{
		dprintf(PSTR("set_channel: vfo selected, turning DDS amp off\r\n"));
		dds_amp_off();
		return;
	}

	eeprom_read_block(&m, &bands[band][channel], sizeof(m));
	dprintf(PSTR("\r\nset_channel: band: %u channel: %u - %u %u - ctcss frequency: %u\r\n"),  band, channel, m.freq_msb, m.freq_lsb, m.tone_mult);

	if(m.tone_mult == 65535)
		m.tone_mult = 0;

	cur_mult = m.tone_mult;	
	
	if(m.freq_msb == 65535 || m.freq_msb == 0 || m.freq_lsb == 65535 || m.freq_msb == 0)
	{
		dds_amp_off();
		ad9833_shutdown();
		return;
	} 
	
	ad9833_setvfo(m.freq_msb, m.freq_lsb);
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
		dprintf(PSTR("ADC out of range for channel: %u\r\n"), c);
#if 0
		ad9833_shutdown();
		cur_mult = 0;
		return;
#endif
	}

	if(lookup_band(b, &new_band) == false)
	{
		dprintf(PSTR("ADC out of range for band: %u\r\n"), b);
//		return;
	}

	if((current_channel == new_channel) && (current_band == new_band))
	{
		dprintf(PSTR("NO changes to make\r\n"));
//		return;
	}
	current_band = new_band;
	current_channel = new_channel;
	set_channel(new_band, new_channel);

}


static void setup_linebuf(void)
{
	rb_linebuf_newbuf(&uart_rx_buf);
	rb_linebuf_newbuf(&uart_tx_buf);
}


static uint16_t adc_avg(void);


static void blink_cb(void *data)
{
	dprintf(PSTR("\r\n* - FAKE BLINK\r\n"));
	led_toggle();
}

static void blink1_cb(void *data)
{
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

	dprintf(PSTR("\r\nFT-221 DDS/CTCSS encoder thing\r\n(C) 2022 Aaron Sethman / N3RYB <androsyn@ratbox.org>\r\n"));
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

	dprintf(PSTR("Enabling interrupts;\r\n"));
  	sei();
  	dprintf(PSTR("Enabled interrupts successfully\r\n"));


 	ad9833_init();

 	adc_init();
 	dprintf(PSTR("Out of adc_init()\r\n"));
 	
	read_channel_ev = rb_event_add(read_channel, NULL, 5000, 0);
	rb_event_add(process_uart, NULL, 50, 0);
	rb_event_add(process_commands, NULL, 20, 0);
	
	event_blink(4);

	rb_event_add(blink1_cb, NULL, 2000, 0);

	dprintf(PSTR("setup finished\r\n"));	
}

int main(void)
{
	setup();
	while(1)
	{
		rb_event_run();
	}
}
