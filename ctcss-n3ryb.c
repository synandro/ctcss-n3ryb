/*
 *  Copyright 2022 Aaron Sethman <androsyn@ratbox.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
 *  USA
 */



#define ADC_MAX  1024 /* adc values above this are discarded as invalid */

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



#include "tools.h"
#include "ad9833.h"
#include "event.h"
#include "pwm-sine.h"
#include "memories.h"

buf_head_t uart_rx_buf;
/* buf_head_t uart_tx_buf; */

uint16_t current_band;
uint16_t current_channel;
uint32_t scan_rate EEMEM = 50;
uint32_t dwell_time EEMEM = 6000;
uint32_t dtime = 6000; /* so we don't need to keep loading dwell_time from eeprom */
uint32_t srate = 50;
uint32_t scan_count = 0;

volatile bool is_split = false;
volatile bool is_squelched = false;
volatile bool is_txing = false;


struct ev_entry *read_channel_ev; 
struct ev_entry *scan_channel_ev = NULL;

static void read_channel(void);
static void set_channel(uint16_t band, uint16_t channel, bool report);
static void do_scan(void);
static void stop_scan(void);
static void start_scan(void);
static void hold_scan(void);
extern struct memory_entry bands[BAND_MAX][CHAN_MAX] EEMEM;





/* min/max range for each band switch */
/* yes, they are out of order on purpose. i wired them wrong!  */
/*
CMD_LISTADC: band = 0, .min = 260, .max = 280 
CMD_LISTADC: band = 1, .min = 200, .max = 230 
CMD_LISTADC: band = 2, .min = 335, .max = 350 
CMD_LISTADC: band = 3, .min = 300, .max = 320 
CMD_LISTADC: band = 4, .min = 520, .max = 540 
CMD_LISTADC: band = 5, .min = 415, .max = 450 
CMD_LISTADC: band = 6, .min = 680, .max = 700 
CMD_LISTADC: band = 7, .min = 598, .max = 620 
CMD_LISTADC: channel = 0, .min = 0, .max = 0 
CMD_LISTADC: channel = 1, .min = 970, .max = 1024 
CMD_LISTADC: channel = 2, .min = 140, .max = 160 
CMD_LISTADC: channel = 3, .min = 120, .max = 131 
CMD_LISTADC: channel = 4, .min = 170, .max = 200 
CMD_LISTADC: channel = 5, .min = 200, .max = 215 
CMD_LISTADC: channel = 6, .min = 245, .max = 260 
CMD_LISTADC: channel = 7, .min = 280, .max = 310 
CMD_LISTADC: channel = 8, .min = 330, .max = 345 
CMD_LISTADC: channel = 9, .min = 840, .max = 890 
CMD_LISTADC: channel = 10, .min = 359, .max = 375 
CMD_LISTADC: channel = 11, .min = 670, .max = 700 
*/

#ifdef ADC_IN_EEPROM
#define ADC_MEM EEMEM
#else 
#define ADC_MEM 
#endif


static const uint16_t band_switch_range[BAND_MAX][2] ADC_MEM  = {
	{ 260, 280 },
	{ 200, 230 },
	{ 335, 350 },
	{ 300, 320 },
	{ 520, 540 },
	{ 415, 450 },
	{ 680, 700 },
	{ 598, 620 }
};

/* min/max range for each band switch the +2 is for the VFO and the scan function*/
static const uint16_t channel_switch_range[CHAN_SWITCH_MAX][2] ADC_MEM = {
	{ 0, 30 },
	{ 970, 1024 },
	{ 140, 160 },
	{ 120, 131 },
	{ 170, 200 },
	{ 200, 215 },
	{ 245, 260 },
	{ 280, 310 },
	{ 330, 345 },
	{ 840, 890 },
	{ 359, 375 },
	{ 670, 700 }
};

typedef void CMDCB(char **parv, uint8_t parc);

struct command_struct 
{
	const char *cmd;
	CMDCB *handler;
	bool iscat; /* a cat command, only match the first two characters */
};




static uint32_t atoul(const char *num)
{
	uint32_t val = 0;
	while (*num && (*num >= '0' && *num <= '9'))
	{
		val = (val * 10) + (uint8_t)(*num - '0');
	    	num++;
	}
	return val;
}

static uint8_t atous(const char *num)
{
	uint8_t val = 0;
	while (*num && (*num >= '0' && *num <= '9'))
	{
	    	val = (val * 10) + (uint8_t)(*num - '0');
	    	num++;
	}
	return val;
}
#if 0
static uint16_t atoui(const char *num)
{
	uint16_t val = 0;
	while (*num && (*num >= '0' && *num <= '9'))
	{
	    	val = (val * 10) + (uint8_t)(*num - '0');
	    	num++;
	}
	return val;
}
#endif

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

static void cmd_uptime(char **argv, uint8_t argc)
{
	dprintf(PSTR("\r\nCMD_UPTIME: %lu milliseconds\r\n"), current_ts());
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

#if 0
int uart_getchar(FILE *stream) 
{
    loop_until_bit_is_set(UCSR1A, RXC1); /* Wait until data exists. */
    return UDR1;
}
#endif
volatile bool usart1_int;

ISR(USART1_RX_vect)
{
	usart1_int = true;
}




FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
// FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);

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
//	stdin  = &uart_input;
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

static void spi_init(void)
{
	/*  PB1 = SCLK, PB2 = MOSI, PB3 = MISO */
	DDRB |= _BV(PB1) | _BV(PB2);
	PORTB &= ~(_BV(PB1) | _BV(PB2)) ;
	SPCR  = _BV(SPE)| _BV(MSTR)| _BV(SPR0) | _BV(CPOL);
}


static inline void ad9833_sselect_high(void)
{
	PORTF |= _BV(PF7);
	wdt_reset();
	_delay_us(40);
}

static inline void ad9833_sselect_low(void)
{
	PORTF &= ~_BV(PF7);
	wdt_reset();
	_delay_us(10);
}


static bool dds_power;


static inline void __attribute__((always_inline)) dds_amp_on(void)
{
	DDRF |= _BV(PF4);
	PORTF |= _BV(PF4); 
}

static inline void __attribute__((always_inline)) dds_amp_off(void)
{
	PORTF &= ~_BV(PF4);
}

static inline void __attribute__((always_inline)) ad9833_off(void)
{
	PORTF &= ~_BV(PF5); 
	dds_power = false;
}

static inline void __attribute__((always_inline)) ad9833_on(void)
{
	DDRF |= _BV(PF5);
	PORTF |= _BV(PF5);
	dds_power = true;
}



static void ad9833_init(void)
{
	spi_init();

	DDRF |= _BV(PF7); /* slave select */
	ad9833_sselect_high();

	ad9833_on();

	wdt_reset();
	_delay_ms(10);

	ad9833_sselect_low();
	SPI_write16(AD9833_RESET);
	ad9833_sselect_high();
	dds_amp_on();
}


static void ad9833_shutdown(void)
{
	dds_amp_off();
	ad9833_sselect_low();
	SPI_write16(AD9833_RESET);
	ad9833_sselect_high();
	
	ad9833_off();
	ad9833_sselect_low();
	dds_power = 0;
}


static void ad9833_setvfo(uint16_t freq_msb, uint16_t freq_lsb, uint16_t rev_msb, uint16_t rev_lsb)
{
	if(dds_power == false)
		ad9833_init();

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if(rev_msb > 0 || rev_lsb > 0)
			is_split = true;
		else
			is_split = false;
	} 

	ad9833_sselect_low();

	SPI_write16(AD9833_B28);
	SPI_write16(freq_lsb | AD9833_D14); /* D14 is freq0 */
	SPI_write16(freq_msb | AD9833_D14);
	
	if(is_split == false)
	{
		rev_lsb = freq_lsb;
		rev_msb = freq_msb;
	}
	SPI_write16(AD9833_B28);
	SPI_write16(rev_lsb | AD9833_D15);
	SPI_write16(rev_msb | AD9833_D15);

	ad9833_sselect_high();
	dds_amp_on();

}
// #define USE_DOUBLE 1
static void calc_freq(const char *f, uint16_t *freq_msb, uint16_t *freq_lsb)
{
	uint32_t freq_data;
#ifdef USE_DOUBLE
	double frequency;
	frequency = atof(f);

	freq_data = (uint32_t)((double)(frequency * pow(2, 28)) / (double)AD9833_FREQ);
#else
	uint32_t frequency;
	frequency = atoul(f);
	freq_data = (uint32_t)((uint64_t)frequency << 28) / AD9833_FREQ;
#endif
	*freq_msb = (freq_data >> 14);
	*freq_lsb = (freq_data & 0x3FFF);
	return;		
}


static void __attribute__((noinline)) set_freq(const char *f) 
{
	uint16_t freq_msb, freq_lsb;

	calc_freq(f, &freq_msb, &freq_lsb);
	
	dprintf(PSTR("set_freq: freq:%s freq_msb:%u freq_lsb:%u\r\n"), f, freq_msb, freq_lsb);

	if(dds_power == false)
		ad9833_init();

	ad9833_sselect_low();

	SPI_write16(AD9833_B28);
	SPI_write16(freq_lsb | AD9833_D14); /* D14 is freq0 */
	SPI_write16(freq_msb | AD9833_D14);
	SPI_write16(AD9833_B28);
	SPI_write16(freq_lsb | AD9833_D15); /* D15 is freq1 */
	SPI_write16(freq_msb | AD9833_D15);

#if 0
	freq_data = ((uint64_t)(frequency + 1200) << 28) / AD9833_FREQ;
	freq_msb = (freq_data >> 14);
	freq_lsb = (freq_data & 0x3FFF);
	SPI_write16(freq_lsb | AD9833_D15); /* D15 is freq1 */
	SPI_write16(freq_msb | AD9833_D15);
#endif
	ad9833_sselect_high();
}


#if 0
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

#endif

#if 0
static void cmd_r(char **argv, uint8_t argc)
{
	SPI_write16(AD9833_FSELECT0);
}
#endif

static void cmd_setfreq(char **argv, uint8_t argc)
{
	uint32_t vfo_a_freq = atoul(argv[1]);
	if(vfo_a_freq > 14000000)
	{
		dprintf(PSTR("\r\nF: frequency too high\r\n"));
		return;
	}
	dprintf(PSTR("\r\nF: Setting VFO A to %s\r\n"), argv[1]);
	set_freq(argv[1]);

}

static void cmd_tone(char **argv, uint8_t argc)
{
	if(argc != 2)
	{
		dprintf(PSTR("\r\ncmd_tone: Not enough arguments:  TONE frequency\r\n"));
		return;
	}
	set_ctcss(atoul(argv[1]));
	dprintf(PSTR("\r\ncmd_tone: Set frequency to (%u / 8)Hz \r\n"), cur_mult);
}

static void adc_avg_channel(uint16_t *channel, uint16_t *band);

static void cmd_adc_cb(void)
{
	uint16_t channel, band;
	adc_avg_channel(&channel, &band);
	dprintf(PSTR("\rADC: channel:%u band:%u"), channel, band);
}

static void cmd_adc(char **argv, uint8_t argc)
{
	static struct ev_entry *adc_ev; 
	uint16_t channel, band;
	adc_avg_channel(&channel, &band);
	
	if(adc_ev == NULL)
	{
		dprintf(PSTR("\rCMD_ADC: starting ADC event\r\n"));
		adc_ev = rb_event_add(cmd_adc_cb, 1000, 0);
	} else {
		rb_event_delete(adc_ev);
		adc_ev = NULL;
	}
}

static void cmd_adcoff(char **argv, uint8_t argc)
{
	dprintf(PSTR("\r\nADC OFF\r\n"));
	rb_event_delete(read_channel_ev);
	read_channel_ev = NULL;
}

static void cmd_adcon(char **argv, uint8_t argc)
{
	dprintf(PSTR("\r\nADC ON\r\n"));
	read_channel_ev = rb_event_add(read_channel, 1500, 0);
}


static void cmd_ddsoff(char **argv, uint8_t argc)
{
	dprintf(PSTR("\r\ncmd_ddsoff: Shutting off ad9833\r\n"));
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
	set_channel(band, channel, true);
}

static void cmd_listadc(char **argv, uint8_t argc)
{
	uint16_t band, channel, range[2];

	dprintf(PSTR("\r\nCMD_LISTADC\r\n"));

	for(band = 0; band < BAND_MAX; band++)
	{
		  
		wdt_reset();
#ifdef ADC_IN_EEPROM
		eeprom_read_block(&range, &band_switch_range[band], sizeof(range));
#else
		range[0] = band_switch_range[band][0];
		range[1] = band_switch_range[band][1];
#endif
		dprintf(PSTR("CMD_LISTADC: band = %u, .min = %u, .max = %u \r\n"), band, range[0], range[1]);
	}

	for(channel = 0; channel < CHAN_SWITCH_MAX; channel++)
	{
		wdt_reset();
#ifdef ADC_IN_EEPROM
		eeprom_read_block(&range, &channel_switch_range[channel], sizeof(range));
#else
		range[0] = channel_switch_range[channel][0];
		range[1] = channel_switch_range[channel][1];
#endif
		dprintf(PSTR("CMD_LISTADC: channel = %u, .min = %u, .max = %u \r\n"), channel, range[0], range[1]);
	}
	

}
static void cmd_listchan(char **argv, uint8_t argc)
{
	uint16_t band, channel;
	struct memory_entry m;

	dprintf(PSTR("CMD_LISTCHAN\r\n"));
	for(band = 0; band < BAND_MAX; band++)
	{
		for(channel = 0; channel < CHAN_MAX; channel++)
		{
			wdt_reset();
#ifdef MEMORY_IN_EEPROM
			eeprom_read_block(&m, &bands[band][channel], sizeof(m));
#else
			memcpy_P(&m, &bands[band][channel], sizeof(m));
#endif
			dprintf(PSTR("CMD_LISTCHAN: band = %u, channel = %u, .m.skip = %u, .ctcss_tone = %u, .freq_msb = %u, .freq_lsb = %u, .rev_msb = %u, .rev_lsb = %u, .desc = %s\r\n"), band, channel+1, m.skip, m.ctcss_tone, m.freq_msb, m.freq_lsb, m.rev_msb, m.rev_lsb, m.desc);
		}
	
	}
	wdt_reset();

}
#ifdef MEMORY_IN_EEPROM
static void cmd_savechan(char **argv, uint8_t argc)
{
	struct memory_entry m;
	uint8_t band, channel;

	/* savechan band channel outfreq infreq tone */
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
	
	/* savechan band channel freq1 freq2 tone */
	calc_freq(argv[3], &m.freq_msb, &m.freq_lsb);
	calc_freq(argv[4], &m.rev_msb, &m.rev_lsb);
	m.ctcss_tone = atoui(argv[5]);
	
	dprintf("SAVECHAN: Write: band: %u, channel: %u msb:%u lsb:%u tone:%u",band, channel, m.freq_msb, m.freq_lsb, m.ctcss_tone);
	wdt_reset();
	eeprom_write_block(&m, &bands[band][channel-1], sizeof(m));
	wdt_reset();
	eeprom_busy_wait();
	wdt_reset();
	eeprom_read_block(&m, &bands[band][channel-1], sizeof(m));
	dprintf("SAVECHAN: Read: band: %u, channel: %u msb:%u lsb:%u tone:%u",band, channel, m.freq_msb, m.freq_lsb, m.ctcss_tone);
}
#endif

#ifdef ADC_IN_EEPROM

/*  useful for adjusting the values of the ADC in development, move the final values to PROGMEM to save eeprom space */
static void cmd_saveadc_c(char **argv, uint8_t argc)
{
	uint8_t channel;
	uint16_t range[2];
	
	if(argc != 4)
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

	if(argc != 4)
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
#endif

static void cmd_reboot(char **argv, uint8_t argc)
{
	typedef void (*do_reboot_t)(void);
	const do_reboot_t do_reboot = (do_reboot_t)((FLASHEND - 511) >> 1);
	
	dprintf(PSTR("\r\n\r\nAttempting to reboot in 5 seconds\r\n"));
	ad9833_shutdown();
	cli();
	MCUSR = 0; 
	TCCR0A = 0;
	TCCR4A = 0;
	TCCR4B = 0;
	TCCR4C = 0;
	TCCR4D = 0; 
	_delay_ms(5000);
	do_reboot();
}

static void cmd_status(char **argv, uint8_t argc)
{
	dprintf(PSTR("\r\nUptime: %lu, current_band: %u current_channel: %u ctcss: %u sin_counter: %u\r\n"), current_ts(), current_band, current_channel, cur_mult, sin_counter);
	dprintf(PSTR("Scanning: %u scan rate: %lu, dwell: %lu scan_count: %lu\r\n"), scan_channel_ev != NULL ? 1 : 0,  eeprom_read_dword(&scan_rate), eeprom_read_dword(&dwell_time), scan_count); 
}

static void cmd_scanrate(char **argv, uint8_t argc)
{
	if(argc != 3)
	{
		wdt_reset();
		dprintf(PSTR("\r\nSCANRATE [scanrate] [dwell time] (in ms)\r\n"));
		srate = eeprom_read_dword(&scan_rate);
		wdt_reset();
		dtime = eeprom_read_dword(&dwell_time);
		dprintf(PSTR("Current rate is: %lu dwell time: %lu\r\n"), srate, dtime);
		return;
	}
	srate = atoul(argv[1]);
	dtime = atoul(argv[2]);
	dprintf(PSTR("\r\nSCANRATE: set scan rate to %lu - dwell time: %lu\r\n"), srate, dtime);
	wdt_reset();
	eeprom_write_dword(&scan_rate, srate);
	wdt_reset();
	eeprom_write_dword(&dwell_time, dtime);
	
} 

static void cmd_help(char **argv, uint8_t argc);

/* not sure this would be worth stuffing in PROGMEM for the complexity it would 
 * add to the command parser 
 */
static const struct command_struct commands[] = {
//	{ .cmd = "FA", .handler = cmd_vfo_a, .iscat = 1 },
	{ .cmd = "setfreq", .handler = cmd_setfreq   },
	{ .cmd = "ddsoff", .handler = cmd_ddsoff },
//	{ .cmd = "FB", .handler = cmd_vfo_b },
//	{ .cmd = "HI", .handler = cmd_hi },
//	{ .cmd = "T", .handler = cmd_t },
//	{ .cmd = "R", .handler = cmd_r },
	{ .cmd = "adc", .handler = cmd_adc },
	{ .cmd = "adcoff", .handler = cmd_adcoff },
	{ .cmd = "adcon", .handler = cmd_adcon },
	{ .cmd = "ddsampoff", .handler = cmd_ddsampoff },
	{ .cmd = "ddsampon", .handler = cmd_ddsampon },
	{ .cmd = "reboot", .handler = cmd_reboot },
	{ .cmd = "tone", .handler = cmd_tone },
	{ .cmd = "chan", .handler = cmd_chan },
#if MEMORIES_IN_EEPROM
	{ .cmd = "savechan", .handler = cmd_savechan }, 
#endif
#if ADC_IN_EEPROM
	{ .cmd = "saveadc_b", .handler = cmd_saveadc_b },
	{ .cmd = "saveadc_c", .handler = cmd_saveadc_c }, 
#endif
	{ .cmd = "listadc", .handler = cmd_listadc }, 
	{ .cmd = "listchan", .handler = cmd_listchan }, 
	{ .cmd = "help", .handler = cmd_help },
	{ .cmd = "uptime", .handler = cmd_uptime },
	{ .cmd = "status", .handler = cmd_status },
	{ .cmd = "scanrate", .handler = cmd_scanrate }, 
	{ .cmd = NULL, .handler = NULL }, 

};

static void cmd_help(char **argv, uint8_t argc)
{
	dprintf(PSTR("\r\nHELP - List of commands for encoder/DDS\r\n"));
	for(uint8_t x = 0; commands[x].cmd != NULL; x++)
	{
		wdt_reset();
		dprintf(PSTR("HELP: %s\r\n"), commands[x].cmd);
	}
}


#define MAX_PARAMS 8

static void process_commands(void)
{
	static char *para[MAX_PARAMS];
	static char buf[BUF_DATA_SIZE+1];

	uint8_t parc;
	uint8_t count = 0;

	while(++count < 128 && ((rb_linebuf_get(&uart_rx_buf, buf, BUF_DATA_SIZE, false, false)) > 0))
	{	
		wdt_reset();
		parc = rb_string_to_array(buf, para, MAX_PARAMS);
		if(parc == 0)
		{
			continue;
		}
		for(uint8_t i = 0; commands[i].cmd != NULL; i++)
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

static void process_uart(void)
{
	static char buf[BUF_DATA_SIZE];
	char *p;
	uint16_t count = 0;

	wdt_reset();
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if(usart1_int == false && !bit_is_set(UCSR1A, RXC1))
			return;
		usart1_int = false;
	}

	p = buf;
	
	while((++count < 32) && bit_is_set(UCSR1A, RXC1))
	{
		wdt_reset();
		
		*p++ = UDR1;
		if((p - buf) == BUF_DATA_SIZE - 1)
		{
			if(rb_linebuf_parse(&uart_rx_buf, buf, p - buf, true) == 0)
			{
				process_commands(); /* process the queue */
			}
			p = buf;
		}
	}

	if(p >= buf) {
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

// #define ADMUX_ADC11  = _BV(MUX0) | _BV(MUX1) |_BV(REFS0);

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

	ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2);
	_delay_us(125);

}

static uint16_t adc_avg(void)
{
	const uint8_t adc_samples = 16; /* keep this powers of 2 otherwise the division is no longer a bit shift */
	uint16_t adc_val = 0;
	
	for(uint8_t i = 0; i < adc_samples; i++)
	{
		ADCSRA |= _BV(ADSC);
		wdt_reset();
		while(!bit_is_set(ADCSRA,ADSC));
		adc_val += ADCW;
	}
	return (adc_val / adc_samples);
}


static void adc_avg_channel(uint16_t *channel, uint16_t *band)
{
	ADMUX = ADMUX_ADC11;
	ADCSRB = ADCSRB_ADC11;
	
	wdt_reset();
	_delay_us(250);
	wdt_reset();	
	*band = adc_avg();
	*band = adc_avg();
	*band = adc_avg();

	ADMUX = ADMUX_ADC12;
	ADCSRB = ADCSRB_ADC12;
	ADCSRA |= _BV(ADSC) | _BV(ADEN);
	wdt_reset();
	_delay_us(250);
	wdt_reset();
	*channel = adc_avg();
	*channel = adc_avg();
	*channel = adc_avg();
	ADMUX = ADMUX_ADC11;
	ADCSRB = ADCSRB_ADC11;
}



static bool lookup_channel(uint16_t adc_val, uint16_t *channel)
{

//	memcpy_P(&channel_range, &channel_switch_range, sizeof(channel_range));
#ifdef ADC_IN_EEPROM
	static uint16_t channel_range[CHAN_SWITCH_MAX][2];
	eeprom_read_block(&channel_range, &channel_switch_range, sizeof(channel_range));
#endif

	for(uint8_t i = 0; i < CHAN_SWITCH_MAX; i++)
	{
		uint16_t c_min, c_max;
#ifdef ADC_IN_EEPROM		
		c_min = channel_range[i][0];
		c_max =  channel_range[i][1];
#else
		c_min = channel_switch_range[i][0];
		c_max = channel_switch_range[i][1];
#endif
		
		if((c_min > ADC_MAX) || (c_max > ADC_MAX))
			continue;
		
		if((adc_val >= c_min) && (adc_val <= c_max))
		{
			*channel = i;
			return true;
		}
		
	}
	return false;
}

static bool lookup_band(uint16_t adc_val, uint16_t *band)
{
#ifdef ADC_IN_EEPROM
	static uint16_t band_range[BAND_MAX][2];
//	memcpy_P(&band_range, &band_switch_range, sizeof(band_range));	
//	eeprom_read_block(&band_range, &band_switch_range, sizeof(band_range));
#endif
	for(uint8_t i = 0; i < BAND_MAX; i++)
	{
		uint16_t b_min, b_max;
#ifdef ADC_IN_EEPROM
		b_min =  band_range[i][0];
		b_max = band_range[i][1];
#else
		b_min = band_switch_range[i][0];
		b_max = band_switch_range[i][1];
#endif

		if((adc_val >= b_min) && adc_val <= b_max)
		{
			*band = i;
			return true;
		}
		
	}
	return false;
}


static void set_channel(uint16_t band, uint16_t channel, bool report)
{
	static struct memory_entry m;
		
	if(band > BAND_MAX - 1 || channel > CHAN_MAX)
	{
		dprintf(PSTR("set_channel: band or channel out of range\r\n"));
		return;
	}
	
	if(channel == CHAN_VFO)
	{
		dprintf(PSTR("set_channel: vfo selected, turning DDS amp off\r\n"));
		ad9833_shutdown();
		return;
	}
#ifdef MEMORIES_IN_EEPROM
	eeprom_read_block(&m, &bands[band][channel-1], sizeof(m));
#else
	memcpy_P(&m, &bands[band][channel-1], sizeof(m));
#endif
	wdt_reset();
	
#if 0	
	if(m.freq_lsb == UINT16_MAX)
		m.freq_lsb = 0;
	
	if(m.freq_msb == UINT16_MAX)
		m.freq_msb = 0;
	
	if(m.ctcss_tone == UINT8_MAX)
		m.ctcss_tone = 0;
#endif
	if(report)
	{
		dprintf(PSTR("set_channel: %s band: %u channel: %u - %u %u - ctcss frequency: %u\r\n"),  m.desc, band, channel, m.freq_msb, m.freq_lsb, m.ctcss_tone);
	}

	if(m.freq_msb == 0 && m.freq_lsb == 0)
	{
		if(report)
			dprintf(PSTR("set_channel: frequency is zero, turning DDS off and turning off CTCSS tone\r\n"));
		cur_mult = 0;
		ad9833_shutdown();
		return;
	} 

	if(m.ctcss_tone >= CTCSS_LAST)
	{
		cur_mult = 0;
	} else
		cur_mult = pgm_read_word(&ctcss_tone_table[m.ctcss_tone]);
	

	ad9833_setvfo(m.freq_msb, m.freq_lsb, m.rev_msb, m.rev_lsb);
}

static void read_channel(void)
{
	static bool pending_band_change = false;
	static bool pending_channel_change = true;
	static uint16_t pending_channel = 0, pending_band = 0;
	bool pending_change_now = false;
	uint16_t c, b;
	uint16_t new_channel = 0, new_band = 0;
	
	wdt_reset();

	if(is_txing == true)
		return;

	adc_avg_channel(&c, &b);			

	/* noise or out of range */
	if(rb_unlikely(lookup_band(b, &new_band) == false))
		return;
	
	if(rb_unlikely(lookup_channel(c, &new_channel) == false))
		return;

	if((current_band == new_band) && (current_channel == new_channel))
	{
		/* band/channel matches resets the pending state */
		pending_band_change = false;
		pending_channel_change = false;
		return;
	}

	if((pending_band_change == false) && (current_band != new_band))
	{
		pending_band_change = true;
		pending_change_now = true;
		pending_band = new_band;	
	}
	
	if((pending_channel_change == false) && (current_channel != new_channel))
	{
		pending_channel_change = true;
		pending_channel = new_channel;
		pending_change_now = true;
	}

	if(pending_change_now == true)
		return;

	if(pending_band_change == true)
	{
		if(new_band != pending_band)
		{
			pending_band_change = true;
			pending_band = new_band;
		} else {
			pending_band_change = false;
			current_band = new_band;
		}
	}

	if(pending_channel_change == true)
	{
		if(new_channel != pending_channel)
		{
			pending_channel_change = true;
			pending_channel = new_channel;
		} else {
			pending_band_change = false;
			current_channel = new_channel;
		}
	}

	wdt_reset();
	
	switch(current_channel)
	{
		case CHAN_SCAN:
			dprintf(PSTR("read_channel: starting channel scan\r\n"));
			start_scan();
			return;
		case CHAN_HOLD:
			dprintf(PSTR("read_channel: hold channel scan\r\n"));
			hold_scan();
			return;
		default:
			stop_scan();
	} 
	
	if(pending_channel_change == false || pending_band_change == false)
	{
		set_channel(current_band, current_channel, true);
	}
}


static void setup_linebuf(void)
{
	rb_linebuf_newbuf(&uart_rx_buf);
//	rb_linebuf_newbuf(&uart_tx_buf);
}


static uint16_t adc_avg(void);


static void setup_squelch(void)
{
	DDRD &= ~_BV(PD0);
	PORTD &= ~_BV(PD0);
	
	EIMSK &= ~_BV(INT0);
	EICRA |= _BV(ISC00);
	EIMSK |= _BV(INT0); 
	if(bit_is_set(PIND, PD0))
		is_squelched = false;
	else
		is_squelched = true;
}


static uint16_t last_channel;
static uint16_t last_band;
static bool should_hold_scan;
static volatile uint16_t last_channel_int;
static volatile uint16_t last_band_int;



ISR(INT0_vect)
{
	if(bit_is_set(PIND, PD0))
	{
		is_squelched = false;
		last_channel_int = last_channel;
		last_band_int = last_band;
	}
	else
		is_squelched = true;
}


static void stop_scan(void)
{
	should_hold_scan = false;
	if(scan_channel_ev == NULL)
		return;
	dprintf(PSTR("Stopping scan\r\n"));
	rb_event_delete(scan_channel_ev);
	scan_channel_ev = NULL;	
	led_on();
}

static void hold_scan(void)
{
	should_hold_scan = true;
}


static void start_scan(void)
{
	should_hold_scan = false;
	if(scan_channel_ev == NULL)
	{
		dprintf(PSTR("start_scan: rate: %lu dwell time: %lu\r\n"), srate, dtime);
		scan_channel_ev = rb_event_add(do_scan, srate, 0);
	}
}




static void do_scan(void)
{
	static uint32_t hangtime;
	static uint32_t last_jump;
	static bool has_notified = false;
	static struct memory_entry m;
	static struct memory_entry lm;
	uint16_t channel, band;
	uint32_t ts;
	static uint32_t last_scan_blip;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		ts = tick; 
		if(is_squelched == false)
			hangtime = ts;

		if(is_txing == true)
		{
			return;
		}
	}
	if(should_hold_scan == true)
	{
		if(last_scan_blip + 2000 < tick)
		{
			led_toggle();
			last_scan_blip = tick;
			memcpy_P(&lm, &bands[last_band][last_channel-1], sizeof(lm));
			dprintf(PSTR("Hold on channel: %s %u %u\r\n"),  lm.desc, last_band, last_channel);

		}
		return;
	}
	if(is_squelched == false)
	{
		led_on();
		if(has_notified == false)
		{
			if(last_channel_int >  0)
			{
				memcpy_P(&lm, &bands[last_band_int][last_channel_int-1], sizeof(lm));
				cur_mult = pgm_read_word(&ctcss_tone_table[lm.ctcss_tone]);
				dprintf(PSTR("Stopped on channel: %s %u %u\r\n"),  lm.desc, last_band_int, last_channel_int);
			}
			has_notified = true;
		}
		if((last_jump + dtime) > ts)
			return;
		hangtime = 0;
	} 
	
	if(hangtime + 2000 > ts)
	{
		led_on();
		return;
	}
	has_notified = false;
	
	led_toggle();
	if(last_band != current_band)
	{
		channel = 1;
	}

	band = current_band;	

	if(last_channel > CHAN_MAX) 
		channel = 1;
	else
		channel = last_channel + 1;

	last_jump = ts;

	scan_count++;
	for(uint16_t i = channel; i < CHAN_MAX - 1; i++)
	{
#ifdef MEMORIES_IN_EEPROM
		eeprom_read_block(&m, &bands[band][i-1], sizeof(m));
#else
		memcpy_P(&m, &bands[band][i-1], sizeof(m));
#endif		
		if(m.skip == true)
			continue;

		if(m.freq_msb == 0 && m.freq_lsb == 0)
			continue;

		last_channel = channel;
		last_band = band;
//		memcpy_P(&lm, &bands[band][i], sizeof(m));
		set_channel(band, i, false);
		return;	
	}
	channel = 1;
	last_channel = 1;
	set_channel(band, channel, false);
}

static void setup_ptt(void)
{
	DDRE &= ~_BV(PE6);
	PORTE &= ~_BV(PE6);

	EIMSK &= ~_BV(INT6);  
	EICRB |= _BV(ISC60);
	EIMSK |= _BV(INT6);
}

ISR(INT6_vect)
{
	bool was_txing = is_txing;

	if(bit_is_clear(PINE, PE6))
		is_txing = false;
	else
		is_txing = true;

	if(is_split == false)
		return;

	if(is_txing == false && was_txing == true)
	{
		dprintf(PSTR("Switching to FREQ0 - RX\r\n"));
		ad9833_sselect_low();
		SPI_write16(AD9833_FSELECT0);
		ad9833_sselect_high();
		return;
	}

	if(was_txing == false && is_txing == true)
	{
		dprintf(PSTR("Switching to FREQ1 - TX\r\n"));
		ad9833_sselect_low();
		SPI_write16(AD9833_FSELECT1);
		ad9833_sselect_high();
		return;
	}
}

static void setup() 
{
	MCUSR &= ~_BV(WDRF); 
	MCUCR |= _BV(PUD);

	wdt_enable(WDTO_2S);
	wdt_reset();
	
	DDRD |= _BV(PD1);
	DDRF |= _BV(PF4) | _BV(PF5); // DDS amp PF4 - AD9833 PF5
	led_on();
	
	cli();
	
	rb_event_init();
	setup_linebuf();	
	uart_init();

	dprintf(PSTR("\r\nFT-221 DDS/CTCSS encoder thing\r\n(C) 2022 Aaron Sethman / N3RYB <androsyn@ratbox.org>\r\n"));

	PLLCSR = _BV(PINDIV) | _BV(PLLE);
	PLLFRQ = _BV(PLLTM1) | _BV(PDIV3) | _BV(PDIV1); /* run at 96MHz  - see datasheet section 6.11.5 */

	/* shut off the USB controller before we enable interrupts again - we'll go splat otherwise */
	USBCON = 0;

	while((PLLCSR & _BV(PLOCK)) == 0);

	PRR0 |= _BV(PRTWI); /* turn off two wire */
	PRR1 |= _BV(PRUSB); /* and usb too */
	
	spi_init();
	setup_ptt();
	setup_squelch();	
	dprintf(PSTR("Enabling interrupts;\r\n"));
  	sei();
  	dprintf(PSTR("Enabled interrupts successfully\r\n"));
	setup_pwm();


 	ad9833_init();

 	adc_init();
	
	/* load the scan values */
	dtime = eeprom_read_dword(&dwell_time);
	srate = eeprom_read_dword(&scan_rate);

	read_channel_ev = rb_event_add(read_channel, 200, 0);
	rb_event_add(process_uart, 50, 0);
	rb_event_add(process_commands, 20, 0);
	led_on();
}

int main(void)
{
	setup();
	dprintf(PSTR("starting event loop\r\n"));
	while(1)
	{
		wdt_reset();
		rb_event_run();
		wdt_reset();
	}
}
