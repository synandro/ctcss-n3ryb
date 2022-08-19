#define F_CPU 16000000UL
#define BAUD 115200
#define BAUD_TOL 3
#define REF_FREQ             25000000



#include <stdint.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
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
#include "ad9833.h"

buf_head_t uart_rx_buf;
buf_head_t uart_tx_buf;
uint32_t vfo_a_freq;

static uint16_t cur_mult;
static uint16_t counter;
static uint8_t cur_freq;




/* the different states we can be in.  start in idle mode  */
enum {
	MODE_IDLE,
	MODE_HALFWAY,
	MODE_PROGRAM,
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

static uint8_t EEMEM saved_frequency = HZ_123;


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
#if 1
/* So we are using a precomputed frequency table for the LSB and MSB half of the frequency
 * this all assumes that the clock on the ad9833 is running at 25MHz
 * 
        uint32_t freq_data = ((uint64_t)frequency << 28) / REF_FREQ;
        uint16_t freq_MSB = (freq_data >> 14);  //| FREQ0;
        uint16_t freq_LSB = (freq_data & 0x3FFF); //| FREQ0;

 */
struct memory_entry {
	const uint16_t freq_msb;
	const uint16_t freq_lsb;
//	const uint16_t band;
//	const uint16_t channel;	
	const uint16_t tone_mult;
};

struct memory_entry EEMEM band_1[11] = {
	{ .tone_mult = MULT_107_2, .freq_msb = 5328, .freq_lsb = 1258 }, /* 145.13000 Haymarket  N3KL */
	{ .tone_mult = MULT_179_9, .freq_msb = 5341, .freq_lsb = 3014 }, /* 145.15000 Martinsburg  W8ORS */
	{ .tone_mult = MULT_118_8, .freq_msb = 5367, .freq_lsb = 6527 }, /* 145.19000 Moorefield  N8VAA */
	{ .tone_mult = MULT_141_3, .freq_msb = 5380, .freq_lsb = 8283 }, /* 145.21000 Front Royal High Knob Mountain K4QJZ */
	{ .tone_mult = MULT_123_0, .freq_msb = 5419, .freq_lsb = 13552 }, /* 145.27000 Meyersdale Hays Mill Fire Tower KQ3M */
	{ .tone_mult = MULT_146_2, .freq_msb = 5498, .freq_lsb = 7707 }, /* 145.39000 Winchester North Mountain KG4Y */
	{ .tone_mult = MULT_123_0, .freq_msb = 5537, .freq_lsb = 12976 }, /* 145.45000 Oldtown Warrior Mountain WMA W3YMW */
	{ .tone_mult = MULT_123_0, .freq_msb = 5564, .freq_lsb = 104 }, /* 145.49000 Bedford  K3NQT */
};

struct memory_entry EEMEM band_2[11] = {
	{ .tone_mult = MULT_131_8, .freq_msb = 5324, .freq_lsb = 13107 }, /* 146.62500 New Market Luray Caverns N4YSA */
	{ .tone_mult = MULT_123_0, .freq_msb = 5403, .freq_lsb = 7261 }, /* 146.74500 Berkeley Springs Cacapon Mountain KK3L */
	{ .tone_mult = MULT_123_0, .freq_msb = 5432, .freq_lsb = 15309 }, /* 146.79000 Clearville Martin Hill K3NQT */
	{ .tone_mult = MULT_123_0, .freq_msb = 5442, .freq_lsb = 12530 }, /* 146.80500 Oakland  KB8NUF */
	{ .tone_mult = MULT_146_2, .freq_msb = 5452, .freq_lsb = 9751 }, /* 146.82000 Winchester Great North Mountain W4RKC */
	{ .tone_mult = MULT_77_0, .freq_msb = 5472, .freq_lsb = 4194 }, /* 146.85000 Charles Town  WA4TXE */
	{ .tone_mult = MULT_123_0, .freq_msb = 5491, .freq_lsb = 15020 }, /* 146.88000 Midland Dan's Mountain W3YMW */
	{ .tone_mult = MULT_100_0, .freq_msb = 5531, .freq_lsb = 3905 }, /* 146.94000 Clear Spring Fairview Mountain W3CWC */
};

struct memory_entry EEMEM band_3[11] = {
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




#endif


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
	PORTB &= ~(1 << PB0);
}

static inline  __attribute__((always_inline)) void led_on(void)
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

static void setup_pwm(void)
{
	TIMSK0 = 0;

	OCR4B = 120;

	TCCR4A = _BV(WGM40) | _BV(WGM41) | _BV(COM4B0);
	TCCR4B = _BV(CS40);
	

//	TCCR0A = _BV(COM1B0) | _BV(WGM00) | _BV(WGM01);
	TCCR0A = _BV(COM0B0) | _BV(WGM00) | _BV(WGM01);
	TCCR0B = _BV(WGM02) | _BV(CS01);
	TIMSK0 = _BV(OCIE0A);		
	OCR0A = 243;

	//DDRB |= (1 << PB7); 
	//DDRB |= (1 << PB5); 
	DDRB |= (1 << PB6); 
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


static void inline ad9833_sselect_high(void)
{
        PORTF |= _BV(PF7);
        _delay_ms(10);
}

static void inline ad9833_sselect_low(void)
{
        PORTF &= ~_BV(PF7);
        _delay_us(10);
}


void spi_init(void)
{
	// PB1 = SCLK, PB2 = MOSI, PB3 = MISO
	
	DDRB |= _BV(PB1) | _BV(PB2);
	PORTB &= ~_BV(PB1);
	PORTB &= ~_BV(PB2);
	SPCR  = _BV(SPE)| _BV(MSTR)| _BV(SPR0) | _BV(CPOL);
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


#if 1

static void __attribute__((noinline)) setWave(uint32_t frequency) 
{
	uint32_t freq_data = ((uint64_t)frequency << 28) / REF_FREQ;
	uint16_t freq_MSB = (freq_data >> 14);  //| FREQ0;
	uint16_t freq_LSB = (freq_data & 0x3FFF); //| FREQ0;

	dprintf("Setting with freq_data: %lu\r\n", freq_data);

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

//	ad9833_sselect_high();
}
#endif


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
	vfo_a_freq = atoul(argv[1]);
	dprintf("Setting VFO A to %lu\r\n", vfo_a_freq);
	setWave(vfo_a_freq);

}

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
#if 0
static void cmd_vfo_b(const char *buf, size_t len)
{
	dprintf("Got cmd_vfo_a: %s %d\r\n", buf, len);


}

#endif

#if 0
#define AD_MUX_ADC0 0x0 // F0
#define AD_MUX_ADC1 (_BV(MUX0)) // F1
#define AD_MUX_ADC4 (_BV(MUX2))  // F4
#define AD_MUX_ADC5 (_BV(MUX0) | _BV(MUX2))  // F5
#define AD_MUX_ADC6 (_BV(MUX1) | _BV(MUX2)) // 20 // F6
#define AD_MUX_ADC7 (_BV(MUX0) | _BV(MUX1) | _BV(MUX2)) // 21 // F7

#define AD_DID_ADC0_PORT DIDR0
#define AD_DID_ADC1_PORT DIDR0
#define AD_DID_ADC4_PORT DIDR0
#define AD_DID_ADC5_PORT DIDR0
#define AD_DID_ADC6_PORT DIDR0
#define AD_DID_ADC7_PORT DIDR0


#define AD_MUX_ADC8 (_BV(MUX5)) // 22 // D4
#define AD_MUX_ADC9 (_BV(MUX5) | _BV(MUX0)) //100 // D6
#define AD_MUX_ADC10 (_BV(MUX5) | _BV(MUX1)) // 101 // D7
#define AD_MUX_ADC11 (_BV(MUX5) | _BV(MUX2) | _BV(MUX1)) // 102 // B4
#define AD_MUX_ADC12 (_BV(MUX5) | _BV(MUX2)) // 110 // B5
#define AD_MUX_ADC13 (_BV(MUX5) | _BV(MUX2) | _BV(MUX0)) // 111 // B6
#define AD_MUX_TEMP (_BV(MUX1) | _BV(MUX2) | _BV(MUX3))


#define AD_DID_ADC8_PORT DIDR1
#define AD_DID_ADC9_PORT DIDR1
#define AD_DID_ADC10_PORT DIDR1
#define AD_DID_ADC11_PORT DIDR1
#define AD_DID_ADC12_PORT DIDR1
#define AD_DID_ADC13_PORT DIDR1




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
			AD_DID_ADC0_PORT |= _BV(ADC0D);
			break;
		case 1:
			ADMUX = AD_MUX_ADC1;
			ADCSRB = AD_SRB_ADC1;
			AD_DID_ADC1_PORT |= _BV(ADC1D);
			break;
		case 4:
			ADMUX = AD_MUX_ADC4;
			ADCSRB = AD_SRB_ADC4;
			AD_DID_ADC4_PORT |= _BV(ADC4D);
			break;
		case 5:
			ADMUX = AD_MUX_ADC5;
			ADCSRB = AD_SRB_ADC5;
			AD_DID_ADC5_PORT |= _BV(ADC5D);
			
			break;
		case 6:
			ADMUX = AD_MUX_ADC6;
			ADCSRB = AD_SRB_ADC6;
			AD_DID_ADC6_PORT |= _BV(ADC6D);

			break;
		case 7:
			ADMUX = AD_MUX_ADC7;
			ADCSRB = AD_SRB_ADC7;
			AD_DID_ADC7_PORT |= _BV(ADC7D);

			break;
		case 8:
			ADMUX = AD_MUX_ADC8;
			ADCSRB = AD_SRB_ADC8;
			AD_DID_ADC8_PORT |= _BV(ADC8D);
			break;
			
		case 9:
			ADMUX = AD_MUX_ADC9;
			ADCSRB = AD_SRB_ADC9;
			AD_DID_ADC9_PORT |= _BV(ADC9D);

			break;
		case 10:
			ADMUX = AD_MUX_ADC10;
			ADCSRB = AD_SRB_ADC10;
			AD_DID_ADC10_PORT |= _BV(ADC10D);
			break;
		case 11:
			ADMUX = AD_MUX_ADC11;
			ADCSRB = AD_SRB_ADC11;
			AD_DID_ADC11_PORT |= _BV(ADC11D);
			
			break;
		case 12:
			ADMUX = AD_MUX_ADC12;
			ADCSRB = AD_SRB_ADC12;
			AD_DID_ADC12_PORT |= _BV(ADC12D);

			break;
		case 13:
			ADMUX = AD_MUX_ADC13;
			ADCSRB = AD_SRB_ADC13;
			AD_DID_ADC13_PORT |= _BV(ADC13D);
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
        ADCSRB= 0x00;                   //Configuring free running mode
        ADCSRA |= (1<<ADSC)|(1<<ADATE);   //Start ADC conversion and enabling Auto trigger


	ADCSRA |= _BV(ADEN); // | _BV(ADATE);	/* turn on the ADC and auto update */ 

//	ADMUX |= _BV(REFS1); 
	ADCSRA |= _BV(ADSC);			/* lets start a conversion */
	_delay_ms(10);
	
	for(uint8_t i = 0; i < 8; i++)
	{
		while(!bit_is_set(ADCSRA,ADSC));

        	uint16_t adc_val = ADCW;
        	
        	for(uint8_t i = 0; i < adc_samples; i++)
        	{
        		while(!bit_is_set(ADCSRA,ADSC));
        		adc_result += ADCW;
        	}
        	
        	dprintf("ADC: adc_val: %u average: %u\r\n", adc_val, adc_result / adc_samples);
        	adc_result = 0;
        	_delay_ms(100);
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
#endif


static void cmd_o(char **argv, uint8_t argc)
{
	dprintf("Shutting off ad9833\r\n");
	ad9833_shutdown();	

}

static void cmd_d(char **argv, uint8_t argc)
{
	dprintf("Shutting off dds amp\r\n");
	dds_amp_off();
}


static void cmd_reboot(char **argv, uint8_t argc)
{
        typedef void (*do_reboot_t)(void);
        const do_reboot_t do_reboot = (do_reboot_t)((FLASHEND - 511) >> 1);
        cli();
	MCUSR = 0; 
        TCCR0A = TCCR4A = 0; // make sure interrupts are off and timers are reset.
        do_reboot();
}

struct command_struct commands[] = {
	{ .cmd = "FA", .handler = cmd_vfo_a, .iscat = 1 },
	{ .cmd = "f", .handler = cmd_f   },
	{ .cmd = "o", .handler = cmd_o },
//	{ .cmd = "FB", .handler = cmd_vfo_b },
//	{ .cmd = "HI", .handler = cmd_hi },
	{ .cmd = "T", .handler = cmd_t },
	{ .cmd = "R", .handler = cmd_r },
//	{ .cmd = "ADC", .handler = cmd_adc, .iscat = 0 },
	{ .cmd = "d", .handler = cmd_d },
	{ .cmd = "reboot", .handler = cmd_reboot },
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
	ADCSRA = 0;
	
	DDRB &= ~_BV(PB4);
	PORTB &= ~_BV(PB4);
	
//	PORTB &= ~_BV(PB4);

//	ADCSRA |= (1<<ADEN);      //Enable ADC module
//	ADMUX= 0x01;            // configuring PB2 to take input
//	                        ADMUX = AD_MUX_ADC11;
//                        ADCSRB = AD_SRB_ADC11;

	//ADMUX = AD_MUX_ADC11;
//	ADCSRB = AD_SRB_ADC11;
//	ADMUX = _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	ADMUX = _BV(MUX0) | _BV(MUX1) |_BV(REFS0);
	ADCSRB = _BV(MUX5);
	DIDR2 |= _BV(ADC11D);
	ADCSRA |= _BV(ADSC) | _BV(ADATE) | _BV(ADEN);
}


// buf_head_t uart_tx_buf;

static void setup_linebuf(void)
{
	rb_linebuf_newbuf(&uart_rx_buf);
	rb_linebuf_newbuf(&uart_tx_buf);
}


static uint16_t adc_avg(void);



static void setup() 
{
	MCUSR = ~(1 << WDRF);
	wdt_disable();

	vfo_a_freq = 8300000;
	uint32_t loops = 0;
	
	cli();
#ifdef DEBUG
	uart_init();
#endif
	dprintf("Starting up and enabling PLL\r\n");
	PLLFRQ = _BV(PLLTM1) | _BV(PDIV3) | _BV(PDIV1) | _BV(PLLUSB); /* run at 96MHz */
	PLLCSR = _BV(PINDIV) | _BV(PLLE);

	/* shut off the USB controller before we enable interrupts again - we'll go splat otherwise */
	USBCON = 0;

	
	while((PLLCSR & (1<<PLOCK)) == 0)
	{
		dprintf("ctcss-n3ryb - waiting for PLL lock\r\n");	
	}

	dprintf("shutting off stuff\r\n");
	PRR0 |= _BV(PRTWI);
	PRR0 |= _BV(PRUSB);
	
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
//  	dprintf("Setting DDS frequency to: %lu cycles\r\n", vfo_a_freq);
 // 	setWave(SINE, vfo_a_freq);
 	ad9833_init();

 	dprintf("Starting adc_init()\r\n");
 	adc_init();
 	

  	while(1)
  	{
  		loops++;
  		
  		if(loops % 100)
  		{
	  		process_uart();
	  		process_commands();
		}
		if(loops % 16384 == 0)
			dprintf("ADC average: %u\r\n", adc_avg());

  	}
  	
#ifdef TESTING	
//	loop_all();
#endif

}

static uint16_t adc_avg(void)
{
	const uint8_t adc_samples = 16; /* keep this powers of 2 otherwise the division is no longer a bit shift */
	uint16_t adc_val = 0;

	for(uint8_t i = 0; i < adc_samples; i++)
	{
		while(!bit_is_set(ADCSRA,ADSC));
		adc_val += ADCW;
		_delay_ms(10);
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

#if 0
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
#endif

ISR(TIMER0_COMPA_vect) 
{
	OCR4B = sine_wave[((counter += cur_mult) >> 8)];
}

int main(void)
{
	setup();
//	loop();	
}

