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



#define CHAN_VFO 0
#define CHAN_SCAN 11
#define CHAN_MAX 12
#define BAND_MAX 8 
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

#define dprintf(...) printf_P(__VA_ARGS__)



#include "tools.h"
#include "ad9833.h"
#include "event.h"
#include "pwm-sine.h"


buf_head_t uart_rx_buf;
/* buf_head_t uart_tx_buf; */

uint16_t current_band;
uint16_t current_channel;
uint32_t scan_rate EEMEM = 500;
uint32_t dwell_time EEMEM = 2000;
uint32_t dtime = 2000; /* so we don't need to keep loading dwell_time from eeprom */

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

enum {
	 CTCSS_NONE,
	 CTCSS_67_0,
	 CTCSS_69_3,
	 CTCSS_71_9,
	 CTCSS_74_4,
	 CTCSS_77_0,
	 CTCSS_79_7,
	 CTCSS_82_5,
	 CTCSS_85_4,
	 CTCSS_88_5,
	 CTCSS_91_5,
	 CTCSS_94_8,
	 CTCSS_97_4,
	 CTCSS_100_0,
	 CTCSS_103_5,
	 CTCSS_107_2,
	 CTCSS_110_9,
	 CTCSS_114_8,
	 CTCSS_118_8,
	 CTCSS_123_0,
	 CTCSS_127_3,
	 CTCSS_131_8,
	 CTCSS_136_5,
	 CTCSS_141_3,
	 CTCSS_146_2,
	 CTCSS_151_4,
	 CTCSS_156_7,
	 CTCSS_162_2,
	 CTCSS_167_9,
	 CTCSS_173_8,
	 CTCSS_179_9,
	 CTCSS_186_2,
	 CTCSS_192_8,
	 CTCSS_203_5,
	 CTCSS_206_5,
	 CTCSS_210_7,
	 CTCSS_218_1,
	 CTCSS_225_7,
	 CTCSS_229_1,
	 CTCSS_233_6,
	 CTCSS_241_8,
	 CTCSS_250_3,
	 CTCSS_254_1,
};


const uint16_t ctcss_tone_table[] PROGMEM = {
	0,  /* HZ_NONE */
	536,  /* HZ_67_0 */
	554,  /* HZ_69_3 */
	575,  /* HZ_71_9 */
	595,  /* HZ_74_4 */
	616,  /* HZ_77_0 */
	637,  /* HZ_79_7 */
	660,  /* HZ_82_5 */
	683,  /* HZ_85_4 */
	708,  /* HZ_88_5 */
	732,  /* HZ_91_5 */
	758,  /* HZ_94_8 */
	779,  /* HZ_97_4 */
	800,  /* HZ_100_0 */
	830,  /* HZ_103_5 */
	857,  /* HZ_107_2 */
	887,  /* HZ_110_9 */
	918,  /* HZ_114_8 */
	953,  /* HZ_118_8 */
	984,  /* HZ_123_0 */
	1018,  /* HZ_127_3 */
	1054,  /* HZ_131_8 */
	1091,  /* HZ_136_5 */
	1130,  /* HZ_141_3 */
	1165,  /* HZ_146_2 */
	1211,  /* HZ_151_4 */
	1253,  /* HZ_156_7 */
	1297,  /* HZ_162_2 */
	1345,  /* HZ_167_9 */
	1390,  /* HZ_173_8 */
	1441,  /* HZ_179_9 */
	1489,  /* HZ_186_2 */
	1541,  /* HZ_192_8 */
	1627,  /* HZ_203_5 */
	1651,  /* HZ_206_5 */
	1685,  /* HZ_210_7 */
	1744,  /* HZ_218_1 */
	1805,  /* HZ_225_7 */
	1832,  /* HZ_229_1 */
	1868,  /* HZ_233_6 */
	1934,  /* HZ_241_8 */
	2001,  /* HZ_250_3 */
	2031,  /* HZ_254_1 */
};



struct memory_entry {
	uint16_t freq_msb;
	uint16_t freq_lsb;
	uint16_t rev_msb;
	uint16_t rev_lsb;
	uint8_t ctcss_tone;
	bool skip;
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

struct memory_entry bands[BAND_MAX][CHAN_MAX] EEMEM = {

	{
		/* band 0 - 144-144.49 */
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0 },
		{ .ctcss_tone = 0, .freq_msb = 5373, .freq_lsb = 15597, .rev_msb = 0, .rev_lsb = 0, .skip = 0 }, /* 144.200 */
		{ .ctcss_tone = 0, .freq_msb = 5498, .freq_lsb = 7707, .rev_msb = 0, .rev_lsb = 0, .skip = 0 }, /* 144.390 */
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
	},	
	{
		/* band 1 - 144.5 - 145.0 */
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
#if 1
//		{ .ctcss_tone = CTCSS_107_2, .freq_msb = 5655, .freq_lsb = 12399, .rev_msb = 5262, .rev_lsb = 8860, .skip = 0 }, /* 145.130 "Haymarket N3KL vfo: 136.500000 */
		{ .ctcss_tone = CTCSS_179_9, .freq_msb = 5668, .freq_lsb = 14155, .rev_msb = 5275, .rev_lsb = 10616, .skip = 0 }, /* 145.150 "Martinsburg W8ORS vfo: 136.500000 */
		{ .ctcss_tone = CTCSS_118_8, .freq_msb = 5695, .freq_lsb = 1284, .rev_msb = 5301, .rev_lsb = 14129, .skip = 0 }, /* 145.190 "Moorefield N8VAA vfo: 136.500000 */
		{ .ctcss_tone = CTCSS_141_3, .freq_msb = 5708, .freq_lsb = 3040, .rev_msb = 5314, .rev_lsb = 15885, .skip = 0 }, /* 145.210 "Front Royal High Knob Mountain" vfo: 136.500000 */
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5747, .freq_lsb = 8309, .rev_msb = 5354, .rev_lsb = 4771, .skip = 0 }, /* 145.270 "Meyersdale Hays Mill Fire Tower" vfo: 136.500000 */
		{ .ctcss_tone = CTCSS_146_2, .freq_msb = 5826, .freq_lsb = 2464, .rev_msb = 5432, .rev_lsb = 15309, .skip = 0 }, /* 145.390 "Winchester North Mountain KG4Y vfo: 136.500000 */
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5865, .freq_lsb = 7733, .rev_msb = 5472, .rev_lsb = 4194, .skip = 0 }, /* 145.450 "Oldtown Warrior Mountain WMA W3YMW" vfo: 136.500000 */
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5891, .freq_lsb = 11245, .rev_msb = 5498, .rev_lsb = 7707, .skip = 0 }, /* 145.490 "Bedford K3NQT vfo: 136.500000 */
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },

#endif
	},	
	{
		/* band 2 - 145.0 - 145.49 */ 	

		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
//		{ .ctcss_tone = CTCSS_107_2, .freq_msb = 5328, .freq_lsb = 1258, .rev_msb = 4934, .rev_lsb = 14103, .skip = 0 }, /* 145.130 "Haymarket N3KL vfo: 137.000000 */
		{ .ctcss_tone = CTCSS_179_9, .freq_msb = 5341, .freq_lsb = 3014, .rev_msb = 4947, .rev_lsb = 15859, .skip = 0 }, /* 145.150 "Martinsburg W8ORS vfo: 137.000000 */
		{ .ctcss_tone = CTCSS_118_8, .freq_msb = 5367, .freq_lsb = 6527, .rev_msb = 4974, .rev_lsb = 2988, .skip = 0 }, /* 145.190 "Moorefield N8VAA vfo: 137.000000 */
		{ .ctcss_tone = CTCSS_141_3, .freq_msb = 5380, .freq_lsb = 8283, .rev_msb = 4987, .rev_lsb = 4744, .skip = 0 }, /* 145.210 "Front Royal High Knob Mountain" vfo: 137.000000 */
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5419, .freq_lsb = 13552, .rev_msb = 5026, .rev_lsb = 10013, .skip = 0 }, /* 145.270 "Meyersdale Hays Mill Fire Tower" vfo: 137.000000 */
		{ .ctcss_tone = CTCSS_146_2, .freq_msb = 5498, .freq_lsb = 7707, .rev_msb = 5105, .rev_lsb = 4168, .skip = 0 }, /* 145.390 "Winchester North Mountain KG4Y vfo: 137.000000 */
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5537, .freq_lsb = 12976, .rev_msb = 5144, .rev_lsb = 9437, .skip = 0 }, /* 145.450 "Oldtown Warrior Mountain WMA W3YMW" vfo: 137.000000 */
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5564, .freq_lsb = 104, .rev_msb = 5170, .rev_lsb = 12949, .skip = 0 }, /* 145.490 "Bedford K3NQT vfo: 137.000000 */
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
//		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
	},
	{	
		/* band 3 - 145.5 - 145.99 */
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
	},
	{	
		/* band 4 - 146.0 - 146.6 */
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5731, .freq_lsb = 2018, .rev_msb = 5337, .rev_lsb = 14863, .skip = 0 }, /* 146.745 "Berkeley Springs Cacapon Mountain KK3L" */
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5760, .freq_lsb = 10066, .rev_msb = 5367, .rev_lsb = 6527, .skip = 0 }, /* 146.790 "Clearville Martin Hill K3NQT */
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5770, .freq_lsb = 7287, .rev_msb = 5377, .rev_lsb = 3748, .skip = 0 }, /* 146.805 "Oakland KB8NUF */
		{ .ctcss_tone = CTCSS_146_2, .freq_msb = 5780, .freq_lsb = 4508, .rev_msb = 5387, .rev_lsb = 969, .skip = 0 }, /* 146.820 "Winchester Great North Mountain W4RKC" */
		{ .ctcss_tone = CTCSS_77_0, .freq_msb = 5799, .freq_lsb = 15335, .rev_msb = 5406, .rev_lsb = 11796, .skip = 0 }, /* 146.850 "Charles Town WA4TXE */
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5819, .freq_lsb = 9777, .rev_msb = 5426, .rev_lsb = 6239, .skip = 0 }, /* 146.880 "Midland Dan's Mountain W3YMW */
		{ .ctcss_tone = CTCSS_100_0, .freq_msb = 5858, .freq_lsb = 15047, .rev_msb = 5465, .rev_lsb = 11508, .skip = 0 }, /* 146.940 "Clear Spring Fairview Mountain W3CWC" */
		{ .ctcss_tone = CTCSS_131_8, .freq_msb = 5652, .freq_lsb = 7864, .rev_msb = 5259, .rev_lsb = 4325, .skip = 1 }, /* 146.625 "New Market Luray Caverns N4YSA" */
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
	},

	{
		/* band 5 - 146.5 - 146.99 */
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0 , .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 5255, .freq_lsb = 16174, .rev_msb = 0, .rev_lsb = 0, .skip = 0 }, /* 146.520 simplex vfo: 138.500000 */
		{ .ctcss_tone = 0, .freq_msb = 5190, .freq_lsb = 7392, .rev_msb = 0, .rev_lsb = 0, .skip = 0 }, /* 146.420 simplex vfo: 138.500000 */
		{ .ctcss_tone = 0, .freq_msb = 5203, .freq_lsb = 9148, .rev_msb = 0, .rev_lsb = 0, .skip = 0 }, /* 146.440 simplex vfo: 138.500000 */
		{ .ctcss_tone = 0, .freq_msb = 5216, .freq_lsb = 10905, .rev_msb = 0, .rev_lsb = 0, .skip = 0 }, /* 146.460 simplex vfo: 138.500000 */
		{ .ctcss_tone = 0, .freq_msb = 5229, .freq_lsb = 12661, .rev_msb = 0, .rev_lsb = 0, .skip = 0 }, /* 146.480 simplex vfo: 138.500000 */
		{ .ctcss_tone = 0, .freq_msb = 5242, .freq_lsb = 14417, .rev_msb = 0, .rev_lsb = 0, .skip = 0 }, /* 146.500 simplex vfo: 138.500000 */
		{ .ctcss_tone = 0, .freq_msb = 5269, .freq_lsb = 1546, .rev_msb = 0, .rev_lsb = 0, .skip = 0 }, /* 146.540 simplex vfo: 138.500000 */
		{ .ctcss_tone = 0, .freq_msb = 5295, .freq_lsb = 5059, .rev_msb = 0, .rev_lsb = 0, .skip = 0 }, /* 146.580 simplex vfo: 138.500000 */
		{ .ctcss_tone = 0, .freq_msb = 5308, .freq_lsb = 6815, .rev_msb = 0, .rev_lsb = 0, .skip = 0 }, /* 146.600 simplex vfo: 138.500000 */
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0 , .skip = 0 },
	},
	{
		/* band 6 - 147.00 - 147.5*/
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .skip = 0 },
		{ .ctcss_tone = CTCSS_146_2, .freq_msb = 5439, .freq_lsb = 7995, .rev_msb = 0, .rev_lsb = 0, .skip = 0 }, /* 147.300 "Bluemont Blue Ridge WA4TSC */
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5409, .freq_lsb = 16331, .rev_msb = 0, .rev_lsb = 0, .skip = 0 }, /* 147.255 "Martinsburg WB8YZV */
		{ .ctcss_tone = CTCSS_103_5, .freq_msb = 5429, .freq_lsb = 10774, .rev_msb = 0, .rev_lsb = 0, .skip = 0 }, /* 147.285 "Circleville Spruce Knob N8HON */
		{ .ctcss_tone = CTCSS_131_8, .freq_msb = 5449, .freq_lsb = 5216, .rev_msb = 0, .rev_lsb = 0, .skip = 0 }, /* 147.315 "Basye Great North Mountain K4MRA" */
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5468, .freq_lsb = 16043, .rev_msb = 0, .rev_lsb = 0, .skip = 0 }, /* 147.345 "Clear Spring K3MAD */
		{ .ctcss_tone = CTCSS_127_3, .freq_msb = 5478, .freq_lsb = 13264, .rev_msb = 0, .rev_lsb = 0, .skip = 0 }, /* 147.360 "Skyline K7SOB */
		{ .ctcss_tone = CTCSS_100_0, .freq_msb = 5321, .freq_lsb = 8572, .rev_msb = 0, .rev_lsb = 0, .skip = 0 }, /* 147.120 "Chambersburg Clark's Knob W3ACH" */
		{ .ctcss_tone = CTCSS_167_9, .freq_msb = 5341, .freq_lsb = 3014, .rev_msb = 0, .rev_lsb = 0 , .skip = 0 }, /* 147.150 "Blue Knob Ski Resort KB3KWD" */
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },

	}, 
	{
		/* band 7 - 147.5 - 148 */
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = CTCSS_146_2, .freq_msb = 5111, .freq_lsb = 13238, .rev_msb = 5505, .rev_lsb = 393, .skip = 0 }, /* 147.300 "Bluemont Blue Ridge WA4TSC"   */
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5082, .freq_lsb = 5190, .rev_msb = 5475, .rev_lsb = 8729, .skip = 0 }, /* 147.255 "Martinsburg WB8YZV" */
		{ .ctcss_tone = CTCSS_103_5, .freq_msb = 5101, .freq_lsb = 16016, .rev_msb = 5495, .rev_lsb = 3171, .skip = 0 }, /* 147.285 "Circleville Spruce Knob N8HON" */
		{ .ctcss_tone = CTCSS_131_8, .freq_msb = 5121, .freq_lsb = 10459, .rev_msb = 5514, .rev_lsb = 13998, .skip = 0 }, /* 147.315 Basye Great North Mountain K4MRA" vfo: 139.500000 */
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5141, .freq_lsb = 4902, .rev_msb = 5534, .rev_lsb = 8441, .skip = 0 }, /* 147.345 Clear Spring K3MAD" vfo: 139.500000 */
		{ .ctcss_tone = CTCSS_127_3, .freq_msb = 5151, .freq_lsb = 2123, .rev_msb = 5544, .rev_lsb = 5662, .skip = 0 }, /* 147.360 Skyline K7SOB" vfo: 139.500000 */
		{ .ctcss_tone = CTCSS_100_0, .freq_msb = 4993, .freq_lsb = 13814, .rev_msb = 5387, .rev_lsb = 969, .skip = 0 }, /* 147.120 Chambersburg Clark's Knob W3ACH vfo: 139.500000 */
		{ .ctcss_tone = CTCSS_167_9, .freq_msb = 5013, .freq_lsb = 8257, .rev_msb = 5406, .rev_lsb = 11796, .skip = 0 }, /* 147.150 Blue Knob Ski Resort KB3KWD vfo: 139.500000 */
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },	
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
		{ .ctcss_tone = 0, .freq_msb = 0, .freq_lsb = 0, .rev_msb = 0, .rev_lsb = 0, .skip = 0 },
	}
};






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
#define ADC_MEM PROGMEM
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

/* min/max range for each band switch */
static const uint16_t channel_switch_range[CHAN_MAX][2] ADC_MEM = {
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
	_delay_us(40);
}

static inline void ad9833_sselect_low(void)
{
	PORTF &= ~_BV(PF7);
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
#if 0
	SPI_write16(AD9833_B28);
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
		memcpy_P(&range, &band_switch_range[band], sizeof(range));
#endif
		dprintf(PSTR("CMD_LISTADC: band = %u, .min = %u, .max = %u \r\n"), band, range[0], range[1]);
	}

	for(channel = 0; channel < CHAN_MAX; channel++)
	{
		wdt_reset();
#ifdef ADC_IN_EEPROM
		eeprom_read_block(&range, &channel_switch_range[channel], sizeof(range));
#else
		memcpy_P(&range, &channel_switch_range[channel], sizeof(range));
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
			eeprom_read_block(&m, &bands[band][channel], sizeof(m));
			dprintf(PSTR("CMD_LISTCHAN: band = %u, channel = %u, .ctcss_tone = %u, .freq_msb = %u, .freq_lsb = %u, .rev_msb = %u, .rev_lsb = %u\r\n"), band, channel, m.ctcss_tone, m.freq_msb, m.freq_lsb, m.rev_msb, m.rev_lsb);
		}
	
	}

}

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
	eeprom_write_block(&m, &bands[band][channel], sizeof(m));
	eeprom_busy_wait();
	eeprom_read_block(&m, &bands[band][channel], sizeof(m));
	dprintf("SAVECHAN: Read: band: %u, channel: %u msb:%u lsb:%u tone:%u",band, channel, m.freq_msb, m.freq_lsb, m.ctcss_tone);
}

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
}

static void cmd_scanrate(char **argv, uint8_t argc)
{
	uint32_t srate;
	if(argc != 3)
	{
		dprintf(PSTR("\r\nSCANRATE [scanrate] [dwell time] (in ms)\r\n"));
		srate = eeprom_read_dword(&scan_rate);
		dtime = eeprom_read_dword(&dwell_time);
		dprintf(PSTR("Current rate is: %lu dwell time: %lu\r\n"), srate, dtime);
		return;
	}
	srate = atoul(argv[1]);
	dtime = atoul(argv[2]);
	dprintf(PSTR("\r\nSCANRATE: set scan rate to %lu - dwell time: %lu\r\n"), srate, dtime);
	eeprom_write_dword(&scan_rate, srate);
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
	{ .cmd = "savechan", .handler = cmd_savechan }, 
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

	int result; 
	uint8_t parc;
	uint8_t count = 0;

	while(++count < 128 && ((result = rb_linebuf_get(&uart_rx_buf, buf, BUF_DATA_SIZE, false, false)) > 0))
	{	
		wdt_reset();
		parc = rb_string_to_array(buf, para, MAX_PARAMS);
		if(parc == 0)
		{
			continue;
		}
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

static void process_uart(void)
{
	static char buf[BUF_DATA_SIZE];
	char *p;
	uint16_t count = 0;


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
	static uint16_t channel_range[CHAN_MAX][2];
	uint8_t i; 

	memcpy_P(&channel_range, &channel_switch_range, sizeof(channel_range));
//	eeprom_read_block(&channel_range, &channel_switch_range, sizeof(channel_range));
	for(i = 0; i < CHAN_MAX; i++)
	{
		uint16_t c_min = channel_range[i][0];
		uint16_t c_max =  channel_range[i][1];
		
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
	static uint16_t band_range[BAND_MAX][2];
	uint8_t i;

	memcpy_P(&band_range, &band_switch_range, sizeof(band_range));	
//	eeprom_read_block(&band_range, &band_switch_range, sizeof(band_range));
	for(i = 0; i < BAND_MAX; i++)
	{
		uint16_t b_min =  band_range[i][0];
		uint16_t b_max = band_range[i][1];

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
	struct memory_entry m;
		
	/* band is zero indexed, channel is indexed on 1, (zero being the vfo channel) */
	if(band > BAND_MAX - 1 || channel > CHAN_MAX)
	{
		dprintf(PSTR("set_channel: band or channel out of range\r\n"));
		return;
	}
	
	if(channel == CHAN_VFO)
	{
		dprintf(PSTR("set_channel: vfo selected, turning DDS amp off\r\n"));
//		rb_event_add(blink1_cb, NULL, 100, 12);
		ad9833_shutdown();
		return;
	}
	eeprom_read_block(&m, &bands[band][channel], sizeof(m));

	wdt_reset();
	if(m.freq_lsb == UINT16_MAX)
		m.freq_lsb = 0;
	
	if(m.freq_msb == UINT16_MAX)
		m.freq_msb = 0;
	
	if(m.ctcss_tone == UINT8_MAX)
		m.ctcss_tone = 0;

	if(report)
		dprintf(PSTR("set_channel: band: %u channel: %u - %u %u - ctcss frequency: %u\r\n"),  band, channel, m.freq_msb, m.freq_lsb, m.ctcss_tone);

	if(m.ctcss_tone > (sizeof(ctcss_tone_table) / sizeof(uint16_t)))
	{
		cur_mult = 0;
	} else
		cur_mult = pgm_read_word(&ctcss_tone_table[m.ctcss_tone]);
	
	if(m.freq_msb == 0 && m.freq_lsb == 0)
	{
		dprintf(PSTR("set_channel: frequency is zero, turning DDS off\r\n"));
		ad9833_shutdown();
		return;
	} 

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
	adc_avg_channel(&c, &b);			

	if(b < 30)
		return;

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
	
	if(current_channel == CHAN_SCAN)
	{
		dprintf(PSTR("read_channel: starting channel scan\r\n"));
		start_scan();
		return;
	} else {
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



ISR(INT0_vect)
{
	if(bit_is_set(PIND, PD0))
		is_squelched = false;
	else
		is_squelched = true;
}


static void stop_scan(void)
{
	if(scan_channel_ev == NULL)
		return;
	dprintf(PSTR("Stopping scan\r\n"));
	rb_event_delete(scan_channel_ev);
	scan_channel_ev = NULL;	
	led_on();
}

static void start_scan(void)
{
	if(scan_channel_ev == NULL)
	{
		dprintf(PSTR("start_scan\r\n"));
		dtime = eeprom_read_dword(&dwell_time);
		if(dtime == UINT32_MAX)
			dtime = 2000;
		scan_channel_ev = rb_event_add(do_scan, eeprom_read_dword(&scan_rate), 0);
	}
}

static void do_scan(void)
{
	static uint32_t hangtime = 0;
	static uint32_t last_jump = 0;
	static uint16_t last_channel;
	static uint16_t last_band;

	struct memory_entry m;
	uint16_t channel, band;
	uint32_t ts;
	
	ts = current_ts();
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if(is_squelched == false)
			hangtime = ts;

		if(is_txing == true)
		{
			dprintf(PSTR("Stopped scanning due to TXing\r\n"));
			return;
		}
	}

	if(is_squelched == false)
	{
		if((last_jump + dtime) > ts)
			return;
		hangtime = 0;
	} 
	
	if(hangtime + 2000 > ts)
	{
		led_on();
		return;
	}
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
	for(uint16_t i = channel; i < CHAN_MAX - 1; i++)
	{
		eeprom_read_block(&m, &bands[band][i], sizeof(m));
		if((m.freq_msb == 0 && m.freq_lsb == 0) || (m.skip == 1))
			continue;
		last_channel = channel;
		last_band = band;
		set_channel(band, channel, false);
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
	if(is_split == false)
		return;

	if(bit_is_clear(PINE, PE6) && is_txing == true)
	{
		dprintf(PSTR("Switching to FREQ0 - RX\r\n"));
		is_txing = false;
		ad9833_sselect_low();
		SPI_write16(AD9833_FSELECT0);
		ad9833_sselect_high();
		return;
	}

	if(bit_is_set(PINE, PE6) && is_txing == false)
	{
		dprintf(PSTR("Switching to FREQ1 - TX\r\n"));
		is_txing = true;
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
 	dprintf(PSTR("Out of adc_init()\r\n"));
 	
	read_channel_ev = rb_event_add(read_channel, 200, 0);
	rb_event_add(process_uart, 50, 0);
	rb_event_add(process_commands, 20, 0);
	led_on();
	dprintf(PSTR("setup finished\r\n"));	
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
