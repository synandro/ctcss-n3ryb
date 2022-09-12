#include <stdbool.h>
#include <stdint.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "memories.h"

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

const uint16_t ctcss_tone_table[CTCSS_LAST] PROGMEM = {
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




const struct memory_entry bands[BAND_MAX][CHAN_MAX] MEMORY_MEM = {

	{
		/* band 0 - 144-144.49 */
		{ .ctcss_tone = 0, .freq_msb = 5373, .freq_lsb = 15597, .skip = 0, .desc = "144.200" }, 
		{ .ctcss_tone = 0, .freq_msb = 5498, .freq_lsb = 7707, .skip = 0, .desc = "144.390" }, 
	},	
	{
		/* band 1 - 144.5 - 145.0 */
	},	
	{
		/* band 2 - 145.0 - 145.49 */ 	
		{ .ctcss_tone = CTCSS_179_9, .freq_msb = 5341, .freq_lsb = 3014, .skip = 0, .desc = "145.150 Martinsburg W8ORS" }, 
		{ .ctcss_tone = CTCSS_118_8, .freq_msb = 5367, .freq_lsb = 6527, .skip = 0, .desc = "145.190 Moorefield N8VAA" }, 
		{ .ctcss_tone = CTCSS_141_3, .freq_msb = 5380, .freq_lsb = 8283, .skip = 0, .desc = "145.210 Front Royal High Knob Mountain" }, 
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5419, .freq_lsb = 13552, .skip = 0, .desc = "145.270 Meyersdale Hays Mill Fire Tower" }, 
		{ .ctcss_tone = CTCSS_146_2, .freq_msb = 5498, .freq_lsb = 7707, .skip = 0, .desc = "145.390 Winchester North Mountain KG4Y" }, 
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5537, .freq_lsb = 12976, .skip = 0, .desc = "145.450 Oldtown Warrior Mountain WMA W3YMW" }, 
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5564, .freq_lsb = 104, .skip = 0, .desc = "145.490 Bedford K3NQT" }, 
		{ .ctcss_tone = CTCSS_107_2, .freq_msb = 5328, .freq_lsb = 1258, .skip = 0, .desc = "145.130 Haymarket N3KL" }, 
	},
	{	
		/* band 3 - 145.5 - 145.99 */
	},
	{	
		/* band 4 - 146.0 - 146.6 */
		{ .ctcss_tone = 0, .freq_msb = 5583, .freq_lsb = 10931, .skip = 0, .desc = "146.520 simplex" }, 
		{ .ctcss_tone = 0, .freq_msb = 5505, .freq_lsb = 393, .skip = 0, .desc = "146.400 simplex" }, 
		{ .ctcss_tone = 0, .freq_msb = 5518, .freq_lsb = 2149, .skip = 0, .desc = "146.420 simplex" }, 
		{ .ctcss_tone = 0, .freq_msb = 5531, .freq_lsb = 3905, .skip = 0, .desc = "146.440 simplex" }, 
		{ .ctcss_tone = 0, .freq_msb = 5544, .freq_lsb = 5662, .skip = 0, .desc = "146.460 simplex" }, 
		{ .ctcss_tone = 0, .freq_msb = 5557, .freq_lsb = 7418, .skip = 0, .desc = "146.480 simplex" }, 
		{ .ctcss_tone = 0, .freq_msb = 5570, .freq_lsb = 9175, .skip = 0, .desc = "146.500 simplex" }, 
		{ .ctcss_tone = 0, .freq_msb = 5596, .freq_lsb = 12687, .skip = 0, .desc = "146.540 simplex" }, 
		{ .ctcss_tone = 0, .freq_msb = 5609, .freq_lsb = 14444, .skip = 0, .desc = "146.560 simplex" }, 
//		{ .ctcss_tone = 0, .freq_msb = 5622, .freq_lsb = 16200, .skip = 0, .desc = "146.580 simplex" }, 
	},

	{
		/* band 5 - 146.5 - 146.99 */
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5403, .freq_lsb = 7261, .skip = 0, .desc = "146.745 Berkeley Springs Cacapon Mountain KK3L" }, 
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5432, .freq_lsb = 15309, .skip = 0, .desc = "146.790 Clearville Martin Hill PA  K3NQT" }, 
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5442, .freq_lsb = 12530, .skip = 0, .desc = "146.805 Oakland MD KB8NUF" }, 
		{ .ctcss_tone = CTCSS_146_2, .freq_msb = 5452, .freq_lsb = 9751, .skip = 0, .desc = "146.820 Winchester Great North Mountain  W4RKC" }, 
		{ .ctcss_tone = CTCSS_77_0, .freq_msb = 5472, .freq_lsb = 4194, .skip = 0, .desc = "146.850 Charles Town WA4TXE" }, 
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5491, .freq_lsb = 15020, .skip = 0, .desc = "146.880 Midland Dan's Mountain W3YMW" }, 
		{ .ctcss_tone = CTCSS_100_0, .freq_msb = 5531, .freq_lsb = 3905, .skip = 0, .desc = "146.940 Clear Spring Fairview Mountain W3CWC" }, 
		{ .ctcss_tone = CTCSS_131_8, .freq_msb = 5324, .freq_lsb = 13107, .skip = 0, .desc = "146.625 New Market Luray Caverns N4YSA" }, 
	},
	{
		/* band 6 - 147.00 - 147.5*/
		{ .ctcss_tone = CTCSS_146_2, .freq_msb = 5439, .freq_lsb = 7995, .skip = 0, .desc = "147.300 Bluemont Blue Ridge WA4TSC" }, 
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5409, .freq_lsb = 16331, .skip = 0, .desc = "147.255 Martinsburg WB8YZV" }, 
		{ .ctcss_tone = CTCSS_103_5, .freq_msb = 5429, .freq_lsb = 10774, .skip = 0, .desc = "147.285 Circleville Spruce Knob N8HON" }, 
		{ .ctcss_tone = CTCSS_131_8, .freq_msb = 5449, .freq_lsb = 5216, .skip = 1, .desc = "147.315 Basye Great North Mountain K4MRA" }, 
		{ .ctcss_tone = CTCSS_123_0, .freq_msb = 5468, .freq_lsb = 16043, .skip = 0, .desc = "147.345 Clear Spring K3MAD" }, 
		{ .ctcss_tone = CTCSS_127_3, .freq_msb = 5478, .freq_lsb = 13264, .skip = 0, .desc = "147.360 Skyline K7SOB" }, 
		{ .ctcss_tone = CTCSS_100_0, .freq_msb = 5321, .freq_lsb = 8572, .skip = 0, .desc = "147.120 Chambersburg Clark's Knob W3ACH" }, 
		{ .ctcss_tone = CTCSS_167_9, .freq_msb = 5341, .freq_lsb = 3014, .skip = 0, .desc = "147.150 Blue Knob Ski Resort KB3KWD" }, 
	}, 
	{
		/* band 7 - 147.5 - 148 */
		{ .ctcss_tone = 0, .freq_msb = 5177, .freq_lsb = 5636, .skip = 0, .desc = "147.400 simplex" }, 
		{ .ctcss_tone = 0, .freq_msb = 5190, .freq_lsb = 7392, .skip = 0, .desc = "147.420 simplex" }, 
		{ .ctcss_tone = 0, .freq_msb = 5203, .freq_lsb = 9148, .skip = 0, .desc = "147.440 simplex" }, 
		{ .ctcss_tone = 0, .freq_msb = 5216, .freq_lsb = 10905, .skip = 0, .desc = "147.460 simplex" }, 
		{ .ctcss_tone = 0, .freq_msb = 5229, .freq_lsb = 12661, .skip = 0, .desc = "147.480 simplex" }, 
		{ .ctcss_tone = 0, .freq_msb = 5242, .freq_lsb = 14417, .skip = 0, .desc = "147.500 simplex" }, 
		{ .ctcss_tone = 0, .freq_msb = 5255, .freq_lsb = 16174, .skip = 0, .desc = "147.520 simplex" }, 
		{ .ctcss_tone = 0, .freq_msb = 5269, .freq_lsb = 1546, .skip = 0, .desc = "147.540 simplex" }, 
		{ .ctcss_tone = 0, .freq_msb = 5282, .freq_lsb = 3303, .skip = 0, .desc = "147.560 simplex" }, 
//		{ .ctcss_tone = 0, .freq_msb = 5295, .freq_lsb = 5059, .skip = 0, .desc = "147.580 simplex" }, 
	}
};
