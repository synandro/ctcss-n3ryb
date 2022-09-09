#ifndef MEMORIES_H
#define MEMORIES_H 1


#define CHAN_VFO 0
#define CHAN_SCAN 11

#define CHAN_MAX 10 
#define CHAN_SWITCH_MAX 12
#define BAND_MAX 8 

// #define MEMORY_IN_EEPROM 1

#ifdef MEMORY_IN_EEPROM
#define MEMORY_MEM EEMEN
#else
#define MEMORY_MEM PROGMEM
#endif


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
	 CTCSS_LAST
};


extern const uint16_t ctcss_tone_table[] PROGMEM;

struct memory_entry {
	uint16_t freq_msb;
	uint16_t freq_lsb;
	uint16_t rev_msb;
	uint16_t rev_lsb;
	uint8_t ctcss_tone;
	bool skip;
	char desc[48];
};


#endif