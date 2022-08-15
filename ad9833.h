

#define AD9833_D15	(1 << 15)	/* must be set to zero */
#define AD9833_D14	(1 << 14)	/* must be set to zero */
#define AD9833_B28	(1 << 13)	/* set to 1 if writing both words of frequency, 0 if changing half */
#define AD9833_HLB	(1 << 12)	/* hlb 1 modify MSB part of frequency, 0 = change the lower half, ignored if B28 = 1 */
#define AD9833_FSELECT0	(0 << 11)	/* freq0 or freq1 */
#define AD9833_FSELECT1	(1 << 11)	/* freq0 or freq1 */
#define AD9833_PSELECT0	(0 << 10)	/* phase0 or phase1 */
#define AD9833_PSELECT1	(1 << 10)	/* phase0 or phase1 */
#define AD9833_D9 	(1 << 9) 	/* reversed bits are the best bits - set to zero */
#define AD9833_RESET	(1 << 8)	/* 1 resets internal registers to 0, which corresponds to an analog output of midscale.= 0 disables reset */

/* sleep1 sleep2 	*/
/* 0      0     = no power down */
/* 0      1     = dac powered down */
/* 1	  0     = internal clock disable */
/* 1      1	= dac and clock both shutdown */
#define AD9833_SLEEP1   (1 << 7)	/* When SLEEP1 = 1, the internal MCLK clock is disabled, and the DAC output remains at its present value because the  NCO is no longer accumulating. When SLEEP1 = 0, MCLK is enabled. */
#define AD9833_SLEEP2   (1 << 6)	/* SLEEP12 = 1 powers down the on chip DAC. This  is useful when the AD9833 is used to output the MSB of the DAC data. SLEEP12 = 0 implies that the DAC is active */
#define AD9833_OPBITEN  (1 << 5) 	/* When OPBITEN = 1, the output of the DAC is no longer available at the VOUT pin. Instead, the MSB (or MSB/2) of the DAC data is connected to the VOUT pin.  */
					/* This is useful as a coarse clock source. The DIV2 bit  controls  whether it is the MSB or MSB/2 that is output. When OPBITEN = 0, the DAC is connected to VOUT. The mode */
					/* bit determines whether it is a sinusoidal or a ramp output that is available. */
#define AD9833_D4	(1 << 4)	/* reversed bits are the best bits - set to zero */
#define AD9833_DIV2	(1 << 3)	/* DIV2 is used in association with D5 (OPBITEN). This is explained further in  Table  15. When DIV2 = 1, the MSB of the DAC  data is passed directly to the VOUT pin. When DIV2 = 0, the */
					/* MSB/2 of the DAC data is output at the VOUT pin. */
#define AD9833_D2	(1 << 2)	/* reversed bits are the best bits - set to zero */
#define AD9833_MODE	(1 << 1)	/* This bit is used in association with OPBITEN (D5). The function of this bit is to control what is output at the VOUT pin when the onchip DAC is connected  to VOUT. */
					/* This bit should be set to 0 if the control bit OPBITEN = 1. This is explained further in Table 15. When  mode = 1, the SIN ROM is bypassed, resulting in a triangle output */
					/* from the DAC. When mode =0, the SIN ROM is used to convert the phase information into amplitude information, which results in a sinusoidal signal at the output. */
#define AD9833_D0	(1 << 0)	/* reversed bits are the best bits - set to zero */

#define AD9833_FREQ             25000000UL



//#define MAX_FREQ                12.5e6
//#define BITS_PER_DEG    11.3777777777778        // 4096 / 360


