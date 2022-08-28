#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

static const char ten_A[]  = "10A";
static const char ten_B[] = "10B";
static const char ten_C[] = "10C";
static const char ten_D[] = "10D";

static const double BAND_LOW = 116;
static const double BAND_HIGH = 118;

struct bands
{
	const double band_start;
	const double band_stop;
	const double hilo;
	const char *bandswitch;
	double vfo;
};

static const struct bands bl[] = {
	{  .band_start = 144.00, .band_stop = 144.50, .bandswitch = ten_A, .hilo = BAND_LOW, .vfo = 136 },
	{  .band_start = 144.50, .band_stop = 145.00, .bandswitch = ten_B, .hilo = BAND_LOW, .vfo = 136.5},
	{  .band_start = 145.00, .band_stop = 145.50, .bandswitch = ten_C, .hilo = BAND_LOW, .vfo = 137 },
	{  .band_start = 145.50, .band_stop = 146.00, .bandswitch = ten_D, .hilo = BAND_LOW, .vfo = 137.5},
#if 0
	{  .band_start = 146.00, .band_stop = 146.50, .bandswitch = ten_A, .hilo = BAND_HIGH, .vfo = 138},

	{  .band_start = 146.50, .band_stop = 147.00, .bandswitch = ten_B, .hilo = BAND_HIGH, .vfo = 138.5},
#endif
	{  .band_start = 146.00, .band_stop = 146.60, .bandswitch = ten_A, .hilo = BAND_HIGH, .vfo = 138},

	{  .band_start = 146.50, .band_stop = 147.00, .bandswitch = ten_B, .hilo = BAND_HIGH, .vfo = 138.5},
	{  .band_start = 147.00, .band_stop = 147.50, .bandswitch = ten_C, .hilo = BAND_HIGH, .vfo = 139.0},
	{  .band_start = 147.50, .band_stop = 148.00, .bandswitch = ten_D, .hilo = BAND_HIGH, .vfo = 139.5},
};


struct repeater_bands {
	const double band_start;
	const double band_stop;
	const double band_shift;
};

static const struct repeater_bands rs[] = {
	{ .band_start = 145.1, .band_stop = 145.5, .band_shift = -.600 },
	{ .band_start = 146.0, .band_stop = 146.4, .band_shift = .600 },
	{ .band_start = 146.6, .band_stop = 147.0, .band_shift = -.600 },
	{ .band_start = 147.0, .band_stop = 147.4, .band_shift = .600 },
	{ .band_start = 147.6, .band_stop = 148.0, .band_shift = -.600 },
};

double repeater_input(double freq)
{
	int x;
	for(x = 0; x < (sizeof(rs) / sizeof(struct repeater_bands)); x++)
	{
		if((freq >= rs[x].band_start) && (freq <= rs[x].band_stop))
		{
			return freq + rs[x].band_shift;
		}
	}
	return 0.0;
}

bool downconvert(double freq, double *outfreq, const char **bandsw, double *vfo)
{
	int x;
	for(x = 0; x < (sizeof(bl) / sizeof(struct bands)); x++)
	{
		if((freq >= bl[x].band_start) && (freq <= bl[x].band_stop))
		{
			*outfreq = freq - bl[x].hilo;
			*bandsw = bl[x].bandswitch;
			*vfo = bl[x].vfo;
			return true;
		}
	}
	return false;
}

#define REF_FREQ 25000000
void calculate_ad9833(double frequency, uint16_t *freq_MSB, uint16_t *freq_LSB)
{
        uint32_t freq_data = (float)(frequency * powf(2, 28)) / (float)REF_FREQ;
        fprintf(stderr, "freq_data: %lu\r\n", freq_data);
        *freq_MSB = (freq_data >> 14);
        *freq_LSB = (freq_data & 0x3FFF); 
}


int main(int argc, char **argv)
{
	double freq, infreq, ten_freq, ten_infreq, vfo, vfoin;
	const char *obs, *ibs;
	uint16_t freq_MSB, freq_LSB;	
	if(argc <= 1)
	{
		fprintf(stderr, "Not enough arguments\n");
		return 1;
	}

	freq = atof(argv[1]);

	infreq = repeater_input(freq);
	downconvert(freq, &ten_freq, &obs, &vfo);
	downconvert(infreq, &ten_infreq, &ibs, &vfoin);
	fprintf(stdout, "Output: %.06f Input: %.06f\n", freq, infreq);
	calculate_ad9833((freq - vfo) * 1000000, &freq_MSB, &freq_LSB);
	fprintf(stdout, "Output: %.06f Input: %.06f VFO: %.06f\n", freq, infreq, (freq - vfo) * 1000000);

	fprintf(stdout, ".freq_msb = %u, .freq_lsb = %u }, /* %.3f */\n", freq_MSB, freq_LSB, freq);
//	fprintf(stdout, ",%.06f,OUTMSB:,%u,OUTLSB:,%u\n", freq, freq_MSB, freq_LSB);
//	fprintf(stdout, "OUT MSB: %u OUTLSB: %u\n", freq_MSB, freq_LSB);
//	calculate_ad9833((infreq - vfoin) * 1000000, &freq_MSB, &freq_LSB);
//	fprintf(stdout, "IN  MSB: %u  INLSB: %u\n", freq_MSB, freq_LSB);

#if 0
	if(downconvert(freq, &ten_freq, &obs) == false || downconvert(infreq, &ten_infreq, &ibs) == false)
	{
		fprintf(stderr, "Unable to calculate offsets\n");
		return 1;
	}

//	fprintf(stdout, "Output: %.06f Input: %.06f\n", freq, infreq);
	calculate_ad9833(frequency, &freq_MS, &freq_LS);
//	calculate_ad9833(frequency, &freq_MS, &freq_LS);
//	fprintf(stdout, "INMSB: %u INLSB: %u\n", freq_MSB, freq_LSB);
	
//	fprintf(stdout, "FR-101: %s %.06f\nFL-101: %s %.06f\n", obs, ten_freq, ibs, ten_infreq);
#endif
	return 0;
}
