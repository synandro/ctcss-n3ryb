#define REF_FREQ 25000000

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

void calculate_ad9833(double frequency, uint16_t *freq_MSB, uint16_t *freq_LSB)
{
        uint32_t freq_data = (double)(frequency * powf(2, 28)) / (double)REF_FREQ;
//        fprintf(stderr, "frequency: %f freq_data: %lu\r\n", frequency, freq_data);
        *freq_MSB = (freq_data >> 14);
        *freq_LSB = (freq_data & 0x3FFF); 
}


int main(int argc, char **argv)
{
	double f;
	uint16_t msb, lsb;
	
	f = atof(argv[1]);
	calculate_ad9833(f, &msb, &lsb);
	
	printf("f:%f msb: %u lsb: %u\n", f, msb, lsb);
	

    

}