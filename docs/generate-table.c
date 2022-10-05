#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>


#if 0

const float ctcssFrequencies[]  = {
                67, 69.3, 71, 71.9, 74.4, 77, 79.7, 82.5, 85.4, 88.5,
                91.5, 94.8, 97.4, 100, 103.5, 107.2, 110.9, 114.8, 118.8, 123,
                127.3, 131.8, 136.5, 141.3, 146.2, 151.4, 156.7, 159.8, 162.2, 165.5,
                167.9, 171.3, 173.8, 177.3, 179.9, 183.5, 186.2, 189.9, 192.8, 196.6,
                199.5, 203.5, 206.5, 210.7, 218.1, 225.7, 229.1, 233.6, 241.8, 250.3,
                254.1};
#endif
const int maxFrequencyIndex = 13;


uint8_t sine256[256];

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void generateSineWave(float freq) {
  char fname[512];
  float lowFreqAmplitude = 330;
  float HighFreqAmplitude = 120;
  float MidFreqAdjustPct = -.10;
  float minfreq = 67;
  float maxfreq = 206.5;
  float lowBound = 5;
  float MaxUpBound = 240;

  float MinUpBound = HighFreqAmplitude/lowFreqAmplitude * MaxUpBound;

  float upBound = mapf(freq,minfreq,maxfreq,MinUpBound,MaxUpBound);

  // Take out mid freuencey hump in the response
  float midFrequence = (minfreq + maxfreq) / 2.0;  
  if  (freq < midFrequence) {
    upBound = (1 + mapf(freq,minfreq,midFrequence,0,MidFreqAdjustPct)) * upBound;
  } else {
    upBound = (1 + mapf(freq,midFrequence,maxfreq,MidFreqAdjustPct,0)) * upBound;
  }

  printf("// sine table for frequency: %f\n", freq);
  printf("const uint8_t ctcss_%02.f[] = { \n", freq);
  for (int x = 0; x < 255; x++) {
      float sineValue = sin(mapf((float)x, 5.0, 255.0, 0.0, 2.0 * M_PI));
      sine256[x] = (uint8_t)constrain((int)(mapf(sineValue, -1.0, 1.0, lowBound, upBound) + .5),0,255);
      if(x > 0 && (x % 8 == 0)) { printf("\n"); };
      printf("0x%x, ", sine256[x]);
  }
  printf("};\n");
}


int main(int argc, char **argv)
{
  printf("#include <stdint.h>\n");
  printf("#include <avr/pgmspace.h>\n");
  
    generateSineWave(123);
  
}