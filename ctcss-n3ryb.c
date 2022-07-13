#include <stdint.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdbool.h>
#define MULT_77 618
#define MULT_100 807
#define MULT_103_5 828
#define MULT_118_8 949
#define MULT_123 984
#define MULT_127_3 1017
#define MULT_131_8 1052
#define MULT_141_3 1129
#define MULT_146_2 1168
#define MULT_167_9 1342
#define MULT_179_9  1439
#define MULT_203_5 1624
#define MULT_NONE 0

struct frequencies 
{
	uint16_t mult;
	uint16_t start;
	uint16_t end;
	uint8_t channel;
};

typedef enum  {
//	HZ_77 = 0,
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
} ctcss_t;


static const struct frequencies freq_table[] = {
	{ .mult = MULT_100, 	.start = 950,	.end = 1024,	.channel = 1 /* .freq = 77.0  */	},
	{ .mult = MULT_103_5, 	.start = 860,	.end = 920,	.channel = 2 /* .freq = 100  */	},
	{ .mult = MULT_118_8, 	.start = 751,	.end = 860,	.channel = 3 /* .freq = 103.5  */},
	{ .mult = MULT_123, 	.start = 651,	.end = 750,	.channel = 4  /* .freq = 118.8 */},
	{ .mult = MULT_127_3, 	.start = 550,	.end = 650,	.channel = 5 /* .freq = 123 */	},
	{ .mult = MULT_131_8, 	.start = 450,	.end = 550,	.channel = 6 /* .freq = 127.3 */},
	{ .mult = MULT_141_3, 	.start = 330,	.end = 450,	.channel = 7 /* .freq = 131.8 */},
	{ .mult = MULT_146_2, 	.start = 300,	.end = 329,	.channel = 8 /* .freq = 141.3 */},
	{ .mult = MULT_167_9, 	.start = 200,	.end = 299,	.channel = 9 /* .freq = 146.2 */},
	{ .mult = MULT_179_9, 	.start = 130,	.end = 200,	.channel = 10 /* .freq = 167.9 */},
	{ .mult = MULT_NONE, 	.start = 30,	.end = 130,	.channel = 11 /* .freq = 179.9 */},
};


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

static uint16_t cur_mult;
static uint16_t counter;
static ctcss_t cur_freq = HZ_NONE+1;

static uint8_t EEMEM saved_frequency = HZ_123;

#define led_off() (PORTB &= ~(1 << PB0));
#define led_on() (PORTB |= (1 << PB0));


static inline void stop_sine(void)
{
	TIMSK = 0;
}

static inline void start_sine(void)
{
	TCCR1 = 1<<PWM1A | 2<<COM1A0 | 1<<CS10;
}


static void set_saved_frequency(void)
{
	eeprom_write_byte(&saved_frequency, cur_freq);
}

static void load_saved_frequency(void)
{
	uint8_t f;
	f = eeprom_read_byte(&saved_frequency);
	change_frequency(f);
}


static void change_frequency(ctcss_t freq)
{
	cur_freq = freq;
	cur_mult = freq_table[freq].mult;
}

static void setup() 
{
	PLLCSR = 1<<PCKE | 1<<PLLE;     

	_delay_ms(100); /* give the PLL a chance to spin up */

	change_frequency(HZ_123);

	TIMSK = 0;                          
	TCCR1 = 1<<PWM1A | 2<<COM1A0 | 1<<CS10;

	TCCR0A = 3<<WGM00;               
	TCCR0B = 1<<WGM02 | 2<<CS00;      
  	TIMSK = 1<<OCIE0A;                
  	OCR0A = 250;

	DDRB |= (1 << PB1);  // PB1 direction to output - this is our pwm output

  	DDRB &= ~(1 << DDB3); 
  	PORTB |= (1 << PORTB2);	/* activate internal pull-up resistor for PB3 */
  	DDRB |= (1 << PB0); // PB0 direction to output too
  	PORTB |= (1 << PB0); // turn the led on 
  	
  	ADCSRA|=(1<<ADEN);      //Enable ADC module
  	ADMUX=0x01; // configuring PB2 to take input
  	ADCSRB=0x00;           //Configuring free running mode
  	ADCSRA|=(1<<ADSC)|(1<<ADATE);   //Start ADC conversion and enabling Auto trigger
            
  	sei();
}

static void do_blink(int msec, int msec2, int count)
{
	for(int i = 0; i < count; i++)
	{
		led_on();
		_delay_ms(200);
		led_off();
		_delay_ms(200);
		led_on();
	}	
}

static uint16_t adc_avg(void)
{
	uint16_t adc_val = 0;
	uint16_t adc_l;
	for(uint8_t i = 0; i < 8; i++)
	{
		adc_l = ADCL;
		adc_val += ((ADCH<<8)|adc_l);
		_delay_ms(5);
	}
	return (adc_val / 8);
}


static void loop() 
{
	uint8_t x;
	uint16_t adc_val;
	uint16_t adc2_val;
	led_on();	

	while(1)
	{
		adc_val = adc_avg();
		_delay_ms(500);
		adc2_val = adc_avg();
		
		for(x = 0; x < sizeof(freq_table)/sizeof(struct frequencies); x++)
		{
			/* do both samples agree */
			if((adc_val >= freq_table[x].start && adc_val <= freq_table[x].end) && (adc2_val >= freq_table[x].start && adc2_val <= freq_table[x].end))
			{
				if(cur_freq == x)
				{
					_delay_ms(30);
					continue;
				} else {
					
					for(int y = 0; y < 2; y++)
					{
						do_blink(0, 0, freq_table[x].channel);
						_delay_ms(1000);
						led_on();
					}
					change_frequency(x);
					_delay_ms(3000); /* don't make any more changes for a wee bit */
					continue;
				
				}
			}
		
		}
	}	

}


ISR(TIMER0_COMPA_vect) 
{
	OCR1A = sine_wave[((counter += cur_mult) >> 8)];
}

int main(void)
{
	setup();
	loop();	
}
