#ifndef PWM_SINE_H
#define PWM_SINE_H 1

extern uint16_t cur_mult; /* multiplier for the sine table */
extern uint16_t sin_counter;
void setup_pwm(void);

/* set the pwm tone output, tone is frequency in (Hertz * 8) */
inline static void __attribute__((always_inline)) set_ctcss(uint16_t tone)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		cur_mult = tone;
	}
}


#endif
