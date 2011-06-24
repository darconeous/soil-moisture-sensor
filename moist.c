
#include "moist.h"
#include <avr/io.h>

#define sbi(x,y)	x|=(uint8_t)(1<<y)
#define cbi(x,y)	x&=(uint8_t)~(1<<y)

#if __HAS_DELAY_CYCLES
#define DELAY_SOME_ARBITRARY_AMOUNT()			__builtin_avr_delay_cycles(32000*2)
#else
#define DELAY_SOME_ARBITRARY_AMOUNT()			for(int tmp_=32000;tmp_;tmp_--){}
#endif

uint16_t
moist_calc() {
	uint8_t v;

	// Make sure pins are configured.
	cbi(PORTB,MOIST_DRIVE_PIN);
	cbi(PORTB,MOIST_COLLECTOR_PIN);
	sbi(DDRB,MOIST_DRIVE_PIN);
	sbi(DDRB,MOIST_COLLECTOR_PIN);
	DELAY_SOME_ARBITRARY_AMOUNT();
	cbi(DDRB,MOIST_DRIVE_PIN);
	cbi(DDRB,MOIST_COLLECTOR_PIN);

	for(v=0;
		(v!=MOIST_MAX_VALUE) && bit_is_clear(PINB,MOIST_COLLECTOR_PIN);
		v++
	) {
		sbi(PORTB,MOIST_DRIVE_PIN);
#if MOIST_FULLY_DRIVE_PULSES
		sbi(DDRB,MOIST_DRIVE_PIN);
		cbi(DDRB,MOIST_DRIVE_PIN);
#endif
		cbi(PORTB,MOIST_DRIVE_PIN);

		DELAY_SOME_ARBITRARY_AMOUNT();
	}

	// Pull both lines low to avoid floating inputs
	sbi(DDRB,MOIST_DRIVE_PIN);
	sbi(DDRB,MOIST_COLLECTOR_PIN);


	return v<<8;
}

