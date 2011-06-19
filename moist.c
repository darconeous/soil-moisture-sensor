
#include "moist.h"
#include <avr/io.h>

#define sbi(x,y)	x|=(1<<y)
#define cbi(x,y)	x&=~(1<<y)

uint8_t moist_value = 0x00;
uint8_t moist_floor = 0xFF;
uint8_t moist_ceiling = 0x00;

#define DELAY_SOME_ARBITRARY_AMOUNT()			for(int tmp_=32000;tmp_;tmp_--){}

void
moist_update() {
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
		(v!=MOIST_MAX_VALUE) && !(PINB&(1<<MOIST_COLLECTOR_PIN));
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

	moist_value = v;

	if(v<moist_floor)
		moist_floor = v;

	if(v>moist_ceiling)
		moist_ceiling = v;
}

