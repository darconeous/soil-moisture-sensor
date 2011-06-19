
#include "moist.h"
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define sbi(x,y)	x|=(1<<y)
#define cbi(x,y)	x&=~(1<<y)

uint8_t moist_value = 0x00;
uint8_t moist_floor = 0xFF;
uint8_t moist_ceiling = 0x00;

void
moist_update() {
	// Make sure pins are configured.
	cbi(PORTB,MOIST_DRIVE_PIN);
	cbi(PORTB,MOIST_COLLECTOR_PIN);
	sbi(DDRB,MOIST_DRIVE_PIN);
	sbi(DDRB,MOIST_COLLECTOR_PIN);
	_delay_ms(50);
	cbi(DDRB,MOIST_DRIVE_PIN);
	cbi(DDRB,MOIST_COLLECTOR_PIN);

	for(moist_value=0;
		(moist_value!=MOIST_MAX_VALUE) && !(PINB&(1<<MOIST_COLLECTOR_PIN));
		moist_value++
	) {
		sbi(PORTB,MOIST_DRIVE_PIN);
#if MOIST_FULLY_DRIVE_PULSES
		sbi(DDRB,MOIST_DRIVE_PIN);
		cbi(DDRB,MOIST_DRIVE_PIN);
#endif
		cbi(PORTB,MOIST_DRIVE_PIN);

		_delay_us(1);
	}

	if(moist_value<moist_floor)
		moist_floor = moist_value;

	if(moist_value>moist_ceiling)
		moist_ceiling = moist_value;
}

