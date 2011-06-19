
#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define sbi(x,y)	x|=(1<<y)
#define cbi(x,y)	x&=~(1<<y)

#define MOIST_FULLY_DRIVE_PULSES	1

#define MOIST_DRIVE_PIN		1
#define MOIST_COLLECTOR_PIN	3
#define MOIST_MAX_VALUE		((1l<<(sizeof(moist_value)*8l))-1)

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

/*

ISR(TIM0_OVF_vect) {
	// Timer has overflowed.

}

ISR(PCINT0_vect) {
	// Pin change interrupt
}

ISR(INT0_vect) {
	// external interrupt
}

#define ONE_WIRE_SLAVE_IOPIN	(1)
#define ONE_WIRE_SLAVE_PORT	PORTB
#define ONE_WIRE_SLAVE_DDR	DDRB
#define ONE_WIRE_SLAVE_PCMSK	PCMSK

void
one_wire_slave() {
	// Start timer with prescaler of 1/1024
	TCCR0B = (1<<CS2)|(1<<CS0);
	
	sbi(PCMSK,ONE_WIRE_SLAVE_IOPIN);
	sbi(GIMV,PCIE);

}
*/

int main(void) {
	while(1) {
		moist_update();
	}
}
