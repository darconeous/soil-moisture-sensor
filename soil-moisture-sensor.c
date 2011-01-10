

#define USE_DRAIN_METHOD 0

#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define sbi(x,y)	x|=(1<<y)
#define cbi(x,y)	x&=~(1<<y)

#if FULLY_DRIVE_PULSES
#define ZONE_A_PULSE()		sbi(PORTB,3),sbi(DDRB,3),_delay_us(100),cbi(DDRB,3),cbi(PORTB,3)
#define ZONE_B_PULSE()		sbi(PORTB,4),sbi(DDRB,4),_delay_us(100),cbi(DDRB,4),cbi(PORTB,4)
#else
#define ZONE_A_PULSE()		sbi(PORTB,3),cbi(PORTB,3)
#define ZONE_B_PULSE()		sbi(PORTB,4),cbi(PORTB,4)
#endif

#define COLLECTOR_CHECK()	(PINB&(1<<2))

uint8_t zone_a;
uint8_t zone_a_floor = 0xFF;
uint8_t zone_a_ceiling;

uint8_t zone_b;
uint8_t zone_b_floor = 0xFF;
uint8_t zone_b_ceiling;

void reset() {
	PORTB = 0x00;
	DDRB = (1<<3)|(1<<4)|(1<<2);
	_delay_ms(50);
	DDRB = 0x00;
}


void zone_a_moisture() {
	uint8_t i;
	zone_a = 0;
	for(i=0;i<1;i++) {
		reset();
#if USE_DRAIN_METHOD
		for(sbi(PORTB,3);!(PINB&(1<<3));zone_a++);
#else
		for(;(zone_a!=((1l<<(sizeof(zone_a)*8l))-1)) && !COLLECTOR_CHECK();zone_a++) {
			ZONE_A_PULSE();
			_delay_us(1);
		}
#endif
	}
	if(zone_a<zone_a_floor)
		zone_a_floor = zone_a;

	zone_a -= zone_a_floor;

	if(zone_a>zone_a_ceiling)
		zone_a_ceiling = zone_a;
}

void zone_b_moisture() {
	uint8_t i;
	zone_b = 0;
	for(i=0;i<1;i++) {
		reset();
#if USE_DRAIN_METHOD
		for(sbi(PORTB,4);!(PINB&(1<<4));zone_b++);
#else
		for(;(zone_b!=((1l<<(sizeof(zone_b)*8l))-1)) && !COLLECTOR_CHECK();zone_b++) {
			ZONE_B_PULSE();
			_delay_us(1);
		}
#endif
	}
	if(zone_b<zone_b_floor)
		zone_b_floor = zone_b;
	
	zone_b -= zone_b_floor;

	if(zone_b>zone_b_ceiling)
		zone_b_ceiling = zone_b;
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

#include "xmas.h"




int main(void) {
/*
PB2 = collector
PB3 = zone a
PB4 = zone b */
	while(1) {
		zone_a_moisture();
		zone_b_moisture();
	}
}
