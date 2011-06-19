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
