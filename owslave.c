#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "owslave.h"

#define sbi(x,y)	x|=(1<<y)
#define cbi(x,y)	x&=~(1<<y)

owslave_addr_t owslave_addr EEMEM = {
	.s.type = OWSLAVE_TYPE_DS18B20,
	.s.serial = { 1,2,3,4,5,6 },
	.s.crc = 0x9E
};

//#define SAVE_POWER()		asm("sleep":: )
#define SAVE_POWER()		do{}while(0)

uint8_t
owslave_read_bit() {
	if(bit_is_clear(PINB,OWSLAVE_IOPIN))
		while(bit_is_clear(PINB,OWSLAVE_IOPIN)) SAVE_POWER();
	while(bit_is_set(PINB,OWSLAVE_IOPIN)) SAVE_POWER();
	_delay_us(30);
	return bit_is_set(PINB,OWSLAVE_IOPIN);
}

void
owslave_write_bit(uint8_t v) {
	if(bit_is_clear(PINB,OWSLAVE_IOPIN))
		while(bit_is_clear(PINB,OWSLAVE_IOPIN)) SAVE_POWER();
	cli();
	while(bit_is_set(PINB,OWSLAVE_IOPIN)) {}
	sbi(DDRB,OWSLAVE_IOPIN);
	if(v==0)
		_delay_us(60);
	cbi(DDRB,OWSLAVE_IOPIN);
	sei();
}

uint8_t
owslave_read_byte() {
	uint8_t ret;
	for(uint8_t i=0;i<8;i++) {
		ret>>=1;
		if(owslave_read_bit())
			sbi(ret,7);
	}
	return ret;
}

uint16_t
owslave_read_word() {
	return owslave_read_byte()+(owslave_read_byte()<<8);
}

void
owslave_write_byte(uint8_t byte) {
	for(uint8_t i=0;i<8;i++) {
		owslave_write_bit(byte&1);
		byte>>=1;
	}
}

uint8_t
owslave_crc_update(uint8_t crc,uint8_t data) {
	uint8_t i;

	crc = crc ^ data;
	for (i = 0; i < 8; i++)
	{
		if (crc & 0x01)
			crc = (crc >> 1) ^ 0x8C;
		else
			crc >>= 1;
	}

	return crc;
}

void
owslave_main() {
	// Reset and Setup
	cbi(PORTB,OWSLAVE_IOPIN);
	cbi(DDRB,OWSLAVE_IOPIN);

	// Allow the 1wire pin to generate interrupts.
	sbi(PCMSK,OWSLAVE_IOPIN);
	sbi(GIMSK,PCIE);
	sei();
	
	// Presence Pulse
	while(bit_is_clear(PINB,OWSLAVE_IOPIN)) SAVE_POWER();
	_delay_us(OWSLAVE_T_PDH);
	sbi(DDRB,OWSLAVE_IOPIN);
	_delay_us(OWSLAVE_T_PDL);
	cbi(DDRB,OWSLAVE_IOPIN);
	
	// Read ROM command
	uint8_t cmd = owslave_read_byte();	
	static uint8_t flags;
	flags=0;
	
	if(cmd==OWSLAVE_ROMCMD_MATCH) {
		flags = _BV(2);
	} else if(cmd==OWSLAVE_ROMCMD_READ) {
		flags = _BV(0);
	} else if(cmd==OWSLAVE_ROMCMD_SEARCH) {
		flags = _BV(0)|_BV(1)|_BV(2);
	} else if(cmd==OWSLAVE_ROMCMD_ALARM_SEARCH && owslave_cb_alarm_condition()) {
		flags = _BV(0)|_BV(1)|_BV(2);
	} else if(cmd==OWSLAVE_ROMCMD_SKIP) {
		flags = 0;
	} else goto wait_for_reset;

	if(flags)
	for(uint8_t i=0;i<8;i++) {
		uint8_t byte = eeprom_read_byte(&owslave_addr.d[i]);
		for(uint8_t j=0;j<8;j++) {
			if(flags&_BV(0))
				owslave_write_bit(byte&1);
			if(flags&_BV(1))
				owslave_write_bit((~byte)&1);
			if(flags&_BV(2)) {
				if((byte&1)^owslave_read_bit())
					goto wait_for_reset;
			}
			byte>>=1;
		}
	}

	// Read function command
	cmd = owslave_read_byte();

	if(cmd==OWSLAVE_FUNCCMD_CONVERT) {
		owslave_cb_convert();
	} else if(cmd==OWSLAVE_FUNCCMD_CONVERT_T) {
		owslave_cb_convert();
	} else if(cmd==OWSLAVE_FUNCCMD_RD_SCRATCH) {
		uint8_t crc = 0; // TODO: implement me
		for(uint8_t i=0;i<7;i++) {
			uint8_t byte = owslave_cb_read_byte(i);
			owslave_write_byte(byte);
			crc = owslave_crc_update(crc,byte);
		}
		owslave_write_byte(crc);
	} else if(cmd==OWSLAVE_FUNCCMD_RD_MEM) {
		uint16_t addr = owslave_read_word();
		uint8_t crc = 0; // TODO: implement me
		for(uint8_t i=0;i<7;i++) {
			uint8_t byte = owslave_cb_read_byte(addr++);
			owslave_write_byte(byte);
			crc = owslave_crc_update(crc,byte);
		}
		owslave_write_byte(crc);
		
	} else if(cmd==OWSLAVE_FUNCCMD_WR_MEM) {
	} else if(cmd==OWSLAVE_FUNCCMD_CP_SCRATCH) {
	} else if(cmd==OWSLAVE_FUNCCMD_RECALL_E2) {
	}


wait_for_reset:
	while(1)
		SAVE_POWER();
}

ISR(TIM0_OVF_vect) {
	// Timer has overflowed. Soft reset.
	((void (*)(void))0x0000)();
}

ISR(INT0_vect) {
	// Pin change interrupt
	TCCR0B = 0; // Stop the timer.

	if(bit_is_clear(PINB,OWSLAVE_IOPIN)) {
		// High-Low Transition
		
		// Reset counter.
		TCNT0 = 0;

		// Enable overflow interrupt.
		sbi(TIMSK0,TOIE0);
		
		// Start timer with prescaler of 1/8. (~.8333µSec per tick & 9.6MHz)
		TCCR0B = (1<<1);
	}
}

ISR(PCINT0_vect) {
	// Pin change interrupt
	TCCR0B = 0; // Stop the timer.
	sbi(GIFR,PCIF);


	if(bit_is_clear(PINB,OWSLAVE_IOPIN)) {
		// High-Low Transition
		
		// Reset counter.
		TCNT0 = 0;

		// Enable overflow interrupt.
		sbi(TIMSK0,TOIE0);
		
		// Start timer with prescaler of 1/8. (~.8333µSec per tick & 9.6MHz)
		TCCR0B = (1<<1);
	}
}
