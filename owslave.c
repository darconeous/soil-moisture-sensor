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

#include <util/crc16.h>
#define owslave_crc_update		_crc_ibutton_update

//#define SAVE_POWER()		__builtin_avr_sleep()
#define SAVE_POWER()		__asm__ __volatile__("sleep")
//#define SAVE_POWER()		do{}while(0)

uint8_t
owslave_read_bit() {
	if(bit_is_clear(PINB,OWSLAVE_IOPIN))
		loop_until_bit_is_set(PINB,OWSLAVE_IOPIN);
	loop_until_bit_is_clear(PINB,OWSLAVE_IOPIN);
	_delay_us(30);
	return bit_is_set(PINB,OWSLAVE_IOPIN);
}

void
owslave_write_bit(uint8_t v) {
	if(bit_is_clear(PINB,OWSLAVE_IOPIN))
		loop_until_bit_is_set(PINB,OWSLAVE_IOPIN);
	cli();
	loop_until_bit_is_clear(PINB,OWSLAVE_IOPIN);
	sbi(DDRB,OWSLAVE_IOPIN);
	if(v==0)
		_delay_us(60);
	cbi(DDRB,OWSLAVE_IOPIN);
	sei();
}

uint8_t
owslave_read_byte() {
	uint8_t ret;
	for(uint8_t i=0;i!=8;i++) {
		ret>>=1;
		if(owslave_read_bit())
			sbi(ret,7);
	}
	return ret;
}

static inline uint16_t
owslave_read_word() {
	return owslave_read_byte()+(owslave_read_byte()<<8);
}

void
owslave_write_byte(uint8_t byte) {
	for(uint8_t i=0;i!=8;i++) {
		owslave_write_bit(byte&1);
		byte>>=1;
	}
}

void
owslave_write_bytes_with_crc(const uint8_t* bytes,uint8_t count)
{
	// Never call this function with a count of less than 1!
	uint8_t crc = 0;
	do {
		uint8_t byte = *bytes++;
		owslave_write_byte(byte);
		crc = owslave_crc_update(crc,byte);
	} while(--count);
	owslave_write_byte(crc);
}

void
owslave_main() {
	uint8_t cmd;
	uint8_t flags;
	uint8_t* scratch_ptr;

	// Reset and Setup
	cbi(PORTB,OWSLAVE_IOPIN);
	cbi(DDRB,OWSLAVE_IOPIN);

	// Allow the 1wire pin to generate interrupts.
	sbi(PCMSK,OWSLAVE_IOPIN);
	sbi(GIMSK,PCIE);
	sei();
	
	// Wait for reset pulse to end.
	while(bit_is_clear(PINB,OWSLAVE_IOPIN)) SAVE_POWER();

	// Send presence Pulse
	_delay_us(OWSLAVE_T_PDH);
	sbi(DDRB,OWSLAVE_IOPIN);
	_delay_us(OWSLAVE_T_PDL);
	cbi(DDRB,OWSLAVE_IOPIN);
	
	// Read ROM command
	cmd = owslave_read_byte();	
	flags = 0;
	
	if(cmd==OWSLAVE_ROMCMD_MATCH) {
		flags = _BV(2);
	} else if(cmd==OWSLAVE_ROMCMD_READ) {
		flags = _BV(0);
	} else if((cmd==OWSLAVE_ROMCMD_SEARCH)
		|| ( (cmd==OWSLAVE_ROMCMD_ALARM_SEARCH)
			&& owslave_cb_alarm_condition()
		)
	) {
		flags = _BV(0)|_BV(1)|_BV(2);
	} else if(cmd==OWSLAVE_ROMCMD_SKIP) {
		flags = 0;
	} else goto wait_for_reset;

	if(flags) {
		for(uint8_t i=0;i!=8;i++) {
			uint8_t byte = eeprom_read_byte(&owslave_addr.d[i]);
			uint8_t j = 8;
			do {
				if(flags&_BV(0))
					owslave_write_bit(byte&1);
				if(flags&_BV(1))
					owslave_write_bit((~byte)&1);
				if(flags&_BV(2)) {
					if((byte&1)^owslave_read_bit())
						goto wait_for_reset;
				}
				byte>>=1;
			} while(--j);
		}
	}

	// Read function command
	cmd = owslave_read_byte();
	scratch_ptr=owslave_cb_get_scratch_ptr();
	
	if((cmd==OWSLAVE_FUNCCMD_CONVERT)
		|| (cmd==OWSLAVE_FUNCCMD_CONVERT_T)
	) {
		owslave_cb_convert();
	} else if(cmd==OWSLAVE_FUNCCMD_RD_SCRATCH) {
		owslave_write_bytes_with_crc(scratch_ptr,8);
	} else if(cmd==OWSLAVE_FUNCCMD_RD_MEM) {
		scratch_ptr+=owslave_read_byte();
		owslave_read_byte();
		owslave_write_bytes_with_crc(scratch_ptr,8);
	} else if(cmd==OWSLAVE_FUNCCMD_RECALL_E2) {
		owslave_cb_recall();
	} else if(cmd==OWSLAVE_FUNCCMD_CP_SCRATCH) {
		owslave_cb_commit();
	} else if(cmd==OWSLAVE_FUNCCMD_WR_SCRATCH) {
		scratch_ptr+=2;
		for(uint8_t i=3;i;--i)
			*scratch_ptr++ = owslave_read_byte();
	}

wait_for_reset:
	for(;;)
		SAVE_POWER();
}

ISR(TIM0_OVF_vect) {
	// Timer has overflowed. Soft reset.
	((void (*)(void))0x0000)();
}

ISR(PCINT0_vect) {
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
