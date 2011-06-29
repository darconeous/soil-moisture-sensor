#include "moist.h"
#include "owslave.h"
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include "eeprom.h"

#define 	ATTR_NO_INIT   __attribute__ ((section (".noinit")))

#define DEVICE_IS_SPACE_CONSTRAINED	(defined (__AVR_ATtiny13__) || defined (__AVR_ATtiny13A__))

struct {
	uint16_t value;
	uint8_t alarm_high;
	uint8_t alarm_low;
	uint8_t config;
	uint8_t calib_range;
	uint8_t calib_offset;
	uint8_t pad;
	
	// End DS18B20-compatible zone

	uint16_t voltage;	
	uint8_t ceiling;	
	uint8_t floor;
	uint8_t count;
} msensor_scratch ATTR_NO_INIT;


void
owslave_cb_convert() {
	uint8_t tmp;
	uint16_t value = moist_calc();
	
	// Calibrate
	value -= msensor_scratch.calib_offset;
	value <<= 8;
	value /= msensor_scratch.calib_range;
	
	msensor_scratch.value = value;
	
	tmp = ((uint8_t*)&msensor_scratch.value)[1];

	if(tmp<msensor_scratch.floor)
		msensor_scratch.floor = tmp;

	if(tmp>msensor_scratch.ceiling)
		msensor_scratch.ceiling = tmp;

	msensor_scratch.count++;
}

bool
owslave_cb_alarm_condition() {
	uint8_t tmp = ((uint8_t*)&msensor_scratch.value)[1];
	return (uint8_t)((uint8_t)(tmp>=msensor_scratch.alarm_high)
		|| (uint8_t)(tmp<=msensor_scratch.alarm_low));
}

uint8_t*
owslave_cb_get_scratch_ptr() {
	return ((uint8_t*)&msensor_scratch);
}

void
owslave_cb_recall() {
	eeprom_busy_wait();
	eeprom_read_block(&msensor_scratch.alarm_high,&msensor_e2.alarm_high,sizeof(msensor_e2));
}

void
owslave_cb_commit() {
	cli();
	eeprom_busy_wait();
	eeprom_write_block(&msensor_scratch.alarm_high,&msensor_e2.alarm_high,sizeof(msensor_e2));
	eeprom_busy_wait();
	sei();
}

void
main(void) {
	if(MCUSR)
	{
		memset(&msensor_scratch,0,sizeof(msensor_scratch));
		owslave_cb_recall();
	}
	MCUSR = 0;

	// Main loop.
	for(;;) owslave_main();
}
