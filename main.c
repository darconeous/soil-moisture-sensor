#include "moist.h"
#include "owslave.h"
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>

#define 	ATTR_NO_INIT   __attribute__ ((section (".noinit")))

struct {
	uint16_t value;
	uint8_t alarm_high;
	uint8_t alarm_low;
	uint8_t config;
	uint8_t calib[3];	// Reserved on DS18B20
	
	// End DS18B20-compatible zone

	uint16_t voltage;	
	uint8_t ceiling;	
	uint8_t floor;
	uint8_t count;
} msensor_scratch ATTR_NO_INIT;

struct {
	uint8_t alarm_high;
	uint8_t alarm_low;
	uint8_t config;
	uint8_t calib[3];
} msensor_e2 EEMEM = {
	.alarm_high = 0xFF,
	.alarm_low = 0x00,
	.config = 0x00,
};

#define BOOT_COOKIE		(0x3c0A)
uint16_t boot_cookie ATTR_NO_INIT;

void
owslave_cb_convert() {
	msensor_scratch.value = moist_calc();

	if(msensor_scratch.value<msensor_scratch.floor)
		msensor_scratch.floor = msensor_scratch.value;

	if(msensor_scratch.value>msensor_scratch.ceiling)
		msensor_scratch.ceiling = msensor_scratch.value;

	msensor_scratch.count++;
}

bool
owslave_cb_alarm_condition() {
	return (((uint8_t*)&msensor_scratch)[1]>=msensor_scratch.alarm_high)
		|| (((uint8_t*)&msensor_scratch)[1]<=msensor_scratch.alarm_low);
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

int
main(void) {
	uint16_t x = BOOT_COOKIE;
	if(boot_cookie!=x) {
		boot_cookie = x;
		memset(&msensor_scratch,0,sizeof(msensor_scratch));
		owslave_cb_recall();
	}
	// Main loop.
	while(1) owslave_main();
}
