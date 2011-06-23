#include "moist.h"
#include "owslave.h"
#include <avr/io.h>

#define 	ATTR_NO_INIT   __attribute__ ((section (".noinit")))

uint8_t moist_value ATTR_NO_INIT;
uint8_t moist_ceiling ATTR_NO_INIT;
uint8_t moist_floor ATTR_NO_INIT;
uint8_t moist_alarm_low ATTR_NO_INIT;
uint8_t moist_alarm_high ATTR_NO_INIT;
uint8_t moist_count ATTR_NO_INIT;

void
owslave_cb_convert() {
	moist_value = moist_calc();

	if(moist_value<moist_floor)
		moist_floor = moist_value;

	if(moist_value>moist_ceiling)
		moist_ceiling = moist_value;
	moist_count++;
}

uint8_t
owslave_cb_read_byte(uint16_t i) {
	return (&moist_value)[i];
}

bool
owslave_cb_alarm_condition() {
	return (moist_value>=moist_alarm_high) || (moist_value<=moist_alarm_low);
}

void
owslave_cb_write_byte(uint16_t i,uint8_t value) {
	(&moist_value)[i] = value;
}

int
main(void) {

	// Main loop.
	while(1) owslave_main();
}
