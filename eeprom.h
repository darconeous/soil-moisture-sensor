
#include <avr/eeprom.h>

struct msensor_e2_t {
	uint8_t alarm_high;
	uint8_t alarm_low;
	uint8_t config;
	uint8_t calib_range;
	uint8_t calib_offset;
	uint8_t pad;
};

extern struct msensor_e2_t msensor_e2 EEMEM;

extern char device_name[32] EEMEM;