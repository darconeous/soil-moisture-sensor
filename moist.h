

#ifndef __MOIST_H__
#define __MOIST_H__

#include <stdint.h>

#define MOIST_FULLY_DRIVE_PULSES	0

#define MOIST_DRIVE_PIN		4
#define MOIST_COLLECTOR_PIN	3
#define MOIST_MAX_VALUE		(const uint8_t)((1l<<(sizeof(uint8_t)*8l))-1)

extern uint8_t moist_calc();

#endif
