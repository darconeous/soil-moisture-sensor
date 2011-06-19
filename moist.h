

#ifndef __MOIST_H__
#define __MOIST_H__

#include <stdint.h>

#define MOIST_FULLY_DRIVE_PULSES	0

#define MOIST_DRIVE_PIN		4
#define MOIST_COLLECTOR_PIN	3
#define MOIST_MAX_VALUE		((1l<<(sizeof(moist_value)*8l))-1)

extern uint8_t moist_value;
extern uint8_t moist_floor;
extern uint8_t moist_ceiling;

extern void moist_update();

#endif
