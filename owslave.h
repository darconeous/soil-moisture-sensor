

#ifndef __OWSLAVE_H__
#define __OWSLAVE_H__ 1

#include <stdbool.h>

#define OWSLAVE_IOPIN	(0)

#define OWSLAVE_T_X		(20)
#define OWSLAVE_T_PDH	(20) // 15-60 uSec
#define OWSLAVE_T_PDL	(80) // 60-240 uSec

#define OWSLAVE_TYPE_DS1920			0x10
#define OWSLAVE_TYPE_DS18B20		0x28 // Temperature Sensor

enum {
	OWSLAVE_STATE_GET_ROMCMD,
	OWSLAVE_STATE_GET_FUNCCMD,
	OWSLAVE_STATE_WAIT_FOR_RESET,

	OWSLAVE_ROMCMD_READ=0x33,
	OWSLAVE_ROMCMD_MATCH=0x55,
	OWSLAVE_ROMCMD_SEARCH=0xF0,
	OWSLAVE_ROMCMD_SKIP=0xCC,
	OWSLAVE_ROMCMD_ALARM_SEARCH=0xEC,

	// DS2450, Quad ADC
	OWSLAVE_FUNCCMD_RD_MEM=0xAA,
	OWSLAVE_FUNCCMD_WR_MEM=0x55,
	OWSLAVE_FUNCCMD_CONVERT=0x3C,
	
	// DS18B20, 1-Wire Thermometer
	OWSLAVE_FUNCCMD_CONVERT_T=0x44,
	OWSLAVE_FUNCCMD_WR_SCRATCH=0x4E,
	OWSLAVE_FUNCCMD_RD_SCRATCH=0xBE,
	OWSLAVE_FUNCCMD_CP_SCRATCH=0x48,
	OWSLAVE_FUNCCMD_RECALL_E2=0xB8,
};

typedef union {
	struct {
		uint8_t type;
		uint8_t serial[6];
		uint8_t crc;
	} s;
	uint8_t d[8];
} owslave_addr_t;


extern void owslave_main();

// You need to define these functions:

extern void owslave_cb_convert();
extern uint8_t owslave_cb_read_byte(uint16_t i);
extern void owslave_cb_write_byte(uint16_t i,uint8_t byte);
extern bool owslave_cb_alarm_condition();
#endif
