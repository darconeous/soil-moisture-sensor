#ifndef __OWSLAVE_H__
#define __OWSLAVE_H__ 1

#include <avr/eeprom.h>
#include <stdint.h>

typedef uint8_t bool;
#define true(bool) (1)
#define false(bool) (0)

#define OWSLAVE_T_X     (20)
#define OWSLAVE_T_PDH   (20)    // 15-60 uSec
#define OWSLAVE_T_PDL   (80)    // 60-240 uSec

enum {
	OWSLAVE_TYPE_CUSTOMFLAG = 0x80,

	OWSLAVE_TYPE_DS2401     = 0x01, // Serial-number only
	OWSLAVE_TYPE_DS1920     = 0x10,
	OWSLAVE_TYPE_DS18B20    = 0x28, // Temperature Sensor
	OWSLAVE_TYPE_DS2450     = 0x20, // 4-channel ADC

	OWSLAVE_TYPE_MOIST      = OWSLAVE_TYPE_DS2450 |
	    OWSLAVE_TYPE_CUSTOMFLAG
};

enum {
	OWSLAVE_ROMCMD_READ=0x33,           // 00110011b
	OWSLAVE_ROMCMD_MATCH=0x55,          // 01010101b
	OWSLAVE_ROMCMD_SKIP=0xCC,           // 11001100b
	OWSLAVE_ROMCMD_SEARCH=0xF0,         // 11110000b
	OWSLAVE_ROMCMD_ALARM_SEARCH=0xEC,   // 11101100b

	// DS2450, Quad ADC
	OWSLAVE_FUNCCMD_RD_MEM=0xAA,
	OWSLAVE_FUNCCMD_WR_MEM=0x55,
	OWSLAVE_FUNCCMD_CONVERT=0x3C,

	OWSLAVE_FUNCCMD_COMMIT_MEM=0x48,
	OWSLAVE_FUNCCMD_RECALL_MEM=0xB8,

	// DS18B20, 1-Wire Thermometer
	OWSLAVE_FUNCCMD_CONVERT_T=0x44,
	OWSLAVE_FUNCCMD_WR_SCRATCH=0x4E,
	OWSLAVE_FUNCCMD_RD_SCRATCH=0xBE,
	OWSLAVE_FUNCCMD_CP_SCRATCH=0x48,
	OWSLAVE_FUNCCMD_RECALL_E2=0xB8,

	// Custom
	OWSLAVE_FUNCCMD_RD_NAME=0xF1,
	OWSLAVE_FUNCCMD_WR_NAME=0xFE,
};

typedef union {
	struct {
		uint8_t type;
		uint8_t serial[6];
		uint8_t crc;
	} s;
	uint8_t d[8];
} owslave_addr_t;

extern owslave_addr_t owslave_addr EEMEM;


extern void owslave_main();

// You need to define these functions:

extern void owslave_cb_convert();
extern uint8_t* owslave_cb_get_scratch_ptr();
extern uint8_t owslave_cb_read_byte(uint16_t i);
extern void owslave_cb_write_byte(
	uint16_t i, uint8_t byte);
extern bool owslave_cb_alarm_condition();
extern void owslave_cb_recall();
extern void owslave_cb_commit();
#if OWSLAVE_SUPPORTS_CONVERT_INDICATOR
extern void owslave_begin_busy();
extern void owslave_end_busy();
#else
#define owslave_begin_busy()    do {} while(0)
#define owslave_end_busy()  do {} while(0)
#endif



#endif
