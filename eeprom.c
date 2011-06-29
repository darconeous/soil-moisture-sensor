#include "eeprom.h"
#include "owslave.h"

owslave_addr_t owslave_addr EEMEM = {
	.s.type = OWSLAVE_TYPE_DS18B20,
	.s.serial = { 1,2,3,4,5,6 },
	.s.crc = 0x9E
};

/* Calibration info calculation
**
**	measured_none: Measured value for sensor suspended in open air at 20¡C, 5v.
**	measured_full: Measured value when sensing element is fully submerged
**	               in salt water at 20¡C, 5v.
**	max_value: Target value for maximum reading.
**	Vih: Minimum voltage for a I/O line to read as 'HIGH'. This value is
**       approx 0.7 volts.
**	Vint: internal voltage refence
**	Vod:	(Vcc-Voh), voltage drop for an I/O pin driven high
**	env_factor:	Compensation for voltage and temperature.
**	
**	calib_offset = measured_none
**	calib_range = (measured_full-measured_none)
**
**	calib_value = (value-calib_offset*env_factor)*env_factor*max_value/calib_range
**
**	Other variables to consider: temperature and voltage.
**
**	Lower voltage will cause *much higher* moisture readings. If 5v gives a
**	reading of 100, then the same reading at 3.3v will give:
**
**		100*(5.0-Vod-Vih)/(3.3-Vod-Vih) = 194.4
**
**	Thus, we need to work this into our calibration model, specifically for
**	the value of calib_range:
**	
**	env_factor = (5.0-Vod-Vih)/(Voh-Vih)
**
**	Where:
**	
**	Voh = Vcc-Vod
**
**	For the ADC, we set the reference voltage to Vcc and set the measured
**	voltage to the bandgap reference, which is around 1.1v. That gives us
**	something like this:
**
**	ADC_OUTPUT = Vint*ADC_MAX/Vcc
**	
**	Since the voltage present at I/O pin that is driven high is less than
**	the I/O voltage(by 0.8 volts), we get something closer to this:
**
**	ADC_OUTPUT = Vint*ADC_MAX/Voh
**
**	Solving for Voh gives us:
**
**	ADC_OUTPUT*Voh = Vint*ADC_MAX
**	...
**	Voh = Vint*ADC_MAX/ADC_OUTPUT
**
**	We plug that into the calib_range' equation from above, and we get:
**
**	env_factor = (5.0-Vod-Vih)/(Vint*ADC_MAX/ADC_OUTPUT-Vih)
**
**	Temperature will affect the moisture readings in complicated ways, for
**	two reasons:
**	
**		* Ice doesn't conduct electricity, so readings should start to
**		  quickly decrease below ~4¡C.
**		* The capacity of the holding capacitor will increase at colder
**		  temps, and decrease at higher temps. This behavior is the inverse
**		  of the ice issue above.
**
**	More to come.
*/

struct msensor_e2_t msensor_e2 EEMEM = {
	.alarm_high = 0xFF,
	.alarm_low = 0x00,
	.config = 0x00,
	.calib_range = 0x1F,
	.calib_offset = 4,
};

char device_name[32] EEMEM = "";
