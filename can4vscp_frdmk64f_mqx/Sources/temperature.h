/*
 * temperature.h
 *
 *  Created on: Jul 17, 2015
 *      Author: Angus
 */

#ifndef SOURCES_VSCP_TEMPERATURE_H_
#define SOURCES_VSCP_TEMPERATURE_H_


#define TEMP_UNIT_KELVIN            0
#define TEMP_UNIT_CELSIUS           1
#define TEMP_UNIT_FAHRENHEIT        2

// Default hysteresis
#define DEFAULT_HYSTERESIS                  2


//
// Defaults
//

// Default value for control register
// bit 0,1 - 01 = Celsius
// Bit 2 - 0 Reserved bit
// Bit 3 = 0 Low alarm disabled
// Bit 4 = 0 High alarm disabled
// Bit 5 = 0 Alarm is sent
// Bit 6 = 0 Normal TurnOn/Off
// Bit 7 = 0 Non continues alarm
//
#define DEFAULT_CONTROL_REG                 0b00000001

//
// Configuration bits
//
// Default value for control register
// bit 0,1 - Temperature
// Bit 2 - Reserved bit
// Bit 3 = Low alarm eanble
// Bit 4 = High alarm enable
// Bit 5 = Enable TurnOn/Off
// Bit 6 = Enable invert TurnOn/TurnOff
// Bit 7 = Continues alarm
#define CONFIG_ENABLE_LOW_ALARM         (1<<3)
#define CONFIG_ENABLE_HIGH_ALARM        (1<<4)
#define CONFIG_ENABLE_TURNX             (1<<5)
#define CONFIG_ENABLE_TURNON_INVERT     (1<<6)
#define CONFIG_ENABLE_CONTINOUS_EVENTS  (1<<7)


int8_t sendTempEvent(uint8_t i);
void setEventData(int v, unsigned char unit);
uint8_t Celsius2Kelvin(int celsius);
uint8_t Celsius2Fahrenheit(int celsius);


#endif /* SOURCES_VSCP_TEMPERATURE_H_ */
