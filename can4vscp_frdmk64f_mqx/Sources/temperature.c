

#include "main.h"
#include "temperature.h"

uint8_t Celsius2Kelvin(int celsius) {
	return 0; // return (celsius + 275) /* todo: need MSB + LSB */
}

uint8_t Celsius2Fahrenheit(int celsius) {

	 float temp = 0;
	 temp = (celsius * 9 / (float) 5.0) + 32;
	 //PRINTF("%d F\r\n", (uint8_t) temp);
	 return (uint8_t) temp;
}

