/*
 * gpio.c
 *
 *  Created on: Aug 24, 2015
 *      Author: Angus
 */

#include <mqx.h>
#include <bsp.h>
#include <lwevent.h>

/* Structures holding information about specific pin */
extern LWGPIO_STRUCT CAN_stb;
extern LWGPIO_STRUCT led1;

void GPIO_init() {

	/******************************************************************************
	       Open the BSP_PINB9 pin as output and drive the output level LOW.
	 ******************************************************************************/
	/* initialize lwgpio handle (CAN_stb)*/
	if (!lwgpio_init(&CAN_stb, BSP_ARDUINO_GPIO2, LWGPIO_DIR_OUTPUT, LWGPIO_VALUE_NOCHANGE))
	{
		printf("Initializing CAN_stb as output failed.\n");
		_task_block();
	}
	/* swich pin functionality (MUX) to GPIO mode */
	lwgpio_set_functionality(&CAN_stb, BSP_ARDUINO_GPIO2_MUX_GPIO);

	/* write logical 0 to the pin */
	lwgpio_set_value(&CAN_stb, LWGPIO_VALUE_LOW); /* set pin to 0 */

	/******************************************************************************
		       Open the BSP_LED1 pin as output
	 ******************************************************************************/

	/* initialize lwgpio handle (led1) for BSP_LED1 pin
	 * (defined in mqx/source/bsp/<bsp_name>/<bsp_name>.h file)
	 */
	if (!lwgpio_init(&led1, BSP_LED1, LWGPIO_DIR_OUTPUT, LWGPIO_VALUE_NOCHANGE))
	{
		printf("Initializing LED1 GPIO as output failed.\n");
		_task_block();
	}
	/* swich pin functionality (MUX) to GPIO mode */
	lwgpio_set_functionality(&led1, BSP_LED1_MUX_GPIO);
}
