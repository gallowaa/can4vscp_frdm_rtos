/*HEADER**********************************************************************
*
* Copyright 2008 Freescale Semiconductor, Inc.
* Copyright 1989-2008 ARC International
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale MQX RTOS License
* distributed with this Material.
* See the MQX_RTOS_LICENSE file distributed for more details.
*
* Brief License Summary:
* This software is provided in source form for you to use free of charge,
* but it is not open source software. You are allowed to use this software
* but you cannot redistribute it or derivative works of it in source form.
* The software may be used only in connection with a product containing
* a Freescale microprocessor, microcontroller, or digital signal processor.
* See license agreement file for full license terms including other
* restrictions.
*****************************************************************************
*
* Comments:
*
*   This file contains the source for the hello2 example program.
*
*
*END************************************************************************/

#include <stdio.h>
#include <mqx.h>
#include <bsp.h>

#include "main.h"

// ***************************************************************************
// 								Definitions
// ***************************************************************************

//#define VSCP_REG_IN_FLASH  // Un-comment this to use FLASH instead of EEPROM for the VSCP registers
//#define DO_PRINT			 // Un-comment this to print out debug info

#define LPTMR_INSTANCE      (0U)
#define BOARD_PIT_INSTANCE  (0U)
#define ADC_0				(0U)

/* Task IDs */
#define HELLO_TASK  	5
#define WORLD_TASK  	6
#define HW_INIT_TASK 	7

extern void hello_task(uint32_t);
extern void world_task(uint32_t);
extern void hw_init_task();


const TASK_TEMPLATE_STRUCT  MQX_template_list[] =
{
   /* Task Index,   Function,   Stack,  Priority, Name,     Attributes,          Param, Time Slice */
    { WORLD_TASK,   world_task, 700,   9,        "world",  MQX_AUTO_START_TASK, 0,     0 },
    { HELLO_TASK,   hello_task, 700,   8,        "hello",  0,                   0,     0 },
	{ HW_INIT_TASK, hw_init_task, 700,   8,      "hw_init",  0,                 0,     0 },

    { 0 }
};

/*TASK*-----------------------------------------------------
*
* Task Name    : main_task
* Comments     :
*    Should initialize all other tasks
*
*END*-----------------------------------------------------*/
void main_task(uint32_t initial_data) {

	_task_id hw_init_task_id;

	// Init mcu and peripherals
	hw_init_task_id = _task_create(0, HW_INIT_TASK,0);
	if (hw_init_task_id == MQX_NULL_TASK_ID) {
	      printf ("\n Could not create hw_init_task\n");
	}

#ifdef NOT_READY_YET

		//can take this out once vscp fully implemented
		vscp_initledfunc = VSCP_LED_BLINK1; //0x02

		// Check VSCP persistent storage and
		// restore if needed

		if( !vscp_check_pstorage() ) {

			spi_eeprom_guid_init();

			// Spoiled or not initialized - reinitialize
			init_app_eeprom();
			init_app_ram();     // Needed because some ram positions
								// are initialized from EEPROM
		}

		// Initialize vscp
		vscp_init();

		while(1)
		{

			vscp_imsg.flags = 0;
			vscp_getEvent(); 		// fetch one vscp event -> vscp_imsg struct

			switch ( vscp_node_state ) {

			case VSCP_STATE_STARTUP: // Cold/warm reset

				// Get nickname from EEPROM
				if (VSCP_ADDRESS_FREE == vscp_nickname) {
					// new on segment need a nickname
					vscp_node_state = VSCP_STATE_INIT;
				} else {
					// been here before - go on
					vscp_node_state = VSCP_STATE_ACTIVE;
					vscp_goActiveState();
				}
				break;

			case VSCP_STATE_INIT: // Assigning nickname
				vscp_handleProbeState();
				break;

			case VSCP_STATE_PREACTIVE:  // Waiting for host initialisation
				vscp_goActiveState();
				break;

			case VSCP_STATE_ACTIVE:     // The normal state

				// Check for incoming event?
				if (vscp_imsg.flags & VSCP_VALID_MSG) {

					if ( VSCP_CLASS1_PROTOCOL == vscp_imsg.vscp_class  ) {

						// Handle protocol event
						vscp_handleProtocolEvent();

					}
					// doDM();
				}
				break;

			case VSCP_STATE_ERROR: // Everything is *very* *very* bad.
				vscp_error();
				break;

			default: // Should not be here...
				vscp_node_state = VSCP_STATE_STARTUP;
				break;

			}

			// do a measurement if needed
			if ( measurement_clock > 1000 ) {

				PRINTF("Transmit send errors: %d\r", numErrors);

				measurement_clock = 0;

				// Do VSCP one second jobs
				vscp_doOneSecondWork();
				seconds++;
				// sendTimer++; // sendTimer should be incremented in the 1ms interrupt


				// Temperature report timers are only updated if in active
				// state guid_reset
				if ( VSCP_STATE_ACTIVE == vscp_node_state ) {

					// Do VSCP one second jobs

					/* temperature is done here, it will check seconds_temp variable
					 * so that it sends the event as per the REPORT_INTERVAL specified */
					doApplicationOneSecondWork();
					seconds_temp++; // Temperature report timers are only updated if in active state
					seconds_accel++;

				}
				doWork();
			}

			// Timekeeping - Can replace this with RTC which defines structs for keeping track of this automatically
			if ( seconds > 59 ) {

				seconds = 0;
				minutes++;

				if ( minutes > 59 ) {
					minutes = 0;
					hours++;
				}

				if ( hours > 23 ) hours = 0;

			}
		}

		vscp_handleProbeState(); //just a test

		/* Never leave main */
		return 0;
#endif
}


/*TASK*-----------------------------------------------------
*
* Task Name    : hardware_init_task
* Comments     :
*    takes place of init() function in vscp paris implementation
*    task blocks forever
*
*END*-----------------------------------------------------*/
void hw_init_task() {

	/* enable clock for PORTs */
	CLOCK_SYS_EnablePortClock(PORTA_IDX); /*! */
	CLOCK_SYS_EnablePortClock(PORTB_IDX); /*! The CAN pins are on port B */
	CLOCK_SYS_EnablePortClock(PORTC_IDX); /*! */
	CLOCK_SYS_EnablePortClock(PORTD_IDX); /*! The SPI pins are on port D */
	CLOCK_SYS_EnablePortClock(PORTE_IDX); /*! The I2C pins are on port E */

	/* Init board clock */
	BOARD_ClockInit();
	dbg_uart_init();

	configure_spi_pins(0); 	// Configure SPI pins for talking to EEPROM w/ MAC address
	configure_i2c_pins(0);  // Configure IIC pins for accelerometer
	configure_can_pins(0);  // Configure CAN pins

	OSA_Init();             //FXOS_Init seems to depend on this, so does OSA_TimeDelay(ms)

#ifdef NOT_READY_YET
	init_spi();				// For EEPROM
	init_flexcan();			// For vscp events

	// Initialize the eCompass.
	i2cDevice.i2cInstance = BOARD_I2C_COMM_INSTANCE;
	FXOS_Init(&i2cDevice, NULL);

#ifdef VSCP_REG_IN_FLASH
	init_flash();
#endif

	STATUS_LED_EN; /* LED1_EN */
	CLOCK_PIN_EN;  /* scope this pin to test the 1 ms clock pulse width */
	INIT_BTN_EN;   /* init sw2 as input */

	// Enable a gpio for taking the STB pin on the CAN PHY low
	CAN0_STB_EN;
	CAN0_STB_LO;

	// Calibrate param Temperature sensor
	calibrateParams(); /* <- taken from adc_low_power_frdmk64f demo */

	init_adc(ADC_0);

	//init_pit(channelConfig0, channelConfig1);
	init_pit();

#endif

	_task_block();
}

/*TASK*-----------------------------------------------------
*
* Task Name    : world_task
* Comments     :
*    This task creates hello_task and then prints " World ".
*
*END*-----------------------------------------------------*/

void world_task
   (
      uint32_t initial_data
   )
{
   _task_id hello_task_id;

   hello_task_id = _task_create(0, HELLO_TASK, 0);
   if (hello_task_id == MQX_NULL_TASK_ID) {
      printf ("\n Could not create hello_task\n");
   } else {
      printf(" World \n");
   }

   _task_block();

}


/*TASK*-----------------------------------------------------
*
* Task Name    : hello_task
* Comments     :
*    This task prints " Hello".
*
*END*-----------------------------------------------------*/

void hello_task
   (
      uint32_t initial_data
   )
{

   printf("\n Hello\n");
   _task_block();

}

/* EOF */
