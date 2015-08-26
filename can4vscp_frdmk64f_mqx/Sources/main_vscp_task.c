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
 *   This file contains the source for the FlexCAN example program.
 *
 *   NOTE: This is a two node test. It requires the software to be loaded
 *   onto two boards, one board programmed as NODE 1 and the other programmed
 *   as NODE 2.  A properly terminated CAN cable (120 ohms on either end) is
 *   used to connect both boards together.  When running, the boards will
 *   exchange a CAN message once per second, and the following output
 *   should be repetitively displayed (where <x> and <y> increment):
 *
 *       Data transmit: <x>
 *       FLEXCAN tx update message. result: 0x0
 *       Received data: <y>
 *       ID is: 0x321
 *       DLC is: 0x1
 *
 *
 *END************************************************************************/

//#define RECEIVE_ONLY

#include <mqx.h>
#include <bsp.h>
#include <lwevent.h>
//#include <time.h>

#include "main.h"

#ifndef PSP_MQX_CPU_IS_KINETIS
#include <flexcan.h>
#endif


#if ! BSPCFG_ENABLE_IO_SUBSYSTEM
#error This application requires BSPCFG_ENABLE_IO_SUBSYSTEM defined non-zero in user_config.h. Please recompile BSP with this option.
#endif


#ifndef BSP_DEFAULT_IO_CHANNEL_DEFINED
#error This application requires BSP_DEFAULT_IO_CHANNEL to be not NULL. Please set corresponding BSPCFG_ENABLE_TTYx to non-zero in user_config.h and recompile BSP with this option.
#endif


/* Structures holding information about specific pin */
LWGPIO_STRUCT CAN_stb;
LWGPIO_STRUCT led1;

HWTIMER hwtimer1;                               // hwtimer handle
LWEVENT_STRUCT event; // Takes place of sync struct in KSDK

volatile uint32_t ms_counter = 0;

/*hwtimer example defines*/
// #define HWTIMER1_FREQUENCY  10          //frequency set to hwtimer 1
#define HWTIMER1_PERIOD     1000     //period set to 1ms to hwtimer 1

/* Task define */
#define MAIN_TASK   1
#define TX_TASK     2
#define RX_TASK     3

/* Tasks */

extern void Main_Task(uint32_t);
extern void Tx_Task(uint32_t);
extern void Rx_Task(uint32_t);

/* Task template list */
TASK_TEMPLATE_STRUCT MQX_template_list[] = {
		{ MAIN_TASK, Main_Task, 1000L, 8L, "Main task", MQX_AUTO_START_TASK},
		{ TX_TASK, Tx_Task, 1000L, 7L, "TX task", 0, 0, 0},
		{ RX_TASK, Rx_Task, 1000L, 7L, "RX task", 0, 0, 0},
		{ 0L, 0L, 0L, 0L, 0L, 0L }
};

#if PSP_MQX_CPU_IS_KINETIS
void MY_FLEXCAN_ISR
(
		/* [IN] FlexCAN base address */
		void   *can_ptr
)
{
	volatile CAN_MemMapPtr        can_reg_ptr;
	volatile uint32_t                               tmp_reg;
	volatile uint32_t                               temp;

	can_reg_ptr = (CAN_MemMapPtr)can_ptr;

	/* get the interrupt flag */
	tmp_reg = (can_reg_ptr->IFLAG1 & CAN_IMASK1_BUFLM_MASK);
	// check Tx/Rx interrupt flag and clear the interrupt
	if(tmp_reg){
		/* clear the interrupt and unlock message buffer */
		/* Start CR# 1751 */
		_lwevent_set(&event, tmp_reg);
		can_reg_ptr->IFLAG1 |= tmp_reg;
		/* End CR# 1751 */
		temp = can_reg_ptr->TIMER;
	}/* Endif */

	// Clear all other interrupts in ERRSTAT register (Error, Busoff, Wakeup)
	tmp_reg = can_reg_ptr->ESR1;
	if(tmp_reg & FLEXCAN_ALL_INT){
		/* Start CR# 1751 */
		can_reg_ptr->ESR1 |= (tmp_reg & FLEXCAN_ALL_INT);
		/* End CR# 1751 */
	} /* Endif */

	return;
}
#else

#endif

/*FUNCTION*----------------------------------------------------------------*
 * Function Name  : hwtimer1_callback
 * Returned Value : void
 * Comments :
 *       Callback for hwtimer1
 *END*--------------------------------------------------------------------*/
static void hwtimer1_callback(void *p)
{
	measurement_clock++;
	vscp_timer++;
	vscp_configtimer++;
	measurement_clock++;
	timeout_clock++;
	sendTimer++;

	vscp_statuscnt++;

	if( (vscp_statuscnt > 100) && (vscp_initledfunc == VSCP_LED_BLINK1) ){
		STATUS_LED_TOGGLE; // blink the vscp status led
		vscp_statuscnt = 0;
	}

	else if (VSCP_LED_ON == vscp_initledfunc){
		STATUS_LED_ON;
	}
}

static void vscp_timer_init() {

	// Initialization of hwtimer1
	printf("Initialization of hwtimer1   :");
	if (MQX_OK != hwtimer_init(&hwtimer1, &BSP_HWTIMER1_DEV, BSP_HWTIMER1_ID, (BSP_DEFAULT_MQX_HARDWARE_INTERRUPT_LEVEL_MAX + 1)))
	{
		printf(" FAILED!\n");
	}
	else
	{
		printf(" OK\n");
	}
	printf("Try to set period %d us to hwtimer1\n", HWTIMER1_PERIOD);
	hwtimer_set_period(&hwtimer1, BSP_HWTIMER1_SOURCE_CLK, HWTIMER1_PERIOD);

	printf("Read frequency from hwtimer1 : %d Hz\n", hwtimer_get_freq(&hwtimer1));
	printf("Read period from hwtimer1    : %d us\n", hwtimer_get_period(&hwtimer1));
	printf("Read modulo from hwtimer1    : %d\n", hwtimer_get_modulo(&hwtimer1));

	// Register isr for hwtimer1
	printf("Register callback for hwtimer1\n");
	hwtimer_callback_reg(&hwtimer1,(HWTIMER_CALLBACK_FPTR)hwtimer1_callback, NULL);
	// Start hwtimer1
	printf("Start hwtimer1\n");
	hwtimer_start(&hwtimer1);
}

/*TASK*-----------------------------------------------------------
 *
 * Task Name : Main_Task
 * Comments :
 *
 *
 *END*-----------------------------------------------------------*/
void Main_Task(uint32_t parameter)
{
	uint32_t result;
	_task_id     created_task;

	/*
	 * Display all exceptions to terminal
	 */
	_int_install_unexpected_isr();

	//////////////////////////////////

	GPIO_init();
	vscp_timer_init();
	FLEXCAN_Init();

	///////////////////////////////////

	/* Set up an event group */
	result = _lwevent_create(&event, LWEVENT_AUTO_CLEAR);
	if (result != MQX_OK) {
		printf("\nCannot create lwevent");
	}

	created_task = _task_create(0, RX_TASK, 0);
	if (created_task == MQX_NULL_TASK_ID)
	{
		printf("\nRx task: task creation failed.");
	}


	created_task = _task_create(0, TX_TASK, 0);
	if (created_task == MQX_NULL_TASK_ID)
	{
		printf("\nTx task: task creation failed.");
	}

	/* Start FLEXCAN */
	result = FLEXCAN_Start(CAN_DEVICE);
	printf("\nFLEXCAN started. result: 0x%lx", result);

	//can take this out once vscp fully implemented
	vscp_initledfunc = VSCP_LED_BLINK1; //0x02

	// Check VSCP persistent storage and
	// restore if needed

	if( !vscp_check_pstorage() ) {

		//TODO: spi driver
		// spi_eeprom_guid_init();

		// Spoiled or not initialized - reinitialize
		// init_app_eeprom();
		init_app_ram();     // Needed because some ram positions
		// are initialized from EEPROM
	}

	// Initialize vscp
	vscp_init();

	// Currently, this task never blocks, but it is at a higher priority than the Rx & Tx tasks, so it can be pre-empted.

	while(1) {

		// do a measurement if needed
		if ( measurement_clock > 1000 ) {
			measurement_clock = 0;

			// signal to tx_task, send a msg
			_lwevent_set(&event, TX_mailbox_num);
		}

#ifdef VSCP

		/* getEvent is taken care of by rx_task */
		// vscp_imsg.flags = 0;
		// vscp_getEvent(); 		// fetch one vscp event -> vscp_imsg struct

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

			//PRINTF("Transmit send errors: %d\r", numErrors);

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
				//doApplicationOneSecondWork();
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

#endif

	}

	//vscp_handleProbeState(); // just a test
}




