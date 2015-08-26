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
 *
 * main.h
 *
 *  Created on: Jun 5, 2015
 *      Author: Angus Galloway
 */

#ifndef SOURCES_MAIN_H_
#define SOURCES_MAIN_H_

#ifndef BSP_CAN_DEVICE
    //#error This application requires BSP_CAN_DEVICE defined non-zero in [board name].h. Please recompile BSP with this option.
#else
#endif

#define CAN_DEVICE  BSP_CAN_DEVICE


#define MY_EVENT_GROUP 123

#define DEBUG				 // Reads eeprom after initializing it in init_app_eeprom();

/** Standard C libs **/

#include <stdbool.h>

#ifdef MQX
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <math.h>
#endif

/** vscp_firmware **/
#include <vscp_firmware.h>				/*! the official vscp firmware library */
#include <vscp_class.h>					/*! defines vscp classes I & II  */
#include <vscp_type.h>					/*! defines vscp event types, as described by official spec */

/* SDK includes */
#ifdef MQX
#include "pin_mux.h"					/*! configure pins with typical use cases */
#include "board.h"						/*! miscellaneous board specific funcs & defs */
#include "fsl_os_abstraction.h"			/*! accel/mag driver for "angle" event */
#include "fsl_device_registers.h"		/*!  */
#include "fsl_clock_manager.h"			/*!  */
#include "fsl_debug_console.h"			/*! makes printf just work */
#include "fsl_lptmr_driver.h"			/*! LPTMR = Low Power Timer, used as time base for OSA functions, like TimeDelay(ms) */
#include "fsl_pit_driver.h"				/*! PIT = Periodic Interrupt Timer, used for vscp clk & adc trigger (up to 4 channels) */
#include "fsl_flexcan_driver.h"			/*! can driver for send/receive vscp events */
#include "fsl_uart_driver.h"			/*! serial  */
#include "fsl_dspi_shared_function.h"	/*! data spi for talking to eeprom */
#include "fsl_device_registers.h"		/*! makes sure the right CPU register defs are included */
//#include "fsl_fxos8700_driver.h"		/*! accel/mag driver for "angle" event */
#include "fsl_adc16_driver.h"			/*! adc driver for "temperature" event */
#include "fsl_smc_hal.h"				/*! SMC = System Mode Controller */
#include "fsl_pmc_hal.h"				/*! Used for getting reference voltage for adc */
#endif

#define DUMMY_DATA 0x55
#define NOT_IMPLEMENTED 0

/* VSCP application level */
#define PAGES 1

/* VSCP core features */
/* There is enough space on the 2kbit eeprom to also use it for the vscp core registers
 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// Note! These #defines specify the actual addresses of the VSCP configuration registers in the attached SPI
//// eeprom. However, note that in the Register Abstraction Model (aka the interface exposed to the outside world in
//// vscp_firmware.c) register space is presented as a block of 256 registers. The bottom half (0x00 to 0x7F)
//// contains application registers, while the top half (0x80 to 0xFF) contains these VSCP configuration registers.
//// You can have up to 65,535 "pages" of 128 bytes for application specific registers, while the VSCP registers are
//// not paged. The actual layout in memory does not have to match this layout presented to the outside world.
//// The vscp_firmware automatically reduces the address passed in from a register read/write to correspond to
//// what is defined here.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define VSCP_EEPROM_BOOTLOADER_FLAG		0x00 //reserved for bootloader flag

#define VSCP_EEPROM_NICKNAME 			0x01	// Persistent nickname id storage - NOTE! For a GUID based on EUI-48, the nickname is also in the 2 LSBs of GUID.
#define VSCP_EEPROM_SEGMENT_CRC			0x02	// Persistent segment crc storage
#define VSCP_EEPROM_CONTROL 			0x03	// Persistent control byte

#define VSCP_EEPROM_REG_USERID 			0x04
#define VSCP_EEPROM_REG_USERID1 			0x05
#define VSCP_EEPROM_REG_USERID2 			0x06
#define VSCP_EEPROM_REG_USERID3 			0x07
#define VSCP_EEPROM_REG_USERID4 			0x08

// The following can be stored in flash or eeprom

#define VSCP_EEPROM_REG_MANUFACTUR_ID0 	0x09
#define VSCP_EEPROM_REG_MANUFACTUR_ID1 	0x0A
#define VSCP_EEPROM_REG_MANUFACTUR_ID2 	0x0B
#define VSCP_EEPROM_REG_MANUFACTUR_ID3 	0x0C

#define VSCP_EEPROM_REG_MANUFACTUR_SUBID0	0x0D
#define VSCP_EEPROM_REG_MANUFACTUR_SUBID1	0x0E
#define VSCP_EEPROM_REG_MANUFACTUR_SUBID2	0x0F

/* Delete this so that we can start the guid at 0x10 instead of 0x11. This way it resides in
 * a single page xxxx 0000 to xxxx 1111 and can be written in one API call without having to
 * issue two separate WRITE commands to the eeprom
 */

//#define VSCP_EEPROM_REG_MANUFACTUR_SUBID3	0x10

// The following can be stored in program ROM or EEPROM
// AG: I am keeping this in eeprom since the EUI-48
// 	   comes from eeprom already.


#define VSCP_EEPROM_REG_GUID 			0x10 	// Start of GUID MSB of 16
												//		 0x10 - 0x1F

#define VSCP_EEPROM_REG_DEVICE_URL		0x20	// Start of Device URL storage
                                                // 		0x20 - 0x3F

#define VSCP_EEPROM_END                 0x40	// marks end of VSCP EEPROM usage
                                                //   (next free position)

/** VSCP Application Registers **/
/* Note effective address is created by first adding VSCP_EEPROM_END **/

#define REG_FRDM_ZONE					0x00
#define REG_FRDM_SUBZONE				0x01

#define REG_TEMP0_REPORT_INTERVAL		0x02

#define REG_ACCEL0_REPORT_INTERVAL		0x03

#define REG_ACCEL0_HIGH_ALARM			0x06
#define REG_TEMP0_HIGH_ALARM			0x07
#define REG_TEMP0_LOW_ALARM				0x08

#define REG_TEMP0_CONTROL				0x09
#define REG_ACCEL0_CONTROL				0x10

/** VSCP Application reg defines **/

#define DEFAULT_REPORT_INTERVAL_TEMP0 	2 		//10 Seconds
#define DEFAULT_REPORT_INTERVAL_ACCEL0 	1		//5 Seconds
#define DEFAULT_ACCEL0_HIGH_ALARM		15		//15 degrees
#define DEFAULT_TEMP0_HIGH_ALARM		32		//32 Degrees C
#define DEFAULT_TEMP0_LOW_ALARM			28		//28 Degrees C
#define MODULE_LOW_ALARM 				1
#define MODULE_HIGH_ALARM 				2

/** Zone & subzone **/
#define ZONE 							5
#define SUBZONE 						9


/* TODO: These will change to use _lwgpio */
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//  	GPIO definitions
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/** VSCP Status LED **/
//#define STATUS_LED_EN 			LED1_EN
#define STATUS_LED_ON 			lwgpio_set_value(&led1, LWGPIO_VALUE_LOW); /* set pin to 0 */
//#define STATUS_LED_OFF 			LED1_OFF
#define STATUS_LED_TOGGLE 		lwgpio_toggle_value(&led1)


#define INIT_BTN_EN 		(GPIO_DRV_InputPinInit(&switchPins[0]))					 /*!< init sw2 as input */


/** FOR 1ms clock check on scope **/

#define CLOCK_PIN_EN  		(GPIO_DRV_OutputPinInit(&gpioPins[0]))  				  /*!< clk pin en */
#define CLOCK_PIN_HI	 	(GPIO_DRV_WritePinOutput(gpioPins[0].pinName, 1))         /*!< clk pin high level */
#define CLOCK_PIN_LO 		(GPIO_DRV_WritePinOutput(gpioPins[0].pinName, 0))         /*!< clk pin low level */
#define CLOCK_PIN_TOGGLE 	(GPIO_DRV_TogglePinOutput(gpioPins[0].pinName))        	  /*!< clk toggle */

/** FOR SPI0 **/

//Don't need a gpio anymore. The problem was that .isChipSelectContinuous was set to false. Fixed by setting to true.

//#define SPI0_CS_EN  		(GPIO_DRV_OutputPinInit(&gpioPins[1]))  				  /*!< Enable target SPI0 CS gpio pin */
//#define SPI0_CS_DESELECT 	(GPIO_DRV_WritePinOutput(gpioPins[1].pinName, 1))         /*!< Turn off cs */
//#define SPI0_CS_SELECT 		(GPIO_DRV_WritePinOutput(gpioPins[1].pinName, 0))         /*!< Turn on cs */


/** FOR FRDM-CAN-VSCP Shield CAN PHY Standby pin **/

#define CAN0_STB_EN  		(GPIO_DRV_OutputPinInit(&gpioPins[2]))  				  /*!< Enable STB pin, connected to PTB9*/
#define CAN0_STB_HI	 		(GPIO_DRV_WritePinOutput(gpioPins[2].pinName, 1))         /*!< CAN PHY is in low power mode */
#define CAN0_STB_LO 		(GPIO_DRV_WritePinOutput(gpioPins[2].pinName, 0))         /*!< CAN PHY is in normal mode */



///////////////////////////////////////////////////////////////////////////////
// 						Prototypes
///////////////////////////////////////////////////////////////////////////////

extern uint32_t RelocateFunction(uint32_t dest, uint32_t size, uint32_t src);

extern uint8_t sendTimer;  // Timer for CAN send

void doApplicationOneSecondWork(void);

void doWork(void);


/* test_vscp_functions_al.c  */

/*!
 * @brief verify functionality of the custom defined vscp functions
 *        vscp_firmware.h @ line 457
 */
void test_vscp_externals();

void vscp_FLASHFlush();

void init_app_eeprom();

void init_app_ram( void );

void doDM( void );

int8_t sendAccelEvent( void );


typedef struct accelData {
    uint8_t xAngle;
    uint8_t yAngle;
} accel_data_t;

///////////////////////////////////////////////////////////////////////////////
// 						 Global variables
///////////////////////////////////////////////////////////////////////////////

/* vscp globals */
volatile uint32_t timeout_clock;        	// Clock used for timeouts
volatile uint32_t measurement_clock; // Clock for measurements
uint8_t sendTimer;  // Timer for CAN send
uint8_t seconds;    // counter for seconds
uint8_t minutes;    // counter for minutes
uint8_t hours;      // Counter for hours

/* vscp app globals */
uint8_t current_temp;
accel_data_t accelData;
uint8_t temp0_low_alarm;
uint8_t temp0_high_alarm;
uint8_t accel0_high_alarm;

/* other globals */

uint8_t seconds_temp;        				// timer for temp event
uint8_t seconds_accel;        				// timer for accel event
uint32_t data_len_code;
uint32_t bit_rate;
uint32_t TX_identifier;
uint32_t RX_identifier;
uint32_t TX_remote_identifier;
uint32_t RX_remote_identifier;
uint32_t format;
uint32_t interrupt;
uint32_t TX_mailbox_num;
uint32_t RX_mailbox_num;
uint32_t TX_remote_mailbox_num;
uint32_t RX_remote_mailbox_num;
uint32_t bit_timing0;
uint32_t bit_timing1;
uint32_t frequency;
uint32_t flexcan_mode;
uint32_t flexcan_error_interrupt;


/* Misc MQX Functions */
extern void print_result(uint32_t);
extern void get_string(char *,uint32_t *);
extern void FLEXCAN_Init();
extern uint32_t FLEXCAN_Tx_Init();
extern uint32_t FLEXCAN_Rx_Init();

extern void GPIO_init();

void MY_FLEXCAN_ISR(void *);


#endif /* SOURCES_MAIN_H_ */
