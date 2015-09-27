/*
 * i2c_accel.c
 *
 *  Created on: Sep 12, 2015
 *      Author: Angus
 */


/*HEADER**********************************************************************
*
* Copyright 2008 Freescale Semiconductor, Inc.
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
*   See https://community.freescale.com/docs/DOC-104587 for a full MQX
*   example that calibrates magnetometer + accel
*
*
*END************************************************************************/


#include <mqx.h>
#include <bsp.h>
//#include "FXOS8700CQ.h"  // Include declarations of FXOS8700CQ registers
#include "I2C_sensor_polled.h"  // Include polled I2C driver for sensor
#include <math.h>  // Include mathematical functions
#include "fxos_def.h"
#include "main.h"

/* Global variables */


LWGPIO_STRUCT ledR, ledG, ledB, btn1;
LWEVENT_STRUCT_PTR lwevent1_ptr;
uint8_t result;
uint8_t buffer[1];
uint8_t AccelMagData[12];
int16_t Xout_Accel_14_bit, Yout_Accel_14_bit, Zout_Accel_14_bit;
int16_t Xout_Mag_16_bit, Yout_Mag_16_bit, Zout_Mag_16_bit;
float Xout_g, Yout_g, Zout_g;
float Xout_uT, Yout_uT, Zout_uT;
float Heading;
// uint8_t DataReady;
LWSEM_STRUCT lwsem;
extern MQX_FILE_PTR fd_i2c;


extern void I2C_init();
extern uint8_t readEEPROM( uint8_t addr);

void FXOS8700CQ_Init();
int16_t absolute_val(int16_t val);


/*TASK*-----------------------------------------------------
*
* Task Name    : Accel_task
* Comments     :
*    This task connects to FXOS8700CQ
*
*END*-----------------------------------------------------*/
void Accel_Task(uint32_t initial_data)
{
	int16_t xAngle, yAngle, zAngle;
	uint8_t rcvdByte=0;
	uint32_t delay=0;

#if SENSOR_USE_INTERRUPT
    int1_init();  // initialize interrupt signal from sensor
#endif
    I2C_init();  // initialize I2C driver
    FXOS8700CQ_Init();  // initialize sensor

    while (1)
    {
        rcvdByte = readEEPROM( VSCP_EEPROM_END + REG_ACCEL0_REPORT_INTERVAL);
        delay = (uint32_t) (1000* rcvdByte);

#ifdef PRINT_ALL
        printf("ACCEL0_REPORT_INTERVAL = %d s, it should be (%d) \r\n", rcvdByte, DEFAULT_REPORT_INTERVAL_ACCEL0);
        printf("ACCEL0_REPORT_INTERVAL = %d ms, it should be (%d) \r\n", delay, DEFAULT_REPORT_INTERVAL_ACCEL0*1000);
#endif

        _time_delay(delay);

#if SENSOR_USE_INTERRUPT

    	//_time_delay(500);
        _lwsem_wait(&lwsem);
#else
        do
        {
            i2c_read_register_polled(fd_i2c, STATUS_REG, buffer, 1);  // Read data status register
        } while (!(buffer[0] && ZYXDR_MASK));  // while new set of accelerometer data is ready
#endif
        {
            i2c_read_register_polled(fd_i2c, OUT_X_MSB_REG, AccelMagData, 12);  // Read data output registers 0x01-0x06 and 0x33 - 0x38

            Xout_Accel_14_bit = ((int16_t)(AccelMagData[0] << 8 | AccelMagData[1]));  // Compute 14-bit X-axis acceleration output value
            Yout_Accel_14_bit = ((int16_t)(AccelMagData[2] << 8 | AccelMagData[3]));  // Compute 14-bit Y-axis acceleration output value
            Zout_Accel_14_bit = ((int16_t)(AccelMagData[4] << 8 | AccelMagData[5]));  // Compute 14-bit Z-axis acceleration output value

            // Convert raw data to angle (normalize to 0-90 degrees).  No negative angles.
            xAngle = absolute_val((int16_t)(Xout_Accel_14_bit * 0.011));
            yAngle = absolute_val((int16_t)(Yout_Accel_14_bit * 0.011));
            zAngle = absolute_val((int16_t)(Zout_Accel_14_bit * 0.011));

            // Print results
            printf("accel_task: X=%d, Y=%d , Z=%d \r\n", xAngle, yAngle, zAngle);
            //printf("Magnetometer Heading=%f \n", Heading);

            accelData.xAngle = (uint8_t) xAngle;
            accelData.yAngle = (uint8_t) yAngle;
            accelData.zAngle = (uint8_t) zAngle;
        }
    }
}

void FXOS8700CQ_Init(void)
{
    i2c_read_register_polled(fd_i2c, WHO_AM_I_REG, buffer, 1);  // WHO_AM_I, result should be 0xC7
    i2c_write_register_polled(fd_i2c, CTRL_REG2, 0x40);  // write 0x00 to accelerometer control register 1 to place FXOS8700CQ into standby mode
    _time_delay(1);  // time delay 1ms

    //i2c_write_register_polled(fd_i2c, XYZ_DATA_CFG_REG, 0x00);  // +/-2g range with 0.244mg/LSB
    i2c_write_register_polled(fd_i2c, XYZ_DATA_CFG_REG, FULL_SCALE_4G);

    //i2c_write_register_polled(fd_i2c, M_CTRL_REG1, 0x1F);  // Hybrid mode (accelerometer + magnetometer), max OSR
    i2c_write_register_polled(fd_i2c, M_CTRL_REG1, (M_RST_MASK | M_OSR_MASK | M_HMS_MASK) );  /* set up Mag OSR and Hybrid mode using M_CTRL_REG1 */

    //i2c_write_register_polled(fd_i2c, M_CTRL_REG2, 0x20);  // M_OUT_X_MSB register 0x33 follows the OUT_Z_LSB register 0x06 (used for burst read)
    i2c_write_register_polled(fd_i2c, M_CTRL_REG2, (M_HYB_AUTOINC_MASK)); /* Enable hyrid mode auto increment using M_CTRL_REG2 */

    //i2c_write_register_polled(fd_i2c, CTRL_REG2, 0x02);  // High Resolution mode

    i2c_write_register_polled(fd_i2c, CTRL_REG3, 0x01);  // Open-drain, active low interrupt
#if SENSOR_USE_INTERRUPT
    i2c_write_register_polled(fd_i2c, CTRL_REG4, 0x01);  // Enable DRDY interrupt
    i2c_write_register_polled(fd_i2c, CTRL_REG5, 0x01);  // DRDY interrupt routed to INT1 - PTD4
#endif

    //i2c_write_register_polled(fd_i2c, CTRL_REG1, 0x35);  // ODR = 3.125Hz, Reduced noise, Active mode
    i2c_write_register_polled(fd_i2c, CTRL_REG1, ( LNOISE_MASK | HYB_DATA_RATE_200HZ | ACTIVE_MASK ));
}

int16_t absolute_val(int16_t val){
	return sqrt(val*val);
}
