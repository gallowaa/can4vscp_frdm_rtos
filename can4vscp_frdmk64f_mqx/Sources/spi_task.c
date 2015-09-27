/*
 * spi_task.c
 *
 *  Created on: Sep 9, 2015
 *      Author: Angus
 */


/*
 * spi_eeprom.c
 *
 *  Created on: Aug 27, 2015
 *      Author: Angus
 */

#include <mqx.h>
#include <bsp.h>
#include <spi.h>


#if ! BSPCFG_ENABLE_SPI0
#error This application requires BSPCFG_ENABLE_SPI0 defined non-zero in user_config.h. Please recompile kernel with this option.
#else
#define TEST_CHANNEL "spi0:"
#endif

#if ! BSPCFG_ENABLE_IO_SUBSYSTEM
#error This application requires BSPCFG_ENABLE_IO_SUBSYSTEM defined non-zero in user_config.h. Please recompile BSP with this option.
#endif


#ifndef BSP_DEFAULT_IO_CHANNEL_DEFINED
#error This application requires BSP_DEFAULT_IO_CHANNEL to be not NULL. Please set corresponding BSPCFG_ENABLE_TTYx to non-zero in user_config.h and recompile BSP with this option.
#endif


#ifndef BSP_SPI_MEMORY_CHANNEL
#error This application requires BSP_SPI_MEMORY_CHANNEL to be defined. Please set it to appropriate SPI channel number in user_config.h and recompile BSP with this option.
#endif


//unsigned char send_buffer[SPI_MEMORY_PAGE_SIZE];
//unsigned char recv_buffer[sizeof(TEST_STRING_LONG)];

MQX_FILE_PTR           spifd;

const char *device_mode[] =
{
    "SPI_DEVICE_MASTER_MODE",
    "SPI_DEVICE_SLAVE_MODE",
};

const char *clock_mode[] =
{
    "SPI_CLK_POL_PHA_MODE0",
    "SPI_CLK_POL_PHA_MODE1",
    "SPI_CLK_POL_PHA_MODE2",
    "SPI_CLK_POL_PHA_MODE3"
};

extern uint8_t  readEEPROM (uint8_t addr);
extern void     writeEEPROM(uint8_t addr, uint8_t data);
extern void spi_eeprom_guid_init();

/*TASK*-----------------------------------------------------------
 *
 * Task Name : Spi_Task
 * Comments :
 *
 *
 *END*-----------------------------------------------------------*/
void Spi_Task(uint32_t parameter) {

	//MQX_FILE_PTR           spifd;
	uint32_t                param, result, i = 0;
	SPI_STATISTICS_STRUCT  stats;
	SPI_READ_WRITE_STRUCT  rw;
	uint8_t addr;
	uint8_t rvByte;

	/* Open the SPI driver */
	spifd = fopen (TEST_CHANNEL, NULL);

	if (NULL == spifd)
	{
		printf ("Error opening SPI driver!\n");
		_time_delay (200L);
		_task_block ();
	}

   /* Display baud rate */
	printf ("Current baud rate ... ");
	if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_GET_BAUD, &param))
	{
		printf ("%d Hz\n", param);
	}
	else
	{
		printf ("ERROR\n");
	}

	/* Set a different rate */
	param = 1000000;
	printf ("Changing the baud rate to %d Hz ... ", param);
	if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_SET_BAUD, &param))
	{
		printf ("OK\n");
	}
	else
	{
		printf ("ERROR\n");
	}
	/* Set clock mode */
	param = SPI_CLK_POL_PHA_MODE0;
	printf ("Setting clock mode to %s ... ", clock_mode[param]);
	if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_SET_MODE, &param))
	{
		printf ("OK\n");
	}
	else
	{
		printf ("ERROR\n");
	}

	/* Get clock mode */
	printf ("Getting clock mode ... ");
	if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_GET_MODE, &param))
	{
		printf ("%s\n", clock_mode[param]);
	}
	else
	{
		printf ("ERROR\n");
	}

	/* Set big endian */
	param = SPI_DEVICE_BIG_ENDIAN;
	printf ("Setting endian to %s ... ", param == SPI_DEVICE_BIG_ENDIAN ? "SPI_DEVICE_BIG_ENDIAN" : "SPI_DEVICE_LITTLE_ENDIAN");
	if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_SET_ENDIAN, &param))
	{
		printf ("OK\n");
	}
	else
	{
		printf ("ERROR\n");
	}

	/* Get endian */
	printf ("Getting endian ... ");
	if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_GET_ENDIAN, &param))
	{
		printf ("%s\n", param == SPI_DEVICE_BIG_ENDIAN ? "SPI_DEVICE_BIG_ENDIAN" : "SPI_DEVICE_LITTLE_ENDIAN");
	}
	else
	{
		printf ("ERROR\n");
	}

	/* Set transfer mode */
	param = SPI_DEVICE_MASTER_MODE;
	printf ("Setting transfer mode to %s ... ", device_mode[param]);
	if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_SET_TRANSFER_MODE, &param))
	{
		printf ("OK\n");
	}
	else
	{
		printf ("ERROR\n");
	}

	/* Get transfer mode */
	printf ("Getting transfer mode ... ");
	if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_GET_TRANSFER_MODE, &param))
	{
		printf ("%s\n", device_mode[param]);
	}
	else
	{
		printf ("ERROR\n");
	}

	while(1) {

		eeprom_guid_init();

		for( addr=0x00; addr<0xFF; addr++)
		{
			writeEEPROM( addr, addr);
			rvByte = readEEPROM( addr);

			printf("(%02X) = %02X\r\n", addr, rvByte);
		}

	}

}

void SPI_init( void ) {

	uint32_t               param, result, i = 0;
	SPI_STATISTICS_STRUCT  stats;
	SPI_READ_WRITE_STRUCT  rw;
	uint8_t addr;
	uint8_t rvByte;

	/* Open the SPI driver */
	spifd = fopen (TEST_CHANNEL, NULL);

	if (NULL == spifd)
	{
		printf ("Error opening SPI driver!\n");
		_time_delay (200L);
		_task_block ();
	}

   /* Display baud rate */
	printf ("Current baud rate ... ");
	if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_GET_BAUD, &param))
	{
		printf ("%d Hz\n", param);
	}
	else
	{
		printf ("ERROR\n");
	}

	/* Set a different rate */
	param = 1000000;
	printf ("Changing the baud rate to %d Hz ... ", param);
	if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_SET_BAUD, &param))
	{
		printf ("OK\n");
	}
	else
	{
		printf ("ERROR\n");
	}
	/* Set clock mode */
	param = SPI_CLK_POL_PHA_MODE0;
	printf ("Setting clock mode to %s ... ", clock_mode[param]);
	if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_SET_MODE, &param))
	{
		printf ("OK\n");
	}
	else
	{
		printf ("ERROR\n");
	}

	/* Get clock mode */
	printf ("Getting clock mode ... ");
	if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_GET_MODE, &param))
	{
		printf ("%s\n", clock_mode[param]);
	}
	else
	{
		printf ("ERROR\n");
	}

	/* Set big endian */
	param = SPI_DEVICE_BIG_ENDIAN;
	printf ("Setting endian to %s ... ", param == SPI_DEVICE_BIG_ENDIAN ? "SPI_DEVICE_BIG_ENDIAN" : "SPI_DEVICE_LITTLE_ENDIAN");
	if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_SET_ENDIAN, &param))
	{
		printf ("OK\n");
	}
	else
	{
		printf ("ERROR\n");
	}

	/* Get endian */
	printf ("Getting endian ... ");
	if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_GET_ENDIAN, &param))
	{
		printf ("%s\n", param == SPI_DEVICE_BIG_ENDIAN ? "SPI_DEVICE_BIG_ENDIAN" : "SPI_DEVICE_LITTLE_ENDIAN");
	}
	else
	{
		printf ("ERROR\n");
	}

	/* Set transfer mode */
	param = SPI_DEVICE_MASTER_MODE;
	printf ("Setting transfer mode to %s ... ", device_mode[param]);
	if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_SET_TRANSFER_MODE, &param))
	{
		printf ("OK\n");
	}
	else
	{
		printf ("ERROR\n");
	}

	/* Get transfer mode */
	printf ("Getting transfer mode ... ");
	if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_GET_TRANSFER_MODE, &param))
	{
		printf ("%s\n", device_mode[param]);
	}
	else
	{
		printf ("ERROR\n");
	}
}
