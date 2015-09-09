/*
 * tx_task.c
 *
 *  Created on: Aug 21, 2015
 *      Author: Angus
 */

#include <mqx.h>
#include <bsp.h>
#include <lwevent.h>
#include "main.h"

extern LWEVENT_STRUCT event; // Takes place of sync struct in KSDK

/*TASK*-----------------------------------------------------------
 *
 * Task Name : Tx_Task
 * Comments :
 *
 *
 *END*-----------------------------------------------------------*/
void Tx_Task(uint32_t parameter)
{
	/* Body */
	unsigned char   data = 0;
	uint32_t result;
	unsigned char string[] =
	{
			0xa, 0xb, 0xe, 0xc, 0xe, 0xd, 0xa, 0x0
	};

	/* Init, Active, Install ISR */

	if(FLEXCAN_OK != FLEXCAN_Tx_Init())
		printf("FLEXCAN tx mailbox initialization. result: 0x%lx \r\n", result);

	/* Let Rx Task start to initialize */
	// _time_delay(1000);


	while(1)
	{
		/* Send Tx Msg 1 sec period message */
		//_time_delay(1000);
		// data++;

		/* Block forever until it is time to send an event */

		if (_lwevent_wait_ticks(&event, 1 << TX_mailbox_num, FALSE, 0) != MQX_OK) {
			printf("Event Wait failed \r\n");
		}


		/* (id, dlc, pdata, FLEXCAN_TX_XTD_FRAME) */

		result = FLEXCAN_Tx_message(CAN_DEVICE, TX_mailbox_num, TX_identifier, format, data_len_code, &data);
			if(result != FLEXCAN_OK)
				printf("\nTransmit error. Error Code: 0x%lx", result);
			else
				printf("\nData transmit: %d", data);

			/*
		result = FLEXCAN_Tx_mailbox(CAN_DEVICE, TX_mailbox_num, &data);

		if(result != FLEXCAN_OK)
			printf("\nTransmit error. Error Code: 0x%lx", result);
		else
			printf("\nData transmit: %d", data);*/


		/*
      string[7] = data;

      result = FLEXCAN_Update_message(CAN_DEVICE, string, 8, format, TX_remote_mailbox_num);
      printf("\nFLEXCAN tx update message. result: 0x%lx", result);
		 */

	}
} /* EndBody */

