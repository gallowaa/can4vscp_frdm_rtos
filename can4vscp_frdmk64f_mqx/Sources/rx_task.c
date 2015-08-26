/*
 * rx_task.c
 *
 *  Created on: Aug 21, 2015
 *      Author: Angus
 */

#include <mqx.h>
#include <bsp.h>
#include <lwevent.h>

#include "main.h"

extern LWEVENT_STRUCT event; // Takes place of OSA state struct in KSDK

/*TASK*-----------------------------------------------------------
*
* Task Name : Rx_Task
* Comments :
*
*
*END*-----------------------------------------------------------*/
void Rx_Task(uint32_t parameter)
{/* Body */
   unsigned char   dptr[8];
   uint32_t result;
   uint32_t DLC = 0;
   uint32_t ID = 0;

   /* Init, Active, Install ISR */

   if(FLEXCAN_OK != FLEXCAN_Rx_Init())
	   printf("FLEXCAN rx mailbox initialization. result: 0x%lx \r\n", result);

   while(1)
   {
	   vscp_imsg.flags = 0;

	  /* Block forever until MY_FLEXCAN_ISR signals event, that a message has been received*/

      if (_lwevent_wait_ticks(&event, 1 << RX_mailbox_num, FALSE, 0) != MQX_OK) {
         printf("Event Wait failed \r\n");
      }

      vscp_getEvent();

   }
} /* EndBody */

#ifdef VERBOSE_TASK
/*TASK*-----------------------------------------------------------
*
* Task Name : Rx_Task VERBOSE
* Comments :
*
*
*END*-----------------------------------------------------------*/
void Rx_Task(uint32_t parameter)
{/* Body */
   unsigned char   dptr[8];
   uint32_t result;
   uint32_t DLC = 0;
   uint32_t ID = 0;

/*
   result = FLEXCAN_Initialize_mailbox( CAN_DEVICE, RX_remote_mailbox_num, RX_remote_identifier,
                                        8, FLEXCAN_TX, format,
                                        interrupt);
   printf("\nFLEXCAN rx remote mailbox initialization. result: 0x%lx", result);
*/

   /* Initialize mailbox */
   result = FLEXCAN_Initialize_mailbox( CAN_DEVICE, RX_mailbox_num, RX_identifier,
                                        data_len_code, FLEXCAN_RX, format,
                                        interrupt);
   printf("\nFLEXCAN rx mailbox initialization. result: 0x%lx", result);

   result = FLEXCAN_Activate_mailbox(CAN_DEVICE, RX_mailbox_num, FLEXCAN_RX_MSG_BUFFER_EMPTY);
   printf("\nFLEXCAN mailbox activation. result: 0x%lx", result);


   /* Install ISR */
   if(interrupt == FLEXCAN_ENABLE)
   {
      result = FLEXCAN_Install_isr( CAN_DEVICE, RX_mailbox_num, MY_FLEXCAN_ISR );
      printf("\nFLEXCAN RX ISR install. result: 0x%lx", result);

/*
      result = FLEXCAN_Install_isr( CAN_DEVICE, RX_remote_mailbox_num, MY_FLEXCAN_ISR  );
      printf("\nFLEXCAN RX remote ISR install. result: 0x%lx", result);
*/
   }

   /*

  if (_lwevent_wait_ticks(&event, 1 << RX_mailbox_num, FALSE, 0) != MQX_OK) {
     printf("\nEvent Wait failed");
  }

  result = FLEXCAN_Lock_mailbox (CAN_DEVICE, RX_mailbox_num);
  if(result != FLEXCAN_OK)
  {
     printf("\nLock mailbox failed. Error Code: 0x%lx", result);
  }

  result = FLEXCAN_Rx_message(CAN_DEVICE, RX_mailbox_num, &ID, format,
                             &DLC, &dptr, interrupt);
  if(result != FLEXCAN_OK)
     printf("\nReceived error. Error Code: 0x%lx", result);
  else
  {
     printf("\nReceived data: ");
     for (result = 0; result < DLC; result++) printf ("0x%x ", dptr[result]);
     printf("\nID is: 0x%x", ID);
     printf("\nDLC is: 0x%x\n", DLC);
  }

  result = FLEXCAN_Unlock_mailbox (CAN_DEVICE);
  if(result != FLEXCAN_OK)
  {
     printf("\nUnlock mailbox failed. Error Code: 0x%lx", result);
  }*/


   while(1)
   {

      if (_lwevent_wait_ticks(&event, 1 << RX_mailbox_num, FALSE, 0) != MQX_OK) {
         printf("\nEvent Wait failed");
      }

      result = FLEXCAN_Lock_mailbox (CAN_DEVICE, RX_mailbox_num);
      if(result != FLEXCAN_OK)
      {
         printf("\nLock mailbox failed. Error Code: 0x%lx", result);
      }

      result = FLEXCAN_Rx_message(CAN_DEVICE, RX_mailbox_num, &ID, format,
                             &DLC, &dptr, interrupt);
      if(result != FLEXCAN_OK)
         printf("\nReceived error. Error Code: 0x%lx", result);
      else
      {
         printf("\nReceived data: ");
         for (result = 0; result < DLC; result++) printf ("0x%x ", dptr[result]);
         printf("\nID is: 0x%x", ID);
         printf("\nDLC is: 0x%x\n", DLC);
      }

      result = FLEXCAN_Unlock_mailbox (CAN_DEVICE);
      if(result != FLEXCAN_OK)
      {
         printf("\nUnlock mailbox failed. Error Code: 0x%lx", result);
      }

      // FLEXCAN_Request_message (CAN_DEVICE, RX_remote_mailbox_num, format);

   }
} /* EndBody */
#endif

/* EOF */
