/*
 * old_main.c
 *
 *  Created on: Aug 21, 2015
 *      Author: Angus
 */



#ifdef TRANSMIT_ONLY


   result = FLEXCAN_Initialize_mailbox( CAN_DEVICE, TX_remote_mailbox_num, TX_remote_identifier,
                                        8, FLEXCAN_TX, format,
                                        interrupt);
   printf("\nFLEXCAN tx remote mailbox initialization. result: 0x%lx", result);


   /* Initialize mailbox */
   result = FLEXCAN_Initialize_mailbox( CAN_DEVICE, TX_mailbox_num, TX_identifier,
                                        data_len_code, FLEXCAN_TX, format,
                                        interrupt);
   printf("\nFLEXCAN tx mailbox initialization. result: 0x%lx", result);

   result = FLEXCAN_Activate_mailbox(CAN_DEVICE, TX_mailbox_num, FLEXCAN_TX_MSG_BUFFER_NOT_ACTIVE);
   printf("\nFLEXCAN tx mailbox activation. result: 0x%lx", result);

   /* Install ISR */
   if(interrupt == FLEXCAN_ENABLE)
   {
      result = FLEXCAN_Install_isr( CAN_DEVICE, TX_mailbox_num, MY_FLEXCAN_ISR  );
      printf("\nFLEXCAN TX ISR install. result: 0x%lx", result);

      result = FLEXCAN_Install_isr( CAN_DEVICE, TX_remote_mailbox_num, MY_FLEXCAN_ISR  );
      printf("\nFLEXCAN TX remote ISR install. result: 0x%lx", result);
   }

   /* Let Rx Task start to initialize */
   // _time_delay(1000);

   result = FLEXCAN_Tx_message(CAN_DEVICE, TX_mailbox_num, TX_identifier,
                              format, data_len_code, &data);
   if(result != FLEXCAN_OK)
      printf("\nTransmit error. Error Code: 0x%lx", result);
   else
      printf("\nData transmit: %d", data);

   while(1)
   {
      /* Let Rx Task receive message */
      _time_delay(1000);

      data++;


      result = FLEXCAN_Tx_mailbox(CAN_DEVICE, TX_mailbox_num, &data);
      if(result != FLEXCAN_OK)
         printf("\nTransmit error. Error Code: 0x%lx", result);
      else
         printf("\nData transmit: %d", data);

   }

#endif

#ifdef RECEIVE_ONLY


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

      }


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
      }


      void MY_FLEXCAN_ISR
      (
         /* [IN] FlexCAN base address */
         void   *can_ptr
      )
      {
         volatile FLEXCAN_REG_STRUCT_PTR        can_reg_ptr;
         volatile uint32_t                               tmp_reg;
         volatile uint32_t                               temp;

         can_reg_ptr = (FLEXCAN_REG_STRUCT_PTR)can_ptr;

         /* get the interrupt flag */
         tmp_reg = (can_reg_ptr->IFLAG & FLEXCAN_IMASK_VALUE);
         // check Tx/Rx interrupt flag and clear the interrupt
         if(tmp_reg){
            /* clear the interrupt and unlock message buffer */
            /* Start CR# 1751 */
            _lwevent_set(&event, tmp_reg);
            can_reg_ptr->IFLAG |= tmp_reg;
            /* End CR# 1751 */
            temp = can_reg_ptr->TIMER;
         }/* Endif */

         // Clear all other interrupts in ERRSTAT register (Error, Busoff, Wakeup)
         tmp_reg = can_reg_ptr->ERRSTAT;
         if(tmp_reg & FLEXCAN_ALL_INT){
            /* Start CR# 1751 */
            can_reg_ptr->ERRSTAT |= (tmp_reg & FLEXCAN_ALL_INT);
            /* End CR# 1751 */
         } /* Endif */

         return;
      }

#endif
