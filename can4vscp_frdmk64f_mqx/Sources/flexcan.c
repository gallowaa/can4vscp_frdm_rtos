/*
 * flexcan.c
 *
 *  Created on: Aug 24, 2015
 *      Author: Angus
 */

#include "main.h"
#include "flexcan.h"


void FLEXCAN_Init() {

	uint32_t result;

	/*
	 * Configures pins for CAN I/O
	 */
	if (_bsp_flexcan_io_init(CAN_DEVICE) != 0)
	{
		printf ("\nError initializing pins for FlexCAN device %d!\n", CAN_DEVICE);
		// _task_block();
	}

	frequency = 125;
	printf("\nselected frequency (Kbps) is: %d", frequency);

	data_len_code = 1;
	printf("\nData length: %d", data_len_code);

	/* Select message format */
	format = FLEXCAN_EXTENDED;

	/* Select mailbox number */
	RX_mailbox_num = 0;
	TX_mailbox_num = 1;

	RX_identifier = 0x900;
	TX_identifier = 0x321;

	/* We use default settings */
	bit_timing0 = bit_timing1 = 0;

	/* Select operation mode */
	flexcan_mode = FLEXCAN_NORMAL_MODE;

	/* Enable interrupt */
	interrupt = FLEXCAN_ENABLE;

	/* Enable error interrupt */
	flexcan_error_interrupt = true;

	/* Reset FLEXCAN device */
	result = FLEXCAN_Softreset ( CAN_DEVICE);
	printf("\nFLEXCAN reset. result: 0x%lx", result);

	/* Initialize FLEXCAN device */
	result = FLEXCAN_Initialize ( CAN_DEVICE, bit_timing0, bit_timing1, frequency, FLEXCAN_IPBUS_CLK);
	printf("\nFLEXCAN initilization. result: 0x%lx", result);

	/* Select mode */
	result = FLEXCAN_Select_mode( CAN_DEVICE, flexcan_mode);
	printf("\nFLEXCAN mode selected. result: 0x%lx", result);

	//result = FLEXCAN_Set_global_stdmask (CAN_DEVICE, 0x222);
	result = FLEXCAN_Set_global_extmask(CAN_DEVICE, 0x000);
	printf("\nFLEXCAN global mask. result: 0x%lx", result);

	/* Enable error interrupts */
	if(flexcan_error_interrupt == true)
	{
		result = FLEXCAN_Install_isr_err_int( CAN_DEVICE, MY_FLEXCAN_ISR );
		printf("\nFLEXCAN Error ISR install, result: 0x%lx", result);

		result = FLEXCAN_Install_isr_boff_int( CAN_DEVICE, MY_FLEXCAN_ISR  );
		printf("\nFLEXCAN Bus off ISR install, result: 0x%lx", result);

		result = FLEXCAN_Error_int_enable(CAN_DEVICE);
		printf("\nFLEXCAN error interrupt enable. result: 0x%lx", result);
	}
}


uint32_t FLEXCAN_Rx_Init() {

	uint32_t result;

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
	return result;
}

uint32_t FLEXCAN_Tx_Init() {

	uint32_t result;

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
	}

	return result;

}

#ifdef THIS_IS_PART_OF_EXTERNAL_VSCP

/*!
    @brief Get a VSCP frame.  Use this function to check for
    	   full receive buffer and extract received data into local buffers.
    @param pid - Pointer to buffer that will be populated with receive ID.
    @param pdlc - Pointer to buffer that will be populated with count of bytes copied in data buffer.
    @param pdata - Pointer to buffer that will be populated with data if there is any
    @param msgFlags - type of can frame
    @return TRUE on success.
 */
flexcan_code_t FLEXCANReceiveMessage(uint32_t *pid, uint32_t *pdlc, uint32_t *pdata, FLEXCAN_RX_MSG_FLAGS *msgFlags) {

	unsigned char   dptr[8];
	uint32_t result;
	uint32_t DLC = 0;
	uint32_t ID = 0;

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
#endif
