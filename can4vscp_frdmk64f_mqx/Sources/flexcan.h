/*
 * flexcan.h
 *
 *  Created on: Aug 24, 2015
 *      Author: Angus
 */

#ifndef SOURCES_FLEXCAN_H_
#define SOURCES_FLEXCAN_H_

#include <mqx.h>
#include <bsp.h>
#include <lwevent.h>

#define CAN_BUFFER_SIZE 8

/*! @brief FlexCAN Message Buffer ID type*/
typedef enum flexcan_tx_msg {
	FLEXCAN_TX_STD_FRAME,        /*!< Standard ID*/
	FLEXCAN_TX_XTD_FRAME         /*!< Extended ID*/
} FLEXCAN_TX_MSG_FLAGS;

/*! @brief FlexCAN Message Buffer ID type*/
typedef enum flexcan_rx_msg {
	FLEXCAN_RX_RTR_FRAME,        /*!< Standard ID*/
	FLEXCAN_RX_XTD_FRAME         /*!< Extended ID*/
} FLEXCAN_RX_MSG_FLAGS;

/*! @brief FlexCAN Message Buffer ID type*/
typedef enum flexcan_status {
	FLEXCAN_FAIL,
	FLEXCAN_SUCCESS
} flexcan_code_t;


void FLEXCAN_Init();

uint32_t FLEXCAN_Rx_Init();

uint32_t FLEXCAN_Tx_Init();

/*!
    @brief Get a VSCP frame.  Use this function to check for
    	   full receive buffer and extract received data into local buffers.
    @param pid - Pointer to buffer that will be populated with receive ID.
    @param pdlc - Pointer to buffer that will be populated with count of bytes copied in data buffer.
    @param pdata - Pointer to buffer that will be populated with data if there is any
    @param msgFlags - type of can frame
    @return TRUE on success.
*/
flexcan_code_t FLEXCANReceiveMessage(uint32_t *pid, uint32_t *pdlc, uint32_t *pdata, FLEXCAN_RX_MSG_FLAGS *msgFlags);


#endif /* SOURCES_FLEXCAN_H_ */
