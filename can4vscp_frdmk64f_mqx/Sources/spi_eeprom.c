
#include <mqx.h>
#include <bsp.h>
#include "main.h"
//#define SPI_DEBUG

/* Number of bytes used for addressing within memory */
#define SPI_TRANSFER_SIZE_BYTES       3
#define TRANSFER_SIZE 				  8
#define GUID_SIZE					  16

/* Memory page size - maximum bytes per write */
//#define SPI_MEMORY_PAGE_SIZE           0x0100

/* The SPI serial memory instructions */
#define SPI_EEPROM_WRITE_STATUS        0x01
#define SPI_EEPROM_WRITE_DATA          0x02
#define SPI_EEPROM_READ_DATA           0x03
#define SPI_EEPROM_WRITE_LATCH_DISABLE 0x04
#define SPI_EEPROM_READ_STATUS         0x05
#define SPI_EEPROM_WRITE_LATCH_ENABLE  0x06
//#define SPI_EEPROM_CHIP_ERASE          0xC7
#define DUMMY_PATTERN				   0x55


/* Funtion prototypes */

//extern void    memory_chip_erase (MQX_FILE_PTR spifd);
static void    eeprom_set_write_latch (bool enable);
//extern void    memory_set_protection (MQX_FILE_PTR spifd, bool protect);
//extern uint8_t  memory_read_status (MQX_FILE_PTR spifd);

void     writeEEPROM( uint8_t addr, unsigned char data);
uint8_t  readEEPROM( uint8_t addr);
void spi_eeprom_guid_init();
void eeprom_guid_init( void );


extern MQX_FILE_PTR           spifd; // SPI0 file descriptor
/*
extern uint32_t memory_write_data (MQX_FILE_PTR spifd, uint32_t addr, uint32_t size, unsigned char *data);
extern uint32_t memory_read_data (MQX_FILE_PTR spifd, uint32_t addr, uint32_t size, unsigned char *data);
*/

/*FUNCTION*---------------------------------------------------------------
*
* Function Name : memory_write_byte
* Comments  : This function writes a data byte to memory
*
*
*END*----------------------------------------------------------------------*/
void writeEEPROM(uint8_t addr, uint8_t data)
{
    _mqx_int result;
    uint8_t buffer[SPI_TRANSFER_SIZE_BYTES];

    /* Each write operation must be enabled in memory */
    eeprom_set_write_latch (TRUE);

    //memory_read_status (spifd);
#ifdef SPI_DEBUG
    printf ("Write byte 0x%02x to location 0x%02x in memory ... ", data, addr);
#endif

    /* Write instruction, address and data to buffer */
    buffer[0] = SPI_EEPROM_WRITE_DATA;
    buffer[1] = addr;
    buffer[2] = data;

    result = fwrite (buffer, 1, SPI_TRANSFER_SIZE_BYTES, spifd);

    /* Deactivate CS */
    fflush (spifd);

    if (result != SPI_TRANSFER_SIZE_BYTES)
    {
        printf ("ERROR\n");
    }
    else
    {
#ifdef SPI_DEBUG
        printf ("done\n", data);
#endif
    }

    /* There is 5 ms internal write cycle needed for memory */
    _time_delay (5);
}

/*FUNCTION*---------------------------------------------------------------
*
* Function Name : memory_read_byte
* Comments  : This function reads a data byte from memory
* Return:
*         Byte read.
*
*END*----------------------------------------------------------------------*/
uint8_t readEEPROM( uint8_t addr)
{
    _mqx_int result;
    uint8_t buffer[SPI_TRANSFER_SIZE_BYTES];
    uint8_t data = 0;

#ifdef SPI_DEBUG
    printf ("Read byte from location 0x%02x in memory ... ", addr);
#endif

    /* Read instruction, address */
    buffer[0] = SPI_EEPROM_READ_DATA;
    buffer[1] = addr - 1;
    buffer[2] = DUMMY_PATTERN;

    //memory_addr_to_buffer(addr, &(buffer[1]));

    /* Write instruction and address */
    result = fwrite (buffer, 1, SPI_TRANSFER_SIZE_BYTES, spifd);

    if (result != SPI_TRANSFER_SIZE_BYTES)
    {
        /* Stop transfer */
        fflush (spifd);
        printf ("ERROR (tx)\n");
        return data;
    }

    /* Read data from memory */
    result = fread (&data, 1, 1, spifd);

    /* Deactivate CS */
    fflush (spifd);

    if (result != 1)
    {
        printf ("ERROR (rx)\n");
    }
    else
    {
        //printf ("0x%02x\n", data);
    }

    return data;
}

/*FUNCTION*---------------------------------------------------------------
*
* Function Name : memory_set_write_latch
* Comments  : This function sets latch to enable/disable memory write
*             operation
*
*END*----------------------------------------------------------------------*/
static void eeprom_set_write_latch (bool enable)
{
    _mqx_int result;
    uint8_t buffer[1];

    if (enable)
    {

#ifdef SPI_DEBUG
        printf ("Enable write latch in memory ... ");
#endif
        buffer[0] = SPI_EEPROM_WRITE_LATCH_ENABLE;
    } else {

#ifdef SPI_DEBUG
        printf ("Disable write latch in memory ... ");
#endif
        buffer[0] = SPI_EEPROM_WRITE_LATCH_DISABLE;
    }

    /* Write instruction */
    result = fwrite (buffer, 1, 1, spifd);

    /* Wait till transfer end (and deactivate CS) */
    fflush (spifd);

    if (result != 1)
    {
        printf ("ERROR\n");
    }
    else
    {
#ifdef SPI_DEBUG
        printf ("OK\n");
#endif
    }
}

/*!
    @brief spi_eeprom_guid_init - Initialize the eeprom with the GUID in the appropriate location
    @purpose The EUI-48 lives in a write protected area by default.
    		 Copy it to an earlier memory location and create a proper vscp GUID with the format
    		 for a GUID based on an ethernet MAC.
 */
void spi_eeprom_guid_init() {

	/* GUID layout in eeprom interfaced over SPI
	 *
	 * FF:FF:FF:FF:FF:FF:FF:FE:YY:YY:YY:YY:YY:YY:XX:XX
	 *
	 * The holder of the address can freely use the two least significant bytes of the GUID.
	 * MAC address in MSB - LSB order. Also called MAC-48 or EUI-48 by IEEE
	 * Source: http://www.vscp.org/docs/vscpspec/doku.php?id=globally_unique_identifiers
	 */

	uint32_t i,j;
	//dspi_status_t dspiResult;
	_mqx_int result;

	uint8_t txBuffer[TRANSFER_SIZE] = {SPI_EEPROM_READ_DATA, EUI48_START-1, 5,5,5,5,5,5};	/*! contains op-code + addr for getting EUI-48 */

	uint8_t rxEUI48[TRANSFER_SIZE] = {0};						/*! buffer specifically for receiving EUI-48 from write protected region */
	uint8_t rxBuffer[TRANSFER_SIZE] = {0};						/*! generic receive buffer for WREN, WRDI writes */


	/* Note that we don't write the last 2 bytes (XX:XX) because they are for the nickname
	 * and are determined later. Array is still 16 bytes as 2 get taken up for opcode + address at beginning */
	uint8_t txGUID[16] = {0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
						// 	  ,	    , 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xYY, 0xYY, 0xYY, 0xYY, 0xYY, 0xYY, 0xXX, 0xXX

	uint8_t rxGUIDbuff[16] = {0};								/*! This may not be needed, but we provide a 16-byte buffer
																 *  to satisfy the api call when writing 16-byte GUID */


	/**********************************************
	 * First, get the EUI48 from the write protected region
	 **********************************************/

	/* Write instruction and address */
	result = fwrite (txBuffer, 1, TRANSFER_SIZE, spifd);

	if (result != TRANSFER_SIZE)
	{
		/* Stop transfer */
		fflush (spifd);
		printf ("ERROR (tx)\n");
		return;
	}

	/* Read data from memory */
	result = fread (&rxEUI48, 1, TRANSFER_SIZE, spifd);

	/* Deactivate CS */
	fflush (spifd);

	j=10; //position 10 is the start index of the EUI-48 in the GUID

	for (i = 2; i < TRANSFER_SIZE; i++)
	{
		txGUID[j] = rxEUI48[i];
		j++;
	}

	/**********************************************
	 * Write the actual data with another API call.
	 **********************************************/

	txGUID[0] = SPI_EEPROM_WRITE_DATA; // WRITE command
	txGUID[1] = VSCP_EEPROM_REG_GUID;  // address to write

	/* Each write operation must be enabled in memory */
	eeprom_set_write_latch (TRUE);

	/* Write instruction and address */
	result = fwrite (txGUID, 1, GUID_SIZE, spifd);

	if (result != GUID_SIZE)
	{
		/* Stop transfer */
		fflush (spifd);
		printf ("ERROR (tx)\n");
		return;
	}

	/* Read data from memory */
	result = fread (&rxGUIDbuff, 1, GUID_SIZE, spifd);

	/* Deactivate CS */
	fflush (spifd);

#ifdef DEBUG
	printf("GUID Rx: ");
	for (i = 0; i < 16; i++) {
		printf(" %02X", rxGUIDbuff[i]);
	}
	printf("\r\n");
#endif
}

/*!
    @brief spi_eeprom_guid_init - Initialize the eeprom with the GUID in the appropriate location
    @purpose The EUI-48 lives in a write protected area by default.
    		 Copy it to an earlier memory location and create a proper vscp GUID with the format
    		 for a GUID based on an ethernet MAC.
 */
void eeprom_guid_init() {

	/* GUID layout in eeprom interfaced over SPI
	 *
	 * FF:FF:FF:FF:FF:FF:FF:FE:YY:YY:YY:YY:YY:YY:XX:XX
	 *
	 * The holder of the address can freely use the two least significant bytes of the GUID.
	 * MAC address in MSB - LSB order. Also called MAC-48 or EUI-48 by IEEE
	 * Source: http://www.vscp.org/docs/vscpspec/doku.php?id=globally_unique_identifiers
	 */

	uint32_t i,j;
	uint8_t addr;
	//dspi_status_t dspiResult;
	_mqx_int result;

	uint8_t txBuffer[TRANSFER_SIZE] = {SPI_EEPROM_READ_DATA, EUI48_START-1, 5,5,5,5,5,5};	/*! contains op-code + addr for getting EUI-48 */

	uint8_t rxEUI48[TRANSFER_SIZE] = {0};						/*! buffer specifically for receiving EUI-48 from write protected region */
	uint8_t rxBuffer[TRANSFER_SIZE] = {0};						/*! generic receive buffer for WREN, WRDI writes */


	/* Note that we don't write the last 2 bytes (XX:XX) because they are for the nickname
	 * and are determined later. Array is still 16 bytes as 2 get taken up for opcode + address at beginning */
	uint8_t txGUID[16] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02};
					   // 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xYY, 0xYY, 0xYY, 0xYY, 0xYY, 0xYY, 0xXX, 0xXX

	uint8_t rxGUIDbuff[16] = {0};								/*! This may not be needed, but we provide a 16-byte buffer
																 *  to satisfy the api call when writing 16-byte GUID */


	/**********************************************
	 * First, get the EUI48 from the write protected region
	 **********************************************/

	addr = EUI48_START; // 0xfa - 1

	for(i=0; i<6 ; i++){
		rxEUI48[i] = readEEPROM(addr);
		addr++;
	}

#ifdef OLD
	/* Write instruction and address */
	result = fwrite (txBuffer, 1, TRANSFER_SIZE, spifd);

	if (result != TRANSFER_SIZE)
	{
		/* Stop transfer */
		fflush (spifd);
		printf ("ERROR (tx)\n");
		return;
	}

	/* Read data from memory */
	result = fread (&rxEUI48, 1, TRANSFER_SIZE, spifd);

	/* Deactivate CS */
	fflush (spifd);
#endif

	j=8; //position 8 is the start index of the EUI-48 in the GUID

	for (i = 0; i < 6; i++)
	{
		txGUID[j] = rxEUI48[i];
		j++;
	}

	/**********************************************
	 * Write the actual data with another API call.
	 **********************************************/

	addr = VSCP_EEPROM_REG_GUID;

	for(i = 0; i < GUID_SIZE; i++) {
		writeEEPROM(addr, txGUID[i]);
		addr++;
	}


#ifdef OLD

	txGUID[0] = SPI_EEPROM_WRITE_DATA; // WRITE command
	txGUID[1] = VSCP_EEPROM_REG_GUID;  // address to write

	/* Each write operation must be enabled in memory */
	eeprom_set_write_latch (TRUE);

	/* Write instruction and address */
	result = fwrite (txGUID, 1, GUID_SIZE, spifd);

	if (result != GUID_SIZE)
	{
		/* Stop transfer */
		fflush (spifd);
		printf ("ERROR (tx)\n");
		return;
	}

	/* Read data from memory */
	result = fread (&rxGUIDbuff, 1, GUID_SIZE, spifd);

	/* Deactivate CS */
	fflush (spifd);
#endif

	addr = VSCP_EEPROM_REG_GUID;

	for(i = 0; i < GUID_SIZE; i++) {
		rxGUIDbuff[i] = readEEPROM(addr);
		addr++;
	}

#ifdef DEBUG
	printf("GUID Rx: ");
	for (i = 0; i < 16; i++) {
		printf(" %02X", rxGUIDbuff[i]);
	}
	printf("\r\n");
#endif
}

