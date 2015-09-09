
#include <mqx.h>
#include <bsp.h>

//#define SPI_DEBUG

/* Number of bytes used for addressing within memory */
#define SPI_TRANSFER_SIZE_BYTES       3

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
static void    eeprom_set_write_latch (MQX_FILE_PTR spifd, bool enable);
//extern void    memory_set_protection (MQX_FILE_PTR spifd, bool protect);
//extern uint8_t  memory_read_status (MQX_FILE_PTR spifd);

void     writeEEPROM(MQX_FILE_PTR spifd, uint8_t addr, unsigned char data);
uint8_t   readEEPROM(MQX_FILE_PTR spifd, uint8_t addr);

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
void writeEEPROM(MQX_FILE_PTR spifd, uint8_t addr, uint8_t data)
{
    _mqx_int result;
    uint8_t buffer[SPI_TRANSFER_SIZE_BYTES];

    /* Each write operation must be enabled in memory */
    eeprom_set_write_latch (spifd, TRUE);

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
uint8_t readEEPROM(MQX_FILE_PTR spifd, uint8_t addr)
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
static void eeprom_set_write_latch (MQX_FILE_PTR spifd, bool enable)
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
