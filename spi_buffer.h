//Spi.h-filen MASTER

#ifndef SPI_INCL_GUARD_BUFFER
#define SPI_INCL_GUARD_BUFFER

#define SPI_BUFFER_SIZE 64 

volatile extern unsigned char spi_rx_not_empty_flag;

void spi_master_init(void);

unsigned char spi_get_byte(void);

void spi_send_byte(unsigned char value);



#endif
