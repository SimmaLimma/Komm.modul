//Spi.h-filen MASTER

#ifndef SPI_INCL_GUARD_BUFFER
#define SPI_INCL_GUARD_BUFFER

#define SPI_BUFFER_SIZE 64 

unsigned char spi_get_byte(void);

void spi_send_byte(unsigned char value);



#endif
