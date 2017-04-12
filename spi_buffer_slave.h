//.h-file for slave's spi-buffer


//for init in main program in slave, remember to put PORTA0 to output (DDRA = 1<<PORTA0)

#ifndef SPI_SLAVE_INCL_GUARD_BUFFER
#define SPI_SLAVE_INCL_GUARD_BUFFER

#define SPI_BUFFER_SIZE 64

volatile extern unsigned char spi_rx_not_empty_flag;

unsigned char spi_get_byte(void);

void spi_send_byte(unsigned char value);

void spi_slave_init(void);

#endif
