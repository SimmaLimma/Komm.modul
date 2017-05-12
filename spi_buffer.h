//Spi.h-filen MASTER

#ifndef SPI_INCL_GUARD_BUFFER
#define SPI_INCL_GUARD_BUFFER

#define SPI_BUFFER_SIZE 64 
#define RECEIVED_SPI_PACKAGE_SIZE 4

volatile extern unsigned char spi_rx_not_empty_flag;

void spi_master_init(void);

unsigned char spi_get_byte(void);

void spi_send_byte(unsigned char value);

unsigned char is_spi_package_received(void);

unsigned char read_steering_info(unsigned char* esc_valueL_ptr,unsigned char* esc_valueH_ptr, unsigned char* steering_valueL_ptr, unsigned char* steering_valueH_ptr);

#endif
