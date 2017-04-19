//.h-file for slave's spi-buffer


//for init in main program in slave, remember to put PORTA0 to output (DDRA = 1<<PORTA0)

#ifndef SPI_SLAVE_INCL_GUARD_BUFFER
#define SPI_SLAVE_INCL_GUARD_BUFFER

#define SPI_BUFFER_SIZE 64
#define RECEIVED_PACKAGE_SIZE 11

struct Sensor_information{
	unsigned char dist_right_line;
	unsigned char dist_sonic_middle;
	unsigned char dist_sonic_left;
	unsigned char dist_sonic_right;
	unsigned char dist_sonic_back;
	unsigned char ang_acc;
	unsigned char car_speed;
	unsigned char dist_to_stop_line;
	unsigned char sign_type; //Not sure we gonna use this one. Depends if camera can detect signs
};


volatile extern unsigned char spi_rx_not_empty_flag;

void spi_slave_init(void);

unsigned char spi_get_byte(void);

void spi_send_byte(unsigned char value);

//returns true if there is more bytes in buffer than package size (inc header byte)
unsigned char is_package_recieved(void);

//returns true (i.e. >0) if succesfull, false (==0) else
unsigned char read_sensor_info(unsigned char* control_mode_ptr, struct Sensor_information* sens_info_ptr);

#endif
