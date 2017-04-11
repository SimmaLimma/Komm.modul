#define F_CPU 14745600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <uart_buffer.h>
#include <spi_buffer.h>
#include <util/delay.h>

//Defining the struct for sensor information
struct Sensor_information{
	unsigned short dist_right_line;
	unsigned short dist_sonic_middle;
	unsigned short dist_sonic_left;
	unsigned short dist_sonic_right;
	unsigned short dist_sonic_back;
	unsigned short ang_acc;
	unsigned short car_speed;
	unsigned short dist_to_stop_line;
	unsigned char sign_type; //Not sure we gonna use this one. Depends if camera can detect signs
};


void USART1_init(unsigned int);
void USART0_init(unsigned int);
void Sens_info_read(struct Sensor_information*);
void spi_master_init(void);

//Initialsing buffer (and counter for receivening data) for UARTs
volatile unsigned char UART1_reciever_buffer[32];
volatile int counter_UART1_reciever;

//Initialising flags to check if UART0 buffer is overflowing and to check if buffers
//are full and also flags to check wheter they are empty
volatile unsigned char uart0_rx_overf_flag = 0;
volatile unsigned char uart0_tx_overf_flag = 0;

volatile unsigned char uart0_rx_full_flag = 0;
volatile unsigned char uart0_tx_full_flag = 0;

volatile unsigned char uart0_rx_not_empty_flag = 0;
volatile unsigned char uart0_tx_not_empty_flag = 0;


/*----------Interrupts---------------*/

ISR(USART1_RX_vect){   
	/*
	*This buffer is specialized for sensorinformation.
	*May overwrite existing data in buffer
	*
	*/
	
	unsigned char in_value;
	
	//Disabling interrupts
	//cli();

	
	in_value = UDR1;
	
	//checking if header (0xFF)
	if (in_value ==  0xFF){ // to check if in_value==0xFF
		counter_UART1_reciever = 0;
	}
	else{ 
		//Write value to buffer
		UART1_reciever_buffer[counter_UART1_reciever] = in_value;
		counter_UART1_reciever++;
	}

	
	//Enabling interrupts
	//sei();

	
}






/*---------------functions---------------*/


void USART1_init(unsigned int baud_setting)
{
	//UART enabling:
	UBRR1 = 0;
	//Enabling reciever and disabling transmitter interrupts
	UCSR1B = (1<<RXEN1) | (0<<TXEN1) | (1<<RXCIE1) | (0<<TXCIE1)| (0<<UDRIE1);
	//Set frame format: 8data, 2 stop bit
	UCSR1C = (1<<USBS1) | (3<<UCSZ10);
	//Setting baud rate
	UBRR1 = baud_setting;
}

void USART0_init(unsigned int baud_setting)
{
	UBRR0 = baud_setting;
	//
	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0) | (0<<TXCIE0)| (0<<UDRIE0);
	//Set frame format: 8data, 2 stop bit
	UCSR0C = (1<<USBS0) | (3<<UCSZ00);
	
	
	//Setting baud rate
	UBRR0 = baud_setting;
}


void spi_master_init(void) //not initialized
{
	//set MOSI and SCK as output, all others as input, and PB4 to output (because Master dont use SS-pin)
	DDRB = (1<<DDB5) | (1<<DDB7) | (1<<DDB4);
	// enable SPI, Master, set cloxk rate fck/16, and enable SPI_STC interrupt, and sets Slave Select (SS) to high
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0) | (0<<SPIE); //CPOL cpha
	//Enable int_2 external interrupt and interrupt happens on rising edge
    EICRA = (1<<ISC20) | (1<<ISC21);
	EIMSK = (1<<INT2);
	

}




void Sens_info_read(struct Sensor_information* sens_info_ptr) //There is no check if the buffer is empty or not
{
	
	//Disable UART1 interrupts to prevent values from changing while reading
	UCSR1B &= ~(1<<RXCIE1);
	
	//Assigning values from buffer to sens_info
	sens_info_ptr->dist_right_line = ((unsigned) (short) UART1_reciever_buffer[1] << 8) | (unsigned) (short) UART1_reciever_buffer[0];
	sens_info_ptr->dist_sonic_middle = ((unsigned) (short) UART1_reciever_buffer[3] << 8) | (unsigned) (short) UART1_reciever_buffer[2];
	sens_info_ptr->dist_sonic_left = ((unsigned) (short) UART1_reciever_buffer[5] << 8) | (unsigned) (short) UART1_reciever_buffer[4];
	sens_info_ptr->dist_sonic_right = ((unsigned) (short) UART1_reciever_buffer[7] << 8) | (unsigned) (short) UART1_reciever_buffer[6];
	sens_info_ptr->dist_sonic_back = ((unsigned) (short) UART1_reciever_buffer[9] << 8) | (unsigned) (short) UART1_reciever_buffer[8];
	sens_info_ptr->ang_acc = ((unsigned) (short) UART1_reciever_buffer[11] << 8) | (unsigned) (short) UART1_reciever_buffer[10];
	sens_info_ptr->car_speed = ((unsigned) (short) UART1_reciever_buffer[13] << 8) | (unsigned) (short) UART1_reciever_buffer[12];
	sens_info_ptr->dist_to_stop_line = ((unsigned) (short) UART1_reciever_buffer[15] << 8) | (unsigned) (short) UART1_reciever_buffer[14];
	sens_info_ptr->sign_type = UART1_reciever_buffer[16];
	
	//Enable UART1 interrupts
	UCSR1B |= (1<<RXCIE1);
}







/*-----------Main--------*/
int main(void)
{
	
	//init var for sensor information from sensormodul and pointer to it aswell
	struct Sensor_information sensor_info;
	struct Sensor_information* sens_info_ptr;
	sens_info_ptr = &sensor_info;
	
	//Disabling interrupts while init.
	cli();
	
	//Initilaising variables for baud rate. 
	//Baud rate is set to 115200, no particular reason given, and buad_setting = 7 gives that rate
	unsigned char baud_setting =  7;
		
		
	
	//initialisation, like bluetooth connection. Maybe some start sign? Recieving map info and destinations, and calculating route
	
	
	//Initilasing UART1 (to sensormodul)
	USART1_init(baud_setting);
	USART0_init(baud_setting);
	
	//Init SPI
	spi_master_init();
	
	//Enabling global interrupts
	sei();
	
	
	
	//Main loop EVERYTHING AFTER HERE IS TEST-----------------------------

	
	DDRA = 0xFF; 
	
	PORTA = 1<<PORTA0;
	
	

	double counter = 0;
	while (counter < 1000){
		counter++;
	}

	PORTA |= (1<<PORTA1);

	//Sens_info_read(sens_info_ptr);
	unsigned char test_var = 'a';
	
	
	// Test for SPI
	SPDR = test_var;
	
	
	
	
	while(1)
	{
	

		
		
		
	}
}


