//hejhej
//.c-file for slaves SPI_buffer

//Comments:
//You need to put volatile "unsigned char spi_rx_not_empty_flag = 0;" in main as global variable.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <spi_buffer_slave.h>

typedef struct {
	unsigned char buffer [SPI_BUFFER_SIZE]; //the buffer, made as an array
	unsigned int i_first; //Index for the oldest input byte
	unsigned int i_last; //Index for the newest input byte
	unsigned int num_bytes;  //Number of bytes currently in buffer
} spi_buffer_typedef;


spi_buffer_typedef rx_spi = {{0}, 0 , 0 ,0}; //declaring a receive buffer
spi_buffer_typedef tx_spi = {{0}, 0 , 0 ,0}; //declaring a transmit buffer

unsigned char spi_stc_chain_in_work = 0;

ISR(SPI_STC_vect){


	
	//Turn PORTA0 to low so master can detect rising edge
	PORTA &= ~(1<<PORTA0);
	
	unsigned char in_value;
	
	//reading value from SPDR
	in_value = SPDR;
	
	//if scrap value -> ignore
	//if not scrap value -> read the byte to rx-buffer
	if(in_value != 0xFD){
		
		rx_spi.buffer[rx_spi.i_last] = in_value;
		rx_spi.i_last++;
		rx_spi.num_bytes++;
		spi_rx_not_empty_flag = 1;
	}
	
	//Turnover for i_last
	if(rx_spi.i_last == SPI_BUFFER_SIZE){
		
		rx_spi.i_last = 0;
	}
	
	//if there is something to send, put the value in SPDR and set PORTA0 to high (handshake)
	if(tx_spi.num_bytes > 0){
	
		SPDR = tx_spi.buffer[tx_spi.i_first];
		tx_spi.i_first++;
		tx_spi.num_bytes--;
		
		PORTA |= (1<<PORTA0);
		
		if(tx_spi.i_first == SPI_BUFFER_SIZE){
			
			tx_spi.i_first = 0;
		}
		
	}
	
	else{  //if tx_spi.num_bytes == 0
		
		//put scrap in SPDR
		SPDR = 0xFD;
		spi_stc_chain_in_work = 0;
		
	} 	
	

}
	
	
void spi_slave_init(void){
	
	//Set MISO output, all others input, DDPB6= MISO
	DDRB = (1<<DDB6);
	//Enable SPI and enable SPI_STC interrupt
	SPCR = (1<<SPE) | (1<<SPIE) |  (0<<MSTR);
	//Setting PORTA0 as output and set it to low, so master can detect rising edge
	DDRA |= (1<<PORTA0);
	PORTA &= ~(1<<PORTA0);
	
	SPDR = 0xFD;
	
}
	
	
		
	
	
unsigned char spi_get_byte(void){
	
	/*get-byte function
	*Returns data if data in buffer exists
	*otherwise, it will return null
	*FIFO ring-buffer
	*/
	
	//if there is no data, value returned will be 0xFE
	unsigned char value = 0xFE;
	
	cli();
	//If data in buffer exists, read to value
	if(rx_spi.num_bytes > 0){

		value = rx_spi.buffer[rx_spi.i_first];
		rx_spi.i_first++;
		rx_spi.num_bytes--;
	}
	
	if(rx_spi.num_bytes == 0){
		spi_rx_not_empty_flag = 0;
	}
	
	
	//turnover for i_first
	if(rx_spi.i_first == SPI_BUFFER_SIZE){
		rx_spi.i_first = 0;
	}
	

	sei();
	
	return value;
	
		
}

void spi_send_byte(unsigned char value){
	
	cli();
	
	//if there is space in tx-buffer, put value in it.
	if (tx_spi.num_bytes < SPI_BUFFER_SIZE){	//if there is room in the buffer
			
		tx_spi.buffer[tx_spi.i_last] = value; //data transfer to buffer
		tx_spi.i_last++;			//inc index of most recent
		tx_spi.num_bytes++;		//inc number of bytes in buffer
	}
		
	//index turn-around
	if(tx_spi.i_last == SPI_BUFFER_SIZE){
		tx_spi.i_last = 0;
	}
	
			
	//if there is only one byte in buffer, no SPI_STC is "in work"
	//Therefore, it is started by adding the byte to SPDR and requesting a send to master
	//PORTA0 is first set to low, so Master can detect rising edge
	if(tx_spi.num_bytes == 1 && !(spi_stc_chain_in_work)){
		
		SPDR = tx_spi.buffer[tx_spi.i_first];
		tx_spi.i_first++;
		tx_spi.num_bytes--;
		PORTA |= (1<<PORTA0);
		spi_stc_chain_in_work = 1;
	}
			
	//index turn-around
	if(tx_spi.i_first == SPI_BUFFER_SIZE){
		tx_spi.i_first = 0;
	}
			
			

	sei();

	
}


unsigned char is_package_recieved(void){
	
	if (rx_spi.num_bytes < (RECEIVED_PACKAGE_SIZE)){
		return 0;
	}
	
	return 1;
}


//returns true (i.e. >0) if succesfull, false (==0) else
//Also deletes all bytes before header byte, in order to reach header byte. Therefore, it is important to only call this function when is_package_recieved() is true.
unsigned char read_sensor_info(unsigned char* control_mode_ptr, struct Sensor_information* sens_info_ptr){
	
	//If no bytes to get, return "failure"
	if(!(rx_spi.num_bytes)){
		return 0;
	}
	
	//First check if header-byte first in buffer and if enough bytes for a package to be recieved
	if( (rx_spi.buffer[rx_spi.i_first] == 0xFF) && (is_package_recieved())){
		
		//dont use header byte
		unsigned char temp_for_header;
		temp_for_header = spi_get_byte();
		
		
		//Read all info and write it to sensor struct

		*control_mode_ptr = spi_get_byte();
		sens_info_ptr->dist_right_line = spi_get_byte();
		sens_info_ptr->dist_sonic_middle = spi_get_byte();
		sens_info_ptr->dist_sonic_left = spi_get_byte();
		sens_info_ptr->dist_sonic_right = spi_get_byte();
		sens_info_ptr->dist_sonic_back = spi_get_byte();
		sens_info_ptr->ang_acc = spi_get_byte();
		sens_info_ptr->car_speed = spi_get_byte();
		sens_info_ptr->dist_to_stop_line = spi_get_byte();
		sens_info_ptr->sign_type = spi_get_byte();
		
		
		return 1;
	} else if (is_package_recieved()){
		
		//if there are bytes before header byte, throw them away. Then go into read_sensor_info again to check wheter there
		//is something to read or not. The return value is evaluated by the last read_sensor_info.
		unsigned char temp_scrap;
		temp_scrap = spi_get_byte();
		return read_sensor_info(control_mode_ptr, sens_info_ptr);
	}
	
	
	//if reaches this point, there are is a header byte but not enough of bytes to read
	return 0;
	
}
