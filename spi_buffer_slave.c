//hejhej
//.c-file for slaves SPI_buffer

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


ISR(SPI_STC_vect){

	//TEST
	PORTA |= (1<<PORTA4);
	//END of test
	
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
	
	else{
		
		//put scrap in SPDR
		SPDR = 0xFD;
		
	} 	
	

}
	
	
void spi_slave_init(void){
	
	//Set MISO output, all others input, DDPB6= MISO
	DDRB = (1<<DDB6);
	//Enable SPI and enable SPI_STC interrupt
	SPCR = (1<<SPE) | (1<<SPIE) | (0<<MSTR);
	//Setting PORTA0 as output
	DDRA |= (1<<PORTA0);
	
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
	if(tx_spi.num_bytes == 1){
		
		SPDR = tx_spi.buffer[tx_spi.i_first];
		tx_spi.i_first++;
		tx_spi.num_bytes--;
		PORTA |= (1<<PORTA0);
	}
			
	//index turn-around
	if(tx_spi.i_first == SPI_BUFFER_SIZE){
		tx_spi.i_first = 0;
	}
			
			

	sei();

	
}
