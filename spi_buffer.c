//Spi.h-filen MASTER

/* Things to consider/fix:
*-Clock is set to fck/16 (too slow?)
*
*
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <spi_buffer.h>

typedef struct {
	unsigned char buffer [SPI_BUFFER_SIZE]; //the buffer, made as an array
	unsigned int i_first; //Index for the oldest input byte
	unsigned int i_last; //Index for the newest input byte
	unsigned int num_bytes;  //Number of bytes currently in buffer
} spi_buffer_typedef;

spi_buffer_typedef rx_spi = {{0}, 0 , 0 ,0}; //declaring a receive buffer
spi_buffer_typedef tx_spi = {{0}, 0 , 0 ,0}; //declaring a transmit buffer
	
//flag for checking status if the spi_stc chain is currently working
unsigned char spi_stc_chain_in_work = 0;

ISR(SPI_STC_vect){
	/* this ISR first reads value, then either send
	*byte from buffer (if there is any)
	*else "stall"
	*
	*/
	
	

	
	
	spi_stc_chain_in_work = 1;
	
	unsigned char in_value;
	
	//reading value
	in_value = SPDR;
	
	//If the read value is not scrap, put it in rx-buffer
	if(in_value != 0xFD){ // if Slave don't sends scrap
		
		rx_spi.buffer[rx_spi.i_last] = in_value;
		rx_spi.i_last++;
		rx_spi.num_bytes++;
		
		//no empty anymore
		spi_rx_not_empty_flag = 1;
	}
	
	
	//turnover
	if(rx_spi.i_last == SPI_BUFFER_SIZE){
		
		rx_spi.i_last = 0;	
	}
	
	
	//If SPI-chain stops, enable INT2-interrupt
	if(tx_spi.num_bytes == 0){
			
		EIMSK |= (1<<INT2);
		spi_stc_chain_in_work = 0;
	}
	
	//if there is byte to send, put it in SPDR
	if(tx_spi.num_bytes > 0){

		SPDR = tx_spi.buffer[tx_spi.i_first];
		tx_spi.i_first++;
		tx_spi.num_bytes--;
		
	}
	
	

	
	//turnover
	if(tx_spi.i_first == SPI_BUFFER_SIZE){
		
		tx_spi.i_first = 0;
	}
	
}

ISR(INT2_vect){
	//Interrupt for the first byte that will be sent from buffer after it has been empty
	
	
	if(tx_spi.num_bytes > 0){
		
		SPDR = tx_spi.buffer[tx_spi.i_first];
		tx_spi.i_first++;
		tx_spi.num_bytes--;
	}
	else {
		//Sending scrap-data
		SPDR = 0xFD; //don-t know if works
		
	}
	
	//turnover
	if(tx_spi.i_first == SPI_BUFFER_SIZE){
		tx_spi.i_first = 0;
	}
}


void spi_master_init(void) //not initialized
{
	//set MOSI and SCK as output, all others as input, and PB4 to output (because Master dont use SS-pin)
	DDRB = (1<<DDB5) | (1<<DDB7) | (1<<DDB4) | (0<<DDB2);
	// enable SPI, Master, set cloxk rate fck/16, and enable SPI_STC interrupt, and sets Slave Select (SS) to high
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0) | (1<<SPIE); //CPOL cpha
	//Enable int_2 external interrupt and interrupt happens on rising edge
	EICRA = (1<<ISC20) | (1<<ISC21);
	EIMSK = (1<<INT2);
	

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
	
	if(rx_spi.num_bytes == 0) {
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
	
	/*Send-byte function
	*Puts the data in buffer, if buffer not full (never overwrites existing data)
	*FIFO ring-buffer
	*Automatically disables INT2-interrupt if buffer empty
	*Automatically starts the SPI_STC-chain
	*/
	
	cli();
	
	//disable int2-interrupt, because transmission will happen anyway
	//if master has data to transmit
	EIMSK &= ~(1<<INT2);
	
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
	//Therefore, it is started by adding the byte to SPDR
	if((tx_spi.num_bytes == 1) && !(spi_stc_chain_in_work)){
		SPDR = tx_spi.buffer[tx_spi.i_first];
		tx_spi.i_first++;
		tx_spi.num_bytes--;
		
		spi_stc_chain_in_work = 1;
	}
	
	//index turn-around
	if(tx_spi.i_first == SPI_BUFFER_SIZE){
		tx_spi.i_first = 0;
	}
	
		

	sei();

	
}
