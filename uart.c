/* Hello */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <uart_buffer.h>

typedef struct {
	unsigned char buffer [BUFFER_SIZE]; //the buffer, made as an array
	unsigned int i_first; //Index for the oldest input byte
	unsigned int i_last; //Index for the newest input byte
	unsigned int num_bytes;  //Number of bytes currently in buffer
} buffer_typedef;

buffer_typedef rx_uart0 = {{0}, 0 , 0 ,0}; //declaring a receive buffer
buffer_typedef tx_uart0 = {{0}, 0 , 0 ,0}; //declaring a transmit buffer

ISR(USART0_RX_vect)
{
	if(rx_uart0.num_bytes == BUFFER_SIZE) { // if buffer full, set flag
		uart0_rx_overf_flag = 1;
		unsigned char scrap_var;
		scrap_var = UDR0; //UDR0 has to be read, therefore the scrap variable
	} else if(rx_uart0.num_bytes < BUFFER_SIZE) { //if there is space in buffer     //CHECK IF THIS IS RIGHT LATER
		
		rx_uart0.buffer[rx_uart0.i_last] = UDR0;
		
		rx_uart0.i_last++;
		rx_uart0.num_bytes++;
		
	}
	
	//Check if buffer is full now. If yes, set full_flag
	if(rx_uart0.num_bytes == BUFFER_SIZE){
		uart0_rx_full_flag = 1;
	}


	//If index has reached the end of buffer, set the i_last to 0 to go around
	if(rx_uart0.i_last == BUFFER_SIZE){
		rx_uart0.i_last = 0;
	}

	uart0_rx_not_empty_flag = 1; //Not empty is something has been placed in buffer
}




ISR(USART0_TX_vect)
{
	
	//check if buffer is full. If yes, clear flag because we gonna make room for
	if(tx_uart0.num_bytes == BUFFER_SIZE){
		uart0_tx_full_flag = 0;
	}
	
	//if data exist, put the sending_byte in the buffer
	if(tx_uart0.num_bytes > 0){
		
		UDR0 = tx_uart0.buffer[tx_uart0.i_first];
		
		tx_uart0.i_first++;
		tx_uart0.num_bytes--;
	}
	
	//if reaches the end of buffer, set it to 0 to go around
	if(tx_uart0.i_first == BUFFER_SIZE){
		tx_uart0.i_first = 0;
	}
	
	//if no more data in buffer, set not_empty_flag to 0
	if(tx_uart0.num_bytes == 0){
		uart0_tx_not_empty_flag = 0;
		
		//Disable UART "TX hw buffer empty" interrupt here
		//I do this by setting UDRIE0 in UCSR0B to 0
		UCSR0B &= ~(1<<TXCIE0);   
		
		//if using shared RX/TX hardware buffer (WHICH I DO, OR DO WE?) enable RX data interrupt here
		//UCSR0B |= (1<<RXEN0);
		
	}
	
}


void uart0_send_byte(unsigned char value){ 
	
	//Disabling interrupts
	cli();
	
	if(tx_uart0.num_bytes == BUFFER_SIZE) {      // if no room in uart buffer
		uart0_tx_overf_flag	= 1;		//set the overflow flag
	} else if (tx_uart0.num_bytes < BUFFER_SIZE){						//if there is room in the buffer
		tx_uart0.buffer[tx_uart0.i_last] = value; //data transfer to buffer
		tx_uart0.i_last++;			//inc index of most recent
		tx_uart0.num_bytes++;		//inc number of bytes in buffer
	}
	
	//setting flag if buffer full
	if(tx_uart0.num_bytes == BUFFER_SIZE){
		uart0_tx_full_flag = 1;
	}
	
	
	//index turn-around
	if(tx_uart0.i_last == BUFFER_SIZE){
		tx_uart0.i_last = 0;
	}
	
	
	//gdfsg
	if((tx_uart0.num_bytes) == 1 && (UCSR0A & (1<<UDRE0)) ){
			UDR0 = tx_uart0.buffer[tx_uart0.i_first];
				
			tx_uart0.i_first++;
			tx_uart0.num_bytes--;
			
			if(tx_uart0.i_first == BUFFER_SIZE){
				tx_uart0.i_first = 0;
			}
		
	}
	
	//enabling interrupts
	sei();
	
	if(tx_uart0.num_bytes > 0){
	//Enable UART "TX hw buffer empty" interrupt here
	UCSR0B |= (1<<TXCIE0);
	
	//if using shared RS/TX hardware buffer (WHICH I DO, OR DO WE?) disable RX data interrupt here
	//UCSR0B &= ~(1<<RXEN0);
	}
	

}


unsigned char uart0_get_byte(void){
	
	//disabling interrupts
	cli();
	
	unsigned char value;
	
	//If the buffer is full, clear full-flag
	if(rx_uart0.num_bytes == BUFFER_SIZE){
		uart0_rx_full_flag = 1;  //0 = false, 1= true
	}
	
	//If data exist in buffer, get oldest value from buffer (at index i_first). If no data, returns null
	if(rx_uart0.num_bytes > 0){
		value = rx_uart0.buffer[rx_uart0.i_first];
		rx_uart0.i_first++;
		rx_uart0.num_bytes--;
	} else {				//if buffer is empty, clear not empty flag
		uart0_rx_not_empty_flag =  0;
		value = 0xFE;  //if no data to recieve, returns null
	}
	
	//if it becomes empty after we get a byte, set not_empty_flag to 0
	if(rx_uart0.num_bytes == 0){
		uart0_rx_not_empty_flag = 0;
	}
	
	//if index reaches end of buffer, roll over index counter
	if(rx_uart0.i_first == BUFFER_SIZE){
		rx_uart0.i_first = 0;	
	}
	
	//enabling interrupts
	sei();
	
	return value;
}
