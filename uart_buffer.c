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
		//uart0_rx_overf_flag = 1;
		unsigned char scrap_var;
		scrap_var = UDR0; //UDR0 has to be read, therefore the scrap variable
		} else if(rx_uart0.num_bytes < BUFFER_SIZE) { //if there is space in buffer     //CHECK IF THIS IS RIGHT LATER
		
		rx_uart0.buffer[rx_uart0.i_last] = UDR0;
		
		rx_uart0.i_last++;
		rx_uart0.num_bytes++;
		
	}
	
	//Check if buffer is full now. If yes, set full_flag
	//if(rx_uart0.num_bytes == BUFFER_SIZE){
	//	uart0_rx_full_flag = 1;
	//}


	//If index has reached the end of buffer, set the i_last to 0 to go around
	if(rx_uart0.i_last == BUFFER_SIZE){
		rx_uart0.i_last = 0;
	}

	uart0_rx_not_empty_flag = 1; //Not empty is something has been placed in buffer
}


ISR(USART0_TX_vect)
{
	
	//check if buffer is full. If yes, clear flag because we gonna make room for
	//if(tx_uart0.num_bytes == BUFFER_SIZE){
	//	uart0_tx_full_flag = 0;
	//}
	
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
		//uart0_tx_not_empty_flag = 0;
		
		//Disable UART "TX hw buffer empty" interrupt here
		//I do this by setting UDRIE0 in UCSR0B to 0
		UCSR0B &= ~(1<<TXCIE0);
		
		//if using shared RX/TX hardware buffer (WHICH I DO, OR DO WE?) enable RX data interrupt here
		//UCSR0B |= (1<<RXEN0);
		
	}
	
}


void USART0_init(unsigned int baud_setting)
{
	UBRR0 = 0;
	
	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0) | (0<<TXCIE0)| (0<<UDRIE0);
	//Set frame format: 8data, 2 stop bit
	UCSR0C = (1<<USBS0) | (3<<UCSZ00);
	
	
	//Setting baud rate
	UBRR0 = baud_setting;
}



void uart0_send_byte(unsigned char value){
	
	//Disabling interrupts
	cli();
	
	if(tx_uart0.num_bytes == BUFFER_SIZE) {      // if no room in uart buffer
		//uart0_tx_overf_flag	= 1;		//set the overflow flag
	}
	else if (tx_uart0.num_bytes < BUFFER_SIZE){						//if there is room in the buffer
		tx_uart0.buffer[tx_uart0.i_last] = value; //data transfer to buffer
		tx_uart0.i_last++;			//inc index of most recent
		tx_uart0.num_bytes++;		//inc number of bytes in buffer
	}
	
	//setting flag if buffer full
	//if(tx_uart0.num_bytes == BUFFER_SIZE){
	//	uart0_tx_full_flag = 1;
	//}
	
	
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
	
	
	if(tx_uart0.num_bytes > 0){
		//Enable UART "TX hw buffer empty" interrupt here
		UCSR0B |= (1<<TXCIE0);
		
		//if using shared RS/TX hardware buffer (WHICH I DO, OR DO WE?) disable RX data interrupt here
		//UCSR0B &= ~(1<<RXEN0);
	}
	
	sei();

}


unsigned char uart0_get_byte(void){
	
	//disabling interrupts
	cli();
	
	unsigned char value;
	
	//If the buffer is full, clear full-flag
	//if(rx_uart0.num_bytes == BUFFER_SIZE){
	//	uart0_rx_full_flag = 1;  //0 = false, 1= true
	//}
	
	//If data exist in buffer, get oldest value from buffer (at index i_first). If no data, returns null
	if(rx_uart0.num_bytes > 0){
		value = rx_uart0.buffer[rx_uart0.i_first];
		rx_uart0.i_first++;
		rx_uart0.num_bytes--;
		} else {				//if buffer is empty, clear not empty flag
		uart0_rx_not_empty_flag =  0;
		value = 0xFE;  //if no data to receive, returns null
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


unsigned char read_comp_info(unsigned char *control_com_ptr, unsigned char *nod_info_ptr, unsigned char *manual_mode_ptr, unsigned char *dest_list_ptr){
	/* Return value is depending on which type of package that has been read (returning package_type)
	*  according to the communication protocol (0x01=init things, 0x2= switching to autonomous,
	*  0x3=command controls, 0x4=remaking of the destination list) and if nothing is read, returning 0x00
	*  If one of the parameters are not relevant, it will be left untouched.
	*
	*  If this function is called without any header byte in buffer, it will flush the buffer and return 0.
	*
	*  Also, this function waits until a whole package is received (i.e. timeout=None)
	*/
	
	//First check if header-byte first in buffer and if there is anything in buffer
	if( (rx_uart0.buffer[rx_uart0.i_first] == 0xFF) && (uart0_rx_not_empty_flag) ){
		
		
		
		unsigned char package_type = 0;
		unsigned char finished_reading = 0; //bool
		unsigned char temp;
		unsigned char temp_control_com = 0;
		int i=0;
		int j=0;
		int k=0;
		int n_of_col = 14; ///num_of_col = number bytes in a node
		
		//don't use header byte
		temp = uart0_get_byte();

		
		//Read all info and write it to sensor struct
		
		
		//Read packge_type byte
		while(!uart0_rx_not_empty_flag);
		//why?
		package_type = uart0_get_byte();
		
		
		
		if (package_type == 3){
			
			*control_com_ptr = 0x00;
		}
		
		
		while(!finished_reading){
			//Reading value
			while(!uart0_rx_not_empty_flag);
			temp = uart0_get_byte();
			
			//Doing something with value
			if(package_type == 0x01){ //Map
				//Do something with info
				
				if ((temp == 0xFE) && (j == 0)){
					finished_reading = 1;
					*(nod_info_ptr + (i*n_of_col + j)) = 0xFE; //contains number of nodes
					*(nod_info_ptr + (i*n_of_col + j) + 1) = 0x00;
				}
				else{
					
					*(nod_info_ptr + (i*n_of_col + j)) = temp;
					if(j<13){
						j++;
					}
					else{
						j=0;
						i++;
					}
					
				}
			}
			
			else if(package_type == 2){ // Switching of mode
				
				
				if(!temp){
					*manual_mode_ptr = 0;
				}
				else{
					*manual_mode_ptr = 1;
				}
				
				finished_reading = 1;

			}
			else if(package_type == 0x03){ // Control commands
				
				if (temp == 'a'){
					*(control_com_ptr) |= (1<<0);
				}
				else if (temp == 'd'){
					*(control_com_ptr) |= (1<<1);
				}
				else if (temp == 'w'){
					*(control_com_ptr)|= (1<<2);
				}
				else if (temp == 's'){
					*(control_com_ptr) |= (1<<3);
				}
				else if (temp == 'r'){
					*(control_com_ptr) |= (1<<4);
				}
				else if (temp == 'f'){
					*(control_com_ptr) |= (1<<5);
				}
				else if (temp == 'x'){
					*(control_com_ptr) |= (1<<6);
				}
				else if (temp == 'q'){
					*(control_com_ptr) |= (1<<7);
				}
				else{ //end-byte for control commands. Sending it as 0xFE, but could in theory be anything that is not valid keystroke
					
					
					
					finished_reading = 1;
					
				}
				
			}
			else if(package_type == 4){ //Switching dest. list
				
				if(temp == 0xFE){
					finished_reading = 1;
					k=0;
				}
				else{
					*(dest_list_ptr+k) = temp;
					k++;
				}
				
			} // package 4
		} // while finished reading
		
		return package_type;

		
	} // if first is header byte

	else if (uart0_rx_not_empty_flag){
		//if there are bytes before header byte, throw them away. Then go into read_sensor_info again to check whether there
		//is something to read or not. The return value is evaluated by the last read_sensor_info.
		uart0_get_byte();
		
		return read_comp_info(control_com_ptr, nod_info_ptr,manual_mode_ptr, dest_list_ptr); // need arguments
	}
	
	//If buffer empty, return 0x00
	return 0;

} // read_comp_info
