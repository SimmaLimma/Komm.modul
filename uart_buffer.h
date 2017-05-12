/* The best include file in the world */

//NO guarantee that flags work properly. Not tested yet.

/*
*guide I am using
*
*/

#ifndef INCL_GUARD_BUFFER
#define INCL_GUARD_BUFFER

//Buffer size. To change buffer size, change here. It is now set to 64
#define BUFFER_SIZE 64 

volatile extern unsigned char uart0_rx_not_empty_flag;
//volatile extern unsigned char uart0_tx_not_empty_flag;
//volatile extern unsigned char uart0_rx_overf_flag;
//volatile extern unsigned char uart0_tx_overf_flag;
//volatile extern unsigned char uart0_rx_full_flag;
//volatile extern unsigned char uart0_tx_full_flag;


//init
void USART0_init(unsigned int baud_setting);

//uart0_send_byte is a function to place a byte in Tx_buffer
void uart0_send_byte(unsigned char value);

//uart0_get_byte is a function to retrieve a byte from Rx_buffer, which is returned
unsigned char uart0_get_byte(void);

//unsigned char read_comp_info();
unsigned char read_comp_info(unsigned char* control_com, unsigned char* nod_info_ptr, unsigned char *manual_mode_ptr, unsigned char *dest_list_ptr);

#endif
