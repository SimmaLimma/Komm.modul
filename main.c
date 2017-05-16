#define F_CPU 14745600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <uart_buffer.h>
#include <spi_buffer.h>
#include <util/delay.h>

//Defining the struct for sensor information
struct Sensor_information{   // add type
	unsigned char dist_right_line;
	unsigned char angular_diff;
	unsigned char dist_sonic_middle;
	unsigned char dist_sonic_left;
	unsigned char dist_sonic_right;
	unsigned char dist_sonic_back;
	unsigned char angle;
	unsigned char car_speed;
	unsigned char dist_to_stop_line;
	unsigned char sign_type; //Not sure we gonna use this one. Depends if camera can detect signs
};



void USART1_init(unsigned int);
void Sens_info_read(struct Sensor_information*);
unsigned char* Dijkstras_alg(unsigned char*, unsigned char, unsigned char, unsigned char*);

//Initialing buffer (and counter for receivening data) for UARTs
volatile unsigned char UART1_reciever_buffer[32];
volatile int counter_UART1_reciever;

//Initialising flags to check if UART0 buffer is overflowing and to check if buffers are empty
volatile unsigned char uart0_rx_not_empty_flag = 0;
volatile unsigned char spi_rx_not_empty_flag = 0;


/*----------Interrupts---------------*/

ISR(USART1_RX_vect){
	/*
	*This buffer is specialized for sensorinformation.
	*May overwrite existing data in buffer
	*
	*/
	
	unsigned char in_value;
	
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


	
}




/*---------------functions---------------*/


void USART1_init(unsigned int baud_setting)
{
	//UART enabling:
	UBRR1 = 0;
	//Enabling receiver and disabling transmitter interrupts
	UCSR1B = (1<<RXEN1) | (0<<TXEN1) | (1<<RXCIE1) | (0<<TXCIE1)| (0<<UDRIE1);
	//Set frame format: 8data, 2 stop bit
	UCSR1C = (1<<USBS1) | (3<<UCSZ10);
	//Setting baud rate
	UBRR1 = baud_setting;
}




void Sens_info_read(struct Sensor_information* sens_info_ptr) //There is no check if the buffer is empty or not
{
	
	//Disable UART1 interrupts to prevent values from changing while reading
	UCSR1B &= ~(1<<RXCIE1);
	
	//Assigning values from buffer to sens_info
	sens_info_ptr->dist_right_line = (unsigned) (char) UART1_reciever_buffer[0];
	sens_info_ptr->angular_diff = (unsigned) (char) UART1_reciever_buffer[1];
	sens_info_ptr->dist_sonic_middle = (unsigned) (char) UART1_reciever_buffer[2];
	sens_info_ptr->dist_sonic_left = (unsigned) (char) UART1_reciever_buffer[3];
	sens_info_ptr->dist_sonic_right = (unsigned) (char) UART1_reciever_buffer[4];
	sens_info_ptr->dist_sonic_back = (unsigned) (char) UART1_reciever_buffer[5];
	sens_info_ptr->angle = (unsigned) (char) UART1_reciever_buffer[6];
	sens_info_ptr->car_speed = (unsigned) (char) UART1_reciever_buffer[7];
	sens_info_ptr->dist_to_stop_line = (unsigned) (char) UART1_reciever_buffer[8];
	//sens_info_ptr->sign_type = UART1_reciever_buffer[9];
	
	counter_UART1_reciever = 0;
	
	//Enable UART1 interrupts
	UCSR1B |= (1<<RXCIE1);
}







/*-----------Main--------*/
int main(void)
{
	//For testing
	DDRA = 0xFF;
	//end of testing

	unsigned short nod_info[250*7]; // nod_info[1][1]=node 1's type
	unsigned char *nod_info_ptr; /// needed here or not??
	nod_info_ptr = &nod_info;
	
	unsigned char manual_mode = 0; // 1 if manual mode, 0 if autonomous mode
	unsigned char *manual_mode_ptr;
	manual_mode_ptr = &manual_mode;
	volatile unsigned char dest_list[50];
	volatile unsigned char *dest_list_ptr;
	dest_list_ptr = &dest_list;
	unsigned char control_com;
	unsigned char *control_com_ptr;
	control_com_ptr = &control_com;
	unsigned char node_route_ptr;
	
	//init var for sensor information from sensormodul and pointer to it aswell
	struct Sensor_information sensor_info;
	struct Sensor_information* sens_info_ptr;
	sens_info_ptr = &sensor_info;
	
	unsigned char esc_valueH = 0;
	unsigned char esc_valueL = 0;
	unsigned char steering_valueH = 0;
	unsigned char steering_valueL = 0;
	
	//For Dijsktras
	unsigned int j = 0;
	unsigned int i = 0;
	unsigned int node;
	unsigned int num_of_nodes = 0;
	unsigned int number=0;
	
	unsigned char end_node_sub;
	unsigned char n_of_col = 7; // correct?
	unsigned char current_node;
	unsigned char scanned[250] = {0};
	unsigned char node_route[250];
	unsigned char prev_node[250] = {0};
	unsigned char prev_node_index[250] = {0};
	unsigned short dist_to_node[250];
	
	unsigned char start_node;
	unsigned char end_node;
	//End of "For Dijsktras"
	
	//for Decision making	
	unsigned int d=0;
	unsigned char way_to_node;
	unsigned char way_from_node;
	unsigned char direction[250] = {0};
	//end of for Decision making;
	
	unsigned int h = 0;
	unsigned int n = 1;
	unsigned int counter_12 = 0;
	unsigned char next_turn_decision = 0;
	unsigned char previous_node = 0;
	unsigned char next_node = 0;
	unsigned char control_mode = 0x00;//Begins at normal street driving
	unsigned char prev_control_mode = 0x00;   
	unsigned char angle_before_crossing;
	
	//TEST
	//Assigning values from buffer to sens_info
	sens_info_ptr->dist_right_line = 0;
	sens_info_ptr->angular_diff = 0;
	sens_info_ptr->angle = 0;
	sens_info_ptr->car_speed = 0;
	sens_info_ptr->dist_to_stop_line = 0;
	sens_info_ptr->sign_type = 0;
	sens_info_ptr->dist_sonic_middle = 0;
	sens_info_ptr->dist_sonic_left = 0;
	sens_info_ptr->dist_sonic_right = 0;
	sens_info_ptr->dist_sonic_back = 0;
	//end TEST
	
	
	//Disabling interrupts while init.
	cli();
	
	//Initilaising variables for baud rate.
	//Baud rate is set to 115200, no particular reason given, and buad_setting = 7 gives that rate
	unsigned char baud_setting =  7;
	
	//Initilasing UART1 (to sensormodul)
	USART1_init(baud_setting);
	USART0_init(baud_setting);
	
	//Init SPI
	//Remember that sending freq is still set to fck/16, and can be set to higher.
	spi_master_init();
	
	//Enabling global interrupts
	sei();
	

	
	//Waiting for bluetooth to confirm. This is done by the PC sending 'g'.
	unsigned char bt_ready = 0;
	while(bt_ready != 'g'){
		if(uart0_rx_not_empty_flag){
			bt_ready = uart0_get_byte();
		}
	}

	
	//Send back confirmation, which is done by sending a 'c'
	uart0_send_byte('c');
	
	
	//Read map which is sent from PC.
	while(!uart0_rx_not_empty_flag);
	read_comp_info(control_com_ptr, nod_info_ptr, manual_mode_ptr, dest_list_ptr);
	//Responding that map is read
	uart0_send_byte('m');
	
	//Read destination list from PC
	while(!uart0_rx_not_empty_flag);
	read_comp_info(control_com_ptr, nod_info_ptr, manual_mode_ptr, dest_list_ptr);
	//Respond that finished reading
	uart0_send_byte('d');
	
	
	
	
	//Calculate optimal way, using Dijkstras
	//------------------DIJSKTRAS ALGORITHM-----------------------

	//check how many nodes we have and set distance to all nodes to 100, risky but works as long as real dist_to_node < 100
	


	node = nod_info[0];
	while(node != 0xFE){

		num_of_nodes++;
		dist_to_node[i+1] = 10000; //does this works??
		i++;
		node = nod_info[i*7];  // *(nod_info_ptr + (i * n_of_col));
		
	}
	

	start_node = dest_list[0];
	end_node = dest_list[1];
	
	unsigned int counter_dest_list = 1;
	//Big loop for A->B
//	while(end_node != 0xFE){ 
	
	
	//just first time
	current_node = start_node;
	dist_to_node[current_node] = 0;
	scanned[current_node] = 1;
	node_route[j] = current_node; //set start_node as the first node of the route, j starts at 0

	while(current_node != end_node){
	PORTA |= (1<<PORTA5);

		//n++;
		//while(*(dest_ptr i) != 0)
		//end_node_sub = *(dest_ptr + n);

		//while(!all_scanned || current_node != end_node_sub){
		unsigned char k=1;


		// Checking if the distance to all nodes from current node is shorter than the current "dist_to_node", (if it is: prev_node sets to current node, so we know what node gave the node it's shortest distance)
		unsigned int x=0;
		unsigned int counter_5 = 0;
		while(counter_5 < 5){
		
			if(scanned[nod_info[(current_node-1)*n_of_col + 1 + counter_5]] != 1){
				if((dist_to_node[current_node] + nod_info[(current_node-1)*n_of_col + 2 + counter_5]) < dist_to_node[nod_info[(current_node-1)*n_of_col + 1 + counter_5]]  ){

					dist_to_node[nod_info[(current_node-1)*n_of_col + 1 + counter_5]] = dist_to_node[current_node] + nod_info[(current_node-1)*n_of_col + 2 + counter_5];
					prev_node[nod_info[(current_node-1)*n_of_col + 1 + counter_5]] = current_node;
					//scanned[*(nod_info_ptr + ((current_node-1)*n_of_col + 1 + counter_5))] = 1;

					//Checking on what index prev_node it at
					while(x < num_of_nodes){
						if(node_route[x] == current_node){
							prev_node_index[nod_info[(current_node-1)*n_of_col + 1 + counter_5]] = x;
							PORTA |= (1<<PORTA6);
						}
						x++;
					}

				}
			}
			
			counter_5 += 2;
			x=0;
		}
		PORTA |= (1<<PORTA7);

		unsigned int num_of_un_scanned=0;
		i=1;
		while(i <= num_of_nodes){
			
			if(scanned[i] == 0){
				num_of_un_scanned++;
			}
			i++;
		}

		unsigned int out=0;
		i=1;
		j=1;
		while(out != 1){
			PORTA |= (1<<PORTA4);
			PORTA = 0x00;
			if(scanned[i] == 0 && i <= num_of_nodes){
				PORTA |= (1<<PORTA3);
					
				while(j <= num_of_nodes){
					if(scanned[j] == 0){
						if(dist_to_node[i] <= dist_to_node[j]){ // <= or <?
							number++;
						}
					}
					j++;

				}
				
			
				if(number == num_of_un_scanned){
					scanned[i] = 1;
					current_node = i;
					node_route[prev_node_index[current_node] + 1] = current_node;
					
					while((prev_node_index[current_node] + 1 + k) <= num_of_nodes){
							node_route[prev_node_index[current_node] + 1 + k] = 0;
							k++;
							PORTA |= (1<<PORTA2);
							
					}
					out = 1;
				}
			}
			i++;
			number=0;
			j=1;
		}
		
	}
	
//	counter_dest_list++;
//	start_node = current_node;
//	end_node = dest_list[counter_dest_list];
	
//	}



	//Decision making
	while(node_route[d] != 0){
		//checking if a node has three ways out from it, which means it's a crossing
		if(nod_info[node_route[d]-1 + 1] != 0){
			if(nod_info[node_route[d]-1 + 3] != 0){
				if(nod_info[node_route[d]-1 + 5] =! 0){
						
					if(node_route[d-1] == nod_info[d-1 + 1]){
						way_to_node = 'n';						//if the previous node has is the same number as nod_info[d-1 + 1] for the current node, the way TO the node is from north			
					}
					else if (node_route[d-1] == nod_info[d-1 + 3]){
						way_to_node = 's';
					}
					else if(node_route[d-1] == nod_info[d-1 + 5]){
						way_to_node = 'e';
					}
						
					if(node_route[d+1] == nod_info[d-1 + 1]){ //if the next node has is the same number as nod_info[d-1 + 1] for the current node, the way FROM the node is to north	
						way_from_node = 'n';
					}
					else if(node_route[d+1] == nod_info[d-1 + 3]){
						way_from_node = 's';
					}
					else if (node_route[d+1] == nod_info[d-1 + 5]){
						way_from_node = 'e';
					}
				}
			}
		}
		else{
			
			direction[d] = 'f';  //if the node is not a crossing, just continue "forward"
		} 
	
		if(way_to_node == 'e' && way_to_node == 'n'){  //the right turn form the "I" in the "T", into the big road.
			direction[d] = 'r';
		}
		else if(way_to_node == 'e' && way_to_node == 's'){ //the left turn from the "I" in the "T", into the big road.
			direction[d] = 'l';
		}
		else if(way_to_node == 'n' && way_to_node == 'e'){ //the left turn from the "-" in the "T", into the small road
			direction[d] = 'l';
		}
		else if(way_to_node == 'n' && way_to_node == 's'){ //the easiest "forward", where the both sides is "helt streckad", like the normal road
			direction[d] = 'f';
		}
		else if(way_to_node == 's' && way_to_node == 'e'){  //the right turn from the "-" in the "T", into the small road
			direction[d] = 'r';
		}
		else if(way_to_node == 's' && way_to_node == 'n'){ //the harder "forward", where only the left hand side is "helt streckad"
			direction[d] = 'F';
		}
	
		d++;
	}

	
	//end of Decision making	
	


	//------------------END OF DIJSKTRAS-------
	
	//Write back optimal way to PC
	
	
	
	//---------------------INIT ENDED (?)-------------------------//

	
	

	//TESTING dijkstras

	
	


	
	
	//END OF TESTING


	/*-------------- MAIN LOOP.---------------*/

	
	while(1){


	
		/*---Autonomous loop---*/

		//Reading Computer Info
		////unsigned char package_type;
		////package_type = read_comp_info(&control_com, &nod_info, &manual_mode, &dest_list);
		
		//Going into manual mode if that is said from PC
		////if(manual_mode){
			//Also check if exit manual mode is false, and exit if that is the case
		////	control_mode = 0x06; //control mode = 6 is manual driving

		////}


		prev_control_mode = control_mode;
		
		//Step 6. "Inlasning av sensorinfo"
		//waiting to receive info from sensor module
		while(counter_UART1_reciever < 9); //checking if enough bytes in buffer to read
		
		Sens_info_read(sens_info_ptr);

		
		if(sensor_info.dist_to_stop_line != 0x00 && control_mode == 0x00){ 
			control_mode = 0x04;
		} 
		
		if(control_mode == 0x04 && prev_control_mode == 0x00){          //if stop line has been detected three times. Number of time detected might have to be adjusted
			//Decision making for the incoming node phase.
			next_turn_decision = direction[node_route[n]];
			previous_node = node_route[n];
			next_node = node_route[n+1];
			n++;
			
		} else if(sensor_info.dist_to_stop_line == 0x00 && control_mode == 0x04){
			//Checking what to do next (next turn decision)
			unsigned short temp_node_type;
			temp_node_type = nod_info[(node_route[n]-1) * 7];
			//if-cases that decides what car should do next
			if(temp_node_type == 0x04){ //If stopplinje-node
				control_mode == 0x00;  //drive "vanlig vag"
				
			} else if(temp_node_type == 0x01){ //if crossing
				if(next_turn_decision == 'f'){ // if super-easy straight forward driving
					control_mode = 0x00;      //drive normal way
				} else { //all other turns in crossing
					control_mode = 0x01; //crossing mode for control mode
				}
			}
		}
		
		
		if(control_mode == 0x01 && prev_control_mode == 0x04 && next_turn_decision != 'F'){  //Reading angle before crossing, to have reference to when crossing mode done
			angle_before_crossing = sensor_info.angle;
		}
		
		//Reading angle and making it absolute value
		unsigned char abs_angle_temp;
		abs_angle_temp = sensor_info.angle - angle_before_crossing;
		if(abs_angle_temp < 0){
			abs_angle_temp = -abs_angle_temp;
		}
		
		//checking if crossing mode done
		if((control_mode == 0x01) && (abs_angle_temp > 80) && (next_turn_decision != 'F')){ 
				control_mode = 0x00; //if crossing mode done, set normal-way-driving
		}
		
		//TO FIX
		//Fix a way to check when 'F'-decision is done
		if(control_mode == 0x01 && next_turn_decision == 'F' && 0){
			control_mode = 0x00;
		}
		
		

		//Step 1.
		
		//Step 2. "Utskrivning av sensorinfo till styrmodul". When this happens the first round, the styrmodul starts (?).
		//To styrmodul
		spi_send_byte(0xFF); //header byte
		spi_send_byte(0x00); //control_mode); //control mode
		spi_send_byte(sensor_info.dist_right_line); //Distance right line
		spi_send_byte(sensor_info.angular_diff); //ang diff
		spi_send_byte(sensor_info.dist_sonic_middle); //UL front
		spi_send_byte(sensor_info.dist_sonic_left);	//UL left
		spi_send_byte(sensor_info.dist_sonic_right); //UL right
		spi_send_byte(sensor_info.dist_sonic_back); //UL back
		spi_send_byte(sensor_info.angle); //Angle
		spi_send_byte(next_turn_decision); //Next turn decision,  (fd car_speed)
		spi_send_byte(sensor_info.dist_to_stop_line); //dist to stop line
		//spi_send_byte(sensor_info.sign_type); //sign type
		//10+1 bytes
		
		
		
		//Step 4. "Bearbetning av rutt". Calculate optimal way, if the destination list has been changed by the user.
		
		
		
		//Step 5. "Utskrivning av sensor- och styrinfo till PC".
		uart0_send_byte(0xFF); //header byte
		uart0_send_byte(control_mode); //control mode
		uart0_send_byte(0x01); //package type 0x01, not sure if needed. Right now sending as scrap-byte, to make it sync (for some reason...)
		uart0_send_byte(esc_valueH); //escValue<<8
		uart0_send_byte(esc_valueL); //escValue
		uart0_send_byte(steering_valueH); //steeringValue <<8
		uart0_send_byte(steering_valueL); //steeringValue
		uart0_send_byte(sensor_info.dist_right_line); //Distance to right line
		uart0_send_byte(sensor_info.angular_diff); //Angular diff
		uart0_send_byte(sensor_info.dist_sonic_middle); //UL front
		uart0_send_byte(sensor_info.dist_sonic_left); //UL left
		uart0_send_byte(sensor_info.dist_sonic_right); //UL right
		uart0_send_byte(sensor_info.dist_sonic_back); //UL back
		uart0_send_byte(sensor_info.angle); //Angle
		uart0_send_byte(sensor_info.car_speed); //speed
		uart0_send_byte(sensor_info.dist_to_stop_line); //Distance to stop line
		//uart0_send_byte(sensor_info.sign_type); //signs that are seen
		//uart0_send_byte(0x01); //crossing type, don't need
		uart0_send_byte(next_turn_decision); //next turn decision
		uart0_send_byte(previous_node); //previous node
		uart0_send_byte(next_node); //next node
		//17+1 bytes
		
		
		
		
		//Step 3. "Inlasning av styrinfo". Should not happen the first round."
		//Read from styrmodul
		if(is_spi_package_received()){
			read_steering_info(&esc_valueL, &esc_valueH, &steering_valueL, &steering_valueH);
		}
		
		
		
		
		
		/*---Manual loop---*/
		
		//Step 1. Wait for control commando to read. Then read and proceed. Timing will be controlled from PC, to avoid getting overflowed buffer.
		
		//Step 2. Write control commando to styrmodul.
		
		//Step 3. Read sensor_information.
		
		//Step 4. Write sensorinformation to PC.
		
		
	}
	
}
















