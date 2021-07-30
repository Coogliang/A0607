/*
 * RS-485_Comm.h
 *
 *  Created on: Jul 7, 2014
 *      Author: Leonard M. Cronk
 */

#ifndef RS_485_COMM_H_
#define RS_485_COMM_H_

#include "hal_stdtypes.h"

/* Set RS-485 transmitter enable, push transmision of a string and reset SCI reciver to recieve an address  */

void	transmit_str_485( char * return_message);



#endif /* RS_485_COMM_H_ */
