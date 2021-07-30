/*
 * DAQ_mode.h
 *
 *  Created on: Jan 9, 2018
 *      Author: TZZ176
 */

#ifndef INCLUDE_DAQ_MODE_H_
#define INCLUDE_DAQ_MODE_H_

#endif /* INCLUDE_DAQ_MODE_H_ */

#include "hal_stdtypes.h"

/* Set RS-485 transmitter enable, push transmission of a string and reset SCI receiver to receive an address  */

void	init_DAQ_mode_fray(void);
void 	init_DAQ_mode_CAN(void);

void 	swap_addresses_based_on_type(void);
unsigned char atoh (unsigned char data);

void	handle_DAQ_notification_fray(void);
void	handle_DAQ_notification_CAN(void);

void 	init_XCP_fast_rate(void);
void 	init_XCP_build_msg_fray(void);
void 	init_XCP_build_msg_CAN(void);
void 	handle_XCP_fast_rate_notification(void);

void 	print_periodic_variables(void);


