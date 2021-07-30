/*
 * B_CIB.H
 *
 *  Created on: Mar 4, 2015
 *      Author: gzwbh0
 *      	added BOARD_ID and SOFTWARE_VERSION elements
 *      	L. Cronk 27JUL15
 */

#ifndef B_CIB_H_
#define B_CIB_H_


struct board_config_stuct		// structure to hold CIB configuration, chars for now
{
	char DIGITAL[50];	// "NO" or "YES"
	char TCPIP[100];		// "NO" or "YES,slot #, IP address, MAC address, snapped_to_IP address"
	char SENSOR[50];	// "NO" or "YES,0-1-2-3"  -- Type 1 or 2 or 3 sensor board design
	char FLEX[50];		// "NO" or "YES"
	char BOARD_ID[3];	// "xx"  hex switch value
	char SOFTWARE_VERSION[50];	// program written for which product and software revision level
};




#endif /* B_CIB_H_ */
