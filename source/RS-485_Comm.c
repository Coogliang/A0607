/*
 * RS-485_Comm.c
 *
 *  Created on: Jul 7, 2014
 *      Author: Leonard M. Cronk
 */


#include "hal_stdtypes.h"
#include "RS-485_Comm.h"
#include "sci.h"
#include "string.h"
#include "reg_sci.h"
#include "reg_spi.h"
#include "gio.h"
#include "het.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <limits.h>

#define CIB 0
#define MONITOR 1
#define WEB 2


#define ADDR_MODE 0x100 //Necessary for RS485 code

#define TX_EMPTY 0x800  //Flag indicating SCI shift register is empty.


// E X T E R N A L   M A I N  V A R S

extern unsigned long LED_TXD_msec_clock;    // rolling time base used for delay, USER LED#4
extern int scilinREG_is_CIB_or_Monitor_or_WEB; // declare this int in this function

extern int WEB_header_TX_complete;         // Used in WEB mode, flag to indicate constructed header portion of LAN message is complete, time to transmite actual message
extern uint32 WEB_data_length;  // Length of return message package for LAN, does not include header portion

extern char words[9][50];

void    transmit_str_485(char * return_message)
{
    uint8 header[60];
    uint32 header_length;

//    uint32 data_length;  needs to be global
    uint32 length;
    uint32 total_length;

    switch(scilinREG_is_CIB_or_Monitor_or_WEB)
    {
    case CIB:

        scilinREG->GCR1 = scilinREG->GCR1 | ADDR_MODE;      //set to interrupt on Address Message only      old: sp_con = MODE2 | REC_ENABLE;
        gioSetBit(hetPORT1, 16, 1); // Set RS485 CS to Receive
        sciSend(scilinREG, strlen(return_message), return_message);     /* USER CODE END */

        break;

    case MONITOR:


        break;

    case WEB:
        /*  return_message string packet in WEB Mode consist of 3 components minimum.
         *  All elements after HeaderTitle are separated by commas, last entry is followed by a "\r".
         *
         *      Packet Size, in bytes does not include this 6 digit decimal number, it is the first element in the string (no comma after)
         *      Header Title, found in command parser at words[0] location, usually the command title.  words[0] can be loaded with unusual push if needed
         *      Header Data, usually a "\r" terminated string, do not use terminator here if a SubHeader follows.
         *          SubHeader Title, added after Header Data to string is needed.
         *          SubHeader Data, usually a "\r" terminated string.
         */
        // local var:  50 for words[0], 6 for decimal number and couple for extra
//        uint8 header[60];
//        uint32 header_length;
//        uint32 data_length;
//        uint32 length;
//        uint32 total_length;

        // Determine # of bytes in header + data
        header_length = strlen(words[0]) + 1;  // adding "\r" to command for length

        WEB_data_length = strlen(return_message);        //debug
        total_length = header_length + WEB_data_length;

        sprintf(header,"%06d",total_length);          // format total length into 6 digit dddddd number for transmission
        strcat(header,words[0]);                // combine length string with command string
        strcat(header,"\r");                    // add carriage return "\r' to end of string

        //WEB_header_TX_complete = FALSE;         // sciNotification(sciBASE_t *sci, uint32 flags)  case SCI_TX_INT: will set completion flag
                                                //  ok to transmit rest of message

        // Send just header, we don't want to add data to header string, it may be large number of bytes.
        sciSend(scilinREG, header_length + 6 , header);     // add the 6 digits to count for sci send

        while((scilinREG->FLR & TX_EMPTY) == 0x800);
        while((scilinREG->FLR & TX_EMPTY) != 0x800);

        sciSend(scilinREG,WEB_data_length, return_message);
        WEB_header_TX_complete = TRUE;  // the ISR will be hit again after the data packet is complete, set this flag so we only run once

        break;

    }   //end switch(scilinREG_is_CIB_or_Monitor_or_WEB)


    // if i ever had to debug string coming in --- set break and step
//  char new_string[750];
//  int new_string_len;
//  new_string_len = strlen(return_message);
//  strcpy(new_string,return_message );



    // talk LED is now use for something else
//  LED_TXD_msec_clock = 300;       // set LED on for 300mS
//  gioSetBit(hetPORT1, 14, 1 );        // SET LED1

}
