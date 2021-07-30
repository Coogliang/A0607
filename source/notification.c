/** @file notification.c 
*   @brief User Notification Definition File
*   @date 11-Dec-2018
*   @version 04.07.01
*
*   This file  defines  empty  notification  routines to avoid
*   linker errors, Driver expects user to define the notification. 
*   The user needs to either remove this file and use their custom 
*   notification function or place their code sequence in this file 
*   between the provided USER CODE BEGIN and USER CODE END.
*
*/

/* 
* Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* Include Files */

#include "esm.h"
#include "sys_selftest.h"
#include "adc.h"
#include "can.h"
#include "gio.h"
#include "mibspi.h"
#include "sci.h"
#include "het.h"
#include "rti.h"
#include "i2c.h"
#include "sys_dma.h"

/* USER CODE BEGIN (0) */

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <limits.h>
#include "reg_system.h"


#include "RS-485_Comm.h"
#include "B_CIB.h"
#include "DAQ_mode.h"

//#include "variable_input.h"
#include "NTC_get.h"

#include "flexray.h"

/***************************************************************************/
/*                                                                         */
/* Usefull bit macros. NEED TO CHECK - BIG ENDIAN functionality            */
/*                                                                         */
/***************************************************************************/

#define   checkbit(var,bit)  (var & (0x01 << (bit)))
#define   setbit(var,bit)    (var |= (0x01 << (bit)))
#define   clrbit(var,bit)    (var &= (~(0x01 << (bit))))

/**************************************************************************/
/*                           New 13AUG12 L. Cronk                         */
/* a=target variable, b=bit number to act upon 0-n                        */
/*                                                                        */
/**************************************************************************/
#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1<<(b)))
#define BIT_CHECK(a,b) ((a) & (1<<(b)))
/* x=target variable, y=mask */
#define BITMASK_SET(x,y) ((x) |= (y))
#define BITMASK_CLEAR(x,y) ((x) &= (~(y)))
#define BITMASK_FLIP(x,y) ((x) ^= (y))
#define BITMASK_CHECK(x,y) ((x) & (y))

#define D_SIZE 9

#define OFF 0
#define ON 1
#define PASS 1
#define FAIL 0
#define TOGGLE 2
#define FLASH 3

#define INIT 0
#define NOT_VALID 0
#define NONE 0
#define RUN 1
#define PAUSE 2
#define STOP 3
#define DERATE 4
#define RUN_ERROR 5
#define NO_CAN 6
#define BACKLIGHT 7
#define SETUP 8

#define IDLE 0
#define REQUEST 1
#define ACTIVE 2

// #define TRUE  1
// #define FALSE  0
#define POSITIVE 1
#define NEGATIVE 0
#define SET 1
#define CLR 0
#define YES 1
#define NO 0
#define POS 1
#define NEG 0
#define NORMAL 1
#define WARM 2
#define CRANK 3
#define HOLD 4

#define TRIGGERED   2   /* state of health interval has expired */
#define COMPLETE 2      /* power mode sequence complete */
#define LOADED 3        /* power mode sequence has some values loaded */

#define HIGH 1
#define LOW 0
#define NONE 0
#define SNAPPING 1
#define SNAPPED1 2
#define SNAPPED2 3
#define RESET 4
#define CIB 0
#define MONITOR 1
#define WEB 2


#define pos_0prcnt 0    /* TOC switch case labels  - 9BXX program*/
#define pos_2prcnt 1
#define neg_2prcnt 2
#define pos_8prcnt 3
#define neg_8prcnt 4
#define pos_16prcnt 5
#define neg_16prcnt 6
#define pos_30prcnt 7
#define neg_30prcnt 8
#define pos_95prcnt 9
#define neg_95prcnt 10

#define pos_0nm     0   /* TOC and Digital sensors switch case labels  - C1xx, T1XX and FCA ADAS programs*/
#define pos_p1nm    1
#define neg_p1nm    2
#define pos_p5nm    3
#define neg_p5nm    4
#define pos_1p00nm  5
#define neg_1p00nm  6
#define pos_1p50nm  7
#define neg_1p50nm  8
#define pos_2p00nm  9
#define neg_2p00nm  10
#define pos_2p50nm  11
#define neg_2p50nm  12
#define pos_3p00nm  13
#define neg_3p00nm  14
#define pos_3p50nm  15
#define neg_3p50nm  16
#define pos_4p00nm  17
#define neg_4p00nm  18
#define pos_6p00nm  19
#define neg_6p00nm  20

#define PRINT_ONE_CHANNEL 1
#define PRINT_ALL_CHANNELS 2

#define ADDR_MODE 0x100 //Necessary for RS485 code

#define TX_EMPTY 0x800  //Flag indicating SCI shift register is empty.

#define ALL 3
#define ME 1
#define BUFFER 0

#define TMS570 0
#define RENSIS 1

#define BAIRBOARD 0
#define FULL 1

#define TARGET_T1XX 1
#define TARGET_9BXX 2
#define TARGET_S550ANALOG 3
#define TARGET_S550DIGITAL 4
#define TARGET_C1XX 5
#define TARGET_FCA_ADAS 6
#define TARGET_CD391_ADAS 7
#define TARGET_G2KCA_ADAS 8
#define TARGET_PSA_CMP 9
#define TARGET_SGMW_CN200 10
#define TARGET_FORD_T3_T6 11
#define TARGET_GM_B_T1XX 12
#define TARGET_RENAULT_NISSAN 13
#define TARGET_K2XX_MTO 14
#define TARGET_T1XX_MTO 15
#define TARGET_BMW_FAAR_WE 16
#define TARGET_BMW_UKL 17
#define TARGET_EHPS 18
#define TARGET_SGMW_CN300 19
#define TARGET_BYD_SA2FL 20
#define TARGET_GWM_A0607 21


/*  ------------- CAN_DATA_REQUESTS  :: Set for number CAN variable request   T1XX = 28   C1XX = 18    B9XX = 36    FCA = 18 -------  */
#define T1XX_CAN_DATA_REQUESTS 28                    /* Used for can1_request_index != can_index_value (previously just a #). Created for easier updating of code */
#define BYD_SA2FL_CAN_DATA_REQUESTS 11                 /* Used for can1_request_index != can_index_value (previously just a #). Created for easier updating of code */
#define GWM_A0607_CAN_DATA_REQUESTS 11                 /* Used for can1_request_index != can_index_value (previously just a #). Created for easier updating of code */
#define C1XX_CAN_DATA_REQUESTS 18                    /* Used for can1_request_index != can_index_value (previously just a #). Created for easier updating of code */
#define GM_9BXX_CAN_DATA_REQUESTS 36                 /* Used for can1_request_index != can_index_value (previously just a #). Created for easier updating of code */
#define FCA_ADAS_CAN_DATA_REQUESTS 9                 /* Used for canX_request_index != can_index_value (previously just a #). Created for easier updating of code */
#define CD391_ADAS_CAN_DATA_REQUESTS 15              /* Used for canX_request_index != can_index_value (previously just a #9). Larger for problems monitoring */
#define G2KCA_ADAS_CAN_DATA_REQUESTS 9               /* Used for canX_request_index != can_index_value (previously just a #). Created for easier updating of code */
#define PSA_CMP_CAN_DATA_REQUESTS 17                 /* Used for canX_request_index != can_index_value (previously just a #). Created for easier updating of code */
#define SGMW_CN200_CAN_DATA_REQUESTS 13              /* Used for canX_request_index != can_index_value (previously just a #). Created for easier updating of code */
#define FORD_T3_T6_CAN_DATA_REQUESTS 9               /* Used for canX_request_index != can_index_value (previously just a #). Created for easier updating of code */
#define RENAULT_NISSAN_CAN_DATA_REQUESTS 13          /* Used for canX_request_index != can_index_value (previously just a #). Created for easier updating of code */
#define TARGET_BMW_FAAR_WE_FRAY_DATA_REQUESTS 20     /* Used for canX_request_index != can_index_value (previously just a #). Created for easier updating of code */
#define TARGET_BMW_UKL_FRAY_DATA_REQUESTS 15         /* Used for canX_request_index != can_index_value (previously just a #). Created for easier updating of code */


#define TESTER_PRESENT_SEND_TIME 1500;
#define XCP_TIMEOUT_VALUE 280 //XCP request timeout value
#define CCP_TIMEOUT_VALUE 280 //CCP request timeout value

void hcurr_change_direction_at_nm(float target);    // function to swap T1/T2 direction at assigned torque (0.0, 0.1, 0.5, 1.0, 2.0, 4.0, 6.0) are valid values

// E X T E R N A L   M A I N  V A R S



int i;      // test var

// board personality configuration vars
extern int target_product;
extern int manufacturing_TOC;       // flag indicates we are using manufacturing TOC with Tester Present CAN messages    or    SENT torque simulation
extern int processor_type;  // flag indicates if using BIG Endian(TMS570 processor) or LITTLE Endian(RENSIS processor) used to evaluate (parse) CAN data
extern int CIB_analog_config;   // flag indicates analog configuration, BAIRBOARD or FULL.  BAIRBOARD has reduced channel count, process a little differently.


extern struct board_config_stuct CIB_config_data;   // CIB_config is a structured variable of type---board_config_stuct


extern int  aa,bb,cc,dd,ee;     // debug test vars, del later
extern unsigned timer_a, timer_b, timer_c, timer_d, timer_e, timer_f, timer_g, timer_h, timer_i, timer_j, timer_k;  // used to benchmark execution of code segments
extern int spi_a,spi_b,spi_c,spi_d,spi_e,spi_f,spi_g;    // debug vars, del later


extern unsigned int het_test1, het_test2, het_test3;    // debugging het interrupts
extern unsigned int HET_msec_countdown_cnt;     // count down timer for delay of start for HET SENT signal generation,, after i2C has completed initilization

extern unsigned long system_msec_clock; // rolling time base (msec ticks) used for delays updated in rti.c
extern unsigned long system_msec_clock_temp;    // rolling time base used for delays, time now
extern unsigned long LED_msec_clock;    // rolling time base used to generate USER LED action for program status
extern unsigned long p_off_continue_periodics_timer; // countdown timer to delay CAN periodic messages after p_off, new for SGMW CN200

//extern unsigned long LED_REC_msec_clock;  // rolling time base used for delay, USER LED#3 (not used in MicroCore)
//extern unsigned long LED_TXD_msec_clock;  // rolling time base used for delay, USER LED#4 (not used in MicroCore)


extern int SW1_high_or_low;             // Initialized to "high" and set to "low" in GIO notification during ISR
extern int SW1_is_snapping_snapped1_snapped2_reset_none;    // init "none", if-when user pushes"snapping", based on sw1 time low: var -> ("snapped1 then "snapped2")or"reset"
extern unsigned long  SW1_active_low_time;      // time stamp of when falling edge of SW1 occurred
extern unsigned long SW1_time_held_low;         // time in mS, SW1 was held low by user, debug
extern unsigned long SNAP_timeout_clock;        // set in GIO notification, and polled in RTI notification for timeout of SNAP feature valid for 10sec
extern unsigned long Monitor_delay_timer;       // used to add delays between xport MONITOR commands in SNAP activity
extern int snapping_LED_scroll;         // simple counter for setting states for LEDs during SNAP activity
extern int scilinREG_is_CIB_or_Monitor_or_WEB;  // sci2 has many functions, first we use Monitor, then switch to CIB
extern int monitor_rec_index;               // within MONITOR, control communication flow in SEND and RECEIVE dialog
extern char command_str_mirror[255];        // save original read from Lantronix, command_str[] is deconstucted into WORDS[][] improperly for parsing string
extern char snap_authorization_str[75]; // store IP of whom im snapped to

extern int WEB_header_TX_complete;         // Used in WEB mode, flag to indicate constructed header portion of LAN message is complete, time to transmite actual message
extern uint32 WEB_data_length;  // Length of return message package for LAN, does not include header portion


extern unsigned int tester_present_timer; // count down variable to send tester present message every 3 seconds or every TESTER_PRESENT_SEND_TIME
extern int tester_present_enable_flag; // flag to start and stop the sending of tester present message
// used so we can stop the message in 'warminit mode' so etool can flash parts

extern int trig_flag; // flag to signal dumperr that a trig has been done..reset after dump
extern int ign1_status;     // combine function of ign_on_flag and Engine_on_Signal Off, Normal and Warm (added dual ign for ADAS)
extern int ign2_status;     // combine function of ign_on_flag and Engine_on_Signal Off, Normal and Warm (added dual ign for ADAS)

// used for building CAN speed, rolling counter and CRC bytes
extern uint8 Veh_speed_count;
extern const uint8 Veh_speed_0_table[16][8];
extern const uint8 Veh_speed_0_table_Ford[16][8];
extern const uint8 Veh_speed_0_table_PSA[16][8];
extern const uint8 Veh_speed_0_table_BYD_SA2FL[16][8];
extern const uint8 Veh_speed_00_table_BYD_SA2FL[16][8];

extern int G2KCA_ADAS_crcVal_16;                   // G2KCA speed message calculation
extern uint8 G2KCA_ADAS_Write_Vehicle_speed[8];      // G2KCA speed message calculation/
extern unsigned char G2KCA_ADAS_CRC_counter;       // G2KCA speed message calculation


extern int  ucurr_flag;                 // flag to main to process user initiated TOC commands
extern int  ucurr_direction_POS_NEG;    // ucurr direction swapper
extern int stop_T1T2switching;          // flag to RTI notification to start/stop switching for low current at 1 sec intervals

extern int  hcurr_direction_POS_NEG;    // hcurr direction swapper, actual direction,   POSITIVE or NEGATIVE
extern int  hcurr_flag;                 // flag to epa4 and main to determine if and when to process load profile
extern int  hcurr_timer;                // count up timer in 1mSec ticks for hcurr load profile timing

extern int hcurr_write_interval_data;   // flag set in notification RTI timer, its time to update load value
extern int  torque_value_case_index;    // set index to value of case to load torque value, see constants

extern unsigned int high_torque_timer_flag; // set flag for notification RTI timer  NOT USED
extern int high_torque_timer_counter; // counter for 10 seconds. counts down in notification RTI timer NOT USED


//  E X T E R N A L   C A N   V A R S
/******************************************************************************************************************************/
/******************************************************************************************************************************/

// C A N 2 -- BMW FAAR WE L02 -- M T S   S T A N D system lab test stand
//70 CAN messages required
//Get it down to 64...


//Inputs into CIB to command angle and torque. Convert these into DtoA outputs.

//Feedback signals (1/5/10 Hz) (periodic outputs)

#define L02_MTS_D1_1D1_Motor_Angle_out               canMESSAGE_BOX1     // 0x1D1   11bit identifier--
#define L02_MTS_D1_2D1_Motor_Torque_out              canMESSAGE_BOX2     // 0x2D1   11bit identifier--
#define L02_MTS_D1_3D1_Motor_Current_out             canMESSAGE_BOX3     // 0x3D1   11bit identifier--
#define L02_MTS_D1_4D1_Motor_Temp_out                canMESSAGE_BOX4     // 0x4D1   11bit identifier--
#define L02_MTS_D1_5D1_Controller_Current_out        canMESSAGE_BOX5     // 0x5D1   11bit identifier--

#define L02_MTS_D2_1D2_Motor_Angle_out               canMESSAGE_BOX6     // 0x1D2   11bit identifier--
#define L02_MTS_D2_2D2_Motor_Torque_out              canMESSAGE_BOX7     // 0x2D2   11bit identifier--
#define L02_MTS_D2_3D2_Motor_Current_out             canMESSAGE_BOX8     // 0x3D2   11bit identifier--
#define L02_MTS_D2_4D2_Motor_Temp_out                canMESSAGE_BOX9     // 0x4D2   11bit identifier--
#define L02_MTS_D2_5D2_Controller_Current_out        canMESSAGE_BOX10    // 0x5D2   11bit identifier--

#define L02_MTS_D3_1D3_Motor_Angle_out               canMESSAGE_BOX11    // 0x1D3   11bit identifier--
#define L02_MTS_D3_2D3_Motor_Torque_out              canMESSAGE_BOX12    // 0x2D3   11bit identifier--
#define L02_MTS_D3_3D3_Motor_Current_out             canMESSAGE_BOX13    // 0x3D3   11bit identifier--
#define L02_MTS_D3_4D3_Motor_Temp_out                canMESSAGE_BOX14    // 0x4D3   11bit identifier--
#define L02_MTS_D3_5D3_Controller_Current_out        canMESSAGE_BOX15    // 0x5D3   11bit identifier--

#define L02_MTS_D4_1D4_Motor_Angle_out               canMESSAGE_BOX16    // 0x1D4   11bit identifier--
#define L02_MTS_D4_2D4_Motor_Torque_out              canMESSAGE_BOX17    // 0x2D4   11bit identifier--
#define L02_MTS_D4_3D4_Motor_Current_out             canMESSAGE_BOX18    // 0x3D4   11bit identifier--
#define L02_MTS_D4_4D4_Motor_Temp_out                canMESSAGE_BOX19    // 0x4D4   11bit identifier--
#define L02_MTS_D4_5D4_Controller_Current_out        canMESSAGE_BOX20    // 0x5D4   11bit identifier--

#define L02_MTS_D5_1D5_Motor_Angle_out               canMESSAGE_BOX21    // 0x1D5   11bit identifier--
#define L02_MTS_D5_2D5_Motor_Torque_out              canMESSAGE_BOX22    // 0x2D5   11bit identifier--
#define L02_MTS_D5_3D5_Motor_Current_out             canMESSAGE_BOX23    // 0x3D5   11bit identifier--
#define L02_MTS_D5_4D5_Motor_Temp_out                canMESSAGE_BOX24    // 0x4D5   11bit identifier--
#define L02_MTS_D5_5D5_Controller_Current_out        canMESSAGE_BOX25    // 0x5D5   11bit identifier--

#define L02_MTS_D6_1D6_Motor_Angle_out               canMESSAGE_BOX26    // 0x1D6   11bit identifier--
#define L02_MTS_D6_2D6_Motor_Torque_out              canMESSAGE_BOX27    // 0x2D6   11bit identifier--
#define L02_MTS_D6_3D6_Motor_Current_out             canMESSAGE_BOX28    // 0x3D6   11bit identifier--
#define L02_MTS_D6_4D6_Motor_Temp_out                canMESSAGE_BOX29    // 0x4D6   11bit identifier--
#define L02_MTS_D6_5D6_Controller_Current_out        canMESSAGE_BOX30    // 0x5D6   11bit identifier--



#define MTS_001_Vehicle_Speed_flt_in                 canMESSAGE_BOX31    // 0x001   11bit identifier--
#define L02_MTS_004_System_State_in                  canMESSAGE_BOX32    // 0x004   11bit identifier--byte[0]: 00=OFF, 02=RUN, 03=WINIT
#define L02_MTS_005_Store_DAQ_Data_Mode_in           canMESSAGE_BOX33    // 0x005   11bit identifier--01 = DAQ store 00 = DAQ NOT stored

//Plan to start DAQ mode right after p_on, but not output any data to labview until MTS stand says so.

#define L02_MTS_D1_11D_Angle_Cmd_fp_in               canMESSAGE_BOX34    // 0x11D   11bit identifier--
#define L02_MTS_D1_12D_Torque_Cmd_fp_in              canMESSAGE_BOX35    // 0x12D   11bit identifier--
#define L02_MTS_D2_21D_Angle_Cmd_fp_in               canMESSAGE_BOX36    // 0x21D   11bit identifier--
#define L02_MTS_D2_22D_Torque_Cmd_fp_in              canMESSAGE_BOX37    // 0x22D   11bit identifier--
#define L02_MTS_D3_31D_Angle_Cmd_fp_in               canMESSAGE_BOX38    // 0x31D   11bit identifier--
#define L02_MTS_D3_32D_Torque_Cmd_fp_in              canMESSAGE_BOX39    // 0x32D   11bit identifier--
#define L02_MTS_D4_41D_Angle_Cmd_fp_in               canMESSAGE_BOX40    // 0x41D   11bit identifier--
#define L02_MTS_D4_42D_Torque_Cmd_fp_in              canMESSAGE_BOX41    // 0x42D   11bit identifier--
#define L02_MTS_D5_51D_Angle_Cmd_fp_in               canMESSAGE_BOX42    // 0x51D   11bit identifier--
#define L02_MTS_D5_52D_Torque_Cmd_fp_in              canMESSAGE_BOX43    // 0x52D   11bit identifier--
#define L02_MTS_D6_61D_Angle_Cmd_fp_in               canMESSAGE_BOX44    // 0x61D   11bit identifier--
#define L02_MTS_D6_62D_Torque_Cmd_fp_in              canMESSAGE_BOX45    // 0x62D   11bit identifier--

//Fault State messages (event driven outputs)
#define L02_MTS_D1_711_General_Fault_State_out       canMESSAGE_BOX46    // 0x711   11bit identifier--
#define L02_MTS_D1_712_FET_Temp_Fault_out            canMESSAGE_BOX47    // 0x712   11bit identifier-- Set limits around this
#define L02_MTS_D1_713_Motor_Temp_Fault_out          canMESSAGE_BOX48    // 0x713   11bit identifier-- Set limits around this
#define L02_MTS_D2_721_General_Fault_State_out       canMESSAGE_BOX49    // 0x721   11bit identifier--
#define L02_MTS_D2_722_FET_Temp_Fault_out            canMESSAGE_BOX50    // 0x722   11bit identifier-- Set limits around this
#define L02_MTS_D2_723_Motor_Temp_Fault_out          canMESSAGE_BOX51    // 0x723   11bit identifier-- Set limits around this
#define L02_MTS_D3_731_General_Fault_State_out       canMESSAGE_BOX52    // 0x731   11bit identifier--
#define L02_MTS_D3_732_FET_Temp_Fault_out            canMESSAGE_BOX53    // 0x732   11bit identifier-- Set limits around this
#define L02_MTS_D3_733_Motor_Temp_Fault_out          canMESSAGE_BOX54    // 0x733   11bit identifier-- Set limits around this
#define L02_MTS_D4_741_General_Fault_State_out       canMESSAGE_BOX55    // 0x741   11bit identifier--
#define L02_MTS_D4_742_FET_Temp_Fault_out            canMESSAGE_BOX56    // 0x742   11bit identifier-- Set limits around this
#define L02_MTS_D4_743_Motor_Temp_Fault_out          canMESSAGE_BOX57    // 0x743   11bit identifier-- Set limits around this
#define L02_MTS_D5_751_General_Fault_State_out       canMESSAGE_BOX58    // 0x751   11bit identifier--
#define L02_MTS_D5_752_FET_Temp_Fault_out            canMESSAGE_BOX59    // 0x752   11bit identifier-- Set limits around this
#define L02_MTS_D5_753_Motor_Temp_Fault_out          canMESSAGE_BOX60    // 0x753   11bit identifier-- Set limits around this
#define L02_MTS_D6_761_General_Fault_State_out       canMESSAGE_BOX61    // 0x761   11bit identifier--
#define L02_MTS_D6_762_FET_Temp_Fault_out            canMESSAGE_BOX62    // 0x762   11bit identifier-- Set limits around this
#define L02_MTS_D6_763_Motor_Temp_Fault_out          canMESSAGE_BOX63    // 0x763   11bit identifier-- Set limits around this

#define MTS_7FF_MTS_Fault_in                         canMESSAGE_BOX64    // 0x7FF   11bit identifier-- Set limits around this



/******************************************************************************************************************************/
// C A N  1  --  M T O    K 2 X X and  T 1 X X     Product
#define PROD_MESS1_ID               canMESSAGE_BOX1      // 0x148    11bit identifier-- Rec T1XX Service Steering Light to EPS
#define CCP_RESPONSE_ID             canMESSAGE_BOX2      // 0x706    11bit identifier-- Rec CCP response from EPS
#define CCP_REQUEST_ID              canMESSAGE_BOX3      // 0x708    11bit identifier-- Trans CCP Request to EPS
#define CCP_SPEED_ID                canMESSAGE_BOX4      // 0x3E9    11bit identifier-- Trans Speed and Engine Status to EPS
#define CCP_ENGINE_RUN_ID           canMESSAGE_BOX5      // 0x0C9    11bit identifier-- Trans Engine Info to EPS
#define CCP_POWER_MODE_ID           canMESSAGE_BOX6      // 0x1F1    11bit identifier-- Trans Ignition Status to EPS
#define XCP_RESPONSE_ID             canMESSAGE_BOX7      // 0x642    11bit identifier-- Rec XCP Response from EPS for data
#define XCP_REQUEST_ID              canMESSAGE_BOX8      // 0x242    11bit identifier-- Trans XCP Request to EPS for data
#define DEBUG_TX_ID                 canMESSAGE_BOX9      // 0x7FF    11bit identifier-- Trans General info on buss for debugging
#define CCP_TRANSMISSION_ID         canMESSAGE_BOX10     // 0x1F5    11bit identifier-- Trans PPEI_Trans_General_Status2 to EPS

#define CCP_RIGHT_Pressure_ECHO_ID  canMESSAGE_BOX11     // 0x7F0   11bit identifier--  Trans Pressure Right echo to Test Stand
#define CCP_LEFT_Pressure_ECHO_ID   canMESSAGE_BOX12     // 0x7F1   11bit identifier--  Trans Pressure Left  echo to Test Stand

/******************************************************************************************************************************/
// C A N  1  --  R E N A U L T _ N I S S A N     Product   D. Bair
#define RENAULT_NISSAN_ECU1_CAN1_PROD_MESS1_ID          canMESSAGE_BOX1  // 0x148   11bit identifier-- Rec Response from EPS (not used)
#define RENAULT_NISSAN_ECU1_CAN1_PROD_MESS2_ID          canMESSAGE_BOX2  // 0x778   11bit identifier-- Rec ON Star from EPS (not used)
#define RENAULT_NISSAN_ECU1_CAN1_CCP_Engine_Run         canMESSAGE_BOX3  // 0x1F6   11bit identifier-- Trans Engine ON to EPS
#define RENAULT_NISSAN_ECU1_CAN1_CCP_Power_Mode_ID      canMESSAGE_BOX4  // 0x1F1   11bit identifier-- Trans Power Mode to EPS
#define RENAULT_NISSAN_ECU1_CAN1_CCP_SPEED_ID           canMESSAGE_BOX5  // 0x29A   11bit identifier-- Trans Speed to EPS
#define RENAULT_NISSAN_ECU1_CAN1_CCP_REQUEST_ID         canMESSAGE_BOX6  // 0x742   11bit identifier-- Trans CCP Request for Data to EPS
#define RENAULT_NISSAN_ECU1_CAN1_CCP_RESPONSE_ID        canMESSAGE_BOX7  // 0x762   11bit identifier-- Rec CCP Response from EPS for Data
#define RENAULT_NISSAN_ECU1_CAN1_XCP_EA3_REQUEST_ID     canMESSAGE_BOX8  // 0x242   11bit identifier-- Trans XCP Request from EPS for data (not used)
#define RENAULT_NISSAN_ECU1_CAN1_XCP_EA3_RESPONSE_ID    canMESSAGE_BOX9  // 0x642   11bit identifier-- Rec XCP Response to EPS for data (not used)
#define RENAULT_NISSAN_ECU1_CAN1_XCP_EA4_REQUEST_ID     canMESSAGE_BOX10 // 0x750   11bit identifier-- Trans XCP Request from EPS for data
#define RENAULT_NISSAN_ECU1_CAN1_XCP_EA4_RESPONSE_ID    canMESSAGE_BOX11 // 0x758   11bit identifier-- Rec XCP Response to EPS for data
#define RENAULT_NISSAN_TESTER_REC_DEBUG_ID              canMESSAGE_BOX12 // 0x7ff   11bit identifier-- Rec from Anyone
#define RENAULT_NISSAN_TESTER_TRANS_DEBUG_ID            canMESSAGE_BOX13 // 0x7fe   11bit identifier-- Transmit to Anyone
#define RENAULT_NISSAN_ECU1_CCP_EA4_AvgVehSpd_ID        canMESSAGE_BOX14 // 0x17D   11bit identifier-- Trans ABS message to EPS
#define RENAULT_NISSAN_ECU1_CAN1_GenericAppDiagEnble_ID canMESSAGE_BOX15 // 0x350   11bit identifier-- Trans Generic Application Diagnostic Enablr


// C A N  2  --  R E N A U L T _ N I S S A N     Product
#define RENAULT_NISSAN_ECU1_CAN2_PROD_MESS1_ID          canMESSAGE_BOX1  // 0x148   11bit identifier-- Rec Response from EPS (not used)
#define RENAULT_NISSAN_ECU1_CAN2_PROD_MESS2_ID          canMESSAGE_BOX2  // 0x778   11bit identifier-- Rec ON Star from EPS (not used)
#define RENAULT_NISSAN_TESTER_REC_DEBUG_ID              canMESSAGE_BOX12 // 0x7ff   11bit identifier-- Rec from Anyone
#define RENAULT_NISSAN_TESTER_TRANS_DEBUG_ID            canMESSAGE_BOX13 // 0x7fe   11bit identifier-- Transmit to Anyone

/******************************************************************************************************************************/
// C A N  1  --  G M _ B   T 1 X X     Product
#define GM_B_T1XX_ECU1_CAN1_CCP_NM_Frame_ID                     canMESSAGE_BOX1       //0x145      11 bit identifier -- Trans EPSECU Wakeup to EPS (NM_Frame)
#define GM_B_T1XX_ECU1_CAN1_CCP_sysPwrMode_Prtctd_ID            canMESSAGE_BOX2       //0x380      11 bit identifier -- Trans Power Mode to EPS (sysPwrMode_Prtctd)
#define GM_B_T1XX_ECU1_CAN1_CCP_sysPwrMode_PCSM_ID              canMESSAGE_BOX3       //0x727      11 bit identifier -- Trans Authoritative counter for 0x380 (sysPwrMode_PCSM)
#define GM_B_T1XX_ECU1_CAN1_CCP_prplStat_Prtctd_ID              canMESSAGE_BOX4       //0x384      11 bit identifier -- Trans propulsion state (prplStat_Prtctd)
#define GM_B_T1XX_ECU1_CAN1_CCP_prplStat_PCSM_ID                canMESSAGE_BOX5       //0x740      11 bit identifier -- Trans Authoritative counter for propulsion state (prplStat_PCSM)
#define GM_B_T1XX_ECU1_CAN1_CCP_frntAngVel_Prtctd_ID            canMESSAGE_BOX6       //0x015      11 bit identifier -- *Trans front vehicle speed (frntAngVel_Prtctd)
#define GM_B_T1XX_ECU1_CAN1_CCP_frntAngVel_PCSM_ID              canMESSAGE_BOX7       //0x21C      11 bit identifier -- *Trans Authoritative counter for front vehicle speed (frntAngVel_PCSM)
#define GM_B_T1XX_ECU1_CAN1_CCP_rearAngVel_Prtctd_ID            canMESSAGE_BOX8       //0x017      11 bit identifier -- *Trans Rear Vehicle speed (rearAngVel_Prtctd)
#define GM_B_T1XX_ECU1_CAN1_CCP_rearAngVel_PCSM_ID              canMESSAGE_BOX9       //0x21E      11 bit identifier -- *Trans Authoritative counter for rear vehicle speed (rearAngVel_PCSM)
#define GM_B_T1XX_ECU1_CAN1_CCP_whlDst_Prtctd_ID                canMESSAGE_BOX10      //0x71E      11 bit identifier -- *Trans Wheel Distance (whlDst_Prtctd)
#define GM_B_T1XX_ECU1_CAN1_CCP_whlDst_PCSM_ID                  canMESSAGE_BOX11      //0x711      11 bit identifier -- *Trans Authoritative Counter for WHeel Distance (whlDst_PCSM)
#define GM_B_T1XX_ECU1_CAN1_CCP_vehSpdDrvn_Prtctd_ID            canMESSAGE_BOX12      //0x229      11 bit identifier -- **Trans Unprotected vehicle speed used as backup if others fail -- no authoritative counter.(vehSpdDrvn_Prtctd)
#define GM_B_T1XX_ECU1_CAN1_CCP_REQUEST_ID                      canMESSAGE_BOX13      //0x18EFF131 29 bit identifier -- Trans CCP Request to EPS for data (XCP_TX)
#define GM_B_T1XX_ECU1_CAN1_CCP_RESPONCE_ID                     canMESSAGE_BOX14      //0x18EF31F1 29 bit identifier -- Rec CCP Response from EPS for Data (XCP_RX)
#define GM_B_T1XX_ECU1_CAN1_Active_Counter_REQUEST_ID           canMESSAGE_BOX15      //0x14DA31F1 29 bit identifier -- Trans Request to EPS for Active Counters (GM_TX)
#define GM_B_T1XX_ECU1_CAN1_Active_Counter_RESPONCE_ID          canMESSAGE_BOX16      //0x14DAF131 29 bit identifier -- Rec Active Counters from EPS (GM_RX)
#define GM_B_T1XX_ECU1_CAN1_XCP_EA4_REQUEST_ID                  canMESSAGE_BOX17      //0x712      11 bit identifier -- Trans XCP Request to EPS for data (MFG_TX)
#define GM_B_T1XX_ECU1_CAN1_XCP_EA4_RESPONCE_ID                 canMESSAGE_BOX18      //0x710      11 bit identifier -- Rec XCP Response from EPS for data (MFG_RX)
#define GM_B_T1XX_ECU1_CAN1_CCP_VEH_SPD_ID                      canMESSAGE_BOX19      //0x335      11 bit identifier -- Trans main vehicle speed (VEH_SPD)

// C A N  2  --  G M _ B   T 1 X X     Product

#define TESTER_REC_DEBUG_ID   canMESSAGE_BOX1    // 0x7ff   11bit identifier-- REC from Anyone
#define TESTER_TRANS_DEBUG_ID canMESSAGE_BOX2    // 0x7fe   11bit identifier-- Transmit to Anyone

/******************************************************************************************************************************/

// C A N  1  --  F O R D   T 3  - T 6    Product
#define FORD_T3_ECU1_CAN1_PROD_MESS1_ID         canMESSAGE_BOX1    // 0x148   11bit identifier-- Rec Response from EPS (not used)
#define FORD_T3_ECU1_CAN1_PROD_MESS2_ID         canMESSAGE_BOX2    // 0x778   11bit identifier-- Rec ON Star from EPS (not used)
#define FORD_T3_ECU1_CAN1_CCP_Engine_Run        canMESSAGE_BOX3    // 0x167   11bit identifier-- Trans Engine ON to EPS
#define FORD_T3_ECU1_CAN1_CCP_Power_Mode_ID     canMESSAGE_BOX4    // 0x3B3   11bit identifier-- Trans Power Mode to EPS(not used)
#define FORD_T3_ECU1_CAN1_CCP_SPEED_ID          canMESSAGE_BOX5    // 0x415   11bit identifier-- Trans Speed to EPS with counter and CRC

#define FORD_T3_ECU1_CAN1_CCP_REQUEST_ID        canMESSAGE_BOX6    // 0x60A   11bit identifier-- Trans CCP Request for Data to EPS
#define FORD_T3_ECU1_CAN1_CCP_RESPONSE_ID       canMESSAGE_BOX7    // 0x60B   11bit identifier-- Rec CCP Response from EPS for Data
#define FORD_T3_ECU1_CAN1_XCP_EA4_REQUEST_ID    canMESSAGE_BOX8    // 8x730   11bit identifier-- Trans XCP Request from EPS for data
#define FORD_T3_ECU1_CAN1_XCP_EA4_RESPONSE_ID   canMESSAGE_BOX9    // 0x738   11bit identifier-- Rec XCP Response to EPS for data

#define FORD_T3_ECU2_CAN1_CCP_REQUEST_ID        canMESSAGE_BOX10   // 0x50A   11bit identifier-- Trans CCP Request for Data to EPS
#define FORD_T3_ECU2_CAN1_CCP_RESPONSE_ID       canMESSAGE_BOX11   // 0x50B   11bit identifier-- Rec CCP Response from EPS for Data
#define FORD_T3_ECU2_CAN1_XCP_EA4_REQUEST_ID    canMESSAGE_BOX12   // 0x630   11bit identifier-- Trans XCP Request from EPS for data
#define FORD_T3_ECU2_CAN1_XCP_EA4_RESPONSE_ID   canMESSAGE_BOX13   // 0x638   11bit identifier-- Rec XCP Response to EPS for data

#define FORD_T3_ECU1_CAN1_TESTER_REC_DEBUG_ID   canMESSAGE_BOX14   // 0x7FF   11bit identifier-- REC from Anyone
#define FORD_T3_ECU1_CAN1_TESTER_TRANS_DEBUG_ID canMESSAGE_BOX15   // 0x7FE   11bit identifier-- Transmit to Anyone

// C A N  2  --  F O R D   T 3  - T 6    Product

#define FORD_T3_ECU1_CAN2_TESTER_REC_DEBUG_ID   canMESSAGE_BOX14   // 0x7FF   11bit identifier-- REC from Anyone , no actual CAN buss, placeholder
#define FORD_T3_ECU1_CAN2_TESTER_TRANS_DEBUG_ID canMESSAGE_BOX15   // 0x7FE   11bit identifier-- Transmit to Anyone , no actual CAN buss, placeholder

/******************************************************************************************************************************/

// C A N  1  --  C N 2 0 0     N O N    A D A S   Product
#define CN200_ECU1_CAN1_PROD_MESS1_ID       canMESSAGE_BOX1  // 0x148   11bit identifier-- Rec Response from EPS (not used)
#define CN200_ECU1_CAN1_PROD_MESS2_ID       canMESSAGE_BOX2  // 0x778   11bit identifier-- Rec ON Star from EPS (not used)
#define CN200_ECU1_CAN1_CCP_Engine_Run      canMESSAGE_BOX3  // 0x0C9   11bit identifier-- Trans Engine ON to EPS
#define CN200_ECU1_CAN1_CCP_Power_Mode_ID   canMESSAGE_BOX4  // 0x1F1   11bit identifier-- Trans Power Mode to EPS
#define CN200_ECU1_CAN1_CCP_SPEED_ID        canMESSAGE_BOX5  // 0x348   11bit identifier-- Trans Speed to EPS

#define CN200_ECU1_CAN1_CCP_REQUEST_ID      canMESSAGE_BOX6  // 0x708   11bit identifier-- Trans CCP Request for Data to EPS
#define CN200_ECU1_CAN1_CCP_RESPONSE_ID     canMESSAGE_BOX7  // 0x706   11bit identifier-- Rec CCP Response from EPS for Data
#define CN200_ECU1_CAN1_XCP_EA3_REQUEST_ID  canMESSAGE_BOX8  // 0x242   11bit identifier-- Trans XCP Request from EPS for data (not used)
#define CN200_ECU1_CAN1_XCP_EA3_RESPONSE_ID canMESSAGE_BOX9  // 0x642   11bit identifier-- Rec XCP Response to EPS for data (not used)
#define CN200_ECU1_CAN1_XCP_EA4_REQUEST_ID  canMESSAGE_BOX10 // 0x750   11bit identifier-- Trans XCP Request from EPS for data
#define CN200_ECU1_CAN1_XCP_EA4_RESPONSE_ID canMESSAGE_BOX11 // 0x758   11bit identifier-- Rec XCP Response to EPS for data

#define CN200_TESTER_REC_DEBUG_ID           canMESSAGE_BOX12 // 0x7ff   11bit identifier-- REC from Anyone
#define CN200_TESTER_TRANS_DEBUG_ID         canMESSAGE_BOX13 // 0x7fe   11bit identifier-- Transmit to Anyone

#define CN200_ECU1_CAN1_CCP_EA4_AvgVehSpd_ID        canMESSAGE_BOX14 // 0x17D   11bit identifier-- Trans ABS message to EPS
#define CN200_ECU1_CAN1_CCP_EA4_34A_ID      canMESSAGE_BOX15 // 0x34A   11bit identifier-- PPEI_Trans_Vehicle_Speed_and_Distance (Msg ID $3E5) DCT Only
#define CN200_ECU1_CAN1_CCP_EA4_3E5_ID      canMESSAGE_BOX16 // 0x3E5   11bit identifier-- NonDriven Wheel Grnd Velocity HS (Msg ID $34A)

#define CN200_ECU1_CAN1_CCP_EA4_348_ID      canMESSAGE_BOX17 // 0x348   11bit identifier-- NonDriven Wheel Grnd Velocity HS (Msg ID $34A)

// C A N 2  --  C N 2 0 0     N O N    A D A S  Product
#define CN200_ECU1_CAN2_PROD_MESS1_ID       canMESSAGE_BOX1  // 0x148   11bit identifier-- Rec Response from EPS (not used)
#define CN200_ECU1_CAN2_PROD_MESS2_ID       canMESSAGE_BOX2  // 0x778   11bit identifier-- Rec ON Star from EPS (not used)

#define CN200_TESTER_REC_DEBUG_ID           canMESSAGE_BOX12 // 0x7ff   11bit identifier-- REC from Anyone
#define CN200_TESTER_TRANS_DEBUG_ID         canMESSAGE_BOX13 // 0x7fe   11bit identifier-- Transmit to Anyone

/******************************************************************************************************************************/


// C A N  1  --  G 2 K C A    A D A S   Products
#define G2KCA_ECU1_CAN1_PROD_MESS1_ID       canMESSAGE_BOX1  // 0x148   11bit identifier-- Rec Response from EPS (not used)
#define G2KCA_ECU1_CAN1_PROD_MESS2_ID       canMESSAGE_BOX2  // 0x778   11bit identifier-- Rec ON Star from EPS (not used)
#define G2KCA_ECU1_CAN1_CCP_Engine_Run      canMESSAGE_BOX3  // 0x0C9   11bit identifier-- Trans Engine ON to EPS
#define G2KCA_ECU1_CAN1_CCP_Power_Mode_ID   canMESSAGE_BOX4  // 0x1F1   11bit identifier-- Trans Power Mode to EPS
#define G2KCA_ECU1_CAN1_CCP_SPEED_ID        canMESSAGE_BOX5  // 0x3E9   11bit identifier-- Trans Speed to EPS with counter and CRC (not used)
#define G2KCA_ECU1_CAN1_CCP_Driven_ID       canMESSAGE_BOX6  // 0x348   11bit identifier-- Trans Wheel 1 Info to EPS
#define G2KCA_ECU1_CAN1_CCP_NonDriven_ID    canMESSAGE_BOX7  // 0x34A   11bit identifier-- Trans Wheel 2 Info to EPS

#define G2KCA_ECU1_CAN1_CCP_REQUEST_ID      canMESSAGE_BOX8  // 0x708   11bit identifier-- Trans CCP Request for Data to EPS
#define G2KCA_ECU1_CAN1_CCP_RESPONSE_ID     canMESSAGE_BOX9  // 0x706   11bit identifier-- Rec CCP Responce from EPS for Data
#define G2KCA_ECU1_CAN1_XCP_EA3_REQUEST_ID  canMESSAGE_BOX10 // 0x242   11bit identifier-- Trans XCP Request from EPS for data (not used)
#define G2KCA_ECU1_CAN1_XCP_EA3_RESPONSE_ID canMESSAGE_BOX11 // 0x642   11bit identifier-- Rec XCP Responce to EPS for data (not used)
#define G2KCA_ECU1_CAN1_XCP_EA4_REQUEST_ID  canMESSAGE_BOX12 // 0x712   11bit identifier-- Trans XCP Request from EPS for data
#define G2KCA_ECU1_CAN1_XCP_EA4_RESPONSE_ID canMESSAGE_BOX13 // 0x710   11bit identifier-- Rec XCP Responce to EPS for data

#define G2KCA_ECU2_CAN1_CCP_REQUEST_ID      canMESSAGE_BOX14 // 0x70A   11bit identifier-- Trans CCP Request for Data to EPS
#define G2KCA_ECU2_CAN1_CCP_RESPONSE_ID     canMESSAGE_BOX15 // 0x70C   11bit identifier-- Rec CCP Responce from EPS for Data
#define G2KCA_ECU2_CAN1_XCP_EA3_REQUEST_ID  canMESSAGE_BOX16 // 0x242   11bit identifier-- Trans XCP Request from EPS for data  (not used)
#define G2KCA_ECU2_CAN1_XCP_EA3_RESPONSE_ID canMESSAGE_BOX17 // 0x642   11bit identifier-- Rec XCP Responce to EPS for data  (not used)
#define G2KCA_ECU2_CAN1_XCP_EA4_REQUEST_ID  canMESSAGE_BOX18 // 0x713   11bit identifier-- Trans XCP Request from EPS for data
#define G2KCA_ECU2_CAN1_XCP_EA4_RESPONSE_ID canMESSAGE_BOX19 // 0x711   11bit identifier-- Rec XCP Responce to EPS for data

#define G2KCA_TESTER_REC_DEBUG_ID           canMESSAGE_BOX20 // 0x7ff   11bit identifier-- REC from Anyone
#define G2KCA_TESTER_TRANS_DEBUG_ID         canMESSAGE_BOX21 // 0x7fe   11bit identifier-- Transmit to Anyone

// C A N 2  --  G 2 K C A   A D A S   Products
#define G2KCA_ECU2_CAN2_PROD_MESS1_ID       canMESSAGE_BOX1  // 0x148   11bit identifier-- Rec Response from EPS (not used)
#define G2KCA_ECU2_CAN2_PROD_MESS2_ID       canMESSAGE_BOX2  // 0x778   11bit identifier-- Rec ON Star from EPS (not used)
#define G2KCA_ECU2_CAN2_CCP_Engine_Run      canMESSAGE_BOX3  // 0x0C9   11bit identifier-- Trans Engine ON to EPS (not used)
#define G2KCA_ECU2_CAN2_CCP_Power_Mode_ID   canMESSAGE_BOX4  // 0x1F1   11bit identifier-- Trans Power Mode to EPS(not used)
#define G2KCA_ECU2_CAN2_CCP_SPEED_ID        canMESSAGE_BOX5  // 0x3E9   11bit identifier-- Trans Speed to EPS with counter and CRC (not used)
#define G2KCA_ECU2_CAN2_CCP_Driven_ID       canMESSAGE_BOX6  // 0x348   11bit identifier-- Trans Wheel 1 Info to EPS
#define G2KCA_ECU2_CAN2_CCP_NonDriven_ID    canMESSAGE_BOX7  // 0x34A   11bit identifier-- Trans Wheel 2 Info to EPS

#define G2KCA_ECU1_CAN2_CCP_REQUEST_ID      canMESSAGE_BOX8  // 0x708   11bit identifier-- Trans CCP Request for Data to EPS (not used)
#define G2KCA_ECU1_CAN2_CCP_RESPONSE_ID     canMESSAGE_BOX9  // 0x706   11bit identifier-- Rec CCP Responce from EPS for Data (not used)
#define G2KCA_ECU1_CAN2_XCP_EA3_REQUEST_ID  canMESSAGE_BOX10 // 0x242   11bit identifier-- Trans XCP Request from EPS for data (not used)
#define G2KCA_ECU1_CAN2_XCP_EA3_RESPONSE_ID canMESSAGE_BOX11 // 0x642   11bit identifier-- Rec XCP Responce to EPS for data (not used)
#define G2KCA_ECU1_CAN2_XCP_EA4_REQUEST_ID  canMESSAGE_BOX12 // 0x712   11bit identifier-- Trans XCP Request from EPS for data (not used)
#define G2KCA_ECU1_CAN2_XCP_EA4_RESPONSE_ID canMESSAGE_BOX13 // 0x710   11bit identifier-- Rec XCP Responce to EPS for data (not used)

#define G2KCA_ECU2_CAN2_CCP_REQUEST_ID      canMESSAGE_BOX14 // 0x70A   11bit identifier-- Trans CCP Request for Data to EPS (not used)
#define G2KCA_ECU2_CAN2_CCP_RESPONSE_ID     canMESSAGE_BOX15 // 0x70C   11bit identifier-- Rec CCP Responce from EPS for Data (not used)
#define G2KCA_ECU2_CAN2_XCP_EA3_REQUEST_ID  canMESSAGE_BOX16 // 0x242   11bit identifier-- Trans XCP Request from EPS for data (not used)
#define G2KCA_ECU2_CAN2_XCP_EA3_RESPONSE_ID canMESSAGE_BOX17 // 0x642   11bit identifier-- Rec XCP Responce to EPS for data (not used)
#define G2KCA_ECU2_CAN2_XCP_EA4_REQUEST_ID  canMESSAGE_BOX18 // 0x713   11bit identifier-- Trans XCP Request from EPS for data (not used)
#define G2KCA_ECU2_CAN2_XCP_EA4_RESPONSE_ID canMESSAGE_BOX19 // 0x711   11bit identifier-- Rec XCP Responce to EPS for data (not used)

#define G2KCA_TESTER_REC_DEBUG_ID           canMESSAGE_BOX20 // 0x7ff   11bit identifier-- REC from Anyone
#define G2KCA_TESTER_TRANS_DEBUG_ID         canMESSAGE_BOX21 // 0x7fe   11bit identifier-- Transmit to Anyone

/******************************************************************************************************************************/


// C A N  1  --  F O R D   A D A S   Products
#define FORD_PROD_MESS1_ID        canMESSAGE_BOX1    // 0x148   11bit identifier-- Rec Response from EPS (not used)
#define FORD_PROD_MESS2_ID        canMESSAGE_BOX2    // 0x778   11bit identifier-- Rec ON Star from EPS (not used)
#define FORD_CCP_Engine_Run       canMESSAGE_BOX3    // 0x167   11bit identifier-- Trans Engine ON to EPS
#define FORD_CCP_Power_Mode_ID    canMESSAGE_BOX4    // 0x1F1   11bit identifier-- Trans Power Mode to EPS(not used)
#define FORD_CCP_SPEED_ID         canMESSAGE_BOX5    // 0x12A   11bit identifier-- Trans Speed to EPS with counter and CRC

#define FORD_CCP_RESPONSE_ID      canMESSAGE_BOX6    // 0x60B   11bit identifier-- Rec CCP Responce from EPS for Data
#define FORD_CCP_REQUEST_ID       canMESSAGE_BOX7    // 0x60A   11bit identifier-- Trans CCP Request for Data to EPS
#define FORD_XCP_EA3_REQUEST_ID   canMESSAGE_BOX8    // 0x730   11bit identifier-- Trans XCP Request from EPS for data
#define FORD_XCP_EA3_RESPONSE_ID  canMESSAGE_BOX9    // 0x738   11bit identifier-- Rec XCP Responce to EPS for data
#define FORD_XCP_EA4_REQUEST_ID   canMESSAGE_BOX10   // 0x712   11bit identifier-- Trans XCP Request from EPS for data (not used)
#define FORD_XCP_EA4_RESPONSE_ID  canMESSAGE_BOX11   // 0x710   11bit identifier-- Rec XCP Responce to EPS for data (not used)

#define FORD_TESTER_REC_DEBUG_ID  canMESSAGE_BOX12   // 0x7ff   11bit identifier-- REC from Anyone
#define FORD_TESTER_TRANS_DEBUG_ID canMESSAGE_BOX13  // 0x7fe   11bit identifier-- Transmit to Anyone

// C A N 2  --  F O R D   A D A S   Products
#define FORD_PROD_MESS1_ID        canMESSAGE_BOX1    // 0x148   11bit identifier-- Rec Response from EPS (not used)
#define FORD_PROD_MESS2_ID        canMESSAGE_BOX2    // 0x778   11bit identifier-- Rec ON Star from EPS (not used)
#define FORD_CCP_Engine_Run_2     canMESSAGE_BOX3    // 0x60C   11bit identifier-- Trans Engine ON to EPS
#define FORD_CCP_Power_Mode_ID    canMESSAGE_BOX4    // 0x1F1   11bit identifier-- Trans Power Mode to EPS(not used)
#define FORD_CCP_SPEED_ID         canMESSAGE_BOX5    // 0x12A   11bit identifier-- Trans Speed to EPS with counter and CRC

#define FORD_CCP_RESPONSE_ID      canMESSAGE_BOX6    // 0x60B   11bit identifier-- Rec CCP Responce from EPS for Data
#define FORD_CCP_REQUEST_ID       canMESSAGE_BOX7    // 0x60A   11bit identifier-- Trans CCP Request for Data to EPS
#define FORD_XCP_EA3_REQUEST_ID   canMESSAGE_BOX8    // 0x730   11bit identifier-- Trans XCP Request from EPS for data
#define FORD_XCP_EA3_RESPONSE_ID  canMESSAGE_BOX9    // 0x738   11bit identifier-- Rec XCP Responce to EPS for data
#define FORD_XCP_EA4_REQUEST_ID   canMESSAGE_BOX10   // 0x712   11bit identifier-- Trans XCP Request from EPS for data (not used)
#define FORD_XCP_EA4_RESPONSE_ID  canMESSAGE_BOX11   // 0x710   11bit identifier-- Rec XCP Responce to EPS for data (not used)

#define FORD_TESTER_REC_DEBUG_ID  canMESSAGE_BOX12   // 0x7ff   11bit identifier-- REC from Anyone
#define FORD_TESTER_TRANS_DEBUG_ID canMESSAGE_BOX13  // 0x7fe   11bit identifier-- Transmit to Anyone

/******************************************************************************************************************************/


// C A N  1  --  F C A   Products
#define FCA_PROD_MESS1_ID         canMESSAGE_BOX1    // 0x148   11bit identifier-- Rec Response from EPS (not used)
#define FCA_PROD_MESS2_ID         canMESSAGE_BOX2    // 0x778   11bit identifier-- Rec ON Star from EPS (not used)
#define FCA_CCP_Engine_Run        canMESSAGE_BOX3    // 0x108   11bit identifier-- Trans Engine ON to EPS
#define FCA_CCP_Power_Mode_ID     canMESSAGE_BOX4    // 0x1F1   11bit identifier-- Trans Power Mode to EPS(not used)
#define FCA_CCP_SPEED_ID          canMESSAGE_BOX5    // 0x11C   11bit identifier-- Trans Speed to EPS with counter and CRC

#define FCA_CCP_RESPONSE_ID       canMESSAGE_BOX6    // 0x59A   11bit identifier-- Rec CCP Responce from EPS for Data
#define FCA_CCP_REQUEST_ID        canMESSAGE_BOX7    // 0x6DA   11bit identifier-- Trans CCP Request for Data to EPS
#define FCA_XCP_EA3_REQUEST_ID    canMESSAGE_BOX8    // 0x75A   11bit identifier-- Trans XCP Request from EPS for data
#define FCA_XCP_EA3_RESPONSE_ID   canMESSAGE_BOX9    // 0x4DA   11bit identifier-- Rec XCP Responce to EPS for data
#define FCA_XCP_EA4_REQUEST_ID    canMESSAGE_BOX10   // 0x712   11bit identifier-- Trans XCP Request from EPS for data
#define FCA_XCP_EA4_RESPONSE_ID   canMESSAGE_BOX11   // 0x710   11bit identifier-- Rec XCP Responce to EPS for data

#define FCA_TESTER_REC_DEBUG_ID   canMESSAGE_BOX12   // 0x7ff   11bit identifier-- REC from Anyone
#define FCA_TESTER_TRANS_DEBUG_ID canMESSAGE_BOX13   // 0x7fe   11bit identifier-- Transmit to Anyone

// C A N 2  --  F C A   Products
#define FCA_PROD_MESS1_ID         canMESSAGE_BOX1    // 0x148   11bit identifier-- Rec Response from EPS (not used)
#define FCA_PROD_MESS2_ID         canMESSAGE_BOX2    // 0x778   11bit identifier-- Rec ON Star from EPS (not used)
#define FCA_CCP_Engine_Run        canMESSAGE_BOX3    // 0x108   11bit identifier-- Trans Engine ON to EPS
#define FCA_CCP_Power_Mode_ID     canMESSAGE_BOX4    // 0x1F1   11bit identifier-- Trans Power Mode to EPS(not used)
#define FCA_CCP_SPEED_ID          canMESSAGE_BOX5    // 0x11C   11bit identifier-- Trans Speed to EPS with counter and CRC

#define FCA_CCP_RESPONSE_ID       canMESSAGE_BOX6    // 0x59A   11bit identifier-- Rec CCP Responce from EPS for Data
#define FCA_CCP_REQUEST_ID        canMESSAGE_BOX7    // 0x6DA   11bit identifier-- Trans CCP Request for Data to EPS
#define FCA_XCP_EA3_REQUEST_ID    canMESSAGE_BOX8    // 0x75A   11bit identifier-- Trans XCP Request from EPS for data
#define FCA_XCP_EA3_RESPONSE_ID   canMESSAGE_BOX9    // 0x4DA   11bit identifier-- Rec XCP Responce to EPS for data
#define FCA_XCP_EA4_REQUEST_ID    canMESSAGE_BOX10   // 0x712   11bit identifier-- Trans XCP Request from EPS for data
#define FCA_XCP_EA4_RESPONSE_ID   canMESSAGE_BOX11   // 0x710   11bit identifier-- Rec XCP Responce to EPS for data

#define FCA_TESTER_REC_DEBUG_ID   canMESSAGE_BOX12   // 0x7ff   11bit identifier-- REC from Anyone
#define FCA_TESTER_TRANS_DEBUG_ID canMESSAGE_BOX13   // 0x7fe   11bit identifier-- Transmit to Anyone

/******************************************************************************************************************************/

// C A N  1  --  G M   Products
#define PROD_MESS1_ID         canMESSAGE_BOX1    // 0x148   11bit identifier-- Rec Response from EPS (not used)
#define PROD_MESS2_ID         canMESSAGE_BOX2    // 0x778   11bit identifier-- Rec ON Star from EPS (not used)
#define CCP_Engine_Run        canMESSAGE_BOX3    // 0x0C9   11bit identifier-- Trans Engine ON to EPS
#define CCP_Power_Mode_ID     canMESSAGE_BOX4    // 0x1F1   11bit identifier-- Trans Power Mode to EPS
#define CCP_SPEED_ID          canMESSAGE_BOX5    // 0x3E9   11bit identifier-- Trans Speed to EPS

#define CCP_RESPONSE_ID       canMESSAGE_BOX6    // 0x706   11bit identifier-- Rec CCP Responce from EPS for Data
#define CCP_REQUEST_ID        canMESSAGE_BOX7    // 0x708   11bit identifier-- Trans CCP Request for Data to EPS
#define XCP_EA3_REQUEST_ID    canMESSAGE_BOX8    // 0x242   11bit identifier-- Trans XCP Request from EPS for data
#define XCP_EA3_RESPONSE_ID   canMESSAGE_BOX9    // 0x642   11bit identifier-- Rec XCP Responce to EPS for data
#define XCP_EA4_REQUEST_ID    canMESSAGE_BOX10   // 0x712   11bit identifier-- Trans XCP Request from EPS for data
#define XCP_EA4_RESPONSE_ID   canMESSAGE_BOX11   // 0x710   11bit identifier-- Rec XCP Responce to EPS for data

#define TESTER_REC_DEBUG_ID   canMESSAGE_BOX12   // 0x7FF   11bit identifier-- REC from Anyone
#define TESTER_TRANS_DEBUG_ID canMESSAGE_BOX13   // 0x7FE   11bit identifier-- Transmit to Anyone

#define VEHICLE_READY canMESSAGE_BOX14           //0x360    11bit identifier-- Vehicle Ready

// C A N 2  --  G M  Products none are ADAS yet

#define CCP_CAN2_RESPONSE_ID  canMESSAGE_BOX3    // 0x642   11bit identifier-- Rec XCP Responce to EPS for data
#define CCP_CAN2_REQUEST_ID   canMESSAGE_BOX4    // 0x242   11bit identifier-- Trans XCP Request from EPS for data
#define PERIOD_CAN2_ID        canMESSAGE_BOX5    // 0x182   11bit identifier-- Trans periodic to EPS


/******************************************************************************************************************************/

// C A N  1  --  G W M A 0 6 07  Products
#define PROD_MESS1_ID               canMESSAGE_BOX1    // 0x148   11bit identifier-- Rec Response from EPS (not used)
#define PROD_MESS2_ID               canMESSAGE_BOX2    // 0x778   11bit identifier-- Rec ON Star from EPS (not used)
#define CCP_Engine_Run              canMESSAGE_BOX3    // 0x271   11bit identifier-- Trans Engine ON to EPS // ECM2 (MSG ID $271)(CAN)
#define GWM_A0607_CCP_Vehspd_ID     canMESSAGE_BOX4    // 0x265   11bit identifier-- Trans Speed to EPS  // ABS3 (MSG ID $265)(CAN)
#define GWM_A0607_CCP_Endspd_ID     canMESSAGE_BOX5    // 0x348   11bit identifier-- Trans EngSpd to EPS      // ECM1 (MSG ID $111)(CAN)

#define GWM_A0607_CCP_RESPONSE_ID   canMESSAGE_BOX7    // 0x63D   11bit identifier-- Rec CCP Response from EPS for Data
#define GWM_A0607_CCP_REQUEST_ID    canMESSAGE_BOX6    // 0x63C   11bit identifier-- Trans CCP Request for Data to EPS
//#define XCP_EA3_REQUEST_ID        canMESSAGE_BOX8    // 0x242   11bit identifier-- Trans XCP Request from EPS for data
//#define XCP_EA3_RESPONSE_ID       canMESSAGE_BOX9    // 0x642   11bit identifier-- Rec XCP Response to EPS for data
#define XCP_EA4_REQUEST_ID          canMESSAGE_BOX10   // 0x712   11bit identifier-- Trans XCP Request from EPS for data
#define XCP_EA4_RESPONSE_ID         canMESSAGE_BOX11   // 0x710   11bit identifier-- Rec XCP Response to EPS for data
#define TESTER_REC_DEBUG_ID         canMESSAGE_BOX12   // 0x7FF   11bit identifier-- REC from Anyone
#define TESTER_TRANS_DEBUG_ID       canMESSAGE_BOX13   // 0x7FE   11bit identifier-- Transmit to Anyone

#define GWM_A0607_CCP_Wss_ID                canMESSAGE_BOX14    // 0x231   11bit identifier-- Trans Wss to EPS      // ABS1 (MSG ID $231) (CAN)
#define GWM_A0607_CCP_MaxEngTrqNorm_ID      canMESSAGE_BOX15    // 0x371   11bit identifier-- Trans MaxEngTrqNorm to EPS      // ECM3 (MSG ID $371) (CAN)
#define GWM_A0607_CCP_NetEngTrq_ID          canMESSAGE_BOX16    // 0x082   11bit identifier-- Trans NetEngTrq to EPS      // ECM4 (MSG ID $082) (CAN)
#define GWM_A0607_CCP_DiagFuncReq_ID        canMESSAGE_BOX17    // 0x760   11bit identifier-- Trans DiagFuncReq to EPS      // DIAG_FUN_REQ (MSG ID $760)
#define GWM_A0607_CCP_VehYawRate_ID         canMESSAGE_BOX18    // 0x245   11bit identifier-- Trans VehYawRate to EPS      // ABM2 (Msg ID $245) (CAN/CANFD)
#define GWM_A0607_CCP_SteerWheelAng_ID      canMESSAGE_BOX19    // 0x0A1   11bit identifier-- Trans SteerWheelAng to EPS      // CSA2 (Msg ID $0A1)(CAN/CANFD)

// C A N 2  --  G W M A 0 6 07  Products none are ADAS yet

//#define CCP_CAN2_RESPONSE_ID  canMESSAGE_BOX3    // 0x642   11bit identifier-- Rec XCP Response to EPS for data
//#define CCP_CAN2_REQUEST_ID   canMESSAGE_BOX4    // 0x242   11bit identifier-- Trans XCP Request from EPS for data
//#define PERIOD_CAN2_ID        canMESSAGE_BOX5    // 0x182   11bit identifier-- Trans periodic to EPS

/******************************************************************************************************************************/

/******************************************************************************************************************************/

// C A N  1  --  P S A    Products
#define PSA_PROD_MESS1_ID         canMESSAGE_BOX1    // 0x148   11bit identifier-- Rec Response from EPS (not used)
#define PSA_PROD_MESS2_ID         canMESSAGE_BOX2    // 0x778   11bit identifier-- Rec ON Star from EPS (not used)
#define PSA_CCP_Engine_Run        canMESSAGE_BOX3    // 0x348   11bit identifier-- Trans Engine ON to EPS
#define PSA_CCP_Power_Mode_ID     canMESSAGE_BOX4    // 0x1F1   11bit identifier-- Trans Power Mode to EPS (not used)
#define PSA_CCP_SPEED_ID          canMESSAGE_BOX5    // 0x38D   11bit identifier-- Trans Speed to EPS

#define PSA_CCP_REQUEST_ID        canMESSAGE_BOX6    // 0x7AE   11bit identifier-- Trans CCP Request for Data to EPS
#define PSA_CCP_RESPONSE_ID       canMESSAGE_BOX7    // 0x7B0   11bit identifier-- Rec CCP Response from EPS for Data
#define PSA_XCP_EA3_REQUEST_ID    canMESSAGE_BOX8    // 0x6B5   11bit identifier-- Trans XCP Request from EPS for data
#define PSA_XCP_EA3_RESPONSE_ID   canMESSAGE_BOX9    // 0x695   11bit identifier-- Rec XCP Response to EPS for data
#define PSA_XCP_EA4_REQUEST_ID    canMESSAGE_BOX10   // 0x712   11bit identifier-- Trans XCP Request from EPS for data (not used)
#define PSA_XCP_EA4_RESPONSE_ID   canMESSAGE_BOX11   // 0x710   11bit identifier-- Rec XCP Response to EPS for data (not used)

#define TESTER_REC_DEBUG_ID       canMESSAGE_BOX12   // 0x7FF   11bit identifier-- REC from Anyone
#define TESTER_TRANS_DEBUG_ID     canMESSAGE_BOX13   // 0x7FE   11bit identifier-- Transmit to Anyone

// C A N 2  --  P S A   Products none are ADAS yet

#define PSA_CCP_CAN2_RESPONSE_ID  canMESSAGE_BOX3    // 0x642   11bit identifier-- Rec XCP Response to EPS for data (not used)
#define PSA_CCP_CAN2_REQUEST_ID   canMESSAGE_BOX4    // 0x242   11bit identifier-- Trans XCP Request from EPS for data (not used)
#define PSA_PERIOD_CAN2_ID        canMESSAGE_BOX5    // 0x182   11bit identifier-- Trans periodic to EPS (not used)

/******************************************************************************************************************************/


extern int can1_message_type_debug;     // debug, if canMessageNotification switch does not process messageBox store box id
extern int can2_message_type_debug;     // debug, if canMessageNotification switch does not process messageBox store box id

extern int hits_on_CAN2_messageBox1;    // debug, inc each time rec is processed

// C A N   G E N E R A L
extern uint8_t can1_rx_data[D_SIZE];        // storage for CAN rec data[D_SIZE] = {0};
extern uint8_t can2_rx_data[D_SIZE];        // storage for CAN rec data[D_SIZE] = {0};

extern uint8_t can1_tx_data[D_SIZE];        // storage for CAN trans message, incase you need to build one, like variable speed
extern uint8_t can2_tx_data[D_SIZE];        // storage for CAN trans message, incase you need to build one


extern int bus1_warn_state;
extern int bus2_warn_state;
extern int bus1_off_state;
extern int bus2_off_state;
extern int can1_stat_temp;
extern int can2_stat_temp;

extern int can1_request_index;      // indicates which piece of data was asked for with CCP request data
extern int can2_request_index;      // indicates which piece of data was asked for with CCP request data

//extern int DTC_XCP_rply_indx; // used for XCP get DTC responce
//extern int clear_dtcs_flag; // flag used in can interupt routine to indicate clear or get DTC's
//extern int dtc_loop_cnt;
//extern int dtc_inner_loop_cnt;

/* Changed mind on single declare, G2KCA with both ECUs on CAN1 buss causes some timing issues with TOC, so break them out */
extern int ECU1_XCP_reply_index; // used for XCP response for ECU1
extern int ECU1_clear_DTCs_flag; // flag used in can interrupt routine to indicate clear or get DTC's on ECU1
extern int ECU1_dtc_loop_cnt;
extern int ECU1_dtc_inner_loop_cnt;

extern int ECU2_XCP_reply_index; // used for XCP response for ECU2
extern int ECU2_clear_DTCs_flag; // flag used in can interrupt routine to indicate clear or get DTC's on ECU1
extern int ECU2_dtc_loop_cnt;
extern int ECU2_dtc_inner_loop_cnt;

// older stuf for 9BXX
extern unsigned int CAN2_recvd_hi_word; //created for CAN2 stuff
extern unsigned int CAN2_recvd_lo_word; //created for CAN2 stuff
extern unsigned char can2_ccp_response_str[9];


//unsigned char dtc[21][5]; // array for storing 14 to 20 DTC's with 5 bytes dtc#(2 bytes),param,status,ignition counter
extern unsigned char dtc1[21][5]; // ECU1 array for storing 14 to 20 DTC's with 5 bytes dtc#(2 bytes),param,status,ignition counter
extern unsigned char dtc2[21][5]; // ECU2 array for storing 14 to 20 DTC's with 5 bytes dtc#(2 bytes),param,status,ignition counter

extern uint8_t can1_dump_err_data[40][9];   // storage for CAN responce messages - skip [0], start at [1] easier to match, must store responce and process in MAIN, stack error if sprintf called from ISR
extern uint8_t can2_dump_err_data[40][9];   // storage for CAN responce messages - skip [0], start at [1] easier to match, must store responce and process in MAIN, stack error if sprintf called from ISR

//  E X T E R N A L   C a l   V A R S

extern int dumperr_Tester1_vign;    /* Vign voltage=volt drop(in mV)*6(divider ratio) =converted to mV(Ex: 9.0V = 9000) */
extern int dumperr_Product1_5volt;
extern int dumperr_Product1_3volt;

extern int dumperr_Tester2_vign;    /* Vign voltage=volt drop(in mV)*6(divider ratio) =converted to mV(Ex: 9.0V = 9000) */
extern int dumperr_Product2_5volt;
extern int dumperr_Product2_3volt;

extern const char N_A_str[4];      // used for filling dump_err and fuctional test strings 12NOV12 L. Cronk
extern int N_A_num;


//  E X T E R N A L   A to D  F I L T E R ,   R E F E R A N C E   and   S T O R A G E
extern uint16 adc1_Group1_history[10][12];      // user storage, rolling history of adc fifo, set up for 12ch/10 sets of samples
extern adcData_t adc1_Group1_raw[12];               // user storage (structured data), to offload adc fifo, set up for 12ch/1 sets of samples
extern uint16 adc1_Group1_sum[12];              // user storage, accumilation of history set
extern uint16 adc1_Group1_filtered[12];         // user storage, FIR filtered data, Use this set for evaluation
extern int adc1_Group1_mV[12];              // user storage, if ad_2_5_vref_flag is TRUE, value is AtoD value in mV, else -1
extern uint32 acd1_count;
//int j;    // used in adcNotification, index into which row were storing data in now
extern int oldest;                              // pointer to oldest entry in circular buffer

extern adcData_t adc2_Group1_raw[9];                // user storage (structured data), to offload adc fifo, set up for 9ch/1 sets of samples
extern uint32 acd2_count;


extern int ad_2_5_vref_flag;                // set when 2.5 volt ref has been established, after > two passes of FIR filter
extern int ad_2_5_vref_val;             // AtoD counts for filtered 2.5volt ref, established pre main(), used in converting channel values to mV
extern int ad_ground_offset_in_tick;        // when using B_Board and 96way 5volt power, offset will compensate for readbacks loss because of board interconnect

extern int AtoD_print_type; /* 0= done, 1= single ch print, 2= all 11chs print */
extern int AtoD_print_count;    /* number of times to print, will print each analog scan until decrimented to 0 */

// E X T R N A L   S P I   V A R S

extern unsigned short DtoA_array[8][3];     /* Intermediate array to store SPI data for chip configuration  */
/* Load or edit prior to kicking off SPI activity.  */
/*  byte0=command&address byte1=dataHigh byte2=dataLow  */
/* 0xffff = 5v, 0x3333/volt = 13,107dec/volt */


// E X T R N A L   H E T  and  S E N T    V A R S

extern int sent_torque_ChA_update_flag; // ACTIVE/REQUEST/IDLE From MAIN ACTIVE while calulating edge time, set to REQUEST to signal IRQ data is ready for load HET RAM, IRQ set to IDLE when complete
extern int sent_torque_ChB_update_flag; // ACTIVE/REQUEST/IDLE From MAIN ACTIVE while calulating edge time, set to REQUEST to signal IRQ data is ready for load HET RAM, IRQ set to IDLE when complete
extern int sent_ChA_handwheel_update_flag;  // ACTIVE/REQUEST/IDLE From MAIN ACTIVE while calulating edge time, set to REQUEST to signal IRQ data is ready for load HET RAM, IRQ set to IDLE when complete
extern int sent_ChB_handwheel_update_flag;  // ACTIVE/REQUEST/IDLE From MAIN ACTIVE while calulating edge time, set to REQUEST to signal IRQ data is ready for load HET RAM, IRQ set to IDLE when complete

extern short sent_data_array[8][5];         // formatted data for -- Sent0 data0,data1,data2,CRC and Sent1 data0,data1,data2,CRC
extern short sent_edge_event_time[8][17];   // contains the times based on SENT clock tick for each edge in SENT message packet,, for 2 channels
extern short het_instruction_data[8][17];       // sent_edge_event_time[][] processed to match het data register format, so quick write in ISR

extern int sent_ChA_toggle_HW_address_counter;      // Rolling counter for alternating address nibble in SENT1/SENT2 and in SENT3/4 from #0 to #1 to #0 to #1 ect on each trigger.
extern int sent_ChB_toggle_HW_address_counter;      //  Plan to ensure time syncronization is to use two counters (one for each ADAS processor to drive address changes).
//  Bump ChA counter on SENT 1/2 IRQ after SENT2 address 1 data has been updated.
//  Bump ChB counter on SENT 3/4 IRQ after SENT4 address 1 data has been updated.
//  A bump will then cause the odd/even MOD to select the oposite set of data to upload. (address 0 or 1)

// E X T R N A L   B U L G A R I A    B O X   V A R S

extern int bulgaria_led_temp;
extern int tester_state;            // state of test INIT, RUN, PAUSE, STOP ect.
extern unsigned short test, test1, test2, test3, test4;

//E X T E R N A L    R S 4 8 5    V A R S

extern char return_message[750];    /* transmit to host after reception buffer */
extern char command_str[255];       /* store rec message  */
extern char words[9][25];           /* row= delimited entity, colm= all chars within entity */
extern unsigned short mp;           /* message pointer */
extern int who_is_addressed;        /* 0=buffer, 1=board_id[] is my ID (me),3= all or everybody 'E' or 'F' */

extern int loop_cnt_a;              /* simple loop var, used in main */
extern long loop_cnt_b;             /* simple loop var, used in main */
extern int loop_cnt_c;              /* simple loop var, used in externs */
extern int case_select;             /* case var used in eval_command */
extern int sscan_cnt;               /* bug in sscanf, return only works with one parameter */

extern char CR = 0xD;               /* carriage return ASCII value */

//extern char board_id[3];          /* board id calculated in main() */
extern int data_frame;              /* flag to indicate if address or data mode */
extern int flag_to_process;     /* complete string received, parse it */

extern uint8 sbuf_tx_main[1024]; //  used for printing to sciREG1, our debug com port

extern uint8 sbuf_tx[1024];         /* transmit buffer */
extern uint8 sbuf_rx[1024];         /* receive buffer  */
#define TX_EMPTY 0x800  /* Flag indicating sci shift register is empty  */
/* used with sci->FLR register                  */
/* bit 11 of FLAG register                      */

/* Reset program vars - Global*/
extern char reset_key_reg[8];       // may be used in future for user reset or flashing of CIB over comm port

extern int tx_to_rx_delay_flag;

// E X T E R N A L   C O N S T A N T S

/* --------------   T 1 X X   CAN  ------------------------  */
// CAN data for each of the CCP requests, do memcpy to CAN message obj &data
// hex Address * 2 example 0x8951 * 2 = 112A2  so CAN data byte 4 = 0xa2 byte 3 = 0x12 byte 2 = 0x01 byte 1 = 0x00
// Looks like {0x0f,0x01,0x02,0x00,0xa2,0x12,0x01,0x00}
// CAN data for each of the CCP requests, do memcpy to CAN message obj &data

extern const unsigned char ccp_request_connect[];

//--------------------------------------------------------
extern const unsigned char ccp_write_PowerModeON[];
extern const unsigned char ccp_write_PowerModeOFF[];

extern const unsigned char Vehicle_ready[];
extern const unsigned char Vehicle_not_ready[];

extern const unsigned char ccp_write_engine_run[];
extern const unsigned char ccp_write_engine_off[];      // keeps product in warm Init mode
extern const unsigned char ccp_write_crank_active[];    // This might be required for Crank Pulse testing/Crank Mode

//------------------------------------------------------------
extern const unsigned char ccp_write_periodic_can2[];   // CAN2 periodic message

//------------------------------------------------------------


extern const unsigned char ccp_write_0_to_speed[]; // engine_on & speed 0kph 8 bytes

extern const unsigned char ccp_write_wheel_speed[]; // new for GM ADAS, write to both ECUs, instead of speed from Transmission


extern const unsigned char xcp_write_session_Nexteer_mode[]; // open Nexteer session for manufacturing mode id730
// this must precede request for DTC's. Session times out approx after 5 seconds.
// id738 should get reply : 01 50 AA AA AA AA AA AA

/* --------------   G W M A 0 6 0 7   CAN  ------------------------ */

//    extern const uint8 Veh_speed_00_table_GWM_A0607[16][8];

// Set Vehspd = 0 //ABS3 (MSG ID $265)(CAN)
    extern unsigned char ccp_GWM_A0607_write_Vehspd[];       /* engine_on & speed 0kph 8 bytes*/   
    extern unsigned char GWM_A0607_Vehspd_CRC_counter;
    extern unsigned char GWM_A0607_Vehspd_CRC_ChKSum;

// Set EngState = 0x02 //ECM2 (MSG ID $271)(CAN)
    extern unsigned char ccp_GWM_A0607_write_EngState[];       /* engine_on 8 bytes*/   
    extern unsigned char GWM_A0607_EngState_CRC_counter;
    extern unsigned char GWM_A0607_EngState_CRC_ChKSum;

// Set EngSpd = 0 // ECM1 (MSG ID $111)(CAN)
    extern const unsigned char ccp_GWM_A0607_write_EngSpd[]; /* EngSpd 8 bytes & EngSpdVldty(bit63) 0x1:Valid*/   

// Set Wss = 0 // ABS1 (MSG ID $231) (CAN)
    extern unsigned char ccp_GWM_A0607_write_Wss[];       /* Wss 8 bytes*/   
    extern unsigned char GWM_A0607_Wss_CRC_counter;
    extern unsigned char GWM_A0607_Wss_CRC_ChKSum;

// Set VehYawRate = 0  // ABM2 (Msg ID $245) (CAN/CANFD)
    extern unsigned char ccp_GWM_A0607_write_VehYawRate[];       /* VehYawRate 8 bytes*/   
    extern unsigned char GWM_A0607_VehYawRate_CRC_counter;
    extern unsigned char GWM_A0607_VehYawRate_CRC_ChKSum;

// Set SteerWheelAng =  // CSA2 (Msg ID $0A1)(CAN/CANFD)
    extern unsigned char ccp_GWM_A0607_write_SteerWheelAng[];       /* SteerWheelAng 8 bytes*/   
    extern unsigned char GWM_A0607_SteerWheelAng_CRC_counter;
    extern unsigned char GWM_A0607_SteerWheelAng_CRC_ChKSum;

// Set MaxEngTrqNorm = 0 // ECM3 (MSG ID $371) (CAN)
    extern unsigned char ccp_GWM_A0607_write_MaxEngTrqNorm[];       /* MaxEngTrqNorm 8 bytes*/   
    extern unsigned char GWM_A0607_MaxEngTrqNorm_CRC_counter;
    extern unsigned char GWM_A0607_MaxEngTrqNorm_CRC_ChKSum;

// Set NetEngTrq = 0 // ECM4 (MSG ID $082) (CAN)
    extern const unsigned char ccp_GWM_A0607_write_NetEngTrq[]; /* NetEngTrq 8 bytes & NetEngTrq = 0 */   



// ======  Manufacturing services ================

extern const unsigned char xcp_EA4_write_get_DTC_rqst1[]   ;        // EA4 format
extern const unsigned char xcp_EA4_write_get_DTC_rqst2[]   ;        // no change from EA3
extern const unsigned char xcp_EA4_write_clear_DTC_rqst1[] ;        // EA4 format

extern const unsigned char xcp_EA3_write_get_DTC_rqst1[]    ;
extern const unsigned char xcp_EA3_write_get_DTC_rqst2[]    ;
extern const unsigned char xcp_EA3_write_clear_DTC_rqst1[]  ;

extern const unsigned char xcp_write_tester_present_msg[]; // tester present message to keep manufacturing services from timing out and stopping.



/*    CAN data requests for RENAULT_NISSAN NON-ADAS    START  ------  N O T E -- L I T T L E   E I N D I A N   in  constant string   -------------------------*/

extern unsigned char RENAULT_NISSAN_CRC_counter;
extern unsigned char RENAULT_NISSAN_CRC_ChkSum;
extern unsigned char RENAULT_NISSAN_write_0_to_speed[]                      ; /* so far T3 does not use rolling speed, via array */

extern const unsigned char ccp_RENAULT_NISSAN_engine_run[]                  ;     // sets system into OPERATE
extern const unsigned char ccp_RENAULT_NISSAN_engine_off[]                  ;     // keeps product in warm Init mode and required to shut off controller
extern  const unsigned char ccp_RENAULT_NISSAN_GenericAppDiagEnble[]         ;  // All fault reports and alerts are enabled

extern const unsigned char ccp_RENAULT_NISSAN_BattVltg_str[]                ;    // 1   0xFEBE4138  0xFE,0xBE,0x41,0xFE     Battery Voltage
extern const unsigned char ccp_RENAULT_NISSAN_Temperature_str[]             ;    // 2   0xFEBE4160  0xFE,0xBE,0x41,0x60     Ecu Temperature Filtered
extern const unsigned char ccp_RENAULT_NISSAN_DigT1_str[]                   ;    // 3   0xFEBE416C  0xFE,0xBE,0x41,0x6C     HwTq 0
extern const unsigned char ccp_RENAULT_NISSAN_DigT2_str[]                   ;    // 4   0xFEBE4170  0xFE,0xBE,0x41,0x70     HwTq 1
extern const unsigned char ccp_RENAULT_NISSAN_MtrCurrQax_str[]              ;    // 5   0xFEBE4190  0xFE,0xBE,0x41,0x90     Motor Current Measurement
extern const unsigned char ccp_RENAULT_NISSAN_SystemState_byt[]             ;    // 6   0xFEBE439C  0xFE,0xBE,0x43,0x9C     System State
extern const unsigned char ccp_RENAULT_NISSAN_HwTrq_HwNm_str[]              ;    // 7   0xFEBE4218  0xFE,0xBE,0x42,0x18     Handwheel Tq total
extern const unsigned char ccp_RENAULT_NISSAN_Abs_Hw_Pos_str[]              ;    // 8   0xFEBE41F8  0xFE,0xBE,0x41,0xF8     Handwheel Angle
extern const unsigned char ccp_RENAULT_NISSAN_MotTrq_Crf_str[]              ;    // 9   0xFEBE39E4  0xFE,0xBE,0x42,0x6C     Motor Torque Crf
extern const unsigned char ccp_RENAULT_NISSAN_MotTrq_Mrf_str[]              ;    // 10  0xFEBE39E8  0xFE,0xBE,0x42 0x70     Motor Torque Mrf
extern const unsigned char ccp_RENAULT_NISSAN_VehSpd_str[]                  ;    // 11  0xFEBE39E8  0xFE,0xBE,0x42 0xEC     Vehicle Speed
extern const unsigned char ccp_RENAULT_NISSAN_Spare2_str[]                  ;    // 12  0xFEBE39E8  0xFE,0xBE,0x42 0xEC     Spair2


/*    CAN data requests for RENAULT_NISSAN  NON-ADAS    END  -----------------------------------------------------------------*/


/*    CAN data requests for FORD T3 - T6     START  ------  N O T E -- L I T T L E   E I N D I A N   in  constant string   -------------------------*/

extern const unsigned char ccp_FORD_T3_engine_run[]   ;
extern const unsigned char ccp_FORD_T3_engine_off[]   ; // keeps product in warm Init mode
extern const unsigned char ccp_FORD_T3_write_0_to_speed[] ; /* so far T3 does not use rolling speed, via array */


extern const unsigned char ccp_FORD_T3_ECU1_BattVltg_str[]               ;    // 1    0xFE,0xBE,0xA4,0xF8     Battery Voltage
extern const unsigned char ccp_FORD_T3_ECU1_Temperature_str[]            ;    // 2    0xFE,0xBE,0xA6,0x14     Ecu Temperature Filtered
extern const unsigned char ccp_FORD_T3_ECU1_DigT1_str[]                  ;    // 3    0xFE,0xBE,0xA6,0xD4     Handwheel channel a trq
extern const unsigned char ccp_FORD_T3_ECU1_DigT2_str[]                  ;    // 4    0xFE,0xBE,0xA6,0xD8     Handwheel channel b trq
extern const unsigned char ccp_FORD_T3_ECU1_MtrCurrQax_str[]             ;    // 5    0xFE,0xBE,0xA5,0x94     Motor Current Measurement
extern const unsigned char ccp_FORD_T3_ECU1_SystemState_byt[]            ;    // 6    0xFE,0xBE,0xA9,0xAA     System State
extern const unsigned char ccp_FORD_T3_ECU1_HwTrq_HwNm_str[]             ;    // 7    0xFE,0xBE,0xA6,0xD0     Handwheel Tq total
extern const unsigned char ccp_FORD_T3_ECU1_Abs_Hw_Pos_str[]             ;    // 8    0xFE,0xBE,0xA6,0xA4     Handwheel Angle ABS

extern const unsigned char ccp_FORD_T3_ECU2_BattVltg_str[]               ;    // 1    0xFE,0xBE,0xEB,0x40     Battery Voltage
extern const unsigned char ccp_FORD_T3_ECU2_Temperature_str[]            ;    // 2    0xFE,0xBE,0xEC,0x58     Ecu Temperature Filtered
extern const unsigned char ccp_FORD_T3_ECU2_DigT1_str[]                  ;    // 3    0xFE,0xBE,0xEB,0x68     Handwheel channel a trq
extern const unsigned char ccp_FORD_T3_ECU2_DigT2_str[]                  ;    // 4    0xFE,0xBE,0xEB,0x74     Handwheel channel b trq
extern const unsigned char ccp_FORD_T3_ECU2_MtrCurrQax_str[]             ;    // 5    0xFE,0xBE,0xEB,0xD8     Motor Current Measurement
extern const unsigned char ccp_FORD_T3_ECU2_SystemState_byt[]            ;    // 6    0xFE,0xBE,0xEF,0xCE     System State
extern const unsigned char ccp_FORD_T3_ECU2_HwTrq_HwNm_str[]             ;    // 7    0xFE,0xBE,0xEC,0xE8     Handwheel Tq total
extern const unsigned char ccp_FORD_T3_ECU2_Abs_Hw_Pos_str[]             ;    // 8    0xFE,0xBE,0xEC,0xAC     Handwheel Angle ABS


/*    CAN data requests for FORD T3 - T6    END  -----------------------------------------------------------------*/


/*    CAN data requests for CN200 NON-ADAS    START  ------  N O T E -- L I T T L E   E I N D I A N   in  constant string   -------------------------*/

extern const unsigned char ccp_CN200_BattVltg_str[]      ;
extern const unsigned char ccp_CN200_Temperature_str[]   ;
extern const unsigned char ccp_CN200_DigT1_str[]         ;
extern const unsigned char ccp_CN200_DigT2_str[]         ;
extern const unsigned char ccp_CN200_MtrCurrQax_str[]    ;
extern const unsigned char ccp_CN200_SystemState_byt[]   ;
extern const unsigned char ccp_CN200_HwTrq_HwNm_str[]    ;
extern const unsigned char ccp_CN200_Abs_Hw_Pos_str[]    ;
extern const unsigned char ccp_CN200_MotTrq_Crf_str[]       ;
extern const unsigned char ccp_CN200_MotTrq_Mrf_str[]       ;
extern const unsigned char ccp_CN200_VehSpd_str[]        ;
extern const unsigned char ccp_CN200_Spare2_str[]        ;


/*    CAN data requests for CN200  NON-ADAS    END  -----------------------------------------------------------------*/


/*    CAN data requests for PSA_CMP    START  ------  N O T E -- B I G    E I N D I A N   in  constant string    -------------------------*/

extern const unsigned char ccp_PSA_CMP_write_engine_locked[]   ;
extern const unsigned char ccp_PSA_CMP_write_engine_cut[]      ;
extern const unsigned char ccp_PSA_CMP_write_engine_start[]    ;
extern const unsigned char ccp_PSA_CMP_write_engine_run[]      ;
extern const unsigned char ccp_PSA_CMP_write_engine_stop[]     ;

extern const unsigned char ccp_PSA_CMP_BattVltg_str[]          ;
extern const unsigned char ccp_PSA_CMP_Temperature_str[]       ;
extern const unsigned char ccp_PSA_CMP_DigT1_str[]             ;
extern const unsigned char ccp_PSA_CMP_DigT2_str[]             ;
extern const unsigned char ccp_PSA_CMP_SystemState_byt[]       ;
extern const unsigned char ccp_PSA_CMP_MtrCurrQax_str[]        ;
extern const unsigned char ccp_PSA_CMP_MtrCurrDax_str[]        ;
extern const unsigned char ccp_PSA_CMP_HwTrq_HwNm_str[]        ;
extern const unsigned char ccp_PSA_CMP_MtrTrqCmd_HwNm_str[]    ;
extern const unsigned char ccp_PSA_CMP_AstMtrTrq_HwNm_str[]    ;
extern const unsigned char ccp_PSA_CMP_Abs_Hw_Pos_str[]        ;
extern const unsigned char ccp_PSA_CMP_Rel_Hw_Pos_str[]        ;
extern const unsigned char ccp_PSA_CMP_Abs_Hw_Pos_Valid_byt[]  ;
extern const unsigned char ccp_PSA_CMP_MtrPos_str[]            ;
extern const unsigned char ccp_PSA_CMP_ADC_MtrCurr1_str[]      ;
extern const unsigned char ccp_PSA_CMP_ADC_MtrCurr2_str[]      ;

/*    CAN data requests for PSA_CMP    END  -----------------------------------------------------------------*/



/*    CAN data requests for G2KCA ADAS    START  ------  N O T E -- L I T T L E   E I N D I A N   in  constant string   -------------------------*/

extern const unsigned char ccp_G2KCA_BattVltg_str[]         ;
extern const unsigned char ccp_G2KCA_Temperature_str[]      ;
extern const unsigned char ccp_G2KCA_DigT1_str[]            ;
extern const unsigned char ccp_G2KCA_DigT2_str[]            ;
extern const unsigned char ccp_G2KCA_MtrCurrQax_str[]       ;
extern const unsigned char ccp_G2KCA_SystemState_byt[]      ;
extern const unsigned char ccp_G2KCA_HwTrq_HwNm_str[]       ;
extern const unsigned char ccp_G2KCA_Abs_Hw_Pos_str[]       ;


/*    CAN data requests for G2KCA ADAS    END  -----------------------------------------------------------------*/


/*    CAN data requests for FORD ADAS    START  ------   -------------------------*/
extern const unsigned char ccp_FORD_engine_run[]    ;
extern const unsigned char ccp_FORD_engine_run_2[]  ;
extern const unsigned char ccp_FORD_engine_off[]    ; // keeps product in warm Init mode
extern const unsigned char ccp_FORD_write_0_to_speed[] ; /* HW valid = 1 & HW angle(0deg)= 3E80 & speed valid=1 & speed=0kph, 8 bytes*/

extern const unsigned char ccp_FORD_BattVltg_str[]          ;
extern const unsigned char ccp_FORD_Temperature_str[]       ;
extern const unsigned char ccp_FORD_DigT1_str[]             ;
extern const unsigned char ccp_FORD_DigT2_str[]             ;
extern const unsigned char ccp_FORD_MtrCurrQax_str[]        ;
extern const unsigned char ccp_FORD_SystemState_byt[]       ;
extern const unsigned char ccp_FORD_HwTrq_HwNm_str[]        ;
extern const unsigned char ccp_FORD_Abs_Hw_Pos_str[]        ;
extern const unsigned char ccp_FORD_Batt_Current_str[]      ;
extern const unsigned char ccp_FORD_LatchFail_ISR_u16[] ;
extern const unsigned char ccp_FORD_MtrCntl_ISR_u16[];
extern const unsigned char ccp_FORD_MinCount_u32[]  ;
extern const unsigned char ccp_FORD_MaxCount_u32[];
extern const unsigned char ccp_FORD_CatGate_cnt[]   ;

/*    CAN data requests for FORD ADAS    END  -----------------------------------------------------------------*/


/*    CAN data requests for FCA    START  ------   -------------------------*/

extern const unsigned char ccp_FCA_BattVltg_str[]                       ;
extern const unsigned char ccp_FCA_Temperature_str[]                    ;
extern const unsigned char ccp_FCA_DigT1_str[]                          ;
extern const unsigned char ccp_FCA_DigT2_str[]                          ;
extern const unsigned char ccp_FCA_MtrCurrQax_str[]                     ;
extern const unsigned char ccp_FCA_SystemState_byt[]                    ;
extern const unsigned char ccp_FCA_HwTrq_HwNm_str[]                     ;
extern const unsigned char ccp_FCA_Abs_Hw_Pos_str[]                     ;


/*    CAN data requests for C1XX    START  ------   -------------------------*/

extern const unsigned char ccp_C1XX_BattVltg_str[]                      ;
extern const unsigned char ccp_C1XX_Temperature_str[]                   ;
extern const unsigned char ccp_C1XX_DigT1_str[]                         ;
extern const unsigned char ccp_C1XX_DigT2_str[]                         ;
extern const unsigned char ccp_C1XX_MtrCurrQax_str[]                    ;
extern const unsigned char ccp_C1XX_OnStateFltAcc_cnt[]                 ;
extern const unsigned char ccp_C1XX_GateDriveFltAcc_cnt[]               ;
extern const unsigned char ccp_C1XX_GateDrvFltSts_cnt[]                 ;
extern const unsigned char ccp_C1XX_SystemState_byt[]                   ;
extern const unsigned char ccp_C1XX_HwTrq_HwNm_str[]                    ;
extern const unsigned char ccp_C1XX_Abs_Hw_Pos_str[]                    ;
extern const unsigned char ccp_C1XX_Rel_Hw_Pos_str[]                    ;
extern const unsigned char ccp_C1XX_ManualTrqCmdEn_cnt[]                ;
extern const unsigned char ccp_C1XX_MtrTrq_CmdNm_str[]                  ;
extern const unsigned char ccp_C1XX_CCD_MSB_Die1_cnt[]                  ;
extern const unsigned char ccp_C1XX_CCD_MSB_Die2_cnt[]                  ;
extern const unsigned char ccp_C1XX_MtrTempEst_MagTempEst_DegC_str[]    ;



/*    CAN data requests for T1XX    START  ----------------------------------------------------------------- */

extern const unsigned char ccp_T1XX_SystemState_byt[]      ;
extern const unsigned char ccp_T1XX_MotCurrQax_str[]       ;
extern const unsigned char ccp_T1XX_HwTq0Meas_HwTq0_str[]  ;
extern const unsigned char ccp_T1XX_HwTq1Meas_HwTq1_str[]  ;
extern const unsigned char ccp_T1XX_HwTq2Meas_HwTq2_str[]  ;
extern const unsigned char ccp_T1XX_HwTq3Meas_HwTq3_str[]  ;
extern const unsigned char ccp_T1XX_EcuTFild_str[]         ;
extern const unsigned char ccp_T1XX_HwAgArbn_HwAg_str[]    ;
extern const unsigned char ccp_T1XX_HwAg0_str[]            ;
extern const unsigned char ccp_T1XX_HwAg1_str[]            ;
extern const unsigned char ccp_T1XX_HwTqArbn_HwTq_str[]    ;
extern const unsigned char ccp_T1XX_MotCurrDax_str[]       ;
extern const unsigned char ccp_T1XX_HwTqArbn_HwTqChA_str[] ;
extern const unsigned char ccp_T1XX_HwTqArbn_HwTqChB_str[] ;
extern const unsigned char ccp_T1XX_MotAgCmp_str[]         ;
extern const unsigned char ccp_T1XX_BattVltg_str[]         ;
extern const unsigned char ccp_T1XX_BattVltgSwd1_str[]     ;
extern const unsigned char ccp_T1XX_BattVltgSwd2_str[]     ;
extern const unsigned char ccp_T1XX_BrdgVltg_str[]         ;
extern const unsigned char ccp_T1XX_AssiCmdBas_str[]       ;
extern const unsigned char ccp_T1XX_MotTqCmd_str[]         ;
extern const unsigned char ccp_T1XX_AssiMechT_str[]        ;
extern const unsigned char ccp_T1XX_MotFetT_str[]          ;
extern const unsigned char ccp_T1XX_MotMagT_str[]          ;
extern const unsigned char ccp_T1XX_MotWidgT_str[]         ;
extern const unsigned char ccp_T1XX_MotREstim_str[]        ;
extern const unsigned char ccp_T1XX_MotTq_str[]            ;


/*    CAN data requests for T1XX    END  -----------------------------------------------------------------*/

/*    CAN data requests for GWM_A0607    START  ----------------------------------------------------------------- */

extern const unsigned char ccp_GWM_A0607_SystemState_byt[]     ;
extern const unsigned char ccp_GWM_A0607_MotCurrQax_fp[]       ;
extern const unsigned char ccp_GWM_A0607_HwTq0Meas_HwTq4_fp[]  ;
extern const unsigned char ccp_GWM_A0607_HwTq1Meas_HwTq5_fp[]  ;
extern const unsigned char ccp_GWM_A0607_MotHwPosn_fp[]        ;
extern const unsigned char ccp_GWM_A0607_BattVltg_fp[]         ;
extern const unsigned char ccp_GWM_A0607_MotTq_fp[]            ;
extern const unsigned char ccp_GWM_A0607_HwTrq_fp[]            ;
extern const unsigned char ccp_GWM_A0607_MotMagTestim_fp[]     ;
extern const unsigned char ccp_GWM_A0607_ECUTFilt_fp[]         ;

/*    CAN data requests for GWM_A0607    END  -----------------------------------------------------------------*/

/* --------------   9 B x x     CAN  ------------------------
// CAN data for each of the CCP requests, do memcpy to CAN message obj &data
// hex Address * 2 example 0x8951 * 2 = 112A2  so CAN data byte 4 = 0xa2 byte 3 = 0x12 byte 2 = 0x01 byte 1 = 0x00
// Looks like {0x0f,0x01,0x02,0x00,0xa2,0x12,0x01,0x00}
// CAN data for each of the CCP requests, do memcpy to CAN message obj &data

extern const unsigned char ccp_request_connect[];


    //--------------------------------------------------------

extern const unsigned char ccp_write_PowerModeON[];
extern const unsigned char ccp_write_PowerModeOFF[];
extern const unsigned char ccp_write_PowerModeCRANK[]; //Added by P. Horny 28Aug14


extern const unsigned char ccp_write_engine_run[];
extern const unsigned char ccp_write_engine_off[]   ; // keeps product in warm Init mode
extern const unsigned char ccp_write_crank_active[] ; // This might be required for Crank Pulse testing/Crank Mode

    //------------------------------------------------------------
extern const unsigned char ccp_write_periodic_can2[] ; // CAN2 periodic message

    //------------------------------------------------------------

extern const unsigned char ccp_write_0_to_speed[]; // engine_on & speed 0kph 8 bytes
//const unsigned char ccp_write_1_to_speed[] = {0x00,0x64,0x00,0x00,0x00,0x00,0x00,0x00}; engine_on & 1kph marker for all 1's- no comm
//const unsigned char ccp_write_2_to_speed[] = {0x00,0xC8,0x00,0x00,0x00,0x00,0x00,0x00}; engine_on & 2kph marker for all 0's- dump w/ign OFF
//const unsigned char ccp_write_3_to_speed[] = {0x01,0x2C,0x00,0x00,0x00,0x00,0x00,0x00};  warm_init & 3kph marker for start of OffStateVoltage
//const unsigned char ccp_write_4_to_speed[] = {0x01,0x90,0x00,0x00,0x00,0x00,0x00,0x00};  warm_init & 4kph marker for end of OffStateVoltage


//extern const unsigned char ccp_write_session_Nexteer_mode[]; // open Nexteer session for manufacturing mode id242
                                                        // this must precede request for DTC's. Session times out approx after 5 seconds.     id642
                                                        // should get reply : 01 50 AA AA AA AA AA AA

// ======  Manufacturing services ================

extern const unsigned char xcp_write_session_Nexteer_mode[]; // open Nexteer session for manufacturing mode id242
                                                        // this must precede request for DTC's. Session times out approx after 5 seconds.     id642
                                                        // should get reply : 01 50 AA AA AA AA AA AA
extern const unsigned char xcp_write_get_DTC_rqst1[]    ;
extern const unsigned char xcp_write_get_DTC_rqst2[]    ;
extern const unsigned char xcp_write_clear_DTC_rqst1[]  ;

extern const unsigned char xcp_write_tester_present_msg[] ; // tester present message to keep manufacturing services from timing out and stopping.

//    CAN data requests for 9Bxx    START  ------  N O T E -- B I G   E I N D I A N   in  constant string  -------------------------

extern const unsigned char ccp_request_system_state[]               ;   // 1       one byte ENUM
extern const unsigned char ccp_request_RelHwPos_str[]               ;   // 2    P1A 4 byte float
extern const unsigned char ccp_request_AbsHwPos_str[]               ;   // 3    P2A 4 byte float
extern const unsigned char ccp_request_ColPos_str[]                 ;   // 4    P1B 4 byte float
extern const unsigned char ccp_request_AnaHwTrq1_str[]              ;   // 5    TP2B 4 byte float
extern const unsigned char ccp_request_AnaHwTrq2_str[]              ;   // 6    Handwheel / Column position in deg 4 byte float
extern const unsigned char ccp_request_Ch1_T1_T2_str[]              ;   // 7    T1A 4 byte float
extern const unsigned char ccp_request_DigHwTrq3_str[]              ;   // 8    T1A 4 byte float
extern const unsigned char ccp_request_DigHwTrq4_str[]              ;   // 9    T2B 4 byte float
extern const unsigned char ccp_request_Ch2_T3_T4_str[]              ;   // 10    T2B 4 byte float
extern const unsigned char ccp_request_HwTrq_Final_str[]            ;   // 11   HwTrqNm  4 byte float
extern const unsigned char ccp_request_battery_voltage_str[]        ;   // 12   4 byte float
extern const unsigned char ccp_request_battery_current_str[]        ;   // 13   4 byte float
extern const unsigned char ccp_request_controller_temp_str[]        ;   // 14   4 byte float
extern const unsigned char ccp_request_MtrCurrQax_str[]             ;   // 15   4 byte float
extern const unsigned char ccp_request_EstQaxCurr_str[]             ;   // 16   4 byte float
extern const unsigned char ccp_request_MtrTrq_Cmd_str[]             ;   // 17   4 byte float returns commanded trq value
extern const unsigned char ccp_request_MtrCurrDax_str[]             ;   // 18   4 byte float
extern const unsigned char ccp_request_MtrCurrA_str[]               ;   // 19   4 byte float
extern const unsigned char ccp_request_MtrCurrB_str[]               ;   // 20   4 byte float
extern const unsigned char ccp_request_MtrCurrC_str[]               ;   // 21   4 byte float
extern const unsigned char ccp_request_DigMSB_MtrPos1_Rev_cnt[]     ;   // 22   4 byte float returns MSP SPI ChA value
extern const unsigned char ccp_request_DigMSB_MtrPos2_Rev_cnt[]     ;   // 23   4 byte float returns MSB SPI ChB value
extern const unsigned char ccp_request_DigMSB_MtrPos1_Roll_cnt[]    ;   // 24   cnt returns ChC MSB Sin value
extern const unsigned char ccp_request_DigMSB_MtrPos2_Roll_cnt[]    ;   // 25   cnt returns ChC MSB Cos value
extern const unsigned char ccp_request_DigMSB_MtrPosMecl_Rev_str[]  ;   // 26   Motor Position currently being used by controller
extern const unsigned char ccp_request_AnaMSB_Cos_cnt[]             ;   // 27   4 byte float returns MSP SPI ChA value
extern const unsigned char ccp_request_AnaMSB_Sin_cnt[]             ;   // 28   4 byte float returns MSB SPI ChB value
extern const unsigned char ccp_request_AnaMSB_MtrPosMecl_Rev_cnt[]  ;   // 29   cnt returns ChC MSB Sin value
extern const unsigned char ccp_request_AnaMSB_MtrPosMecl_Roll_cnt[] ;   // 30   cnt returns ChC MSB Cos value
extern const unsigned char ccp_request_MtrCurrPos_Rev_str[]         ;   // 31   Motor Position currently being used by controller
extern const unsigned char ccp_request_ElecMtrPos_Rev_str[]         ;   // 32   Motor Position currently being used by controller
extern const unsigned char ccp_request_prod_3p3V_str[]              ;   // 33   Motor Position currently being used by controller
extern const unsigned char ccp_request_prod_5V_P1_str[]             ;   // 34   Motor Position currently being used by controller
extern const unsigned char ccp_request_prod_5V_P2_str[]             ;   // 35   Motor Position currently being used by controller

//    CAN data requests for 9Bxx    END  ----------------------------------------------------------------- */

// Torque values for  TOC T1XX, C1XX,9BXX using 'FD0D' manufacturing service.
//    Ratiometric scaling to 2048 as 1nm

//extern const unsigned char ms_write_torque_value_pos_0nm[]     ;   // 0 nm torque *** note 5th byte is 00 not 03 for bytes to follow

//extern const unsigned char ms_write_torque_value_pos_p1nm[]    ;   // positive .1 nm 00 CD = 205  205/2048 or 2^11 =.1
//extern const unsigned char ms_write_torque_value_neg_p1nm[]    ;   // negative .1 nm       = -205 =  FF 33

//extern const unsigned char ms_write_torque_value_pos_p5nm[]    ;   // positive .5 nm 04 00 = 1024  1024/2048 or 2^11 =.5
//extern const unsigned char ms_write_torque_value_neg_p5nm[]    ;   // negative .5 nm       = -1024 = FC 00

//extern const unsigned char ms_write_torque_value_pos_1p00nm[]  ;   // positive 1.0 nm 08 00 = 2048  2048/2048 or 2^11 =1
//extern const unsigned char ms_write_torque_value_neg_1p00nm[]  ;   // negative 1.0 nm       = -2048  = F8 00

//extern const unsigned char ms_write_torque_value_pos_2p00nm[]  ;   // positive 2.0 nm 10 00 =    2 * 2048 or 2^11 = 4096
//extern const unsigned char ms_write_torque_value_neg_2p00nm[]  ;   // negative 2.0 nm       = -4096  = F0 00

//extern const unsigned char ms_write_torque_value_pos_4p00nm[]  ;   // positive 4.00 nm 20 00 =   4 * 2048 or 2^11 =     8192
//extern const unsigned char ms_write_torque_value_neg_4p00nm[]  ;   // negative 4.00 nm       = -8192  = F E0 00 ( F is dropped per looking at etool)

//extern const unsigned char ms_write_torque_value_pos_6p00nm[]  ;   // positive 6.00 nm 30 00 =   6 * 2048 or 2^11 =     12288
//extern const unsigned char ms_write_torque_value_neg_6p00nm[]  ;   // negative 6.00 nm       = -12288  = F D0 00    ( F is dropped per looking at etool)


/*    CAN data requests for TOC T1XX, C1XX,9BXX    END  -----------------------------------------------------------------*/

/*  ---------    Torque values for  TOC G2KCA EA4 using 'FD5F' manufacturing service.  START   ---------------------------------------*/
/*                  The 'FD5F' SHORT TERM ADJUST service spans two messages, with (significant delay between each transmit - Phil )   */
/*                  Note: The 'FD5F' turn off is a single message, we use to set to Zero Torque - Phil                                */

// Lets try using the standard service and set torque to  0.0nm  - Leonard 30MAY17

extern const unsigned char ms_write_torque_value_x30[]         ;    // start service ?

// Lets try using the standard service 'FD40', and set torque to  0.0nm  - Leonard 30MAY17
extern const unsigned char ms_write_torque_value_pos_0nm[]     ;   // positive 0 nm = 00 00 00 00
extern const unsigned char ms_write_torque_value_pos_0nm2[]    ;

extern const unsigned char ms_write_torque_value_pos_p1nm[]    ;   // positive .1 nm = 3D CC CC CD
extern const unsigned char ms_write_torque_value_pos_p1nm2[]   ;

extern const unsigned char ms_write_torque_value_neg_p1nm[]    ;   // negative .1 nm = BD CC CC CD
extern const unsigned char ms_write_torque_value_neg_p1nm2[]   ;

extern const unsigned char ms_write_torque_value_pos_p5nm[]    ;   // positive .5 nm = 3F 00 00 00
extern const unsigned char ms_write_torque_value_pos_p5nm2[]   ;

extern const unsigned char ms_write_torque_value_neg_p5nm[]    ;   // negative .5 nm = BF 00 00 00
extern const unsigned char ms_write_torque_value_neg_p5nm2[]   ;

extern const unsigned char ms_write_torque_value_pos_1p00nm[]  ;   // positive 1.0 nm = 3F 80 00 00
extern const unsigned char ms_write_torque_value_pos_1p00nm2[] ;

extern const unsigned char ms_write_torque_value_neg_1p00nm[]  ;   // negative 1.0 nm = BF 80 00 00
extern const unsigned char ms_write_torque_value_neg_1p00nm2[] ;

extern const unsigned char ms_write_torque_value_pos_1p50nm[]  ;   // positive 1.5 nm = 3F C0 00 00
extern const unsigned char ms_write_torque_value_pos_1p50nm2[] ;

extern const unsigned char ms_write_torque_value_neg_1p50nm[]  ;   // negative 1.5 nm = BF C0 00 00
extern const unsigned char ms_write_torque_value_neg_1p50nm2[] ;

extern const unsigned char ms_write_torque_value_pos_2p00nm[]  ;   // positive 2.0 nm = 40 00 00 00
extern const unsigned char ms_write_torque_value_pos_2p00nm2[] ;

extern const unsigned char ms_write_torque_value_neg_2p00nm[]  ;   // negative 2.0 nm = C0 00 00 00
extern const unsigned char ms_write_torque_value_neg_2p00nm2[] ;

extern const unsigned char ms_write_torque_value_pos_2p50nm[]  ;   // positive 2.5 nm = 40 20 00 00
extern const unsigned char ms_write_torque_value_pos_2p50nm2[] ;

extern const unsigned char ms_write_torque_value_neg_2p50nm[]  ;   // negative 2.5 nm = C0 20 00 00
extern const unsigned char ms_write_torque_value_neg_2p50nm2[] ;

extern const unsigned char ms_write_torque_value_pos_3p00nm[]  ;   // positive 3.0 nm = 40 40 00 00
extern const unsigned char ms_write_torque_value_pos_3p00nm2[] ;

extern const unsigned char ms_write_torque_value_neg_3p00nm[]  ;   // negative 3.0 nm = C0 40 00 00
extern const unsigned char ms_write_torque_value_neg_3p00nm2[] ;

extern const unsigned char ms_write_torque_value_pos_3p50nm[]  ;   // positive 3.5 nm = 40 60 00 00
extern const unsigned char ms_write_torque_value_pos_3p50nm2[] ;

extern const unsigned char ms_write_torque_value_neg_3p50nm[]  ;   // negative 3.5 nm = C0 00 00 00
extern const unsigned char ms_write_torque_value_neg_3p50nm2[] ;

extern const unsigned char ms_write_torque_value_pos_4p00nm[]  ;   // positive 4.00 nm = 40 80 00 00
extern const unsigned char ms_write_torque_value_pos_4p00nm2[] ;

extern const unsigned char ms_write_torque_value_neg_4p00nm[]  ;   // negative 4.00 nm = C0 80 00 00
extern const unsigned char ms_write_torque_value_neg_4p00nm2[] ;

extern const unsigned char ms_write_torque_value_pos_6p00nm[]  ;   // positive 6.00 nm = 40 C0 00 00
extern const unsigned char ms_write_torque_value_pos_6p00nm2[] ;

extern const unsigned char ms_write_torque_value_neg_6p00nm[]  ;   // negative 6.00 nm = C0 C0 00 00
extern const unsigned char ms_write_torque_value_neg_6p00nm2[] ;


/*    CAN data requests for TOC G2KCA EA4    END     -----------------------------------------------------------------*/






// Torque values for  TOC using 'FD0D' manufacturing service, (floats) in Nm value for cal/functionals.
// 9Bxx uses percents

extern const unsigned char ms_write_motor_torq_serv_pos_0prcnt[]    ;   // 0 nm torque *** note 5th byte is 00 not 03 for bytes to follow

//const unsigned char ms_write_motor_torq_serv_pos_0prcnt[] = {0x06,0x2F,0xFD,0x0D,0x03,0x00,0x00,0x00}; // positive values NOT correct
//const unsigned char ms_write_motor_torq_serv_neg_0prcnt[] = {0x06,0x2F,0xFD,0x0D,0x03,0x00,0x00,0x00}; // negative values NOT correct

extern const unsigned char ms_write_motor_torq_serv_pos_2prcnt[]    ; //0p10nm, 2.42A
extern const unsigned char ms_write_motor_torq_serv_neg_2prcnt[]    ;

extern const unsigned char ms_write_motor_torq_serv_pos_8prcnt[]    ; //0p35nm, 9.68A
extern const unsigned char ms_write_motor_torq_serv_neg_8prcnt[]    ;

extern const unsigned char ms_write_motor_torq_serv_pos_16prcnt[]   ; //0p80nm, 19.36A
extern const unsigned char ms_write_motor_torq_serv_neg_16prcnt[]   ;

extern const unsigned char ms_write_motor_torq_serv_pos_30prcnt[]   ; //1p70nm, 36.3A
extern const unsigned char ms_write_motor_torq_serv_neg_30prcnt[]   ;

extern const unsigned char ms_write_motor_torq_serv_pos_95prcnt[]   ; //imax, 114.95A (95%)
extern const unsigned char ms_write_motor_torq_serv_neg_95prcnt[]   ;

//--------------------FLEXRAY STUFF---------------------------------------------------------

uint32 count1 = 0;
uint32 bullshit_count = 0;
uint32 bullshit_array[500][2] = {0};

//Declarations of nessecary confiuration data structures.
bufferTransferConfig outputBufferTransferConfigStruct;
bufferTransferConfig *outputBufferTransferSettings = &outputBufferTransferConfigStruct;
extern headerSectionConfig *V_VEH_COG_Header;
extern bufferTransferConfig *V_VEH_COG_Buffer;
extern uint32 go_ahead;

extern uint16 CON_VEH_FAAR_WE_crc_id;
extern uint16 ST_CENG_FAAR_WE_crc_id;
extern uint16 VEH_COG_FAAR_WE_crc_id;

extern headerSectionConfig *XCP_TX_Header;
extern bufferTransferConfig *XCP_TX_Buffer;

//Flexray Error notification variables for dumps/restart bus, etc
extern int fray_error_flag;                     //used in main to restart fray comms if bus_halt error detected.
extern int fray_error_occurred_flag;            //set to 0 after dumperr/data sent, indicates jump to frayErrorNotif occurred
extern int fray_error_counter;                  //counts # of times frayErrorNotif has been called, set limit on this to max uint32 counts
extern uint32 fray_error_notification;          //stores which notification caused frayErrorNotif
extern int wait_for_fray_cold_start_node_flag;  //Set to 1 in frayStartCommunication() if cold start not active (flexray_comm_status_vector_CCSV == 0x2). Period check of cold start node in rtiNotif until detect it is active.
extern uint32 flexray_comm_status_vector_CCSV;  //Used to check cold start node active.
extern int flexray_normal_status_updates_flag;  //Used to tell rtiNotification it is OK to check status of flexray bus every 500ms

extern uint8 MFG_TX_system_state_MSG[64][4]     ;
extern uint8 MFG_TX_request_software_version_MSG[64][4] ;
extern uint8 MFG_TX_request_software_rev_MSG[64][4] ;
extern uint8 MFG_TX_get_dtcs_MSG[64][4]         ;
extern uint8 MFG_TX_clear_dtcs_MSG[64][4]       ;
extern uint8 MFG_TX_cca_part_num_MSG[64][4]     ;
extern uint8 MFG_TX_sys_serial_num_MSG[64][4]   ;
extern uint8 MFG_TX_mtr_trq_cmd_MSG[64][4]  ;

int system_state_MFG;
char software_version_MFG[100];
char software_rev_MFG[10];


//--------------------END Generic FLEXRAY STUFF---------------------------------------------------------

//-----------B M W   U K L ---------------------------------------------------------------------------------------
extern uint8 fray_rx_data[64][4];

extern int fray_rx_data_length;
extern int fray_request_index;

extern flexrayPacket KLEMMEN_struct;
extern flexrayPacket *KLEMMEN;

extern flexrayPacket VEH_COG_struct;
extern flexrayPacket *VEH_COG;

extern flexrayPacket DT_PT_struct;
extern flexrayPacket *DT_PT;

extern flexrayPacket AVL_struct;
extern flexrayPacket *AVL;

extern flexrayPacket XCP_TX_struct;
extern flexrayPacket *XCP_TX;

extern flexrayPacket XCP_RX_struct;
extern flexrayPacket *XCP_RX;

extern flexrayPacket MFG_TX_struct;
extern flexrayPacket *MFG_TX ;

extern flexrayPacket MFG_RX_struct;
extern flexrayPacket *MFG_RX;

extern uint8 XCP_UKL_Batt_Volt_Rqst[64][4];
extern uint8 XCP_UKL_Batt_Curr_Rqst[64][4];
extern uint8 XCP_UKL_Mot_Curr_Rqst[64][4];
extern uint8 XCP_UKL_Mot_Vel_Rqst[64][4];
extern uint8 XCP_UKL_Comm_Torque_Rqst[64][4];
extern uint8 XCP_UKL_Lim_Torque_Rqst[64][4];
extern uint8 XCP_UKL_PCB_Temp_Rqst[64][4];
extern uint8 XCP_UKL_Mot_Temp_Rqst[64][4];
extern uint8 XCP_UKL_Junction_Temp_Rqst[64][4];
extern uint8 XCP_UKL_HW_Angle_Rqst[64][4];
extern uint8 XCP_UKL_Diff_Torque_Rqst[64][4];
extern uint8 XCP_UKL_sys_state_Rqst[64][4];
extern uint8 XCP_UKL_T1_Volt_Rqst[64][4];
extern uint8 XCP_UKL_T2_Volt_Rqst[64][4];

extern uint8 HW_ACK[64][4];
extern uint8 Get_UKL_DTC_Rqst[64][4];

extern short KLEMMEN_CNT;
extern short DT_PT_CNT;
extern short VEH_COG_CNT;

extern uint8 KLEMMEN_MSG[64][4];
extern uint8 DT_PT_MSG[64][4];
extern uint8 VEH_COG_MSG[64][4];
extern uint8 XCP_TX_request_connect_MSG[64][4];

extern uint8_t fray_dump_err_data[22][9];      // storage for CAN responce messages, must store responce and process in MAIN, error stack if sprintf called from ISR
uint8 fray_variable_data[8];
///-----------------------------END OF FLEXRAY STUFF--------------------------------------------------
//-----------F A A R    W E ------------------------------------------------------------------------------------------------------------

extern uint8_t start_torque;
extern uint32_t prev_val;

extern uint8 swapped_fray_rx_data[64][4];

extern flexrayPacket CON_VEH_FAAR_WE_struct;
extern flexrayPacket *CON_VEH_FAAR_WE;

extern flexrayPacket ST_CENG_FAAR_WE_struct;
extern flexrayPacket *ST_CENG_FAAR_WE;

extern flexrayPacket VEH_COG_FAAR_WE_struct;
extern flexrayPacket *VEH_COG_FAAR_WE;

extern const unsigned char EA4_Get_DTC_rqst1[];
extern const unsigned char EA4_Get_DTC_rqst2[];

extern uint8 CON_VEH_FAAR_WE_MSG[64][4];
extern uint8 ST_CENG_FAAR_WE_MSG[64][4];
extern uint8 VEH_COG_FAAR_WE_MSG[64][4];

extern uint8 VEH_COG_FAAR_WE_MSG_0kph[4];
extern uint8 VEH_COG_FAAR_WE_MSG_load_kph[4];
extern uint8 VEH_COG_FAAR_WE_MSG_valid_msg[4];

extern uint8 XCP_FAAR_WE_SysStMod_Rqst[64][4];
extern uint8 XCP_FAAR_WE_BattVltg_BrdgVltg_Rqst[64][4];
extern uint8 XCP_FAAR_WE_BattRtnCurrAmpr_Rqst[64][4];
extern uint8 XCP_FAAR_WE_HwAgArbn_Rqst[64][4];
extern uint8 XCP_FAAR_WE_HwTq4Meas_Rqst[64][4];
extern uint8 XCP_FAAR_WE_HwTq5Meas_Rqst[64][4];
extern uint8 XCP_FAAR_WE_HwTqArbn_Rqst[64][4];
extern uint8 XCP_FAAR_WE_MotCurrSumA_Rqst[64][4];
extern uint8 XCP_FAAR_WE_MotCurrSumB_Rqst[64][4];
extern uint8 XCP_FAAR_WE_MotCurrSumC_Rqst[64][4];
extern uint8 XCP_FAAR_WE_MotVelCrf_Rqst[64][4];
extern uint8 XCP_FAAR_WE_MotTqCmd_Rqst[64][4];
extern uint8 XCP_FAAR_WE_MotTqEstimd_Rqst[64][4];
extern uint8 XCP_FAAR_WE_LoaSca_Rqst[64][4];
extern uint8 XCP_FAAR_WE_EcuTMeas_Rqst[64][4];
extern uint8 XCP_FAAR_WE_MotWidgT_Rqst[64][4];
extern uint8 XCP_FAAR_WE_MotMagT_Rqst[64][4];
extern uint8 XCP_FAAR_WE_MotFetT_Rqst[64][4];

//extern uint8 XCP_FAAR_WE_MotTq_Rqst[64][4];

extern uint8 fray_EA4_Get_DTC_Rqst[64][4];

extern short CON_VEH_FAAR_WE_CNT;
extern short ST_CENG_FAAR_W_CNT;
extern uint8 VEH_COG_FAAR_WE_CNT;

extern flexrayPacket XCP_DYNAMIC_RX_struct;
extern flexrayPacket *XCP_DYNAMIC_RX;

/*Below is hard-coded CRC for vehicle speed = 0KPH. Need to make modifications to CRC for little endian (swap last 4 bytes or something)*/
static uint8 CrcTable_1[15] = {0x95, 0x08, 0xB2, 0x2F, 0xDB, 0x46, 0xFC, 0x61, 0x09, 0x94, 0x2E, 0xB3, 0x47, 0xDa, 0x60};


//---------------------------------- END OF FAAR WE FLEXRAY STUFF --------------------------------------------------------------------------

//----------------------------MTS variables for systems lab machine control and feedback--------------------------------------------------

extern float MTS_Vehicle_Speed_flt;     // float variable for vehicle speed from the MTS machine
extern uint16 MTS_Vehicle_Speed_uint16;   // vehicle speed converted to 16 bit (2 bytes) form float
extern uint8 MTS_Vehicle_Speed_HEX_LoByte;  // scaled low byte HEX value of vehicle speed
extern uint8 MTS_Vehicle_Speed_HEX_HiByte;  // scaled high byte HEX value of vehicle speed

extern uint8 MTS_Store_DAQ_Data_Mode_HEX_Byte; // variable for getting DAQ storage mode from the MTS machine and outputting the value
// in the output stream to signal the LabView app to store(1) or NOT store(0) data

extern uint8 MTS_running_flag; //byte set in canMessagenotif if getting CAN messages from MTS. 8 sec no CAN, send p_off from rti_notif
extern int MTS_running_counter;

extern uint8 MTS_System_State_HEX_Byte;        // byte to store system state from the MTS machine
extern uint8 MTS_System_State_HEX_Byte_old;    // byte to check for CHANGE of system state from the MTS machine
extern uint8 MTS_System_State_change_flag;     // flag to signal a change in desired system state coming from the MTS stand

extern float MTS_Angle_Cmd_fp_in;             // float variable to store Angle command from MTS to Device
extern float MTS_Torque_Cmd_fp_in;            // float variable to store torque command from MTS to Device
extern float old_MTS_torque_Cmd_fp_in;

extern uint8 MTS_Bus_Fault_State_HEX_Byte;        // byte to store bus (fault) state from the MTS machine

extern float DUT_Motor_torque_fp; //variable to store motor torque value for comparison to input from MTS

extern unsigned char DUT_Motor_Angle_out_str[]; // string for CAN out message for DUT Motor Angle out fp on CAN bus#2
extern unsigned char DUT_Motor_Torque_out_str[]; // string for CAN out message for DUT Motor Torque out fp on CAN bus#2
extern unsigned char DUT_Motor_Current_out_str[];  //string for CAN out message for DUT Motor Current out fp on CAN bus#2
extern unsigned char DUT_Motor_Temp_out_str[];     // string for CAN out message for DUT Motor Temo out fp on CAN bus#2
extern unsigned char DUT_Controller_Current_out_str[]; // string for CAN out message for DUT Controller Current out fp on CAN bus#2


extern uint8 General_Fault_state_out_flag;     // byte to hold general fault state status
extern uint8 Fault_FET_temp_out_flag;           // byte to hold fault FET temperature status
extern uint8 Fault_Motor_temp_out_flag;         // byte to hold fault motor temp status

extern unsigned char General_Fault_state_out_str[];
extern unsigned char Fault_FET_temp_out_str[];
extern unsigned char Fault_Motor_temp_out_str[];


//----------------------------END OF  MTS variables for systems lab machine control and feedback------------------------------------------

// --------------------------------------------- DAQ mode vars -----------------------------------------------------------------------//
//Suggest using generic variable names for DAQ mode XCP stuff. Just copy appropriate message box.
#define CAN_XCP_DAQ_TX canMESSAGE_BOX10 //Copy appropriate message box used for XCP over CAN
#define CAN_XCP_DAQ_RX canMESSAGE_BOX11 //Copy appropriate message box used for XCP over CAN

#define CAN 0
#define FRAY 1
extern int comm_is_CAN_or_FRAY;
extern unsigned int DAQ_mode_on_off;  // toggle variable for DAQ mode ON/OFF

extern int XCP_fast_rate_period;
extern int XCP_fast_rate_active_flag;
extern int XCP_fast_rate_index;

extern uint8 xcp_dynamic_rx_data[256];
// -----------------------------------------Variable Input Vars (variable_input.c) -----------------------------------------------------------------------//
extern int read_variables_flag;
// -----------------------------------------End of BMW XCP Fast Rate Vars -----------------------------------------------------------------------//

//E2E CRC calculate for GWM A0607 Checksum 
unsigned char E2E_CRC_Calculation (unsigned char E2E_data_byte_array[])
{
    const unsigned char E2E_POLY = 0x1D; 
    unsigned char E2E_CRC = 0x00;
    int E2E_byte_index;
    int E2E_bit_index;

    // calculate CRC based on E2E vefifcation //
    for(E2E_byte_index = 0; E2E_byte_index < 9; ++E2E_byte_index)
        {
          E2E_CRC^= E2E_data_byte_array[E2E_byte_index]; 
          for(E2E_bit_index = 0; E2E_bit_index < 8; ++E2E_bit_index)
            {
                if((E2E_CRC & 0x80) != 0)
                E2E_CRC = (E2E_CRC<<1)^E2E_POLY;
                else
                E2E_CRC<<=1;
             }
        }
            E2E_CRC ^= 0x00;
    return E2E_CRC;
}

/* USER CODE END */
#pragma WEAK(esmGroup1Notification)
void esmGroup1Notification(uint32 channel)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (1) */
    /* USER CODE END */
}

/* USER CODE BEGIN (2) */
/* USER CODE END */
#pragma WEAK(esmGroup2Notification)
void esmGroup2Notification(uint32 channel)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (3) */
    /* USER CODE END */
}

/* USER CODE BEGIN (4) */
/* USER CODE END */
#pragma WEAK(memoryPort0TestFailNotification)
void memoryPort0TestFailNotification(uint32 groupSelect, uint32 dataSelect, uint32 address, uint32 data)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (5) */

    /* USER CODE END */
}

/* USER CODE BEGIN (6) */
/* USER CODE END */
#pragma WEAK(memoryPort1TestFailNotification)
void memoryPort1TestFailNotification(uint32 groupSelect, uint32 dataSelect, uint32 address, uint32 data)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (7) */

    /* USER CODE END */
}

/* USER CODE BEGIN (8) */
/* USER CODE END */
#pragma WEAK(rtiNotification)
void rtiNotification(uint32 notification)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (9) */

    // real time interrupt ISR, various potential sources, "notification" contains which one

    switch(notification)
    {
    case rtiNOTIFICATION_COMPARE0:
        //      timer_g = rtiGetCurrentTick(rtiCOMPARE0);       //Debug Benchmark timmer
        system_msec_clock++;
        LED_msec_clock++;

        //  HET code to renable normal SENT signals after delay set in p_on (113mS) after i2C finishes initilizing
        HET_msec_countdown_cnt --;
        if (HET_msec_countdown_cnt == 0)
        {
            switch (target_product)
            {
            case TARGET_T1XX:
            case TARGET_BYD_SA2FL:
            case TARGET_GWM_A0607:
            case TARGET_S550DIGITAL:
            case TARGET_FCA_ADAS:
            case TARGET_G2KCA_ADAS:
            case TARGET_SGMW_CN200:
            case TARGET_FORD_T3_T6:
            case TARGET_RENAULT_NISSAN:

                break;

            case TARGET_C1XX:
            case TARGET_CD391_ADAS:
            case TARGET_PSA_CMP:
                hetRAM1->Instruction[0].Program  = 0x0002ba00;  // causes BR01 to run HET code normally looking for triggers and creating SENT signals
                hetRAM1->Instruction[0].Control  = 0x00002220;  // causes BR01 to run HET code normally looking for triggers and creating SENT signals
                break;
            }
        }

        //      timer_e = rtiGetCurrentTick(rtiCOMPARE0);       Debug Benchmark timer

        if((system_msec_clock % 8) == 0)        // slowed down to 8mS 125Hz aquire, was 4mS 250Hz
        {
            adcStartConversion(adcREG1, adcGROUP1); // Start A/D single scan for group1 adc[0 thur 11] 12 channels, Group of 12ch each 8mS
        }

        /******************************************************
         * Flexray cold start node, if main call of frayStartCommunication() didn't detect
         *  cold start node, wait_for_fray_cold_start_node_flag sets = 1.
         *  Periodically check if cold start node is ready.
         *  Done here to prevent holding up main in flexray.c
         * ***************************************************/
        if(((system_msec_clock % 10) == 0)  && (wait_for_fray_cold_start_node_flag == 1))
        {
            if(flexray_comm_status_vector_CCSV == CCSV_NORMAL_ACTIVE) //if active state, get out of loop
            {
                wait_for_fray_cold_start_node_flag = 0;
                frayCommREG->SIR = 0xFFFFFFFF; //Clear any current interrupts
                frayCommREG->EIR = 0xFFFFFFFF;
            }
        }

        /*******************************************************
         * DAQ XCP_fast_rate stuff
         * *****************************************************/
        if(((system_msec_clock % XCP_fast_rate_period) == 0)&&(XCP_fast_rate_active_flag == YES))     /* periodic XCP request transmission  */
        {
            if(comm_is_CAN_or_FRAY == CAN)
            {
                ECU1_XCP_reply_index = 10000;   // set index so old dumperr response code doesn't happen
                XCP_fast_rate_index = 0;
                transmitFlexray(XCP_TX_request_connect_MSG, XCP_TX,2);
            }
            else if(comm_is_CAN_or_FRAY == FRAY)
            {
                fray_request_index = 100000;    // set index so old dumperr response code doesn't happen
                XCP_fast_rate_index = 0;
                transmitFlexray(XCP_TX_request_connect_MSG, XCP_TX,2);
            }
        }
        extern int NTC_get_period;
        extern int NTC_get_flag;
        extern int NTC_main_get_flag;
        if(((system_msec_clock % NTC_get_period) == 0)&&(NTC_get_flag == YES))    /* periodic XCP request transmission  */
        {
            NTC_main_get_flag = 1;
        }

        extern int Set_DAQ_List_mode_DAQ_rate;
        extern int enable_data_stream_flag;
        extern int MTS_only_print;
        if(((system_msec_clock % Set_DAQ_List_mode_DAQ_rate) == 0)&&(ign1_status == 0)&&(enable_data_stream_flag == 1))
        {
            MTS_only_print = 1;
        }

        /***********************************************************
         * MTS_running_flag sets = 1 when any CAN message comes from MTS stand.
         * if no response for 8000 msec, set torque = 0, cuz not sure what state we should be in.
         **********************************************************/
        if((MTS_running_counter == 0)&&(MTS_running_flag == 1)) //MTS_running_counter set = 8000 every time MTS CAN message happens
        {
            MTS_running_flag = 0;
            General_Fault_state_out_flag = 1;
            //strcpy(command_str,"p_off");
            //flag_to_process = 1;
            MTS_System_State_change_flag = YES;
            MTS_System_State_HEX_Byte_old = 0x00;
        }
        else
        {
            MTS_running_counter--;
        }


        //      adcStartConversion(adcREG1, adcGROUP1); // Start A/D single scan for group1 adc[0 thur 11] 12 channels, Group of 12ch each 1mS

        //      timer_f = rtiGetCurrentTick(rtiCOMPARE0);       Debug Benchmark timer

        /*      OLD B_Board
        // LED activity for "SW1_is_snapping_snapped1_snapped2_reset_none" effects LED 1,2,3,4 at GIOA[4,5,6,7]
        //  snapping = each 500mS toggle a LED, scrolling 4then5then6then7then4, ect for 10 seconds(timeout,set to"NONE") or until "SNAPPED"
        //  snapped1  = each 500mS flash all LED, on_off_on ect for 3 seconds, set to "SNAPPED2"
        //  snapped2  = similar to "NONE", with I'm officially snapped and on the WEB
        //  reset    = just turn on all LEDs, and done, don't even think we will get here before the reset occures
        //  none     = every 1000mS test "ign_on_flag" true -> alternate LED3,4 to on_off,, false-> flash LED3, turn and leave LED4 off
         */

        // LED activity for "SW1_is_snapping_snapped1_snapped2_reset_none" effects LED 1,2,3,4 at HET[14,15,18,19]
        //  snapping = each 500mS toggle a LED, scrolling 4then5then6then7then4, ect for 10 seconds(timeout,set to"NONE") or until "SNAPPED"
        //  snapped1  = each 500mS flash all LED, on_off_on ect for 3 seconds, set to "SNAPPED2"
        //  snapped2  = similar to "NONE", with I'm officially snapped and on the WEB
        //  reset    = just turn on all LEDs, and done, don't even think we will get here before the reset occures
        //  none     = every 1000mS test "ign_on_flag" true -> alternate LED3,4 to on_off,, false-> flash LED3, turn and leave LED4 off

        switch(SW1_is_snapping_snapped1_snapped2_reset_none)
        {
        case NONE:
        case SNAPPED2:
            // User indicator for RS485 activity, REC LED will be on briefly when CIB is addressed, TXD will be on briefly when CIB is transmitting
            //          if(LED_REC_msec_clock == 0)
            //          {
            //              gioSetBit(hetPORT1, 14, 0 );        // CLR LED1
            //          }
            //          else
            //          {
            //              LED_REC_msec_clock--;
            //          }
            //
            //          if(LED_TXD_msec_clock == 0)
            //          {
            //              gioSetBit(hetPORT1, 15, 0 );        // CLR LED2
            //          }
            //          else
            //          {
            //              LED_TXD_msec_clock--;
            //          }

            if((LED_msec_clock % 1000) == 0)        // update Bulgarian CIB board LEDs each second
            {
                switch (ign1_status)
                {
                case    NORMAL:     // alternate LEDs each second in RUN
                case    WARM:

                    if(gioGetBit(hetPORT1, 18))     // Test LED3 state, if ON
                    {
                        gioSetBit(hetPORT1, 18, 0 );        // CLR LED3
                        gioSetBit(hetPORT1, 19, 1 );        // SET LED4
                    }
                    else
                    {
                        gioSetBit(hetPORT1, 18, 1 );        // SET LED3
                        gioSetBit(hetPORT1, 19, 0 );        // CLR LED4
                    }

                    break;

                case    OFF:        // flash one LED each second in STOP

                    gioToggleBit(hetPORT1, 18);     // Flash LED3 if IGN_OFF
                    gioSetBit(hetPORT1, 19, 0 );        // CLR LED4, make sure its off

                    break;

                }

            }       // end of if((system_msec_clock % 1000) == 0)
            break;
        case SNAPPING:
            if(system_msec_clock < SNAP_timeout_clock)      // while we are waiting for SNAP, spin LEDs, if no SNAP, reset to status to "NONE"
            {
                if((LED_msec_clock % 100) == 0)
                {
                    //                  snapping_LED_scroll++;
                    switch(snapping_LED_scroll)
                    {
                    case 0:
                        hetREG1->DCLR = 0X000CC000; // HET[14,15,18,19]  LEDs 1,2,3,4 = OFF
                        gioSetBit(hetPORT1, 14, 1 );    // LED1 = ON
                        snapping_LED_scroll = 1;
                        break;
                    case 1:
                        gioSetBit(hetPORT1, 14, 0 );    // LED1 = OFF
                        gioSetBit(hetPORT1, 15, 1 );    // LED2 = ON
                        snapping_LED_scroll = 2;
                        break;
                    case 2:
                        gioSetBit(hetPORT1, 15, 0 );    // LED2 = OFF
                        gioSetBit(hetPORT1, 19, 1 );    // LED4 = ON
                        snapping_LED_scroll = 3;
                        break;
                    case 3:
                        gioSetBit(hetPORT1, 19, 0 );    // LED4 = OFF
                        gioSetBit(hetPORT1, 18, 1 );    // LED3 = ON
                        snapping_LED_scroll = 0;
                        break;
                    default:
                        snapping_LED_scroll = 0;
                        break;

                    }
                }   // end if((LED_msec_clock % 100) == 0)
            }
            else    // SNAP did not occure within 10 seconds, timed out,, reset activity to none, SW1 depression will now be acknowdged
            {
                SW1_is_snapping_snapped1_snapped2_reset_none = NONE;
                gioSetBit(gioPORTA, 5, 0 );     // clr Lantronix port[0], so UDP:30704 MONITOR command 13h00h00h00h00h will send a "0" for this port

            }

            break;
        case SNAPPED1:
            // SNAP_timeout_clock reuse,, new set in MAIN, "snap" command completed
            if(system_msec_clock < SNAP_timeout_clock)      // SNAP seuccessful, flash all LEDs for 3 seconds
            {
                if((LED_msec_clock % 500) == 0)
                {
                    snapping_LED_scroll++;
                    if((snapping_LED_scroll % 1 == 0))
                    {
                        hetREG1->DSET = 0X000CC000; // HET[14,15,18,19]  LEDs 1,2,3,4 = ON
                    }
                    else
                    {
                        hetREG1->DCLR = 0X000CC000; // HET[14,15,18,19]  LEDs 1,2,3,4 = OFF
                    }
                }
            }  // if(system_msec_clock < SNAP_timeout_clock)
            else    // 3 seconds complete,
            {
                SW1_is_snapping_snapped1_snapped2_reset_none = SNAPPED2;    // process complete, get back to normal-snapped-web operations
                gioSetBit(gioPORTA, 5, 0 );     // clr Lantronix port[0], so UDP:30704 MONITOR command 13h00h00h00h00h will send a "0" for this port

            }

            break;
        case RESET: // Just a place holder. Technically I do not want to interfer with RESET LED action. Reality, the code will never get here, it will soft reset.
            break;
        }   // end of switch(SW1_is_snapping_snapped1_snapped2_reset_none)


        //  T E S T I N G   O F   S E N T ---  F O R C E D   T R I G G E R  vvvvvvvvvvvv  S T A R T  vvvvvvvvvvvvvvv

        //      if((system_msec_clock % 2 == 0))      /* Testing out hetRam access, using a direct pointer to ASM line */
        //      {
        //          if(ign1_status != OFF)  // only create trigger if ign is ON
        //          {
        //              gioSetBit(hetPORT1, 24, 1);     // for testing Digital I/O Board Sent 2ch or 6ch boards, bang N2HET1[24]/Interlock to create trigger, clip jumper to SENTx out
        //              gioSetBit(hetPORT1, 24, 0);     //   HalCoGen (for now has set up N2HET1[24]/Interlock to be open drain), this shorting will create trigger on HET[2 or 3]
        //              for(loop_cnt_c = 0; loop_cnt_c < 15 ; loop_cnt_c++);
        //              gioSetBit(hetPORT1, 24, 1);
        //          }
        //      }

        timer_g = system_msec_clock;  //  Debug

        //  T E S T I N G   O F   S E N T ---  F O R C E D   T R I G G E R ^^^^^^^^^^^^   E N D  ^^^^^^^^^^^^^^^^^

        //  h c u r r,   p_on and   common drive - Load profiles for various products    vvvvvvvvvvvvvvvvvvvv  START  vvvvvvvvvvvvvvvv
        switch(target_product)
        {
        case TARGET_T1XX:
        case TARGET_BYD_SA2FL:
        case TARGET_GWM_A0607:
        case TARGET_C1XX:
        case TARGET_G2KCA_ADAS:
        case TARGET_SGMW_CN200:
        case TARGET_RENAULT_NISSAN:

        {
            if(hcurr_flag == 1) // do hcurr torque profile
            {

                switch(hcurr_timer)
                {
                case 0:  // 0 seconds beginning of sequence
                    if(hcurr_direction_POS_NEG == POS)   // swap directions on every hcurr command
                    {
                        hcurr_direction_POS_NEG = NEG; // swap direction
//                      torque_value_case_index = neg_1p00nm; // 16% negative torque value  1.00nm (Original)
                        torque_value_case_index = neg_p5nm; // 15% negative torque value  0.50nm
                    }
                    else
                    {
                        hcurr_direction_POS_NEG = POS; // WAS 0 or NEG...D. Bair 11/22 swap direction
//                      torque_value_case_index = pos_1p00nm; // 16% positive torque value  1.0nm (Original)
                        torque_value_case_index = pos_p5nm; // 15% positive torque value  0.50nm
                    }
                    hcurr_write_interval_data = YES;

                    break;
                    /* T1XX wanted to store extra stuff */
                case 2000:     // 2 seconds into 16% Qax in Hcurr
                    can1_request_index = 2000; // CAN isr case for storing Qax and HwwTq readings during Hcurr
                    can2_request_index = 2000; // CAN isr case for storing Qax and HwwTq readings during Hcurr

                    switch(target_product)
                    {
                    case TARGET_G2KCA_ADAS:
                        canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_MtrCurrQax_str);
                        canTransmit(canREG1, G2KCA_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_MtrCurrQax_str);
                        break;

                    case TARGET_T1XX:
                        canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_MotCurrQax_str);
                        break;

                    case TARGET_BYD_SA2FL:
//                        canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_BYD_SA2FL_MotCurrQax_fp);
                        break;

                    case TARGET_GWM_A0607:
                        canTransmit(canREG1, GWM_A0607_CCP_REQUEST_ID, ( uint8 *) ccp_GWM_A0607_MotCurrQax_fp);
                        break;

                    case  TARGET_C1XX:
                        canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_MtrCurrQax_str);
                        break;

                    case  TARGET_SGMW_CN200:
                        canTransmit(canREG1, CN200_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_CN200_MtrCurrQax_str);
                        break;

                    case  TARGET_RENAULT_NISSAN:
                        canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_RENAULT_NISSAN_MtrCurrQax_str);
                        break;

                    }
                    break;

                    case 4000:  // 4 seconds
                        if(hcurr_direction_POS_NEG == POS)   // swap directions on every hcurr command
                        {
                            hcurr_direction_POS_NEG = NEG; // swap direction
//                            torque_value_case_index = neg_2p00nm; // 30% negative torque value  2.00nm (Original)
                            torque_value_case_index = neg_1p00nm; // 30% negative torque value  1.00nm
                        }
                        else
                        {
                            hcurr_direction_POS_NEG = POS; // swap direction
//                            torque_value_case_index = pos_2p00nm; // 30% positive torque value  2.00nm (Original)
                            torque_value_case_index = pos_1p00nm; // 30% positive torque value  1.00nm
                        }
                        hcurr_write_interval_data = YES;

                        break;

                        /* T1XX wanted to store extra stuff */
                    case 5000:     // at 5seconds, 1 second into 30% Qax in Hcurr
                        can1_request_index = 2010; // CAN isr case for storeing Qax readings during Hcurr
                        can2_request_index = 2010; // CAN isr case for storeing Qax readings during Hcurr
                        switch(target_product)
                        {
                        case TARGET_G2KCA_ADAS:
                            canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_MtrCurrQax_str);
                            canTransmit(canREG1, G2KCA_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_MtrCurrQax_str);
                            break;
                        case TARGET_T1XX:
                            canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_MotCurrQax_str);
                            break;

                        case TARGET_BYD_SA2FL:
//                            canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_BYD_SA2FL_MotCurrQax_fp);
                            break;

                        case TARGET_GWM_A0607:
                            canTransmit(canREG1, GWM_A0607_CCP_REQUEST_ID, ( uint8 *) ccp_GWM_A0607_MotCurrQax_fp);
                            break;

                        case  TARGET_C1XX:
                            canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_MtrCurrQax_str);
                            break;

                        case  TARGET_SGMW_CN200:
                            canTransmit(canREG1, CN200_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_CN200_MtrCurrQax_str);
                            break;

                        case  TARGET_RENAULT_NISSAN:
                            canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_RENAULT_NISSAN_MtrCurrQax_str);
                            break;

                        }
                        break;


                        case 6000: // 6 seconds
                            if(hcurr_direction_POS_NEG == POS)   // swap directions on every hcurr command
                            {
                                hcurr_direction_POS_NEG = NEG; // swap direction
                                torque_value_case_index = neg_p5nm; // 8% negative torque value  .5nm
                            }
                            else
                            {
                                hcurr_direction_POS_NEG = POS; // swap direction
                                torque_value_case_index = pos_p5nm; // 8% positive torque value  .5nm
                            }
                            hcurr_write_interval_data = YES;

                            break;

                            /* T1XX wanted to store extra stuff */
                        case 8000:     // at 8 seconds, 2 seconds into 8% Qax in Hcurr
                            can1_request_index = 2020; // CAN isr case for storeing Qax readings during Hcurr
                            can2_request_index = 2020; // CAN isr case for storeing Qax readings during Hcurr
                            switch(target_product)
                            {
                            case TARGET_G2KCA_ADAS:
                                canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_MtrCurrQax_str);
                                canTransmit(canREG1, G2KCA_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_MtrCurrQax_str);
                                break;
                            case TARGET_T1XX:
                                canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_MotCurrQax_str);
                                break;

                            case TARGET_BYD_SA2FL:
//                                canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_BYD_SA2FL_MotCurrQax_fp);
                                break;

                            case TARGET_GWM_A0607:
                                 canTransmit(canREG1, GWM_A0607_CCP_REQUEST_ID, ( uint8 *) ccp_GWM_A0607_MotCurrQax_fp);
                                 break;

                            case  TARGET_C1XX:
                                canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_MtrCurrQax_str);
                                break;

                            case  TARGET_SGMW_CN200:
                                canTransmit(canREG1, CN200_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_CN200_MtrCurrQax_str);
                                break;

                            case  TARGET_RENAULT_NISSAN:
                                canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_RENAULT_NISSAN_MtrCurrQax_str);
                                break;

                            }
                            break;

                            case 10000: // 10 seconds, done, set to default
                                if(hcurr_direction_POS_NEG == POS)   // swap directions on every hcurr command
                                {
                                    hcurr_direction_POS_NEG = NEG; // swap direction
                                    torque_value_case_index = neg_p1nm; // 2% negative torque value  .1nm (Original)
                                }
                                else
                                {
                                    hcurr_direction_POS_NEG = POS; // swap direction
                                    torque_value_case_index = pos_p1nm; // 2% positive torque value  .1nm (Original)
                                }
                                hcurr_write_interval_data = YES;

                                break;
                }  // end switch hcurr_timer

                hcurr_timer ++;

            } // end if hcurr_flag = 1
        }  // case TARGET_C1XX: case TARGET_T1XX:

        case TARGET_FCA_ADAS:
        {
            if(hcurr_flag == 1)
            {

                switch(hcurr_timer)
                {
                case 0:
                    // start with high current at 0 seconds, also change T1/T2 opposite direction of max current above
                    hcurr_change_direction_at_nm(4.0);  // set to 4.0nm
                    hcurr_write_interval_data = YES;

                    break;

                case 500:
                    // 500 mS into high current request data
                    can1_request_index = 2000; // CAN isr case for storeing Qax readings during Hcurr
                    canTransmit(canREG1, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_MtrCurrQax_str);

                    can2_request_index = 2000; // CAN isr case for storeing Qax readings during Hcurr
                    canTransmit(canREG2, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_MtrCurrQax_str);

                    break;

                case 1100:
                    // start of medium current at 1.1 seconds, also change T1/T2 opposite direction of max current above
                    hcurr_change_direction_at_nm(2.0);
                    hcurr_write_interval_data = YES;

                    break;

                case 1600:
                    // 500 mS into med current request data
                    can1_request_index = 2010; // CAN isr case for storeing Qax readings during Hcurr
                    canTransmit(canREG1, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_MtrCurrQax_str);

                    can2_request_index = 2010; // CAN isr case for storeing Qax readings during Hcurr
                    canTransmit(canREG2, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_MtrCurrQax_str);

                    break;


                case 3600:
                    // switch T1/T2 half way thru med curr
                    hcurr_change_direction_at_nm(2.0); // flip flop T1/T2 for med torque direction
                    hcurr_write_interval_data = YES;

                    break;

                case 6100:
                    hcurr_change_direction_at_nm(0.5); // switch T1/T2 to 10% or 0.5nm
                    hcurr_write_interval_data = YES;

                    break;

                case 6600:
                    // 500 mS into low current request data
                    can1_request_index = 2020; // CAN isr case for storing Qax readings
                    canTransmit(canREG1, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_MtrCurrQax_str);

                    can2_request_index = 2020; // CAN isr case for storing Qax readings
                    canTransmit(canREG2, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_MtrCurrQax_str);

                    break;

                case 7100:
                    stop_T1T2switching = 0;  // start switching for low current at 1 sec intervals

                    break;

                case 56500:
                    stop_T1T2switching = 1;  // stop switching for low current

                    // start of medium current so change T1/T2 opposite of low current direction above
                    hcurr_change_direction_at_nm(2.0); // flip flop T1/T2 for med torque direction
                    hcurr_write_interval_data = YES;

                    break;

                case 59000:
                    hcurr_change_direction_at_nm(2.0); // flip flop T1/T2 for med torque direction
                    hcurr_write_interval_data = YES;

                    break;

                case 61500:
                    hcurr_change_direction_at_nm(0.5); // switch T1/T2 to 10% or 0.5nm
                    hcurr_write_interval_data = YES;

                    stop_T1T2switching = 0;  // start switching for normal current

                    break;

                case 111500:
                    hcurr_change_direction_at_nm(0.0); // switch T1/T2 to 0% or 0.0nm
                    hcurr_write_interval_data = YES;

                    stop_T1T2switching = 1;  // stop switching for zero amps
                    break;


                case 146500:
                    hcurr_flag =0;  // end of high curr command for direction =0. 1.5sec of hi and 5 sec med =6.5 sec
                    torque_value_case_index = pos_0nm; // set torque to 0amps
                    hcurr_write_interval_data = YES;

                    stop_T1T2switching = 1;  // stop switching for zero amps


                    break;

                } // end switch hcurr clk case hcurr direction high to low



                if(hcurr_timer > 147000)    // safety incase hcurr clk goes past 147 secs it gets reset
                {
                    hcurr_flag =0;  // end of high curr command. 3.5sec of hi 10 sec med =13.5 sec
                    torque_value_case_index = pos_0nm; // set torque to 0amps
                    hcurr_write_interval_data = YES;
                }

                hcurr_timer ++;


            } // end high curr flag is true


            if(((system_msec_clock % 1000) == 0) && (stop_T1T2switching == 0)) /* have waited one second, so change T1 and T2 states */
                // and T1/T2 stop switching flag is NOT set
            {
                hcurr_change_direction_at_nm(0.5);
                hcurr_write_interval_data = YES;

            }  // end if sys_clk mods with  1sec
        }  // case TARGET_FCA_ADAS

        case TARGET_PSA_CMP:
        {
            if(hcurr_flag == 1)
            {

                switch(hcurr_timer)
                {
                // just place holders for future "on demand" load profiles
                // start at 0mS
                case 0:
                    break;

                case 500:
                    break;

                case 1100:
                    break;

                } // end switch hcurr clk case hcurr direction high to low

                //              if(hcurr_timer > 147000)  // safety in case hcurr clk goes past xxx secs it gets reset
                //                {
                //                  hcurr_flag =0;  // end of high curr command. 3.5sec of hi 10 sec med =13.5 sec
                //                  torque_value_case_index = pos_0nm; // set torque to 0amps
                //                  hcurr_write_interval_data = YES;
                //                }
                //
                //                hcurr_timer ++;

            } // end high curr flag is true


            if(((system_msec_clock % 1000) == 0) && (stop_T1T2switching == 0)) /* have waited one second, so change T1 and T2 states */
                // and T1/T2 stop switching flag is NOT set
            {
                // For PSA we want to torque polarity to switch at any torque value, IF stop_T1T2switching is false
                switch (torque_value_case_index)
                {   // if positive make negative, if negative make positive, afterwards set flag for main to process
                case pos_0nm   : torque_value_case_index = pos_0nm;   break;
                case pos_p1nm  : torque_value_case_index = neg_p1nm;  break;
                case neg_p1nm  : torque_value_case_index = pos_p1nm;  break;
                case pos_p5nm  : torque_value_case_index = neg_p5nm;  break;
                case neg_p5nm  : torque_value_case_index = pos_p5nm;  break;
                case pos_1p00nm: torque_value_case_index = neg_1p00nm;  break;
                case neg_1p00nm: torque_value_case_index = pos_1p00nm;  break;
                case pos_1p50nm: torque_value_case_index = neg_1p50nm;  break;
                case neg_1p50nm: torque_value_case_index = pos_1p50nm;  break;
                case pos_2p00nm: torque_value_case_index = neg_2p00nm;  break;
                case neg_2p00nm: torque_value_case_index = pos_2p00nm;  break;
                case pos_2p50nm: torque_value_case_index = neg_2p50nm;  break;
                case neg_2p50nm: torque_value_case_index = pos_2p50nm;  break;
                case pos_3p00nm: torque_value_case_index = neg_3p00nm;  break;
                case neg_3p00nm: torque_value_case_index = pos_3p00nm;  break;
                case pos_3p50nm: torque_value_case_index = neg_3p50nm;  break;
                case neg_3p50nm: torque_value_case_index = pos_3p50nm;  break;
                case pos_4p00nm: torque_value_case_index = neg_4p00nm;  break;
                case neg_4p00nm: torque_value_case_index = pos_4p00nm;  break;
                case pos_6p00nm: torque_value_case_index = neg_6p00nm;  break;
                case neg_6p00nm: torque_value_case_index = pos_6p00nm;  break;

                }
                hcurr_write_interval_data = YES;

            }  // end if sys_clk mods with  1sec
        }  // case TARGET_PSA_CMP

        }  //switch(target_product) hcurr

        //  h c u r r   p_on Load profiles for various products    ^^^^^^^^^^^^^^^^^^^^^  END   ^^^^^^^^^^^^^^


        //  In case someone wants to use a high_torque timer
        if(high_torque_timer_flag == 1)  // flag is set for 10 second timer
        {
            high_torque_timer_counter--; // decrement counter till zero

            //          if(high_torque_timer_counter < 1)
            //          {
            //              high_torque_timer_flag = 0; // timer over so set flag to off
            //          }

        } // end if high torque timer flag is true



        /*  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV   S T A N D A R D    I G N   S E R V I C I N G     VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  */

        switch(target_product)  // switch(target_product) for ign1_status > OFF and ign1_status = OFF
        {

        case TARGET_BMW_FAAR_WE:
        {
            if(tester_present_enable_flag == 1)
            {
                tester_present_timer--;
                extern uint8 XCP_FAAR_WE_TesterPresent_Rqst_MSG[64][4];
                if(tester_present_timer < 1)
                {
                    tester_present_timer = TESTER_PRESENT_SEND_TIME; // reset timer to 3 seconds ..must be sent sooner than every 5 seconds
                    ECU1_XCP_reply_index = 20; // Do nothing index..do nothing with reply message
                    transmitFlexray(XCP_FAAR_WE_TesterPresent_Rqst_MSG,MFG_TX, 3);

                } // end if (tester_present_timer < 1)
            } //end if(tester_present_enable_flag == 1)
            break;
        } //end case TARGET_BMW_FAAR_WE:

        case TARGET_BMW_UKL:
        {

            break;
        } //end case TARGET_BMW_UKL:

        case TARGET_FORD_T3_T6:
        {
            /* -----------  i g n _ s t a t u s  >  O F F     ----------( ON or Warm)-----------------    */
            if(ign1_status > OFF)   // ign1_status is ON or WARM
            {
                if(manufacturing_TOC == YES)
                {
                    if(tester_present_enable_flag == 1)
                    {
                        tester_present_timer--;

                        if(tester_present_timer < 1)
                        {
                            tester_present_timer = TESTER_PRESENT_SEND_TIME; // reset timer to 3 seconds ..must be sent sooner than every 5 seconds
                            ECU1_XCP_reply_index = 20; // Do nothing index..do nothing with reply message
                            ECU2_XCP_reply_index = 20; // Do nothing index..do nothing with reply message
                            canTransmit(canREG1, FORD_T3_ECU1_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_write_tester_present_msg);
                            canTransmit(canREG1, FORD_T3_ECU2_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_write_tester_present_msg);
                        } // end if (tester_present_timer < 1)

                    } // end if tester_present_enable_flag = 1
                }   // end if manufacturing_TOC == YES

                if((system_msec_clock % 20) == 0)   // was 10mS, told 20ms 27OCT17 email Kenneth Pryczynski
                {
                    /* periodic transmission of engine_on CAN at 10ms  */

                    if(ign1_status == NORMAL)
                        //              if (Engine_on_Signal == 1)
                    {
                        canTransmit(canREG1, FORD_T3_ECU1_CAN1_CCP_Engine_Run, ( uint8 *) ccp_FORD_T3_engine_run);
                    }
                    else    // Put in warm_init
                    {
                        canTransmit(canREG1, FORD_T3_ECU1_CAN1_CCP_Engine_Run, ( uint8 *) ccp_FORD_T3_engine_off);
                    }

                }      // end if timer2_msec mods with 10


                if((system_msec_clock % 20) == 0)     /* periodic transmission of speed at 20ms  */
                {
                    // speed always (default) = zero
                    canTransmit(canREG1, FORD_T3_ECU1_CAN1_CCP_SPEED_ID, ( uint8 *) ccp_FORD_T3_write_0_to_speed);

                }   // end if timer2_msec mods with 20 for speed

            }     // ign1_status is ON or WARM

            /* VVVVVVVVVVVVVVVV  i g n _ s t a t u s  ==  O F F  VVVVVVVVVVVVVVVVVVVVVVVVV  */

            else     // ign1_status is OFF  ----   ign_on_flag = FALSE  or ignition OFF just send power mode OFF message
            {

                if((system_msec_clock % 10) == 0)
                {
                    canTransmit(canREG1, FORD_T3_ECU1_CAN1_CCP_Engine_Run, ( uint8 *) ccp_FORD_T3_engine_off);

                }      // end if timer2_msec mods with 10

                if((system_msec_clock % 20) == 0)     /* periodic transmission of speed at 20ms  */
                {
                    // speed always (default) = zero
                    canTransmit(canREG1, FORD_T3_ECU1_CAN1_CCP_SPEED_ID, ( uint8 *) ccp_FORD_T3_write_0_to_speed);

                }   // end if timer2_msec mods with 20 for speed

            }   // end if ignition OFF  else statement
        }  // case TARGET_FORD_T3_T6:
        break; // case TARGET_FORD_T3_T6:

        case TARGET_G2KCA_ADAS:
        {
            /* -----------  i g n _ s t a t u s  >  O F F     ----------( ON or Warm)-----------------    */
            if(ign1_status > OFF)   // ign1_status is ON or WARM
            {
                if(manufacturing_TOC == YES)
                {
                    if(tester_present_enable_flag == 1)
                    {
                        tester_present_timer--;

                        if(tester_present_timer < 1)
                        {
                            tester_present_timer = TESTER_PRESENT_SEND_TIME; // reset timer to 3 seconds ..must be sent sooner than every 5 seconds
                            ECU1_XCP_reply_index = 20; // Do nothing index..do nothing with reply message
                            ECU2_XCP_reply_index = 20; // Do nothing index..do nothing with reply message
                            canTransmit(canREG1, G2KCA_ECU1_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_write_tester_present_msg);
                            canTransmit(canREG1, G2KCA_ECU2_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_write_tester_present_msg);
                        } // end if (tester_present_timer < 1)

                    } // end if tester_present_enable_flag = 1
                }   // end if manufacturing_TOC == YES

                if((system_msec_clock % 12) == 0)
                {
                    /* periodic transmission of engine_on CAN at 12ms  */

                    if(ign1_status == NORMAL)
                        //              if (Engine_on_Signal == 1)
                    {
                        canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_Engine_Run, ( uint8 *) ccp_write_engine_run);
                    }
                    else    // Put in warm_init
                    {
                        canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_Engine_Run, ( uint8 *) ccp_write_engine_off);
                    }

                }      // end if timer2_msec mods with 12

                /* periodic transmission of Power Mode Run 100ms  */
                if((system_msec_clock % 100) == 0)
                {
                    canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_Power_Mode_ID, ( uint8 *) ccp_write_PowerModeON);
                }      // end if timer2_msec mods with 100

                if((system_msec_clock % 37) == 0)     /* periodic transmission of speed at 100ms  */
                    // CHANGED to 56 now 37 ms because power mode is 100ms also
                {
                    // speed always (default) = zero
                    // not used             canTransmit(canREG1, G2KCA_CCP_SPEED_ID, ( uint8 *) ccp_write_0_to_speed);
                    // not used             canTransmit(canREG2, G2KCA_CCP_SPEED_ID, ( uint8 *) ccp_write_0_to_speed);

                    // not used             canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_Driven_ID, ( uint8 *) ccp_write_wheel_speed);
                    // not used             canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_NonDriven_ID, ( uint8 *) ccp_write_wheel_speed);

                    // not used             canTransmit(canREG2, G2KCA_ECU2_CAN2_CCP_Driven_ID, ( uint8 *) ccp_write_wheel_speed);
                    // not used             canTransmit(canREG2, G2KCA_ECU2_CAN2_CCP_NonDriven_ID, ( uint8 *) ccp_write_wheel_speed);

                    if(G2KCA_ADAS_CRC_counter == 3)
                    {
                        G2KCA_ADAS_CRC_counter = 0;
                    }
                    else
                    {
                        G2KCA_ADAS_CRC_counter++;
                    }

                    G2KCA_ADAS_Write_Vehicle_speed[6] = (G2KCA_ADAS_CRC_counter) << 3;  // write 2 bit rolling counter, bits 3&4 of byte 6

                    G2KCA_ADAS_crcVal_16 = G2KCA_ADAS_Write_Vehicle_speed[0]+G2KCA_ADAS_Write_Vehicle_speed[1]+G2KCA_ADAS_Write_Vehicle_speed[2]+G2KCA_ADAS_Write_Vehicle_speed[3]\
                            +G2KCA_ADAS_Write_Vehicle_speed[4]+G2KCA_ADAS_Write_Vehicle_speed[5]+G2KCA_ADAS_CRC_counter+0x69;

                    G2KCA_ADAS_Write_Vehicle_speed[6] = G2KCA_ADAS_Write_Vehicle_speed[6] + (G2KCA_ADAS_crcVal_16 >>8);

                    G2KCA_ADAS_Write_Vehicle_speed[7] = ((G2KCA_ADAS_crcVal_16<<8)>>8);

                    canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_Driven_ID, ( uint8 *) G2KCA_ADAS_Write_Vehicle_speed);
                    canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_NonDriven_ID, ( uint8 *) G2KCA_ADAS_Write_Vehicle_speed);

                    canTransmit(canREG2, G2KCA_ECU2_CAN2_CCP_Driven_ID, ( uint8 *) G2KCA_ADAS_Write_Vehicle_speed);
                    canTransmit(canREG2, G2KCA_ECU2_CAN2_CCP_NonDriven_ID, ( uint8 *) G2KCA_ADAS_Write_Vehicle_speed);


                }   // end if timer2_msec mods with 56 now 37 for speed

            }     // ign1_status is ON or WARM

            /* VVVVVVVVVVVVVVVV  i g n _ s t a t u s  ==  O F F  VVVVVVVVVVVVVVVVVVVVVVVVV  */

            else     // ign1_status is OFF  ----   ign_on_flag = FALSE  or ignition OFF just send power mode OFF message
            {
                if((system_msec_clock % 100) == 0)
                {
                    /* periodic transmission of Power Mode Run 100ms  */
                    canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_Power_Mode_ID, ( uint8 *) ccp_write_PowerModeOFF);
                }      // end if timer2_msec mods with 100

                if((system_msec_clock % 12) == 0)
                {
                    /* periodic transmission of engine_on CAN at 12ms  */

                    canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_Engine_Run, ( uint8 *) ccp_write_engine_off);

                }      // end if timer2_msec mods with 12

            }   // end if ignition OFF  else statement
        }  // case TARGET_G2KCA:
        break; // case TARGET_G2KCA:

        case TARGET_T1XX:
        case TARGET_C1XX:
        case TARGET_9BXX:
        {
            /* -----------  i g n _ s t a t u s  >  O F F     ----------( ON or Warm)-----------------    */
            if(ign1_status > OFF)   // ign1_status is ON or WARM
                //        if(ign_on_flag == TRUE)
            {
                if(manufacturing_TOC == YES)
                {
                    if(tester_present_enable_flag == 1)
                    {
                        tester_present_timer--;

                        if(tester_present_timer < 1)
                        {
                            tester_present_timer = TESTER_PRESENT_SEND_TIME; // reset timer to 3 seconds ..must be sent sooner than every 5 seconds
                            ECU1_XCP_reply_index = 20; // Do nothing index..do nothing with reply message
                            canTransmit(canREG1, XCP_EA3_REQUEST_ID, ( uint8 *) xcp_write_tester_present_msg);
                        } // end if (tester_present_timer < 1)

                    } // end if tester_present_enable_flag = 1
                }

                if((system_msec_clock % 12) == 0)
                {
                    /* periodic transmission of engine_on CAN at 12ms  */

                    if(ign1_status == NORMAL)
                        //              if (Engine_on_Signal == 1)
                    {
                        canTransmit(canREG1, CCP_Engine_Run, ( uint8 *) ccp_write_engine_run);
                    }
                    else    // Put in warm_init
                    {
                        canTransmit(canREG1, CCP_Engine_Run, ( uint8 *) ccp_write_engine_off);
                    }

                }      // end if timer2_msec mods with 12

                /* periodic transmission of Power Mode Run 100ms  */
                if((system_msec_clock % 100) == 0)
                {
                    canTransmit(canREG1, CCP_Power_Mode_ID, ( uint8 *) ccp_write_PowerModeON);
                }      // end if timer2_msec mods with 100

                if((system_msec_clock % 37) == 0)     /* periodic transmission of speed at 100ms  */
                    // CHANGED to 56 now 37 ms because power mode is 100ms also
                {
                    // speed always (default) = zero
                    canTransmit(canREG2, PERIOD_CAN2_ID, ( uint8 *) ccp_write_periodic_can2);
                }   // end if timer2_msec mods with 53 for CAN2 periodic

                /* 9Bxx requires second CAN to send this periodic  */
                if(target_product == TARGET_9BXX)
                {
                    if((system_msec_clock % 53) == 0)     /* periodic transmission of PERIODIC_CAN2, $182, 100ms  */
                        // CHANGED to 53 ms because power mode is 100ms also
                    {
                        // speed always (default) = zero
                        canTransmit(canREG2, PERIOD_CAN2_ID, ( uint8 *) ccp_write_periodic_can2);
                    }   // end if timer2_msec mods with 53 for CAN2 periodic
                }
                /* T1XX requires second CAN to send this periodic  */
                if(target_product == TARGET_T1XX)
                {
                    if((system_msec_clock % 100) == 0)
                        /* Brad McGivern 03AUG17: $182 in SCIR says 8 bytes but DBC says 5 bytes,
                         * Validation is sending $182 on CE Bus at 50ms and 8 bytes.  Should be 100ms and 5 bytes.  Correct?
                         */
                        // until better clarification, message length is still 8 bytes long.  Leonard 04AUG17
                    {
                        // speed always (default) = zero
                        canTransmit(canREG2, PERIOD_CAN2_ID, ( uint8 *) ccp_write_periodic_can2);
                    }   // end if timer2_msec mods with 53 for CAN2 periodic
                }
            }     // end if ign on flag is true

            /* -----------  i g n _ s t a t u s  ==  O F F     ---------------------------    */

            else     // ign1_status is OFF  ----   ign_on_flag = FALSE  or ignition OFF just send power mode OFF message
            {
                if((system_msec_clock % 100) == 0)
                {
                    /* periodic transmission of Power Mode Run 100ms  */
                    canTransmit(canREG1, CCP_Power_Mode_ID, ( uint8 *) ccp_write_PowerModeOFF);
                }      // end if timer2_msec mods with 100

                /* 9Bxx requires second CAN to send this periodic  */
                if(target_product == TARGET_9BXX)
                {
                    if((system_msec_clock % 53) == 0)     /* periodic transmission of PERIODIC_CAN2, $182, 100ms  */
                        // CHANGED to 53 ms because power mode is 100ms also
                    {
                        // speed always (default) = zero
                        canTransmit(canREG2, PERIOD_CAN2_ID, ( uint8 *) ccp_write_periodic_can2);
                    }   // end if timer2_msec mods with 53 for CAN2 periodic
                }

                if((system_msec_clock % 12) == 0)
                {
                    /* periodic transmission of engine_on CAN at 12ms  */

                    canTransmit(canREG1, CCP_Engine_Run, ( uint8 *) ccp_write_engine_off);
                    //   L.C.  cant see any case where ignition is off and we need to send ccp_write_engine_run message, ect.

                    //              CAN1.Obj[5].con1 =  0x5a;       // 01,01,10,01 reset rmtpnd, reset txrqst, set cpuupd, reset newdat
                    //              if (Engine_on_Signal == 1)
                    //              {
                    //                  memcpy(&CAN1.Obj[5].Dat.Byte,&ccp_write_engine_run,8);  // move data in
                    //              }
                    //              else
                    //              {
                    //                  memcpy(&CAN1.Obj[5].Dat.Byte,&ccp_write_engine_off,8);  // puts product in warm init (no current)
                    //              }
                    //
                    //              CAN1.Obj[5].con1 =  0x55;       // 01,01,01,01 reset rmtpnd, reset txrqst, reset cpuupd, reset newdat
                    //              CAN1.Obj[5].con1 =  TxRqst;     // push message

                }      // end if timer2_msec mods with 12

            }   // end if ignition OFF  else statement
        }  // case TARGET_T1XX: case TARGET_C1XX: case TARGET_9BXX:
        break;

        case TARGET_BYD_SA2FL:
        {
            /* -----------  i g n _ s t a t u s  >  O F F     ----------( ON or Warm)-----------------    */
            if(ign1_status > OFF)   // ign1_status is ON or WARM
                //        if(ign_on_flag == TRUE)
            {
                if(manufacturing_TOC == YES)
                {
                    if(tester_present_enable_flag == 1)
                    {
                        tester_present_timer--;

                        if(tester_present_timer < 1)
                        {
                            tester_present_timer = TESTER_PRESENT_SEND_TIME; // reset timer to 3 seconds ..must be sent sooner than every 5 seconds
                            ECU1_XCP_reply_index = 20; // Do nothing index..do nothing with reply message
                            canTransmit(canREG1, XCP_EA3_REQUEST_ID, ( uint8 *) xcp_write_tester_present_msg);
                        } // end if (tester_present_timer < 1)

                    } // end if tester_present_enable_flag = 1
                }

                if((system_msec_clock % 10) == 0)
                {
                    /* periodic transmission of engine_on CAN at 12ms  */

                    if(ign1_status == NORMAL)
                        //              if (Engine_on_Signal == 1)
                    {
                       //canTransmit(canREG1, CCP_Engine_Run, ( uint8 *) ccp_write_engine_run);
                    }
                    else    // Put in warm_init
                    {
                        //canTransmit(canREG1, CCP_Engine_Run, ( uint8 *) ccp_write_engine_off);
                    }

                }      // end if timer2_msec mods with 12

                /* periodic transmission of Vehicle Ready 100ms  */
                if((system_msec_clock % 20) == 0)
                {
                    canTransmit(canREG1, VEHICLE_READY, ( uint8 *) Vehicle_ready);//here
                }      // end if timer2_msec mods with 100

                /* periodic transmission of Power Mode Run 100ms  */
                if((system_msec_clock % 100) == 0)
                {
                    // canTransmit(canREG1, CCP_Power_Mode_ID, ( uint8 *) ccp_write_PowerModeON);
                }      // end if timer2_msec mods with 100

                if((system_msec_clock % 10) == 0)     /* periodic transmission of speed at 100ms  */
                    // CHANGED to 56 now 37 ms because power mode is 100ms also
                {
                    // speed always (default) = zero
                    //canTransmit(canREG1, CCP_SPEED_ID, ( uint8 *) ccp_write_0_to_speed);
                    if(Veh_speed_count > 15)
                    {
                        Veh_speed_count = 0;
                    }
                    canTransmit(canREG1, CCP_SPEED_ID, *Veh_speed_0_table_BYD_SA2FL+Veh_speed_count*8);

                    Veh_speed_count ++;
                }   // end if timer2_msec mods with 56 now 37 for speed
            }     // end if ign on flag is true

            /* -----------  i g n _ s t a t u s  ==  O F F     ---------------------------    */

            else     // ign1_status is OFF  ----   ign_on_flag = FALSE  or ignition OFF just send power mode OFF message
            {
                if((system_msec_clock % 100) == 0)
                {
                    /* periodic transmission of Power Mode Run 100ms  */
                    canTransmit(canREG1, CCP_Power_Mode_ID, ( uint8 *) ccp_write_PowerModeOFF);
                }      // end if timer2_msec mods with 100

                //Added zero vehicle speed periodic message to off state.  Part will not shutdown without this.  G.Wood 16JAN19
                if((system_msec_clock % 10) == 0)     /* periodic transmission of speed at 100ms  */
                                    // CHANGED to 56 now 37 ms because power mode is 100ms also
                {
                    // speed always (default) = zero
                    //canTransmit(canREG1, CCP_SPEED_ID, ( uint8 *) ccp_write_0_to_speed);
                    if(Veh_speed_count > 15)
                    {
                        Veh_speed_count = 0;
                    }
                    canTransmit(canREG1, CCP_SPEED_ID, *Veh_speed_00_table_BYD_SA2FL+Veh_speed_count*8);
                    Veh_speed_count ++;
                }   // end if timer2_msec mods with 56 now 37 for speed

                //Added vehicle ready periodic message to off state.
                if((system_msec_clock % 20) == 0)
                {
                    canTransmit(canREG1, VEHICLE_READY, ( uint8 *) Vehicle_not_ready);//here
                }      // end if timer2_msec mods with 100

                /* periodic transmission of Power Mode Run 100ms  */

                if((system_msec_clock % 12) == 0)
                {
                    /* periodic transmission of engine_on CAN at 12ms  */

                    //canTransmit(canREG1, CCP_Engine_Run, ( uint8 *) ccp_write_engine_off);
                    //   L.C.  cant see any case where ignition is off and we need to send ccp_write_engine_run message, ect.

                    //              CAN1.Obj[5].con1 =  0x5a;       // 01,01,10,01 reset rmtpnd, reset txrqst, set cpuupd, reset newdat
                    //              if (Engine_on_Signal == 1)
                    //              {
                    //                  memcpy(&CAN1.Obj[5].Dat.Byte,&ccp_write_engine_run,8);  // move data in
                    //              }
                    //              else
                    //              {
                    //                  memcpy(&CAN1.Obj[5].Dat.Byte,&ccp_write_engine_off,8);  // puts product in warm init (no current)
                    //              }
                    //
                    //              CAN1.Obj[5].con1 =  0x55;       // 01,01,01,01 reset rmtpnd, reset txrqst, reset cpuupd, reset newdat
                    //              CAN1.Obj[5].con1 =  TxRqst;     // push message

                }      // end if timer2_msec mods with 12

            }   // end if ignition OFF  else statement
        }  // case BYD_SA2FL:
        break;

// GWM_A0607
        case TARGET_GWM_A0607:
        {
            /* -----------  i g n _ s t a t u s  >  O F F     ----------( ON or Warm)-----------------    */
            if(ign1_status > OFF)   // ign1_status is ON or WARM
                //        if(ign_on_flag == TRUE)
            {
                if(manufacturing_TOC == YES)
                {
                    if(tester_present_enable_flag == 1)
                    {
                        tester_present_timer--;

                        if(tester_present_timer < 1)
                        {
                            tester_present_timer = TESTER_PRESENT_SEND_TIME; // reset timer to 3 seconds ..must be sent sooner than every 5 seconds
                            ECU1_XCP_reply_index = 20; // Do nothing index..do nothing with reply message
                            canTransmit(canREG1, XCP_EA3_REQUEST_ID, ( uint8 *) xcp_write_tester_present_msg);
                        } // end if (tester_present_timer < 1)

                    } // end if tester_present_enable_flag = 1
                }

                if((system_msec_clock % 20) == 0)
                {
                    /* periodic transmission of engine_on CAN at 20ms  */

                    if(ign1_status == NORMAL)
                        //              if (Engine_on_Signal == 1)
                    {
                       //canTransmit(canREG1, CCP_Engine_Run, ( uint8 *) ccp_write_engine_run);
                        
                        ccp_GWM_A0607_write_EngState[1] = 0x00; 
                        ccp_GWM_A0607_write_EngState[2] = 0x00; 
                        ccp_GWM_A0607_write_EngState[3] = 0x00; 
                        ccp_GWM_A0607_write_EngState[4] = 0x00; 
                        ccp_GWM_A0607_write_EngState[5] = 0x00; 
                        ccp_GWM_A0607_write_EngState[6] = 0x00; 
                        ccp_GWM_A0607_write_EngState[7] = (GWM_A0607_EngState_CRC_counter + 0x40) ; // // set EngState (Bit 62) to 0x2:Running

                        //CRC calculation & 9 bytes --[DataID_(High byte) + DataID_(Low byte) + 7 Bytes (data_byte1~ data_byte 7)]// 
                        //DataID: High_byte: 0x00 Low_byte:0x1E// 
                        unsigned char E2E_EngState_data_byte_array[] = {0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, ccp_GWM_A0607_write_EngState[7]};      

                        unsigned char E2E_EngState_CRC;
                        E2E_EngState_CRC = E2E_CRC_Calculation(E2E_EngState_data_byte_array);

                        ccp_GWM_A0607_write_EngState[0] = E2E_EngState_CRC;
                        canTransmit(canREG1, CCP_Engine_Run, ( uint8 *) ccp_GWM_A0607_write_EngState);

                        GWM_A0607_EngState_CRC_counter ++; 
                     if(GWM_A0607_EngState_CRC_counter > 15)
                        {
                          GWM_A0607_EngState_CRC_counter = 0;
                        }
                        
                    }
                    else    // Put in warm_init
                    {
                        //canTransmit(canREG1, CCP_Engine_Run, ( uint8 *) ccp_write_engine_off);
                    
                        ccp_GWM_A0607_write_EngState[1] = 0x00; 
                        ccp_GWM_A0607_write_EngState[2] = 0x00; 
                        ccp_GWM_A0607_write_EngState[3] = 0x00; 
                        ccp_GWM_A0607_write_EngState[4] = 0x00; 
                        ccp_GWM_A0607_write_EngState[5] = 0x00; 
                        ccp_GWM_A0607_write_EngState[6] = 0x00; 
                        ccp_GWM_A0607_write_EngState[7] = GWM_A0607_EngState_CRC_counter ; // // set EngState (Bit 62) to 0x0:Stopped

                        //CRC calculation & 9 bytes --[DataID_(High byte) + DataID_(Low byte) + 7 Bytes (data_byte1~ data_byte 7)]// 
                        //DataID: High_byte: 0x00 Low_byte:0x1E// 
                        unsigned char E2E_EngState_data_byte_array[] = {0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, ccp_GWM_A0607_write_EngState[7]};      

                        unsigned char E2E_EngState_CRC;
                        E2E_EngState_CRC = E2E_CRC_Calculation(E2E_EngState_data_byte_array);

                        ccp_GWM_A0607_write_EngState[0] = E2E_EngState_CRC;
                        canTransmit(canREG1, CCP_Engine_Run, ( uint8 *) ccp_GWM_A0607_write_EngState);

                        GWM_A0607_EngState_CRC_counter ++; 
                     if(GWM_A0607_EngState_CRC_counter > 15)
                        {
                          GWM_A0607_EngState_CRC_counter = 0;
                        }
                    
                    }

                }      // end if timer2_msec mods with 12

               if((system_msec_clock % 20) == 0)     //periodic transmission of Vehspd at 20ms  //
                    // VehSpd -- ABS3 (MSG ID $265)(CAN)
                {
                    // speed always (default) = zero

                    ccp_GWM_A0607_write_Vehspd[1] = 0x20;       // set VehSpdVld (Bit13) to 0x01: Valid
                    ccp_GWM_A0607_write_Vehspd[2] = 0x00;
                    ccp_GWM_A0607_write_Vehspd[3] = 0x00;
                    ccp_GWM_A0607_write_Vehspd[4] = 0x00;
                    ccp_GWM_A0607_write_Vehspd[5] = 0x00;
                    ccp_GWM_A0607_write_Vehspd[6] = 0x00;
                    ccp_GWM_A0607_write_Vehspd[7] = GWM_A0607_Vehspd_CRC_counter;   

                    /*CRC calculation & 9 bytes --[DataID_(High byte) + DataID_(Low byte) + 7 Bytes (data_byte1~ data_byte 7)]*/  
                    /*DataID: High_byte: 0x00 Low_byte:0x28*/ 
                    unsigned char E2E_Vehspd_data_byte_array[] = { 0x28, 0x00, ccp_GWM_A0607_write_Vehspd[1], 0x00, 0x00, 0x00, 0x00, 0x00, ccp_GWM_A0607_write_Vehspd[7]};  
                        //   E2E_data_byte_array[0] = 0x28;   //DataID: Low_byte:0x28
                        //   E2E_data_byte_array[1] = 0x00;   //DataID: High_byte: 0x00 
                        //   E2E_data_byte_array[2] = ccp_GWM_A0607_write_Vehspd[1];
                        //   E2E_data_byte_array[3] = ccp_GWM_A0607_write_Vehspd[2];
                        //   E2E_data_byte_array[4] = ccp_GWM_A0607_write_Vehspd[3];
                        //   E2E_data_byte_array[5] = ccp_GWM_A0607_write_Vehspd[4];
                        //   E2E_data_byte_array[6] = ccp_GWM_A0607_write_Vehspd[5];
                        //   E2E_data_byte_array[7] = ccp_GWM_A0607_write_Vehspd[6];
                        //   E2E_data_byte_array[8] = ccp_GWM_A0607_write_Vehspd[7];
                
                    unsigned char E2E_Vehspd_CRC;
                    E2E_Vehspd_CRC = E2E_CRC_Calculation(E2E_Vehspd_data_byte_array);

                    ccp_GWM_A0607_write_Vehspd[0] = E2E_Vehspd_CRC;

                    canTransmit(canREG1, GWM_A0607_CCP_Vehspd_ID, ( uint8 *) ccp_GWM_A0607_write_Vehspd);

                    GWM_A0607_Vehspd_CRC_counter ++; 
                    if(GWM_A0607_Vehspd_CRC_counter > 15)
                    {
                        GWM_A0607_Vehspd_CRC_counter = 0;
                    }
                }   // end if timer2_msec mods with 56 now 37 for speed

                /* periodic transmission of EngSpd = 0   */
                if((system_msec_clock % 20) == 0)
                {
                    // ECM1 (MSG ID $111)(CAN)
                    canTransmit(canREG1, GWM_A0607_CCP_Endspd_ID, ( uint8 *) ccp_GWM_A0607_write_EngSpd);
                }      // end if timer2_msec mods with 20

                /* periodic transmission of Wss 20ms  */
                if((system_msec_clock % 20) == 0)
                {
                    //ABS1 (MSG ID $231) (CAN)
                    
                    ccp_GWM_A0607_write_Wss[1] = 0x00; 
                    ccp_GWM_A0607_write_Wss[2] = 0x00; 
                    ccp_GWM_A0607_write_Wss[3] = 0x00; 
                    ccp_GWM_A0607_write_Wss[4] = 0x00; 
                    ccp_GWM_A0607_write_Wss[5] = 0x00; 
                    ccp_GWM_A0607_write_Wss[6] = 0x00; 
                    ccp_GWM_A0607_write_Wss[7] = (GWM_A0607_Wss_CRC_counter + 0xC0); //set WssEdgesSumVld(Bit62 & 63) to 0x1: Valid

                    //CRC calculation & 9 bytes --[DataID_(High byte) + DataID_(Low byte) + 7 Bytes (data_byte1~ data_byte 7)]// 
                    //DataID: High_byte: 0x00 Low_byte:0x26
                    unsigned char E2E_Wss_data_byte_array[] = {0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, ccp_GWM_A0607_write_Wss[7]};      

                    unsigned char E2E_Wss_CRC;
                    E2E_Wss_CRC = E2E_CRC_Calculation(E2E_Wss_data_byte_array);

                    ccp_GWM_A0607_write_Wss[0] = E2E_Wss_CRC;

                    canTransmit(canREG1, GWM_A0607_CCP_Wss_ID, ( uint8 *) ccp_GWM_A0607_write_Wss);

                    GWM_A0607_Wss_CRC_counter ++; 
                    if(GWM_A0607_Wss_CRC_counter > 15)
                      {
                        GWM_A0607_Wss_CRC_counter = 0;
                      }
                    
                }      // end if timer2_msec mods with 20

                /* periodic transmission of SteerWheelAng with 10ms  */
                if((system_msec_clock % 10) == 0)
                {
                    // CSA2 (Msg ID $0A1)(CAN/CANFD)
                    
                    ccp_GWM_A0607_write_SteerWheelAng[1] = 0x00; 
                    ccp_GWM_A0607_write_SteerWheelAng[2] = 0x00; 
                    ccp_GWM_A0607_write_SteerWheelAng[3] = 0x00; 
                    ccp_GWM_A0607_write_SteerWheelAng[4] = 0x00; 
                    ccp_GWM_A0607_write_SteerWheelAng[5] = 0x00; 
                    ccp_GWM_A0607_write_SteerWheelAng[6] = 0x00; 
                    ccp_GWM_A0607_write_SteerWheelAng[7] = GWM_A0607_SteerWheelAng_CRC_counter; //set SteerWheelAng = 0 and SAS_Sts = 0x0:SAS Angle And Speed Correct

                    //CRC calculation & 9 bytes --[DataID_(High byte) + DataID_(Low byte) + 7 Bytes (data_byte1~ data_byte 7)]// 
                    //DataID: High_byte: 0x00 Low_byte:0x38
                    unsigned char E2E_SteerWheelAng_data_byte_array[] = {0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, ccp_GWM_A0607_write_SteerWheelAng[7]};      

                    unsigned char E2E_SteerWheelAng_CRC;
                    E2E_SteerWheelAng_CRC = E2E_CRC_Calculation(E2E_SteerWheelAng_data_byte_array);

                    ccp_GWM_A0607_write_SteerWheelAng[0] = E2E_SteerWheelAng_CRC;

                    canTransmit(canREG1, GWM_A0607_CCP_SteerWheelAng_ID, ( uint8 *) ccp_GWM_A0607_write_SteerWheelAng);

                    GWM_A0607_SteerWheelAng_CRC_counter ++; 
                    if(GWM_A0607_SteerWheelAng_CRC_counter > 15)
                      {
                        GWM_A0607_SteerWheelAng_CRC_counter = 0;
                      }

                }      // end if timer2_msec mods with 10

                /* periodic transmission of SteerWheelAng with 20ms  */
                if((system_msec_clock % 20) == 0)
                {
                   //  ABM2 (Msg ID $245) (CAN/CANFD)
                    
                    ccp_GWM_A0607_write_VehYawRate[1] = 0x00; 
                    ccp_GWM_A0607_write_VehYawRate[2] = 0x00; 
                    ccp_GWM_A0607_write_VehYawRate[3] = 0x00; 
                    ccp_GWM_A0607_write_VehYawRate[4] = 0x00; 
                    ccp_GWM_A0607_write_VehYawRate[5] = 0x00; 
                    ccp_GWM_A0607_write_VehYawRate[6] = 0x00; 
                    ccp_GWM_A0607_write_VehYawRate[7] = (GWM_A0607_VehYawRate_CRC_counter + 0x20); //set VehDynYawRateVld(Bit61) to 0x1: Valid

                    //CRC calculation & 9 bytes --[DataID_(High byte) + DataID_(Low byte) + 7 Bytes (data_byte1~ data_byte 7)]// 
                    //DataID: High_byte: 0x00 Low_byte:0x3B
                    unsigned char E2E_VehYawRate_data_byte_array[] = {0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, ccp_GWM_A0607_write_VehYawRate[7]};      

                    unsigned char E2E_VehYawRate_CRC;
                    E2E_VehYawRate_CRC = E2E_CRC_Calculation(E2E_VehYawRate_data_byte_array);

                    ccp_GWM_A0607_write_VehYawRate[0] = E2E_VehYawRate_CRC;

                    canTransmit(canREG1, GWM_A0607_CCP_VehYawRate_ID, ( uint8 *) ccp_GWM_A0607_write_VehYawRate);

                    GWM_A0607_VehYawRate_CRC_counter ++; 
                    if(GWM_A0607_VehYawRate_CRC_counter > 15)
                      {
                        GWM_A0607_VehYawRate_CRC_counter = 0;
                      }

                }      // end if timer2_msec mods with 20

                /* periodic transmission of MaxEngTrqNorm with 20ms  */
                if((system_msec_clock % 100) == 0)
                {
                     //  ECM3 (MSG ID $371) (CAN)
                    
                    ccp_GWM_A0607_write_MaxEngTrqNorm[1] = 0x00; 
                    ccp_GWM_A0607_write_MaxEngTrqNorm[2] = 0x00; 
                    ccp_GWM_A0607_write_MaxEngTrqNorm[3] = 0x00; 
                    ccp_GWM_A0607_write_MaxEngTrqNorm[4] = 0x00; 
                    ccp_GWM_A0607_write_MaxEngTrqNorm[5] = 0x00; 
                    ccp_GWM_A0607_write_MaxEngTrqNorm[6] = 0x00; 
                    ccp_GWM_A0607_write_MaxEngTrqNorm[7] = GWM_A0607_MaxEngTrqNorm_CRC_counter; //set MaxEngTrqNorm = 0

                    //CRC calculation & 9 bytes --[DataID_(High byte) + DataID_(Low byte) + 7 Bytes (data_byte1~ data_byte 7)]// 
                    //DataID: High_byte: 0x00 Low_byte:0x1F
                    unsigned char E2E_MaxEngTrqNorm_data_byte_array[] = {0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, ccp_GWM_A0607_write_MaxEngTrqNorm[7]};      

                    unsigned char E2E_MaxEngTrqNorm_CRC;
                    E2E_MaxEngTrqNorm_CRC = E2E_CRC_Calculation(E2E_MaxEngTrqNorm_data_byte_array);

                    ccp_GWM_A0607_write_MaxEngTrqNorm[0] = E2E_MaxEngTrqNorm_CRC;

                    canTransmit(canREG1, GWM_A0607_CCP_MaxEngTrqNorm_ID, ( uint8 *) ccp_GWM_A0607_write_MaxEngTrqNorm);

                    GWM_A0607_MaxEngTrqNorm_CRC_counter ++; 
                    if(GWM_A0607_MaxEngTrqNorm_CRC_counter > 15)
                      {
                        GWM_A0607_MaxEngTrqNorm_CRC_counter = 0;
                      }
                      
                }      // end if timer2_msec mods with 100

                /* periodic transmission of NetEngTrq with 10ms  */
                if((system_msec_clock % 10) == 0)
                {
                    // ECM1 (MSG ID $111)(CAN)
                    canTransmit(canREG1, GWM_A0607_CCP_NetEngTrq_ID, ( uint8 *) ccp_GWM_A0607_write_NetEngTrq);
                }      // end if timer2_msec mods with 100

                if((system_msec_clock % 10) == 0)
                {
                    // canTransmit(canREG1, CCP_Power_Mode_ID, ( uint8 *) ccp_write_PowerModeON);
                }      // end if timer2_msec mods with 100

            }     // end if ign on flag is true

            /* -----------  i g n _ s t a t u s  ==  O F F     ---------------------------    */

            else     // ign1_status is OFF  ----   ign_on_flag = FALSE  or ignition OFF just send power mode OFF message
            {
                if((system_msec_clock % 100) == 0)
                {
                    /* periodic transmission of Power Mode Run 100ms  */
                //    canTransmit(canREG1, CCP_Power_Mode_ID, ( uint8 *) ccp_write_PowerModeOFF);
                 /*
                 ccp_GWM_A0607_write_EngState[7] = GWM_A0607_EngState_CRC_counter ; // // set EngState (Bit 62) to 0x0:Stopped

                        //CRC calculation & 9 bytes --[DataID_(High byte) + DataID_(Low byte) + 7 Bytes (data_byte1~ data_byte 7)]// 
                        //DataID: High_byte: 0x00 Low_byte:0x1E// 
                        unsigned char E2E_EngState_data_byte_array[] = {0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, ccp_GWM_A0607_write_EngState[7]};      

                        unsigned char E2E_EngState_CRC;
                        E2E_EngState_CRC = E2E_CRC_Calculation(E2E_EngState_data_byte_array);

                        ccp_GWM_A0607_write_EngState[0] = E2E_EngState_CRC;
                        canTransmit(canREG1, CCP_Engine_Run, ( uint8 *) ccp_GWM_A0607_write_EngState);

                        GWM_A0607_EngState_CRC_counter ++; 
                     if(GWM_A0607_EngState_CRC_counter > 15)
                        {
                          GWM_A0607_EngState_CRC_counter = 0;
                        }
                        */
            
                }      // end if timer2_msec mods with 100

                //Added zero vehicle speed periodic message to off state.  Part will not shutdown without this.  G.Wood 16JAN19
                if((system_msec_clock % 10) == 0)     /* periodic transmission of speed at 100ms  */
                                    // CHANGED to 56 now 37 ms because power mode is 100ms also
                {
                    // speed always (default) = zero
                    //canTransmit(canREG1, CCP_SPEED_ID, ( uint8 *) ccp_write_0_to_speed);
                    if(Veh_speed_count > 15)
                    {
                        Veh_speed_count = 0;
                    }
                //    canTransmit(canREG1, CCP_SPEED_ID, *Veh_speed_00_table_BYD_SA2FL+Veh_speed_count*8);
                    Veh_speed_count ++;
                }   // end if timer2_msec mods with 56 now 37 for speed

                //Added vehicle ready periodic message to off state.
                if((system_msec_clock % 20) == 0)
                {
                 //  canTransmit(canREG1, VEHICLE_READY, ( uint8 *) Vehicle_not_ready);//here
                }      // end if timer2_msec mods with 100

                /* periodic transmission of Power Mode Run 100ms  */

                if((system_msec_clock % 12) == 0)
                {
                    /* periodic transmission of engine_on CAN at 12ms  */

                    //canTransmit(canREG1, CCP_Engine_Run, ( uint8 *) ccp_write_engine_off);
                    //   L.C.  cant see any case where ignition is off and we need to send ccp_write_engine_run message, ect.

                    //              CAN1.Obj[5].con1 =  0x5a;       // 01,01,10,01 reset rmtpnd, reset txrqst, set cpuupd, reset newdat
                    //              if (Engine_on_Signal == 1)
                    //              {
                    //                  memcpy(&CAN1.Obj[5].Dat.Byte,&ccp_write_engine_run,8);  // move data in
                    //              }
                    //              else
                    //              {
                    //                  memcpy(&CAN1.Obj[5].Dat.Byte,&ccp_write_engine_off,8);  // puts product in warm init (no current)
                    //              }
                    //
                    //              CAN1.Obj[5].con1 =  0x55;       // 01,01,01,01 reset rmtpnd, reset txrqst, reset cpuupd, reset newdat
                    //              CAN1.Obj[5].con1 =  TxRqst;     // push message

                }      // end if timer2_msec mods with 12

            }   // end if ignition OFF  else statement
        }  // case GWM_A0607:
        break;

        case TARGET_SGMW_CN200:
        {
            /* -----------  i g n _ s t a t u s  >  O F F     ----------( ON or Warm)-----------------    */
            if(ign1_status > OFF)   // ign1_status is ON or WARM
                //        if(ign_on_flag == TRUE)
            {
                if(manufacturing_TOC == YES)
                {
                    if(tester_present_enable_flag == 1)
                    {
                        tester_present_timer--;

                        if(tester_present_timer < 1)
                        {
                            tester_present_timer = TESTER_PRESENT_SEND_TIME; // reset timer to 3 seconds ..must be sent sooner than every 5 seconds
                            ECU1_XCP_reply_index = 20; // Do nothing index..do nothing with reply message
                            canTransmit(canREG1, CN200_ECU1_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_write_tester_present_msg);
                        } // end if (tester_present_timer < 1)

                    } // end if tester_present_enable_flag = 1
                }

                if((system_msec_clock % 10) == 0)
                {
                    /* periodic transmission of engine_on CAN at 12ms  */

                    if(ign1_status == NORMAL)
                        //              if (Engine_on_Signal == 1)
                    {
                        canTransmit(canREG1, CN200_ECU1_CAN1_CCP_Engine_Run, ( uint8 *) ccp_write_engine_run);
                    }
                    else    // Put in warm_init
                    {
                        canTransmit(canREG1, CN200_ECU1_CAN1_CCP_Engine_Run, ( uint8 *) ccp_write_engine_off);
                    }

                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_EA4_34A_ID, ( uint8 *) ccp_write_0_to_speed);
                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_EA4_348_ID, ( uint8 *) ccp_write_0_to_speed);
                }      // end if timer2_msec mods with 10

                /* periodic transmission of Power Mode Run 100ms  */
                if((system_msec_clock % 100) == 0)
                {
                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_Power_Mode_ID, ( uint8 *) ccp_write_PowerModeON);
                }      // end if timer2_msec mods with 100

                if((system_msec_clock % 37) == 0)     /* periodic transmission of speed at 100ms  */
                    // CHANGED to 56 now 37 ms because power mode is 100ms also
                {
                    // speed always (default) = zero
                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_SPEED_ID, ( uint8 *) ccp_write_0_to_speed);
                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_EA4_AvgVehSpd_ID, ( uint8 *) ccp_write_0_to_speed);
                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_EA4_3E5_ID, ( uint8 *) ccp_write_0_to_speed);
                }   // end if timer2_msec mods with 56 now 37 for speed

            }     // end if ign on flag is true

            /* -----------  i g n _ s t a t u s  ==  O F F     ---------------------------    */

            else     // ign1_status is OFF  ----   ign_on_flag = FALSE  or ignition OFF just send power mode OFF message
            {
                if(p_off_continue_periodics_timer > 0)
                {
                    p_off_continue_periodics_timer--;
                    if((system_msec_clock % 100) == 0)
                    {
                        /* periodic transmission of Power Mode Run 100ms  */
                        canTransmit(canREG1, CN200_ECU1_CAN1_CCP_Power_Mode_ID, ( uint8 *) ccp_write_PowerModeOFF);
                    }      // end if timer2_msec mods with 100
                    if((system_msec_clock % 12) == 0)
                    {
                        /* periodic transmission of engine_on CAN at 12ms  */

                        canTransmit(canREG1, CN200_ECU1_CAN1_CCP_Engine_Run, ( uint8 *) ccp_write_engine_off);
                        //   L.C.  cant see any case where ignition is off and we need to send ccp_write_engine_run message, ect.

                    }      // end if timer2_msec mods with 12
                    if((system_msec_clock % 37) == 0)     /* periodic transmission of speed at 100ms  */
                        // CHANGED to 56 now 37 ms because power mode is 100ms also
                    {
                        // speed always (default) = zero
                        canTransmit(canREG1, CN200_ECU1_CAN1_CCP_SPEED_ID, ( uint8 *) ccp_write_0_to_speed);
                        canTransmit(canREG1, CN200_ECU1_CAN1_CCP_EA4_AvgVehSpd_ID, ( uint8 *) ccp_write_0_to_speed);
                        canTransmit(canREG1, CN200_ECU1_CAN1_CCP_EA4_3E5_ID, ( uint8 *) ccp_write_0_to_speed);
                    }   // end if timer2_msec mods with 56 now 37 for speed
                } //end if(periodic_continue_timer > 0)

            }   // end if ignition OFF  else statement
        }  // case TARGET_SGMW_CN200
        break;

        case TARGET_FCA_ADAS:
        {
            /* -----------  i g n _ s t a t u s  >  O F F     ----------( ON or Warm)-----------------    */

            if(ign1_status > OFF)   // ign1_status is ON or WARM
                //        if(ign_on_flag == TRUE)
            {

                if (system_msec_clock % 20 == 0)
                {       //Send vehicle speed

                    if(Veh_speed_count > 15)
                    {
                        Veh_speed_count = 0;
                    }
                    canTransmit(canREG1, FCA_CCP_SPEED_ID, *Veh_speed_0_table+Veh_speed_count*8);
                    canTransmit(canREG2, FCA_CCP_SPEED_ID, *Veh_speed_0_table+Veh_speed_count*8);

                    Veh_speed_count ++;

                }
                if (system_msec_clock % 10 == 0)
                {           //Send Engine Status
                    canTransmit(canREG1, FCA_CCP_Engine_Run, (uint8 *) ccp_write_engine_run);
                    canTransmit(canREG2, FCA_CCP_Engine_Run, (uint8 *) ccp_write_engine_run);

                }

            }     // end if ign on flag is true

            /* -----------  i g n _ s t a t u s  ==  O F F     ---------------------------    */

            else     // ign1_status is OFF  ----   ign_on_flag = FALSE  or ignition OFF just send power mode OFF message
            {

                // As far as i can tell no action is required during p_off for FCA ADAS

            }   // end if ignition OFF  else statement
        }  // case TARGET_FCA_ADAS:
        break; // case TARGET_FCA_ADAS:

        case TARGET_CD391_ADAS:
        {
            /* -----------  i g n _ s t a t u s  >  O F F     ----------( ON or Warm)-----------------    */

            if(ign1_status > OFF)   // ign1_status is ON or WARM
            {

                if (system_msec_clock % 20 == 0)
                {       //Send vehicle speed
                    if(Veh_speed_count > 15)
                    {
                        Veh_speed_count = 0;
                    }

                    canTransmit(canREG1, FORD_CCP_SPEED_ID, (uint8 *) Veh_speed_0_table_Ford[Veh_speed_count]);
                    canTransmit(canREG2, FORD_CCP_SPEED_ID, (uint8 *) ccp_FORD_write_0_to_speed);
                    Veh_speed_count ++;
                }
                if (system_msec_clock % 10 == 0)
                {
                    if(ign1_status == NORMAL)
                    {
                        canTransmit(canREG1, FORD_CCP_Engine_Run, (uint8 *) ccp_FORD_engine_run);
                        canTransmit(canREG2, FORD_CCP_Engine_Run_2, (uint8 *) ccp_FORD_engine_run_2);
                    }
                    else    // Put in warm_init
                    {
                        canTransmit(canREG1, FORD_CCP_Engine_Run, (uint8 *) ccp_FORD_engine_off);
                        canTransmit(canREG2, FORD_CCP_Engine_Run_2, (uint8 *) ccp_FORD_engine_off);
                    }

                }

            }     // end if ign on flag is true

            /* -----------  i g n _ s t a t u s  ==  O F F     ---------------------------    */

            else     // ign1_status is OFF  ----   ign_on_flag = FALSE  or ignition OFF just send power mode OFF message
            {

                if (system_msec_clock % 20 == 0)
                {       //Send vehicle speed

                    if(Veh_speed_count > 15)
                    {
                        Veh_speed_count = 0;
                    }
                    canTransmit(canREG1, FORD_CCP_SPEED_ID, (uint8 *) Veh_speed_0_table_Ford[Veh_speed_count]);
                    canTransmit(canREG2, FORD_CCP_SPEED_ID, (uint8 *) ccp_FORD_write_0_to_speed);
                    Veh_speed_count ++;
                }
                if (system_msec_clock % 10 == 0)
                {
                    canTransmit(canREG1, FORD_CCP_Engine_Run, (uint8 *) ccp_FORD_engine_off);
                    canTransmit(canREG2, FORD_CCP_Engine_Run_2, (uint8 *) ccp_FORD_engine_off);
                }

            }   // end if ignition OFF  else statement
        }  // case TARGET_CD391_ADAS:
        break;  // case TARGET_CD391_ADAS:

        case TARGET_PSA_CMP:
        {
            /* -----------  i g n _ s t a t u s  >  O F F     ----------( ON or Warm)-----------------    */
            if(ign1_status > OFF)   // ign1_status is ON or WARM
                //        if(ign_on_flag == TRUE)
            {
                if(manufacturing_TOC == YES)
                {
                    if(tester_present_enable_flag == 1)
                    {
                        tester_present_timer--;

                        if(tester_present_timer < 1)
                        {
                            tester_present_timer = TESTER_PRESENT_SEND_TIME; // reset timer to 3 seconds ..must be sent sooner than every 5 seconds
                            ECU1_XCP_reply_index = 20; // Do nothing index..do nothing with reply message
                            canTransmit(canREG1, PSA_XCP_EA3_REQUEST_ID, ( uint8 *) xcp_write_tester_present_msg);
                        } // end if (tester_present_timer < 1)

                    } // end if tester_present_enable_flag = 1
                }

                if((system_msec_clock % 20) == 0)
                {
                    /* periodic transmission of engine_on CAN at 12ms  */

                    if(ign1_status == NORMAL)
                        //              if (Engine_on_Signal == 1)
                    {
                        canTransmit(canREG1, PSA_CCP_Engine_Run, ( uint8 *) ccp_PSA_CMP_write_engine_run);
                    }
                    else    // Put in warm_init
                    {
                        canTransmit(canREG1, PSA_CCP_Engine_Run, ( uint8 *) ccp_PSA_CMP_write_engine_locked);
                    }

                }      // end if timer2_msec mods with 12

                if((system_msec_clock % 40) == 0)     /* periodic transmission of speed at 40ms  */

                {
                    // speed always (default) = zero
                    if(Veh_speed_count > 15)
                    {
                        Veh_speed_count = 0;
                    }
                    canTransmit(canREG1, PSA_CCP_SPEED_ID, *Veh_speed_0_table_PSA+Veh_speed_count*8);
                    Veh_speed_count ++;
                }   // end if timer2_msec mods with 40 for speed


            }     // end if ign on flag is true

            /* -----------  i g n _ s t a t u s  ==  O F F     ---------------------------    */

            else     // ign1_status is OFF  ----   ign_on_flag = FALSE  Phil wants power mode OFF and speed at zero message
            {
                if((system_msec_clock % 20) == 0)
                {
                    /* periodic transmission of engine_on CAN at 12ms  */

                    canTransmit(canREG1, PSA_CCP_Engine_Run, ( uint8 *) ccp_PSA_CMP_write_engine_locked);

                }      // end if timer2_msec mods with 12

                if((system_msec_clock % 40) == 0)     /* periodic transmission of speed at 40ms  */

                {
                    // speed always (default) = zero
                    if(Veh_speed_count > 15)
                    {
                        Veh_speed_count = 0;
                    }
                    canTransmit(canREG1, PSA_CCP_SPEED_ID, *Veh_speed_0_table_PSA+Veh_speed_count*8);
                    Veh_speed_count ++;
                }   // end if timer2_msec mods with 40 for speed


            }   // end if ignition OFF  else statement
        }  // case TARGET_PSA_CMP
        break;

        case TARGET_RENAULT_NISSAN:
        {
            /* -----------  i g n _ s t a t u s  >  O F F     ----------( ON or Warm)-----------------    */
            if(ign1_status > 0)   // ign1_status is ON or WARM
                //        if(ign_on_flag == TRUE)
            {
                if(manufacturing_TOC == YES)
                {
                    if(tester_present_enable_flag == 1)
                    {
                        tester_present_timer--;

                        if(tester_present_timer < 1)
                        {
                            tester_present_timer = TESTER_PRESENT_SEND_TIME; // reset timer to 3 seconds ..must be sent sooner than every 5 seconds
                            ECU1_XCP_reply_index = 20; // Do nothing index..do nothing with reply message
                            canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_write_tester_present_msg);
                        } // end if (tester_present_timer < 1)

                    } // end if tester_present_enable_flag = 1
                }

                if((system_msec_clock % 10) == 0)
                {
                    /* periodic transmission of engine_on CAN at 10ms  */

                    if(ign1_status == NORMAL)
                        //              if (Engine_on_Signal == 1)
                    {
                        canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_Engine_Run, ( uint8 *) ccp_RENAULT_NISSAN_engine_run);
                    }
                    else    // Put in warm_init
                    {
                        canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_Engine_Run, ( uint8 *) ccp_RENAULT_NISSAN_engine_off);
                    }

                }      // end if timer2_msec mods with 12

                /* periodic transmission of Power Mode Run 100ms  */
                if((system_msec_clock % 20) == 0)   // NO powermode message for RENAULT NISSAN
                {
                    //  canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_Power_Mode_ID, ( uint8 *) ccp_write_PowerModeON);
                    //  canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_SPEED_ID, ( uint8 *) ccp_RENAULT_NISSAN_write_0_to_speed);
                }      // end if timer2_msec mods with 100

                /* periodic transmission of Generic App Diag Enable 100ms  */
                if((system_msec_clock % 100) == 0)   // NO powermode message for RENAULT NISSAN
                {
                    canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_GenericAppDiagEnble_ID, ( uint8 *) ccp_RENAULT_NISSAN_GenericAppDiagEnble);
                }      // end if timer2_msec mods with 100



                if((system_msec_clock % 20) == 0)     /* periodic transmission of speed at 20ms  */

                {
                    RENAULT_NISSAN_CRC_counter ++;  //increment message counter
                    if(RENAULT_NISSAN_CRC_counter > 15)
                    {
                        RENAULT_NISSAN_CRC_counter = 0;  // reset counter 0-F or 0 - 15
                    }

                    RENAULT_NISSAN_write_0_to_speed[6] = RENAULT_NISSAN_CRC_counter;

                    RENAULT_NISSAN_CRC_ChkSum = ~(RENAULT_NISSAN_write_0_to_speed[0] + RENAULT_NISSAN_write_0_to_speed[1] +\
                            RENAULT_NISSAN_write_0_to_speed[2] + RENAULT_NISSAN_write_0_to_speed[3] +\
                            RENAULT_NISSAN_write_0_to_speed[4] + RENAULT_NISSAN_write_0_to_speed[5] +\
                            RENAULT_NISSAN_write_0_to_speed[6]);
                    // RENAULT_NISSAN_CRC_ChkSum = ((ccp_RENAULT_NISSAN_write_0_to_speed[0] + ccp_RENAULT_NISSAN_write_0_to_speed[1] +\
                    //ccp_RENAULT_NISSAN_write_0_to_speed[2] + ccp_RENAULT_NISSAN_write_0_to_speed[3] +\
                    //ccp_RENAULT_NISSAN_write_0_to_speed[4] +ccp_RENAULT_NISSAN_write_0_to_speed[5] +\
                    //ccp_RENAULT_NISSAN_write_0_to_speed[6])%256);
                    //ccp_RENAULT_NISSAN_write_0_to_speed[7] = RENAULT_NISSAN_CRC_ChkSum;

                    // speed always (default) = zero
                    canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_SPEED_ID, ( uint8 *) RENAULT_NISSAN_write_0_to_speed);
                }   // end if timer2_msec mods with 56 now 37 for speed

            }     // end if ign on flag is true

            /* -----------  i g n _ s t a t u s  ==  O F F     ---------------------------    */

            else     // ign1_status is OFF  ----   ign_on_flag = FALSE  or ignition OFF just send power mode OFF message
            {
                if((system_msec_clock % 100) == 0)    // NO powermode message for RENAULT NISSAN
                {
                    /* periodic transmission of Power Mode Run 100ms  */
                    //  canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_Power_Mode_ID, ( uint8 *) ccp_write_PowerModeOFF);
                }      // end if timer2_msec mods with 100


                if((system_msec_clock % 12) == 0)
                {
                    /* periodic transmission of engine_on CAN at 12ms  */

                    canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_Engine_Run, ( uint8 *) ccp_RENAULT_NISSAN_engine_off);
                    //   L.C.  cant see any case where ignition is off and we need to send ccp_write_engine_run message, ect.

                }      // end if timer2_msec mods with 12

            }   // end if ignition OFF  else statement
        }  // case TARGET_RENAULT_NISSAN
        break;


        }  // switch(target_product) for ign1_status > OFF and ign1_status = OFF

        /*  ------------  if requested, write AtoD values to RS485  ----------  S T A R T --------------------*/
        if((system_msec_clock % 201 == 0))
        {
            if(AtoD_print_count > 0)
            {
                if(ad_2_5_vref_flag== TRUE) /* (print in mVolts) else in AtoD ticks */
                {
                    sprintf(return_message,"ref=%d 0=%d 1=%d 2=%d 3=%d 4=%d 5=%d 6=%d 7=%d 8=%d 9=%d 10=%d 11=%d\r",\
                            ad_2_5_vref_val,\
                            adc1_Group1_mV[0],adc1_Group1_mV[1],\
                            adc1_Group1_mV[2],adc1_Group1_mV[3],\
                            adc1_Group1_mV[4],adc1_Group1_mV[5],\
                            adc1_Group1_mV[6],adc1_Group1_mV[7],\
                            adc1_Group1_mV[8],adc1_Group1_mV[9],\
                            adc1_Group1_mV[10],adc1_Group1_mV[11]);
                }
                else
                {
                    if(ad_2_5_vref_flag== FALSE)    /* print AtoD in counts */
                    {
                        sprintf(return_message,"ref=%d 0=%d 1=%d 2=%d 3=%d 4=%d 5=%d 6=%d 7=%d 8=%d 9=%d 10=%d 11=%d\r",\
                                ad_2_5_vref_val,\
                                adc1_Group1_filtered[0],adc1_Group1_filtered[1],\
                                adc1_Group1_filtered[2],adc1_Group1_filtered[3],\
                                adc1_Group1_filtered[4],adc1_Group1_filtered[5],\
                                adc1_Group1_filtered[6],adc1_Group1_filtered[7],\
                                adc1_Group1_filtered[8],adc1_Group1_filtered[9],\
                                adc1_Group1_filtered[10],adc1_Group1_filtered[11]);
                    }
                    else
                    {
                        sprintf(return_message,"vref_flag corrupt=%d\r",ad_2_5_vref_flag);
                    }
                }
                transmit_str_485(return_message);

                AtoD_print_count--;
            }
            else
            {
                AtoD_print_type = 0;    /* done printing */
            }
        }  // end if((system_msec_clock % 201 == 0)   /* if reqested, write AtoD values to RS485  */

        /*  -------------------  if reqested, write AtoD values to RS485  ----------  E N D  --------------------*/



        /*  Phil's Fix for all 8's in RTS  Log -------  Delay setting up RS485 RX after TX  ---------------------------*/
        if((tx_to_rx_delay_flag == 1) && ((scilinREG->FLR & TX_EMPTY) == 0x800))
        {
            gioSetBit(hetPORT1, 16, 0); // Set RS485 CS to Receive
            scilinREG->GCR1 = scilinREG->GCR1 | ADDR_MODE; //Set to interrupt on address message only

            sciReceive(scilinREG, 1, sbuf_rx);
            tx_to_rx_delay_flag = 0;
        }
        break;  // case == rtiNOTIFICATION_COMPARE0

        case rtiNOTIFICATION_COMPARE1:

            break;

        case rtiNOTIFICATION_COMPARE2:

            break;

        case rtiNOTIFICATION_COMPARE3:

            break;

        case rtiNOTIFICATION_TIMEBASE:

            break;

        case rtiNOTIFICATION_COUNTER0:

            break;

        case rtiNOTIFICATION_COUNTER1:

            break;

    }

    /* USER CODE END */
}

/* USER CODE BEGIN (10) */
/* USER CODE END */
#pragma WEAK(adcNotification)
void adcNotification(adcBASE_t *adc, uint32 group)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (11) */

    // debug - time benchmark
    timer_a = rtiGetCurrentTick(rtiCOMPARE0);

    if(adc == adcREG1)
    {

        switch (group)
        {
        case 0:         // event group
            break;
        case 1:         // group1

            switch (CIB_analog_config)
            {
            case BAIRBOARD:     // reduced channel count  (still reads all based on HalCoGen setup, don't process all channels, save time in ISR)
                /* ---------- Scan of 12ch complete, move 12channels of data from adc buffer to user space ------------------- */

                acd1_count = adcGetData(adcREG1, adcGROUP1, adc1_Group1_raw);       // move 12channels of data from adc buffer to user space

                /* ---------- Subtract the oldest data from sum ----------------------------- */

                //  example         summ -= adc1_Group1_data[oldestt][11].value;            // subtract the oldest data from sum
                //adc1_Group1_sum[0] -= adc1_Group1_history[oldest][0];
                //adc1_Group1_sum[1] -= adc1_Group1_history[oldest][1];
                //adc1_Group1_sum[2] -= adc1_Group1_history[oldest][2];
                //adc1_Group1_sum[3] -= adc1_Group1_history[oldest][3];
                //adc1_Group1_sum[4] -= adc1_Group1_history[oldest][4];
                //adc1_Group1_sum[5] -= adc1_Group1_history[oldest][5];
                //adc1_Group1_sum[6] -= adc1_Group1_history[oldest][6];
                //adc1_Group1_sum[7] -= adc1_Group1_history[oldest][7];
                adc1_Group1_sum[8] -= adc1_Group1_history[oldest][8];
                adc1_Group1_sum[9] -= adc1_Group1_history[oldest][9];
                adc1_Group1_sum[10] -= adc1_Group1_history[oldest][10];
                adc1_Group1_sum[11] -= adc1_Group1_history[oldest][11];

                /* ---------- Add the newest data to sum  ----------------------------------- */

                //  example         summ += adc1_Group1_data[10][11].value;                 // Add the newest data to sum
                //adc1_Group1_sum[0] += adc1_Group1_raw[0].value;
                //adc1_Group1_sum[1] += adc1_Group1_raw[1].value;
                //adc1_Group1_sum[2] += adc1_Group1_raw[2].value;
                //adc1_Group1_sum[3] += adc1_Group1_raw[3].value;
                //adc1_Group1_sum[4] += adc1_Group1_raw[4].value;
                //adc1_Group1_sum[5] += adc1_Group1_raw[5].value;
                //adc1_Group1_sum[6] += adc1_Group1_raw[6].value;
                //adc1_Group1_sum[7] += adc1_Group1_raw[7].value;
                adc1_Group1_sum[8] += adc1_Group1_raw[8].value;
                adc1_Group1_sum[9] += adc1_Group1_raw[9].value;
                adc1_Group1_sum[10] += adc1_Group1_raw[10].value;
                adc1_Group1_sum[11] += adc1_Group1_raw[11].value;

                /* ---------- Insert the newest data into history in circular buffer --------- */

                //  example         adc1_Group1_data[oldestt][11].value = adc1_Group1_data[10][11].value;       // insert the newest data into history
                //adc1_Group1_history[oldest][0] = adc1_Group1_raw[0].value;
                //adc1_Group1_history[oldest][1] = adc1_Group1_raw[1].value;
                //adc1_Group1_history[oldest][2] = adc1_Group1_raw[2].value;
                //adc1_Group1_history[oldest][3] = adc1_Group1_raw[3].value;
                //adc1_Group1_history[oldest][4] = adc1_Group1_raw[4].value;
                //adc1_Group1_history[oldest][5] = adc1_Group1_raw[5].value;
                //adc1_Group1_history[oldest][6] = adc1_Group1_raw[6].value;
                //adc1_Group1_history[oldest][7] = adc1_Group1_raw[7].value;
                adc1_Group1_history[oldest][8] = adc1_Group1_raw[8].value;
                adc1_Group1_history[oldest][9] = adc1_Group1_raw[9].value;
                adc1_Group1_history[oldest][10] = adc1_Group1_raw[10].value;
                adc1_Group1_history[oldest][11] = adc1_Group1_raw[11].value;

                /* ---------- Bump pointer to next slot in the circular buffer, it now the oldest data --------- */

                oldest += 1;                                            // bump the pointer to next slot in the circular buffer, it now the oldest data

                /* ---------- FIR filter using average of the sums ---------------------------- */

                //  when using B_Board and 96way 5volt power, offset will compensate for readbacks loss because of board interconnect
                //adc1_Group1_filtered[0] = (adc1_Group1_sum[0] / 10) + ad_ground_offset_in_tick;
                //adc1_Group1_filtered[1] = (adc1_Group1_sum[1] / 10) + ad_ground_offset_in_tick;
                //adc1_Group1_filtered[2] = (adc1_Group1_sum[2] / 10) + ad_ground_offset_in_tick;
                //adc1_Group1_filtered[3] = (adc1_Group1_sum[3] / 10) + ad_ground_offset_in_tick;
                //adc1_Group1_filtered[4] = (adc1_Group1_sum[4] / 10) + ad_ground_offset_in_tick;
                //adc1_Group1_filtered[5] = (adc1_Group1_sum[5] / 10) + ad_ground_offset_in_tick;
                //adc1_Group1_filtered[6] = (adc1_Group1_sum[6] / 10) + ad_ground_offset_in_tick;
                //adc1_Group1_filtered[7] = (adc1_Group1_sum[7] / 10) + ad_ground_offset_in_tick;
                adc1_Group1_filtered[8] = (adc1_Group1_sum[8] / 10) + ad_ground_offset_in_tick;
                adc1_Group1_filtered[9] = (adc1_Group1_sum[9] / 10) + ad_ground_offset_in_tick;
                adc1_Group1_filtered[10] = (adc1_Group1_sum[10] / 10) + ad_ground_offset_in_tick;
                adc1_Group1_filtered[11] = (adc1_Group1_sum[11] / 10) + ad_ground_offset_in_tick;

                /* ---------- Check if 2.5V ref flag is TRUE (GOOD VALUES), if so, convert counts to mVolts, else set to -1 to flag user ---------------------------- */

                if(ad_2_5_vref_flag == TRUE)
                {
                    //adc1_Group1_mV[0]  = (adc1_Group1_filtered[0]  * 2500.0) / (ad_2_5_vref_val * 1.0);   /* value in mVolts, ie  1V=1000mV */
                    //adc1_Group1_mV[1]  = (adc1_Group1_filtered[1]  * 2500.0) / (ad_2_5_vref_val * 1.0);
                    //adc1_Group1_mV[2]  = (adc1_Group1_filtered[2]  * 2500.0) / (ad_2_5_vref_val * 1.0);
                    //adc1_Group1_mV[3]  = (adc1_Group1_filtered[3]  * 2500.0) / (ad_2_5_vref_val * 1.0);
                    //adc1_Group1_mV[4]  = (adc1_Group1_filtered[4]  * 2500.0) / (ad_2_5_vref_val * 1.0);
                    //adc1_Group1_mV[5]  = (adc1_Group1_filtered[5]  * 2500.0) / (ad_2_5_vref_val * 1.0);
                    //adc1_Group1_mV[6]  = (adc1_Group1_filtered[6]  * 2500.0) / (ad_2_5_vref_val * 1.0);
                    //adc1_Group1_mV[7]  = (adc1_Group1_filtered[7]  * 2500.0) / (ad_2_5_vref_val * 1.0);
                    adc1_Group1_mV[8]  = (adc1_Group1_filtered[8]  * 2500.0) / (ad_2_5_vref_val * 1.0);
                    adc1_Group1_mV[9]  = (adc1_Group1_filtered[9]  * 2500.0) / (ad_2_5_vref_val * 1.0);
                    adc1_Group1_mV[10] = (adc1_Group1_filtered[10] * 2500.0) / (ad_2_5_vref_val * 1.0);
                    adc1_Group1_mV[11] = (adc1_Group1_filtered[11] * 2500.0) / (ad_2_5_vref_val * 1.0);

                    dumperr_Tester1_vign    = adc1_Group1_mV[10] * 6;   /* voltage divider of 5k --*-- 1k, so 6*value = vign =converted to mV(Ex: 9.0V = 9000) */
                    dumperr_Product1_5volt  = adc1_Group1_mV[9];
                    dumperr_Product1_3volt  = adc1_Group1_mV[8];

                }
                else
                {
                    //adc1_Group1_mV[0]  = -1;
                    //adc1_Group1_mV[1]  = -1;
                    //adc1_Group1_mV[2]  = -1;
                    //adc1_Group1_mV[3]  = -1;
                    //adc1_Group1_mV[4]  = -1;
                    //adc1_Group1_mV[5]  = -1;
                    //adc1_Group1_mV[6]  = -1;
                    //adc1_Group1_mV[7]  = -1;
                    adc1_Group1_mV[8]  = -1;
                    adc1_Group1_mV[9]  = -1;
                    adc1_Group1_mV[10] = -1;
                    adc1_Group1_mV[11] = adc1_Group1_filtered[11];      // user can see the vaule that caused vref failure

                }

                /* ---------- If reached the end of the ring, adjust pointer ---------------------------- */

                if (oldest >= 10) oldest = 0;                           // if reached the end of the ring, adjust pointer


                break;   //end case BAIRBOARD:      // reduced channel count

            case FULL:          // All channels present
                /* ---------- Scan of 12ch complete, move 12channels of data from adc buffer to user space ------------------- */

                acd1_count = adcGetData(adcREG1, adcGROUP1, adc1_Group1_raw);       // move 12channels of data from adc buffer to user space

                /* ---------- Subtract the oldest data from sum ----------------------------- */

                //  example             summ -= adc1_Group1_data[oldestt][11].value;            // subtract the oldest data from sum
                adc1_Group1_sum[0] -= adc1_Group1_history[oldest][0];
                adc1_Group1_sum[1] -= adc1_Group1_history[oldest][1];
                adc1_Group1_sum[2] -= adc1_Group1_history[oldest][2];
                adc1_Group1_sum[3] -= adc1_Group1_history[oldest][3];
                adc1_Group1_sum[4] -= adc1_Group1_history[oldest][4];
                adc1_Group1_sum[5] -= adc1_Group1_history[oldest][5];
                adc1_Group1_sum[6] -= adc1_Group1_history[oldest][6];
                adc1_Group1_sum[7] -= adc1_Group1_history[oldest][7];
                adc1_Group1_sum[8] -= adc1_Group1_history[oldest][8];
                adc1_Group1_sum[9] -= adc1_Group1_history[oldest][9];
                adc1_Group1_sum[10] -= adc1_Group1_history[oldest][10];
                adc1_Group1_sum[11] -= adc1_Group1_history[oldest][11];

                /* ---------- Add the newest data to sum  ----------------------------------- */

                //  example         summ += adc1_Group1_data[10][11].value;                 // Add the newest data to sum
                adc1_Group1_sum[0] += adc1_Group1_raw[0].value;
                adc1_Group1_sum[1] += adc1_Group1_raw[1].value;
                adc1_Group1_sum[2] += adc1_Group1_raw[2].value;
                adc1_Group1_sum[3] += adc1_Group1_raw[3].value;
                adc1_Group1_sum[4] += adc1_Group1_raw[4].value;
                adc1_Group1_sum[5] += adc1_Group1_raw[5].value;
                adc1_Group1_sum[6] += adc1_Group1_raw[6].value;
                adc1_Group1_sum[7] += adc1_Group1_raw[7].value;
                adc1_Group1_sum[8] += adc1_Group1_raw[8].value;
                adc1_Group1_sum[9] += adc1_Group1_raw[9].value;
                adc1_Group1_sum[10] += adc1_Group1_raw[10].value;
                adc1_Group1_sum[11] += adc1_Group1_raw[11].value;

                /* ---------- Insert the newest data into history in circular buffer --------- */

                //  example         adc1_Group1_data[oldestt][11].value = adc1_Group1_data[10][11].value;       // insert the newest data into history
                adc1_Group1_history[oldest][0] = adc1_Group1_raw[0].value;
                adc1_Group1_history[oldest][1] = adc1_Group1_raw[1].value;
                adc1_Group1_history[oldest][2] = adc1_Group1_raw[2].value;
                adc1_Group1_history[oldest][3] = adc1_Group1_raw[3].value;
                adc1_Group1_history[oldest][4] = adc1_Group1_raw[4].value;
                adc1_Group1_history[oldest][5] = adc1_Group1_raw[5].value;
                adc1_Group1_history[oldest][6] = adc1_Group1_raw[6].value;
                adc1_Group1_history[oldest][7] = adc1_Group1_raw[7].value;
                adc1_Group1_history[oldest][8] = adc1_Group1_raw[8].value;
                adc1_Group1_history[oldest][9] = adc1_Group1_raw[9].value;
                adc1_Group1_history[oldest][10] = adc1_Group1_raw[10].value;
                adc1_Group1_history[oldest][11] = adc1_Group1_raw[11].value;

                /* ---------- Bump pointer to next slot in the circular buffer, it now the oldest data --------- */

                oldest += 1;                                            // bump the pointer to next slot in the circular buffer, it now the oldest data

                /* ---------- FIR filter using average of the sums ---------------------------- */

                //  when using B_Board and 96way 5volt power, offset will compensate for readbacks loss because of board interconnect
                adc1_Group1_filtered[0] = (adc1_Group1_sum[0] / 10) + ad_ground_offset_in_tick;
                adc1_Group1_filtered[1] = (adc1_Group1_sum[1] / 10) + ad_ground_offset_in_tick;
                adc1_Group1_filtered[2] = (adc1_Group1_sum[2] / 10) + ad_ground_offset_in_tick;
                adc1_Group1_filtered[3] = (adc1_Group1_sum[3] / 10) + ad_ground_offset_in_tick;
                adc1_Group1_filtered[4] = (adc1_Group1_sum[4] / 10) + ad_ground_offset_in_tick;
                adc1_Group1_filtered[5] = (adc1_Group1_sum[5] / 10) + ad_ground_offset_in_tick;
                adc1_Group1_filtered[6] = (adc1_Group1_sum[6] / 10) + ad_ground_offset_in_tick;
                adc1_Group1_filtered[7] = (adc1_Group1_sum[7] / 10) + ad_ground_offset_in_tick;
                adc1_Group1_filtered[8] = (adc1_Group1_sum[8] / 10) + ad_ground_offset_in_tick;
                adc1_Group1_filtered[9] = (adc1_Group1_sum[9] / 10) + ad_ground_offset_in_tick;
                adc1_Group1_filtered[10] = (adc1_Group1_sum[10] / 10) + ad_ground_offset_in_tick;
                adc1_Group1_filtered[11] = (adc1_Group1_sum[11] / 10) + ad_ground_offset_in_tick;

                /* ---------- Check if 2.5V ref flag is TRUE (GOOD VALUES), if so, convert counts to mVolts, else set to -1 to flag user ---------------------------- */

                if(ad_2_5_vref_flag == TRUE)
                {
                    adc1_Group1_mV[0]  = (adc1_Group1_filtered[0]  * 2500.0) / (ad_2_5_vref_val * 1.0); /* value in mVolts, ie  1V=1000mV */
                    adc1_Group1_mV[1]  = (adc1_Group1_filtered[1]  * 2500.0) / (ad_2_5_vref_val * 1.0);
                    adc1_Group1_mV[2]  = (adc1_Group1_filtered[2]  * 2500.0) / (ad_2_5_vref_val * 1.0);
                    adc1_Group1_mV[3]  = (adc1_Group1_filtered[3]  * 2500.0) / (ad_2_5_vref_val * 1.0);
                    adc1_Group1_mV[4]  = (adc1_Group1_filtered[4]  * 2500.0) / (ad_2_5_vref_val * 1.0);
                    adc1_Group1_mV[5]  = (adc1_Group1_filtered[5]  * 2500.0) / (ad_2_5_vref_val * 1.0);
                    adc1_Group1_mV[6]  = (adc1_Group1_filtered[6]  * 2500.0) / (ad_2_5_vref_val * 1.0);
                    adc1_Group1_mV[7]  = (adc1_Group1_filtered[7]  * 2500.0) / (ad_2_5_vref_val * 1.0);
                    adc1_Group1_mV[8]  = (adc1_Group1_filtered[8]  * 2500.0) / (ad_2_5_vref_val * 1.0);
                    adc1_Group1_mV[9]  = (adc1_Group1_filtered[9]  * 2500.0) / (ad_2_5_vref_val * 1.0);
                    adc1_Group1_mV[10] = (adc1_Group1_filtered[10] * 2500.0) / (ad_2_5_vref_val * 1.0);
                    adc1_Group1_mV[11] = (adc1_Group1_filtered[11] * 2500.0) / (ad_2_5_vref_val * 1.0);

                    dumperr_Tester1_vign    = adc1_Group1_mV[10] * 6;   /* voltage divider of 5k --*-- 1k, so 6*value = vign =converted to mV(Ex: 9.0V = 9000) */
                    dumperr_Product1_5volt  = adc1_Group1_mV[9];
                    dumperr_Product1_3volt  = adc1_Group1_mV[8];

                }
                else
                {
                    adc1_Group1_mV[0]  = -1;
                    adc1_Group1_mV[1]  = -1;
                    adc1_Group1_mV[2]  = -1;
                    adc1_Group1_mV[3]  = -1;
                    adc1_Group1_mV[4]  = -1;
                    adc1_Group1_mV[5]  = -1;
                    adc1_Group1_mV[6]  = -1;
                    adc1_Group1_mV[7]  = -1;
                    adc1_Group1_mV[8]  = -1;
                    adc1_Group1_mV[9]  = -1;
                    adc1_Group1_mV[10] = -1;
                    adc1_Group1_mV[11] = adc1_Group1_filtered[11];      // user can see the vaule that caused vref failure

                }

                /* ---------- If reached the end of the ring, adjust pointer ---------------------------- */

                if (oldest >= 10) oldest = 0;                           // if reached the end of the ring, adjust pointer


                break;  // end case FULL:           // All channels present
            }



            break;  // end case 1:          // group1

            case 2:         // group2
                break;

        }       // end switch on group for adcREG1, group#

    }   // end if(adc == adcREG1)

    if(adc == adcREG2)
    {

        switch (group)
        {
        case 0:         // event group
            break;
        case 1:         // group1
        {
            // all acd2 data is for threshold reading of 0 or 1, no filtering required
            acd2_count = adcGetData(adcREG2, adcGROUP1, adc2_Group1_raw);       // move 12channels of data from adc buffer to user space

        }

        break;
        case 2:         // group2
            break;

        }       // end switch on group for adcREG2, group#

    }   // end if(adc == adcREG2)

    // debug - time benchmark
    timer_b = rtiGetCurrentTick(rtiCOMPARE0);
    timer_c = timer_b - timer_a;


    /* USER CODE END */
}

/* USER CODE BEGIN (12) */
/* USER CODE END */
#pragma WEAK(canErrorNotification)
void canErrorNotification(canBASE_t *node, uint32 notification)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (13) */

    //    @param[in] notification Error notification code:
    // *           - canLEVEL_PASSIVE    (0x20) : When RX- or TX error counter are between 32 and 63
    // *           - canLEVEL_WARNING    (0x40) : When RX- or TX error counter are between 64 and 127
    // *           - canLEVEL_BUS_OFF    (0x80) : When RX- or TX error counter are between 128 and 255
    // *           - canLEVEL_PARITY_ERR (0x100): When RX- or TX error counter are above 256

    if(node == canREG1)
    {
        can1_stat_temp = notification;

        if(checkbit(can1_stat_temp,6))  /* buss warn, not much to do*/
        {
            bus1_warn_state = 1;    // set bus_warn_state to 1 so we know the condition exists
            // it is cleared in dumperr after it is sent out in dumperr string
        }

        if(checkbit(can1_stat_temp,7))  /* buss off */
        {
            bus1_off_state = 1; //set flag to one to indicate interrupt after bus off
            // it is cleared after a dumperr so we know there was
            // a bussoff somewhere before this dump

            clrbit(canREG1->CTL,0); // clr software init to start recovery, TMS570
            //          canREG1->CTL &= (uint32)(0xFFFFFFFEU);
            //          gioSetBit(canREG1->CTL, 0, 0 );

            // *check for TMS570 code ** clrbit(CAN1.can_con,0);    // clr software init to start recovery
        }     // end if buss off reset
    }
    if(node == canREG2)
    {
        can2_stat_temp = notification;

        if(checkbit(can2_stat_temp,6))  /* buss warn, not much to do*/
        {
            bus2_warn_state = 1;    // set bus_warn_state to 1 so we know the condition exists
            // it is cleared in dumperr after it is snet out in dumperr string
        }

        if(checkbit(can2_stat_temp,7))  /* buss off */
        {
            bus2_off_state = 1; //set flag to one to indicate interrupt after bus off
            // it is cleared after a dumperr so we no there was
            // a bussoff somewhere before this dump

            clrbit(canREG2->CTL,0); // clr software init to start recovery, TMS570
            //          canREG2->CTL &= (uint32)(0xFFFFFFFEU);
            //          gioSetBit(canREG2->CTL, 0, 0 );

            // *check for TMS570 code **   clrbit(CAN2.can_con,0);  // clr software init to start recovery
        }     // end if buss off reset
    }


    /* USER CODE END */
}

#pragma WEAK(canStatusChangeNotification)
void canStatusChangeNotification(canBASE_t *node, uint32 notification)  
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (14) */
    /* USER CODE END */
}

#pragma WEAK(canMessageNotification)
void canMessageNotification(canBASE_t *node, uint32 messageBox)  
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (15) */
    // can message interrupt ISR, various potential sources, "notification" contains which one

    if(node == canREG1)
    {

        //THIS is a dummy case for reference of how to build into software. I would probably
        //  create a special XCP_CAN_DAQ_TX message box and look at that outside of switch(target_product).
        //  I have not confirmed this steps functionality.
        if(messageBox == CAN_XCP_DAQ_RX)
        {
            canGetData(canREG1,CAN_XCP_DAQ_RX, can1_rx_data); // get data, store in rx_data array

            if(DAQ_mode_on_off == ON)
            {
                handle_DAQ_notification_CAN();
            }
            else if(XCP_fast_rate_active_flag == ON)
            {
                handle_XCP_fast_rate_notification();
            }

        }
        switch(target_product)
        {
        case TARGET_T1XX:       //  VVVVVVVVVVVVVVVVVVVVVVVVVVVVV  T 1 X X  C A N 1  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
        {
            switch(messageBox)
            {
            case PROD_MESS1_ID:     // 0x148    canMESSAGE_BOX1 11bit identifier-- Rec Response from EPS (not used)
                break;

            case PROD_MESS2_ID:     // 0x778    canMESSAGE_BOX2 11bit identifier-- Rec ON Star from EPS  (not used)
                break;

            case CCP_Engine_Run:    // 0x0C9    canMESSAGE_BOX3 11bit identifier-- Trans Engine ON to EPS
                break;

            case CCP_Power_Mode_ID:       // 0x1F1    canMESSAGE_BOX4 11bit identifier-- Trans Power Mode to EPS
                break;

            case CCP_SPEED_ID:      // 0x3E9    canMESSAGE_BOX5 11bit identifier-- Trans Speed and Validity Status to EPS

                break;

            case CCP_RESPONSE_ID:     // 0x706  canMESSAGE_BOX6 11bit identifier-- Rec CCP response from EPS
                // recieve and decide what the REQ was, parse and store
                // Standard dump of all ccp request

                canGetData(canREG1, CCP_RESPONSE_ID, can1_rx_data); // get data, store in rx_data array

                switch(can1_request_index)      // requesting multiple vaiables using same messageBox, can1_request_index keeps track, store and parse in main
                {

                case 0:     // do nothing with request_connect data, setup and trans next request

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_SystemState_byt);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 1:     // store ccp_SystemState_byt, setup and trans next request

                    memcpy(&can1_dump_err_data[1],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_MotCurrQax_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 2:     // store ccp_MotCurrQax_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[2],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_HwTq0Meas_HwTq0_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 3:     // store ccp_HwTq0Meas_HwTq0_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[3],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_HwTq1Meas_HwTq1_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 4:     // store ccp_HwTq1Meas_HwTq1_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[4],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_HwTq2Meas_HwTq2_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 5:     // store ccp_HwTq2Meas_HwTq2_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[5],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_HwTq3Meas_HwTq3_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 6:     // store ccp_HwTq3Meas_HwTq3_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[6],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_EcuTFild_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 7:     // store ccp_EcuTFild_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[7],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_HwAgArbn_HwAg_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 8:     // store ccp_HwAgArbn_HwAg_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[8],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_HwAg0_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 9:     //  store ccp_HwAg0_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[9],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_HwAg1_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 10:        // store ccp_HwAg1_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[10],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_HwTqArbn_HwTq_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 11:        // store ccp_HwTqArbn_HwTq_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[11],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_MotCurrDax_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 12:        // store ccp_MotCurrDax_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[12],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_HwTqArbn_HwTqChA_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 13:        // store ccp_HwTqArbn_HwTqChA_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[13],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_HwTqArbn_HwTqChB_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 14:        // store ccp_HwTqArbn_HwTqChB_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[14],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_MotAgCmp_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 15:        // store ccp_MotAgCmp_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[15],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_BattVltg_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 16:        // store ccp_BattVltg_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[16],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_BattVltgSwd1_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 17:        // store ccp_BattVltgSwd1_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[17],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_BattVltgSwd2_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 18:        // store ccp_BattVltgSwd2_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[18],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_BrdgVltg_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 19:        // store ccp_BrdgVltg_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[19],&can1_rx_data,8);
                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_AssiCmdBas_str);
                    can1_request_index++;

                    break;

                case 20:        // store ccp_AssiCmdBas_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[20],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_MotTqCmd_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 21:        // store ccp_MotTqCmd_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[21],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_AssiMechT_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 22:        // store ccp_AssiMechT_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[22],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_MotFetT_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 23:        // store ccp_MotFetT_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[23],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_MotMagT_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 24:        // store ccp_MotMagT_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[24],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_MotWidgT_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 25:        // store ccp_MotWidgT_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[25],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_MotREstim_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 26:        // store ccp_MotREstim_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[26],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_MotTq_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 27:        // store ccp_MotTq_str, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can1_dump_err_data[27],&can1_rx_data,8);


                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                    //  ----------------   H c u r r    T1XX reading cases 2000-2031 done during normal testing

                case 2000:      // Hcurr Qax readings(x3) // store can_MotCurrQax1_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[28],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_HwTqArbn_HwTq_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2001:      // store ccp_HwTqArbn_HwTq1str, setup and trans next reques

                    memcpy(&can1_dump_err_data[29],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2010:      // store can_MotCurrQax2_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[30],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_HwTqArbn_HwTq_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2011:      // store ccp_HwTqArbn_HwTq2str, setup and trans next reques

                    memcpy(&can1_dump_err_data[31],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2020:      // store can_MotCurrQax3_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[32],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_HwTqArbn_HwTq_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2021:      // store ccp_HwTqArbn_HwTq3str, setup and trans next reques

                    memcpy(&can1_dump_err_data[33],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2999:

                    can1_request_index = 3000;  // don't care response is done...ok to send new request.

                    break;

                default:
                {
                }

                }   // end switch(can1_request_index) for T1XX   C A N 1

                break;  // end case CCP_RESPONSE_ID:

                case CCP_REQUEST_ID:        // 0x708    canMESSAGE_BOX7 11bit identifier-- Trans CCP Request for Data to EPS
                    break;

                case XCP_EA3_REQUEST_ID:     // 0x242   canMESSAGE_BOX8 11bit identifier-- Trans CCP Request for Data to EPS
                    break;

                case XCP_EA3_RESPONSE_ID:    // 0x642   canMESSAGE_BOX9 11bit identifier-- Rec XCP Response from EPS for data
                    // receive and decide what the REQ was, parse and store

                    break; // end case XCP_RESPONSE_ID

                case XCP_EA4_REQUEST_ID:     // 0x712   canMESSAGE_BOX10 11bit identifier-- Trans XCP Request from EPS for data
                    break;

                case XCP_EA4_RESPONSE_ID:     // 0x710   canMESSAGE_BOX11 11bit identifier-- Rec XCP Response to EPS for data
                    canGetData(canREG1, XCP_EA4_RESPONSE_ID, can1_rx_data); // get data, store in rx_data array

                    switch(ECU1_XCP_reply_index)    // requesting multiple requests using same messageBox, DCT_XCP_rply_indx keeps track, clear_dtcs_flag determines action
                    {

                    case 1:
                        //EA4 is different
                        if((can1_rx_data[1] == 0x50) && (can1_rx_data[2] == 0x7E))  // positive responce from enter Nexteer mode
                        {
                            if(ECU1_clear_DTCs_flag == 1)         // clear DTC's mode
                            {
                                canTransmit(canREG1, XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_clear_DTC_rqst1); // request clear DTCs
                            }

                            else        // get DTC's mode

                            {
                                canTransmit(canREG1, XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_get_DTC_rqst1);   // request DTCs first string
                            }

                            ECU1_XCP_reply_index++;
                        }  // end positive responce nexteer mode
                        break;

                    case 2:

                        if((can1_rx_data[0] == 0x10) && (can1_rx_data[2] == 0x62))    // get DTC's mode
                        {

                            dtc1[0][0] =  can1_rx_data[5];
                            dtc1[0][1] =  can1_rx_data[6];
                            dtc1[0][2] =  can1_rx_data[7];

                            canTransmit(canREG1, XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_get_DTC_rqst2);   // request DTCs 2nd string

                            ECU1_XCP_reply_index++;
                        }

                        if((can1_rx_data[0] == 0x04) && (can1_rx_data[1] == 0x71))  // responce to clear DTC's mode
                        {
                            if(ECU1_clear_DTCs_flag == 1)
                            {
                                ECU1_XCP_reply_index = 0;
                                ECU1_clear_DTCs_flag = 0;   // reset flag and index, done
                            }
                        }

                        break;

                    case 3:
                        if(can1_rx_data[0] == 0x21)
                        {
                            dtc1[0][3] =  can1_rx_data[1];
                            dtc1[0][4] =  can1_rx_data[2];
                            dtc1[1][0] =  can1_rx_data[3];
                            dtc1[1][1] =  can1_rx_data[4];
                            dtc1[1][2] =  can1_rx_data[5];
                            dtc1[1][3] =  can1_rx_data[6];
                            dtc1[1][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 4:
                        if(can1_rx_data[0] == 0x22)
                        {
                            dtc1[2][0] =  can1_rx_data[1];
                            dtc1[2][1] =  can1_rx_data[2];
                            dtc1[2][2] =  can1_rx_data[3];
                            dtc1[2][3] =  can1_rx_data[4];
                            dtc1[2][4] =  can1_rx_data[5];
                            dtc1[3][0] =  can1_rx_data[6];
                            dtc1[3][1] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 5:
                        if(can1_rx_data[0] == 0x23)
                        {
                            dtc1[3][2] =  can1_rx_data[1];
                            dtc1[3][3] =  can1_rx_data[2];
                            dtc1[3][4] =  can1_rx_data[3];
                            dtc1[4][0] =  can1_rx_data[4];
                            dtc1[4][1] =  can1_rx_data[5];
                            dtc1[4][2] =  can1_rx_data[6];
                            dtc1[4][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;

                        break;

                    case 6:
                        if(can1_rx_data[0] == 0x24)
                        {
                            dtc1[4][4] =  can1_rx_data[1];
                            dtc1[5][0] =  can1_rx_data[2];
                            dtc1[5][1] =  can1_rx_data[3];
                            dtc1[5][2] =  can1_rx_data[4];
                            dtc1[5][3] =  can1_rx_data[5];
                            dtc1[5][4] =  can1_rx_data[6];
                            dtc1[6][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 7:
                        if(can1_rx_data[0] == 0x25)
                        {
                            dtc1[6][1] =  can1_rx_data[1];
                            dtc1[6][2] =  can1_rx_data[2];
                            dtc1[6][3] =  can1_rx_data[3];
                            dtc1[6][4] =  can1_rx_data[4];
                            dtc1[7][0] =  can1_rx_data[5];
                            dtc1[7][1] =  can1_rx_data[6];
                            dtc1[7][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 8:
                        if(can1_rx_data[0] == 0x26)
                        {
                            dtc1[7][3] =  can1_rx_data[1];
                            dtc1[7][4] =  can1_rx_data[2];
                            dtc1[8][0] =  can1_rx_data[3];
                            dtc1[8][1] =  can1_rx_data[4];
                            dtc1[8][2] =  can1_rx_data[5];
                            dtc1[8][3] =  can1_rx_data[6];
                            dtc1[8][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 9:
                        if(can1_rx_data[0] == 0x27)
                        {
                            dtc1[9][0] =  can1_rx_data[1];
                            dtc1[9][1] =  can1_rx_data[2];
                            dtc1[9][2] =  can1_rx_data[3];
                            dtc1[9][3] =  can1_rx_data[4];
                            dtc1[9][4] =  can1_rx_data[5];
                            dtc1[10][0] = can1_rx_data[6];
                            dtc1[10][1] = can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 10:
                        if(can1_rx_data[0] == 0x28)
                        {
                            dtc1[10][2] =  can1_rx_data[1];
                            dtc1[10][3] =  can1_rx_data[2];
                            dtc1[10][4] =  can1_rx_data[3];
                            dtc1[11][0] =  can1_rx_data[4];
                            dtc1[11][1] =  can1_rx_data[5];
                            dtc1[11][2] =  can1_rx_data[6];
                            dtc1[11][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 11:
                        if(can1_rx_data[0] == 0x29)
                        {
                            dtc1[11][4] =  can1_rx_data[1];
                            dtc1[12][0] =  can1_rx_data[2];
                            dtc1[12][1] =  can1_rx_data[3];
                            dtc1[12][2] =  can1_rx_data[4];
                            dtc1[12][3] =  can1_rx_data[5];
                            dtc1[12][4] =  can1_rx_data[6];
                            dtc1[13][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 12:
                        if(can1_rx_data[0] == 0x2A)
                        {
                            dtc1[13][1] =  can1_rx_data[1];
                            dtc1[13][2] =  can1_rx_data[2];
                            dtc1[13][3] =  can1_rx_data[3];
                            dtc1[13][4] =  can1_rx_data[4];
                            dtc1[14][0] =  can1_rx_data[5];
                            dtc1[14][1] =  can1_rx_data[6];
                            dtc1[14][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 13:
                        if(can1_rx_data[0] == 0x2B)
                        {
                            dtc1[14][3] =  can1_rx_data[1];
                            dtc1[14][4] =  can1_rx_data[2];
                        }
                        ECU1_XCP_reply_index = 14;   // last read set to 14 for get_DTCs function
                        break;

                    default:

                        ECU1_XCP_reply_index = 0;

                        break;

                    } // end switch on DTC_XCP_rply_indx

                    break;

                    case TESTER_REC_DEBUG_ID:    // 0x7FF    canMESSAGE_BOX12 11bit identifier-- REC from Anyone

                        break;

                    case TESTER_TRANS_DEBUG_ID:  // 0x7FE     canMESSAGE_BOX13 11bit identifier-- Transmit to Anyone

                        break;

                    case canMESSAGE_BOX14:      // 0x14  11bit identifier-- Not Used

                        break;

                    case canMESSAGE_BOX15:      // 0x15  11bit identifier-- Not Usedr

                        break;

                    case canMESSAGE_BOX16:      // 0x16  11bit identifier-- Not Used

                        break;

                    default:

                    {
                        can1_message_type_debug = messageBox;  // an unknown messageBox
                    }
                    break;

            }   // end switch(messageBox) for T1XX on canREG1

        }   // end case T1XX    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^  T 1 X X  C A N 1  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        break;

 /*
        case TARGET_BYD_SA2FL:       //  VVVVVVVVVVVVVVVVVVVVVVVVVVVVV  G A C  9 0  C A N 1  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
        {
            switch(messageBox)
            {
            case PROD_MESS1_ID:     // 0x148    canMESSAGE_BOX1 11bit identifier-- Rec Response from EPS (not used)
                break;

            case PROD_MESS2_ID:     // 0x778    canMESSAGE_BOX2 11bit identifier-- Rec ON Star from EPS  (not used)
                break;

            case CCP_Engine_Run:    // 0x0C9    canMESSAGE_BOX3 11bit identifier-- Trans Engine ON to EPS
                break;

            case CCP_Power_Mode_ID:       // 0x1F1    canMESSAGE_BOX4 11bit identifier-- Trans Power Mode to EPS
                break;

            case CCP_SPEED_ID:      // 0x3E9    canMESSAGE_BOX5 11bit identifier-- Trans Speed and Validity Status to EPS

                break;

            case CCP_RESPONSE_ID:     // 0x706  canMESSAGE_BOX6 11bit identifier-- Rec CCP response from EPS
                // recieve and decide what the REQ was, parse and store
                // Standard dump of all ccp request

                canGetData(canREG1, CCP_RESPONSE_ID, can1_rx_data); // get data, store in rx_data array

                switch(can1_request_index)      // requesting multiple vaiables using same messageBox, can1_request_index keeps track, store and parse in main
                {

                case 0:     // do nothing with request_connect data, setup and trans next request

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_BYD_SA2FL_SystemState_byt);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 1:     // store ccp_SystemState_byt, setup and trans next request

                    memcpy(&can1_dump_err_data[1],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_BYD_SA2FL_MotCurrQax_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 2:     // store ccp_MotCurrQax_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[2],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_BYD_SA2FL_HwTq0Meas_HwTq4_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 3:     // store ccp_BYD_SA2FL_HwTq0Meas_HwTq4_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[3],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_BYD_SA2FL_HwTq1Meas_HwTq5_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 4:     // store ccp_BYD_SA2FL_HwTq1Meas_HwTq5_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[4],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_BYD_SA2FL_MotHwPosn_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 5:     // store ccp_BYD_SA2FL_MotHwPosn_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[5],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_BYD_SA2FL_BattVltg_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 6:     // store ccp_BYD_SA2FL_BattVltg_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[6],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_BYD_SA2FL_MotTq_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 7:     // store ccp_BYD_SA2FL_MotTq_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[7],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_BYD_SA2FL_HwTrq_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 8:     // store ccp_BYD_SA2FL_HwTrq_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[8],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_BYD_SA2FL_MotMagTestim_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 9:     //  store ccp_BYD_SA2FL_MotMagTestim_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[9],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_BYD_SA2FL_ECUTFilt_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 10:     // store ccp_BYD_SA2FL_ECUTFilt_fp, setup and trans next reques
                    //  all done, index bumped for main to test
                    memcpy(&can1_dump_err_data[10],&can1_rx_data,8);

                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                    //  ----------------   H c u r r    BYD_SA2FL reading cases 2000-2031 done during normal testing

                case 2000:      // Hcurr Qax readings(x3) // store can_MotCurrQax1_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[11],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_BYD_SA2FL_HwTrq_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2001:      // store ccp_HwTqArbn_HwTq1str, setup and trans next reques

                    memcpy(&can1_dump_err_data[12],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2010:      // store can_MotCurrQax2_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[13],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_BYD_SA2FL_HwTrq_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2011:      // store ccp_HwTqArbn_HwTq2str, setup and trans next reques

                    memcpy(&can1_dump_err_data[14],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2020:      // store can_MotCurrQax3_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[15],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_BYD_SA2FL_HwTrq_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2021:      // store ccp_HwTqArbn_HwTq3str, setup and trans next reques

                    memcpy(&can1_dump_err_data[16],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2999:

                    can1_request_index = 3000;  // don't care response is done...ok to send new request.

                    break;

                default:
                {
                }

                }   // end switch(can1_request_index) for BYD_SA2FL   C A N 1

                break;  // end case CCP_RESPONSE_ID:

                case CCP_REQUEST_ID:        // 0x708    canMESSAGE_BOX7 11bit identifier-- Trans CCP Request for Data to EPS
                    break;

                case XCP_EA3_REQUEST_ID:     // 0x242   canMESSAGE_BOX8 11bit identifier-- Trans CCP Request for Data to EPS
                    break;

                case XCP_EA3_RESPONSE_ID:    // 0x642   canMESSAGE_BOX9 11bit identifier-- Rec XCP Response from EPS for data
                    // receive and decide what the REQ was, parse and store

                    break; // end case XCP_RESPONSE_ID

                case XCP_EA4_REQUEST_ID:     // 0x712   canMESSAGE_BOX10 11bit identifier-- Trans XCP Request from EPS for data
                    break;

                case XCP_EA4_RESPONSE_ID:     // 0x710   canMESSAGE_BOX11 11bit identifier-- Rec XCP Response to EPS for data
                    canGetData(canREG1, XCP_EA4_RESPONSE_ID, can1_rx_data); // get data, store in rx_data array

                    switch(ECU1_XCP_reply_index)    // requesting multiple requests using same messageBox, DCT_XCP_rply_indx keeps track, clear_dtcs_flag determines action
                    {

                    case 1:
                        //EA4 is different
                        if((can1_rx_data[1] == 0x50) && (can1_rx_data[2] == 0x7E))  // positive responce from enter Nexteer mode
                        {
                            if(ECU1_clear_DTCs_flag == 1)        // clear DTC's mode
                            {
                                canTransmit(canREG1, XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_clear_DTC_rqst1);    // request clear DTCs
                            }

                            else       // get DTC's mode

                            {
                                canTransmit(canREG1, XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_get_DTC_rqst1);  // request DTCs first string
                            }

                            ECU1_XCP_reply_index++;
                        }  // end positive responce nexteer mode
                        break;

                    case 2:

                        if((can1_rx_data[0] == 0x10) && (can1_rx_data[2] == 0x62))    // get DTC's mode
                        {

                            dtc1[0][0] =  can1_rx_data[5];
                            dtc1[0][1] =  can1_rx_data[6];
                            dtc1[0][2] =  can1_rx_data[7];

                            canTransmit(canREG1, XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_get_DTC_rqst2);   // request DTCs 2nd string

                            ECU1_XCP_reply_index++;
                        }

                        if((can1_rx_data[0] == 0x04) && (can1_rx_data[1] == 0x71))  // responce to clear DTC's mode
                        {
                            if(ECU1_clear_DTCs_flag == 1)
                            {
                                ECU1_XCP_reply_index = 0;
                                ECU1_clear_DTCs_flag = 0;   // reset flag and index, done
                            }
                        }

                        break;

                    case 3:
                        if(can1_rx_data[0] == 0x21)
                        {
                            dtc1[0][3] =  can1_rx_data[1];
                            dtc1[0][4] =  can1_rx_data[2];
                            dtc1[1][0] =  can1_rx_data[3];
                            dtc1[1][1] =  can1_rx_data[4];
                            dtc1[1][2] =  can1_rx_data[5];
                            dtc1[1][3] =  can1_rx_data[6];
                            dtc1[1][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 4:
                        if(can1_rx_data[0] == 0x22)
                        {
                            dtc1[2][0] =  can1_rx_data[1];
                            dtc1[2][1] =  can1_rx_data[2];
                            dtc1[2][2] =  can1_rx_data[3];
                            dtc1[2][3] =  can1_rx_data[4];
                            dtc1[2][4] =  can1_rx_data[5];
                            dtc1[3][0] =  can1_rx_data[6];
                            dtc1[3][1] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 5:
                        if(can1_rx_data[0] == 0x23)
                        {
                            dtc1[3][2] =  can1_rx_data[1];
                            dtc1[3][3] =  can1_rx_data[2];
                            dtc1[3][4] =  can1_rx_data[3];
                            dtc1[4][0] =  can1_rx_data[4];
                            dtc1[4][1] =  can1_rx_data[5];
                            dtc1[4][2] =  can1_rx_data[6];
                            dtc1[4][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;

                        break;

                    case 6:
                        if(can1_rx_data[0] == 0x24)
                        {
                            dtc1[4][4] =  can1_rx_data[1];
                            dtc1[5][0] =  can1_rx_data[2];
                            dtc1[5][1] =  can1_rx_data[3];
                            dtc1[5][2] =  can1_rx_data[4];
                            dtc1[5][3] =  can1_rx_data[5];
                            dtc1[5][4] =  can1_rx_data[6];
                            dtc1[6][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 7:
                        if(can1_rx_data[0] == 0x25)
                        {
                            dtc1[6][1] =  can1_rx_data[1];
                            dtc1[6][2] =  can1_rx_data[2];
                            dtc1[6][3] =  can1_rx_data[3];
                            dtc1[6][4] =  can1_rx_data[4];
                            dtc1[7][0] =  can1_rx_data[5];
                            dtc1[7][1] =  can1_rx_data[6];
                            dtc1[7][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 8:
                        if(can1_rx_data[0] == 0x26)
                        {
                            dtc1[7][3] =  can1_rx_data[1];
                            dtc1[7][4] =  can1_rx_data[2];
                            dtc1[8][0] =  can1_rx_data[3];
                            dtc1[8][1] =  can1_rx_data[4];
                            dtc1[8][2] =  can1_rx_data[5];
                            dtc1[8][3] =  can1_rx_data[6];
                            dtc1[8][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 9:
                        if(can1_rx_data[0] == 0x27)
                        {
                            dtc1[9][0] =  can1_rx_data[1];
                            dtc1[9][1] =  can1_rx_data[2];
                            dtc1[9][2] =  can1_rx_data[3];
                            dtc1[9][3] =  can1_rx_data[4];
                            dtc1[9][4] =  can1_rx_data[5];
                            dtc1[10][0] = can1_rx_data[6];
                            dtc1[10][1] = can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 10:
                        if(can1_rx_data[0] == 0x28)
                        {
                            dtc1[10][2] =  can1_rx_data[1];
                            dtc1[10][3] =  can1_rx_data[2];
                            dtc1[10][4] =  can1_rx_data[3];
                            dtc1[11][0] =  can1_rx_data[4];
                            dtc1[11][1] =  can1_rx_data[5];
                            dtc1[11][2] =  can1_rx_data[6];
                            dtc1[11][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 11:
                        if(can1_rx_data[0] == 0x29)
                        {
                            dtc1[11][4] =  can1_rx_data[1];
                            dtc1[12][0] =  can1_rx_data[2];
                            dtc1[12][1] =  can1_rx_data[3];
                            dtc1[12][2] =  can1_rx_data[4];
                            dtc1[12][3] =  can1_rx_data[5];
                            dtc1[12][4] =  can1_rx_data[6];
                            dtc1[13][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 12:
                        if(can1_rx_data[0] == 0x2A)
                        {
                            dtc1[13][1] =  can1_rx_data[1];
                            dtc1[13][2] =  can1_rx_data[2];
                            dtc1[13][3] =  can1_rx_data[3];
                            dtc1[13][4] =  can1_rx_data[4];
                            dtc1[14][0] =  can1_rx_data[5];
                            dtc1[14][1] =  can1_rx_data[6];
                            dtc1[14][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 13:
                        if(can1_rx_data[0] == 0x2B)
                        {
                            dtc1[14][3] =  can1_rx_data[1];
                            dtc1[14][4] =  can1_rx_data[2];
                        }
                        ECU1_XCP_reply_index = 14;   // last read set to 14 for get_DTCs function
                        break;

                    case 100:
                        ECU1_XCP_reply_index = 101;     // case used for multi message for TOC
                        break;

                    case 101:
                        ECU1_XCP_reply_index = 102;     // case used for multi message for TOC
                        break;

                    default:

                        ECU1_XCP_reply_index = 0;

                        break;

                    } // end switch on DTC_XCP_rply_indx

                    break;

                    case TESTER_REC_DEBUG_ID:    // 0x7FF    canMESSAGE_BOX12 11bit identifier-- REC from Anyone

                        break;

                    case TESTER_TRANS_DEBUG_ID:  // 0x7FE     canMESSAGE_BOX13 11bit identifier-- Transmit to Anyone

                        break;

                    case canMESSAGE_BOX14:      // 0x14  11bit identifier-- Not Used

                        break;

                    case canMESSAGE_BOX15:      // 0x15  11bit identifier-- Not Usedr

                        break;

                    case canMESSAGE_BOX16:      // 0x16  11bit identifier-- Not Used

                        break;

                    default:

                    {
                        can1_message_type_debug = messageBox;  // an unknown messageBox
                    }
                    break;

            }   // end switch(messageBox) for BYD_SA2FL on canREG1

        }   // end case BYD_SA2FL    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^  G A C 2 6  C A N 1  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        break;
*/

       case TARGET_GWM_A0607:       //  VVVVVVVVVVVVVVVVVVVVVVVVVVVVV  G W M A 0 6 0 7  C A N 1  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
        {
            switch(messageBox)
            {
            case PROD_MESS1_ID:     // 0x148    canMESSAGE_BOX1 11bit identifier-- Rec Response from EPS (not used)
                break;

            case PROD_MESS2_ID:     // 0x778    canMESSAGE_BOX2 11bit identifier-- Rec ON Star from EPS  (not used)
                break;

            case CCP_Engine_Run:    // 0x271    canMESSAGE_BOX3 11bit identifier-- Trans Engine ON to EPS
                break;

            case GWM_A0607_CCP_Vehspd_ID:       // 0x265    canMESSAGE_BOX4 11bit identifier-- Trans VehSpd to EPS
                break;

            case GWM_A0607_CCP_Endspd_ID:      // 0x348    canMESSAGE_BOX5 11bit identifier-- Trans Speed and Validity Status to EPS

                break;

            case GWM_A0607_CCP_RESPONSE_ID:     // 0x706  0x63D  canMESSAGE_BOX6 11bit identifier-- Rec CCP response from EPS
                // recieve and decide what the REQ was, parse and store
                // Standard dump of all ccp request

                canGetData(canREG1, GWM_A0607_CCP_RESPONSE_ID, can1_rx_data); // get data, store in rx_data array

                switch(can1_request_index)      // requesting multiple vaiables using same messageBox, can1_request_index keeps track, store and parse in main
                {

                case 0:     // do nothing with request_connect data, setup and trans next request

                    canTransmit(canREG1, GWM_A0607_CCP_REQUEST_ID, ( uint8 *) ccp_GWM_A0607_SystemState_byt);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 1:     // store ccp_SystemState_byt, setup and trans next request

                    memcpy(&can1_dump_err_data[1],&can1_rx_data,8);

                    canTransmit(canREG1, GWM_A0607_CCP_REQUEST_ID, ( uint8 *) ccp_GWM_A0607_MotCurrQax_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 2:     // store ccp_MotCurrQax_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[2],&can1_rx_data,8);

                    canTransmit(canREG1, GWM_A0607_CCP_REQUEST_ID, ( uint8 *) ccp_GWM_A0607_HwTq0Meas_HwTq4_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 3:     // store ccp_GWM_A0607_HwTq0Meas_HwTq4_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[3],&can1_rx_data,8);

                    canTransmit(canREG1, GWM_A0607_CCP_REQUEST_ID, ( uint8 *) ccp_GWM_A0607_HwTq1Meas_HwTq5_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 4:     // store ccp_GWM_A0607_HwTq1Meas_HwTq5_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[4],&can1_rx_data,8);

                    canTransmit(canREG1, GWM_A0607_CCP_REQUEST_ID, ( uint8 *) ccp_GWM_A0607_MotHwPosn_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 5:     // store ccp_GWM_A0607_MotHwPosn_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[5],&can1_rx_data,8);

                    canTransmit(canREG1, GWM_A0607_CCP_REQUEST_ID, ( uint8 *) ccp_GWM_A0607_BattVltg_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 6:     // store ccp_GWM_A0607_BattVltg_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[6],&can1_rx_data,8);

                    canTransmit(canREG1, GWM_A0607_CCP_REQUEST_ID, ( uint8 *) ccp_GWM_A0607_MotTq_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 7:     // store ccp_GWM_A0607_MotTq_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[7],&can1_rx_data,8);

                    canTransmit(canREG1, GWM_A0607_CCP_REQUEST_ID, ( uint8 *) ccp_GWM_A0607_HwTrq_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 8:     // store ccp_GWM_A0607_HwTrq_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[8],&can1_rx_data,8);

                    canTransmit(canREG1, GWM_A0607_CCP_REQUEST_ID, ( uint8 *) ccp_GWM_A0607_MotMagTestim_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 9:     //  store ccp_GWM_A0607_MotMagTestim_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[9],&can1_rx_data,8);

                    canTransmit(canREG1, GWM_A0607_CCP_REQUEST_ID, ( uint8 *) ccp_GWM_A0607_ECUTFilt_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 10:     // store ccp_GWM_A0607_ECUTFilt_fp, setup and trans next reques
                    //  all done, index bumped for main to test
                    memcpy(&can1_dump_err_data[10],&can1_rx_data,8);

                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                    //  ----------------   H c u r r    GWM_A0607 reading cases 2000-2031 done during normal testing

                case 2000:      // Hcurr Qax readings(x3) // store can_MotCurrQax1_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[11],&can1_rx_data,8);

                    canTransmit(canREG1, GWM_A0607_CCP_REQUEST_ID, ( uint8 *) ccp_GWM_A0607_HwTrq_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2001:      // store ccp_HwTqArbn_HwTq1str, setup and trans next reques

                    memcpy(&can1_dump_err_data[12],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2010:      // store can_MotCurrQax2_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[13],&can1_rx_data,8);

                    canTransmit(canREG1, GWM_A0607_CCP_REQUEST_ID, ( uint8 *) ccp_GWM_A0607_HwTrq_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2011:      // store ccp_HwTqArbn_HwTq2str, setup and trans next reques

                    memcpy(&can1_dump_err_data[14],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2020:      // store can_MotCurrQax3_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[15],&can1_rx_data,8);

                    canTransmit(canREG1, GWM_A0607_CCP_REQUEST_ID, ( uint8 *) ccp_GWM_A0607_HwTrq_fp);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2021:      // store ccp_HwTqArbn_HwTq3str, setup and trans next reques

                    memcpy(&can1_dump_err_data[16],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2999:

                    can1_request_index = 3000;  // don't care response is done...ok to send new request.

                    break;

                default:
                {
                }

                }   // end switch(can1_request_index) for GWM_A0607   C A N 1

                break;  // end case CCP_RESPONSE_ID:

                case GWM_A0607_CCP_REQUEST_ID:        // 0x708 0x63C   canMESSAGE_BOX7 11bit identifier-- Trans CCP Request for Data to EPS
                    break;

                case XCP_EA3_REQUEST_ID:     // 0x242   canMESSAGE_BOX8 11bit identifier-- Trans CCP Request for Data to EPS
                    break;

                case XCP_EA3_RESPONSE_ID:    // 0x642   canMESSAGE_BOX9 11bit identifier-- Rec XCP Response from EPS for data
                    // receive and decide what the REQ was, parse and store

                    break; // end case XCP_RESPONSE_ID

                case XCP_EA4_REQUEST_ID:     // 0x712   canMESSAGE_BOX10 11bit identifier-- Trans XCP Request from EPS for data
                    break;

                case XCP_EA4_RESPONSE_ID:     // 0x710   canMESSAGE_BOX11 11bit identifier-- Rec XCP Response to EPS for data
                    canGetData(canREG1, XCP_EA4_RESPONSE_ID, can1_rx_data); // get data, store in rx_data array

                    switch(ECU1_XCP_reply_index)    // requesting multiple requests using same messageBox, DCT_XCP_rply_indx keeps track, clear_dtcs_flag determines action
                    {

                    case 1:
                        //EA4 is different
                        if((can1_rx_data[1] == 0x50) && (can1_rx_data[2] == 0x7E))  // positive responce from enter Nexteer mode
                        {
                            if(ECU1_clear_DTCs_flag == 1)        // clear DTC's mode
                            {
                                canTransmit(canREG1, XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_clear_DTC_rqst1);    // request clear DTCs
                            }

                            else       // get DTC's mode

                            {
                                canTransmit(canREG1, XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_get_DTC_rqst1);  // request DTCs first string
                            }

                            ECU1_XCP_reply_index++;
                        }  // end positive responce nexteer mode
                        break;

                    case 2:

                        if((can1_rx_data[0] == 0x10) && (can1_rx_data[2] == 0x62))    // get DTC's mode
                        {

                            dtc1[0][0] =  can1_rx_data[5];
                            dtc1[0][1] =  can1_rx_data[6];
                            dtc1[0][2] =  can1_rx_data[7];

                            canTransmit(canREG1, XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_get_DTC_rqst2);   // request DTCs 2nd string

                            ECU1_XCP_reply_index++;
                        }

                        if((can1_rx_data[0] == 0x04) && (can1_rx_data[1] == 0x71))  // responce to clear DTC's mode
                        {
                            if(ECU1_clear_DTCs_flag == 1)
                            {
                                ECU1_XCP_reply_index = 0;
                                ECU1_clear_DTCs_flag = 0;   // reset flag and index, done
                            }
                        }

                        break;

                    case 3:
                        if(can1_rx_data[0] == 0x21)
                        {
                            dtc1[0][3] =  can1_rx_data[1];
                            dtc1[0][4] =  can1_rx_data[2];
                            dtc1[1][0] =  can1_rx_data[3];
                            dtc1[1][1] =  can1_rx_data[4];
                            dtc1[1][2] =  can1_rx_data[5];
                            dtc1[1][3] =  can1_rx_data[6];
                            dtc1[1][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 4:
                        if(can1_rx_data[0] == 0x22)
                        {
                            dtc1[2][0] =  can1_rx_data[1];
                            dtc1[2][1] =  can1_rx_data[2];
                            dtc1[2][2] =  can1_rx_data[3];
                            dtc1[2][3] =  can1_rx_data[4];
                            dtc1[2][4] =  can1_rx_data[5];
                            dtc1[3][0] =  can1_rx_data[6];
                            dtc1[3][1] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 5:
                        if(can1_rx_data[0] == 0x23)
                        {
                            dtc1[3][2] =  can1_rx_data[1];
                            dtc1[3][3] =  can1_rx_data[2];
                            dtc1[3][4] =  can1_rx_data[3];
                            dtc1[4][0] =  can1_rx_data[4];
                            dtc1[4][1] =  can1_rx_data[5];
                            dtc1[4][2] =  can1_rx_data[6];
                            dtc1[4][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;

                        break;

                    case 6:
                        if(can1_rx_data[0] == 0x24)
                        {
                            dtc1[4][4] =  can1_rx_data[1];
                            dtc1[5][0] =  can1_rx_data[2];
                            dtc1[5][1] =  can1_rx_data[3];
                            dtc1[5][2] =  can1_rx_data[4];
                            dtc1[5][3] =  can1_rx_data[5];
                            dtc1[5][4] =  can1_rx_data[6];
                            dtc1[6][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 7:
                        if(can1_rx_data[0] == 0x25)
                        {
                            dtc1[6][1] =  can1_rx_data[1];
                            dtc1[6][2] =  can1_rx_data[2];
                            dtc1[6][3] =  can1_rx_data[3];
                            dtc1[6][4] =  can1_rx_data[4];
                            dtc1[7][0] =  can1_rx_data[5];
                            dtc1[7][1] =  can1_rx_data[6];
                            dtc1[7][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 8:
                        if(can1_rx_data[0] == 0x26)
                        {
                            dtc1[7][3] =  can1_rx_data[1];
                            dtc1[7][4] =  can1_rx_data[2];
                            dtc1[8][0] =  can1_rx_data[3];
                            dtc1[8][1] =  can1_rx_data[4];
                            dtc1[8][2] =  can1_rx_data[5];
                            dtc1[8][3] =  can1_rx_data[6];
                            dtc1[8][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 9:
                        if(can1_rx_data[0] == 0x27)
                        {
                            dtc1[9][0] =  can1_rx_data[1];
                            dtc1[9][1] =  can1_rx_data[2];
                            dtc1[9][2] =  can1_rx_data[3];
                            dtc1[9][3] =  can1_rx_data[4];
                            dtc1[9][4] =  can1_rx_data[5];
                            dtc1[10][0] = can1_rx_data[6];
                            dtc1[10][1] = can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 10:
                        if(can1_rx_data[0] == 0x28)
                        {
                            dtc1[10][2] =  can1_rx_data[1];
                            dtc1[10][3] =  can1_rx_data[2];
                            dtc1[10][4] =  can1_rx_data[3];
                            dtc1[11][0] =  can1_rx_data[4];
                            dtc1[11][1] =  can1_rx_data[5];
                            dtc1[11][2] =  can1_rx_data[6];
                            dtc1[11][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 11:
                        if(can1_rx_data[0] == 0x29)
                        {
                            dtc1[11][4] =  can1_rx_data[1];
                            dtc1[12][0] =  can1_rx_data[2];
                            dtc1[12][1] =  can1_rx_data[3];
                            dtc1[12][2] =  can1_rx_data[4];
                            dtc1[12][3] =  can1_rx_data[5];
                            dtc1[12][4] =  can1_rx_data[6];
                            dtc1[13][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 12:
                        if(can1_rx_data[0] == 0x2A)
                        {
                            dtc1[13][1] =  can1_rx_data[1];
                            dtc1[13][2] =  can1_rx_data[2];
                            dtc1[13][3] =  can1_rx_data[3];
                            dtc1[13][4] =  can1_rx_data[4];
                            dtc1[14][0] =  can1_rx_data[5];
                            dtc1[14][1] =  can1_rx_data[6];
                            dtc1[14][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 13:
                        if(can1_rx_data[0] == 0x2B)
                        {
                            dtc1[14][3] =  can1_rx_data[1];
                            dtc1[14][4] =  can1_rx_data[2];
                        }
                        ECU1_XCP_reply_index = 14;   // last read set to 14 for get_DTCs function
                        break;

                    case 100:
                        ECU1_XCP_reply_index = 101;     // case used for multi message for TOC
                        break;

                    case 101:
                        ECU1_XCP_reply_index = 102;     // case used for multi message for TOC
                        break;

                    default:

                        ECU1_XCP_reply_index = 0;

                        break;

                    } // end switch on DTC_XCP_rply_indx

                    break;

                    case TESTER_REC_DEBUG_ID:    // 0x7FF    canMESSAGE_BOX12 11bit identifier-- REC from Anyone

                        break;

                    case TESTER_TRANS_DEBUG_ID:  // 0x7FE     canMESSAGE_BOX13 11bit identifier-- Transmit to Anyone

                        break;

                    case canMESSAGE_BOX14:      // 0x14  11bit identifier-- Not Used

                        break;

                    case canMESSAGE_BOX15:      // 0x15  11bit identifier-- Not Usedr

                        break;

                    case canMESSAGE_BOX16:      // 0x16  11bit identifier-- Not Used

                        break;

                    default:

                    {
                        can1_message_type_debug = messageBox;  // an unknown messageBox
                    }
                    break;

            }   // end switch(messageBox) for GWM_A0607 on canREG1

        }   // end case GWM_A0607    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^  G W M A 0 6 0 7  C A N 1  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        break;

        case TARGET_9BXX:
        {

        }
        break;

        case TARGET_S550ANALOG:
        {

        }
        break;

        case TARGET_S550DIGITAL:
        {

        }
        break;

        case TARGET_C1XX:       //  VVVVVVVVVVVVVVVVVVVVVVVVVV  C 1 X X  C A N 1  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
        {
            switch(messageBox)
            {
            case PROD_MESS1_ID:     // 0x148    canMESSAGE_BOX1 11bit identifier-- Rec Response from EPS (not used)
                break;

            case PROD_MESS2_ID:     // 0x778    canMESSAGE_BOX2 11bit identifier-- Rec ON Star from EPS  (not used)
                break;

            case CCP_Engine_Run:    // 0x0C9    canMESSAGE_BOX3 11bit identifier-- Trans Engine ON to EPS
                break;

            case CCP_Power_Mode_ID: // 0x1F1      canMESSAGE_BOX4 11bit identifier-- Trans Power Mode to EPS
                break;

            case CCP_SPEED_ID:      // 0x3E9    canMESSAGE_BOX5 11bit identifier-- Trans Speed and Validity Status to EPS

                break;

            case CCP_RESPONSE_ID:     // 0x706  canMESSAGE_BOX6 11bit identifier-- Rec CCP response from EPS
                // recieve and decide what the REQ was, parse and store
                // Standard dump of all ccp request

                canGetData(canREG1, CCP_RESPONSE_ID, can1_rx_data); // get data, store in rx_data array

                switch(can1_request_index)      // requesting multiple vaiables using same messageBox, can1_request_index keeps track, store and parse in main
                {
                case 0:     // do nothing with request_connect data, setup and trans next request

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_BattVltg_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 1:     // store ccp_C1XX_BattVltg_str, setup and trans next request

                    memcpy(&can1_dump_err_data[1],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_Temperature_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 2:     // store ccp_C1XX_Temperature_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[2],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_DigT1_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 3:     // store ccp_C1XX_DigT1_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[3],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_DigT2_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 4:     // store ccp_C1XX_DigT2_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[4],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_MtrCurrQax_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 5:     // store ccp_C1XX_MtrCurrQax_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[5],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_OnStateFltAcc_cnt);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 6:     // store ccp_C1XX_OnStateFltAcc_cnt, setup and trans next reques

                    memcpy(&can1_dump_err_data[6],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_GateDriveFltAcc_cnt);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 7:     // store ccp_C1XX_GateDriveFltAcc_cnt, setup and trans next reques

                    memcpy(&can1_dump_err_data[7],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_GateDrvFltSts_cnt);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 8:     // store ccp_C1XX_GateDrvFltSts_cnt, setup and trans next reques

                    memcpy(&can1_dump_err_data[8],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_SystemState_byt);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 9:     //  store ccp_C1XX_SystemState_byt, setup and trans next reques

                    memcpy(&can1_dump_err_data[9],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;
                case 10:        // store ccp_C1XX_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[10],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_Abs_Hw_Pos_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 11:        // store ccp_C1XX_Abs_Hw_Pos_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[11],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_Rel_Hw_Pos_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 12:        // store ccp_C1XX_Rel_Hw_Pos_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[12],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_ManualTrqCmdEn_cnt);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 13:        // store ccp_C1XX_ManualTrqCmdEn_cnt, setup and trans next reques

                    memcpy(&can1_dump_err_data[13],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_MtrTrq_CmdNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 14:        // store ccp_C1XX_MtrTrq_CmdNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[14],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_CCD_MSB_Die1_cnt);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 15:        // store ccp_C1XX_CCD_MSB_Die1_cnt, setup and trans next reques

                    memcpy(&can1_dump_err_data[15],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_CCD_MSB_Die2_cnt);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 16:        // store ccp_C1XX_CCD_MSB_Die2_cnt, setup and trans next reques

                    memcpy(&can1_dump_err_data[16],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_MtrTempEst_MagTempEst_DegC_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 17:        // store ccp_C1XX_MtrTempEst_MagTempEst_DegC_str, setup and trans next reques
                    //  all done, index bumped for main to test
                    memcpy(&can1_dump_err_data[17],&can1_rx_data,8);

                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;


                    //  ----------------   H c u r r    C1XX reading cases 2000-2031 done during normal testing

                case 2000:      // Hcurr Qax readings(x3) // store can_C1XX_MotCurrQax1_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[28],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2001:      // store ccp_C1XX_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[29],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2010:      // store can_MotCurrQax2_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[30],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2011:      // store ccp_C1XX_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[31],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2020:      // store can_C1XX_MotCurrQax3_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[32],&can1_rx_data,8);

                    canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_C1XX_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2021:      // store ccp_C1XX_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[33],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2999:

                    can1_request_index = 3000;  // don't care response is done...ok to send new request.

                    break;

                default:
                {
                }

                }   // end switch(can1_request_index)

                break;  // end case CCP_RESPONSE_ID:

                case CCP_REQUEST_ID:        // 0x708    canMESSAGE_BOX7 11bit identifier-- Trans CCP Request for Data to EPS
                    break;

                case XCP_EA3_REQUEST_ID:         // 0x242   canMESSAGE_BOX8 11bit identifier-- Trans CCP Request for Data to EPS
                    break;

                case XCP_EA3_RESPONSE_ID:        // 0x642   canMESSAGE_BOX9 11bit identifier-- Rec XCP Responce from EPS for data
                    // receive and decide what the REQ was, parse and store

                    canGetData(canREG1, XCP_EA3_RESPONSE_ID, can1_rx_data); // get data, store in rx_data array

                    switch(ECU1_XCP_reply_index)    // requesting multiple requests using same messageBox, DCT_XCP_rply_indx keeps track, clear_dtcs_flag determines action
                    {

                    case 1:

                        if((can1_rx_data[0] == 0x01) && (can1_rx_data[1] == 0x50))  // positive responce from enter Nexteer mode
                        {
                            if(ECU1_XCP_reply_index == 1)         // clear DTC's mode
                            {
                                canTransmit(canREG1, XCP_EA3_REQUEST_ID, ( uint8 *) xcp_EA3_write_clear_DTC_rqst1); // request clear DTCs
                            }

                            else        // get DTC's mode

                            {
                                canTransmit(canREG1, XCP_EA3_REQUEST_ID, ( uint8 *) xcp_EA3_write_get_DTC_rqst1);   // request DTCs first string
                            }

                            ECU1_XCP_reply_index++;
                        }  // end positive responce nexteer mode
                        break;

                    case 2:

                        if((can1_rx_data[0] == 0x10) && (can1_rx_data[2] == 0x62))    // get DTC's mode
                        {

                            dtc1[0][0] =  can1_rx_data[5];
                            dtc1[0][1] =  can1_rx_data[6];
                            dtc1[0][2] =  can1_rx_data[7];

                            canTransmit(canREG1, XCP_EA3_REQUEST_ID, ( uint8 *) xcp_EA3_write_get_DTC_rqst2);   // request DTCs 2nd string

                            ECU1_XCP_reply_index++;
                        }

                        if((can1_rx_data[0] == 0x04) && (can1_rx_data[1] == 0x71))  // responce to clear DTC's mode
                        {
                            if(ECU1_XCP_reply_index == 1)
                            {
                                ECU1_XCP_reply_index = 0;
                                ECU1_XCP_reply_index = 0;   // reset flag and index, done
                            }
                        }

                        break;

                    case 3:
                        if(can1_rx_data[0] == 0x21)
                        {
                            dtc1[0][3] =  can1_rx_data[1];
                            dtc1[0][4] =  can1_rx_data[2];
                            dtc1[1][0] =  can1_rx_data[3];
                            dtc1[1][1] =  can1_rx_data[4];
                            dtc1[1][2] =  can1_rx_data[5];
                            dtc1[1][3] =  can1_rx_data[6];
                            dtc1[1][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 4:
                        if(can1_rx_data[0] == 0x22)
                        {
                            dtc1[2][0] =  can1_rx_data[1];
                            dtc1[2][1] =  can1_rx_data[2];
                            dtc1[2][2] =  can1_rx_data[3];
                            dtc1[2][3] =  can1_rx_data[4];
                            dtc1[2][4] =  can1_rx_data[5];
                            dtc1[3][0] =  can1_rx_data[6];
                            dtc1[3][1] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 5:
                        if(can1_rx_data[0] == 0x23)
                        {
                            dtc1[3][2] =  can1_rx_data[1];
                            dtc1[3][3] =  can1_rx_data[2];
                            dtc1[3][4] =  can1_rx_data[3];
                            dtc1[4][0] =  can1_rx_data[4];
                            dtc1[4][1] =  can1_rx_data[5];
                            dtc1[4][2] =  can1_rx_data[6];
                            dtc1[4][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 6:
                        if(can1_rx_data[0] == 0x24)
                        {
                            dtc1[4][4] =  can1_rx_data[1];
                            dtc1[5][0] =  can1_rx_data[2];
                            dtc1[5][1] =  can1_rx_data[3];
                            dtc1[5][2] =  can1_rx_data[4];
                            dtc1[5][3] =  can1_rx_data[5];
                            dtc1[5][4] =  can1_rx_data[6];
                            dtc1[6][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 7:
                        if(can1_rx_data[0] == 0x25)
                        {
                            dtc1[6][1] =  can1_rx_data[1];
                            dtc1[6][2] =  can1_rx_data[2];
                            dtc1[6][3] =  can1_rx_data[3];
                            dtc1[6][4] =  can1_rx_data[4];
                            dtc1[7][0] =  can1_rx_data[5];
                            dtc1[7][1] =  can1_rx_data[6];
                            dtc1[7][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 8:
                        if(can1_rx_data[0] == 0x26)
                        {
                            dtc1[7][3] =  can1_rx_data[1];
                            dtc1[7][4] =  can1_rx_data[2];
                            dtc1[8][0] =  can1_rx_data[3];
                            dtc1[8][1] =  can1_rx_data[4];
                            dtc1[8][2] =  can1_rx_data[5];
                            dtc1[8][3] =  can1_rx_data[6];
                            dtc1[8][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 9:
                        if(can1_rx_data[0] == 0x27)
                        {
                            dtc1[9][0] =  can1_rx_data[1];
                            dtc1[9][1] =  can1_rx_data[2];
                            dtc1[9][2] =  can1_rx_data[3];
                            dtc1[9][3] =  can1_rx_data[4];
                            dtc1[9][4] =  can1_rx_data[5];
                            dtc1[10][0] = can1_rx_data[6];
                            dtc1[10][1] = can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 10:
                        if(can1_rx_data[0] == 0x28)
                        {
                            dtc1[10][2] =  can1_rx_data[1];
                            dtc1[10][3] =  can1_rx_data[2];
                            dtc1[10][4] =  can1_rx_data[3];
                            dtc1[11][0] =  can1_rx_data[4];
                            dtc1[11][1] =  can1_rx_data[5];
                            dtc1[11][2] =  can1_rx_data[6];
                            dtc1[11][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 11:
                        if(can1_rx_data[0] == 0x29)
                        {
                            dtc1[11][4] =  can1_rx_data[1];
                            dtc1[12][0] =  can1_rx_data[2];
                            dtc1[12][1] =  can1_rx_data[3];
                            dtc1[12][2] =  can1_rx_data[4];
                            dtc1[12][3] =  can1_rx_data[5];
                            dtc1[12][4] =  can1_rx_data[6];
                            dtc1[13][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 12:
                        if(can1_rx_data[0] == 0x2A)
                        {
                            dtc1[13][1] =  can1_rx_data[1];
                            dtc1[13][2] =  can1_rx_data[2];
                            dtc1[13][3] =  can1_rx_data[3];
                            dtc1[13][4] =  can1_rx_data[4];
                            dtc1[14][0] =  can1_rx_data[5];
                            dtc1[14][1] =  can1_rx_data[6];
                            dtc1[14][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 13:
                        if(can1_rx_data[0] == 0x2B)
                        {
                            dtc1[14][3] =  can1_rx_data[1];
                            dtc1[14][4] =  can1_rx_data[2];
                        }
                        ECU1_XCP_reply_index = 14;   // last read set to 14 for get_DTCs function

                        break;

                    default:

                        ECU1_XCP_reply_index = 0;

                        break;

                    } // end switch on DTC_XCP_rply_indx

                    break; // end case XCP_RESPONSE_ID

                    case XCP_EA4_REQUEST_ID:     // 0x712   canMESSAGE_BOX10 11bit identifier-- Trans XCP Request from EPS for data
                        break;

                    case XCP_EA4_RESPONSE_ID:     // 0x710   canMESSAGE_BOX11 11bit identifier-- Rec XCP Responce to EPS for data
                        break;

                    case TESTER_REC_DEBUG_ID:    // 0x7FF    canMESSAGE_BOX12 11bit identifier-- REC from Anyone

                        break;

                    case TESTER_TRANS_DEBUG_ID: // 0x7FE     canMESSAGE_BOX13 11bit identifier-- Transmit to Anyone

                        break;

                    case canMESSAGE_BOX14:      // 0x14  11bit identifier-- Not Used

                        break;

                    case canMESSAGE_BOX15:      // 0x15  11bit identifier-- Not Usedr

                        break;

                    case canMESSAGE_BOX16:      // 0x16  11bit identifier-- Not Used

                        break;

                    default:

                    {
                        can1_message_type_debug = messageBox;  // an unknown messageBox
                    }
                    break;

            }   // end switch(messageBox) for C1XX on canREG1
        }  // end case C1XX    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^    C 1 X X   C A N 1  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        break;

        case TARGET_FCA_ADAS:       //  VVVVVVVVVVVVVVVVVVVVVVVVVV  F C A    A D A S   C A N 1  VVVVVVVVVVVVVVVVVVVVVVVVV
        {
            switch(messageBox)
            {
            case FCA_PROD_MESS1_ID: // 0x148    canMESSAGE_BOX1 11bit identifier-- Rec Response from EPS (not used)
                break;

            case FCA_PROD_MESS2_ID: // 0x778    canMESSAGE_BOX2 11bit identifier-- Rec ON Star from EPS (not used)
                break;

            case FCA_CCP_Engine_Run:// 0x108    canMESSAGE_BOX3 11bit identifier-- Trans Engine ON to EPS
                break;

            case FCA_CCP_Power_Mode_ID:  // 0x1F1     canMESSAGE_BOX4 11bit identifier-- Trans Power Mode to EPS(not used)
                break;

            case FCA_CCP_SPEED_ID:  // 0x011C   canMESSAGE_BOX5 11bit identifier-- Trans Speed to EPS with counter and CRC
                break;

            case FCA_CCP_RESPONSE_ID: // 0x59A  canMESSAGE_BOX6 11bit identifier-- Rec CCP Responce from EPS for Data
                // recieve and decide what the REQ was, parse and store
                // Standard dump of all ccp request

                canGetData(canREG1, FCA_CCP_RESPONSE_ID, can1_rx_data); // get data, store in rx_data array

                switch(can1_request_index)      // requesting multiple vaiables using same messageBox, can1_request_index keeps track, store and parse in main
                {
                case 0:     // do nothing with request_connect data, setup and trans next request

                    canTransmit(canREG1, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_BattVltg_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 1:     // store ccp_FCA_BattVltg_str, setup and trans next request

                    memcpy(&can1_dump_err_data[1],&can1_rx_data,8);

                    canTransmit(canREG1, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_Temperature_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 2:     // store ccp_FCA_Temperature_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[2],&can1_rx_data,8);

                    canTransmit(canREG1, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_DigT1_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 3:     // store ccp_FCA_DigT1_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[3],&can1_rx_data,8);

                    canTransmit(canREG1, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_DigT2_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 4:     // store ccp_FCA_DigT2_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[4],&can1_rx_data,8);

                    canTransmit(canREG1, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_MtrCurrQax_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 5:     // store ccp_FCA_MtrCurrQax_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[5],&can1_rx_data,8);

                    canTransmit(canREG1, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_SystemState_byt);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 6:     // store ccp_FCA_SystemState_byt, setup and trans next reques

                    memcpy(&can1_dump_err_data[6],&can1_rx_data,8);

                    canTransmit(canREG1, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 7:     // store ccp_FCA_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[7],&can1_rx_data,8);

                    canTransmit(canREG1, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_Abs_Hw_Pos_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 8:     // store ccp_FCA_Abs_Hw_Pos_str, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can1_dump_err_data[8],&can1_rx_data,8);

                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;


                    //  ----------------   H c u r r    FCA reading cases 2000-2031 done during normal testing

                case 2000:      // Hcurr Qax readings(x3) // store can1_FCA_MtrCurrQax1_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[9],&can1_rx_data,8);

                    canTransmit(canREG1, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2001:      // store can1_FCA_HwTrq_HwNm1_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[10],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2010:      // store can1_FCA_MtrCurrQax2_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[11],&can1_rx_data,8);

                    canTransmit(canREG1, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2011:      // store can1_FCA_HwTrq_HwNm2_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[12],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2020:      // store can1_FCA_MtrCurrQax3_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[13],&can1_rx_data,8);

                    canTransmit(canREG1, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2021:      // store can1_FCA_HwTrq_HwNm3_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[14],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2999:

                    can1_request_index = 3000;  // don't care response is done...ok to send new request.

                    break;

                default:
                {
                }

                }   // end switch(can1_request_index)

                break;  // end case CCP_RESPONSE_ID:

                case FCA_CCP_REQUEST_ID:        // 0x708    canMESSAGE_BOX7 11bit identifier-- Trans CCP Request for Data to EPS
                    break;

                case FCA_XCP_EA3_REQUEST_ID:         // 0x242   canMESSAGE_BOX8 11bit identifier-- Trans CCP Request for Data to EPS
                    break;

                case FCA_XCP_EA3_RESPONSE_ID:        // 0x642   canMESSAGE_BOX9 11bit identifier-- Rec XCP Responce from EPS for data
                    // receive and decide what the REQ was, parse and store

                    canGetData(canREG1, FCA_XCP_EA3_RESPONSE_ID, can1_rx_data); // get data, store in rx_data array

                    switch(ECU1_XCP_reply_index)    // requesting multiple requests using same messageBox, DCT_XCP_rply_indx keeps track, clear_dtcs_flag determines action
                    {

                    case 1:

                        if((can1_rx_data[1] == 0x50) && (can1_rx_data[2] == 0x7E))  // positive responce from enter Nexteer mode for FCA
                        {
                            if(ECU1_clear_DTCs_flag == 1)         // clear DTC's mode
                            {
                                canTransmit(canREG1, FCA_XCP_EA3_REQUEST_ID, ( uint8 *) xcp_EA3_write_clear_DTC_rqst1); // request clear DTCs
                            }

                            else        // get DTC's mode

                            {
                                canTransmit(canREG1, FCA_XCP_EA3_REQUEST_ID, ( uint8 *) xcp_EA3_write_get_DTC_rqst1);   // request DTCs first string
                            }

                            ECU1_XCP_reply_index++;
                        }  // end positive responce nexteer mode
                        break;

                    case 2:

                        if((can1_rx_data[0] == 0x10) && (can1_rx_data[2] == 0x62))    // get DTC's mode
                        {

                            dtc1[0][0] =  can1_rx_data[5];
                            dtc1[0][1] =  can1_rx_data[6];
                            dtc1[0][2] =  can1_rx_data[7];

                            canTransmit(canREG1, FCA_XCP_EA3_REQUEST_ID, ( uint8 *) xcp_EA3_write_get_DTC_rqst2);   // request DTCs 2nd string

                            ECU1_XCP_reply_index++;
                        }

                        if((can1_rx_data[0] == 0x04) && (can1_rx_data[1] == 0x71))  // responce to clear DTC's mode
                        {
                            if(ECU1_clear_DTCs_flag == 1)
                            {
                                ECU1_XCP_reply_index = 0;
                                ECU1_clear_DTCs_flag = 0;   // reset flag and index, done
                            }
                        }

                        break;

                    case 3:
                        if(can1_rx_data[0] == 0x21)
                        {
                            dtc1[0][3] =  can1_rx_data[1];
                            dtc1[0][4] =  can1_rx_data[2];
                            dtc1[1][0] =  can1_rx_data[3];
                            dtc1[1][1] =  can1_rx_data[4];
                            dtc1[1][2] =  can1_rx_data[5];
                            dtc1[1][3] =  can1_rx_data[6];
                            dtc1[1][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 4:
                        if(can1_rx_data[0] == 0x22)
                        {
                            dtc1[2][0] =  can1_rx_data[1];
                            dtc1[2][1] =  can1_rx_data[2];
                            dtc1[2][2] =  can1_rx_data[3];
                            dtc1[2][3] =  can1_rx_data[4];
                            dtc1[2][4] =  can1_rx_data[5];
                            dtc1[3][0] =  can1_rx_data[6];
                            dtc1[3][1] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 5:
                        if(can1_rx_data[0] == 0x23)
                        {
                            dtc1[3][2] =  can1_rx_data[1];
                            dtc1[3][3] =  can1_rx_data[2];
                            dtc1[3][4] =  can1_rx_data[3];
                            dtc1[4][0] =  can1_rx_data[4];
                            dtc1[4][1] =  can1_rx_data[5];
                            dtc1[4][2] =  can1_rx_data[6];
                            dtc1[4][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 6:
                        if(can1_rx_data[0] == 0x24)
                        {
                            dtc1[4][4] =  can1_rx_data[1];
                            dtc1[5][0] =  can1_rx_data[2];
                            dtc1[5][1] =  can1_rx_data[3];
                            dtc1[5][2] =  can1_rx_data[4];
                            dtc1[5][3] =  can1_rx_data[5];
                            dtc1[5][4] =  can1_rx_data[6];
                            dtc1[6][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 7:
                        if(can1_rx_data[0] == 0x25)
                        {
                            dtc1[6][1] =  can1_rx_data[1];
                            dtc1[6][2] =  can1_rx_data[2];
                            dtc1[6][3] =  can1_rx_data[3];
                            dtc1[6][4] =  can1_rx_data[4];
                            dtc1[7][0] =  can1_rx_data[5];
                            dtc1[7][1] =  can1_rx_data[6];
                            dtc1[7][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 8:
                        if(can1_rx_data[0] == 0x26)
                        {
                            dtc1[7][3] =  can1_rx_data[1];
                            dtc1[7][4] =  can1_rx_data[2];
                            dtc1[8][0] =  can1_rx_data[3];
                            dtc1[8][1] =  can1_rx_data[4];
                            dtc1[8][2] =  can1_rx_data[5];
                            dtc1[8][3] =  can1_rx_data[6];
                            dtc1[8][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 9:
                        if(can1_rx_data[0] == 0x27)
                        {
                            dtc1[9][0] =  can1_rx_data[1];
                            dtc1[9][1] =  can1_rx_data[2];
                            dtc1[9][2] =  can1_rx_data[3];
                            dtc1[9][3] =  can1_rx_data[4];
                            dtc1[9][4] =  can1_rx_data[5];
                            dtc1[10][0] = can1_rx_data[6];
                            dtc1[10][1] = can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 10:
                        if(can1_rx_data[0] == 0x28)
                        {
                            dtc1[10][2] =  can1_rx_data[1];
                            dtc1[10][3] =  can1_rx_data[2];
                            dtc1[10][4] =  can1_rx_data[3];
                            dtc1[11][0] =  can1_rx_data[4];
                            dtc1[11][1] =  can1_rx_data[5];
                            dtc1[11][2] =  can1_rx_data[6];
                            dtc1[11][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 11:
                        if(can1_rx_data[0] == 0x29)
                        {
                            dtc1[11][4] =  can1_rx_data[1];
                            dtc1[12][0] =  can1_rx_data[2];
                            dtc1[12][1] =  can1_rx_data[3];
                            dtc1[12][2] =  can1_rx_data[4];
                            dtc1[12][3] =  can1_rx_data[5];
                            dtc1[12][4] =  can1_rx_data[6];
                            dtc1[13][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 12:
                        if(can1_rx_data[0] == 0x2A)
                        {
                            dtc1[13][1] =  can1_rx_data[1];
                            dtc1[13][2] =  can1_rx_data[2];
                            dtc1[13][3] =  can1_rx_data[3];
                            dtc1[13][4] =  can1_rx_data[4];
                            dtc1[14][0] =  can1_rx_data[5];
                            dtc1[14][1] =  can1_rx_data[6];
                            dtc1[14][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 13:
                        if(can1_rx_data[0] == 0x2B)
                        {
                            dtc1[14][3] =  can1_rx_data[1];
                            dtc1[14][4] =  can1_rx_data[2];
                        }
                        ECU1_XCP_reply_index = 14;   // last read set to 14 for get_DTCs function
                        break;

                    default:

                        ECU1_XCP_reply_index = 0;

                        break;

                    } // end switch on DTC_XCP_rply_indx

                    break; // end case XCP_RESPONSE_ID

                    case FCA_XCP_EA4_REQUEST_ID: // 0x712   canMESSAGE_BOX10 11bit identifier-- Trans XCP Request from EPS for data
                        break;

                    case FCA_XCP_EA4_RESPONSE_ID: // 0x710   canMESSAGE_BOX11 11bit identifier-- Rec XCP Responce to EPS for data
                        break;

                    case FCA_TESTER_REC_DEBUG_ID:    // 0x7FF    canMESSAGE_BOX12 11bit identifier-- REC from Anyone

                        break;

                    case FCA_TESTER_TRANS_DEBUG_ID:    // 0x7FE     canMESSAGE_BOX13 11bit identifier-- Transmit to Anyone

                        break;

                    case canMESSAGE_BOX14:      // 0x14  11bit identifier-- Not Used

                        break;

                    case canMESSAGE_BOX15:      // 0x15  11bit identifier-- Not Usedr

                        break;

                    case canMESSAGE_BOX16:      // 0x16  11bit identifier-- Not Used

                        break;

                    default:

                    {
                        can1_message_type_debug = messageBox;  // an unknown messageBox
                    }
                    break;

            }   // end switch(messageBox) for FCA ADAS on canREG1

        }  // end case FCA ADAS    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^    F C A   A D A S   C A N 1 ^^^^^^^^^^^^^^^^^^^^^^^^^^^
        break;

        case TARGET_CD391_ADAS:     //  VVVVVVVVVVVVVVVVVVVVVVVVVV  F O R D     A D A S   C A N 1  VVVVVVVVVVVVVVVVVVVVVVVVV
        {
            switch(messageBox)
            {
            case FORD_PROD_MESS1_ID: // 0x148   canMESSAGE_BOX1 11bit identifier-- Rec Response from EPS (not used)
                break;

            case FORD_PROD_MESS2_ID: // 0x778   canMESSAGE_BOX2 11bit identifier-- Rec ON Star from EPS (not used)
                break;

            case FORD_CCP_Engine_Run:// 0x108   canMESSAGE_BOX3 11bit identifier-- Trans Engine ON to EPS
                break;

            case FORD_CCP_Power_Mode_ID:  // 0x1F1    canMESSAGE_BOX4 11bit identifier-- Trans Power Mode to EPS(not used)
                break;

            case FORD_CCP_SPEED_ID:  // 0x011C   canMESSAGE_BOX5    11bit identifier-- Trans Speed to EPS with counter and CRC
                break;

            case FORD_CCP_RESPONSE_ID: // 0x59A canMESSAGE_BOX6 11bit identifier-- Rec CCP Responce from EPS for Data
                // recieve and decide what the REQ was, parse and store
                // Standard dump of all ccp request

                canGetData(canREG1, FORD_CCP_RESPONSE_ID, can1_rx_data);    // get data, store in rx_data array

                switch(can1_request_index)      // requesting multiple vaiables using same messageBox, can1_request_index keeps track, store and parse in main
                {
                case 0:     // do nothing with request_connect data, setup and trans next request

                    canTransmit(canREG1, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_BattVltg_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 1:     // store ccp_FORD_BattVltg_str, setup and trans next request

                    memcpy(&can1_dump_err_data[1],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_Temperature_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 2:     // store ccp_FORD_Temperature_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[2],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_DigT1_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 3:     // store ccp_FORD_DigT1_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[3],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_DigT2_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 4:     // store ccp_FORD_DigT2_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[4],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_MtrCurrQax_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 5:     // store ccp_FORD_MtrCurrQax_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[5],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_SystemState_byt);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 6:     // store ccp_FORD_SystemState_byt, setup and trans next reques

                    memcpy(&can1_dump_err_data[6],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 7:     // store ccp_FORD_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[7],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_Abs_Hw_Pos_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 8:     // store ccp_FORD_Abs_Hw_Pos_str, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can1_dump_err_data[8],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_Batt_Current_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 9:     // store ccp_FORD_Batt_Current_str, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can1_dump_err_data[9],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_LatchFail_ISR_u16);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 10:        // store ccp_FORD_LatchFail_ISR_u16, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can1_dump_err_data[10],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_MtrCntl_ISR_u16);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 11:        // store ccp_FORD_MtrCntl_ISR_u16, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can1_dump_err_data[11],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_MinCount_u32);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 12:        // store ccp_FORD_MinCount_u32, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can1_dump_err_data[12],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_MaxCount_u32);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 13:        // store ccp_FORD_MaxCount_u32, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can1_dump_err_data[13],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_CatGate_cnt);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 14:        // store ccp_FORD_CatGate_cnt, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can1_dump_err_data[14],&can1_rx_data,8);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;


                    //  ----------------   Low - Med - High     FORD reading cases 2000-2031 done during normal testing

                case 2000:      //  Qax readings(x3) // store can1_FORD_MtrCurrQax1_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[15],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2001:      // store can1_FORD_HwTrq_HwNm1_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[16],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2010:      // store can1_FORD_MtrCurrQax2_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[17],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2011:      // store can1_FORD_HwTrq_HwNm2_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[18],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2020:      // store can1_FORD_MtrCurrQax3_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[19],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2021:      // store can1_FORD_HwTrq_HwNm3_fp, setup and trans next reques

                    memcpy(&can1_dump_err_data[20],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2999:

                    can1_request_index = 3000;  // don't care response is done...ok to send new request.

                    break;

                default:
                {
                }

                }   // end switch(can1_request_index)

                break;  // end case CCP_RESPONSE_ID:

                case FORD_CCP_REQUEST_ID:       // 0x708    canMESSAGE_BOX7 11bit identifier-- Trans CCP Request for Data to EPS
                    break;

                case FORD_XCP_EA3_REQUEST_ID:        // 0x242   canMESSAGE_BOX8 11bit identifier-- Trans CCP Request for Data to EPS
                    break;

                case FORD_XCP_EA3_RESPONSE_ID:       // 0x642   canMESSAGE_BOX9 11bit identifier-- Rec XCP Responce from EPS for data
                    // receive and decide what the REQ was, parse and store

                    canGetData(canREG1, FORD_XCP_EA3_RESPONSE_ID, can1_rx_data);    // get data, store in rx_data array

                    switch(ECU1_XCP_reply_index)    // requesting multiple requests using same messageBox, DCT_XCP_rply_indx keeps track, clear_dtcs_flag determines action
                    {

                    case 1:

                        if((can1_rx_data[1] == 0x50) && (can1_rx_data[2] == 0x7E))  // positive response from enter Nexteer mode for FCA
                        {
                            if(ECU1_clear_DTCs_flag == 1)         // clear DTC's mode
                            {
                                canTransmit(canREG1, FORD_XCP_EA3_REQUEST_ID, ( uint8 *) xcp_EA3_write_clear_DTC_rqst1);    // request clear DTCs
                            }

                            else        // get DTC's mode

                            {
                                canTransmit(canREG1, FORD_XCP_EA3_REQUEST_ID, ( uint8 *) xcp_EA3_write_get_DTC_rqst1);  // request DTCs first string
                            }

                            ECU1_XCP_reply_index++;
                        }  // end positive response nexteer mode
                        break;

                    case 2:

                        if((can1_rx_data[0] == 0x10) && (can1_rx_data[2] == 0x62))    // get DTC's mode
                        {

                            dtc1[0][0] =  can1_rx_data[5];
                            dtc1[0][1] =  can1_rx_data[6];
                            dtc1[0][2] =  can1_rx_data[7];

                            canTransmit(canREG1, FORD_XCP_EA3_REQUEST_ID, ( uint8 *) xcp_EA3_write_get_DTC_rqst2);  // request DTCs 2nd string

                            ECU1_XCP_reply_index++;
                        }

                        if((can1_rx_data[0] == 0x04) && (can1_rx_data[1] == 0x71))  // responce to clear DTC's mode
                        {
                            if(ECU1_clear_DTCs_flag == 1)
                            {
                                ECU1_XCP_reply_index = 0;
                                ECU1_clear_DTCs_flag = 0;   // reset flag and index, done
                            }
                        }

                        break;

                    case 3:
                        if(can1_rx_data[0] == 0x21)
                        {
                            dtc1[0][3] =  can1_rx_data[1];
                            dtc1[0][4] =  can1_rx_data[2];
                            dtc1[1][0] =  can1_rx_data[3];
                            dtc1[1][1] =  can1_rx_data[4];
                            dtc1[1][2] =  can1_rx_data[5];
                            dtc1[1][3] =  can1_rx_data[6];
                            dtc1[1][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 4:
                        if(can1_rx_data[0] == 0x22)
                        {
                            dtc1[2][0] =  can1_rx_data[1];
                            dtc1[2][1] =  can1_rx_data[2];
                            dtc1[2][2] =  can1_rx_data[3];
                            dtc1[2][3] =  can1_rx_data[4];
                            dtc1[2][4] =  can1_rx_data[5];
                            dtc1[3][0] =  can1_rx_data[6];
                            dtc1[3][1] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 5:
                        if(can1_rx_data[0] == 0x23)
                        {
                            dtc1[3][2] =  can1_rx_data[1];
                            dtc1[3][3] =  can1_rx_data[2];
                            dtc1[3][4] =  can1_rx_data[3];
                            dtc1[4][0] =  can1_rx_data[4];
                            dtc1[4][1] =  can1_rx_data[5];
                            dtc1[4][2] =  can1_rx_data[6];
                            dtc1[4][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 6:
                        if(can1_rx_data[0] == 0x24)
                        {
                            dtc1[4][4] =  can1_rx_data[1];
                            dtc1[5][0] =  can1_rx_data[2];
                            dtc1[5][1] =  can1_rx_data[3];
                            dtc1[5][2] =  can1_rx_data[4];
                            dtc1[5][3] =  can1_rx_data[5];
                            dtc1[5][4] =  can1_rx_data[6];
                            dtc1[6][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 7:
                        if(can1_rx_data[0] == 0x25)
                        {
                            dtc1[6][1] =  can1_rx_data[1];
                            dtc1[6][2] =  can1_rx_data[2];
                            dtc1[6][3] =  can1_rx_data[3];
                            dtc1[6][4] =  can1_rx_data[4];
                            dtc1[7][0] =  can1_rx_data[5];
                            dtc1[7][1] =  can1_rx_data[6];
                            dtc1[7][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 8:
                        if(can1_rx_data[0] == 0x26)
                        {
                            dtc1[7][3] =  can1_rx_data[1];
                            dtc1[7][4] =  can1_rx_data[2];
                            dtc1[8][0] =  can1_rx_data[3];
                            dtc1[8][1] =  can1_rx_data[4];
                            dtc1[8][2] =  can1_rx_data[5];
                            dtc1[8][3] =  can1_rx_data[6];
                            dtc1[8][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 9:
                        if(can1_rx_data[0] == 0x27)
                        {
                            dtc1[9][0] =  can1_rx_data[1];
                            dtc1[9][1] =  can1_rx_data[2];
                            dtc1[9][2] =  can1_rx_data[3];
                            dtc1[9][3] =  can1_rx_data[4];
                            dtc1[9][4] =  can1_rx_data[5];
                            dtc1[10][0] = can1_rx_data[6];
                            dtc1[10][1] = can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 10:
                        if(can1_rx_data[0] == 0x28)
                        {
                            dtc1[10][2] =  can1_rx_data[1];
                            dtc1[10][3] =  can1_rx_data[2];
                            dtc1[10][4] =  can1_rx_data[3];
                            dtc1[11][0] =  can1_rx_data[4];
                            dtc1[11][1] =  can1_rx_data[5];
                            dtc1[11][2] =  can1_rx_data[6];
                            dtc1[11][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 11:
                        if(can1_rx_data[0] == 0x29)
                        {
                            dtc1[11][4] =  can1_rx_data[1];
                            dtc1[12][0] =  can1_rx_data[2];
                            dtc1[12][1] =  can1_rx_data[3];
                            dtc1[12][2] =  can1_rx_data[4];
                            dtc1[12][3] =  can1_rx_data[5];
                            dtc1[12][4] =  can1_rx_data[6];
                            dtc1[13][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 12:

                        if(can1_rx_data[0] == 0x2A)
                        {
                            dtc1[13][1] =  can1_rx_data[1];
                            dtc1[13][2] =  can1_rx_data[2];
                            dtc1[13][3] =  can1_rx_data[3];
                            dtc1[13][4] =  can1_rx_data[4];
                            dtc1[14][0] =  can1_rx_data[5];
                            dtc1[14][1] =  can1_rx_data[6];
                            dtc1[14][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 13:

                        if(can1_rx_data[0] == 0x2B)
                        {
                            dtc1[14][3] =  can1_rx_data[1];
                            dtc1[14][4] =  can1_rx_data[2];
                        }
                        ECU1_XCP_reply_index = 14;   // last read set to 14 for get_DTCs function
                        break;

                    default:

                        ECU1_XCP_reply_index = 0;

                        break;

                    } // end switch on DTC_XCP_rply_indx

                    break; // end case XCP_RESPONSE_ID

                    case FORD_XCP_EA4_REQUEST_ID: // 0x712  canMESSAGE_BOX10 11bit identifier-- Trans XCP Request from EPS for data
                        break;

                    case FORD_XCP_EA4_RESPONSE_ID: // 0x710  canMESSAGE_BOX11 11bit identifier-- Rec XCP Responce to EPS for data
                        break;

                    case FORD_TESTER_REC_DEBUG_ID:   // 0x7FF    canMESSAGE_BOX12 11bit identifier-- REC from Anyone

                        break;

                    case FORD_TESTER_TRANS_DEBUG_ID:    // 0x7FE     canMESSAGE_BOX13 11bit identifier-- Transmit to Anyone

                        break;

                    case canMESSAGE_BOX14:      // 0x14  11bit identifier-- Not Used

                        break;

                    case canMESSAGE_BOX15:      // 0x15  11bit identifier-- Not Usedr

                        break;

                    case canMESSAGE_BOX16:      // 0x16  11bit identifier-- Not Used

                        break;

                    default:

                    {
                        can1_message_type_debug = messageBox;  // an unknown messageBox
                    }
                    break;

            }   // end switch(messageBox) for FORD ADAS on canREG1

        }  // end case TARGET_CD391_ADAS    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^    F O R D    A D A S   C A N 1 ^^^^^^^^^^^^^^^^^^^^^^^^^^^
        break;

        case TARGET_G2KCA_ADAS:     //  VVVVVVVVVVVVVVVVVVVVVVVVVV  G 2 K C A    A D A S   C A N 1  VVVVVVVVVVVVVVVVVVVVVVVVV
        {
            switch(messageBox)
            {
            case G2KCA_ECU1_CAN1_PROD_MESS1_ID:         // 0x148    canMESSAGE_BOX1 11bit identifier-- Rec Response from EPS (not used)
                break;

            case G2KCA_ECU1_CAN1_PROD_MESS2_ID:         // 0x778    canMESSAGE_BOX2 11bit identifier-- Rec ON Star from EPS (not used)
                break;

            case G2KCA_ECU1_CAN1_CCP_Engine_Run:        // 0x108    canMESSAGE_BOX3 11bit identifier-- Trans Engine ON to EPS
                break;

            case G2KCA_ECU1_CAN1_CCP_Power_Mode_ID:     // 0x1F1      canMESSAGE_BOX4 11bit identifier-- Trans Power Mode to EPS
                break;

            case G2KCA_ECU1_CAN1_CCP_SPEED_ID:          // 0x03E9   canMESSAGE_BOX5 11bit identifier-- Trans Speed to EPS with counter and CRC (not used)
                break;

            case G2KCA_ECU1_CAN1_CCP_Driven_ID:         // 0x348    canMESSAGE_BOX6 11bit identifier-- Trans Wheel 1 Info to EPS
                break;

            case G2KCA_ECU1_CAN1_CCP_NonDriven_ID:      // 0x34A    canMESSAGE_BOX7 11bit identifier-- Trans Wheel 2 Info to EPS
                break;

            case G2KCA_ECU1_CAN1_CCP_REQUEST_ID:        // 0x708    canMESSAGE_BOX8 11bit identifier-- Trans CCP Request for Data to EPS
                break;

            case G2KCA_ECU1_CAN1_CCP_RESPONSE_ID:       // 0x706    canMESSAGE_BOX9 11bit identifier-- Rec CCP Responce from EPS for Data
                // recieve and decide what the REQ was, parse and store
                // Standard dump of all ccp request

                canGetData(canREG1, G2KCA_ECU1_CAN1_CCP_RESPONSE_ID, can1_rx_data); // get data, store in rx_data array

                switch(can1_request_index)      // requesting multiple vaiables using same messageBox, can1_request_index keeps track, store and parse in main
                {
                case 0:     // do nothing with request_connect data, setup and trans next request

                    canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_BattVltg_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 1:     // store ccp_G2KCA_BattVltg_str, setup and trans next request

                    memcpy(&can1_dump_err_data[1],&can1_rx_data,8);

                    canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_Temperature_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 2:     // store ccp_G2KCA_Temperature_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[2],&can1_rx_data,8);

                    canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_DigT1_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 3:     // store ccp_G2KCA_DigT1_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[3],&can1_rx_data,8);

                    canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_DigT2_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 4:     // store ccp_G2KCA_DigT2_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[4],&can1_rx_data,8);

                    canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_MtrCurrQax_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 5:     // store ccp_G2KCA_MtrCurrQax_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[5],&can1_rx_data,8);

                    canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_SystemState_byt);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 6:     // store ccp_G2KCA_SystemState_byt, setup and trans next reques

                    memcpy(&can1_dump_err_data[6],&can1_rx_data,8);

                    canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 7:     // store ccp_G2KCA_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[7],&can1_rx_data,8);

                    canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_Abs_Hw_Pos_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 8:     // store ccp_G2KCA_Abs_Hw_Pos_str, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can1_dump_err_data[8],&can1_rx_data,8);

                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;


                    //  ----------------   H c u r r    FCA reading cases 2000-2031 done during normal testing

                case 2000:      // Hcurr Qax readings(x3) // store ccp_G2KCA_MtrCurrQax_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[9],&can1_rx_data,8);

                    canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2001:      // store ccp_G2KCA_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[10],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2010:      // store ccp_G2KCA_MtrCurrQax_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[11],&can1_rx_data,8);

                    canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2011:      // store ccp_G2KCA_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[12],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2020:      // store ccp_G2KCA_MtrCurrQax_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[13],&can1_rx_data,8);

                    canTransmit(canREG1, G2KCA_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2021:      // store ccp_G2KCA_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[14],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2999:

                    can1_request_index = 3000;  // don't care response is done...ok to send new request.

                    break;

                default:
                {
                }

                }   // end switch(can1_request_index)

                break;  // end case CCP_RESPONSE_ID:

                case G2KCA_ECU1_CAN1_XCP_EA3_REQUEST_ID:    // 0x242    canMESSAGE_BOX10 11bit identifier-- Trans XCP Request from EPS for data (not used)
                    break;

                case G2KCA_ECU1_CAN1_XCP_EA3_RESPONSE_ID:   // 0x642    canMESSAGE_BOX11 11bit identifier-- Rec XCP Responce from EPS for data (not used)
                    break;


                case G2KCA_ECU1_CAN1_XCP_EA4_REQUEST_ID:    // 0x712    canMESSAGE_BOX12 11bit identifier-- Trans XCP Request from EPS for data
                    break;

                case G2KCA_ECU1_CAN1_XCP_EA4_RESPONSE_ID:   // 0x642    canMESSAGE_BOX13 11bit identifier-- Rec XCP Responce from EPS for data
                    // receive and decide what the REQ was, parse and store
                    canGetData(canREG1, G2KCA_ECU1_CAN1_XCP_EA4_RESPONSE_ID, can1_rx_data); // get data, store in rx_data array

                    switch(ECU1_XCP_reply_index)    // requesting multiple requests using same messageBox, DCT_XCP_rply_indx keeps track, clear_dtcs_flag determines action
                    {

                    case 1:
                        //EA4 is different
                        if((can1_rx_data[1] == 0x50) && (can1_rx_data[2] == 0x7E))  // positive responce from enter Nexteer mode for FCA
                        {
                            if(ECU1_clear_DTCs_flag == 1)         // clear DTC's mode
                            {
                                canTransmit(canREG1, G2KCA_ECU1_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_clear_DTC_rqst1); // request clear DTCs
                            }
                            else        // get DTC's mode
                            {
                                canTransmit(canREG1, G2KCA_ECU1_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_get_DTC_rqst1);   // request DTCs first string
                            }

                            ECU1_XCP_reply_index++;
                        }  // end positive responce nexteer mode
                        break;

                    case 2:

                        if((can1_rx_data[0] == 0x10) && (can1_rx_data[2] == 0x62))    // get DTC's mode
                        {
                            dtc1[0][0] =  can1_rx_data[5];
                            dtc1[0][1] =  can1_rx_data[6];
                            dtc1[0][2] =  can1_rx_data[7];

                            canTransmit(canREG1, G2KCA_ECU1_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_get_DTC_rqst2);   // request DTCs 2nd string

                            ECU1_XCP_reply_index++;
                        }

                        if((can1_rx_data[0] == 0x04) && (can1_rx_data[1] == 0x71))  // responce to clear DTC's mode
                        {
                            if(ECU1_clear_DTCs_flag == 1)
                            {
                                ECU1_XCP_reply_index = 0;
                                ECU1_clear_DTCs_flag = 0;   // reset flag and index, done
                            }
                        }

                        break;

                    case 3:
                        if(can1_rx_data[0] == 0x21)
                        {
                            dtc1[0][3] =  can1_rx_data[1];
                            dtc1[0][4] =  can1_rx_data[2];
                            dtc1[1][0] =  can1_rx_data[3];
                            dtc1[1][1] =  can1_rx_data[4];
                            dtc1[1][2] =  can1_rx_data[5];
                            dtc1[1][3] =  can1_rx_data[6];
                            dtc1[1][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 4:
                        if(can1_rx_data[0] == 0x22)
                        {
                            dtc1[2][0] =  can1_rx_data[1];
                            dtc1[2][1] =  can1_rx_data[2];
                            dtc1[2][2] =  can1_rx_data[3];
                            dtc1[2][3] =  can1_rx_data[4];
                            dtc1[2][4] =  can1_rx_data[5];
                            dtc1[3][0] =  can1_rx_data[6];
                            dtc1[3][1] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 5:
                        if(can1_rx_data[0] == 0x23)
                        {
                            dtc1[3][2] =  can1_rx_data[1];
                            dtc1[3][3] =  can1_rx_data[2];
                            dtc1[3][4] =  can1_rx_data[3];
                            dtc1[4][0] =  can1_rx_data[4];
                            dtc1[4][1] =  can1_rx_data[5];
                            dtc1[4][2] =  can1_rx_data[6];
                            dtc1[4][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 6:
                        if(can1_rx_data[0] == 0x24)
                        {
                            dtc1[4][4] =  can1_rx_data[1];
                            dtc1[5][0] =  can1_rx_data[2];
                            dtc1[5][1] =  can1_rx_data[3];
                            dtc1[5][2] =  can1_rx_data[4];
                            dtc1[5][3] =  can1_rx_data[5];
                            dtc1[5][4] =  can1_rx_data[6];
                            dtc1[6][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 7:
                        if(can1_rx_data[0] == 0x25)
                        {
                            dtc1[6][1] =  can1_rx_data[1];
                            dtc1[6][2] =  can1_rx_data[2];
                            dtc1[6][3] =  can1_rx_data[3];
                            dtc1[6][4] =  can1_rx_data[4];
                            dtc1[7][0] =  can1_rx_data[5];
                            dtc1[7][1] =  can1_rx_data[6];
                            dtc1[7][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 8:
                        if(can1_rx_data[0] == 0x26)
                        {
                            dtc1[7][3] =  can1_rx_data[1];
                            dtc1[7][4] =  can1_rx_data[2];
                            dtc1[8][0] =  can1_rx_data[3];
                            dtc1[8][1] =  can1_rx_data[4];
                            dtc1[8][2] =  can1_rx_data[5];
                            dtc1[8][3] =  can1_rx_data[6];
                            dtc1[8][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 9:
                        if(can1_rx_data[0] == 0x27)
                        {
                            dtc1[9][0] =  can1_rx_data[1];
                            dtc1[9][1] =  can1_rx_data[2];
                            dtc1[9][2] =  can1_rx_data[3];
                            dtc1[9][3] =  can1_rx_data[4];
                            dtc1[9][4] =  can1_rx_data[5];
                            dtc1[10][0] = can1_rx_data[6];
                            dtc1[10][1] = can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 10:
                        if(can1_rx_data[0] == 0x28)
                        {
                            dtc1[10][2] =  can1_rx_data[1];
                            dtc1[10][3] =  can1_rx_data[2];
                            dtc1[10][4] =  can1_rx_data[3];
                            dtc1[11][0] =  can1_rx_data[4];
                            dtc1[11][1] =  can1_rx_data[5];
                            dtc1[11][2] =  can1_rx_data[6];
                            dtc1[11][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 11:
                        if(can1_rx_data[0] == 0x29)
                        {
                            dtc1[11][4] =  can1_rx_data[1];
                            dtc1[12][0] =  can1_rx_data[2];
                            dtc1[12][1] =  can1_rx_data[3];
                            dtc1[12][2] =  can1_rx_data[4];
                            dtc1[12][3] =  can1_rx_data[5];
                            dtc1[12][4] =  can1_rx_data[6];
                            dtc1[13][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 12:
                        if(can1_rx_data[0] == 0x2A)
                        {
                            dtc1[13][1] =  can1_rx_data[1];
                            dtc1[13][2] =  can1_rx_data[2];
                            dtc1[13][3] =  can1_rx_data[3];
                            dtc1[13][4] =  can1_rx_data[4];
                            dtc1[14][0] =  can1_rx_data[5];
                            dtc1[14][1] =  can1_rx_data[6];
                            dtc1[14][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 13:
                        if(can1_rx_data[0] == 0x2B)
                        {
                            dtc1[14][3] =  can1_rx_data[1];
                            dtc1[14][4] =  can1_rx_data[2];

                            dtc1[14][3] =  can1_rx_data[1];
                            dtc1[14][4] =  can1_rx_data[2];
                            dtc1[15][0] =  can1_rx_data[3];
                            dtc1[15][1] =  can1_rx_data[4];
                            dtc1[15][2] =  can1_rx_data[5];
                            dtc1[15][3] =  can1_rx_data[6];
                            dtc1[15][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index = 14;
                        break;

                    case 14:
                        if(can1_rx_data[0] == 0x2C)
                        {
                            dtc1[16][0] =  can1_rx_data[1];
                            dtc1[16][1] =  can1_rx_data[2];
                            dtc1[16][2] =  can1_rx_data[3];
                            dtc1[16][3] =  can1_rx_data[4];
                            dtc1[16][4] =  can1_rx_data[5];
                            dtc1[17][0] =  can1_rx_data[6];
                            dtc1[17][1] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 15:
                        if(can1_rx_data[0] == 0x2D)
                        {
                            dtc1[17][2] =  can1_rx_data[1];
                            dtc1[17][3] =  can1_rx_data[2];
                            dtc1[17][4] =  can1_rx_data[3];
                            dtc1[18][0] =  can1_rx_data[4];
                            dtc1[18][1] =  can1_rx_data[5];
                            dtc1[18][2] =  can1_rx_data[6];
                            dtc1[18][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 16:
                        if(can1_rx_data[0] == 0x2E)
                        {
                            dtc1[18][4] =  can1_rx_data[1];
                            dtc1[19][0] =  can1_rx_data[2];
                            dtc1[19][1] =  can1_rx_data[3];
                            dtc1[19][2] =  can1_rx_data[4];
                            dtc1[19][3] =  can1_rx_data[5];
                            dtc1[19][4] =  can1_rx_data[6];
                        }
                        ECU1_XCP_reply_index = 17;  // last read set to 17 for get_DTCs function
                        break;

                    case 100:
                        ECU1_XCP_reply_index = 101;     // case used for multi message for TOC
                        break;

                    case 101:
                        ECU1_XCP_reply_index = 102;     // case used for multi message for TOC
                        break;

                    default:
                        ECU1_XCP_reply_index = 0;
                        break;

                    } // end switch on DTC_XCP_rply_indx

                    break; // end case G2KCA_ECU1_CAN1_XCP_EA4_RESPONSE_ID

                    case G2KCA_ECU2_CAN1_CCP_REQUEST_ID:        // 0x70A    canMESSAGE_BOX14    11bit identifier-- Trans CCP Request for Data to EPS
                        break;

                    case G2KCA_ECU2_CAN1_CCP_RESPONSE_ID:       // 0x70C    canMESSAGE_BOX15    11bit identifier-- Rec CCP Responce from EPS for Data
                        // recieve and decide what the REQ was, parse and store
                        // Standard dump of all ccp request

                        canGetData(canREG1, G2KCA_ECU2_CAN1_CCP_RESPONSE_ID, can2_rx_data); // get data, store in rx_data array

                        switch(can2_request_index)      // requesting multiple vaiables using same messageBox, can2_request_index keeps track, store and parse in main
                        {
                        case 0:     // do nothing with request_connect data, setup and trans next request

                            canTransmit(canREG1, G2KCA_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_BattVltg_str);
                            can2_request_index++;       // bump index so next receive will switch to correct message
                            break;

                        case 1:     // store ccp_G2KCA_BattVltg_str, setup and trans next request

                            memcpy(&can2_dump_err_data[1],&can2_rx_data,8);

                            canTransmit(canREG1, G2KCA_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_Temperature_str);
                            can2_request_index++;       // bump index so next receive will switch to correct message
                            break;

                        case 2:     // store ccp_G2KCA_Temperature_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[2],&can2_rx_data,8);

                            canTransmit(canREG1, G2KCA_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_DigT1_str);
                            can2_request_index++;       // bump index so next receive will switch to correct message
                            break;

                        case 3:     // store ccp_G2KCA_DigT1_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[3],&can2_rx_data,8);

                            canTransmit(canREG1, G2KCA_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_DigT2_str);
                            can2_request_index++;       // bump index so next receive will switch to correct message
                            break;

                        case 4:     // store ccp_G2KCA_DigT2_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[4],&can2_rx_data,8);

                            canTransmit(canREG1, G2KCA_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_MtrCurrQax_str);
                            can2_request_index++;       // bump index so next receive will switch to correct message
                            break;

                        case 5:     // store ccp_G2KCA_MtrCurrQax_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[5],&can2_rx_data,8);

                            canTransmit(canREG1, G2KCA_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_SystemState_byt);
                            can2_request_index++;       // bump index so next receive will switch to correct message
                            break;

                        case 6:     // store ccp_G2KCA_SystemState_byt, setup and trans next reques

                            memcpy(&can2_dump_err_data[6],&can2_rx_data,8);

                            canTransmit(canREG1, G2KCA_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_HwTrq_HwNm_str);
                            can2_request_index++;       // bump index so next receive will switch to correct message
                            break;

                        case 7:     // store ccp_G2KCA_HwTrq_HwNm_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[7],&can2_rx_data,8);

                            canTransmit(canREG1, G2KCA_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_Abs_Hw_Pos_str);
                            can2_request_index++;       // bump index so next receive will switch to correct message
                            break;

                        case 8:     // store ccp_G2KCA_Abs_Hw_Pos_str, setup and trans next reques
                            //  all done, index bumped for main to test

                            memcpy(&can2_dump_err_data[8],&can2_rx_data,8);

                            can2_request_index++;       // bump index so next receive will switch to correct message
                            break;


                            //  ----------------   H c u r r    G2KCA reading cases 2000-2031 done during normal testing

                        case 2000:      // Hcurr Qax readings(x3) // store ccp_G2KCA_MtrCurrQax_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[9],&can2_rx_data,8);

                            canTransmit(canREG1, G2KCA_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_HwTrq_HwNm_str);
                            can2_request_index++;       // bump index so next receive will switch to correct message

                            break;

                        case 2001:      // store ccp_G2KCA_HwTrq_HwNm_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[10],&can2_rx_data,8);

                            can2_request_index = 2999;      // arbitrary value, all done for now

                            break;

                        case 2010:      // store ccp_G2KCA_MtrCurrQax_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[11],&can2_rx_data,8);

                            canTransmit(canREG1, G2KCA_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_HwTrq_HwNm_str);
                            can2_request_index++;       // bump index so next receive will switch to correct message

                            break;

                        case 2011:      // store ccp_G2KCA_HwTrq_HwNm_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[12],&can2_rx_data,8);

                            can2_request_index = 2999;      // arbitrary value, all done for now

                            break;

                        case 2020:      // store ccp_G2KCA_MtrCurrQax_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[13],&can2_rx_data,8);

                            canTransmit(canREG1, G2KCA_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_G2KCA_HwTrq_HwNm_str);
                            can2_request_index++;       // bump index so next receive will switch to correct message

                            break;

                        case 2021:      // store ccp_G2KCA_HwTrq_HwNm_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[14],&can2_rx_data,8);

                            can2_request_index = 2999;      // arbitrary value, all done for now

                            break;

                        case 2999:

                            can2_request_index = 3000;  // don't care response is done...ok to send new request.

                            break;

                        default:
                        {
                        }

                        }   // end switch(can1_request_index)

                        break;  // end case CCP_RESPONSE_ID:

                        case G2KCA_ECU2_CAN1_XCP_EA3_REQUEST_ID:    // 0x0242   canMESSAGE_BOX16    11bit identifier-- Trans XCP Request from EPS for data  (not used)
                            break;

                        case G2KCA_ECU2_CAN1_XCP_EA3_RESPONSE_ID:   // 0x0642   canMESSAGE_BOX17    11bit identifier-- Rec XCP Responce to EPS for data  (not used)
                            break;

                        case G2KCA_ECU2_CAN1_XCP_EA4_REQUEST_ID:    // 0x0713   canMESSAGE_BOX18    11bit identifier-- Trans XCP Request from EPS for data
                            break;

                        case G2KCA_ECU2_CAN1_XCP_EA4_RESPONSE_ID:    // 0x642   canMESSAGE_BOX19    11bit identifier-- Rec XCP Responce from EPS for data
                            // receive and decide what the REQ was, parse and store
                            canGetData(canREG1, G2KCA_ECU2_CAN1_XCP_EA4_RESPONSE_ID, can2_rx_data); // get data, store in rx_data array

                            switch(ECU2_XCP_reply_index)    // requesting multiple requests using same messageBox, DCT_XCP_rply_indx keeps track, clear_dtcs_flag determines action
                            {

                            case 1:
                                //EA4 is different
                                if((can2_rx_data[1] == 0x50) && (can2_rx_data[2] == 0x7E))  // positive response from enter Nexteer mode for FCA
                                {
                                    if(ECU2_clear_DTCs_flag == 1)         // clear DTC's mode
                                    {
                                        canTransmit(canREG1, G2KCA_ECU2_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_clear_DTC_rqst1); // request clear DTCs
                                    }
                                    else        // get DTC's mode
                                    {
                                        canTransmit(canREG1, G2KCA_ECU2_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_get_DTC_rqst1);   // request DTCs first string
                                    }

                                    ECU2_XCP_reply_index++;
                                }  // end positive response nexteer mode
                                break;

                            case 2:

                                if((can2_rx_data[0] == 0x10) && (can2_rx_data[2] == 0x62))    // get DTC's mode
                                {
                                    dtc2[0][0] =  can2_rx_data[5];
                                    dtc2[0][1] =  can2_rx_data[6];
                                    dtc2[0][2] =  can2_rx_data[7];

                                    canTransmit(canREG1, G2KCA_ECU2_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_get_DTC_rqst2);   // request DTCs 2nd string

                                    ECU2_XCP_reply_index++;
                                }

                                if((can2_rx_data[0] == 0x04) && (can2_rx_data[1] == 0x71))  // responce to clear DTC's mode
                                {
                                    if(ECU2_clear_DTCs_flag == 1)
                                    {
                                        ECU2_XCP_reply_index = 0;
                                        ECU2_clear_DTCs_flag = 0;   // reset flag and index, done
                                    }
                                }

                                break;

                            case 3:
                                if(can2_rx_data[0] == 0x21)
                                {
                                    dtc2[0][3] =  can2_rx_data[1];
                                    dtc2[0][4] =  can2_rx_data[2];
                                    dtc2[1][0] =  can2_rx_data[3];
                                    dtc2[1][1] =  can2_rx_data[4];
                                    dtc2[1][2] =  can2_rx_data[5];
                                    dtc2[1][3] =  can2_rx_data[6];
                                    dtc2[1][4] =  can2_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 4:
                                if(can2_rx_data[0] == 0x22)
                                {
                                    dtc2[2][0] =  can2_rx_data[1];
                                    dtc2[2][1] =  can2_rx_data[2];
                                    dtc2[2][2] =  can2_rx_data[3];
                                    dtc2[2][3] =  can2_rx_data[4];
                                    dtc2[2][4] =  can2_rx_data[5];
                                    dtc2[3][0] =  can2_rx_data[6];
                                    dtc2[3][1] =  can2_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 5:
                                if(can2_rx_data[0] == 0x23)
                                {
                                    dtc2[3][2] =  can2_rx_data[1];
                                    dtc2[3][3] =  can2_rx_data[2];
                                    dtc2[3][4] =  can2_rx_data[3];
                                    dtc2[4][0] =  can2_rx_data[4];
                                    dtc2[4][1] =  can2_rx_data[5];
                                    dtc2[4][2] =  can2_rx_data[6];
                                    dtc2[4][3] =  can2_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 6:
                                if(can2_rx_data[0] == 0x24)
                                {
                                    dtc2[4][4] =  can2_rx_data[1];
                                    dtc2[5][0] =  can2_rx_data[2];
                                    dtc2[5][1] =  can2_rx_data[3];
                                    dtc2[5][2] =  can2_rx_data[4];
                                    dtc2[5][3] =  can2_rx_data[5];
                                    dtc2[5][4] =  can2_rx_data[6];
                                    dtc2[6][0] =  can2_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 7:
                                if(can2_rx_data[0] == 0x25)
                                {
                                    dtc2[6][1] =  can2_rx_data[1];
                                    dtc2[6][2] =  can2_rx_data[2];
                                    dtc2[6][3] =  can2_rx_data[3];
                                    dtc2[6][4] =  can2_rx_data[4];
                                    dtc2[7][0] =  can2_rx_data[5];
                                    dtc2[7][1] =  can2_rx_data[6];
                                    dtc2[7][2] =  can2_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 8:
                                if(can2_rx_data[0] == 0x26)
                                {
                                    dtc2[7][3] =  can2_rx_data[1];
                                    dtc2[7][4] =  can2_rx_data[2];
                                    dtc2[8][0] =  can2_rx_data[3];
                                    dtc2[8][1] =  can2_rx_data[4];
                                    dtc2[8][2] =  can2_rx_data[5];
                                    dtc2[8][3] =  can2_rx_data[6];
                                    dtc2[8][4] =  can2_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 9:
                                if(can2_rx_data[0] == 0x27)
                                {
                                    dtc2[9][0] =  can2_rx_data[1];
                                    dtc2[9][1] =  can2_rx_data[2];
                                    dtc2[9][2] =  can2_rx_data[3];
                                    dtc2[9][3] =  can2_rx_data[4];
                                    dtc2[9][4] =  can2_rx_data[5];
                                    dtc2[10][0] = can2_rx_data[6];
                                    dtc2[10][1] = can2_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 10:
                                if(can2_rx_data[0] == 0x28)
                                {
                                    dtc2[10][2] =  can2_rx_data[1];
                                    dtc2[10][3] =  can2_rx_data[2];
                                    dtc2[10][4] =  can2_rx_data[3];
                                    dtc2[11][0] =  can2_rx_data[4];
                                    dtc2[11][1] =  can2_rx_data[5];
                                    dtc2[11][2] =  can2_rx_data[6];
                                    dtc2[11][3] =  can2_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 11:
                                if(can2_rx_data[0] == 0x29)
                                {
                                    dtc2[11][4] =  can2_rx_data[1];
                                    dtc2[12][0] =  can2_rx_data[2];
                                    dtc2[12][1] =  can2_rx_data[3];
                                    dtc2[12][2] =  can2_rx_data[4];
                                    dtc2[12][3] =  can2_rx_data[5];
                                    dtc2[12][4] =  can2_rx_data[6];
                                    dtc2[13][0] =  can2_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 12:
                                if(can2_rx_data[0] == 0x2A)
                                {
                                    dtc2[13][1] =  can2_rx_data[1];
                                    dtc2[13][2] =  can2_rx_data[2];
                                    dtc2[13][3] =  can2_rx_data[3];
                                    dtc2[13][4] =  can2_rx_data[4];
                                    dtc2[14][0] =  can2_rx_data[5];
                                    dtc2[14][1] =  can2_rx_data[6];
                                    dtc2[14][2] =  can2_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 13:
                                if(can2_rx_data[0] == 0x2B)
                                {
                                    dtc2[14][3] =  can1_rx_data[1];
                                    dtc2[14][4] =  can1_rx_data[2];

                                    dtc2[14][3] =  can1_rx_data[1];
                                    dtc2[14][4] =  can1_rx_data[2];
                                    dtc2[15][0] =  can1_rx_data[3];
                                    dtc2[15][1] =  can1_rx_data[4];
                                    dtc2[15][2] =  can1_rx_data[5];
                                    dtc2[15][3] =  can1_rx_data[6];
                                    dtc2[15][4] =  can1_rx_data[7];
                                }
                                ECU2_XCP_reply_index = 14;
                                break;

                            case 14:
                                if(can2_rx_data[0] == 0x2C)
                                {
                                    dtc2[16][0] =  can1_rx_data[1];
                                    dtc2[16][1] =  can1_rx_data[2];
                                    dtc2[16][2] =  can1_rx_data[3];
                                    dtc2[16][3] =  can1_rx_data[4];
                                    dtc2[16][4] =  can1_rx_data[5];
                                    dtc2[17][0] =  can1_rx_data[6];
                                    dtc2[17][1] =  can1_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 15:
                                if(can2_rx_data[0] == 0x2D)
                                {
                                    dtc2[17][2] =  can1_rx_data[1];
                                    dtc2[17][3] =  can1_rx_data[2];
                                    dtc2[17][4] =  can1_rx_data[3];
                                    dtc2[18][0] =  can1_rx_data[4];
                                    dtc2[18][1] =  can1_rx_data[5];
                                    dtc2[18][2] =  can1_rx_data[6];
                                    dtc2[18][3] =  can1_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 16:
                                if(can2_rx_data[0] == 0x2E)
                                {
                                    dtc2[18][4] =  can1_rx_data[1];
                                    dtc2[19][0] =  can1_rx_data[2];
                                    dtc2[19][1] =  can1_rx_data[3];
                                    dtc2[19][2] =  can1_rx_data[4];
                                    dtc2[19][3] =  can1_rx_data[5];
                                    dtc2[19][4] =  can1_rx_data[6];
                                }
                                ECU2_XCP_reply_index = 17;  // last read set to 17 for get_DTCs function
                                break;

                            case 100:
                                ECU2_XCP_reply_index = 101;     // case used for multi message for TOC
                                break;

                            case 101:
                                ECU2_XCP_reply_index = 102;     // case used for multi message for TOC
                                break;

                            default:
                                ECU2_XCP_reply_index = 0;
                                break;

                            } // end switch on DTC_XCP_rply_indx

                            break; // end case G2KCA_ECU2_CAN1_XCP_EA4_RESPONSE_ID

                            case G2KCA_TESTER_REC_DEBUG_ID:             // 0x07FF   canMESSAGE_BOX20    11bit identifier-- REC from Anyone
                                break;

                            case G2KCA_TESTER_TRANS_DEBUG_ID:           // 0x07FE   canMESSAGE_BOX21        11bit identifier-- Transmit to Anyone
                                break;

                            case canMESSAGE_BOX22:                      // 0x22  11bit identifier-- Not Used
                                break;

                            default:

                            {
                                can1_message_type_debug = messageBox;  // an unknown messageBox
                            }

                            break;      // end switch(messageBox) for G2KCA ADAS on canREG1

            }   // end switch(messageBox) for G2KCA ADAS on canREG1

        }  // end case G2KCA ADAS    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^    G 2 K C A   A D A S   C A N 1 ^^^^^^^^^^^^^^^^^^^^^^^^^^^
        break;

        case TARGET_PSA_CMP:       //  VVVVVVVVVVVVVVVVVVVVVVVVVVVVV  P S A _ C M P   C A N 1  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
        {
            switch(messageBox)
            {
            case PSA_PROD_MESS1_ID:     // 0x148    canMESSAGE_BOX1 11bit identifier-- Rec Response from EPS (not used)
                break;

            case PSA_PROD_MESS2_ID:     // 0x778    canMESSAGE_BOX2 11bit identifier-- Rec ON Star from EPS  (not used)
                break;

            case PSA_CCP_Engine_Run:    // 0x348    canMESSAGE_BOX3 11bit identifier-- Trans Engine ON to EPS
                break;

            case PSA_CCP_Power_Mode_ID: // 0x1F1    canMESSAGE_BOX4 11bit identifier-- Trans Power Mode to EPS (not used)
                break;

            case PSA_CCP_SPEED_ID:      // 0x38D    canMESSAGE_BOX5 11bit identifier-- Trans Speed and Validity Status to EPS

                break;


            case PSA_CCP_REQUEST_ID:        // 0x7AE    canMESSAGE_BOX6 11bit identifier-- Trans CCP Request for Data to EPS
                break;

            case PSA_CCP_RESPONSE_ID:     // 0x7B0  canMESSAGE_BOX7 11bit identifier-- Rec CCP response from EPS
                // Receive and decide what the REQ was, parse and store
                // Standard dump of all ccp request

                canGetData(canREG1, PSA_CCP_RESPONSE_ID, can1_rx_data); // get data, store in rx_data array

                switch(can1_request_index)      // requesting multiple variables using same messageBox, can1_request_index keeps track, store and parse in main
                {

                case 0:     // do nothing with request_connect data, setup and trans next request

                    canTransmit(canREG1, PSA_CCP_REQUEST_ID, ( uint8 *) ccp_PSA_CMP_BattVltg_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 1:     // store ccp_SystemState_byt, setup and trans next request

                    memcpy(&can1_dump_err_data[1],&can1_rx_data,8);

                    canTransmit(canREG1, PSA_CCP_REQUEST_ID, ( uint8 *) ccp_PSA_CMP_Temperature_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 2:     // store ccp_MotCurrQax_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[2],&can1_rx_data,8);

                    canTransmit(canREG1, PSA_CCP_REQUEST_ID, ( uint8 *) ccp_PSA_CMP_DigT1_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 3:     // store ccp_HwTq0Meas_HwTq0_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[3],&can1_rx_data,8);

                    canTransmit(canREG1, PSA_CCP_REQUEST_ID, ( uint8 *) ccp_PSA_CMP_DigT2_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 4:     // store ccp_HwTq1Meas_HwTq1_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[4],&can1_rx_data,8);

                    canTransmit(canREG1, PSA_CCP_REQUEST_ID, ( uint8 *) ccp_PSA_CMP_SystemState_byt);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 5:     // store ccp_HwTq2Meas_HwTq2_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[5],&can1_rx_data,8);

                    canTransmit(canREG1, PSA_CCP_REQUEST_ID, ( uint8 *) ccp_PSA_CMP_MtrCurrQax_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 6:     // store ccp_HwTq3Meas_HwTq3_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[6],&can1_rx_data,8);

                    canTransmit(canREG1, PSA_CCP_REQUEST_ID, ( uint8 *) ccp_PSA_CMP_MtrCurrDax_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 7:     // store ccp_EcuTFild_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[7],&can1_rx_data,8);

                    canTransmit(canREG1, PSA_CCP_REQUEST_ID, ( uint8 *) ccp_PSA_CMP_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 8:     // store ccp_HwAgArbn_HwAg_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[8],&can1_rx_data,8);

                    canTransmit(canREG1, PSA_CCP_REQUEST_ID, ( uint8 *) ccp_PSA_CMP_MtrTrqCmd_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 9:     //  store ccp_HwAg0_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[9],&can1_rx_data,8);

                    canTransmit(canREG1, PSA_CCP_REQUEST_ID, ( uint8 *) ccp_PSA_CMP_AstMtrTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 10:        // store ccp_HwAg1_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[10],&can1_rx_data,8);

                    canTransmit(canREG1, PSA_CCP_REQUEST_ID, ( uint8 *) ccp_PSA_CMP_Abs_Hw_Pos_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 11:        // store ccp_HwTqArbn_HwTq_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[11],&can1_rx_data,8);

                    canTransmit(canREG1, PSA_CCP_REQUEST_ID, ( uint8 *) ccp_PSA_CMP_Rel_Hw_Pos_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 12:        // store ccp_MotCurrDax_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[12],&can1_rx_data,8);

                    canTransmit(canREG1, PSA_CCP_REQUEST_ID, ( uint8 *) ccp_PSA_CMP_Abs_Hw_Pos_Valid_byt);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 13:        // store ccp_HwTqArbn_HwTqChA_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[13],&can1_rx_data,8);

                    canTransmit(canREG1, PSA_CCP_REQUEST_ID, ( uint8 *) ccp_PSA_CMP_MtrPos_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 14:        // store ccp_HwTqArbn_HwTqChB_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[14],&can1_rx_data,8);

                    canTransmit(canREG1, PSA_CCP_REQUEST_ID, ( uint8 *) ccp_PSA_CMP_ADC_MtrCurr1_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 15:        // store ccp_MotAgCmp_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[15],&can1_rx_data,8);

                    canTransmit(canREG1, PSA_CCP_REQUEST_ID, ( uint8 *) ccp_PSA_CMP_ADC_MtrCurr2_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 16:        // store ccp_BattVltg_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[16],&can1_rx_data,8);

                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;


                    //  left 2000 cases for placeholders, just in case
                    //  ----------------   H c u r r    PSA reading cases 2000-2031 done during normal testing  (2000-2021 not used, placeholder)

                case 2000:      // Hcurr Qax readings(x3) // store can_MotCurrQax1_str, setup and trans next reques

                    //        memcpy(&can1_dump_err_data[28],&can1_rx_data,8);

                    //        canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_HwTqArbn_HwTq_str);
                    //        can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2001:      // store ccp_HwTqArbn_HwTq1str, setup and trans next request

                    //        memcpy(&can1_dump_err_data[29],&can1_rx_data,8);

                    //        can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2010:      // store can_MotCurrQax2_str, setup and trans next request

                    //        memcpy(&can1_dump_err_data[30],&can1_rx_data,8);

                    //        canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_HwTqArbn_HwTq_str);
                    //        can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2011:      // store ccp_HwTqArbn_HwTq2str, setup and trans next request

                    //        memcpy(&can1_dump_err_data[31],&can1_rx_data,8);

                    //        can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2020:      // store can_MotCurrQax3_str, setup and trans next request

                    //        memcpy(&can1_dump_err_data[32],&can1_rx_data,8);

                    //        canTransmit(canREG1, CCP_REQUEST_ID, ( uint8 *) ccp_T1XX_HwTqArbn_HwTq_str);
                    //        can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2021:      // store ccp_HwTqArbn_HwTq3str, setup and trans next request

                    //        memcpy(&can1_dump_err_data[33],&can1_rx_data,8);

                    //        can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2999:

                    can1_request_index = 3000;  // don't care response is done...ok to send new request.

                    break;

                default:
                {
                }

                }   // end switch(can1_request_index) for PSA   C A N 1

                break;  // end case PSA_CCP_RESPONSE_ID:



                case PSA_XCP_EA3_REQUEST_ID:   // 0x6B5     canMESSAGE_BOX8 11bit identifier-- Trans CCP Request for Data to EPS
                    break;

                case PSA_XCP_EA3_RESPONSE_ID:  // 0x695     canMESSAGE_BOX9 11bit identifier-- Rec XCP Responce from EPS for data
                    // receive and decide what the REQ was, parse and store

                    canGetData(canREG1, PSA_XCP_EA3_RESPONSE_ID, can1_rx_data);    // get data, store in rx_data array

                    switch(ECU1_XCP_reply_index)    // requesting multiple requests using same messageBox, DCT_XCP_rply_indx keeps track, clear_dtcs_flag determines action
                    {

                    case 1:

                        if((can1_rx_data[1] == 0x50) && (can1_rx_data[2] == 0x7E))  // positive responce from enter Nexteer mode for FCA
                        {
                            if(ECU1_clear_DTCs_flag == 1)        // clear DTC's mode
                            {
                                canTransmit(canREG1, PSA_XCP_EA3_REQUEST_ID, ( uint8 *) xcp_EA3_write_clear_DTC_rqst1);   // request clear DTCs
                            }

                            else       // get DTC's mode

                            {
                                canTransmit(canREG1, PSA_XCP_EA3_REQUEST_ID, ( uint8 *) xcp_EA3_write_get_DTC_rqst1); // request DTCs first string
                            }

                            ECU1_XCP_reply_index++;
                        }  // end positive response nexteer mode
                        break;

                    case 2:

                        if((can1_rx_data[0] == 0x10) && (can1_rx_data[2] == 0x62))    // get DTC's mode
                        {

                            dtc1[0][0] =  can1_rx_data[5];
                            dtc1[0][1] =  can1_rx_data[6];
                            dtc1[0][2] =  can1_rx_data[7];

                            canTransmit(canREG1, PSA_XCP_EA3_REQUEST_ID, ( uint8 *) xcp_EA3_write_get_DTC_rqst2);  // request DTCs 2nd string

                            ECU1_XCP_reply_index++;
                        }

                        if((can1_rx_data[0] == 0x04) && (can1_rx_data[1] == 0x71))  // Response to clear DTC's mode
                        {
                            if(ECU1_clear_DTCs_flag == 1)
                            {
                                ECU1_XCP_reply_index = 0;
                                ECU1_clear_DTCs_flag = 0;   // reset flag and index, done
                            }
                        }

                        break;

                    case 3:
                        if(can1_rx_data[0] == 0x21)
                        {
                            dtc1[0][3] =  can1_rx_data[1];
                            dtc1[0][4] =  can1_rx_data[2];
                            dtc1[1][0] =  can1_rx_data[3];
                            dtc1[1][1] =  can1_rx_data[4];
                            dtc1[1][2] =  can1_rx_data[5];
                            dtc1[1][3] =  can1_rx_data[6];
                            dtc1[1][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 4:
                        if(can1_rx_data[0] == 0x22)
                        {
                            dtc1[2][0] =  can1_rx_data[1];
                            dtc1[2][1] =  can1_rx_data[2];
                            dtc1[2][2] =  can1_rx_data[3];
                            dtc1[2][3] =  can1_rx_data[4];
                            dtc1[2][4] =  can1_rx_data[5];
                            dtc1[3][0] =  can1_rx_data[6];
                            dtc1[3][1] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 5:
                        if(can1_rx_data[0] == 0x23)
                        {
                            dtc1[3][2] =  can1_rx_data[1];
                            dtc1[3][3] =  can1_rx_data[2];
                            dtc1[3][4] =  can1_rx_data[3];
                            dtc1[4][0] =  can1_rx_data[4];
                            dtc1[4][1] =  can1_rx_data[5];
                            dtc1[4][2] =  can1_rx_data[6];
                            dtc1[4][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 6:
                        if(can1_rx_data[0] == 0x24)
                        {
                            dtc1[4][4] =  can1_rx_data[1];
                            dtc1[5][0] =  can1_rx_data[2];
                            dtc1[5][1] =  can1_rx_data[3];
                            dtc1[5][2] =  can1_rx_data[4];
                            dtc1[5][3] =  can1_rx_data[5];
                            dtc1[5][4] =  can1_rx_data[6];
                            dtc1[6][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 7:
                        if(can1_rx_data[0] == 0x25)
                        {
                            dtc1[6][1] =  can1_rx_data[1];
                            dtc1[6][2] =  can1_rx_data[2];
                            dtc1[6][3] =  can1_rx_data[3];
                            dtc1[6][4] =  can1_rx_data[4];
                            dtc1[7][0] =  can1_rx_data[5];
                            dtc1[7][1] =  can1_rx_data[6];
                            dtc1[7][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 8:
                        if(can1_rx_data[0] == 0x26)
                        {
                            dtc1[7][3] =  can1_rx_data[1];
                            dtc1[7][4] =  can1_rx_data[2];
                            dtc1[8][0] =  can1_rx_data[3];
                            dtc1[8][1] =  can1_rx_data[4];
                            dtc1[8][2] =  can1_rx_data[5];
                            dtc1[8][3] =  can1_rx_data[6];
                            dtc1[8][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 9:
                        if(can1_rx_data[0] == 0x27)
                        {
                            dtc1[9][0] =  can1_rx_data[1];
                            dtc1[9][1] =  can1_rx_data[2];
                            dtc1[9][2] =  can1_rx_data[3];
                            dtc1[9][3] =  can1_rx_data[4];
                            dtc1[9][4] =  can1_rx_data[5];
                            dtc1[10][0] = can1_rx_data[6];
                            dtc1[10][1] = can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 10:
                        if(can1_rx_data[0] == 0x28)
                        {
                            dtc1[10][2] =  can1_rx_data[1];
                            dtc1[10][3] =  can1_rx_data[2];
                            dtc1[10][4] =  can1_rx_data[3];
                            dtc1[11][0] =  can1_rx_data[4];
                            dtc1[11][1] =  can1_rx_data[5];
                            dtc1[11][2] =  can1_rx_data[6];
                            dtc1[11][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 11:
                        if(can1_rx_data[0] == 0x29)
                        {
                            dtc1[11][4] =  can1_rx_data[1];
                            dtc1[12][0] =  can1_rx_data[2];
                            dtc1[12][1] =  can1_rx_data[3];
                            dtc1[12][2] =  can1_rx_data[4];
                            dtc1[12][3] =  can1_rx_data[5];
                            dtc1[12][4] =  can1_rx_data[6];
                            dtc1[13][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 12:

                        if(can1_rx_data[0] == 0x2A)
                        {
                            dtc1[13][1] =  can1_rx_data[1];
                            dtc1[13][2] =  can1_rx_data[2];
                            dtc1[13][3] =  can1_rx_data[3];
                            dtc1[13][4] =  can1_rx_data[4];
                            dtc1[14][0] =  can1_rx_data[5];
                            dtc1[14][1] =  can1_rx_data[6];
                            dtc1[14][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 13:

                        if(can1_rx_data[0] == 0x2B)
                        {
                            dtc1[14][3] =  can1_rx_data[1];
                            dtc1[14][4] =  can1_rx_data[2];
                        }
                        ECU1_XCP_reply_index = 14;   // last read set to 14 for get_DTCs function
                        break;


                    default:

                        ECU1_XCP_reply_index = 0;

                        break;

                    } // end switch on DTC_XCP_rply_indx

                    break; // end case XCP_RESPONSE_ID

                    case PSA_XCP_EA4_REQUEST_ID: // 0x712  canMESSAGE_BOX10 11bit identifier-- Trans XCP Request from EPS for data
                        break;

                    case PSA_XCP_EA4_RESPONSE_ID: // 0x710  canMESSAGE_BOX11 11bit identifier-- Rec XCP Responce to EPS for data
                        break;

                    case TESTER_REC_DEBUG_ID:    // 0x7FF    canMESSAGE_BOX12 11bit identifier-- REC from Anyone

                        break;

                    case TESTER_TRANS_DEBUG_ID:  // 0x7FE     canMESSAGE_BOX13 11bit identifier-- Transmit to Anyone

                        break;

                    case canMESSAGE_BOX14:      // 0x14  11bit identifier-- Not Used

                        break;

                    case canMESSAGE_BOX15:      // 0x15  11bit identifier-- Not Usedr

                        break;

                    case canMESSAGE_BOX16:      // 0x16  11bit identifier-- Not Used

                        break;

                    default:

                    {
                        can1_message_type_debug = messageBox;  // an unknown messageBox
                    }
                    break;

            }   // end switch(messageBox) for TARGET_PSA_CMP on canREG1
        }   // end case TARGET_PSA_CMP    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^  P S A _ C M P   C A N 1  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        break;

        case TARGET_SGMW_CN200:       //  VVVVVVVVVVVVVVVVVVVVVVVVVVVVV  C N 2 0 0  C A N 1  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
        {
            switch(messageBox)
            {
            case CN200_ECU1_CAN1_PROD_MESS1_ID:     // 0x148    canMESSAGE_BOX1 11bit identifier-- Rec Response from EPS (not used)
                break;

            case CN200_ECU1_CAN1_PROD_MESS2_ID:     // 0x778    canMESSAGE_BOX2 11bit identifier-- Rec ON Star from EPS  (not used)
                break;

            case CN200_ECU1_CAN1_CCP_Engine_Run:    // 0x0C9    canMESSAGE_BOX3 11bit identifier-- Trans Engine ON to EPS
                break;

            case CN200_ECU1_CAN1_CCP_Power_Mode_ID: // 0x1F1    canMESSAGE_BOX4 11bit identifier-- Trans Power Mode to EPS
                break;

            case CN200_ECU1_CAN1_CCP_SPEED_ID:      // 0x348    canMESSAGE_BOX5 11bit identifier-- Trans Speed and Validity Status to EPS

                break;

            case CN200_ECU1_CAN1_CCP_REQUEST_ID:   // 0x708    canMESSAGE_BOX6 11bit identifier-- Trans CCP Request for Data to EPS
                break;


            case CN200_ECU1_CAN1_CCP_RESPONSE_ID: // 0x706  canMESSAGE_BOX7 11bit identifier-- Rec CCP response from EPS
                // receive and decide what the REQ was, parse and store
                // Standard dump of all ccp request

                canGetData(canREG1, CN200_ECU1_CAN1_CCP_RESPONSE_ID, can1_rx_data); // get data, store in rx_data array

                switch(can1_request_index)      // requesting multiple variables using same messageBox, can1_request_index keeps track, store and parse in main
                {

                case 0:     // do nothing with request_connect data, setup and trans next request

                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_CN200_BattVltg_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 1:     // store ccp_CN200_BattVltg_str, setup and trans next request

                    memcpy(&can1_dump_err_data[1],&can1_rx_data,8);

                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_CN200_Temperature_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 2:     // store ccp_CN200_Temperature_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[2],&can1_rx_data,8);

                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_CN200_DigT1_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 3:     // store ccp_CN200_DigT1_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[3],&can1_rx_data,8);

                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_CN200_DigT2_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 4:     // store ccp_CN200_DigT2_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[4],&can1_rx_data,8);

                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_CN200_MtrCurrQax_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 5:     // store ccp_CN200_MtrCurrQax_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[5],&can1_rx_data,8);

                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_CN200_SystemState_byt);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 6:     // store ccp_CN200_SystemState_byt, setup and trans next reques

                    memcpy(&can1_dump_err_data[6],&can1_rx_data,8);

                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_CN200_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 7:     // store ccp_CN200_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[7],&can1_rx_data,8);

                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_CN200_Abs_Hw_Pos_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 8:     // store ccp_CN200_Abs_Hw_Pos_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[8],&can1_rx_data,8);

                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_CN200_MotTrq_Crf_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 9:     //  store ccp_CN200_MotTrq_Crf_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[9],&can1_rx_data,8);

                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_CN200_MotTrq_Mrf_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 10:        // store ccp_CN200_MotTrq_Mrf_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[10],&can1_rx_data,8);

                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_CN200_VehSpd_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 11:        // store ccp_CN200_VehSpd_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[11],&can1_rx_data,8);

                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_CN200_Spare2_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 12:        // store ccp_CN200_Spare2_str, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can1_dump_err_data[12],&can1_rx_data,8);

                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;



                    //  ----------------   H c u r r    CN200 reading cases 2000-2031 done during normal testing

                case 2000:      // Hcurr Qax readings(x3) // store ccp_CN200_MtrCurrQax_str 1, setup and trans next reques

                    memcpy(&can1_dump_err_data[13],&can1_rx_data,8);

                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_CN200_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2001:      // store ccp_CN200_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[14],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2010:      // store ccp_CN200_MtrCurrQax_str 2, setup and trans next reques

                    memcpy(&can1_dump_err_data[15],&can1_rx_data,8);

                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_CN200_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2011:      // store ccp_CN200_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[16],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2020:      // store ccp_CN200_MtrCurrQax_str 3, setup and trans next reques

                    memcpy(&can1_dump_err_data[17],&can1_rx_data,8);

                    canTransmit(canREG1, CN200_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_CN200_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2021:      // store ccp_CN200_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[18],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2999:

                    can1_request_index = 3000;  // don't care response is done...ok to send new request.

                    break;

                default:
                {
                }

                }   // end switch(can1_request_index) for CN200  C A N 1

                break;  // end case CN200_ECU1_CAN1_CCP_RESPONSE_ID:


                case CN200_ECU1_CAN1_XCP_EA3_REQUEST_ID:     // 0x242   canMESSAGE_BOX8 11bit identifier-- Trans CCP Request for Data to EPS
                    break;

                case CN200_ECU1_CAN1_XCP_EA3_RESPONSE_ID:    // 0x642   canMESSAGE_BOX9 11bit identifier-- Rec XCP Response from EPS for data
                    // receive and decide what the REQ was, parse and store

                    break; // end case XCP_RESPONSE_ID

                case CN200_ECU1_CAN1_XCP_EA4_REQUEST_ID:     // 0x712   canMESSAGE_BOX10 11bit identifier-- Trans XCP Request from EPS for data
                    break;

                case CN200_ECU1_CAN1_XCP_EA4_RESPONSE_ID:     // 0x710   canMESSAGE_BOX11 11bit identifier-- Rec XCP Response to EPS for data
                    canGetData(canREG1, CN200_ECU1_CAN1_XCP_EA4_RESPONSE_ID, can1_rx_data); // get data, store in rx_data array

                    switch(ECU1_XCP_reply_index)    // requesting multiple requests using same messageBox, DCT_XCP_rply_indx keeps track, clear_dtcs_flag determines action
                    {

                    case 1:
                        //EA4 is different
                        if((can1_rx_data[1] == 0x50) && (can1_rx_data[2] == 0x7E))  // positive response from enter Nexteer mode
                        {
                            if(ECU1_clear_DTCs_flag == 1)        // clear DTC's mode
                            {
                                canTransmit(canREG1, CN200_ECU1_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_clear_DTC_rqst1);    // request clear DTCs
                            }

                            else       // get DTC's mode

                            {
                                canTransmit(canREG1, CN200_ECU1_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_get_DTC_rqst1);  // request DTCs first string
                            }

                            ECU1_XCP_reply_index++;
                        }  // end positive response nexteer mode
                        break;

                    case 2:

                        if((can1_rx_data[0] == 0x10) && (can1_rx_data[2] == 0x62))    // get DTC's mode
                        {

                            dtc1[0][0] =  can1_rx_data[5];
                            dtc1[0][1] =  can1_rx_data[6];
                            dtc1[0][2] =  can1_rx_data[7];

                            canTransmit(canREG1, CN200_ECU1_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_get_DTC_rqst2);   // request DTCs 2nd string

                            ECU1_XCP_reply_index++;
                        }

                        if((can1_rx_data[0] == 0x04) && (can1_rx_data[1] == 0x71))  // responce to clear DTC's mode
                        {
                            if(ECU1_clear_DTCs_flag == 1)
                            {
                                ECU1_XCP_reply_index = 0;
                                ECU1_clear_DTCs_flag = 0;   // reset flag and index, done
                            }
                        }

                        break;

                    case 3:
                        if(can1_rx_data[0] == 0x21)
                        {
                            dtc1[0][3] =  can1_rx_data[1];
                            dtc1[0][4] =  can1_rx_data[2];
                            dtc1[1][0] =  can1_rx_data[3];
                            dtc1[1][1] =  can1_rx_data[4];
                            dtc1[1][2] =  can1_rx_data[5];
                            dtc1[1][3] =  can1_rx_data[6];
                            dtc1[1][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 4:
                        if(can1_rx_data[0] == 0x22)
                        {
                            dtc1[2][0] =  can1_rx_data[1];
                            dtc1[2][1] =  can1_rx_data[2];
                            dtc1[2][2] =  can1_rx_data[3];
                            dtc1[2][3] =  can1_rx_data[4];
                            dtc1[2][4] =  can1_rx_data[5];
                            dtc1[3][0] =  can1_rx_data[6];
                            dtc1[3][1] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 5:
                        if(can1_rx_data[0] == 0x23)
                        {
                            dtc1[3][2] =  can1_rx_data[1];
                            dtc1[3][3] =  can1_rx_data[2];
                            dtc1[3][4] =  can1_rx_data[3];
                            dtc1[4][0] =  can1_rx_data[4];
                            dtc1[4][1] =  can1_rx_data[5];
                            dtc1[4][2] =  can1_rx_data[6];
                            dtc1[4][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;

                        break;

                    case 6:
                        if(can1_rx_data[0] == 0x24)
                        {
                            dtc1[4][4] =  can1_rx_data[1];
                            dtc1[5][0] =  can1_rx_data[2];
                            dtc1[5][1] =  can1_rx_data[3];
                            dtc1[5][2] =  can1_rx_data[4];
                            dtc1[5][3] =  can1_rx_data[5];
                            dtc1[5][4] =  can1_rx_data[6];
                            dtc1[6][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 7:
                        if(can1_rx_data[0] == 0x25)
                        {
                            dtc1[6][1] =  can1_rx_data[1];
                            dtc1[6][2] =  can1_rx_data[2];
                            dtc1[6][3] =  can1_rx_data[3];
                            dtc1[6][4] =  can1_rx_data[4];
                            dtc1[7][0] =  can1_rx_data[5];
                            dtc1[7][1] =  can1_rx_data[6];
                            dtc1[7][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 8:
                        if(can1_rx_data[0] == 0x26)
                        {
                            dtc1[7][3] =  can1_rx_data[1];
                            dtc1[7][4] =  can1_rx_data[2];
                            dtc1[8][0] =  can1_rx_data[3];
                            dtc1[8][1] =  can1_rx_data[4];
                            dtc1[8][2] =  can1_rx_data[5];
                            dtc1[8][3] =  can1_rx_data[6];
                            dtc1[8][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 9:
                        if(can1_rx_data[0] == 0x27)
                        {
                            dtc1[9][0] =  can1_rx_data[1];
                            dtc1[9][1] =  can1_rx_data[2];
                            dtc1[9][2] =  can1_rx_data[3];
                            dtc1[9][3] =  can1_rx_data[4];
                            dtc1[9][4] =  can1_rx_data[5];
                            dtc1[10][0] = can1_rx_data[6];
                            dtc1[10][1] = can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 10:
                        if(can1_rx_data[0] == 0x28)
                        {
                            dtc1[10][2] =  can1_rx_data[1];
                            dtc1[10][3] =  can1_rx_data[2];
                            dtc1[10][4] =  can1_rx_data[3];
                            dtc1[11][0] =  can1_rx_data[4];
                            dtc1[11][1] =  can1_rx_data[5];
                            dtc1[11][2] =  can1_rx_data[6];
                            dtc1[11][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 11:
                        if(can1_rx_data[0] == 0x29)
                        {
                            dtc1[11][4] =  can1_rx_data[1];
                            dtc1[12][0] =  can1_rx_data[2];
                            dtc1[12][1] =  can1_rx_data[3];
                            dtc1[12][2] =  can1_rx_data[4];
                            dtc1[12][3] =  can1_rx_data[5];
                            dtc1[12][4] =  can1_rx_data[6];
                            dtc1[13][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 12:
                        if(can1_rx_data[0] == 0x2A)
                        {
                            dtc1[13][1] =  can1_rx_data[1];
                            dtc1[13][2] =  can1_rx_data[2];
                            dtc1[13][3] =  can1_rx_data[3];
                            dtc1[13][4] =  can1_rx_data[4];
                            dtc1[14][0] =  can1_rx_data[5];
                            dtc1[14][1] =  can1_rx_data[6];
                            dtc1[14][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 13:
                        if(can1_rx_data[0] == 0x2B)
                        {
                            dtc1[14][3] =  can1_rx_data[1];
                            dtc1[14][4] =  can1_rx_data[2];
                        }
                        ECU1_XCP_reply_index = 14;   // last read set to 14 for get_DTCs function
                        break;

                    case 100:
                        ECU1_XCP_reply_index = 101;     // case used for multi message for TOC
                        break;

                    case 101:
                        ECU1_XCP_reply_index = 102;     // case used for multi message for TOC
                        break;

                    default:

                        ECU1_XCP_reply_index = 0;

                        break;

                    } // end switch on DTC_XCP_rply_indx

                    break;

                    case CN200_TESTER_REC_DEBUG_ID:    // 0x7FF    canMESSAGE_BOX12 11bit identifier-- REC from Anyone

                        break;

                    case CN200_TESTER_TRANS_DEBUG_ID:  // 0x7FE     canMESSAGE_BOX13 11bit identifier-- Transmit to Anyone

                        break;

                    case canMESSAGE_BOX14:      // 0x14  11bit identifier-- Not Used

                        break;

                    case canMESSAGE_BOX15:      // 0x15  11bit identifier-- Not Usedr

                        break;

                    case canMESSAGE_BOX16:      // 0x16  11bit identifier-- Not Used

                        break;

                    default:

                    {
                        can1_message_type_debug = messageBox;  // an unknown messageBox
                    }
                    break;

            }   // end switch(messageBox) for TARGET_SGMW_CN200 on canREG1
        }   // end case TARGET_SGMW_CN200    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^  C N 2 0 0   C A N 1  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        break;

        case TARGET_FORD_T3_T6:     //  VVVVVVVVVVVVVVVVVVVVVVVVVV  F O R D   T 3  -  T 6  C A N 1  VVVVVVVVVVVVVVVVVVVVVVVVV
        {
            switch(messageBox)
            {
            case FORD_T3_ECU1_CAN1_PROD_MESS1_ID:     // 0x148    canMESSAGE_BOX1 11bit identifier-- Rec Response from EPS (not used)
                break;

            case FORD_T3_ECU1_CAN1_PROD_MESS2_ID:     // 0x778    canMESSAGE_BOX2 11bit identifier-- Rec ON Star from EPS (not used)
                break;

            case FORD_T3_ECU1_CAN1_CCP_Engine_Run:    // 0x167    canMESSAGE_BOX3 11bit identifier-- Trans Engine ON to EPS
                break;

            case FORD_T3_ECU1_CAN1_CCP_Power_Mode_ID: // 0x3B3    canMESSAGE_BOX4 11bit identifier-- Trans Power Mode to EPS(not used)
                break;

            case FORD_T3_ECU1_CAN1_CCP_SPEED_ID:      // 0x415    canMESSAGE_BOX5 11bit identifier-- Trans Speed to EPS with counter and CRC
                break;

            case FORD_T3_ECU1_CAN1_CCP_REQUEST_ID:    // 0x60A    canMESSAGE_BOX6 11bit identifier-- Trans CCP Request for Data to EPS
                break;

            case FORD_T3_ECU1_CAN1_CCP_RESPONSE_ID:   // 0x60B    canMESSAGE_BOX7 11bit identifier-- Rec CCP Response from EPS for Data
                // Receive and decide what the REQ was, parse and store
                // Standard dump of all ccp request

                canGetData(canREG1, FORD_T3_ECU1_CAN1_CCP_RESPONSE_ID, can1_rx_data); // get data, store in rx_data array

                switch(can1_request_index)      // requesting multiple variables using same messageBox, can1_request_index keeps track, store and parse in main
                {
                case 0:     // do nothing with request_connect data, setup and trans next request

                    canTransmit(canREG1, FORD_T3_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU1_BattVltg_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 1:     // store ccp_FORD_T3_ECU1_BattVltg_str, setup and trans next request

                    memcpy(&can1_dump_err_data[1],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_T3_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU1_Temperature_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 2:     // store ccp_FORD_T3_ECU1_Temperature_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[2],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_T3_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU1_DigT1_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 3:     // store ccp_FORD_T3_ECU1_DigT1_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[3],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_T3_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU1_DigT2_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 4:     // store ccp_FORD_T3_ECU1_DigT2_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[4],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_T3_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU1_MtrCurrQax_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 5:     // store ccp_FORD_T3_ECU1_MtrCurrQax_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[5],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_T3_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU1_SystemState_byt);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 6:     // store ccp_FORD_T3_ECU1_SystemState_byt, setup and trans next reques

                    memcpy(&can1_dump_err_data[6],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_T3_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU1_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 7:     // store ccp_FORD_T3_ECU1_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[7],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_T3_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU1_Abs_Hw_Pos_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 8:     // store ccp_FORD_T3_ECU1_Abs_Hw_Pos_str, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can1_dump_err_data[8],&can1_rx_data,8);

                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;


                    //  ----------------   H c u r r    FORD T3 reading cases 2000-2031 done during normal testing

                case 2000:      // Hcurr Qax readings(x3) // store ccp_FORD_T3_ECU1_MtrCurrQax_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[9],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_T3_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU1_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2001:      // store ccp_FORD_T3_ECU1_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[10],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2010:      // store ccp_FORD_T3_ECU1_MtrCurrQax_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[11],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_T3_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU1_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2011:      // store ccp_FORD_T3_ECU1_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[12],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2020:      // store ccp_FORD_T3_ECU1_MtrCurrQax_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[13],&can1_rx_data,8);

                    canTransmit(canREG1, FORD_T3_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU1_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2021:      // store ccp_FORD_T3_ECU1_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[14],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2999:

                    can1_request_index = 3000;  // don't care response is done...ok to send new request.

                    break;

                default:
                {
                }

                }   // end switch(can1_request_index)

                break;  // end case CCP_RESPONSE_ID:


                case FORD_T3_ECU1_CAN1_XCP_EA4_REQUEST_ID:    // 8x730    canMESSAGE_BOX8 11bit identifier-- Trans XCP Request from EPS for data
                    break;

                case FORD_T3_ECU1_CAN1_XCP_EA4_RESPONSE_ID:   // 0x738    canMESSAGE_BOX9 11bit identifier-- Rec XCP Response from EPS for data
                    // receive and decide what the REQ was, parse and store
                    canGetData(canREG1, FORD_T3_ECU1_CAN1_XCP_EA4_RESPONSE_ID, can1_rx_data); // get data, store in rx_data array

                    switch(ECU1_XCP_reply_index)    // requesting multiple requests using same messageBox, DCT_XCP_rply_indx keeps track, clear_dtcs_flag determines action
                    {

                    case 1:
                        //EA4 is different
                        if((can1_rx_data[1] == 0x50) && (can1_rx_data[2] == 0x7E))  // positive response from enter Nexteer mode for FCA
                        {
                            if(ECU1_clear_DTCs_flag == 1)        // clear DTC's mode
                            {
                                canTransmit(canREG1, FORD_T3_ECU1_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_clear_DTC_rqst1);    // request clear DTCs
                            }
                            else       // get DTC's mode
                            {
                                canTransmit(canREG1, FORD_T3_ECU1_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_get_DTC_rqst1);  // request DTCs first string
                            }

                            ECU1_XCP_reply_index++;
                        }  // end positive response nexteer mode
                        break;

                    case 2:

                        if((can1_rx_data[0] == 0x10) && (can1_rx_data[2] == 0x62))    // get DTC's mode
                        {
                            dtc1[0][0] =  can1_rx_data[5];
                            dtc1[0][1] =  can1_rx_data[6];
                            dtc1[0][2] =  can1_rx_data[7];

                            canTransmit(canREG1, FORD_T3_ECU1_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_get_DTC_rqst2);   // request DTCs 2nd string

                            ECU1_XCP_reply_index++;
                        }

                        if((can1_rx_data[0] == 0x04) && (can1_rx_data[1] == 0x71))  // Response to clear DTC's mode
                        {
                            if(ECU1_clear_DTCs_flag == 1)
                            {
                                ECU1_XCP_reply_index = 0;
                                ECU1_clear_DTCs_flag = 0;   // reset flag and index, done
                            }
                        }

                        break;

                    case 3:
                        if(can1_rx_data[0] == 0x21)
                        {
                            dtc1[0][3] =  can1_rx_data[1];
                            dtc1[0][4] =  can1_rx_data[2];
                            dtc1[1][0] =  can1_rx_data[3];
                            dtc1[1][1] =  can1_rx_data[4];
                            dtc1[1][2] =  can1_rx_data[5];
                            dtc1[1][3] =  can1_rx_data[6];
                            dtc1[1][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 4:
                        if(can1_rx_data[0] == 0x22)
                        {
                            dtc1[2][0] =  can1_rx_data[1];
                            dtc1[2][1] =  can1_rx_data[2];
                            dtc1[2][2] =  can1_rx_data[3];
                            dtc1[2][3] =  can1_rx_data[4];
                            dtc1[2][4] =  can1_rx_data[5];
                            dtc1[3][0] =  can1_rx_data[6];
                            dtc1[3][1] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 5:
                        if(can1_rx_data[0] == 0x23)
                        {
                            dtc1[3][2] =  can1_rx_data[1];
                            dtc1[3][3] =  can1_rx_data[2];
                            dtc1[3][4] =  can1_rx_data[3];
                            dtc1[4][0] =  can1_rx_data[4];
                            dtc1[4][1] =  can1_rx_data[5];
                            dtc1[4][2] =  can1_rx_data[6];
                            dtc1[4][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 6:
                        if(can1_rx_data[0] == 0x24)
                        {
                            dtc1[4][4] =  can1_rx_data[1];
                            dtc1[5][0] =  can1_rx_data[2];
                            dtc1[5][1] =  can1_rx_data[3];
                            dtc1[5][2] =  can1_rx_data[4];
                            dtc1[5][3] =  can1_rx_data[5];
                            dtc1[5][4] =  can1_rx_data[6];
                            dtc1[6][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 7:
                        if(can1_rx_data[0] == 0x25)
                        {
                            dtc1[6][1] =  can1_rx_data[1];
                            dtc1[6][2] =  can1_rx_data[2];
                            dtc1[6][3] =  can1_rx_data[3];
                            dtc1[6][4] =  can1_rx_data[4];
                            dtc1[7][0] =  can1_rx_data[5];
                            dtc1[7][1] =  can1_rx_data[6];
                            dtc1[7][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 8:
                        if(can1_rx_data[0] == 0x26)
                        {
                            dtc1[7][3] =  can1_rx_data[1];
                            dtc1[7][4] =  can1_rx_data[2];
                            dtc1[8][0] =  can1_rx_data[3];
                            dtc1[8][1] =  can1_rx_data[4];
                            dtc1[8][2] =  can1_rx_data[5];
                            dtc1[8][3] =  can1_rx_data[6];
                            dtc1[8][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 9:
                        if(can1_rx_data[0] == 0x27)
                        {
                            dtc1[9][0] =  can1_rx_data[1];
                            dtc1[9][1] =  can1_rx_data[2];
                            dtc1[9][2] =  can1_rx_data[3];
                            dtc1[9][3] =  can1_rx_data[4];
                            dtc1[9][4] =  can1_rx_data[5];
                            dtc1[10][0] = can1_rx_data[6];
                            dtc1[10][1] = can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 10:
                        if(can1_rx_data[0] == 0x28)
                        {
                            dtc1[10][2] =  can1_rx_data[1];
                            dtc1[10][3] =  can1_rx_data[2];
                            dtc1[10][4] =  can1_rx_data[3];
                            dtc1[11][0] =  can1_rx_data[4];
                            dtc1[11][1] =  can1_rx_data[5];
                            dtc1[11][2] =  can1_rx_data[6];
                            dtc1[11][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 11:
                        if(can1_rx_data[0] == 0x29)
                        {
                            dtc1[11][4] =  can1_rx_data[1];
                            dtc1[12][0] =  can1_rx_data[2];
                            dtc1[12][1] =  can1_rx_data[3];
                            dtc1[12][2] =  can1_rx_data[4];
                            dtc1[12][3] =  can1_rx_data[5];
                            dtc1[12][4] =  can1_rx_data[6];
                            dtc1[13][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 12:
                        if(can1_rx_data[0] == 0x2A)
                        {
                            dtc1[13][1] =  can1_rx_data[1];
                            dtc1[13][2] =  can1_rx_data[2];
                            dtc1[13][3] =  can1_rx_data[3];
                            dtc1[13][4] =  can1_rx_data[4];
                            dtc1[14][0] =  can1_rx_data[5];
                            dtc1[14][1] =  can1_rx_data[6];
                            dtc1[14][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 13:
                        if(can1_rx_data[0] == 0x2B)
                        {
                            dtc1[14][3] =  can1_rx_data[1];
                            dtc1[14][4] =  can1_rx_data[2];

                            dtc1[14][3] =  can1_rx_data[1];
                            dtc1[14][4] =  can1_rx_data[2];
                            dtc1[15][0] =  can1_rx_data[3];
                            dtc1[15][1] =  can1_rx_data[4];
                            dtc1[15][2] =  can1_rx_data[5];
                            dtc1[15][3] =  can1_rx_data[6];
                            dtc1[15][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index = 14;
                        break;

                    case 14:
                        if(can1_rx_data[0] == 0x2C)
                        {
                            dtc1[16][0] =  can1_rx_data[1];
                            dtc1[16][1] =  can1_rx_data[2];
                            dtc1[16][2] =  can1_rx_data[3];
                            dtc1[16][3] =  can1_rx_data[4];
                            dtc1[16][4] =  can1_rx_data[5];
                            dtc1[17][0] =  can1_rx_data[6];
                            dtc1[17][1] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 15:
                        if(can1_rx_data[0] == 0x2D)
                        {
                            dtc1[17][2] =  can1_rx_data[1];
                            dtc1[17][3] =  can1_rx_data[2];
                            dtc1[17][4] =  can1_rx_data[3];
                            dtc1[18][0] =  can1_rx_data[4];
                            dtc1[18][1] =  can1_rx_data[5];
                            dtc1[18][2] =  can1_rx_data[6];
                            dtc1[18][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 16:
                        if(can1_rx_data[0] == 0x2E)
                        {
                            dtc1[18][4] =  can1_rx_data[1];
                            dtc1[19][0] =  can1_rx_data[2];
                            dtc1[19][1] =  can1_rx_data[3];
                            dtc1[19][2] =  can1_rx_data[4];
                            dtc1[19][3] =  can1_rx_data[5];
                            dtc1[19][4] =  can1_rx_data[6];
                        }
                        ECU1_XCP_reply_index = 17;  // last read set to 17 for get_DTCs function
                        break;

                    case 100:
                        ECU1_XCP_reply_index = 101;     // case used for multi message for TOC
                        break;

                    case 101:
                        ECU1_XCP_reply_index = 102;     // case used for multi message for TOC
                        break;

                    default:
                        ECU1_XCP_reply_index = 0;
                        break;

                    } // end switch on DTC_XCP_rply_indx

                    break; // end case FORD_T3_ECU1_CAN1_XCP_EA4_RESPONSE_ID

                    case FORD_T3_ECU2_CAN1_CCP_REQUEST_ID:        // 0x50A    canMESSAGE_BOX10    11bit identifier-- Trans CCP Request for Data to EPS
                        break;

                    case FORD_T3_ECU2_CAN1_CCP_RESPONSE_ID:       // 0x50B    canMESSAGE_BOX11    11bit identifier-- Rec CCP Response from EPS for Data
                        // Receive and decide what the REQ was, parse and store
                        // Standard dump of all ccp request

                        canGetData(canREG1, FORD_T3_ECU2_CAN1_CCP_RESPONSE_ID, can2_rx_data); // get data, store in rx_data array

                        switch(can2_request_index)      // requesting multiple variables using same messageBox, can2_request_index keeps track, store and parse in main
                        {
                        case 0:     // do nothing with request_connect data, setup and trans next request

                            canTransmit(canREG1, FORD_T3_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU2_BattVltg_str);
                            can2_request_index++;       // bump index so next receive will switch to correct message
                            break;

                        case 1:     // store ccp_FORD_T3_ECU2_BattVltg_str, setup and trans next request

                            memcpy(&can2_dump_err_data[1],&can2_rx_data,8);

                            canTransmit(canREG1, FORD_T3_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU2_Temperature_str);
                            can2_request_index++;       // bump index so next receive will switch to correct message
                            break;

                        case 2:     // store ccp_FORD_T3_ECU2_Temperature_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[2],&can2_rx_data,8);

                            canTransmit(canREG1, FORD_T3_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU2_DigT1_str);
                            can2_request_index++;       // bump index so next receive will switch to correct message
                            break;

                        case 3:     // store ccp_FORD_T3_ECU2_DigT1_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[3],&can2_rx_data,8);

                            canTransmit(canREG1, FORD_T3_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU2_DigT2_str);
                            can2_request_index++;       // bump index so next receive will switch to correct message
                            break;

                        case 4:     // store ccp_FORD_T3_ECU2_DigT2_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[4],&can2_rx_data,8);

                            canTransmit(canREG1, FORD_T3_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU2_MtrCurrQax_str);
                            can2_request_index++;       // bump index so next receive will switch to correct message
                            break;

                        case 5:     // store ccp_FORD_T3_ECU2_MtrCurrQax_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[5],&can2_rx_data,8);

                            canTransmit(canREG1, FORD_T3_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU2_SystemState_byt);
                            can2_request_index++;       // bump index so next receive will switch to correct message
                            break;

                        case 6:     // store ccp_FORD_T3_ECU2_SystemState_byt, setup and trans next reques

                            memcpy(&can2_dump_err_data[6],&can2_rx_data,8);

                            canTransmit(canREG1, FORD_T3_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU2_HwTrq_HwNm_str);
                            can2_request_index++;       // bump index so next receive will switch to correct message
                            break;

                        case 7:     // store ccp_FORD_T3_ECU2_HwTrq_HwNm_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[7],&can2_rx_data,8);

                            canTransmit(canREG1, FORD_T3_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU2_Abs_Hw_Pos_str);
                            can2_request_index++;       // bump index so next receive will switch to correct message
                            break;

                        case 8:     // store ccp_FORD_T3_ECU2_Abs_Hw_Pos_str, setup and trans next reques
                            //  all done, index bumped for main to test

                            memcpy(&can2_dump_err_data[8],&can2_rx_data,8);

                            can2_request_index++;       // bump index so next receive will switch to correct message
                            break;


                            //  ----------------   H c u r r    G2KCA reading cases 2000-2031 done during normal testing

                        case 2000:      // Hcurr Qax readings(x3) // store ccp_FORD_T3_ECU2_MtrCurrQax_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[9],&can2_rx_data,8);

                            canTransmit(canREG1, FORD_T3_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU2_HwTrq_HwNm_str);
                            can2_request_index++;       // bump index so next receive will switch to correct message

                            break;

                        case 2001:      // store ccp_FORD_T3_ECU2_HwTrq_HwNm_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[10],&can2_rx_data,8);

                            can2_request_index = 2999;      // arbitrary value, all done for now

                            break;

                        case 2010:      // store ccp_FORD_T3_ECU2_MtrCurrQax_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[11],&can2_rx_data,8);

                            canTransmit(canREG1, FORD_T3_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU2_HwTrq_HwNm_str);
                            can2_request_index++;       // bump index so next receive will switch to correct message

                            break;

                        case 2011:      // store ccp_FORD_T3_ECU2_HwTrq_HwNm_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[12],&can2_rx_data,8);

                            can2_request_index = 2999;      // arbitrary value, all done for now

                            break;

                        case 2020:      // store ccp_FORD_T3_ECU2_MtrCurrQax_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[13],&can2_rx_data,8);

                            canTransmit(canREG1, FORD_T3_ECU2_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_T3_ECU2_HwTrq_HwNm_str);
                            can2_request_index++;       // bump index so next receive will switch to correct message

                            break;
                        case 2021:      // store ccp_FORD_T3_ECU2_HwTrq_HwNm_str, setup and trans next reques

                            memcpy(&can2_dump_err_data[14],&can2_rx_data,8);

                            can2_request_index = 2999;      // arbitrary value, all done for now

                            break;

                        case 2999:

                            can2_request_index = 3000;  // don't care response is done...ok to send new request.

                            break;

                        default:
                        {
                        }

                        }   // end switch(can1_request_index)

                        break;  // end case CCP_RESPONSE_ID:


                        case FORD_T3_ECU2_CAN1_XCP_EA4_REQUEST_ID:    // 0x630   canMESSAGE_BOX12    11bit identifier-- Trans XCP Request from EPS for data
                            break;

                        case FORD_T3_ECU2_CAN1_XCP_EA4_RESPONSE_ID:    // 0x638   canMESSAGE_BOX13    11bit identifier-- Rec XCP Response from EPS for data
                            // receive and decide what the REQ was, parse and store
                            canGetData(canREG1, FORD_T3_ECU2_CAN1_XCP_EA4_RESPONSE_ID, can2_rx_data); // get data, store in rx_data array

                            switch(ECU2_XCP_reply_index)    // requesting multiple requests using same messageBox, DCT_XCP_rply_indx keeps track, clear_dtcs_flag determines action
                            {

                            case 1:
                                //EA4 is different
                                if((can2_rx_data[1] == 0x50) && (can2_rx_data[2] == 0x7E))  // positive response from enter Nexteer mode for FCA
                                {
                                    if(ECU2_clear_DTCs_flag == 1)        // clear DTC's mode
                                    {
                                        canTransmit(canREG1, FORD_T3_ECU2_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_clear_DTC_rqst1);    // request clear DTCs
                                    }
                                    else       // get DTC's mode
                                    {
                                        canTransmit(canREG1, FORD_T3_ECU2_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_get_DTC_rqst1);  // request DTCs first string
                                    }

                                    ECU2_XCP_reply_index++;
                                }  // end positive response nexteer mode
                                break;

                            case 2:

                                if((can2_rx_data[0] == 0x10) && (can2_rx_data[2] == 0x62))    // get DTC's mode
                                {
                                    dtc2[0][0] =  can2_rx_data[5];
                                    dtc2[0][1] =  can2_rx_data[6];
                                    dtc2[0][2] =  can2_rx_data[7];

                                    canTransmit(canREG1, FORD_T3_ECU2_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_get_DTC_rqst2);   // request DTCs 2nd string

                                    ECU2_XCP_reply_index++;
                                }

                                if((can2_rx_data[0] == 0x04) && (can2_rx_data[1] == 0x71))  // Response to clear DTC's mode
                                {
                                    if(ECU2_clear_DTCs_flag == 1)
                                    {
                                        ECU2_XCP_reply_index = 0;
                                        ECU2_clear_DTCs_flag = 0;   // reset flag and index, done
                                    }
                                }

                                break;

                            case 3:
                                if(can2_rx_data[0] == 0x21)
                                {
                                    dtc2[0][3] =  can2_rx_data[1];
                                    dtc2[0][4] =  can2_rx_data[2];
                                    dtc2[1][0] =  can2_rx_data[3];
                                    dtc2[1][1] =  can2_rx_data[4];
                                    dtc2[1][2] =  can2_rx_data[5];
                                    dtc2[1][3] =  can2_rx_data[6];
                                    dtc2[1][4] =  can2_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 4:
                                if(can2_rx_data[0] == 0x22)
                                {
                                    dtc2[2][0] =  can2_rx_data[1];
                                    dtc2[2][1] =  can2_rx_data[2];
                                    dtc2[2][2] =  can2_rx_data[3];
                                    dtc2[2][3] =  can2_rx_data[4];
                                    dtc2[2][4] =  can2_rx_data[5];
                                    dtc2[3][0] =  can2_rx_data[6];
                                    dtc2[3][1] =  can2_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 5:
                                if(can2_rx_data[0] == 0x23)
                                {
                                    dtc2[3][2] =  can2_rx_data[1];
                                    dtc2[3][3] =  can2_rx_data[2];
                                    dtc2[3][4] =  can2_rx_data[3];
                                    dtc2[4][0] =  can2_rx_data[4];
                                    dtc2[4][1] =  can2_rx_data[5];
                                    dtc2[4][2] =  can2_rx_data[6];
                                    dtc2[4][3] =  can2_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 6:
                                if(can2_rx_data[0] == 0x24)
                                {
                                    dtc2[4][4] =  can2_rx_data[1];
                                    dtc2[5][0] =  can2_rx_data[2];
                                    dtc2[5][1] =  can2_rx_data[3];
                                    dtc2[5][2] =  can2_rx_data[4];
                                    dtc2[5][3] =  can2_rx_data[5];
                                    dtc2[5][4] =  can2_rx_data[6];
                                    dtc2[6][0] =  can2_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 7:
                                if(can2_rx_data[0] == 0x25)
                                {
                                    dtc2[6][1] =  can2_rx_data[1];
                                    dtc2[6][2] =  can2_rx_data[2];
                                    dtc2[6][3] =  can2_rx_data[3];
                                    dtc2[6][4] =  can2_rx_data[4];
                                    dtc2[7][0] =  can2_rx_data[5];
                                    dtc2[7][1] =  can2_rx_data[6];
                                    dtc2[7][2] =  can2_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 8:
                                if(can2_rx_data[0] == 0x26)
                                {
                                    dtc2[7][3] =  can2_rx_data[1];
                                    dtc2[7][4] =  can2_rx_data[2];
                                    dtc2[8][0] =  can2_rx_data[3];
                                    dtc2[8][1] =  can2_rx_data[4];
                                    dtc2[8][2] =  can2_rx_data[5];
                                    dtc2[8][3] =  can2_rx_data[6];
                                    dtc2[8][4] =  can2_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 9:
                                if(can2_rx_data[0] == 0x27)
                                {
                                    dtc2[9][0] =  can2_rx_data[1];
                                    dtc2[9][1] =  can2_rx_data[2];
                                    dtc2[9][2] =  can2_rx_data[3];
                                    dtc2[9][3] =  can2_rx_data[4];
                                    dtc2[9][4] =  can2_rx_data[5];
                                    dtc2[10][0] = can2_rx_data[6];
                                    dtc2[10][1] = can2_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 10:
                                if(can2_rx_data[0] == 0x28)
                                {
                                    dtc2[10][2] =  can2_rx_data[1];
                                    dtc2[10][3] =  can2_rx_data[2];
                                    dtc2[10][4] =  can2_rx_data[3];
                                    dtc2[11][0] =  can2_rx_data[4];
                                    dtc2[11][1] =  can2_rx_data[5];
                                    dtc2[11][2] =  can2_rx_data[6];
                                    dtc2[11][3] =  can2_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 11:
                                if(can2_rx_data[0] == 0x29)
                                {
                                    dtc2[11][4] =  can2_rx_data[1];
                                    dtc2[12][0] =  can2_rx_data[2];
                                    dtc2[12][1] =  can2_rx_data[3];
                                    dtc2[12][2] =  can2_rx_data[4];
                                    dtc2[12][3] =  can2_rx_data[5];
                                    dtc2[12][4] =  can2_rx_data[6];
                                    dtc2[13][0] =  can2_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 12:
                                if(can2_rx_data[0] == 0x2A)
                                {
                                    dtc2[13][1] =  can2_rx_data[1];
                                    dtc2[13][2] =  can2_rx_data[2];
                                    dtc2[13][3] =  can2_rx_data[3];
                                    dtc2[13][4] =  can2_rx_data[4];
                                    dtc2[14][0] =  can2_rx_data[5];
                                    dtc2[14][1] =  can2_rx_data[6];
                                    dtc2[14][2] =  can2_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 13:
                                if(can2_rx_data[0] == 0x2B)
                                {
                                    dtc2[14][3] =  can1_rx_data[1];
                                    dtc2[14][4] =  can1_rx_data[2];

                                    dtc2[14][3] =  can1_rx_data[1];
                                    dtc2[14][4] =  can1_rx_data[2];
                                    dtc2[15][0] =  can1_rx_data[3];
                                    dtc2[15][1] =  can1_rx_data[4];
                                    dtc2[15][2] =  can1_rx_data[5];
                                    dtc2[15][3] =  can1_rx_data[6];
                                    dtc2[15][4] =  can1_rx_data[7];
                                }
                                ECU2_XCP_reply_index = 14;
                                break;

                            case 14:
                                if(can2_rx_data[0] == 0x2C)
                                {
                                    dtc2[16][0] =  can1_rx_data[1];
                                    dtc2[16][1] =  can1_rx_data[2];
                                    dtc2[16][2] =  can1_rx_data[3];
                                    dtc2[16][3] =  can1_rx_data[4];
                                    dtc2[16][4] =  can1_rx_data[5];
                                    dtc2[17][0] =  can1_rx_data[6];
                                    dtc2[17][1] =  can1_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 15:
                                if(can2_rx_data[0] == 0x2D)
                                {
                                    dtc2[17][2] =  can1_rx_data[1];
                                    dtc2[17][3] =  can1_rx_data[2];
                                    dtc2[17][4] =  can1_rx_data[3];
                                    dtc2[18][0] =  can1_rx_data[4];
                                    dtc2[18][1] =  can1_rx_data[5];
                                    dtc2[18][2] =  can1_rx_data[6];
                                    dtc2[18][3] =  can1_rx_data[7];
                                }
                                ECU2_XCP_reply_index++;
                                break;

                            case 16:
                                if(can2_rx_data[0] == 0x2E)
                                {
                                    dtc2[18][4] =  can1_rx_data[1];
                                    dtc2[19][0] =  can1_rx_data[2];
                                    dtc2[19][1] =  can1_rx_data[3];
                                    dtc2[19][2] =  can1_rx_data[4];
                                    dtc2[19][3] =  can1_rx_data[5];
                                    dtc2[19][4] =  can1_rx_data[6];
                                }
                                ECU2_XCP_reply_index = 17;  // last read set to 17 for get_DTCs function
                                break;

                            case 100:
                                ECU2_XCP_reply_index = 101;     // case used for multi message for TOC
                                break;

                            case 101:
                                ECU2_XCP_reply_index = 102;     // case used for multi message for TOC
                                break;

                            default:
                                ECU2_XCP_reply_index = 0;
                                break;

                            } // end switch on DTC_XCP_rply_indx

                            break; // end case FORD_T3_ECU1_CAN1_XCP_EA4_RESPONSE_ID

                            case FORD_T3_ECU1_CAN1_TESTER_REC_DEBUG_ID:             // 0x07FF   canMESSAGE_BOX14    11bit identifier-- REC from Anyone
                                break;

                            case FORD_T3_ECU1_CAN1_TESTER_TRANS_DEBUG_ID:           // 0x07FE   canMESSAGE_BOX15        11bit identifier-- Transmit to Anyone
                                break;

                            case canMESSAGE_BOX16:                      // id 16   11bit identifier-- Not Used
                                break;

                            default:

                            {
                                can1_message_type_debug = messageBox;  // an unknown messageBox
                            }

                            break;     // end switch(messageBox) for TARGET_FORD_T3_T6 on canREG1

            }   // end switch(messageBox) for TARGET_FORD_T3_T6 on canREG1

        }  // end case TARGET_FORD_T3_T6     ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^    F O R D   T 3   -   T 6   C A N 1 ^^^^^^^^^^^^^^^^^^^^^^^^^^^
        break;

        case TARGET_RENAULT_NISSAN:       //  VVVVVVVVVVVVVVVVVVVVVVVVVVVVV  N A U L T N I S S A N  C A N 1  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
        {
            switch(messageBox)
            {
            case RENAULT_NISSAN_ECU1_CAN1_PROD_MESS1_ID:     // 0x148    canMESSAGE_BOX1 11bit identifier-- Rec Response from EPS (not used)
                break;

            case RENAULT_NISSAN_ECU1_CAN1_PROD_MESS2_ID:     // 0x778    canMESSAGE_BOX2 11bit identifier-- Rec ON Star from EPS  (not used)
                break;

            case RENAULT_NISSAN_ECU1_CAN1_CCP_Engine_Run:    // 0x0C9    canMESSAGE_BOX3 11bit identifier-- Trans Engine ON to EPS
                break;

            case RENAULT_NISSAN_ECU1_CAN1_CCP_Power_Mode_ID: // 0x1F1    canMESSAGE_BOX4 11bit identifier-- Trans Power Mode to EPS
                break;

            case RENAULT_NISSAN_ECU1_CAN1_CCP_SPEED_ID:      // 0x348    canMESSAGE_BOX5 11bit identifier-- Trans Speed and Validity Status to EPS

                break;

            case RENAULT_NISSAN_ECU1_CAN1_CCP_REQUEST_ID:   // 0x708    canMESSAGE_BOX6 11bit identifier-- Trans CCP Request for Data to EPS
                break;


            case RENAULT_NISSAN_ECU1_CAN1_CCP_RESPONSE_ID: // 0x706  canMESSAGE_BOX7 11bit identifier-- Rec CCP response from EPS
                // receive and decide what the REQ was, parse and store
                // Standard dump of all ccp request

                canGetData(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_RESPONSE_ID, can1_rx_data); // get data, store in rx_data array

                switch(can1_request_index)      // requesting multiple variables using same messageBox, can1_request_index keeps track, store and parse in main
                {

                case 0:     // do nothing with request_connect data, setup and trans next request

                    canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_RENAULT_NISSAN_BattVltg_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 1:     // store ccp_RENAULT_NISSAN_BattVltg_str, setup and trans next request

                    memcpy(&can1_dump_err_data[1],&can1_rx_data,8);

                    canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_RENAULT_NISSAN_Temperature_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 2:     // store ccp_RENAULT_NISSAN_Temperature_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[2],&can1_rx_data,8);

                    canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_RENAULT_NISSAN_DigT1_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 3:     // store ccp_RENAULT_NISSAN_DigT1_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[3],&can1_rx_data,8);

                    canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_RENAULT_NISSAN_DigT2_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 4:     // store ccp_RENAULT_NISSAN_DigT2_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[4],&can1_rx_data,8);

                    canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_RENAULT_NISSAN_MtrCurrQax_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 5:     // store ccp_RENAULT_NISSAN_MtrCurrQax_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[5],&can1_rx_data,8);

                    canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_RENAULT_NISSAN_SystemState_byt);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 6:     // store ccp_RENAULT_NISSAN_SystemState_byt, setup and trans next reques

                    memcpy(&can1_dump_err_data[6],&can1_rx_data,8);

                    canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_RENAULT_NISSAN_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 7:     // store ccp_RENAULT_NISSAN_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[7],&can1_rx_data,8);

                    canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_RENAULT_NISSAN_Abs_Hw_Pos_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 8:     // store ccp_RENAULT_NISSAN_Abs_Hw_Pos_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[8],&can1_rx_data,8);

                    canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_RENAULT_NISSAN_MotTrq_Crf_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 9:     //  store ccp_RENAULT_NISSAN_MotTrq_Crf_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[9],&can1_rx_data,8);

                    canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_RENAULT_NISSAN_MotTrq_Mrf_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 10:        // store ccp_RENAULT_NISSAN_MotTrq_Mrf_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[10],&can1_rx_data,8);

                    canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_RENAULT_NISSAN_VehSpd_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 11:        // store ccp_RENAULT_NISSAN_VehSpd_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[11],&can1_rx_data,8);

                    canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_RENAULT_NISSAN_Spare2_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 12:        // store ccp_RENAULT_NISSAN_Spare2_str, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can1_dump_err_data[12],&can1_rx_data,8);

                    can1_request_index++;       // bump index so next receive will switch to correct message
                    break;



                    //  ----------------   H c u r r    RENAULT_NISSAN reading cases 2000-2031 done during normal testing

                case 2000:      // Hcurr Qax readings(x3) // store ccp_RENAULT_NISSAN_MtrCurrQax_str 1, setup and trans next reques

                    memcpy(&can1_dump_err_data[13],&can1_rx_data,8);

                    canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_RENAULT_NISSAN_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2001:      // store ccp_RENAULT_NISSAN_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[14],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2010:      // store ccp_RENAULT_NISSAN_MtrCurrQax_str 2, setup and trans next reques

                    memcpy(&can1_dump_err_data[15],&can1_rx_data,8);

                    canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_RENAULT_NISSAN_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2011:      // store ccp_RENAULT_NISSAN_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[16],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2020:      // store ccp_RENAULT_NISSAN_MtrCurrQax_str 3, setup and trans next reques

                    memcpy(&can1_dump_err_data[17],&can1_rx_data,8);

                    canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_CCP_REQUEST_ID, ( uint8 *) ccp_RENAULT_NISSAN_HwTrq_HwNm_str);
                    can1_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2021:      // store ccp_RENAULT_NISSAN_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can1_dump_err_data[18],&can1_rx_data,8);

                    can1_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2999:

                    can1_request_index = 3000;  // don't care response is done...ok to send new request.

                    break;

                default:
                {
                }

                }   // end switch(can1_request_index) for RENAULT_NISSAN  C A N 1

                break;  // end case RENAULT_NISSAN_ECU1_CAN1_CCP_RESPONSE_ID:


                case RENAULT_NISSAN_ECU1_CAN1_XCP_EA3_REQUEST_ID:     // 0x242   canMESSAGE_BOX8 11bit identifier-- Trans CCP Request for Data to EPS
                    break;

                case RENAULT_NISSAN_ECU1_CAN1_XCP_EA3_RESPONSE_ID:    // 0x642   canMESSAGE_BOX9 11bit identifier-- Rec XCP Response from EPS for data
                    // receive and decide what the REQ was, parse and store

                    break; // end case XCP_RESPONSE_ID

                case RENAULT_NISSAN_ECU1_CAN1_XCP_EA4_REQUEST_ID:     // 0x712   canMESSAGE_BOX10 11bit identifier-- Trans XCP Request from EPS for data
                    break;

                case RENAULT_NISSAN_ECU1_CAN1_XCP_EA4_RESPONSE_ID:     // 0x710   canMESSAGE_BOX11 11bit identifier-- Rec XCP Response to EPS for data
                    canGetData(canREG1, RENAULT_NISSAN_ECU1_CAN1_XCP_EA4_RESPONSE_ID, can1_rx_data); // get data, store in rx_data array

                    switch(ECU1_XCP_reply_index)    // requesting multiple requests using same messageBox, DCT_XCP_rply_indx keeps track, clear_dtcs_flag determines action
                    {

                    case 1:
                        //EA4 is different
                        if((can1_rx_data[1] == 0x50) && (can1_rx_data[2] == 0x7E))  // positive response from enter Nexteer mode
                        {
                            if(ECU1_clear_DTCs_flag == 1)        // clear DTC's mode
                            {
                                canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_clear_DTC_rqst1);    // request clear DTCs
                            }

                            else       // get DTC's mode

                            {
                                canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_get_DTC_rqst1);  // request DTCs first string
                            }

                            ECU1_XCP_reply_index++;
                        }  // end positive response nexteer mode
                        break;

                    case 2:

                        if((can1_rx_data[0] == 0x10) && (can1_rx_data[2] == 0x62))    // get DTC's mode
                        {

                            dtc1[0][0] =  can1_rx_data[5];
                            dtc1[0][1] =  can1_rx_data[6];
                            dtc1[0][2] =  can1_rx_data[7];

                            canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_EA4_write_get_DTC_rqst2);   // request DTCs 2nd string

                            ECU1_XCP_reply_index++;
                        }

                        if((can1_rx_data[0] == 0x04) && (can1_rx_data[1] == 0x71))  // responce to clear DTC's mode
                        {
                            if(ECU1_clear_DTCs_flag == 1)
                            {
                                ECU1_XCP_reply_index = 0;
                                ECU1_clear_DTCs_flag = 0;   // reset flag and index, done
                            }
                        }

                        break;

                    case 3:
                        if(can1_rx_data[0] == 0x21)
                        {
                            dtc1[0][3] =  can1_rx_data[1];
                            dtc1[0][4] =  can1_rx_data[2];
                            dtc1[1][0] =  can1_rx_data[3];
                            dtc1[1][1] =  can1_rx_data[4];
                            dtc1[1][2] =  can1_rx_data[5];
                            dtc1[1][3] =  can1_rx_data[6];
                            dtc1[1][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 4:
                        if(can1_rx_data[0] == 0x22)
                        {
                            dtc1[2][0] =  can1_rx_data[1];
                            dtc1[2][1] =  can1_rx_data[2];
                            dtc1[2][2] =  can1_rx_data[3];
                            dtc1[2][3] =  can1_rx_data[4];
                            dtc1[2][4] =  can1_rx_data[5];
                            dtc1[3][0] =  can1_rx_data[6];
                            dtc1[3][1] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 5:
                        if(can1_rx_data[0] == 0x23)
                        {
                            dtc1[3][2] =  can1_rx_data[1];
                            dtc1[3][3] =  can1_rx_data[2];
                            dtc1[3][4] =  can1_rx_data[3];
                            dtc1[4][0] =  can1_rx_data[4];
                            dtc1[4][1] =  can1_rx_data[5];
                            dtc1[4][2] =  can1_rx_data[6];
                            dtc1[4][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;

                        break;

                    case 6:
                        if(can1_rx_data[0] == 0x24)
                        {
                            dtc1[4][4] =  can1_rx_data[1];
                            dtc1[5][0] =  can1_rx_data[2];
                            dtc1[5][1] =  can1_rx_data[3];
                            dtc1[5][2] =  can1_rx_data[4];
                            dtc1[5][3] =  can1_rx_data[5];
                            dtc1[5][4] =  can1_rx_data[6];
                            dtc1[6][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 7:
                        if(can1_rx_data[0] == 0x25)
                        {
                            dtc1[6][1] =  can1_rx_data[1];
                            dtc1[6][2] =  can1_rx_data[2];
                            dtc1[6][3] =  can1_rx_data[3];
                            dtc1[6][4] =  can1_rx_data[4];
                            dtc1[7][0] =  can1_rx_data[5];
                            dtc1[7][1] =  can1_rx_data[6];
                            dtc1[7][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 8:
                        if(can1_rx_data[0] == 0x26)
                        {
                            dtc1[7][3] =  can1_rx_data[1];
                            dtc1[7][4] =  can1_rx_data[2];
                            dtc1[8][0] =  can1_rx_data[3];
                            dtc1[8][1] =  can1_rx_data[4];
                            dtc1[8][2] =  can1_rx_data[5];
                            dtc1[8][3] =  can1_rx_data[6];
                            dtc1[8][4] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 9:
                        if(can1_rx_data[0] == 0x27)
                        {
                            dtc1[9][0] =  can1_rx_data[1];
                            dtc1[9][1] =  can1_rx_data[2];
                            dtc1[9][2] =  can1_rx_data[3];
                            dtc1[9][3] =  can1_rx_data[4];
                            dtc1[9][4] =  can1_rx_data[5];
                            dtc1[10][0] = can1_rx_data[6];
                            dtc1[10][1] = can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 10:
                        if(can1_rx_data[0] == 0x28)
                        {
                            dtc1[10][2] =  can1_rx_data[1];
                            dtc1[10][3] =  can1_rx_data[2];
                            dtc1[10][4] =  can1_rx_data[3];
                            dtc1[11][0] =  can1_rx_data[4];
                            dtc1[11][1] =  can1_rx_data[5];
                            dtc1[11][2] =  can1_rx_data[6];
                            dtc1[11][3] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 11:
                        if(can1_rx_data[0] == 0x29)
                        {
                            dtc1[11][4] =  can1_rx_data[1];
                            dtc1[12][0] =  can1_rx_data[2];
                            dtc1[12][1] =  can1_rx_data[3];
                            dtc1[12][2] =  can1_rx_data[4];
                            dtc1[12][3] =  can1_rx_data[5];
                            dtc1[12][4] =  can1_rx_data[6];
                            dtc1[13][0] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 12:
                        if(can1_rx_data[0] == 0x2A)
                        {
                            dtc1[13][1] =  can1_rx_data[1];
                            dtc1[13][2] =  can1_rx_data[2];
                            dtc1[13][3] =  can1_rx_data[3];
                            dtc1[13][4] =  can1_rx_data[4];
                            dtc1[14][0] =  can1_rx_data[5];
                            dtc1[14][1] =  can1_rx_data[6];
                            dtc1[14][2] =  can1_rx_data[7];
                        }
                        ECU1_XCP_reply_index++;
                        break;

                    case 13:
                        if(can1_rx_data[0] == 0x2B)
                        {
                            dtc1[14][3] =  can1_rx_data[1];
                            dtc1[14][4] =  can1_rx_data[2];
                        }
                        ECU1_XCP_reply_index = 14;   // last read set to 14 for get_DTCs function
                        break;

                    case 100:
                        ECU1_XCP_reply_index = 101;     // case used for multi message for TOC
                        break;

                    case 101:
                        ECU1_XCP_reply_index = 102;     // case used for multi message for TOC
                        break;

                    default:

                        ECU1_XCP_reply_index = 0;

                        break;

                    } // end switch on DTC_XCP_rply_indx

                    break;

                    case RENAULT_NISSAN_TESTER_REC_DEBUG_ID:    // 0x7FF    canMESSAGE_BOX12 11bit identifier-- REC from Anyone

                        break;

                    case RENAULT_NISSAN_TESTER_TRANS_DEBUG_ID:  // 0x7FE     canMESSAGE_BOX13 11bit identifier-- Transmit to Anyone

                        break;

                    case RENAULT_NISSAN_ECU1_CCP_EA4_AvgVehSpd_ID:      // 0x14  11bit identifier-- Trans ABS message to EPS

                        break;

                    case RENAULT_NISSAN_ECU1_CAN1_GenericAppDiagEnble_ID:      // 0x15  11bit identifier-- Trans Generic Application Diagnostic Enablr

                        break;

                    case canMESSAGE_BOX16:      // 0x16  11bit identifier-- Not Used

                        break;

                    default:

                    {
                        can1_message_type_debug = messageBox;  // an unknown messageBox
                    }
                    break;

            }   // end switch(messageBox) for TARGET_RENAULT_NISSAN on canREG1

        }   // end case TARGET_RENAULT_NISSAN    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^  N A U L T N I S S A N   C A N 1  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

        break;  // break from case TARGET_RENAULT_NISSAN:


        }   // end switch(target_product)  C A N 1

    }   // end if(node == canREG1)

    if(node == canREG2)
    {
        switch(target_product)
        {

        case TARGET_BMW_FAAR_WE:
        {
            switch(messageBox)
            {
            case MTS_001_Vehicle_Speed_flt_in:     // Message box31 CAN ID 0x001 vehicle speed input

                MTS_running_flag = 1;
                MTS_running_counter = 8000;
                General_Fault_state_out_flag = 0;

                canGetData(canREG2, MTS_001_Vehicle_Speed_flt_in, can2_rx_data);    // get vehicle speed data
                memcpy(&MTS_Vehicle_Speed_flt,&can2_rx_data[0],4);                // store first 4 bytes float value of vehicle speed in variable

                if(MTS_Vehicle_Speed_flt > 350)    //Speed high limit
                {
                    MTS_Vehicle_Speed_flt = 350;
                }

                if(MTS_Vehicle_Speed_flt < 0)         //Speed low limit
                {
                    MTS_Vehicle_Speed_flt = 0;
                }

                float temp_MTS_Vehicle_Speed_flt;
                temp_MTS_Vehicle_Speed_flt = MTS_Vehicle_Speed_flt * 64;   // vehicle speed is KPH * 64

                MTS_Vehicle_Speed_uint16 = temp_MTS_Vehicle_Speed_flt;
                MTS_Vehicle_Speed_HEX_LoByte = MTS_Vehicle_Speed_uint16;
                MTS_Vehicle_Speed_HEX_HiByte = (MTS_Vehicle_Speed_uint16 >> 8);

                break;   // end case MTS_001_Vehicle_Speed_flt_in Message box31 vehicle speed input

            case L02_MTS_004_System_State_in:  // Message box32 CAN ID 0x004  System State In

                canGetData(canREG2, L02_MTS_004_System_State_in, can2_rx_data);    // get system state byte(0)
                MTS_System_State_HEX_Byte = can2_rx_data[0];                       // store system state in MTS_System_State_HEX_Byte

                if(MTS_System_State_HEX_Byte != MTS_System_State_HEX_Byte_old )
                {
                    MTS_System_State_HEX_Byte_old = MTS_System_State_HEX_Byte;
                    MTS_System_State_change_flag = 1;
                }

                break;  // end case L02_MTS_004_System_State_in: Message box32 CAN ID 0x003  System State In

            case L02_MTS_005_Store_DAQ_Data_Mode_in:  // Message box33 CAN ID 0x005  DAQ

                canGetData(canREG2, L02_MTS_005_Store_DAQ_Data_Mode_in, can2_rx_data);    // get DAQ data storage mode from MTSbyte(0)

                MTS_Store_DAQ_Data_Mode_HEX_Byte = can2_rx_data[0];
                // the variable 'MTS_Store_DAQ_Data_Mode_HEX_Byte' will be ouptut in the streaming data to the LabView app
                // the value of this variable will signal the LabView app to store (1) or not store(0) streaming data

                break;  // end case L02_MTS_005_Store_DAQ_Data_Mode_in Message box33 CAN ID 0x005

            case L02_MTS_D1_11D_Angle_Cmd_fp_in:  // Message box34 Angle command in from MTS for device #1

                canGetData(canREG2, L02_MTS_D1_11D_Angle_Cmd_fp_in, can2_rx_data);   // MTS angle command input- Only CIB ID that matches the switch settings will react

                if(CIB_config_data.BOARD_ID[0] == '1')        // CAN ID 0x11D for device #1 on CIB #1
                {
                    memcpy(&MTS_Angle_Cmd_fp_in,&can2_rx_data[0],4);
                    // SET Angle = New Angle with D/A output

                }

                break;   // END case get angle command from MTS for device #1

            case L02_MTS_D1_12D_Torque_Cmd_fp_in:  // Message box35 Torque command in from MTS for device #1

                canGetData(canREG2, L02_MTS_D1_12D_Torque_Cmd_fp_in, can2_rx_data);   // MTS torque command input- Only CIB ID that matches the switch settings will react

                if(CIB_config_data.BOARD_ID[0] == '1')        // CAN ID 0x12D for device #1 on CIB #1
                {
                    if((ECU1_XCP_reply_index == 0)&&(ign1_status > 0)) //check if in the middle of other stuff
                    {
                        memcpy(&MTS_Torque_Cmd_fp_in,&can2_rx_data[0],4);

                        //THIS WORKS, can use if needed: if(MTS_Torque_Cmd_fp_in != DUT_Motor_torque_fp) //check if the value from the product matches
                        //Want to try something else:
                        if(MTS_Torque_Cmd_fp_in != old_MTS_torque_Cmd_fp_in)
                        {
                            //now transmit new motor torque command based on input.
                            MFG_TX_mtr_trq_cmd_MSG[3][3] = can2_rx_data[0];
                            MFG_TX_mtr_trq_cmd_MSG[3][2] = can2_rx_data[1];
                            MFG_TX_mtr_trq_cmd_MSG[3][1] = can2_rx_data[2];
                            MFG_TX_mtr_trq_cmd_MSG[3][0] = can2_rx_data[3];

                            memcpy(&old_MTS_torque_Cmd_fp_in,&MTS_Torque_Cmd_fp_in,4);
                            ECU1_XCP_reply_index = 100; // used to get us to default case. Default case will set = 0.
                            transmitFlexray(MFG_TX_mtr_trq_cmd_MSG,MFG_TX, 4); //length of 3 is probabl fine
                        }
                    }
                }

                break;   // END case get Torque command from MTS for device #1

            case L02_MTS_D2_21D_Angle_Cmd_fp_in:  // Message box36 Angle command in from MTS for device #2

                canGetData(canREG2, L02_MTS_D2_21D_Angle_Cmd_fp_in, can2_rx_data);   // MTS angle command input- Only CIB ID that matches the switch settings will react

                if(CIB_config_data.BOARD_ID[0] == '2')        // CAN ID 0x21D for device #2 on CIB #2
                {
                    memcpy(&MTS_Angle_Cmd_fp_in,&can2_rx_data[0],4);
                    // SET Angle = New Angle with D/A output

                }

                break;   // END case get angle command from MTS for device #2

            case L02_MTS_D2_22D_Torque_Cmd_fp_in:  // Message box37 Torque command in from MTS for device #2

                canGetData(canREG2, L02_MTS_D2_22D_Torque_Cmd_fp_in, can2_rx_data);   // MTS torque command input- Only CIB ID that matches the switch settings will react

                if(CIB_config_data.BOARD_ID[0] == '2')        // CAN ID 0x22D for device #2 on CIB #2
                {
                    if((ECU1_XCP_reply_index == 0)&&(ign1_status > 0)) //check if in the middle of other stuff
                    {
                        memcpy(&MTS_Torque_Cmd_fp_in,&can2_rx_data[0],4);

                        if(MTS_Torque_Cmd_fp_in != old_MTS_torque_Cmd_fp_in)
                        {
                            //now transmit new motor torque command based on input.
                            MFG_TX_mtr_trq_cmd_MSG[3][3] = can2_rx_data[0];
                            MFG_TX_mtr_trq_cmd_MSG[3][2] = can2_rx_data[1];
                            MFG_TX_mtr_trq_cmd_MSG[3][1] = can2_rx_data[2];
                            MFG_TX_mtr_trq_cmd_MSG[3][0] = can2_rx_data[3];

                            ECU1_XCP_reply_index = 100; // used to get us to default case. Default case will set = 0.
                            transmitFlexray(MFG_TX_mtr_trq_cmd_MSG,MFG_TX, 4); //length of 3 is probabl fine
                        }
                    }

                }

                break;   // END case get Torque command from MTS for device #2

            case L02_MTS_D3_31D_Angle_Cmd_fp_in:  // Message box38 Angle command in from MTS for device #3

                canGetData(canREG2, L02_MTS_D3_31D_Angle_Cmd_fp_in, can2_rx_data);   // MTS angle command input- Only CIB ID that matches the switch settings will react

                if(CIB_config_data.BOARD_ID[0] == '3')        // CAN ID 0x31D for device #3 on CIB #3
                {
                    memcpy(&MTS_Angle_Cmd_fp_in,&can2_rx_data[0],4);
                    // SET Angle = New Angle with D/A output

                }

                break;   // END case get angle command from MTS for device #3

            case L02_MTS_D3_32D_Torque_Cmd_fp_in:  // Message box39 Torque command in from MTS for device #3

                canGetData(canREG2, L02_MTS_D3_32D_Torque_Cmd_fp_in, can2_rx_data);   // MTS torque command input- Only CIB ID that matches the switch settings will react

                if(CIB_config_data.BOARD_ID[0] == '3')        // CAN ID 0x32D for device #3 on CIB #3
                {
                    if((ECU1_XCP_reply_index == 0)&&(ign1_status > 0)) //check if in the middle of other stuff
                    {
                        memcpy(&MTS_Torque_Cmd_fp_in,&can2_rx_data[0],4);

                        if(MTS_Torque_Cmd_fp_in != old_MTS_torque_Cmd_fp_in)
                        {
                            //now transmit new motor torque command based on input.
                            MFG_TX_mtr_trq_cmd_MSG[3][3] = can2_rx_data[0];
                            MFG_TX_mtr_trq_cmd_MSG[3][2] = can2_rx_data[1];
                            MFG_TX_mtr_trq_cmd_MSG[3][1] = can2_rx_data[2];
                            MFG_TX_mtr_trq_cmd_MSG[3][0] = can2_rx_data[3];

                            ECU1_XCP_reply_index = 100; // used to get us to default case. Default case will set = 0.
                            transmitFlexray(MFG_TX_mtr_trq_cmd_MSG,MFG_TX, 4); //length of 3 is probabl fine
                        }
                    }

                }

                break;   // END case get Torque command from MTS for device #3

            case L02_MTS_D4_41D_Angle_Cmd_fp_in:  // Message box40 Angle command in from MTS for device #4

                canGetData(canREG2, L02_MTS_D4_41D_Angle_Cmd_fp_in, can2_rx_data);   // MTS angle command input- Only CIB ID that matches the switch settings will react

                if(CIB_config_data.BOARD_ID[0] == '4')        // CAN ID 0x41D for device #4 on CIB #4
                {
                    memcpy(&MTS_Angle_Cmd_fp_in,&can2_rx_data[0],4);
                    // SET Angle = New Angle with D/A output

                }

                break;   // END case get angle command from MTS for device #4

            case L02_MTS_D4_42D_Torque_Cmd_fp_in:  // Message box41 Torque command in from MTS for device #4

                canGetData(canREG2, L02_MTS_D4_42D_Torque_Cmd_fp_in, can2_rx_data);   // MTS torque command input- Only CIB ID that matches the switch settings will react

                if(CIB_config_data.BOARD_ID[0] == '4')        // CAN ID 0x42D for device #4 on CIB #4
                {
                    if((ECU1_XCP_reply_index == 0)&&(ign1_status > 0)) //check if in the middle of other stuff
                    {
                        memcpy(&MTS_Torque_Cmd_fp_in,&can2_rx_data[0],4);

                        if(MTS_Torque_Cmd_fp_in != old_MTS_torque_Cmd_fp_in)
                        {
                            //now transmit new motor torque command based on input.
                            MFG_TX_mtr_trq_cmd_MSG[3][3] = can2_rx_data[0];
                            MFG_TX_mtr_trq_cmd_MSG[3][2] = can2_rx_data[1];
                            MFG_TX_mtr_trq_cmd_MSG[3][1] = can2_rx_data[2];
                            MFG_TX_mtr_trq_cmd_MSG[3][0] = can2_rx_data[3];

                            ECU1_XCP_reply_index = 100; // used to get us to default case. Default case will set = 0.
                            transmitFlexray(MFG_TX_mtr_trq_cmd_MSG,MFG_TX, 4); //length of 3 is probabl fine
                        }
                    }

                }

                break;   // END case get Torque command from MTS for device #4

            case L02_MTS_D5_51D_Angle_Cmd_fp_in:  // Message box42 Angle command in from MTS for device #5

                canGetData(canREG2, L02_MTS_D5_51D_Angle_Cmd_fp_in, can2_rx_data);   // MTS angle command input- Only CIB ID that matches the switch settings will react

                if(CIB_config_data.BOARD_ID[0] == '5')        // CAN ID 0x51D for device #5 on CIB #5
                {
                    memcpy(&MTS_Angle_Cmd_fp_in,&can2_rx_data[0],4);
                    // SET Angle = New Angle with D/A output

                }

                break;   // END case get angle command from MTS for device #5

            case L02_MTS_D5_52D_Torque_Cmd_fp_in:  // Message box43 Torque command in from MTS for device #5

                canGetData(canREG2, L02_MTS_D5_52D_Torque_Cmd_fp_in, can2_rx_data);   // MTS torque command input- Only CIB ID that matches the switch settings will react

                if(CIB_config_data.BOARD_ID[0] == '5')        // CAN ID 0x52D for device #5 on CIB #5
                {
                    if((ECU1_XCP_reply_index == 0)&&(ign1_status > 0)) //check if in the middle of other stuff
                    {
                        memcpy(&MTS_Torque_Cmd_fp_in,&can2_rx_data[0],4);

                        if(MTS_Torque_Cmd_fp_in != old_MTS_torque_Cmd_fp_in)
                        {
                            //now transmit new motor torque command based on input.
                            MFG_TX_mtr_trq_cmd_MSG[3][3] = can2_rx_data[0];
                            MFG_TX_mtr_trq_cmd_MSG[3][2] = can2_rx_data[1];
                            MFG_TX_mtr_trq_cmd_MSG[3][1] = can2_rx_data[2];
                            MFG_TX_mtr_trq_cmd_MSG[3][0] = can2_rx_data[3];

                            ECU1_XCP_reply_index = 100; // used to get us to default case. Default case will set = 0.
                            transmitFlexray(MFG_TX_mtr_trq_cmd_MSG,MFG_TX, 4); //length of 3 is probabl fine
                        }
                    }

                }

                break;   // END case get Torque command from MTS for device #5

            case L02_MTS_D6_61D_Angle_Cmd_fp_in:  // Message box44 Angle command in from MTS for device #6

                canGetData(canREG2, L02_MTS_D6_61D_Angle_Cmd_fp_in, can2_rx_data);   // MTS angle command input- Only CIB ID that matches the switch settings will react

                if(CIB_config_data.BOARD_ID[0] == '6')        // CAN ID 0x61D for device #6 on CIB #6
                {
                    memcpy(&MTS_Angle_Cmd_fp_in,&can2_rx_data[0],4);
                    // SET Angle = New Angle with D/A output

                }

                break;   // END case get angle command from MTS for device #6

            case L02_MTS_D6_62D_Torque_Cmd_fp_in:  // Message box45 Torque command in from MTS for device #6

                canGetData(canREG2, L02_MTS_D6_62D_Torque_Cmd_fp_in, can2_rx_data);   // MTS torque command input- Only CIB ID that matches the switch settings will react

                if(CIB_config_data.BOARD_ID[0] == '6')        // CAN ID 0x62D for device #6 on CIB #6
                {
                    if((ECU1_XCP_reply_index == 0)&&(ign1_status > 0)) //check if in the middle of other stuff
                    {
                        memcpy(&MTS_Torque_Cmd_fp_in,&can2_rx_data[0],4);

                        if(MTS_Torque_Cmd_fp_in != old_MTS_torque_Cmd_fp_in)
                        {
                            //now transmit new motor torque command based on input.
                            MFG_TX_mtr_trq_cmd_MSG[3][3] = can2_rx_data[0];
                            MFG_TX_mtr_trq_cmd_MSG[3][2] = can2_rx_data[1];
                            MFG_TX_mtr_trq_cmd_MSG[3][1] = can2_rx_data[2];
                            MFG_TX_mtr_trq_cmd_MSG[3][0] = can2_rx_data[3];

                            ECU1_XCP_reply_index = 100; // used to get us to default case. Default case will set = 0.
                            transmitFlexray(MFG_TX_mtr_trq_cmd_MSG,MFG_TX, 4); //length of 3 is probabl fine
                        }
                    }
                }

                break;   // END case get Torque command from MTS for device #6

            case  MTS_7FF_MTS_Fault_in:      // message box 64 MTS fault input

                canGetData(canREG2, MTS_7FF_MTS_Fault_in, can2_rx_data);
                MTS_Bus_Fault_State_HEX_Byte = can2_rx_data[0];

                if(MTS_Bus_Fault_State_HEX_Byte > 0)
                {
                    // put part into shutdown??
                }


                break;

            } //end switch(messageBox)
            break;

        } // end case TARGET_BMW_FAAR_WE      ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^    BMW FAAR WE   C A N 2 ^^^^^^^^^^^^^^^^^^^^^^^^^^^
        break;

        case TARGET_9BXX:       //  VVVVVVVVVVVVVVVVVVVVVVVVVVVVV  9 B X X   C A N 2  VVVVVVVVVVVVVVVVVVVVVVVVVVVVV
        {
            switch(messageBox)
            {
            case TESTER_REC_DEBUG_ID:   // 0x7ff 11bit identifier-- REC from Anyone (debug)


                hits_on_CAN2_messageBox1++;     // debug, counter
                canGetData(canREG2, TESTER_REC_DEBUG_ID, can2_rx_data); // get date, store in rx_data array
                break;

            case TESTER_TRANS_DEBUG_ID:     // 0x7fe    11bit identifier-- Transmit to Anyone (debug)

                break;

            case CCP_CAN2_RESPONSE_ID:      // 0x642    11bit identifier-- Rec XCP Responce to EPS for data

                // CCP_CAN2_RESPONSE_ID $642;   // CAN2 CCP_RESPONCE-from product
                // recieve string and store for print in dump string
                canGetData(canREG2, CCP_CAN2_RESPONSE_ID, can2_rx_data);    // get date, store in rx_data array

                CAN2_recvd_hi_word = (can2_rx_data[1] << 8) + (can2_rx_data[2]);
                CAN2_recvd_lo_word = (can2_rx_data[3] << 8) + (can2_rx_data[4]);

                //                        sprintf(can2_ccp_response_str,"%.4X%.4X",CAN2_recvd_hi_word,CAN2_recvd_lo_word);

                break;
            case CCP_CAN2_REQUEST_ID:       // 0x242    11bit identifier-- Trans XCP Request from EPS for data

                break;

            case PERIOD_CAN2_ID:        // 0x182    11bit identifier-- Trans periodic to EPS

                break;

            case canMESSAGE_BOX6:

                break;

            case canMESSAGE_BOX7:

                break;

            default:

            {
                can2_message_type_debug = messageBox;  // an unknown messageBox
            }
            break;

            }

        } //case TARGET_9BXX:
        break;

        case TARGET_FCA_ADAS:   //  VVVVVVVVVVVVVVVVVVVVVVVVVV  F C A    A D A S   C A N 2  VVVVVVVVVVVVVVVVVVVVVVV
        {
            switch(messageBox)
            {
            case FCA_PROD_MESS1_ID: // 0x148    canMESSAGE_BOX1 11bit identifier-- Rec Response from EPS (not used)
                break;

            case FCA_PROD_MESS2_ID: // 0x778    canMESSAGE_BOX2 11bit identifier-- Rec ON Star from EPS (not used)
                break;

            case FCA_CCP_Engine_Run:// 0x108    canMESSAGE_BOX3 11bit identifier-- Trans Engine ON to EPS
                break;

            case FCA_CCP_Power_Mode_ID:  // 0x1F1     canMESSAGE_BOX4 11bit identifier-- Trans Power Mode to EPS(not used)
                break;

            case FCA_CCP_SPEED_ID:  // 0x011C   canMESSAGE_BOX5 11bit identifier-- Trans Speed to EPS with counter and CRC
                break;

            case FCA_CCP_RESPONSE_ID: // 0x59A  canMESSAGE_BOX6 11bit identifier-- Rec CCP Responce from EPS for Data
                // recieve and decide what the REQ was, parse and store
                // Standard dump of all ccp request

                canGetData(canREG2, FCA_CCP_RESPONSE_ID, can2_rx_data); // get data, store in rx_data array

                switch(can2_request_index)      // requesting multiple vaiables using same messageBox, can2_request_index keeps track, store and parse in main
                {
                case 0:     // do nothing with request_connect data, setup and trans next request

                    canTransmit(canREG2, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_BattVltg_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 1:     // store ccp_FCA_BattVltg_str, setup and trans next request

                    memcpy(&can2_dump_err_data[1],&can2_rx_data,8);

                    canTransmit(canREG2, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_Temperature_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 2:     // store ccp_FCA_Temperature_str, setup and trans next reques

                    memcpy(&can2_dump_err_data[2],&can2_rx_data,8);

                    canTransmit(canREG2, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_DigT1_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 3:     // store ccp_FCA_DigT1_str, setup and trans next reques

                    memcpy(&can2_dump_err_data[3],&can2_rx_data,8);

                    canTransmit(canREG2, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_DigT2_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 4:     // store ccp_FCA_DigT2_str, setup and trans next reques

                    memcpy(&can2_dump_err_data[4],&can2_rx_data,8);

                    canTransmit(canREG2, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_MtrCurrQax_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 5:     // store ccp_FCA_MtrCurrQax_str, setup and trans next reques

                    memcpy(&can2_dump_err_data[5],&can2_rx_data,8);

                    canTransmit(canREG2, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_SystemState_byt);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 6:     // store ccp_FCA_SystemState_byt, setup and trans next reques

                    memcpy(&can2_dump_err_data[6],&can2_rx_data,8);

                    canTransmit(canREG2, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_HwTrq_HwNm_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 7:     // store ccp_FCA_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can2_dump_err_data[7],&can2_rx_data,8);

                    canTransmit(canREG2, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_Abs_Hw_Pos_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 8:     // store ccp_FCA_Abs_Hw_Pos_str, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can2_dump_err_data[8],&can2_rx_data,8);

                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;


                    //  ----------------   H c u r r    FCA reading cases 2000-2031 done during normal testing

                case 2000:      // Hcurr Qax readings(x3) // store can2_FCA_MtrCurrQax1_fp, setup and trans next reques

                    memcpy(&can2_dump_err_data[9],&can2_rx_data,8);

                    canTransmit(canREG2, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_HwTrq_HwNm_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2001:      // store can2_FCA_HwTrq_HwNm1_fp, setup and trans next reques

                    memcpy(&can2_dump_err_data[10],&can2_rx_data,8);

                    can2_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2010:      // store can2_FCA_MtrCurrQax2_fp, setup and trans next reques

                    memcpy(&can2_dump_err_data[11],&can2_rx_data,8);

                    canTransmit(canREG2, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_HwTrq_HwNm_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2011:      // store can2_FCA_HwTrq_HwNm2_fp, setup and trans next reques

                    memcpy(&can2_dump_err_data[12],&can2_rx_data,8);

                    can2_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2020:      // store can2_FCA_MtrCurrQax3_fp, setup and trans next reques

                    memcpy(&can2_dump_err_data[13],&can2_rx_data,8);

                    canTransmit(canREG2, FCA_CCP_REQUEST_ID, ( uint8 *) ccp_FCA_HwTrq_HwNm_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2021:      // store can2_FCA_HwTrq_HwNm3_fp, setup and trans next reques

                    memcpy(&can2_dump_err_data[14],&can2_rx_data,8);

                    can2_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2999:

                    can2_request_index = 3000;  // don't care response is done...ok to send new request.

                    break;

                default:
                {
                }

                }   // end switch(can2_request_index)

                break;  // end case CCP_RESPONSE_ID:

                case FCA_CCP_REQUEST_ID:    // 0x6DA    canMESSAGE_BOX7 11bit identifier-- Trans CCP Request for Data to EPS
                    break;

                case FCA_XCP_EA3_REQUEST_ID: // 0x75A   canMESSAGE_BOX8 11bit identifier-- Trans XCP Request from EPS for data
                    break;

                case FCA_XCP_EA3_RESPONSE_ID:// 0x4DA   canMESSAGE_BOX9 11bit identifier-- Rec XCP Responce to EPS for data

                    canGetData(canREG2, FCA_XCP_EA3_RESPONSE_ID, can2_rx_data); // get data, store in rx_data array

                    switch(ECU2_XCP_reply_index)    // requesting multiple requests using same messageBox, DCT_XCP_rply_indx keeps track, clear_dtcs_flag determines action
                    {

                    case 1:

                        if((can2_rx_data[1] == 0x50) && (can2_rx_data[2] == 0x7E))  // positive responce from enter Nexteer mode for FCA
                        {
                            if(ECU2_clear_DTCs_flag == 1)         // clear DTC's mode
                            {
                                canTransmit(canREG2, FCA_XCP_EA3_REQUEST_ID, ( uint8 *) xcp_EA3_write_clear_DTC_rqst1); // request clear DTCs
                            }
                            else        // get DTC's mode
                            {
                                canTransmit(canREG2, FCA_XCP_EA3_REQUEST_ID, ( uint8 *) xcp_EA3_write_get_DTC_rqst1);   // request DTCs first string
                            }

                            ECU2_XCP_reply_index++;
                        }  // end positive responce nexteer mode
                        break;

                    case 2:

                        if((can2_rx_data[0] == 0x10) && (can2_rx_data[2] == 0x62))    // get DTC's mode
                        {
                            dtc2[0][0] =  can2_rx_data[5];
                            dtc2[0][1] =  can2_rx_data[6];
                            dtc2[0][2] =  can2_rx_data[7];

                            canTransmit(canREG2, FCA_XCP_EA3_REQUEST_ID, ( uint8 *) xcp_EA3_write_get_DTC_rqst2);   // request DTCs 2nd string

                            ECU2_XCP_reply_index++;
                        }

                        if((can2_rx_data[0] == 0x04) && (can2_rx_data[1] == 0x71))  // responce to clear DTC's mode
                        {
                            if(ECU2_clear_DTCs_flag == 1)
                            {
                                ECU2_XCP_reply_index = 0;
                                ECU2_clear_DTCs_flag = 0;   // reset flag and index, done
                            }
                        }

                        break;

                    case 3:
                        if(can2_rx_data[0] == 0x21)
                        {
                            dtc2[0][3] =  can2_rx_data[1];
                            dtc2[0][4] =  can2_rx_data[2];
                            dtc2[1][0] =  can2_rx_data[3];
                            dtc2[1][1] =  can2_rx_data[4];
                            dtc2[1][2] =  can2_rx_data[5];
                            dtc2[1][3] =  can2_rx_data[6];
                            dtc2[1][4] =  can2_rx_data[7];
                        }
                        ECU2_XCP_reply_index++;
                        break;

                    case 4:
                        if(can2_rx_data[0] == 0x22)
                        {
                            dtc2[2][0] =  can2_rx_data[1];
                            dtc2[2][1] =  can2_rx_data[2];
                            dtc2[2][2] =  can2_rx_data[3];
                            dtc2[2][3] =  can2_rx_data[4];
                            dtc2[2][4] =  can2_rx_data[5];
                            dtc2[3][0] =  can2_rx_data[6];
                            dtc2[3][1] =  can2_rx_data[7];
                        }
                        ECU2_XCP_reply_index++;
                        break;

                    case 5:
                        if(can2_rx_data[0] == 0x23)
                        {
                            dtc2[3][2] =  can2_rx_data[1];
                            dtc2[3][3] =  can2_rx_data[2];
                            dtc2[3][4] =  can2_rx_data[3];
                            dtc2[4][0] =  can2_rx_data[4];
                            dtc2[4][1] =  can2_rx_data[5];
                            dtc2[4][2] =  can2_rx_data[6];
                            dtc2[4][3] =  can2_rx_data[7];
                        }
                        ECU2_XCP_reply_index++;
                        break;

                    case 6:
                        if(can2_rx_data[0] == 0x24)
                        {
                            dtc2[4][4] =  can2_rx_data[1];
                            dtc2[5][0] =  can2_rx_data[2];
                            dtc2[5][1] =  can2_rx_data[3];
                            dtc2[5][2] =  can2_rx_data[4];
                            dtc2[5][3] =  can2_rx_data[5];
                            dtc2[5][4] =  can2_rx_data[6];
                            dtc2[6][0] =  can2_rx_data[7];
                        }
                        ECU2_XCP_reply_index++;
                        break;

                    case 7:
                        if(can2_rx_data[0] == 0x25)
                        {
                            dtc2[6][1] =  can2_rx_data[1];
                            dtc2[6][2] =  can2_rx_data[2];
                            dtc2[6][3] =  can2_rx_data[3];
                            dtc2[6][4] =  can2_rx_data[4];
                            dtc2[7][0] =  can2_rx_data[5];
                            dtc2[7][1] =  can2_rx_data[6];
                            dtc2[7][2] =  can2_rx_data[7];
                        }
                        ECU2_XCP_reply_index++;
                        break;

                    case 8:
                        if(can2_rx_data[0] == 0x26)
                        {
                            dtc2[7][3] =  can2_rx_data[1];
                            dtc2[7][4] =  can2_rx_data[2];
                            dtc2[8][0] =  can2_rx_data[3];
                            dtc2[8][1] =  can2_rx_data[4];
                            dtc2[8][2] =  can2_rx_data[5];
                            dtc2[8][3] =  can2_rx_data[6];
                            dtc2[8][4] =  can2_rx_data[7];
                        }
                        ECU2_XCP_reply_index++;
                        break;

                    case 9:
                        if(can2_rx_data[0] == 0x27)
                        {
                            dtc2[9][0] =  can2_rx_data[1];
                            dtc2[9][1] =  can2_rx_data[2];
                            dtc2[9][2] =  can2_rx_data[3];
                            dtc2[9][3] =  can2_rx_data[4];
                            dtc2[9][4] =  can2_rx_data[5];
                            dtc2[10][0] = can2_rx_data[6];
                            dtc2[10][1] = can2_rx_data[7];
                        }
                        ECU2_XCP_reply_index++;
                        break;

                    case 10:
                        if(can2_rx_data[0] == 0x28)
                        {
                            dtc2[10][2] =  can2_rx_data[1];
                            dtc2[10][3] =  can2_rx_data[2];
                            dtc2[10][4] =  can2_rx_data[3];
                            dtc2[11][0] =  can2_rx_data[4];
                            dtc2[11][1] =  can2_rx_data[5];
                            dtc2[11][2] =  can2_rx_data[6];
                            dtc2[11][3] =  can2_rx_data[7];
                        }
                        ECU2_XCP_reply_index++;
                        break;

                    case 11:
                        if(can2_rx_data[0] == 0x29)
                        {
                            dtc2[11][4] =  can2_rx_data[1];
                            dtc2[12][0] =  can2_rx_data[2];
                            dtc2[12][1] =  can2_rx_data[3];
                            dtc2[12][2] =  can2_rx_data[4];
                            dtc2[12][3] =  can2_rx_data[5];
                            dtc2[12][4] =  can2_rx_data[6];
                            dtc2[13][0] =  can2_rx_data[7];
                        }
                        ECU2_XCP_reply_index++;
                        break;

                    case 12:
                        if(can2_rx_data[0] == 0x2A)
                        {
                            dtc2[13][1] =  can2_rx_data[1];
                            dtc2[13][2] =  can2_rx_data[2];
                            dtc2[13][3] =  can2_rx_data[3];
                            dtc2[13][4] =  can2_rx_data[4];
                            dtc2[14][0] =  can2_rx_data[5];
                            dtc2[14][1] =  can2_rx_data[6];
                            dtc2[14][2] =  can2_rx_data[7];
                        }
                        ECU2_XCP_reply_index++;
                        break;

                    case 13:
                        if(can2_rx_data[0] == 0x2B)
                        {
                            dtc2[14][3] =  can2_rx_data[1];
                            dtc2[14][4] =  can2_rx_data[2];
                        }
                        ECU2_XCP_reply_index = 14;   // last read set to 14 for get_DTCs function
                        break;

                    default:

                        ECU2_XCP_reply_index = 0;

                        break;

                    } // end switch on DTC_XCP_rply_indx

                    break; // end case XCP_RESPONSE_ID

                    case FCA_XCP_EA4_REQUEST_ID: // 0x712   canMESSAGE_BOX10 11bit identifier-- Trans XCP Request from EPS for data
                        break;

                    case FCA_XCP_EA4_RESPONSE_ID: // 0x710   canMESSAGE_BOX11 11bit identifier-- Rec XCP Responce to EPS for data
                        break;

                    case FCA_TESTER_REC_DEBUG_ID:    // 0x7FF    canMESSAGE_BOX12 11bit identifier-- REC from Anyone

                        break;

                    case FCA_TESTER_TRANS_DEBUG_ID:    // 0x7FE     canMESSAGE_BOX13 11bit identifier-- Transmit to Anyone

                        break;

                    case canMESSAGE_BOX14:      // 0x14  11bit identifier-- Not Used

                        break;

                    case canMESSAGE_BOX15:      // 0x15  11bit identifier-- Not Usedr

                        break;

                    case canMESSAGE_BOX16:      // 0x16  11bit identifier-- Not Used

                        break;

                    default:

                    {
                        can2_message_type_debug = messageBox;  // an unknown messageBox
                    }
                    break;

            }  // switch(messageBox) for FCA ADAS on canREG2

        }  // end case TARGET_FCA_ADAS:  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^    F C A   A D A S   C A N 2 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        break;

        case TARGET_CD391_ADAS: //  VVVVVVVVVVVVVVVVVVVVVVVVVV  F O R D    A D A S   C A N 2  VVVVVVVVVVVVVVVVVVVVVVV
        {
            switch(messageBox)
            {
            case FORD_PROD_MESS1_ID: // 0x148   canMESSAGE_BOX1 11bit identifier-- Rec Response from EPS (not used)
                break;

            case FORD_PROD_MESS2_ID: // 0x778   canMESSAGE_BOX2 11bit identifier-- Rec ON Star from EPS (not used)
                break;

            case FORD_CCP_Engine_Run_2:// 0x108 canMESSAGE_BOX3 11bit identifier-- Trans Engine ON to EPS
                break;

            case FORD_CCP_Power_Mode_ID:  // 0x1F1    canMESSAGE_BOX4 11bit identifier-- Trans Power Mode to EPS(not used)
                break;

            case FORD_CCP_SPEED_ID:  // 0x011C   canMESSAGE_BOX5    11bit identifier-- Trans Speed to EPS with counter and CRC
                break;

            case FORD_CCP_RESPONSE_ID: // 0x59A canMESSAGE_BOX6 11bit identifier-- Rec CCP Responce from EPS for Data
                // recieve and decide what the REQ was, parse and store
                // Standard dump of all ccp request

                canGetData(canREG2, FORD_CCP_RESPONSE_ID, can2_rx_data);    // get data, store in rx_data array

                switch(can2_request_index)      // requesting multiple vaiables using same messageBox, can2_request_index keeps track, store and parse in main
                {
                case 0:     // do nothing with request_connect data, setup and trans next request

                    canTransmit(canREG2, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_BattVltg_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 1:     // store ccp_FORD_BattVltg_str, setup and trans next request

                    memcpy(&can2_dump_err_data[1],&can2_rx_data,8);

                    canTransmit(canREG2, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_Temperature_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 2:     // store ccp_FORD_Temperature_str, setup and trans next reques

                    memcpy(&can2_dump_err_data[2],&can2_rx_data,8);

                    canTransmit(canREG2, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_DigT1_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 3:     // store ccp_FORD_DigT1_str, setup and trans next reques

                    memcpy(&can2_dump_err_data[3],&can2_rx_data,8);

                    canTransmit(canREG2, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_DigT2_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 4:     // store ccp_FORD_DigT2_str, setup and trans next reques

                    memcpy(&can2_dump_err_data[4],&can2_rx_data,8);

                    canTransmit(canREG2, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_MtrCurrQax_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 5:     // store ccp_FORD_MtrCurrQax_str, setup and trans next reques

                    memcpy(&can2_dump_err_data[5],&can2_rx_data,8);

                    canTransmit(canREG2, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_SystemState_byt);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 6:     // store ccp_FORD_SystemState_byt, setup and trans next reques

                    memcpy(&can2_dump_err_data[6],&can2_rx_data,8);

                    canTransmit(canREG2, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_HwTrq_HwNm_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 7:     // store ccp_FORD_HwTrq_HwNm_str, setup and trans next reques

                    memcpy(&can2_dump_err_data[7],&can2_rx_data,8);

                    canTransmit(canREG2, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_Abs_Hw_Pos_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 8:     // store ccp_FORD_Abs_Hw_Pos_str, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can2_dump_err_data[8],&can2_rx_data,8);

                    canTransmit(canREG2, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_Batt_Current_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 9:     // store ccp_FORD_Batt_Current_str, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can2_dump_err_data[9],&can2_rx_data,8);

                    canTransmit(canREG2, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_LatchFail_ISR_u16);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 10:        // store ccp_FORD_LatchFail_ISR_u16, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can2_dump_err_data[10],&can2_rx_data,8);

                    canTransmit(canREG2, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_MtrCntl_ISR_u16);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 11:        // store ccp_FORD_MtrCntl_ISR_u16, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can2_dump_err_data[11],&can2_rx_data,8);

                    canTransmit(canREG2, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_MinCount_u32);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 12:        // store ccp_FORD_MinCount_u32, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can2_dump_err_data[12],&can2_rx_data,8);

                    canTransmit(canREG2, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_MaxCount_u32);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 13:        // store ccp_FORD_MaxCount_u32, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can2_dump_err_data[13],&can2_rx_data,8);

                    canTransmit(canREG2, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_CatGate_cnt);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                case 14:        // store ccp_FORD_CatGate_cnt, setup and trans next reques
                    //  all done, index bumped for main to test

                    memcpy(&can2_dump_err_data[14],&can2_rx_data,8);
                    can2_request_index++;       // bump index so next receive will switch to correct message
                    break;

                    //  ----------------   Low - Med -High    FORD reading cases 2000-2031 done during normal testing

                case 2000:      // Qax readings(x3) // store can2_FORD_MtrCurrQax1_fp, setup and trans next reques

                    memcpy(&can2_dump_err_data[10],&can2_rx_data,8);

                    canTransmit(canREG2, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_HwTrq_HwNm_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2001:      // store can2_FORD_HwTrq_HwNm1_fp, setup and trans next reques

                    memcpy(&can2_dump_err_data[11],&can2_rx_data,8);

                    can2_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2010:      // store can2_FORD_MtrCurrQax2_fp, setup and trans next reques

                    memcpy(&can2_dump_err_data[12],&can2_rx_data,8);

                    canTransmit(canREG2, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_HwTrq_HwNm_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2011:      // store can2_FORD_HwTrq_HwNm2_fp, setup and trans next reques

                    memcpy(&can2_dump_err_data[13],&can2_rx_data,8);

                    can2_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2020:      // store can2_FORD_MtrCurrQax3_fp, setup and trans next reques

                    memcpy(&can2_dump_err_data[14],&can2_rx_data,8);

                    canTransmit(canREG2, FORD_CCP_REQUEST_ID, ( uint8 *) ccp_FORD_HwTrq_HwNm_str);
                    can2_request_index++;       // bump index so next receive will switch to correct message

                    break;

                case 2021:      // store can2_FORD_HwTrq_HwNm3_fp, setup and trans next reques

                    memcpy(&can2_dump_err_data[15],&can2_rx_data,8);

                    can2_request_index = 2999;      // arbitrary value, all done for now

                    break;

                case 2999:

                    can2_request_index = 3000;  // don't care response is done...ok to send new request.

                    break;

                default:
                {
                }

                }   // end switch(can2_request_index)

                break;  // end case CCP_RESPONSE_ID:

                case FORD_CCP_REQUEST_ID:    // 0x6DA    canMESSAGE_BOX7    11bit identifier-- Trans CCP Request for Data to EPS
                    break;

                case FORD_XCP_EA3_REQUEST_ID: // 0x75A   canMESSAGE_BOX8    11bit identifier-- Trans XCP Request from EPS for data
                    break;

                case FORD_XCP_EA3_RESPONSE_ID:// 0x4DA   canMESSAGE_BOX9    11bit identifier-- Rec XCP Responce to EPS for data

                    canGetData(canREG2, FORD_XCP_EA3_RESPONSE_ID, can2_rx_data);    // get data, store in rx_data array

                    switch(ECU2_XCP_reply_index)    // requesting multiple requests using same messageBox, DCT_XCP_rply_indx keeps track, clear_dtcs_flag determines action
                    {

                    case 1:

                        if((can2_rx_data[1] == 0x50) && (can2_rx_data[2] == 0x7E))  // positive responce from enter Nexteer mode for FCA
                        {
                            if(ECU2_clear_DTCs_flag == 1)         // clear DTC's mode
                            {
                                canTransmit(canREG2, FORD_XCP_EA3_REQUEST_ID, ( uint8 *) xcp_EA3_write_clear_DTC_rqst1);    // request clear DTCs
                            }

                            else        // get DTC's mode

                            {
                                canTransmit(canREG2, FORD_XCP_EA3_REQUEST_ID, ( uint8 *) xcp_EA3_write_get_DTC_rqst1);  // request DTCs first string
                            }

                            ECU2_XCP_reply_index++;
                        }  // end positive response nexteer mode
                        break;

                    case 2:

                        if((can2_rx_data[0] == 0x10) && (can2_rx_data[2] == 0x62))    // get DTC's mode
                        {
                            dtc2[0][0] =  can2_rx_data[5];
                            dtc2[0][1] =  can2_rx_data[6];
                            dtc2[0][2] =  can2_rx_data[7];

                            canTransmit(canREG2, FORD_XCP_EA3_REQUEST_ID, ( uint8 *) xcp_EA3_write_get_DTC_rqst2);  // request DTCs 2nd string

                            ECU2_XCP_reply_index++;
                        }

                        if((can2_rx_data[0] == 0x04) && (can2_rx_data[1] == 0x71))  // Response to clear DTC's mode
                        {
                            if(ECU2_clear_DTCs_flag == 1)
                            {
                                ECU2_XCP_reply_index = 0;
                                ECU2_clear_DTCs_flag = 0;   // reset flag and index, done
                            }
                        }

                        break;

                    case 3:
                        if(can2_rx_data[0] == 0x21)
                        {
                            dtc2[0][3] =  can2_rx_data[1];
                            dtc2[0][4] =  can2_rx_data[2];
                            dtc2[1][0] =  can2_rx_data[3];
                            dtc2[1][1] =  can2_rx_data[4];
                            dtc2[1][2] =  can2_rx_data[5];
                            dtc2[1][3] =  can2_rx_data[6];
                            dtc2[1][4] =  can2_rx_data[7];
                        }
                        ECU2_XCP_reply_index++;
                        break;

                    case 4:
                        if(can2_rx_data[0] == 0x22)
                        {
                            dtc2[2][0] =  can2_rx_data[1];
                            dtc2[2][1] =  can2_rx_data[2];
                            dtc2[2][2] =  can2_rx_data[3];
                            dtc2[2][3] =  can2_rx_data[4];
                            dtc2[2][4] =  can2_rx_data[5];
                            dtc2[3][0] =  can2_rx_data[6];
                            dtc2[3][1] =  can2_rx_data[7];
                        }
                        ECU2_XCP_reply_index++;
                        break;

                    case 5:
                        if(can2_rx_data[0] == 0x23)
                        {
                            dtc2[3][2] =  can2_rx_data[1];
                            dtc2[3][3] =  can2_rx_data[2];
                            dtc2[3][4] =  can2_rx_data[3];
                            dtc2[4][0] =  can2_rx_data[4];
                            dtc2[4][1] =  can2_rx_data[5];
                            dtc2[4][2] =  can2_rx_data[6];
                            dtc2[4][3] =  can2_rx_data[7];
                        }
                        ECU2_XCP_reply_index++;
                        break;

                    case 6:
                        if(can2_rx_data[0] == 0x24)
                        {
                            dtc2[4][4] =  can2_rx_data[1];
                            dtc2[5][0] =  can2_rx_data[2];
                            dtc2[5][1] =  can2_rx_data[3];
                            dtc2[5][2] =  can2_rx_data[4];
                            dtc2[5][3] =  can2_rx_data[5];
                            dtc2[5][4] =  can2_rx_data[6];
                            dtc2[6][0] =  can2_rx_data[7];
                        }
                        ECU2_XCP_reply_index++;
                        break;

                    case 7:
                        if(can2_rx_data[0] == 0x25)
                        {
                            dtc2[6][1] =  can2_rx_data[1];
                            dtc2[6][2] =  can2_rx_data[2];
                            dtc2[6][3] =  can2_rx_data[3];
                            dtc2[6][4] =  can2_rx_data[4];
                            dtc2[7][0] =  can2_rx_data[5];
                            dtc2[7][1] =  can2_rx_data[6];
                            dtc2[7][2] =  can2_rx_data[7];
                        }
                        ECU2_XCP_reply_index++;
                        break;

                    case 8:
                        if(can2_rx_data[0] == 0x26)
                        {
                            dtc2[7][3] =  can2_rx_data[1];
                            dtc2[7][4] =  can2_rx_data[2];
                            dtc2[8][0] =  can2_rx_data[3];
                            dtc2[8][1] =  can2_rx_data[4];
                            dtc2[8][2] =  can2_rx_data[5];
                            dtc2[8][3] =  can2_rx_data[6];
                            dtc2[8][4] =  can2_rx_data[7];
                        }
                        ECU2_XCP_reply_index++;
                        break;

                    case 9:
                        if(can2_rx_data[0] == 0x27)
                        {
                            dtc2[9][0] =  can2_rx_data[1];
                            dtc2[9][1] =  can2_rx_data[2];
                            dtc2[9][2] =  can2_rx_data[3];
                            dtc2[9][3] =  can2_rx_data[4];
                            dtc2[9][4] =  can2_rx_data[5];
                            dtc2[10][0] = can2_rx_data[6];
                            dtc2[10][1] = can2_rx_data[7];
                        }
                        ECU2_XCP_reply_index++;
                        break;

                    case 10:
                        if(can2_rx_data[0] == 0x28)
                        {
                            dtc2[10][2] =  can2_rx_data[1];
                            dtc2[10][3] =  can2_rx_data[2];
                            dtc2[10][4] =  can2_rx_data[3];
                            dtc2[11][0] =  can2_rx_data[4];
                            dtc2[11][1] =  can2_rx_data[5];
                            dtc2[11][2] =  can2_rx_data[6];
                            dtc2[11][3] =  can2_rx_data[7];
                        }
                        ECU2_XCP_reply_index++;
                        break;

                    case 11:
                        if(can2_rx_data[0] == 0x29)
                        {
                            dtc2[11][4] =  can2_rx_data[1];
                            dtc2[12][0] =  can2_rx_data[2];
                            dtc2[12][1] =  can2_rx_data[3];
                            dtc2[12][2] =  can2_rx_data[4];
                            dtc2[12][3] =  can2_rx_data[5];
                            dtc2[12][4] =  can2_rx_data[6];
                            dtc2[13][0] =  can2_rx_data[7];
                        }
                        ECU2_XCP_reply_index++;
                        break;

                    case 12:
                        if(can2_rx_data[0] == 0x2A)
                        {
                            dtc2[13][1] =  can2_rx_data[1];
                            dtc2[13][2] =  can2_rx_data[2];
                            dtc2[13][3] =  can2_rx_data[3];
                            dtc2[13][4] =  can2_rx_data[4];
                            dtc2[14][0] =  can2_rx_data[5];
                            dtc2[14][1] =  can2_rx_data[6];
                            dtc2[14][2] =  can2_rx_data[7];
                        }
                        ECU2_XCP_reply_index++;
                        break;

                    case 13:
                        if(can2_rx_data[0] == 0x2B)
                        {
                            dtc2[14][3] =  can2_rx_data[1];
                            dtc2[14][4] =  can2_rx_data[2];
                        }
                        ECU2_XCP_reply_index = 14;   // last read set to 14 for get_DTCs function

                        break;

                    default:

                        ECU2_XCP_reply_index = 0;

                        break;

                    } // end switch on DTC_XCP_rply_indx

                    break; // end case XCP_RESPONSE_ID

                    case FORD_XCP_EA4_REQUEST_ID: // 0x712  canMESSAGE_BOX10 11bit identifier-- Trans XCP Request from EPS for data
                        break;

                    case FORD_XCP_EA4_RESPONSE_ID: // 0x710  canMESSAGE_BOX11 11bit identifier-- Rec XCP Responce to EPS for data
                        break;

                    case FORD_TESTER_REC_DEBUG_ID:   // 0x7FF    canMESSAGE_BOX12 11bit identifier-- REC from Anyone

                        break;

                    case FORD_TESTER_TRANS_DEBUG_ID:    // 0x7FE     canMESSAGE_BOX13 11bit identifier-- Transmit to Anyone

                        break;

                    case canMESSAGE_BOX14:      // 0x14  11bit identifier-- Not Used

                        break;

                    case canMESSAGE_BOX15:      // 0x15  11bit identifier-- Not Usedr

                        break;

                    case canMESSAGE_BOX16:      // 0x16  11bit identifier-- Not Used

                        break;

                    default:

                    {
                        can2_message_type_debug = messageBox;  // an unknown messageBox
                    }
                    break;

            }  // switch(messageBox) for FCA ADAS on canREG2
        }  // end case TARGET_CD391_ADAS:  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^    F O R D   A D A S   C A N 2 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        break;

        case TARGET_G2KCA_ADAS:     //  VVVVVVVVVVVVVVVVVVVVVVVVVV  G 2 K C A    A D A S   C A N 2  VVVVVVVVVVVVVVVVVVVVVVVVV
        {
            switch(messageBox)
            {
            case G2KCA_ECU2_CAN2_PROD_MESS1_ID:         // 0x148    canMESSAGE_BOX1 11bit identifier-- Rec Response from EPS (not used)
                break;

            case G2KCA_ECU2_CAN2_PROD_MESS2_ID:         // 0x778    canMESSAGE_BOX2 11bit identifier-- Rec ON Star from EPS (not used)
                break;

            case G2KCA_ECU2_CAN2_CCP_Engine_Run:        // 0x108    canMESSAGE_BOX3 11bit identifier-- Trans Engine ON to EPS (not used)
                break;

            case G2KCA_ECU2_CAN2_CCP_Power_Mode_ID:     // 0x1F1      canMESSAGE_BOX4 11bit identifier-- Trans Power Mode to EPS (not used)
                break;

            case G2KCA_ECU2_CAN2_CCP_SPEED_ID:          // 0x03E9   canMESSAGE_BOX5 11bit identifier-- Trans Speed to EPS with counter and CRC (not used)
                break;

            case G2KCA_ECU2_CAN2_CCP_Driven_ID:         // 0x348    canMESSAGE_BOX6 11bit identifier-- Trans Wheel 1 Info to EPS
                break;

            case G2KCA_ECU2_CAN2_CCP_NonDriven_ID:      // 0x34A    canMESSAGE_BOX7 11bit identifier-- Trans Wheel 2 Info to EPS
                break;

            case G2KCA_ECU1_CAN2_CCP_REQUEST_ID:        // 0x708    canMESSAGE_BOX8 11bit identifier-- Trans CCP Request for Data to EPS (not used)
                break;

            case G2KCA_ECU1_CAN2_CCP_RESPONSE_ID:       // 0x706    canMESSAGE_BOX9 11bit identifier-- Rec CCP Responce from EPS for Data (not used)
                break;

            case G2KCA_ECU1_CAN2_XCP_EA3_REQUEST_ID:    // 0x242    canMESSAGE_BOX10 11bit identifier-- Trans XCP Request from EPS for data (not used)
                break;

            case G2KCA_ECU1_CAN2_XCP_EA3_RESPONSE_ID:   // 0x642    canMESSAGE_BOX11 11bit identifier-- Rec XCP Responce from EPS for data (not used)
                break;


            case G2KCA_ECU1_CAN2_XCP_EA4_REQUEST_ID:    // 0x712    canMESSAGE_BOX12 11bit identifier-- Trans XCP Request from EPS for data (not used)
                break;

            case G2KCA_ECU1_CAN2_XCP_EA4_RESPONSE_ID:   // 0x642    canMESSAGE_BOX13 11bit identifier-- Rec XCP Responce from EPS for data (not used)
                break;

            case G2KCA_ECU2_CAN2_CCP_REQUEST_ID:        // 0x70A    canMESSAGE_BOX14    11bit identifier-- Trans CCP Request for Data to EPS (not used)
                break;

            case G2KCA_ECU2_CAN2_CCP_RESPONSE_ID:       // 0x70C    canMESSAGE_BOX15    11bit identifier-- Rec CCP Responce from EPS for Data (not used)
                break;

            case G2KCA_ECU2_CAN2_XCP_EA3_REQUEST_ID:    // 0x0242   canMESSAGE_BOX16    11bit identifier-- Trans XCP Request from EPS for data  (not used)
                break;

            case G2KCA_ECU2_CAN2_XCP_EA3_RESPONSE_ID:   // 0x0642   canMESSAGE_BOX17    11bit identifier-- Rec XCP Responce to EPS for data  (not used)
                break;

            case G2KCA_ECU2_CAN2_XCP_EA4_REQUEST_ID:    // 0x0713   canMESSAGE_BOX18    11bit identifier-- Trans XCP Request from EPS for data (not used)
                break;

            case G2KCA_ECU2_CAN1_XCP_EA4_RESPONSE_ID:    // 0x642   canMESSAGE_BOX19    11bit identifier-- Rec XCP Responce from EPS for data (not used)
                break;

            case G2KCA_TESTER_REC_DEBUG_ID:             // 0x07FF   canMESSAGE_BOX20    11bit identifier-- REC from Anyone
                break;

            case G2KCA_TESTER_TRANS_DEBUG_ID:           // 0x7FE    canMESSAGE_BOX21    11bit identifier-- Transmit to Anyone
                break;

            case canMESSAGE_BOX22:                      // 0x22  11bit identifier-- Not Used
                break;

            default:

            {
                can1_message_type_debug = messageBox;  // an unknown messageBox
            }

            break;      // end switch(messageBox) for G2KCA ADAS on canREG2

            }   // end switch(messageBox) for G2KCA ADAS on canREG2

            break;
        }  // end case G2KCA ADAS    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^    G 2 K C A   A D A S   C A N 2 ^^^^^^^^^^^^^^^^^^^^^^^^^^^

        }  // end switch(target_product)  C A N 2

    }  // if(node == canREG2)

    /* USER CODE END */
}

/* USER CODE BEGIN (16) */
/* USER CODE END */
#pragma WEAK(gioNotification)
void gioNotification(gioPORT_t *port, uint32 bit)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (19) */
    // OLD CODE - Just Playing
    // gio interrupt ISR, various potential sources, "notification" contains which one
    // GIOA[0-7] = 1-8,, GIOB[0-7] = 9-16,, ect..


    //  gioDisableNotification(3);

    /** - disable interrupts */
    //   gioREG->INTENACLR =  0U         /* Bit 0 */
    //                      | (0U << 1U)  /* Bit 1 */
    //                      | (0U << 2U)  /* Bit 2 */
    //                      | (1U << 3U)  /* Bit 3 writing a 1 disables interrupt, during A?D scans */
    //                      | (0U << 4U)  /* Bit 4 */
    //                      | (0U << 5U)  /* Bit 5 */
    //                      | (0U << 6U)  /* Bit 6 */
    //                      | (0U << 7U);  /* Bit 7 */

    switch(bit)
    {
    case 0:
        // No interrupt is pending.
        break;

    case 1:

        break;

    case 2: // YELLOW UserSwitch1 SW1 on GIOA[2] ISR, inited to capture falling edge, then reset to capture rising edge, and process SNAP or RESET

        // don't process switch if SNAPPING or RESET active
        // SNAPPING activity will either timeout, setting var back to "NONE" or
        // snap will take place and activity will set var to "SNAPPED1" or "SNAPPED2".
        // if a RESET is activated, we don't care because all get reinited.
        if((SW1_is_snapping_snapped1_snapped2_reset_none == NONE) || (SW1_is_snapping_snapped1_snapped2_reset_none == SNAPPED1) || (SW1_is_snapping_snapped1_snapped2_reset_none == SNAPPED2))
        {

            switch(SW1_high_or_low)
            {
            case HIGH:                      // got here because of signal high, going low
                SW1_active_low_time = system_msec_clock;    // set timer to time now, will compair later to see what action to take
                SW1_is_snapping_snapped1_snapped2_reset_none = NONE;        // ensure initial action state is "NONE" selected
                SW1_high_or_low = LOW;      // set for next state of switch, ISR will be hit on signal low going high, SW1 released

                //  set ISR to int on rising edge
                /** @b initialize @b interrupts */

                /** - interrupt polarity */
                gioREG->POL =  0U         /* Bit 0 */
                        | (0U << 1U)  /* Bit 1 */
                        | (1U << 2U)  /* Bit 2 User SW-1 (YELLOW B_CIB)*/
                        | (0U << 3U)  /* Bit 3 */
                        | (0U << 4U)  /* Bit 4 */
                        | (0U << 5U)  /* Bit 5 */
                        | (0U << 6U)  /* Bit 6 */
                        | (0U << 7U); /* Bit 7 */


                break;

            case LOW:                       // got here because of signal low, going high
                SW1_time_held_low = system_msec_clock - SW1_active_low_time;    // time SW1 was depressed, HIGH to LOW to HIGH
                if(SW1_time_held_low < 3000)    // if SW1 held low for greater than 3 seconds, we want to push a SOFT RESET, else its a request for a TCPIP "SNAP"
                {
                    SW1_is_snapping_snapped1_snapped2_reset_none = SNAPPING;
                    SNAP_timeout_clock = system_msec_clock + 10000;     // set timout to 10 seconds into future
                    // set gioA(bit0=1)->Lantastic LAN CP1=1, network broadcast query of port status will show a high on CP1
                    // RTI notification will monitor for SNAP feature timeout and reset gioA(bit0=0).
                    gioSetBit(gioPORTA, 5, 1 ); // set Lantronix port[0], so UDP:30704 MONITOR command 13h00h00h00h00h will send a "1" for this port
                }
                else
                {
                    hetREG1->DCLR = 0X000CC000; // turn on HET[14,15,18,19]  LEDs 1,2,3,4 = ON

                    for(loop_cnt_b = 0; loop_cnt_b < 60000000 ; loop_cnt_b++);      // delay loop to see LEDs on before softReset

                    SW1_is_snapping_snapped1_snapped2_reset_none = RESET;
                    // push software reset -- System Exception Control Register (SYSECR)
                    //SYSECR: Write 0 to bit 14.
                    //SYSECR: Write 1 to bit 15

                    systemREG1->SYSECR = systemREG1->SYSECR | 0x8000; //1000 0000 0000 0000
                }

                SW1_high_or_low = HIGH;     // reset for next switch push, ISR will be hit on signal going low, SW1 depressed

                //  set ISR to int on falling edge
                /** @b initialize @b interrupts */

                /** - interrupt polarity */
                gioREG->POL =  0U         /* Bit 0 */
                        | (0U << 1U)  /* Bit 1 */
                        | (0U << 2U)  /* Bit 2 User SW-1 (YELLOW B_CIB)*/
                        | (0U << 3U)  /* Bit 3 */
                        | (0U << 4U)  /* Bit 4 */
                        | (0U << 5U)  /* Bit 5 */
                        | (0U << 6U)  /* Bit 6 */
                        | (0U << 7U); /* Bit 7 */
                break;

            }   // end of switch(SW1_high_or_low)
        } // end if(SW1_is_snapping_snapped1_snapped2_reset_none == NONE)

        break;

    case 3:

        break;

    case 4:

        break;

    case 5:

        break;

    case 6: // GIOA[6] EXAR SPI UART IRQ#, falling edge signals an interrupt request from uart side to TMS570, service me

        break;

    case 7:


        break;


    default:

    {
        // an unused gio pins (use maybe later)
    }
    break;

    }

    /* USER CODE END */
}

/* USER CODE BEGIN (20) */
/* USER CODE END */
#pragma WEAK(i2cNotification)
void i2cNotification(i2cBASE_t *i2c, uint32 flags)      
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (21) */


    /* USER CODE END */
}

/* USER CODE BEGIN (22) */
/* USER CODE END */
#pragma WEAK(mibspiNotification)
void mibspiNotification(mibspiBASE_t *mibspi, uint32 flags)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (25) */

    if(mibspi == mibspiREG1)
    {
        switch(flags)
        {
        case 0x200: //(0U << 9U)  /* TXINT */

            break;

        case 0x100: //(0U << 8U)  /* RXINT */

            break;

        case 0x40:  //(0U << 6U)  /* OVRNINT */

            break;

        case 0x10:  //(0U << 4U)  /* BITERR */

            break;

        case 0x8:   //(0U << 3U)  /* DESYNC */

            break;

        case 0x4:   //(0U << 2U)  /* PARERR */

            break;

        case 0x2:   //(0U << 1U) /* TIMEOUT */

            break;

        case 0x1:   //(0U);  /* DLENERR */

            break;

        default:

        {
            // an unknown flag
        }
        break;
        }    // end switch(flags)
    }   // end if(mibspi == mibspiREG1)


    if(mibspi == mibspiREG3)
    {

    }   // end if(mibspi == mibspiREG3)


    if(mibspi == mibspiREG5)
    {

    }   // end if(mibspi == mibspiREG5)

    /* USER CODE END */
}

/* USER CODE BEGIN (26) */
/* USER CODE END */
#pragma WEAK(mibspiGroupNotification)
void mibspiGroupNotification(mibspiBASE_t *mibspi, uint32 group)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (27) */


    if(mibspi == mibspiREG1)
    {               // spiREG1 [CS0] is used for D to A converter, [CS1]= SPI to UART to Display (final solution)

        switch(group)
        {
        case 0:         /* Transfer Group 0 interrupt */

            // Test for last write to DtoA chip, if last, no further action..
            //      if not, move next set of data into spi data buffer, then transmit, tx complete will cause interupt and return here
            //  We test after the ISR is called, so we test what data went out by looking at the spiRAM1->tx[0].data.
            // Remember [0] = command and D/A ch#, [1] = Data HiByte, [2] = Data LowByte
            //      also, we have encoded into last channel, #7, the "Output All" for the converter, it the "2" in the "27" value.

            test1 = mibspiRAM1->tx[0].data;                         // just some debug code, can I view the tx register, and whats in it?
            test2 = mibspiRAM1->tx[1].data;                         // just some debug code, can I view the tx register, and whats in it?
            test3 = mibspiRAM1->tx[2].data;                         // just some debug code, can I view the tx register, and whats in it?
            test4 = mibspiRAM1->tx[3].data;                         // just some debug code, can I view the tx register, and whats in it?

            //          test1 = DtoA_array[(mibspiRAM1->tx[0].data) +1][0];    // just some debug code, will I really point to next channel?

            if(mibspiRAM1->tx[0].data != 0x0027)    // if not last channel in transmit buffer, send next channel in DtoA_array[]
            {
                mibspiSetData(mibspiREG1, 0, (uint16 *)DtoA_array[(mibspiRAM1->tx[0].data +1)]);        // pull next channel based on what we just sent

                mibspiTransfer(mibspiREG1, 0);  //Initiates a transfer for the specified transfer group, interrupts will send other TGs.
            }
            else
            {
                // do nothing, last message sent..
            }
            break;

        case 1:         /* Transfer Group 1 interrupt */

            break;

        case 2:         /* Transfer Group 2 interrupt */

            break;

        case 3:         /* Transfer Group 3 interrupt */
            // all done, do nothing.

            break;

        case 4:         /* Transfer Group 4 interrupt */

            break;

        case 5:         /* Transfer Group 5 interrupt */

            break;

        case 6:         /* Transfer Group 6 interrupt */

            break;

        case 7:         /* Transfer Group 7 interrupt */

            break;

        default:

        {
            // an unknown group
        }
        break;

        }   // end of switch(group)
    }   // end if(mibspi == mibspiREG1)


    if(mibspi == mibspiREG3)
    {
        // spiREG3 [CS0] is used SD Card, [CS1]= CAN FD   ( debug so can use MicroCore daughter board slot :SPI to UART to Display)
        switch(group)
        {

        case 0:         /* Transfer Group 0 interrupt */

            break;

        case 1:         /* Transfer Group 1 interrupt */


            break;

        case 2:         /* Transfer Group 2 interrupt */



            break;

        case 3:         /* Transfer Group 3 interrupt */



            break;

        case 4:         /* Transfer Group 4 interrupt */


            break;

        case 5:         /* Transfer Group 5 interrupt */

            break;

        case 6:         /* Transfer Group 6 interrupt */

            break;

        case 7:         /* Transfer Group 7 interrupt */

            break;

        default:

        {
            // an unknown group
        }
        break;

        }   // end of switch(group)
    }   // end if(mibspi == mibspiREG3)


    if(mibspi == mibspiREG5)
    {

    }   // end if(mibspi == mibspiREG5)



    /* USER CODE END */
}
/* USER CODE BEGIN (28) */
/* USER CODE END */

#pragma WEAK(sciNotification)
void sciNotification(sciBASE_t *sci, uint32 flags)     
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (29) */

    //  #define sciREG = SCI1     #define scilinREG = SCI2
    if(sci == sciREG)
    {

    }

    if(sci == scilinREG)
    {

        switch(flags)
        {
        case SCI_RX_INT:
            switch(scilinREG_is_CIB_or_Monitor_or_WEB)
            {
            case CIB:
                /* ----------   SCI2 for RS485    ----------------   */
                if(mp < 59)
                {
                    if(scilinREG->RD < 128) /* throw away any garbage */
                    {
                        command_str[mp++] = scilinREG->RD;  // keep adding to command_str until <CR> or mp>59


                        if( scilinREG->RD == 0x0d)  /* if carrage return rec, process message */
                        {
                            if(data_frame == 1)
                            {
                                /* force to MODE 2 just in case there is no trans, if there is MODE 2 is set after a [CR] is sent in trans ISR*/
                                scilinREG->GCR1 = scilinREG->GCR1 | ADDR_MODE; //set to interrupt on address message only   //old: sp_con = MODE2 | REC_ENABLE;
                                mp = 0; /* its my address, reset command_str pointer to receive next command */
                                flag_to_process = 1;     // flag for main to evaluate command string
                                data_frame = FALSE; /* reset to recieve next address */
                            }  // end this is a data frame
                            else
                            {
                                mp = 0;     // reset message pointer
                                if((command_str[0] == CIB_config_data.BOARD_ID[0]) || (command_str[0] == 'E') || (command_str[0] == 'e') || (command_str[0] == '0') || (command_str[0] == 'F') || (command_str[0] == 'f'))  /* is it my address? or all  */
                                {
                                    if((command_str[0] == 'E') || (command_str[0] == 'e') || (command_str[0] == 'F') || (command_str[0] == 'f'))
                                    {
                                        who_is_addressed = ALL; // ALL=3, CIB's (F) or ALL CIB's and Buffer (E)
                                        data_frame = 1;  //  ALL CIB's so listen for data command
                                        scilinREG->GCR1 = scilinREG->GCR1 & ~ADDR_MODE;//set to interrupt on normal message for command data    //old: sp_con = MODE3 | REC_ENABLE;  // switch to space parity to receive command data
                                    }
                                    if((command_str[0] == CIB_config_data.BOARD_ID[0]))
                                    {
                                        who_is_addressed = 1; // ME=1, my address only  just me
                                        data_frame = 1;  //  my address, so listen for data command
                                        scilinREG->GCR1 = scilinREG->GCR1 & ~ADDR_MODE; //set to interrupt on normal message for command data   //old: sp_con = MODE3 | REC_ENABLE;  // switch to space parity to receive command data
                                        //                                      LED_REC_msec_clock = 200;       // set LED on for 100mS
                                        //                                      gioSetBit(hetPORT1, 15, 1 );        // SET LED
                                    }
                                    if((command_str[0] == '0'))
                                    {
                                        who_is_addressed = 0; // BUFFER=0, buffer address only-  do not talk
                                        // just for THIS test- allow address to read command for address '0'
                                        // I need to do this so I can turn on power mode message and NOT engine on message with a p_on to the buffer board address '0'
                                        // I will look at 'who is addressed' in p_on command and if 'E' turn on everything...'0' turn on power mode message and trigger for
                                        // external relay (P1.7) to siganl voltage stand to start sending its waveforms.

                                        data_frame = 1;  //  my address, so listen for data command BUT DO NOT TALK.
                                        scilinREG->GCR1 = scilinREG->GCR1 & ~ADDR_MODE; //set to interrupt on normal message for command data   //old: sp_con = MODE3 | REC_ENABLE;
                                    }

                                }  // end if address is me,all, or buffer
                                // if address is another CIB only, drop through this without changing parity or
                                // setting data frame true so it just waits for next 'mark' parity address to evaluate

                            }  // end else is an address frame
                        }  // if sbuf_rx == carriage return
                    }  /* if  sbuf_rx > 0 */
                } // if mp< 59
                else  // mp > 59
                {

                    /* print message overflow */
                    data_frame = FALSE; /* reset to receive next address */
                    mp=0;
                    memset(command_str,0,255);  /* clears command buffer for next message */
                    memset(words[0],0,225); /* clear words for message processing */
                    flag_to_process = FALSE;
                    strcpy(return_message,"message overflow\r");
                    //  transmit_str_485(return_message);

                }

                /* Re-initialize sciReceive after handling data*/
                break;  // end of case CIB:

            case MONITOR:
                if(mp < 254)
                {
                    if(scilinREG->RD > 0) /* throw away any garbage */
                    {
                        command_str[mp++] = scilinREG->RD;  // keep adding to command_str until <CR> or mp>59
                        sciSendByte(sciREG, scilinREG->RD);     // echo, debug to terminal on port1

                        switch(monitor_rec_index)       // how to handle, variety return bundles of strings from Lantronix based on requests
                        {
                        case 0:     // requested to get into MONTOR Mode, "*** NodeSet 2.0 ***\r\n0>"
                            if( scilinREG->RD == 0x3e)  // try 0x3e = ">"  /* if line feed return rec, 0x0a =\n, process message */
                            {
                                //                              memcpy(command_str, &command_str[strlen(command_str)-2], 3);    // delete "*** NodeSet 2.0 ***" from Lantastic reply and leave ">0" portion
                                //                              strcat(command_str, "\n");  // add CR so main's command parser will process normally
                                strcpy(command_str_mirror, command_str);
                                strcpy(command_str, "0>\n");    // push MONITOR command so main's command parser will process normally, then key off monitor_rec_index
                                flag_to_process = 1;     // flag for main to evaluate command string
                                mp = 0;                 // reset message pointer
                                //                              monitor_rec_index = 0;      // bump for next responce from Lantronix
                                //                              sciSend(scilinREG, 3, "NC\r");  // request IP address
                            }  // if sbuf_rx == try 0x3e = ">"  was line feed

                            break;

                        case 1:     //requested read and store till we see a ">", parse and store IP, push for MAC
                            if( scilinREG->RD == 0x3e)  // try 0x3e = ">"  /* if line feed return rec, 0x0a =\n, process message */
                            {
                                strcpy(command_str_mirror, command_str);
                                strcpy(command_str, "0>\n");    // push MONITOR command so main's command parser will process normally, then key off monitor_rec_index
                                flag_to_process = 1;     // flag for main to evaluate command string
                                mp = 0;                 // reset message pointer
                                //                          monitor_rec_index = 1;      // bump for next responce from Lantronix
                                //                          sciSend(scilinREG, 3, "GM\r");  // request MAC address
                            }  // if sbuf_rx == try 0x3e = ">"  was line feed

                            break;

                        case 2:     //requested read and store till we see a ">", parse and store MAC, push for QUIT and exit MONOTOR MODE
                            if( scilinREG->RD == 0x3e)  // try 0x3e = ">"  /* if line feed return rec, 0x0a =\n, process message */
                            {
                                strcpy(command_str_mirror, command_str);
                                strcpy(command_str, "0>\n");    // push MONITOR command so main's command parser will process normally, then key off monitor_rec_index
                                flag_to_process = 1;     // flag for main to evaluate command string
                                mp = 0;                 // reset message pointer
                                //                          monitor_rec_index = 2;      // bump for next responce from Lantronix
                                //                          sciSend(scilinREG, 3, "QU\r");  // quit and exit MONITOR MODE


                            }  // if sbuf_rx == try 0x3e = ">"  was line feed

                            break;

                        case 3:     //requested read and store till we see a "\n" 0x0a newline, push for QUIT and exit MONOTOR MODE
                            if( scilinREG->RD == 0x0a)  // /* if line feed return rec, 0x0a =\n, process message */
                            {
                                strcpy(command_str_mirror, command_str);
                                strcpy(command_str, "0>\n");    // push MONITOR command so main's command parser will process normally, then key off monitor_rec_index
                                flag_to_process = 1;     // flag for main to evaluate command string
                                mp = 0;                 // reset message pointer
                                //                          monitor_rec_index = 3;      // bump for next responce from Lantronix
                                //                          sciSend(scilinREG, 3, "QU\r");  // quit and exit MONITOR MODE


                            }  // if sbuf_rx == try 0x3e = ">"  was line feed

                            break;

                        default:
                        {
                            //                          flag_to_process = 1;     // flag for main to evaluate command string
                            //                          mp = 0;     // reset message pointer
                        }
                        }   // end of switch(monitor_rec_index)


                    }  /* if  sbuf_rx > 0 */
                } // if mp< 254
                else  // mp > 254
                {

                    /* print message overflow */
                    mp=0;
                    memset(command_str,0,255);  /* clears command buffer for next message */
                    memset(words[0],0,225); /* clear words for message proccessing */
                    flag_to_process = FALSE;
                    strcpy(return_message,"message overflow\r");
                    //  sciSend(scilinREG, strlen(return_message), return_message);

                }

                /* Re-initialize sciReceive after handling data  -- now done at end of case SCI_RX_INT:*/
                //          sciReceive(scilinREG, 1,sbuf_rx);

                break;  ////end of case MONITOR:

            case WEB:
                /* This is a function for variable_input.c ONLY*/
                if(read_variables_flag == 1)
                {
                    read_variables();
                }
                else
                {
                    if(mp < 254)
                    {
                        if(scilinREG->RD > 0) /* throw away any garbage */
                        {
                            command_str[mp++] = scilinREG->RD;  // keep adding to command_str until <CR> or mp>59
                            // try to speed up                  sciSendByte(sciREG, scilinREG->RD);
                            //              sciSend(sciREG, 1,scilinREG->RD);       // echo to port1 - CCS terminal

                            switch(SW1_is_snapping_snapped1_snapped2_reset_none)
                            {
                            case NONE:

                                // User indicator for COMM activity, REC LED will be on briefly when CIB is receiving , TXD will be on briefly when CIB is transmitting
                                if(mp < 59)
                                {
                                    if(scilinREG->RD > 0) /* throw away any garbage */
                                    {
                                        //                                      command_str[mp++] = scilinREG->RD;  // keep adding to command_str until <CR> or mp>59
                                        if( scilinREG->RD == 0x0d)  /* if carrage return rec, process message */
                                        {
                                            flag_to_process = 1;     // flag for main to evaluate command string
                                            mp = 0;                 // reset message pointer
                                            who_is_addressed = ME; // ME=1, my address only  just me
                                            data_frame = 1;  //  my address, so listen for data command
                                            //                                              LED_REC_msec_clock = 200;       // set LED on for 100mS
                                            //                                              gioSetBit(hetPORT1, 15, 1 );        // SET LED

                                        }  // if sbuf_rx == carriage return
                                    }  /* if  sbuf_rx > 0 */
                                } // if mp< 59
                                else  // mp > 59
                                {
                                    /* print message overflow */
                                    mp=0;
                                    memset(command_str,0,255);  /* clears command buffer for next message */
                                    memset(words[0],0,225); /* clear words for message proccessing */
                                    flag_to_process = FALSE;
                                    //                                  strcpy(return_message,"message overflow\r");
                                    //                                  transmit_str_485(return_message);

                                }

                                /* Re-initialize sciReceive after handling data  -- now done at end of case SCI_RX_INT:*/
                                //                                  sciReceive(scilinREG, 1,sbuf_rx);
                                break;
                            case SNAPPING:
                                // while we are waiting for SNAP, spin LEDs, if no SNAP, reset to status to "NONE"
                                if( scilinREG->RD == 0x0d)  /* if carrage return rec, process message, should have rec = snap\s192.168.0.2\r */
                                {
                                    timer_d = system_msec_clock;    // debug

                                    flag_to_process = 1;     // flag for main to evaluate command string
                                    mp = 0;                 // reset message pointer
                                    who_is_addressed = ME; // ME=1, my address only  just me
                                    data_frame = 1;  //  my address, so listen for data command

                                }  // if sbuf_rx == carriage return

                                /* Re-initialize sciReceive after handling data  -- now done at end of case SCI_RX_INT:*/
                                //                              sciReceive(scilinREG, 1,sbuf_rx);
                                break;

                            case SNAPPED1:
                            case SNAPPED2:
                                // "SNAPPED1" and SNAPPED2" indicate snapped to WEB and we will process messages when we recieve a carrage return 0x0d.
                                //      SNAPPED1 only last for 3 seconds, while we flash LEDs to indicate snap is complete, once timed out, personality switches to
                                //      SMAPPED2, this is where it will normally remain for program execution.
                                if( scilinREG->RD == 0x0d)  /* if carrage return rec, process message */
                                {
                                    timer_d = system_msec_clock;    // debug

                                    flag_to_process = 1;     // flag for main to evaluate command string
                                    mp = 0;                 // reset message pointer
                                    who_is_addressed = ME; // ME=1, my address only  just me
                                    data_frame = 1;  //  my address, so listen for data command
                                    //                                  LED_REC_msec_clock = 200;       // set LED on for 100mS
                                    //                                  gioSetBit(hetPORT1, 15, 1 );        // SET LED

                                }  // if sbuf_rx == carriage return

                                /* Re-initialize sciReceive after handling data  -- now done at end of case SCI_RX_INT:*/
                                //                              sciReceive(scilinREG, 1,sbuf_rx);
                                break;

                            case RESET:
                                break;
                            }   // end of switch(SW1_is_snapping_snapped1_snapped2_reset_none)

                        }  /* if  sbuf_rx > 0 */
                    } // if mp< 254
                    else  // mp > 254
                    {

                        /* print message overflow */
                        mp=0;
                        memset(command_str,0,255);  /* clears command buffer for next message */
                        memset(words[0],0,225); /* clear words for message proccessing */
                        flag_to_process = FALSE;
                        strcpy(return_message,"message overflow\r");
                        //  sciSend(scilinREG, strlen(return_message), return_message);

                    }

                    break;  //end of case WEB:
                } //end else(read_variables_flag == 1)

            }   //end switch(scilinREG_is_CIB_or_Monitor_or_WEB)

            /* Re-initialize sciReceive interupt every time, set to interupt after each byte, due to g_sciTransfer_t[1U].length == 0U */
            // there has got to be a better way...
            sciReceive(scilinREG, 1,sbuf_rx);
            break;  // endof case SCI_RX_INT

            case SCI_TX_INT:

                switch(scilinREG_is_CIB_or_Monitor_or_WEB)
                {
                case CIB:
                    /* ---------- SCI2 for RS485    ----------------   */
                    // Note that this ISR case is hit only after the last byte of message is sent to the TX hardware, bits are just starting on the buss

                    //Necessary for RS485 code
                    /* Re-initialize sciReceive after TRANSMITTING data*/
                    /* set to MODE 2 reception (address, only), Mark parity */
                    //spiREG5->PCCLR = 0x00000800; // Set RS485 CS to Receive                                   //old: clrbit(p2reg,3); /* set 485 to rec */
                    //scilinREG->GCR1 = scilinREG->GCR1 | ADDR_MODE; //Set to interrupt on address message only     //old:sp_con = MODE2 | REC_ENABLE;

                    /* Re-initialize sciReceive after handling data*/
                    //sciReceive(scilinREG, 1,sbuf_rx);

                    /*******************************
                     * METHOD #2
                     * (Get out of SCI_TX_INT as fast as possible)
                     * 1. Set tx_to_rx_delay_flag = 1 in SCI_TX_INT
                     * 2. Every 1ms rtiCompare0 checks if flag TX_EMPTY = 1 (indicating sci shift register is empty/last byte sent) and if tx_to_rx_delay_flag = 1
                     * 3. If both requirements met, then enable RX in ADDR mode
                     * 4. Result is RX enable does not occur immediately after last byte is sent, since we have to wait for the next call to rtiCompare0
                     * TESTING:
                     * - Ran for 5 days on 9Bxx parts, no 8's.
                     *******************************/
                    tx_to_rx_delay_flag = 1;
                    break;

                case MONITOR:
                    sciReceive(scilinREG, 1,sbuf_rx);
                    break;

                case WEB:
                    sciReceive(scilinREG, 1,sbuf_rx);

                    if(WEB_header_TX_complete == FALSE)
                    {
                        // SCI_TX_INT hit because header transmit complete, ok to transmit data packet
                        // Now send "data" portion of transmission
                        sciSend(scilinREG,WEB_data_length, return_message);
                        WEB_header_TX_complete = TRUE;  // the ISR will be hit again after the data packet is complete, set this flag so we only run once
                    }

                    break;
                }   // end switch(scilinREG_is_CIB_or_Monitor_or_WEB)


                break;  // endof case SCI_TX_INT
                default:

                    break;

        }   // end switch(flags)
    }   // end if(sci == scilinREG)
    /* USER CODE END */
}

/* USER CODE BEGIN (30) */
/* USER CODE END */

#pragma WEAK(pwmNotification)
void pwmNotification(hetBASE_t * hetREG,uint32 pwm, uint32 notification)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (35) */
    /* USER CODE END */
}

/* USER CODE BEGIN (36) */
/* USER CODE END */
#pragma WEAK(edgeNotification)
void edgeNotification(hetBASE_t * hetREG,uint32 edge)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (37) */
    /* USER CODE END */
}

/* USER CODE BEGIN (38) */
/* USER CODE END */
#pragma WEAK(hetNotification)
void hetNotification(hetBASE_t *het, uint32 offset)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (39) */
    // offset relates to which line of HET code caused interrupt or overflow or exception

    // In SENT, an IRQ is triggered on edge of HET[2] and HET[3] as inputs from product.  Here we stuff new torque values into HET delay timers.
    //      Values are based on torque settings determined in MAIN and stored in array. A flag will indicate if array data update is in progress.
    //      If flag is not set, we skip the HET updates until next IRQ trigger.

    //  aa = offset;    // debug
    timer_h = rtiGetCurrentTick(rtiCOMPARE0);
    timer_j = rtiGetCurrentTick(rtiCOMPARE1);

    //  gioSetBit(hetPORT, 28, 1);      // for testing Digital I/O Board Sent 2ch or 6ch boards, bang Flexray error1 to create pulse on HET[28]
    //  gioSetBit(hetPORT, 28, 0);      //   HalCoGen (for now has set up Flexray error1 HET[30] to be TTL output)
    //  for(loop_cnt_c = 0; loop_cnt_c < 20 ; loop_cnt_c++);
    //  gioSetBit(hetPORT, 28, 1);


    switch(offset)
    {
    case 0:         /* No Interrupt          */

        break;

    case 1:         /* Instruction 0, 32, 64, 96 */
        // Handwheel - Position ChA  will cause a interupt, (for now), just exit, all Handwheel HET Ram updates will be processed on ChB after Address#1 is transmitted

        break;

    case 2:         /* Instruction 1, 33, 65, 97 */

        break;

    case 3:         /* Instruction 2, 34, 66, 98 */

        break;

    case 4:         /* Instruction 3, 35, 67, 99 */

        break;

    case 5:         /* Instruction 4, 36, 68, 100 */

        break;

    case 6:         /* Instruction 5, 37, 69, 101*/

        break;

    case 7:         /* Instruction 6, 38, 70, 102 */

        break;

    case 8:         /* Instruction 7, 39, 71, 103 */


        break;

    case 9:         /* Instruction 8, 40, 72, 104 */

        // H A N D W H E E L   P O S I T I O N  - Load data for transmission as calculated, load into HET .Data register --- Handwheel - Position ChB Address#1 caused interrupt
        // go to HET_T1XX_SENT6ch_1200ns_tick_1_torque_trig_01.h to find address of Instruction in hetRAM11 (built by HET IDE, and placed into CCS Include folder by HalCoGen)
        //      (for now) must process, and update address and CRC in data, plan to change HET code later to update only if handwheel position changes by user

        if(sent_ChB_toggle_HW_address_counter == 0)  // Update handwheel data in sets, (zero set)
        {

            /*      //  ChA  Addr0  SENT1 Data=2048 [0][5]
                hetRAM1->Instruction[9].Data  = het_instruction_data[0][6];   // Falling - Start of SENT_Fixed, Delay for Data0
                hetRAM1->Instruction[10].Data  =    het_instruction_data[0][7];   // Rising  - Start of Data1
                hetRAM1->Instruction[11].Data  =    het_instruction_data[0][8];   // Falling - Start of SENT_Fixed, Delay for Data1
                hetRAM1->Instruction[12].Data  =    het_instruction_data[0][9];   // Rising  - Start of Data2
                hetRAM1->Instruction[13].Data  =    het_instruction_data[0][10];  // Falling - Start of SENT_Fixed, Delay for Data2
                hetRAM1->Instruction[14].Data  =    het_instruction_data[0][11];  // Rising  - Start of CRC
                hetRAM1->Instruction[15].Data  =    het_instruction_data[0][12];  // Falling - Start of SENT_Fixed, Delay for Address
                hetRAM1->Instruction[16].Data  =    het_instruction_data[0][13];  // Rising  - End of SENT_FIxed
                hetRAM1->Instruction[17].Data  =    het_instruction_data[0][14];  // Falling - Start of SENT_Fixed, Delay for CRC
                hetRAM1->Instruction[18].Data  =    het_instruction_data[0][15];  // Rising  - End of SENT_FIxed
             */
            //  ChB  Addr0  SENT2 Data=2048 [1][5]
            hetRAM1->Instruction[30].Data  =    het_instruction_data[1][6];       // Falling - Start of SENT_Fixed, Delay for Data0
            hetRAM1->Instruction[31].Data  =    het_instruction_data[1][7];    // Rising  - Start of Data1
            hetRAM1->Instruction[32].Data  =    het_instruction_data[1][8];    // Falling - Start of SENT_Fixed, Delay for Data1
            hetRAM1->Instruction[33].Data  =    het_instruction_data[1][9];    // Rising  - Start of Data2
            hetRAM1->Instruction[34].Data  =    het_instruction_data[1][10];  // Falling - Start of SENT_Fixed, Delay for Data2
            hetRAM1->Instruction[35].Data  =    het_instruction_data[1][11];  // Rising  - Start of CRC
            hetRAM1->Instruction[36].Data  =    het_instruction_data[1][12];  // Falling - Start of SENT_Fixed, Delay for Address
            hetRAM1->Instruction[37].Data  =    het_instruction_data[1][13];  // Rising  - End of SENT_FIxed
            hetRAM1->Instruction[38].Data  =    het_instruction_data[1][14];  // Falling - Start of SENT_Fixed, Delay for CRC
            hetRAM1->Instruction[39].Data  =    het_instruction_data[1][15];  // Rising  - End of SENT_FIxed



            sent_ChB_toggle_HW_address_counter = 1;
        }

        else        // one's set
        {

            /*      //  ChA  Addr1  SENT3 Data=0 [2][5]
                hetRAM1->Instruction[9].Data  = het_instruction_data[2][6];       // Falling - Start of SENT_Fixed, Delay for Data0
                hetRAM1->Instruction[10].Data  =    het_instruction_data[2][7];   // Rising  - Start of Data1
                hetRAM1->Instruction[11].Data  =    het_instruction_data[2][8];   // Falling - Start of SENT_Fixed, Delay for Data1
                hetRAM1->Instruction[12].Data  =    het_instruction_data[2][9];   // Rising  - Start of Data2
                hetRAM1->Instruction[13].Data  =    het_instruction_data[2][10];  // Falling - Start of SENT_Fixed, Delay for Data2
                hetRAM1->Instruction[14].Data  =    het_instruction_data[2][11];  // Rising  - Start of CRC
                hetRAM1->Instruction[15].Data  =    het_instruction_data[2][12];  // Falling - Start of SENT_Fixed, Delay for Address
                hetRAM1->Instruction[16].Data  =    het_instruction_data[2][13];  // Rising  - End of SENT_FIxed
                hetRAM1->Instruction[17].Data  =    het_instruction_data[2][14];  // Falling - Start of SENT_Fixed, Delay for CRC
                hetRAM1->Instruction[18].Data  =    het_instruction_data[2][15];  // Rising  - End of SENT_FIxed
             */
            //  ChB  Addr1  SENT4 Data=0 [3][5]
            hetRAM1->Instruction[30].Data  =    het_instruction_data[3][6];       // Falling - Start of SENT_Fixed, Delay for Data0
            hetRAM1->Instruction[31].Data  =    het_instruction_data[3][7];    // Rising  - Start of Data1
            hetRAM1->Instruction[32].Data  =    het_instruction_data[3][8];    // Falling - Start of SENT_Fixed, Delay for Data1
            hetRAM1->Instruction[33].Data  =    het_instruction_data[3][9];    // Rising  - Start of Data2
            hetRAM1->Instruction[34].Data  =    het_instruction_data[3][10];  // Falling - Start of SENT_Fixed, Delay for Data2
            hetRAM1->Instruction[35].Data  =    het_instruction_data[3][11];  // Rising  - Start of CRC
            hetRAM1->Instruction[36].Data  =    het_instruction_data[3][12];  // Falling - Start of SENT_Fixed, Delay for Address
            hetRAM1->Instruction[37].Data  =    het_instruction_data[3][13];  // Rising  - End of SENT_FIxed
            hetRAM1->Instruction[38].Data  =    het_instruction_data[3][14];  // Falling - Start of SENT_Fixed, Delay for CRC
            hetRAM1->Instruction[39].Data  =    het_instruction_data[3][15];  // Rising  - End of SENT_FIxed


            sent_ChB_handwheel_update_flag = IDLE;  // Address 1 is last SENT signal sent, done updating HET RAM pair with new data, OK to update new pair, have 2mS to do it
            sent_ChB_toggle_HW_address_counter = 0;
        }


        /*  this code is for debugging --- will cause a blip on het24 to see exactly when hetram is update ends relative to SENT signal on scope
            //  gioSetBit(hetPORT, 28, 1);      // for testing Digital I/O Board Sent 2ch or 6ch boards, bang Flexray error1 to create pulse on HET[28]
            //  gioSetBit(hetPORT, 28, 0);      //   HalCoGen (for now has set up Flexray error1 HET[30] to be TTL output)
            //  for(loop_cnt_c = 0; loop_cnt_c < 20 ; loop_cnt_c++);
            //  gioSetBit(hetPORT, 28, 1);
         */
        break;

    case 10:        /* Instruction 9, 41, 73, 105 */

        if(sent_torque_ChA_update_flag == REQUEST)      // Main is done processing new data, OK to write to HET RAM, else leave immediatly
        {
            // T O R Q U E - Load data for transmission as calculated using tables, shift data Left 7 to aline correctly in HET .Data register.
            // Example- To find address of Instruction in hetRAM1 go to HET_FOAA_SENT2ch_1175ns_tick_03.h  (built by HET IDE, and placed into CCS Include folder by HalCoGen)
            //   Use address to determine offset (case #) to insert hetRAM1 update code.  ISR vector is determend by is associated calling line(Instruction) number.

            /*  this code is for debugging --- will cause a blip on het28 to see exactly when hetram is update starts relative to SENT signal on scope */
            //  gioSetBit(hetPORT, 28, 1);      // for testing Digital I/O Board Sent 2ch or 6ch boards, bang Flexray error1 to create pulse on HET[28]
            //  gioSetBit(hetPORT, 28, 0);      //   HalCoGen (for now has set up Flexray error1 HET[30] to be TTL output)
            //  for(loop_cnt_c = 0; loop_cnt_c < 20 ; loop_cnt_c++);
            //  gioSetBit(hetPORT, 28, 1);

            //  ChA  T1  SENT5 Data=2048 [4][4]
            hetRAM1->Instruction[50].Data  =    het_instruction_data[4][6];   // Falling - Start of SENT_Fixed, Delay for Data0
            hetRAM1->Instruction[51].Data  =    het_instruction_data[4][7];   // Rising  - Start of Data1
            hetRAM1->Instruction[52].Data  =    het_instruction_data[4][8];   // Falling - Start of SENT_Fixed, Delay for Data1
            hetRAM1->Instruction[53].Data  =    het_instruction_data[4][9];   // Rising  - Start of Data2
            hetRAM1->Instruction[54].Data  =    het_instruction_data[4][10];  // Falling - Start of SENT_Fixed, Delay for Data2
            hetRAM1->Instruction[55].Data  =    het_instruction_data[4][11];  // Rising  - Start of CRC
            hetRAM1->Instruction[56].Data  =    het_instruction_data[4][12];  // Falling - Start of SENT_Fixed, Delay for CRC
            hetRAM1->Instruction[57].Data  =    het_instruction_data[4][13];  // Rising  - End of SENT_FIxed

            //  ChA  T2  SENT6 Data=2048 [5][4]
            hetRAM1->Instruction[65].Data  =    het_instruction_data[5][6];   // Falling - Start of SENT_Fixed, Delay for Data0
            hetRAM1->Instruction[66].Data  =    het_instruction_data[5][7];   // Rising  - Start of Data1
            hetRAM1->Instruction[67].Data  =    het_instruction_data[5][8];   // Falling - Start of SENT_Fixed, Delay for Data1
            hetRAM1->Instruction[68].Data  =    het_instruction_data[5][9];   // Rising  - Start of Data2
            hetRAM1->Instruction[69].Data  =    het_instruction_data[5][10];  // Falling - Start of SENT_Fixed, Delay for Data2
            hetRAM1->Instruction[70].Data  =    het_instruction_data[5][11];  // Rising  - Start of CRC
            hetRAM1->Instruction[71].Data  =    het_instruction_data[5][12];  // Falling - Start of SENT_Fixed, Delay for CRC
            hetRAM1->Instruction[72].Data  =    het_instruction_data[5][13];  // Rising  - End of SENT_FIxed

            sent_torque_ChA_update_flag = IDLE;

            /*  this code is for debugging --- will cause a blip on het28 to see exactly when hetram is update ends relative to SENT signal on scope

    //  gioSetBit(hetPORT, 28, 1);      // for testing Digital I/O Board Sent 2ch or 6ch boards, bang Flexray error1 to create pulse on HET[28]
    //  gioSetBit(hetPORT, 28, 0);      //   HalCoGen (for now has set up Flexray error1 HET[30] to be TTL output)
    //  for(loop_cnt_c = 0; loop_cnt_c < 20 ; loop_cnt_c++);
    //  gioSetBit(hetPORT, 28, 1);
             */
        }

        break;

    case 11:        /* Instruction 10, 42, 74, 106 */

        break;

    case 12:        /* Instruction 11, 43, 75, 107 */

        break;

    case 13:        /* Instruction 12, 44, 76, 108 */

        break;

    case 14:        /* Instruction 13, 45, 77, 109 */

        if(sent_torque_ChB_update_flag == REQUEST)      // Main is done processing new data, OK to write to HET RAM, else leave immediatly
        {
            //  ChB  T1  SENT7 Data=2048 [6][4]
            hetRAM1->Instruction[86].Data  =    het_instruction_data[6][6];   // Falling - Start of SENT_Fixed, Delay for Data0
            hetRAM1->Instruction[87].Data  =    het_instruction_data[6][7];   // Rising  - Start of Data1
            hetRAM1->Instruction[88].Data  =    het_instruction_data[6][8];   // Falling - Start of SENT_Fixed, Delay for Data1
            hetRAM1->Instruction[89].Data  =    het_instruction_data[6][9];   // Rising  - Start of Data2
            hetRAM1->Instruction[90].Data  =    het_instruction_data[6][10];  // Falling - Start of SENT_Fixed, Delay for Data2
            hetRAM1->Instruction[91].Data  =    het_instruction_data[6][11];  // Rising  - Start of CRC
            hetRAM1->Instruction[92].Data  =    het_instruction_data[6][12];  // Falling - Start of SENT_Fixed, Delay for CRC
            hetRAM1->Instruction[93].Data  =    het_instruction_data[6][13];  // Rising  - End of SENT_FIxed

            //  ChB  T2  SENT8 Data=2048 [7][4]
            hetRAM1->Instruction[101].Data  =   het_instruction_data[7][6];   // Falling - Start of SENT_Fixed, Delay for Data0
            hetRAM1->Instruction[102].Data  =   het_instruction_data[7][7];   // Rising  - Start of Data1
            hetRAM1->Instruction[103].Data  =   het_instruction_data[7][8];   // Falling - Start of SENT_Fixed, Delay for Data1
            hetRAM1->Instruction[104].Data  =   het_instruction_data[7][9];   // Rising  - Start of Data2
            hetRAM1->Instruction[105].Data  =   het_instruction_data[7][10];  // Falling - Start of SENT_Fixed, Delay for Data2
            hetRAM1->Instruction[106].Data  =   het_instruction_data[7][11];  // Rising  - Start of CRC
            hetRAM1->Instruction[107].Data  =   het_instruction_data[7][12];  // Falling - Start of SENT_Fixed, Delay for CRC
            hetRAM1->Instruction[108].Data  =   het_instruction_data[7][13];  // Rising  - End of SENT_FIxed


            sent_torque_ChB_update_flag = IDLE;
        }

        break;

    case 15:        /* Instruction 14, 46, 78, 110 */

        break;

    case 16:        /* Instruction 15, 47, 79, 111 */

        break;

    case 17:        /* Instruction 16, 48, 80, 112 */

        break;

    case 18:        /* Instruction 17, 49, 81, 113 */

        break;

    case 19:        /* Instruction 18, 50, 82, 114 */

        break;

    case 20:        /* Instruction 19, 51, 83, 115 */

        if(sent_ChA_toggle_HW_address_counter == 0)  // Update handwheel data in sets, (zero set)
        {
            //  ChA  Addr0  SENT1 Data=2048 [0][5]
            hetRAM1->Instruction[9].Data  = het_instruction_data[0][6];   // Falling - Start of SENT_Fixed, Delay for Data0
            hetRAM1->Instruction[10].Data  =    het_instruction_data[0][7];   // Rising  - Start of Data1
            hetRAM1->Instruction[11].Data  =    het_instruction_data[0][8];   // Falling - Start of SENT_Fixed, Delay for Data1
            hetRAM1->Instruction[12].Data  =    het_instruction_data[0][9];   // Rising  - Start of Data2
            hetRAM1->Instruction[13].Data  =    het_instruction_data[0][10];  // Falling - Start of SENT_Fixed, Delay for Data2
            hetRAM1->Instruction[14].Data  =    het_instruction_data[0][11];  // Rising  - Start of CRC
            hetRAM1->Instruction[15].Data  =    het_instruction_data[0][12];  // Falling - Start of SENT_Fixed, Delay for Address
            hetRAM1->Instruction[16].Data  =    het_instruction_data[0][13];  // Rising  - End of SENT_FIxed
            hetRAM1->Instruction[17].Data  =    het_instruction_data[0][14];  // Falling - Start of SENT_Fixed, Delay for CRC
            hetRAM1->Instruction[18].Data  =    het_instruction_data[0][15];  // Rising  - End of SENT_FIxed

            sent_ChA_toggle_HW_address_counter = 1;
        }

        else        // one's set
        {


            //  ChA  Addr1  SENT3 Data=0 [2][5]
            hetRAM1->Instruction[9].Data  = het_instruction_data[2][6];       // Falling - Start of SENT_Fixed, Delay for Data0
            hetRAM1->Instruction[10].Data  =    het_instruction_data[2][7];   // Rising  - Start of Data1
            hetRAM1->Instruction[11].Data  =    het_instruction_data[2][8];   // Falling - Start of SENT_Fixed, Delay for Data1
            hetRAM1->Instruction[12].Data  =    het_instruction_data[2][9];   // Rising  - Start of Data2
            hetRAM1->Instruction[13].Data  =    het_instruction_data[2][10];  // Falling - Start of SENT_Fixed, Delay for Data2
            hetRAM1->Instruction[14].Data  =    het_instruction_data[2][11];  // Rising  - Start of CRC
            hetRAM1->Instruction[15].Data  =    het_instruction_data[2][12];  // Falling - Start of SENT_Fixed, Delay for Address
            hetRAM1->Instruction[16].Data  =    het_instruction_data[2][13];  // Rising  - End of SENT_FIxed
            hetRAM1->Instruction[17].Data  =    het_instruction_data[2][14];  // Falling - Start of SENT_Fixed, Delay for CRC
            hetRAM1->Instruction[18].Data  =    het_instruction_data[2][15];  // Rising  - End of SENT_FIxed

            sent_ChA_handwheel_update_flag = IDLE;  // Address 1 is last SENT signal sent, done updating HET RAM pair with new data, OK to update new pair, have 2mS to do it
            sent_ChA_toggle_HW_address_counter = 0;
        }

        break;

    case 21:        /* Instruction 20, 52, 84, 116 */

        break;

    case 22:        /* Instruction 21, 53, 85, 117 */

        break;

    case 23:        /* Instruction 22, 54, 86, 118 */

        break;

    case 24:        /* Instruction 23, 55, 87, 119 */

        break;

    case 25:        /* Instruction 24, 56, 88, 120 */

        break;

    case 26:        /* Instruction 25, 57, 89, 121 */

        break;

    case 27:        /* Instruction 26, 58, 90, 122 */

        break;

    case 28:        /* Instruction 27, 59, 91, 123 */

        break;

    case 29:        /* Instruction 28, 60, 92, 124 */

        break;

    case 30:        /* Instruction 29, 61, 93, 125 */

        break;

    case 31:        /* Instruction 30, 62, 94, 126 */

        break;

    case 32:        /* Instruction 31, 63, 95, 127 */

        break;

    case 33:        /* Program OverFlow */
        //  gioSetBit(hetPORT, 28, 1);      // for testing Digital I/O Board Sent 2ch or 6ch boards, bang Flexray error1 to create pulse on HET[28]
        //  gioSetBit(hetPORT, 28, 0);      //   HalCoGen (for now has set up Flexray error1 HET[30] to be TTL output)
        //  for(loop_cnt_c = 0; loop_cnt_c < 20 ; loop_cnt_c++);
        //  gioSetBit(hetPORT, 28, 1);
        break;

    case 34:        /* APCNT Underflow */
        //  gioSetBit(hetPORT, 28, 1);      // for testing Digital I/O Board Sent 2ch or 6ch boards, bang Flexray error1 to create pulse on HET[28]
        //  gioSetBit(hetPORT, 28, 0);      //   HalCoGen (for now has set up Flexray error1 HET[30] to be TTL output)
        //  for(loop_cnt_c = 0; loop_cnt_c < 20 ; loop_cnt_c++);
        //  gioSetBit(hetPORT, 28, 1);
        break;

    case 35:        /* APCNT overflow */
        //  gioSetBit(hetPORT, 28, 1);      // for testing Digital I/O Board Sent 2ch or 6ch boards, bang Flexray error1 to create pulse on HET[28]
        //  gioSetBit(hetPORT, 28, 0);      //   HalCoGen (for now has set up Flexray error1 HET[30] to be TTL output)
        //  for(loop_cnt_c = 0; loop_cnt_c < 20 ; loop_cnt_c++);
        //  gioSetBit(hetPORT, 28, 1);
        break;

    default:

    {
        // an unknown flag
    }
    break;

    }   // end of switch(offset)

    timer_i = rtiGetCurrentTick(rtiCOMPARE0);
    timer_k = rtiGetCurrentTick(rtiCOMPARE1);

    /* USER CODE END */
}

/* USER CODE BEGIN (40) */
/* USER CODE END */


/* USER CODE BEGIN (43) */
/* USER CODE END */


/* USER CODE BEGIN (47) */
/* USER CODE END */


/* USER CODE BEGIN (50) */
/* USER CODE END */


/* USER CODE BEGIN (53) */
/* USER CODE END */

#pragma WEAK(dmaGroupANotification)
void dmaGroupANotification(dmaInterrupt_t inttype, uint32 channel)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
/* USER CODE BEGIN (54) */
    /* USER CODE END */
}
/* USER CODE BEGIN (55) */
/* USER CODE END */

/* USER CODE BEGIN (56) */
/* USER CODE END */

/* USER CODE BEGIN (58) */
/* USER CODE END */

/* USER CODE BEGIN (60) */
#pragma WEAK(frayStatusChangeNotification)
void frayStatusChangeNotification(frayCommBASE_t *cc, uint32 notification)
{
    switch(target_product)
    {
    case TARGET_BMW_FAAR_WE:
    {
        switch(notification)
        {
        case frayNOTIFICATION_MSG_RECEIVED:
        {
            uint32 intCase = frayCommREG->NDAT1;

            if((intCase & 0x4000) == 0x4000)
            {

                recieveFlexray(fray_rx_data,XCP_RX);
                memcpy(&fray_variable_data[1],&fray_rx_data[1],3);\
                memcpy(&fray_variable_data[0],&fray_rx_data[2][3], 1);

                if(DAQ_mode_on_off == ON)
                {
                    handle_DAQ_notification_fray();
                }
                else if(XCP_fast_rate_active_flag == YES)
                {
                    handle_XCP_fast_rate_notification();
                }
                else //else dumperr requests. Don't want other 2 interfering
                {
                    switch(fray_request_index)  // requesting multiple vaiables using same messageBox, can_request_index keeps track, store and parse in main
                    {

                    case 0:     // do nothing with request_connect data, setup and trans next request

                        transmitFlexray(XCP_FAAR_WE_SysStMod_Rqst,XCP_TX, 3);

                        break;

                    case 1:

                        memcpy(&fray_dump_err_data[1],&fray_variable_data,8);

                        transmitFlexray(XCP_FAAR_WE_BattVltg_BrdgVltg_Rqst,XCP_TX, 3);

                        break;

                    case 2:

                        memcpy(&fray_dump_err_data[2],&fray_variable_data,8);

                        transmitFlexray(XCP_FAAR_WE_BattRtnCurrAmpr_Rqst,XCP_TX, 3);

                        break;

                    case 3:

                        memcpy(&fray_dump_err_data[3],&fray_variable_data,8);

                        transmitFlexray(XCP_FAAR_WE_HwAgArbn_Rqst,XCP_TX, 3);

                        break;

                    case 4:

                        memcpy(&fray_dump_err_data[4],&fray_variable_data,8);

                        transmitFlexray(XCP_FAAR_WE_HwTq4Meas_Rqst,XCP_TX, 3);

                        break;

                    case 5:

                        memcpy(&fray_dump_err_data[5],&fray_variable_data,8);

                        transmitFlexray(XCP_FAAR_WE_HwTq5Meas_Rqst,XCP_TX, 3);

                        break;

                    case 6:

                        memcpy(&fray_dump_err_data[6],&fray_variable_data,8);

                        transmitFlexray(XCP_FAAR_WE_HwTqArbn_Rqst,XCP_TX, 3);

                        break;

                    case 7:

                        memcpy(&fray_dump_err_data[7],&fray_variable_data,8);

                        transmitFlexray(XCP_FAAR_WE_MotCurrSumA_Rqst,XCP_TX, 3);

                        break;

                    case 8:

                        memcpy(&fray_dump_err_data[8],&fray_variable_data,8);

                        transmitFlexray(XCP_FAAR_WE_MotCurrSumB_Rqst,XCP_TX, 3);

                        break;

                    case 9:

                        memcpy(&fray_dump_err_data[9],&fray_variable_data,8);

                        transmitFlexray(XCP_FAAR_WE_MotCurrSumC_Rqst,XCP_TX, 3);

                        break;

                    case 10:

                        memcpy(&fray_dump_err_data[10],&fray_variable_data,8);

                        transmitFlexray(XCP_FAAR_WE_MotVelCrf_Rqst,XCP_TX, 3);

                        break;

                    case 11:

                        memcpy(&fray_dump_err_data[11],&fray_variable_data,8);

                        transmitFlexray(XCP_FAAR_WE_MotTqCmd_Rqst,XCP_TX, 3);

                        break;

                    case 12:

                        memcpy(&fray_dump_err_data[12],&fray_variable_data,8);

                        transmitFlexray(XCP_FAAR_WE_MotTqEstimd_Rqst,XCP_TX, 3);

                        break;

                    case 13:

                        memcpy(&fray_dump_err_data[13],&fray_variable_data,8);

                        transmitFlexray(XCP_FAAR_WE_LoaSca_Rqst,XCP_TX, 3);

                        break;

                    case 14:

                        memcpy(&fray_dump_err_data[14],&fray_variable_data,8);

                        transmitFlexray(XCP_FAAR_WE_EcuTMeas_Rqst,XCP_TX, 3);

                        break;

                    case 15:

                        memcpy(&fray_dump_err_data[15],&fray_variable_data,8);

                        transmitFlexray(XCP_FAAR_WE_MotWidgT_Rqst,XCP_TX, 3);

                        break;

                    case 16:

                        memcpy(&fray_dump_err_data[16],&fray_variable_data,8);

                        transmitFlexray(XCP_FAAR_WE_MotMagT_Rqst,XCP_TX, 3);

                        break;

                    case 17:

                        memcpy(&fray_dump_err_data[17],&fray_variable_data,8);

                        transmitFlexray(XCP_FAAR_WE_MotFetT_Rqst,XCP_TX, 3);

                        break;

                    case 18:
                        //  all done, index bumped for main to test
                        memcpy(&fray_dump_err_data[18],&fray_variable_data,8);

                        break;

                        //                        case 19:
                            //                            //  all done, index bumped for main to test
                        //
                        //                            memcpy(&fray_dump_err_data[19],&fray_variable_data,8);
                        //
                        //                            break;

                    }   // end switch(can_request_index)

                    fray_request_index++;
                }


            } //end if((intCase & 0x4000) == 0x4000){
            //Interrupt case for message buffer input 0x12 (XCP Dynamic)
            //This is specifically for DAQ mode responses
            else if((intCase & 0x40000) == 0x40000)
            {
                recieveFlexray(fray_rx_data,XCP_DYNAMIC_RX);
                //***** Swap fray data from little endian ****//
                int i,j;
                for(j=0;j<64;j++){for(i=0;i<4;i++)
                {
                    swapped_fray_rx_data[j][i] = fray_rx_data[j][3-i]; //WORKS as intended, PKH 16APR18
                }}

                //***** Store in 1 big array ****//
                memcpy(xcp_dynamic_rx_data,swapped_fray_rx_data,256); //WORKS as intended, PKH 16APR18

                if(DAQ_mode_on_off == ON)
                {
                    //call same function, but DAQ_mode_step_cntr should be at 3000 for the "store data" case.
                    handle_DAQ_notification_fray();
                }

                //uint8 T1[8] = {0};
                //uint8 T2[8] = {0};
                //memcpy(&T1[0], &fray_rx_data[2][1],4);
                //memcpy(&T2[0], &fray_rx_data[3][1],4);

                //canTransmit(canREG1, 0x02, ( uint8 *) T1);
                //canTransmit(canREG1, 0x03, ( uint8 *) T2);


                //memcpy t1adc
                //memcpy t2adc

                //cantransmit on ID 11 transmit T1adc
                //canTransmit on ID 12 transmit T2adc

                //memcpy(&fray_variable_data[1],&fray_rx_data[1],3);
            } //end else if((intCase & 0x40000) == 0x40000)
            //THIS IS MFG_RX (manufacturing service flexray ID) response interrupt handler
            else if((intCase & 0x20000) == 0x20000)
            {
                recieveFlexray(fray_rx_data,MFG_RX);
                int i,j;
                for(j=0;j<64;j++)
                {
                    for(i=0;i<4;i++)
                    {
                        swapped_fray_rx_data[j][i] = fray_rx_data[j][3-i];
                    }
                }
                fray_rx_data_length = swapped_fray_rx_data[1][3];
                intCase = intCase;

                switch(ECU1_XCP_reply_index)   // requesting multiple requests using same messageBox, ECU1_XCP_reply_index keeps track, clear_dtcs_flag determines action
                {

                //DTCs manufact service -------------------------------------------------------
                case 1:
                    if(fray_rx_data[0][0] = 0x30 && fray_rx_data[0][2] == 0xF4 ) //&& fray_rx_data[1][0] == 0x02 && fray_rx_data[1][2] == 0x02 )  // positive responce from enter Nexteer mode
                    {
                        if(ECU1_clear_DTCs_flag == 1)         // clear DTC's mode
                        {
                            transmitFlexray(MFG_TX_clear_dtcs_MSG,MFG_TX, 3);
                        }
                        else       // get DTC's mode

                        {
                            transmitFlexray(MFG_TX_get_dtcs_MSG,MFG_TX, 3);
                        }

                        ECU1_XCP_reply_index++;
                    }  // end positive responce nexteer mode

                    break;

                case 2:
                    //24-3 = 21 (0x18-0x03) bytes of data stored
                    if(fray_rx_data[1][2] == 0x18)
                    {
                        dtc1[0][0] =  fray_rx_data[2][0];
                        dtc1[0][1] =  fray_rx_data[3][3];
                        dtc1[0][2] =  fray_rx_data[3][2];
                        dtc1[0][3] =  fray_rx_data[3][1];
                        dtc1[0][4] =  fray_rx_data[3][0];

                        dtc1[1][0] =  fray_rx_data[4][3];
                        dtc1[1][1] =  fray_rx_data[4][2];
                        dtc1[1][2] =  fray_rx_data[4][1];
                        dtc1[1][3] =  fray_rx_data[4][0];
                        dtc1[1][4] =  fray_rx_data[5][3];

                        dtc1[2][0] =  fray_rx_data[5][2];
                        dtc1[2][1] =  fray_rx_data[5][1];
                        dtc1[2][2] =  fray_rx_data[5][0];
                        dtc1[2][3] =  fray_rx_data[6][3];
                        dtc1[2][4] =  fray_rx_data[6][2];

                        dtc1[3][0] =  fray_rx_data[6][1];
                        dtc1[3][1] =  fray_rx_data[6][0];
                        dtc1[3][2] =  fray_rx_data[7][3];
                        dtc1[3][3] =  fray_rx_data[7][2];
                        dtc1[3][4] =  fray_rx_data[7][1];

                        dtc1[4][0] =  fray_rx_data[7][0];

                        transmitFlexray(HW_ACK,MFG_TX, 2);
                    }

                    ECU1_XCP_reply_index++;

                    break;

                case 3:
                    //26 (0x1A) bytes of data stored
                    if(fray_rx_data[1][3] == 0x51)
                    {
                        dtc1[4][1] =  fray_rx_data[1][1];
                        dtc1[4][2] =  fray_rx_data[1][0];
                        dtc1[4][3] =  fray_rx_data[2][3];
                        dtc1[4][4] =  fray_rx_data[2][2];

                        dtc1[5][0] =  fray_rx_data[2][1];
                        dtc1[5][1] =  fray_rx_data[2][0];
                        dtc1[5][2] =  fray_rx_data[3][3];
                        dtc1[5][3] =  fray_rx_data[3][2];
                        dtc1[5][4] =  fray_rx_data[3][1];

                        dtc1[6][0] =  fray_rx_data[3][0];
                        dtc1[6][1] =  fray_rx_data[4][3];
                        dtc1[6][2] =  fray_rx_data[4][2];
                        dtc1[6][3] =  fray_rx_data[4][1];
                        dtc1[6][4] =  fray_rx_data[4][0];

                        dtc1[7][0] =  fray_rx_data[5][3];
                        dtc1[7][1] =  fray_rx_data[5][2];
                        dtc1[7][2] =  fray_rx_data[5][1];
                        dtc1[7][3] =  fray_rx_data[5][0];
                        dtc1[7][4] =  fray_rx_data[6][3];

                        dtc1[8][0] =  fray_rx_data[6][2];
                        dtc1[8][1] =  fray_rx_data[6][1];
                        dtc1[8][2] =  fray_rx_data[6][0];
                        dtc1[8][3] =  fray_rx_data[7][3];
                        dtc1[8][4] =  fray_rx_data[7][2];

                        dtc1[9][0] =  fray_rx_data[7][1];
                        dtc1[9][1] =  fray_rx_data[7][0];

                    }

                    ECU1_XCP_reply_index++;

                    break;

                case 4:
                    //26 (0x1A) bytes of data stored
                    if(fray_rx_data[1][3]  == 0x52)
                    {

                        dtc1[9][2] =  fray_rx_data[1][1];
                        dtc1[9][3] =  fray_rx_data[1][0];
                        dtc1[9][4] =  fray_rx_data[2][3];

                        dtc1[10][0] =  fray_rx_data[2][2];
                        dtc1[10][1] =  fray_rx_data[2][1];
                        dtc1[10][2] =  fray_rx_data[2][0];
                        dtc1[10][3] =  fray_rx_data[3][3];
                        dtc1[10][4] =  fray_rx_data[3][2];

                        dtc1[11][0] =  fray_rx_data[3][1];
                        dtc1[11][1] =  fray_rx_data[3][0];
                        dtc1[11][2] =  fray_rx_data[4][3];
                        dtc1[11][3] =  fray_rx_data[4][2];
                        dtc1[11][4] =  fray_rx_data[4][1];

                        dtc1[12][0] =  fray_rx_data[4][0];
                        dtc1[12][1] =  fray_rx_data[5][3];
                        dtc1[12][2] =  fray_rx_data[5][2];
                        dtc1[12][3] =  fray_rx_data[5][1];
                        dtc1[12][4] =  fray_rx_data[5][0];

                        dtc1[13][0] =  fray_rx_data[6][3];
                        dtc1[13][1] =  fray_rx_data[6][2];
                        dtc1[13][2] =  fray_rx_data[6][1];
                        dtc1[13][3] =  fray_rx_data[6][0];
                        dtc1[13][4] =  fray_rx_data[7][3];

                        dtc1[14][0] =  fray_rx_data[7][2];
                        dtc1[14][1] =  fray_rx_data[7][1];
                        dtc1[14][2] =  fray_rx_data[7][0];
                        //transmitFlexray(HW_ACK,MFG_TX, 1);
                    }

                    ECU1_XCP_reply_index++;

                    break;

                case 5:
                    //26 (0x1A) bytes of data stored, 1 byte left
                    if(fray_rx_data[1][3]  == 0x53)
                    {

                        dtc1[14][3] =  fray_rx_data[1][1];
                        dtc1[14][4] =  fray_rx_data[1][0];

                        dtc1[15][0] =  fray_rx_data[2][3];
                        dtc1[15][1] =  fray_rx_data[2][2];
                        dtc1[15][2] =  fray_rx_data[2][1];
                        dtc1[15][3] =  fray_rx_data[2][0];
                        dtc1[15][4] =  fray_rx_data[3][3];

                        dtc1[16][0] =  fray_rx_data[3][2];
                        dtc1[16][1] =  fray_rx_data[3][1];
                        dtc1[16][2] =  fray_rx_data[3][0];
                        dtc1[16][3] =  fray_rx_data[4][3];
                        dtc1[16][4] =  fray_rx_data[4][2];

                        dtc1[17][0] =  fray_rx_data[4][1];
                        dtc1[17][1] =  fray_rx_data[4][0];
                        dtc1[17][2] =  fray_rx_data[5][3];
                        dtc1[17][3] =  fray_rx_data[5][2];
                        dtc1[17][4] =  fray_rx_data[5][1];

                        dtc1[18][0] =  fray_rx_data[5][0];
                        dtc1[18][1] =  fray_rx_data[6][3];
                        dtc1[18][2] =  fray_rx_data[6][2];
                        dtc1[18][3] =  fray_rx_data[6][1];
                        dtc1[18][4] =  fray_rx_data[6][0];

                        dtc1[19][0] =  fray_rx_data[7][3];
                        dtc1[19][1] =  fray_rx_data[7][2];
                        dtc1[19][2] =  fray_rx_data[7][1];
                        dtc1[19][3] =  fray_rx_data[7][0];

                        //transmitFlexray(HW_ACK,MFG_TX, 1);
                    }

                    ECU1_XCP_reply_index++;

                    break;

                case 6:
                    //Store last byte of NTC requests
                    if(fray_rx_data[1][3]  == 0x90)
                    {

                        dtc1[19][4] =  fray_rx_data[1][1];
                    }
                    ECU1_XCP_reply_index++;
                    break;

                    //sys state manufact service -------------------------------------------------------
                case 11:
                    if(swapped_fray_rx_data[0][3] = 0x30 && swapped_fray_rx_data[0][1] == 0xF4 ) //&& fray_rx_data[1][0] == 0x02 && fray_rx_data[1][2] == 0x02 )  // positive responce from enter Nexteer mode
                    {
                        transmitFlexray(MFG_TX_system_state_MSG,MFG_TX, 4); //length of 3 is probabl fine
                        ECU1_XCP_reply_index++;
                    }  // end positive responce nexteer mode

                    break;

                case 12:

                    system_state_MFG = swapped_fray_rx_data[2][3];
                    ECU1_XCP_reply_index++;

                    break;

                    //software version manufact service -------------------------------------------------------
                case 21:
                    if(fray_rx_data[0][0] = 0x30 && fray_rx_data[0][2] == 0xF4 ) //&& fray_rx_data[1][0] == 0x02 && fray_rx_data[1][2] == 0x02 )  // positive responce from enter Nexteer mode
                    {
                        //send request service F194
                        transmitFlexray(MFG_TX_request_software_version_MSG,MFG_TX, 4); //length of 3 is probabl fine
                        ECU1_XCP_reply_index++;
                    }  // end positive responce nexteer mode

                    break;

                case 22:
                    if(fray_rx_data[0][0] = 0x30 && fray_rx_data[0][2] == 0xF4 ) //&& fray_rx_data[1][0] == 0x02 && fray_rx_data[1][2] == 0x02 )  // positive responce from enter Nexteer mode
                    {
                        //store software version
                        //determine how many words to parse through

                        memcpy(&software_version_MFG[0],&swapped_fray_rx_data[2][3],fray_rx_data_length-3);

                        //send request for software rev F195
                        transmitFlexray(MFG_TX_request_software_rev_MSG,MFG_TX, 4); //length of 3 is probabl fine

                        ECU1_XCP_reply_index++;
                    }  // end positive responce nexteer mode

                    break;

                case 23:

                    memcpy(&software_rev_MFG[0],&swapped_fray_rx_data[2][3],fray_rx_data_length-3);
                    ECU1_XCP_reply_index++;

                    break;

                default:

                    ECU1_XCP_reply_index = 0;

                    break;

                } // end switch on ECU1_XCP_reply_index

            } //end else if((intCase & 0x20000) == 0x20000){


            break;
        } //end case frayNOTIFICATION_MSG_RECEIVED:


        case frayNOTIFICATION_MSG_TRANSMITTED:
        {
            //if(ign1_status > 0)
            //{
            uint32 intCase = ((frayCommREG->MHDS)&0x7F0000)>>16;
            uint8 crc;

            //if(ign1_status > OFF)
            //{
            bullshit_count++;

            if(bullshit_count >499){
                bullshit_count = 0;
            }

            crc = 0;
            if(intCase == 0xB){
                count1 = 1;
                CON_VEH_FAAR_WE_CNT++;
                bullshit_array[bullshit_count][0] = (frayCommREG->SCV) & 0x7FF;
                bullshit_array[bullshit_count][1] = (frayCommREG->MHDS)&0x7F0000;
                if(CON_VEH_FAAR_WE_CNT > 0xF){
                    CON_VEH_FAAR_WE_CNT = 0;
                }
                CON_VEH_FAAR_WE_MSG[0][2] = (CON_VEH_FAAR_WE_MSG[0][2] & 0xF0) | CON_VEH_FAAR_WE_CNT; //result is X0 | CNT

                uint8 longMessage[7] = {0};
                CON_VEH_FAAR_WE_MSG[0][3] = 0; //Don't use previous CRC to calculate next CRC
                longMessage[0] = CON_VEH_FAAR_WE_MSG[0][2];
                longMessage[1] = CON_VEH_FAAR_WE_MSG[0][1];
                longMessage[2] = CON_VEH_FAAR_WE_MSG[0][0];
                longMessage[3] = CON_VEH_FAAR_WE_MSG[1][3];
                longMessage[4] = CON_VEH_FAAR_WE_MSG[1][2];
                longMessage[5] = CON_VEH_FAAR_WE_MSG[1][1];
                longMessage[6] = CON_VEH_FAAR_WE_MSG[1][0];

                crc = BMW_CRC(0xC5,&longMessage[0],7);
                //crc = 0;
                //CON_VEH_FAAR_WE_MSG[0][3] = crc;
                //crc = BMW_CRC(0x197,&CON_VEH_FAAR_WE_MSG[0],7);
                CON_VEH_FAAR_WE_MSG[0][3] = crc;
                transmitFlexray(CON_VEH_FAAR_WE_MSG,CON_VEH_FAAR_WE,2);
            }
            else if(intCase == 0xC){

                count1 = 2;
                ST_CENG_FAAR_W_CNT++;
                bullshit_array[bullshit_count][0] = (frayCommREG->SCV) & 0x7FF;
                bullshit_array[bullshit_count][1] = (frayCommREG->MHDS)&0x7F0000;

                if(ST_CENG_FAAR_W_CNT > 0xF){
                    ST_CENG_FAAR_W_CNT = 0;
                }
                ST_CENG_FAAR_WE_MSG[0][2] = (ST_CENG_FAAR_WE_MSG[0][2] & 0xF0) | ST_CENG_FAAR_W_CNT;
                uint8 longMessage[7] = {0};
                ST_CENG_FAAR_WE_MSG[0][3] = 0; //Don't use previous CRC to calculate next CRC
                longMessage[0] = ST_CENG_FAAR_WE_MSG[0][2];
                longMessage[1] = ST_CENG_FAAR_WE_MSG[0][1];
                longMessage[2] = ST_CENG_FAAR_WE_MSG[0][0];
                longMessage[3] = ST_CENG_FAAR_WE_MSG[1][3];
                longMessage[4] = ST_CENG_FAAR_WE_MSG[1][2];
                longMessage[5] = ST_CENG_FAAR_WE_MSG[1][1];
                longMessage[6] = ST_CENG_FAAR_WE_MSG[1][0];

                crc = BMW_CRC(0x78,&longMessage[0],7);
                //crc = BMW_CRC(0x78,&ST_CENG_FAAR_WE_MSG[0],4);
                ST_CENG_FAAR_WE_MSG[0][3] = crc;

                transmitFlexray(ST_CENG_FAAR_WE_MSG,ST_CENG_FAAR_WE,2);
            }

            else if(intCase == 0xF){

                if(count1 != 3 && count1 != 0){
                    count1 = 3;
                    uint32 test = (((frayCommREG->MTCCV) & 0x3F0000)>>16)-3;
                    if((test % 4) == 0){
                        bullshit_array[bullshit_count][0] = (frayCommREG->SCV) & 0x7FF;
                        bullshit_array[bullshit_count][1] = (frayCommREG->MHDS)&0x7F0000;
                        VEH_COG_FAAR_WE_CNT = VEH_COG_FAAR_WE_CNT + 1;//VEH_COG_FAAR_WE_CNT = VEH_COG_FAAR_WE_MSG[0][2] + 1;

                        if(VEH_COG_FAAR_WE_CNT > 0xF){
                            VEH_COG_FAAR_WE_CNT = 0;
                        }
                        VEH_COG_FAAR_WE_MSG[0][2] = (VEH_COG_FAAR_WE_MSG[0][2] & 0xF0) | VEH_COG_FAAR_WE_CNT; //result is 0xXXXX0 | CNT = 0xXXXXCCCC

                        // vehicle speed bytes
                        memcpy(&VEH_COG_FAAR_WE_MSG[0][0], &MTS_Vehicle_Speed_HEX_LoByte,1);  // veh speed low byte
                        memcpy(&VEH_COG_FAAR_WE_MSG[0][1], &MTS_Vehicle_Speed_HEX_HiByte,1);  // heh speed high byte

                        uint8 long_message[5] = {0};
                        VEH_COG_FAAR_WE_MSG[0][3] = 0; //Don't use previous CRC to calculate next CRC
                        long_message[0] = VEH_COG_FAAR_WE_MSG[0][2];
                        long_message[1] = VEH_COG_FAAR_WE_MSG[0][1];
                        long_message[2] = VEH_COG_FAAR_WE_MSG[0][0];
                        long_message[3] = VEH_COG_FAAR_WE_MSG[1][3];
                        long_message[4] = VEH_COG_FAAR_WE_MSG[1][2];
                        //crc = BMW_CRC(0xC,&VEH_COG_FAAR_WE_MSG[0],5);
                        crc = BMW_CRC(0xC,&long_message[0],4);
                        VEH_COG_FAAR_WE_MSG[0][3] = crc;
                        //VEH_COG_FAAR_WE_MSG[0][3] = CrcTable_1[VEH_COG_FAAR_WE_CNT];
                        //Currently CRC is hard-coded. Need to swap first 4 bytes of crc calc for endianness. Without swap, CRC is wrong.
                        transmitFlexray(VEH_COG_FAAR_WE_MSG,VEH_COG_FAAR_WE,2);
                    }
                }
            }
            else {
                crc = 1;
            }
            //} //end if(ign1_status > OFF)
        } //end case frayNOTIFICATION_MSG_TRANSMITTED:
        default:
        {
            break;
        }
        } //end switch(notification)
    }
    break; // end case TARGET_BMW_FAAR_WE:

    case TARGET_BMW_UKL:
    {
        switch(notification)
        {
        case frayNOTIFICATION_MSG_RECEIVED:
        {
            uint32 intCase = frayCommREG->NDAT1;

            if((intCase & 0x4000) == 0x4000){

                recieveFlexray(fray_rx_data,XCP_RX);
                memcpy(&fray_variable_data[1],&fray_rx_data[1],3);\
                memcpy(&fray_variable_data[0],&fray_rx_data[2][3], 1);

                if(DAQ_mode_on_off == ON)
                {
                    handle_DAQ_notification_fray();
                }
                else if(XCP_fast_rate_active_flag == YES)
                {
                    handle_XCP_fast_rate_notification();
                }
                else //else dumperr requests. Don't want other 2 interfering
                {
                    switch(fray_request_index)       // requesting multiple variables using same messageBox, can_request_index keeps track, store and parse in main
                    {

                    case 0:     // do nothing with request_connect data, setup and trans next request

                        transmitFlexray(XCP_UKL_Batt_Volt_Rqst,XCP_TX, 3);

                        break;

                    case 1:     // store XCP_Batt_Volt_Rqst, setup and trans next request

                        memcpy(&fray_dump_err_data[1],&fray_variable_data,8);

                        transmitFlexray(XCP_UKL_Batt_Curr_Rqst,XCP_TX, 3);

                        break;

                    case 2:     // store XCP_Batt_Curr_Rqst, setup and trans next reques

                        memcpy(&fray_dump_err_data[2],&fray_variable_data,8);

                        transmitFlexray(XCP_UKL_Mot_Curr_Rqst,XCP_TX, 3);

                        break;

                    case 3:     // store XCP_Mot_Curr_Rqst, setup and trans next reques

                        memcpy(&fray_dump_err_data[3],&fray_variable_data,8);

                        transmitFlexray(XCP_UKL_Mot_Vel_Rqst,XCP_TX, 3);

                        break;

                    case 4:     // store XCP_Mot_Vel_Rqst, setup and trans next reques

                        memcpy(&fray_dump_err_data[4],&fray_variable_data,8);

                        transmitFlexray(XCP_UKL_Comm_Torque_Rqst,XCP_TX, 3);

                        break;

                    case 5:     // store XCP_Comm_Torque_Rqst, setup and trans next reques

                        memcpy(&fray_dump_err_data[5],&fray_variable_data,8);

                        transmitFlexray(XCP_UKL_Lim_Torque_Rqst,XCP_TX, 3);

                        break;

                    case 6:     // store XCP_Lim_Torque_Rqst, setup and trans next reques

                        memcpy(&fray_dump_err_data[6],&fray_variable_data,8);

                        transmitFlexray(XCP_UKL_PCB_Temp_Rqst,XCP_TX, 3);

                        break;

                    case 7:     // store XCP_PCB_Temp_Rqst, setup and trans next reques

                        memcpy(&fray_dump_err_data[7],&fray_variable_data,8);

                        transmitFlexray(XCP_UKL_Mot_Temp_Rqst,XCP_TX, 3);

                        break;

                    case 8:     // store XCP_Mot_Temp_Rqst, setup and trans next reques

                        memcpy(&fray_dump_err_data[8],&fray_variable_data,8);

                        transmitFlexray(XCP_UKL_Junction_Temp_Rqst,XCP_TX, 3);

                        break;

                    case 9:     //  store XCP_Junction_Temp_Rqst, setup and trans next reques

                        memcpy(&fray_dump_err_data[9],&fray_variable_data,8);

                        transmitFlexray(XCP_UKL_HW_Angle_Rqst,XCP_TX, 3);

                        break;

                    case 10:        // store XCP_HW_Angle_Rqst, setup and trans next reques

                        memcpy(&fray_dump_err_data[10],&fray_variable_data,8);

                        transmitFlexray(XCP_UKL_Diff_Torque_Rqst,XCP_TX, 3);

                        break;

                    case 11:        // store XCP_Diff_Torque_Rqst, setup and trans next reques

                        memcpy(&fray_dump_err_data[11],&fray_variable_data,8);

                        transmitFlexray(XCP_UKL_sys_state_Rqst,XCP_TX, 3);

                        break;

                    case 12:        // store XCP_sys_state_Rqst, setup and trans next reques

                        memcpy(&fray_dump_err_data[12],&fray_variable_data,8);

                        transmitFlexray(XCP_UKL_T1_Volt_Rqst,XCP_TX, 3);

                        break;

                    case 13:        // store XCP_T1_Volt_Rqst, setup and trans next reques

                        memcpy(&fray_dump_err_data[13],&fray_variable_data,8);

                        transmitFlexray(XCP_UKL_T2_Volt_Rqst,XCP_TX, 3);

                        break;

                    case 14:        // store XCP_T2_Volt_Rqst, done

                        memcpy(&fray_dump_err_data[14],&fray_variable_data,8);

                        break;


                    }   // end switch(can_request_index)

                    fray_request_index++;

                } //end else dumperr requests. Don't want other 2 interfering


            } //end if((intCase & 0x4000) == 0x4000){
            else if((intCase & 0x20000) == 0x20000){
                recieveFlexray(fray_rx_data,MFG_RX);
                intCase = intCase;

                switch(ECU1_XCP_reply_index)   // requesting multiple requests using same messageBox, DCT_XCP_rply_indx keeps track, clear_dtcs_flag determines action
                {
                case 0:
                    if(fray_rx_data[0][0] = 0x34 && fray_rx_data[0][1] == 0x30 && fray_rx_data[0][2] == 0xFB && fray_rx_data[0][3] == 0x01 ){
                        //Hardware Ack, ignore it!
                    }
                    ECU1_XCP_reply_index++;
                    break;

                case 1:

                    if((fray_rx_data[1][2] == 0x50) && (fray_rx_data[1][1] == 0x7E))  // positive responce from enter Nexteer mode
                    {
                        if(ECU1_clear_DTCs_flag == 1)         // clear DTC's mode
                        {
                            //canTransmit(canREG1, XCP_REQUEST_ID, ( uint8 *) xcp_write_clear_DTC_rqst1);    // request clear DTCs
                        }

                        else       // get DTC's mode

                        {
                            transmitFlexray(HW_ACK,MFG_TX, 1);

                        }

                        ECU1_XCP_reply_index++;
                    }  // end positive responce nexteer mode

                    break;

                case 2:
                    if(fray_rx_data[0][0] = 0x34 && fray_rx_data[0][1] == 0x30 && fray_rx_data[0][2] == 0xFB && fray_rx_data[0][3] == 0x01 ){
                        //Hardware Ack, ignore it!
                    }
                    ECU1_XCP_reply_index++;

                    break;

                case 3:

                    if(fray_rx_data[0][0] == 0x40)
                    {
                        //First 32 bytes of ntc data is here, parse it and store it.

                        dtc1[0][0] =  fray_rx_data[2][3];
                        dtc1[0][1] =  fray_rx_data[2][2];
                        dtc1[0][2] =  fray_rx_data[2][1];
                        dtc1[0][3] =  fray_rx_data[2][0];
                        dtc1[0][4] =  fray_rx_data[3][3];

                        dtc1[1][0] =  fray_rx_data[3][2];
                        dtc1[1][1] =  fray_rx_data[3][1];
                        dtc1[1][2] =  fray_rx_data[3][0];
                        dtc1[1][3] =  fray_rx_data[4][3];
                        dtc1[1][4] =  fray_rx_data[4][2];

                        dtc1[2][0] =  fray_rx_data[4][1];
                        dtc1[2][1] =  fray_rx_data[4][0];
                        dtc1[2][2] =  fray_rx_data[5][3];
                        dtc1[2][3] =  fray_rx_data[5][2];
                        dtc1[2][4] =  fray_rx_data[5][1];

                        dtc1[3][0] =  fray_rx_data[5][0];
                        dtc1[3][1] =  fray_rx_data[6][3];
                        dtc1[3][2] =  fray_rx_data[6][2];
                        dtc1[3][3] =  fray_rx_data[6][1];
                        dtc1[3][4] =  fray_rx_data[6][0];

                        dtc1[4][0] =  fray_rx_data[7][3];
                        dtc1[4][1] =  fray_rx_data[7][2];
                        dtc1[4][2] =  fray_rx_data[7][1];
                        dtc1[4][3] =  fray_rx_data[7][0];



                        uint8 DTC2[64][4] = {0};
                        memcpy(&DTC2[0], &HW_ACK[0],64);
                        DTC2[0][0] = 0x30; DTC2[1][3] = 0x10;
                        transmitFlexray(DTC2,MFG_TX, 2);
                    }

                    ECU1_XCP_reply_index++;

                    break;

                case 4:
                    //Second 32 bytes of data come here, parse it and store it.
                    if(fray_rx_data[0][0] == 0x21)
                    {

                        dtc1[5][0] =  fray_rx_data[1][3];

                        dtc1[5][0] =  fray_rx_data[1][2];
                        dtc1[5][1] =  fray_rx_data[1][1];
                        dtc1[5][2] =  fray_rx_data[1][0];
                        dtc1[5][3] =  fray_rx_data[2][3];
                        dtc1[5][4] =  fray_rx_data[2][2];

                        dtc1[6][0] =  fray_rx_data[2][1];
                        dtc1[6][1] =  fray_rx_data[2][0];
                        dtc1[6][2] =  fray_rx_data[3][3];
                        dtc1[6][3] =  fray_rx_data[3][2];
                        dtc1[6][4] =  fray_rx_data[3][1];

                        dtc1[7][0] =  fray_rx_data[3][0];
                        dtc1[7][1] =  fray_rx_data[4][3];
                        dtc1[7][2] =  fray_rx_data[4][2];
                        dtc1[7][3] =  fray_rx_data[4][1];
                        dtc1[7][4] =  fray_rx_data[4][0];

                        dtc1[8][0] =  fray_rx_data[5][3];
                        dtc1[8][1] =  fray_rx_data[5][2];
                        dtc1[8][2] =  fray_rx_data[5][1];
                        dtc1[8][3] =  fray_rx_data[5][0];
                        dtc1[8][4] =  fray_rx_data[6][3];

                        dtc1[9][0] =  fray_rx_data[6][2];
                        dtc1[9][1] =  fray_rx_data[6][1];
                        dtc1[9][2] =  fray_rx_data[6][0];
                        dtc1[9][3] =  fray_rx_data[7][3];
                        dtc1[9][4] =  fray_rx_data[7][2];

                        dtc1[10][0] =  fray_rx_data[7][1];
                        dtc1[10][1] =  fray_rx_data[7][0];

                    }

                    ECU1_XCP_reply_index++;

                    break;

                case 5:
                    //last 28 bytes of ntc data come here, parse and store.
                    if(fray_rx_data[0][0]  == 0x22)
                    {

                        dtc1[10][2] =  fray_rx_data[1][3];
                        dtc1[10][3] =  fray_rx_data[1][2];
                        dtc1[10][4] =  fray_rx_data[1][1];

                        dtc1[11][0] =  fray_rx_data[1][0];
                        dtc1[11][1] =  fray_rx_data[2][3];
                        dtc1[11][2] =  fray_rx_data[2][2];
                        dtc1[11][3] =  fray_rx_data[2][1];
                        dtc1[11][4] =  fray_rx_data[2][0];

                        dtc1[12][0] =  fray_rx_data[3][3];
                        dtc1[12][1] =  fray_rx_data[3][2];
                        dtc1[12][2] =  fray_rx_data[3][1];
                        dtc1[12][3] =  fray_rx_data[3][0];
                        dtc1[12][4] =  fray_rx_data[4][3];

                        dtc1[13][0] =  fray_rx_data[4][2];
                        dtc1[13][1] =  fray_rx_data[4][1];
                        dtc1[13][2] =  fray_rx_data[4][0];
                        dtc1[13][3] =  fray_rx_data[5][3];
                        dtc1[13][4] =  fray_rx_data[5][2];

                        dtc1[14][0] =  fray_rx_data[5][1];
                        dtc1[14][1] =  fray_rx_data[5][0];
                        dtc1[14][2] =  fray_rx_data[6][3];
                        dtc1[14][3] =  fray_rx_data[6][2];
                        dtc1[14][4] =  fray_rx_data[6][1];


                        transmitFlexray(HW_ACK,MFG_TX, 1);
                    }

                    ECU1_XCP_reply_index++;

                    break;

                default:

                    ECU1_XCP_reply_index = 0;

                    break;

                } // end switch on ECU1_XCP_reply_index

            } //end else if((intCase & 0x20000) == 0x20000){
            else if((intCase & 0x10000) == 0x10000){ //???????????want XCP_DYNAMIC_RX

                //XCP_DYNAMIC_RX
                handle_DAQ_notification_fray();
            }



            break;
        } //end case frayNOTIFICATION_MSG_RECEIVED:


        case frayNOTIFICATION_MSG_TRANSMITTED:
        {
            uint8 veh_speed_test[4] = {0};
            uint32 intCase = ((frayCommREG->MHDS)&0x7F0000)>>16;
            uint8 crc;

            bullshit_count++;

            if(bullshit_count >499){
                bullshit_count = 0;
            }

            crc = 0;
            if(intCase == 0xB){
                count1 = 1;
                KLEMMEN_CNT++;
                bullshit_array[bullshit_count][0] = (frayCommREG->SCV) & 0x7FF;
                bullshit_array[bullshit_count][1] = (frayCommREG->MHDS)&0x7F0000;
                if(KLEMMEN_CNT > 0xF){
                    KLEMMEN_CNT = 0;
                }
                KLEMMEN_MSG[0][2] = KLEMMEN_CNT;

                crc = BMW_CRC(19,&KLEMMEN_MSG[0],8);
                KLEMMEN_MSG[0][3] = crc;
                transmitFlexray(KLEMMEN_MSG,KLEMMEN,2);
            }
            else if(intCase == 0xC){

                count1 = 2;
                DT_PT_CNT++;
                bullshit_array[bullshit_count][0] = (frayCommREG->SCV) & 0x7FF;
                bullshit_array[bullshit_count][1] = (frayCommREG->MHDS)&0x7F0000;

                if(DT_PT_CNT > 0xF){
                    DT_PT_CNT = 0;
                }
                DT_PT_MSG[0][2] = DT_PT_CNT;

                crc = BMW_CRC(55,&DT_PT_MSG[0],16);
                DT_PT_MSG[0][3] = crc;

                transmitFlexray(DT_PT_MSG,DT_PT,2);
            }

            else if(intCase == 0xF){

                if(count1 != 3 && count1 != 0){
                    count1 = 3;
                    uint32 test = (((frayCommREG->MTCCV) & 0x3F0000)>>16)-3;
                    if((test % 4) == 0){
                        bullshit_array[bullshit_count][0] = (frayCommREG->SCV) & 0x7FF;
                        bullshit_array[bullshit_count][1] = (frayCommREG->MHDS)&0x7F0000;
                        VEH_COG_CNT = VEH_COG_MSG[0][2] + 1;



                        if(VEH_COG_CNT > 0xF){
                            VEH_COG_CNT = 0;
                        }
                        VEH_COG_MSG[0][2] = VEH_COG_CNT;
                        memcpy(&veh_speed_test[0], &VEH_COG_MSG[0][2],1);
                        memcpy(&veh_speed_test[1], &VEH_COG_MSG[0][1],1);
                        memcpy(&veh_speed_test[2], &VEH_COG_MSG[0][0],1);
                        memcpy(&veh_speed_test[3], &VEH_COG_MSG[1][3],1);

                        //crc = BMW_CRC(12,&veh_speed_test[0],4); <----THIS might be incorrect? Shouldn't CRC be the entire message?
                                crc = BMW_CRC(12,&VEH_COG_MSG[0],4);//    <----Try this instead
                                VEH_COG_MSG[0][3] = crc;

                                transmitFlexray(VEH_COG_MSG,VEH_COG,2);
                    }
                }
            }
            else {
                crc = 1;
            }

        } //end case frayNOTIFICATION_MSG_TRANSMITTED:
        default:
        {
            break;
        }
        } //end switch(notification)
    }
    break; //end case TARGET_BMW_UKL:
    }//end switch(target_product)

} //end void frayStatusChangeNotification(...)

#pragma WEAK(frayErrorNotification)
void frayErrorNotification(frayCommBASE_t *cc, uint32 notification)
{
    fray_error_notification = notification;                     //store which error occurred
    switch(notification)
    {
    case frayNOTIFICATION_BUS_HALTED: //FRAY_PEMC_INT
    {
        //frayRestartCommunication(); //moving this to main so main program can still write if error occurs, just store fray status in dumperr and daq_data
        flexray_comm_status_vector_CCSV = get_CCSV_state();
        frayDisableErrorInterrupt(); //On bus halt, disable interrupts until main() calls frayRestartCommunication
        break;
    }
    default:
    {
        break;
    }
    }

}
/* USER CODE END */
