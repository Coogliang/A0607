/** @file flexray.c
 *   @brief FlexRay Software Driver Function Implementations
 *   @date 30.May.2014
 *   @version 04.00.00
 *
 *   This file contains:
 *   - Implementations of the Software Driver Functions defined in flexray.h.
 */

#include "flexray.h"

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


extern int target_product;


//Global Flexray Stuff--------------------------------------
extern const unsigned char fray_Nexteer_session1[];
extern const unsigned char fray_Nexteer_session2[];

extern uint8 MFG_TX_nexteer_session_MSG[64][4]	;
extern uint8 Nexteer_session[64][4]				;
extern uint8 XCP_TX_request_connect_MSG[64][4]	;
extern uint8 XCP_Var_Rqst[64][4]				;

extern uint8 MFG_TX_system_state_MSG[64][4]		;
extern uint8 MFG_TX_request_software_version_MSG[64][4]	;
extern uint8 MFG_TX_request_software_rev_MSG[64][4]	;
extern uint8 MFG_TX_get_dtcs_MSG[64][4]			;
extern uint8 MFG_TX_clear_dtcs_MSG[64][4]		;
extern uint8 MFG_TX_cca_part_num_MSG[64][4]		;
extern uint8 MFG_TX_sys_serial_num_MSG[64][4]	;
extern uint8 MFG_TX_mtr_trq_cmd_MSG[64][4]	;

//Declarations of nessecary confiuration data structures.
headerSectionConfig headerSectionConfigStruct;
headerSectionConfig *currentMessageBufferSettings = &headerSectionConfigStruct;
bufferTransferConfig bufferTransferConfigStruct;
bufferTransferConfig *bufferTransferSettings = &bufferTransferConfigStruct;

static uint8 CrcTable[256] =
        {0x00, 0x1d, 0x3a, 0x27, 0x74, 0x69, 0x4e, 0x53,
         0xe8, 0xf5, 0xd2, 0xcf, 0x9c, 0x81, 0xa6, 0xbb,
         0xcd, 0xd0, 0xf7, 0xea, 0xb9, 0xa4, 0x83, 0x9e,
         0x25, 0x38, 0x1f, 0x02, 0x51, 0x4c, 0x6b, 0x76,
         0x87, 0x9a, 0xbd, 0xa0, 0xf3, 0xee, 0xc9, 0xd4,
         0x6f, 0x72, 0x55, 0x48, 0x1b, 0x06, 0x21, 0x3c,
         0x4a, 0x57, 0x70, 0x6d, 0x3e, 0x23, 0x04, 0x19,
         0xa2, 0xbf, 0x98, 0x85, 0xd6, 0xcb, 0xec, 0xf1,
         0x13, 0x0e, 0x29, 0x34, 0x67, 0x7a, 0x5d, 0x40,
         0xfb, 0xe6, 0xc1, 0xdc, 0x8f, 0x92, 0xb5, 0xa8,
         0xde, 0xc3, 0xe4, 0xf9, 0xaa, 0xb7, 0x90, 0x8d,
         0x36, 0x2b, 0x0c, 0x11, 0x42, 0x5f, 0x78, 0x65,
         0x94, 0x89, 0xae, 0xb3, 0xe0, 0xfd, 0xda, 0xc7,
         0x7c, 0x61, 0x46, 0x5b, 0x08, 0x15, 0x32, 0x2f,
         0x59, 0x44, 0x63, 0x7e, 0x2d, 0x30, 0x17, 0x0a,
         0xb1, 0xac, 0x8b, 0x96, 0xc5, 0xd8, 0xff, 0xe2,
         0x26, 0x3b, 0x1c, 0x01, 0x52, 0x4f, 0x68, 0x75,
         0xce, 0xd3, 0xf4, 0xe9, 0xba, 0xa7, 0x80, 0x9d,
         0xeb, 0xf6, 0xd1, 0xcc, 0x9f, 0x82, 0xa5, 0xb8,
         0x03, 0x1e, 0x39, 0x24, 0x77, 0x6a, 0x4d, 0x50,
         0xa1, 0xbc, 0x9b, 0x86, 0xd5, 0xc8, 0xef, 0xf2,
         0x49, 0x54, 0x73, 0x6e, 0x3d, 0x20, 0x07, 0x1a,
         0x6c, 0x71, 0x56, 0x4b, 0x18, 0x05, 0x22, 0x3f,
         0x84, 0x99, 0xbe, 0xa3, 0xf0, 0xed, 0xca, 0xd7,
         0x35, 0x28, 0x0f, 0x12, 0x41, 0x5c, 0x7b, 0x66,
         0xdd, 0xc0, 0xe7, 0xfa, 0xa9, 0xb4, 0x93, 0x8e,
         0xf8, 0xe5, 0xc2, 0xdf, 0x8c, 0x91, 0xb6, 0xab,
         0x10, 0x0d, 0x2a, 0x37, 0x64, 0x79, 0x5e, 0x43,
         0xb2, 0xaf, 0x88, 0x95, 0xc6, 0xdb, 0xfc, 0xe1,
         0x5a, 0x47, 0x60, 0x7d, 0x2e, 0x33, 0x14, 0x09,
         0x7f, 0x62, 0x45, 0x58, 0x0b, 0x16, 0x31, 0x2c,
         0x97, 0x8a, 0xad, 0xb0, 0xe3, 0xfe, 0xd9, 0xc4};

/*Flexray Error notification variables for dumps/restart bus, etc ------------------------*/
int fray_error_flag;					//used in main to restart fray comms if bus_halt error detected.
int fray_error_occurred_flag;			//set to 0 after dumperr/data sent, indicates jump to frayErrorNotif occurred
int fray_error_counter;					//counts # of times frayErrorNotif has been called, set limit on this to max uint32 counts
uint32 fray_error_notification;			//stores which notification caused frayErrorNotif
int wait_for_fray_cold_start_node_flag;	//Set to 1 in frayStartCommunication() if cold start not active (flexray_comm_status_vector_CCSV == 0x2).
										// Period check of cold start node in rtiNotif until detect it is active.
uint32 flexray_comm_status_vector_CCSV;	//Used to check cold start node active.
int flexray_normal_status_updates_flag = 0;


//BMW UKL----------------------------------------
flexrayPacket KLEMMEN_struct;
flexrayPacket *KLEMMEN = &KLEMMEN_struct;

flexrayPacket VEH_COG_struct;
flexrayPacket *VEH_COG = &VEH_COG_struct;

flexrayPacket DT_PT_struct;
flexrayPacket *DT_PT = &DT_PT_struct;

flexrayPacket AVL_struct;
flexrayPacket *AVL = &AVL_struct;

flexrayPacket XCP_TX_struct;
flexrayPacket *XCP_TX = &XCP_TX_struct;

flexrayPacket XCP_RX_struct;
flexrayPacket *XCP_RX = &XCP_RX_struct;

flexrayPacket MFG_TX_struct;
flexrayPacket *MFG_TX = &MFG_TX_struct;

flexrayPacket MFG_RX_struct;
flexrayPacket *MFG_RX = &MFG_RX_struct;

//FAAR WE----------------------------------------
flexrayPacket CON_VEH_FAAR_WE_struct;
flexrayPacket *CON_VEH_FAAR_WE = &CON_VEH_FAAR_WE_struct;

flexrayPacket ST_CENG_FAAR_WE_struct;
flexrayPacket *ST_CENG_FAAR_WE = &ST_CENG_FAAR_WE_struct;

flexrayPacket VEH_COG_FAAR_WE_struct;
flexrayPacket *VEH_COG_FAAR_WE = &VEH_COG_FAAR_WE_struct;

flexrayPacket XCP_DYNAMIC_RX_struct;
flexrayPacket *XCP_DYNAMIC_RX = &XCP_DYNAMIC_RX_struct;



//BMW UKL----------------------------------------------------------------
extern const unsigned char ccp_UKL_request_cal_Batt_Volt_str[];
extern const unsigned char ccp_UKL_request_cal_Batt_Curr_str[];
extern const unsigned char ccp_UKL_request_cal_Mot_Curr_str[];
extern const unsigned char ccp_UKL_request_cal_Mot_Vel_str[];
extern const unsigned char ccp_UKL_request_Comm_Torque_str[];
extern const unsigned char ccp_UKL_request_Lim_Torque_str[];
extern const unsigned char ccp_UKL_request_PCB_Temp_str[];
extern const unsigned char ccp_UKL_request_Mot_Temp_str[];
extern const unsigned char ccp_UKL_request_Junction_Temp_str[];
extern const unsigned char ccp_UKL_request_HW_Angle_str[];
extern const unsigned char ccp_UKL_request_Diff_Torque_str[];
extern const unsigned char ccp_UKL_request_sys_state_byt[];
extern const unsigned char ccp_UKL_request_T1_Volt_str[];
extern const unsigned char ccp_UKL_request_T2_Volt_str[];

extern const unsigned char fray_EA3_Get_DTC_rqst1[];
extern const unsigned char fray_EA3_Get_DTC_rqst2[];

extern const unsigned char BMW_UKL_Nexteer_Ack  [];


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

extern uint8 Get_UKL_DTC_Rqst[64][4];
extern uint8 HW_ACK[64][4];

extern uint8 KLEMMEN_MSG[64][4];
extern uint8 DT_PT_MSG[64][4];
extern uint8 VEH_COG_MSG[64][4];

//FAAR WE----------------------------------------------------------------

extern const unsigned char ccp_FAAR_WE_SysStMod_byt[]          ;   // 1    FE BE F3 1F     System State
extern const unsigned char ccp_FAAR_WE_BattVltg_BrdgVltg_str[] ;   // 2    FE BE EF 0C     Battery Voltage
extern const unsigned char ccp_FAAR_WE_BattRtnCurrAmpr_str[]   ;   // 3    FE BF 8C C0     Measured Battery Current
extern const unsigned char ccp_FAAR_WE_HwAgArbn_str[]          ;   // 4    FE BE EF 24     Steering wheel angle
extern const unsigned char ccp_FAAR_WE_HwTq4Meas_str[]         ;   // 5    FE BE EF C4     Torque Sensor T1
extern const unsigned char ccp_FAAR_WE_HwTq5Meas_str[]         ;   // 6    FE BE EF C8     Torque Sensor T2
extern const unsigned char ccp_FAAR_WE_HwTqArbn_str[]          ;   // 7    FE BE F0 C8     Final Input Torque
extern const unsigned char ccp_FAAR_WE_MotCurrSumA_str[]       ;   // 8    FE BE F0 34     Phase A Current
extern const unsigned char ccp_FAAR_WE_MotCurrSumB_str[]       ;   // 9    FE BE F0 38     Phase B Current
extern const unsigned char ccp_FAAR_WE_MotCurrSumC_str[]       ;   // 10   FE BE F0 3C     Phase C Current
extern const unsigned char ccp_FAAR_WE_MotVelCrf_str[]         ;   // 11   FE BE F0 5C     Rotor Speed
extern const unsigned char ccp_FAAR_WE_MotTqCmd_str[]          ;   // 12   FE BE F1 54     Requested Motor Torque
extern const unsigned char ccp_FAAR_WE_MotTqEstimd_str[]       ;   // 13   FE BE F1 50     Estimated Motor Torque
extern const unsigned char ccp_FAAR_WE_LoaSca_str[]            ;   // 14   FE BE F0 D8     Derating Vector
extern const unsigned char ccp_FAAR_WE_EcuTMeas_str[]          ;   // 15   FE BF 8B 20     Board Temp
extern const unsigned char ccp_FAAR_WE_MotWidgT_str[]          ;   // 16   FE BF 8B 54     Estimated Motor Winding Temp
extern const unsigned char ccp_FAAR_WE_MotMagT_str[]           ;   // 17   FE BF 8B 50     Estimated Magnet Temp
extern const unsigned char ccp_FAAR_WE_MotFetT_str[]           ;   // 18   FE BF 8B 4C     Estimated FET Temp

//extern const unsigned char ccp_FAAR_WE_MotTq_str[]          ;   // 19   fe be be b8     Motor Torque

extern uint8 XCP_FAAR_WE_TesterPresent_Rqst_MSG[64][4];

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

extern const unsigned char fray_EA4_Get_DTC_Rqst1[];
extern const unsigned char fray_EA4_Get_DTC_Rqst2[];

extern uint8 fray_EA4_Get_DTC_Rqst[64][4];

extern const unsigned char BMW_FAAR_WE_Nexteer_Ack[];

extern uint8 CON_VEH_FAAR_WE_MSG[64][4];
extern uint8 ST_CENG_FAAR_WE_MSG[64][4];
extern uint8 VEH_COG_FAAR_WE_MSG[64][4];

extern uint8 VEH_COG_FAAR_WE_MSG_0kph[4];
extern uint8 VEH_COG_FAAR_WE_MSG_load_kph[4];
extern uint8 VEH_COG_FAAR_WE_MSG_valid_msg[4];


//15JUL14 - Trace Hill - The following secempts to
//make the configuration of the FlexRay Communcation Controller
//and message buffers more streamlined and easily changable by a program in the future
uint32 frayCfgRegAddrs[] =
{
 0xFFF7C820u /* EIR */,
 0xFFF7C824u /* SIR */,
 0xFFF7C828u /* EILS */,
 0xFFF7C82Cu /* SILS */,
 0xFFF7C830u /* EIES */,
 0xFFF7C834u /* EIER */,
 0xFFF7C838u /* SIES */,
 0xFFF7C83Cu /* SIER */,
 0xFFF7C840u /* ILE */,
 0xFFF7C844u /* T0C */,
 0xFFF7C848u /* T1C */,
 0xFFF7C84Cu /* STPW1 */,
 0xFFF7C880u /* SUCC1 */,
 0xFFF7C884u /* SUCC2 */,
 0xFFF7C888u /* SUCC3 */,
 0xFFF7C88Cu /* NEMC */,
 0xFFF7C890u /* PRTC1 */,
 0xFFF7C894u /* PRTC2 */,
 0xFFF7C898u /* MHDC */,
 0xFFF7C8A0u /* GTUC1 */,
 0xFFF7C8A4u /* GTUC2 */,
 0xFFF7C8A8u /* GTUC3 */,
 0xFFF7C8ACu /* GTUC4 */,
 0xFFF7C8B0u /* GTUC5 */,
 0xFFF7C8B4u /* GTUC6 */,
 0xFFF7C8B8u /* GTUC7 */,
 0xFFF7C8BCu /* GTUC8 */,
 0xFFF7C8C0u /* GTUC9 */,
 0xFFF7C8C4u /* GTUC10 */,
 0xFFF7C8C8u /* GTUC11 */,
 0xFFF7CB00u /* MRC */,
 0xFFF7CB04u /* FRF */,
 0xFFF7CB08u /* FRFM */,
 0xFFF7CB10u /* MHDS */
};

uint32 frayCfgRegVals[] =
{
 0xFFFFFFFFu /* EIR */,
 0xFFFFFFFFu /* SIR */,
 0x00000000u /* EILS */,
 0x00000000u /* SILS */,
 0x00000000u /* EIES */,
 0x00000000u /* EIER */,
 0x00000010u /* SIES */,
 0x00000000u /* SIER */,
 0x00000003u /* ILE */,
 0x00000000u /* T0C */,
 0x00000000u /* T1C */,
 0x00000000u /* STPW1 */,
 /**********************************************
  * Generated By E-Ray Configuration Tool
  * Date: Fri Aug 8 11:33:09 2014
  * Configured By: Trace Hill
  * Fibex Used: E:/Fbx3_UKL_KW21_12_V5_A_FlexRay_V26_EPS_V90_lokal_Patch_20120817_18Dec13.xml
  *********************************************/

 /********************************************************************
  * System Universal Control Configuration Register 1
  * ------------------------------------------------------------------
  * Description                                    MSB LSB       Value
  * ------------------------------------------------------------------
  * Connected to Ch. B                              27  27           0
  * Connected to Ch. A                              26  26           1
  * Send MTS on Ch.B                                25  25           0
  * Send MTS on Ch. A                               24  24           0
  * Halt Due to Clock Sync Error                    23  23           0
  * Transmission (All-Slot = 1,Single-Slot = 0)     22  22           0
  * Wakeup Channel Select (A = 0, B = 1)            21  21           0
  * Passive to Active                               20  16           0
  * Cold Start Attempts                             15  11           8
  * Transmit Sync Frame in Key Slot                  9   9           1
  * Transmit Startup Frame in Key Slot               8   8           1
  * POC Busy (Read-Only)                             7   7           0
  * CMD (Non-Configurable)                           3   0           0
  *******************************************************************/
 0x04004300u,
 /***************************************************
  * System Universal Control Configuration Register 2
  * -------------------------------------------------
  * Description             MSB LSB       Value
  * -------------------------------------------------
  * Listen Timeout Noise     27  24           1
  * Listen Timeout           20   0      400242
  **************************************************/
 0x01061B72u,
 /*************************************************************************
  * System Universal Control Configuration Register 3
  * -----------------------------------------------------------------------
  * Description                                         MSB LSB       Value
  * -----------------------------------------------------------------------
  * Max Cycle Pairs without Clock Correction Fatal        7   4           2
  * Max Cycle Pairs without Clock Correction Passive      3   0           2
  ************************************************************************/
 0x00000022u,
 /*********************************************************
  * Network Managment Configuration Register
  * -------------------------------------------------------
  * Description                         MSB LSB       Value
  * -------------------------------------------------------
  * Network Management Vector Length      3   0           0
  ********************************************************/
 0x00000000u,
 /******************************************************************
  * Protocol Configuration Register 1
  * ----------------------------------------------------------------
  * Description                                  MSB LSB       Value
  * ----------------------------------------------------------------
  * Repetitions of Wakeup Pattern                 31  26          33
  * Wake-Up Symbol Receive Window                 24  16         301
  * Bus Speed (10 Mbit/s = 0, 5 = 1, 2.5 = 2)     15  14           0
  * Strobe Point Position (Always 0)              13  12           0
  * Collision Avoidance Symbol Max                10   4          79
  * Transmission Start Sequence Length             3   0           9
  *****************************************************************/
 0x852D04F9u,
 /******************************************************
  * Protocol Configuration Register 2
  * ----------------------------------------------------
  * Description                      MSB LSB       Value
  * ----------------------------------------------------
  * Wake-Up: Transmit Low Length      29  24          60
  * Wake-Up: Transmit Idle Length     23  16         180
  * Wake-Up: Receive Low Length       13   8          50
  * Wake-Up:Receive Idle Length        5   0          59
  *****************************************************/
 0x3CB4323Bu,
 /*************************************************
  * Message Handler Configuration Register
  * -----------------------------------------------
  * Description                 MSB LSB       Value
  * -----------------------------------------------
  * Latest Slot Allowed          28  16         269
  * Static Frame Data Length      6   0           8
  ************************************************/
 0x010D0008u,
 /*********************************************
  * Global Timing Unit Configuration Register 1
  * -------------------------------------------
  * Description             MSB LSB       Value
  * -------------------------------------------
  * Microticks Per Cycle     19   0      200000
  ********************************************/
 0x00030D40u,
 /*********************************************
  * Global Timing Unit Configuration Register 2
  * -------------------------------------------
  * Description            MSB LSB       Value
  * -------------------------------------------
  * Sync Node Max           19  16          15
  * Macrotick Per Cycle     13   0        3636
  ********************************************/
 0x000F0E34u,
 /*******************************************************
  * Global Timing Unit Configuration Register 3
  * -----------------------------------------------------
  * Description                       MSB LSB       Value
  * -----------------------------------------------------
  * Macrotick Initial Offset Ch. B     30  24           3
  * Macrotick Initial Offset Ch. A     22  16           3
  * Microtick Initial Offset Ch. B     15   8           6
  * Microtick Initial Offset Ch. A      7   0           6
  ******************************************************/
 0x03030606u,
 /************************************************
  * Global Timing Unit Configuration Register 4
  * ----------------------------------------------
  * Description                MSB LSB       Value
  * ----------------------------------------------
  * Offset Correction Start     29  16        3631
  * Network Idle Time           13   0        3628
  ***********************************************/
 0x0E2F0E2Cu,
 /*************************************************
  * Global Timing Unit Configuration Register 5
  * -----------------------------------------------
  * Description                 MSB LSB       Value
  * -----------------------------------------------
  * Decoding Correction          31  24          48
  * Cluster Drift Damping        20  16           2
  * Delay Compensation Ch. B     15   8           1
  * Delay Compensation Ch. A      7   0           1
  ************************************************/
 0x30020101u,
 /***********************************************
  * Global Timing Unit Configuration Register 6
  * ---------------------------------------------
  * Description               MSB LSB       Value
  * ---------------------------------------------
  * Max Oscillator Drift       26  16         121
  * Accepted Startup Range     10   0         212
  **********************************************/
 0x007900D4u,
 /***********************************************
  * Global Timing Unit Configuration Register 7
  * ---------------------------------------------
  * Description               MSB LSB       Value
  * ---------------------------------------------
  * Number of Static Slots     25  16          91
  * Static Slot Length          9   0          24
  **********************************************/
 0x005B0018u,
 /*********************************************
  * Global Timing Unit Configuration Register 8
  * -------------------------------------------
  * Description            MSB LSB       Value
  * -------------------------------------------
  * Number of Minislots     28  16         289
  * Minislot Length          5   0           5
  ********************************************/
 0x01210005u,
 /*****************************************************
  * Global Timing Unit Configuration Register 9
  * ---------------------------------------------------
  * Description                     MSB LSB       Value
  * ---------------------------------------------------
  * Dynamic Slot Idle Phase          17  16           1
  * Minislot action Point Offset     12   8           2
  * Action Point Offset               5   0           2
  ****************************************************/
 0x00010202u,
 /**********************************************
  * Global Timing Unit Configuration Register 10
  * --------------------------------------------
  * Description              MSB LSB       Value
  * --------------------------------------------
  * Max Rate Correction       26  16         121
  * Max Offset Correction     13   0         126
  *********************************************/
 0x0079007Eu,
 /***********************************************************
  * Global Timing Unit Configuration Register 11
  * ---------------------------------------------------------
  * Description                           MSB LSB       Value
  * ---------------------------------------------------------
  * External Rate Correction Value         26  24           0
  * External Offset Correction Value       18  16           0
  * External Rate Correction Control        9   8           0
  * External Offset Correction Control      1   0           0
  **********************************************************/
 0x00000000u,



 0x01800003u /* FRF */,
 0x00000000u /* FRFM */,
 0x0000007Fu /* MHDS */
};


void frayInit()
{
    //The frayInit() function initializes the registers used to
    //configure the FlexRay Communcation Controller.  The Bosch E-Ray Module
    //which is the FlexRay Communication Controller used by TI in the TMS570LS20x and 10x,
    //uses a Finite State Machine to govern the configuration, wakeup (if applicable), startup, and halting
    //of the FlexRay Communication Controller.  The programmer therefore retains the ability to command the Communcation
    //Controller through th state of the Finite State Machine by changing the Command Vector of the System Universal
    //Control 1 register (SUCC1.CMD).  The handshake for changing states is shown below:
    //	1) Wait for Protocol Operation Control (POC) to NOT be busy
    //	2) Request a state change by writing a CMD constant (see "flexray.h")
    //	   to the SUCC1.CMD vector 	.
    //	3) Wait until the Communcaiton Controller Status Vector reports that the state change has completed.


    /*< HALT any communcation that may be present >*/
    wait_on_POC();
    set_CHI_state(CMD_FREEZE); // check for command not accepted??
    while(get_CCSV_state() != CCSV_HALT); // check for or adda timeout??

    //At this point the Communcation Controller is in the HALT state.  In order to configure
    //the controller's registers, we must manually move it to the DEFAULT_CONFIG state by writing the
    //CONFIG command.
    wait_on_POC();
    set_CHI_state(CMD_CONFIG);
    while(get_CCSV_state() != CCSV_DEFAULT_CONFIG);

    //After halting all communcation and internal transfers, to be complete we will
    //clear the message RAM before doing any configuration on it.  This is also done by a hardware reset
    //however, if we aren't assuming complete reset conditions, we need to manually perform this command.
    wait_on_POC();
    set_CHI_state(CMD_CLEAR_RAMS);
    while((frayCommREG->MHDS & 0x00000080) != 0x00000000); //Wait for MHDS.CRAM flag to reset indicating operation is complete.

    //Mopve the Controller to the actual CONFIG state from the DEFAULT_CONFIG state by writing CONFIG.
    wait_on_POC();
    set_CHI_state(CMD_CONFIG);
    while(get_CCSV_state() != CCSV_CONFIG);

    //Using configuration data strucutures to configure the
    //Communcation Controller....loop through the registers.
    int ccRegIndex;
    for(ccRegIndex = 0; ccRegIndex < 34; ccRegIndex++)
    {
        *((volatile int *) (frayCfgRegAddrs[ccRegIndex])) = frayCfgRegVals[ccRegIndex];
    }

    /*
     * Message RAM Configuration.
     */


    /**< Partition the Message RAM >**/
    frayCommREG->MRC = 0x0014800A;
    //23-16: Last Configured Buffer
    //15-8: First FIFO Buffer
    //7-0: First Dynamic Buffer

    /**
     * Notes on Data Pointer (currentMessageBufferSettings->dp):
     *  - lowest configured data pointer has to be (Last Configured Buffer + 1) * 4.
     */

    /**< Initialize the Message Buffer and Buffer Transfer Settings with default values >**/
    //Write Header Section Register 1 (WRHS1)
    currentMessageBufferSettings->mbi  = 0;   // message buffer interrupt
    currentMessageBufferSettings->txm  = 0;   // transmission mode - continuous mode
    currentMessageBufferSettings->ppit = 0;   // network management Enable
    currentMessageBufferSettings->cfg  = 0;   // message buffer configuration bit (0=RX, 1 = TX)
    currentMessageBufferSettings->chb  = 0;   // Ch B
    currentMessageBufferSettings->cha  = 1;   // Ch A
    currentMessageBufferSettings->cyc  = 0;   // Cycle Filtering Code (no cycle filtering)
    currentMessageBufferSettings->fid  = 0;   // Frame I
    //Write Header Section 2 (WRHS2)
    currentMessageBufferSettings->plc   = 0;   // Payload Length
    //Write Header Section 3 (WRHS3)
    currentMessageBufferSettings->dp   = 0;   // Pointer to start of data in message RAM
    //Extra Header Information not included in WRHS registers but needed for CRC calculation
    currentMessageBufferSettings->sfi  = 0;   // startup frame indicator
    currentMessageBufferSettings->sync = 0;   // sync frame indicator


    //Initialize Buffer Transfer Settings default values
    bufferTransferSettings->ibrh = 0;  // input buffer number
    bufferTransferSettings->ibsyh = 1; // check for input buffer busy host
    bufferTransferSettings->ibsys = 1; // check for input buffer busy shadow
    bufferTransferSettings->stxrh= 0;  // set transmission request
    bufferTransferSettings->ldsh = 0;  // load data section
    bufferTransferSettings->lhsh = 0;  // load header section
    bufferTransferSettings->obrs = 0;  // output buffer number
    bufferTransferSettings->rdss = 0;  // read data section
    bufferTransferSettings->rhss = 0;  // read header section




    /**< Configure the Message Buffers >**/
    //Message Buffer 0. Send Sync & Startup on Frame 1.
    currentMessageBufferSettings->sync = 1;
    currentMessageBufferSettings->sfi = 1;
    currentMessageBufferSettings->mbi = 0;
    currentMessageBufferSettings->txm = 0;
    currentMessageBufferSettings->ppit = 0;
    currentMessageBufferSettings->cfg = 1; //0 = rx, 1 = tx
    currentMessageBufferSettings->chb = 0;
    currentMessageBufferSettings->cha = 1;
    currentMessageBufferSettings->cyc = 0;
    currentMessageBufferSettings->fid = 64;
    currentMessageBufferSettings->plc = 8;
    currentMessageBufferSettings->crc = header_crc_calc(currentMessageBufferSettings);
    currentMessageBufferSettings->dp = 0x18;
    write_header_section(currentMessageBufferSettings);
    bufferTransferSettings->lhsh = 1;
    bufferTransferSettings->ibrh = 0;
    transfer_to_RAM(bufferTransferSettings);


    //Message Buffer 1.  Receive Sync and Startup on Frame 2.
    currentMessageBufferSettings->sync = 0;
    currentMessageBufferSettings->sfi = 0;
    currentMessageBufferSettings->mbi = 0;
    currentMessageBufferSettings->txm = 0;
    currentMessageBufferSettings->ppit = 0;
    currentMessageBufferSettings->cfg = 0;
    currentMessageBufferSettings->chb = 0;
    currentMessageBufferSettings->cha = 1;
    currentMessageBufferSettings->cyc = 0;
    currentMessageBufferSettings->fid = 61;
    currentMessageBufferSettings->plc = 8;
    currentMessageBufferSettings->crc = header_crc_calc(currentMessageBufferSettings);
    currentMessageBufferSettings->dp = 0x1C;
    write_header_section(currentMessageBufferSettings);
    bufferTransferSettings->lhsh = 1;
    bufferTransferSettings->ibrh = 1;
    transfer_to_RAM(bufferTransferSettings);


    switch(target_product)
    {
		case TARGET_BMW_UKL:
		{
		    //KLEMMEN Signal.

		    KLEMMEN->buffer.ibsyh = 1; // check for input buffer busy host
		    KLEMMEN->buffer.ibsys = 1; // check for input buffer busy shadow
		    KLEMMEN->buffer.stxrh= 1;  // set transmission request
		    KLEMMEN->buffer.ldsh = 1;  // load data section
		    KLEMMEN->buffer.lhsh = 0;  // load header section
		    KLEMMEN->buffer.obrs = 0xB;  // output buffer number
		    KLEMMEN->buffer.rdss = 0;  // read data section
		    KLEMMEN->buffer.rhss = 0;  // read header section

		    KLEMMEN->header.sync = 0;
		    KLEMMEN->header.sfi = 0;
		    KLEMMEN->header.mbi = 1;
		    KLEMMEN->header.txm = 0;
		    KLEMMEN->header.ppit = 0;
		    KLEMMEN->header.cfg = 1;
		    KLEMMEN->header.chb = 0;
		    KLEMMEN->header.cha = 1;
		    KLEMMEN->header.cyc = 2;
		    KLEMMEN->header.fid = 116;
		    KLEMMEN->header.plc = 8;
		    KLEMMEN->header.crc = header_crc_calc(&KLEMMEN->header);
		    KLEMMEN->header.dp = 84;
		    write_header_section(&KLEMMEN->header);
		    KLEMMEN->buffer.lhsh = 1;
		    KLEMMEN->buffer.ibrh = 0xB;

		    //ST_DRV_VEH Signal.  D. Bair 2/17

		    DT_PT->buffer.ibsyh = 1; // check for input buffer busy host
		    DT_PT->buffer.ibsys = 1; // check for input buffer busy shadow
		    DT_PT->buffer.stxrh= 1;  // set transmission request
		    DT_PT->buffer.ldsh = 1;  // load data section
		    DT_PT->buffer.lhsh = 0;  // load header section
		    DT_PT->buffer.obrs = 0xC;  // output buffer number
		    DT_PT->buffer.rdss = 0;  // read data section
		    DT_PT->buffer.rhss = 0;  // read header section
		    DT_PT->header.sync = 0;
		    DT_PT->header.sfi = 0;
		    DT_PT->header.mbi = 1;
		    DT_PT->header.txm = 0;
		    DT_PT->header.ppit = 0;
		    DT_PT->header.cfg = 1;
		    DT_PT->header.chb = 0;
		    DT_PT->header.cha = 1;
		    DT_PT->header.cyc = 2;
		    DT_PT->header.fid = 230;
		    DT_PT->header.plc = 8;
		    DT_PT->header.crc = header_crc_calc(&DT_PT->header);
		    DT_PT->header.dp = 92;
		    write_header_section(&DT_PT->header);
		    DT_PT->buffer.lhsh = 1;
		    DT_PT->buffer.ibrh = 0xC;   //???? D. Bair


		    //try Rx frame #49  D. Bair 3/17
		    AVL->buffer.ibsyh = 1; // check for input buffer busy host
		    AVL->buffer.ibsys = 1; // check for input buffer busy shadow
		    AVL->buffer.stxrh= 1;  // set transmission request
		    AVL->buffer.ldsh = 1;  // load data section
		    AVL->buffer.lhsh = 0;  // load header section
		    AVL->buffer.obrs = 4;  // output buffer number
		    AVL->buffer.rdss = 0;  // read data section
		    AVL->buffer.rhss = 0;  // read header section
		    AVL->header.sync = 0;
		    AVL->header.sfi = 0;
		    AVL->header.mbi = 0; // Not interrupting right now
		    AVL->header.txm = 0;
		    AVL->header.ppit = 0;
		    AVL->header.cfg = 0;
		    AVL->header.chb = 0;
		    AVL->header.cha = 1;
		    AVL->header.cyc = 2;
		    AVL->header.fid = 49;
		    AVL->header.plc = 8;
		    AVL->header.crc = header_crc_calc(&AVL->header);
		    AVL->header.dp = 96;
		    write_header_section(&AVL->header);
		    AVL->buffer.lhsh = 1;
		    AVL->buffer.ibrh = 0x4;   //???? D. Bair-
		    transfer_to_RAM(&AVL->buffer);

		    //CCP Tx Signal.

		    XCP_TX->buffer.ibsyh = 1; // check for input buffer busy host
		    XCP_TX->buffer.ibsys = 1; // check for input buffer busy shadow
		    XCP_TX->buffer.stxrh= 1;  // set transmission request
		    XCP_TX->buffer.ldsh = 1;  // load data section
		    XCP_TX->buffer.lhsh = 0;  // load header section
		    XCP_TX->buffer.obrs = 0;  // output buffer number
		    XCP_TX->buffer.rdss = 0;  // read data section
		    XCP_TX->buffer.rhss = 0;  // read header section
		    XCP_TX->header.sync = 0;
		    XCP_TX->header.sfi = 0;
		    XCP_TX->header.mbi = 0;
		    XCP_TX->header.txm = 1;
		    XCP_TX->header.ppit = 0;
		    XCP_TX->header.cfg = 1;
		    XCP_TX->header.chb = 0;
		    XCP_TX->header.cha = 1;
		    XCP_TX->header.cyc = 0;
		    XCP_TX->header.fid = 219;
		    XCP_TX->header.plc = 16;
		    XCP_TX->header.crc = header_crc_calc(&XCP_TX->header);
		    XCP_TX->header.dp = 124;
		    write_header_section(&XCP_TX->header);
		    XCP_TX->buffer.lhsh = 1;
		    XCP_TX->buffer.ibrh = 0xD;
		    // transfer_to_RAM(bufferTransferSettings);

		    //try Rx frame #49  D. Bair 3/17
		    XCP_RX->buffer.ibsyh = 1; // check for input buffer busy host
		    XCP_RX->buffer.ibsys = 1; // check for input buffer busy shadow
		    XCP_RX->buffer.stxrh= 0;  // set transmission request
		    XCP_RX->buffer.ldsh = 0;  // load data section
		    XCP_RX->buffer.lhsh = 0;  // load header section
		    XCP_RX->buffer.obrs = 0xE;  // output buffer number
		    XCP_RX->buffer.rdss = 1;  // read data section
		    XCP_RX->buffer.rhss = 1;  // read header section
		    XCP_RX->header.sync = 0;
		    XCP_RX->header.sfi = 0;
		    XCP_RX->header.mbi = 1;
		    XCP_RX->header.txm = 0;
		    XCP_RX->header.ppit = 0;
		    XCP_RX->header.cfg = 0;
		    XCP_RX->header.chb = 0;
		    XCP_RX->header.cha = 1;
		    XCP_RX->header.cyc = 0;
		    XCP_RX->header.fid = 220;
		    XCP_RX->header.plc = 32;
		    XCP_RX->header.crc = header_crc_calc(&XCP_RX->header);
		    XCP_RX->header.dp = 108;
		    write_header_section(&XCP_RX->header);
		    XCP_RX->buffer.lhsh = 1;
		    XCP_RX->buffer.ibrh = 0xE;   //???? D. Bair
		    transfer_to_RAM(&XCP_RX->buffer);


		    VEH_COG->buffer.ibsyh = 1; // check for input buffer busy host
		    VEH_COG->buffer.ibsys = 1; // check for input buffer busy shadow
		    VEH_COG->buffer.stxrh= 1;  // set transmission request
		    VEH_COG->buffer.ldsh = 1;  // load data section
		    VEH_COG->buffer.lhsh = 0;  // load header section
		    VEH_COG->buffer.obrs = 0xF;  // output buffer number
		    VEH_COG->buffer.rdss = 0;  // read data section
		    VEH_COG->buffer.rhss = 0;  // read header section
		    VEH_COG->header.sync = 0;
		    VEH_COG->header.sfi = 0;
		    VEH_COG->header.mbi = 1;
		    VEH_COG->header.txm = 0;
		    VEH_COG->header.ppit = 0;
		    VEH_COG->header.cfg  = 1;
		    VEH_COG->header.chb = 0;
		    VEH_COG->header.cha = 1;
		    VEH_COG->header.cyc  = 0x07;
		    VEH_COG->header.fid = 55;
		    VEH_COG->header.plc =8;
		    VEH_COG->header.crc = header_crc_calc(&VEH_COG->header);
		    VEH_COG->header.dp = 132;
		    write_header_section(&VEH_COG->header);
		    VEH_COG->buffer.lhsh = 1;
		    VEH_COG->buffer.ibrh = 0xF;   //???? D. Bair


		    MFG_TX->buffer.ibsyh = 1; // check for input buffer busy host
		    MFG_TX->buffer.ibsys = 1; // check for input buffer busy shadow
		    MFG_TX->buffer.stxrh= 1;  // set transmission request
		    MFG_TX->buffer.ldsh = 1;  // load data section
		    MFG_TX->buffer.lhsh = 0;  // load header section
		    MFG_TX->buffer.obrs = 0;  // output buffer number
		    MFG_TX->buffer.rdss = 0;  // read data section
		    MFG_TX->buffer.rhss = 0;  // read header section
		    MFG_TX->header.sync = 0;
		    MFG_TX->header.sfi = 0;
		    MFG_TX->header.mbi = 0;
		    MFG_TX->header.txm = 1;
		    MFG_TX->header.ppit = 0;
		    MFG_TX->header.cfg = 1;
		    MFG_TX->header.chb = 0;
		    MFG_TX->header.cha = 1;
		    MFG_TX->header.cyc = 0;
		    MFG_TX->header.fid = 147;
		    MFG_TX->header.plc = 16;
		    MFG_TX->header.crc = header_crc_calc(&MFG_TX->header);
		    MFG_TX->header.dp = 136;
		    write_header_section(&MFG_TX->header);
		    MFG_TX->buffer.lhsh = 1;
		    MFG_TX->buffer.ibrh = 0x10;
		    // transfer_to_RAM(bufferTransferSettings);


		    //try Rx frame #49  D. Bair 3/17
		    MFG_RX->buffer.ibsyh = 1; // check for input buffer busy host
		    MFG_RX->buffer.ibsys = 1; // check for input buffer busy shadow
		    MFG_RX->buffer.stxrh= 1;  // set transmission request
		    MFG_RX->buffer.ldsh = 1;  // load data section
		    MFG_RX->buffer.lhsh = 0;  // load header section
		    MFG_RX->buffer.obrs = 0x11;  // output buffer number
		    MFG_RX->buffer.rdss = 1;  // read data section
		    MFG_RX->buffer.rhss = 1;  // read header section
		    MFG_RX->header.sync = 0;
		    MFG_RX->header.sfi = 0;
		    MFG_RX->header.mbi = 1;
		    MFG_RX->header.txm = 0;
		    MFG_RX->header.ppit = 0;
		    MFG_RX->header.cfg = 0;
		    MFG_RX->header.chb = 0;
		    MFG_RX->header.cha = 1;
		    MFG_RX->header.cyc = 0;
		    MFG_RX->header.fid = 183;
		    MFG_RX->header.plc = 32;
		    MFG_RX->header.crc = header_crc_calc(&MFG_RX->header);
		    MFG_RX->header.dp = 140;
		    write_header_section(&MFG_RX->header);
		    MFG_RX->buffer.lhsh = 1;
		    MFG_RX->buffer.ibrh = 0x11;   //???? D. Bair
		            transfer_to_RAM(&MFG_RX->buffer);
		}//end case TARGET_BMW_UKL:
		break;
		case TARGET_BMW_FAAR_WE:
		{
		    //KLEMMEN Signal.

		    CON_VEH_FAAR_WE->buffer.ibsyh = 1; // check for input buffer busy host
		    CON_VEH_FAAR_WE->buffer.ibsys = 1; // check for input buffer busy shadow
		    CON_VEH_FAAR_WE->buffer.stxrh= 1;  // set transmission request
		    CON_VEH_FAAR_WE->buffer.ldsh = 1;  // load data section
		    CON_VEH_FAAR_WE->buffer.lhsh = 0;  // load header section
		    CON_VEH_FAAR_WE->buffer.obrs = 0xB;  // output buffer number
		    CON_VEH_FAAR_WE->buffer.rdss = 0;  // read data section
		    CON_VEH_FAAR_WE->buffer.rhss = 0;  // read header section

		    CON_VEH_FAAR_WE->header.sync = 0;
		    CON_VEH_FAAR_WE->header.sfi = 0;
		    CON_VEH_FAAR_WE->header.mbi = 1;
		    CON_VEH_FAAR_WE->header.txm = 0;
		    CON_VEH_FAAR_WE->header.ppit = 0;
		    CON_VEH_FAAR_WE->header.cfg = 1;
		    CON_VEH_FAAR_WE->header.chb = 0;
		    CON_VEH_FAAR_WE->header.cha = 1;
		    CON_VEH_FAAR_WE->header.cyc = 3;//2;
		    CON_VEH_FAAR_WE->header.fid = 121;
		    CON_VEH_FAAR_WE->header.plc = 8; //# of words//bytres
		    CON_VEH_FAAR_WE->header.crc = header_crc_calc(&CON_VEH_FAAR_WE->header);
		    CON_VEH_FAAR_WE->header.dp = 84; //start of RAM location plc+dp < dp of next fray msg
		    write_header_section(&CON_VEH_FAAR_WE->header);
		    CON_VEH_FAAR_WE->buffer.lhsh = 1;
		    CON_VEH_FAAR_WE->buffer.ibrh = 0xB;

		    uint16 CON_VEH_FAAR_WE_crc_id = 0xC5; //from fibex file
		    //ST_DRV_VEH Signal.  D. Bair 2/17

		    ST_CENG_FAAR_WE->buffer.ibsyh = 1; // check for input buffer busy host
		    ST_CENG_FAAR_WE->buffer.ibsys = 1; // check for input buffer busy shadow
		    ST_CENG_FAAR_WE->buffer.stxrh= 1;  // set transmission request
		    ST_CENG_FAAR_WE->buffer.ldsh = 1;  // load data section
		    ST_CENG_FAAR_WE->buffer.lhsh = 0;  // load header section
		    ST_CENG_FAAR_WE->buffer.obrs = 0xC;  // output buffer number
		    ST_CENG_FAAR_WE->buffer.rdss = 0;  // read data section
		    ST_CENG_FAAR_WE->buffer.rhss = 0;  // read header section

		    ST_CENG_FAAR_WE->header.sync = 0;
		    ST_CENG_FAAR_WE->header.sfi = 0;
		    ST_CENG_FAAR_WE->header.mbi = 1;
		    ST_CENG_FAAR_WE->header.txm = 0;
		    ST_CENG_FAAR_WE->header.ppit = 0;
		    ST_CENG_FAAR_WE->header.cfg = 1;
		    ST_CENG_FAAR_WE->header.chb = 0;
		    ST_CENG_FAAR_WE->header.cha = 1;
		    ST_CENG_FAAR_WE->header.cyc = 2;
		    ST_CENG_FAAR_WE->header.fid = 117;
		    ST_CENG_FAAR_WE->header.plc = 8;
		    ST_CENG_FAAR_WE->header.crc = header_crc_calc(&ST_CENG_FAAR_WE->header);
		    ST_CENG_FAAR_WE->header.dp = 92; //start of RAM location plc+dp < dp of next fray msg
		    write_header_section(&ST_CENG_FAAR_WE->header);
		    ST_CENG_FAAR_WE->buffer.lhsh = 1;
		    ST_CENG_FAAR_WE->buffer.ibrh = 0xC;   //???? D. Bair

		    uint16 ST_CENG_FAAR_WE_crc_id = 0x78; //from fibex file

		    //try Rx frame #49  D. Bair 3/17
		    AVL->buffer.ibsyh = 1; // check for input buffer busy host
		    AVL->buffer.ibsys = 1; // check for input buffer busy shadow
		    AVL->buffer.stxrh= 1;  // set transmission request
		    AVL->buffer.ldsh = 1;  // load data section
		    AVL->buffer.lhsh = 0;  // load header section
		    AVL->buffer.obrs = 4;  // output buffer number
		    AVL->buffer.rdss = 0;  // read data section
		    AVL->buffer.rhss = 0;  // read header section
		    AVL->header.sync = 0;
		    AVL->header.sfi = 0;
		    AVL->header.mbi = 0; // Not interrupting right now
		    AVL->header.txm = 0;
		    AVL->header.ppit = 0;
		    AVL->header.cfg = 0;
		    AVL->header.chb = 0;
		    AVL->header.cha = 1;
		    AVL->header.cyc = 2;
		    AVL->header.fid = 49;
		    AVL->header.plc = 8;
		    AVL->header.crc = header_crc_calc(&AVL->header);
		    AVL->header.dp = 96; //start of RAM location plc+dp < dp of next fray msg
		    write_header_section(&AVL->header);
		    AVL->buffer.lhsh = 1;
		    AVL->buffer.ibrh = 0x4;   //???? D. Bair-
		    transfer_to_RAM(&AVL->buffer);

		    //CCP Tx Signal.

		    XCP_TX->buffer.ibsyh = 1; // check for input buffer busy host
		    XCP_TX->buffer.ibsys = 1; // check for input buffer busy shadow
		    XCP_TX->buffer.stxrh= 1;  // set transmission request
		    XCP_TX->buffer.ldsh = 1;  // load data section
		    XCP_TX->buffer.lhsh = 0;  // load header section
		    XCP_TX->buffer.obrs = 0;  // output buffer number
		    XCP_TX->buffer.rdss = 0;  // read data section
		    XCP_TX->buffer.rhss = 0;  // read header section
		    XCP_TX->header.sync = 0;
		    XCP_TX->header.sfi = 0;
		    XCP_TX->header.mbi = 0;
		    XCP_TX->header.txm = 1;
		    XCP_TX->header.ppit = 0;
		    XCP_TX->header.cfg = 1;
		    XCP_TX->header.chb = 0;
		    XCP_TX->header.cha = 1;
		    XCP_TX->header.cyc = 0;
		    XCP_TX->header.fid = 219;
		    XCP_TX->header.plc = 16;
		    XCP_TX->header.crc = header_crc_calc(&XCP_TX->header);
		    XCP_TX->header.dp = 124;
		    write_header_section(&XCP_TX->header);
		    XCP_TX->buffer.lhsh = 1;
		    XCP_TX->buffer.ibrh = 0xD;
		    // transfer_to_RAM(bufferTransferSettings);

		    //try Rx frame #49  D. Bair 3/17
		    XCP_RX->buffer.ibsyh = 1; // check for input buffer busy host
		    XCP_RX->buffer.ibsys = 1; // check for input buffer busy shadow
		    XCP_RX->buffer.stxrh= 0;  // set transmission request
		    XCP_RX->buffer.ldsh = 0;  // load data section
		    XCP_RX->buffer.lhsh = 0;  // load header section
		    XCP_RX->buffer.obrs = 0xE;  // output buffer number
		    XCP_RX->buffer.rdss = 1;  // read data section
		    XCP_RX->buffer.rhss = 1;  // read header section
		    XCP_RX->header.sync = 0;
		    XCP_RX->header.sfi = 0;
		    XCP_RX->header.mbi = 1;
		    XCP_RX->header.txm = 0;
		    XCP_RX->header.ppit = 0;
		    XCP_RX->header.cfg = 0;
		    XCP_RX->header.chb = 0;
		    XCP_RX->header.cha = 1;
		    XCP_RX->header.cyc = 0;
		    XCP_RX->header.fid = 220;
		    XCP_RX->header.plc = 32;
		    XCP_RX->header.crc = header_crc_calc(&XCP_RX->header);
		    XCP_RX->header.dp = 108;
		    write_header_section(&XCP_RX->header);
		    XCP_RX->buffer.lhsh = 1;
		    XCP_RX->buffer.ibrh = 0xE;   //???? D. Bair
		    transfer_to_RAM(&XCP_RX->buffer);

		    VEH_COG_FAAR_WE->buffer.ibsyh = 1; // check for input buffer busy host
		    VEH_COG_FAAR_WE->buffer.ibsys = 1; // check for input buffer busy shadow
		    VEH_COG_FAAR_WE->buffer.stxrh= 1;  // set transmission request
		    VEH_COG_FAAR_WE->buffer.ldsh = 1;  // load data section
		    VEH_COG_FAAR_WE->buffer.lhsh = 0;  // load header section
		    VEH_COG_FAAR_WE->buffer.obrs = 0xF;  // output buffer number
		    VEH_COG_FAAR_WE->buffer.rdss = 0;  // read data section
		    VEH_COG_FAAR_WE->buffer.rhss = 0;  // read header section
		    VEH_COG_FAAR_WE->header.sync = 0;
		    VEH_COG_FAAR_WE->header.sfi = 0;
		    VEH_COG_FAAR_WE->header.mbi = 1;
		    VEH_COG_FAAR_WE->header.txm = 0;
		    VEH_COG_FAAR_WE->header.ppit = 0;
		    VEH_COG_FAAR_WE->header.cfg  = 1;
		    VEH_COG_FAAR_WE->header.chb = 0;
		    VEH_COG_FAAR_WE->header.cha = 1;
		    VEH_COG_FAAR_WE->header.cyc  = 7;
		    VEH_COG_FAAR_WE->header.fid = 55;
		    VEH_COG_FAAR_WE->header.plc =8;
		    VEH_COG_FAAR_WE->header.crc = header_crc_calc(&VEH_COG_FAAR_WE->header);
		    VEH_COG_FAAR_WE->header.dp = 132;
		    write_header_section(&VEH_COG_FAAR_WE->header);
		    VEH_COG_FAAR_WE->buffer.lhsh = 1;
		    VEH_COG_FAAR_WE->buffer.ibrh = 0xF;   //???? D. Bair

		    uint16 VEH_COG_FAAR_WE_crc_id = 0xC; //from fibex file

		    MFG_TX->buffer.ibsyh = 1; // check for input buffer busy host
		    MFG_TX->buffer.ibsys = 1; // check for input buffer busy shadow
		    MFG_TX->buffer.stxrh= 1;  // set transmission request
		    MFG_TX->buffer.ldsh = 1;  // load data section
		    MFG_TX->buffer.lhsh = 0;  // load header section
		    MFG_TX->buffer.obrs = 0;  // output buffer number
		    MFG_TX->buffer.rdss = 0;  // read data section
		    MFG_TX->buffer.rhss = 0;  // read header section
		    MFG_TX->header.sync = 0;
		    MFG_TX->header.sfi = 0;
		    MFG_TX->header.mbi = 0;
		    MFG_TX->header.txm = 1;
		    MFG_TX->header.ppit = 0;
		    MFG_TX->header.cfg = 1;
		    MFG_TX->header.chb = 0;
		    MFG_TX->header.cha = 1;
		    MFG_TX->header.cyc = 0;
		    MFG_TX->header.fid = 147;
		    MFG_TX->header.plc = 16;
		    MFG_TX->header.crc = header_crc_calc(&MFG_TX->header);
		    MFG_TX->header.dp = 136;
		    write_header_section(&MFG_TX->header);
		    MFG_TX->buffer.lhsh = 1;
		    MFG_TX->buffer.ibrh = 0x10;
		    // transfer_to_RAM(bufferTransferSettings);


		    //try Rx frame #49  D. Bair 3/17
		    MFG_RX->buffer.ibsyh = 1; // check for input buffer busy host
		    MFG_RX->buffer.ibsys = 1; // check for input buffer busy shadow
		    MFG_RX->buffer.stxrh= 1;  // set transmission request
		    MFG_RX->buffer.ldsh = 1;  // load data section
		    MFG_RX->buffer.lhsh = 0;  // load header section
		    MFG_RX->buffer.obrs = 0x11;  // output buffer number
		    MFG_RX->buffer.rdss = 1;  // read data section
		    MFG_RX->buffer.rhss = 1;  // read header section
		    MFG_RX->header.sync = 0;
		    MFG_RX->header.sfi = 0;
		    MFG_RX->header.mbi = 1;
		    MFG_RX->header.txm = 0;
		    MFG_RX->header.ppit = 0;
		    MFG_RX->header.cfg = 0;
		    MFG_RX->header.chb = 0;
		    MFG_RX->header.cha = 1;
		    MFG_RX->header.cyc = 0;
		    MFG_RX->header.fid = 183;
		    MFG_RX->header.plc = 32;
		    MFG_RX->header.crc = header_crc_calc(&MFG_RX->header);
		    MFG_RX->header.dp = 140;
		    write_header_section(&MFG_RX->header);
		    MFG_RX->buffer.lhsh = 1;
		    MFG_RX->buffer.ibrh = 0x11;   //???? D. Bair
		    transfer_to_RAM(&MFG_RX->buffer);


			XCP_DYNAMIC_RX->buffer.ibsyh = 1; // check for input buffer busy host
			XCP_DYNAMIC_RX->buffer.ibsys = 1; // check for input buffer busy shadow
			XCP_DYNAMIC_RX->buffer.stxrh= 0;  // set transmission request
			XCP_DYNAMIC_RX->buffer.ldsh = 0;  // load data section
			XCP_DYNAMIC_RX->buffer.lhsh = 0;  // load header section
			XCP_DYNAMIC_RX->buffer.obrs = 0x12;  // output buffer number
			XCP_DYNAMIC_RX->buffer.rdss = 1;  // read data section
			XCP_DYNAMIC_RX->buffer.rhss = 1;  // read header section
			XCP_DYNAMIC_RX->header.sync = 0;
			XCP_DYNAMIC_RX->header.sfi = 0;
			XCP_DYNAMIC_RX->header.mbi = 1;
			XCP_DYNAMIC_RX->header.txm = 0;
			XCP_DYNAMIC_RX->header.ppit = 0;
			XCP_DYNAMIC_RX->header.cfg = 0;
			XCP_DYNAMIC_RX->header.chb = 0;
			XCP_DYNAMIC_RX->header.cha = 1;
			XCP_DYNAMIC_RX->header.cyc = 0;
			XCP_DYNAMIC_RX->header.fid = 294;
			XCP_DYNAMIC_RX->header.plc = 96; //NOT SURE IF SIZE NEEDS TO BE BIGGER OR SMALLER. No downsides so far, and reads max DAQ data on Faar We. - PKH 16APR18
			XCP_DYNAMIC_RX->header.crc = header_crc_calc(&XCP_DYNAMIC_RX->header);
			XCP_DYNAMIC_RX->header.dp = 172;
			write_header_section(&XCP_DYNAMIC_RX->header);
			XCP_DYNAMIC_RX->buffer.lhsh = 1;
			XCP_DYNAMIC_RX->buffer.ibrh = 0x12;   //???? D. Bair
			transfer_to_RAM(&XCP_DYNAMIC_RX->buffer);

		}//end case TARGET_BMW_FAAR_WE:
		break;
    } //end switch(target_product)

    //147 => MFG service tx
            //183 => MFG serivce RX

            //Write the lock key to exit configuration mode
            while ((frayCommREG->SUCC1 & 0x00000080) != 0);
            frayCommREG->LCK = 0x000000CE;
            frayCommREG->LCK = 0x00000031;

            // CHI command READY
            set_CHI_state(CMD_READY);
            while (get_CCSV_state() != CCSV_READY);

            //Interrupts Initialization: later this will have to be converted to read in from a data file somewhere
            //and some interrupts will need to be enabled.
            frayCommREG->SIR = 0xFFFFFFFF; //Clear any current interrupts
            frayCommREG->EIR = 0xFFFFFFFF;
            frayCommREG->EILS = 0x00000000; //Map any interrupts to CC_int0 (High priority interrupts)
            frayCommREG->SILS = 0x00000000;
            frayCommREG->EIER = 0xFFFFFFFF;
            frayCommREG->SIER = 0xFFFFFFFF;
            frayCommREG->SIES = 0x00000018; //Enable Receive Interrupt
            frayCommREG->EIES = 0x00000001; //Enable PEMC Interrupt for if the CC changes to PASSIVE or HALT
            frayCommREG->ILE = 0x00000003;  //Enable both CC_int0 and CC_int1 interrupt lines


            frayTrsfREG->GCS = 0x131;
            frayTrsfREG->TCCIES1 =0x8C00;


}

void frayDisableErrorInterrupt()
{
	//frayCommREG->EILS = 0xFFFFFFFF; //Disable PEMC (error) Interrupt for if the CC changes to PASSIVE or HALT
    frayCommREG->EIER = 0xFFFFFFFF; //clear pending error interrupts
    frayCommREG->SIER = 0xFFFFFFFF; //clear pending status interruprs
    frayCommREG->ILE = 0x00000000;  //Enable both CC_int0 and CC_int1 interrupt lines

}
void frayEnableErrorInterrupt()
{
    frayCommREG->EIER = 0xFFFFFFFF; //clear pending error interrupts
    frayCommREG->SIER = 0xFFFFFFFF; //clear pending status interruprs
	frayCommREG->ILE = 0x00000003;  //Enable both CC_int0 and CC_int1 interrupt lines
}

void frayStartCommunication()
{
    wait_on_POC();
    set_CHI_state(CMD_ALLOW_COLDSTART);

    // CHI command RUN
    wait_on_POC();
    set_CHI_state(CMD_RUN);

    //OLD method from Keegans/Traces code,
    //	Held up entire program if no fray comms. Instead, check every 10ms in notification
    //uint32 value32bit;

    //do{
    //    value32bit = get_CCSV_state();
    //}while(value32bit != 0x00000002);
    flexray_comm_status_vector_CCSV = get_CCSV_state();
    if(flexray_comm_status_vector_CCSV != 0x00000002)
    {
    	wait_for_fray_cold_start_node_flag = 1;
    }
    else
    {
    	flexray_normal_status_updates_flag = 1;
		frayCommREG->SIR = 0xFFFFFFFF; //Clear any current interrupts
		frayCommREG->EIR = 0xFFFFFFFF;
    }

}

void frayRestartCommunication()
{
    //31JUL14, Trace Hill - This message was called from notification.c if EIR.PEMC was set as an interrupt.
    //In this event, the bus stopped for some reason that at this point could be either handled by finding
    //the specific error that caused the pause, or just attempting to restart, which at this point in time is all that
    //will happen.

    //Read the state of the POC.  PEMC goes up if the controller changes from ACTIVE to PASSIVE or HALT.  if the controller is
    //in HALT, we want to attempt a restart.  NORMAL_PASSIVE is still allowable and if uncorrected will turn to HALT anyhow.
	flexray_comm_status_vector_CCSV = get_CCSV_state();
    if((flexray_comm_status_vector_CCSV == CCSV_NORMAL_PASSIVE) || (flexray_comm_status_vector_CCSV == CCSV_HALT))
    {
    	frayInit();
    	frayStartCommunication();
    }

}



//Function writes the header section of the message using a pointer to the headerSectionConfig structure.  This
//function essentially just copies values in for that.
void write_header_section(headerSectionConfig *h)
{
    int wrhs1;
    int wrhs2;
    wrhs1  = ((h->mbi) & 0x1)  <<29;
    wrhs1 |= (h->txm & 0x1)  << 28;
    wrhs1 |= (h->ppit & 0x1) << 27;
    wrhs1 |= (h->cfg & 0x1)  << 26;
    wrhs1 |= (h->chb & 0x1)  << 25;
    wrhs1 |= (h->cha & 0x1)  << 24;
    wrhs1 |= (h->cyc & 0x7F) << 16;
    wrhs1 |= (h->fid & 0x7FF);
    frayCommREG->WRHS1 = wrhs1;

    wrhs2  = ((h->plc & 0x7F) << 16) | (h->crc & 0x7FF);
    frayCommREG->WRHS2 = wrhs2;

    frayCommREG->WRHS3 = (h->dp & 0x7FF);
}


void transfer_to_RAM(bufferTransferConfig *b)
{
    // ensure nothing is pending
    while ((frayCommREG->IBCR & 0x0008000) != 0);
    frayCommREG->IBCM=((b->stxrh & 0x1) << 2) | ((b->ldsh & 0x1) << 1) | (b->lhsh & 0x1);
    frayCommREG->IBCR=(b->ibrh & 0x3F);
    // optimization possible for future by not gating like below
    // wait for completion on host registers
    while ((b->ibsyh != 0) && ((frayCommREG->IBCR & 0x00008000) != 0));
    // wait for completion on shadow registers
    while ((b->ibsys != 0) && ((frayCommREG->IBCR & 0x80000000) != 0));
}

void transfer_from_RAM(bufferTransferConfig *b)
{
    // ensure no transfer in progress on shadow registers
    while (((frayCommREG->OBCR) & 0x00008000) != 0);
    frayCommREG->OBCM=(((b->rdss & 0x1) << 1) | (b->rhss & 0x1));
    frayCommREG->OBCR=((1 << 9) | (b->obrs & 0x3F)); //req=1, view=0
    // wait for completion on shadow registers
    while (((frayCommREG->OBCR) & 0x00008000) != 0);

    frayCommREG->OBCM=(((b->rdss & 0x1) << 1) | (b->rhss & 0x1));
    frayCommREG->OBCR=((1 << 8) | (b->obrs & 0x3F)); //req=0, view=1
}


/***********************************************************************
	header_crc_calc
	This function calculates the header CRC.
 ***********************************************************************/
int header_crc_calc(headerSectionConfig *h)
{
    unsigned int header;

    int CrcInit = 0x1A;
    int length  = 20;
    int CrcNext;
    unsigned long CrcPoly  = 0x385;
    unsigned long CrcReg_X = CrcInit;
    unsigned long header_temp, reg_temp;

    //If the message was sent from a key slot AND if key slot is
    //told by SUCC1 to be startup/sync.
    header  = ((h->sync & 0x1)  << 19) | ((h->sfi & 0x1) << 18);
    header |= ((h->fid & 0x7FF) <<  7) |  (h->plc & 0x7F);

    header   <<= 11;
    CrcReg_X <<= 21;
    CrcPoly  <<= 21;

    while(length--)
    {
        header    <<= 1;
        header_temp = header & 0x80000000;
        reg_temp    = CrcReg_X & 0x80000000;

        if(header_temp ^ reg_temp)
        {  // Step 1
            CrcNext = 1;
        }
        else
        {
            CrcNext = 0;
        }

        CrcReg_X <<= 1;              // Step 2

        if(CrcNext)
        {
            CrcReg_X ^= CrcPoly;       // Step 3
        }

    }

    CrcReg_X >>= 21;

    return CrcReg_X;
}
void initFrayMsg(){

switch(target_product)
{
	case TARGET_BMW_UKL:
	{
		uint8 cpyArray[4] = {0xD0,0x00,0x00,0x00};
	    memcpy(&KLEMMEN_MSG[0], &cpyArray,4);

	    uint8 cpyArray1[4] = {0x00, 0x82, 0x00, 0x00};
	    memcpy(&DT_PT_MSG[0], &cpyArray1,4);

	    uint8 cpyArray2[4] = {0x32, 0x00, 0x00, 0x00};
	    uint8 cpyArray2_1[4] = {0x00, 0x00, 0x00, 0x01};

	    //memset(&VEH_COG_MSG,0xFF,64*4);
	    memcpy(&VEH_COG_MSG[0], &cpyArray2, 4);
	    memcpy(&VEH_COG_MSG[1], &cpyArray2_1, 4);

	    uint8 cpyArray3[4] = {0x00, 0xFB, 0x30, 0x01};
	    memcpy(&MFG_TX_nexteer_session_MSG[0], &cpyArray3, 4);
	    uint8 cpyArray4[4] = {0xFF, 0x7E,0x10, 0x02};
	    memcpy(&MFG_TX_nexteer_session_MSG[1], &cpyArray4, 4);

	    uint8 cpyArray5[4] = {0x02, 0x00, 0xEE, 0x30};
	    memcpy(&XCP_TX_request_connect_MSG[0], &cpyArray5, 4);
	    uint8 cpyArray6[4] = {0x00, 0x00,0x00, 0xFF};
	    memcpy(&XCP_TX_request_connect_MSG[1], &cpyArray6, 4);

	    uint8 cpyArray7[4] = {0x08, 0x00, 0xEE,0x30};
	    uint8 cpyArray8[4] = {0x00, 0x00, 0x04,0xF4};
	    memcpy(&XCP_UKL_Batt_Volt_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_UKL_Batt_Volt_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_UKL_Batt_Volt_Rqst[2], &ccp_UKL_request_cal_Batt_Volt_str, 4);

	    memcpy(&XCP_UKL_Batt_Curr_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_UKL_Batt_Curr_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_UKL_Batt_Curr_Rqst[2], &ccp_UKL_request_cal_Batt_Curr_str, 4);

	    memcpy(&XCP_UKL_Mot_Curr_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_UKL_Mot_Curr_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_UKL_Mot_Curr_Rqst[2], &ccp_UKL_request_cal_Mot_Curr_str, 4);

	    memcpy(&XCP_UKL_Mot_Vel_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_UKL_Mot_Vel_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_UKL_Mot_Vel_Rqst[2], &ccp_UKL_request_cal_Mot_Vel_str, 4);

	    memcpy(&XCP_UKL_Comm_Torque_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_UKL_Comm_Torque_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_UKL_Comm_Torque_Rqst[2], &ccp_UKL_request_Comm_Torque_str, 4);

	    memcpy(&XCP_UKL_Lim_Torque_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_UKL_Lim_Torque_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_UKL_Lim_Torque_Rqst[2], &ccp_UKL_request_Lim_Torque_str, 4);

	    memcpy(&XCP_UKL_PCB_Temp_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_UKL_PCB_Temp_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_UKL_PCB_Temp_Rqst[2], &ccp_UKL_request_PCB_Temp_str, 4);

	    memcpy(&XCP_UKL_Mot_Temp_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_UKL_Mot_Temp_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_UKL_Mot_Temp_Rqst[2], &ccp_UKL_request_Mot_Temp_str, 4);

	    memcpy(&XCP_UKL_Junction_Temp_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_UKL_Junction_Temp_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_UKL_Junction_Temp_Rqst[2], &ccp_UKL_request_Junction_Temp_str, 4);

	    memcpy(&XCP_UKL_HW_Angle_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_UKL_HW_Angle_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_UKL_HW_Angle_Rqst[2], &ccp_UKL_request_HW_Angle_str, 4);

	    memcpy(&XCP_UKL_Diff_Torque_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_UKL_Diff_Torque_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_UKL_Diff_Torque_Rqst[2], &ccp_UKL_request_Diff_Torque_str, 4);

	    memcpy(&XCP_UKL_sys_state_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_UKL_sys_state_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_UKL_sys_state_Rqst[2], &ccp_UKL_request_sys_state_byt, 4);

	    memcpy(&XCP_UKL_T1_Volt_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_UKL_T1_Volt_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_UKL_T1_Volt_Rqst[2], &ccp_UKL_request_T1_Volt_str, 4);

	    memcpy(&XCP_UKL_T2_Volt_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_UKL_T2_Volt_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_UKL_T2_Volt_Rqst[2], &ccp_UKL_request_T2_Volt_str, 4);

	    memcpy(&Nexteer_session[0], &fray_Nexteer_session1, 4);
	    memcpy(&Nexteer_session[1], &fray_Nexteer_session2, 4);

	    memcpy(&HW_ACK[0], &BMW_UKL_Nexteer_Ack, 4);

	    memcpy(&Get_UKL_DTC_Rqst[0], &fray_EA3_Get_DTC_rqst1, 4);
	    memcpy(&Get_UKL_DTC_Rqst[1], &fray_EA3_Get_DTC_rqst2, 4);
	} //end case TARGET_BMW_UKL:
	break;


	case TARGET_BMW_FAAR_WE:
	{

		//----------Build flexray msgs for periodics -----------------------------------
		/*CON_VEH_FAAR_WE_MSG format
		 *	Testing...
		 *	Input =
		 *		uint8 cpyArray[4] = {0x12,0x34,0x56,0x78};
	     *		memcpy(&CON_VEH_FAAR_WE_MSG[1], &cpyArray,4);
	     *
	     *	Output =
	     *
	     *		28 0E 00 00 78 56 34 12 00 00 00 00 00 00 00 00
		 *	0			1
		 *Load as:		3		2		1		0		3		2		1		0
		 *Bus shows:	0		1		2		3		4		5		6		7
		 *				4	0	12 	8	20	16	28	24	36	32	44	40	52	48	60	56
		 *Definition:	C 	C	CTR	A	FK	FK	FK	FK	FK	FK	0	FK	0	ST	0	0
		 *RUN Def:		X	X	A	X	0	0	0	0	0	0	0	0	0	A	0	0
		 *OFF Def:		X	X	C	X	0	0	0	0	0	0	0	0	0	C	0	0
		 *
		 * */
		uint8 cpyArray[4] = {0x00,0x00,0xA0,0x00};
		uint8 cpyArray_1[4] = {0xFF,0x0A,0x00,0x00};
	    memcpy(&CON_VEH_FAAR_WE_MSG[0], &cpyArray,4);
	    memcpy(&CON_VEH_FAAR_WE_MSG[1], &cpyArray_1,4);

	    /*ST_CENG_MSG format
		 *	Testing...
		 *	Input =
	     * uint8 cpyArray1[4] = {0x00, 0x82, 0x00, 0x00};
	     * memcpy(&ST_CENG_FAAR_WE_MSG[0], &cpyArray1,4);
	     *
	     *	Output =
	     *		08 01 82 00 00 00 00 00 00 00 00 00 00 00 00 00
		 *	0			1
		 *Load as:		3		2		1		0		3				2		1		0
		 *Bus shows:	0		1		2		3		4				5		6		7
		 *				4	0	12 	8	20	16	28	24	36	34  32		44	40	52	48	60	56
		 *Definition:	C 	C	0	A	UDP	DRV	E_S	E_S	0	STOP+MOT	0	0	0	0	0	0
		 *RUN Def:		C	C	F	A	0	2	0	0	0	!00 + !01	0	0	0	0	0	0
		 *OFF Def:		C	C	F	A	0	0	0	0	0	!01 + !00	0	0	0	0	0	0
		 *													x01 or x04
		 *	ST_UDP (UDP above) Needs to change during voltage tests
		 *	ST_ENERG_SUPY (E_S above) needs to change during voltage tests
		 *	ST_MOT_DRV (MOT above) = 0x1 when running, 0x0 when off
		 *	ST_AN0_MSA_ENG_STOP (STOP above) = 0x0 when running, 0x01 when off
		 * */
	    uint8 cpyArray1[4] = {0x00, 0x02, 0xF0, 0x00};
	    uint8 cpyArray1_1[4] = {0xFF, 0xFF, 0xFC, 0x01};
	    memcpy(&ST_CENG_FAAR_WE_MSG[0], &cpyArray1,4);
	    memcpy(&ST_CENG_FAAR_WE_MSG[1], &cpyArray1_1,4);

	    /*VEH_COG_FAAR_WE_MSG format
		 *	Testing...
		 *	Input =
	     * uint8 cpyArray2[4] = {0x32, 0x00, 0x00, 0x00};
	     * uint8 cpyArray2_1[4] = {0x00, 0x00, 0x00, 0x01};
	     *
	     *	Output =
	     *		0F 9A 00 32 01 00 00 00 00 00 00 00
		 *	0			1
		 *Load as:		3		2		1		0		3		2		1		0
		 *Bus shows:	0		1		2		3		4		5		6		7
		 *				4	0	12 	8	20	16	28	24	36	32	44	40	52	48	60	56
		 *Definition:	C 	C	NSS	A	CG4	CG3	CG2 CG1	0	QU	0	0	0	0	0	0
		 *0 KPH Def:	C	C	D/C	A	0	0	0	0	0	1	0	0	0	0	0	0
		 *100 KPH def:	X	X	D	X	0	0	6	4	0	1	0	0	0	0	0	0
		 *OFF Def:		X	X	0	X	0	0	0	0	0	0	0	0	0	0	0
		 *	X = crc/counters
		 *	ST_V_VEH_NSS (NSS above) = 0xD for not at stand still, = 0xC for near stand still
		 *	V_VEH_COG (CG1-CG4 above) = Vehicle speed = 16 bits, 0x0000 0xFFFF
		 *	QU_V_VEH_COG (QU above) = 0x01 is valid/plausible
		 * */
		memcpy(&VEH_COG_FAAR_WE_MSG_load_kph, &VEH_COG_FAAR_WE_MSG_0kph, 4); //put 0kph message into load KPH.
																			 //can just write to ...MSG_load_kph[0] and ...MSG_load_kph[1] instead.
	    memcpy(&VEH_COG_FAAR_WE_MSG[0], &VEH_COG_FAAR_WE_MSG_load_kph, 4);
	    memcpy(&VEH_COG_FAAR_WE_MSG[1], &VEH_COG_FAAR_WE_MSG_valid_msg, 4);


	    //----------Build flexray msgs for Manufacturing services, connect, start session, etc requests-----------------------------------

	    int i;

		//0x3E01 XCP_FAAR_WE_TesterPresent_Rqst_MSG
	    uint8 XCP_FAAR_WE_TesterPresent_Rqst_Build[3][4] ={{0xF4, 0x00, 0x30, 0x00},
	                                                   {0x02, 0x00, 0x02, 0x40},
	                                                   {0x00, 0x00, 0x01, 0x3E}};
	    //00 30 00 F4 40 02 00 02 3E 01
	    for(i=0;i<=3;i++){memcpy(&XCP_FAAR_WE_TesterPresent_Rqst_MSG[i], &XCP_FAAR_WE_TesterPresent_Rqst_Build[i], 4);}

	    //0x107E MFG_TX_connect_MSG 			-> connect for MFG service mode		//00 30 00 F4 40 02 00 02 10 7E
	    uint8 MFG_TX_nexteer_session_MSG_build[3][4] = {{0xF4, 0x00, 0x30, 0x00},
														{0x02, 0x00, 0x02, 0x40},
														{0x00, 0x00, 0x7E, 0x10}};
	    for(i=0;i<=3;i++){memcpy(&MFG_TX_nexteer_session_MSG[i], &MFG_TX_nexteer_session_MSG_build[i], 4);}

	    //0xF194 MFG_TX_software_version_MSG 	-> Get product software MFG service //00 30 00 F4 40 03 00 03 22 F1 94 00
	    uint8 MFG_TX_request_software_version_MSG_build[3][4] = {	{0xF4, 0x00, 0x30, 0x00},
	    															{0x03, 0x00, 0x03, 0x40},
																	{0x00, 0x94, 0xF1, 0x22}};
	    for(i=0;i<=3;i++){memcpy(&MFG_TX_request_software_version_MSG[i], &MFG_TX_request_software_version_MSG_build[i], 4);}

	    //0xF195 MFG_TX_software_version_MSG 	-> Get product software MFG service //00 30 00 F4 40 03 00 03 22 F1 95 00
	    uint8 MFG_TX_request_software_rev_MSG_build[3][4] = {	{0xF4, 0x00, 0x30, 0x00},
	    														{0x03, 0x00, 0x03, 0x40},
																{0x00, 0x95, 0xF1, 0x22}};
	    for(i=0;i<=3;i++){memcpy(&MFG_TX_request_software_rev_MSG[i], &MFG_TX_request_software_rev_MSG_build[i], 4);}

	    //0xFD00 MFG_TX_system_state_MSG 		-> Get system state MFG service 	//00 30 00 F4 40 03 00 03 22 FD 00 00
	    uint8 MFG_TX_system_state_MSG_build[3][4] = {	{0xF4, 0x00, 0x30, 0x00},
	    												{0x03, 0x00, 0x03, 0x40},
														{0x00, 0x00, 0xFD, 0x22}};
	    for(i=0;i<=3;i++){memcpy(&MFG_TX_system_state_MSG[i], &MFG_TX_system_state_MSG_build[i], 4);}

	    //0xFD60 MFG_TX_get_dtcs_MSG 			-> Get trouble codes MFG service	//00 30 00 F4 40 03 00 03 22 FD 60 00
	    uint8 MFG_TX_get_dtcs_MSG_build[3][4] = {	{0xF4, 0x00, 0x30, 0x00},
	    											{0x03, 0x00, 0x03, 0x40},
													{0x00, 0x60, 0xFD, 0x22}};
	    for(i=0;i<=3;i++){memcpy(&MFG_TX_get_dtcs_MSG[i], &MFG_TX_get_dtcs_MSG_build[i], 4);}

	    //0xFD70 MFG_TX_clear_dtcs_MSG 			-> Clear trouble codes MFG service	//00 30 00 F4 40 04 00 04 31 01 FD 70
	    uint8 MFG_TX_clear_dtcs_MSG_build[3][4] = {	{0xF4, 0x00, 0x30, 0x00},
	    											{0x04, 0x00, 0x04, 0x40},
													{0x70, 0xFD, 0x01, 0x31}};
	    for(i=0;i<=3;i++){memcpy(&MFG_TX_clear_dtcs_MSG[i], &MFG_TX_clear_dtcs_MSG_build[i], 4);}

	    //0xFD31 MFG_TX_sys_serial_num_MSG 	-> Get system serial number				//00 30 00 F4 40 03 00 03 22 FD 31 00
	    uint8 MFG_TX_sys_serial_num_MSG_build[3][4] = {	{0xF4, 0x00, 0x30, 0x00},
	    												{0x03, 0x00, 0x03, 0x40},
														{0x00, 0x31, 0xFD, 0x22}};
	    for(i=0;i<=3;i++){memcpy(&MFG_TX_sys_serial_num_MSG[i], &MFG_TX_sys_serial_num_MSG_build[i], 4);}

	    //0xFD32 MFG_TX_cca_part_num_MSG 		-> Get cca hardware part number		//00 30 00 F4 40 03 00 03 22 FD 30 00
	    uint8 MFG_TX_cca_part_num_MSG_build[3][4] = {	{0xF4, 0x00, 0x30, 0x00},
	    												{0x03, 0x00, 0x03, 0x40},
														{0x00, 0x32, 0xFD, 0x22}};
	    for(i=0;i<=3;i++){memcpy(&MFG_TX_cca_part_num_MSG[i], &MFG_TX_cca_part_num_MSG_build[i], 4);}

	    //0xFD40 MFG_TX_mtr_trq_cmd_MSG 		-> Command motor torque		//00 30 00 F4 40 05 00 05 22 FD 40 HH LL
	    uint8 MFG_TX_mtr_trq_cmd_MSG_build[4][4] = {	{0xF4, 0x00, 0x30, 0x00},
	    												{0x08, 0x00, 0x08, 0x40},
														{0x03, 0x40, 0xFD, 0x2F},
														{0x00, 0x00, 0x00, 0x00}};//3F 00 00 00 = 0.5Nm, so want 00 00 00 3F
	    for(i=0;i<=4;i++){memcpy(&MFG_TX_mtr_trq_cmd_MSG[i], &MFG_TX_mtr_trq_cmd_MSG_build[i], 4);}



	    uint8 XCP_TX_request_connect_MSG_build[2][4] = {{0x01, 0x00, 0x00, 0x30},
	    												{0x00, 0x00,0x00, 0xFF}};
	    for(i=0;i<=2;i++){memcpy(&XCP_TX_request_connect_MSG[i], &XCP_TX_request_connect_MSG_build[i], 4);}

	    //memcpy(&Nexteer_session[0], &fray_Nexteer_session1, 4);
	    //memcpy(&Nexteer_session[1], &fray_Nexteer_session2, 4);

	    uint8 XCP_TX_header[4] = {0xF4, 0x00, 0x30, 0x00};
	    memcpy(&HW_ACK[0], &XCP_TX_header, 4);
	    memcpy(&HW_ACK[1], &BMW_FAAR_WE_Nexteer_Ack, 4);

	    memcpy(&fray_EA4_Get_DTC_Rqst[0], &fray_EA4_Get_DTC_Rqst1, 4);
	    memcpy(&fray_EA4_Get_DTC_Rqst[1], &fray_EA4_Get_DTC_Rqst2, 4);

	    //----------Build flexray msgs for XCP requests-----------------------------------
	    uint8 cpyArray7[4] = {0x08, 0x00, 0xEE,0x30};
	    uint8 cpyArray8[4] = {0x00, 0x00, 0x04,0xF4};

	    memcpy(&XCP_FAAR_WE_SysStMod_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_FAAR_WE_SysStMod_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_FAAR_WE_SysStMod_Rqst[2], &ccp_FAAR_WE_SysStMod_byt, 4);

	    memcpy(&XCP_FAAR_WE_BattVltg_BrdgVltg_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_FAAR_WE_BattVltg_BrdgVltg_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_FAAR_WE_BattVltg_BrdgVltg_Rqst[2], &ccp_FAAR_WE_BattVltg_BrdgVltg_str, 4);

	    memcpy(&XCP_FAAR_WE_BattRtnCurrAmpr_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_FAAR_WE_BattRtnCurrAmpr_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_FAAR_WE_BattRtnCurrAmpr_Rqst[2], &ccp_FAAR_WE_BattRtnCurrAmpr_str, 4);

	    memcpy(&XCP_FAAR_WE_HwAgArbn_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_FAAR_WE_HwAgArbn_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_FAAR_WE_HwAgArbn_Rqst[2], &ccp_FAAR_WE_HwAgArbn_str, 4);

	    memcpy(&XCP_FAAR_WE_HwTq4Meas_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_FAAR_WE_HwTq4Meas_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_FAAR_WE_HwTq4Meas_Rqst[2], &ccp_FAAR_WE_HwTq4Meas_str, 4);

	    memcpy(&XCP_FAAR_WE_HwTq5Meas_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_FAAR_WE_HwTq5Meas_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_FAAR_WE_HwTq5Meas_Rqst[2], &ccp_FAAR_WE_HwTq5Meas_str, 4);

	    memcpy(&XCP_FAAR_WE_HwTqArbn_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_FAAR_WE_HwTqArbn_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_FAAR_WE_HwTqArbn_Rqst[2], &ccp_FAAR_WE_HwTqArbn_str, 4);

	    memcpy(&XCP_FAAR_WE_MotCurrSumA_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_FAAR_WE_MotCurrSumA_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_FAAR_WE_MotCurrSumA_Rqst[2], &ccp_FAAR_WE_MotCurrSumA_str, 4);

	    memcpy(&XCP_FAAR_WE_MotCurrSumB_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_FAAR_WE_MotCurrSumB_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_FAAR_WE_MotCurrSumB_Rqst[2], &ccp_FAAR_WE_MotCurrSumB_str, 4);

	    memcpy(&XCP_FAAR_WE_MotCurrSumC_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_FAAR_WE_MotCurrSumC_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_FAAR_WE_MotCurrSumC_Rqst[2], &ccp_FAAR_WE_MotCurrSumC_str, 4);

	    memcpy(&XCP_FAAR_WE_MotVelCrf_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_FAAR_WE_MotVelCrf_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_FAAR_WE_MotVelCrf_Rqst[2], &ccp_FAAR_WE_MotVelCrf_str, 4);

	    memcpy(&XCP_FAAR_WE_MotTqCmd_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_FAAR_WE_MotTqCmd_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_FAAR_WE_MotTqCmd_Rqst[2], &ccp_FAAR_WE_MotTqCmd_str, 4);

	    memcpy(&XCP_FAAR_WE_MotTqEstimd_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_FAAR_WE_MotTqEstimd_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_FAAR_WE_MotTqEstimd_Rqst[2], &ccp_FAAR_WE_MotTqEstimd_str, 4);

	    memcpy(&XCP_FAAR_WE_LoaSca_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_FAAR_WE_LoaSca_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_FAAR_WE_LoaSca_Rqst[2], &ccp_FAAR_WE_LoaSca_str, 4);

	    memcpy(&XCP_FAAR_WE_EcuTMeas_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_FAAR_WE_EcuTMeas_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_FAAR_WE_EcuTMeas_Rqst[2], &ccp_FAAR_WE_EcuTMeas_str, 4);

	    memcpy(&XCP_FAAR_WE_MotWidgT_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_FAAR_WE_MotWidgT_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_FAAR_WE_MotWidgT_Rqst[2], &ccp_FAAR_WE_MotWidgT_str, 4);

	    memcpy(&XCP_FAAR_WE_MotMagT_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_FAAR_WE_MotMagT_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_FAAR_WE_MotMagT_Rqst[2], &ccp_FAAR_WE_MotMagT_str, 4);

	    memcpy(&XCP_FAAR_WE_MotFetT_Rqst[0], &cpyArray7, 4);
	    memcpy(&XCP_FAAR_WE_MotFetT_Rqst[1], &cpyArray8, 4);
	    memcpy(&XCP_FAAR_WE_MotFetT_Rqst[2], &ccp_FAAR_WE_MotFetT_str, 4);

//	    memcpy(&XCP_FAAR_WE_MotTq_Rqst[0], &cpyArray7, 4);
//	    memcpy(&XCP_FAAR_WE_MotTq_Rqst[1], &cpyArray8, 4);
//	    memcpy(&XCP_FAAR_WE_MotTq_Rqst[2], &ccp_FAAR_WE_MotTq_str, 4);

	} //end case TARGET_BMW_UKL:
break;

} //end switch(target_product)




}
void recieveFlexray(uint8 data[64][4],flexrayPacket *pckt)
{
    //frayCommREG->WRDS[0] = data>>32;
    //frayCommREG->WRDS[1] = data;
    int i = 0;

    write_header_section(&pckt->header);
    transfer_from_RAM(&pckt->buffer);

    for(i = 0; i < 64; i++){
        memcpy(&data[i],&frayCommREG->RDDS[i], 4);
    }


}

void transmitFlexray(uint8 data[64][4], flexrayPacket *pckt, int words)
{
    //frayCommREG->WRDS[0] = data>>32;
    //frayCommREG->WRDS[1] = data;

	extern uint8 XCP_output_counter;
	switch(target_product)
	{
	case TARGET_BMW_FAAR_WE:
		if(pckt->header.fid == 219)
		{
			data[0][2] = XCP_output_counter;
		}
	break;
	}


    int i = 0;
    memset(&frayCommREG->WRDS[0],0,64);
    //memcpy(&data[0], 0, )
    for(i = 0; i < words; i++){
        //        byteFlip[i][0] = data[i][3];
        //        byteFlip[i][1] = data[i][2];
        //        byteFlip[i][2] = data[i][1];
        //        byteFlip[i][3] = data[i][0];

        memcpy(&frayCommREG->WRDS[i], &data[i], 4);
        //frayCommREG->WRDS[i] = data[i];
    }
    write_header_section(&pckt->header);
    transfer_to_RAM(&pckt->buffer);

	switch(target_product)
	{
	case TARGET_BMW_FAAR_WE:
		if(pckt->header.fid == 219)
		{
			XCP_output_counter++;
		}
	break;
	}
}

uint8 BMW_CRC(uint16 crc_id, uint8  MsgData[4], uint8 MsgLength){
    uint8 CheckSum;
    uint16 kk = 0;
   // MsgData[0][3] = 0x00;
    /* Calculate CRC of low byte of ApplicationID */
    CheckSum = CrcTable[(uint8)(crc_id & 0x00FF)];

    /*Calculate CRC of high byte of ApplicationId */
    CheckSum = CrcTable[CheckSum ^  (uint8)(crc_id >>8)];

    /* 8Bit CRC */
    for(kk = 0; kk <MsgLength; kk++){

        CheckSum = CrcTable[CheckSum ^ MsgData[kk]];

    }
    return(CheckSum);
}

#pragma INTERRUPT(frayHighLevelInterrupt,IRQ)

void frayHighLevelInterrupt(void)
{
    //Read the status of the two registers to fiure out which
    //specific error/status was generated.
    //Because VIM RAM only allocates 1 frayHigh Level Interrupt,
    //we have to handle both possible error and status interrupts in
    //this function.

    //First, check to see if any error messages are what caused the interrupt
    uint32 EIR = frayCommREG->EIR;
    if((EIR & FRAY_PEMC_INT)==FRAY_PEMC_INT)
    {
        //An error did occur that needs to be attended to if it is what caused the interrupt
        //27JUN14 - No error interrupts are explicitly handled.
        frayErrorNotification(frayCommREG,FRAY_PEMC_INT);

    }

    //Now check the status interrupts by prioritizing the receive function first
    uint32 SIR = frayCommREG->SIR;
    if((SIR & FRAY_RXI_INT) == FRAY_RXI_INT)
    {
        //A frame was received, so pass to notification.c
        frayStatusChangeNotification(frayCommREG,FRAY_RXI_INT);
        frayCommREG->SIR = 0xFFFFFFFF;
        return;
    }

    if((SIR & FRAY_TXI_INT) == FRAY_TXI_INT)
    {
        //A frame was received, so pass to notification.c
        frayCommREG->SIR = 0xFFFFFFFF;
        frayStatusChangeNotification(frayCommREG,FRAY_TXI_INT);
        frayCommREG->MHDS = 0;
        return;
    }


}

#pragma INTERRUPT(frayTUInterrupt,IRQ)

void frayTUInterrupt(void){
    frayTrsfREG->GCS = 0x31;

}
