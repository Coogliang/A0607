/*
 * NTC_get.c
 *
 *  Created on: Feb 14, 2018
 *      Author: TZZ176
 */


#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <limits.h>


#include "NTC_get.h"
#include "can.h"
#include "flexray.h"


int NTC_get_period = 0;//20000;
int NTC_get_flag = 0;
int NTC_main_get_flag = 0;
int NTC_get_print_flag = 0;

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

// C A N 2  --  G M  Products none are ADAS yet

#define CCP_CAN2_RESPONSE_ID  canMESSAGE_BOX3    // 0x642   11bit identifier-- Rec XCP Responce to EPS for data
#define CCP_CAN2_REQUEST_ID   canMESSAGE_BOX4    // 0x242   11bit identifier-- Trans XCP Request from EPS for data
#define PERIOD_CAN2_ID        canMESSAGE_BOX5    // 0x182   11bit identifier-- Trans periodic to EPS


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

extern const unsigned char xcp_write_session_Nexteer_mode[];


//--------BMW Flexray generic------------
extern flexrayPacket MFG_TX_struct;
extern flexrayPacket *MFG_TX;

extern flexrayPacket MFG_RX_struct;
extern flexrayPacket *MFG_RX;

//--------BMW UKL------------------------
extern uint8 Get_UKL_DTC_Rqst[64][4];

//--------BMW Faar We--------------------
extern uint8 Nexteer_session[64][4];
extern uint8 MFG_TX_nexteer_session_MSG[64][4];

//--------NTC_get.h Dependants-----------
extern int ECU1_dtc_loop_cnt;
extern int ECU1_dtc_inner_loop_cnt;
extern int tester_present_timer;
extern int target_product;
extern unsigned long system_msec_clock;
extern unsigned long system_msec_clock_temp;
extern char temp_return_message[300];
extern int ECU1_XCP_reply_index;
extern int ECU1_clear_DTCs_flag;


//--------New VARS for new function------
unsigned char dtc1[21][5];
char dtc1_bkgd_return_message[257];
int MAX_ECU1_XCP_reply_index_bkgd = 0;


/*******************************************************************
 * name: get_DTCs1_background
 * purpose: Collect NTCs without wait statements
 * 			Kick off NTC manufact service then return to main()
 * 			main() will check for ECU1_XCP_reply_index = MAX_ECU1_XCP_reply_index before calling build_dtc1_bkdg_return_message()
 *
 *******************************************************************/
void get_DTCs1_background(void)
{

	// set all DTCs to -1 in case service fails
	// when called set timeout or  DTC_XCP_rply_indx = end

    for(ECU1_dtc_loop_cnt = 0;ECU1_dtc_loop_cnt < 21;ECU1_dtc_loop_cnt++)
	 {
	   for(ECU1_dtc_inner_loop_cnt = 0;ECU1_dtc_inner_loop_cnt < 5;ECU1_dtc_inner_loop_cnt++)
	    {
		   dtc1[ECU1_dtc_loop_cnt][ECU1_dtc_inner_loop_cnt] =	0xFF;
		}
	 }

    tester_present_timer = 3000;  // reset timer for 3 seconds to ensure TOC messages not stepped on


    ECU1_XCP_reply_index = 1;
    ECU1_clear_DTCs_flag = 0; // make sure it is get DTC's  NOT clear

	switch (target_product)
	{
	case TARGET_T1XX:		//EA4 is different
		MAX_ECU1_XCP_reply_index_bkgd = 14;
		canTransmit(canREG1, XCP_EA4_REQUEST_ID, ( uint8 *) xcp_write_session_Nexteer_mode);	// open Nexteer session prior to XCP
	break;

	case TARGET_C1XX:
	case TARGET_9BXX:
		MAX_ECU1_XCP_reply_index_bkgd = 14;
		canTransmit(canREG1, XCP_EA3_REQUEST_ID, ( uint8 *) xcp_write_session_Nexteer_mode);	// open Nexteer session prior to XCP

	break;
	case TARGET_FCA_ADAS:
		MAX_ECU1_XCP_reply_index_bkgd = 14;
		canTransmit(canREG1, FCA_XCP_EA3_REQUEST_ID, ( uint8 *) xcp_write_session_Nexteer_mode);	// open Nexteer session prior to XCP

	break;
	case TARGET_CD391_ADAS:
		MAX_ECU1_XCP_reply_index_bkgd = 14;
		canTransmit(canREG1, FORD_XCP_EA3_REQUEST_ID, ( uint8 *) xcp_write_session_Nexteer_mode);	// open Nexteer session prior to XCP

	break;
	case TARGET_G2KCA_ADAS:		//EA4 is different  GM ADAS is different
		MAX_ECU1_XCP_reply_index_bkgd = 17;
		canTransmit(canREG1, G2KCA_ECU1_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_write_session_Nexteer_mode);	// open Nexteer session prior to XCP

	break;
    case TARGET_PSA_CMP:
    	MAX_ECU1_XCP_reply_index_bkgd = 14;
    	canTransmit(canREG1, PSA_XCP_EA3_REQUEST_ID, ( uint8 *) xcp_write_session_Nexteer_mode);    // open Nexteer session prior to XCP

    break;
    case TARGET_SGMW_CN200:       //EA4 is different
    	MAX_ECU1_XCP_reply_index_bkgd = 17;
    	canTransmit(canREG1, CN200_ECU1_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_write_session_Nexteer_mode);    // open Nexteer session prior to XCP

    break;
    case TARGET_FORD_T3_T6:       //EA4 is different
    	MAX_ECU1_XCP_reply_index_bkgd = 17;
    	canTransmit(canREG1, FORD_T3_ECU1_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_write_session_Nexteer_mode);    // open Nexteer session prior to XCP

    break;

    case TARGET_RENAULT_NISSAN:       //EA4 is different
        MAX_ECU1_XCP_reply_index_bkgd = 17;
        canTransmit(canREG1, RENAULT_NISSAN_ECU1_CAN1_XCP_EA4_REQUEST_ID, ( uint8 *) xcp_write_session_Nexteer_mode);    // open Nexteer session prior to XCP

    break;

    case TARGET_BMW_UKL:
		//BMW UKL NEEDS TO BE FIXED in frayNotification(), need to get rid of the msec waits below:
		MAX_ECU1_XCP_reply_index_bkgd = 7;
		transmitFlexray(Nexteer_session, MFG_TX,2);

		system_msec_clock_temp = system_msec_clock + 250;	// set time out to 500mS for rev2 5/09/13 WAS 150ms normally
		while((system_msec_clock < system_msec_clock_temp)); 	  // changed to <14 from 12 for NEW 5 byte DTCs number

		transmitFlexray(Get_UKL_DTC_Rqst,MFG_TX, 2);

		system_msec_clock_temp = system_msec_clock + 250;   // set time out to 500mS for rev2 5/09/13 WAS 150ms normally
		while(system_msec_clock < system_msec_clock_temp);       // changed to <14 from 12 for NEW 5 byte DTCs number

    break;

    case TARGET_BMW_FAAR_WE:
    	MAX_ECU1_XCP_reply_index_bkgd = 7;
		transmitFlexray(MFG_TX_nexteer_session_MSG, MFG_TX,3);

    break;
	}

}

void build_dtc1_bkgd_return_message(void)
{
	//ONLY BMW_FAAR_WE and BMW_UKL are built
	switch (target_product)
	{
	case TARGET_T1XX:
	{
	// rows 39-42 DTCs 1-4	47-50
    sprintf(dtc1_bkgd_return_message,\
    "47,%.1X%.2X:%.2X:%.2X:%.2X,48,%.1X%.2X:%.2X:%.2X:%.2X,49,%.1X%.2X:%.2X:%.2X:%.2X,50,%.1X%.2X:%.2X:%.2X:%.2X,",\
    dtc1[0][0],dtc1[0][1],dtc1[0][2],dtc1[0][3],dtc1[0][4],dtc1[1][0],dtc1[1][1],dtc1[1][2],dtc1[1][3],dtc1[1][4],dtc1[2][0],dtc1[2][1],\
    dtc1[2][2],dtc1[2][3],dtc1[2][4],dtc1[3][0],dtc1[3][1],dtc1[3][2],dtc1[3][3],dtc1[3][4]);

	// rows 43-46 DTCs 5-8	51-54
    sprintf(temp_return_message,\
    "51,%.1X%.2X:%.2X:%.2X:%.2X,52,%.1X%.2X:%.2X:%.2X:%.2X,53,%.1X%.2X:%.2X:%.2X:%.2X,54,%.1X%.2X:%.2X:%.2X:%.2X,",\
    dtc1[4][0],dtc1[4][1],dtc1[4][2],dtc1[4][3],dtc1[4][4],dtc1[5][0],dtc1[5][1],dtc1[5][2],dtc1[5][3],dtc1[5][4],dtc1[6][0],dtc1[6][1],\
    dtc1[6][2],dtc1[6][3],dtc1[6][4],dtc1[7][0],dtc1[7][1],dtc1[7][2],dtc1[7][3],dtc1[7][4]);

	strcat(dtc1_bkgd_return_message,temp_return_message);

	// rows 47-50 DTCs 9-12		55-58
    sprintf(temp_return_message,\
    "55,%.1X%.2X:%.2X:%.2X:%.2X,56,%.1X%.2X:%.2X:%.2X:%.2X,57,%.1X%.2X:%.2X:%.2X:%.2X,58,%.1X%.2X:%.2X:%.2X:%.2X,",\
    dtc1[8][0],dtc1[8][1],dtc1[8][2],dtc1[8][3],dtc1[8][4],dtc1[9][0],dtc1[9][1],dtc1[9][2],dtc1[9][3],dtc1[9][4],dtc1[10][0],dtc1[10][1],\
    dtc1[10][2],dtc1[10][3],dtc1[10][4],dtc1[11][0],dtc1[11][1],dtc1[11][2],dtc1[11][3],dtc1[11][4]);

	strcat(dtc1_bkgd_return_message,temp_return_message);

	// rows 41-53 DTCs 13-15	59-61
    sprintf(temp_return_message,\
    "59,%.1X%.2X:%.2X:%.2X:%.2X,60,%.1X%.2X:%.2X:%.2X:%.2X,61,%.1X%.2X:%.2X:%.2X:%.2X \r",\
    dtc1[12][0],dtc1[12][1],dtc1[12][2],dtc1[12][3],dtc1[12][4],dtc1[13][0],dtc1[13][1],dtc1[13][2],dtc1[13][3],dtc1[13][4],dtc1[14][0],dtc1[14][1],\
    dtc1[14][2],dtc1[14][3],dtc1[14][4]);

	strcat(dtc1_bkgd_return_message,temp_return_message);
	break;
	}
// in the case we have only one ECU, we store 15 consecutive DCTs, unlike the case where we have two ECUs
	case TARGET_C1XX:
    case TARGET_PSA_CMP:
    case TARGET_SGMW_CN200:
    case TARGET_RENAULT_NISSAN:
	{
	// rows 39-42 DTCs 1-4
    sprintf(dtc1_bkgd_return_message,\
    "39,%.1X%.2X:%.2X:%.2X:%.2X,40,%.1X%.2X:%.2X:%.2X:%.2X,41,%.1X%.2X:%.2X:%.2X:%.2X,42,%.1X%.2X:%.2X:%.2X:%.2X,",\
    dtc1[0][0],dtc1[0][1],dtc1[0][2],dtc1[0][3],dtc1[0][4],dtc1[1][0],dtc1[1][1],dtc1[1][2],dtc1[1][3],dtc1[1][4],dtc1[2][0],dtc1[2][1],\
    dtc1[2][2],dtc1[2][3],dtc1[2][4],dtc1[3][0],dtc1[3][1],dtc1[3][2],dtc1[3][3],dtc1[3][4]);

	// rows 43-46 DTCs 5-8
    sprintf(temp_return_message,\
    "43,%.1X%.2X:%.2X:%.2X:%.2X,44,%.1X%.2X:%.2X:%.2X:%.2X,45,%.1X%.2X:%.2X:%.2X:%.2X,46,%.1X%.2X:%.2X:%.2X:%.2X,",\
    dtc1[4][0],dtc1[4][1],dtc1[4][2],dtc1[4][3],dtc1[4][4],dtc1[5][0],dtc1[5][1],dtc1[5][2],dtc1[5][3],dtc1[5][4],dtc1[6][0],dtc1[6][1],\
    dtc1[6][2],dtc1[6][3],dtc1[6][4],dtc1[7][0],dtc1[7][1],dtc1[7][2],dtc1[7][3],dtc1[7][4]);

	strcat(dtc1_bkgd_return_message,temp_return_message);

	// rows 47-51 DTCs 9-12
    sprintf(temp_return_message,\
    "47,%.1X%.2X:%.2X:%.2X:%.2X,48,%.1X%.2X:%.2X:%.2X:%.2X,49,%.1X%.2X:%.2X:%.2X:%.2X,50,%.1X%.2X:%.2X:%.2X:%.2X,",\
    dtc1[8][0],dtc1[8][1],dtc1[8][2],dtc1[8][3],dtc1[8][4],dtc1[9][0],dtc1[9][1],dtc1[9][2],dtc1[9][3],dtc1[9][4],dtc1[10][0],dtc1[10][1],\
    dtc1[10][2],dtc1[10][3],dtc1[10][4],dtc1[11][0],dtc1[11][1],dtc1[11][2],dtc1[11][3],dtc1[11][4]);

	strcat(dtc1_bkgd_return_message,temp_return_message);

	// rows 51-53 DTCs 13-15
    sprintf(temp_return_message,\
    "51,%.1X%.2X:%.2X:%.2X:%.2X,52,%.1X%.2X:%.2X:%.2X:%.2X,53,%.1X%.2X:%.2X:%.2X:%.2X \r",\
    dtc1[12][0],dtc1[12][1],dtc1[12][2],dtc1[12][3],dtc1[12][4],dtc1[13][0],dtc1[13][1],dtc1[13][2],dtc1[13][3],dtc1[13][4],dtc1[14][0],dtc1[14][1],\
    dtc1[14][2],dtc1[14][3],dtc1[14][4]);

	strcat(dtc1_bkgd_return_message,temp_return_message);
	break;
	}
// in the case we have two ECUs, for ECU1 we only store the first 7 dcts, saving the rest (of the 14 + one N/A field- to visually separate) of the fields for ECU2.
	case TARGET_FCA_ADAS:
	case TARGET_CD391_ADAS:
	case TARGET_G2KCA_ADAS:
    case TARGET_FORD_T3_T6:
	{
	// rows 39-42 DTCs 1-4
    sprintf(dtc1_bkgd_return_message,\
    "39,%.1X%.2X:%.2X:%.2X:%.2X,40,%.1X%.2X:%.2X:%.2X:%.2X,41,%.1X%.2X:%.2X:%.2X:%.2X,42,%.1X%.2X:%.2X:%.2X:%.2X,",\
    dtc1[0][0],dtc1[0][1],dtc1[0][2],dtc1[0][3],dtc1[0][4],dtc1[1][0],dtc1[1][1],dtc1[1][2],dtc1[1][3],dtc1[1][4],dtc1[2][0],dtc1[2][1],\
    dtc1[2][2],dtc1[2][3],dtc1[2][4],dtc1[3][0],dtc1[3][1],dtc1[3][2],dtc1[3][3],dtc1[3][4]);

	// rows 43-45 DTCs 5-7
    sprintf(temp_return_message,\
    "43,%.1X%.2X:%.2X:%.2X:%.2X,44,%.1X%.2X:%.2X:%.2X:%.2X,45,%.1X%.2X:%.2X:%.2X:%.2X,",\
    dtc1[4][0],dtc1[4][1],dtc1[4][2],dtc1[4][3],dtc1[4][4],dtc1[5][0],dtc1[5][1],dtc1[5][2],dtc1[5][3],dtc1[5][4],dtc1[6][0],dtc1[6][1],\
    dtc1[6][2],dtc1[6][3],dtc1[6][4]);

	strcat(dtc1_bkgd_return_message,temp_return_message);

	break;
	}

	case TARGET_9BXX:
	{
		sprintf(dtc1_bkgd_return_message,"Haven't been written yet.");
		break;
	}

	//BUILT THESE 14May18 P. Horny
	case TARGET_BMW_FAAR_WE:
	case TARGET_BMW_UKL:
	{
	    // DTCs 1-4
	    sprintf(dtc1_bkgd_return_message,\
	            "ntc_data,%.1X%.2X:%.2X:%.2X:%.2X,%.1X%.2X:%.2X:%.2X:%.2X,%.1X%.2X:%.2X:%.2X:%.2X,%.1X%.2X:%.2X:%.2X:%.2X,",\
	            dtc1[0][0],dtc1[0][1],dtc1[0][2],dtc1[0][3],dtc1[0][4],dtc1[1][0],dtc1[1][1],dtc1[1][2],dtc1[1][3],dtc1[1][4],dtc1[2][0],dtc1[2][1],\
	            dtc1[2][2],dtc1[2][3],dtc1[2][4],dtc1[3][0],dtc1[3][1],dtc1[3][2],dtc1[3][3],dtc1[3][4]);

	    // DTCs 5-8
	    sprintf(temp_return_message,\
	            "%.1X%.2X:%.2X:%.2X:%.2X,%.1X%.2X:%.2X:%.2X:%.2X,%.1X%.2X:%.2X:%.2X:%.2X,%.1X%.2X:%.2X:%.2X:%.2X,",\
	            dtc1[4][0],dtc1[4][1],dtc1[4][2],dtc1[4][3],dtc1[4][4],dtc1[5][0],dtc1[5][1],dtc1[5][2],dtc1[5][3],dtc1[5][4],dtc1[6][0],dtc1[6][1],\
	            dtc1[6][2],dtc1[6][3],dtc1[6][4],dtc1[7][0],dtc1[7][1],dtc1[7][2],dtc1[7][3],dtc1[7][4]);

	    strcat(dtc1_bkgd_return_message,temp_return_message);

	    // DTCs 9-12
	    sprintf(temp_return_message,\
	            "%.1X%.2X:%.2X:%.2X:%.2X,%.1X%.2X:%.2X:%.2X:%.2X,%.1X%.2X:%.2X:%.2X:%.2X,%.1X%.2X:%.2X:%.2X:%.2X,",\
	            dtc1[8][0],dtc1[8][1],dtc1[8][2],dtc1[8][3],dtc1[8][4],dtc1[9][0],dtc1[9][1],dtc1[9][2],dtc1[9][3],dtc1[9][4],dtc1[10][0],dtc1[10][1],\
	            dtc1[10][2],dtc1[10][3],dtc1[10][4],dtc1[11][0],dtc1[11][1],dtc1[11][2],dtc1[11][3],dtc1[11][4]);

	    strcat(dtc1_bkgd_return_message,temp_return_message);

	    // DTCs 13-15
	    sprintf(temp_return_message,\
	            "%.1X%.2X:%.2X:%.2X:%.2X,%.1X%.2X:%.2X:%.2X:%.2X,%.1X%.2X:%.2X:%.2X:%.2X\r",\
	            dtc1[12][0],dtc1[12][1],dtc1[12][2],dtc1[12][3],dtc1[12][4],dtc1[13][0],dtc1[13][1],dtc1[13][2],dtc1[13][3],dtc1[13][4],dtc1[14][0],dtc1[14][1],\
	            dtc1[14][2],dtc1[14][3],dtc1[14][4]);

	    strcat(dtc1_bkgd_return_message,temp_return_message);
		break;
	}
	}  // end switch (target_product)



}
