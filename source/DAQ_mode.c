/*
 * DAQ_mode.c
 *
 *  Created on: Jan 9, 2018
 *      Author: TZZ176
 */
#include "hal_stdtypes.h"
#include "DAQ_mode.h"

#include "can.h"
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
#include "reg_system.h"


#include "flexray.h"
//#include "variable_input.h"
#include "can.h"

/************************************************************************************************************************************
 * DAQ_mode.c
 *
 * 1) start_daq_mode command is called in main, if words[1] = "daq" or "xcp_fast_rate", then manufacturing service DAQ mode is enabled
 * 2) Command checks for CAN/FRAY and a valid period
 *      If FRAY, build appropriate list...
 *      if CAN, build appropriate list...
 * 3)
 *
 *
 *
 * */

    struct variable_input {
        char name[100];
        uint8 address[4];
        uint8 data[9];
        char type[15];
    } variable_info[30];

    char temp_address[30][8];

    /*Dependents*********/
    #define CAN 0
    #define FRAY 1
    extern int comm_is_CAN_or_FRAY;
    extern char endian[15];
    extern char target_processor[15];
    extern char target_product_str[50];
    /********************/

    //uint8 DAQ_data[30][9];        // storage for XCP data response messages, must store response and process in MAIN, error stack if sprintf called from ISR

    //struct variable_input variable_info[30];
// ------------------------------------- XCP Fast Rate vars (periodic XCP request mode) ---------------------------------------
    int XCP_fast_rate_period = 2000;
    unsigned int xcp_fast_rate_period_limit = 10; //ms
    int XCP_fast_rate_active_flag = 0;

    int XCP_fast_rate_index = 0;

    extern int print_data_flag;
    extern int number_of_DAQ_variables;

    float printable_XCP_fast_rate_data[30] = {0.0};// message ready to print after converting to appropriate types

    uint8 XCP_request_fray_variable_0[64][4];
    uint8 XCP_request_fray_variable_1[64][4];
    uint8 XCP_request_fray_variable_2[64][4];
    uint8 XCP_request_fray_variable_3[64][4];
    uint8 XCP_request_fray_variable_4[64][4];
    uint8 XCP_request_fray_variable_5[64][4];
    uint8 XCP_request_fray_variable_6[64][4];
    uint8 XCP_request_fray_variable_7[64][4];
    uint8 XCP_request_fray_variable_8[64][4];
    uint8 XCP_request_fray_variable_9[64][4];
    uint8 XCP_request_fray_variable_10[64][4];
    uint8 XCP_request_fray_variable_11[64][4];
    uint8 XCP_request_fray_variable_12[64][4];
    uint8 XCP_request_fray_variable_13[64][4];
    uint8 XCP_request_fray_variable_14[64][4];
    uint8 XCP_request_fray_variable_15[64][4];
    uint8 XCP_request_fray_variable_16[64][4];
    uint8 XCP_request_fray_variable_17[64][4];
    uint8 XCP_request_fray_variable_18[64][4];
    uint8 XCP_request_fray_variable_19[64][4];
    uint8 XCP_request_fray_variable_20[64][4];
    uint8 XCP_request_fray_variable_21[64][4];
    uint8 XCP_request_fray_variable_22[64][4];
    uint8 XCP_request_fray_variable_23[64][4];
    uint8 XCP_request_fray_variable_24[64][4];
    uint8 XCP_request_fray_variable_25[64][4];
    uint8 XCP_request_fray_variable_26[64][4];
    uint8 XCP_request_fray_variable_27[64][4];
    uint8 XCP_request_fray_variable_28[64][4];
    uint8 XCP_request_fray_variable_29[64][4];

    uint8 XCP_request_fray_variable[30][64][4];
    uint8 XCP_request_CAN_variable[30][8];

    uint8 xcp_dynamic_rx_data[256];

// -------------------------------------Generic XCP DAQ MODE VARS ---------------------------------------
int number_of_DAQ_variables = 0; //<<<<<<< This is # of variables to load for DAQ mode
int daq_period_limit = 9; //ms

int buffered_idx = 0;

unsigned int var_cnt;   // loop counter for loading DAQ stuff
int number_of_ODTs;
int ODT_cnt;
int var_idx;
int loop_count;         // another loop counter for DAQ stuff
unsigned int DAQ_mode_on_off;  // toggle variable for DAQ mode ON/OFF

uint8 sbuf_tx_DAQ_data[1024]; //  used for printing to sciREG1, our debug com port

int DAQ_mode_step_cntr;
int active_DAQ_table = 0;
int total_DAQ_to_tx = 0;
int Set_DAQ_List_mode_DAQ_rate = 10;
int print_data_flag = 0;

uint8_t ODT_rx_byte0;
unsigned int DAQ_slow_or_standard = 1;

int call_start_daq_after_this_command_flag = 0;


// -------------------------------------CAN XCP DAQ MODE VARS ---------------------------------------
// --------------------------------------------- CAN DAQ mode code -----------------------------------------------------------------------//

//Suggest using generic variable names for DAQ mode XCP stuff. Just copy appropriate message box.
#define CAN_XCP_DAQ_TX canMESSAGE_BOX10 //Copy appropriate message box used for XCP over CAN
#define CAN_XCP_DAQ_RX canMESSAGE_BOX11 //Copy appropriate message box used for XCP over CAN




extern unsigned int DAQ_mode_on_off;  // toggle variable for DAQ mode ON/OFF

extern uint8 sbuf_tx_DAQ_data[1024]; //  used for printing to sciREG1, our debug com port
uint8 XCP_connect_CAN[]                  = {0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8 XCP_free_DAQ_CAN[]                 = {0xD6,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8 XCP_allocate_DAQ_list_CAN[]        = {0xD5,0x00,0x01,0x00,0x00,0x00,0x00,0x00}; //byte[2] = # of DAQ lists to allocate
uint8 XCP_allocate_ODT_CAN[]             = {0xD4,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint8 XCP_allocate_ODT_entries_CAN[30][8] =     {       //byte[4] = variable slot to allocate, byte[5] = daq list to allocate, daq list 0x00 in this case
                                                        {0xD3,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
                                                        {0xD3,0x00,0x00,0x00,0x01,0x00,0x00,0x00},
                                                        {0xD3,0x00,0x00,0x00,0x02,0x00,0x00,0x00},
                                                        {0xD3,0x00,0x00,0x00,0x03,0x00,0x00,0x00},
                                                        {0xD3,0x00,0x00,0x00,0x04,0x00,0x00,0x00},
                                                        {0xD3,0x00,0x00,0x00,0x05,0x00,0x00,0x00},
                                                        {0xD3,0x00,0x00,0x00,0x06,0x00,0x00,0x00},
                                                        {0xD3,0x00,0x00,0x00,0x07,0x00,0x00,0x00},
                                                        {0xD3,0x00,0x00,0x00,0x08,0x00,0x00,0x00},
                                                        {0xD3,0x00,0x00,0x00,0x09,0x00,0x00,0x00},
                                                    {0xD3,0x00,0x00,0x00,0x0A,0x00,0x00,0x00},
                                                    {0xD3,0x00,0x00,0x00,0x0B,0x00,0x00,0x00},
                                                    {0xD3,0x00,0x00,0x00,0x0C,0x00,0x00,0x00},
                                                    {0xD3,0x00,0x00,0x00,0x0D,0x00,0x00,0x00},
                                                    {0xD3,0x00,0x00,0x00,0x0E,0x00,0x00,0x00},
                                                    {0xD3,0x00,0x00,0x00,0x0F,0x00,0x00,0x00},
                                                    {0xD3,0x00,0x00,0x00,0x10,0x00,0x00,0x00},
                                                    {0xD3,0x00,0x00,0x00,0x11,0x00,0x00,0x00},
                                                    {0xD3,0x00,0x00,0x00,0x12,0x00,0x00,0x00},
                                                    {0xD3,0x00,0x00,0x00,0x13,0x00,0x00,0x00},
                                                        {0xD3,0x00,0x00,0x00,0x14,0x00,0x00,0x00},
                                                        {0xD3,0x00,0x00,0x00,0x15,0x00,0x00,0x00},
                                                        {0xD3,0x00,0x00,0x00,0x16,0x00,0x00,0x00},
                                                        {0xD3,0x00,0x00,0x00,0x17,0x00,0x00,0x00},
                                                        {0xD3,0x00,0x00,0x00,0x18,0x00,0x00,0x00},
                                                        {0xD3,0x00,0x00,0x00,0x19,0x00,0x00,0x00},
                                                        {0xD3,0x00,0x00,0x00,0x1A,0x00,0x00,0x00},
                                                        {0xD3,0x00,0x00,0x00,0x1B,0x00,0x00,0x00},
                                                        {0xD3,0x00,0x00,0x00,0x1C,0x00,0x00,0x00},
                                                        {0xD3,0x00,0x00,0x00,0x1D,0x00,0x00,0x00}};


uint8 XCP_Set_DAQ_pointer_CAN[30][8]      = {           //This message points to slot where 0xE1 message is stored
                                                        {0xE2,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
                                                        {0xE2,0x00,0x00,0x00,0x01,0x00,0x00,0x00},
                                                        {0xE2,0x00,0x00,0x00,0x02,0x00,0x00,0x00},
                                                        {0xE2,0x00,0x00,0x00,0x03,0x00,0x00,0x00},
                                                        {0xE2,0x00,0x00,0x00,0x04,0x00,0x00,0x00},
                                                        {0xE2,0x00,0x00,0x00,0x05,0x00,0x00,0x00},
                                                        {0xE2,0x00,0x00,0x00,0x06,0x00,0x00,0x00},
                                                        {0xE2,0x00,0x00,0x00,0x07,0x00,0x00,0x00},
                                                        {0xE2,0x00,0x00,0x00,0x08,0x00,0x00,0x00},
                                                        {0xE2,0x00,0x00,0x00,0x09,0x00,0x00,0x00},
                                                    {0xE2,0x00,0x00,0x00,0x0A,0x00,0x00,0x00},
                                                    {0xE2,0x00,0x00,0x00,0x0B,0x00,0x00,0x00},
                                                    {0xE2,0x00,0x00,0x00,0x0C,0x00,0x00,0x00},
                                                    {0xE2,0x00,0x00,0x00,0x0D,0x00,0x00,0x00},
                                                    {0xE2,0x00,0x00,0x00,0x0E,0x00,0x00,0x00},
                                                    {0xE2,0x00,0x00,0x00,0x0F,0x00,0x00,0x00},
                                                    {0xE2,0x00,0x00,0x00,0x10,0x00,0x00,0x00},
                                                    {0xE2,0x00,0x00,0x00,0x11,0x00,0x00,0x00},
                                                    {0xE2,0x00,0x00,0x00,0x12,0x00,0x00,0x00},
                                                    {0xE2,0x00,0x00,0x00,0x13,0x00,0x00,0x00},
                                                        {0xE2,0x00,0x00,0x00,0x14,0x00,0x00,0x00},
                                                        {0xE2,0x00,0x00,0x00,0x15,0x00,0x00,0x00},
                                                        {0xE2,0x00,0x00,0x00,0x16,0x00,0x00,0x00},
                                                        {0xE2,0x00,0x00,0x00,0x17,0x00,0x00,0x00},
                                                        {0xE2,0x00,0x00,0x00,0x18,0x00,0x00,0x00},
                                                        {0xE2,0x00,0x00,0x00,0x19,0x00,0x00,0x00},
                                                        {0xE2,0x00,0x00,0x00,0x1A,0x00,0x00,0x00},
                                                        {0xE2,0x00,0x00,0x00,0x1B,0x00,0x00,0x00},
                                                        {0xE2,0x00,0x00,0x00,0x1C,0x00,0x00,0x00},
                                                        {0xE2,0x00,0x00,0x00,0x1D,0x00,0x00,0x00}};


uint8 XCP_write_DAQ_CAN[30][8]            ={    //        byte [3] = #ofbytes requested
                                                //        bytes [4-7] = address
                                            {0xE1,0xFF,0x04,0x00,0xF8,0xA2,0xBF,0xFE}, // 0xFEBF A2 F8+04 Rte_CDD_GmT1xxMcuCfg_BattVltg_Val
                                            {0xE1,0xFF,0x01,0x00,0x52,0xAA,0xBF,0xFE}, // 0xFEBF AA 52+01 System State
                                            {0xE1,0xFF,0x04,0x00,0x90,0xA6,0xBF,0xFE}, // 0xFEBF A4 E0+04 Vehicle Speed
                                            {0xE1,0xFF,0x01,0x00,0x38,0xAA,0xBF,0xFE}, // 0xFEBF AA 38+01 AbsltMotPosABDifStHlth_Val
                                            {0xE1,0xFF,0x01,0x00,0x39,0xAA,0xBF,0xFE}, // 0xFEBF AA 39+01 AbsltMotPosACDifStHlth_Val
                                            {0xE1,0xFF,0x01,0x00,0x3A,0xAA,0xBF,0xFE}, // 0xFEBF AA 3A+01 AbsltMotPosBCDifStHlth_Val
                                            {0xE1,0xFF,0x01,0x00,0x3D,0xAA,0xBF,0xFE}, // 0xFEBF AA 3D+01 CurrMotSumABCStHlth_Val
                                            {0xE1,0xFF,0x01,0x00,0x3E,0xAA,0xBF,0xFE}, // 0xFEBF AA 3E+01 CurrMotSumDEFStHlth_Val
                                            {0xE1,0xFF,0x01,0x00,0x40,0xAA,0xBF,0xFE}, // 0xFEBF AA 40+01 DigTqSnsrAStHlth_Val
                                            {0xE1,0xFF,0x01,0x00,0x41,0xAA,0xBF,0xFE}, //0xFEBF AA 41+01 DigTqSnsrBStHlth_Val
                                                {0xE1,0xFF,0x04,0x00,0xF8,0xA2,0xBF,0xFE}, // 0xFEBF A2 F8+04 Rte_CDD_GmT1xxMcuCfg_BattVltg_Val
                                                {0xE1,0xFF,0x01,0x00,0x52,0xAA,0xBF,0xFE}, // 0xFEBF AA 52+01 System State
                                                {0xE1,0xFF,0x04,0x00,0x90,0xA6,0xBF,0xFE}, // 0xFEBF A4 E0+04 Vehicle Speed
                                                {0xE1,0xFF,0x01,0x00,0x38,0xAA,0xBF,0xFE}, // 0xFEBF AA 38+01 AbsltMotPosABDifStHlth_Val
                                                {0xE1,0xFF,0x01,0x00,0x39,0xAA,0xBF,0xFE}, // 0xFEBF AA 39+01 AbsltMotPosACDifStHlth_Val
                                                {0xE1,0xFF,0x01,0x00,0x3A,0xAA,0xBF,0xFE}, // 0xFEBF AA 3A+01 AbsltMotPosBCDifStHlth_Val
                                                {0xE1,0xFF,0x01,0x00,0x3D,0xAA,0xBF,0xFE}, // 0xFEBF AA 3D+01 CurrMotSumABCStHlth_Val
                                                {0xE1,0xFF,0x01,0x00,0x3E,0xAA,0xBF,0xFE}, // 0xFEBF AA 3E+01 CurrMotSumDEFStHlth_Val
                                                {0xE1,0xFF,0x01,0x00,0x40,0xAA,0xBF,0xFE}, // 0xFEBF AA 40+01 DigTqSnsrAStHlth_Val
                                                {0xE1,0xFF,0x01,0x00,0x41,0xAA,0xBF,0xFE}, //0xFEBF AA 41+01 DigTqSnsrBStHlth_Val
                                            {0xE1,0xFF,0x04,0x00,0xF8,0xA2,0xBF,0xFE}, // 0xFEBF A2 F8+04 Rte_CDD_GmT1xxMcuCfg_BattVltg_Val
                                            {0xE1,0xFF,0x01,0x00,0x52,0xAA,0xBF,0xFE}, // 0xFEBF AA 52+01 System State
                                            {0xE1,0xFF,0x04,0x00,0x90,0xA6,0xBF,0xFE}, // 0xFEBF A4 E0+04 Vehicle Speed
                                            {0xE1,0xFF,0x01,0x00,0x38,0xAA,0xBF,0xFE}, // 0xFEBF AA 38+01 AbsltMotPosABDifStHlth_Val
                                            {0xE1,0xFF,0x01,0x00,0x39,0xAA,0xBF,0xFE}, // 0xFEBF AA 39+01 AbsltMotPosACDifStHlth_Val
                                            {0xE1,0xFF,0x01,0x00,0x3A,0xAA,0xBF,0xFE}, // 0xFEBF AA 3A+01 AbsltMotPosBCDifStHlth_Val
                                            {0xE1,0xFF,0x01,0x00,0x3D,0xAA,0xBF,0xFE}, // 0xFEBF AA 3D+01 CurrMotSumABCStHlth_Val
                                            {0xE1,0xFF,0x01,0x00,0x3E,0xAA,0xBF,0xFE}, // 0xFEBF AA 3E+01 CurrMotSumDEFStHlth_Val
                                            {0xE1,0xFF,0x01,0x00,0x40,0xAA,0xBF,0xFE}, // 0xFEBF AA 40+01 DigTqSnsrAStHlth_Val
                                            {0xE1,0xFF,0x01,0x00,0x41,0xAA,0xBF,0xFE}}; //0xFEBF AA 41+01 DigTqSnsrBStHlth_Val



uint8 XCP_Set_DAQ_List_mode_CAN[]              = {0xE0,0x10,0x00,0x00,0x00,0x00,0x30,0x00};  // timing is in byte [7] (x30) = 48 48x2ms = 96ms repeat rate
uint8 XCP_Set_DAQ_select_mode_CAN[]            = {0xDE,0x02,0x00,0x00,0x00,0x00,0x00,0x00};  // Mode 0x02 = ???
uint8 XCP_Start_all_selcted_DAQs_CAN[]         = {0xDD,0x01,0x00,0x00,0x00,0x00,0x00,0x00};  // 0x01 starts all DAQ?
uint8 XCP_Stop_all_selcted_DAQs_CAN[]          = {0xDD,0x00,0x00,0x00,0x00,0x00,0x00,0x00};


unsigned char swap[4];  // temp array for reversing bytes for floats (little endian)

// -----------------------------------------End of T1xx DAQ mode code -----------------------------------------------------------------------//
// -------------------------------------End of CAN XCP DAQ MODE VARS ---------------------------------------


// -------------------------------------BMW for FLEXRAY DAQ---------------------------------------
int number_of_DAQ_variables_built;
int even_or_odd_0_or_1;

uint8 XCP_0xDA[64][4];
uint8 XCP_0xD9[64][4];
uint8 XCP_0xD7[64][4];
uint8 XCP_0xF5[64][4];

uint8 XCP_connect_fray[64][4];              //0xFF
uint8 XCP_free_DAQ_fray[64][4];             //0xD6
uint8 XCP_allocate_DAQ_list_fray[64][4];    //0xD5
uint8 XCP_allocate_ODT_fray[64][4];         //0xD4
uint8 XCP_allocate_ODT_entries_fray[64][4]; //0xD3
uint8 XCP_Set_DAQ_pointer_fray[64][4];      //0xE2
uint8 XCP_write_DAQ_fray[64][4];            //0xE1
uint8 XCP_Set_DAQ_List_mode_fray[64][4];    //0xE0
uint8 XCP_Set_DAQ_select_mode_fray[64][4];  //0xDE
uint8 XCP_Start_all_selcted_DAQs_fray[64][4];//0xDD
uint8 XCP_Stop_all_selcted_DAQs_fray[64][4];//0xDD

uint8 XCP_transport_layer_1[64][4];
uint8 XCP_transport_layer_2[64][4];
uint8 XCP_transport_layer_3[64][4];
uint8 XCP_transport_layer_4[64][4];
uint8 XCP_transport_layer_5[64][4];
uint8 XCP_transport_layer_6[64][4];
uint8 XCP_transport_layer_7[64][4];
uint8 XCP_transport_layer_8[64][4];
uint8 XCP_transport_layer_9[64][4];
uint8 XCP_transport_layer_10[64][4];
uint8 XCP_transport_layer_11[64][4];

uint8 stored_DAQ_data[150000];

uint8 XCP_allocate_ODT_entries_fray_msg_2[30][4] =  { //0xD3
                                                        {0x00,0x00,0x02,0x00},
                                                        {0x00,0x00,0x02,0x01},
                                                        {0x00,0x00,0x02,0x02},
                                                        {0x00,0x00,0x02,0x03},
                                                        {0x00,0x00,0x02,0x04},
                                                        {0x00,0x00,0x02,0x05},
                                                        {0x00,0x00,0x02,0x06},
                                                        {0x00,0x00,0x02,0x07},
                                                        {0x00,0x00,0x02,0x08},
                                                        {0x00,0x00,0x02,0x09},
                                                        {0x00,0x00,0x02,0x0A},
                                                        {0x00,0x00,0x02,0x0B},
                                                        {0x00,0x00,0x02,0x0C},
                                                        {0x00,0x00,0x02,0x0D},
                                                        {0x00,0x00,0x02,0x0E},
                                                        {0x00,0x00,0x02,0x0F},
                                                        {0x00,0x00,0x02,0x10},
                                                        {0x00,0x00,0x02,0x11},
                                                        {0x00,0x00,0x02,0x12},
                                                        {0x00,0x00,0x02,0x13},
                                                        {0x00,0x00,0x02,0x14},
                                                        {0x00,0x00,0x02,0x15},
                                                        {0x00,0x00,0x02,0x16},
                                                        {0x00,0x00,0x02,0x17},
                                                        {0x00,0x00,0x02,0x18},
                                                        {0x00,0x00,0x02,0x19},
                                                        {0x00,0x00,0x02,0x1A},
                                                        {0x00,0x00,0x02,0x1B},
                                                        {0x00,0x00,0x02,0x1C},
                                                        {0x00,0x00,0x02,0x1D}};

uint8 XCP_Set_DAQ_pointer_fray_msg_2[30][4] =   { //0xE2
                                                        {0x00,0x00,0x00,0x00}, //vars from previous bmw ukl
                                                        {0x00,0x00,0x00,0x01}, //vars from previous bmw ukl
                                                        {0x00,0x00,0x00,0x02}, //vars from previous bmw ukl
                                                        {0x00,0x00,0x00,0x03}, //vars from previous bmw ukl
                                                        {0x00,0x00,0x00,0x04}, //vars from previous bmw ukl
                                                        {0x00,0x00,0x00,0x05}, //vars from previous bmw ukl
                                                        {0x00,0x00,0x00,0x06}, //vars from previous bmw ukl
                                                        {0x00,0x00,0x00,0x07}, //vars from previous bmw ukl
                                                        {0x00,0x00,0x00,0x08}, //vars from previous bmw ukl
                                                        {0x00,0x00,0x00,0x09}, //vars from previous bmw ukl
                                                        {0x00,0x00,0x00,0x0A}, //vars from previous bmw ukl
                                                        {0x00,0x00,0x00,0x0B}, //vars from previous bmw ukl
                                                        {0x00,0x00,0x00,0x0C}, //vars from previous bmw ukl
                                                        {0x00,0x00,0x00,0x0D}, //vars from previous bmw ukl
                                                        {0x00,0x00,0x00,0x0E},
                                                        {0x00,0x00,0x00,0x0F},
                                                        {0x00,0x00,0x00,0x10},
                                                        {0x00,0x00,0x00,0x11},
                                                        {0x00,0x00,0x00,0x12},
                                                        {0x00,0x00,0x00,0x13},
                                                        {0x00,0x00,0x00,0x14},
                                                        {0x00,0x00,0x00,0x15},
                                                        {0x00,0x00,0x00,0x16},
                                                        {0x00,0x00,0x00,0x17},
                                                        {0x00,0x00,0x00,0x18},
                                                        {0x00,0x00,0x00,0x19},
                                                        {0x00,0x00,0x00,0x1A},
                                                        {0x00,0x00,0x00,0x1B},
                                                        {0x00,0x00,0x00,0x1C},
                                                        {0x00,0x00,0x00,0x1D}};

//VARIABLE ADDRESS are entered into these arrays: XCP_write_DAQ_fray_msg_1 and _2.
uint8 XCP_write_DAQ_fray_msg_1[30][4]     ={    //0xE1
                                                    //USING ASSUMPTION: Pulling 4 bytes using DAQ mode even for 1, 2, or 3 byte variables.
                                                    //                  Will parse and store appropriate bytes using print_periodic_variables()
                                                    //        byte [1] = #ofbytes requested

                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1},
                                                        {0x00,0x04,0xFF,0xE1}};

uint8 XCP_write_DAQ_fray_msg_2[30][4]     ={    //0xE1 - Addresses get stuff in init_DAQ_mode_fray() and init_DAQ_mode_CAN()
                                                        {0x04,0x90,0x01,0x08},
                                                        {0xA0,0x88,0x01,0x08},
                                                        {0x94,0x90,0x01,0x08},
                                                        {0xAC,0xA2,0x01,0x08},
                                                        {0xF8,0xA1,0x01,0x08},
                                                        {0xEC,0xA1,0x01,0x08},
                                                        {0x0C,0x90,0x01,0x08},
                                                        {0xC0,0x90,0x01,0x08},
                                                        {0xBC,0x90,0x01,0x08},
                                                        {0xDC,0xA1,0x01,0x08},
                                                        {0x30,0x90,0x01,0x08},
                                                        {0xC4,0xA3,0x01,0x08},
                                                        {0x74,0x90,0x01,0x08},
                                                        {0x78,0x90,0x01,0x08},
                                                        {0x4A,0xAA,0xBF,0xFE},
                                                        {0x4B,0xAA,0xBF,0xFE},
                                                        {0x4C,0xAA,0xBF,0xFE},
                                                        {0x20,0xA4,0xBF,0xFE},
                                                        {0xB8,0xA2,0xBF,0xFE},
                                                        {0x3D,0xAA,0xBF,0xFE},
                                                        {0x3E,0xAA,0xBF,0xFE},
                                                        {0x40,0xAA,0xBF,0xFE},
                                                        {0x41,0xAA,0xBF,0xFE},
                                                        {0x45,0xAA,0xBF,0xFE},
                                                        {0x47,0xAA,0xBF,0xFE},
                                                        {0x48,0xAA,0xBF,0xFE},
                                                        {0x49,0xAA,0xBF,0xFE},
                                                        {0x4A,0xAA,0xBF,0xFE},
                                                        {0x4B,0xAA,0xBF,0xFE},
                                                        {0x4C,0xAA,0xBF,0xFE}};

    //PREVIOUSLY VERIFIED - The various DAQ mode messages were copied properly for use on BMW UKL
    //VERIFIED  - The correct data is copied from variable_info.address into XCP_write_DAQ_fray_msg_2
    //NOT VERIFIED - Any DAQ list use for Faar We. Not sure if XCP service stuff has changed either.
    // - PKH 27FEB18
    void init_DAQ_mode_fray(void) //build flexray words, also stuffs XCP_write_DAQ_fray_msg_2 with addresses
    {
        extern int target_product;
        #define TARGET_BMW_UKL 12
        #define TARGET_BMW_FAAR_WE 14

        switch(target_product)
        {
            case TARGET_BMW_UKL:
            {
                uint8 cpyArrayX[4] = {0xFF, 0x7E, 0x10, 0x02};
                uint8 cpyArray2[4] = {0x02, 0x00, 0xEE, 0x30};
                uint8 cpyArray8[4] = {0x08, 0x00, 0xEE, 0x30};
                uint8 cpyArray6[4] = {0x06, 0x00, 0xEE, 0x30};
                uint8 cpyArray5[4] = {0x05, 0x00, 0xEE, 0x30};
                uint8 cpyArray4[4] = {0x04, 0x00, 0xEE, 0x30};
                //init fray DAQ mode stuff:
                //2 words
                const unsigned char XCP_connect_fray_msg[]                  = {0x00,0x00,0x00,0xFF};
                memcpy(&XCP_connect_fray[0], &cpyArray4, 4);
                memcpy(&XCP_connect_fray[1], &XCP_connect_fray_msg, 4);

                //2 words
                const unsigned char XCP_free_DAQ_fray_msg[]                 = {0x00,0x00,0x00,0xD6};
                memcpy(&XCP_free_DAQ_fray[0], &cpyArray4, 4);
                memcpy(&XCP_free_DAQ_fray[1], &XCP_free_DAQ_fray_msg, 4);

                //2 words
                const unsigned char XCP_allocate_DAQ_list_fray_msg[]        = {0x01,0x00,0x00,0xD5};
                memcpy(&XCP_allocate_DAQ_list_fray[0], &cpyArray4, 4);
                memcpy(&XCP_allocate_DAQ_list_fray[1], &XCP_allocate_DAQ_list_fray_msg, 4);

                //3 words
                unsigned char XCP_allocate_ODT_fray_msg[2][4]           = { {0x00,0x00,0x00,0xD4},
                                                                            {0x00,0x00,0x00,0x00}};
                memcpy(&XCP_allocate_ODT_fray[0], &cpyArray8, 4);
                memcpy(&XCP_allocate_ODT_fray[1], &XCP_allocate_ODT_fray_msg[0], 4);
                memcpy(&XCP_allocate_ODT_fray[2], &XCP_allocate_ODT_fray_msg[1], 4);

                //3 words
                uint8 XCP_allocate_ODT_entries_fray_msg_1[] =   {0x00,0x00,0x00,0xD3};
                memcpy(&XCP_allocate_ODT_entries_fray[0], &cpyArray8, 4);
                memcpy(&XCP_allocate_ODT_entries_fray[1], &XCP_allocate_ODT_entries_fray_msg_1, 4);
                //word[2] done in flexray notif

                //3 words
                uint8 XCP_Set_DAQ_pointer_fray_msg_1[]      =   {0x00,0x00,0x00,0xE2};
                memcpy(&XCP_Set_DAQ_pointer_fray[0], &cpyArray6, 4);
                memcpy(&XCP_Set_DAQ_pointer_fray[1], &XCP_Set_DAQ_pointer_fray_msg_1, 4);
                //word[2] done in flexray notif

                //3 words
                memcpy(&XCP_write_DAQ_fray[0], &cpyArray8, 4);
                //word[1] done in handle_DAQ_notification_fray
                //word[2] done in handle_DAQ_notification_fray

                //3 words
                unsigned char XCP_Set_DAQ_List_mode_fray_msg[2][4]      ={  {0x00,0x00,0x10,0xE0},
                                                                            {0x00,0x01,0x01,0x00}};  // timing is in byte 07 (x14) =20 20x2ms = 40ms repeat rate
                memcpy(&XCP_Set_DAQ_List_mode_fray[0], &cpyArray8, 4);
                memcpy(&XCP_Set_DAQ_List_mode_fray[1], &XCP_Set_DAQ_List_mode_fray_msg[0], 4);
                memcpy(&XCP_Set_DAQ_List_mode_fray[2], &XCP_Set_DAQ_List_mode_fray_msg[1], 4);

                //2 words
                unsigned char XCP_Set_DAQ_select_mode_fray_msg[]            = {0x00,0x00,0x02,0xDE};
                memcpy(&XCP_Set_DAQ_select_mode_fray[0], &cpyArray4, 4);
                memcpy(&XCP_Set_DAQ_select_mode_fray[1], &XCP_Set_DAQ_select_mode_fray_msg, 4);

                //2 words
                unsigned char XCP_Start_all_selcted_DAQs_fray_msg[]         = {0x00,0x00,0x01,0xDD};
                memcpy(&XCP_Start_all_selcted_DAQs_fray[0], &cpyArray4, 4);
                memcpy(&XCP_Start_all_selcted_DAQs_fray[1], &XCP_Start_all_selcted_DAQs_fray_msg, 4);

                //2 words
                unsigned char XCP_Stop_all_selcted_DAQs_fray_msg[]          = {0x00,0x00,0x00,0xDD};
                memcpy(&XCP_Stop_all_selcted_DAQs_fray[0], &cpyArray4, 4);
                memcpy(&XCP_Stop_all_selcted_DAQs_fray[1], &XCP_Stop_all_selcted_DAQs_fray_msg, 4);
            } // end case TARGET_BMW_UKL:
            break;

            case TARGET_BMW_FAAR_WE:
            {
                //init fray DAQ mode stuff:
                //2 words
                //byte[0][2] looks to be a counter, i'd guess +1 each message after XCP connect is sent. Include this in XCP handler...
                //
                //30?   CNTR    D/C     Bytes of data
                //30    2B      0       1
                //30    2C      0       4

                //------0xDA
                uint8 XCP_0xDA_msg[2][4] = {{0x01,0x00,0x00,0x30},
                                            {0x00,0x00,0x00,0xDA}};
                memcpy(&XCP_0xDA, &XCP_0xDA_msg, 8);

                //------0xD9
                uint8 XCP_0xD9_msg[2][4] = {{0x01,0x00,0x00,0x30},
                                            {0x00,0x00,0x00,0xD9}};
                memcpy(&XCP_0xD9, &XCP_0xD9_msg, 8);

                //------0xD7
                uint8 XCP_0xD7_msg[2][4] = {{0x04,0x00,0x00,0x30},
                                            {0x00,0x00,0x00,0xD7}};
                memcpy(&XCP_0xD7, &XCP_0xD7_msg, 8);

                //------0xF5
                uint8 XCP_0xF5_msg[2][4] = {{0x02,0x00,0x00,0x30},
                                            {0x00,0x00,0x17,0xF5}};
                memcpy(&XCP_0xF5, &XCP_0xF5_msg, 8);

                //------0xD6
                //2 words
                uint8 XCP_free_DAQ_fray_msg[2][4] = {{0x01,0x00,0x00,0x30},
                                                     {0x00,0x00,0x00,0xD6}};
                memcpy(&XCP_free_DAQ_fray, &XCP_free_DAQ_fray_msg, 8);

                //------0xD5
                //2 words
                //byte[1][1] = # of DAQ lists to be allocated
                //      ---->typically desire only 1 DAQ list
                uint8 XCP_allocate_DAQ_list_fray_msg[2][4] = {{0x04,0x00,0x00,0x30},
                                                              {0x00,0x01,0x00,0xD5}};
                memcpy(&XCP_allocate_DAQ_list_fray, &XCP_allocate_DAQ_list_fray_msg, 8);

                //------0xD4
                //3 words
                //byte[1][0] = Which DAQ list to assign ODTs(variables) to, this case is list 0x00
                //byte[2][3] = number of ODTs(Variables) to be assigned to DAQ list ----------->assigned in handle_DAQ_notification_fray()
                uint8 XCP_allocate_ODT_fray_msg[3][4] = {{0x05,0x00,0x00,0x30},
                                                         {0x00,0x00,0x00,0xD4},
                                                         {0x00,0x00,0x00,0x00}};
                memcpy(&XCP_allocate_ODT_fray, &XCP_allocate_ODT_fray_msg, 12);

                //------0xD3
                //3 words
                //byte[2][3] = ODT (variable) # to assign a slot ----------->assigned in handle_DAQ_notification_fray()
                //byte[2][2] = # of ODT entries to assign to the ODT, in this case 1
                uint8 XCP_allocate_ODT_entries_fray_msg_1[2][4] = {{0x06,0x00,0x00,0x30},
                                                                   {0x00,0x00,0x00,0xD3}};
                memcpy(&XCP_allocate_ODT_entries_fray, &XCP_allocate_ODT_entries_fray_msg_1, 8);

                //------0xE2
                //3 words
                //word[2] of XCP_Set_DAQ_pointer_fray is memcpy'd in handle_DAQ_notif using XCP_allocate_ODT_entries_fray_msg_2
                uint8 XCP_Set_DAQ_pointer_fray_msg_1[2][4] = {{0x06,0x00,0x00,0x30},
                                                              {0x00,0x00,0x00,0xE2}};
                memcpy(&XCP_Set_DAQ_pointer_fray, &XCP_Set_DAQ_pointer_fray_msg_1, 8);

                //------0xE1
                //3 words
                //word[2] built from XCP_write_DAQ_fray_msg_1 in handle_DAQ_notif
                //word[3] built from XCP_write_DAQ_fray_msg_2 in handle_DAQ_notif
                uint8 XCP_write_DAQ_fray_msg_0[1][4] = {{0x08,0x00,0x00,0x30}};
                memcpy(&XCP_write_DAQ_fray, &XCP_write_DAQ_fray_msg_0, 4);

                //------0xE0
                //3 words
                //byte[1][2] = Mode, 0x10 = timestamp ON, 0x20 = PID_OFF. Defaulting to timestamp OFF, PID ON
                //byte[2][1] = timing prescaler, set this in handle_DAQ_notif using period, defaulting to 0x05 x 2 = 10ms
                uint8 XCP_Set_DAQ_List_mode_fray_msg[3][4]      ={  {0x08,0x00,0x00,0x30},
                                                                    {0x00,0x00,0x10,0xE0},
                                                                    {0x00,0x01,0x00,0x00}}; //0x05 x 2 = 10 ms
                memcpy(&XCP_Set_DAQ_List_mode_fray, &XCP_Set_DAQ_List_mode_fray_msg, 12);

                //------0xDE
                //2 words
                //byte[1][2] = select lists mode
                //byte[1][2] = lists to select
                uint8 XCP_Set_DAQ_select_mode_fray_msg[2][4] = {{0x04,0x00,0x00,0x30},
                                                                {0x00,0x00,0x02,0xDE}};
                memcpy(&XCP_Set_DAQ_select_mode_fray, &XCP_Set_DAQ_select_mode_fray_msg, 8);

                //------0xF2
                uint8 XCP_transport_layer_msg_1[4][4] ={    {0x0C,0x00,0x00,0x30},
                                                            {0x00,0x00,0xFF,0xF2},
                                                            {0x01,0x00,0x01,0x2D},
                                                            {0x01,0x65,0xFE,0x00}};
                memcpy(&XCP_transport_layer_1, &XCP_transport_layer_msg_1, 16);

                uint8 XCP_transport_layer_msg_2[4][4] ={    {0x0C,0x00,0x00,0x30},
                                                            {0x00,0x01,0xFF,0xF2},
                                                            {0x01,0x00,0x01,0x2A},
                                                            {0x04,0xAE,0xFE,0x00}};
                memcpy(&XCP_transport_layer_2, &XCP_transport_layer_msg_2, 16);

                uint8 XCP_transport_layer_msg_3[4][4] ={    {0x0C,0x00,0x00,0x30},
                                                            {0x00,0x02,0xFF,0xF2},
                                                            {0x01,0x00,0x01,0x27},
                                                            {0x06,0xA2,0xFE,0x00}};
                memcpy(&XCP_transport_layer_3, &XCP_transport_layer_msg_3, 16);

                uint8 XCP_transport_layer_msg_4[4][4] ={    {0x0C,0x00,0x00,0x30},
                                                            {0x00,0x03,0xFF,0xF2},
                                                            {0x01,0x00,0x01,0x28},
                                                            {0x07,0x44,0xFE,0x00}};
                memcpy(&XCP_transport_layer_4, &XCP_transport_layer_msg_4, 16);

                uint8 XCP_transport_layer_msg_5[4][4] ={    {0x0C,0x00,0x00,0x30},
                                                            {0x00,0x04,0xFF,0xF2},
                                                            {0x01,0x00,0x01,0x26},
                                                            {0x07,0x57,0xFE,0x00}};
                memcpy(&XCP_transport_layer_5, &XCP_transport_layer_msg_5, 16);

                uint8 XCP_transport_layer_msg_6[4][4] ={    {0x0C,0x00,0x00,0x30},
                                                            {0x00,0x05,0xFF,0xF2},
                                                            {0x01,0x00,0x01,0x29},
                                                            {0x06,0xB1,0xFE,0x00}};
                memcpy(&XCP_transport_layer_6, &XCP_transport_layer_msg_6, 16);

                uint8 XCP_transport_layer_msg_7[4][4] ={    {0x0C,0x00,0x00,0x30},
                                                            {0x00,0x06,0xFF,0xF2},
                                                            {0x01,0x00,0x01,0x2C},
                                                            {0x00,0x90,0xFE,0x00}};
                memcpy(&XCP_transport_layer_7, &XCP_transport_layer_msg_7, 16);

                uint8 XCP_transport_layer_msg_8[4][4] ={    {0x0C,0x00,0x00,0x30},
                                                            {0x00,0x07,0xFF,0xF2},
                                                            {0x01,0x00,0x00,0x0B},
                                                            {0x01,0xFF,0x10,0x00}};
                memcpy(&XCP_transport_layer_8, &XCP_transport_layer_msg_8, 16);

                uint8 XCP_transport_layer_msg_9[4][4] ={    {0x0C,0x00,0x00,0x30},
                                                            {0x00,0x09,0xFF,0xF2},
                                                            {0x01,0x00,0x01,0x2B},
                                                            {0x05,0x5B,0xFE,0x00}};
                memcpy(&XCP_transport_layer_9, &XCP_transport_layer_msg_9, 16);

                uint8 XCP_transport_layer_msg_10[4][4] ={   {0x0C,0x00,0x00,0x30},
                                                            {0x10,0x04,0xFF,0xF2},
                                                            {0x01,0x00,0x01,0x26},
                                                            {0x07,0x57,0xFE,0x00}};
                memcpy(&XCP_transport_layer_10, &XCP_transport_layer_msg_10, 16);

                uint8 XCP_transport_layer_msg_11[2][4] ={   {0x01,0x00,0x00,0x30},
                                                            {0x00,0x00,0x00,0xDC}};
                memcpy(&XCP_transport_layer_11, &XCP_transport_layer_msg_11, 8);


                //------0xDD
                //2 words
                //byte[1][2] = 0x01 = START all selected lists
                uint8 XCP_Start_all_selcted_DAQs_fray_msg[2][4] = {{0x02,0x00,0x00,0x30},
                                                                   {0x00,0x00,0x01,0xDD}};
                memcpy(&XCP_Start_all_selcted_DAQs_fray, &XCP_Start_all_selcted_DAQs_fray_msg, 8);

                //2 words
                //byte[1][2] = 0x01 = STOP all selected lists
                uint8 XCP_Stop_all_selcted_DAQs_fray_msg[2][4] = {{0x02,0x00,0x00,0x30},
                                                                  {0x00,0x00,0x00,0xDD}};
                //memcpy(&XCP_Stop_all_selcted_DAQs_fray[0], &cpyArray4, 4);
                memcpy(&XCP_Stop_all_selcted_DAQs_fray, &XCP_Stop_all_selcted_DAQs_fray_msg, 8);
            } // end case TARGET_BMW_FAAR_WE:
            break;
        }

        int i;
        if(variable_info[0].address != 0) //check if any variable info addresses were added. Quick way to check how code user is entering variables (constants or using the struct)
        {
            for(i=0;i<number_of_DAQ_variables;i++)
            {
                memcpy(XCP_write_DAQ_fray_msg_2[i],variable_info[i].address,4);
            }
            if((number_of_DAQ_variables % 2) != 0)
            {
                memcpy(XCP_write_DAQ_fray_msg_2[number_of_DAQ_variables],variable_info[number_of_DAQ_variables-1].address,4); //repeat last address request for odd # of DAQ vars
            }
        }

    }

    //VERIFIED that the correct data is copied from variable_info.address into XCP_write_DAQ_CAN
    // - PKH 27FEB18
    void init_DAQ_mode_CAN(void) //Store addresses into correct CAN message array
    {
        int i,u;
        if(variable_info[0].address != 0) //check if any variable info addresses were added. Quick way to check how code user is entering variables (constants or using the struct)
        {
            for(i=0;i<number_of_DAQ_variables;i++)
            {
                for(u=0;u<4;u++)
                {
                    XCP_write_DAQ_CAN[i][4+u] = variable_info[i].address[u];
                }
            }
        }
    }

    //VERIFIED that char temp_address is properly swapped, converted to uint8, and stored to variable_info.address based on endian or CAN/FRAY
    //-P. Horny 27Feb18
    void swap_addresses_based_on_type(void)
    {
        int u,l,k,j = 0;

        //------------------------------------------
        //NO ORDER SWAP
        //get here if FRAY + little_endian
        //or
        //get here if CAN + big_endian
        if( ((comm_is_CAN_or_FRAY == FRAY)&&(strcmp(endian,"little_endian") == 0)) || ((comm_is_CAN_or_FRAY == CAN)&&(strcmp(endian,"big_endian") == 0)) )
        {
            for(k=0;k<number_of_DAQ_variables;k++)
            {u=0;l=0;
                for(j=0;j<8;j++){
                    if(((j % 2) == 0)||(j==0)){
                        variable_info[k].address[u] = atoh(temp_address[k][j]);
                        u++;
                    }
                    else{
                        variable_info[k].address[l] = (variable_info[k].address[l] << 4) + atoh(temp_address[k][j]);
                        l++;
                    }
                } //end for(j=0...
            } //end for(k=0...
        }//end if(comm_is_CAN_or_FRAY != FRAY)

        //------------------------------------------
        //YES ORDER SWAP
        //get here if FRAY + big_endian
        //or
        //get here if CAN + little_endian
        else if( ((comm_is_CAN_or_FRAY == FRAY)&&(strcmp(endian,"big_endian") == 0)) || ((comm_is_CAN_or_FRAY == CAN)&&(strcmp(endian,"little_endian") == 0)) )
        {
            for(k=0;k<number_of_DAQ_variables;k++)
            {u=3;l=3;
                for(j=0;j<strlen(temp_address[k]);j++){
                    if(((j % 2) == 0)||(j==0)){
                        variable_info[k].address[u] = atoh(temp_address[k][j]);
                        u--;
                    }
                    else{
                        variable_info[k].address[l] = (variable_info[k].address[l] << 4) + atoh(temp_address[k][j]);
                        l--;
                    }
                } //end for(j=0...
            } //end for(k=0...
        }//end if(comm_is_CAN_or_FRAY != FRAY)
    }

    /*Ascii to Hex function*/
    //VERIFIED working
    //-P. Horny 27Feb18
    unsigned char atoh (unsigned char data)
    {
        if (data > '9')
        {
            data += 9;
        }
        return (data &= 0x0F);
    }

    //PREVIOUSLY VERIFIED - On BMW UKL, confirmed that all messages send out. However, did not get positive responses for all.
    //                        It is possible BMW UKL did not have DAQ mode enabled, or some message/byte is wrong.
    //NOT VERIFIED case 3000 storage of data using for() loop. Previously copied directly.
    void handle_DAQ_notification_fray(void)
    {
        extern flexrayPacket XCP_TX_struct;
        extern flexrayPacket *XCP_TX;

        extern uint8 fray_rx_data[64][4];

        if((number_of_DAQ_variables % 2) == 0)
        {
            number_of_DAQ_variables_built = number_of_DAQ_variables; //for even numbers
            even_or_odd_0_or_1 = 0;
        }
        else
        {
            number_of_DAQ_variables_built = number_of_DAQ_variables + 1; //for odd numbers, put in dummy repeat of previous address.
            even_or_odd_0_or_1 = 1;
        }

        switch(DAQ_mode_step_cntr)
        {

        case 2000:    // reply from connect message Que Free DAQ D6

                transmitFlexray(XCP_0xDA,XCP_TX, 2);
                DAQ_mode_step_cntr++;  // increment for next case

        break;

        case 2001:    // reply from connect message Que Free DAQ D6

                transmitFlexray(XCP_0xD9,XCP_TX, 2);
                DAQ_mode_step_cntr++;  // increment for next case

        break;

        case 2002:    // reply from connect message Que Free DAQ D6

                transmitFlexray(XCP_0xD7,XCP_TX, 2);
                DAQ_mode_step_cntr++;  // increment for next case

        break;

        case 2003:    // reply from connect message Que Free DAQ D6

                transmitFlexray(XCP_0xF5,XCP_TX, 2);
                DAQ_mode_step_cntr++;  // increment for next case

        break;

        case 2004:    // reply from connect message Que Free DAQ D6

                transmitFlexray(XCP_free_DAQ_fray,XCP_TX, 2);
                DAQ_mode_step_cntr = 2010;  // increment for next case

        break;

        case 2010:    // reply from Free DAQ  Que Allocate DAQ list DAQ D5

                transmitFlexray(XCP_allocate_DAQ_list_fray,XCP_TX, 2);
                DAQ_mode_step_cntr = 2020;  // increment for next case

        break;

        case 2020:    // reply from Allocate DAQ list DAQ  Que Allocate ODT's D4


            XCP_allocate_ODT_fray[2][3] = number_of_DAQ_variables_built/2;

            transmitFlexray(XCP_allocate_ODT_fray,XCP_TX, 3);
            DAQ_mode_step_cntr = 2030;  // increment for next case
            var_cnt = 0;

        break;

        case 2030:    // reply from Allocate ODT's D4  Que XCP_allocate_ODT_entries D3

            //first part of message built in init_DAQ_mode_fray, just need to update word[2]
            memcpy(&XCP_allocate_ODT_entries_fray[2], &XCP_allocate_ODT_entries_fray_msg_2[var_cnt][0], 4);
            transmitFlexray(XCP_allocate_ODT_entries_fray,XCP_TX, 3);
            var_cnt++;

            if (var_cnt == (number_of_DAQ_variables_built / 2))
            {
                DAQ_mode_step_cntr = 2070;  // increment for next case
                var_cnt  = 0;
                loop_count = 0;
            }

        break;


        case 2070:    // reply from XCP_allocate_ODT_entries D3  Que XCP_Set_DAQ_pointer E2

            //first part of message built in init_DAQ_mode_fray, just need to update word[2]
            memcpy(&XCP_Set_DAQ_pointer_fray[2], &XCP_Set_DAQ_pointer_fray_msg_2[loop_count][0], 4); //loop_count
            transmitFlexray(XCP_Set_DAQ_pointer_fray,XCP_TX, 3);

            DAQ_mode_step_cntr = 2080;  // go to write DAQ

        break;

        case 2080:    // reply from XCP_Set_DAQ_pointer E2  Que XCP_write_DAQ E1

            memcpy(&XCP_write_DAQ_fray[1], &XCP_write_DAQ_fray_msg_1[var_cnt][0], 4);
            memcpy(&XCP_write_DAQ_fray[2], &XCP_write_DAQ_fray_msg_2[var_cnt][0], 4);
            transmitFlexray(XCP_write_DAQ_fray,XCP_TX, 3);

            var_cnt++;
               if (var_cnt < (number_of_DAQ_variables_built))
               {

                   if(var_cnt % 2 == 0)
                   {
                      DAQ_mode_step_cntr = 2070;  // back to DAQ pointer to load another variable
                      loop_count++;
                   }

               }

               else
               {
                   var_cnt  = 0;
                   DAQ_mode_step_cntr = 2230;  // DONE loading variables...go to select DAQ mode
               }

        break;

        case 2230:    // reply from XCP_write_DAQ_fray E1 Que XCP_Set_DAQ_List_mode_fray E0

            XCP_Set_DAQ_List_mode_fray[2][1] = Set_DAQ_List_mode_DAQ_rate / 2; //divide desired ms rate by 2 and stuff into fray msg
            transmitFlexray(XCP_Set_DAQ_List_mode_fray,XCP_TX, 3);
            DAQ_mode_step_cntr = 2240;  // increment for next case

        break;

        case 2240:    // reply from XCP_Set_DAQ_List_mode_fray E0  Que XCP_Set_DAQ_select_mode_fray DE

            transmitFlexray(XCP_Set_DAQ_select_mode_fray,XCP_TX, 3);
            DAQ_mode_step_cntr = 2250;  // increment for next case

        break;


        //-----Transport layer messages-------------------------------------------------------------

        case 2250:    // reply from XCP_Set_DAQ_List_mode_fray E0  Que XCP_Set_DAQ_select_mode_fray DE

            transmitFlexray(XCP_transport_layer_1,XCP_TX, 4);
            DAQ_mode_step_cntr++;  // increment for next case

        break;

        case 2251:    // reply from XCP_Set_DAQ_List_mode_fray E0  Que XCP_Set_DAQ_select_mode_fray DE

            transmitFlexray(XCP_transport_layer_2,XCP_TX, 4);
            DAQ_mode_step_cntr++;  // increment for next case

        break;

        case 2252:    // reply from XCP_Set_DAQ_List_mode_fray E0  Que XCP_Set_DAQ_select_mode_fray DE

            transmitFlexray(XCP_transport_layer_3,XCP_TX, 4);
            DAQ_mode_step_cntr++;  // increment for next case

        break;

        case 2253:    // reply from XCP_Set_DAQ_List_mode_fray E0  Que XCP_Set_DAQ_select_mode_fray DE

            transmitFlexray(XCP_transport_layer_4,XCP_TX, 4);
            DAQ_mode_step_cntr++;  // increment for next case

        break;

        case 2254:    // reply from XCP_Set_DAQ_List_mode_fray E0  Que XCP_Set_DAQ_select_mode_fray DE

            transmitFlexray(XCP_transport_layer_5,XCP_TX, 4);
            DAQ_mode_step_cntr++;  // increment for next case

        break;

        case 2255:    // reply from XCP_Set_DAQ_List_mode_fray E0  Que XCP_Set_DAQ_select_mode_fray DE

            transmitFlexray(XCP_transport_layer_6,XCP_TX, 4);
            DAQ_mode_step_cntr++;  // increment for next case

        break;

        case 2256:    // reply from XCP_Set_DAQ_List_mode_fray E0  Que XCP_Set_DAQ_select_mode_fray DE

            transmitFlexray(XCP_transport_layer_7,XCP_TX, 4);
            DAQ_mode_step_cntr++;  // increment for next case

        break;

        case 2257:    // reply from XCP_Set_DAQ_List_mode_fray E0  Que XCP_Set_DAQ_select_mode_fray DE

            transmitFlexray(XCP_transport_layer_8,XCP_TX, 4);
            DAQ_mode_step_cntr++;  // increment for next case

        break;

        case 2258:    // reply from XCP_Set_DAQ_List_mode_fray E0  Que XCP_Set_DAQ_select_mode_fray DE

            transmitFlexray(XCP_transport_layer_9,XCP_TX, 4);
            DAQ_mode_step_cntr++;  // increment for next case

        break;

        case 2259:    // reply from XCP_Set_DAQ_List_mode_fray E0  Que XCP_Set_DAQ_select_mode_fray DE

            transmitFlexray(XCP_transport_layer_10,XCP_TX, 4);
            DAQ_mode_step_cntr++;  // increment for next case

        break;

        case 2260:    // reply from XCP_Set_DAQ_List_mode_fray E0  Que XCP_Set_DAQ_select_mode_fray DE

            transmitFlexray(XCP_transport_layer_11,XCP_TX, 2);
            DAQ_mode_step_cntr = 2300;  // increment for next case

        break;

        //----------- Start ALL DAQ Tables -------------------------------------------------------------
        case 2300:    // reply from XCP_Set_DAQ_select_mode DE Que XCP_Start_all_selcted_DAQs DD

            transmitFlexray(XCP_Start_all_selcted_DAQs_fray,XCP_TX, 3);
            DAQ_mode_step_cntr = 2310;  // last step

        break;

        case 2310:    // reply from XCP_Start_all_selcted_DAQs_fray DD, set up to store data on remaining requests

            transmitFlexray(XCP_Start_all_selcted_DAQs_fray,XCP_TX, 3);
            DAQ_mode_step_cntr = 3000;  // last step

        break;

        //----------- Collect and store DAQ -------------------------------------------------------------
        case 3000:



                ODT_rx_byte0 = xcp_dynamic_rx_data[4];
                number_of_ODTs = number_of_DAQ_variables_built / 2; //need to figure out odd numbers of DAQ requests...

                var_cnt = 0;
                /* Store data from each request into DAQ*/
            for(ODT_cnt=0;ODT_cnt<number_of_ODTs;ODT_cnt++)
            {
                /* Store data into variable_info.data based on # of bytes requested in XCP_write_DAQ_fray_msg_1[30][4] // byte [X][1] = #ofbytes requested  */
                /* DAQ item 0x00 uses different bytes */
                uint8 i;
                if(ODT_cnt == 0) //look at var_cnt instead of ODT_rx_byte0 because ODT_rx_byte0 could contain error messages from DAQ, so we might miss storing the data. Can probably look at error message here too.
                {
                    //see excel doc DAQ mode and BMW Faar We SRS.xlsx for daq return string used to build daq table
                    //store variable 0

                    //******NOTE:
                    //      Storing data "as is". print_periodic_variables will store each variable
                    //      in the appropriate order based on if little endian or fray. Will have to
                    //      check logic and proper settings for BMW Faar We.
                    var_idx = 7;
                    //store variable 1
                    for(i=0;i<XCP_write_DAQ_fray_msg_1[i][1];i++)//store # of bytes requested by this daq slot
                    {
                             if(i==0){variable_info[var_cnt].data[0] = xcp_dynamic_rx_data[var_idx];var_idx++;} //7
                             else if(i==1){variable_info[var_cnt].data[1] = xcp_dynamic_rx_data[var_idx];var_idx++;} //8
                             else if(i==2){variable_info[var_cnt].data[2] = xcp_dynamic_rx_data[var_idx];var_idx++;} //9
                             else if(i==3){variable_info[var_cnt].data[3] = xcp_dynamic_rx_data[var_idx];var_idx++;} //10
                    }
                    var_cnt++;
                    //store variable 1
                    for(i=0;i<XCP_write_DAQ_fray_msg_1[i][1];i++)//store # of bytes requested by this daq slot
                    {
                             if(i==0){variable_info[var_cnt].data[0] = xcp_dynamic_rx_data[var_idx];var_idx++;} //11
                             else if(i==1){variable_info[var_cnt].data[1] = xcp_dynamic_rx_data[var_idx];var_idx++;} //12
                             else if(i==2){variable_info[var_cnt].data[2] = xcp_dynamic_rx_data[var_idx];var_idx++;} //13
                             else if(i==3){variable_info[var_cnt].data[3] = xcp_dynamic_rx_data[var_idx];var_idx++;} //14
                    }
                    var_cnt++;
                }
                else if(ODT_cnt == 1) //ODT 1 is 2 bytes past ODT 0
                {
                    //store variable 2
                    var_idx = var_idx + 2; //should be 17 normally
                    for(i=0;i<XCP_write_DAQ_fray_msg_1[i][1];i++)//store # of bytes requested by this daq slot
                    {
                             if(i==0){variable_info[var_cnt].data[0] = xcp_dynamic_rx_data[var_idx];var_idx++;} //17
                             else if(i==1){variable_info[var_cnt].data[1] = xcp_dynamic_rx_data[var_idx];var_idx++;} //18
                             else if(i==2){variable_info[var_cnt].data[2] = xcp_dynamic_rx_data[var_idx];var_idx++;} //19
                             else if(i==3){variable_info[var_cnt].data[3] = xcp_dynamic_rx_data[var_idx];var_idx++;} //20
                    }
                    var_cnt++;
                    //store variable 3
                    for(i=0;i<XCP_write_DAQ_fray_msg_1[i][1];i++)//store # of bytes requested by this daq slot
                    {
                             if(i==0){variable_info[var_cnt].data[0] = xcp_dynamic_rx_data[var_idx];var_idx++;} //21
                             else if(i==1){variable_info[var_cnt].data[1] = xcp_dynamic_rx_data[var_idx];var_idx++;} //22
                             else if(i==2){variable_info[var_cnt].data[2] = xcp_dynamic_rx_data[var_idx];var_idx++;} //23
                             else if(i==3){variable_info[var_cnt].data[3] = xcp_dynamic_rx_data[var_idx];var_idx++;} //24
                    }
                    var_cnt++;
                }
                else //ODTs 2+ are 5 bytes past previous ODT
                {
                    //store variables 4+
                    var_idx = var_idx + 4;
                    for(i=0;i<XCP_write_DAQ_fray_msg_1[i][1];i++)//store # of bytes requested by this daq slot
                    {
                             if(i==0){variable_info[var_cnt].data[0] = xcp_dynamic_rx_data[var_idx];var_idx++;}
                             else if(i==1){variable_info[var_cnt].data[1] = xcp_dynamic_rx_data[var_idx];var_idx++;}
                             else if(i==2){variable_info[var_cnt].data[2] = xcp_dynamic_rx_data[var_idx];var_idx++;}
                             else if(i==3){variable_info[var_cnt].data[3] = xcp_dynamic_rx_data[var_idx];var_idx++;}
                    }
                    var_cnt++;
                    //store variable 5+
                    for(i=0;i<XCP_write_DAQ_fray_msg_1[i][1];i++)//store # of bytes requested by this daq slot
                    {
                             if(i==0){variable_info[var_cnt].data[0] = xcp_dynamic_rx_data[var_idx];var_idx++;}
                             else if(i==1){variable_info[var_cnt].data[1] = xcp_dynamic_rx_data[var_idx];var_idx++;}
                             else if(i==2){variable_info[var_cnt].data[2] = xcp_dynamic_rx_data[var_idx];var_idx++;}
                             else if(i==3){variable_info[var_cnt].data[3] = xcp_dynamic_rx_data[var_idx];var_idx++;}
                    }
                    var_cnt++;
                }
            }
            if(ODT_cnt == number_of_ODTs)
            {
                var_cnt = 0;
                print_data_flag = 1;
            }
        break;

            default:
            break;
        } //end switch(DAQ_mode_step_cntr)
    }

    //PREVIOUSLY VERIFIED - Before copy from HBOX T1XX software, DAQ mode stuff worked great.
    //NOT VERIFIED case 3000 storage of data using for() loop. Previously copied directly.
    void handle_DAQ_notification_CAN (void)
    {
        extern uint8_t can1_rx_data[9];

        switch(DAQ_mode_step_cntr)
        {

        case 2000:    // reply from connect message Que Free DAQ D6

               canTransmit(canREG1,CAN_XCP_DAQ_TX, (uint8 *) XCP_free_DAQ_CAN);
               DAQ_mode_step_cntr = 2010;  // increment for next case

        break;

        case 2010:    // reply from Free DAQ  Que Allocate DAQ list DAQ D5

               canTransmit(canREG1,CAN_XCP_DAQ_TX, (uint8 *) XCP_allocate_DAQ_list_CAN);
               DAQ_mode_step_cntr = 2020;  // increment for next case

        break;

        case 2020:    // reply from Allocate DAQ list DAQ  Que Allocate ODT's D4

               XCP_allocate_ODT_CAN[4] = number_of_DAQ_variables;
               canTransmit(canREG1,CAN_XCP_DAQ_TX, (uint8 *) XCP_allocate_ODT_CAN);
               //DAQ_mode_step_cntr = 2030;  // increment for next case
               DAQ_mode_step_cntr = 2030;  // increment for next case

        break;

        case 2030:    // reply from Allocate ODT's D4  Que XCP_allocate_ODT_entries D3

               canTransmit(canREG1,CAN_XCP_DAQ_TX, (uint8 *) XCP_allocate_ODT_entries_CAN[var_cnt]);

               var_cnt++;

             if (var_cnt == (number_of_DAQ_variables))
             {
                     DAQ_mode_step_cntr = 2070;  // increment for next case
                     var_cnt  = 0;
             }

        break;


        case 2070:    // reply from XCP_allocate_ODT_entries D3  Que XCP_Set_DAQ_pointer E2

                canTransmit(canREG1,CAN_XCP_DAQ_TX, (uint8 *) XCP_Set_DAQ_pointer_CAN[var_cnt]);
               DAQ_mode_step_cntr = 2080;  // go to write DAQ

        break;

        case 2080:    // reply from XCP_Set_DAQ_pointer E2  Que XCP_write_DAQ E1

               canTransmit(canREG1,CAN_XCP_DAQ_TX, (uint8 *) XCP_write_DAQ_CAN[var_cnt]);
               var_cnt++;

               if (var_cnt < (number_of_DAQ_variables))  {

                   DAQ_mode_step_cntr = 2070;  // back to DAQ pointer to load another variable

               }

               else   {
                   var_cnt  = 0;
                   DAQ_mode_step_cntr = 2230;//4030;  // DONE loading variables...go to select DAQ mode
               }

        break;

        case 2230:    // reply from XCP_Set_DAQ_List_mode E0 Que XCP_Set_DAQ_select_mode DE

               canTransmit(canREG1,CAN_XCP_DAQ_TX, (uint8 *) XCP_Set_DAQ_List_mode_CAN);
               DAQ_mode_step_cntr = 2240;//4030;//2240;//4230;  // increment for next case

        break;



        case 2240:    // reply from XCP_Set_DAQ_List_mode E0 Que XCP_Set_DAQ_select_mode DE

               canTransmit(canREG1,CAN_XCP_DAQ_TX, (uint8 *) XCP_Set_DAQ_select_mode_CAN);
               DAQ_mode_step_cntr = 2250;//4240;//3000;  // increment for next case

        break;


        //----------- Start ALL DAQ Tables -------------------------------------------------------------

        case 2250:

            canTransmit(canREG1,CAN_XCP_DAQ_TX, (uint8 *) XCP_Start_all_selcted_DAQs_CAN);
            DAQ_mode_step_cntr = 3000;  // last step
            var_cnt  = 0; // TEMP message counter
        break;

        //----------- Collect and store DAQ -------------------------------------------------------------
       case 3000:

            //use BYTE01 as an indicator

            ODT_rx_byte0 = can1_rx_data[0]; //this is first byte of DAQ response. tells you which DAQ this data pertains to

                /* Store data from each request into DAQ*/
            if(var_cnt < number_of_DAQ_variables)
            {
                    /* DAQ item 0x00 uses different bytes */
                uint8 i;
                if(var_cnt == 0)
                {
                    for(i=0;i<XCP_write_DAQ_CAN[i][2];i++)//store # of bytes requested by this daq slot
                    {
                        variable_info[ODT_rx_byte0].data[i] = can1_rx_data[3+i]; //does data really start at byte[3] for first DAQ slot? Check in t1xx data that I have saved. - PKH 26FEB18
                    }
                    var_cnt++;
                }
                else
                {
                    for(i=0;i<XCP_write_DAQ_CAN[i][2];i++)//store # of bytes requested by this daq slot
                    {
                        variable_info[ODT_rx_byte0].data[i] = can1_rx_data[1+i];
                    }
                    var_cnt++;
                }
            }
            else
            {
                var_cnt = 0;
                print_data_flag = 1;
            }

        break;

        default:
            // do nothing
        break;


        } // end switch DAQ_mode_step_cntr
    }

//----------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------- XCP Fast Rate functions (periodic XCP request mode) ---------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
    void init_XCP_fast_rate(void)
    {
        //not sure this will ever be needed
        if(comm_is_CAN_or_FRAY == CAN)
        {
            init_XCP_build_msg_CAN();
        }
        else if(comm_is_CAN_or_FRAY == FRAY)
        {
            init_XCP_build_msg_fray();
        }
    }

    //VERIFIED - This correctly writes XCP_request_fray_variable[30][64][4] = 08 00 EE 30, 00 00 04 F4, AA AA AA AA for number_of_DAQ_variables
    //-P. Horny 27Feb18
    void init_XCP_build_msg_fray(void)
    {
        //variable_info_1.address = {0x04, 0x90, 0x01, 0x08};
        uint8 cpyArray7[4] = {0x08, 0x00, 0xEE,0x30};
        uint8 cpyArray8[4] = {0x00, 0x00, 0x04,0xF4};

        int i;
        for(i=0;i<number_of_DAQ_variables;i++)
        {
                memcpy(&XCP_request_fray_variable[i][0], &cpyArray7, 4);
                memcpy(&XCP_request_fray_variable[i][1], &cpyArray8, 4);
                memcpy(&XCP_request_fray_variable[i][2], &variable_info[i].address, 4);
        }
    }

    //VERIFIED - This correctly writes XCP_request_CAN_variable[30][8] = F4 04 00 00 AA AA AA AA for number_of_DAQ_variables
    //-P. Horny 27Feb18
    void init_XCP_build_msg_CAN(void)
    {
        //Reference: const unsigned char ccp_PSA_CMP_BattVltg_str[]               = {0xF4,0x04,0x00,0x00,0x08,0x01,0xE4,0x78};
        //Reference: uint8 XCP_request_CAN_variable[30][8] = {0xF4,0x04,0x00,0x00,0x00,0x00,0x00,0x00};
        //Always going to request 4 bytes and print appropriate bytes based on variable_info.type, doing same with flexray
        int i,u;
        if(variable_info[0].address != 0) //check if any variable info addresses were added. Quick way to check how code user is entering variables (constants or using the struct)
        {
            for(i=0;i<number_of_DAQ_variables;i++)
            {
                XCP_request_CAN_variable[i][0] = 0xF4;
                XCP_request_CAN_variable[i][1] = 0x04;
                for(u=0;u<4;u++)
                {
                    XCP_request_CAN_variable[i][4+u] = variable_info[i].address[u];
                }
            }
        }
    }

    //VERIFIED - Previously checked this sends flexray out requests < number_of_DAQ_variables.
    //NOT FULLY VERIFIED -  Not checked if data is copied into variable_info[30].data[9], previously used a DAQ_data[30][8] variable, and was OK, so assuming OK after changing to struct.
    //                      CAN is not verified, but looks OK.
    //-P. Horny 27FEB18
    void handle_XCP_fast_rate_notification(void)
    {
        extern flexrayPacket XCP_TX_struct;
        extern flexrayPacket *XCP_TX;

        extern uint8 fray_variable_data[8];
        extern uint8_t can1_rx_data[9];

        /****************************
         * 1) Check if CAN or FRAY, if index !=0, store can/fray data into variable_info[index-1].data.
         * 2) All cases, transmit XCP next request, increment index.
         * 3) When index = number_of_variables, set print_data_flag = 1 and index = 100(do nothing case)
         * **************************/
        if(XCP_fast_rate_index < number_of_DAQ_variables) //store variables until number_of_DAQ_variables has been met
        {
            if(comm_is_CAN_or_FRAY == CAN)
            {
                if(XCP_fast_rate_index != 0)
                {
                    memcpy(&variable_info[XCP_fast_rate_index-1].data,&can1_rx_data,8);
                }
                canTransmit(canREG1, CAN_XCP_DAQ_TX, ( uint8 *) XCP_request_CAN_variable[XCP_fast_rate_index]);
            }
            else if(comm_is_CAN_or_FRAY == FRAY)
            {
                if(XCP_fast_rate_index != 0)
                {
                    memcpy(&variable_info[XCP_fast_rate_index-1].data,&fray_variable_data,8);
                }
                transmitFlexray(XCP_request_fray_variable[XCP_fast_rate_index],XCP_TX, 3); //3 might have to change depending on outputs...
            }

            XCP_fast_rate_index++;        // bump index so next receive will switch to correct message;
        }

        else if(XCP_fast_rate_index == number_of_DAQ_variables) //last case don't want to send an undefined request, so just store variables
        {
            if(comm_is_CAN_or_FRAY == CAN)
            {
                if(XCP_fast_rate_index != 0)
                {
                    memcpy(&variable_info[XCP_fast_rate_index-1].data,&can1_rx_data,8);
                }
            }
            else if(comm_is_CAN_or_FRAY == FRAY)
            {
                if(XCP_fast_rate_index != 0)
                {
                    memcpy(&variable_info[XCP_fast_rate_index-1].data,&fray_variable_data,8);
                }
            }

            print_data_flag = 1;
            XCP_fast_rate_index = 100; //do nothing case, waiting for rtinotif to kick off next set of requests
        }
        else
        {
            //undefined case for index > number_of_DAQ_variables
        }

    }


    //VERIFIED - This function checks variable_info.type, endian, and comm_is_CAN_or_FRAY to store daq_data into a return_message using sprintf and strcat functions.
    //           Everythign looks OKAY.
    //-P. Horny 28Feb18
    void print_periodic_variables(void)
    {
        #define CAN 0
        #define FRAY 1
        extern int number_of_DAQ_variables;
        extern char endian[15];

        #define OFF 0
        #define ON 1
        extern int debug_flag1;
        extern int debug_flag2;
        extern char return_message[];
        extern char temp_return_message[];
        unsigned char swap[4];  // method 1 var
        int q,r;

        float print_float;
        uint8 print_uint8;
        uint16 print_uint16;
        uint32 print_uint32;
        char print_string[8];

        strcpy(return_message,"daq_data");

            for(q=0;q<number_of_DAQ_variables;q++)
            {
                //float-----Check if .type is float, if yes, check endian/fray, build string as float, strcat to return_message ----------------------------------------------------
                if(strcmp(variable_info[q].type,"Float") == 0){
                    if((strcmp(endian,"little_endian") == 0) || (comm_is_CAN_or_FRAY == FRAY)){
                        for(r=0;r<4;r++){swap[r]= variable_info[q].data[3-r];}
                        memcpy(&print_float, &swap,4);}
                    else{memcpy(&print_float, &variable_info[q].data,4);}

                    sprintf(temp_return_message,",%.3f",print_float);
                    strcat(return_message,temp_return_message);
                }
                //uint8-----Check if .type is uint8, if yes, check endian/fray, build string as integer, strcat to return_message ----------------------------------------------------
                else if((strcmp(variable_info[q].type,"Char") == 0)||(strcmp(variable_info[q].type,"Unsigned Char") == 0)){
                    //if(comm_is_CAN_or_FRAY == FRAY){
                    //  print_uint8 = variable_info[q].data[3];}
                    //else{
                        print_uint8 = variable_info[q].data[0];
                    //}

                    sprintf(temp_return_message,",%d",print_uint8);
                    strcat(return_message,temp_return_message);
                }
                //uint16-----Check if .type is uint16, if yes, check endian/fray, build string as integer, strcat to return_message ----------------------------------------------------
                else if((strcmp(variable_info[q].type,"Short") == 0)||(strcmp(variable_info[q].type,"Unsigned Short") == 0)){
                    if(comm_is_CAN_or_FRAY == FRAY){
                        print_uint16 = (variable_info[q].data[3] << 8) + (variable_info[q].data[2]);}
                    else{print_uint16 = (variable_info[q].data[1] << 8) + (variable_info[q].data[2]);}

                    sprintf(temp_return_message,",%d",print_uint16);
                    strcat(return_message,temp_return_message);
                }
                //uint32-----Check if .type is uint32, if yes, check endian/fray, build string as integer, strcat to return_message ----------------------------------------------------
                else if((strcmp(variable_info[q].type,"Long") == 0)||(strcmp(variable_info[q].type,"Unsigned Long") == 0)){
                    if(comm_is_CAN_or_FRAY == FRAY){
                        print_uint32 =(variable_info[q].data[3] << 24) + (variable_info[q].data[2] << 16) + (variable_info[q].data[1] << 8) +(variable_info[q].data[0]);}
                    else{print_uint32 =(variable_info[q].data[1] << 24) + (variable_info[q].data[2] << 16) + (variable_info[q].data[3] << 8) +(variable_info[q].data[4]);}

                    sprintf(temp_return_message,",%d",print_uint32);
                    strcat(return_message,temp_return_message);
                }
                //default-----Check if .type is default, if yes, check endian/fray, build string as string, strcat to return_message ----------------------------------------------------
                else if(strcmp(variable_info[q].type,"default") == 0){
                    if((strcmp(endian,"little_endian") == 0) || (comm_is_CAN_or_FRAY == FRAY)){
                        for(r=0;r<4;r++){swap[r]= variable_info[q].data[3-r];}
                        memcpy(&print_string, &swap,4);}
                    else{memcpy(&print_string, &variable_info[q].data,4);}

                    sprintf(temp_return_message,",%s",print_string);
                    strcat(return_message,temp_return_message);
                //-----If not any, check endian/fray, assume default, build string as string, strcat to return_message ----------------------------------------------------
                }
                else{
                    if((strcmp(endian,"little_endian") == 0) || (comm_is_CAN_or_FRAY == FRAY)){
                        for(r=0;r<4;r++){swap[r]= variable_info[q].data[3-r];}
                        memcpy(&print_string, &swap,4);}
                    else{memcpy(&print_string, &variable_info[q].data,4);}

                    sprintf(temp_return_message,",%s",print_string);
                    strcat(return_message,temp_return_message);
                }
            } //end for(q=0;q<number_of_DAQ_variables;q++)
    } //end void print_periodic_variables(void)
