/*
 * variable_input.c
 *
 *  Created on: Jan 11, 2018
 *      Author: TZZ176
 */

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <limits.h>

#include "hal_stdtypes.h"
#include "DAQ_mode.h"
#include "sci.h"
#include "string.h"
#include "reg_sci.h"
#include "reg_spi.h"
#include "gio.h"
#include "het.h"


#include "DAQ_mode.h"
#include "flexray.h"
#include "variable_input.h"
#include "target_product.h"

/******************************************************************************************************************************
 * variable_input.c
 *
 *
 * Currently NOT in use.
 *
 * Some functions and flags are set in sys_main, those do not need to be carried over at the moment.
 * Should be...
 * read_variables_flag in sys_main.c and notification.c
 * read_variables() function in notification.c
 * parse_config_input() function in notification.c
 *
 *
 *Use:
 * 1) set_header_variables command received, flag in notification.c is set so next set of data is the header variable string
 * 2) read_variables() handles the storage of the comma delimited string into read_variables_buf
 * 		Format:
 * 		software rev,#ofvars,variable name 1, variable name 2,...address 1, address 2,...type 1, type 2...
 * 			(type = uint8, uint16, uint32, float)
 *
 * 3) Read \r, call parse_config_input() to parse variables
 * 		a) "software rev" is loaded into set_target_product_parameters() to set target_product, comm_is_CAN_or_FRAY, big/little_endian, processor type
 * 				Eventually, this will track the "personalities and revisions of personalities" available in the current software. Basically, multiple dumpver list.
 * 		b) #ofvars used to determine how many variables and addresses to load
 * 		c) variable name, address, and type are parsed into struct variable_input variable_info[30];
				struct variable_input
				{
					char name[100];
					uint8 address[4];
					uint8 data[4];
					char type[15];
				};
		d) Storing of addresses into struct is done based on big/little endian and CAN/FRAY
 *
 ******************************************************************************************************************************/


/*******************************/
/*Dependents *******************/
/*******************************/
#define CAN 0
#define FRAY 1
extern int comm_is_CAN_or_FRAY;
extern char endian[15];
extern char target_processor[15];
extern char target_product_str[50];
extern struct variable_input {
	char name[100];
	uint8 address[4];
	uint8 data[9];
	char type[15];
} variable_info[30];
extern unsigned int DAQ_slow_or_standard;
/*******************************/
char read_config_buf[5070];
char stored_config_buf[5070];

char parsed_XCP_names[30][100];
uint8 parsed_XCP_addr[30][4];
char parsed_XCP_types[30][20];

extern int number_of_DAQ_variables;

int read_variables_flag = 0;
int rx_cntr = 0;


void set_header_variables(void)
{

}

void get_config(void)
{
	transmit_str_485(stored_config_buf);
}

void read_variables(void)
{
	if(rx_cntr < 5070) //dont let buffer overflow if no \r is sent for 5070 bytes
	{
		read_config_buf[rx_cntr++] = scilinREG->RD;
		if(scilinREG->RD == 0x0d)  /* if carrage return rec, parse data */
		{
			rx_cntr = 0;
			parse_config_input();
			read_variables_flag = 0;
			memset(read_config_buf,0,5070);  /* clears command buffer for next message */
		}
	}
	else  // rx_cntr >= 5070
	{
		rx_cntr=0;
		memset(read_config_buf,0,5070);  /* clears command buffer for next message */
		read_variables_flag = 0;
	}
}

void parse_config_input(void)
{
	const char comma[] = ",";
	char token[30][100];
	char temp[10];
	char *token_ptr;
	#define OFF 0
	#define ON 1
	extern int debug_flag1;

	if(debug_flag1 == OFF)
	{
		extern int debug_flag4;
		if(debug_flag4 == ON)
		{
		//char test[500] = 	"bmw_ukl rev01,14,"\
							"Batt_Volt_str,Batt_Curr_str,Mot_Curr_str,Mot_Vel_str,Comm_Torque_str,Lim_Torque_str,PCB_Temp_str,Mot_Temp_str,Junction_Temp_str,HW_Angle_str,Diff_Torque_str,sys_state_byt,T1_Volt_str,T2_Volt_str,"\
							"08019004,080188A0,08019094,0801A2AC,0801A1F8,0801A1EC,0801900C,080190C0,080190BC,0801A1DC,08019030,0801A3C6,08019074,08019078,"\
							"float,float,float,float,float,float,float,float,float,float,float,uint8,float,float\r";
			char test[500] = 	"1,100,30,"\
								"Variable Header,3,"\
										"Rte_BattVltg_BrdgVltg_Val,0xfebec2c0,Float,"\
										"Rte_TEstimn_MotFetT_Val,0xfebf4b80,Float,"\
										"Rte_SysStMod_SysSt_Val,0xfebec628,Char,"\
								"Defeat Header,"\
								"\r";


			//char test[1500] = 	"bmw_faar_we rev01,22,"\
										"_Rte_SysStMod_SysSt_Val,_Rte_CDD_MotCtrlMgr_MotCurrQax_Val,_Rte_CDD_HwTq4Meas_HwTq4_Val,_Rte_CDD_HwTq5Meas_HwTq5_Val,"\
										"_Rte_CDD_Bmw5441McuCfg_EcuT_Val,_Rte_CmnMfgSrv_HwAg_Val,_Rte_CmnMfgSrv_HwAg0_Val,_Rte_CmnMfgSrv_HwAg1_Val,"\
										"_Rte_HwTqArbn_HwTq_Val,_Rte_CDD_MotCtrlMgr_MotCurrDax_Val,_Rte_CDD_MotAgCmp_MotAgCmpMotCtrlMotAgMeclPrev,_Rte_CDD_Bmw5441McuCfg_BattVltg_Val,"\
										"_Rte_CDD_Bmw5441McuCfg_BattVltgSwd1_Val,_Rte_BattVltg_BrdgVltg_Val,_Rte_Assi_AssiCmdBas_Val,_Rte_AssiSumLim_MotTqCmd_Val,"\
										"_Rte_TEstimn_AssiMechT_Val,_Rte_TEstimn_MotFetT_Val,_Rte_TEstimn_MotMagT_Val,_Rte_TEstimn_MotWidgT_Val,"\
										"_Rte_MotCtrlPrmEstimn_MotREstimd_Val,_Rte_PwrLimr_MotTqCmdPwrLimd_Val,"\
										"febec01f,febebda4,febebd28,febebd2c,febebd20,febebdcc,febebdd0,febebdd4,"\
										"febebe20,febebda0,febf1240,febebd18,febebd1c,febebcc0,febf2278,febebc9c,"\
										"febf20d0,febf20d8,febf20dc,febf20e0,febebe40,febebeb8,"\
										"uint16,float,float,float,float,float,float,float,"\
										"float,float,float,float,float,float,float,float,"\
										"float,float,float,float,float,float\r";

			memcpy(read_config_buf,test,strlen(test));
			memcpy(stored_config_buf,read_config_buf,strlen(read_config_buf)); //IN THE FUTURE will include write to FEE
		}
		else
		{
			memcpy(stored_config_buf,read_config_buf,strlen(read_config_buf));
		}

		//NEW FORMAT:
		// personality,
		// 0/1 = slow or standard DAQ,
		// #ms = daq rate, 10ms or higher,
		// #seconds = NTC rate, 1 second or higher,
		// Variable Header,
		// #ofvars,
		// name0,addr0,type0,
		// name1,addr1,type1,...
		// Defeats Header


		int buf_length = strlen(read_config_buf);
		/***********************************************/
		/* Get personality type*/
		/***********************************************/
		//token_ptr = strtok(read_config_buf, comma);
		//strcpy(target_product_str,token_ptr);
		//set_target_product_parameters();

		/***********************************************/
		/* Set DAQ type 0=slow or 1=standard */
		/***********************************************/
		token_ptr = strtok(read_config_buf, comma);//token_ptr = strtok(NULL,comma);
		DAQ_slow_or_standard = atoi(token_ptr);

		/***********************************************/
		/* Set DAQ rate in ms */
		/***********************************************/
		token_ptr = strtok(NULL,comma);
		int temp_period = atoi(token_ptr);
		extern int daq_period_limit,xcp_fast_rate_period_limit,Set_DAQ_List_mode_DAQ_rate,XCP_fast_rate_period;
		if((temp_period >= daq_period_limit) && (DAQ_slow_or_standard == 1))//code to set some limit in the future
		{
			Set_DAQ_List_mode_DAQ_rate = temp_period;
		}
		if((temp_period >= xcp_fast_rate_period_limit) && (DAQ_slow_or_standard == 0))
		{
			XCP_fast_rate_period = temp_period;
		}
		/***********************************************/
		/* Set NTC rate in seconds */
		/***********************************************/
		token_ptr = strtok(NULL,comma);
		extern int NTC_get_period;
		NTC_get_period = atoi(token_ptr) * 1000; //store it in milliseconds

		/***********************************************/
		/* Read Variable Header, do nothing with it so far */
		/***********************************************/
		token_ptr = strtok(NULL,comma);

		/***********************************************/
		/* Get total # of variables */
		/***********************************************/
		token_ptr = strtok(NULL, comma);
		number_of_DAQ_variables = atoi(token_ptr);

		/***********************************************/
		/* Parse and store variable info in this order:
		 * 	name0,addr0,type0,name1,addr1,type1...	*/
		/***********************************************/
		int i,j,k = 0;
		extern char temp_address[30][8];
		for(j=0;j<number_of_DAQ_variables;j++)
		{
			for(i=0;i<3;i++)
			{
				if(i==0) //names
				{
					token_ptr = strtok(NULL, comma);
					strcpy(variable_info[j].name,token_ptr);
				}
				else if(i==1) //addresses
				{
					//LEFT OFF RIGHT HERE
					token_ptr = strtok(NULL, comma);
					strcpy(token[j],token_ptr);
					//for(k=0;k<8;k++)
					//{
					//	temp_address[j][k] = token[j][k+2];
					//}
					memcpy(temp_address[j],token[j],8);
				}
				else if(i==2) //types
				{
					token_ptr = strtok(NULL, comma);
					strcpy(variable_info[j].type,token_ptr);
				}
			}
		}

		swap_addresses_based_on_type();

		/***********************************************/
		/* Read Defeat Header, do nothing with it so far */
		/***********************************************/
		token_ptr = strtok(NULL,comma);

		//j=0;
		//handle very last message, since no comma for strtok() to point to.
		//while(read_variables_buf[buf_length-j-1] != NULL) //strlen(read_variables_buf)
		//{
		//	j++;
		//}
		//memcpy(&variable_info[number_of_DAQ_variables-1].type,&read_variables_buf[buf_length-j],j-1);

	} //end if(debug_flag1 == OFF)
	//Below manually copies some data into arrays to verify parsing is correct. All has been verified, can probably remove this.
	else //debug_flag1 == ON
	{
		comm_is_CAN_or_FRAY = FRAY;
		strcpy(endian,"big_endian");
		number_of_DAQ_variables = 14;
		char addr[200] = "08019004,080188A0,08019094,0801A2AC,0801A1F8,0801A1EC,0801900C,080190C0,080190BC,0801A1DC,08019030,0801A3C4,08019074,08019078";



		/*************************************************/
		/* 1) Use strtok to parse input string			 */
		/*************************************************/
		/* get the first token */
		token_ptr = strtok(addr, comma);

		/* walk through other tokens */
		int j = 0;
		while( token_ptr != NULL ) {
			   strcpy(token[j],token_ptr);
			   j++;
			   token_ptr = strtok(NULL, comma);
		}

		/***********************************************/
		/* 2) NEED TO DECIDE HOW string will be sent */
		/* 		Choices are:
		 * 			A) name,name...addr,addr...endian,endian...type,type
		 * 			B) name,addr,endian,type,name,addr,endian,type...
		 */
		/***********************************************/

		/***********************************************/
		/* 3) Store name strings to variable_info.name */
		/***********************************************/

		memcpy(variable_info[0].name, "Batt_Volt_str",strlen("Batt_Volt_str"));
		memcpy(variable_info[1].name, "Batt_Curr_str",strlen("Batt_Curr_str"));
		memcpy(variable_info[2].name, "Mot_Curr_str",strlen("Mot_Curr_str"));
		memcpy(variable_info[3].name, "Mot_Vel_str",strlen("Mot_Vel_str"));
		memcpy(variable_info[4].name, "Comm_Torque_str",strlen("Comm_Torque_str"));

		memcpy(variable_info[5].name, "Lim_Torque_str",strlen("Lim_Torque_str"));
		memcpy(variable_info[6].name, "PCB_Temp_str",strlen("v"));
		memcpy(variable_info[7].name, "Mot_Temp_str",strlen("Mot_Temp_str"));
		memcpy(variable_info[8].name, "Junction_Temp_str",strlen("Junction_Temp_str"));
		memcpy(variable_info[9].name, "HW_Angle_str",strlen("HW_Angle_str"));

		memcpy(variable_info[10].name, "Diff_Torque_str",strlen("Diff_Torque_str"));
		memcpy(variable_info[11].name, "sys_state_byt",strlen("sys_state_byt"));
		memcpy(variable_info[12].name, "T1_Volt_str",strlen("T1_Volt_str"));
		memcpy(variable_info[13].name, "T2_Volt_str",strlen("T2_Volt_str"));

		//uint32 token32 = (uint32)atoi(token[0]);
		//uint8 token8[4];
		//token8[3] = (uint8)((token32 << 24) >> 24);
		//token8[2] = (uint8)((token32 << 16) >> 24);
		//token8[1] = ((token32 << 8) >> 24);
		//token8[0] = (token32 >> 24);

		/*******************************************************************************************/
		/* 4) Convert and store address string to 4 byte uint8 strings in variable_info.address[4] */
		/*******************************************************************************************/
		int u,l,k = 0;

		if(comm_is_CAN_or_FRAY == CAN){		//&& (endian = BIG_ENDIAN)
			for(k=0;k<number_of_DAQ_variables;k++)
			{u=0;l=0;
				for(j=0;j<strlen(token[k]);j++){
					if(((j % 2) == 0)||(j==0)){
						variable_info[k].address[u] = atoh(token[k][j]);
						u++;
					}
					else{
						variable_info[k].address[l] = (variable_info[k].address[l] << 4) + atoh(token[k][j]);
						l++;
					}
				} //end for(j=0...
			} //end for(k=0...
		}//end if(comm_is_CAN_or_FRAY != FRAY)
		else if(comm_is_CAN_or_FRAY == FRAY)//comm is FRAY, so store addresses in reverse order. I'll do the same for Endianness
		{									// || (endian = LITTLE_ENDIAN)
			for(k=0;k<number_of_DAQ_variables;k++)
			{u=3;l=3;
				for(j=0;j<strlen(token[k]);j++){
					if(((j % 2) == 0)||(j==0)){
						variable_info[k].address[u] = atoh(token[k][j]);
						u--;
					}
					else{
						variable_info[k].address[l] = (variable_info[k].address[l] << 4) + atoh(token[k][j]);
						l--;
					}
				} //end for(j=0...
			} //end for(k=0...
		}//end if(comm_is_CAN_or_FRAY != FRAY)

	}//else debug_flag == OFF
}
