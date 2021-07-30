/*
 * target_product.c
 *
 *  Created on: Jan 19, 2018
 *      Author: TZZ176
 */
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <limits.h>

#include "string.h"

/****************************************************
 * target_product.c
 *
 * Currently not in use.
 *
 * Currently nothing should be dependent on definitions here.
 *
 * Goal will be to define target product, endian, processor type, CAN/FRAY
 *
 *
 * P. Horny
 * 27Feb18
 *
 * */


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
#define TARGET_GACA_90 20
#define TARGET_GACA_26 21

char print_available_personalities[] ={
"gm_a_t1xx rev01\r\n\
9bxx rev01\r\n\
s550analog rev01\r\n\
s550digital rev01\r\n\
c1xx rev01\r\n\
fca_adas rev01\r\n\
cd391_adas rev01\r\n\
g2kca_adas rev01\r\n\
psa_cmp rev01\r\n\
sgmw_cn200 rev01\r\n\
ford_t3_t6 rev01\r\n\
gm_b_t1xx rev01\r\n\
renault_nissan rev01\r\n\
k2xx_mto rev01\r\n\
t1xx_mto rev01\r\n\
bmw_faar_we rev01\r\n\
bmw_ukl rev01\r\n\
ehps rev01\r\n\
sgmw_cn300 rev01\r\n"};

/*
char print_available_personalities[] ={
"t1xx_global_a rev01\r\n\
9bxx rev01\r\n\
s550analog rev01\r\n\
s550digital rev01\r\n\
c1xx rev01\r\n\
fca_adas rev01\r\n\
cd391_adas rev01\r\n\
g2kca_adas rev01\r\n\
psa_cmp rev01\r\n\
sgmw_cn200 rev01\r\n\
ford_t3_t6 rev01\r\n\
bmw_ukl rev01\r\n\
t1xx_global_b rev01\r\n\
bmw_faar_we rev01\r\n"};
*/

//char personalities[][40] = {
//"t1xx_global_b rev01",	  	/* 0 */
//"9bxx rev01",		  		/* 1 */
//"s550analog rev01",  		/* 2 */
//"s550digital rev01",	  	/* 3 */
//"c1xx rev01",	  			/* 4 */
//"fca_adas rev01",	  		/* 5 */
//"cd391_adas rev01",     	/* 6 */
//"g2kca_adas rev01",     	/* 7 */
//"psa_cmp rev01",     		/* 8 */
//"sgmw_cn200 rev01",	  		/* 9 */
//"ford_t3_t6 rev01",     	/* 10 */
//"bmw_ukl rev01",     		/* 11 */
//"t1xx_global_b rev01",	  	/* 12 */
//"bmw_faar_we rev01"};     	/* 13 */

//#define NUM_OF_PRODUCTS 14

char personalities[][40] = {
"gm_a_t1xx rev01",      /* 0 */
"9bxx rev01",               /* 1 */
"s550analog rev01",         /* 2 */
"s550digital rev01",        /* 3 */
"c1xx rev01",               /* 4 */
"fca_adas rev01",           /* 5 */
"cd391_adas rev01",         /* 6 */
"g2kca_adas rev01",         /* 7 */
"psa_cmp rev01",            /* 8 */
"sgmw_cn200 rev01",         /* 9 */
"ford_t3_t6 rev01",         /* 10 */
"gm_b_t1xx rev01",          /* 11 */
"renault_nissan rev01",     /* 12 */
"k2xx_mto rev01",           /* 13 */
"t1xx_mto rev01",           /* 14 */
"bmw_faar_we rev01",        /* 15 */
"bmw_ukl rev01",            /* 16 */
"ehps rev01",               /* 17 */
"sgmw_cn300 rev01"};        /* 18 */


#define NUM_OF_PRODUCTS 19


extern int target_product;

char target_product_str[50];
#define CAN 0
#define FRAY 1
extern int comm_is_CAN_or_FRAY;
extern char endian[15];
extern char target_processor[15];

//This function is currently called in variable_input.c after call to parse_config_input()
// Normal CIB mode won't need this function.
void set_target_product_parameters(void)
{
	int product_select;
	for(product_select = 0; product_select < NUM_OF_PRODUCTS; product_select++)
	{
		if(((strcmp(target_product_str,personalities[product_select])) == 0))
		{
			break;
		}
	}

	switch(product_select+1)
	{
	case TARGET_T1XX:

		target_product = TARGET_T1XX;
		comm_is_CAN_or_FRAY = CAN;
		strcpy(endian,"big_endian");
		strcpy(target_processor,"tms570");

	break;
	case TARGET_9BXX:

		target_product = TARGET_9BXX;
		comm_is_CAN_or_FRAY = CAN;
		strcpy(endian,"big_endian");
		strcpy(target_processor,"tms570");

	break;
	case TARGET_S550ANALOG:

		target_product = TARGET_S550ANALOG;
		comm_is_CAN_or_FRAY = CAN;
		strcpy(endian,"big_endian");
		strcpy(target_processor,"tms570");

	break;
	case TARGET_S550DIGITAL:

		target_product = TARGET_S550DIGITAL;
		comm_is_CAN_or_FRAY = CAN;
		strcpy(endian,"big_endian");
		strcpy(target_processor,"tms570");

	break;
	case TARGET_C1XX:

		target_product = TARGET_C1XX;
		comm_is_CAN_or_FRAY = CAN;
		strcpy(endian,"big_endian");
		strcpy(target_processor,"tms570");

	break;
	case TARGET_FCA_ADAS:

		target_product = TARGET_FCA_ADAS;
		comm_is_CAN_or_FRAY = CAN;
		strcpy(endian,"big_endian");
		strcpy(target_processor,"tms570");

	break;
	case TARGET_CD391_ADAS:

		target_product = TARGET_CD391_ADAS;
		comm_is_CAN_or_FRAY = CAN;
		strcpy(endian,"big_endian");
		strcpy(target_processor,"tms570");

	break;
	case TARGET_G2KCA_ADAS:

		target_product = TARGET_G2KCA_ADAS;
		comm_is_CAN_or_FRAY = CAN;
		strcpy(endian,"little_endian");
		strcpy(target_processor,"renasis");

	break;
	case TARGET_PSA_CMP:

		target_product = TARGET_PSA_CMP;
		comm_is_CAN_or_FRAY = CAN;
		strcpy(endian,"big_endian");
		strcpy(target_processor,"tms570");

	break;
	case TARGET_SGMW_CN200:

		target_product = TARGET_SGMW_CN200;
		comm_is_CAN_or_FRAY = CAN;
		strcpy(endian,"little_endian");
		strcpy(target_processor,"renasis");

	break;
	case TARGET_FORD_T3_T6:

		target_product = TARGET_FORD_T3_T6;
		comm_is_CAN_or_FRAY = CAN;
		strcpy(endian,"little_endian");
		strcpy(target_processor,"renasis");

	break;
    case TARGET_GM_B_T1XX:

        target_product = TARGET_GM_B_T1XX;
        comm_is_CAN_or_FRAY = CAN;
        strcpy(endian,"little_endian");
        strcpy(target_processor,"renasis");

    break;
	case TARGET_RENAULT_NISSAN:

		target_product = TARGET_RENAULT_NISSAN;
		comm_is_CAN_or_FRAY = CAN;
		strcpy(endian,"little_endian");
		strcpy(target_processor,"renasis");

	break;
    case TARGET_K2XX_MTO:

        target_product = TARGET_K2XX_MTO;
        comm_is_CAN_or_FRAY = CAN;
        strcpy(endian,"big_endian");
        strcpy(target_processor,"tms570");

    break;
    case TARGET_T1XX_MTO:

        target_product = TARGET_T1XX_MTO;
        comm_is_CAN_or_FRAY = CAN;
        strcpy(endian,"big_endian");
        strcpy(target_processor,"tms570");

    break;
    case TARGET_BMW_FAAR_WE:

        target_product = TARGET_BMW_FAAR_WE;
        comm_is_CAN_or_FRAY = FRAY;
        strcpy(endian,"little_endian");
        strcpy(target_processor,"renasis");

    break;
	case TARGET_BMW_UKL:

		target_product = TARGET_BMW_UKL;
		comm_is_CAN_or_FRAY = FRAY;
		strcpy(endian,"big_endian");
		strcpy(target_processor,"tms570");

	break;
    case TARGET_EHPS:

        target_product = TARGET_EHPS;
        comm_is_CAN_or_FRAY = CAN;
        strcpy(endian,"big_endian");
        strcpy(target_processor,"tms570");

    break;
    case TARGET_SGMW_CN300:

        target_product = TARGET_SGMW_CN300;
        comm_is_CAN_or_FRAY = CAN;
        strcpy(endian,"big_endian");
        strcpy(target_processor,"tms570");

    break;


	} //end switch(target_product)
} //end void set_target_product_parameters
