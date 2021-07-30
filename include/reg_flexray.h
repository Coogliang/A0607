/** @file reg_flexray.h
*   @brief Register Definitions for the FlexRay Module.
*   @date 30.May.2014
*   @version 04.00.00
*
*   This file contains:
*   - Definitions of Base Adresses and register maps for the FlexRay Transfer Unit Register Set
*     and the FlexRay Communication Controller Register Set
*
*/

#ifndef __REG_FLEXRAY_H__
#define __REG_FLEXRAY_H__

#include "sys_common.h"

/** @struct frayTrsfBASE
 *  @brief FlexRay Transfer Unit Control Register.
 *
 *  This structure is used to access the FlexRay Transfer Unit Register Set.
 */
/** @typedef frayTrsfBASE_t
 *  @brief FlexRay Transfer Unit Register Set Type Definition.
 */
typedef volatile struct frayTrsfBASE
{

	uint32 GSN0;   /**< 0x000 Global Static Number 0 >**/
	uint32 GSN1;   /**< 0x004 Global Static Number 1 >**/
	uint32 rsvd1;  /**< 0x008 Reserved (2) >**/
	uint32 rsvd2;
	uint32 GCS;    /**< 0x010 Global Control Set >**/
	uint32 GCR;    /**< 0x014 Global Control Reset >**/
	uint32 TSCB;   /**< 0x018 Transfer Status >**/
	uint32 LTBCC;  /**< 0x01C Last Trsfd Buffer to Comm. Controller >**/
	uint32 LTBSM;  /**< 0x020 Last Trsfd Buffer to Sys. Memory >**/
	uint32 TBA;    /**< 0x024 Transfer Base Address >**/
	uint32 NTBA;   /**< 0x028 Next Transfer Base Address >**/
	uint32 BAMS;   /**< 0x02C Base Address of Mirrored Status >**/
	uint32 SAMP;   /**< 0x030 Start Address of Memory Protection >**/
	uint32 EAMP;   /**< 0x034 End Address of Memory Protection >**/
	uint32 rsvd3;  /**< 0x038 Reserved (2) >**/
	uint32 rsvd4;
	uint32 TSMO1;  /**< 0x040 Trsf to Sys. Mem. Occured (Buffer 0-31) >**/
	uint32 TSMO2;  /**< 0x044 Trsf to Sys. Mem. Occured (Buffer 32-63) >**/
	uint32 TSMO3;  /**< 0x048 Trsf to Sys. Mem. Occured (Buffer 64-95) >**/
	uint32 TSMO4;  /**< 0x04C Trsf to Sys. Mem. Occured (Buffer 96-127) >**/
	uint32 TCCO1;  /**< 0x050 Trsf to CC Occured (Buffer 0-31) >**/
	uint32 TCCO2;  /**< 0x054 Trsf to CC Occured (Buffer 32-63) >**/
	uint32 TCCO3;  /**< 0x058 Trsf to CC Occured (Buffer 64-95) >**/
	uint32 TCCO4;  /**< 0x05C Trsf to CC Occured (Buffer 36-127) >**/
	uint32 TOOFF;  /**< 0x060 Trsf Occured Offset >**/
	uint32 rsvd5;  /**< 0x064 Reserved (2) >**/
	uint32 rsvd6;
	uint32 PEADR;  /**< 0x070 Parity Error Address >**/
	uint32 TEIR;   /**< 0x074 Trsf Error Interrupt >**/
	uint32 TEIRES; /**< 0x078 Trsf Error Interrupt Enable Set >**/
	uint32 TEIRER; /**< 0x07C Trsf Error Interrupt Enable Reset >**/

	//Trigger Transfers to System Memory
	uint32 TTSMS1; /**< 0x080 Trigger Trsf to Sys. Mem. Set (Buffer 0-31) >**/
	uint32 TTSMR1; /**< 0x084 Trigger Trsf to Sys. Mem. Reset (Buffer 0-31) >**/
	uint32 TTSMS2; /**< 0x088 Trigger Trsf to Sys. Mem. Set (Buffer 32-63) >**/
	uint32 TTSMR2; /**< 0x08C Trigger Trsf to Sys. Mem. Reset (Buffer 32-63) >**/
	uint32 TTSMS3; /**< 0x090 Trigger Trsf to Sys. Mem. Set (Buffer 64-95) >**/
	uint32 TTSMR3; /**< 0x094 Trigger Trsf to Sys. Mem. Reset (Buffer 64-95) >**/
	uint32 TTSMS4; /**< 0x098 Trigger Trsf to Sys. Mem. Set (Buffer 96-127) >**/
	uint32 TTSMR4; /**< 0x09C Trigger Trsf to Sys. Mem. Reset (Buffer 96-127) >**/

	//Trigger Transfers to Comm. Controller
	uint32 TTCCS1; /**< 0x0A0 Trigger Trsf to CC Set (Buffer 0-31) >**/
	uint32 TTCCR1; /**< 0x0A4 Trigger Trsf to CC Reset (Buffer 0-31) >**/
	uint32 TTCCS2; /**< 0x0A8 Trigger Trsf to CC Set (Buffer 32-63) >**/
	uint32 TTCCR2; /**< 0x0AC Trigger Trsf to CC Reset (Buffer 32-63) >**/
	uint32 TTCCS3; /**< 0x0B0 Trigger Trsf to CC Set (Buffer 64-95) >**/
	uint32 TTCCR3; /**< 0x0B4 Trigger Trsf to CC Reset (Buffer 64-95) >**/
	uint32 TTCCS4; /**< 0x0B8 Trigger Trsf to CC Set (Buffer 96-127) >**/
	uint32 TTCCR4; /**< 0x0BC Trigger Trsf to CC Reset (Buffer 96-127) >**/

	//Enable Transfers on Event to Sys. Memory
	uint32 ETESMS1; /**< 0x0C0 Enable Trsf on Event to Sys. Mem. Set (Buffer 0-31) >**/
	uint32 ETESMR1; /**< 0x0C4 Enable Trsf on Event to Sys. Mem. Reset (Buffer 0-31) >**/
	uint32 ETESMS2; /**< 0x0C8 Enable Trsf on Event to Sys. Mem. Set (Buffer 32-63) >**/
	uint32 ETESMR2; /**< 0x0CC Enable Trsf on Event to Sys. Mem. Reset (Buffer 32-63) >**/
	uint32 ETESMS3; /**< 0x0D0 Enable Trsf on Event to Sys. Mem. Set (Buffer 64-95) >**/
	uint32 ETESMR3; /**< 0x0D4 Enable Trsf on Event to Sys. Mem. Reset (Buffer 64-95) >**/
	uint32 ETESMS4; /**< 0x0D8 Enable Trsf on Event to Sys. Mem. Set (Buffer 96-127) >**/
	uint32 ETESMR4; /**< 0x0DC Enable Trsf on Event to Sys. Mem. Reset (Buffer 96-127) >**/

	//Clear on Event to Sys. Memory
	uint32 CESMS1;  /**< 0x0E0 Clear on Event to Sys. Mem. Set (Buffer 0-31) >**/
	uint32 CESMR1;  /**< 0x0E4 Clear on Event to Sys. Mem. Reset (Buffer 0-31) >**/
	uint32 CESMS2;  /**< 0x0E8 Clear on Event to Sys. Mem. Set (Buffer 32-63) >**/
	uint32 CESMR2;  /**< 0x0EC Clear on Event to Sys. Mem. Reset (Buffer 32-63) >**/
	uint32 CESMS3;  /**< 0x0F0 Clear on Event to Sys. Mem. Set (Buffer 64-95) >**/
	uint32 CESMR3;  /**< 0x0F4 Clear on Event to Sys. Mem. Reset (Buffer 64-95) >**/
	uint32 CESMS4;  /**< 0x0F8 Clear on Event to Sys. Mem. Set (Buffer 96-127) >**/
	uint32 CESMR4;  /**< 0x0FC Clear on Event to Sys. Mem. Reset (Buffer 96-127) >**/

	//Transfer to Sys. Memory Interrupt Enable
	uint32 TSMIES1; /**< 0x100 Trsf to Sys. Mem. Int. Enable Set (Buffer 0-31) >**/
	uint32 TSMIER1; /**< 0x104 Trsf to Sys. Mem. Int. Enable Reset (Buffer 0-31) >**/
	uint32 TSMIES2; /**< 0x108 Trsf to Sys. Mem. Int. Enable Set (Buffer 32-63) >**/
	uint32 TSMIER2; /**< 0x10C Trsf to Sys. Mem. Int. Enable Reset (Buffer 32-63) >**/
	uint32 TSMIES3; /**< 0x110 Trsf to Sys. Mem. Int. Enable Set (Buffer 64-95) >**/
	uint32 TSMIER3; /**< 0x114 Trsf to Sys. Mem. Int. Enable Reset (Buffer 64-95) >**/
	uint32 TSMIES4; /**< 0x118 Trsf to Sys. Mem. Int. Enable Set (Buffer 96-127) >**/
	uint32 TSMIER4; /**< 0x11C Trsf to Sys. Mem. Int. Enable Reset (Buffer 96-127) >**/

	//Transfer to Communication Controller Interrupt Enable
	uint32 TCCIES1; /**< 0x120 Trsf to CC Int. Enable Set (Buffer 0-31) >**/
	uint32 TCCIER1; /**< 0x124 Trsf to CC Int. Enable Reset (Buffer 0-31) >**/
	uint32 TCCIES2; /**< 0x128 Trsf to CC Int. Enable Set (Buffer 32-63) >**/
	uint32 TCCIER2; /**< 0x12C Trsf to CC Int. Enable Reset (Buffer 32-63) >**/
	uint32 TCCIES3; /**< 0x130 Trsf to CC Int. Enable Set (Buffer 64-95) >**/
	uint32 TCCIER3; /**< 0x134 Trsf to CC Int. Enable Reset (Buffer 64-95) >**/
	uint32 TCCIES4; /**< 0x138 Trsf to CC Int. Enable Set (Buffer 96-127) >**/
	uint32 TCCIER4; /**< 0x13C Trsf to CC Int. Enable Reset (Buffer 96-127) >**/

}frayTrsfBASE_t;

/** @def frayTrsfREG
*   @brief FlexRay Transfer Unit Register Pointer.
*
*   Pointer used by the FlexRay driver to access the FlexRay Transfer Unit Registers.
*/
#define frayTrsfREG ((frayTrsfBASE_t *)0xFFF7A000U)


/** @struct frayCommBASE
*   @brief FlexRay Communication Controller Register.
*
*   This structure is used to access the FlexRay Communication Controller Registers.
*/
/** @typedef frayCommBASE_t
*   @brief FlexRay Communication Register Frame Type Definition
*
*   This type is used to access the FlexRay Communcation Controller Registers.
*/
typedef volatile struct frayCommBASE
{
	uint32 rsvd1[4];   /**< 0x0000 - 0x000C Reserved >**/

	//Special Registers
	uint32 TEST1;      /**< 0x0010 Test Register 1 >**/
	uint32 TEST2;      /**< 0x0014 Test Register 2 >**/
	uint32 rsvd2;
	uint32 LCK;        /**< 0x001C Lock Register >**/

	//Interrupt Registers
	uint32 EIR;        /**< 0x0020 Error Interrupt Registers >**/
	uint32 SIR;        /**< 0x0024 Status Interrupt Registers >**/
	uint32 EILS;       /**< 0x0028 Error Interrupt Line Select >**/
	uint32 SILS;       /**< 0x002C Status Interrupt Line Select >**/
	uint32 EIES;       /**< 0x0030 Error Interrupt Enable Set >**/
	uint32 EIER;       /**< 0x0034 Error Interrupt Enable Reset >**/
	uint32 SIES;       /**< 0x0038 Status Interrupt Enable Set >**/
	uint32 SIER;       /**< 0x003C Status Interrupt Enable Reset >**/
	uint32 ILE;        /**< 0x0040 Interrupt Line Enable >**/
	uint32 T0C;        /**< 0x0044 Timer 0 Config. >**/
	uint32 T1C;        /**< 0x0048 Timer 1 Config. >**/
	uint32 STPW1;      /**< 0x004C Stop Watch Register 1 >**/
	uint32 STPW2;      /**< 0x0050 Stop Watch Register 2 >**/
	uint32 rsvd3[11];  /**< 0x0054 - 0x007C Reserved >**/


	//Communication Controller Registers
	uint32 SUCC1;      /**< 0x0080 SUC Config. Register 1 >**/
	uint32 SUCC2;      /**< 0x0084 SUC Config. Register 2 >**/
	uint32 SUCC3;      /**< 0x0088 SUC Config. Register 3 >**/
	uint32 NEMC;       /**< 0x008C NEM config. Register >**/
	uint32 PRTC1;      /**< 0x0090 PRT Config. Register 1 >**/
	uint32 PRTC2;      /**< 0x0094 PRT Config. Register 2 >**/
	uint32 MHDC;       /**< 0x0098 MHD Config. Register >**/
	uint32 rsvd4;      /**< 0x009C Reserved >**/
	uint32 GTUC1;      /**< 0x00A0 GTU Config. Register 1 >**/
	uint32 GTUC2;      /**< 0x00A4 GTU Config. Register 2 >**/
	uint32 GTUC3;      /**< 0x00A8 GTU Config. Register 3 >**/
	uint32 GTUC4;      /**< 0x00AC GTU Config. Register 4 >**/
	uint32 GTUC5;      /**< 0x00B0 GTU Config. Register 5 >**/
	uint32 GTUC6;      /**< 0x00B4 GTU Config. Register 6 >**/
	uint32 GTUC7;      /**< 0x00B8 GTU Config. Register 7 >**/
	uint32 GTUC8;      /**< 0x00BC GTU Config. Register 8 >**/
	uint32 GTUC9;      /**< 0x00C0 GTU Config. Register 9 >**/
	uint32 GTUC10;     /**< 0x00C4 GTU Config. Register 10 >**/
	uint32 GTUC11;     /**< 0x00C8 GTU Config. Register 11 >**/
	uint32 rsvd5[13];  /**< 0x00CC - 0x00FC Reserved >**/

	//Communication Controller (CC) Status Registers
	uint32 CCSV;       /**< 0x0100 CC Status Vector >**/
	uint32 CCEV;       /**< 0x0104 CC Error Vector >**/
	uint32 rsvd6[2];   /**< 0x0108-0x010C Reserved >**/
	uint32 SCV;        /**< 0x0110 Slot Counter Value >**/
	uint32 MTCCV;      /**< 0x0114 Macrotick and Cycle Counter Value >**/
	uint32 RCV;        /**< 0x0118 Rate Correction Value >**/
	uint32 OCV;        /**< 0x011C Offset Correction Value >**/
	uint32 SFS;        /**< 0x0120 Sync Frame Status >**/
	uint32 SWNIT;      /**< 0x0124 Symbol Window and NIT Status >**/
	uint32 ACS;        /**< 0x0128 Aggregated Channel Status >**/
	uint32 rsvd7;      /**< 0x012C Reserved >**/
	uint32 ESID[15];   /**< 0x0130 - 0x0168 Even Sync ID >**/
	uint32 rsvd8;      /**< 0x016C Reserved >**/
	uint32 OSID[15];   /**< 0x0170 - 0x0178 Odd Sync ID >**/
	uint32 rsvd9;  	   /**< 0x01AC Reserved >**/
	uint32 NMV[3];     /**< 0x01B0 - 0x01B8 Network Management Vector >**/
	uint32 rsvd10[81]; /**< 0x01BC - 0x02FC Reserved >**/

	//Message Buffer Control Registers
	uint32 MRC;        /**< 0x0300 Message RAM Config. >**/
	uint32 FRF;        /**< 0x0304 FIFO Rejection Filter >**/
	uint32 FRFM;       /**< 0x0308 FIFO REjection Filter Mask >**/
	uint32 FCL;        /**< 0x030C FIFO Critical Level >**/

	//Message Buffer Status Registers
	uint32 MHDS;       /**< 0x0310 Message Handler Status >**/
	uint32 LDTS;       /**< 0x0314 Last Dynamic Transmit Slot >**/
	uint32 FSR;        /**< 0x0318 FIFO Status Register >**/
	uint32 MHDF;       /**< 0x031C Message Handler Constraint Flags >**/
	uint32 TXRQ1;      /**< 0x0320 Trasmission Request 1 >**/
	uint32 TXRQ2;      /**< 0x0324 Trasmission Request 2 >**/
	uint32 TXRQ3;      /**< 0x0328 Trasmission Request 3 >**/
	uint32 TXRQ4;      /**< 0x032C Trasmission Request 4 >**/
	uint32 NDAT1;      /**< 0x0330 New Data 1 >**/
	uint32 NDAT2;      /**< 0x0334 New Data 2 >**/
	uint32 NDAT3;      /**< 0x0338 New Data 3 >**/
	uint32 NDAT4;      /**< 0x033C New Data 4 >**/
	uint32 MBSC1;      /**< 0x0340 Message Buffer Status Changed 1 >**/
	uint32 MBSC2;      /**< 0x0344 Message Buffer Status Changed 2 >**/
	uint32 MBSC3;      /**< 0x0348 Message Buffer Status Changed 3 >**/
	uint32 MBSC4;      /**< 0x034C Message Buffer Status Changed 4 >**/
	uint32 rsvd11[40]; /**< 0x0350 - 0x03EC Reserved >**/

	//Identification Registers
	uint32 CREL;       /**< 0x03F0 Core Release Register >**/
	uint32 ENDN;       /**< 0x03F4 Endian Register >**/
	uint32 rsvd12[2];  /**< 0x03F8 - 0x03FC Reserved >**/

	//Input Buffer
	uint32 WRDS[64];   /**< 0x0400 - 0x04FC Write Data Section >**/
	uint32 WRHS1;      /**< 0x0500 Write Header Section 1 >**/
	uint32 WRHS2;      /**< 0x0504 Write Header Section 2 >**/
	uint32 WRHS3;      /**< 0x0508 Write Header Section 3 >**/
	uint32 rsvd13;     /**< 0x050C Reserved >**/
	uint32 IBCM;       /**< 0x0510 Input Buffer Command Mask >**/
	uint32 IBCR;       /**< 0x0514 Input Buffer Command Request >**/
	uint32 rsvd14[58]; /**< 0x0518 - 0x05FC Reserved >**/

	//Output Buffer
	uint32 RDDS[64];   /**< 0x0600 - 0x06FC Read Data Section >**/
	uint32 RDHS1;      /**< 0x0700 Read Header Section 1 >**/
	uint32 RDHS2;      /**< 0x0704 Read Header Section 2 >**/
	uint32 RDHS3;      /**< 0x0708 Read Header Section 3 >**/
	uint32 MBS;        /**< 0x070C Message Buffer Status >**/
	uint32 OBCM;       /**< 0x0710 Output Buffer Command Mask >**/
	uint32 OBCR;       /**< 0x0714 Output Buffer Command Request >**/
	uint32 rsvd15[58]; /**< 0x0718 - 0x07FC Reserved >**/

}frayCommBASE_t;


/** @def frayCommREG
*   @brief FlexRay Communcation Controller Register Pointer.
*
*   Pointer used by the FlexRay driver to access the FlexRaay Communication Controller Register.
*/
#define frayCommREG ((frayCommBASE_t *)0xFFF7C800U)

#endif
