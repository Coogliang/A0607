/** @file flexray.h
*   @brief FlexRay Header File
*   @date 30.May.2014
*   @version 04.00.00
*
*   This file contains:
*   - Named Constants
*   - Configuration Data Strucutres (which mimic HalCoGen-set parameters)
*   - Inline Functions that are used frequently in "flexray.c"
*   - Software Driver Function Headers
*/
#ifndef __FLEXRAY_H__
#define __FLEXRAY_H__

#include "reg_flexray.h"

/*
 * @def frayCommIntFlags
 * @brief FlexRay Interrupt Flags.
 *
 * Flags found in the Status Interrupt and Error Interrupt Registers.
 */
enum frayCommIntFlags
{
	/*< Error Interrupt Flags (EIR) >*/
	FRAY_TABB_INT = 0x04000000U, /*< Transmission Across Slot Boundary on CH B >*/
	FRAY_LTVB_INT = 0x02000000U, /*< Latest Transmit Violation on CH B >*/
	FRAY_EDB_INT =  0x01000000U, /*< Error Detected on CH B >*/
	FRAY_TABA_INT = 0x00040000U, /*< Transmission Across Slot Boundary on CH A >*/
	FRAY_LTVA_INT = 0x00020000U, /*< Latest Transmit Violation on CH A >*/
	FRAY_EDA_INT =  0x00010000U, /*< Error Detected on CH A >*/
	FRAY_MHF_INT =  0x00000800U, /*< Message Handler Failure >*/
	FRAY_IOBA_INT = 0x00000400U, /*< Illegal Output Buffer Access >*/
	FRAY_IIBA_INT = 0x00000200U, /*< Illegal Input Buffer Access >*/
	FRAY_EFA_INT  = 0x00000100U, /*< Empty FIFO Access Occured >*/
	FRAY_RFO_INT  = 0x00000080U, /*< Receive FIFO Overrun >*/
	FRAY_PERR_INT = 0x00000040U, /*< Parity Error Detected >*/
	FRAY_CCL_INT  = 0x00000020U, /*< CHI Command Locked >*/
	FRAY_CCF_INT  = 0x00000010U, /*< Clock Correction Failed >*/
	FRAY_SFO_INT  = 0x00000008U, /*< Sync Frame Overflow >*/
	FRAY_SFBM_INT = 0x00000004U, /*< Sync Frames Below Minimum >*/
	FRAY_CNA_INT  = 0x00000002U, /*< CHI Command Not Accepted >*/
	FRAY_PEMC_INT = 0x00000001U, /*< POC Error Mode Changed >*/

	/*< Status Interrupt Flags (SIR) >*/
	FRAY_MTSB_INT = 0x02000000U, /*< Transmission Across Slot Boundary on CH B >*/
	FRAY_WUPB_INT = 0x01000000U, /*< Latest Transmit Violation on CH B >*/
	FRAY_MTSA_INT = 0x00020000U, /*< Error Detected on CH B >*/
	FRAY_WUPA_INT = 0x00010000U, /*< Transmission Across Slot Boundary on CH A >*/
	FRAY_SDS_INT  = 0x00008000U, /*< Latest Transmit Violation on CH A >*/
	FRAY_MBSI_INT = 0x00004000U, /*< Error Detected on CH A >*/
	FRAY_SUCS_INT = 0x00002000U, /*< Message Handler Failure >*/
	FRAY_SWE_INT  = 0x00001000U, /*< Illegal Output Buffer Access >*/
	FRAY_TOBC_INT = 0x00000800U, /*< Illegal Input Buffer Access >*/
	FRAY_TIBC_INT = 0x00000400U, /*< Empty FIFO Access Occured >*/
	FRAY_TI1_INT  = 0x00000200U, /*< Receive FIFO Overrun >*/
	FRAY_TI0_INT  = 0x00000100U, /*< Parity Error Detected >*/
	FRAY_NMVC_INT = 0x00000080U, /*< CHI Command Locked >*/
	FRAY_RFCL_INT = 0x00000040U, /*< Clock Correction Failed >*/
	FRAY_RFNE_INT = 0x00000020U, /*< Sync Frame Overflow >*/
	FRAY_RXI_INT  = 0x00000010U, /*< Sync Frames Below Minimum >*/
	FRAY_TXI_INT  = 0x00000008U, /*< Transmit Successful >*/
	FRAY_CYCS_INT = 0x00000004U, /*< Communication Cycle Started >*/
	FRAY_CAS_INT  = 0x00000002U, /*< CAS Symbol Received >*/
	FRAY_WST_INT  = 0x00000001U /*< Wakeup Status Changed >*/
};

//Message Received
#define frayNOTIFICATION_MSG_RECEIVED 0x00000010U // Message REceived
#define frayNOTIFICATION_BUS_HALTED  0x00000001U  //Bus stopped due to some error
#define frayNOTIFICATION_MSG_TRANSMITTED 0x00000008U
/*
 * The following constants name the commands given to SUCC1.CMD (System Universal Control)
 * which governs the state changes of the finite state machine.  In "flexray.c", these constants
 * are used in place of an explicit register write to improve code's readability and flexibility.
 */
#define CMD_command_not_accepted             0x0
#define CMD_CONFIG                           0x1
#define CMD_READY                            0x2
#define CMD_WAKEUP                           0x3
#define CMD_RUN                              0x4
#define CMD_ALL_SLOTS                        0x5
#define CMD_HALT                             0x6
#define CMD_FREEZE                           0x7
#define CMD_SEND_MTS                         0x8
#define CMD_ALLOW_COLDSTART                  0x9
#define CMD_RESET_STATUS_INDICATORS          0xA
#define CMD_MONITOR_MODE                     0xB
#define CMD_CLEAR_RAMS                       0xC
#define CMD_ASYNCHRONOUS_TRANSFER_MODE       0xE

/*
 * The following constants name the different statuses of the Communcation Controller as reported by the
 * Communication Controller Status Vector (CCSV).  The CCSV register is read-only and these constants are
 * used during FlexRay driver functions to verify and wait that a user-requested change to the SUCC1.CMD
 * vector has succesfully been recognized and performed by the system respectively.
 */
#define CCSV_DEFAULT_CONFIG 0x0
#define CCSV_READY          0x1
#define CCSV_NORMAL_ACTIVE  0x2
#define CCSV_NORMAL_PASSIVE 0x3
#define CCSV_HALT           0x4
#define CCSV_MONITOR_MODE   0x5
#define CCSV_LOOPBACK_MODE  0xD
#define CCSV_CONFIG         0xF
#define CCSV_WAKEUP_STANDBY 0x10
#define CCSV_WAKEUP_LISTEN  0x11
#define CCSV_WAKEUP_SEND    0x12
#define CCSV_WAKEUP_DETECT  0x13
#define CCSV_STARTUP_PREPARE 0x20
#define CCSV_COLDSTART_LISTEN 0x21
#define CCSV_COLDSTART_COLLISION_RESOLUTION 0x22
#define CCSV_COLDSTART_CONSISTENCY_CHECK 0x23
#define CCSV_COLDSTART_GAP 0x24
#define CCSV_COLDSTART_JOIN  0x25
#define CCSV_INTEGRATION_COLDSTART_CHECK 0x26
#define CCSV_INTEGRATION_LISTEN      0x27
#define CCSV_INTEGRATION_CONSISTENCY_CHECK  0x28
#define CCSV_INITIALIZE_SCHEDULE    0x29


#define XCP_RX_INT 0x4000;
#define MFG_RX_INT 0x8000;
/*
 * The following methods are defined inline because they are frequently used during
 * the flexray initializtion and startup routines.
 */

//set_CHI_state(int) accepts an integer, which is intended to be one of the
//constants with the "CMD" prefix, and changes the state of Protocol Operation Controller (POC)
//to that new state.
inline void set_CHI_state(int new_state)
{
	frayCommREG->SUCC1 = (frayCommREG->SUCC1 & 0xFFFFFFF0)| new_state;
}
//wait_on_POC() waits for the SUCC1.PBSY flag to become 0, indicating that the Protocol Operation Controller
//is no longer busy and can accept another command.
inline void wait_on_POC()
{
	while(((frayCommREG->SUCC1) & 0x00000080) != 0);
}
//get_CCSV_state() is generally used in wait statements (i.e. while(get_CCSV_state() != CCSV_READY))
//to verify that the Communcation Controller's state has been changed after a user-initiated state change
//(i.e. set_CHI_state(CMD_READY))
inline int get_CCSV_state()
{
	return frayCommREG->CCSV & 0x0000003F;
}


/**
 * The following structure containts bit values used when writing to the Write Header Section (WRHS1-WRHS3)
 * registers, which can take place both during initialization of the FlexRay Communcation Controller
 * or while the Communcation Controller is already up and running (the bus has been started).
 * This structure was employed to make defining a new message buffer easier and more explicit to the user,
 * as each bit is spelled out in the code as opposed to three register writes (to WRHS1-WRHS3) that require
 * decoding.  This is helpful during troubleshooting, as the message buffer settings are easily visible.
 * The other reason this strucutre was employed was to make the calculation of the Header's Cylcic Redunancy
 * Code easier to port over from example code's header_crc_calc() function.
 */
typedef volatile struct headerSectionConfig
{
	int mbi; /**< Message Buffer Interrupt 0=TXI or RXI is enabled. >**/
	int txm; /**< Transmission Mode 0 = Continuous, 1 = Single-Shot>**/
	int ppit;/**< Payload Preamble Indicator Transmit 1=Buffer holds Network Mngmnt Info >**/
	int cfg; /**< Dircetion Configuration.  0 = Receive, 1 = Transmit >**/
	int chb; /**< Transmit/Receive on Channel B.  1 = Yes >**/
	int cha; /**< Transmit/Receive on Channel A.  1 = Yes >**/
	int cyc; /**< Cycle Filtering Code (6-0). See manual. >**/
	int fid; /**< Frame ID (10-0). >**/
	int plc; /**< Payload Length (6-0). >**/
	int crc; /**< Header CRC (10-0). >**/
	int dp;  /**< Data Pointer (1st word in RAM) (10-0) >**/
	int sync;/**< Sync Frame Indicator >**/
	int sfi; /**< Startup Frame Indicator >**/
} headerSectionConfig;

/*
 * The bufferTransferConfig structure contains bit values used during the transfer of header and payload information from
 * the Input/Output Buffer registers (see Tech Ref Guide) easier and more visible and to explicitly define the
 * settings during each transfer.
 */
typedef volatile struct bufferTransferConfig
{
	int ibrh;  /**< Target Msg Buffer for Trsf from Input Buffer (6-0) >**/
	int stxrh; /**< Set Transmission Request Flag >**/
	int ldsh;  /**< Load Data Section from Input Buffer to RAM >**/
	int lhsh;  /**< Load Header Section from Input Buffer to RAM >**/
	int ibsyh; /**< Wait for Input Buffer to not be busy on shadow side >**/
	int ibsys; /**< Wait for Input Buffer to not be busy on host side >**/
	int obrs;  /**< Source Msg Buffer for Trsf from Ouput Buffer (6-0) >**/
	int rdss;  /**< Read Data Section from RAM to Output Buffer >**/
	int rhss;  /**< Read Header Section from RAM to Output Buffer >**/
} bufferTransferConfig;

typedef volatile struct flexrayPacket
{
    headerSectionConfig header;
    bufferTransferConfig buffer;

} flexrayPacket;

/*< Function Headers >*/
void frayInit();
void frayDisableErrorInterrupt();
void frayEnableErrorInterrupt();
void frayStartCommunication();
void frayRestartCommunication();
void write_header_section(headerSectionConfig *);
void transfer_to_RAM(bufferTransferConfig *);
void transfer_from_RAM(bufferTransferConfig *);
int header_crc_calc(headerSectionConfig *);
void transmitFlexray(uint8 data[64][4], flexrayPacket *pckt, int words);
void initFrayMsg();
void recieveFlexray(uint8 data[64][4],flexrayPacket *pckt);
uint8 BMW_CRC(uint16, uint8 data[4], uint8);



//Interrupt Handler
void frayHighLevelInterrupt(void);

#endif
