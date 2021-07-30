/** @file spi.c 
*   @brief SPI Driver Implementation File
*   @date 02-Mar-2016
*   @version 04.05.02
*
*/

/* 
* Copyright (C) 2009-2016 Texas Instruments Incorporated - www.ti.com  
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


/* USER CODE BEGIN (0) */
/* USER CODE END */

#include "spi.h"

/* USER CODE BEGIN (1) */
/* USER CODE END */

/** @fn void spiInit(void)
*   @brief Initializes the SPI Driver
*
*   This function initializes the SPI module.
*/
void spiInit(void)
{
sint32 i ;

/* USER CODE BEGIN (2) */
/* USER CODE END */
    /** @b initialize @b SPI1 */

    /** bring SPI out of reset */
    spiREG1->GCR0 = 1U;

    /** enable SPI1 multibuffered mode and enable buffer RAM */
    spiREG1->MIBSPIE = (spiREG1->MIBSPIE & 0xFFFFFFFEU) | 1U;

    /** SPI1 master mode and clock configuration */
    spiREG1->GCR1 = (spiREG1->GCR1 & 0xFFFFFFFCU) | ((1U << 1U)  /* CLOKMOD */
                  | 1U);  /* MASTER */

    /** SPI1 enable pin configuration */
    spiREG1->INT0 = (spiREG1->INT0 & 0xFEFFFFFFU) | (0U << 24U);  /* ENABLE HIGHZ */

    /** - Delays */
    spiREG1->DELAY = (0U << 24U)  /* C2TDELAY */
                   | (0U << 16U)  /* T2CDELAY */
                   | (0U << 8U)  /* T2EDELAY */
                   | 0U;  /* C2EDELAY */

    /** - Data Format 0 */
    spiREG1->FMT0 = (0U << 24U)  /* wdelay */
                  | (0U << 23U)  /* parity Polarity */
                  | (0U << 22U)  /* parity enable */
                  | (0U << 21U)  /* wait on enable */
                  | (0U << 20U)  /* shift direction */
                  | (1U << 17U)  /* clock polarity */
                  | (0U << 16U)  /* clock phase */
                  | (79U << 8U) /* baudrate prescale */
                  | 8U;  /* data word length */

    /** - Data Format 1 */
    spiREG1->FMT1 = (0U << 24U)  /* wdelay */
                  | (0U << 23U)  /* parity Polarity */
                  | (0U << 22U)  /* parity enable */
                  | (0U << 21U)  /* wait on enable */
                  | (0U << 20U)  /* shift direction */
                  | (0U << 17U)  /* clock polarity */
                  | (0U << 16U)  /* clock phase */
                  | (79U << 8U) /* baudrate prescale */
                  | 16U;  /* data word length */

    /** - Data Format 2 */
    spiREG1->FMT2 = (0U << 24U)  /* wdelay */
                  | (0U << 23U)  /* parity Polarity */
                  | (0U << 22U)  /* parity enable */
                  | (0U << 21U)  /* wait on enable */
                  | (0U << 20U)  /* shift direction */
                  | (0U << 17U)  /* clock polarity */
                  | (0U << 16U)  /* clock phase */
                  | (79U << 8U) /* baudrate prescale */
                  | 16U;  /* data word length */

    /** - Data Format 3 */
    spiREG1->FMT3 = (0U << 24U)  /* wdelay */
                  | (0U << 23U)  /* parity Polarity */
                  | (0U << 22U)  /* parity enable */
                  | (0U << 21U)  /* wait on enable */
                  | (0U << 20U)  /* shift direction */
                  | (0U << 17U)  /* clock polarity */
                  | (0U << 16U)  /* clock phase */
                  | (79U << 8U) /* baudrate prescale */
                  | 16U;  /* data word length */

    /** - wait for buffer initialization complete before accessing MibSPI registers */
    while ((spiREG1->FLG & 0x01000000U) != 0U) 
    { 
	} /* Wait */

    /** - initialize transfer groups */
    spiREG1->TGCTRL[0U] = (1U << 30U)  /* oneshot */
                       | (0U << 29U)  /* pcurrent reset */
                       | (TRG_ALWAYS << 20U)  /* trigger event */
                       | (TRG_DISABLED << 16U)  /* trigger source */
                       | (0U << 8U);  /* start buffer */

    spiREG1->TGCTRL[1U] = (1U << 30U)  /* oneshot */
                       | (0U << 29U)  /* pcurrent reset */
                       | (TRG_ALWAYS << 20U)  /* trigger event */
                       | (TRG_DISABLED << 16U)  /* trigger source */
                       | (3U << 8U);  /* start buffer */

    spiREG1->TGCTRL[2U] = (1U << 30U)  /* oneshot */
                       | (0U << 29U)  /* pcurrent reset */
                       | (TRG_ALWAYS << 20U)  /* trigger event */
                       | (TRG_DISABLED << 16U)  /* trigger source */
                       | ((3U+2U) << 8U);  /* start buffer */

    spiREG1->TGCTRL[3U] = (1U << 30U)  /* oneshot */
                       | (0U << 29U)  /* pcurrent reset */
                       | (TRG_ALWAYS << 20U)  /* trigger event */
                       | (TRG_DISABLED << 16U)  /* trigger source */
                       | ((3U+2U+2U) << 8U);  /* start buffer */

    spiREG1->TGCTRL[4U] = (1U << 30U)  /* oneshot */
                       | (0U << 29U)  /* pcurrent reset */
                       | (TRG_ALWAYS << 20U)  /* trigger event */
                       | (TRG_DISABLED << 16U)  /* trigger source */
                       | ((3U+2U+2U+2U) << 8U);  /* start buffer */

    spiREG1->TGCTRL[5U] = (1U << 30U)  /* oneshot */
                       | (0U << 29U)  /* pcurrent reset */
                       | (TRG_ALWAYS << 20U)  /* trigger event */
                       | (TRG_DISABLED << 16U)  /* trigger source */
                       | ((3U+2U+2U+2U+0U) << 8U);  /* start buffer */

    spiREG1->TGCTRL[6U] = (1U << 30U)  /* oneshot */
                       | (0U << 29U)  /* pcurrent reset */
                       | (TRG_ALWAYS << 20U)  /* trigger event */
                       | (TRG_DISABLED << 16U)  /* trigger source */
                       | ((3U+2U+2U+2U+0U+0U) << 8U);  /* start buffer */

    spiREG1->TGCTRL[7U] = (1U << 30U)  /* oneshot */
                       | (0U << 29U)  /* pcurrent reset */
                       | (TRG_ALWAYS << 20U)  /* trigger event */
                       | (TRG_DISABLED << 16U)  /* trigger source */
                       | ((3U+2U+2U+2U+0U+0U+0U) << 8U);  /* start buffer */

    
    spiREG1->TGCTRL[8U] = (3U+2U+2U+2U+0U+0U+0U+0U) << 8U;

    spiREG1->LTGPEND = (spiREG1->LTGPEND & 0xFFFF00FFU) | (((3U+2U+2U+2U+0U+0U+0U+0U)-1U) << 8U);

    /** - initialize buffer ram */
    {   
        i = 0U;
        if (3U > 0U)
        {
            while (i < (3U-1U))
            {
                spiRAM1->tx[i++].control = (4U << 13U)  /* buffer mode */
                                         | (1U << 12U)  /* hold chip select */
                                         | (0U << 11U)  /* lock transmission */
                                         | (0U << 8U)  /* data format */
                                         | CS_0;  /* chip select */
            }
            spiRAM1->tx[i++].control = (4U << 13U)  /* buffer mode */
                                     | (0U << 12U) /* chip select hold */
                                     | (0U << 10U)  /* enable WDELAY */
                                     | (0U << 8U)  /* data format */
                                     | CS_0;  /* chip select */
        }
        if (2U > 0U)
        {
            while (i < ((3U+2U)-1U))
            {
                spiRAM1->tx[i++].control = (4U << 13U)  /* buffer mode */
                                         | (1 << 12)  /* hold chip select */
                                         | (0U << 11U)  /* lock transmission */
                                         | (0U << 8U)  /* data format */
                                         | CS_0;  /* chip select */
            }
            spiRAM1->tx[i++].control = (4U << 13U)  /* buffer mode */
                                     | (0U << 12U) /* chip select hold */
                                     | (0U << 10U)  /* enable WDELAY */
                                     | (0U << 8U)  /* data format */
                                     | CS_0;  /* chip select */
        }
        if (2U > 0U)
        {
            while (i < ((3U+2U+2U)-1U))
            {
                spiRAM1->tx[i++].control = (4U << 13U)  /* buffer mode */
                                         | (1 << 12)  /* hold chip select */
                                         | (0U << 11U)  /* lock transmission */
                                         | (0U << 8U)  /* data format */
                                         | CS_0;  /* chip select */
            }
            spiRAM1->tx[i++].control = (4U << 13U)  /* buffer mode */
                                     | (0U << 12U) /* chip select hold */
                                     | (0U << 10U)  /* enable WDELAY */
                                     | (0U << 8U)  /* data format */
                                     | CS_0;  /* chip select */
        }
        if (2U > 0U)
        {
            while (i < ((3U+2U+2U+2U)-1U))
            {
                spiRAM1->tx[i++].control = (4U << 13U)  /* buffer mode */
                                         | (1 << 12)  /* hold chip select */
                                         | (0U << 11U)  /* lock transmission */
                                         | (0U << 8U)  /* data format */
                                         | CS_0;  /* chip select */
            }
            spiRAM1->tx[i++].control = (4U << 13U)  /* buffer mode */
                                     | (0U << 12U) /* chip select hold */
                                     | (0U << 10U)  /* enable WDELAY */
                                     | (0U << 8U)  /* data format */
                                     | CS_0;  /* chip select */
        }
        if (0U > 0U)
        {
            while (i < ((3U+2U+2U+2U+0U)-1U))
            {
                spiRAM1->tx[i++].control = (4U << 13U)  /* buffer mode */
                                         | (1 << 12)  /* hold chip select */
                                         | (0U << 11U)  /* lock transmission */
                                         | (0U << 8U)  /* data format */
                                         | CS_4;  /* chip select */
            }
            spiRAM1->tx[i++].control = (4U << 13U)  /* buffer mode */
                                     | (0U << 12U) /* chip select hold */
                                     | (0U << 10U)  /* enable WDELAY */
                                     | (0U << 8U)  /* data format */
                                     | CS_4;  /* chip select */
        }
        if (0U > 0U)
        {
            while (i < ((3U+2U+2U+2U+0U+0U)-1U))
            {
                spiRAM1->tx[i++].control = (4U << 13U)  /* buffer mode */
                                         | (1 << 12)  /* hold chip select */
                                         | (0U << 11U)  /* lock transmission */
                                         | (0U << 8U)  /* data format */
                                         | CS_5;  /* chip select */
            }
            spiRAM1->tx[i++].control = (4U << 13U)  /* buffer mode */
                                     | (0U << 12U) /* chip select hold */
                                     | (0U << 10U)  /* enable WDELAY */
                                     | (0U << 8U)  /* data format */
                                     | CS_5;  /* chip select */
        }
        if (0U > 0U)
        {
            while (i < ((3U+2U+2U+2U+0U+0U+0U)-1U))
            {
                spiRAM1->tx[i++].control = (4U << 13U)  /* buffer mode */
                                         | (1 << 12)  /* hold chip select */
                                         | (0U << 11U)  /* lock transmission */
                                         | (0U << 8U)  /* data format */
                                         | CS_6;  /* chip select */
            }
            spiRAM1->tx[i++].control = (4U << 13U)  /* buffer mode */
                                     | (0U << 12U) /* chip select hold */
                                     | (0U << 10U)  /* enable WDELAY */
                                     | (0U << 8U)  /* data format */
                                     | CS_6;  /* chip select */
        }
        if (0U > 0U)
        {
            while (i < ((3U+2U+2U+2U+0U+0U+0U+0U)-1U))
            {
                spiRAM1->tx[i++].control = (4U << 13U)  /* buffer mode */
                                         | (1 << 12)  /* hold chip select */
                                         | (0U << 11U)  /* lock transmission */
                                         | (0U << 8U)  /* data format */
                                         | CS_7;  /* chip select */
            }
            spiRAM1->tx[i++].control = (4U << 13U)  /* buffer mode */
                                     | (0U << 12U) /* chip select hold */
                                     | (0U << 10U)  /* enable WDELAY */
                                     | (0U << 8U)  /* data format */
                                     | CS_7;  /* chip select */
        }
    }

    /** - set interrupt levels */
    spiREG1->LVL = (0U << 9U)  /* TXINT */
                 | (0U << 8U)  /* RXINT */
                 | (0U << 6U)  /* OVRNINT */
                 | (0U << 4U)  /* BITERR */
                 | (0U << 3U)  /* DESYNC */
                 | (0U << 2U)  /* PARERR */
                 | (0U << 1U) /* TIMEOUT */
                 | (0U);  /* DLENERR */

    /** - clear any pending interrupts */
    spiREG1->FLG |= 0xFFFFU;

    /** - enable interrupts */
    spiREG1->INT0 = (spiREG1->INT0 & 0xFFFF0000U) 
				  | (0U << 9U)  /* TXINT */
                  | (0U << 8U)  /* RXINT */
                  | (0U << 6U)  /* OVRNINT */
                  | (0U << 4U)  /* BITERR */
                  | (0U << 3U)  /* DESYNC */
                  | (0U << 2U)  /* PARERR */
                  | (0U << 1U) /* TIMEOUT */
                  | (0U);  /* DLENERR */

    /** @b initialize @b SPI1 @b Port */

    /** - SPI1 Port output values */
    spiREG1->PCDOUT =  0U        /* SCS[0] */
                    | (0U << 1U)  /* SCS[1] */
                    | (0U << 2U)  /* SCS[2] */
                    | (0U << 3U)  /* SCS[3] */
                    | (0U << 8U)  /* ENA */
                    | (0U << 9U)  /* CLK */
                    | (0U << 10U)  /* SIMO */
                    | (0U << 11U); /* SOMI */

    /** - SPI1 Port direction */
    spiREG1->PCDIR  =  1U        /* SCS[0] */
                    | (1U << 1U)  /* SCS[1] */
                    | (1U << 2U)  /* SCS[2] */
                    | (0U << 3U)  /* SCS[3] */
                    | (0U << 8U)  /* ENA */
                    | (1U << 9U)  /* CLK */
                    | (1U << 10U)  /* SIMO */
                    | (0U << 11U); /* SOMI */

    /** - SPI1 Port open drain enable */
    spiREG1->PCPDR  =  0U        /* SCS[0] */
                    | (0U << 1U)  /* SCS[1] */
                    | (0U << 2U)  /* SCS[2] */
                    | (0U << 3U)  /* SCS[3] */
                    | (0U << 8U)  /* ENA */
                    | (0U << 9U)  /* CLK */
                    | (0U << 10U)  /* SIMO */
                    | (0U << 11U); /* SOMI */

    /** - SPI1 Port pullup / pulldown selection */
    spiREG1->PCPSL  =  1U        /* SCS[0] */
                    | (1U << 1U)  /* SCS[1] */
                    | (1U << 2U)  /* SCS[2] */
                    | (0U << 3U)  /* SCS[3] */
                    | (1U << 8U)  /* ENA */
                    | (1U << 9U)  /* CLK */
                    | (1U << 10U)  /* SIMO */
                    | (1U << 11U); /* SOMI */

    /** - SPI1 Port pullup / pulldown enable*/
    spiREG1->PCDIS  =  0U        /* SCS[0] */
                    | (0U << 1U)  /* SCS[1] */
                    | (0U << 2U)  /* SCS[2] */
                    | (0U << 3U)  /* SCS[3] */
                    | (0U << 8U)  /* ENA */
                    | (0U << 9U)  /* CLK */
                    | (0U << 10U)  /* SIMO */
                    | (0U << 11U); /* SOMI */

    /* SPI1 set all pins to functional */
    spiREG1->PCFUN  =  1U        /* SCS[0] */
                    | (1U << 1U)  /* SCS[1] */
                    | (1U << 2U)  /* SCS[2] */
                    | (0U << 3U)  /* SCS[3] */
                    | (1U << 8U)  /* ENA */
                    | (1U << 9U)  /* CLK */
                    | (1U << 10U)  /* SIMO */
                    | (1U << 11U); /* SOMI */

    /** - Finally start SPI1 */
    spiREG1->GCR1 = (spiREG1->GCR1 & 0xFEFFFFFFU) | (1U << 24U);




/* USER CODE BEGIN (3) */
/* USER CODE END */

}


/** @fn void spiSetFunctional(spiBASE_t *spi, uint32 port)
*   @brief Change functional behavior of pins at runtime.
*   @param[in] spi   - Spi module base address
*   @param[in] port  - Value to write to PCFUN register
*
*   Change the value of the PCFUN register at runtime, this allows to
*   dynamically change the functionality of the SPI pins between functional
*   and GIO mode.
*/
void spiSetFunctional(spiBASE_t *spi, uint32 port)
{
/* USER CODE BEGIN (4) */
/* USER CODE END */
    spi->PCFUN = port;
/* USER CODE BEGIN (5) */
/* USER CODE END */
}


/** @fn void spiSetData(spiBASE_t *spi, uint32 group, uint16 * data)
*   @brief Set Buffer Data
*   @param[in] spi   - Spi module base address
*   @param[in] group - Transfer group (0..7)
*   @param[in] data  - new data for transfer group
*
*   This function updates the data for the specified transfer group,
*   the length of the data must match the length of the transfer group.
*/
void spiSetData(spiBASE_t *spi, uint32 group, uint16 * data)
{
/* USER CODE BEGIN (6) */
/* USER CODE END */
    spiRAM_t *ram    = spi == spiREG1 ? spiRAM1 : (spi == spiREG3 ? spiRAM3 : spiRAM5);
    uint32  start  = (spi->TGCTRL[group] >> 8U) & 0xFFU;
    uint32 end    = group == 7U ? (((spi->LTGPEND & 0x0000EF00U) >> 8U) + 1U) : ((spi->TGCTRL[group+1U] >> 8U) & 0xFFU);

    if (end < start) {end = 128U;}

    while (start < end)
    {
        ram->tx[start].data = *data++;
        start++;
    }
/* USER CODE BEGIN (7) */
/* USER CODE END */
}


/** @fn void spiGetData(spiBASE_t *spi, uint32 group, uint16 * data)
*   @brief Retrieves Buffer Data from receive buffer
*   @param[in]  spi   - Spi module base address
*   @param[in]  group - Transfer group (0..7)
*   @param[out] data  - pointer to data array
*
*   @return error flags from data buffer, if there was a receive error on
*           one of the buffers this will be reflected in the return value.
*
*   This function transfers the data from the specified transfer group receive
*   buffers to the data array,  the length of the data must match the length 
*   of the transfer group.
*/
uint32 spiGetData(spiBASE_t *spi, uint32 group, uint16 * data)
{
/* USER CODE BEGIN (8) */
/* USER CODE END */
    spiRAM_t *ram    = spi == spiREG1 ? spiRAM1 : (spi == spiREG3 ? spiRAM3 : spiRAM5);
    uint32 start  = (spi->TGCTRL[group] >> 8U) & 0xFFU;
    uint32 end    = group == 7U ? (((spi->LTGPEND & 0x00007F00U) >> 8U) + 1U) : ((spi->TGCTRL[group+1U] >> 8U) & 0xFFU);
    uint32  flags  = 0U;

    if (end < start) {end = 128U;}

    while (start < end)
    {
        flags  |= ram->rx[start].flags;
        *data++ = ram->rx[start].data;
        start++;
    }
/* USER CODE BEGIN (9) */
/* USER CODE END */
    return (flags >> 8U) & 0x5FU;
}


/** @fn void spiTransfer(spiBASE_t *spi, uint32 group)
*   @brief Transmit Transfer Group
*   @param[in] spi   - Spi module base address
*   @param[in] group - Transfer group (0..7)
*
*   Initiates a transfer for the specified transfer group.
*/
void spiTransfer(spiBASE_t *spi, uint32 group)
{
/* USER CODE BEGIN (10) */
/* USER CODE END */
    spi->TGCTRL[group] |= 0x80000000U;
/* USER CODE BEGIN (11) */
/* USER CODE END */
}


/** @fn uint32 spiIsTransferComplete(spiBASE_t *spi, uint32 group)
*   @brief Check for Transfer Group Ready
*   @param[in] spi   - Spi module base address
*   @param[in] group - Transfer group (0..7)
*
*   @return 1 is transfer complete, otherwise 0.
*
*   Checks to see if the transfer for the specified transfer group
*   has finished.
*/
uint32 spiIsTransferComplete(spiBASE_t *spi, uint32 group)
{
/* USER CODE BEGIN (12) */
/* USER CODE END */
    return ((((spi->TGINTFLG & 0xFFFF0000U) >> 16U) >> group) & 1U);
}


/** @fn void spiEnableGroupNotification(spiBASE_t *spi, uint32 group, uint32 level)
*   @brief Enable Transfer Group interrupt
*   @param[in] spi   - Spi module base address
*   @param[in] group - Transfer group (0..7)
*   @param[in] level - Interrupt level
*
*   This function enables the transfer group finished interrupt.
*/
void spiEnableGroupNotification(spiBASE_t *spi, uint32 group, uint32 level)
{
    if (level != 0U)
    {
        spi->TGITLVST = (spi->TGITLVST & 0x0000FFFFU) | ((1U << group) << 16U);
    }
    else
    {
        spi->TGITLVCR = (spi->TGITLVCR & 0x0000FFFFU) | ((1U << group) << 16U);
    }
    spi->TGITENST = (spi->TGITENST & 0x0000FFFFU) | ((1U << group) << 16U);
}


/** @fn void spiDisableGroupNotification(spiBASE_t *spi, uint32 group)
*   @brief Disable Transfer Group interrupt
*   @param[in] spi   - Spi module base address
*   @param[in] group - Transfer group (0..7)
*
*   This function disables the transfer group finished interrupt.
*/
void spiDisableGroupNotification(spiBASE_t *spi, uint32 group)
{
    spi->TGITENCR = (spi->TGITENCR & 0x0000FFFFU) | ((1U << group) << 16U);
}


/** @fn void spi1HighLevelInterrupt(void)
*   @brief Level0 Interrupt for SPI1
*/
#pragma INTERRUPT(spi1HighLevelInterrupt, IRQ)

void spi1HighLevelInterrupt(void)
{
    uint32 vec = spiREG1->INTVECT0;

    if (vec > 0x21U)
    {
        uint32 spiFlags = (spiREG1->FLG &0x0000FFFFU) & (~(spiREG1->LVL) & 0x035FU);
        spiREG1->FLG = (spiREG1->FLG & 0xFFFF0000U) | spiFlags;
        spiNotification(spiREG1, spiFlags);
    }
    else
    {
        spiGroupNotification(spiREG1, ((vec & 0x3FU) >> 1U) - 1U);
    }
}


/** @fn void spi1LowLevelInterrupt(void)
*   @brief Level1 Interrupt for SPI1
*/
#pragma INTERRUPT(spi1LowLevelInterrupt, IRQ)

void spi1LowLevelInterrupt(void)
{
    uint32 vec = spiREG1->INTVECT1;

    if (vec > 0x21U)
    {
        uint32 spiFlags = (spiREG1->FLG & 0x0000FFFFU) & (spiREG1->LVL & 0x035FU);
        spiREG1->FLG = (spiREG1->FLG & 0xFFFF0000U) | spiFlags;
        spiNotification(spiREG1, spiFlags);
    }
    else
    {
        spiGroupNotification(spiREG1, ((vec & 0x3FU) >> 1U) - 1U);
    }
}


/** @fn void spi3HighInterruptLevel(void)
*   @brief Level0 Interrupt for SPI3
*/
#pragma INTERRUPT(spi3HighInterruptLevel, IRQ)

void spi3HighInterruptLevel(void)
{
    uint32 vec = spiREG3->INTVECT0;

    if (vec > 0x21U)
    {
        uint32 spiFlags = (spiREG3->FLG & 0x0000FFFFU) & (~(spiREG3->LVL) & 0x035FU);
        spiREG3->FLG = (spiREG3->FLG & 0xFFFF0000U) | spiFlags;
        spiNotification(spiREG3, spiFlags);
    }
    else
    {
        spiGroupNotification(spiREG3, ((vec & 0x3FU) >> 1U) - 1U);
    }
}


/** @fn void spi3LowLevelInterrupt(void)
*   @brief Level1 Interrupt for SPI3
*/
#pragma INTERRUPT(spi3LowLevelInterrupt, IRQ)

void spi3LowLevelInterrupt(void)
{
    uint32 vec = spiREG3->INTVECT1;

    if (vec > 0x21U)
    {
        uint32 spiFlags = (spiREG3->FLG & 0x0000FFFFU) & (spiREG3->LVL & 0x035FU);
        spiREG3->FLG = (spiREG3->FLG & 0xFFFF0000U) | spiFlags;
        spiNotification(spiREG3, spiFlags);
    }
    else
    {
        spiGroupNotification(spiREG3, ((vec & 0x3FU) >> 1U) - 1U);
    }
}


/** @fn void spi5HighLevelInterrupt(void)
*   @brief Level0 Interrupt for SPI5
*/
#pragma INTERRUPT(spi5HighLevelInterrupt, IRQ)

void spi5HighLevelInterrupt(void)
{
    uint32 vec = spiREG5->INTVECT0;

    if (vec > 0x21U)
    {
        uint32 spiFlags = (spiREG5->FLG & 0x0000FFFFU) & (~(spiREG5->LVL) & 0x035FU);
        spiREG5->FLG = (spiREG5->FLG & 0xFFFF0000U) | spiFlags;
        spiNotification(spiREG5, spiFlags);
    }
    else
    {
        spiGroupNotification(spiREG5, ((vec & 0x3FU) >> 1U) - 1U);
    }
}


/** @fn void spi5LowLevelInterrupt(void)
*   @brief Level1 Interrupt for SPI5
*/
#pragma INTERRUPT(spi5LowLevelInterrupt, IRQ)

void spi5LowLevelInterrupt(void)
{
    uint32 vec = spiREG5->INTVECT1;

    if (vec > 0x21U)
    {
        uint32 spiFlags = (spiREG5->FLG & 0x0000FFFFU) & (spiREG5->LVL & 0x035FU);
        spiREG5->FLG = (spiREG5->FLG & 0xFFFF0000U) | spiFlags;
        spiNotification(spiREG5, spiFlags);
    }
    else
    {
        spiGroupNotification(spiREG5, ((vec & 0x3FU) >> 1U) - 1U);
    }
}

