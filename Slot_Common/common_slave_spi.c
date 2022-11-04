/*
 * common_slave_spi.c
 *
 *  Created on: 4 nov. 2022
 *      Author: BEL-LinPat
 */

#include "common_slave_spi.h"
#include "rail.h"
#include "spidrv.h"
#include "sl_spidrv_instances.h"
#include "app_init.h"
#include "common_debug.h"

// use SPI handle for EXP header (configured in project settings)
#define SPI_HANDLE sl_spidrv_exp_handle

// SPI TX/RX timeout
#define SPI_TIMEOUT (80)
// Flag to signal that transfer is complete
static volatile bool gTransfer_complete = false;


void common_callbackSPISlave(SPIDRV_HandleData_t *handle,
                       Ecode_t transfer_status,
                       int items_transferred)
{
  (void)&handle;
  (void)items_transferred;

  // Post semaphore to signal to application
  // task that transfer is successful
  if (transfer_status == ECODE_EMDRV_SPIDRV_OK) {
    gTransfer_complete = true;
  }
}

/**
 * @brief
 * Init SPI in slave mode.
 *
 */
void common_initSPISlave(void)
{
    gTransfer_complete = false;
}

/**
 * @brief
 * Start a SPI transfert (Master).
 *
 * @param[in] buffSize TX/RX buffer size -> size to TX/RX
 * @param[in] txBuffer TX buffer ptr
 * @param[in] rxBuffer RX buffer ptr
 */
void common_startSPISlavetransfert(uint8_t buffSize, uint8_t* txBuffer, uint8_t* rxBuffer)
{
    Ecode_t ecode;

    // Non-blocking data transfer to master. When complete, rx buffer
    // will be filled.
    ecode = SPIDRV_STransfer(SPI_HANDLE, txBuffer, rxBuffer, buffSize, common_callbackSPISlave, 0);
    EFM_ASSERT(ecode == ECODE_OK);
}

/**
 * @brief
 * Wait for the end of the SPI transfer
 *
 * @return result TRUE successful, FALSE timeout.
*/
int common_waitSPISlaveTransfertDone(void)
{
    RAIL_Time_t startTime = RAIL_GetTime();
    RAIL_Time_t gap = 0;

    while(!gTransfer_complete)
    {
        gap = RAIL_GetTime() - startTime;
        if (gap > SPI_TIMEOUT)
        {
            gTransfer_complete = false;
            return gap;
        }
    }

    gTransfer_complete = false;
    return gap;
}
