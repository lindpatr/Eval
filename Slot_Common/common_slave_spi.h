/*
 * common_slave_spi.h
 *
 *  Created on: 4 nov. 2022
 *      Author: BEL-LinPat
 */

#ifndef COMMON_SLAVE_SPI_H_
#define COMMON_SLAVE_SPI_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief
 * Init SPI in slave mode.
 *
 */
void common_initSPISlave(void);

/**
 * @brief
 * Start a SPI transfert (Master).
 *
 * @param[in] buffSize TX/RX buffer size -> size to TX/RX
 * @param[in] txBuffer TX buffer ptr
 * @param[in] rxBuffer RX buffer ptr
 */
void common_startSPISlavetransfert(uint8_t buffSize, uint8_t* txBuffer, uint8_t* rxBuffer);

/**
 * @brief
 * Wait for the end of the SPI transfer
 *
 * @return result TRUE sucessful, FALSE timout.
*/
int common_waitSPISlaveTransfertDone(void);

#endif /* COMMON_SLAVE_SPI_H_ */
