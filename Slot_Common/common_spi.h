/*
 * common_spi.h
 *
 *  Created on: 4 oct. 2022
 *      Author: BEL-LinPat
 */

#ifndef COMMON_SPI_H_
#define COMMON_SPI_H_

#include <stdbool.h>
#include <stdint.h>
#include "em_usart.h"

typedef enum
{
    device0 = 0,
    device1
} DeviceIdentEnum_t;

/**
 * @brief
 * Init SPI.
 *
 * @param[in] baudrate Spi baudrate [bit/s]
 * @param[in] clockPhase Clock phase
 */
void common_initSPI(uint32_t baudrate, USART_ClockMode_TypeDef clockPhase);

/**
 * @brief
 * Start a SPI transfert (Master).
 *
 * @param[in] device Device number to select which cs is asserted
 * @param[in] buffSize TX/RX buffer size -> size to TX/RX
 * @param[in] txBuffer TX buffer ptr
 * @param[in] rxBuffer RX buffer ptr
 */
void common_startSPItransfert(DeviceIdentEnum_t device, uint8_t buffSize, uint8_t* txBuffer, uint8_t* rxBuffer);

/**
 * @brief
 * Wait for the end of the SPI transfer
 *
 * @param[in] device Device number to select which cs is asserted
 * @return result TRUE sucessfull, FALSE timout.
*/
bool common_waitSPITransfertDone(DeviceIdentEnum_t device);

#endif /* COMMON_SPI_H_ */
