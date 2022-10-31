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
    device1 = 1
} DeviceIdentEnum_t;

// Chip select definition structure // Max 2 CS
#define SPI_CS_NUMBER   (2)

typedef struct
{
    int32_t portNumber;
    int32_t pinNumber;
} SpiCsStruct_t;

/**
 * @brief
 * Init SPI.
 *
 * @param[in] baudrate Spi baudrate [bit/s]
 * @param[in] clockPhase Clock phase
 * @param[in] cs Chipselect tab definition
 */
void common_initSPI(bool master, uint32_t baudrate, USART_ClockMode_TypeDef clockPhase, SpiCsStruct_t cs[]);

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

void common_startSPItransfertSlave(DeviceIdentEnum_t device, uint8_t buffSize, uint8_t* txBuffer, uint8_t* rxBuffer);

/**
 * @brief
 * Wait for the end of the SPI transfer
 *
 * @param[in] device Device number to select which cs is asserted
 * @return result TRUE sucessfull, FALSE timout.
*/
bool common_waitSPITransfertDone(DeviceIdentEnum_t device);

bool common_waitSPITransfertDoneSlave(DeviceIdentEnum_t device);

void LDMA_Start();

void LDMA_Stop();



#endif /* COMMON_SPI_H_ */
