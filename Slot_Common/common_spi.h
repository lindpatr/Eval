/*
 * common_spi.h
 *
 *  Created on: 4 oct. 2022
 *      Author: BEL-LinPat
 */

#ifndef COMMON_SPI_H_
#define COMMON_SPI_H_

#include <stdbool.h>

void common_initSPI(void);

void common_startSPItransfert(bool txOnly);

bool common_waitSPITransfertDone(bool txOnly);

#endif /* COMMON_SPI_H_ */
