/*
 * tmp126_spi.h
 *
 *  Created on: 17 oct. 2022
 *      Author: BEL-LinPat
 */

#ifndef COMMON_TMP126_SPI_H_
#define COMMON_TMP126_SPI_H_

#include <stdint.h>
#include "sl_pwm.h"

typedef struct
{
    uint16_t regAddr:8;

    uint16_t r_w:1;
    uint16_t addrInc:1;
    uint16_t len:4;
    uint16_t crc:1;
    uint16_t reserved:1;

    uint16_t word;
}CmdWordStruct;

typedef struct
{
    uint16_t conv_period:3;
    uint16_t mode:1;
    uint16_t one_shot:1;
    uint16_t int_comp:1;
    uint16_t reserved2:1;
    uint16_t avg:1;

    uint16_t reset:1;
    uint16_t reserved:7;
}ConfWordStruct;

typedef struct
{
    uint16_t dataready_alert:1;
    uint16_t TLow_alert:1;
    uint16_t THigh_alert:1;
    uint16_t slew_alert:1;
    uint16_t crc_alert:1;
    uint16_t reserved2:3;

    uint16_t reserved:8;
}AlertEnWordStruct;

typedef union
{
    CmdWordStruct       CmdTmp126;
    ConfWordStruct      ConfTmp126;
    AlertEnWordStruct   Alert_En;
    uint16_t            word;
    uint8_t             bytes[2];
}CmdWordUnion;

/**
 * @brief
 * Init SPI TMP126.
 *
 * @param[in] tempLimLow    Temp limit low [°C]
 * @param[in] tempLimHigh   Temp limit high [°C]
 *
 * @return true if successful, false if timeout
 */
bool spi_tmp126_init(float tempLimLow, float tempLimHigh);

/**
 * @brief
 * Return TMP126 product ID.
 *
 * @return product ID if successful, 0 if timeout
 */
uint16_t spi_tmp126_getID(void);

/**
 * @brief
 * Return temperature measure.
 *
 * @return temperature [AD value] if successful, UINT_MAX if timeout
 */
uint16_t spi_tmp126_getTemp(void);

/**
 * @brief
 * Convert temperature measure [AD value] to [°C].
 *
 * @return temperature [°C]
 */
float spi_temp126_TempToDeg(int16_t tempAd);

#endif /* COMMON_TMP126_SPI_H_ */
