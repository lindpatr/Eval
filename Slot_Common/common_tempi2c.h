/*
 * common_tempi2c.h
 *
 *  Created on: 22 sept. 2022
 *      Author: BEL-LinPat
 */

#ifndef COMMON_TEMPI2C_H_
#define COMMON_TEMPI2C_H_

#include <stdbool.h>
#include <stdint.h>

#define kMaxDevice 4

// I2c structure tmp116
/* Data register structure */
#pragma pack (2)
typedef struct
{
    uint16_t   ChipAddr;           /* TMP116 Chip Address          */
    uint16_t   ConfigReg;          /* TMP116 Configuration bits    */
    uint16_t   LastReg;            /* Last addressed register      */
    uint16_t   HystLimTempHigh;    /* Hyst high level limit to activate error signal   */
    uint16_t   HystLimTempLow;     /* Hyst low level limit to de-activate error signal */
} TMP116ConfigStruct;

/*****************************************************************************
 * @brief
 *   Fill in configuration structuer for all tmp116 devices (max 4). Do not send any i2C messages.
 *
 * @note
 *   Do not send any i2C messages.
 *   Use common_tempi2cSetup to send config to devices
 *
 * @param[in] index Device index
 * @param[in] chipAdr I2C device address
 * @param[in] limTempHigh Temp hysteresis high (set error output pin to high)
 * @param[in] limTempLow Temp hysteresis low (set error output pin to low)
 *
 ******************************************************************************/
void common_tempi2cConfig(uint8_t index, uint8_t chipAdr, float limTempHigh, float limTempLow);

/***************************************************************************//**
 * @brief
 *   Send any i2C configuration messages to device(s)
 *
 * @note
 *   Send config register data and LimHigh and LimLow values
 *
 * @return  true if successful
 ******************************************************************************/
bool common_tempi2cSetup();

/***************************************************************************//**
 * @brief
 *   Get temperature value from TMP116  device
 *
 * @note
 *   Do not send any i2C messages.
 *   Use common_tempi2cSetup to send config to devices
 *
 * @param[in] index i2C interface number (0...kMaxDevice)
 * @param[in] temp temperature value (not converted)
 *
 * @return True if successful
 ******************************************************************************/
bool common_tempi2cReadTemp(uint8_t index, uint16_t* temp);


#endif /* COMMON_TEMPI2C_H_ */
