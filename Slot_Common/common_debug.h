/*
 * common_config.h
 *
 *  Created on: 6 sept. 2022
 *      Author: BEL-LinPat
 */

#ifndef COMMON_DEBUG_H_
#define COMMON_DEBUG_H_

// Include standard type headers to help define structures
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "app_process.h"
#include "app_log.h"
#include "app_init.h"

// GPIO debug
#define DEBUG_PORT_B            gpioPortB
#define DEBUG_PORT_D            gpioPortD
#define DEBUG_PIN_H09           1           // PB01 --> Exp Header 9
#define DEBUG_PIN_H11           2           // PD02 --> Exp Header 11
#define DEBUG_PIN_H13           3           // PD03 --> Exp Header 13

// Do not use this
#define DEBUG_PIN_SET(pin)      (pin == DEBUG_PIN_H09 ? GPIO_PinOutSet(DEBUG_PORT_B, pin) : GPIO_PinOutSet(DEBUG_PORT_D, pin))
#define DEBUG_PIN_RESET(pin)    (pin == DEBUG_PIN_H09 ? GPIO_PinOutClear(DEBUG_PORT_B, pin) : GPIO_PinOutClear(DEBUG_PORT_D, pin))

#define SET                     1
#define RESET                   0

// Use that
#define DEBUG_PIN_TX_SET                DEBUG_PIN_SET(DEBUG_PIN_H09)
#define DEBUG_PIN_TX_RESET              DEBUG_PIN_RESET(DEBUG_PIN_H09)
#define DEBUG_PIN_RX_SET                DEBUG_PIN_SET(DEBUG_PIN_H11)
#define DEBUG_PIN_RX_RESET              DEBUG_PIN_RESET(DEBUG_PIN_H11)
#define DEBUG_PIN_CB_SET                DEBUG_PIN_SET(DEBUG_PIN_H13)
#define DEBUG_PIN_CB_RESET              DEBUG_PIN_RESET(DEBUG_PIN_H13)

// IADC conversion time
#define DEBUG_PIN_IADC_SET
#define DEBUG_PIN_IADC_RESET

// I2C Send configuration register
#define DEBUG_PIN_I2C_CFG_SET           /*DEBUG_PIN_SET(DEBUG_PIN_H09)*/
#define DEBUG_PIN_I2C_CFG_RESET         /*DEBUG_PIN_RESET(DEBUG_PIN_H09)*/
// I2C Send R/W request/get temperature
#define DEBUG_PIN_I2C_RW_TEMP_SET       /*DEBUG_PIN_SET(DEBUG_PIN_H11)*/
#define DEBUG_PIN_I2C_RW_TEMP_RESET     /*DEBUG_PIN_RESET(DEBUG_PIN_H11)*/
// I2C Get temperature
#define DEBUG_PIN_I2C_RO_TEMP_SET       /*DEBUG_PIN_SET(DEBUG_PIN_H13)*/
#define DEBUG_PIN_I2C_RO_TEMP_RESET     /*DEBUG_PIN_RESET(DEBUG_PIN_H13)*/


/******************************************************************************
 * InitGPIODebug : Setup GPIO debug specific pins
 *****************************************************************************/
void InitGPIODebug(void);

/******************************************************************************
 * PrintStatus : print return <> RAIL_STATUS_NO_ERROR status from RAIL functions
 *****************************************************************************/
void PrintStatus(RAIL_Status_t status, char *text);

/******************************************************************************
 * PrintError : print error event code from RAIL callback
 *****************************************************************************/
void PrintError(uint64_t errcode, char *text);

/******************************************************************************
 * PrintInfo : print info
 *****************************************************************************/
void PrintInfo(char *text);


#endif /* COMMON_DEBUG_H_ */
