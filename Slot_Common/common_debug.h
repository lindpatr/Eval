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

#define DEBUG_PIN_SET(pin)      (pin == DEBUG_PIN_H09 ? GPIO_PinOutSet(DEBUG_PORT_B, pin) : GPIO_PinOutSet(DEBUG_PORT_D, pin))
#define DEBUG_PIN_RESET(pin)    (pin == DEBUG_PIN_H09 ? GPIO_PinOutClear(DEBUG_PORT_B, pin) : GPIO_PinOutClear(DEBUG_PORT_D, pin))

#define SET                     1
#define RESET                   0


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
