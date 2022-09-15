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
