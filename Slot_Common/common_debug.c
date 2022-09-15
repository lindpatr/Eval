/*
 * common_config.c
 *
 *  Created on: 6 sept. 2022
 *      Author: BEL-LinPat
 */

#ifndef COMMON_DEBUG_H_
#include "common_debug.h"
#endif

#include <stdbool.h>

#include "app_log.h"


/******************************************************************************
 * PrintStatus : print return <> RAIL_STATUS_NO_ERROR status from RAIL functions
 *****************************************************************************/
 __INLINE void PrintStatus(RAIL_Status_t status, char *text)
{
#if (qPrintErrorsL1)
    if (status != RAIL_STATUS_NO_ERROR)
    {
        app_log_warning("%s (%d)\n", text, status);
    }
#else   // To avoid compile warnings
    (void) status;
    (void) text;
#endif  // qPrintErrorsL1
}

/******************************************************************************
 * PrintError : print error event code from RAIL callback
 *****************************************************************************/
 __INLINE void PrintError(uint64_t errcode, char *text)
{
#if (qPrintErrorsL2)
    if (errcode != gPrevErrorCode)
    {
        app_log_error("%s (0x%llX)\n", text, errcode);
        gPrevErrorCode = errcode;
    }
#else   // To avoid compile warnings
    (void) errcode;
    (void) text;
#endif  // qPrintErrorsL2
}

/******************************************************************************
 * PrintInfo : print info
 *****************************************************************************/
 __INLINE void PrintInfo(char *text)
{
#if (qPrintInfo)
    app_log_info("%s\n", text);
#else   // To avoid compile warnings
    (void) text;
#endif  // qPrintInfo
}

