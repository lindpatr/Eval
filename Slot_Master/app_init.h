/***************************************************************************//**
 * @file
 * @brief app_init.h
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#ifndef APP_INIT_H
#define APP_INIT_H

// -----------------------------------------------------------------------------
//                                   Includes
// -----------------------------------------------------------------------------
#include "rail.h"
#include "common_config.h"
// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------
// Debug general directive
//#if (SL_DEBUG)
#define qDebug          1           // Additional debug info
//#else
//#define qDebug        0           // Release
//#endif  // SL_DEBUG
#define qPrintTX        0     // Print out sent data on serial COM
#define qPrintRX        0     // Print out received data on serial COM
#define qPrintEvents    1     // Print out events on serial COM
#define qPrintErrorsL1  1     // Print out RAIL errors on serial COM (generally return of RAIL_functions)
#define qPrintErrorsL2  0     // Print out RAIL errors on serial COM (kTimeOutTx, kTimeOutRx, kErrorTx, kErrorRx)
#define qPrintInfo      1     // Print out info serial COM
#define qPrintStat      1     // Print out statistics serial COM

#define TRANSITION_TIMING_BEST_EFFORT  1

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
/// A static handle of a RAIL instance
extern volatile RAIL_Handle_t gRailHandle;
/// A static var for TX schedule config
extern RAIL_ScheduleTxConfig_t gRailScheduleCfgTX;
/// A static var for RX transition
extern RAIL_StateTransitions_t gRailTransitionRX;
/// A static var for TX transition
extern RAIL_StateTransitions_t gRailTransitionTX;
/// A static var that contains config data for the device
extern PROT_AddrMap_t* gDeviceCfgAddr;
// Value, indicating sync period on CLI
extern volatile RAIL_Time_t gSyncPeriod;
// Value, indicating sync timeout for Slave on CLI
extern volatile RAIL_Time_t gSyncTimeOut;
// Value, indicating time of a slot in the protocol on CLI
extern volatile uint32_t gTimeSlot;

// -----------------------------------------------------------------------------
//                          Public Function Declarations
// -----------------------------------------------------------------------------


/**************************************************************************//**
 * The function is used for some basic initialization related to the app.
 *
 * @param None
 * @returns RAIL_Handle_t RAIL handle
 *
 * It ensures the followings:
 * - Turn OFF LEDs
 * - Start RAIL reception
 * - Printf start message
 *****************************************************************************/
void app_init(void);

#endif  // APP_INIT_H
