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

// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------
// Compile directives
#if (SL_MASTER)
#define qMaster			1			// Master
#else
#define qMaster			0			// Slave
#endif	// SL_MASTER

#define qUseDisplay     1			// Display statistics
#define qPrintTX        0			// Print out sent data on serial COM
#define qPrintRX        0			// Print out received data on serial COM
#define qPrintEvents	1			// Print out events on serial COM
#define qPrintErrors	0			// Print out RAIL errors on serial COM

#if (qMaster)
#define qUseScheduleRx	1			// 0 = RAIL_StartRx, 1 = RAIL_ScheduleRx
#else
#define qUseScheduleRx	0			// 0 = RAIL_StartRx, 1 = RAIL_ScheduleRx
#endif	// qMaster
#define qUseScheduleTx	0			// 0 = RAIL_StartTx, 1 = RAIL_StartScheduledTx
#if (qMaster)
#define qUseTimeOutRx	0			// 0 = no timeout with RAIL_StartRx, 1 = use MultiTimer for timeout RAIL_StartRx
#else
#define qUseTimeOutRx	0			// 0 = no timeout with RAIL_StartRx, 1 = use MultiTimer for timeout RAIL_StartRx
#endif	// qMaster
#define qUseTimeOutTx	1			// 0 = no timeout with RAIL_StartTx, 1 = use MultiTimer for timeout RAIL_StartTx

// GPIO debug
#define DEBUG_PIN_TX    1         	// PB01 --> Exp Header 9
#define DEBUG_PIN_RX    2         	// PB02 --> Exp Header 15
#define DEBUG_PIN_MISC	3		  	// PB03 --> Exp Header 16
#define DEBUG_PORT      gpioPortB
#define SET             1			// Set PIN
#define RESET           0			// Reset PIN

// Timeout for Start RX and TX RAIL functions
#if (qMaster)
#define RX_TIMEOUT	(650U/*850U*/)					// in us
#define TX_START	(80U)							// in us
#define TX_TIMEOUT	(2.05*RX_TIMEOUT)				// in us
#else	// !qMaster
#define RX_TIMEOUT	(850U/*850U*/)					// in us
#define TX_START	(80U)							// in us
#define TX_TIMEOUT	(2.05*RX_TIMEOUT)				// in us
#endif	// qMaster

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
/// A static handle of a RAIL instance
extern volatile RAIL_Handle_t gRailHandle;
/// A static var for RX schedule config
extern RAIL_ScheduleRxConfig_t gRailScheduleCfgRX;
/// A static var for TX schedule config
extern RAIL_ScheduleTxConfig_t gRailScheduleCfgTX;
/// A static var for RX transition
extern RAIL_StateTransitions_t gRailTransitionRX;
/// A static var for TX transition
extern RAIL_StateTransitions_t gRailTransitionTX;

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
