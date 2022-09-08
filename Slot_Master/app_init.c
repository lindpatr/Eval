/***************************************************************************//**
 * @file
 * @brief app_init.c
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

// -----------------------------------------------------------------------------
//                                   Includes
// -----------------------------------------------------------------------------
#include <stdint.h>
#include "sl_component_catalog.h"
#include "rail.h"
#include "sl_rail_util_init.h"
#include "app_process.h"
#include "sl_simple_led_instances.h"
#include "app_log.h"
#include "sl_rail_util_init_inst0_config.h"
#include "sl_rail_util_protocol_types.h"
#include "rail_config.h"

#include "em_gpio.h"
#include "app_init.h"
#include "em_system.h"

// LCD display
#include "dmd.h"
#include "glib.h"
#include "printf.h"


#if defined(RAIL0_CHANNEL_GROUP_1_PROFILE_WISUN_OFDM)
#include "sl_rail_util_pa_config.h"
#include "rail_chip_specific.h"
#endif


// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------
/*****************************************************************************
* Checks phy setting to avoid errors at packet sending
*****************************************************************************/
static void validation_check(void);

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
/// A static handle of a RAIL instance
volatile RAIL_Handle_t gRailHandle;
/// A static var for RX schedule config
RAIL_ScheduleRxConfig_t gRailScheduleCfgRX;
/// A static var for TX schedule config
RAIL_ScheduleTxConfig_t gRailScheduleCfgTX;
/// A static var for RX transition
RAIL_StateTransitions_t gRailTransitionRX;
/// A static var for TX transition
RAIL_StateTransitions_t gRailTransitionTX;
/// A static var for state timings
RAIL_StateTiming_t gRailStateTimings;

// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------
/// LCD variables
/// Context used all over the graphics
static GLIB_Context_t gGlibContext;

// -----------------------------------------------------------------------------
//                          Private Function Definitions
// -----------------------------------------------------------------------------
/*******************************************************************************
 * @brief Initializes the graphics stack.
 * @note This function will /hang/ if errors occur (usually
 *       caused by faulty displays.
 ******************************************************************************/
void graphics_init(void)
{
  EMSTATUS status;

  /* Initialize the DMD module for the DISPLAY device driver. */
  status = DMD_init(0);
  if (DMD_OK != status)
  {
#if (qPrintErrorsL1)
    app_log_error("Error DMD_init (%d)\n", status);
#endif  //qPrintErrorsL1
    return;
  }

  status = GLIB_contextInit(&gGlibContext);
  if (GLIB_OK != status)
  {
#if (qPrintErrorsL1)
    app_log_error("Error GLIB_contextInit (%d)\n", status);
#endif  //qPrintErrorsL1
    return;
  }

  gGlibContext.backgroundColor = White;
  gGlibContext.foregroundColor = Black;

  // Clear what's currently on screen
  GLIB_clear(&gGlibContext);

  GLIB_setFont(&gGlibContext, (GLIB_Font_t*) &GLIB_FontNarrow6x8);

  GLIB_drawStringOnLine(&gGlibContext, "****************", 1, GLIB_ALIGN_CENTER, 0, 0, 0);
  GLIB_drawStringOnLine(&gGlibContext, "* SLOT PROTOCOL*", 2, GLIB_ALIGN_CENTER, 0, 0, 0);
#if (qMaster)
  GLIB_drawStringOnLine(&gGlibContext, "*   (Master)   *", 3, GLIB_ALIGN_CENTER, 0, 0, 0);
#else
  GLIB_drawStringOnLine(&gGlibContext, "*   (Slave)    *", 3, GLIB_ALIGN_CENTER, 0, 0, 0);
#endif  // qMaster
  GLIB_drawStringOnLine(&gGlibContext, "****************", 4, GLIB_ALIGN_CENTER, 0, 0, 0);
  GLIB_setFont(&gGlibContext, (GLIB_Font_t*) &GLIB_FontNormal8x8);
  GLIB_drawStringOnLine(&gGlibContext, "Press BTN0", 6, GLIB_ALIGN_CENTER, 0, 0, 0);
  GLIB_drawStringOnLine(&gGlibContext, "to display", 7, GLIB_ALIGN_CENTER, 0, 0, 0);
  GLIB_drawStringOnLine(&gGlibContext, "statistics", 8, GLIB_ALIGN_CENTER, 0, 0, 0);

  GLIB_setFont(&gGlibContext, (GLIB_Font_t*) &GLIB_FontNarrow6x8);
  GLIB_drawStringOnLine(&gGlibContext, "*** STOPPED ***", 12, GLIB_ALIGN_CENTER, 0, 0, 0);

  // Force a redraw
  DMD_updateDisplay();
}

// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------

/******************************************************************************
 * The function is used for some basic initialization related to the app.
 *****************************************************************************/
void app_init(void)
{
  RAIL_Status_t status = RAIL_STATUS_NO_ERROR;

  validation_check();

  // Get RAIL handle, used later by the application
  gRailHandle = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);

  // Turn OFF LEDs
  sl_led_turn_off(&sl_led_led0);
  sl_led_turn_off(&sl_led_led1);

  // Debug pins init + set to 0
  GPIO_PinModeSet(DEBUG_PORT, DEBUG_PIN_TX, gpioModePushPull, RESET);
  GPIO_PinModeSet(DEBUG_PORT, DEBUG_PIN_RX, gpioModePushPull, RESET);
  GPIO_PinModeSet(DEBUG_PORT, DEBUG_PIN_MISC, gpioModePushPull, RESET);

#if (qUseScheduleRx)
  // Set timeout for scheduled RX
  gRailScheduleCfgRX.start = 0;
  gRailScheduleCfgRX.startMode = RAIL_TIME_DELAY;
  gRailScheduleCfgRX.end = RX_TIMEOUT;
  gRailScheduleCfgRX.endMode = RAIL_TIME_DELAY;
  gRailScheduleCfgRX.hardWindowEnd = false;
  gRailScheduleCfgRX.rxTransitionEndSchedule = false;
#endif  // qUseScheduleRx

#if (qUseScheduleTx)
  // Set timeout for scheduled TX
  gRailScheduleCfgTX.when = TX_START;
  gRailScheduleCfgTX.mode = RAIL_TIME_DELAY;
  gRailScheduleCfgTX.txDuringRx = RAIL_SCHEDULED_TX_DURING_RX_ABORT_TX;   // ou RAIL_SCHEDULED_TX_DURING_RX_POSTPONE_TX --> à voir?
#endif  // qUseScheduleTx

#if (qAutoTransition)
  // Set RX and TX transition
#if (qRx2TxAutoTransition)
  gRailTransitionRX.success = RAIL_RF_STATE_TX;   // RX Ok  -> TX
#else
  gRailTransitionRX.success = RAIL_RF_STATE_RX;   // RX Ok  -> RX
#endif  // qRx2TxAutoTransition
  gRailTransitionRX.error = RAIL_RF_STATE_RX;     // RX Err -> RX
  gRailTransitionTX.success = RAIL_RF_STATE_RX;   // TX Ok  -> RX
  gRailTransitionTX.error = RAIL_RF_STATE_RX;     // TX Err -> RX

  status = RAIL_SetRxTransitions(gRailHandle, &gRailTransitionRX);
  if (status != RAIL_STATUS_NO_ERROR)
  {
#if (qPrintErrorsL1)
        app_log_warning("Warning RAIL_SetRxTransitions (%d)\n", status);
  #endif  // qPrintErrorsL1
  }
  status = RAIL_SetTxTransitions(gRailHandle, &gRailTransitionTX);
  if (status != RAIL_STATUS_NO_ERROR)
  {
#if (qPrintErrorsL1)
        app_log_warning("Warning RAIL_SetTxTransitions (%d)\n", status);
  #endif  // qPrintErrorsL1
  }

  // State timings
  gRailStateTimings.idleToRx = (TRANSITION_TIMING_BESTOF ? 0 : 100);
  gRailStateTimings.txToRx = (TRANSITION_TIMING_BESTOF ? 0 : 182);
  gRailStateTimings.idleToTx = (TRANSITION_TIMING_BESTOF ? 0 : 100);
  gRailStateTimings.rxToTx = (TRANSITION_TIMING_BESTOF ? 0 : 192);
  status = RAIL_SetStateTiming(gRailHandle, &gRailStateTimings);
  if (status != RAIL_STATUS_NO_ERROR)
  {
#if (qPrintErrorsL1)
    app_log_warning("Warning RAIL_SetStateTiming (%d)\n", status);
#endif  // qPrintErrorsL1
  }
#endif  // qAutoTransition

  graphics_init();

  // Start reception
//  status = RAIL_StartRx(gRailHandle, CHANNEL, NULL);
//  if (status != RAIL_STATUS_NO_ERROR)
//  {
//#if (qPrintErrorsL1)
//    app_log_warning("Warning RAIL_StartRx (%d)\n", status);
//#endif  // qDebugPrintErr
//  }

  // Print Id software
  const char string[] = "\nTest EFR32xG32 - ";
  app_log_info("%s", string);

  const char string2[] = "Master";
  const char string3[] = "SLOT PROTOCOL";

  // CLI info message
  app_log_info("%s (%s) ID: 0x%llx\n", string2, string3, SYSTEM_GetUnique());
  app_log_info("--------------------------------\n");

  // Set up timers
  RAIL_ConfigMultiTimer(true);
}

// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------
/*****************************************************************************
* Checks phy setting to avoid errors at packet sending
*****************************************************************************/
static void validation_check(void)
{
  _Static_assert(SL_RAIL_UTIL_INIT_PROTOCOL_INST0_DEFAULT == SL_RAIL_UTIL_PROTOCOL_PROPRIETARY,
                 "Please use the Flex (RAIL) - Simple TRX Standards sample app instead, which is designed to show the protocol usage.");
#if defined(RAIL0_CHANNEL_GROUP_1_PROFILE_WISUN_OFDM) && !defined(HARDWARE_BOARD_HAS_EFF)
  _Static_assert(SL_RAIL_UTIL_PA_SELECTION_SUBGHZ == RAIL_TX_POWER_MODE_OFDM_PA,
                 "Please use the OFDM PA settings in the sl_rail_util_pa_config.h for OFDM phys.");
#endif
#if defined(RAIL0_CHANNEL_GROUP_1_PROFILE_WISUN_OFDM) && RAIL_SUPPORTS_EFF && defined(HARDWARE_BOARD_HAS_EFF)
  _Static_assert(SL_RAIL_UTIL_PA_SELECTION_SUBGHZ >= RAIL_TX_POWER_MODE_OFDM_PA_EFF_30DBM,
                 "Please use the OFDM PA for EFF settings in the sl_rail_util_pa_config.h for OFDM phys.");
#endif
}
