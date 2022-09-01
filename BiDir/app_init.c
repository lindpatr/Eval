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
#if defined(RAIL0_CHANNEL_GROUP_1_PROFILE_WISUN_OFDM)
#include "sl_rail_util_pa_config.h"
#include "rail_chip_specific.h"
#endif

#if defined(SL_CATALOG_KERNEL_PRESENT)
#include "app_task_init.h"
#endif

#include "em_gpio.h"
#include "app_init.h"

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
/// A static handle of a RAIL instance
volatile RAIL_Handle_t gRailHandle;
/// A static var for RX schedule config
RAIL_ScheduleRxConfig_t gRailScheduleCfgRX;
/// A static var for TX schedule config
RAIL_ScheduleRxConfig_t gRailScheduleCfgTX;

// -----------------------------------------------------------------------------
//                          Private Function Definitions
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------
/******************************************************************************
 * Print sample app name
 *****************************************************************************/
SL_WEAK void print_sample_app_name(const char *app_name)
{
	app_log_info("%s\n", app_name);
}

/******************************************************************************
 * The function is used for some basic initialization related to the app.
 *****************************************************************************/
void app_init(void)
{
	validation_check();

	// Get RAIL handle, used later by the application
	gRailHandle = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);

	// Set up TX FIFO
	set_up_tx_fifo();

	// Turn OFF LEDs
	sl_led_turn_off(&sl_led_led0);
	sl_led_turn_off(&sl_led_led1);

	// Debug pins
	GPIO_PinModeSet(DEBUG_PORT, DEBUG_PIN_TX, gpioModePushPull, RESET);
	GPIO_PinModeSet(DEBUG_PORT, DEBUG_PIN_RX, gpioModePushPull, RESET);
	GPIO_PinModeSet(DEBUG_PORT, DEBUG_PIN_MISC, gpioModePushPull, RESET);

	// LCD start
	graphics_init();

	// Set timeout for scheduled RX
	gRailScheduleCfgRX.start = 0;
	gRailScheduleCfgRX.startMode = RAIL_TIME_DELAY;
	gRailScheduleCfgRX.end = RX_TIMEOUT;
	gRailScheduleCfgRX.endMode = RAIL_TIME_DELAY;
	gRailScheduleCfgRX.hardWindowEnd = false;
	gRailScheduleCfgRX.rxTransitionEndSchedule = false;

	// Set scheduled TX
	gRailScheduleCfgTX.start = TX_START;
	gRailScheduleCfgTX.startMode = RAIL_TIME_DELAY;
	gRailScheduleCfgTX.end = TX_START + TX_TIMEOUT;
	gRailScheduleCfgTX.endMode = RAIL_TIME_DELAY;
	gRailScheduleCfgTX.hardWindowEnd = false;
	gRailScheduleCfgTX.rxTransitionEndSchedule = false;

	// Print Id software
	const char string[] = "\nTest EFR32xG32 - ";
	app_log_info("%s", string);
#if (qMaster)
	const char string2[] = "Master";
#else // !qMaster
	const char string2[] = "Slave";
#endif  // qMaster
	const char string3[] = "BiDir";

	// CLI info message
	app_log_info("%s (%s)\n", string2, string3);
	app_log_info("--------------------------------\n");

	// Set up timers
	RAIL_ConfigMultiTimer(true);

#if (!qMaster)
	// Enable Start reception (without timeout)
	RAIL_Status_t status = RAIL_StartRx(gRailHandle, CHANNEL, NULL);
	if (status != RAIL_STATUS_NO_ERROR)
	{
#if (qDebugPrintErr)
		app_log_warning("Warning RAIL_StartRx (%d)\n", status);
#endif	// qDebugPrintErr
	}
#endif	// !qMaster
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
