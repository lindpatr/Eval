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
#include "app_assert.h"

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
 * Command line user defined
 *****************************************************************************/
void cli_user_init(void);

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
/// A static var that contains config data for the device
PROT_AddrMap_t* gDeviceCfgAddr;

// Filtrage sur 1 byte avec un offset de 0 byte depuis le débute de la trame.
const RAIL_AddrConfig_t addrConfig = { .offsets = { 0, 0 }, .sizes = { 1, 0 },.matchTable = ADDRCONFIG_MATCH_TABLE_SINGLE_FIELD };
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
	char textBuf[32];

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

	GLIB_drawStringOnLine(&gGlibContext, "******************", 1, GLIB_ALIGN_CENTER, 0, 0, 0);
	GLIB_drawStringOnLine(&gGlibContext, "* SLOT PROTOCOL  *", 2, GLIB_ALIGN_CENTER, 0, 0, 0);
    snprintf(textBuf, sizeof(textBuf),   "* %s Adr %03d *", gDeviceCfgAddr->name, gDeviceCfgAddr->internalAddr);
	GLIB_drawStringOnLine(&gGlibContext, textBuf, 3, GLIB_ALIGN_CENTER, 0, 0, 0);
	GLIB_drawStringOnLine(&gGlibContext, "******************", 4, GLIB_ALIGN_CENTER, 0, 0, 0);

	GLIB_setFont(&gGlibContext, (GLIB_Font_t*) &GLIB_FontNormal8x8);
	GLIB_drawStringOnLine(&gGlibContext, "Press BTN0 on", 6, GLIB_ALIGN_CENTER, 0, 0, 0);
	GLIB_drawStringOnLine(&gGlibContext, "Master to start", 7, GLIB_ALIGN_CENTER, 0, 0, 0);
	GLIB_drawStringOnLine(&gGlibContext, "processing", 8, GLIB_ALIGN_CENTER, 0, 0, 0);

	GLIB_setFont(&gGlibContext, (GLIB_Font_t*) &GLIB_FontNarrow6x8);
	snprintf(textBuf, sizeof(textBuf), "0x%llX", SYSTEM_GetUnique());
	GLIB_drawStringOnLine(&gGlibContext, textBuf, 12, GLIB_ALIGN_CENTER, 0, 0, 0);

	// Force a redraw
	DMD_updateDisplay();
	if (GLIB_OK != status)
	{
#if (qPrintErrorsL1)
		app_log_error("Error DMD_updateDisplay (%d)\n", status);
#endif  //qPrintErrorsL1
		return;
	}
}

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

    /* Get configuration data for my device */
    gDeviceCfgAddr = common_getConfig(SYSTEM_GetUnique());
    app_assert(gDeviceCfgAddr != NULL, "Invalid device configuration data (0x%llX)\n", SYSTEM_GetUnique());
    app_assert(gDeviceCfgAddr->ismaster == false, "Error Master device with Slave software (0x%llX)\n", SYSTEM_GetUnique());

	// Get RAIL handle, used later by the application
	gRailHandle = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);

	// Turn OFF LEDs
	sl_led_turn_off(&sl_led_led0);
	sl_led_turn_off(&sl_led_led1);

	// Debug pins init + set to 0
	GPIO_PinModeSet(DEBUG_PORT, DEBUG_PIN_TX, gpioModePushPull, RESET);
	GPIO_PinModeSet(DEBUG_PORT, DEBUG_PIN_RX, gpioModePushPull, RESET);
	GPIO_PinModeSet(DEBUG_PORT, DEBUG_PIN_MISC, gpioModePushPull, RESET);

	// Read Slave address
	// TODO Read Slave address and replace SLAVE_ADDR const

	// Set timeout for scheduled RX
	// TODO Scheduled RX: à voir si utilisé?
	gRailScheduleCfgRX.start = 0;
	gRailScheduleCfgRX.startMode = RAIL_TIME_DELAY;
	gRailScheduleCfgRX.end = RX_TIMEOUT;
	gRailScheduleCfgRX.endMode = RAIL_TIME_DELAY;
	gRailScheduleCfgRX.hardWindowEnd = false;
	gRailScheduleCfgRX.rxTransitionEndSchedule = false;

	// Set timeout for scheduled TX
	// TODO Scheduled TX: à voir si utilisé?
	gRailScheduleCfgTX.when = gDeviceCfgAddr->slotTime;
	gRailScheduleCfgTX.mode = RAIL_TIME_DELAY;
	gRailScheduleCfgTX.txDuringRx = RAIL_SCHEDULED_TX_DURING_RX_POSTPONE_TX; // TODO Scheduled TX: RAIL_SCHEDULED_TX_DURING_RX_POSTPONE_TX: Risque d'être problématique: SLAVE devrait plutôt être en IDLE (mais commutation prend du temps) à voir / analyser / tester?

	// Set RX and TX transition
	// TODO RX and TX auto transition: à voir si utilisé?
	gRailTransitionRX.success = RAIL_RF_STATE_RX;   // RX Ok  -> RX
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
	if (TRANSITION_TIMING_BEST_EFFORT)
	{
		gRailStateTimings.idleToRx = 0;
		gRailStateTimings.txToRx = 0;
		gRailStateTimings.idleToTx = 0;
		gRailStateTimings.rxToTx = 0;
		status = RAIL_SetStateTiming(gRailHandle, &gRailStateTimings);
		if (status != RAIL_STATUS_NO_ERROR)
		{
#if (qPrintErrorsL1)
			app_log_warning("Warning RAIL_SetStateTiming (%d)\n", status);
#endif  // qPrintErrorsL1
		}
	}
	// else keep value from RAIL configurator!

	  // Le filtage est hardware.
	  // Configuration : RAIL Utility, Initialization (inst0) -> Radio Event Configuration -> RX Address Filtered = true/false
	  // Si Filtered = true, on recevra un event dans le callback pour indiquer que l'adresse a été filtrée et le message annulé.
	  // Si Filtered = false on NE recevra pas d'event dans le callback pour indiquer que l'adresse a été filtrée et le message annulé.
	  uint8_t addr = 0xFF;

	  // Configuration du filtrage des adresses
	  // Filtrage sur 1 byte avec un offset de 0 byte depuis le débute de la trame.
	  RAIL_ConfigAddressFilter(gRailHandle, &addrConfig);
	  // set de la valeur d'adresse à filtrer
	  RAIL_SetAddressFilterAddress(gRailHandle, 0, 1, &addr, 1);
	  // activation
	  RAIL_EnableAddressFilter(gRailHandle, true);

	// Init display
	graphics_init();

	// User commands add to CLI
	cli_user_init();

	// Start reception without timeout
	GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_RX);
	status = RAIL_StartRx(gRailHandle, CHANNEL, NULL);
	if (status != RAIL_STATUS_NO_ERROR)
	{
#if (qPrintErrorsL1)
		app_log_warning("Warning RAIL_StartRx (%d)\n", status);
#endif  // qPrintErrorsL1
	}

	// Print Id software
	const char string[] = "\nTest EFR32xG32 - ";
	app_log_info("%s", string);

	const char string3[] = "Slot Protocol";

	// CLI info message
	app_log_info("%s Addr %03d (%s)\n", gDeviceCfgAddr->name, gDeviceCfgAddr->internalAddr, string3);
	app_log_info("-----------------------------------------------\n");

	// Set up timers
	if (!RAIL_ConfigMultiTimer(true))
	{
#if (qPrintErrorsL1)
		app_log_warning("Warning RAIL_ConfigMultiTimer failed\n");
#endif  // qPrintErrorsL1
	}
}

// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------

