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

// Base components
// ---------------
#include <stdint.h>                 // Standard lib
#include "sl_component_catalog.h"   // Installed components
#include "app_assert.h"             // Assert functions
#include "app_log.h"                // Log functions
#include "rail.h"                   // Radio functions
#include "rail_config.h"            // Radio config

// Additional components
// ---------------------
#include "em_system.h"              // System functions
#include "em_msc.h"                 // Memory access functions
#include "nvm3.h"                   // NVM Flash Memory functions
#include "nvm3_hal_flash.h"         // NVM Flash Memory instance
#include "printf.h"                 // Tiny printf

#include "sl_rail_util_init.h"      // Radio tools
#include "sl_rail_util_init_inst0_config.h"
                                    // Radio instance config
#include "sl_rail_util_protocol_types.h"
                                    // Radio protocol

#if defined(RAIL0_CHANNEL_GROUP_1_PROFILE_WISUN_OFDM)
#include "sl_rail_util_pa_config.h"
#include "rail_chip_specific.h"
#endif

// User components
// ---------------
#include "app_init.h"               // Initialize functions
#include "app_process.h"            // Main app
#include "common_debug.h"           // Debug functions
#include "common_stat.h"            // Statistics functions


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
// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
/// A static handle of a RAIL instance
volatile RAIL_Handle_t gRailHandle;

/// A static var for RX schedule config
//RAIL_ScheduleRxConfig_t gRailScheduleCfgRX;
/// A static var for TX schedule config
RAIL_ScheduleTxConfig_t gRailScheduleCfgTX;

/// A static var for RX transition
RAIL_StateTransitions_t gRailTransitionRX;
/// A static var for TX transition
RAIL_StateTransitions_t gRailTransitionTX;

/// A static var for state timings
RAIL_StateTiming_t gRailStateTimings;

volatile RAIL_Time_t gSyncPeriod = 0;       // Value, indicating sync period on CLI
volatile RAIL_Time_t gSyncTimeOut = 0;      // Value, indicating sync timeout for Slave on CLI
volatile uint32_t gTimeSlot = 0;            // Value, indicating time of a slot in the protocol on CLI
volatile RAIL_TxPower_t gTxPower = TX_POWER_DEF;// Value, indicating tx power on CLI

/// A static var that contains config data for the device
PROT_AddrMap_t* gDeviceCfgAddr;
uint8_t gNbOfSlave;
uint8_t gNbOfEnabledSlave;

uint8_t gAddrToPos[UINT8_MAX] = {0};


// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
//                          Private Function Definitions
// -----------------------------------------------------------------------------



/*******************************************************************************
 * @brief Read config data
 ******************************************************************************/
void get_config(void)
{
    /* Get configuration data for my device */
    gDeviceCfgAddr = common_getConfig(SYSTEM_GetUnique());

    gNbOfSlave = common_getNbrDeviceOfType(SLAVE_TYPE, ALL);
    gNbOfEnabledSlave = common_getNbrDeviceOfType(SLAVE_TYPE, ENABLED);

    // Consistencies config checks
    // Check handler
    app_assert(gDeviceCfgAddr != NULL, "Invalid device configuration data (0x%llX)\n", SYSTEM_GetUnique());
    // Do I have the right role / app??
    app_assert(gDeviceCfgAddr->ismaster == true, "Error Slave device with Master software (0x%llX)\n", SYSTEM_GetUnique());
    // Am I disabled?
    app_assert((gDeviceCfgAddr->enable), "Device disabled in network (enable = %d in addr_table)\n", gDeviceCfgAddr->enable);
    // Only one Master (enabled) is permitted
    app_assert((common_getNbrDeviceOfType(MASTER_TYPE, ENABLED) == 1), "More than one Master (%d) enabled in network (isMaster is true in addr_table)\n", common_getNbrDeviceOfType(MASTER_TYPE, ENABLED));
    // At least one Slave must be enabled
    app_assert((gNbOfEnabledSlave > 0), "No Slave (%d) enabled in network (isMaster is false in addr_table)\n", gNbOfEnabledSlave);

    for (uint8_t i = 0; i < ADDR_TRANSLATION_TABLE_SIZE; i++)
    {
        PROT_AddrMap_t* device = common_getConfigTable(i);
        if (device != NULL)
        {
            gAddrToPos[device->internalAddr] = device->posTab;
        }
    }
}

/*******************************************************************************
 * @brief Initializes the RAIL radio schedule TX/RX.
 * @note This function overwrite some settings of the radio configurator!

 ******************************************************************************/
void config_rail_schedule(void)
{
    // Set timeout for scheduled RX --> none
//    gRailScheduleCfgRX.start = 0;
//    gRailScheduleCfgRX.startMode = RAIL_TIME_DELAY;
//    gRailScheduleCfgRX.end = RX_TIMEOUT;
//    gRailScheduleCfgRX.endMode = RAIL_TIME_DELAY;
//    gRailScheduleCfgRX.hardWindowEnd = false;
//    gRailScheduleCfgRX.rxTransitionEndSchedule = false;

    // Set timeout for scheduled TX --> none
//    gRailScheduleCfgTX.when = gTimeSlot;
//    gRailScheduleCfgTX.mode = RAIL_TIME_DELAY;
//    gRailScheduleCfgTX.txDuringRx = RAIL_SCHEDULED_TX_DURING_RX_POSTPONE_TX;
}

/*******************************************************************************
 * @brief Initializes the RAIL radio transition.
 * @note This function overwrite some settings of the radio configurator!

 ******************************************************************************/
void config_rail_transition(void)
{
    RAIL_Status_t status = RAIL_STATUS_NO_ERROR;

    app_assert(gRailHandle != NULL, "Error Not a valid RAIL handle (0x%llX)\n", gRailHandle);

    // Set RX and TX transition
    gRailTransitionRX.success = RAIL_RF_STATE_RX;   // RX Ok  -> RX
    gRailTransitionRX.error = RAIL_RF_STATE_RX;     // RX Err -> RX
    gRailTransitionTX.success = RAIL_RF_STATE_RX;   // TX Ok  -> RX
    gRailTransitionTX.error = RAIL_RF_STATE_RX;     // TX Err -> RX

    status = RAIL_SetRxTransitions(gRailHandle, &gRailTransitionRX);
    PrintStatus(status, "Warning RAIL_SetRxTransitions");

    status = RAIL_SetTxTransitions(gRailHandle, &gRailTransitionTX);
    PrintStatus(status, "Warning RAIL_SetTxTransitions");

    // State timings
    if (TRANSITION_TIMING_BEST_EFFORT)
    {
        gRailStateTimings.idleToRx = 0;
        gRailStateTimings.txToRx = 0;
        gRailStateTimings.idleToTx = 0;
        gRailStateTimings.rxToTx = 0;
        status = RAIL_SetStateTiming(gRailHandle, &gRailStateTimings);
        PrintStatus(status, "Warning RAIL_SetStateTiming");
    }
    // else keep value from RAIL configurator!
}

/*******************************************************************************
 * @brief Initializes the RAIL radio events for callback.
 * @note This function overwrite some settings of the radio configurator!

 ******************************************************************************/
void config_rail_events_callback(void)
{
    app_assert(gRailHandle != NULL, "Error Not a valid RAIL handle (0x%llX)\n", gRailHandle);

    RAIL_Status_t status = RAIL_ConfigEvents(gRailHandle,
                                            // Active (= enable in radio configurator)
                                            // TX Ok
                                            RAIL_EVENT_TX_PACKET_SENT
                                            //| RAIL_EVENT_TX_STARTED                 // --> only for debugging purposes (DEBUG_PIN_MISC)

                                            // TX errors
                                            | RAIL_EVENT_TX_ABORTED
                                            | RAIL_EVENT_TX_BLOCKED
                                            | RAIL_EVENT_TX_UNDERFLOW
                                            | RAIL_EVENT_TX_CHANNEL_BUSY
                                            // | RAIL_EVENT_TX_SCHEDULED_TX_MISSED  --> not enabled in Master

                                            // RX Ok
                                            | RAIL_EVENT_RX_PACKET_RECEIVED
                                            // RX errors
                                            | RAIL_EVENT_RX_PACKET_ABORTED
                                            | RAIL_EVENT_RX_FRAME_ERROR
                                            | RAIL_EVENT_RX_FIFO_OVERFLOW,
                                            // | RAIL_EVENT_RX_ADDRESS_FILTERED      --> not enabled in Masternot considered as an RX error
                                            // | RAIL_EVENT_RX_SCHEDULED_RX_MISSED   --> not enabled in this app

                                            // generate a callback
                                            RAIL_EVENTS_TX_COMPLETION       // RAIL_EVENT_TX_SCHEDULED_TX_MISSED:   part of the callback through TX_COMPLETION but not part of the enabled event!
                                            //| RAIL_EVENT_TX_STARTED                 // --> only for debugging purposes (DEBUG_PIN_MISC)
                                            | RAIL_EVENTS_RX_COMPLETION     // RAIL_EVENT_RX_ADDRESS_FILTERED:      part of the callback through RX_COMPLETION but not part of the enabled event!
                                                                            // RAIL_EVENT_RX_SCHEDULED_RX_MISSED:   part of the callback through RX_COMPLETION but not part of the enabled event!
                                            | RAIL_EVENT_CAL_NEEDED);

    PrintStatus(status, "Warning RAIL_ConfigEvents");
}

/*******************************************************************************
 * @brief Initializes the RAIL radio.
 * @note This function overwrite some settings of the radio configurator!

 ******************************************************************************/
void config_rail(void)
{
    RAIL_Status_t status = RAIL_STATUS_NO_ERROR;

    // Get RAIL handle, used later by the application
    gRailHandle = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);
    app_assert(gRailHandle != NULL, "Error No valid RAIL handle for SL_RAIL_UTIL_HANDLE_INST0 (0x%llX)\n", gRailHandle);

    config_rail_schedule();
    config_rail_transition();
    config_rail_events_callback();

    // TX power
    status = RAIL_SetTxPowerDbm(gRailHandle, (RAIL_TxPower_t)gTxPower);
    PrintStatus(status, "Warning RAIL_SetTxPowerDbm");

    // RX options
    status = RAIL_ConfigRxOptions(gRailHandle, RAIL_RX_OPTION_REMOVE_APPENDED_INFO, (RSSI_LQI_MES ? 0 : RAIL_RX_OPTION_REMOVE_APPENDED_INFO));
    PrintStatus(status, "Warning RAIL_ConfigRxOptions");
}

/*******************************************************************************
 * @brief Initialize some protocol global vars.
 ******************************************************************************/
void config_protocol(void)
{
    // Slot start time (when a slave shall transmit its data)
    gTimeSlot  = gDeviceCfgAddr->slotTime;                                                     // Consistency between all devices participating to the network is the responsability of the dev

    // Sync period (when the master shall send a sync frame)
    gSyncPeriod = (RAIL_Time_t) (TIME_SLOT_MASTER_TX + TIME_SLOT_ACQ + (gNbOfEnabledSlave * TIME_SLOT_SLAVE)  - TIME_SLOT_CORR);

    // Sync timeout (when a slave is considering no more receiving sync from a master)
    gSyncTimeOut = (RAIL_Time_t) ((float)gSyncPeriod * (1.0f+SYNC_TIMEOUT_VAR) * (float)SYNC_TIMEOUT_NB);       // Consistency with gSyncPeriod (gSyncTimeOut > gSyncPeriod) is the responsability of the dev

//    if (!gDeviceCfgAddr->ismaster && (gDeviceCfgAddr->internalAddr > 1))
//    {
//        app_assert(gTimeSlot > 0, "Error TIME_SLOT shall be greater than %d\n", gTimeSlot);
//    }

    app_assert(gSyncPeriod > 0, "Error SYNC_PERIOD shall be greater than %d\n", gSyncPeriod);
    app_assert(gSyncTimeOut > gSyncPeriod, "Error SYNC_TIMEOUT shall be greater than %d\n", gSyncPeriod);
}

/*******************************************************************************
 * @brief Initializes the needed GPIO.
 * @note This function overwrite some settings of the radio configurator!
gSyncPeriod
 ******************************************************************************/
void config_gpio(void)
{
    // Debug pins init + set to 0
    InitGPIODebug();
}

/*******************************************************************************
 * @brief Initializes the graphics stack.
 * @note This function will /hang/ if errors occur (usually
 *       caused by faulty displays.
 ******************************************************************************/
void graphics_init(void)
{
#if (SL_BOARD_ENABLE_DISPLAY)
	EMSTATUS status;
	char textBuf[32];

	/* Initialize the DMD module for the DISPLAY device driver. */
	status = DMD_init(0);
	PrintStatus(status, "Error DMD_init");

	status = GLIB_contextInit(&gGlibContext);
	PrintStatus(status, "Error GLIB_contextInit");

	gGlibContext.backgroundColor = White;
	gGlibContext.foregroundColor = Black;

	// Clear what's currently on screen
	GLIB_clear(&gGlibContext);

	GLIB_setFont(&gGlibContext, (GLIB_Font_t*) &GLIB_FontNarrow6x8);

    GLIB_drawStringOnLine(&gGlibContext, "******************", 1, GLIB_ALIGN_CENTER, 0, 0, 0);
    GLIB_drawStringOnLine(&gGlibContext, "* SLOT PROTOCOL  *", 2, GLIB_ALIGN_CENTER, 0, 0, 0);
    GLIB_drawStringOnLine(&gGlibContext, "******************", 3, GLIB_ALIGN_CENTER, 0, 0, 0);

    GLIB_setFont(&gGlibContext, (GLIB_Font_t*) &GLIB_FontNormal8x8);

    snprintf(textBuf, sizeof(textBuf),   "%s Adr %03d", gDeviceCfgAddr->name, gDeviceCfgAddr->internalAddr);
    GLIB_drawStringOnLine(&gGlibContext, textBuf, 5, GLIB_ALIGN_CENTER, 0, 0, 0);

    GLIB_drawStringOnLine(&gGlibContext, "Press BTN0 on", 7, GLIB_ALIGN_CENTER, 0, 0, 0);
    GLIB_drawStringOnLine(&gGlibContext, "to start", 8, GLIB_ALIGN_CENTER, 0, 0, 0);
    GLIB_drawStringOnLine(&gGlibContext, "processing", 9, GLIB_ALIGN_CENTER, 0, 0, 0);

    GLIB_setFont(&gGlibContext, (GLIB_Font_t*) &GLIB_FontNarrow6x8);
    snprintf(textBuf, sizeof(textBuf), "0x%llX", SYSTEM_GetUnique());
    GLIB_drawStringOnLine(&gGlibContext, textBuf, 12, GLIB_ALIGN_CENTER, 0, 0, 0);

	// Force a redraw
	DMD_updateDisplay();
	PrintStatus(status, "Error DMD_updateDisplay");
#endif // SL_BOARD_ENABLE_DISPLAY
}

/*******************************************************************************
 * @brief Print out welcome info on serial COM.
 ******************************************************************************/
void serial_init(void)
{
    RAIL_Status_t status = RAIL_STATUS_NO_ERROR;
    uint16_t channel;

    // Print Id software
    char string[80];

    sprintf(string, "\nSlot Protocol - %s (Addr #%03d)", gDeviceCfgAddr->name, gDeviceCfgAddr->internalAddr);
    PrintInfo(string);
    PrintInfo("---------------------------------");
    sprintf(string, "MCU ID     : 0x%llx", SYSTEM_GetUnique());
    PrintInfo(string);
    sprintf(string, "Sys clock  : %0.3f MHz", SystemSYSCLKGet()/1000000.0f);
    PrintInfo(string);

    status = RAIL_IsValidChannel(gRailHandle, CHANNEL);
    if (status != RAIL_STATUS_NO_ERROR)
    {
        sprintf(string, "%d isn't a valid channel", CHANNEL);
        PrintStatus(status, string);
    }
    else
    {
        status = RAIL_GetChannel(gRailHandle, &channel);

        if (status != RAIL_STATUS_NO_ERROR)
        {
            sprintf(string, "Warning RAIL_GetChannel (%d)", CHANNEL);
            PrintStatus(status, string);
        }
        else
        {
            RAIL_ChannelMetadata_t channelMetadata;
            uint16_t length = sizeof(RAIL_ChannelMetadata_t);
            channel = CHANNEL;

            status = RAIL_GetChannelMetadata(gRailHandle, &channelMetadata, &length, channel, channel);
            PrintStatus(status, "Warning RAIL_GetChannelMetadata");

            sprintf(string, "Channel    : %d (%0.3f GHz)", channel, (channelMetadata.frequency/1000000000.0f));
            PrintInfo(string);
        }
    }

    sprintf(string, "Protocol (slot = %d us, sync period = %d us, sync to = %d us)\n", gTimeSlot, gSyncPeriod, gSyncTimeOut);
    PrintInfo(string);
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
	validation_check();

	// Read config parameters
	get_config();

	// Config GPIO
	config_gpio();

    // Config protocol
    config_protocol();

	// Config RAIL
	config_rail();

	// User commands add to CLI
	cli_user_init();

    // Enable and display welcome page on LCD
    graphics_init();

    // Print out welcome strings on serial COM
    serial_init();

    // Init statistics
    StatInit();

	// Set up timers
	bool ret = RAIL_ConfigMultiTimer(true);
	PrintStatus((ret == false), "Warning RAIL_ConfigMultiTimer failed");
}

