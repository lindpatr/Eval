/***************************************************************************//**
 * @file
 * @brief app_cli.c
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
#include "em_chip.h"
#include "app_log.h"
#include "sl_cli.h"

#include "app_init.h"
#include "common_stat.h"
#include "common_debug.h"


// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------
void cli_set_stat_period(sl_cli_command_arg_t *arguments);
void cli_start_process(sl_cli_command_arg_t *arguments);
void cli_req_stat(sl_cli_command_arg_t *arguments);
void cli_set_slot_time(sl_cli_command_arg_t *arguments);
void cli_set_sync_period(sl_cli_command_arg_t *arguments);
void cli_set_sync_timeout(sl_cli_command_arg_t *arguments);
void cli_set_tx_power(sl_cli_command_arg_t *arguments);


// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
/// Flag, indicating a start process request (button was pressed / CLI start request has occurred)
extern volatile bool gStartProcess;
/// Flag, indicating a request to print statistics (button was pressed / CLI statistics request has occurred)
extern volatile bool gStatReq;
/// Button pressed simulation with CLI, start process
extern volatile bool gBtnPressed;

/// Value, indicating print stat delay on CLI
extern volatile RAIL_Time_t gStatDelay;
/// Value, indicating time of a slot in the protocol on CLI
extern volatile uint32_t gTimeSlot;
/// Value, indicating sync period for Master on CLI
extern volatile RAIL_Time_t gSyncPeriod;
/// Value, indicating sync timeout for Slave on CLI
extern volatile RAIL_Time_t gSyncTimeOut;
/// Value, indicating tx power on CLI
extern volatile RAIL_TxPower_t gTxPower;

/// A static var for TX schedule config
extern RAIL_ScheduleTxConfig_t gRailScheduleCfgTX;
/// A static var that contains config data for the device
extern PROT_AddrMap_t* gDeviceCfgAddr;


// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------
// User additional CLI command
static const sl_cli_command_info_t cli_cmd__stat_period =
SL_CLI_COMMAND(cli_set_stat_period,
		"Set statistics print period (s)",
		"1 - 3180 (53 min) / 0 = default",
		{	SL_CLI_ARG_UINT16, SL_CLI_ARG_END,});

static const sl_cli_command_info_t cli_cmd__slot_time =
SL_CLI_COMMAND(cli_set_slot_time,
        "Set slot time (us)",
        "10 - 100000 (100 ms) / 0 = default",
        {   SL_CLI_ARG_UINT32, SL_CLI_ARG_END,});

static const sl_cli_command_info_t cli_cmd__sync_period =
SL_CLI_COMMAND(cli_set_sync_period,
        "Set sync period (us)",
        "400 - 1000000 (1 s) / 0 = default",
        {   SL_CLI_ARG_UINT32, SL_CLI_ARG_END,});

static const sl_cli_command_info_t cli_cmd__sync_timeout =
SL_CLI_COMMAND(cli_set_sync_timeout,
        "Set sync timeout (us)",
        "500 - 5000000 (5 s)  / 0 = default",
        {   SL_CLI_ARG_UINT32, SL_CLI_ARG_END,});

static const sl_cli_command_info_t cli_cmd__start_process =
SL_CLI_COMMAND(cli_start_process,
		"Start process",
		"",
        {SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__req_stat =
SL_CLI_COMMAND(cli_req_stat,
		"Print statistics",
		"",
        {SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__tx_power =
SL_CLI_COMMAND(cli_set_tx_power,
        "Set TX power (dbm)",
        "-60 - 60 / 0 = default",
        {   SL_CLI_ARG_INT16, SL_CLI_ARG_END,});

// User command table
const sl_cli_command_entry_t cli_my_command_table[] =
{
{ "stat", &cli_cmd__stat_period, false },
{ "slot", &cli_cmd__slot_time, false },
{ "sync", &cli_cmd__sync_period, false },
{ "sync_to", &cli_cmd__sync_timeout, false },
{ "start", &cli_cmd__start_process, false },
{ "print_stat", &cli_cmd__req_stat, false },
{ "power", &cli_cmd__tx_power, false },
{ NULL, NULL, false }, };

// User CLI grouo
sl_cli_command_group_t cli_my_command_group =
{
{ NULL },
false, cli_my_command_table };

char *strNew = "NEW";
char *strDefault = "DEFAULT";

// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------
/******************************************************************************
 * CLI - info message: Unique ID of the board
 *****************************************************************************/
void cli_info(sl_cli_command_arg_t *arguments)
{
	(void) arguments;

    app_log_info("Info:\n");
    app_log_info("  MCU Id     : 0x%llx\n", SYSTEM_GetUnique());

    app_log_info("Radio network:\n");
    app_log_info("  Enable     : %s\n", (gDeviceCfgAddr->enable ? "true" : "false"));
    app_log_info("  Role       : %s\n", gDeviceCfgAddr->name);
    app_log_info("  Addr       : %03d\n", gDeviceCfgAddr->internalAddr);
    app_log_info("  TX power   : %0.1f dB\n", (float)gTxPower/10.0f);
    app_log_info("  Transition : %s\n", (TRANSITION_TIMING_BEST_EFFORT ? "Best effort" : "Standard"));
    if (gDeviceCfgAddr->ismaster)
    {
        app_log_info("  Nbr slaves : %d\n", common_getNbrDeviceOfType(SLAVE_TYPE, ENABLED));
        app_log_info("  Sync period: %d us\n", gSyncPeriod);
        app_log_info("  ACQ time   : %d us\n", TIME_SLOT_ACQ);
        app_log_info("  TX master  : %d us\n", TIME_SLOT_MASTER_TX);
        app_log_info("  TX slave   : %d us\n", TIME_SLOT_SLAVE);
        app_log_info("  Corr. time : %d us\n", TIME_SLOT_CORR);
    }
    else
    {
        app_log_info("  Slot time  : %d us\n", gTimeSlot);
        app_log_info("  Sync TO    : %d us\n", gSyncTimeOut);
    }

    app_log_info("Options:\n");
    app_log_info("  DEBUG         : %s\n", (qDebug ? "Y" : "N"));
    app_log_info("  PRINT INFO    : %s\n", (qPrintInfo ? "Y" : "N"));
    app_log_info("  PRINT STAT    : %s\n", (qPrintStat ? "Y" : "N"));
    app_log_info("  PRINT EVENT   : %s\n", (qPrintEvents ? "Y" : "N"));
    app_log_info("  PRINT ERR L1  : %s\n", (qPrintErrorsL1 ? "Y" : "N"));
    app_log_info("  PRINT ERR L2  : %s\n", (qPrintErrorsL2 ? "Y" : "N"));
    app_log_info("  PRINT TX DATA : %s\n", (qPrintTX ? "Y" : "N"));
    app_log_info("  PRINT RX DATA : %s\n", (qPrintRX ? "Y" : "N"));
}

/******************************************************************************
 * CLI - Set period: set the statistics timeout
 *****************************************************************************/
void cli_set_stat_period(sl_cli_command_arg_t *arguments)
{
	uint16_t arg = sl_cli_get_argument_uint16(arguments, 0);
	char *str;

	if (arg > 0)
	{
	    arg = MIN(arg, STAT_PERIOD_MAX);
	    arg = MAX(arg, 1);

	    gStatDelay = (RAIL_Time_t)(arg * SEC);

	    str = strNew;
	}
	else    // Default value
	{
	    gStatDelay = (RAIL_Time_t)STAT_PERIOD_us;
        str = strDefault;
	}
	app_log_info("Info Set automatic print statistics period to %d sec (%s)\n", (gStatDelay / SEC), str);
}

/******************************************************************************
 * CLI - Set slot time: set schedule TX deadline
 *****************************************************************************/
void cli_set_slot_time(sl_cli_command_arg_t *arguments)
{
    uint32_t arg = sl_cli_get_argument_uint32(arguments, 0);
    char *str;

    if (!gStartProcess)
    {
        if (!gDeviceCfgAddr->ismaster)
        {
            if (arg > 0)
            {
                arg = MIN(arg, TIME_SLOT_MAX);
                arg = MAX(arg, TIME_SLOT_MIN);

                gTimeSlot = arg;

                str = strNew;
            }
            else    // Default value
            {
                gTimeSlot = gDeviceCfgAddr->slotTime;

                str = strDefault;
            }

            gRailScheduleCfgTX.when = gTimeSlot;

            if (gTimeSlot > 0UL)
                gRailTransitionRX.success = RAIL_RF_STATE_RX;   // RX Ok  -> RX because RAIL_StartScheduledTx is used!
            else    // TX immediate
                gRailTransitionRX.success = RAIL_RF_STATE_TX;   // RX Ok  -> TX because RAIL_StartScheduledTx is not used!

            RAIL_Status_t status = RAIL_SetRxTransitions(gRailHandle, &gRailTransitionRX);
            PrintStatus(status, "Warning RAIL_SetRxTransitions");

            app_log_info("Info Set TIME_SLOT to %d us (%s)\n", gTimeSlot, str);
            app_log_warning("Warning The user is responsible to adjust SYNC_PERIOD (current = %d us) on Master and SYNC_TIMEOUT (current = %d us) on all slaves\n", gSyncPeriod, gSyncTimeOut);
        }
        else
        {
            app_log_warning("Warning Only available on Slaves\n");
        }
    }
    else
    {
        app_log_warning("Warning Only available before starting process -> do a reset to use it!\n");
    }
}

/******************************************************************************
 * CLI - Set sync period: set sync period (periodically send a SYNC frame)
 *****************************************************************************/
void cli_set_sync_period(sl_cli_command_arg_t *arguments)
{
    uint32_t arg = sl_cli_get_argument_uint32(arguments, 0);
    char *str;

    if (!gStartProcess)
    {
        if (gDeviceCfgAddr->ismaster)
        {
            if (arg > 0)
            {
                arg = MIN(arg, SYNC_PERIOD_MAX);
                arg = MAX(arg, SYNC_PERIOD_MIN);

                gSyncPeriod = arg;

                str = strNew;
            }
            else    // Default value
            {
                // Sync period (when the master shall send a sync frame)
                gSyncPeriod = (RAIL_Time_t) (TIME_SLOT_MASTER_TX + TIME_SLOT_ACQ + (common_getNbrDeviceOfType(SLAVE_TYPE, ENABLED) * TIME_SLOT_SLAVE)  - TIME_SLOT_CORR);

                // Sync timeout (when a slave is considering no more receiving sync from a master)
                gSyncTimeOut = (RAIL_Time_t) ((float)gSyncPeriod * (1.0f+SYNC_TIMEOUT_VAR) * (float)SYNC_TIMEOUT_NB);       // Consistency with gSyncPeriod (gSyncTimeOut > gSyncPeriod) is the responsability of the dev

                str = strDefault;
            }

            app_log_info("Info Set SYNC_PERIOD to %d us (N_SLAVE = %d, TIME_SLOT_MAX = %d us) (%s)\n", gSyncPeriod, common_getNbrDeviceOfType(SLAVE_TYPE, ENABLED), common_getMaxSlotTime(), str);
            app_log_warning("Warning The user is responsible to eventually adjust TIME_SLOT (max = %d us) and SYNC_TIMEOUT (current = %d us) on all slaves\n", common_getMaxSlotTime(), gSyncTimeOut);
        }
        else
        {
            app_log_warning("Warning Command only available on Master\n");
        }
    }
    else
    {
        app_log_warning("Warning Only available before starting process -> do a hard reset to use it!\n");
    }
}

/******************************************************************************
 * CLI - Set sync period timeout: set the sync period timeout in Slave
 *****************************************************************************/
void cli_set_sync_timeout(sl_cli_command_arg_t *arguments)
{
    uint32_t arg = sl_cli_get_argument_uint32(arguments, 0);
    char *str;

    if (!gStartProcess)
    {
        if (!gDeviceCfgAddr->ismaster)
        {
            if (arg > 0)
            {
                arg = MIN(arg, SYNC_TIMOUT_MAX);
                arg = MAX(arg, SYNC_TIMOUT_MIN);

                gSyncTimeOut = arg;

                str = strNew;
            }
            else     // Default value
            {
                gSyncTimeOut = (RAIL_Time_t) ((float)gSyncPeriod * (1.0f+SYNC_TIMEOUT_VAR) * (float)SYNC_TIMEOUT_NB);       // Consistency with gSyncPeriod (gSyncTimeOut > gSyncPeriod) is the responsability of the dev

                str = strDefault;
            }

            app_log_info("Info Set SYNC_TIMEOUT to %d us (%s)\n", gSyncTimeOut, str);

            if ((gSyncTimeOut <= gSyncPeriod) || (gSyncTimeOut <= common_getMaxSlotTime()))
            {
                app_log_error("Error SYNC_TIMEOUT (%d us) is less or equal to SYNC_PERIOD (%d us) or MAX_TIME_SLOT (%d us)\n", gSyncPeriod, gSyncTimeOut, common_getMaxSlotTime());
            }
            else
            {
                app_log_warning("Warning The user is responsible to eventually adjust TIME_SLOT (max = %d us) and SYNC_PERIOD (current = %d us) on all slaves\n", common_getMaxSlotTime(), gSyncPeriod);
            }
        }
        else
        {
            app_log_warning("Warning Command only available on Slaves\n");
        }
    }
    else
    {
        app_log_warning("Warning Only available before starting process -> do a hard reset to use it!\n");
    }
}

/******************************************************************************
 * CLI - Start process: Sets a flag indicating that process started
 *****************************************************************************/
void cli_start_process(sl_cli_command_arg_t *arguments)
{
	(void) arguments;

	if (gDeviceCfgAddr->ismaster)
	{
        if (!gStartProcess)
        {
            gBtnPressed = true;
            gStartProcess = true;
        }
        else
            app_log_warning("Warning Process already started!\n");
	}
	else
    {
        app_log_warning("Warning Command only available on Master\n");
    }
}

/******************************************************************************
 * CLI - Stat request: Sets a flag indicating that a statistic is requested
 *****************************************************************************/
void cli_req_stat(sl_cli_command_arg_t *arguments)
{
	(void) arguments;

	gStatReq = true;

    if (gDeviceCfgAddr->ismaster)
    {
        app_log_info("Info Statistics print out requested synchronously on master and all slaves\n");
    }
    else
    {
        app_log_info("Info Statistics print requested\n");
        app_log_warning("Warning On slaves, statistics print is an asynchron process!\n");
    }
}

/******************************************************************************
 * CLI - Set TX power in dBm
 *****************************************************************************/
void cli_set_tx_power(sl_cli_command_arg_t *arguments)
{
    int16_t arg = sl_cli_get_argument_int16(arguments, 0);
    char *str;

    if (!gStartProcess)
    {
            if (arg != 0)
            {
                arg = MIN(arg, TX_POWER_MAX);
                arg = MAX(arg, TX_POWER_MIN);

                gTxPower = arg;

                str = strNew;
            }
            else    // Default value
            {
                gTxPower = TX_POWER;

                str = strDefault;
            }

            RAIL_Status_t status = RAIL_SetTxPowerDbm(gRailHandle, (RAIL_TxPower_t)gTxPower);
            PrintStatus(status, "Warning RAIL_SetTxPowerDbm");

            app_log_info("Info Set TX_POWER to %d dBm (%s)\n", gTxPower, str);
    }
    else
    {
        app_log_warning("Warning Only available before starting process -> do a reset to use it!\n");
    }
}

/******************************************************************************
 * CLI - send: Sets a flag indicating that a packet has to be sent
 *****************************************************************************/
void cli_send_packet(sl_cli_command_arg_t *arguments)
{
    (void) arguments;

    app_log_warning("Warning N/A in this application!\n");
}

/******************************************************************************
 * CLI - receive: Turn on/off received message
 *****************************************************************************/
void cli_receive_packet(sl_cli_command_arg_t *arguments)
{
    (void) arguments;

    app_log_warning("Warning N/A in this application!\n");
}

// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------

/*****************************************************************************
 * Command line user defined
 *****************************************************************************/
void cli_user_init(void)
{
	sl_cli_command_add_command_group(sl_cli_default_handle, &cli_my_command_group);
}
