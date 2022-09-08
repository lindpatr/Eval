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
#include "app_stat.h"


// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------
void cli_set_period(sl_cli_command_arg_t *arguments);
void cli_start_process(sl_cli_command_arg_t *arguments);
void cli_req_stat(sl_cli_command_arg_t *arguments);

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
/// Flag, indicating a start process request (button was pressed / CLI start request has occurred)
extern volatile bool gStartProcess;
/// Flag, indicating a request to print statistics (button was pressed / CLI statistics request has occurred)
extern volatile bool gStatReq;
/// Flag, indicating print stat delay on CLI
extern volatile uint32_t gStatDelay;

// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------
// User additional CLI command
static const sl_cli_command_info_t cli_cmd__delay =
SL_CLI_COMMAND(cli_set_period,
		"Set statistics print delay",
		"",
		{	SL_CLI_ARG_UINT16, SL_CLI_ARG_END,});

static const sl_cli_command_info_t cli_cmd__start_process =
SL_CLI_COMMAND(cli_start_process,
		"Start process",
		"",
        {SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__req_stat =
SL_CLI_COMMAND(cli_req_stat,
		"Request statistics print",
		"",
        {SL_CLI_ARG_END, });

// User command table
const sl_cli_command_entry_t cli_my_command_table[] =
{
{ "delay", &cli_cmd__delay, false },
{ "start", &cli_cmd__start_process, false },
{ "stat", &cli_cmd__req_stat, false },
{ NULL, NULL, false }, };

// User CLI grouo
sl_cli_command_group_t cli_my_command_group =
{
{ NULL },
false, cli_my_command_table };

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
	app_log_info("  MCU Id: 0x%llx\n", SYSTEM_GetUnique());
	app_log_info("  Role  : %s\n", "Slave");
	app_log_info("  Addr  : %03d\n", SLAVE_ADDR);		// TODO Replace SLAVE_ADDR by Addr read in the info structure
}

/******************************************************************************
 * CLI - send: Sets a flag indicating that a packet has to be sent
 *****************************************************************************/
void cli_send_packet(sl_cli_command_arg_t *arguments)
{
	(void) arguments;

	app_log_info("No more supported!\n");
}

/******************************************************************************
 * CLI - receive: Turn on/off received message
 *****************************************************************************/
void cli_receive_packet(sl_cli_command_arg_t *arguments)
{
	(void) arguments;

	app_log_info("No more supported!\n");
}

/******************************************************************************
 * CLI - Set period: set the statistics timeout
 *****************************************************************************/
void cli_set_period(sl_cli_command_arg_t *arguments)
{
	uint16_t delay = sl_cli_get_argument_uint16(arguments, 0);
	gStatDelay = delay * SEC;
	app_log_info("Print automatic statistics after %d sec\n", delay);
}

/******************************************************************************
 * CLI - Start process: Sets a flag indicating that process started
 *****************************************************************************/
void cli_start_process(sl_cli_command_arg_t *arguments)
{
	(void) arguments;
//	gStartProcess = true;
//	gElapsedTime = RAIL_GetTime();
//	app_log_info("Start process now ...\n");

	app_log_info("Only available on Master\n");
}

/******************************************************************************
 * CLI - Stat request: Sets a flag indicating that a statistic is requested
 *****************************************************************************/
void cli_req_stat(sl_cli_command_arg_t *arguments)
{
	(void) arguments;
	gStatReq = true;
	gOldElapsedTime = gElapsedTime;
	gElapsedTime = RAIL_GetTime();
	app_log_info("Statistics print requested\n");
}

// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------

/*****************************************************************************
 * Command line user defined
 *****************************************************************************/
void cli_user_init(void)
{
	sl_cli_command_add_command_group(sl_cli_default_handle, &cli_my_command_group);
}
