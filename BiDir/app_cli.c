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


// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------
/// Used for indicates the current status of forwarding rx packets on UART
#define ON   "ON"
/// Used for indicates the current status of forwarding rx packets on UART
#define OFF  "OFF"

// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------
void cli_set_period(sl_cli_command_arg_t *arguments);

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
/// Flag, indicating transmit request (button was pressed / CLI transmit request has occurred)
extern volatile bool gTX_requested;
/// Flag, indicating received packet is forwarded on CLI or not
extern volatile bool gRX_requested;
/// Flag, indicating stat period on CLI
extern volatile uint32_t gSTAT_period;

// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------

// User additional CLI command
static const sl_cli_command_info_t cli_cmd__period = \
  SL_CLI_COMMAND(cli_set_period,
                 "Set statistics period",
				 "",
                 {SL_CLI_ARG_UINT16, SL_CLI_ARG_END, });

// User command table
const sl_cli_command_entry_t cli_my_command_table[] = {
  { "period", &cli_cmd__period, false },
  { NULL, NULL, false },
};

// User CLI grouo
sl_cli_command_group_t cli_my_command_group = {
  { NULL },
  false,
  cli_my_command_table
};

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
  app_log_info("  MCU Id:       0x%llx\n", SYSTEM_GetUnique());
  app_log_info("  Fw RX Packet: %s\n", (gRX_requested == true) ? ON : OFF);
}

/******************************************************************************
 * CLI - send: Sets a flag indicating that a packet has to be sent
 *****************************************************************************/
void cli_send_packet(sl_cli_command_arg_t *arguments)
{
  (void) arguments;
  gTX_requested = true;
  app_log_info("Send packet request\n");
}

/******************************************************************************
 * CLI - Set period: set the statistics period
 *****************************************************************************/
void cli_set_period(sl_cli_command_arg_t *arguments)
{
  uint16_t period = sl_cli_get_argument_uint16(arguments, 0);
  gSTAT_period = 1000000UL * period;
  app_log_info("Send period stat to %d s\n", period);
}

/******************************************************************************
 * CLI - receive: Turn on/off received message
 *****************************************************************************/
void cli_receive_packet(sl_cli_command_arg_t *arguments)
{
  uint8_t rxForward = sl_cli_get_argument_uint8(arguments, 0);
  const char* str_rx_fw;
  if (rxForward == 0) {
    gRX_requested = false;
    str_rx_fw = OFF;
  } else {
    gRX_requested = true;
    str_rx_fw = ON;
  }

  app_log_info("Received packets: %s\n", str_rx_fw);
}

void cli_user_init(void)
{
	sl_cli_command_add_command_group(sl_cli_default_handle, &cli_my_command_group);
}
// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------
