/***************************************************************************//**
 * @file
 * @brief app_tick.c
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
#include "app_assert.h"
#include "app_log.h"
#include "rail.h"
#include "app_process.h"
#include "sl_simple_button_instances.h"
#include "sl_simple_led_instances.h"
#include "rail_config.h"
#include "sl_flex_packet_asm.h"

#include "app_init.h"

#if (qUseDisplay)
#include "dmd.h"
#include "glib.h"
#include "printf.h"
#endif  // qUseDisplay

#include "em_gpio.h"

// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------
/// Size of RAIL RX/TX FIFO
#define RAIL_FIFO_SIZE (512U)
/// Transmit data length
#define TX_PAYLOAD_LENGTH (06U)

#define kStatPeriod	20000000

/// State machine of simple_trx
typedef enum
{
	kNone, kInit, kStop,

	kWaitRx, kMsgReceived, kErrorRx, kSwitchRx,

	kSendMsg, kMsgSent, kErrorTx, kSwitchTx,

	kErrorCal
} StateEnum;

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
/// The variable shows the actual state of the state machine
static volatile StateEnum gProtocolState = kNone;
static volatile StateEnum gPrevProtocolState = kNone;

/// Contains the last RAIL Rx/Tx error events
static volatile uint64_t error_code = 0;

/// Contains the status of RAIL Calibration
static volatile RAIL_Status_t calibration_status = 0;

static RAIL_Time_t timeout = 0UL;

/// Receive and Send FIFO
static uint8_t rx_fifo[RAIL_FIFO_SIZE];

static union
{
	// Used to align this buffer as needed
	RAIL_FIFO_ALIGNMENT_TYPE align[RAIL_FIFO_SIZE / RAIL_FIFO_ALIGNMENT];
	uint8_t fifo[RAIL_FIFO_SIZE];
} tx_fifo;

/// Transmit packet
static uint8_t out_packet[TX_PAYLOAD_LENGTH] =
{ 0x0F, 0x16, 0x11, 0x22, 0x33, 0x44 };

/// Flags to update state machine from interrupt
static volatile bool packet_recieved = false;
static volatile bool packet_sent = false;
static volatile bool rx_error = false;
static volatile bool tx_error = false;
static volatile bool cal_error = false;

/// TX and RX counters
static uint32_t RX_counter = 0UL;
static uint32_t TX_counter = 0UL;
static uint32_t old_TX_counter = 0UL;
static uint32_t old_RX_counter = 0UL;
static uint32_t prev_RX_counter = 0UL;

/// Tables for error statistics
static uint32_t TX_tab[3] =
{ 0 };    // 0 = TX_OK, 1 = TX_Err, 2 = ND
static uint32_t RX_tab[3] =
{ 0 };    // 0 = RX_OK, 1 = RX_Err, 2 = RX_TimeOut
static uint32_t CAL_tab[3] =
{ 0 };   // 0 = ND,    1 = CAL_Err, 2 = ND

/// LCD variables
/// Context used all over the graphics
#if (qUseDisplay)
static GLIB_Context_t glib_context;
#endif  // qUseDisplay

/// Flag, indicating transmit request (button was pressed / CLI transmit request has occurred)
volatile bool tx_requested = false;
/// Flag, indicating received packet is forwarded on CLI or not
volatile bool rx_requested = true;
volatile bool rx_first = false;

RAIL_ScheduleRxConfig_t	rail_schedule_cfg;
RAIL_SchedulerInfo_t rail_schedule_info;

// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------

/******************************************************************************
 * The API helps to unpack the received packet, point to the payload and returns the length.
 *
 * @param rx_destination Where should the full packet be unpacked
 * @param packet_information Where should all the information of the packet stored
 * @param start_of_payload Pointer where the payload starts
 * @return The length of the received payload
 *****************************************************************************/
static uint16_t unpack_packet(uint8_t *rx_destination, const RAIL_RxPacketInfo_t *packet_information, uint8_t **start_of_payload);

/******************************************************************************
 * The API prepares the packet for sending and load it in the RAIL TX FIFO
 *
 * @param grail_handle Which rail handlers should be used for the TX FIFO writing
 * @param out_data The payload buffer
 * @param length The length of the payload
 *****************************************************************************/
static void prepare_package(uint8_t *out_data, uint16_t length);

// -----------------------------------------------------------------------------
//                          Private Function Declarations
// -----------------------------------------------------------------------------
/******************************************************************************
 * SetState : change state machine and backup previous state machine
 *****************************************************************************/
void SetState(StateEnum state)
{
	gPrevProtocolState = gProtocolState;
	gProtocolState = state;
}

/******************************************************************************
 * CfgRxMode : change to RX mode
 *****************************************************************************/
void CfgRxMode(void)
{
	// Start RX and check result
	rail_schedule_cfg.start = 0;
	rail_schedule_cfg.startMode = RAIL_TIME_DELAY;
	rail_schedule_cfg.end = 1500;
	rail_schedule_cfg.endMode = RAIL_TIME_DELAY;
	rail_schedule_cfg.hardWindowEnd = false;
	rail_schedule_cfg.rxTransitionEndSchedule = false;

//	RAIL_StateTiming_t timing;
//
//	timing.rxSearchTimeout = (RAIL_TransitionTime_t)1500;

//	RAIL_Status_t rail_status = RAIL_SetStateTiming(grail_handle, &timing);
	// For oscillo debug
	GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_RX);

	RAIL_Status_t rail_status = RAIL_ScheduleRx(grail_handle, CHANNEL, &rail_schedule_cfg, NULL);


	if (rail_status == RAIL_STATUS_NO_ERROR)
	{
		//rail_status = RAIL_StartRx(grail_handle, CHANNEL, NULL);

		if (rail_status != RAIL_STATUS_NO_ERROR)
		{
			app_log_warning("Warning RAIL_StartRx (%d)\n", rail_status);
		}
	}
	else
		app_log_warning("Warning RAIL_ScheduleRx (%d)\n", rail_status);
}

/******************************************************************************
 * CfgTxMode : prepare buffer and change to TX mode
 *****************************************************************************/
void CfgTxMode(void)
{
	// Data
	TX_counter++;

	// Initialize buffer
	out_packet[2] = (TX_counter & 0x000000FF) >> 0;
	out_packet[3] = (TX_counter & 0x0000FF00) >> 8;
	out_packet[4] = (TX_counter & 0x00FF0000) >> 16;
	out_packet[5] = (TX_counter & 0xFF000000) >> 24;

	// Initialize radio buffer
	prepare_package(out_packet, sizeof(out_packet));

	// For oscillo debug
	GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_TX);

	// Start TX and check result
	RAIL_Status_t rail_status = RAIL_StartTx(grail_handle, CHANNEL,
	RAIL_TX_OPTIONS_DEFAULT, NULL);

	if (rail_status != RAIL_STATUS_NO_ERROR)
	{
		app_log_warning("Warning RAIL_StartTx (%d)\n", rail_status);
	}
}

/******************************************************************************
 * DisplayReceivedMsg : print received data
 *****************************************************************************/
void DisplayReceivedMsg(const uint8_t *const rx_buffer, uint16_t length)
{
	// Backup previous data
	prev_RX_counter = RX_counter;
	// Extract received data
	RX_counter = (uint32_t) ((rx_buffer[2] << 0) + (rx_buffer[3] << 8) + (rx_buffer[4] << 16) + (rx_buffer[5] << 24));

#if (qPrintRX)
	// Print received data on serial COM
	app_log_info("RX: ");
	for (uint16_t i = 0; i < length; i++) {
		app_log_info("0x%02X, ", rx_buffer[i]);
	}
	app_log_info("\n");
#endif

}

/******************************************************************************
 * DisplaySentMsg : print sent data
 *****************************************************************************/
void DisplaySentMsg(void)
{
#if (qPrintTX)
	// Print sent data on serial COM
	app_log_info("TX: %d\n",TX_counter);
#endif
}

/******************************************************************************
 * DecodeReceivedMsg : decode received data
 *****************************************************************************/
void DecodeReceivedMsg(void)
{
	// RAIL Rx packet handles
	RAIL_RxPacketHandle_t rx_packet_handle;
	RAIL_RxPacketInfo_t packet_info;
	// Status indicator of the RAIL API calls
	RAIL_Status_t rail_status = RAIL_STATUS_NO_ERROR;

	// Packet received:
	//  - Check whether RAIL_HoldRxPacket() was successful, i.e. packet handle is valid
	//  - Copy it to the application FIFO
	//  - Free up the radio FIFO
	//  - Return to app IDLE state (RAIL will automatically switch back to Rx radio state)
	rx_packet_handle = RAIL_GetRxPacketInfo(grail_handle,
	RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);
	while (rx_packet_handle != RAIL_RX_PACKET_HANDLE_INVALID)
	{
		uint8_t *start_of_packet = 0;
		uint16_t packet_size = unpack_packet(rx_fifo, &packet_info, &start_of_packet);
		rail_status = RAIL_ReleaseRxPacket(grail_handle, rx_packet_handle);
		if (rail_status != RAIL_STATUS_NO_ERROR)
		{
			app_log_warning("Error ReleaseRxPacket (%d)\n", rail_status);
		}
		if (rx_requested)
		{
			DisplayReceivedMsg(start_of_packet, packet_size);
			if ((RX_counter - prev_RX_counter) > 1)
				RX_tab[2]++;
			else
				RX_tab[0]++;
		}
		sl_led_toggle(&sl_led_led0);
		rx_packet_handle = RAIL_GetRxPacketInfo(grail_handle,
		RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);
	}
	GPIO_PinOutClear(DEBUG_PORT, DEBUG_PIN_RX);
}

/******************************************************************************
 * DisplayRx : print and display RX statistics
 *****************************************************************************/
void DisplayRx(void)
{
	// Statistics
	float localStat = (float) (RX_counter - old_RX_counter) / (float) (kStatPeriod / 1000000);
	float localStat3 = 100.0f * (float) (RX_tab[1] + RX_tab[2]) / (float) RX_counter;

	// Print on serial COM
	app_log_info("RX Error: %0.3f%% (Err:%d/TO:%d)\n", localStat3, RX_tab[1], RX_tab[2]);
	app_log_info("RX Rate : %0.2f msg/s\n", localStat);

#if (qUseDisplay)
	// Print on LCD display
	char textBuf[32];

	GLIB_setFont(&glib_context, (GLIB_Font_t*) &GLIB_FontNormal8x8);
	GLIB_drawStringOnLine(&glib_context, "--- STATS RX ---", 5, GLIB_ALIGN_CENTER, 0, 0, true);
	snprintf(textBuf, sizeof(textBuf), "OK    %lu          ", RX_tab[0]);
	GLIB_drawStringOnLine(&glib_context, textBuf, 6, GLIB_ALIGN_LEFT, 0, 0, true);
	snprintf(textBuf, sizeof(textBuf), "Error %lu          ", RX_tab[1]);
	GLIB_drawStringOnLine(&glib_context, textBuf, 7, GLIB_ALIGN_LEFT, 0, 0, true);
	snprintf(textBuf, sizeof(textBuf), "TO    %lu           ", RX_tab[2]);
	GLIB_drawStringOnLine(&glib_context, textBuf, 8, GLIB_ALIGN_LEFT, 0, 0, true);
	snprintf(textBuf, sizeof(textBuf), "Error %0.3f%%      ", localStat3);
	GLIB_drawStringOnLine(&glib_context, textBuf, 9, GLIB_ALIGN_LEFT, 0, 0, true);
	snprintf(textBuf, sizeof(textBuf), "Rate  %0.1f fs     ", localStat);
	GLIB_drawStringOnLine(&glib_context, textBuf, 10, GLIB_ALIGN_LEFT, 0, 0, true);

	GLIB_setFont(&glib_context, (GLIB_Font_t*) &GLIB_FontNarrow6x8);
	GLIB_drawStringOnLine(&glib_context, ">>> RUNNING >>>", 12, GLIB_ALIGN_CENTER, 0, 0, true);

	// Force a redraw
	DMD_updateDisplay();
#endif  // qUseDisplay

	old_RX_counter = RX_counter;
}

/******************************************************************************
 * DisplayTx : print and display TX statistics
 *****************************************************************************/
void DisplayTx(void)
{
	// Statistics
	float localStat = (float) (TX_counter - old_TX_counter) / (float) (kStatPeriod / 1000000);
	float localStat2 = (10.0f * 10100.0f) / localStat;
	float localStat3 = 100.0f * (float) (TX_tab[1] + TX_tab[2]) / (float) TX_counter;

	// Print on serial COM
	app_log_info("TX Error: %0.3f%% (Err:%d/TO:%d)\n", localStat3, TX_tab[1], TX_tab[2]);
	app_log_info("TX Rate :    %0.2f msg/s (loop: %0.2f ms)\n", localStat, localStat2);

#if (qUseDisplay)
	// Print on LCD display
	char textBuf[32];

	GLIB_setFont(&glib_context, (GLIB_Font_t*) &GLIB_FontNormal8x8);
	GLIB_drawStringOnLine(&glib_context, "--- STATS TX ---", 5, GLIB_ALIGN_CENTER, 0, 0, true);
	snprintf(textBuf, sizeof(textBuf), "OK    %lu          ", TX_tab[0]);
	GLIB_drawStringOnLine(&glib_context, textBuf, 6, GLIB_ALIGN_LEFT, 0, 0, true);
	snprintf(textBuf, sizeof(textBuf), "Error %lu          ", TX_tab[1]);
	GLIB_drawStringOnLine(&glib_context, textBuf, 7, GLIB_ALIGN_LEFT, 0, 0, true);
	snprintf(textBuf, sizeof(textBuf), "Error %0.3f%%      ", localStat3);
	GLIB_drawStringOnLine(&glib_context, textBuf, 8, GLIB_ALIGN_LEFT, 0, 0, true);
	snprintf(textBuf, sizeof(textBuf), "Rate  %0.1f fs     ", localStat);
	GLIB_drawStringOnLine(&glib_context, textBuf, 9, GLIB_ALIGN_LEFT, 0, 0, true);
	snprintf(textBuf, sizeof(textBuf), "Loop  %0.1f ms     ", localStat2);
	GLIB_drawStringOnLine(&glib_context, textBuf, 10, GLIB_ALIGN_LEFT, 0, 0, true);

	// Force a redraw
	DMD_updateDisplay();
#endif  // qUseDisplay

	old_TX_counter = TX_counter;
}

/******************************************************************************
 * DisplayStat : print and display statistics
 *****************************************************************************/
void DisplayStat(void)
{
	// Statistics
	float localStat = (float) (TX_counter + RX_counter - old_TX_counter - old_RX_counter) / (float) (kStatPeriod / 1000000);
	float localStat2 = (10.0f * 10100.0f) / localStat;
	float localStat3 = 100.0f * (float) (TX_tab[1] + TX_tab[2]) / (float) TX_counter;
	float localStat4 = 100.0f * (float) (RX_tab[1] + RX_tab[2]) / (float) RX_counter;

	// Print on serial COM
	app_log_info("TX Error: %0.3f%% (Err:%d/TO:%d)\n", localStat3, TX_tab[1], TX_tab[2]);
	app_log_info("RX Error: %0.3f%% (Err:%d/TO:%d)\n", localStat4, RX_tab[1], RX_tab[2]);
	app_log_info("Rate :    %0.2f msg/s (loop: %0.2f ms)\n", localStat, localStat2);

#if (qUseDisplay)
	// Print on LCD display
	char textBuf[32];

	GLIB_setFont(&glib_context, (GLIB_Font_t*) &GLIB_FontNormal8x8);
	snprintf(textBuf, sizeof(textBuf), "TX OK %lu          ", TX_tab[0]);
	GLIB_drawStringOnLine(&glib_context, textBuf, 5, GLIB_ALIGN_LEFT, 0, 0, true);
	snprintf(textBuf, sizeof(textBuf), "RX OK %lu          ", RX_tab[0]);
	GLIB_drawStringOnLine(&glib_context, textBuf, 6, GLIB_ALIGN_LEFT, 0, 0, true);
	snprintf(textBuf, sizeof(textBuf), "TX Err %0.3f%%      ", localStat3);
	GLIB_drawStringOnLine(&glib_context, textBuf, 7, GLIB_ALIGN_LEFT, 0, 0, true);
	snprintf(textBuf, sizeof(textBuf), "RX Err %0.3f%%      ", localStat4);
	GLIB_drawStringOnLine(&glib_context, textBuf, 8, GLIB_ALIGN_LEFT, 0, 0, true);
	snprintf(textBuf, sizeof(textBuf), "Rate  %0.1f fs     ", localStat);
	GLIB_drawStringOnLine(&glib_context, textBuf, 9, GLIB_ALIGN_LEFT, 0, 0, true);
	snprintf(textBuf, sizeof(textBuf), "Loop  %0.1f ms     ", localStat2);
	GLIB_drawStringOnLine(&glib_context, textBuf, 10, GLIB_ALIGN_LEFT, 0, 0, true);

	// Force a redraw
	DMD_updateDisplay();
#endif  // qUseDisplay

	old_TX_counter = TX_counter;
	old_RX_counter = RX_counter;
}

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
#if (qUseDisplay)
	EMSTATUS status;

	/* Initialize the DMD module for the DISPLAY device driver. */
	status = DMD_init(0);
	if (DMD_OK != status)
	{
		while (1)
			;
	}

	status = GLIB_contextInit(&glib_context);
	if (GLIB_OK != status)
	{
		while (1)
			;
	}

	glib_context.backgroundColor = White;
	glib_context.foregroundColor = Black;

	// Clear what's currently on screen
	GLIB_clear(&glib_context);

	GLIB_setFont(&glib_context, (GLIB_Font_t*) &GLIB_FontNarrow6x8);

	GLIB_drawStringOnLine(&glib_context, "****************", 1, GLIB_ALIGN_CENTER, 0, 0, 0);
	GLIB_drawStringOnLine(&glib_context, "* TEST BIDIR  *", 2, GLIB_ALIGN_CENTER, 0, 0, 0);
#if (qMaster)
	GLIB_drawStringOnLine(&glib_context, "*   (Master)   *", 3, GLIB_ALIGN_CENTER, 0, 0, 0);
#else
	GLIB_drawStringOnLine(&glib_context, "*   (Slave)    *", 3, GLIB_ALIGN_CENTER, 0, 0, 0);
#endif  // qMaster
	GLIB_drawStringOnLine(&glib_context, "****************", 4, GLIB_ALIGN_CENTER, 0, 0, 0);
	GLIB_setFont(&glib_context, (GLIB_Font_t*) &GLIB_FontNormal8x8);
	GLIB_drawStringOnLine(&glib_context, "Press BTN0", 6, GLIB_ALIGN_CENTER, 0, 0, 0);
	GLIB_drawStringOnLine(&glib_context, "on Master to", 7, GLIB_ALIGN_CENTER, 0, 0, 0);
	GLIB_drawStringOnLine(&glib_context, "start / stop TX", 8, GLIB_ALIGN_CENTER, 0, 0, 0);

	GLIB_setFont(&glib_context, (GLIB_Font_t*) &GLIB_FontNarrow6x8);
	GLIB_drawStringOnLine(&glib_context, "*** STOPPED ***", 12, GLIB_ALIGN_CENTER, 0, 0, 0);

	// Force a redraw
	DMD_updateDisplay();
#endif  // qUseDisplay
}

// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------
/******************************************************************************
 * Application state machine, called infinitely
 *****************************************************************************/
void app_process_action(void)
{
	RAIL_Status_t calibration_status_buff = RAIL_STATUS_NO_ERROR;

	if (packet_recieved)
	{
		packet_recieved = false;
		GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_RX);
		if (!rx_first)
		{
			old_TX_counter = TX_counter;
			old_RX_counter = RX_counter;
			rx_first = true;
		}
		SetState(kMsgReceived);
	}
	else if (packet_sent)
	{
		packet_sent = false;
		GPIO_PinOutClear(DEBUG_PORT, DEBUG_PIN_TX);
		TX_tab[0]++;
		SetState(kMsgSent);
	}
	else if (rx_error)
	{
		rx_error = false;
		RX_tab[1]++;
		SetState(kErrorRx);
	}
	else if (tx_error)
	{
		tx_error = false;
		TX_tab[1]++;
		SetState(kErrorTx);
	}
	else if (cal_error)
	{
		cal_error = false;
		CAL_tab[1]++;
		SetState(kErrorCal);
	}

	switch (gProtocolState)
	{
	case kNone:
		// Wait TX start
		break;

	case kInit:
#if (qMaster)
		// Emission avec timeout
		SetState(kSendMsg);
		CfgTxMode();
#else
		// attente d'un message
		SetState(kWaitRx);
		CfgRxMode();
#endif
		break;

	case kStop:
		// TX stopped
		break;

	case kWaitRx:
		// timeout management ?
		break;

	case kMsgReceived:
		SetState(kSwitchTx);
		DecodeReceivedMsg();
		break;

	case kSendMsg:
		break;

	case kMsgSent:
		//#if (!qMaster)
		//		if (gFirstTimoutSet)
		//		{
		//			RF_SetRxTimeout( kRxTimeOut, true );
		//			gFirstTimoutSet = false;
		//		}
		//#endif

		SetState(kSwitchRx);	// bidirectional

		DisplaySentMsg();
		break;

	case kSwitchRx:
		SetState(kWaitRx);
		CfgRxMode();
		break;

	case kSwitchTx:
		SetState(kSendMsg);
		CfgTxMode();
		break;

	case kErrorRx:
		SetState(kSwitchRx);
		app_log_error("RX Error (%llX)\n", error_code);
		break;

	case kErrorTx:
		SetState(kSwitchTx);
		app_log_error("TX Error (%llX)\n", error_code);
		break;

	case kErrorCal:
		calibration_status_buff = calibration_status;
		app_log_error("CAL Error (%llX / %d)\n", error_code, calibration_status_buff);
		break;

	default:
		app_log_error("Unknown state (%d)\n", gProtocolState);
		break;
	}

#if (qMaster)
	if (tx_requested)
	{
		if (RAIL_GetTime() >= timeout)
		{
			DisplayStat();
			timeout = RAIL_GetTime() + kStatPeriod;
		}
	}
#else
	if (rx_first)
	{
		if (RAIL_GetTime() >= timeout)
		{
			DisplayStat();
			timeout = RAIL_GetTime() + kStatPeriod;
		}
	}
#endif	// qMaster

}

/******************************************************************************
 * RAIL callback, called if a RAIL event occurs.
 *****************************************************************************/
void sl_rail_util_on_event(RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
	error_code = events;

	// Handle Rx events
	if (events & RAIL_EVENTS_RX_COMPLETION)
	{
		if (events & RAIL_EVENT_RX_PACKET_RECEIVED)
		{
			// Keep the packet in the radio buffer, download it later at the state machine
			RAIL_HoldRxPacket(rail_handle);
			packet_recieved = true;
		}
		else
		{
			// Handle Rx error
			rx_error = true;
		}
	}

	// Handle RX timeout
	if (events & (1ULL << RAIL_EVENT_RX_SCHEDULED_RX_END_SHIFT))
	{
#if (qMaster)
    	SetState(kSwitchTx);
#else
		// Handle Rx error
		rx_error = true;
#endif
	}

	// Handle Tx events
	if (events & RAIL_EVENTS_TX_COMPLETION)
	{
		if (events & RAIL_EVENT_TX_PACKET_SENT)
		{
			packet_sent = true;
		}
		else
		{
			// Handle Tx error
			tx_error = true;
		}
	}

	// Perform all calibrations when needed
	if (events & RAIL_EVENT_CAL_NEEDED)
	{
		calibration_status = RAIL_Calibrate(rail_handle, NULL,
		RAIL_CAL_ALL_PENDING);
		if (calibration_status != RAIL_STATUS_NO_ERROR)
		{
			cal_error = true;
		}
	}
}

/******************************************************************************
 * Button callback, called if any button is pressed or released.
 *****************************************************************************/
void sl_button_on_change(const sl_button_t *handle)
{
	if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED)
	{
		tx_requested = !tx_requested;
#if (qUseDisplay)
		GLIB_setFont(&glib_context, (GLIB_Font_t*) &GLIB_FontNarrow6x8);
#endif  // qUseDisplay
		if (tx_requested)
		{
			app_log_info("> Start TX\n");
			sl_led_turn_on(&sl_led_led1);
#if (qUseDisplay)
			GLIB_drawStringOnLine(&glib_context, ">>> RUNNING >>>", 12, GLIB_ALIGN_CENTER, 0, 0, true);
#endif  // qUseDisplay
			timeout = RAIL_GetTime() + kStatPeriod;
			old_TX_counter = TX_counter;
			SetState(kInit);
		}
		else
		{
			app_log_info("> Stop TX\n");
			sl_led_turn_off(&sl_led_led1);
#if (qUseDisplay)
			GLIB_drawStringOnLine(&glib_context, "*** STOPPED ***", 12, GLIB_ALIGN_CENTER, 0, 0, true);
#endif  // qUseDisplay
			SetState(kStop);
		}
#if (qUseDisplay)
		// Force a redraw
		DMD_updateDisplay();
#endif  // qUseDisplay
	}
}

/******************************************************************************
 * Set up the rail TX fifo for later usage
 * @param grail_handle Which rail handler should be updated
 *****************************************************************************/
void set_up_tx_fifo(void)
{
	uint16_t allocated_tx_fifo_size = 0;
	allocated_tx_fifo_size = RAIL_SetTxFifo(grail_handle, tx_fifo.fifo, 0,
	RAIL_FIFO_SIZE);
	app_assert(allocated_tx_fifo_size == RAIL_FIFO_SIZE, "RAIL_SetTxFifo() failed to allocate a large enough fifo (%d bytes instead of %d bytes)\n", allocated_tx_fifo_size, RAIL_FIFO_SIZE);
}

// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------

#if defined(RAIL0_CHANNEL_GROUP_1_PHY_IEEE802154_SUN_FSK_169MHZ_4FSK_9P6KBPS)  \
		|| defined(RAIL0_CHANNEL_GROUP_1_PHY_IEEE802154_SUN_FSK_169MHZ_2FSK_4P8KBPS) \
		|| defined(RAIL0_CHANNEL_GROUP_1_PHY_IEEE802154_SUN_FSK_169MHZ_2FSK_2P4KBPS) \
		|| defined(RAIL0_CHANNEL_GROUP_1_PHY_IEEE802154_SUN_FSK_450MHZ_2FSK_4P8KBPS) \
		|| defined(RAIL0_CHANNEL_GROUP_1_PHY_IEEE802154_SUN_FSK_450MHZ_4FSK_9P6KBPS) \
		|| defined(RAIL0_CHANNEL_GROUP_1_PHY_IEEE802154_SUN_FSK_896MHZ_2FSK_40KBPS)  \
		|| defined(RAIL0_CHANNEL_GROUP_1_PHY_IEEE802154_SUN_FSK_915MHZ_2FSK_10KBPS)  \
		|| defined(RAIL0_CHANNEL_GROUP_1_PHY_IEEE802154_SUN_FSK_920MHZ_4FSK_400KBPS)
#undef RAIL0_CHANNEL_GROUP_1_PROFILE_BASE
#define RAIL0_CHANNEL_GROUP_1_PROFILE_WISUN_FSK
#endif

#if defined(RAIL0_CHANNEL_GROUP_1_PROFILE_WISUN) || defined(RAIL0_CHANNEL_GROUP_1_PROFILE_WISUN_FSK) || defined(RAIL0_CHANNEL_GROUP_1_PROFILE_WISUN_FAN_1_0) || defined(RAIL0_CHANNEL_GROUP_1_PROFILE_WISUN_HAN)
/******************************************************************************
 * The API helps to unpack the received packet, point to the payload and returns the length.
 *****************************************************************************/
static uint16_t unpack_packet(uint8_t *rx_destination, const RAIL_RxPacketInfo_t *packet_information, uint8_t **start_of_payload)
{
	sl_flex_802154_packet_mhr_frame_t rx_mhr = { 0 };
	uint16_t payload_size = 0;
	uint8_t rx_phr_config = 0U;
	RAIL_CopyRxPacket(rx_destination, packet_information);

	*start_of_payload = sl_flex_802154_packet_unpack_g_opt_data_frame(&rx_phr_config,
			&rx_mhr,
			&payload_size,
			rx_destination);
	return ((payload_size > (RAIL_FIFO_SIZE - SL_FLEX_IEEE802154_MHR_LENGTH)) ? (RAIL_FIFO_SIZE - SL_FLEX_IEEE802154_MHR_LENGTH) : payload_size);
}

/******************************************************************************
 * The API prepares the packet for sending and load it in the RAIL TX FIFO
 *****************************************************************************/
static void prepare_package(uint8_t *out_data, uint16_t length)
{
	// Check if write fifo has written all bytes
	uint16_t bytes_writen_in_fifo = 0;
	uint16_t packet_size = 0U;
	uint8_t tx_phr_config = SL_FLEX_IEEE802154G_PHR_MODE_SWITCH_OFF
			| SL_FLEX_IEEE802154G_PHR_CRC_4_BYTE
			| SL_FLEX_IEEE802154G_PHR_DATA_WHITENING_ON;
	sl_flex_802154_packet_mhr_frame_t tx_mhr = {
			.frame_control          = MAC_FRAME_TYPE_DATA                \
			| MAC_FRAME_FLAG_PANID_COMPRESSION \
			| MAC_FRAME_DESTINATION_MODE_SHORT \
			| MAC_FRAME_VERSION_2006           \
			| MAC_FRAME_SOURCE_MODE_SHORT,
			.sequence_number        = 0U,
			.destination_pan_id     = (0xFFFF),
			.destination_address    = (0xFFFF),
			.source_address         = (0x0000)
	};
	uint8_t tx_frame_buffer[256];
	sl_flex_802154_packet_pack_g_opt_data_frame(tx_phr_config,
			&tx_mhr,
			length,
			out_data,
			&packet_size,
			tx_frame_buffer);
	bytes_writen_in_fifo = RAIL_WriteTxFifo(grail_handle, tx_frame_buffer, packet_size, true);
	app_assert(bytes_writen_in_fifo == packet_size,
			"RAIL_WriteTxFifo() failed to write in fifo (%d bytes instead of %d bytes)\n",
			bytes_writen_in_fifo,
			packet_size);
}
#elif defined(RAIL0_CHANNEL_GROUP_1_PROFILE_WISUN_OFDM)
/******************************************************************************
 * The API helps to unpack the received packet, point to the payload and returns the length.
 *****************************************************************************/
static uint16_t unpack_packet(uint8_t *rx_destination,
		const RAIL_RxPacketInfo_t *packet_information,
		uint8_t **start_of_payload)
{
	uint16_t payload_size = 0U;
	uint8_t rate = 0U;
	uint8_t scrambler = 0U;

	RAIL_CopyRxPacket(rx_destination, packet_information);
	*start_of_payload = sl_flex_802154_packet_unpack_ofdm_data_frame(packet_information,
			&rate,
			&scrambler,
			&payload_size,
			rx_destination);
	return payload_size;
}

/******************************************************************************
 * The API prepares the packet for sending and load it in the RAIL TX FIFO
 *****************************************************************************/
static void prepare_package(uint8_t *out_data, uint16_t length)
{
	// Check if write fifo has written all bytes
	uint16_t bytes_writen_in_fifo = 0;
	uint16_t packet_size = 0U;
	uint8_t tx_frame_buffer[256];
	uint8_t rate = 0x06;     // rate: 5 bits wide, The Rate field (RA4-RA0) specifies the data rate of the payload and is equal to the numerical value of the MCS
	// 0x0 BPSK, coding rate 1/2, 4 x frequency repetition
	// 0x1 BPSK, coding rate 1/2, 2 x frequency repetition
	// 0x2 QPSK, coding rate 1/2, 2 x frequency repetition
	// 0x3 QPSK, coding rate 1/2
	// 0x4 QPSK, coding rate 3/4
	// 0x5 16-QAM, coding rate 1/2
	// 0x6 16-QAM, coding rate 3/4
	uint8_t scrambler = 0; // scrambler: 2 bits wide, The Scrambler field (S1-S0) specifies the scrambling seed

	sl_flex_802154_packet_pack_ofdm_data_frame(rate,
			scrambler,
			length,
			out_data,
			&packet_size,
			tx_frame_buffer);
	bytes_writen_in_fifo = RAIL_WriteTxFifo(grail_handle, tx_frame_buffer, packet_size, true);
	app_assert(bytes_writen_in_fifo == packet_size,
			"RAIL_WriteTxFifo() failed to write in fifo (%d bytes instead of %d bytes)\n",
			bytes_writen_in_fifo,
			packet_size);
}

#elif defined(RAIL0_CHANNEL_GROUP_1_PROFILE_SUN_OQPSK)
/******************************************************************************
 * The API helps to unpack the received packet, point to the payload and returns the length.
 *****************************************************************************/
static uint16_t unpack_packet(uint8_t *rx_destination, const RAIL_RxPacketInfo_t *packet_information, uint8_t **start_of_payload)
{
	uint16_t payload_size = 0U;
	bool spreadingMode = false;
	uint8_t rateMode = 0U;

	RAIL_CopyRxPacket(rx_destination, packet_information);
	*start_of_payload = sl_flex_802154_packet_unpack_oqpsk_data_frame(packet_information,
			&spreadingMode,
			&rateMode,
			&payload_size,
			rx_destination);
	return payload_size;
}

/******************************************************************************
 * The API prepares the packet for sending and load it in the RAIL TX FIFO
 *****************************************************************************/
static void prepare_package(uint8_t *out_data, uint16_t length)
{
	// Check if write fifo has written all bytes
	uint16_t bytes_writen_in_fifo = 0;
	uint16_t packet_size = 0U;
	uint8_t tx_frame_buffer[256];
	bool spreadingMode = false;
	uint8_t rateMode = 0; // rateMode: 2 bits wide

	sl_flex_802154_packet_pack_oqpsk_data_frame(spreadingMode,
			rateMode,
			length,
			out_data,
			&packet_size,
			tx_frame_buffer);

	bytes_writen_in_fifo = RAIL_WriteTxFifo(grail_handle, tx_frame_buffer, packet_size, true);
	app_assert(bytes_writen_in_fifo == packet_size,
			"RAIL_WriteTxFifo() failed to write in fifo (%d bytes instead of %d bytes)\n",
			bytes_writen_in_fifo,
			packet_size);
}
#else
/******************************************************************************
 * The API helps to unpack the received packet, point to the payload and returns the length.
 *****************************************************************************/
static uint16_t unpack_packet(uint8_t *rx_destination, const RAIL_RxPacketInfo_t *packet_information, uint8_t **start_of_payload)
{
	RAIL_CopyRxPacket(rx_destination, packet_information);
	*start_of_payload = rx_destination;
	return ((packet_information->packetBytes > RAIL_FIFO_SIZE) ?
	RAIL_FIFO_SIZE :
																	packet_information->packetBytes);
}

/******************************************************************************
 * The API prepares the packet for sending and load it in the RAIL TX FIFO
 *****************************************************************************/
static void prepare_package(uint8_t *out_data, uint16_t length)
{
	// Check if write fifo has written all bytes
	uint16_t bytes_writen_in_fifo = 0;
	bytes_writen_in_fifo = RAIL_WriteTxFifo(grail_handle, out_data, length,
	true);
	app_assert(bytes_writen_in_fifo == TX_PAYLOAD_LENGTH, "RAIL_WriteTxFifo() failed to write in fifo (%d bytes instead of %d bytes)\n", bytes_writen_in_fifo, TX_PAYLOAD_LENGTH);
}
#endif
