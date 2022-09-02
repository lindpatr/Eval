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
#define RAIL_FIFO_SIZE (512U)						// in bytes
/// Transmit data length (fixed payload)
#define TX_PAYLOAD_LENGTH (06U)						// in bytes
// Period to print statistics
#define STAT_PERIOD_s (20U)							// in us
#define STAT_PERIOD_us (STAT_PERIOD_s * 1000000ULL)	// in sec

#define SIZE_UINT64_IN_BITS	(int)(8*sizeof(uint64_t))

/// State machine
typedef enum
{
	kInit = 0, kStart, kStop, kIdle,

	kWaitRx, kMsgReceived, kErrorRx, kTimeOutRx, kSwitchRx,

	kSendMsg, kMsgSent, kErrorTx, kTimeOutTx, kSwitchTx,

	kErrorCal
} StateEnum;

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
/// The variable shows the actual state of the state machine
static volatile StateEnum gProtocolState = kInit;
static volatile StateEnum gPrevProtocolState = kInit;

/// Contains the last RAIL Rx/Tx error events
static volatile uint64_t gErrorCode = RAIL_EVENTS_NONE;
static uint64_t gOldErrorCode = RAIL_EVENTS_NONE;

/// Contains the status of RAIL Calibration
static volatile RAIL_Status_t gCalibrationStatus = RAIL_STATUS_NO_ERROR;

/// Timeout for printing and displaying the statistics
//static RAIL_Time_t gStatPeriodTimeout = 0UL;
static volatile RAIL_Time_t gElapsedTime = 0UL;
static volatile RAIL_Time_t gOldElapsedTime = 0UL;
/// A static handle of a RAIL timer
static RAIL_MultiTimer_t gStatPeriodTimer, gRX_timeout, gTX_timeout;


/// Receive and Send FIFO
static uint8_t gRX_fifo[RAIL_FIFO_SIZE];

static union
{
	// Used to align this buffer as needed
	RAIL_FIFO_ALIGNMENT_TYPE align[RAIL_FIFO_SIZE / RAIL_FIFO_ALIGNMENT];
	uint8_t fifo[RAIL_FIFO_SIZE];
} gTX_fifo;

/// Transmit packet
static uint8_t gTX_packet[TX_PAYLOAD_LENGTH] =
//{ 0xAA, 0x06, 0x11, 0x22, 0x33, 0x44 };
{ 0x0F, 0x16, 0x11, 0x22, 0x33, 0x44 };

/// Flags to update state machine from interrupt
static volatile bool gPacketRecieved = false;
static volatile bool gPacketSent = false;
static volatile bool gRX_error = false;
static volatile bool gRX_timeout_error = false;
static volatile bool gTX_error = false;
static volatile bool gTX_timeout_error = false;
static volatile bool gCAL_error = false;
static volatile bool gStatTimerDone = false;

/// TX and RX counters
static volatile uint32_t gRX_counter = 0UL;
static volatile uint32_t gTX_counter = 0UL;
static volatile uint32_t gTX_counter_old = 0UL;
static volatile uint32_t gRX_counter_old = 0UL;
static uint32_t gRX_counter_prev = 0UL;

/// Tables for error statistics
static volatile uint32_t gCB_tab[SIZE_UINT64_IN_BITS] = {0};	   // index: see RAIL_ENUM_GENERIC(RAIL_Events_t, uint64_t) in rail_types.h
static uint32_t gTX_tab[3] = { 0 };    // 0 = TX_OK, 1 = TX_Err, 2 = ND
static uint32_t gRX_tab[3] = { 0 };    // 0 = RX_OK, 1 = RX_Err, 2 = RX_TimeOut
static uint32_t gCAL_tab[3] = { 0 };   // 0 = ND,    1 = CAL_Err, 2 = ND

/// LCD variables
/// Context used all over the graphics
#if (qUseDisplay)
static GLIB_Context_t gGlibContext;
#endif  // qUseDisplay

/// Flag, indicating transmit request (button was pressed / CLI transmit request has occurred)
volatile bool gTX_requested = false;
/// Flag, indicating received packet is forwarded on CLI or not
volatile bool gRX_requested = true;
volatile bool gRX_first = false;
volatile bool gPrintStat = false;
static volatile bool gStartProcess = false;


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
 * @param gRailHandle Which rail handlers should be used for the TX FIFO writing
 * @param out_data The payload buffer
 * @param length The length of the payload
 *****************************************************************************/
static void prepare_package(uint8_t *out_data, uint16_t length);

/******************************************************************************
 * Timer callback, called if any button is pressed or released.
 *****************************************************************************/
void timer_callback(RAIL_MultiTimer_t *tmr, RAIL_Time_t expectedTimeOfEvent, void *cbArg);

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

	// For oscillo debug
	GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_RX);

	// RX with timeout
//	RAIL_Status_t status = RAIL_ScheduleRx(gRailHandle, CHANNEL, &gRailScheduleCfgRX, NULL);
	RAIL_Status_t status = RAIL_StartRx(gRailHandle, CHANNEL, NULL);
	RAIL_SetMultiTimer(&gRX_timeout, RX_TIMEOUT, RAIL_TIME_DELAY, &timer_callback, NULL);

	if (status != RAIL_STATUS_NO_ERROR)
	{
#if (qDebugPrintErr)
//		app_log_warning("Warning RAIL_ScheduleRx (%d)\n", status);
		app_log_warning("Warning RAIL_StartRx (%d)\n", status);
#endif	// qDebugPrintErr
	}
}

/******************************************************************************
 * CfgTxMode : prepare buffer and change to TX mode
 *****************************************************************************/
void CfgTxMode(void)
{
	// Data
	gTX_counter++;

	// Initialize buffer
	gTX_packet[2] = (gTX_counter & 0x000000FF) >> 0;
	gTX_packet[3] = (gTX_counter & 0x0000FF00) >> 8;
	gTX_packet[4] = (gTX_counter & 0x00FF0000) >> 16;
	gTX_packet[5] = (gTX_counter & 0xFF000000) >> 24;

	// Initialize radio buffer
	prepare_package(gTX_packet, sizeof(gTX_packet));

	// For oscillo debug
	GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_TX);

	// Start TX and check result
	RAIL_Status_t status = RAIL_StartTx(gRailHandle, CHANNEL,RAIL_TX_OPTIONS_DEFAULT, NULL);
	// TX with timeout
//	RAIL_Status_t status = RAIL_StartScheduledTx(gRailHandle, CHANNEL, RAIL_TX_OPTIONS_DEFAULT, &gRailScheduleCfgTX, NULL);

	if (status != RAIL_STATUS_NO_ERROR)
	{
#if (qDebugPrintErr)
//		app_log_warning("Warning RAIL_StartScheduledTx (%d)\n", status);
		app_log_warning("Warning RAIL_StartTx (%d)\n", status);
#endif	// qDebugPrintErr
	}
}


/******************************************************************************
 * DisplayReceivedMsg : print received data
 *****************************************************************************/
void DisplayReceivedMsg(const uint8_t *const rx_buffer, uint16_t length)
{
#if (qPrintRX)
	// Print received data on serial COM
	app_log_info("RX: ");
	for (uint16_t i = 0; i < length; i++) {
		app_log_info("0x%02X, ", rx_buffer[i]);
	}
	app_log_info("\n");
#else
	(void) length;
#endif
	// Backup previous data
	gRX_counter_prev = gRX_counter;
	// Extract received data
	gRX_counter = (uint32_t) ((rx_buffer[2] << 0) + (rx_buffer[3] << 8) + (rx_buffer[4] << 16) + (rx_buffer[5] << 24));

}

/******************************************************************************
 * DisplaySentMsg : print sent data
 *****************************************************************************/
void DisplaySentMsg(void)
{
#if (qPrintTX)
	// Print sent data on serial COM
	app_log_info("TX: %d\n",gTX_counter);
#endif
	// Indicate TX in progress on LED1
	sl_led_toggle(&sl_led_led1);
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
	RAIL_Status_t status = RAIL_STATUS_NO_ERROR;

	// Packet received:
	//  - Check whether RAIL_HoldRxPacket() was successful, i.e. packet handle is valid
	//  - Copy it to the application FIFO
	//  - Free up the radio FIFO
	//  - Return to app IDLE state (RAIL will automatically switch back to Rx radio state)

	rx_packet_handle = RAIL_GetRxPacketInfo(gRailHandle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);
	while (rx_packet_handle != RAIL_RX_PACKET_HANDLE_INVALID)
	{
		uint8_t *start_of_packet = 0;
		uint16_t packet_size = unpack_packet(gRX_fifo, &packet_info, &start_of_packet);
		status = RAIL_ReleaseRxPacket(gRailHandle, rx_packet_handle);
		if (status != RAIL_STATUS_NO_ERROR)
		{
#if (qDebugPrintErr)
			app_log_warning("Error ReleaseRxPacket (%d)\n", status);
#endif	// qDebugPrintErr
		}
		if (gRX_requested)
		{
			DisplayReceivedMsg(start_of_packet, packet_size);
//			if ((gRX_counter - gRX_counter_prev) > 1)
//				gRX_tab[2]++;
//			else
				gRX_tab[0]++;
		}
		// Indicate RX in progress on LED0
		sl_led_toggle(&sl_led_led0);
		rx_packet_handle = RAIL_GetRxPacketInfo(gRailHandle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);
	}
	GPIO_PinOutClear(DEBUG_PORT, DEBUG_PIN_RX);
}

/******************************************************************************
 * DisplayStat : print and display statistics
 *****************************************************************************/
void DisplayStat(void)
{
	// Statistics
	float localStat = (float)(gTX_counter + gRX_counter - gTX_counter_old - gRX_counter_old) / ((float)(gElapsedTime - gOldElapsedTime) / 1000000.0f);
	float localStat2 = (10.0f * 10100.0f) / localStat;
	float localStat3 = 100.0f * (float) (gTX_tab[1] + gTX_tab[2]) / (float) gTX_counter;
	float localStat4 = 100.0f * (float) (gRX_tab[1] + gRX_tab[2]) / (float) gRX_counter;

	// Print on serial COM
	app_log_info("\nTX Count: %lu\n", gTX_tab[0]);
	app_log_info("RX Count: %lu\n", gRX_tab[0]);
	app_log_info("TX Error: %0.3f%% (Err:%d/TO:%d)\n", localStat3, gTX_tab[1], gTX_tab[2]);
	app_log_info("RX Error: %0.3f%% (Err:%d/TO:%d)\n", localStat4, gRX_tab[1], gRX_tab[2]);
	app_log_info("Rate :    %0.2f msg/s (loop: %0.2f ms)\n", localStat, localStat2);

#if (qUseDisplay)
	// Print on LCD display
	char textBuf[32];

	GLIB_setFont(&gGlibContext, (GLIB_Font_t*) &GLIB_FontNormal8x8);
	snprintf(textBuf, sizeof(textBuf), "TX Cnt %lu          ", gTX_tab[0]);
	GLIB_drawStringOnLine(&gGlibContext, textBuf, 5, GLIB_ALIGN_LEFT, 0, 0, true);
	snprintf(textBuf, sizeof(textBuf), "RX Cnt %lu          ", gRX_tab[0]);
	GLIB_drawStringOnLine(&gGlibContext, textBuf, 6, GLIB_ALIGN_LEFT, 0, 0, true);
	snprintf(textBuf, sizeof(textBuf), "TX Err %0.3f%%      ", localStat3);
	GLIB_drawStringOnLine(&gGlibContext, textBuf, 7, GLIB_ALIGN_LEFT, 0, 0, true);
	snprintf(textBuf, sizeof(textBuf), "RX Err %0.3f%%      ", localStat4);
	GLIB_drawStringOnLine(&gGlibContext, textBuf, 8, GLIB_ALIGN_LEFT, 0, 0, true);
	snprintf(textBuf, sizeof(textBuf), "Rate  %0.1f fs     ", localStat);
	GLIB_drawStringOnLine(&gGlibContext, textBuf, 9, GLIB_ALIGN_LEFT, 0, 0, true);
	snprintf(textBuf, sizeof(textBuf), "Loop  %0.1f ms     ", localStat2);
	GLIB_drawStringOnLine(&gGlibContext, textBuf, 10, GLIB_ALIGN_LEFT, 0, 0, true);

	// Force a redraw
	DMD_updateDisplay();
#endif  // qUseDisplay
	gTX_counter_old = gTX_counter;
	gRX_counter_old = gRX_counter;
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
#if (qDebugPrintErr)
		app_log_error("Error DMD_init (%d)\n", status);
#endif	//qDebugPrintErr
		return;
	}

	status = GLIB_contextInit(&gGlibContext);
	if (GLIB_OK != status)
	{
#if (qDebugPrintErr)
		app_log_error("Error GLIB_contextInit (%d)\n", status);
#endif	//qDebugPrintErr
		return;
	}

	gGlibContext.backgroundColor = White;
	gGlibContext.foregroundColor = Black;

	// Clear what's currently on screen
	GLIB_clear(&gGlibContext);

	GLIB_setFont(&gGlibContext, (GLIB_Font_t*) &GLIB_FontNarrow6x8);

	GLIB_drawStringOnLine(&gGlibContext, "****************", 1, GLIB_ALIGN_CENTER, 0, 0, 0);
	GLIB_drawStringOnLine(&gGlibContext, "* TEST BIDIR  *", 2, GLIB_ALIGN_CENTER, 0, 0, 0);
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
	//GPIO_PinOutClear(DEBUG_PORT, DEBUG_PIN_MISC);

	/// -----------------------------------------
	/// Decode from interrupt / callback
	/// -----------------------------------------
	if (gPacketRecieved)
	{
		gPacketRecieved = false;
		// DEBUG RAIL_CancelMultiTimer(&gRX_timeout);

#if (!qMaster)
		if (!gRX_first)
		{
			gRX_first = true;
//#if (qUseDisplay)
//			GLIB_setFont(&gGlibContext, (GLIB_Font_t*) &GLIB_FontNarrow6x8);
//			GLIB_drawStringOnLine(&gGlibContext, ">>> RUNNING >>>", 12, GLIB_ALIGN_CENTER, 0, 0, true);
//			// Force a redraw
//			DMD_updateDisplay();
//#endif  // qUseDisplay
			gTX_counter_old = gTX_counter;
			gRX_counter_old = gRX_counter;
			//RAIL_SetMultiTimer(&gStatPeriodTimer, STAT_PERIOD_us, RAIL_TIME_DELAY, &timer_callback, NULL);
			//gStatPeriodTimeout = RAIL_GetTime() + STAT_PERIOD_us;
			gElapsedTime = RAIL_GetTime();
		}
#endif	// !qMaster
		SetState(kMsgReceived);
	}
	else if (gPacketSent)
	{
		gPacketSent = false;
		GPIO_PinOutClear(DEBUG_PORT, DEBUG_PIN_TX);
		gTX_tab[0]++;
		SetState(kMsgSent);
	}
	else if (gRX_error)
	{
		gRX_error = false;
		gRX_tab[1]++;
		SetState(kErrorRx);
	}
	else if (gRX_timeout_error)
	{
		gRX_timeout_error = false;
		GPIO_PinOutClear(DEBUG_PORT, DEBUG_PIN_MISC);
		gRX_tab[2]++;
		SetState(kTimeOutRx);
	}
	else if (gTX_error)
	{
		gTX_error = false;
		gTX_tab[1]++;
		SetState(kErrorTx);
	}
	else if (gTX_timeout_error)
	{
		gTX_timeout_error = false;
		gTX_tab[2]++;
		SetState(kTimeOutTx);
	}
	else if (gCAL_error)
	{
		gCAL_error = false;
		gCAL_tab[1]++;
		SetState(kErrorCal);
	}
	else if (gStartProcess)
	{
		gStartProcess = false;
		SetState(kStart);
	}

	/// -----------------------------------------
	/// State machine
	/// -----------------------------------------
	switch (gProtocolState)
	{
	// -----------------------------------------
	// Startup + flow control
	// -----------------------------------------
	case kInit:
		// Sate after POR, wait until pressing BTN0 on Master or receiving message (on Slave)
		prepare_package(gTX_packet, sizeof(gTX_packet));
#if (qMaster)
		SetState(kIdle);
#else
#if (qUseDisplay)
		GLIB_setFont(&gGlibContext, (GLIB_Font_t*) &GLIB_FontNarrow6x8);
		GLIB_drawStringOnLine(&gGlibContext, ">>> RUNNING >>>", 12, GLIB_ALIGN_CENTER, 0, 0, true);
		// Force a redraw
		DMD_updateDisplay();
#endif  // qUseDisplay
		SetState(kWaitRx);
#endif	// qMaster
		break;

	case kStart:
		// TX started by pressing BTN0
		SetState(kSwitchTx);
		app_log_info("> Start TX\n");
		sl_led_turn_on(&sl_led_led1);
#if (qUseDisplay)
		GLIB_setFont(&gGlibContext, (GLIB_Font_t*) &GLIB_FontNarrow6x8);
		GLIB_drawStringOnLine(&gGlibContext, ">>> RUNNING >>>", 12, GLIB_ALIGN_CENTER, 0, 0, true);
		// Force a redraw
		DMD_updateDisplay();
#endif  // qUseDisplay
		//gStatPeriodTimeout = RAIL_GetTime() + STAT_PERIOD_us;
		//RAIL_SetMultiTimer(&gStatPeriodTimer, STAT_PERIOD_us, RAIL_TIME_DELAY, &timer_callback, NULL);
		//gTX_counter_old = gTX_counter;
		break;

	case kStop:
		// TX stopped by pressing BTN0
		SetState(kIdle);
		app_log_info("> Stop TX\n");
		sl_led_turn_off(&sl_led_led1);
#if (qUseDisplay)
		GLIB_setFont(&gGlibContext, (GLIB_Font_t*) &GLIB_FontNarrow6x8);
		GLIB_drawStringOnLine(&gGlibContext, "*** END ***", 12, GLIB_ALIGN_CENTER, 0, 0, true);
		// Force a redraw
		DMD_updateDisplay();
#endif  // qUseDisplay
		break;

	case kIdle:
		// Wait action
		break;

		// -----------------------------------------
		// RX
		// -----------------------------------------
	case kWaitRx:
		// Wait receiving message
		break;

	case kMsgReceived:
		// Message received
		SetState(kSwitchTx);
		DecodeReceivedMsg();
		break;

	case kErrorRx:
		// Error on RX
#if (qMaster)
		SetState(kSwitchRx);
#else
		SetState(kSwitchRx);
#endif
#if (qDebugPrintErr)
		if (gErrorCode != gOldErrorCode)
		{
			app_log_error("RX Error (0x%llX)\n", gErrorCode);
			gOldErrorCode = gErrorCode;
		}
#endif	// qDebugPrintErr
		break;

	case kTimeOutRx:
		// Timeout on RX
#if (qMaster)
		SetState(kSwitchTx);
#else
		SetState(kSwitchRx);
#endif
#if (qDebugPrintErr)
		if (gErrorCode != gOldErrorCode)
		{
			app_log_error("RX TimeOut (0x%llX)\n", gErrorCode);
			gOldErrorCode = gErrorCode;
		}
#endif	// qDebugPrintErr
		break;

	case kSwitchRx:
		SetState(kWaitRx);
		CfgRxMode();
		break;

		// -----------------------------------------
		// TX
		// -----------------------------------------
	case kSendMsg:
		// Wait until message sent
		break;

	case kMsgSent:
		SetState(kSwitchRx);	// bidirectional
		DisplaySentMsg();
		break;

	case kErrorTx:
#if (qMaster)
		SetState(kSwitchTx);
#else
		SetState(kSwitchTx);
#endif
#if (qDebugPrintErr)
		if (gErrorCode != gOldErrorCode)
		{
			app_log_error("TX Error (0x%llX)\n", gErrorCode);
			gOldErrorCode = gErrorCode;
		}
#endif	// qDebugPrintErr
		break;

	case kTimeOutTx:
		// Timeout on TX
#if (qMaster)
		// ??? SetState(kSwitchTx);
#else
		// ??? SetState(kSwitchTx);
#endif
#if (qDebugPrintErr)
		if (gErrorCode != gOldErrorCode)
		{
			app_log_error("TX TimeOut (0x%llX)\n", gErrorCode);
			gOldErrorCode = gErrorCode;
		}
#endif	// qDebugPrintErr
		break;

	case kSwitchTx:
		SetState(kSendMsg);
		CfgTxMode();
		break;

		// -----------------------------------------
		// CAL
		// -----------------------------------------
	case kErrorCal:
#if (qDebugPrintErr)
		if (gErrorCode != gOldErrorCode)
		{
			app_log_error("CAL Error (0x%llX / %d)\n", gErrorCode, gCalibrationStatus);
			gOldErrorCode = gErrorCode;
		}
#endif	// qDebugPrintErr
		break;

	default:
		app_assert(false, "Unknown state (%d)\n", gProtocolState);
		break;
	}

	/// -----------------------------------------
	/// Statistics
	/// -----------------------------------------
	if (gPrintStat)
	{
		gPrintStat = false;
		//if (gStatTimerDone/*gRAIL_GetTime() >= gStatPeriodTimeout*/)
		{
			gStatTimerDone = false;
			DisplayStat();
			//gStatPeriodTimeout = RAIL_GetTime() + STAT_PERIOD_us;
			//RAIL_SetMultiTimer(&gStatPeriodTimer, STAT_PERIOD_us, RAIL_TIME_DELAY, &timer_callback, NULL);
		}
	}
}

/******************************************************************************
 * RAIL callback, called if a RAIL event occurs.
 *****************************************************************************/


static __INLINE void DecodeEvents(RAIL_Events_t *events)
{
	uint64_t ev = *events;

	for (int i = 0; i <SIZE_UINT64_IN_BITS; i++)
	{
		if (ev & 0x1ULL)
			gCB_tab[i]++;

		ev >>= 1;
	}
}

void sl_rail_util_on_event(RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
	gErrorCode = events;

	//GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_MISC);

	//DecodeEvents(&events);

	// Handle Rx events
	if (events & RAIL_EVENTS_RX_COMPLETION)
	{
//		if (gProtocolState == kWaitRx)
		{
			if (events & RAIL_EVENT_RX_PACKET_RECEIVED)
			{
				// Keep the packet in the radio buffer, download it later at the state machine
				RAIL_HoldRxPacket(rail_handle);
				gPacketRecieved = true;
			}
			else
			{
				// Handle Rx error
				gRX_error = true;
			}
		}
	}

	// Handle Rx timeout
//	if (events & (1ULL << RAIL_EVENT_RX_SCHEDULED_RX_END_SHIFT))
//	{
//		gRX_timeout_error = true;
//	}

	// Handle Tx events
	if (events & RAIL_EVENTS_TX_COMPLETION)
	{
//		if (gProtocolState == kSendMsg)
		{
			if (events & RAIL_EVENT_TX_PACKET_SENT)
			{
				// Handle next step
				gPacketSent = true;
			}
			else
			{
				// Handle Tx error
				gTX_error = true;
			}
		}
	}

	// Handle TX timeout
//	if (events & (1ULL << ???))
//	{
//		gTX_timeout_error = true;
//	}
//
	// Perform all calibrations when needed
	if (events & RAIL_EVENT_CAL_NEEDED)
	{
		gCalibrationStatus = RAIL_Calibrate(rail_handle, NULL, RAIL_CAL_ALL_PENDING);
		if (gCalibrationStatus != RAIL_STATUS_NO_ERROR)
		{
			gCAL_error = true;
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
#if (qMaster)
		if (!gTX_requested)
		{
			gStartProcess = true;

			gTX_requested = true;
			gElapsedTime = RAIL_GetTime();

		}
		else
		{
			gOldElapsedTime = gElapsedTime;
			gElapsedTime = RAIL_GetTime();
			gPrintStat = true;
		}
#else
	gOldElapsedTime = gElapsedTime;
	gElapsedTime = RAIL_GetTime();
	gPrintStat = true;

#endif	// qMaster
	}
}

/******************************************************************************
 * Timer callback, called if any button is pressed or released.
 *****************************************************************************/
void timer_callback(RAIL_MultiTimer_t *tmr, RAIL_Time_t expectedTimeOfEvent, void *cbArg)
{
	(void) expectedTimeOfEvent;
	(void) cbArg;

	if (tmr == &gStatPeriodTimer)
	{
		gStatTimerDone = true;
	}
	else if (tmr == &gRX_timeout)
	{
		gRX_timeout_error = true;
		GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_MISC);
	}
	else if (tmr == &gTX_timeout)
	{
		gTX_timeout_error = true;
	}
}

/******************************************************************************
 * Set up the rail TX fifo for later usage
 * @param gRailHandle Which rail handler should be updated
 *****************************************************************************/
void set_up_tx_fifo(void)
{
	uint16_t allocated_tx_fifo_size = 0;
	allocated_tx_fifo_size = RAIL_SetTxFifo(gRailHandle, gTX_fifo.fifo, 0, RAIL_FIFO_SIZE);
	app_assert(allocated_tx_fifo_size == RAIL_FIFO_SIZE, "RAIL_SetTxFifo() failed to allocate a large enough fifo (%d bytes instead of %d bytes)\n", allocated_tx_fifo_size, RAIL_FIFO_SIZE);
}

// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------

/******************************************************************************
 * The API helps to unpack the received packet, point to the payload and returns the length.
 *****************************************************************************/
static uint16_t unpack_packet(uint8_t *rx_destination, const RAIL_RxPacketInfo_t *packet_information, uint8_t **start_of_payload)
{
	RAIL_CopyRxPacket(rx_destination, packet_information);
	*start_of_payload = rx_destination;
	return ((packet_information->packetBytes > RAIL_FIFO_SIZE) ? RAIL_FIFO_SIZE : packet_information->packetBytes);
}

/******************************************************************************
 * The API prepares the packet for sending and load it in the RAIL TX FIFO
 *****************************************************************************/
static void prepare_package(uint8_t *out_data, uint16_t length)
{
	// Check if write fifo has written all bytes
	uint16_t bytes_writen_in_fifo = 0;
	bytes_writen_in_fifo = RAIL_WriteTxFifo(gRailHandle, out_data, length, true);
	app_assert(bytes_writen_in_fifo == TX_PAYLOAD_LENGTH, "RAIL_WriteTxFifo() failed to write in fifo (%d bytes instead of %d bytes)\n", bytes_writen_in_fifo, TX_PAYLOAD_LENGTH);
}
