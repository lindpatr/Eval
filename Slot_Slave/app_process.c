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

#include "common_stat.h"
#include "common_debug.h"

// Specific to LCD display
#include "dmd.h"
#include "glib.h"
#include "printf.h"

// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------
/// Size of RAIL RX/TX FIFO
#define RAIL_FIFO_RX_SIZE (512U)          // in bytes
#define RAIL_FIFO_TX_SIZE (128U)          // in bytes // don't try below 128 --> assert
/// Transmit data length (fixed payload --> see Radio Configurator)
#define TX_PAYLOAD_LENGTH (6U)            // in bytes


/// State machine
typedef enum
{
	kInit = 0,
	//  ------------
	//  Idle
	//  ------------
    kIdle,
    //  ------------
    //	RX
    //  ------------
	kSyncReceived,
	kErrorRx,
	kListen,
    //  ------------
    //	TX
    //  ------------
	kSendingMsg,
	kMsgSent,
	kErrorTx,
	//kScheduleSending,
    //  ------------
    //  Calibration
    //  ------------
    kCalReq,
    //  ------------
    //  Statistics
    //  ------------
    kStatistics,
    //  ------------
    //  Timers
    //  ------------
    kSyncLost,
    //  ------------
    //  IHM & CLI
    //  ------------
    kBtnPressed,
    kStatReq

} StateEnum;

// Position in the data payload
typedef enum
{
    kAddr       = 0,
    kNone       = 1,
    kCounter0   = 2,
    kCounter1   = 3,
    kCounter2   = 4,
    kCounter3   = 5
} PayloadPosEnum;

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
/// The variable shows the actual and previous state of the state machine
static volatile StateEnum gProtocolState = kInit;
static volatile StateEnum gProtocolPrevState = kInit;

/// Contains the last and previous RAIL Rx/Tx error events
static volatile uint64_t gErrorCode = RAIL_EVENTS_NONE;
static uint64_t gPrevErrorCode = RAIL_EVENTS_NONE;

/// A static handle of a RAIL timers
static RAIL_MultiTimer_t gStatDelayTimer, gSyncTimeOutTimer;

/// Receive and Send FIFO
/// RX
static uint8_t gRX_fifo[RAIL_FIFO_RX_SIZE];
/// TX
static union
{
	// Used to align this buffer as needed
	RAIL_FIFO_ALIGNMENT_TYPE align[RAIL_FIFO_TX_SIZE / RAIL_FIFO_ALIGNMENT];
	uint8_t fifo[RAIL_FIFO_TX_SIZE];
} gTX_fifo;

/// Flags to update state machine from interrupt
static volatile bool gRX_ok = false;
static volatile bool gTX_ok = false;
static volatile bool gRX_error = false;
static volatile bool gTX_error = false;
static volatile bool gCAL_req = false;
static volatile bool gSYNC_timeout = false;

/// Flags to update strat print statistics
static volatile bool gStatTimerDone = false;	// Timeout of the delay for printing stat
volatile bool gStatReq = false;					// Flag, indicating a request to print statistics (button was pressed / CLI statistics request has occurred)

/// TX and RX counters
volatile uint32_t gRX_counter[MAX_NODE] = {0UL};
volatile union32_t gTX_counter = {.u32 = 1UL};
volatile uint32_t gTX_counter_old = 0UL;
volatile uint32_t gRX_counter_old[MAX_NODE] = {0UL};
uint32_t gRX_counter_prev[MAX_NODE] = {0UL};

/// Various flags
volatile bool gStartProcess = false;			// Flag, indicating a start process request (button was pressed / CLI start request has occurred)
static bool gRX_first = false;					// Indicate first RX packet received on Slave to start statistics
volatile bool gBtnPressed = false;              // Button pressed simulation with CLI, start process
static bool gPauseCycleConf = false;            // Flag to indicate a end of cycle of transmission with the slaves in order to avoid side effect of the print statistics
static bool gPauseCycleReq = false;             // Flag to indicate that a print stat is pending and request to pause the cycle at the next occasion (end of cycle)

// Various var
static uint8_t me;                                                  // Position in the config file (for address, time slot, ...)
/// Value, indicating print stat delay on CLI
extern volatile RAIL_Time_t gStatDelay;
/// Value, indicating sync timeout for Slave on CLI
extern volatile RAIL_Time_t gSyncTimeOut;

// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
//                          Callback Function Definitions
// -----------------------------------------------------------------------------
/******************************************************************************
 * RAIL callback, called if a RAIL event occurs.
 *****************************************************************************/
void sl_rail_util_on_event(RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
	gErrorCode = events;  // Save events context

	//GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_MISC);

	DecodeEvents(&events);  // Count events for debug and statistics

	// Handle RX events
	if (events & RAIL_EVENTS_RX_COMPLETION)
	{

		if (events & RAIL_EVENT_RX_PACKET_RECEIVED)
		{
		    // For debug on oscillo purposes --> check delta between all slave
		    GPIO_PinOutClear(DEBUG_PORT, DEBUG_PIN_RX);
			// Keep the packet in the radio buffer, download it later at the state machine
			RAIL_HoldRxPacket(rail_handle);
			gRX_ok = true;
		}
		else  //  | RAIL_EVENT_RX_PACKET_ABORTED
			  //  | RAIL_EVENT_RX_FRAME_ERROR
			  //  | RAIL_EVENT_RX_FIFO_OVERFLOW
			  //  | RAIL_EVENT_RX_ADDRESS_FILTERED
			  //  | RAIL_EVENT_RX_SCHEDULED_RX_MISSED
		{
			// Handle Rx error
			gRX_error = true;
		}
	}

	// Handle TX events
	if (events & RAIL_EVENTS_TX_COMPLETION)
	{
		if (events & RAIL_EVENT_TX_PACKET_SENT)
		{
			// Handle next step
		    GPIO_PinOutClear(DEBUG_PORT, DEBUG_PIN_TX);
		    GPIO_PinOutClear(DEBUG_PORT, DEBUG_PIN_MISC);

			gTX_ok = true;

			// REMOVED BEGIN
//			if (gPauseCycleReq)
//			    gPauseCycleConf = true;     // Cycle ended, permit a print stat
			// REMOVED END
		}
		else  //  | RAIL_EVENT_TX_ABORTED
			  //  | RAIL_EVENT_TX_BLOCKED
			  //  | RAIL_EVENT_TX_UNDERFLOW
			  //  | RAIL_EVENT_TX_CHANNEL_BUSY
			  //  | RAIL_EVENT_TX_SCHEDULED_TX_MISSED)
		{
			// Handle Tx error
			gTX_error = true;
		}
	}

	// TODO To remove and disable RX_TX_SCHEDULED_RX_TX_STARTE events (if not additional code is required)
	// Scheduled transmission is starting
	if (events & RAIL_EVENT_SCHEDULED_TX_STARTED)
	{
	    // For oscillo debug purposes
	    GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_MISC);
	}
    if (events & RAIL_EVENT_TX_STARTED)
    {
        // For oscillo debug purposes
        GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_MISC);
    }

	// Perform all calibrations when needed
	if (events & RAIL_EVENT_CAL_NEEDED)
	{
	    gCAL_req = true;
	}

	//GPIO_PinOutClear(DEBUG_PORT, DEBUG_PIN_MISC);
}

/******************************************************************************
 * Button callback, called if any button is pressed or released.
 *****************************************************************************/
void sl_button_on_change(const sl_button_t *handle)
{
	if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED)
	{
		gBtnPressed = true;
	}
}

/******************************************************************************
 * Timer callback, called if any button is pressed or released.
 *****************************************************************************/
void timer_callback(RAIL_MultiTimer_t *tmr, RAIL_Time_t expectedTimeOfEvent, void *cbArg)
{
	(void) expectedTimeOfEvent;   // To avoid warnings
	(void) cbArg;         // To avoid warnings

	if (tmr == &gStatDelayTimer) // used to print stat after delay
	{
		gStatTimerDone = true;
	}
	else if (tmr == &gSyncTimeOutTimer) // used to test if Master is still alive
	{
	    gSYNC_timeout = true;
	}
}

// -----------------------------------------------------------------------------
//                          Private Function Definitions
// ----------------------------------------------------------------------------

/******************************************************************************
 * SetState : change state machine and backup previous state machine
 *****************************************************************************/
static __INLINE void SetState(StateEnum state)
{
	gProtocolPrevState = gProtocolState;
	gProtocolState = state;
}

/******************************************************************************
 * StopRadio : stop radio and clear all flags
 *****************************************************************************/
static __INLINE void StopRadio(void)
{
    RAIL_Idle(gRailHandle, RAIL_IDLE_FORCE_SHUTDOWN_CLEAR_FLAGS, true);

    gRX_ok = false;
    gTX_ok = false;
    gRX_error = false;
    gTX_error = false;
    gSYNC_timeout = false;
    gCAL_req = false;
}

/******************************************************************************
 * RestartRadio : start radio
 *****************************************************************************/
static __INLINE void RestartRadio(void)
{
    RAIL_Idle(gRailHandle, RAIL_IDLE, true);
}

/******************************************************************************
 * PrintStatistics : print out statistics
 *****************************************************************************/
static __INLINE bool PrintStatistics(void)
{
#if (qPrintStat)
    gPauseCycleReq = true;      // Request to print stat

    gOldElapsedTime = gElapsedTime;
    gElapsedTime = RAIL_GetTime();

    gCountPrintStat++;

    StopRadio();

    DisplayStat();

    RestartRadio();

    gTX_counter_old = gTX_counter.u32;
    gRX_counter_old[me] = gRX_counter[me];

    gPauseCycleReq = false;
    gPauseCycleConf = false;

    return !gPauseCycleReq;

#endif  // qPrintStat
}

/******************************************************************************
 * StartTimerStat : start timer to compute statistics
 *****************************************************************************/
static __INLINE void StartTimerStat(void)
{
	RAIL_Status_t status = RAIL_SetMultiTimer(&gStatDelayTimer, gStatDelay, RAIL_TIME_DELAY, &timer_callback, NULL);
	gElapsedTime = RAIL_GetTime();
	PrintStatus(status, "Warning RAIL_SetMultiTimer STAT PERIOD");
}

/******************************************************************************
 * StartTimerSyncTO : start sync cycle timeout timer
 *****************************************************************************/
static __INLINE void StartTimerSyncTO(void)
{
    RAIL_Status_t status = RAIL_SetMultiTimer(&gSyncTimeOutTimer, gSyncTimeOut, RAIL_TIME_DELAY, &timer_callback, NULL);
    PrintStatus(status, "Warning RAIL_SetMultiTimer SYNC TIMEOUT");
}



/******************************************************************************
 * The API helps to unpack the received packet, point to the payload and returns the length.
 *****************************************************************************/
static __INLINE uint16_t unpack_packet_from_rx(uint8_t *rx_destination, const RAIL_RxPacketInfo_t *packet_information, uint8_t **start_of_payload)
{
	RAIL_CopyRxPacket(rx_destination, packet_information);
	*start_of_payload = rx_destination;
	return ((packet_information->packetBytes > RAIL_FIFO_RX_SIZE) ? RAIL_FIFO_RX_SIZE : packet_information->packetBytes);
}

/******************************************************************************
 * The API prepares the packet for sending and load it in the RAIL TX FIFO
 *****************************************************************************/
static __INLINE void prepare_packet_to_tx(void)
{
	gTX_fifo.fifo[kCounter0] = gTX_counter.u8[0];
	gTX_fifo.fifo[kCounter1] = gTX_counter.u8[1];
	gTX_fifo.fifo[kCounter2] = gTX_counter.u8[2];
	gTX_fifo.fifo[kCounter3] = gTX_counter.u8[3];

	uint16_t allocated_tx_fifo_size = RAIL_SetTxFifo(gRailHandle, gTX_fifo.fifo, TX_PAYLOAD_LENGTH, RAIL_FIFO_TX_SIZE);
	app_assert(allocated_tx_fifo_size == RAIL_FIFO_TX_SIZE, "RAIL_SetTxFifo() failed to allocate a large enough fifo (%d bytes instead of %d bytes)\n", allocated_tx_fifo_size, RAIL_FIFO_TX_SIZE);
}

/******************************************************************************
 * StartReceive : change to RX mode
 *****************************************************************************/
static __INLINE void StartReceive(void)
{
	// Start RX and check result

	// For oscillo debug purposes
	GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_RX);

	// Start RX
	RAIL_Status_t status = RAIL_StartRx(gRailHandle, CHANNEL, NULL);
	PrintStatus(status, "Warning RAIL_StartRx");
}

/******************************************************************************
 * DecodeReceivedMsg : decode received data
 *****************************************************************************/
static __INLINE void DecodeReceivedMsg(void)
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
		uint16_t packet_size = unpack_packet_from_rx(gRX_fifo, &packet_info, &start_of_packet);
		status = RAIL_ReleaseRxPacket(gRailHandle, rx_packet_handle);
		PrintStatus(status, "Warning ReleaseRxPacket");

		if (packet_info.packetStatus == RAIL_RX_PACKET_READY_SUCCESS)
		{
			DisplayReceivedMsg(start_of_packet, packet_size);
			// Backup previous data
			gRX_counter_prev[me] = gRX_counter[me];
			// Extract received data
			gRX_counter[me] = (uint32_t) ((start_of_packet[kCounter0] << 0) + (start_of_packet[kCounter1] << 8) + (start_of_packet[kCounter2] << 16) + (start_of_packet[kCounter3] << 24));

			uint32_t delta = gRX_counter[me] - gRX_counter_prev[me];
			if (delta != 1)
			{
				gRX_tab[me][TAB_POS_RX_GAP]++;
				gRX_tab[me][TAB_POS_RX_GAP_MAX] = (delta > gRX_tab[me][TAB_POS_RX_GAP_MAX] ? delta : gRX_tab[me][TAB_POS_RX_GAP_MAX]);
			}
			else
				gRX_tab[me][TAB_POS_RX_OK]++;
		}
		else if (packet_info.packetStatus == RAIL_RX_PACKET_READY_CRC_ERROR)
		{
			gRX_tab[me][TAB_POS_RX_CRC_ERR]++;
		}

		// Indicate RX in progress on LED0
		sl_led_toggle(&sl_led_led0);
		rx_packet_handle = RAIL_GetRxPacketInfo(gRailHandle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);
	}
	//GPIO_PinOutClear(DEBUG_PORT, DEBUG_PIN_RX);
}

// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------

/******************************************************************************
 * Application state machine, called infinitely
 *****************************************************************************/
//static RAIL_RadioStateDetail_t radio_old_state_detail = RAIL_RF_STATE_DETAIL_INACTIVE;

void app_process_action(void)
{
    RAIL_Status_t status = RAIL_STATUS_NO_ERROR;

    /// -----------------------------------------
    /// Decode from interrupt / callback
    /// -----------------------------------------
    if (gRX_ok)                     // Msg received
    {
        gRX_ok = false;

        SetState(kSyncReceived);
    }
    else if (gTX_ok)                // Msg sent
    {
        gTX_ok = false;

        SetState(kMsgSent);
    }
    else if (gRX_error)             // Error during RX
    {
        gRX_error = false;

        SetState(kErrorRx);
    }
    else if (gTX_error)             // Error during TX
    {
        gTX_error = false;

        SetState(kErrorTx);
    }
    else if (gCAL_req)              // Calibration request
    {
        gCAL_req = false;

        SetState(kCalReq);
    }
    else if (gSYNC_timeout)         // Mast sync timeout
    {
        gSYNC_timeout = false;

        SetState(kSyncLost);
    }
    else if (gStatTimerDone)        // Periodic stat
    {
        gStatTimerDone = false;

        SetState(kStatistics);
    }
    else if (gBtnPressed)           // Button pressed: wait process starts then print stat
    {
        gBtnPressed = false;

        SetState(kBtnPressed);
    }
    else if (gStatReq)              // Stat via CLI
    {
        gStatReq = false;

        SetState(kStatistics);
    }

	/// -----------------------------------------
	/// State machine
	/// -----------------------------------------
	switch (gProtocolState)
	{
        // -----------------------------------------
        // Startup
        // -----------------------------------------
	case kInit:
		// State after POR

	    // Set node address
	    gTX_fifo.fifo[kAddr] = gDeviceCfgAddr->internalAddr;
	    me = gDeviceCfgAddr->posTab;

	    // TX fifo ready
		prepare_packet_to_tx();

        if (!gStartProcess)
            PrintInfo("\nInfo Ready ...\n");
        else
            PrintInfo("\nInfo Waiting next stat ...\n");

        gElapsedTime = RAIL_GetTime();
        gOldElapsedTime = gElapsedTime;

        // Listen from Master
        SetState(kListen);
		break;

        // -----------------------------------------
        // Idle
        // -----------------------------------------
	case kIdle:
		// Wait sync from master
		break;

        // -----------------------------------------
        // RX
        // -----------------------------------------
    case kListen:
        StartReceive();
        SetState(kIdle);
        break;

    case kSyncReceived:
        GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_TX);

        if (gRailScheduleCfgTX.when)
        {
            status = RAIL_StartScheduledTx(gRailHandle, CHANNEL, RAIL_TX_OPTIONS_DEFAULT, &gRailScheduleCfgTX, NULL);
            PrintStatus(status, "Warning RAIL_StartScheduledTx");
        }
        else    // don't necessary as transition auto RX -> TX
        {
            GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_MISC);
        }

        StartTimerSyncTO();         // Test if Master is still alive and sending Sync periodically

        if (!gStartProcess)
            gStartProcess = true;

        if (!gRX_first)
        {
            gRX_first = true;
            gTX_counter_old = gTX_counter.u32;
            gRX_counter_old[me] = gRX_counter[me];
            StartTimerStat();           // Print stat after deadline
        }

        // Extract received data from buffer
        DecodeReceivedMsg();

        SetState(kIdle);
        break;

    case kErrorRx:
        gRX_tab[me][TAB_POS_RX_ERR] = gCB_tab[RAIL_EVENT_RX_PACKET_ABORTED_SHIFT] +
                                      gCB_tab[RAIL_EVENT_RX_FRAME_ERROR_SHIFT] +
                                      gCB_tab[RAIL_EVENT_RX_FIFO_OVERFLOW_SHIFT] +
                                      gCB_tab[RAIL_EVENT_RX_SCHEDULED_RX_MISSED_SHIFT];

        PrintError(gErrorCode, "Error RX");

        // Auto transition to RX after unsuccessfull receive
        // For oscillo debug purposes
        GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_RX);

        SetState(kIdle);
        break;

        // -----------------------------------------
        // TX
        // -----------------------------------------
    case kMsgSent:
        gTX_tab[TAB_POS_TX_OK] = gCB_tab[RAIL_EVENT_TX_PACKET_SENT_SHIFT];

        // Increment counter and prepare new data
        gTX_counter.u32++;    // pas de test overflow
        prepare_packet_to_tx();

        if (gPauseCycleReq)
            gPauseCycleConf = true;     // Cycle ended, permit a print stat

        DisplaySentMsg();
        // Indicate TX in progress on LED1
        sl_led_toggle(&sl_led_led1);

        // Auto transition to RX after successfull transmit
        // For oscillo debug purposes
        GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_RX);

        SetState(kIdle);
        break;

    case kErrorTx:
        gTX_tab[TAB_POS_TX_ERR] = gCB_tab[RAIL_EVENT_TX_ABORTED_SHIFT] +
                                  gCB_tab[RAIL_EVENT_TX_BLOCKED_SHIFT] +
                                  gCB_tab[RAIL_EVENT_TX_UNDERFLOW_SHIFT] +
                                  gCB_tab[RAIL_EVENT_TX_CHANNEL_BUSY_SHIFT] +
                                  gCB_tab[RAIL_EVENT_TX_SCHEDULED_TX_MISSED_SHIFT];

         // Same data but tell RAIL that FIFO is set
        prepare_packet_to_tx();

        PrintError(gErrorCode, "Error TX");

        // Auto transition to RX after unsuccessfull transmit
        // For oscillo debug purposes
        GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_RX);

        SetState(kIdle);
        break;

        // -----------------------------------------
        // Calibration
        // -----------------------------------------
    case kCalReq:
        gCAL_tab[TAB_POS_CAL_REQ]++;

        status = RAIL_Calibrate(gRailHandle, NULL, RAIL_CAL_ALL_PENDING);
        if (status != RAIL_STATUS_NO_ERROR)
        {
            gCAL_tab[TAB_POS_CAL_ERR]++;

#if (qPrintErrorsL1)
            if (gErrorCode != gPrevErrorCode)
            {
                app_log_error("Error CAL (0x%llX / %d)\n", gErrorCode, status);
                gPrevErrorCode = gErrorCode;
            }
#endif  // qPrintErrorsL1
        }
        SetState(gProtocolPrevState);
        break;

        // -----------------------------------------
        // Timers
        // -----------------------------------------
    case kSyncLost:
        gRX_first = false;

        gRX_tab[me][TAB_POS_RX_SYNC_LOST]++;

        PrintInfo("\nWarning Master sync lost!\n");

        SetState(kInit);

        break;

        // -----------------------------------------
        // IHM
        // -----------------------------------------
    case kBtnPressed:
        if (!gStartProcess)
        {
            // Do nothing as process not yet started

            SetState(gProtocolPrevState);
        }
        else
        {
            SetState(kStatistics);
        }
        break;

    case kStatReq:                  // CLI stat
        gStatReq = false;

        SetState(kStatistics);
        break;

        // -----------------------------------------
        // Statistics
        // -----------------------------------------
    case kStatistics:
        // On timeout delay || Button pressed || CLI command
        if (PrintStatistics())
        {
            SetState(kInit);       // Restart the cycle
        }
        else // stay in this state upon stat print completion
            SetState(kStatistics);
        //

        break;

	default:
		app_assert(false, "Unknown state (%d)\n", gProtocolState);
		break;
	}
}

