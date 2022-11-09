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
#include "sl_udelay.h"              // Active delay

// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------
/// Size of RAIL RX/TX FIFO
#define RAIL_FIFO_RX_SIZE (512U)					// in bytes
#define RAIL_FIFO_TX_SIZE (128U)					// in bytes // don't try below 128k --> assert
/// Transmit data length (fixed payload)
#define TX_PAYLOAD_LENGTH (6U)						// in bytes

// Period to print statistics
#define STAT_PERIOD_s (20U)							// in us
#define STAT_PERIOD_us (STAT_PERIOD_s * 1000000ULL)	// in sec

#define SIZE_UINT64_IN_BITS	(int)(8*sizeof(uint64_t))

// Msg header
#define kNormalHeader       0xAA
#define kStatReqHeader      0x55

/// State machine
typedef enum
{
	kInit = 0, kStart, kStop, kIdle,

	kWaitRx, kMsgReceived, kErrorRx, kTimeOutRx, kSwitchRx,

	kSendMsg, kMsgSent, kErrorTx, kTimeOutTx, kSwitchTx,

	kErrorCal
} StateEnum;

/// Statistics tab position
typedef enum
{
    kTX_Ok          = 0,
    kRX_Ok          = 0,
    kCAL_Req        = 0,
    kTX_Err         = 1,
    kRX_Err         = 1,
    kCAL_Err        = 1,
    kRX_TimeOut     = 2,
    kTX_TimeOut     = 2,
    kTX_Retransmit  = 3,
    kRX_GapCount    = 3,
    kRX_MaxGap      = 4,
    kTX_Invalid     = 5,
    kRX_Invalid     = 5,
    kRX_CRC_Err     = 6

} StatTabPosEnum;

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
/// The variable shows the actual state of the state machine
static volatile StateEnum gProtocolState = kInit;
static volatile StateEnum gPrevProtocolState = kInit;

/// Contains the last RAIL Rx/Tx error events
static volatile uint64_t gErrorCode = RAIL_EVENTS_NONE;
#if (qPrintErrorsL1 || qPrintErrorsL2)
static uint64_t gOldErrorCode = RAIL_EVENTS_NONE;
#endif	// qPrintErrorsL2

/// Contains the status of RAIL Calibration
static volatile RAIL_Status_t gCalibrationStatus = RAIL_STATUS_NO_ERROR;

/// Timeout for printing and displaying the statistics
//static RAIL_Time_t gStatPeriodTimeout = 0UL;
static volatile uint32_t gCountPrintStat = 0UL;                       // Stat print counter (how many times)
static volatile uint32_t gCountTimerStat = 0UL;
/// A static handle of a RAIL timer
static RAIL_MultiTimer_t gStatPeriodTimer, gTX_timeout;
#if (qUseTimeOutRx)
static RAIL_MultiTimer_t gRX_timeout;
#endif  // qUseTimeOutRx


/// Receive and Send FIFO
static uint8_t gRX_fifo[RAIL_FIFO_RX_SIZE];

static union
{
	// Used to align this buffer as needed
	RAIL_FIFO_ALIGNMENT_TYPE align[RAIL_FIFO_TX_SIZE / RAIL_FIFO_ALIGNMENT];
	uint8_t fifo[RAIL_FIFO_TX_SIZE];
} gTX_fifo = { .fifo = {0x00, 0x06, 0x00, 0x00, 0x00, 0x00} };

/// Flags to update state machine from interrupt
static volatile bool gPacketRecieved = false;
static volatile bool gPacketSent = false;
static volatile bool gRX_error = false;
static volatile bool gRX_invalid = false;
static volatile bool gRX_timeout_error = false;
static volatile bool gTX_error = false;
static volatile bool gTX_invalid = false;
static volatile bool gTX_timeout_error = false;
static volatile bool gCAL_error = false;
//static volatile bool gStatTimerDone = false;
static volatile bool gStatReq = false;
static volatile bool gStatConf = false;

/// TX and RX counters
static volatile uint32_t gRxCount = 0UL;
static volatile uint32_t gTxCount = 1UL;
static uint32_t gRxPrevCount = 0UL;

/// Tables for error statistics
static volatile uint32_t gCB_tab[SIZE_UINT64_IN_BITS] = {0};	   // index: see RAIL_ENUM_GENERIC(RAIL_Events_t, uint64_t) in rail_types.h
const char *gCB_descr_tab[SIZE_UINT64_IN_BITS] = {
		  "RSSI_AVERAGE_DONE   ",
		  "RX_ACK_TIMEOUT      ",
		  "RX_FIFO_ALMOST_FULL ",
		  "RX_PACKET_RECEIVED  ",
		  "RX_PREAMBLE_LOST    ",
		  "RX_PREAMBLE_DETECT  ",
		  "RX_SYNC1_DETECT     ",
		  "RX_SYNC2_DETECT     ",
		  "RX_FRAME_ERROR      ",
		  "RX_FIFO_FULL        ",
		  "RX_FIFO_OVERFLOW    ",
		  "RX_ADDRESS_FILTERED ",
		  "RX_TIMEOUT          ",
		  "SCHEDULED_xX_STARTED",
		  "RX_SCHEDULED_RX_END ",
		  "RX_SCHEDULED_RX_MISS",
		  "RX_PACKET_ABORTED   ",
		  "RX_FILTER_PASSED    ",
		  "RX_TIMING_LOST      ",
		  "RX_TIMING_DETECT    ",
		  "RX_DUTY_CYCLE_RX_END",
		  "MFM_TX_BUFFER_DONE  ",
		  "ZWAVE_BEAM          ",
		  "TX_FIFO_ALMOST_EMPTY",
		  "TX_PACKET_SENT      ",
		  "TXACK_PACKET_SENT   ",
		  "TX_ABORTED          ",
		  "TXACK_ABORTED       ",
		  "TX_BLOCKED          ",
		  "TXACK_BLOCKED       ",
		  "TX_UNDERFLOW        ",
		  "TXACK_UNDERFLOW     ",
		  "TX_CHANNEL_CLEAR    ",
		  "TX_CHANNEL_BUSY     ",
		  "TX_CCA_RETRY        ",
		  "TX_START_CCA        ",
		  "TX_STARTED          ",
		  "TX_SCHEDULED_TX_MISS",
		  "CONFIG_UNSCHEDULED  ",
		  "CONFIG_SCHEDULED    ",
		  "SCHEDULER_STATUS    ",
		  "CAL_NEEDED          ",
		  "RF_SENSED           ",
		  "PA_PROTECTION       ",
		  "SIGNAL_DETECTED     ",
		  "IEEE802154_MSW_START",
		  "IEEE802154_MSW_END  ",
		  "DETECT_RSSI_THRSHOLD"};

// Statistics tab
static uint32_t gTX_tab[7] = { 0 };    // 0 = #TX_OK	1 = #TX_Err	 	2 = ND	 		3 = #Retransmit		4 = ND					5 = #TX_Invalid		6 = ND
static uint32_t gRX_tab[7] = { 0 };    // 0 = #RX_OK 	1 = #RX_Err 	2 = #RX_TimeOut	3 = #Gap RX count	4 = Max gap RX count	5 = #RX_Invalid		6 = #CRC_Err
static uint32_t gCAL_tab[7] = { 0 };   // 0 = #CAL_REQ 	1 = #CAL_Err	2 = ND			3 = ND				4 = ND					5 = ND				6 = ND

/// LCD variables
/// Context used all over the graphics
#if (qUseDisplay)
static GLIB_Context_t gGlibContext;
#endif  // qUseDisplay

/// Flag, indicating transmit request (button was pressed / CLI transmit request has occurred)
volatile bool gTX_requested = false;
/// Flag, indicating received packet is forwarded on CLI or not
volatile bool gRX_requested = true;
/// Flag, indicating stat period on CLI
volatile uint32_t gSTAT_period = STAT_PERIOD_us;
/// Various flags
static bool gFirstMsg = true;					// Indicate first RX packet received on Slave to start statistics
//static volatile bool gPrintStat = false;		// Print and display statistics
static volatile bool gStartProcess = false;		// Start ping pong
//static bool gTX_retry_on_error = false;			// Indicate a retransmit on error in order to bypass incrementation of TX counter

// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------

/******************************************************************************
 * Decode RAIL events and increase event's counters
 *****************************************************************************/
static void DecodeEvents(RAIL_Events_t *events);

/******************************************************************************
 * StartTimerTO_Stat : start timer to compute statistics
 *****************************************************************************/
static void StartTimerTO_Stat(void);

/******************************************************************************
 * StartTimerTO_RX : start timer on RX
 *****************************************************************************/
static void StartTimerTO_RX(void);

/******************************************************************************
 * StartTimerTO_TX : start timer on TX
 *****************************************************************************/
static void StartTimerTO_TX(void);


/******************************************************************************
 * RAIL callback, called if a RAIL event occurs.
 *****************************************************************************/
void sl_rail_util_on_event(RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
    gErrorCode = events;    // Save events context

    DecodeEvents(&events);  // Count events for debug and statistics

    // Handle RX events
    if (events & RAIL_EVENTS_RX_COMPLETION)
    {
        if (events & RAIL_EVENT_RX_PACKET_RECEIVED)
        {
            // Keep the packet in the radio buffer, download it later at the state machine
            RAIL_HoldRxPacket(rail_handle);
            gPacketRecieved = true;
        }
        else    //  | RAIL_EVENT_RX_PACKET_ABORTED
                //  | RAIL_EVENT_RX_FRAME_ERROR
                //  | RAIL_EVENT_RX_FIFO_OVERFLOW
                //  | RAIL_EVENT_RX_ADDRESS_FILTERED
                //  | RAIL_EVENT_RX_SCHEDULED_RX_MISSED
        {
            // Handle Rx error
            gRX_error = true;
        }
    }

#if (qUseScheduleRx)
    // Handle Rx timeout --> needed when RAIL_ScheduleRx used
    if (events & (1ULL << RAIL_EVENT_RX_SCHEDULED_RX_END_SHIFT))
    {
        gRX_timeout_error = true;
    }
#endif  // qUseScheduleRx

    // Handle TX events
    if (events & RAIL_EVENTS_TX_COMPLETION)
    {
        if (events & RAIL_EVENT_TX_PACKET_SENT)
        {
            // Handle next step
            gPacketSent = true;
        }
        else    //  | RAIL_EVENT_TX_ABORTED
                //  | RAIL_EVENT_TX_BLOCKED
                //  | RAIL_EVENT_TX_UNDERFLOW
                //  | RAIL_EVENT_TX_CHANNEL_BUSY
                //  | RAIL_EVENT_TX_SCHEDULED_TX_MISSED)
        {
            // Handle Tx error
            gTX_error = true;
        }
    }

    // Perform all calibrations when needed
    if (events & RAIL_EVENT_CAL_NEEDED)
    {
        gCAL_tab[kCAL_Req]++;
        gCalibrationStatus = RAIL_Calibrate(rail_handle, NULL, RAIL_CAL_ALL_PENDING);
        if (gCalibrationStatus != RAIL_STATUS_NO_ERROR)
        {
            gCAL_error = true;
        }
    }
}

/******************************************************************************
 * Timer callback, called if any button is pressed or released.
 *****************************************************************************/
void timer_callback(RAIL_MultiTimer_t *tmr, RAIL_Time_t expectedTimeOfEvent, void *cbArg)
{
    (void) expectedTimeOfEvent;     // To avoid warnings
    (void) cbArg;                   // To avoid warnings

    if (tmr == &gStatPeriodTimer)   // used to print stat periodically
    {
        gCountTimerStat++;
#if (qMaster)
        gStatReq = true;
#endif  // qMaster
    }
#if (qUseTimeOutRx)
    else if (tmr == &gRX_timeout)   // used with standard StartRx when RAIL_ScheduleRx isn't used
    {
        gRX_timeout_error = true;
        GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_MISC);
    }
#endif  // qUseTimeOutRx
    else if (tmr == &gTX_timeout)   // used with standard StartTx when RAIL_StartScheduledTx isn't used
    {
        gTX_timeout_error = true;
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
        }
        else
        {

        }
#else   // !qMaster

#endif  // qMaster
    }
}
// -----------------------------------------------------------------------------
//                          Private Function Declarations
// -----------------------------------------------------------------------------
/******************************************************************************
 * SetState : change state machine and backup previous state machine
 *****************************************************************************/
static __INLINE void SetState(StateEnum state)
{
	gPrevProtocolState = gProtocolState;
	gProtocolState = state;
}

/******************************************************************************
 * Decode RAIL events and increase event's counters
 *****************************************************************************/
static __INLINE void DecodeEvents(RAIL_Events_t *events)
{
#if (qPrintEvents)
    uint64_t ev = *events;

    for (int i = 0; i <SIZE_UINT64_IN_BITS; i++)
    {
        if (ev & 0x1ULL)
            gCB_tab[i]++;

        ev >>= 1;
    }
#else
    (void) events;          // To avoid warnings
#endif  // qPrintEvents
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
    if (gStatReq)
    {
#if (qMaster)
        gTX_fifo.fifo[2] = (gCountPrintStat & 0x000000FF) >> 0;
        gTX_fifo.fifo[3] = (gCountPrintStat & 0x0000FF00) >> 8;
        gTX_fifo.fifo[4] = (gCountPrintStat & 0x00FF0000) >> 16;
        gTX_fifo.fifo[5] = (gCountPrintStat & 0xFF000000) >> 24;
#endif  // qMaster
    }
    else
    {
        gTX_fifo.fifo[2] = (gTxCount & 0x000000FF) >> 0;
        gTX_fifo.fifo[3] = (gTxCount & 0x0000FF00) >> 8;
        gTX_fifo.fifo[4] = (gTxCount & 0x00FF0000) >> 16;
        gTX_fifo.fifo[5] = (gTxCount & 0xFF000000) >> 24;
    }

    uint16_t allocated_tx_fifo_size = RAIL_SetTxFifo(gRailHandle, gTX_fifo.fifo, TX_PAYLOAD_LENGTH, RAIL_FIFO_TX_SIZE);
    app_assert(allocated_tx_fifo_size == RAIL_FIFO_TX_SIZE, "RAIL_SetTxFifo() failed to allocate a large enough fifo (%d bytes instead of %d bytes)\n", allocated_tx_fifo_size, RAIL_FIFO_TX_SIZE);
}

/******************************************************************************
 * EncodeTXMsg : build packet to be transmitted
 *****************************************************************************/
static __INLINE void EncodeTXMsg(void)
{
    if (gStatReq)
    {
#if (qMaster)
        gTX_fifo.fifo[0] = kStatReqHeader;

        gCountPrintStat = gCountTimerStat;
#endif  // qMaster
    }
    else
    {
        gTX_fifo.fifo[0] = kNormalHeader;

        gTxCount++;      // pas de test over
    }

    prepare_packet_to_tx();
}

/******************************************************************************
 * InitiateRX : change to RX mode
 *****************************************************************************/
static __INLINE void InitiateRX(void)
{
	// Start RX and check result

	// For oscillo debug purposes
	GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_RX);

	// RX with timeout
#if (qUseScheduleRx)
	RAIL_Status_t status = RAIL_ScheduleRx(gRailHandle, CHANNEL, &gRailScheduleCfgRX, NULL);
#else
//  Or standard StartRx with timeout handled by a timer
	RAIL_Status_t status = RAIL_StartRx(gRailHandle, CHANNEL, NULL);
#endif	// qUseScheduleRx

	if (status != RAIL_STATUS_NO_ERROR)
	{
#if (qPrintErrorsL1)
#if (qUseScheduleRx)
		app_log_warning("Warning RAIL_ScheduleRx (%d)\n", status);
#else
		app_log_warning("Warning RAIL_StartRx (%d)\n", status);
#endif	// qUseScheduleRx
#endif	// qPrintErrorsL1
	}

	StartTimerTO_RX();
}

/******************************************************************************
 * InitiateTX : prepare buffer and change to TX mode
 *****************************************************************************/
static __INLINE void InitiateTX(void)
{
    // Update radio buffer
    EncodeTXMsg();

	// For oscillo debug purposes
	GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_TX);

#if (qUseScheduleTx)
	// TX delayed
//	RAIL_Status_t status = RAIL_StartScheduledTx(gRailHandle, CHANNEL, RAIL_TX_OPTIONS_DEFAULT, &gRailScheduleCfgTX, NULL);
#else
	// TX now
//  Or standard StartTx with timeout handled by a timer
	RAIL_Status_t status = RAIL_StartTx(gRailHandle, CHANNEL, RAIL_TX_OPTIONS_DEFAULT, NULL);
#endif	// qUseScheduleTx

	if (status != RAIL_STATUS_NO_ERROR)
	{
#if (qPrintErrorsL1)
#if (qUseScheduleTx)
		app_log_warning("Warning RAIL_StartScheduledTx (%d)\n", status);
#else
		app_log_warning("Warning RAIL_StartTx (%d)\n", status);
#endif	// qUseScheduleTx
#endif	// qPrintErrorsL1
	}

	StartTimerTO_TX();
}

/******************************************************************************
 * StartTimerTO_Stat : start timer to compute statistics
 *****************************************************************************/
static __INLINE void StartTimerTO_Stat(void)
{
	RAIL_Status_t status = RAIL_SetMultiTimer(&gStatPeriodTimer, gSTAT_period, RAIL_TIME_DELAY, &timer_callback, NULL);
	if (status != RAIL_STATUS_NO_ERROR)
	{
#if (qPrintErrorsL1)
		app_log_warning("Warning RAIL_SetMultiTimer STAT (%d)\n", status);
#endif	// qPrintErrorsL1
	}
}

/******************************************************************************
 * StartTimerTO_RX : start timer on RX
 *****************************************************************************/
static __INLINE void StartTimerTO_RX(void)
{
#if (qUseTimeOutRx)
	RAIL_Status_t status = RAIL_SetMultiTimer(&gRX_timeout, RX_TIMEOUT, RAIL_TIME_DELAY, &timer_callback, NULL);

	if (status != RAIL_STATUS_NO_ERROR)
	{
#if (qPrintErrorsL1)
		app_log_warning("Warning RAIL_SetMultiTimer RX (%d)\n", status);
#endif	// qPrintErrorsL1
	}
#endif	// qUseTimeOutRx
}

/******************************************************************************
 * StartTimerTO_TX : start timer on TX
 *****************************************************************************/
static __INLINE void StartTimerTO_TX(void)
{
#if (qUseTimeOutTx)
	RAIL_Status_t status = RAIL_SetMultiTimer(&gTX_timeout, TX_TIMEOUT, RAIL_TIME_DELAY, &timer_callback, NULL);

	if (status != RAIL_STATUS_NO_ERROR)
	{
#if (qPrintErrorsL1)
		app_log_warning("Warning RAIL_SetMultiTimer RX (%d)\n", status);
#endif	// qPrintErrorsL1
	}
#endif	// qUseTimeOutTx
}

/******************************************************************************
 * DisplayRXMsg : print received data
 *****************************************************************************/
static __INLINE void DisplayRXMsg(const uint8_t *const rx_buffer, uint16_t length)
{
#if (qPrintRX)
	// Print received data on serial COM
	app_log_info("RX: ");
	for (uint16_t i = 0; i < length; i++) {
		app_log_info("0x%02X, ", rx_buffer[i]);
	}
	app_log_info("\n");
#else
    (void) rx_buffer;   // To avoid warnings
	(void) length;		// To avoid warnings
#endif
}

/******************************************************************************
 * DisplayTXMsg : print sent data
 *****************************************************************************/
static __INLINE void DisplayTXMsg(void)
{
#if (qPrintTX)
	// Print sent data on serial COM
	app_log_info("TX: %d\n",gTxCount);
#endif
}

/******************************************************************************
 * DecodeRXMsg : decode received data
 *****************************************************************************/
static __INLINE void DecodeRXMsg(void)
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
		if (status != RAIL_STATUS_NO_ERROR)
		{
#if (qPrintErrorsL1)
			app_log_warning("Error ReleaseRxPacket (%d)\n", status);
#endif	// qPrintErrorsL1
		}

		if ((packet_info.packetStatus == RAIL_RX_PACKET_READY_SUCCESS))
		{
		    if (start_of_packet[0] == kNormalHeader)
		    {
                // Backup previous data
                gRxPrevCount = gRxCount;
                // Extract received data
                gRxCount = (uint32_t) ((start_of_packet[2] << 0) + (start_of_packet[3] << 8) + (start_of_packet[4] << 16) + (start_of_packet[5] << 24));

                uint32_t delta = gRxCount - gRxPrevCount;
                if (delta != 1)
                {
                    gRX_tab[kRX_GapCount]++;
                    gRX_tab[kRX_MaxGap] = (delta > gRX_tab[kRX_MaxGap] ? delta : gRX_tab[kRX_MaxGap]);
                }
                else
                    gRX_tab[kRX_Ok]++;

                DisplayRXMsg(start_of_packet, packet_size);
		    }
		    else if (start_of_packet[0] == kStatReqHeader)
		    {
#if (!qMaster)
		        gStatConf = true;

		        gCountPrintStat = (uint32_t) ((start_of_packet[2] << 0) + (start_of_packet[3] << 8) + (start_of_packet[4] << 16) + (start_of_packet[5] << 24));
#endif  // !qMaster
		    }
		}
		else
			gRX_tab[kRX_CRC_Err]++;

		// Indicate RX in progress on LED0
		sl_led_toggle(&sl_led_led0);
		rx_packet_handle = RAIL_GetRxPacketInfo(gRailHandle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);
	}
	GPIO_PinOutClear(DEBUG_PORT, DEBUG_PIN_RX);
}

/******************************************************************************
 * DisplayStat : print event counters
 *****************************************************************************/
void DisplayCallBackStat(void)
{
#if (qPrintEvents)
	app_log_info("\n");
	app_log_info("Radio events detail\n");
	app_log_info("-------------------\n");
	for (int i = 0; i <SIZE_UINT64_IN_BITS; i++)
	{
		if (gCB_tab[i] >0)
		{
			app_log_info("b%02d %s: %d\n", i, gCB_descr_tab[i], gCB_tab[i]);
		}
	}
#endif	// qPrintEvents
}
/******************************************************************************
 * DisplayStat : print and display statistics
 *****************************************************************************/
void DisplayStat(void)
{
    if (gStatConf)
    {
#if (qMaster)
        // Disable TX timeout
        bool bstatus = RAIL_CancelMultiTimer(&gTX_timeout);
#if (qPrintErrorsL1)
    if (bstatus != true)
    {
        app_log_warning("RAIL_CancelMultiTimer (%d)\n", bstatus);
    }
#endif  // qPrintErrorsL1
#endif  // qMaster

        // Stop radio
        RAIL_Idle(gRailHandle, RAIL_IDLE_FORCE_SHUTDOWN_CLEAR_FLAGS, true);

        // Statistics
        float time = (float) ((float) gCountPrintStat * STAT_PERIOD_s);
        float localStat = (float) (gTxCount + gRxCount) / time;
        float localStat2 = (10.0f * 10100.0f) / localStat;
        float localStat3 = 1000000.0f * (float) (gTX_tab[kTX_Err] + gTX_tab[kTX_TimeOut] + gTX_tab[kTX_Invalid]) / (float) gTxCount;
        float localStat4 = 1000000.0f * (float) (gRX_tab[kRX_Err] + gRX_tab[kRX_TimeOut] + gTX_tab[kRX_Invalid] + gRX_tab[kRX_CRC_Err] + gRX_tab[kRX_GapCount]) / (float) gRxCount;

        // Print on serial COM
        app_log_info("\n");
        app_log_info("Performance statistics\n");
        app_log_info("----------------------\n");
        app_log_info("#Stat              : %d\n", gCountPrintStat);
        app_log_info("Elapsed time       : %0.2f sec\n", time);
#if (qMaster)
        app_log_info("Count (#TX Master) : %d\n", gTxCount);
        app_log_info("Count (#TX Slave)  : %d\n", gRxCount);
#else   // !qMaster
        app_log_info("Count (#TX Slave)  : %d\n", gTxCount);
        app_log_info("Count (#TX Master) : %d\n", gRxCount);
#endif  // qMaster
        app_log_info("#TX OK             : %d\n", gTX_tab[kTX_Ok]);
        app_log_info("#RX OK             : %d\n", gRX_tab[kRX_Ok]);
        app_log_info("TX Error see below : %0.0f ppm\n", localStat3);
        app_log_info("#Err/#TO/#Inv      : %d/%d/%d\n", gTX_tab[kTX_Err], gTX_tab[kTX_TimeOut], gTX_tab[kTX_Invalid]);
        app_log_info("TX retransmit count: %d\n", gTX_tab[kTX_Retransmit]);
        app_log_info("RX Error see below : %0.0f ppm\n", localStat4);
        app_log_info("#Err/#TO/#Inv/#CRC : %d/%d/%d/%d\n", gRX_tab[kRX_Err], gRX_tab[kRX_TimeOut], gRX_tab[kRX_Invalid], gRX_tab[kRX_CRC_Err]);
#if (qMaster)
        app_log_info("Slave counter #gap : %d (max:%d)\n", gRX_tab[kRX_GapCount], gRX_tab[kRX_MaxGap]);
#else   // !qMaster
        app_log_info("Master counter #gap: %d (max:%d)\n", gRX_tab[kRX_GapCount], gRX_tab[kRX_MaxGap]);
#endif  // qMaster
        app_log_info("Cal request (#Err) : %d (%d)\n", gCAL_tab[kCAL_Req], gCAL_tab[kCAL_Err]);
        app_log_info("Msg Rate (loop 100): %0.2f f/s (%0.2f ms)\n", localStat, localStat2);

        DisplayCallBackStat();

#if (qUseDisplay)
        // Print on LCD display
        char textBuf[32];

        GLIB_setFont(&gGlibContext, (GLIB_Font_t*) &GLIB_FontNormal8x8);
        snprintf(textBuf, sizeof(textBuf), "TX Cnt %d          ", gTX_tab[kTX_Ok]);
        GLIB_drawStringOnLine(&gGlibContext, textBuf, 5, GLIB_ALIGN_LEFT, 0, 0, true);
        snprintf(textBuf, sizeof(textBuf), "RX Cnt %d          ", gRX_tab[kRX_Ok]);
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

        // Restart radio
        RAIL_Idle(gRailHandle, RAIL_IDLE, true);

        gStatReq = false;
        gStatConf = false;
        gFirstMsg = true;

#if (qMaster)
        sl_udelay_wait(5000);       // As Master is starting first his print and print volume can vary between Master and Slave,
        // wait 5 ms to allow time to Slave to be ready after its stat print out
        SetState(kStart);                // At the end of stat print, restart the cycle
#else
        // Enable Start reception (without timeout)
        RAIL_Status_t status = RAIL_StartRx(gRailHandle, CHANNEL, NULL);
        if (status != RAIL_STATUS_NO_ERROR)
        {
#if (qDebugPrintErr)
            app_log_warning("Warning RAIL_StartRx (%d)\n", status);
#endif  // qDebugPrintErr
        }

        SetState(kWaitRx);
#endif  // qMaster
    }
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
#if (qPrintErrorsL1)
		app_log_error("Error DMD_init (%d)\n", status);
#endif	//qPrintErrorsL1
		return;
	}

	status = GLIB_contextInit(&gGlibContext);
	if (GLIB_OK != status)
	{
#if (qPrintErrorsL1)
		app_log_error("Error GLIB_contextInit (%d)\n", status);
#endif	//qPrintErrorsL1
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
	RAIL_RadioState_t radio_state = RAIL_RF_STATE_INACTIVE;

	/// -----------------------------------------
	/// Decode from interrupt / callback
	/// -----------------------------------------
	if (gPacketRecieved)
	{
		gPacketRecieved = false;

#if (!qMaster)
		if (gFirstMsg)
		{
			gFirstMsg = false;
			StartTimerTO_Stat();
		}
#endif	// !qMaster

		SetState(kMsgReceived);
	}
	else if (gPacketSent)
	{
		gPacketSent = false;
		GPIO_PinOutClear(DEBUG_PORT, DEBUG_PIN_TX);
		gTX_tab[kTX_Ok]++;

		SetState(kMsgSent);
	}
	else if (gRX_error)
	{
		gRX_error = false;
		gRX_tab[kRX_Err]++;

		SetState(kErrorRx);
	}
	else if (gRX_timeout_error)
	{
		gRX_timeout_error = false;
		GPIO_PinOutClear(DEBUG_PORT, DEBUG_PIN_MISC);
		gRX_tab[kRX_TimeOut]++;

		SetState(kTimeOutRx);
	}
	else if (gRX_invalid)
	{
		gRX_invalid = false;
		gRX_tab[kRX_Invalid]++;

		SetState(kErrorRx);
	}
	else if (gTX_error)
	{
		gTX_error = false;
		gTX_tab[kTX_Err]++;

		SetState(kErrorTx);
	}
	else if (gTX_timeout_error)
	{
		gTX_timeout_error = false;
		gTX_tab[kTX_TimeOut]++;

		SetState(kTimeOutTx);
	}
	else if (gTX_invalid)
	{
		gTX_invalid = false;
		gTX_tab[kTX_Invalid]++;

		SetState(kErrorTx);
	}
	else if (gCAL_error)
	{
		gCAL_error = false;
		gCAL_tab[kCAL_Err]++;

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
	    // Initialize radio buffer
	    prepare_packet_to_tx();
#if (qMaster)
		SetState(kIdle);
#else	// !qMaster
		SetState(kWaitRx);
#if (qUseDisplay)
		GLIB_setFont(&gGlibContext, (GLIB_Font_t*) &GLIB_FontNarrow6x8);
		GLIB_drawStringOnLine(&gGlibContext, ">>> RUNNING >>>", 12, GLIB_ALIGN_CENTER, 0, 0, true);
		// Force a redraw
		DMD_updateDisplay();
#endif  // qUseDisplay
#endif	// qMaster
		break;

	case kStart:
		// TX started by pressing BTN0
		SetState(kSwitchTx);

		if (gCountPrintStat == 0)
		    app_log_info("Start ping pong ...\n");
		else
            app_log_info("Continue ping pong ...\n");
		sl_led_turn_on(&sl_led_led1);
#if (qUseDisplay)
		GLIB_setFont(&gGlibContext, (GLIB_Font_t*) &GLIB_FontNarrow6x8);
		GLIB_drawStringOnLine(&gGlibContext, ">>> RUNNING >>>", 12, GLIB_ALIGN_CENTER, 0, 0, true);
		// Force a redraw
		DMD_updateDisplay();
#endif  // qUseDisplay
		StartTimerTO_Stat();
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
        if (!gStatConf)
        {
            // Message received
#if (qRx2TxAutoTransition)
            SetState(kIdle);        // Auto transition to TX after successfull receive
            // For oscillo debug purposes
            GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_TX);
            StartTimerTO_TX();      // use TX_timeout in order to be sure to restart with TX only on Master if time outs
#else
            SetState(kSwitchTx);
#endif  // qNoRx2TxAutoTransition
            DecodeRXMsg();
        }
		break;

	case kErrorRx:
		// Error on RX
#if (qAutoTransition)
		SetState(kIdle);		// Auto transition to RX after unsuccessfull receive
		// For oscillo debug purposes
		GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_RX);
		StartTimerTO_RX();		// use RX_timeout in order to be sure to restart with TX only on Master if time outs
#if (qMaster)
		SetState(kSwitchRx);
#else
		SetState(kSwitchRx);
#endif	// qMaster
#endif	// qAutoTransition
#if (qPrintErrorsL2)
		if (gErrorCode != gOldErrorCode)
		{
			app_log_error("RX Error (0x%llX)\n", gErrorCode);
			gOldErrorCode = gErrorCode;
		}
#endif	// qPrintErrorsL2
		break;

	case kTimeOutRx:
		// Timeout on RX
#if (qMaster)
		SetState(kSwitchTx);
#else
		SetState(kSwitchRx);
#endif
#if (qPrintErrorsL2)
		if (gErrorCode != gOldErrorCode)
		{
			app_log_error("RX TimeOut (0x%llX)\n", gErrorCode);
			gOldErrorCode = gErrorCode;
		}
#endif	// qPrintErrorsL2
		break;

	case kSwitchRx:
		radio_state = RAIL_GetRadioState(gRailHandle);

		if ((radio_state != RAIL_RF_STATE_TX_ACTIVE) &&
			(radio_state != RAIL_RF_STATE_RX_ACTIVE))
		{
			SetState(kWaitRx);
			InitiateRX();
		}
		else
		{
			SetState(kSwitchRx);	// Wait until transition ready
		}
		break;

		// -----------------------------------------
		// TX
		// -----------------------------------------
	case kSendMsg:
		// Wait until message sent
		break;

	case kMsgSent:
	    if (gTX_fifo.fifo[0] == kStatReqHeader)
	    {
#if (qMaster)
	        gStatConf = true;
#endif  // qMaster
	    }
	    else
	    {
            DisplayTXMsg();
#if (qAutoTransition)
	        // Initialize radio buffer
	        EncodeTXMsg();
	        SetState(kIdle);        // Auto transition to RX after successfull transmit
	        // For oscillo debug purposes
	        GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_RX);
	        StartTimerTO_RX();      // use RX_timeout in order to be sure to restart with TX only on Master if time outs
#else
	        SetState(kSwitchRx);    // bidirectional
#endif  // qAutoTransition

	        // Indicate TX in progress on LED1
	        sl_led_toggle(&sl_led_led1);
	    }
	    break;

	case kErrorTx:
#if (qAutoTransition)
		SetState(kIdle);		// Auto transition to RX after unsuccessfull transmit
		// For oscillo debug purposes
		GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_RX);
		StartTimerTO_RX();		// use RX_timeout in order to be sure to restart with TX only on Master if time outs
#else
#if (qMaster)
		SetState(kSwitchRx);
#else
		SetState(kSwitchRx);
#endif	// qMaster
#endif	// qAutoTransition

#if (qPrintErrorsL2)
		if (gErrorCode != gOldErrorCode)
		{
			app_log_error("TX Error (0x%llX)\n", gErrorCode);
			gOldErrorCode = gErrorCode;
		}
#endif	// qPrintErrorsL2
		break;

	case kTimeOutTx:
		// Timeout on TX
#if (qMaster)
#if (qRx2TxAutoTransition)
		SetState(kSwitchTx);
#else	// !qRx2TxAutoTransition
		SetState(kSwitchRx);
#endif	// qRx2TxAutoTransition
#else	// !qMaster
		SetState(kSwitchRx);
#endif	// qMaster
#if (qPrintErrorsL2)
		if (gErrorCode != gOldErrorCode)
		{
			app_log_error("TX TimeOut (0x%llX)\n", gErrorCode);
			gOldErrorCode = gErrorCode;
		}
#endif	// qPrintErrorsL2
		break;

	case kSwitchTx:
		radio_state = RAIL_GetRadioState(gRailHandle);

		if ((radio_state != RAIL_RF_STATE_TX_ACTIVE) &&
			(radio_state != RAIL_RF_STATE_RX_ACTIVE))
		{
			SetState(kSendMsg);
			InitiateTX();
		}
		else
		{
			SetState(kSwitchTx);	// Wait until transition ready
		}
		break;

		// -----------------------------------------
		// CAL
		// -----------------------------------------
	case kErrorCal:
#if (qPrintErrorsL1)
		if (gErrorCode != gOldErrorCode)
		{
			app_log_error("CAL Error (0x%llX / %d)\n", gErrorCode, gCalibrationStatus);
			gOldErrorCode = gErrorCode;
		}
#endif	// qPrintErrorsL1
		break;

	default:
		app_assert(false, "Unknown state (%d)\n", gProtocolState);
		break;
	}

	/// -----------------------------------------
	/// Statistics
	/// -----------------------------------------
	DisplayStat();
}




