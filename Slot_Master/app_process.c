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
#include "app_init.h"

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
    kStart,
    kIdle,
//  ------------
//	RX
//  ------------
    kWaitMsg,
    kMsgReceived,
    kErrorRx,
    kListen,
//  ------------
//	TX
//  ------------
    kSendingMsg,
    kMsgSent,
    kErrorTx,
    kSendSync,
//  ------------
//	CAL
//  ------------
    kErrorCal

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
static volatile StateEnum gPrevProtocolState = kInit;

/// Contains the last and previous RAIL Rx/Tx error events
static volatile uint64_t gErrorCode = RAIL_EVENTS_NONE;
static uint64_t gPrevErrorCode = RAIL_EVENTS_NONE;

/// Contains the status of RAIL Calibration
static volatile RAIL_Status_t gCalibrationStatus = RAIL_STATUS_NO_ERROR;

/// A static handle of a RAIL timers
static RAIL_MultiTimer_t gStatDelayTimer, gSyncPeriodTimer;

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
static volatile bool gSyncTimerDone = false;
static volatile bool gCAL_error = false;

/// Flags to update start print statistics
static volatile bool gPrintStat = false;    	// Print and display statistics
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
volatile bool gBtnPressed = false;   			// Button pressed, start process
static bool gPauseCycleConf = false;            // Flag to indicate a end of cycle of transmission with the slaves in order to avoid side effect of the print statistics
static bool gPauseCycleReq = false;             // Flag to indicate that a print stat is pending and request to pause the cycle at the next occasion (end of cycle)

// Various var
static uint8_t me;                                              // Position in the config file (for address, time slot, ...)
/// Value, indicating print stat delay on CLI
extern volatile RAIL_Time_t gStatDelay;

// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------
/******************************************************************************
 * StartTimerSync : start sync cycle timer
 *****************************************************************************/
static __INLINE void StartTimerSync(void);


// -----------------------------------------------------------------------------
//                          Callback Function Definitions
// -----------------------------------------------------------------------------
/******************************************************************************
 * RAIL callback, called if a RAIL event occurs.
 *****************************************************************************/
void sl_rail_util_on_event(RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
    gErrorCode = events;  // Save events context

    DecodeEvents(&events);  // Count events for debug and statistics

    // Handle RX events
    if (events & RAIL_EVENTS_RX_COMPLETION)
    {

        if (events & RAIL_EVENT_RX_PACKET_RECEIVED)
        {
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

            gTX_ok = true;
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

    // Perform all calibrations when needed
    if (events & RAIL_EVENT_CAL_NEEDED)
    {
        gCAL_tab[TAB_POS_CAL_REQ]++;
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
        if (!gStartProcess)
        {
            gBtnPressed = true;

            gStartProcess = true;
            gElapsedTime = RAIL_GetTime();
        }
        else
        {
            gOldElapsedTime = gElapsedTime;
            gElapsedTime = RAIL_GetTime();
            gPrintStat = true;
        }
    }
}

/******************************************************************************
 * Timer callback, called if any button is pressed or released.
 *****************************************************************************/
void timer_callback(RAIL_MultiTimer_t *tmr, RAIL_Time_t expectedTimeOfEvent, void *cbArg)
{
    (void) expectedTimeOfEvent;   // To avoid warnings
    (void) cbArg;         // To avoid warnings

    if (tmr == &gStatDelayTimer) // used to print stat periodically
    {
        gStatTimerDone = true;
        gOldElapsedTime = gElapsedTime;
        gElapsedTime = RAIL_GetTime();
    }
    else if (tmr == &gSyncPeriodTimer) // used with standard StartTx when RAIL_StartScheduledTx isn't used
    {
        gSyncTimerDone = true;

        if (!gPauseCycleReq)       // If no request to print stat is pending, restart cycle with the slaves
        {
            // In order to be the most accurate as possible, start StartTX directly from here
            GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_TX);
            /*RAIL_Status_t status = */RAIL_StartTx(gRailHandle, CHANNEL, RAIL_TX_OPTIONS_DEFAULT, NULL);
            StartTimerSync();
        }
        else
        {
            gPauseCycleConf = true;// Cycle ended, permit a print stat
        }
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
    gPrevProtocolState = gProtocolState;
    gProtocolState = state;
}

/******************************************************************************
 * PrintStatus : print return <> RAIL_STATUS_NO_ERROR status from RAIL functions
 *****************************************************************************/
static __INLINE void PrintStatus(RAIL_Status_t status, char *text)
{
#if (qPrintErrorsL1)
    if (status != RAIL_STATUS_NO_ERROR)
    {
        app_log_warning("%s (%d)\n", text, status);
    }
#else	// To avoid compile warnings
	(void) status;
	(void) text;
#endif  // qPrintErrorsL1
}

/******************************************************************************
 * PrintError : print error event code from RAIL callback
 *****************************************************************************/
static __INLINE void PrintError(uint64_t errcode, char *text)
{
#if (qPrintErrorsL2)
    if (errcode != gPrevErrorCode)
    {
      app_log_error("%s (0x%llX)\n", text, errcode);
      gPrevErrorCode = errcode;
    }
#else	// To avoid compile warnings
    (void) errcode;
    (void) text;
#endif  // qPrintErrorsL2
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
 * StartTimerSync : start sync cycle timer
 *****************************************************************************/
static __INLINE void StartTimerSync(void)
{
    RAIL_Status_t status = RAIL_SetMultiTimer(&gSyncPeriodTimer, gSyncPeriod, RAIL_TIME_DELAY, &timer_callback, NULL);
    PrintStatus(status, "Warning RAIL_SetMultiTimer SYNC PERIOD");
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
    gSyncTimerDone = false;
    gCAL_error = false;
}

/******************************************************************************
 * RestartRadio : start radio
 *****************************************************************************/
static __INLINE void RestartRadio(void)
{
    RAIL_Idle(gRailHandle, RAIL_IDLE, true);
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

    // Start RX without timeout
    RAIL_Status_t status = RAIL_StartRx(gRailHandle, CHANNEL, NULL);
    PrintStatus(status, "Warning RAIL_StartRx");
}

/******************************************************************************
 * StartTransmit : prepare buffer and change to TX mode
 *****************************************************************************/
static __INLINE void StartTransmit(void)
{
    // Initialize radio buffer
    prepare_packet_to_tx();

    // For oscillo debug purposes
    GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_TX);

    RAIL_Status_t status = RAIL_StartTx(gRailHandle, CHANNEL, RAIL_TX_OPTIONS_DEFAULT, NULL);
    PrintStatus(status, "Warning RAIL_StartTx");

    StartTimerSync();
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

            uint8_t addr = start_of_packet[kAddr];

            // Backup previous data
            gRX_counter_prev[addr] = gRX_counter[addr];
            // Extract received data
            gRX_counter[addr] = (uint32_t) ((start_of_packet[kCounter0] << 0) + (start_of_packet[kCounter1] << 8) + (start_of_packet[kCounter2] << 16) + (start_of_packet[kCounter3] << 24));

            uint32_t delta = gRX_counter[addr] - gRX_counter_prev[addr];
            if (delta != 1)
            {
                gRX_tab[addr][TAB_POS_RX_GAP]++;
                gRX_tab[addr][TAB_POS_RX_GAP_MAX] = (delta > gRX_tab[addr][TAB_POS_RX_GAP_MAX] ? delta : gRX_tab[addr][TAB_POS_RX_GAP_MAX]);
            }
            else
                gRX_tab[addr][TAB_POS_RX_OK]++;
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
void app_process_action(void)
{
    RAIL_RadioState_t radio_state = RAIL_RF_STATE_INACTIVE;

    /// -----------------------------------------
    /// Decode from interrupt / callback
    /// -----------------------------------------
    if (gRX_ok)
    {
        gRX_ok = false;

        SetState(kMsgReceived);
    }
    else if (gTX_ok)
    {
        gTX_ok = false;
        //GPIO_PinOutClear(DEBUG_PORT, DEBUG_PIN_TX);
        //gTX_tab[TAB_POS_TX_OK]++;
        gTX_tab[TAB_POS_TX_OK] = gCB_tab[RAIL_EVENT_TX_PACKET_SENT_SHIFT];
        // Increment counter and prepare new data
        gTX_counter.u32++;    // pas de test overflow
        prepare_packet_to_tx();

        SetState(kMsgSent);
    }
    else if (gRX_error)
    {
        gRX_error = false;
        //gRX_tab[me][TAB_POS_RX_ERR]++;
        gRX_tab[me][TAB_POS_RX_ERR] = gCB_tab[RAIL_EVENT_RX_PACKET_ABORTED_SHIFT] +
                                                          gCB_tab[RAIL_EVENT_RX_FRAME_ERROR_SHIFT] +
                                                          gCB_tab[RAIL_EVENT_RX_FIFO_OVERFLOW_SHIFT] +
                                                          gCB_tab[RAIL_EVENT_RX_SCHEDULED_RX_MISSED_SHIFT];

        SetState(kErrorRx);
    }
    else if (gTX_error)
    {
        gTX_error = false;
        //gTX_tab[TAB_POS_TX_ERR]++;
        gTX_tab[TAB_POS_TX_ERR] = gCB_tab[RAIL_EVENT_TX_ABORTED_SHIFT] +
                                  gCB_tab[RAIL_EVENT_TX_BLOCKED_SHIFT] +
                                  gCB_tab[RAIL_EVENT_TX_UNDERFLOW_SHIFT] +
                                  gCB_tab[RAIL_EVENT_TX_CHANNEL_BUSY_SHIFT] +
                                  gCB_tab[RAIL_EVENT_TX_SCHEDULED_TX_MISSED_SHIFT];

        prepare_packet_to_tx();

        SetState(kErrorTx);
    }
    else if (gSyncTimerDone)
    {
        gSyncTimerDone = false;

        //SetState(kSendSync);
        SetState(kSendingMsg);            // instead kSendSync because StarTX called directly in the timer callback
    }
    else if (gCAL_error)
    {
        gCAL_error = false;
        gCAL_tab[TAB_POS_CAL_ERR]++;

        SetState(kErrorCal);
    }
    else if (gBtnPressed)
    {
        gBtnPressed = false;

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
        // State after POR

        // Set node address
        gTX_fifo.fifo[kAddr] = gDeviceCfgAddr->internalAddr;
        me = gDeviceCfgAddr->posTab;

        // TX fifo ready
        prepare_packet_to_tx();

        app_log_info("\nInfo Ready ...\n");

        // Wait event
        SetState(kIdle);
        break;

    case kStart:
        // TX started by pressing BTN0
        SetState(kSendSync);

        app_log_info("\nInfo Start process ...\n");

        sl_led_turn_on(&sl_led_led1);
        StartTimerStat();
        gTX_counter_old = gTX_counter.u32;
        break;

    case kIdle:
        // Wait event (until pressing BTN0 on Master)
        break;

        // -----------------------------------------
        // RX
        // -----------------------------------------
    case kWaitMsg:
        // Wait receiving message
        break;

    case kMsgReceived:
        // Message received
        SetState(kWaitMsg);    // Auto transition to RX after successfull receive

        // For oscillo debug purposes
        GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_RX);

        // Extract received data from buffer
        DecodeReceivedMsg();
        break;

    case kErrorRx:
        // Error on RX
        SetState(kWaitMsg);    // Auto transition to RX after unsuccessfull receive

        // For oscillo debug purposes
        GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_RX);

        // Abnormal event
        PrintError(gErrorCode, "Error RX");
        break;

    case kListen:
        radio_state = RAIL_GetRadioState(gRailHandle);

        if ((radio_state != RAIL_RF_STATE_TX_ACTIVE) && (radio_state != RAIL_RF_STATE_RX_ACTIVE))
        {
            SetState(kWaitMsg);
            StartReceive();
        }
        else
        {
            SetState(kListen);  // Wait until transition to TX ready
        }
        break;

        // -----------------------------------------
        // TX
        // -----------------------------------------
    case kSendingMsg:
        // Wait until message sent
        break;

    case kMsgSent:
        SetState(kWaitMsg);    // Auto transition to RX after successfull transmit

        // For oscillo debug purposes
        GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_RX);

        DisplaySentMsg();
        // Indicate TX in progress on LED1
        sl_led_toggle(&sl_led_led1);
        break;

    case kErrorTx:
        SetState(kSendSync);            // Force to retransmit

        PrintError(gErrorCode, "Error TX");
        break;

    case kSendSync:

        radio_state = RAIL_GetRadioState(gRailHandle);

        if ((radio_state != RAIL_RF_STATE_TX_ACTIVE) && (radio_state != RAIL_RF_STATE_RX_ACTIVE))
        {
            SetState(kSendingMsg);
            StartTransmit();
        }
        else
        {
            SetState(kSendSync);  // Wait until transition ready
        }
        break;

        // -----------------------------------------
        // CAL
        // -----------------------------------------
    case kErrorCal:
#if (qPrintErrorsL1)
        if (gErrorCode != gPrevErrorCode)
        {
            app_log_error("Error CAL (0x%llX / %d)\n", gErrorCode, gCalibrationStatus);
            gPrevErrorCode = gErrorCode;
        }
#endif  // qPrintErrorsL1
        break;

    default:
        app_assert(false, "Unknown state (%d)\n", gProtocolState);
        break;
    }

    /// -----------------------------------------
    /// Statistics
    /// -----------------------------------------
    if (gStatTimerDone || gPrintStat || gStatReq)		// On timeout delay || Button pressed || CLI command
    {
        gPauseCycleReq = true;      // Request to print stat

        if (gPauseCycleConf)        // Permission to print stat
        {
            gStatTimerDone = false;
            gPrintStat = false;
            gStatReq = false;
            gCountPrintStat++;

            StopRadio();

            DisplayStat();

            RestartRadio();

            gTX_counter_old = gTX_counter.u32;
            memcpy((uint32_t *)gRX_counter_old, (uint32_t *)gRX_counter, MAX_NODE);

            gPauseCycleReq = false;
            gPauseCycleConf = false;

            SetState(kStart);       // Restart the cycle

            gElapsedTime = RAIL_GetTime();
            gOldElapsedTime = gElapsedTime;
        }
    }
}

