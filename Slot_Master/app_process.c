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
#include "printf.h"                 // Tiny printf
#include "em_chip.h"                // Chip functions
#include "sl_simple_button_instances.h"
                                    // Button functions
#include "sl_flex_packet_asm.h"     // Flex packet
#include "sl_udelay.h"              // Active delay

// User components
// ---------------
#include "app_init.h"               // Initialize functions
#include "app_process.h"            // Main app
#include "common_debug.h"           // Debug functions
#include "common_stat.h"            // Statistics functions


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
    //  ------------
    //  Idle
    //  ------------
    kIdle,
    //  ------------
    //	RX
    //  ------------
    kMsgReceived,
    kErrorRx,
    //  ------------
    //	TX
    //  ------------
    kMsgSent,
    kErrorTx,
    //  ------------
    //	Calibration
    //  ------------
    kCalReq,
    //  ------------
    //  Statistics
    //  ------------
    kStatistics,
    //  ------------
    //  Timers
    //  ------------
    kSyncReq,
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
    kCmd        = 1,
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
static volatile bool gCAL_req = false;
static volatile bool gSYNC_timeout = false;

/// Flags to update start print statistics
static volatile bool gStatTimerDone = false;	// Timeout of the delay for printing stat
volatile bool gStatReq = false;					// Flag, indicating a request to print statistics (button was pressed / CLI statistics request has occurred)

/// Data content (payload)
/// Address
/// --
/// Message type
TypeMsgEnum gTX_msg_type = kInvalidMsg;
/// TX and RX counters
volatile union32_t gRX_counter[MAX_NODE] = { 0UL };
volatile union32_t gTX_counter = { .u32 = 0UL };
uint32_t gRX_counter_prev[MAX_NODE] = { 0UL };

/// Various flags
volatile bool gStartProcess = false;			// Flag, indicating a start process request (button was pressed / CLI start request has occurred)
volatile bool gResetProcess = false;            // Flag, indicating a reset process request (CLI reset request has occurred)
static bool gTX_first = false;                  // Indicate first TX packet transmitted to Slave to start statistics
volatile bool gBtnPressed = false;   			// Button pressed, start process
static bool gPauseCycleConf = false;            // Flag to indicate a end of cycle of transmission with the slaves in order to avoid side effect of the print statistics
static bool gPauseCycleReq = false;             // Flag to indicate that a print stat is pending and request to pause the cycle at the next occasion (end of cycle)

// Various var
static uint8_t me;                                              // Position in the config file (for address, time slot, ...)
/// Value, indicating print stat delay on CLI
extern volatile RAIL_Time_t gStatDelay;

#if (qPrintStatTiming)
// Debug
static RAIL_Time_t TagTime_CB_RX, TagTime_CB_TX;
static RAIL_Time_t TagTimeDelta_CB_RX, TagTimeDelta_CB_TX;
static RAIL_Time_t TagTime1, TagTime1Prev, TagTime2, TagTime2Prev, TagTime3, TagTime3Prev, TagTime4, TagTime4Prev, TagTime5, TagTime5Prev;
static RAIL_Time_t TagTimeDelta1, TagTimeDelta2, TagTimeDelta3, TagTimeDelta4, TagTimeDelta5;
static RAIL_Time_t TagTimeDelta2_1, TagTimeDelta3_2, TagTimeDelta4_3, TagTimeDelta5_4;
#endif  // qPrintStatTiming

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

    DEBUG_PIN_CB_SET;

    DecodeEvents(&events);  // Count events for debug and statistics

    // Handle RX events
    if (events & RAIL_EVENTS_RX_COMPLETION)
    {

        if (events & RAIL_EVENT_RX_PACKET_RECEIVED)
        {
#if (qPrintStatTiming)
            TagTime_CB_RX = RAIL_GetTime();
#endif  // qPrintStatTiming
            DEBUG_PIN_RX_RESET;
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
#if (qPrintStatTiming)
            TagTime_CB_TX = RAIL_GetTime();
#endif  // qPrintStatTiming
            // Handle next step
            DEBUG_PIN_TX_RESET;

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

//    if (events & RAIL_EVENT_TX_STARTED)
//    {
//        // For oscillo debug purposes
//        DEBUG_PIN_TX_SET;
//    }

    // Perform all calibrations when needed
    if (events & RAIL_EVENT_CAL_NEEDED)
    {
        gCAL_req = true;
    }

    DEBUG_PIN_CB_RESET;
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

    if (tmr == &gStatDelayTimer) // used to print stat periodically
    {
        gStatTimerDone = true;
    }
    else if (tmr == &gSyncPeriodTimer) // used with standard StartTx when RAIL_StartScheduledTx isn't used
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
#if (qComputeStat)
    gPauseCycleReq = true;      // Request to print stat

    if (gPauseCycleConf)        // Permission to print stat
    {
        DEBUG_PIN_STAT_REQ_SET;

        gOldElapsedTime = gElapsedTime;
        gElapsedTime = RAIL_GetTime();
#if (qPrintStatTiming)
        TagTime4 = gElapsedTime;
        TagTimeDelta4_3 = TagTime4 - TagTime3;
#endif  // qPrintStatTiming
        if (gElapsedTime < gOldElapsedTime)
        {
            uint64_t tmp = gElapsedTime + UINT32_MAX - gOldElapsedTime;
            gTotalElapsedTime += (uint32_t) tmp;
        }
        else
            gTotalElapsedTime += (gElapsedTime - gOldElapsedTime);

        DEBUG_PIN_STAT_REQ_RESET;

        gCountPrintStat++;

        bool status = RAIL_CancelMultiTimer(&gSyncPeriodTimer);
        PrintStatus((status == false), "Warning RAIL_CancelMultiTimer SYNC PERIOD");

        StopRadio();

        CalcStat();

        RestartRadio();

        gPauseCycleReq = false;
        gPauseCycleConf = false;

#if (qPrintStatTiming)
        app_log_info("\nDEBUG timing statistics\n");
        app_log_info("-----------------------\n");

        app_log_info("Timer period       : %d us\n", TagTimeDelta1);
        app_log_info("Time from CB RX    : %d us\n", TagTimeDelta_CB_RX);
        app_log_info("Timer->req start   : %d us\n", TagTimeDelta2_1);
        app_log_info("Req period         : %d us (delta with timer period: %d us)\n", TagTimeDelta2, TagTimeDelta2-TagTimeDelta1);
        app_log_info("Time from CB TX    : %d us\n", TagTimeDelta_CB_TX);
        app_log_info("Req start->TX ok   : %d us\n", TagTimeDelta3_2);
        app_log_info("TX ok -> print     : %d us\n", TagTimeDelta4_3);
        app_log_info("Print out time     : %d us\n", TagTimeDelta5_4);

        TagTime5 = RAIL_GetTime();
        TagTimeDelta5_4 = TagTime5 - TagTime4;
#endif  // qPrintStatTiming
        sl_udelay_wait(5000);       // As Master is starting first his print and print volume can vary between Master and Slave,
                                    // wait 5 ms to allow time to Slaves to be ready after their stat print out

    }

    return !gPauseCycleReq;

#endif  // qComputeStat
}

/******************************************************************************
 * StartTimerStat : start timer to compute statistics
 *****************************************************************************/
static __INLINE void StartTimerStat(void)
{
    RAIL_Status_t status = RAIL_SetMultiTimer(&gStatDelayTimer, gStatDelay, RAIL_TIME_DELAY, &timer_callback, NULL);
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
    gTX_fifo.fifo[kCmd]      = gTX_msg_type;
    gTX_fifo.fifo[kCounter0] = gTX_counter.u8[0];
    gTX_fifo.fifo[kCounter1] = gTX_counter.u8[1];
    gTX_fifo.fifo[kCounter2] = gTX_counter.u8[2];
    gTX_fifo.fifo[kCounter3] = gTX_counter.u8[3];

    uint16_t allocated_tx_fifo_size = RAIL_SetTxFifo(gRailHandle, gTX_fifo.fifo, TX_PAYLOAD_LENGTH, RAIL_FIFO_TX_SIZE);
    app_assert(allocated_tx_fifo_size == RAIL_FIFO_TX_SIZE, "RAIL_SetTxFifo() failed to allocate a large enough fifo (%d bytes instead of %d bytes)\n", allocated_tx_fifo_size, RAIL_FIFO_TX_SIZE);
}

/******************************************************************************
 * StartTransmit : prepare buffer and change to TX mode
 *****************************************************************************/
static __INLINE void StartTransmit(void)
{
    // Initialize radio buffer
    prepare_packet_to_tx();

    // For oscillo debug purposes
    DEBUG_PIN_TX_SET;

    RAIL_Status_t status = RAIL_StartTx(gRailHandle, CHANNEL, RAIL_TX_OPTIONS_DEFAULT, NULL);
    PrintStatus(status, "Warning RAIL_StartTx");

    StartTimerSync();
}

/******************************************************************************
 * DecodeReceivedMsg : decode received data
 *****************************************************************************/
#if (RSSI_LQI_MES)
static uint32_t count_packet[MAX_NODE] = {0U};
static int64_t sum_rssi[MAX_NODE] = {0};
static uint64_t sum_lqi[MAX_NODE] = {0U};
#endif  // RSSI_LQI_MES

static __INLINE bool DecodeReceivedMsg(void)
{
    // RAIL Rx packet handles
    RAIL_RxPacketHandle_t rx_packet_handle;
    RAIL_RxPacketInfo_t packet_info;
    // Status indicator of the RAIL API calls
    RAIL_Status_t status = RAIL_STATUS_NO_ERROR;
#if (RSSI_LQI_MES)
    RAIL_RxPacketDetails_t packet_info_detail;
    int8_t rssi;
    uint8_t lqi;
#endif  // RSSI_LQI_MES
    bool ret = false;

    // Packet received:
    //  - Check whether RAIL_HoldRxPacket() was successful, i.e. packet handle is valid
    //  - Copy it to the application FIFO
    //  - Free up the radio FIFO
    //  - Return to app IDLE state (RAIL will automatically switch back to Rx radio state)

    rx_packet_handle = RAIL_GetRxPacketInfo(gRailHandle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);
    while (rx_packet_handle != RAIL_RX_PACKET_HANDLE_INVALID)
    {
        if (packet_info.packetStatus != RAIL_RX_PACKET_RECEIVING)
        {
            // Unpack and decode
            uint8_t *start_of_packet = 0;
            uint16_t packet_size = unpack_packet_from_rx(gRX_fifo, &packet_info, &start_of_packet);
#if (!RSSI_LQI_MES)
            status = RAIL_ReleaseRxPacket(gRailHandle, rx_packet_handle);
            PrintStatus(status, "Warning ReleaseRxPacket");
#endif  // !RSSI_LQI_MES

            if (packet_info.packetStatus == RAIL_RX_PACKET_READY_SUCCESS)
            {
                DisplayReceivedMsg(start_of_packet, packet_size);

                ret = true;

                uint8_t pos = gAddrToPos[start_of_packet[kAddr]];
                //uint8_t pos = start_of_packet[kAddr];
                TypeMsgEnum msg_type = start_of_packet[kCmd];

                if (msg_type == kDataMsg)
                {
                    // Backup previous data
                    gRX_counter_prev[pos] = gRX_counter[pos].u32;
                    // Extract received data
                    //gRX_counter[addr] = (uint32_t) ((start_of_packet[kCounter0] << 0) + (start_of_packet[kCounter1] << 8) + (start_of_packet[kCounter2] << 16) + (start_of_packet[kCounter3] << 24));
                    gRX_counter[pos].u8[0] = start_of_packet[kCounter0];
                    gRX_counter[pos].u8[1] = start_of_packet[kCounter1];
                    gRX_counter[pos].u8[2] = start_of_packet[kCounter2];
                    gRX_counter[pos].u8[3] = start_of_packet[kCounter3];

                    uint32_t delta = gRX_counter[pos].u32 - gRX_counter_prev[pos];
                    if (delta != 1)
                    {
                        gRX_tab[pos][TAB_POS_RX_GAP]++;
                        gRX_tab[pos][TAB_POS_RX_GAP_MAX] = (delta > gRX_tab[pos][TAB_POS_RX_GAP_MAX] ? delta : gRX_tab[pos][TAB_POS_RX_GAP_MAX]);
                    }
                    else
                    {
                        gRX_tab[pos][TAB_POS_RX_OK]++;
                    }

#if (RSSI_LQI_MES)
                    status = RAIL_GetRxPacketDetailsAlt(gRailHandle, rx_packet_handle, &packet_info_detail);
                    PrintStatus(status, "Warning RAIL_GetRxPacketDetails");

                    // i == 0: index = me (= calc for all slaves)
                    // i == 1: index = pos (= calc for the specific slave)
                    uint8_t who = me;
                    for (int i = 0; i < 2; i++)
                    {
                        if (i == 1)
                            who = pos;

                        // Count received packet
                        count_packet[who]++;

                        if (count_packet[who] == UINT32_MAX)
                        {
                            count_packet[who] = 1U;
                            sum_rssi[who] = 0;
                            sum_lqi[who] = 0U;
                        }
                        // Get quality information: rssi and lqi
                        rssi = packet_info_detail.rssi;
                        sum_rssi[who] += rssi;
                        gRX_tab[who][TAB_POS_RX_RSSI_MOY] = (int8_t)(sum_rssi[who]/count_packet[who]);
                        gRX_tab[who][TAB_POS_RX_RSSI_MIN] = (rssi < (int8_t)gRX_tab[who][TAB_POS_RX_RSSI_MIN] ? rssi : (int8_t)gRX_tab[who][TAB_POS_RX_RSSI_MIN]);
                        gRX_tab[who][TAB_POS_RX_RSSI_MAX] = (rssi > (int8_t)gRX_tab[who][TAB_POS_RX_RSSI_MAX] ? rssi : (int8_t)gRX_tab[who][TAB_POS_RX_RSSI_MAX]);
                        lqi = packet_info_detail.lqi;
                        sum_lqi[who] += lqi;
                        gRX_tab[who][TAB_POS_RX_LQI_MOY] = (uint8_t)(sum_lqi[who]/count_packet[who]);
                        gRX_tab[who][TAB_POS_RX_LQI_MIN] = (lqi < (uint8_t)gRX_tab[who][TAB_POS_RX_LQI_MIN] ? lqi : gRX_tab[who][TAB_POS_RX_LQI_MIN]);
                        gRX_tab[who][TAB_POS_RX_LQI_MAX] = (lqi > (uint8_t)gRX_tab[who][TAB_POS_RX_LQI_MAX] ? lqi : gRX_tab[who][TAB_POS_RX_LQI_MAX]);
                    }

#endif  // RSSI_LQI_MES
                }
                // else if () --> currently, Slave don't handle other frame than data
            }
            else if (packet_info.packetStatus == RAIL_RX_PACKET_READY_CRC_ERROR)
            {
                gRX_tab[me][TAB_POS_RX_CRC_ERR]++;
            }

#if (RSSI_LQI_MES)
            // Release packet in radio buffer
            status = RAIL_ReleaseRxPacket(gRailHandle, rx_packet_handle);
            PrintStatus(status, "Warning ReleaseRxPacket");
#endif  // RSSI_LQI_MES

        }

       rx_packet_handle = RAIL_GetRxPacketInfo(gRailHandle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);
    }

    return ret;
}

// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------

/******************************************************************************
 * Application state machine, called infinitely
 *****************************************************************************/
void app_process_action(void)
{
    /// -----------------------------------------
    /// Decode from interrupt / callback
    /// -----------------------------------------

    /// Msg received successfully
    /// -------------------------
    if (gRX_ok)
    {
        gRX_ok = false;

        SetState(kMsgReceived);
    }
    /// Msg sent successfully
    /// ---------------------
    else if (gTX_ok)
    {
        gTX_ok = false;

        SetState(kMsgSent);
    }
    /// Error during RX (does not include additional check in a data level)
    /// -------------------------------------------------------------------
    else if (gRX_error)
    {
        gRX_error = false;

        SetState(kErrorRx);
    }
    /// Error during TX
    /// ---------------
    else if (gTX_error)
    {
        gTX_error = false;

        SetState(kErrorTx);
    }
    /// Calibration request
    /// -------------------
    else if (gCAL_req)
    {
        gCAL_req = false;

        SetState(kCalReq);
    }
    /// Sync period
    /// -----------
    else if (gSYNC_timeout)
    {
        gSYNC_timeout = false;

        SetState(kSyncReq);
    }
    /// Periodic stat print out request
    /// -------------------------------
    else if (gStatTimerDone)
    {
#if (qPrintStatTiming)
        TagTime1Prev = TagTime1;
        TagTime1 = RAIL_GetTime();
        TagTimeDelta1 = TagTime1-TagTime1Prev;
#endif  // qPrintStatTiming
        gStatTimerDone = false;

        SetState(kStatistics);
    }
    /// Button pressed: start process then print stat
    /// ---------------------------------------------
    else if (gBtnPressed)
    {
        gBtnPressed = false;

        SetState(kBtnPressed);
    }
    /// Reset process via CLI
    /// ------------------------------
    else if (gResetProcess)
    {
        gResetProcess = false;

        // Normal mode (data)
        gTX_msg_type = kResetMsg;
        // Send reset
        StartTransmit();

        sl_udelay_wait(500);

        // Send reset (2nd time to be "sure" all slaves resets ?!?
        StartTransmit();

        sl_udelay_wait(500);

        CHIP_Reset();
    }
    /// Stat print out request via CLI
    /// ------------------------------
    else if (gStatReq)
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

        /// POR
        /// ---
    case kInit:
        // State after POR

        // Set node address
        gTX_fifo.fifo[kAddr] = gDeviceCfgAddr->internalAddr;
        // Set default message type to invalid
        gTX_msg_type = kInvalidMsg;
        // Pos tab for various counters (callback, RX/TX_counters)
        me = gDeviceCfgAddr->posTab;

        // Ready to start process
        PrintInfo("\nInfo Press BTN0 to start process\n");

        // Wait event
        SetState(kIdle);
        break;

        /// Start or restart process
        /// ------------------------
    case kStart:
        // Start process
        PrintInfo("\nInfo Running ...\n");

        // Normal mode (data)
        gTX_msg_type = kDataMsg;

        // To track first sync transmitted to slaves
        gTX_first = false;

        // Increment counter (no overflow test!)
        gTX_counter.u32++;
#if (qPrintStatTiming)
        TagTime1Prev = RAIL_GetTime();
        TagTime2Prev = TagTime1Prev;
#endif  // qPrintStatTiming
        // Send sync
        StartTransmit();

        // Wait replies from slaves
        SetState(kIdle);
        break;

        // -----------------------------------------
        // Idle
        // -----------------------------------------
    case kIdle:
        // Wait BTN0 activation or slaves replies
        break;

        // -----------------------------------------
        // RX
        // -----------------------------------------

        /// RX successfully
        /// ---------------
    case kMsgReceived:
        if (gTX_msg_type == kDataMsg)       // Don't take in account the error generated by other command than normal mode (data)
        {
            // Extract received data from buffer
            if (DecodeReceivedMsg())
            {
                // Nothing additional specific to do than already treated in DecodeReceivedMsg
            }
            else
            {
                // Data received in the buffer was not treated (CRC error or partial frame)
            }
        }

        // Auto transition to RX after successfull receive
        // For oscillo debug purposes
        DEBUG_PIN_RX_SET;

        SetState(kIdle);
        break;

        /// RX error
        /// --------
    case kErrorRx:
        if (gTX_msg_type == kDataMsg)       // Don't take in account the error generated by other command than normal mode (data)
        {
            gRX_tab[me][TAB_POS_RX_ERR] = gCB_tab[RAIL_EVENT_RX_PACKET_ABORTED_SHIFT] +
                                          gCB_tab[RAIL_EVENT_RX_FRAME_ERROR_SHIFT] +
                                          gCB_tab[RAIL_EVENT_RX_FIFO_OVERFLOW_SHIFT] +
                                          gCB_tab[RAIL_EVENT_RX_SCHEDULED_RX_MISSED_SHIFT];

            // Abnormal event
            PrintError(gErrorCode, "Error RX");
        }

        // Auto transition to RX after unsuccessfull receive
        // For oscillo debug purposes
        DEBUG_PIN_RX_SET;

        SetState(kIdle);
        break;

        // -----------------------------------------
        // TX
        // -----------------------------------------

        /// TX successfully
        /// ---------------
    case kMsgSent:
        // Actions according command type
        switch (gTX_msg_type)
        {
            /// Normal mode (data)
        case kDataMsg:
            gTX_tab[TAB_POS_TX_OK] = gCB_tab[RAIL_EVENT_TX_PACKET_SENT_SHIFT];

            if (!gTX_first)
            {
                StartTimerStat();

                DEBUG_PIN_STAT_REQ_SET;

                gTX_first = true;

                gElapsedTime = RAIL_GetTime();
                gOldElapsedTime = gElapsedTime;

                DEBUG_PIN_STAT_REQ_RESET;
            }

            DisplaySentMsg();

            // Auto transition to RX after successfull transmit
            // For oscillo debug purposes
            DEBUG_PIN_RX_SET;

            SetState(kIdle);
            break;

            /// Stat print out command
        case kStatMsg:
#if (qPrintStatTiming)
            TagTime3 = RAIL_GetTime();
            TagTimeDelta3_2 = TagTime3 - TagTime2;
            TagTimeDelta_CB_TX = TagTime3 - TagTime_CB_TX;
#endif  // qPrintStatTiming
            gPauseCycleConf = true;         // Cycle ended, permit a print stat

            SetState(kStatistics);          // Print out statistics
            break;

            /// No yet implemented commands
        case kInvalidMsg:
        case kDiagMsg:
        case kServiceMsg:
        case kSetupMsg:
            /// Shall not happen
        default:
            app_assert(false, "Unknown msg type (%d)\n", gTX_msg_type);
            break;
        }
        break;

        /// TX error
        /// --------
    case kErrorTx:
        if (gTX_msg_type == kDataMsg)       // Don't take in account the error generated by other command than normal mode (data)
        {
            gTX_tab[TAB_POS_TX_ERR] = gCB_tab[RAIL_EVENT_TX_ABORTED_SHIFT] +
                                      gCB_tab[RAIL_EVENT_TX_BLOCKED_SHIFT] +
                                      gCB_tab[RAIL_EVENT_TX_UNDERFLOW_SHIFT] +
                                      gCB_tab[RAIL_EVENT_TX_CHANNEL_BUSY_SHIFT] +
                                      gCB_tab[RAIL_EVENT_TX_SCHEDULED_TX_MISSED_SHIFT];

            PrintError(gErrorCode, "Error TX");
        }

        StartTransmit();        // Force to retransmit

        SetState(kIdle);
        break;

        // -----------------------------------------
        // Calibration
        // -----------------------------------------
    case kCalReq:
        gCAL_tab[TAB_POS_CAL_REQ]++;

        RAIL_Status_t status = RAIL_Calibrate(gRailHandle, NULL, RAIL_CAL_ALL_PENDING);
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
    case kSyncReq:
        if (!gPauseCycleReq)                // If no pending request to print stat -> restart cycle with the slaves
        {
            // Increment counter (no overflow test!)
            gTX_counter.u32++;

            StartTransmit();                // Send sync cmd
            SetState(kIdle);                // and wait TX ok confirmation and then replies from slaves
        }
        else                                // There is a pending request to print stat
        {
#if (qPrintStatTiming)
            TagTime2Prev = TagTime2;
            TagTime2 = RAIL_GetTime();
            TagTimeDelta2 = TagTime2-TagTime2Prev;
            TagTimeDelta2_1 = TagTime2 - TagTime1;
            TagTimeDelta_CB_RX = TagTime2 - TagTime_CB_RX;
#endif  // qPrintStatTiming
            gTX_msg_type = kStatMsg;        // Send the stat cmd to the slaves in order they can print stat too
            StartTransmit();                // and wait until TX ok confirmation
            SetState(kIdle);
        }
        break;

        // -----------------------------------------
        // IHM and CLI
        // -----------------------------------------

        /// BTN0 button
        /// -----------
    case kBtnPressed:
        if (!gStartProcess)
        {
            gStartProcess = true;           // First button activation = start process

            SetState(kStart);
        }
        else                                // Following button activation = print out statistics
        {
            SetState(kStatistics);          // Start stat print out pcrocess
        }
        break;

        /// stat CLI
        /// --------
    case kStatReq:                          // CLI stat
        gStatReq = false;

        SetState(kStatistics);              // Start stat print out pcrocess
        break;

        // -----------------------------------------
        // Statistics
        // -----------------------------------------
    case kStatistics:
        // On timeout delay || Button pressed || CLI command
        if (PrintStatistics())
        {
            SetState(kStart);               // Restart the cycle
        }
        else                                // Start stat print out pcrocess (wait next sync period to synchronize the process)
            SetState(kStatistics);

        break;

        /// Shall not happen
    default:
        app_assert(false, "Unknown state (%d)\n", gProtocolState);
        break;
    }
}

