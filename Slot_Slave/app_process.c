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
#include "sl_udelay.h"              // Active delay
#include "sl_simple_button_instances.h"
// Button functions
#include "sl_flex_packet_asm.h"     // Flex packet

// Customized driver
#include "sl_pwm_instances.h"       // PWM functions
#include "common_custom_sl_pwm.h"   // Additional method for sl_pwm driver

// Specific to LCD display
//#include "dmd.h"                  // LCD driver
//#include "glib.h"                 // Graphics lib

// User components
// ---------------
#include "app_process.h"            // Main app
#include "common_debug.h"           // Debug functions
#include "common_mbox.h"            // Global var container
#include "common_stat.h"            // Statistics functions
#include "common_iadc.h"            // ADC functions
#include "common_tmp126_spi.h"      // Temp via SPI

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
    kMsgSent,
    kErrorTx,
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
    kAddr = 0,
    kMsgType = 1,
    kCounter0 = 2,
    kCounter1 = 3,
    kCounter2 = 4,
    kCounter3 = 5
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
//static RAIL_MultiTimer_t gSyncTimeOutTimer;

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

/// Flags to update stat print statistics
volatile bool gStatReq = false;					// Flag, indicating a request to print statistics (button was pressed / CLI statistics request has occurred)

/// Data content (payload)
/// Address
/// --
/// Message type
TypeMsgEnum gRX_msg_type = kInvalidMsg;
/// TX and RX counters
volatile union32_t gRX_counter[MAX_NODE] = { 0UL };
volatile union32_t gTX_counter = { .u32 = 0UL };
uint32_t gRX_counter_prev[MAX_NODE] = { 0UL };

/// Various flags
volatile bool gStartProcess = false;			// Flag, indicating a start process request (button was pressed / CLI start request has occurred)
volatile bool gResetProcess = false;            // Flag, indicating a reset process request (CLI reset request has occurred)
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
            // For debug on oscillo purposes --> check delta between all slave
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

    // TODO To remove and disable RX_TX_SCHEDULED_RX_TX_STARTE events (if not additional code is required)
    // Scheduled transmission is starting
    if (events & RAIL_EVENT_SCHEDULED_TX_STARTED)
    {
        // For oscillo debug purposes
        DEBUG_PIN_TX_SET;
    }

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

///******************************************************************************
// * Timer callback, called if any timer time out.
// *****************************************************************************/
//void timer_callback(RAIL_MultiTimer_t *tmr, RAIL_Time_t expectedTimeOfEvent, void *cbArg)
//{
//    (void) expectedTimeOfEvent;    // To avoid warnings
//    (void) cbArg;                  // To avoid warnings
//
//    if (tmr == &gSyncTimeOutTimer) // used to test if Master is still alive
//    {
//        gSYNC_timeout = true;
//    }
//}

/******************************************************************************
 * Timer callback, called if timer time out.
 *****************************************************************************/
void timer_callback(RAIL_Handle_t cbArg)
{
    (void) cbArg;                  // To avoid warnings

    gSYNC_timeout = true;
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
        TagTimeDelta4_3 = TagTime4 - TagTime1;
#endif  // qPrintStatTiming
        if (gElapsedTime < gOldElapsedTime)
        {
            uint64_t tmp = gElapsedTime + UINT32_MAX - gOldElapsedTime;
            gTotalElapsedTime += tmp;
        }
        else
            gTotalElapsedTime += (gElapsedTime - gOldElapsedTime);

        DEBUG_PIN_STAT_REQ_RESET;

        gCountPrintStat++;

//        bool status = RAIL_CancelMultiTimer(&gSyncTimeOutTimer);
//        PrintStatus((status == false), "Warning RAIL_CancelMultiTimer SYNC TIMEOUT");
        RAIL_CancelTimer(gRailHandle);

        StopRadio();

        CalcStat();

        RestartRadio();

        gPauseCycleReq = false;
        gPauseCycleConf = false;

#if (qPrintStatTiming)
        app_log_info("\nDEBUG timing statistics\n");
        app_log_info("-----------------------\n");

        app_log_info("Time from CB RX    : %d us\n", TagTimeDelta_CB_RX);
        app_log_info("Stat req period    : %d us\n", TagTimeDelta1);

        app_log_info("Stat req -> print  : %d us\n", TagTimeDelta4_3);
        app_log_info("Print out time     : %d us\n", TagTimeDelta5_4);

        app_log_info("Restart period     : %d us (delta with stat req: %d us)\n", TagTimeDelta3, TagTimeDelta3-TagTimeDelta1);
        app_log_info("End stat->restart  : %d us\n", TagTimeDelta3_2);
        app_log_info("(Time from CB RX)  : %d us\n", TagTimeDelta_CB_TX);

        app_log_info("CB RX req->restart : %d us\n", TagTimeDelta_CB_RX+TagTimeDelta4_3+TagTimeDelta5_4+TagTimeDelta3_2);

        TagTime5 = RAIL_GetTime();
        TagTimeDelta5_4 = TagTime5 - TagTime4;
#endif  // qPrintStatTiming
    }

    return !gPauseCycleReq;

#endif  // qComputeStat
}

/******************************************************************************
 * StartTimerSyncTO : start sync cycle timeout timer
 *****************************************************************************/
static __INLINE void StartTimerSyncTO(void)
{
//    RAIL_Status_t status = RAIL_SetMultiTimer(&gSyncTimeOutTimer, gSyncTimeOut, RAIL_TIME_DELAY, &timer_callback, NULL);
//    PrintStatus(status, "Warning RAIL_SetMultiTimer SYNC TIMEOUT");
    RAIL_Status_t status = RAIL_SetTimer(gRailHandle, gSyncTimeOut, RAIL_TIME_DELAY, &timer_callback);
    PrintStatus(status, "Warning RAIL_SetTimer (SYNC TIMEOUT)");
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
    DEBUG_PIN_RX_SET;

    // Start RX
    RAIL_Status_t status = RAIL_StartRx(gRailHandle, CHANNEL, NULL);
    PrintStatus(status, "Warning RAIL_StartRx");
}

/******************************************************************************
 * DecodeReceivedMsg : decode received data
 *****************************************************************************/
#if (RSSI_LQI_MES)
static uint32_t count_packet = 0U;
static int64_t sum_rssi = 0;
static uint64_t sum_lqi = 0U;
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
#endif  // RSSI_LQI_MES

            if (packet_info.packetStatus == RAIL_RX_PACKET_READY_SUCCESS)
            {
                // In debug mode, print received raw data
                DisplayReceivedMsg(start_of_packet, packet_size);

                ret = true;

                gRX_msg_type = start_of_packet[kMsgType];

                if (gRX_msg_type == kDataMsg)
                {
                    if (!gStartProcess)             // One shot process started
                        gStartProcess = true;

                    if (!gRX_first)                 // First time and after each stat print out
                    {
                        DEBUG_PIN_STAT_REQ_SET;

                        gRX_first = true;

                        gElapsedTime = RAIL_GetTime();
                        gOldElapsedTime = gElapsedTime;
#if (qPrintStatTiming)
                        TagTime3Prev = TagTime3;
                        TagTime3 = gElapsedTime;
                        TagTimeDelta3 = TagTime3 - TagTime3Prev;
                        TagTimeDelta3_2 = TagTime3 - TagTime5;
                        TagTimeDelta_CB_TX = TagTime3 - TagTime_CB_RX;
#endif  // qPrintStatTiming
                        DEBUG_PIN_STAT_REQ_RESET;
                    }

                    // Backup previous data
                    gRX_counter_prev[me] = gRX_counter[me].u32;
                    // Extract received data
                    //gRX_counter[me] = (uint32_t) ((start_of_packet[kCounter0] << 0) + (start_of_packet[kCounter1] << 8) + (start_of_packet[kCounter2] << 16) + (start_of_packet[kCounter3] << 24))
                    gRX_counter[me].u8[0] = start_of_packet[kCounter0];
                    gRX_counter[me].u8[1] = start_of_packet[kCounter1];
                    gRX_counter[me].u8[2] = start_of_packet[kCounter2];
                    gRX_counter[me].u8[3] = start_of_packet[kCounter3];

                    uint32_t delta = gRX_counter[me].u32 - gRX_counter_prev[me];
                    if (delta != 1)
                    {
                        gRX_tab[me][TAB_POS_RX_GAP]++;
                        gRX_tab[me][TAB_POS_RX_GAP_MAX] = (delta > gRX_tab[me][TAB_POS_RX_GAP_MAX] ? delta : gRX_tab[me][TAB_POS_RX_GAP_MAX]);
                    }
                    else
                        gRX_tab[me][TAB_POS_RX_OK]++;

#if (RSSI_LQI_MES)
                    status = RAIL_GetRxPacketDetailsAlt(gRailHandle, rx_packet_handle, &packet_info_detail);
                    PrintStatus(status, "Warning RAIL_GetRxPacketDetails");

                    // Count received packet
                    count_packet++;
                    if (count_packet == UINT32_MAX)
                    {
                        count_packet = 1U;
                        sum_rssi = 0;
                        sum_lqi = 0U;
                    }
                    // Get quality information: rssi and lqi
                    rssi = packet_info_detail.rssi;
                    sum_rssi += rssi;
                    gRX_tab[me][TAB_POS_RX_RSSI_MOY] = (int8_t) (sum_rssi / count_packet);
                    gRX_tab[me][TAB_POS_RX_RSSI_MIN] = (rssi < (int8_t) gRX_tab[me][TAB_POS_RX_RSSI_MIN] ? rssi : (int8_t) gRX_tab[me][TAB_POS_RX_RSSI_MIN]);
                    gRX_tab[me][TAB_POS_RX_RSSI_MAX] = (rssi > (int8_t) gRX_tab[me][TAB_POS_RX_RSSI_MAX] ? rssi : (int8_t) gRX_tab[me][TAB_POS_RX_RSSI_MAX]);
                    lqi = packet_info_detail.lqi;
                    sum_lqi += lqi;
                    gRX_tab[me][TAB_POS_RX_LQI_MOY] = (uint8_t) (sum_lqi / count_packet);
                    gRX_tab[me][TAB_POS_RX_LQI_MIN] = (lqi < (uint8_t) gRX_tab[me][TAB_POS_RX_LQI_MIN] ? lqi : gRX_tab[me][TAB_POS_RX_LQI_MIN]);
                    gRX_tab[me][TAB_POS_RX_LQI_MAX] = (lqi > (uint8_t) gRX_tab[me][TAB_POS_RX_LQI_MAX] ? lqi : gRX_tab[me][TAB_POS_RX_LQI_MAX]);
#endif  // RSSI_LQI_MES
                }
            }
            else if (packet_info.packetStatus == RAIL_RX_PACKET_READY_CRC_ERROR)
            {
                gRX_tab[me][TAB_POS_RX_CRC_ERR]++;
            }
            else    // Normally, no more cases to treat!!
            {
                PrintStatus(packet_info.packetStatus, "Unknown packet status");
                app_assert(false, "Unknown packet status (%d)\n", packet_info.packetStatus);
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

/******************************************************************************
 * Analog_read : AD conversions
 *****************************************************************************/
static __INLINE void Analog_read(void)
{
    bool status = common_getIADCdata();
    PrintStatus((status == false), "Warning common_getIADCdata timeout");
}

/******************************************************************************
 * I2C_temp_read : Temp acquisition via SPI
 *****************************************************************************/
static __INLINE void SPI_temp_read(void)
{
    // TODO BEGIN TEST PURPOSES
    sl_udelay_wait(60);       // Simulate SPI transfer at 1 Mbps for TMPxxx
    gMBoxTempCell = 25000;
    // TODO END TEST PURPOSES
}

/******************************************************************************
 * DoAllAcq : Process all acquisitions
 *****************************************************************************/
static __INLINE void DoAllAcq(void)
{
    DEBUG_PIN_ACQ_SET;

    // Start AD conversion
    common_startIADC();

    // Read temp via SPI
    // SPI_temp_read();
    gMBoxTempCell = spi_tmp126_getTemp(device0);

    // Get results of ADC
    Analog_read();

    // Increment counter
    gTX_counter.u32++;    // pas de test overflow

    DEBUG_PIN_ACQ_RESET;
}

// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------

/******************************************************************************
 * Application state machine, called infinitely
 *****************************************************************************/
//static uint16_t pwm_count = 1;

void app_process_action(void)
{
    RAIL_Status_t status = RAIL_STATUS_NO_ERROR;

    /// -----------------------------------------
    /// Decode from interrupt / callback
    /// -----------------------------------------

    /// Msg received successfully
    /// -------------------------
    if (gRX_ok)
    {
        gRX_ok = false;

        SetState(kSyncReceived);
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
    /// Master Sync timeout check
    /// -------------------------
    else if (gSYNC_timeout)
    {
        gSYNC_timeout = false;

        SetState(kSyncLost);
    }
    /// Button pressed: start process then print stat
    /// ---------------------------------------------
    else if (gBtnPressed)
    {
        gBtnPressed = false;

        SetState(kBtnPressed);
    }
    /// Reset process via CLI or via message
    /// ------------------------------
    else if (gResetProcess)
    {
        gResetProcess = false;

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
    case kInit:
        // State after POR

        // Set node address
        gTX_fifo.fifo[kAddr] = gDeviceCfgAddr->internalAddr;
        // Slave send (currently) only data frame
        gTX_fifo.fifo[kMsgType] = kDataMsg;
        // TX fifo ready (in case of auto transition RX->TX is activated!)
        prepare_packet_to_tx();

        // Pos tab for various counters (callback, RX/TX_counters)
        me = gDeviceCfgAddr->posTab;

        // To track first sync received from master
        gRX_first = false;

        // Print out
        if (!gStartProcess)
            PrintInfo("\nInfo Ready\n");
        else
            PrintInfo("\nInfo Running ...\n");

        // Listen from Master
        SetState(kListen);
        break;

        // -----------------------------------------
        // Idle
        // -----------------------------------------
    case kIdle:
        // Wait sync or stat commands from master
        break;

        // -----------------------------------------
        // RX
        // -----------------------------------------

        /// Switch to RX
        /// ------------
    case kListen:
        // Put slave in receiving mode
        StartReceive();
        // Wait command from master
        SetState(kIdle);
        break;

        /// RX successfully
        /// ---------------
    case kSyncReceived:
        // Extract received data from buffer
        if (DecodeReceivedMsg())
        {
            //        // Do all acquisition
            //        DoAllAcq();
            //        prepare_packet_to_tx();

            // Actions according command type
            switch (gRX_msg_type)
            {
            /// Normal mode (data)
            case kDataMsg:
                DEBUG_PIN_SCHEDULE_SET;
                if (gTimeSlot > 0UL)
                {
                    // Schedule next TX
                    status = RAIL_StartScheduledTx(gRailHandle, CHANNEL, RAIL_TX_OPTIONS_DEFAULT, &gRailScheduleCfgTX, NULL);
                    PrintStatus(status, "Warning RAIL_StartScheduledTx");

                    // Do all acquisitions and update TX buffer
                    DoAllAcq();
                    prepare_packet_to_tx();     // TODO To check if buffer still can be changed after calling RAIL_StartScheduledTx ?!?
                }
                else
                {
                    // Do all acquisitions and update TX buffer
                    DoAllAcq();
                    prepare_packet_to_tx();

                    DEBUG_PIN_TX_SET;
                    // Start immediately after acquisition
                    status = RAIL_StartTx(gRailHandle, CHANNEL, RAIL_TX_OPTIONS_DEFAULT, NULL);
                    PrintStatus(status, "Warning RAIL_StartTx");
                }

                StartTimerSyncTO();             // In order to test if Master is still alive and sending Sync periodically
                DEBUG_PIN_SCHEDULE_RESET;

                SetState(kIdle);
                break;

                /// Stat print out command
            case kStatMsg:
#if (qPrintStatTiming)
                TagTime1Prev = TagTime1;
                TagTime1 = RAIL_GetTime();
                TagTimeDelta1 = TagTime1-TagTime1Prev;
                TagTimeDelta_CB_RX = TagTime1-TagTime_CB_RX;
#endif  // qPrintStatTiming
                gPauseCycleReq = true;
                gPauseCycleConf = true;         // Cycle ended, permit a print stat

                SetState(kStatistics);          // Print out statistics
                break;

            case kResetMsg:
                gResetProcess = true;           // Master requesta a chip reset
                SetState(kIdle);
                break;

                /// No yet implemented commands from Master
            case kInvalidMsg:
            case kDiagMsg:
            case kServiceMsg:
            case kSetupMsg:
                /// Shall not happen
            default:
                app_assert(false, "Unknown msg type (%d)\n", gRX_msg_type);
                break;
            }
        }
        else
        {
            SetState(kIdle);
        }
        break;

        /// RX error
        /// --------
    case kErrorRx:
        if (gRX_msg_type == kDataMsg)       // Don't take in account the error generated by other command than normal mode (data)
        {
            gRX_tab[me][TAB_POS_RX_ERR] = gCB_tab[RAIL_EVENT_RX_PACKET_ABORTED_SHIFT] + gCB_tab[RAIL_EVENT_RX_FRAME_ERROR_SHIFT] + gCB_tab[RAIL_EVENT_RX_FIFO_OVERFLOW_SHIFT]
                    + gCB_tab[RAIL_EVENT_RX_SCHEDULED_RX_MISSED_SHIFT];

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
        if (gRX_msg_type == kDataMsg)       // Don't take in account the error generated by other command than normal mode (data)
        {
            gTX_tab[TAB_POS_TX_OK] = gCB_tab[RAIL_EVENT_TX_PACKET_SENT_SHIFT];

            // Print raw data sent for debug purposes
            DisplaySentMsg();

            // TODO BEGIN TEST PURPOSES
//            sl_pwm_set_duty_cycle_step(&sl_pwm_pwm0, pwm_count);
//            sl_pwm_set_duty_cycle_step(&sl_pwm_pwm1, pwm_count);
//
//            if (++pwm_count > sl_pwm_get_max_duty_cycle_step(&sl_pwm_pwm0))
//                pwm_count = 0;
            // TODO END TEST PURPOSES
        }

        // Auto transition to RX after successfull transmit
        // For oscillo debug purposes
        DEBUG_PIN_RX_SET;

        SetState(kIdle);
        break;

        /// TX error
        /// --------
    case kErrorTx:
        if (gRX_msg_type == kDataMsg)       // Don't take in account the error generated by other command than normal mode (data)
        {
            gTX_tab[TAB_POS_TX_ERR] = gCB_tab[RAIL_EVENT_TX_ABORTED_SHIFT] + gCB_tab[RAIL_EVENT_TX_BLOCKED_SHIFT] + gCB_tab[RAIL_EVENT_TX_UNDERFLOW_SHIFT] + gCB_tab[RAIL_EVENT_TX_CHANNEL_BUSY_SHIFT]
                    + gCB_tab[RAIL_EVENT_TX_SCHEDULED_TX_MISSED_SHIFT];

            PrintError(gErrorCode, "Error TX");
        }

        // Same data but tell RAIL that FIFO is set but no change in the previous data
        prepare_packet_to_tx();

        // Auto transition to RX after unsuccessfull transmit
        // For oscillo debug purposes
        DEBUG_PIN_RX_SET;

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

        gStartProcess = false;

        PrintInfo("\nWarning Master sync lost!\n");

        SetState(kInit);
        break;

        // -----------------------------------------
        // IHM
        // -----------------------------------------

        /// BTN0 button
        /// -----------
    case kBtnPressed:
        if (!gStartProcess)
        {
            SetState(gProtocolPrevState);   // Do nothing as process not yet started; process is started with the first sync command RX from Master
        }
        else
        {
            SetState(kStatistics);          // Start stat print out pcrocess --> no more possible (only stat sync on Master (timer or btn pressed on Master)
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
            SetState(kInit);                // At the end of stat print, restart the cycle
        }
        else
            // Start stat print out process (wait next sync period to synchronize the process)
            SetState(kStatistics);
        //
        break;

        /// Shall not happen
    default:
        app_assert(false, "Unknown state (%d)\n", gProtocolState);
        break;
    }
}

