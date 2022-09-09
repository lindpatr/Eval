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
#include "app_stat.h"

// Specific to LCD display
#include "dmd.h"
#include "glib.h"
#include "printf.h"

// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------
/// Size of RAIL RX/TX FIFO
#define RAIL_FIFO_RX_SIZE (512U)          // in bytes
#define RAIL_FIFO_TX_SIZE (128U)          // in bytes // don't try below 128k --> assert
/// Transmit data length (fixed payload)
#define TX_PAYLOAD_LENGTH (6U)            // in bytes

// Period to print statistics
#define STAT_PERIOD_s (60U)             // in us
#define STAT_PERIOD_us (STAT_PERIOD_s * 1000000ULL) // in sec

/// State machine
typedef enum
{
  kInit = 0, kStart, kIdle,

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

/// A static handle of a RAIL timer
static RAIL_MultiTimer_t gStatPeriodTimer, gRX_timeout, gTX_timeout;

/// Receive and Send FIFO
static uint8_t gRX_fifo[RAIL_FIFO_RX_SIZE];

static union
{
  // Used to align this buffer as needed
  RAIL_FIFO_ALIGNMENT_TYPE align[RAIL_FIFO_TX_SIZE / RAIL_FIFO_ALIGNMENT];
  uint8_t fifo[RAIL_FIFO_TX_SIZE];
} gTX_fifo;

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
static volatile bool gStatTimerDone = false;

/// TX and RX counters
volatile uint32_t gRX_counter = 0UL;
volatile uint32_t gTX_counter = 1UL;
volatile uint32_t gTX_counter_old = 0UL;
volatile uint32_t gRX_counter_old = 0UL;
uint32_t gRX_counter_prev = 0UL;

/// Flag, indicating transmit request (button was pressed / CLI transmit request has occurred)
volatile bool gTX_requested = false;

/// Flag, indicating stat period on CLI
volatile uint32_t gSTAT_period = STAT_PERIOD_us;

/// Various flags
static volatile bool gPrintStat = false;    // Print and display statistics
static volatile bool gStartProcess = false;   // Start ping pong

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
static uint16_t unpack_packet_from_rx(uint8_t *rx_destination, const RAIL_RxPacketInfo_t *packet_information, uint8_t **start_of_payload);

/******************************************************************************
 * The API prepares the packet for sending and load it in the RAIL TX FIFO
 *
 * @param rail_handle Which rail handlers should be used for the TX FIFO writing
 * @param out_data The payload buffer
 * @param length The length of the payload
 *****************************************************************************/
static void prepare_packet_to_tx(void);

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
 * Timer callback, called if any button is pressed or released.
 *****************************************************************************/
void timer_callback(RAIL_MultiTimer_t *tmr, RAIL_Time_t expectedTimeOfEvent, void *cbArg);

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
/// Flag, indicating transmit request (button was pressed / CLI transmit request has occurred)
volatile bool tx_requested = false;
/// Flag, indicating received packet is forwarded on CLI or not
volatile bool rx_requested = true;

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
        // Keep the packet in the radio buffer, download it later at the state machine
        RAIL_HoldRxPacket(rail_handle);
        gPacketRecieved = true;
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

  // Handle Rx timeout --> needed when RAIL_ScheduleRx used
  if (events & (1ULL << RAIL_EVENT_RX_SCHEDULED_RX_END_SHIFT))
  {
      gRX_timeout_error = true;
  }

  // Handle TX events
  if (events & RAIL_EVENTS_TX_COMPLETION)
  {
      if (events & RAIL_EVENT_TX_PACKET_SENT)
      {
        // Handle next step
        gPacketSent = true;
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
    gCAL_tab[0]++;
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
  }
}

/******************************************************************************
 * Timer callback, called if any button is pressed or released.
 *****************************************************************************/
void timer_callback(RAIL_MultiTimer_t *tmr, RAIL_Time_t expectedTimeOfEvent, void *cbArg)
{
  (void) expectedTimeOfEvent;   // To avoid warnings
  (void) cbArg;         // To avoid warnings

  if (tmr == &gStatPeriodTimer) // used to print stat periodically
  {
    gStatTimerDone = true;
    gOldElapsedTime = gElapsedTime;
    gElapsedTime = RAIL_GetTime();
  }
  else if (tmr == &gRX_timeout) // used with standard StartRx when RAIL_ScheduleRx isn't used
  {
    gRX_timeout_error = true;
    GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_MISC);
  }
  else if (tmr == &gTX_timeout) // used with standard StartTx when RAIL_StartScheduledTx isn't used
  {
    gTX_timeout_error = true;
  }
}

// -----------------------------------------------------------------------------
//                          Private Function Definitions
// ----------------------------------------------------------------------------
static __INLINE void PrintStatus(RAIL_Status_t status, char* text)
{
#if (qPrintErrorsL1)
  if (status != RAIL_STATUS_NO_ERROR)
  {
    app_log_warning("%s (%d)\n", text, status);
  }
#endif  // qPrintErrorsL1
}

static __INLINE void PrintError(uint64_t errcode, char* text)
{
#if (qPrintErrorsL2)
    if (errcode != gOldErrorCode)
    {
      app_log_error("%s (0x%llX)\n", text, errcode);
      gOldErrorCode = errcode;
    }
#endif  // qPrintErrorsL2
}

/******************************************************************************
 * SetState : change state machine and backup previous state machine
 *****************************************************************************/
static __INLINE void SetState(StateEnum state)
{
  gPrevProtocolState = gProtocolState;
  gProtocolState = state;
}

/******************************************************************************
 * CfgRxMode : change to RX mode
 *****************************************************************************/
static __INLINE void CfgRxMode(void)
{
  // Start RX and check result

  // For oscillo debug purposes
  GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_RX);

  // Start RX
// ->  RAIL_Status_t status = RAIL_StartRx(gRailHandle, CHANNEL, NULL);
  RAIL_Status_t status = RAIL_ScheduleRx(gRailHandle, CHANNEL, &gRailScheduleCfgRX, NULL);

  PrintStatus(status, "Warning RAIL_StartRx");
}

/******************************************************************************
 * CfgTxMode : prepare buffer and change to TX mode
 *****************************************************************************/
static __INLINE void CfgTxMode(void)
{
  // Initialize radio buffer
  prepare_packet_to_tx();

  // For oscillo debug purposes
  GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_TX);

  RAIL_Status_t status = RAIL_StartTx(gRailHandle, CHANNEL, RAIL_TX_OPTIONS_DEFAULT, NULL);
  PrintStatus(status, "Warning RAIL_StartTx");

  StartTimerTO_TX();
}

/******************************************************************************
 * StartTimerTO_Stat : start timer to compute statistics
 *****************************************************************************/
static __INLINE void StartTimerTO_Stat(void)
{
  RAIL_Status_t status = RAIL_SetMultiTimer(&gStatPeriodTimer, gSTAT_period, RAIL_TIME_DELAY, &timer_callback, NULL);
  gElapsedTime = RAIL_GetTime();
  PrintStatus(status, "Warning RAIL_SetMultiTimer STAT");
}

/******************************************************************************
 * StartTimerTO_RX : start timer on RX
 *****************************************************************************/
static __INLINE void StartTimerTO_RX(void)
{
  RAIL_Status_t status = RAIL_SetMultiTimer(&gRX_timeout, RX_TIMEOUT, RAIL_TIME_DELAY, &timer_callback, NULL);
  PrintStatus(status, "Warning RAIL_SetMultiTimer RX");
}

/******************************************************************************
 * StartTimerTO_TX : start timer on TX
 *****************************************************************************/
static __INLINE void StartTimerTO_TX(void)
{
  RAIL_Status_t status = RAIL_SetMultiTimer(&gTX_timeout, TX_TIMEOUT, RAIL_TIME_DELAY, &timer_callback, NULL);
  PrintStatus(status, "Warning RAIL_SetMultiTimer TX");
}

/******************************************************************************
 * Set up the rail TX fifo for later usage
 * @param rail_handle Which rail handler should be updated
 *****************************************************************************/
void set_up_tx_fifo(RAIL_Handle_t rail_handle)
{
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
  gTX_fifo.align[0] = gTX_counter;
  uint16_t allocated_tx_fifo_size = RAIL_SetTxFifo(gRailHandle, gTX_fifo.fifo, TX_PAYLOAD_LENGTH, RAIL_FIFO_TX_SIZE);
  app_assert(allocated_tx_fifo_size == RAIL_FIFO_TX_SIZE, "RAIL_SetTxFifo() failed to allocate a large enough fifo (%d bytes instead of %d bytes)\n", allocated_tx_fifo_size, RAIL_FIFO_TX_SIZE);
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

    if (status != RAIL_STATUS_NO_ERROR)
    {
#if (qPrintErrorsL1)
      app_log_warning("Error ReleaseRxPacket (%d)\n", status);
#endif  // qPrintErrorsL1
    }

    if (/*gRX_requested &&*/ (packet_info.packetStatus == RAIL_RX_PACKET_READY_SUCCESS))
    {
      DisplayReceivedMsg(start_of_packet, packet_size);
      uint32_t delta = gRX_counter - gRX_counter_prev;
      if (delta > 1)
      {
        gRX_tab[3] ++;
        gRX_tab[4] = (delta > gRX_tab[4] ? delta : gRX_tab[4]);
      }
      else
        gRX_tab[0]++;
    }
    else
      gRX_tab[6]++;

    // Indicate RX in progress on LED0
    sl_led_toggle(&sl_led_led0);
    rx_packet_handle = RAIL_GetRxPacketInfo(gRailHandle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);
  }
  GPIO_PinOutClear(DEBUG_PORT, DEBUG_PIN_RX);
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
    SetState(kMsgReceived);
  }
  else if (gPacketSent)
  {
    gPacketSent = false;
    GPIO_PinOutClear(DEBUG_PORT, DEBUG_PIN_TX);
    gTX_tab[0]++;
    // Increment counter and prepare new data
    gTX_counter++;    // pas de test over
    prepare_packet_to_tx();

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
  else if (gRX_invalid)
  {
    gRX_invalid = false;
    gRX_tab[5]++;
    SetState(kErrorRx);
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
  else if (gTX_invalid)
  {
    gTX_invalid = false;
    gTX_tab[5]++;
    SetState(kErrorTx);
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
    // State after POR, wait until pressing BTN0 on Master
    prepare_packet_to_tx();
    SetState(kIdle);
    break;

  case kStart:
    // TX started by pressing BTN0
    SetState(kSwitchTx);
    app_log_info("Start\n");
    sl_led_turn_on(&sl_led_led1);
    StartTimerTO_Stat();
    gTX_counter_old = gTX_counter;
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
#if (qRx2TxAutoTransition)
    SetState(kIdle);    // Auto transition to TX after successfull receive
    // For oscillo debug purposes
    GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_TX);
    StartTimerTO_TX();    // use TX_timeout in order to be sure to restart with TX only on Master if time outs
#else
    SetState(kSwitchTx);
#endif  // qNoRx2TxAutoTransition
    DecodeReceivedMsg();
    break;

  case kErrorRx:
    // Error on RX
#if (qAutoTransition)
    SetState(kIdle);    // Auto transition to RX after unsuccessfull receive
    // For oscillo debug purposes
    GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_RX);
    StartTimerTO_RX();    // use RX_timeout in order to be sure to restart with TX only on Master if time outs
    SetState(kSwitchRx);
#endif  // qAutoTransition

    PrintError(gErrorCode, "RX Error");
    break;

  case kTimeOutRx:
    // Timeout on RX
    SetState(kSwitchTx);
    PrintError(gErrorCode, "RX TimeOut");
    break;

  case kSwitchRx:
    radio_state = RAIL_GetRadioState(gRailHandle);

    if ((radio_state != RAIL_RF_STATE_TX_ACTIVE) ||
      (radio_state != RAIL_RF_STATE_RX_ACTIVE))
    {
      SetState(kWaitRx);
      CfgRxMode();
    }
    else
    {
      SetState(kSwitchRx);  // Wait until transition ready
    }
    break;

    // -----------------------------------------
    // TX
    // -----------------------------------------
  case kSendMsg:
    // Wait until message sent
    break;

  case kMsgSent:
#if (qAutoTransition)
    SetState(kIdle);    // Auto transition to RX after successfull transmit
    // For oscillo debug purposes
    GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_RX);
    StartTimerTO_RX();    // use RX_timeout in order to be sure to restart with TX only on Master if time outs
#else
    SetState(kSwitchRx);  // bidirectional
#endif  // qAutoTransition
    DisplaySentMsg();
    // Indicate TX in progress on LED1
    sl_led_toggle(&sl_led_led1);
    break;

  case kErrorTx:
#if (qAutoTransition)
    SetState(kIdle);    // Auto transition to RX after unsuccessfull transmit
    // For oscillo debug purposes
    GPIO_PinOutSet(DEBUG_PORT, DEBUG_PIN_RX);
    StartTimerTO_RX();    // use RX_timeout in order to be sure to restart with TX only on Master if time outs
#else
    SetState(kSwitchRx);
#endif  // qAutoTransition

    PrintError(gErrorCode, "TX Error");
    break;

  case kTimeOutTx:
    // Timeout on TX
#if (qRx2TxAutoTransition)
    SetState(kSwitchTx);
#else // !qRx2TxAutoTransition
    SetState(kSwitchRx);
#endif  // qRx2TxAutoTransition

    PrintError(gErrorCode, "TX TimeOut");
    break;

  case kSwitchTx:
    radio_state = RAIL_GetRadioState(gRailHandle);

    if ((radio_state != RAIL_RF_STATE_TX_ACTIVE) ||
      (radio_state != RAIL_RF_STATE_RX_ACTIVE))
    {
      SetState(kSendMsg);
      CfgTxMode();
    }
    else
    {
      SetState(kSwitchTx);  // Wait until transition ready
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
#endif  // qPrintErrorsL1
    break;

  default:
    app_assert(false, "Unknown state (%d)\n", gProtocolState);
    break;
  }

  /// -----------------------------------------
  /// Statistics
  /// -----------------------------------------
  if (gStatTimerDone || gPrintStat)
  {
    gStatTimerDone = false;
    gPrintStat = false;
    DisplayStat();

    gTX_counter_old = gTX_counter;
    gRX_counter_old = gRX_counter;
  }
}



