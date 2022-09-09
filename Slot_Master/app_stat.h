/*
 * app_stat.h
 *
 *  Created on: 8 sept. 2022
 *      Author: BEL-LinPat
 */

#ifndef APP_STAT_H_
#define APP_STAT_H_

#include <stdint.h>
#include "app_process.h"
#include "app_log.h"

#define SIZE_UINT64_IN_BITS (int)(8*sizeof(uint64_t))

/// Tables for error statistics
static volatile uint32_t gCB_tab[SIZE_UINT64_IN_BITS ] = { 0 };     // index: see RAIL_ENUM_GENERIC(RAIL_Events_t, uint64_t) in rail_types.h

#if (qPrintEvents)
static char *gCB_descr_tab[SIZE_UINT64_IN_BITS ] =
{ 		"RSSI_AVERAGE_DONE   ",
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
		"DETECT_RSSI_THRSHOLD" };
#endif	// qPrintEvents

extern uint32_t gTX_tab[7];    // 0 = #TX_OK  	1 = #TX_Err   2 = ND      		3 = #Retransmit   	4 = ND          		5 = #TX_Invalid   	6 = ND
extern uint32_t gRX_tab[7];    // 0 = #RX_OK  	1 = #RX_Err   2 = #RX_TimeOut 	3 = #Gap RX count 	4 = Max gap RX count  	5 = #RX_Invalid   	6 = #CRC_Err
extern uint32_t gCAL_tab[7];   // 0 = #CAL_REQ  1 = #CAL_Err  2 = ND      		3 = ND        		4 = ND          		5 = ND        		6 = ND

/// Timeout for printing and displaying the statistics
extern volatile RAIL_Time_t gElapsedTime;
extern volatile RAIL_Time_t gOldElapsedTime;

/******************************************************************************
 * DisplayReceivedMsg : print received data
 *****************************************************************************/
static __INLINE void DisplayReceivedMsg(const uint8_t *const rx_buffer, uint16_t length)
{
#if (qPrintRX)
  // Print received data on serial COM
  app_log_info("RX: ");
  for (uint16_t i = 0; i < length; i++) {
    app_log_info("0x%02X, ", rx_buffer[i]);
  }
  app_log_info("\n");
#else	// To avoid compile warnings
  	(void) rx_buffer;
	(void) length;
#endif
}

/******************************************************************************
 * DisplaySentMsg : print sent data
 *****************************************************************************/
static __INLINE void DisplaySentMsg(void)
{
#if (qPrintTX)
  // Print sent data on serial COM
  app_log_info("TX: %d\n",gTX_counter);
#endif
}

/******************************************************************************
 * DisplayStat : print event counters
 *****************************************************************************/
static __INLINE void DisplayEvents(void)
{
#if (qPrintEvents)
  app_log_info("\n");
  app_log_info("Radio events detail\n");
  app_log_info("-------------------\n");
  for (int i = 0; i <SIZE_UINT64_IN_BITS; i++)
  {
    if (gCB_tab[i] >0)
    {
      app_log_info("b%02d %s: %lu\n", i, gCB_descr_tab[i], gCB_tab[i]);
    }
  }
#endif  // qPrintEvents
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
#else	// To avoid compile warnings
	(void) events;
#endif  // qPrintEvents
}

/******************************************************************************
 * DisplayStat : print and display statistics
 *****************************************************************************/
static __INLINE void DisplayStat(void)
{
	// Statistics
	float deltaTime = (float) (gElapsedTime - gOldElapsedTime) / 1000000.0f;
	float localStat = (float) (gTX_counter + gRX_counter - gTX_counter_old - gRX_counter_old) / deltaTime;
	float localStat2 = (10.0f * 10100.0f) / localStat;
	float localStat3 = 100.0f * (float) (gTX_tab[1] + gTX_tab[2] + gTX_tab[5]) / (float) gTX_counter;
	float localStat4 = 100.0f * (float) (gRX_tab[1] + gRX_tab[2] + gRX_tab[5] + gRX_tab[6]) / (float) gRX_counter;

	// Print on serial COM
	app_log_info("\n");
	app_log_info("Performance statistics\n");
	app_log_info("----------------------\n");
	app_log_info("Elapsed time       : %0.2f sec\n", deltaTime);

	app_log_info("#Counter (TX)      : %lu\n", gTX_tab[0]);
	app_log_info("#Counter (RX)      : %lu\n", gRX_tab[0]);

	app_log_info("TX Err (see below) : %0.3f%%\n", localStat3);
	app_log_info("#Err/#TO/#Inv      : %lu/%lu/%lu\n", gTX_tab[1], gTX_tab[2], gTX_tab[5]);
	app_log_info("TX retransmit count: %lu\n", gTX_tab[3]);
	app_log_info("RX Err (see below) : %0.3f%%\n", localStat4);
	app_log_info("#Err/#TO/#Inv/#CRC : %lu/%lu/%lu/%lu\n", gRX_tab[1], gRX_tab[2], gRX_tab[5], gRX_tab[6]);
	app_log_info("Counter #gap (max) : %lu (%d)\n", gRX_tab[3], gRX_tab[4]);

	app_log_info("Cal #request (#Err): %lu (%lu)\n", gCAL_tab[0], gCAL_tab[1]);
	app_log_info("Rate (loop 100)    : %0.2f msg/s (%0.2f ms)\n", localStat, localStat2);

	DisplayEvents();
}

#endif /* APP_STAT_H_ */
