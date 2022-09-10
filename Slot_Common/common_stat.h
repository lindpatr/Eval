/*
 * app_stat.h
 *
 *  Created on: 8 sept. 2022
 *      Author: BEL-LinPat
 */

#ifndef COMMON_STAT_H_
#define COMMON_STAT_H_

#include <stdint.h>
#include "app_process.h"
#include "app_log.h"
#include "app_init.h"


#define SIZE_UINT64_IN_BITS (int)(8*sizeof(uint64_t))

#define TIMER_TIMEOUT_RX_SHIFT  (RAIL_EVENT_DETECT_RSSI_THRESHOLD_SHIFT+1)
#define TIMER_TIMEOUT_TX_SHIFT  (TIMER_TIMEOUT_RX_SHIFT+1)

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
		"DETECT_RSSI_THRSHOLD",
        "TIMER_TIMEOUT_RX    ",
        "TIMER_TIMEOUT_TX    "};
#endif	// qPrintEvents

// Position in the gTX_tab / gRX_tab / gCAL_tab
typedef enum
{
    TAB_POS_TX_OK       = 0,
    TAB_POS_RX_OK       = 0,
    TAB_POS_CAL_REQ     = 0,
    TAB_POS_TX_ERR      = 1,                            // RAIL_EVENT_TX_ABORTED + RAIL_EVENT_TX_BLOCKED + RAIL_EVENT_TX_UNDERFLOW + RAIL_EVENT_TX_CHANNEL_BUSY + RAIL_EVENT_TX_SCHEDULED_TX_MISSED
    TAB_POS_RX_ERR      = 1,                            // RAIL_EVENT_RX_PACKET_ABORTED + RAIL_EVENT_RX_FRAME_ERROR + RAIL_EVENT_RX_FIFO_OVERFLOW + RAIL_EVENT_RX_ADDRESS_FILTERED (don't activate it!) + RAIL_EVENT_RX_SCHEDULED_RX_MISSED
    TAB_POS_CAL_ERR     = 1,
    TAB_POS_RX_TIMEOUT  = 2,
    TAB_POS_TX_TIMEOUT  = 2,
    TAB_POS_RX_GAP      = 3,
    TAB_POS_RX_GAP_MAX  = 4,
    TAB_POS_RX_CRC_ERR  = 5,
    TAB_POS_LAST
} TabPosEnum;

extern uint32_t gTX_tab[TAB_POS_LAST];                 // 0 = #TX_OK  	1 = #TX_Err   2 = #TX_TimeOut 	3 = ND             	4 = ND          		5 = ND
extern uint32_t gRX_tab[MAX_NODE][TAB_POS_LAST];      // 0 = #RX_OK  	1 = #RX_Err   2 = #RX_TimeOut 	3 = #Gap RX count 	4 = Max gap RX count  	5 = #CRC_Err
extern uint32_t gCAL_tab[TAB_POS_LAST];                // 0 = #CAL_REQ  1 = #CAL_Err 2 = ND      		3 = ND        		4 = ND          		5 = ND

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
    float deltaTime, localStat, localStat2, localStat3, localStat4;
    uint32_t sumRelativeCounter = (gTX_counter - gTX_counter_old);
    uint32_t sumRXCounter = 0U;
    uint32_t sumRXGap = 0U;
    uint32_t sumRXOk = 0U;
    uint32_t deltaRXGap = 0U;

    // Elapsed time since last stat
    deltaTime = (float) (gElapsedTime - gOldElapsedTime) / 1000000.0f;

    // Compute sum of absolute RX (from Slave) counters and sum of relative (since last stat) RX (from Slave) counters
	if (gDeviceCfgAddr->ismaster)
	{
	    // Master node --> take in account all slaves
	    for (int i = 1; i < MAX_NODE; i++)
	    {
	        if (gRX_counter[i])
	        {
	            sumRXCounter += gRX_counter[i];                               // Sum of absolute RX (from Slave) counters
	            sumRelativeCounter += (gRX_counter[i] - gRX_counter_old[i]);  // Sum of absolute RX (from Slave) "old" counters
	            sumRXGap += gRX_tab[i][TAB_POS_RX_GAP];                       // Sum of absolute RX (from Slave) gap occurences
	            sumRXOk += gRX_tab[i][TAB_POS_RX_OK];                         // Sum of TX Ok (from Slave) counters
	        }
	    }
	}
	else    // Slave node -> take in account only the concerned slave
	{
	    sumRXCounter = gRX_counter[gDeviceCfgAddr->posTab];
	    sumRelativeCounter = ((gTX_counter - gTX_counter_old) + (gRX_counter[gDeviceCfgAddr->posTab] - gRX_counter_old[gDeviceCfgAddr->posTab]));
	    sumRXGap = gRX_tab[gDeviceCfgAddr->posTab][TAB_POS_RX_GAP];
	    sumRXOk = gRX_tab[gDeviceCfgAddr->posTab][TAB_POS_RX_OK];
	}

	deltaRXGap = ((sumRXGap > gRX_tab[gDeviceCfgAddr->posTab][TAB_POS_RX_ERR]) ? (sumRXGap - gRX_tab[gDeviceCfgAddr->posTab][TAB_POS_RX_ERR]) : 0);

    // Relative (since last stat) Slave TX and Master TX pro second
    localStat = (float)(sumRelativeCounter) / deltaTime;
	// Then compare to the key transmission rate of 1 master + 100 slaves @10ms
	localStat2 = (10.0f * 10100.0f) / localStat;
	// Cumulative TX error rate
	localStat3 = 100.0f * (float) (gTX_tab[TAB_POS_TX_ERR] + gTX_tab[TAB_POS_TX_TIMEOUT]) / (float) gTX_counter;
	// Cumulative RX error rate
	localStat4 = 100.0f * (float) (gRX_tab[gDeviceCfgAddr->posTab][TAB_POS_RX_ERR] + gRX_tab[gDeviceCfgAddr->posTab][TAB_POS_RX_TIMEOUT] + gRX_tab[gDeviceCfgAddr->posTab][TAB_POS_RX_CRC_ERR] + deltaRXGap) / (float) sumRXCounter;

	// Print on serial COM
	app_log_info("\n");
	app_log_info("%s #%03d statistics\n", (gDeviceCfgAddr->ismaster ? "MASTER" : "SLAVE"), gDeviceCfgAddr->internalAddr);
	app_log_info("----------------------\n");
	app_log_info("Elapsed time               : %0.2f sec\n", deltaTime);

	// Counters TX and RX
	app_log_info("#Counter %s            : %lu\n", (gDeviceCfgAddr->ismaster ? "      " : "      "), gTX_tab[TAB_POS_TX_OK]);
    app_log_info("#Counter (%s)          : %lu\n", (gDeviceCfgAddr->ismaster ? "Slaves" : "Master"), sumRXOk);

    // Errors TX and RX
	app_log_info("TX Err (#err/#TO)          : %0.3f%% (%lu/%lu)\n", localStat3, gTX_tab[TAB_POS_TX_ERR], gTX_tab[TAB_POS_TX_TIMEOUT]);
	app_log_info("RX Err (#err/#TO/#CRC/#gap): %0.3f%% (%lu/%lu/%lu/%lu)\n", localStat4,
	                                                                         gRX_tab[gDeviceCfgAddr->posTab][TAB_POS_RX_ERR],
	                                                                         gRX_tab[gDeviceCfgAddr->posTab][TAB_POS_RX_TIMEOUT],
	                                                                         gRX_tab[gDeviceCfgAddr->posTab][TAB_POS_RX_CRC_ERR],
	                                                                         deltaRXGap);
    // Transmission rate
    if (gDeviceCfgAddr->ismaster)
    {
        app_log_info("Rate (loop 100)            : %0.2f msg/s (%0.2f ms)\n", localStat, localStat2);
    }
    else
    {
        app_log_info("Rate                       : %0.2f msg/s\n", localStat);
    }

    // Calibration
    app_log_info("Cal #request (#err)        : %lu (%lu)\n", gCAL_tab[TAB_POS_CAL_REQ], gCAL_tab[TAB_POS_CAL_ERR]);

	// Slave detail
	app_log_info("\n");
	app_log_info("%s detail\n", (gDeviceCfgAddr->ismaster ? "Slaves" : "Slave"));
	app_log_info("-------------\n");
    if (gDeviceCfgAddr->ismaster)
    {
        // Master node --> take in account all slaves
        for (int i = 1; i < MAX_NODE; i++)
        {
            if (gRX_counter[i])
            {
                app_log_info("Slave #%03d #cnt (#gap/max) : %lu (%lu/%lu)\n",
                        i,
                        gRX_tab[i][TAB_POS_RX_OK],
                        gRX_tab[i][TAB_POS_RX_GAP],
                        gRX_tab[i][TAB_POS_RX_GAP_MAX]);
            }
        }
    }
    else    // Slave node -> take in account only the concerned slave
    {
        app_log_info("#Counter (#gap/max)        : %lu (%lu=%lu-%lu#fr.err / max: %lu)\n", gRX_tab[gDeviceCfgAddr->posTab][TAB_POS_RX_OK],
                                                                   deltaRXGap,
                                                                   gRX_tab[gDeviceCfgAddr->posTab][TAB_POS_RX_GAP],
                                                                   gRX_tab[gDeviceCfgAddr->posTab][TAB_POS_RX_ERR],
                                                                   gRX_tab[gDeviceCfgAddr->posTab][TAB_POS_RX_GAP_MAX]);
    }

    // Detail of callback events
	DisplayEvents();
}

#endif /* APP_STAT_H_ */
