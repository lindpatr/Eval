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



#define TIMER_TIMEOUT_RX_SHIFT  (RAIL_EVENT_DETECT_RSSI_THRESHOLD_SHIFT+1)
#define TIMER_TIMEOUT_TX_SHIFT  (TIMER_TIMEOUT_RX_SHIFT+1)

// Period to print statistics
#define STAT_PERIOD_s (60U/*1800U*/)                // Default value, in us (30 min) --> CLI command delay xx for other value (not stored! ... yet)
#define STAT_PERIOD_us (STAT_PERIOD_s * SEC) // in sec
#define STAT_PERIOD_MAX (53*60)              // in sec (= 53 min); RAIL function permit max. 3221 sec !?!

#define SIZE_UINT64_IN_BITS (int)(8*sizeof(uint64_t))

/// Tables for error statistics
extern volatile uint32_t gCB_tab[SIZE_UINT64_IN_BITS];

// Position in the gTX_tab / gRX_tab / gCAL_tab
typedef enum
{
    TAB_POS_RX_OK       = 0,
    TAB_POS_RX_ERR      = 1,                            // RAIL_EVENT_RX_PACKET_ABORTED + RAIL_EVENT_RX_FRAME_ERROR + RAIL_EVENT_RX_FIFO_OVERFLOW + RAIL_EVENT_RX_ADDRESS_FILTERED (don't activate it!) + RAIL_EVENT_RX_SCHEDULED_RX_MISSED
    TAB_POS_RX_TIMEOUT  = 2,
    TAB_POS_RX_GAP      = 3,
    TAB_POS_RX_GAP_MAX  = 4,
    TAB_POS_RX_CRC_ERR  = 5,
    TAB_POS_RX_SYNC_LOST= 6,

    TAB_POS_TX_OK       = 0,
    TAB_POS_TX_ERR      = 1,                            // RAIL_EVENT_TX_ABORTED + RAIL_EVENT_TX_BLOCKED + RAIL_EVENT_TX_UNDERFLOW + RAIL_EVENT_TX_CHANNEL_BUSY + RAIL_EVENT_TX_SCHEDULED_TX_MISSED
    TAB_POS_TX_TIMEOUT  = 2,
    //                  = 3,
    //                  = 4,
    //                  = 5,
    //                  = 6,

    TAB_POS_CAL_REQ     = 0,
    TAB_POS_CAL_ERR     = 1,
    //                  = 2,
    //                  = 3,
    //                  = 4,
    //                  = 5,
    //                  = 6,

    TAB_POS_LAST        = 7

} TabPosEnum;

extern uint32_t gTX_tab[TAB_POS_LAST];                 // 0 = #TX_OK  	1 = #TX_Err   2 = #TX_TimeOut 	3 = ND             	4 = ND          		5 = ND
extern uint32_t gRX_tab[MAX_NODE][TAB_POS_LAST];       // 0 = #RX_OK  	1 = #RX_Err   2 = #RX_TimeOut 	3 = #Gap RX count 	4 = Max gap RX count  	5 = #CRC_Err    6 = #SYNC_LOST
extern uint32_t gCAL_tab[TAB_POS_LAST];                // 0 = #CAL_REQ  1 = #CAL_Err  2 = ND      		3 = ND        		4 = ND          		5 = ND

extern uint32_t gTX_tab_old[TAB_POS_LAST];
extern uint32_t gRX_tab_old[MAX_NODE][TAB_POS_LAST];
extern uint32_t gCAL_tab_old[TAB_POS_LAST];

/// Timeout for printing and displaying the statistics
extern volatile RAIL_Time_t gElapsedTime;
extern volatile uint64_t gTotalElapsedTime;
extern volatile RAIL_Time_t gOldElapsedTime;
extern volatile uint32_t gCountPrintStat;              // Stat print counter (how many times)
extern volatile RAIL_Time_t gStatDelay;                 // Value, indicating stat period on CLI


/******************************************************************************
 * DisplayReceivedMsg : print received data
 *****************************************************************************/
void DisplayReceivedMsg(const uint8_t *const rx_buffer, uint16_t length);

/******************************************************************************
 * DisplaySentMsg : print sent data
 *****************************************************************************/
void DisplaySentMsg(void);

/******************************************************************************
 * Decode RAIL events and increase event's counters
 *****************************************************************************/
void DecodeEvents(RAIL_Events_t *events);

/******************************************************************************
 * DisplayStat : compute, print and display statistics
 *****************************************************************************/
void DisplayStat(void);

#endif /* APP_STAT_H_ */
