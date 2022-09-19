/*
 * app_stat.c
 *
 *  Created on: 8 sept. 2022
 *      Author: BEL-LinPat
 */

#ifndef COMMON_STAT_H_
#include "common_stat.h"
#endif

uint32_t gTX_tab[TAB_POS_LAST] = { 0 };                // 0 = #TX_OK    1 = #TX_Err   2 = #TX_TimeOut   3 = ND              4 = ND                  5 = ND
uint32_t gRX_tab[MAX_NODE][TAB_POS_LAST] = { 0 };      // 0 = #RX_OK    1 = #RX_Err   2 = #RX_TimeOut   3 = #Gap RX count   4 = Max gap RX count    5 = #CRC_Err
uint32_t gCAL_tab[TAB_POS_LAST] = { 0 };               // 0 = #CAL_REQ  1 = #CAL_Err 2 = ND             3 = ND              4 = ND                  5 = ND

uint32_t gTX_tab_old[TAB_POS_LAST] = { 0 };
uint32_t gRX_tab_old[MAX_NODE][TAB_POS_LAST] = { 0 };
uint32_t gCAL_tab_old[TAB_POS_LAST] = { 0 };


/// Timeout for printing and displaying the statistics
volatile RAIL_Time_t gElapsedTime = 0UL;
volatile uint64_t gTotalElapsedTime = 0UL;
volatile RAIL_Time_t gOldElapsedTime = 0UL;
volatile uint32_t gCountPrintStat = 0UL;                       // Stat print counter (how many times)
volatile RAIL_Time_t gStatDelay = (RAIL_Time_t)STAT_PERIOD_us;  // Value, indicating stat period on CLI

/// Tables for error statistics
volatile uint32_t gCB_tab[SIZE_UINT64_IN_BITS ] = { 0 };     // index: see RAIL_ENUM_GENERIC(RAIL_Events_t, uint64_t) in rail_types.h

#if (qPrintEvents)
static char *gCB_descr_tab[SIZE_UINT64_IN_BITS ] =
{       "RSSI_AVERAGE_DONE   ",
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
#endif  // qPrintEvents

/******************************************************************************
 * DisplayReceivedMsg : print received data
 *****************************************************************************/
__INLINE void DisplayReceivedMsg(const uint8_t *const rx_buffer, uint16_t length)
{
#if (qPrintRX)
  // Print received data on serial COM
  app_log_info("RX: ");
  for (uint16_t i = 0; i < length; i++) {
    app_log_info("0x%02X, ", rx_buffer[i]);
  }
  app_log_info("\n");
#else   // To avoid compile warnings
    (void) rx_buffer;
    (void) length;
#endif
}

/******************************************************************************
 * DisplaySentMsg : print sent data
 *****************************************************************************/
__INLINE void DisplaySentMsg(void)
{
#if (qPrintTX)
  // Print sent data on serial COM
  app_log_info("TX: %d\n",gTX_counter.u32);
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
      app_log_info("b%02d %s: %d\n", i, gCB_descr_tab[i], gCB_tab[i]);
    }
  }
#endif  // qPrintEvents
}

/******************************************************************************
 * Decode RAIL events and increase event's counters
 *****************************************************************************/
__INLINE void DecodeEvents(RAIL_Events_t *events)
{
#if (qPrintEvents)
  uint64_t ev = *events;

  for (int i = 0; i <SIZE_UINT64_IN_BITS; i++)
  {
    if (ev & 0x1ULL)
      gCB_tab[i]++;

    ev >>= 1;
  }
#else   // To avoid compile warnings
    (void) events;
#endif  // qPrintEvents
}

/******************************************************************************
 * DisplayStat : compute, print and display statistics
 *****************************************************************************/
#define REF_MSG_PER_SEC (10.0f * 10100.0f)      //  key transmission rate of 1 master + 100 slaves @10ms

__INLINE void DisplayStat(void)
{
#if (qREL_STAT)
    // Relative
    // --------
    uint32_t relRXCounter = 0U;
    uint32_t relRXGap = 0U;
    uint32_t relRXOk = 0U;

    float statRelMsgPerSec, statRelMsgPerSecRef, statRelTXErr, statRelRXErr;
#endif  // qREL_STAT

#if (qABS_STAT)
    // Absolute
    // --------
    uint32_t absRXCounter = 0U;
    uint32_t absRXGap = 0U;
    uint32_t absRXOk = 0U;

    float statAbsMsgPerSec, statAbsMsgPerSecRef, statAbsTXErr, statAbsRXErr;
#endif  // qABS_STAT

    float relElapsedTime;
    RAIL_Time_t deltaElapsedTime;
    bool isMaster = gDeviceCfgAddr->ismaster;
    uint8_t myDevice = gDeviceCfgAddr->posTab;


    // Common processing
    // -----------------

    // Based on 32 bits counter, gElapsedTime roll over every 1h 11 min 34.96 sec
    if (gElapsedTime < gOldElapsedTime)
    {
        uint64_t tmp = gElapsedTime + UINT32_MAX - gOldElapsedTime;
        deltaElapsedTime = (uint32_t)tmp;
    }
    else
        deltaElapsedTime = (gElapsedTime - gOldElapsedTime);

    // Compute sum of absolute RX (from Slave) counters and sum of relative (since last stat) RX (from Slave) counters
#if (qREL_STAT)
    uint32_t relTXCounter = (gTX_counter.u32 - gTX_counter_old);
#endif  // qREL_STAT
#if (qABS_STAT)
    uint32_t absTXCounter = gTX_counter.u32;
#endif  // qABS_STAT

    if (isMaster)
    {
        // Master node --> take in account all enabled slaves [1..N]
        for (int i = 1; i <= common_getNbrDeviceOfType(SLAVE_TYPE, ALL); i++)
        {
            if (common_getConfigTable(i)->enable)
            {
                uint8_t pos = common_getConfigTable(i)->posTab;
                if (gRX_counter[pos].u32 > 0UL)
                {
#if (qREL_STAT)
                // Relative
                // --------
                relRXCounter += (gRX_counter[i].u32 - gRX_counter_old[i]);                                   // Sum of relative RX (from Slave) and TX counters
                relRXGap += (gRX_tab[i][TAB_POS_RX_GAP] - gRX_tab_old[i][TAB_POS_RX_GAP]);               // Sum of absolute RX (from Slave) gap occurences
                relRXOk += (gRX_tab[i][TAB_POS_RX_OK] - gRX_tab_old[i][TAB_POS_RX_OK]);                  // Sum of absolute TX Ok (from Slave) counters
#endif  // qREL_STAT
#if (qABS_STAT)
                // Absolute
                // --------
                absRXCounter += gRX_counter[i].u32;                         // Sum of absolute RX (from Slave) counters
                absRXGap += gRX_tab[i][TAB_POS_RX_GAP];                 // Sum of absolute RX (from Slave) gap occurences
                absRXOk += gRX_tab[i][TAB_POS_RX_OK];                   // Sum of absolute TX Ok (from Slave) counters
#endif  // qABS_STAT
                }
            }
        }
    }
    else    // Slave node -> take in account only the concerned slave
    {
#if (qREL_STAT)
        // Relative
        // --------
        relRXCounter = (gRX_counter[myDevice].u32 - gRX_counter_old[myDevice]);
        relRXGap = (gRX_tab[myDevice][TAB_POS_RX_GAP] - gRX_tab_old[myDevice][TAB_POS_RX_GAP]);
        relRXOk = (gRX_tab[myDevice][TAB_POS_RX_OK] - gRX_tab_old[myDevice][TAB_POS_RX_OK]);
#endif  // qREL_STAT
#if (qABS_STAT)
        // Absolute
        // --------
        absRXCounter = gRX_counter[myDevice].u32;
        absRXGap = gRX_tab[myDevice][TAB_POS_RX_GAP];
        absRXOk = gRX_tab[myDevice][TAB_POS_RX_OK];
#endif  // qABS_STAT
    }

#if (qREL_STAT)
    uint32_t relAllCounter = relTXCounter + relRXCounter;
#endif  // qREL_STAT
#if (qABS_STAT)
    uint32_t absAllCounter = absTXCounter + absRXCounter;
#endif  // qABS_STAT

#if (qREL_STAT)
    // Relative stat (since last occurence)
    // -------------
    // Elapsed time
    relElapsedTime = (float) (deltaElapsedTime) / (float)(SEC);
    // Messages pro second
    statRelMsgPerSec = (float)(relAllCounter) / relElapsedTime;
    // Then compare to the key transmission rate of 1 master + 100 slaves @10ms
    statRelMsgPerSecRef = REF_MSG_PER_SEC / statRelMsgPerSec;

    uint32_t relTXOk = (gTX_tab[TAB_POS_TX_OK] - gTX_tab_old[TAB_POS_TX_OK]);
    uint32_t relCalOk = (gCAL_tab[TAB_POS_CAL_REQ] - gCAL_tab_old[TAB_POS_CAL_REQ]);
    uint32_t relCalErr = (gCAL_tab[TAB_POS_CAL_ERR] - gCAL_tab_old[TAB_POS_CAL_ERR]);

    // TX error rate
    uint32_t relTXErr = (gTX_tab[TAB_POS_TX_ERR] - gTX_tab_old[TAB_POS_TX_ERR]);
    uint32_t relTXTimeOut = (gTX_tab[TAB_POS_TX_TIMEOUT] - gTX_tab_old[TAB_POS_TX_TIMEOUT]);

    statRelTXErr = 100.0f * (float) (relTXErr + relTXTimeOut) / (float) (relTXCounter);

    // RX error rate
    uint32_t relRXErr = (gRX_tab[myDevice][TAB_POS_RX_ERR] - gRX_tab_old[myDevice][TAB_POS_RX_ERR]);
    uint32_t relRXTimeOut = (gRX_tab[myDevice][TAB_POS_RX_TIMEOUT] - gRX_tab_old[myDevice][TAB_POS_RX_TIMEOUT]);
    uint32_t relCRCErr = (gRX_tab[myDevice][TAB_POS_RX_CRC_ERR] - gRX_tab_old[myDevice][TAB_POS_RX_CRC_ERR]);
    uint32_t relSYNCErr = (gRX_tab[myDevice][TAB_POS_RX_SYNC_LOST] - gRX_tab_old[myDevice][TAB_POS_RX_SYNC_LOST]);
    uint32_t remainingRelRXGap = ((relRXGap > relRXErr) ? (relRXGap - relRXErr): 0);        // Frame error implies a gap -> compute gap delta due to other reasons

    statRelRXErr = 100.0f * (float) (relRXErr + relRXTimeOut + relCRCErr + remainingRelRXGap) / (float) (relRXCounter);
#endif  // qREL_STAT

#if (qABS_STAT)
    // Absolute stat
    // -------------
    // Cumulative time
    gTotalElapsedTime += (uint64_t)(deltaElapsedTime);
    relElapsedTime = (float)((double) (gTotalElapsedTime) / (float)(SEC));
    // Messages pro second
    statAbsMsgPerSec = (float)(absAllCounter) / relElapsedTime;
    // Then compare to the key transmission rate of 1 master + 100 slaves @10ms
    statAbsMsgPerSecRef = REF_MSG_PER_SEC / statAbsMsgPerSec;

    uint32_t absTXOk = gTX_tab[TAB_POS_TX_OK];
    uint32_t absCalOk = gCAL_tab[TAB_POS_CAL_REQ];
    uint32_t absCalErr = gCAL_tab[TAB_POS_CAL_ERR];

    // TX error rate
    uint32_t absTXErr = gTX_tab[TAB_POS_TX_ERR];
    uint32_t absTXTimeOut = gTX_tab[TAB_POS_TX_TIMEOUT];

    statAbsTXErr = 100.0f * (float) (absTXErr + absTXTimeOut) / (float) (absTXCounter);
    if (statAbsTXErr > 100.0f)
        statAbsTXErr = 100.0f;

    // RX error rate
    uint32_t absRXErr = gRX_tab[myDevice][TAB_POS_RX_ERR];
    uint32_t absRXTimeOut = gRX_tab[myDevice][TAB_POS_RX_TIMEOUT];

    // Detect never responding slaves
    if (isMaster)   // From a master point of view
    {
        for (int i = 1; i <= common_getNbrDeviceOfType(SLAVE_TYPE, ALL); i++)
        {
            if (common_getConfigTable(i)->enable)
            {
                uint8_t pos = common_getConfigTable(i)->posTab;
                if (gRX_counter[pos].u32 == 0UL)
                {
                    absRXTimeOut += absTXOk;
                }
            }
        }
    }
    else    // From a slave point of view
    {
        if (absTXOk == 0)
        {
            absTXTimeOut = absRXOk;
        }
    }

    uint32_t absCRCErr = gRX_tab[myDevice][TAB_POS_RX_CRC_ERR];
    uint32_t absSYNCErr = gRX_tab[myDevice][TAB_POS_RX_SYNC_LOST];
    uint32_t remainingAbsRXGap = ((absRXGap > absRXErr) ? (absRXGap - absRXErr): 0);            // Frame error implies a gap -> compute gap delta due to other reasons

    statAbsRXErr = 100.0f * (float) (absRXErr + absRXTimeOut + absCRCErr + remainingAbsRXGap) / (float) (absRXCounter);
    if (statAbsRXErr > 100.0f)
        statAbsRXErr = 100.0f;
#endif  // qABS_STAT

    // Printing
    // --------
    // Print on serial COM

    app_log_info("\n");
    app_log_info("%s #%03d statistics\n", (isMaster ? "MASTER" : "SLAVE"), gDeviceCfgAddr->internalAddr);
    app_log_info("----------------------\n");
    app_log_info("Stat print #count          : %d\n", gCountPrintStat);

#if (qREL_STAT)
    // Relative stat (since last occurence)
    // -------------
    app_log_info("\nRelative (since last ocurence)\n");
    app_log_info("------------------------------\n");
    app_log_info("Elapsed time               : %0.2f sec\n", relElapsedTime);

    // Counters TX and RX
    app_log_info("#Counter %s            : %d\n", (isMaster ? "      " : "      "), relTXOk);
    app_log_info("#Counter (%s)          : %d\n", (isMaster ? "Slaves" : "Master"), relRXOk);

    // Errors TX and RX
    app_log_info("TX Err (#err/#TO)          : %0.3f%% (%d/%d)\n", statRelTXErr, relTXErr, relTXTimeOut);
    app_log_info("RX Err (#err/#TO/#CRC/#gap): %0.3f%% (%d/%d/%d/%d)\n", statRelRXErr, relRXErr, relRXTimeOut, relCRCErr, remainingRelRXGap);

    // Transmission rate
    if (isMaster)
    {
        app_log_info("Rate (loop 100)            : %0.2f msg/s (%0.2f ms)\n", statRelMsgPerSec, statRelMsgPerSecRef);
    }
    else
    {
        app_log_info("#Sync lost                 : %d\n", relSYNCErr);
        app_log_info("Rate                       : %0.2f msg/s\n", statRelMsgPerSec);
    }

    // Calibration
    app_log_info("Cal #request (#err)        : %d (%d)\n", relCalOk, relCalErr);

    // Slave detail
    app_log_info("\n");
    app_log_info("%s detail\n", (isMaster ? "Slaves" : "Slave"));
    app_log_info("-------------\n");
    if (isMaster)
    {
        // Master node --> take in account all slaves
        for (int i = 1; i < MAX_NODE; i++)
        {
            if (gRX_counter[i].u32 > 0UL)
            {
                app_log_info("Slave #%03d #cnt (#gap/max) : %d (%d/%d)\n",
                        i,
                        (gRX_tab[i][TAB_POS_RX_OK] - gRX_tab_old[i][TAB_POS_RX_OK]),
                        (gRX_tab[i][TAB_POS_RX_GAP] - gRX_tab_old[i][TAB_POS_RX_GAP]),
                        (gRX_tab[i][TAB_POS_RX_GAP_MAX] - gRX_tab_old[i][TAB_POS_RX_GAP_MAX]));
            }
        }
    }
    else    // Slave node -> take in account only the concerned slave
    {
        app_log_info("#Counter (#gap/max)        : %d (%d=%d gap-%d FErr / max: %d)\n", relRXOk, remainingRelRXGap, relRXGap, relRXErr, gRX_tab[myDevice][TAB_POS_RX_GAP_MAX]);
    }
#endif  // qREL_STAT

#if (qABS_STAT)
    // Absolute stat
    // -------------
    app_log_info("\nAbsolute\n");
    app_log_info("--------\n");
    app_log_info("Elapsed time               : %0.2f sec\n", relElapsedTime);

    // Counters TX and RX
    app_log_info("#Counter %s            : %d\n", (isMaster ? "      " : "      "), absTXOk);
    app_log_info("#Counter (%s)          : %d\n", (isMaster ? "Slaves" : "Master"), absRXOk);

    // Errors TX and RX
    app_log_info("TX Err (#err/#TO)          : %0.3f%% (%d/%d)\n", statAbsTXErr, absTXErr, absTXTimeOut);
    app_log_info("RX Err (#err/#TO/#CRC/#gap): %0.3f%% (%d/%d/%d/%d)\n", statAbsRXErr, absRXErr, absRXTimeOut, absCRCErr, remainingAbsRXGap);

    // Transmission rate
    if (isMaster)
    {
        app_log_info("Rate (loop 100)            : %0.2f msg/s (%0.2f ms)\n", statAbsMsgPerSec, statAbsMsgPerSecRef);
    }
    else
    {
        app_log_info("#Sync lost                 : %d\n", absSYNCErr);
        app_log_info("Rate                       : %0.2f msg/s\n", statAbsMsgPerSec);
    }

    // Calibration
    app_log_info("Cal #request (#err)        : %d (%d)\n", absCalOk, absCalErr);

    // Slave detail
    app_log_info("\n");
    app_log_info("%s detail\n", (isMaster ? "Slaves" : "Slave"));
    app_log_info("-------------\n");
    if (isMaster)
    {
        // Master node --> take in account all slaves
        for (int i = 1; i <= common_getNbrDeviceOfType(SLAVE_TYPE, ALL); i++)
        {
            if (common_getConfigTable(i)->enable)
            {
                uint8_t pos = common_getConfigTable(i)->posTab;
                uint8_t addr = common_getConfigTable(i)->internalAddr;
    //            if (gRX_counter[pos].u32 > 0UL)
    //            {
                    app_log_info("#%03d #cnt (#RX-TO/#gap/max): %d (%d/%d/%d)\n",
                            addr,
                            gRX_tab[pos][TAB_POS_RX_OK],
                            gRX_tab[pos][TAB_POS_TX_TIMEOUT],
                            gRX_tab[pos][TAB_POS_RX_GAP],
                            gRX_tab[pos][TAB_POS_RX_GAP_MAX]);
    //            }
            }
        }
    }
    else    // Slave node -> take in account only the concerned slave
    {
        app_log_info("#Counter (#gap/max)        : %d (%d=%d gap-%d FErr / max: %d)\n", absRXOk, remainingAbsRXGap, absRXGap, absRXErr, gRX_tab[myDevice][TAB_POS_RX_GAP_MAX]);
    }
#endif  // qABS_STAT

    // Detail of callback events
    DisplayEvents();

    // Update previous references
    memcpy(gTX_tab_old, gTX_tab, sizeof(gTX_tab));
    memcpy(gRX_tab_old, gRX_tab, sizeof(gRX_tab));
    memcpy(gCAL_tab_old, gCAL_tab, sizeof(gCAL_tab));
}
