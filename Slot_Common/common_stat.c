/*
 * app_stat.c
 *
 *  Created on: 8 sept. 2022
 *      Author: BEL-LinPat
 */


#include "common_stat.h"
#include "common_mbox.h"


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


#define CONVERT_TO_DEGRES(temp)     ((float)temp/1000.0f)
#define CONVERT_TO_DEGRES_2(temp)   (((float)temp/4.0f) - 273.4f)
#define CONVERT_TO_VOLT(u)          ((float)u * 4.0f * 2.42f / 4095.0f)

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
    // Absolute
    // --------
    uint32_t absRXCounter = 0U;
    uint32_t absRXGap = 0U;
    uint32_t absRXOk = 0U;

    float statAbsMsgPerSec, statAbsMsgPerSecRef, statAbsTXErr, statAbsRXErr;

    float relElapsedTime;
    bool isMaster = gDeviceCfgAddr->ismaster;
    uint8_t myDevice = gDeviceCfgAddr->posTab;


    // Common processing
    // -----------------

    // Compute sum of absolute RX (from Slave) counters and sum of relative (since last stat) RX (from Slave) counters
    uint32_t absTXCounter = gTX_counter.u32;

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
                    // Absolute
                    // --------
                    absRXCounter += gRX_counter[i].u32;                         // Sum of absolute RX (from Slave) counters
                    absRXGap += gRX_tab[i][TAB_POS_RX_GAP];                 // Sum of absolute RX (from Slave) gap occurences
                    absRXOk += gRX_tab[i][TAB_POS_RX_OK];                   // Sum of absolute TX Ok (from Slave) counters
                }
            }
        }
    }
    else    // Slave node -> take in account only the concerned slave
    {
        // Absolute
        // --------
        absRXCounter = gRX_counter[myDevice].u32;
        absRXGap = gRX_tab[myDevice][TAB_POS_RX_GAP];
        absRXOk = gRX_tab[myDevice][TAB_POS_RX_OK];
    }

    uint32_t absAllCounter = absTXCounter + absRXCounter;


    // Absolute stat
    // -------------
    // Cumulative time
    relElapsedTime = (float)((double) (gTotalElapsedTime) / (float)(SEC));
    // Messages pro second
    statAbsMsgPerSec = (float)(absAllCounter) / relElapsedTime;
    // Then compare to the key transmission rate of 1 master + 100 slaves @10ms
    //statAbsMsgPerSecRef = REF_MSG_PER_SEC / statAbsMsgPerSec;
    uint8_t n_slaves = common_getNbrDeviceOfType(SLAVE_TYPE, ENABLED);
    float calc_sync_period = ((1000000.0f * (1.0f / statAbsMsgPerSec)) * (n_slaves + 1));
    uint32_t th_sync_period = (TIME_SLOT_MASTER_TX + TIME_SLOT_ACQ + (common_getNbrDeviceOfType(SLAVE_TYPE, ENABLED) * TIME_SLOT_SLAVE)  - TIME_SLOT_CORR);
    uint32_t deduc = TIME_SLOT_MASTER_TX + TIME_SLOT_ACQ + ((uint32_t)calc_sync_period - (th_sync_period + TIME_SLOT_CORR))/* = TIME_SLOT_RES*/;
    statAbsMsgPerSecRef = (((float)MAX_SLAVE * (float)((calc_sync_period - deduc) / (float)n_slaves)) + deduc) / 1000.0f;

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
    float period = 1000.0f / ((float)absTXOk / (float)relElapsedTime);

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

    // RX quality
    int8_t statAbsRssiMoy = (int8_t)gRX_tab[myDevice][TAB_POS_RX_RSSI_MOY];
    int8_t statAbsRssiMin = (int8_t)gRX_tab[myDevice][TAB_POS_RX_RSSI_MIN];
    int8_t statAbsRssiMax = (int8_t)gRX_tab[myDevice][TAB_POS_RX_RSSI_MAX];
    uint8_t statAbsLqiMoy = (uint8_t)gRX_tab[myDevice][TAB_POS_RX_LQI_MOY];
    uint8_t statAbsLqiMin = (uint8_t)gRX_tab[myDevice][TAB_POS_RX_LQI_MIN];
    uint8_t statAbsLqiMax = (uint8_t)gRX_tab[myDevice][TAB_POS_RX_LQI_MAX];

    // Printing
    // --------
    // Print on serial COM

    app_log_info("\n");
    app_log_info("%s #%03d statistics\n", (isMaster ? "MASTER" : "SLAVE"), gDeviceCfgAddr->internalAddr);
    app_log_info("----------------------\n");
    app_log_info("Stat print #count          : %d\n", gCountPrintStat);


    // Absolute stat
    // -------------
    app_log_info("\nElapsed time               : %0.2f sec\n", relElapsedTime);

    // Counters TX and RX
    app_log_info("#Counter %s            : %d\n", (isMaster ? "      " : "      "), absTXOk);
    app_log_info("#Counter (%s)          : %d\n", (isMaster ? "Slaves" : "Master"), absRXOk);

    // Errors TX and RX
    app_log_info("TX Err (#err/#TO)          : %03.1f ppm/%0.3f%% (%d/%d)\n", statAbsTXErr * 10000.0f, statAbsTXErr, absTXErr, absTXTimeOut);
    app_log_info("RX Err (#err/#TO/#CRC/#gap): %03.1f ppm/%0.3f%% (%d/%d/%d/%d)\n", statAbsRXErr * 10000.0f, statAbsRXErr, absRXErr, absRXTimeOut, absCRCErr, remainingAbsRXGap);
    // RX quality
    app_log_info("RX rssi/lqi (min/max)      : %d dbm (%d/%d) / %d (%d/%d)\n", statAbsRssiMoy, statAbsRssiMin, statAbsRssiMax, statAbsLqiMoy, statAbsLqiMin, statAbsLqiMax);

    // Transmission rate
    if (isMaster)
    {
        app_log_info("Est. loop/100 (sync/rate)  : %0.2f ms (%0.3f ms/%0.2f msg/s)\n", statAbsMsgPerSecRef, period, statAbsMsgPerSec);
    }
    else
    {
        app_log_info("#Sync lost                 : %d\n", absSYNCErr);
        app_log_info("Sync period                : %0.3f ms\n", period);
        app_log_info("AD VDDA/IO/D/TCPU / I2C TA : %0.2fV/%0.2fV/%0.2fV/%0.1f°C/%0.1f°C\n", CONVERT_TO_VOLT(gMboxADMes.u.detail.vdda), CONVERT_TO_VOLT(gMboxADMes.u.detail.ucell), CONVERT_TO_VOLT(gMboxADMes.u.detail.icell), CONVERT_TO_DEGRES_2(gMboxADMes.u.detail.internaltemp), CONVERT_TO_DEGRES(gMBoxTempCell));
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
                    app_log_info("%03d Err (#cnt/#TO/#gap/max): %03.1f ppm (%d/%d/%d/%d)\n",
                            addr,
                            (1000000.0f *(gRX_tab[pos][TAB_POS_TX_TIMEOUT]+gRX_tab[pos][TAB_POS_RX_GAP]))/(float)gRX_tab[pos][TAB_POS_RX_OK],
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

    // Detail of callback events
    DisplayEvents();

    // Update previous references
    memcpy(gTX_tab_old, gTX_tab, sizeof(gTX_tab));
    memcpy(gRX_tab_old, gRX_tab, sizeof(gRX_tab));
    memcpy(gCAL_tab_old, gCAL_tab, sizeof(gCAL_tab));
}

/******************************************************************************
 * StatInit : init statistics
 *****************************************************************************/
void StatInit(void)
{
    for (int i = 0; i < MAX_NODE; i++)
    {
        gRX_tab[i][TAB_POS_RX_RSSI_MIN] = 127;
        gRX_tab[i][TAB_POS_RX_RSSI_MAX] = -128;

        gRX_tab[i][TAB_POS_RX_LQI_MIN] = 255;
        gRX_tab[i][TAB_POS_RX_LQI_MAX] = 0;
    }
}
