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
        "SCHEDULED_TX_STARTED",
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
#define CONVERT_TO_VOLT_2(u)        ((float)u * 2.42f / 4095.0f)
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
 * DisplayEvents : print event counters
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
 * DisplayStat : Print and display statistics
 *****************************************************************************/
#if (qPrintStat)
static __INLINE void DisplayStat(void)
{
    bool isMaster = gDeviceCfgAddr->ismaster;
    uint8_t me = gDeviceCfgAddr->posTab;

    // Printing
    // --------
    // Print on serial COM

    app_log_info("\n");
    app_log_info("%s #%03d statistics\n", (isMaster ? "MASTER" : "SLAVE"), gDeviceCfgAddr->internalAddr);
    app_log_info("----------------------\n");
    app_log_info("Stat print #count          : %d\n", gCommonStat.CountPrintStat);


    // Absolute stat
    // -------------
    app_log_info("\nElapsed time               : %0.0f sec\n", gCommonStat.RelElapsedTime);

    // Counters TX and RX
    app_log_info("#Last counter to %s    : %d\n", (isMaster ? "Slaves" : "Master"), gCommonStat.TXCounter);
    app_log_info("#Last counter from %s  : %d\n", (isMaster ? "Slaves" : "Master"), gCommonStat.RXCounter);

    // Errors TX and RX
    app_log_info("TX Err (#err/#TO)          : %03.1f ppm/%0.3f%% (%d/%d)\n", gCommonStat.TXErrInPpm, gCommonStat.TXErrInPercent, gCommonStat.TXErr, gCommonStat.TXTimeOut);
    app_log_info("RX Err (#err/#TO/#CRC/#gap): %03.1f ppm/%0.3f%% (%d/%d/%d/%d)\n", gCommonStat.RXErrInPpm, gCommonStat.RXErrInPercent, gCommonStat.RXErr, gCommonStat.RXTimeOut, gCommonStat.CRCErr, gCommonStat.RXGap);
#if (RSSI_LQI_MES)
    // RX quality
    app_log_info("RX rssi/lqi (min/max)      : %d dbm (%d/%d) / %d (%d/%d)\n", gCommonStat.RssiMoy, gCommonStat.RssiMin, gCommonStat.RssiMax, gCommonStat.LqiMoy, gCommonStat.LqiMin, gCommonStat.LqiMax);
#endif  // RSSI_LQI_MES

    // Transmission rate
    if (isMaster)
    {
        app_log_info("Est. loop/100 (sync/rate)  : %0.2f ms (%0.3f ms/%0.2f msg/s)\n", gCommonStat.MsgPerSecRef, gCommonStat.Period, gCommonStat.MsgPerSec);
    }
    else
    {
        app_log_info("#Sync lost                 : %d\n", gCommonStat.SYNCErr);
        app_log_info("Sync period                : %0.3f ms\n", gCommonStat.Period);
        app_log_info("AD VDDA/IO/D/TCPU / SPI TA : %0.2fV/%0.2fV/%0.2fmA/%0.1f°C/%0.1f°C\n", CONVERT_TO_VOLT(gMboxADMes.u.detail.vdda), CONVERT_TO_VOLT_2(gMboxADMes.u.detail.ucell), CONVERT_TO_VOLT_2(gMboxADMes.u.detail.icell), CONVERT_TO_DEGRES_2(gMboxADMes.u.detail.internaltemp), CONVERT_TO_DEGRES(gMBoxTempCell));
    }

    // Calibration
    app_log_info("Cal #request (#err)        : %d (%d)\n", gCommonStat.CalOk, gCommonStat.CalErr);

    // Slave detail
    app_log_info("\n");
    app_log_info("%s detail\n", (isMaster ? "Slaves" : "Slave"));
    app_log_info("-------------\n");
    if (isMaster)
    {
        // Master node --> take in account all slaves
        for (int i = 1; i <= gNbOfSlave; i++)
        {
            PROT_AddrMap_t* device = common_getConfigTable(i);

            if (device != NULL)
            {
                if (device->enable)
                {
                    uint8_t pos = device->posTab;
                    uint8_t addr = device->internalAddr;

                    app_log_info("%03d Err (#OK/#TO/#gap/max) : %03.1f ppm (%d/%d/%d/%d)\n",
                            addr,
                            gCommonStat.SlaveDetail[pos].RXErrInPpm,
                            gCommonStat.SlaveDetail[pos].RXOk,
                            gCommonStat.SlaveDetail[pos].RXTimeOut,
                            gCommonStat.SlaveDetail[pos].RXGap,
                            gCommonStat.SlaveDetail[pos].GapMax);
                }
            }
        }
    }
    else    // Slave node -> take in account only the concerned slave
    {
        app_log_info("#Counter (#gap/max)        : %d (%d/%d)\n", gCommonStat.RXOk, gCommonStat.RXGap, gRX_tab[me][TAB_POS_RX_GAP_MAX]);
    }
}
#endif  // qPrintStat

/******************************************************************************
 * CalcStat : compute, print and display statistics
 *****************************************************************************/
__INLINE void CalcStat(void)
{
    uint32_t absRXGap = 0U;

    bool isMaster = gDeviceCfgAddr->ismaster;
    uint8_t me = gDeviceCfgAddr->posTab;

    // Common processing
    // -----------------
    gCommonStat.CountPrintStat = gCountPrintStat;
    gCommonStat.RelElapsedTime = gCountPrintStat;

    gCommonStat.RXCounter = 0U;
    gCommonStat.RXOk = 0U;

    // Compute sum of absolute RX (from Slave) counters and sum of relative (since last stat) RX (from Slave) counters
    gCommonStat.TXCounter = gTX_counter.u32;

    if (isMaster)
    {
        // Master node --> take in account all enabled slaves [1..N]
        for (int i = 1; i <= gNbOfSlave; i++)
        {
            PROT_AddrMap_t* device = common_getConfigTable(i);

            if (device != NULL)
            {
                if (device->enable)
                {
                    uint8_t pos = device->posTab;
                    if (gRX_counter[pos].u32 > 0UL)
                    {
                        // Absolute
                        // --------
                        gCommonStat.RXCounter += gRX_counter[pos].u32;                         // Sum of absolute RX (from Slave) counters
                        absRXGap += gRX_tab[pos][TAB_POS_RX_GAP];                 // Sum of absolute RX (from Slave) gap occurences
                        gCommonStat.RXOk += gRX_tab[pos][TAB_POS_RX_OK];                   // Sum of absolute TX Ok (from Slave) counters
                    }
                }

            }
        }
    }
    else    // Slave node -> take in account only the concerned slave
    {
        // Absolute
        // --------
        gCommonStat.RXCounter = gRX_counter[me].u32;
        absRXGap = gRX_tab[me][TAB_POS_RX_GAP];
        gCommonStat.RXOk = gRX_tab[me][TAB_POS_RX_OK];
    }

    uint32_t absAllCounter = gCommonStat.TXCounter + gCommonStat.RXCounter;


    // Absolute stat
    // -------------
    // Cumulative time
    gCommonStat.RelElapsedTime = (float)(gTotalElapsedTime / (SEC*1.0f));
    // Messages pro second
    gCommonStat.MsgPerSec = (float)(absAllCounter) / gCommonStat.RelElapsedTime;
    // Then compare to the key transmission rate of 1 master + 100 slaves @10ms
    uint8_t n_slaves = gNbOfEnabledSlave;
    float calc_sync_period = ((1000000.0f * (1.0f / gCommonStat.MsgPerSec)) * (n_slaves + 1));
    uint32_t th_sync_period = (TIME_SLOT_MASTER_TX + TIME_SLOT_ACQ + (gNbOfEnabledSlave * TIME_SLOT_SLAVE)  - TIME_SLOT_CORR);
    uint32_t deduc = TIME_SLOT_MASTER_TX + TIME_SLOT_ACQ + ((uint32_t)calc_sync_period - (th_sync_period + TIME_SLOT_CORR));
    gCommonStat.MsgPerSecRef = (((float)MAX_SLAVE * (float)((calc_sync_period - deduc) / (float)n_slaves)) + deduc) / 1000.0f;

    gCommonStat.TXOk = gTX_tab[TAB_POS_TX_OK];
    gCommonStat.CalOk = gCAL_tab[TAB_POS_CAL_REQ];
    gCommonStat.CalErr = gCAL_tab[TAB_POS_CAL_ERR];

    // TX error rate
    gCommonStat.TXErr = gTX_tab[TAB_POS_TX_ERR];
    gCommonStat.TXTimeOut = gTX_tab[TAB_POS_TX_TIMEOUT];

    gCommonStat.TXErrInPercent = 100.0f * (float) (gCommonStat.TXErr + gCommonStat.TXTimeOut) / (float) (isMaster ? (gCB_tab[RAIL_EVENT_TX_PACKET_SENT_SHIFT]-gCountPrintStat) : gCB_tab[RAIL_EVENT_TX_PACKET_SENT_SHIFT]);
    if (gCommonStat.TXErrInPercent > 100.0f)
        gCommonStat.TXErrInPercent = 100.0f;

    gCommonStat.TXErrInPpm = gCommonStat.TXErrInPercent * 10000.0f;

    // RX error rate
    gCommonStat.RXErr = gRX_tab[me][TAB_POS_RX_ERR];
    gCommonStat.RXTimeOut = gRX_tab[me][TAB_POS_RX_TIMEOUT];
    gCommonStat.Period = 1000.0f / ((float)gCommonStat.TXOk / (float)gCommonStat.RelElapsedTime);

    // Detect never responding slaves
    if (isMaster)   // From a master point of view
    {
        for (int i = 1; i <= gNbOfSlave; i++)
        {
            PROT_AddrMap_t* device = common_getConfigTable(i);

            if (device != NULL)
            {
                if (device->enable)
                {
                    uint8_t pos = device->posTab;
                    if (gRX_counter[pos].u32 == 0UL)
                    {
                        gCommonStat.RXTimeOut += gCommonStat.TXOk;
                    }
                }
            }
        }
    }
    else    // From a slave point of view
    {
        if (gCommonStat.TXOk == 0)
        {
            gCommonStat.TXTimeOut = gCommonStat.RXOk;
        }
    }

    gCommonStat.CRCErr = gRX_tab[me][TAB_POS_RX_CRC_ERR];
    gCommonStat.SYNCErr = gRX_tab[me][TAB_POS_RX_SYNC_LOST];
    gCommonStat.RXGap = ((absRXGap > gCommonStat.RXErr) ? (absRXGap - gCommonStat.RXErr): 0);            // Frame error implies a gap -> compute gap delta due to other reasons

    gCommonStat.RXErrInPercent = 100.0f * (float) (gCommonStat.RXErr + gCommonStat.RXTimeOut + gCommonStat.CRCErr + absRXGap) / (float) (isMaster ? gCB_tab[RAIL_EVENT_RX_PACKET_RECEIVED_SHIFT] : (gCB_tab[RAIL_EVENT_RX_PACKET_RECEIVED_SHIFT]-gCountPrintStat));
    if (gCommonStat.RXErrInPercent > 100.0f)
        gCommonStat.RXErrInPercent = 100.0f;

    gCommonStat.RXErrInPpm = gCommonStat.RXErrInPercent * 10000.0f;

#if (RSSI_LQI_MES)
    // RX quality
    gCommonStat.RssiMoy = (int8_t)gRX_tab[me][TAB_POS_RX_RSSI_MOY];
    gCommonStat.RssiMin = (int8_t)gRX_tab[me][TAB_POS_RX_RSSI_MIN];
    gCommonStat.RssiMax = (int8_t)gRX_tab[me][TAB_POS_RX_RSSI_MAX];
    gCommonStat.LqiMoy = (uint8_t)gRX_tab[me][TAB_POS_RX_LQI_MOY];
    gCommonStat.LqiMin = (uint8_t)gRX_tab[me][TAB_POS_RX_LQI_MIN];
    gCommonStat.LqiMax = (uint8_t)gRX_tab[me][TAB_POS_RX_LQI_MAX];
#endif  // RSSI_LQI_MES

    if (isMaster)
    {
        // Master node --> take in account all slaves
        for (int i = 1; i <= gNbOfSlave; i++)
        {
            PROT_AddrMap_t* device = common_getConfigTable(i);

            if (device != NULL)
            {
                if (device->enable)
                {
                    uint8_t pos = device->posTab;

                    gCommonStat.SlaveDetail[pos].RXOk = gRX_tab[pos][TAB_POS_RX_OK];
                    gCommonStat.SlaveDetail[pos].RXTimeOut = gRX_tab[pos][TAB_POS_TX_TIMEOUT];
                    gCommonStat.SlaveDetail[pos].RXGap = gRX_tab[pos][TAB_POS_RX_GAP];
                    gCommonStat.SlaveDetail[pos].GapMax = gRX_tab[pos][TAB_POS_RX_GAP_MAX];
                    gCommonStat.SlaveDetail[pos].RXErrInPpm = (1000000.0f *(gRX_tab[pos][TAB_POS_TX_TIMEOUT]+gRX_tab[pos][TAB_POS_RX_GAP]))/(float)(gRX_tab[pos][TAB_POS_RX_OK]+gRX_tab[pos][TAB_POS_TX_TIMEOUT]+gRX_tab[pos][TAB_POS_RX_GAP]);
                }
            }
        }
    }

    // Update previous references
    memcpy(gTX_tab_old, gTX_tab, sizeof(gTX_tab));
    memcpy(gRX_tab_old, gRX_tab, sizeof(gRX_tab));
    memcpy(gCAL_tab_old, gCAL_tab, sizeof(gCAL_tab));

    // Print out or send stat to LabVIEW (according DisplayStat implementation)
    DisplayStat();

    // Detail of callback events (only print out)
    DisplayEvents();
}

/******************************************************************************
 * StatInit : init statistics
 *****************************************************************************/
void StatInit(void)
{
    // RSSI and LQI min / max table
    for (int i = 0; i < MAX_NODE; i++)
    {
        gRX_tab[i][TAB_POS_RX_RSSI_MIN] = INT8_MAX;
        gRX_tab[i][TAB_POS_RX_RSSI_MAX] = INT8_MIN;

        gRX_tab[i][TAB_POS_RX_LQI_MIN] = UINT8_MAX;
        gRX_tab[i][TAB_POS_RX_LQI_MAX] = 0;
    }
}
