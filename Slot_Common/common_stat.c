/*
 * app_stat.c
 *
 *  Created on: 8 sept. 2022
 *      Author: BEL-LinPat
 */

#include "common_stat.h"
#include "common_mbox.h"
#include "common_algo_stuffing.h"

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
 * DisplayStat : ASCII TERMINAL OUTPUT
 *****************************************************************************/
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
 * DisplayStat : LABVIEW BINARY FRAMED OUTPUT (Antenna test)
 *****************************************************************************/
#if (qPrintStatLabView)
// Number values always sent (1..MAX_LABVIEW_DATA).
#define MAX_LABVIEW_DATA    28
// Initial ID for dynamic events values
#define EVENT_INITIAL_ID    128
// Size of temporary buffer
#define MAX_LABVIEW_BUFFER  (MAX_LABVIEW_DATA + SIZE_UINT64_IN_BITS)

enum
{
    kUint32 = 1,
    kFLoat = 2,
    kInt8 = 3,
    kUint8 = 4
};

typedef struct
{
    uint16_t    ID;         // Value identifier for labview
    uint16_t    valType;    // Data type (uint32_t, float...)
    uint32_t    value;      // Value cast in uint32_t
}SerialFrame;

// Encoded buffer to send
uint8_t buffer[((MAX_LABVIEW_BUFFER) * sizeof(SerialFrame) * 2) + 5];

// Frame structure
static SerialFrame  statFrame[MAX_LABVIEW_BUFFER];
// stat data pointer
static uint32_t* statDataPtr[MAX_LABVIEW_BUFFER];

// Initialize Frame tab and data ptr tab
#define setvalue(index, sframe, id, valtype, initdata, dataPtrTab, dataptr)    ({sframe[index].ID = id; sframe[index].valType =valtype; sframe[index].value = 0; dataPtrTab[index] = (uint32_t*)dataptr;})

/******************************************************************************
 * InitLabViewStat : initialize tabs for LabView statistics
 *****************************************************************************/
static __INLINE void InitLabViewStat()
{
    setvalue(0, statFrame, 1, kUint32, 0, statDataPtr, &gCommonStat.CountPrintStat);  //uint32_t gCommonStat.CountPrintStat;    // Stat print #count
    setvalue(1, statFrame, 2, 2,  0, statDataPtr, &gCommonStat.RelElapsedTime);  //float gCommonStat.RelElapsedTime;      // Elapsed time

    // TX
    setvalue(2, statFrame, 3, kUint32,  0, statDataPtr, &gCommonStat.TXCounter);       //uint32_t gCommonStat.TXCounter;         // Last counter Master to Slave
    setvalue(3, statFrame, 4, kUint32,  0, statDataPtr, &gCommonStat.TXOk);            //uint32_t gCommonStat.TXOk;              // Count of Ok
    setvalue(4, statFrame, 5, kUint32,  0, statDataPtr, &gCommonStat.TXErr);           //uint32_t gCommonStat.TXErr;             // Count of TX errors
    setvalue(5, statFrame, 6, kUint32,  0, statDataPtr, &gCommonStat.TXTimeOut);       //uint32_t gCommonStat.TXTimeOut;         // Count of TX timeout

    setvalue(6, statFrame, 7, kFLoat,  0, statDataPtr, &gCommonStat.TXErrInPpm);      //float gCommonStat.TXErrInPpm;           // TX error rate [ppm]
    setvalue(7, statFrame, 8, kFLoat,  0, statDataPtr, &gCommonStat.TXErrInPercent);  //float gCommonStat.TXErrInPercent;       // TX error rate [%]

    // RX
    setvalue(8, statFrame, 9, kUint32,  0, statDataPtr, &gCommonStat.RXCounter);   //uint32_t gCommonStat.RXCounter;         // Sum of RX (from Slave) counters
    setvalue(9, statFrame, 10, kUint32,  0, statDataPtr, &gCommonStat.RXOk);        //uint32_t gCommonStat.RXOk;              // Count of RX Ok
    setvalue(10, statFrame, 11, kUint32,  0, statDataPtr, &gCommonStat.RXErr);       //uint32_t gCommonStat.RXErr;             // Count of RX errors
    setvalue(11, statFrame, 12, kUint32,  0, statDataPtr, &gCommonStat.RXTimeOut);   //uint32_t gCommonStat.RXTimeOut;         // Count of RX timeout
    setvalue(12, statFrame, 13, kUint32,  0, statDataPtr, &gCommonStat.CRCErr);      //uint32_t gCommonStat.CRCErr;            // Count of CRC errors
    setvalue(13, statFrame, 14, kUint32,  0, statDataPtr, &gCommonStat.SYNCErr);     //uint32_t gCommonStat.SYNCErr;           // Count of SYNC lost (Slave only)
    setvalue(14, statFrame, 15, kUint32,  0, statDataPtr, &gCommonStat.RXGap);       //uint32_t gCommonStat.RXGap;             // Sum of RX gap occurences

    setvalue(15, statFrame, 16, kFLoat,  0, statDataPtr, &gCommonStat.RXErrInPpm);      //float gCommonStat.RXErrInPpm;           // RX error rate [ppm]
    setvalue(16, statFrame, 17, kFLoat,  0, statDataPtr, &gCommonStat.RXErrInPercent);  //float gCommonStat.RXErrInPercent;       // RX error rate [%]

    setvalue(17, statFrame, 18, kInt8,  0, statDataPtr, &gCommonStat.RssiMoy);     //int8_t gCommonStat.RssiMoy;             // Average RSSI
    setvalue(18, statFrame, 19, kInt8,  0, statDataPtr, &gCommonStat.RssiMin);     //int8_t gCommonStat.RssiMin;             // Min RSSI
    setvalue(19, statFrame, 20, kInt8,  0, statDataPtr, &gCommonStat.RssiMax);     //int8_t gCommonStat.RssiMax;             // Max RSSI
    setvalue(20, statFrame, 21, kUint8,  0, statDataPtr, &gCommonStat.LqiMoy);      //uint8_t gCommonStat.LqiMoy;             // Average LQI
    setvalue(21, statFrame, 22, kUint8,  0, statDataPtr, &gCommonStat.LqiMin);      //uint8_t gCommonStat.LqiMin;             // Min LQI
    setvalue(22, statFrame, 23, kUint8,  0, statDataPtr, &gCommonStat.LqiMax);      //uint8_t gCommonStat.LqiMax;             // Max LQI

    // Rate
    setvalue(23, statFrame, 24, kFLoat,  0, statDataPtr, &gCommonStat.MsgPerSec);       //float gCommonStat.MsgPerSec;            // Num of messages / sec (Master only)
    setvalue(24, statFrame, 25, kFLoat,  0, statDataPtr, &gCommonStat.Period);          //float gCommonStat.Period;               // Period beetween 2 sync request TX
    setvalue(25, statFrame, 26, kFLoat,  0, statDataPtr, &gCommonStat.MsgPerSecRef);    //float gCommonStat.MsgPerSecRef;         // Estimation of the loop period for 100 slaves (Master only)

    // Radio Calibration
    setvalue(26, statFrame, 27, kUint32,  0, statDataPtr, &gCommonStat.CalOk);           //uint32_t gCommonStat.CalOk;             // Num of calibraition requests
    setvalue(27, statFrame, 28, kUint32,  0, statDataPtr, &gCommonStat.CalErr);          //uint32_t gCommonStat.CalErr;            // Num of calibration errors

    // NOT USED
    //    // See commonStatSlaveDetail_t (Master only)
    //    for (int i = 0; i < MAX_NODE; i++)
    //    {
    //        // RX
    //        statFrame[x] = {1, 1, gCommonStat.RXCounter};   //uint32_t RXCounter;         // Sum of RX (from Slave) counters
    //        statFrame[x] = {1, 1, gCommonStat.RXOk};        //uint32_t RXOk;              // Count of RX Ok
    //        statFrame[x] = {1, 1, gCommonStat.RXTimeOut};   //uint32_t RXTimeOut;         // Count of RX timeout
    //        statFrame[x] = {1, 1, gCommonStat.RXGap};       //uint32_t RXGap;             // Count of RX gap occurences
    //        statFrame[x] = {1, 1, gCommonStat.GapMax};      //uint32_t GapMax;            // Max gap discovered
    //        statFrame[x] = {1, 2, gCommonStat.RXErrInPpm};  //float RXErrInPpm;           // RX error rate [ppm]
    //    }
}

/******************************************************************************
 * DisplayStat : Format, encode and send statistics to LabView
 *****************************************************************************/
static __INLINE void DisplayStat(void)
{
    // Fill buffer with first MAX_LABVIEW_DATA static ID
    for (int idx = 0; idx < MAX_LABVIEW_DATA; idx++)
    {
        statFrame[idx].value = *statDataPtr[idx];
    }

    // Fill buffer with dynamic events values, added only if greater than 0
    uint8_t index = MAX_LABVIEW_DATA;
    uint8_t eventAdded = 0;
    for (int idx = 0; idx < SIZE_UINT64_IN_BITS; idx++)
    {
        if (gCB_tab[idx] >0)
        {
            statFrame[index].ID = EVENT_INITIAL_ID + idx;
            statFrame[index].valType = kUint32;
            statFrame[index].value = gCB_tab[idx];
            index++;
            eventAdded++;
        }
    }

    uint32_t len = base252_encode((uint8_t*)statFrame, sizeof(SerialFrame) * (MAX_LABVIEW_DATA + eventAdded), buffer);

    if (len != 0)
    {
        sl_status_t status = sl_iostream_write(sl_iostream_vcom_handle, buffer, len);

        if (status == SL_STATUS_IDLE)
        {
            status = 23;
        }
    }

}

#endif // qPrintStatLabView

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

#if (qPrintStat)
    // Detail of callback events (only print out)
    DisplayEvents();
#endif
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

#if (qPrintStatLabView)
    // ==================================
    // ATTENTION : Il faut désactiver l'option "Convert \n to \r\n" dans le driver VCOM pour que cette transmission (binaire) fonctionne
    // ==================================
    sl_iostream_uart_set_auto_cr_lf(sl_iostream_uart_vcom_handle, false);
    InitLabViewStat();
#endif // qPrintStatLabView

#if (qPrintStat)
    // ==================================
    // ATTENTION : Il faut activer l'option "Convert \n to \r\n" dans le driver VCOM pour que le print (mode text)
    //             fonctionne sur tous les terminaux
    // ==================================
    sl_iostream_uart_set_auto_cr_lf(sl_iostream_uart_vcom_handle, true);
#endif // qPrintStat

}
