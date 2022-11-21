/*
 * common_mbox.h
 *
 *  Created on: 14 sept. 2022
 *      Author: BEL-LinPat
 */

#ifndef COMMON_MBOX_H_
#define COMMON_MBOX_H_

#include <stdint.h>
#include "common_config.h"

typedef enum
{
    iadcVDDA = 0,                     // Internal ADC ref (SUPPLY)
    iadcicell,                        // Cell current
    iadcucell,                        // Cell voltage
    endofIADCmeas = iadcucell,        // ---- index of last IADC measure
    internalTemp,                     // Internal MCU temperature (not a IADC value)
    endofmeastab                      // ---- size of the measure tab

}commonMeasureIndex_t;

typedef struct commonMeasure
{
    uint32_t    vdda;
    uint32_t    icell;
    uint32_t    ucell;
    uint32_t    internaltemp;   // mesure brute
}commonMeasure_t;

typedef struct commonAnalogdata
{
    union
    {
        commonMeasure_t     detail;
        uint32_t            dataTab[endofmeastab];
    }u;
}commonAnalogdata_t;

extern commonAnalogdata_t gMboxADMes;
extern int32_t gMBoxTempCell;

typedef struct
{
    // RX
    uint32_t RXCounter;         // Sum of RX (from Slave) counters
    uint32_t RXOk;              // Count of RX Ok
    uint32_t RXTimeOut;         // Count of RX timeout
    uint32_t RXGap;             // Count of RX gap occurences
    uint32_t GapMax;            // Max gap discovered
    float RXErrInPpm;           // RX error rate [ppm]
    int8_t RssiMoy;             // Average RSSI
    int8_t RssiMin;             // Min RSSI
    int8_t RssiMax;             // Max RSSI
    uint8_t LqiMoy;             // Average LQI
    uint8_t LqiMin;             // Min LQI
    uint8_t LqiMax;             // Max LQI

} commonStatSlaveDetail_t;

typedef struct
{
    // General
    uint32_t CountPrintStat;    // Stat print #count
    float RelElapsedTime;       // Elapsed time

    // TX
    uint32_t TXCounter;         // Last counter Master to Slave
    uint32_t TXOk;              // Count of Ok
    uint32_t TXErr;             // Count of TX errors
    uint32_t TXTimeOut;         // Count of TX timeout

    float TXErrInPpm;           // TX error rate [ppm]
    float TXErrInPercent;       // TX error rate [%]

    // RX
    uint32_t RXCounter;         // Sum of RX (from Slave) counters
    uint32_t RXOk;              // Count of RX Ok
    uint32_t RXErr;             // Count of RX errors
    uint32_t RXTimeOut;         // Count of RX timeout
    uint32_t CRCErr;            // Count of CRC errors
    uint32_t SYNCErr;           // Count of SYNC lost (Slave only)
    uint32_t RXGap;             // Sum of RX gap occurences

    float RXErrInPpm;           // RX error rate [ppm]
    float RXErrInPercent;       // RX error rate [%]

    int8_t RssiMoy;             // Average RSSI
    int8_t RssiMin;             // Min RSSI
    int8_t RssiMax;             // Max RSSI
    uint8_t LqiMoy;             // Average LQI
    uint8_t LqiMin;             // Min LQI
    uint8_t LqiMax;             // Max LQI

    commonStatSlaveDetail_t SlaveDetail[MAX_NODE];  // See commonStatSlaveDetail_t (Master only)

    // Rate
    float MsgPerSec;            // Num of messages / sec (Master only)
    float Period;               // Period beetween 2 sync request TX
    float MsgPerSecRef;         // Estimation of the loop period for 100 slaves (Master only)

    // Radio Calibration
    uint32_t CalOk;             // Num of calibraition requests
    uint32_t CalErr;            // Num of calibration errors

} commonStat_t;

extern commonStat_t gCommonStat;

#endif /* COMMON_MBOX_H_ */
