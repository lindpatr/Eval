/*
 * common_mbox.h
 *
 *  Created on: 14 sept. 2022
 *      Author: BEL-LinPat
 */

#ifndef COMMON_MBOX_H_
#define COMMON_MBOX_H_

#include <stdint.h>

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

#endif /* COMMON_MBOX_H_ */
