/*
 * common_iadc.h
 *
 *  Created on: 14 sept. 2022
 *      Author: BEL-LinPat
 */

#ifndef COMMON_IADC_H_
#define COMMON_IADC_H_

#include <stdint.h>

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_iadc.h"


  // Start scan
#define common_IadcStart() IADC_command(IADC0, iadcCmdStartScan)

/**
 * Init IADC measure.
 */
void common_initIADC(void);

/**
 * Return IADC conversion status.
 *
 * @return  true conversion terminated.
 */
bool common_isIADCready(void);

/**
 * Make IADC measure, wait until conversion is terminated or timeout.
 * In case of success save data to mbox measure structure
 *
 * @return  TRUE if data were saved in mbox, FALSE if timeout.
 */
bool common_getIADCdata(void);

#endif /* COMMON_IADC_H_ */
