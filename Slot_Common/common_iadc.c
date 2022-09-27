/*
 * common_iadc.c
 *
 *  Created on: 14 sept. 2022
 *      Author: BEL-LinPat
 */

#ifndef COMMON_IADC_H_
#include "common_iadc.h"
#endif

#include "rail.h"
#include "app_assert.h"
#include "app_log.h"

#include "common_mbox.h"

// Set CLK_ADC to 10 MHz
#define CLK_SRC_ADC_FREQ    20000000  // CLK_SRC_ADC
#define CLK_ADC_FREQ        10000000  // CLK_ADC - 10 MHz max in normal mode

/*
 * Specify the IADC input using the IADC_PosInput_t typedef.  This
 * must be paired with a corresponding macro definition that allocates
 * the corresponding ABUS to the IADC.  These are...
 *
 * GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AEVEN0_ADC0
 * GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AODD0_ADC0
 * GPIO->BBUSALLOC |= GPIO_BBUSALLOC_BEVEN0_ADC0
 * GPIO->BBUSALLOC |= GPIO_BBUSALLOC_BODD0_ADC0
 * GPIO->CDBUSALLOC |= GPIO_CDBUSALLOC_CDEVEN0_ADC0
 * GPIO->CDBUSALLOC |= GPIO_CDBUSALLOC_CDODD0_ADC0
 *
 * ...for port A, port B, and port C/D pins, even and odd, respectively.
 */
#define IADC_INPUT_0_PORT_PIN     iadcPosInputPortCPin4;
#define IADC_INPUT_1_PORT_PIN     iadcPosInputPortCPin5;

#define IADC_INPUT_0_BUS          CDBUSALLOC
#define IADC_INPUT_0_BUSALLOC     GPIO_CDBUSALLOC_CDEVEN0_ADC0
#define IADC_INPUT_1_BUS          CDBUSALLOC
#define IADC_INPUT_1_BUSALLOC     GPIO_CDBUSALLOC_CDODD0_ADC0

#define IADC_TIMEOUT (20)

/**
 * Init IADC measure.
 */
void common_initIADC(void)
{
    // Declare initialization structures
    IADC_Init_t init = IADC_INIT_DEFAULT;
    IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
    IADC_InitScan_t initScan = IADC_INITSCAN_DEFAULT;

    // Scan table structure
    IADC_ScanTable_t scanTable = IADC_SCANTABLE_DEFAULT;

    CMU_ClockEnable(cmuClock_IADC0, true);

    // Use the FSRC0 as the IADC clock so it can run in EM2
    CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);

    // Shutdown between conversions to reduce current
    init.warmup = iadcWarmupNormal;

    // Set the HFSCLK prescale value here
    init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

    /*
     * Configuration 0 is used by both scan and single conversions by
     * default.  Use internal bandgap as the reference and specify the
     * reference voltage in mV.
     *
     * Resolution is not configurable directly but is based on the
     * selected oversampling ratio (osrHighSpeed), which defaults to
     * 2x and generates 12-bit results.
     */
    initAllConfigs.configs[0].reference = iadcCfgReferenceInt1V2;
    initAllConfigs.configs[0].vRef = 1210;
    initAllConfigs.configs[0].osrHighSpeed = iadcCfgOsrHighSpeed2x;
    initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain0P5x;

    /*
     * CLK_SRC_ADC must be prescaled by some value greater than 1 to
     * derive the intended CLK_ADC frequency.
     *
     * Based on the default 2x oversampling rate (OSRHS)...
     *
     * conversion time = ((4 * OSRHS) + 2) / fCLK_ADC
     *
     * ...which results in a maximum sampling rate of 833 ksps with the
     * 2-clock input multiplexer switching time is included.
     */
    initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                       CLK_ADC_FREQ,
                                                                       0,
                                                                       iadcCfgModeNormal,
                                                                       init.srcClkPrescale);

    /*
     * Trigger continuously once scan is started.  Note that
     * initScan.start = false by default, so conversions must be started
     * manually with IADC_command(IADC0, iadcCmdStartScan).
     *
     * Set the SCANFIFODVL flag when scan FIFO holds 2 entries.  In this
     * example, the interrupt associated with the SCANFIFODVL flag in
     * the IADC_IF register is not used.
     *
     * Enable DMA wake-up to save the results when the specified FIFO
     * level is hit.
     */
    initScan.triggerAction = /*iadcTriggerActionContinuous*/ iadcTriggerActionOnce;
    initScan.dataValidLevel = iadcFifoCfgDvl3;
    initScan.fifoDmaWakeup = true;
    initScan.showId = true;

    // Check if commonIADCIndex_t and IADC configuration have the same size
    app_assert(initScan.dataValidLevel == (uint8_t)endofIADCmeas, "IADC Error: data tab and IADC number of acq is not equal (IADC:%d TAB:%d)\n", initScan.dataValidLevel, endofIADCmeas);

    /*
     * Configure entries in the scan table.  CH0 is single-ended from
     * input 0; CH1 is single-ended from input 1.
     */
    scanTable.entries[0].posInput = iadcPosInputAvdd;      // Add AVDD to scan for demonstration purposes
    scanTable.entries[0].negInput = iadcNegInputGnd | 1;   // When measuring a supply, PINNEG must be odd (1, 3, 5,...)
    scanTable.entries[0].includeInScan = true;

    scanTable.entries[1].posInput = IADC_INPUT_0_PORT_PIN;
    scanTable.entries[1].negInput = iadcNegInputGnd;
    scanTable.entries[1].includeInScan = true;

    scanTable.entries[2].posInput = IADC_INPUT_1_PORT_PIN;
    scanTable.entries[2].negInput = iadcNegInputGnd;
    scanTable.entries[2].includeInScan = true;

    // Initialize IADC
    IADC_init(IADC0, &init, &initAllConfigs);

    // Initialize scan
    IADC_initScan(IADC0, &initScan, &scanTable);

    // Allocate the analog bus for ADC0 inputs
    GPIO->IADC_INPUT_0_BUS |= IADC_INPUT_0_BUSALLOC;
    GPIO->IADC_INPUT_1_BUS |= IADC_INPUT_1_BUSALLOC;
}

/**
 * Return IADC conversion status.
 *
 * @return  true conversion terminated.
 */
bool common_isIADCready(void)
{
    // Wait for conversion to be complete (combined status bits 8 & 6 don't equal 1 and 0 respectively)
    return ((IADC0->STATUS & (_IADC_STATUS_CONVERTING_MASK | _IADC_STATUS_SINGLEFIFODV_MASK)) == IADC_STATUS_SINGLEFIFODV);
}

/**
 * Start IADC conversion.
 *
 * @return  --.
 */
void common_startIADC(void)
{
    // Start IADC
    IADC_command(IADC0, iadcCmdStartScan);
}

/**
 * Make IADC measure, wait until conversion is terminated or timeout.
 * In case of success save data to mbox measure structure
 *
 * @return  TRUE if data were saved in mbox, FALSE if timeout.
 */
bool common_getIADCdata(void)
{
    RAIL_Time_t startTime = RAIL_GetTime();
    RAIL_Time_t gap;

    // Wait conversion end or timeout
    while ((IADC0->STATUS & (_IADC_STATUS_CONVERTING_MASK | _IADC_STATUS_SCANFIFODV_MASK)) != IADC_STATUS_SCANFIFODV)
    {
        //while combined status bits 8 & 6 don't equal 1 and 0 respectively
        gap = RAIL_GetTime() - startTime;
        if (gap > IADC_TIMEOUT)
        {
            return false;
        }
    }

    // Get ADC result
    IADC_Result_t sample;
    while (IADC_getScanFifoCnt(IADC0))
    {
        // Pull a scan result from the FIFO
        sample = IADC_pullScanFifoResult(IADC0);

        if (sample.id <= endofIADCmeas)
        {
            gMboxADMes.u.dataTab[sample.id] = sample.data;
        }
        else
        {
            app_assert(false, "Invalid AD id (out of range).");
        }

    }

    // convert in Celsius ((float) ((gMboxADMes[internalTemp].u.detail.internaltemp) >> _EMU_TEMP_TEMPLSB_SHIFT) ) / 4.0f - EMU_TEMP_ZERO_C_IN_KELVIN /*273.4*/);
    gMboxADMes.u.detail.internaltemp  = ((EMU->TEMP & (_EMU_TEMP_TEMP_MASK | _EMU_TEMP_TEMPLSB_MASK) ) >> _EMU_TEMP_TEMPLSB_SHIFT);

    return true;
}
