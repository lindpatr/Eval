/*
 * tmp126_spi.c
 *
 *  Created on: 17 oct. 2022
 *      Author: BEL-LinPat
 */

#include "common_tmp126_spi.h"
#include "common_spi.h"
#include "limits.h"

// Size of the data buffers
#define BUFLEN  20

// Outgoing data
uint8_t txbuf[BUFLEN];

// Incoming data
uint8_t rxbuf[BUFLEN];


/**
 * @brief
 * Init SPI TMP126.
 *
 * @param[in] tempLimLow    Temp limit low [°C]
 * @param[in] tempLimHigh   Temp limit high [°C]
 *
 * @return true if successful, false if timeout
 */
bool spi_tmp126_init(DeviceIdentEnum_t device, float tempLimLow, float tempLimHigh)
{
    // Command word
    CmdWordUnion cmd =
    {
        .CmdTmp126.r_w = 0,
        .CmdTmp126.addrInc = 1,         // Addr auto increment
        .CmdTmp126.len = 0,
        .CmdTmp126.crc = 0,
        .CmdTmp126.reserved = 0,
        .CmdTmp126.regAddr = 0x03,      // Initial register
    };

    txbuf[0] = cmd.bytes[1];         // X - CRC - L4 L3 L2 L1 - A - R/W - ADDR
    txbuf[1] = cmd.bytes[0];         // x -  0  - 0  0  0  0  - 1 -   0 - 0x03

    CmdWordUnion conf =
    {
        .ConfTmp126.conv_period = 1,    // 001b = 31.25 ms / 32 Hz
        .ConfTmp126.mode = 0,
        .ConfTmp126.one_shot = 0,
        .ConfTmp126.int_comp = 1,       // 1b = Comparator mode
        .ConfTmp126.reserved2 = 0,
        .ConfTmp126.avg = 0,
        .ConfTmp126.reset = 0,
        .ConfTmp126.reserved = 0,
    };

    // reg = 0x03 -- Configuration
    txbuf[2] = conf.bytes[1];        //      reserved     Reset - AVG - Res - Int_Cmp - 1Shot - Mode - Conv Period
    txbuf[3] = conf.bytes[0];        // 0x21 xxxxxxx      0       0     0     1         0       1      001

    CmdWordUnion alert =
    {
        .Alert_En.dataready_alert = 0,
        .Alert_En.TLow_alert = 1,       // When in comparator mode, enables the THigh_Status to assert the ALERT
        .Alert_En.THigh_alert = 1,      // When in comparator mode, enables the TLow_Status to assert the ALERT
        .Alert_En.slew_alert = 0,
        .Alert_En.crc_alert = 0,
        .Alert_En.reserved2 = 0,
    };

    // reg = 0x04 -- Alert_Enable
    txbuf[4] = alert.bytes[1];       //      reserved     CRC - Slew - THigh - TLow - DataRdy
    txbuf[5] = alert.bytes[0];       // 0x06 xxxxxxxxxxx  0     0      1       1      0

    // LIM LOW = 0x05 LSB = 0.03125°C
    uint16_t tempL = ((int16_t)(tempLimLow / 0.03125)) << 2;
    txbuf[6] = (uint8_t)(tempL >> 8);;    //
    txbuf[7] = (uint8_t)(tempL & 0x00FF);

    // LIM HIGH = 0x06 LSB = 0.03125°C
    uint16_t tempH = ((int16_t)(tempLimHigh / 0.03125)) << 2;
    txbuf[8] = (uint8_t)(tempH >> 8);    // 30 deg / 0.03125
    txbuf[9] = (uint8_t)(tempH & 0x00FF);

    // Hyst High 0.5 °C * 0x0Axx
    // Hyst High 0.5 °C * 0xxx0A
    txbuf[10] = 0x0A;
    txbuf[11] = 0x0A;

    common_startSPItransfert(device, 12, (uint8_t*)txbuf, (uint8_t*)rxbuf);
    return common_waitSPITransfertDone(device);
}

/**
 * @brief
 * Return TMP126 product ID.
 *
 * @return product ID if successful, 0 if timeout
 */
uint16_t spi_tmp126_getID(DeviceIdentEnum_t device)
{
    // Command word
    CmdWordUnion cmd =
    {
        .CmdTmp126.r_w = 1,
        .CmdTmp126.addrInc = 0,
        .CmdTmp126.len = 0,
        .CmdTmp126.crc = 0,
        .CmdTmp126.reserved = 0,
        .CmdTmp126.regAddr = 0x0C,   // ID register
    };

    txbuf[0] = cmd.bytes[1];         // X - CRC - L4 L3 L2 L1 - A - R/W - ADDR
    txbuf[1] = cmd.bytes[0];         // x -  0  - 0  0  0  0  - 0 -   1 - 0x0C

    txbuf[2] = 0xFF;    // FF
    txbuf[3] = 0xFF;    // FF

    common_startSPItransfert(device, 4, (uint8_t*)txbuf, (uint8_t*)rxbuf);

    bool success = common_waitSPITransfertDone(device);

    if (success)
    {
        return ((rxbuf[2] << 8) + rxbuf[3]);
    }

    return 0;
}

/**
 * @brief
 * Return temperature measure.
 *
 * @return temperature [AD value] if successful, UINT_MAX if timeout
 */
uint16_t spi_tmp126_getTemp(DeviceIdentEnum_t device)
{
    // Command word
    CmdWordUnion cmd =
    {
        .CmdTmp126.r_w = 1,         // read
        .CmdTmp126.addrInc = 0,
        .CmdTmp126.len = 0,
        .CmdTmp126.crc = 0,
        .CmdTmp126.reserved = 0,
        .CmdTmp126.regAddr = 0x00,  // Temperature Register
    };

    txbuf[0] = cmd.bytes[1];    // X - CRC - L4 L3 L2 L1 - A - R/W - ADDR
    txbuf[1] = cmd.bytes[0];    // x -  0  - 0  0  0  0  - 0 - 1   - 0x00   // Temp

    txbuf[2] = 0xFF;    // FF
    txbuf[3] = 0xFF;    // FF

    common_startSPItransfert(device, 4, (uint8_t*)txbuf, (uint8_t*)rxbuf);

    bool success = common_waitSPITransfertDone(device);

    if (success)
    {
        return ((rxbuf[2] << 8) + rxbuf[3]);
    }

    return USHRT_MAX;
}

void spi_tmp126_requestTemp(DeviceIdentEnum_t device)
{
    // Command word
    CmdWordUnion cmd =
    {
        .CmdTmp126.r_w = 1,         // read
        .CmdTmp126.addrInc = 0,
        .CmdTmp126.len = 0,
        .CmdTmp126.crc = 0,
        .CmdTmp126.reserved = 0,
        .CmdTmp126.regAddr = 0x00,  // Temperature Register
    };

    txbuf[0] = cmd.bytes[1];    // X - CRC - L4 L3 L2 L1 - A - R/W - ADDR
    txbuf[1] = cmd.bytes[0];    // x -  0  - 0  0  0  0  - 0 - 1   - 0x00   // Temp

    txbuf[2] = 0xFF;    // FF
    txbuf[3] = 0xFF;    // FF

    common_startSPItransfert(device, 4, (uint8_t*)txbuf, (uint8_t*)rxbuf);
}

uint16_t spi_tmp126_waitTemp(DeviceIdentEnum_t device)
{
    bool success = common_waitSPITransfertDone(device);

    if (success)
    {
        return ((rxbuf[2] << 8) + rxbuf[3]);
    }

    return USHRT_MAX;
}

/**
 * @brief
 * Convert temperature measure [AD value] to [°C].
 *
 * @return temperature [°C]
 */
float spi_temp126_TempToDeg(int16_t tempAd)
{
    return (tempAd >> 2) * 0.03125;
}


