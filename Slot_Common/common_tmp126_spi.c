/*
 * tmp126_spi.c
 *
 *  Created on: 17 oct. 2022
 *      Author: BEL-LinPat
 */

#include "common_tmp126_spi.h"
#include "common_spi.h"

// Size of the data buffers
#define BUFLEN  20      // tests simulateur SPI Alex

// Outgoing data
uint8_t txbuf[BUFLEN];

// Incoming data
uint8_t rxbuf[BUFLEN];

// TODO Vérifier le fonctionnement des limites (allumage de la LED end cas de dépassement)
// TODO Gestion des erreurs
// TODO Suppression des boucles d'attente

void spi_tmp126_init(void)
{
    // Init SPI
    common_initSPI(1000000, usartClockMode0);

    // Command word
    CmdWordUnion cmd =
    {
        .CmdTmp126.r_w = 0,
        .CmdTmp126.addrInc = 1,
        .CmdTmp126.len = 0,
        .CmdTmp126.crc = 0,
        .CmdTmp126.reserved = 0,
        .CmdTmp126.regAddr = 0x03,
    };

    // 0x03 -- Configuration
    //txbuf[2] = 0x00;    // reserved      Reset - AVG - Res - Int_Cmp - 1Shot - Mode - Conv Period
    //txbuf[3] = 0x09;    // xxxxxxx       0       0     0     0         0       1      001

    CmdWordUnion conf =
    {
        .ConfTmp126.conv_period = 1,
        .ConfTmp126.mode = 1,
        .ConfTmp126.one_shot = 0,
        .ConfTmp126.int_comp = 0,
        .ConfTmp126.reserved2 = 0,
        .ConfTmp126.avg = 0,
        .ConfTmp126.reset = 0,
        .ConfTmp126.reserved = 0,
    };

    CmdWordUnion alert =
    {
        .Alert_En.dataready_alert = 0,
        .Alert_En.TLow_alert = 1,
        .Alert_En.THigh_alert = 1,
        .Alert_En.slew_alert = 0,
        .Alert_En.crc_alert = 0,
        .Alert_En.reserved2 = 0,
    };


    txbuf[0] = cmd.bytes[1];         // X - CRC - L4 L3 L2 L1 - A - R/W - ADDR
    txbuf[1] = cmd.bytes[0];         // x -  0  - 0  0  0  0  - 1 -   0 - 0x03

    // reg = 0x03 -- Configuration
    txbuf[2] = conf.bytes[1];        //      reserved     Reset - AVG - Res - Int_Cmp - 1Shot - Mode - Conv Period
    txbuf[3] = conf.bytes[0];        // 0x09 xxxxxxx      0       0     0     0         0       1      001

    // reg = 0x04 -- Alert_Enable
    txbuf[4] = alert.bytes[1];       //      reserved     CRC - Slew - THigh - TLow - DataRdy
    txbuf[5] = alert.bytes[0];       // 0x06 xxxxxxxxxxx  0     0      1       1      0

    // LOW = 0x05 LSB = 0.03125°C
    uint16_t tempL = ((uint16_t)(25.00 / 0.03125)) << 2;
    txbuf[6] = (uint8_t)(tempL >> 8);;    //
    txbuf[7] = (uint8_t)(tempL & 0x00FF);

    // HIGH = 0x06 LSB = 0.03125°C
    uint16_t tempH = ((uint16_t)(35.00 / 0.03125)) << 2;
    txbuf[8] = (uint8_t)(tempH >> 8);    // 30 deg / 0.03125
    txbuf[9] = (uint8_t)(tempH & 0x00FF);

    common_startSPItransfert(device0, 10, (uint8_t*)txbuf, (uint8_t*)rxbuf);

    bool success = common_waitSPITransfertDone(device0);

    if (success)
    {
        for (uint32_t i = 0; i < 1000; i++);
    }
}

uint16_t spi_tmp126_getID(void)
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

    common_startSPItransfert(device0, 4, (uint8_t*)txbuf, (uint8_t*)rxbuf);

    bool success = common_waitSPITransfertDone(device0);

    if (success)
    {
        return ((rxbuf[2] << 8) + rxbuf[3]);
    }

    return 0;
}

uint16_t spi_tmp126_getTemp(void)
{
    // Command word
    CmdWordUnion cmd =
    {
        .CmdTmp126.r_w = 1,
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

    common_startSPItransfert(device0, 4, (uint8_t*)txbuf, (uint8_t*)rxbuf);

    bool success = common_waitSPITransfertDone(device0);

    if (success)
    {
        return ((rxbuf[2] << 8) + rxbuf[3]);
    }

    return 0;
}

float spi_temp126_TempToDeg(uint16_t tempAd)
{
    uint16_t temper = (rxbuf[2] << 8) + rxbuf[3];

    //uint16_t cmp2 = (temper ^ 0xEFFF) + 1;

    return (temper >> 2) * 0.03125;
}

// Code de test
//spi_tmp126_init();
//uint16_t ID = spi_tmp126_getID();
//
//
//char string[80];
//if (ID == 0x2126)
//{
//    while(1)
//    {
//        uint16_t temp = spi_tmp126_getTemp();
//        float temperature = spi_temp126_TempToDeg(temp);
//
//        if (temperature > 0.0)
//        {
//            sprintf(string, "Temp %f \n", temperature);
//            for (uint32_t i = 0; i < 10000000; i++);
//        }
//    }
//}

