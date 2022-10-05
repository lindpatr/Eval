/*
 * common_spi.c
 *
 *  Created on: 4 oct. 2022
 *      Author: BEL-LinPat
 */


#include "common_spi.h"
#include "em_gpio.h"
#include "em_ldma.h"
#include "em_usart.h"
#include "em_cmu.h"
#include "rail.h"

// Ports and pins for SPI interface
#define US0MOSI_PORT  gpioPortC
#define US0MOSI_PIN   0
#define US0MISO_PORT  gpioPortC
#define US0MISO_PIN   1
#define US0CLK_PORT   gpioPortC
#define US0CLK_PIN    2
// DAC
#define US0CS_PORT    gpioPortC
#define US0CS_PIN     3
// TEMP
#define US1CS_PORT    gpioPortA
#define US1CS_PIN     7

// LDMA channel for receive and transmit servicing
#define RX_LDMA_CHANNEL 6
#define TX_LDMA_CHANNEL 7

// LDMA descriptor and transfer configuration structures for USART TX channel
LDMA_Descriptor_t ldmaTXDescriptor;
LDMA_TransferCfg_t ldmaTXConfig;

// LDMA descriptor and transfer configuration structures for USART RX channel
LDMA_Descriptor_t ldmaRXDescriptor;
LDMA_TransferCfg_t ldmaRXConfig;

// SPI TX/RX timeout
#define SPI_TIMEOUT (300)

// Size of the data buffers
#define BUFLEN  28      // tests simulateur SPI Alex

// Outgoing data
uint8_t outbuf[BUFLEN];

// Incoming data
uint8_t inbuf[BUFLEN];

/**
 * Init SPI i/o pin.
 */
void common_SPIinitGPIO(void)
{
    // Enable clock
    CMU_ClockEnable(cmuClock_GPIO, true);

    // Configure TX pin as an output
    GPIO_PinModeSet(US0MOSI_PORT, US0MOSI_PIN, gpioModePushPull, 0);

    // Configure RX pin as an input
    GPIO_PinModeSet(US0MISO_PORT, US0MISO_PIN, gpioModeInput, 0);

    // Configure CLK pin as an output low (CPOL = 0)
    GPIO_PinModeSet(US0CLK_PORT, US0CLK_PIN, gpioModePushPull, 0);

    // Configure CS pin as an output and drive inactive high
    GPIO_PinModeSet(US0CS_PORT, US0CS_PIN, gpioModePushPull, 0);

    // Configure CS pin as an output and drive inactive high
    GPIO_PinModeSet(US1CS_PORT, US1CS_PIN, gpioModePushPull, 1);
}

/**
 * Init SPI serial part.
 */
void common_SPIinitUSART0(void)
{
    // Enable clock
    CMU_ClockEnable(cmuClock_USART0, true);

    // Default asynchronous initializer (main mode, 1 Mbps, 8-bit data)
    USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;

    init.msbf = true;           // MSB first transmission for SPI compatibility
    init.autoCsEnable = true;   // Allow the USART to assert CS
    init.baudrate = 1000000;     /* 1 Mbits/s. */
    init.clockMode = usartClockMode2;    // tests simulateur SPI Alex
    // init.clockMode = usartClockMode0; // capteur de pression atmosphérique
    /*
     * The autoCsSetup and autoCsHold members allow chip select setup and
     * hold times delays to be insert before and after assertion.
     * However, the predefined options of 0, 1, 2, 3, and 7 bit times can
     * be too short if the secondary device needs time to setup reception.
     *
     * These delays can be increased by using the programmable USART
     * timer comparators.  In this case, setting autoCsSetup and
     * autoCsHold to 5 uses the count programmed into USART0->TIMECMP0
     * to do this.  USART0->USART_TIMECMP0_TCMPVAL is programmed to
     * insert 15 bit times (15 us @ 1 MHz) of setup and hold delay.
     */
    init.autoCsSetup = 1;
    init.autoCsHold = 1;

    USART0->TIMECMP0 = 15;
    USART0->TIMECMP0 |= USART_TIMECMP0_RESTARTEN;

    /*
     * Route USART0 RX, TX, and CLK to the specified pins.  Note that CS is
     * not controlled by USART0 so there is no write to the corresponding
     * USARTROUTE register to do this.
     */
    GPIO->USARTROUTE[0].TXROUTE = (US0MOSI_PORT << _GPIO_USART_TXROUTE_PORT_SHIFT)
        | (US0MOSI_PIN << _GPIO_USART_TXROUTE_PIN_SHIFT);
    GPIO->USARTROUTE[0].RXROUTE = (US0MISO_PORT << _GPIO_USART_RXROUTE_PORT_SHIFT)
        | (US0MISO_PIN << _GPIO_USART_RXROUTE_PIN_SHIFT);
    GPIO->USARTROUTE[0].CLKROUTE = (US0CLK_PORT << _GPIO_USART_CLKROUTE_PORT_SHIFT)
        | (US0CLK_PIN << _GPIO_USART_CLKROUTE_PIN_SHIFT);

// TODO : Il faudra certainement gérer les CS manuellement pour pouvoir utliser 2 fois la même instance de la lib
//    GPIO->USARTROUTE[0].CSROUTE = (US0CS_PORT << _GPIO_USART_CSROUTE_PORT_SHIFT)
//        | (US0CS_PIN << _GPIO_USART_CSROUTE_PIN_SHIFT);

    // Enable USART interface pins
    GPIO->USARTROUTE[0].ROUTEEN = GPIO_USART_ROUTEEN_RXPEN |    // MISO
                                  GPIO_USART_ROUTEEN_TXPEN |    // MOSI
                                  GPIO_USART_ROUTEEN_CLKPEN /*|
                                  GPIO_USART_ROUTEEN_CSPEN*/;

    // Configure and enable USART0
    USART_InitSync(USART0, &init);
}

/**
 * Init SPI dma part.
 */
void common_SPIinitLDMA(void)
{
  // First, initialize the LDMA unit itself
  LDMA_Init_t ldmaInit = LDMA_INIT_DEFAULT;
  LDMA_Init(&ldmaInit);

  // Source is outbuf, destination is USART0_TXDATA, and length is BUFLEN
  ldmaTXDescriptor = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(outbuf, &(USART0->TXDATA), BUFLEN);
  ldmaTXDescriptor.xfer.blockSize = ldmaCtrlBlockSizeUnit2;    // Transfers 2 units per arbitration
  ldmaTXDescriptor.xfer.ignoreSrec = 1;    // Ignores single requests

  // Transfer 2 bytes on free space in the USART buffer
  ldmaTXConfig = (LDMA_TransferCfg_t)LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART0_TXBL);

  // Source is USART0_RXDATA, destination is inbuf, and length is BUFLEN
  ldmaRXDescriptor = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_SINGLE_P2M_BYTE(&(USART0->RXDATA), inbuf, BUFLEN);
  ldmaRXDescriptor.xfer.blockSize = ldmaCtrlBlockSizeUnit2;    // Transfers 2 units per arbitration
  ldmaRXDescriptor.xfer.ignoreSrec = 1;    // Ignores single requests

  // Transfer 2 bytes on receive data valid
  ldmaRXConfig = (LDMA_TransferCfg_t)LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART0_RXDATAV);
}

/**
 * @brief
 * Init SPI.
 */
void common_initSPI(void)
{
    common_SPIinitGPIO();
    common_SPIinitUSART0();
    common_SPIinitLDMA();

    for (int i = 0; i < BUFLEN; i++)
    {
        outbuf[i] = i;
        inbuf[i] = 0;
    }
}

void common_startSPItransfert(bool txOnly)
{
    GPIO_PinOutClear(US0CS_PORT, US0CS_PIN);

    if (!txOnly)
    {
        // RX channel
        LDMA_StartTransfer(RX_LDMA_CHANNEL, &ldmaRXConfig, &ldmaRXDescriptor);
    }

    // TX channel
    LDMA_StartTransfer(TX_LDMA_CHANNEL, &ldmaTXConfig, &ldmaTXDescriptor);
}


bool common_waitSPITransfertDone(bool txOnly)
{
    uint32_t flags = LDMA_IntGet();
    uint32_t mask = ((1 << TX_LDMA_CHANNEL) + (1 << RX_LDMA_CHANNEL));
    RAIL_Time_t startTime = RAIL_GetTime();
    RAIL_Time_t gap;

    if (txOnly)
    {
        mask = (1 << TX_LDMA_CHANNEL);
    }

    while((flags & mask) != mask)
    {
        gap = RAIL_GetTime() - startTime;
        if (gap > SPI_TIMEOUT)
        {
            GPIO_PinOutSet(US0CS_PORT, US0CS_PIN);
            return false;
        }
        flags = LDMA_IntGet();
    }

    if ((flags & mask) == mask)
    {
        LDMA_IntClear(1 << TX_LDMA_CHANNEL);
        if (!txOnly)
        {
            LDMA_IntClear(1 << RX_LDMA_CHANNEL);
        }
    }

    GPIO_PinOutSet(US0CS_PORT, US0CS_PIN);

    return true;
}
