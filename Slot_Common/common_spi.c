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

#define DMA_IF_MASK ((1 << TX_LDMA_CHANNEL) + (1 << RX_LDMA_CHANNEL))

// LDMA descriptor and transfer configuration structures for USART TX channel
LDMA_Descriptor_t ldmaTXDescriptor;
LDMA_TransferCfg_t ldmaTXConfig;

// LDMA descriptor and transfer configuration structures for USART RX channel
LDMA_Descriptor_t ldmaRXDescriptor;
LDMA_TransferCfg_t ldmaRXConfig;

// SPI TX/RX timeout
#define SPI_TIMEOUT_SECURITY (20)

// Chip select definition structure
#define SPI_CS_NUMBER   (2)

typedef struct
{
    uint32_t portNumber;
    uint32_t pinNumber;
} SpiCsStruct_t;

// Chip select initialization
static SpiCsStruct_t ChipSelectTab[SPI_CS_NUMBER] =
{
    {US0CS_PORT, US0CS_PIN},
    {US1CS_PORT, US1CS_PIN}
};

static uint32_t gBaudRate = 0;
static uint32_t gTimeOut = 0;

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
    GPIO_PinModeSet(ChipSelectTab[device0].portNumber, ChipSelectTab[device0].pinNumber, gpioModePushPull, 1);

    // Configure CS pin as an output and drive inactive high
    GPIO_PinModeSet(ChipSelectTab[device1].portNumber, ChipSelectTab[device1].pinNumber, gpioModePushPull, 1);
}

/**
 * Init SPI serial part.
 */
void common_SPIinitUSART0(uint32_t baudrate, USART_ClockMode_TypeDef clockPhase)
{
    // Enable clock
    CMU_ClockEnable(cmuClock_USART0, true);

    // Default asynchronous initializer (main mode, 1 Mbps, 8-bit data)
    USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;

    init.msbf = true;            // MSB first transmission for SPI compatibility
    init.autoCsEnable = false;   // Allow the USART to assert CS
    init.baudrate = baudrate;    // bits/s
    gBaudRate = baudrate;

    init.clockMode = clockPhase;/*usartClockMode2;*/    // tests simulateur SPI Alex

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

    // Enable USART interface pins
    GPIO->USARTROUTE[0].ROUTEEN = GPIO_USART_ROUTEEN_RXPEN |    // MISO
                                  GPIO_USART_ROUTEEN_TXPEN |    // MOSI
                                  GPIO_USART_ROUTEEN_CLKPEN;

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

  // USART0_TXDATA, descriptor. Buffer & length are define in common_startSPItransfert(...) function to
  // be able to change the buffer size dynamically (different size for different slave device)
  ldmaTXDescriptor.xfer.blockSize = ldmaCtrlBlockSizeUnit2;    // Transfers 2 units per arbitration
  ldmaTXDescriptor.xfer.ignoreSrec = 1;    // Ignores single requests

  // Transfer 2 bytes on free space in the USART buffer
  ldmaTXConfig = (LDMA_TransferCfg_t)LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART0_TXBL);

  // USART0_RXDATA, descriptor. Buffer & length are define in common_startSPItransfert(...) function to
  // be able to change the buffer size dynamically (different size for different slave device)
  ldmaRXDescriptor.xfer.blockSize = ldmaCtrlBlockSizeUnit2;    // Transfers 2 units per arbitration
  ldmaRXDescriptor.xfer.ignoreSrec = 1;    // Ignores single requests

  // Transfer 2 bytes on receive data valid
  ldmaRXConfig = (LDMA_TransferCfg_t)LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART0_RXDATAV);
}

/**
 * @brief
 * Init SPI.
 *
 * @param[in] baudrate Spi baudrate [bit/s]
 * @param[in] clockPhase Clock phase
 */
void common_initSPI(uint32_t baudrate, USART_ClockMode_TypeDef clockPhase)
{
    common_SPIinitGPIO();
    common_SPIinitUSART0(baudrate, clockPhase);
    common_SPIinitLDMA();
}

/**
 * @brief
 * Start a SPI transfert (Master).
 *
 * @param[in] device Device number to select which cs is asserted
 * @param[in] buffSize TX/RX buffer size -> size to TX/RX
 * @param[in] txBuffer TX buffer ptr
 * @param[in] rxBuffer RX buffer ptr
 */
void common_startSPItransfert(DeviceIdentEnum_t device, uint8_t buffSize, uint8_t* txBuffer, uint8_t* rxBuffer)
{
    // Source is txBuffer, destination is USART0_TXDATA, and length is buffSize
    ldmaTXDescriptor = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(txBuffer, &(USART0->TXDATA), buffSize);
    // Source is USART0_RXDATA, destination is rxBuffer, and length is buffSize
    ldmaRXDescriptor = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_SINGLE_P2M_BYTE(&(USART0->RXDATA), rxBuffer, buffSize);

    // attempt to calculate the timeout
    gTimeOut = (((1.0 / (float)gBaudRate) * 8.0 * buffSize) * 1000000.0) + SPI_TIMEOUT_SECURITY;

    // CS asserted
    GPIO_PinOutClear(ChipSelectTab[device].portNumber, ChipSelectTab[device].pinNumber);

    // RX channel
    LDMA_StartTransfer(RX_LDMA_CHANNEL, &ldmaRXConfig, &ldmaRXDescriptor);

    // TX channel
    LDMA_StartTransfer(TX_LDMA_CHANNEL, &ldmaTXConfig, &ldmaTXDescriptor);
}

/**
 * @brief
 * Wait for the end of the SPI transfer
 *
 * @param[in] device Device number to select which cs is asserted
 * @return result TRUE sucessfull, FALSE timout.
*/
bool common_waitSPITransfertDone(DeviceIdentEnum_t device)
{
    uint32_t flags = LDMA_IntGet();
    RAIL_Time_t startTime = RAIL_GetTime();
    RAIL_Time_t gap;

    while((flags & DMA_IF_MASK) != DMA_IF_MASK)
    {
        gap = RAIL_GetTime() - startTime;
        if (gap > gTimeOut)
        {
            GPIO_PinOutSet(ChipSelectTab[device].portNumber, ChipSelectTab[device].pinNumber);
            return false;
        }
        flags = LDMA_IntGet();
    }

    LDMA_IntClear(1 << TX_LDMA_CHANNEL);
    LDMA_IntClear(1 << RX_LDMA_CHANNEL);

    GPIO_PinOutSet(ChipSelectTab[device].portNumber, ChipSelectTab[device].pinNumber);

    return true;
}
