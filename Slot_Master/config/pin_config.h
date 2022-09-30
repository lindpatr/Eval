#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// $[CMU]
// [CMU]$

// $[LFXO]
// [LFXO]$

// $[PRS.ASYNCH0]
// [PRS.ASYNCH0]$

// $[PRS.ASYNCH1]
// [PRS.ASYNCH1]$

// $[PRS.ASYNCH2]
// [PRS.ASYNCH2]$

// $[PRS.ASYNCH3]
// [PRS.ASYNCH3]$

// $[PRS.ASYNCH4]
// [PRS.ASYNCH4]$

// $[PRS.ASYNCH5]
// [PRS.ASYNCH5]$

// $[PRS.ASYNCH6]
// [PRS.ASYNCH6]$

// $[PRS.ASYNCH7]
// [PRS.ASYNCH7]$

// $[PRS.ASYNCH8]
// [PRS.ASYNCH8]$

// $[PRS.ASYNCH9]
// [PRS.ASYNCH9]$

// $[PRS.ASYNCH10]
// [PRS.ASYNCH10]$

// $[PRS.ASYNCH11]
// [PRS.ASYNCH11]$

// $[PRS.SYNCH0]
// [PRS.SYNCH0]$

// $[PRS.SYNCH1]
// [PRS.SYNCH1]$

// $[PRS.SYNCH2]
// [PRS.SYNCH2]$

// $[PRS.SYNCH3]
// [PRS.SYNCH3]$

// $[GPIO]
// GPIO SWV on PA03
#define GPIO_SWV_PORT                            gpioPortA
#define GPIO_SWV_PIN                             3

// [GPIO]$

// $[TIMER0]
// [TIMER0]$

// $[TIMER1]
// [TIMER1]$

// $[TIMER2]
// [TIMER2]$

// $[TIMER3]
// [TIMER3]$

// $[TIMER4]
// [TIMER4]$

// $[USART0]
// USART0 CLK on PC02
#define USART0_CLK_PORT                          gpioPortC
#define USART0_CLK_PIN                           2

// USART0 CS on PC03
#define USART0_CS_PORT                           gpioPortC
#define USART0_CS_PIN                            3

// USART0 RX on PC01
#define USART0_RX_PORT                           gpioPortC
#define USART0_RX_PIN                            1

// USART0 TX on PC00
#define USART0_TX_PORT                           gpioPortC
#define USART0_TX_PIN                            0

// [USART0]$

// $[USART1]
// [USART1]$

// $[I2C1]
// [I2C1]$

// $[PDM]
// [PDM]$

// $[LETIMER0]
// [LETIMER0]$

// $[IADC0]
// [IADC0]$

// $[I2C0]
// [I2C0]$

// $[EUART0]
// EUART0 RX on PA06
#define EUART0_RX_PORT                           gpioPortA
#define EUART0_RX_PIN                            6

// EUART0 TX on PA05
#define EUART0_TX_PORT                           gpioPortA
#define EUART0_TX_PIN                            5

// [EUART0]$

// $[PTI]
// PTI DCLK on PC06
#define PTI_DCLK_PORT                            gpioPortC
#define PTI_DCLK_PIN                             6

// PTI DFRAME on PC05
#define PTI_DFRAME_PORT                          gpioPortC
#define PTI_DFRAME_PIN                           5

// PTI DOUT on PC04
#define PTI_DOUT_PORT                            gpioPortC
#define PTI_DOUT_PIN                             4

// [PTI]$

// $[MODEM]
// [MODEM]$

// $[CUSTOM_PIN_NAME]
#define SWCLK_DEBUG_PORT                         gpioPortA
#define SWCLK_DEBUG_PIN                          1

#define SWDIO_DEBUG_PORT                         gpioPortA
#define SWDIO_DEBUG_PIN                          2

#define SWO_DEBUG_PORT                           gpioPortA
#define SWO_DEBUG_PIN                            3

#define MX25_SD_PORT                             gpioPortA
#define MX25_SD_PIN                              4

#define UART_TxD_PORT                            gpioPortA
#define UART_TxD_PIN                             5

#define UART_RxD_PORT                            gpioPortA
#define UART_RxD_PIN                             6

#define EN_SPI_FLASH_PORT                        gpioPortA
#define EN_SPI_FLASH_PIN                         7

#define BTN0_PORT                                gpioPortB
#define BTN0_PIN                                 0

#define DEBUG_1_PORT                             gpioPortB
#define DEBUG_1_PIN                              1

#define EN_UART_PORT                             gpioPortB
#define EN_UART_PIN                              4

#define SPI_MOSI_PORT                            gpioPortC
#define SPI_MOSI_PIN                             0

#define SPI_MISO_PORT                            gpioPortC
#define SPI_MISO_PIN                             1

#define SPI_CLK_PORT                             gpioPortC
#define SPI_CLK_PIN                              2

#define SPI_CS_PORT                              gpioPortC
#define SPI_CS_PIN                               3

#define PTI_SPI_DOUT_PORT                        gpioPortC
#define PTI_SPI_DOUT_PIN                         4

#define PTI_SPI_CS_PORT                          gpioPortC
#define PTI_SPI_CS_PIN                           5

#define PTI_SPI_CLK_PORT                         gpioPortC
#define PTI_SPI_CLK_PIN                          6

#define XTAL1_PORT                               gpioPortD
#define XTAL1_PIN                                0

#define XTAL2_PORT                               gpioPortD
#define XTAL2_PIN                                1

#define DEBUG_2_PORT                             gpioPortD
#define DEBUG_2_PIN                              2

#define DEBUG_3_PORT                             gpioPortD
#define DEBUG_3_PIN                              3

// [CUSTOM_PIN_NAME]$

#endif // PIN_CONFIG_H

