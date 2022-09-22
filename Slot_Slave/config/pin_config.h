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
// EUART0 CTS on PA08
#define EUART0_CTS_PORT                          gpioPortA
#define EUART0_CTS_PIN                           8

// EUART0 RTS on PA07
#define EUART0_RTS_PORT                          gpioPortA
#define EUART0_RTS_PIN                           7

// EUART0 RX on PA06
#define EUART0_RX_PORT                           gpioPortA
#define EUART0_RX_PIN                            6

// EUART0 TX on PA05
#define EUART0_TX_PORT                           gpioPortA
#define EUART0_TX_PIN                            5

// [EUART0]$

// $[PTI]
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
#define UART_TxD_PORT                            gpioPortA
#define UART_TxD_PIN                             5

#define UART_RxD_PORT                            gpioPortA
#define UART_RxD_PIN                             6

#define UART_RTS_PORT                            gpioPortA
#define UART_RTS_PIN                             7

#define UART_CTS_PORT                            gpioPortA
#define UART_CTS_PIN                             8

#define BTN_0_PORT                               gpioPortB
#define BTN_0_PIN                                0

#define DEBUG_1_PORT                             gpioPortB
#define DEBUG_1_PIN                              1

#define EN_UART_PORT                             gpioPortB
#define EN_UART_PIN                              4

#define LCD_SPI_MOSI_PORT                        gpioPortC
#define LCD_SPI_MOSI_PIN                         0

#define LCD_SPI_MISO_PORT                        gpioPortC
#define LCD_SPI_MISO_PIN                         1

#define LCD_SPI_CLK_PORT                         gpioPortC
#define LCD_SPI_CLK_PIN                          2

#define EN_DISPL_PORT                            gpioPortC
#define EN_DISPL_PIN                             7

#define DEBUG_2_PORT                             gpioPortD
#define DEBUG_2_PIN                              2

#define DEBUG_3_PORT                             gpioPortD
#define DEBUG_3_PIN                              3

// [CUSTOM_PIN_NAME]$

#endif // PIN_CONFIG_H

