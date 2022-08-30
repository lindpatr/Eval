# Evaluation EFR32xG22 - Unidirectional

## Introduction

The applicarion implements a role of

* Master Transmitter
* Slave Receiver

## Compile directives

```
> qMaster
```
The role is chosen base the compile directive `qMaster`:

`qMaster = 1` > Master Transmitter
`qMaster = 0` > Slave Receiver

```
> qPrintTX and qPrintRX
```
To print all transmitted or received data, you can use compile directive `qPrintTx` respectively compile directive `qPrintRX` 

`qPrintTX/qPrintRX = 1` > print detail on serial com
`qPrintTX/qPrintRX = 0` > no detail printed on serial com

```
> qUseDisplay
```

To display statistics on LCD

`qUseDisplay = 1` > print statistics on LCD
`qUseDisplay = 0` > no LCD display used

## Utilization

By pressing BTNO on Master, transmission is enabled. Pressing another time, transmission is disabled, and so on.
When transmission is enabled:

* >>> RUNNING >>> is displayed on LCD
* LED1 is ON
* Every 20 sec, the statistics are displayed

When transmission is disabled:

* *** STOPPED **** is displayed on LCD
* LED1 is OFF

On Slave 

* >>> RUNNING >>> is displayed on LCD
* Every 20 sec, the statistics are displayed

If transmission is stopped on Master, received frames with OK will remain at the same value and the timeout errors (TO) will increase.