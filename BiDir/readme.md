# Evaluation EFR32xG22 - Bidirectional

## Introduction

The application implements a "ping pong" party between 2 EV boards. 
Each board has to play a role. The role are:

* Master
* Slave

Each boards is transmitting and receiving, alternatively. 

* Master TX - Slave RX - Slave Tx - Master RX and so on

In case of an error

> TX Error: board retransmits
> RX Error: board is listening
> RX Timeout: Master initiate a TX and Slave is listening to avoid deadlock in the party
> TX Timeout: not yet implemented

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

`qPrintTX/qPrintRX = 1` > print out detail on serial COM
`qPrintTX/qPrintRX = 0` > no detail is printed out on serial COM

```
> qUseDisplay
```
To display statistics on LCD:

`qUseDisplay = 1` > print statistics on LCD
`qUseDisplay = 0` > no LCD display used

Note that the statistics are always printed out on serial COM.

```
> qPrintEvents
```
Additionnaly to the statistics, counters for all enabled radio events is printed out on serial COM.

`qPrintEvents = 1` > print out enabled radio event on serial COM
`qPrintEvents = 0` > no radio event is printed out

```
> qPrintErrors
```

To print out on serial COM every error: 

`qPrintErrors = 1` > print out enabled radio event on serial COM
`qPrintErrors = 0` > no error is printed out

Errors printed out are: 
> TX and RX errors
> TX and RX timeout
> Unsuccessfull Rail functions return


## Utilization

On POR or Reset, the role (Master or Slave) is displayed on LCD:

`Test EFR32xG32 - Master (BiDir)`
`--------------------------------`


By pressing BTNO on Master and/or on Slave, "ping pong" party is enabled is LCD displays: 

`> Start ping pong`

When "ping pong" is started:

* >>> RUNNING >>> is displayed on LCD (bottom line)
* LED1 toggles each TX
* LED0 toggles each RX
* Each time the BTN0 is pressed , the statistics are displayed and printed out on serial COM


After "ping pong" is started, pressing on BTN0 will display the statistics on LCD (if enabled with the compile directive `qUseDisplay`) and print out on serial COM.


`Count (#TX Master) : 89688`
`Count (#TX Slave)  : 89607`
`TX Error (#Err/#TO): 0.000% (0/0)`
`RX Error (#Err/#TO): 0.084% (0/75)`
`TX retransmit count: 0`
`Slave counter #gap : 5 (max:2)`
`Rate (loop 100)    : 3442.36 msg/s (29.34 ms)`

> #TX Master is the counter sent by the master
> #TX Slave is the counter sent by the slave
> TX Error: #Err is the number of TX Error and #TO is the number of TX timeout
> RX Error: #Err is the number of RX Error and #TO is the number of RX timeout. Please note that a request to display and print out the statistics will introduce a "freeze" of about 60-70 ms and generate some RX Timeout. This is degrading the subsequent statistics!
> TX retransmit count is the number of retransmission in case of TX Error or TX Timeout (this counter is increase in case of RX Timeout)
> Slave counter #gap si the number of occurences a gap is decoded in the counter in the RX frame
> Slave counter max gap is the gap max between to sucessfull RX (normal is 1 when no frame are lost)

If is enabled, additionnaly the radio events counters will be displayed

`Radio events detail`
`-------------------`
`b03 RX_PACKET_RECEIVED  : 89613`
`b24 TX_PACKET_SENT      : 89688`
`b41 CAL_NEEDED          : 1`

Note that `bXX` indicates the position in the RAIL_Events_t bitfield.
