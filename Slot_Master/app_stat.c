/*
 * app_stat.c
 *
 *  Created on: 8 sept. 2022
 *      Author: BEL-LinPat
 */

#ifndef APP_STAT_H_
#include "app_stat.h"
#endif

uint32_t gTX_tab[7] = { 0 };    // 0 = #TX_OK  1 = #TX_Err   2 = ND      3 = #Retransmit   4 = ND          5 = #TX_Invalid   6 = ND
uint32_t gRX_tab[7] = { 0 };    // 0 = #RX_OK  1 = #RX_Err   2 = #RX_TimeOut 3 = #Gap RX count 4 = Max gap RX count  5 = #RX_Invalid   6 = #CRC_Err
uint32_t gCAL_tab[7] = { 0 };   // 0 = #CAL_REQ  1 = #CAL_Err  2 = ND      3 = ND        4 = ND          5 = ND        6 = ND

/// Timeout for printing and displaying the statistics
volatile RAIL_Time_t gElapsedTime = 0UL;
volatile RAIL_Time_t gOldElapsedTime = 0UL;
