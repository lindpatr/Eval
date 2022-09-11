/*
 * app_stat.c
 *
 *  Created on: 8 sept. 2022
 *      Author: BEL-LinPat
 */

#ifndef COMMON_STAT_H_
#include "common_stat.h"
#endif

uint32_t gTX_tab[TAB_POS_LAST] = { 0 };                // 0 = #TX_OK    1 = #TX_Err   2 = #TX_TimeOut   3 = ND              4 = ND                  5 = ND
uint32_t gRX_tab[MAX_NODE][TAB_POS_LAST] = { 0 };      // 0 = #RX_OK    1 = #RX_Err   2 = #RX_TimeOut   3 = #Gap RX count   4 = Max gap RX count    5 = #CRC_Err
uint32_t gCAL_tab[TAB_POS_LAST] = { 0 };               // 0 = #CAL_REQ  1 = #CAL_Err 2 = ND             3 = ND              4 = ND                  5 = ND

uint32_t gTX_tab_old[TAB_POS_LAST] = { 0 };
uint32_t gRX_tab_old[MAX_NODE][TAB_POS_LAST] = { 0 };
uint32_t gCAL_tab_old[TAB_POS_LAST] = { 0 };


/// Timeout for printing and displaying the statistics
volatile RAIL_Time_t gElapsedTime = 0UL;
volatile RAIL_Time_t gTotalElapsedTime = 0UL;
volatile RAIL_Time_t gOldElapsedTime = 0UL;
