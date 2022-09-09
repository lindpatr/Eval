/*
 * common_config.c
 *
 *  Created on: 6 sept. 2022
 *      Author: BEL-LinPat
 */

#ifndef COMMON_CONFIG_H_
#include "common_config.h"
#endif

#include <stdbool.h>

/**
 * @brief Adresses mapping for all devices
 */
PROT_AddrMap_t addr_table[ADDR_TRANSLATION_TABLE_SIZE] =
{
    /* ID                Addr slot [us]   Is master   Name          */
	{0x385B44FFFEC085D3, 255, 0,          true,       "MASTER\0"},
	{0x385b44fffec0862b, 1	, 10000,  	  false,      "SLAVE\0"},
	{0x385b44fffe5f5af2, 2	, 20000,      false,      "SLAVE\0"},
    {0x385b44fffe5f5b23, 3  , 30000,      false,      "SLAVE\0"},
    {0x385b44fffec08638, 4  , 40000,      false,      "SLAVE\0"}
};

/**
 * Return the device internal address according to the unique ID.
 *
 * @param[in] device unique ID.
 * @return  device internal address or 255 (not found).
 */
uint8_t common_getAddr(uint64_t uniqueId)
{
	for (int i = 0; i < ADDR_TRANSLATION_TABLE_SIZE; i++)
	{
		if (addr_table[i].uinqueId == uniqueId)
		{
			return addr_table[i].internalAddr;
		}
	}

	return 255;
}

/**
 * Return the device internal info according to the unique ID.
 *
 * @param[in] device unique ID.
 * @return  device internal info data or NULL (not found).
 */
PROT_AddrMap_t* common_getConfig(uint64_t uniqueId)
{
	for (int i = 0; i < ADDR_TRANSLATION_TABLE_SIZE; i++)
	{
		if (addr_table[i].uinqueId == uniqueId)
		{
			return &addr_table[i];
		}
	}

	return NULL;
}
