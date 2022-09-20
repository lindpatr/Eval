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

// Table (for fixed time slot = f(Addr)) for M-S1-S2-S3-S4 (1+4)
//PROT_AddrMap_t addr_table[ADDR_TRANSLATION_TABLE_SIZE] =
//{
//    /* ID              syncPos    Addr    slot [us]         Is master   Name          */
//	{0x385B44FFFEC085D3, 0,     255,    0,                true,       "MASTER\0"},
//	{0x385b44fffec0862b, 1,     1,      (0*TIME_SLOT),    false,      "SLAVE\0"},
//	{0x385b44fffe5f5af2, 2,     2,      (3*TIME_SLOT),    false,      "SLAVE\0"},
//  {0x385b44fffe5f5b23, 3,     3,      (1*TIME_SLOT),    false,      "SLAVE\0"},
//  {0x385b44fffec08638, 4,     4,      (2*TIME_SLOT),    false,      "SLAVE\0"}
//};

// Table (non optimized (aligned)) for M-S1-S2-S3-S4 (1+4)
// N = 4
// Sync = 960???
// Auto transition S1: N
// Rate = ?? ms
//PROT_AddrMap_t addr_table[ADDR_TRANSLATION_TABLE_SIZE] =
//{
//    /* ID                Pos    Enable  Addr    Slot time         Is master   Name          */
//    {0x385B44FFFEC085D3, 0,     true,   255,    0,                true,       "MASTER\0"},
//    {0x385b44fffec0862b, 1,     true,   1,      1,                false,      "SLAVE\0"},
//    {0x385b44fffe5f5af2, 2,     true,   2,      300,              false,      "SLAVE\0"},
//    {0x385b44fffe5f5b23, 3,     true,   3,      480,              false,      "SLAVE\0"},
//    {0x385b44fffec08638, 4,     true,   4,      670,              false,      "SLAVE\0"},
//};

// Table (optimized) for M-S1-S2-S3-S4 (1+4)
// N = 4
// Sync = 855?
// Auto transition S1: N
// Rate = 19.11 ms
//PROT_AddrMap_t addr_table[ADDR_TRANSLATION_TABLE_SIZE] =
//{
//    /* ID                Pos    Enable  Addr    Slot time         Is master   Name          */
//    {0x385B44FFFEC085D3, 0,     true,   255,    0,                true,       "MASTER\0"},
//    {0x385b44fffec0862b, 1,     true,   1,      1,                false,      "SLAVE\0"},
//    {0x385b44fffe5f5af2, 2,     true,   2,      270,              false,      "SLAVE\0"},
//    {0x385b44fffe5f5b23, 3,     true,   3,      455,              false,      "SLAVE\0"},
//    {0x385b44fffec08638, 4,     true,   4,      610,              false,      "SLAVE\0"},
//};


// Table (non optimized (aligned)) for M-S1-S3-S4 (1+3)
// N = 3
// Sync = ???
// Auto transition S1: N
// Rate = ?? ms
PROT_AddrMap_t addr_table[ADDR_TRANSLATION_TABLE_SIZE] =
{
    /* ID                Pos    Enable  Addr    Slot time         Is master   Name          */
    {0x385B44FFFEC085D3, 0,     true,   255,    0,                true,       "MASTER\0"},
    {0x385b44fffec0862b, 1,     true,   1,      1,                false,      "SLAVE\0"},
    {0x385b44fffe5f5af2, 2,     false,  2,      670,              false,      "SLAVE\0"},
    {0x385b44fffe5f5b23, 3,     true,   3,      300,              false,      "SLAVE\0"},
    {0x385b44fffec08638, 4,     true,   4,      490,              false,      "SLAVE\0"},
};

// Table (optimized) for M-S1-S3-S4 (1+3)
// N = 3
// Sync = 740
// Auto transition S1: N
// Rate = 19.85 ms
//PROT_AddrMap_t addr_table[ADDR_TRANSLATION_TABLE_SIZE] =
//{
//    /* ID                Pos    Enable  Addr    Slot time         Is master   Name          */
//    {0x385B44FFFEC085D3, 0,     true,   255,    0,                true,       "MASTER\0"},
//    {0x385b44fffec0862b, 1,     true,   1,      1,                false,      "SLAVE\0"},
//    {0x385b44fffe5f5af2, 2,     false,  2,      610,              false,      "SLAVE\0"},
//    {0x385b44fffe5f5b23, 3,     true,   3,      270,              false,      "SLAVE\0"},
//    {0x385b44fffec08638, 4,     true,   4,      455,              false,      "SLAVE\0"},
//};

// Table (optimized) for M-S1-S3-S4 (1+3) with delay 100 us for S1
// N = 3
// Sync = 740
// Auto transition S1: N
// Rate = 19.85 ms
//PROT_AddrMap_t addr_table[ADDR_TRANSLATION_TABLE_SIZE] =
//{
//    /* ID                Pos    Enable  Addr    Slot time         Is master   Name          */
//    {0x385B44FFFEC085D3, 0,     true,   255,    0,                true,       "MASTER\0"},
//    {0x385b44fffec0862b, 1,     true,   1,      0+100,                false,      "SLAVE\0"},
//    {0x385b44fffe5f5af2, 2,     false,  2,      610+100,              false,      "SLAVE\0"},
//    {0x385b44fffe5f5b23, 3,     true,   3,      270+100,              false,      "SLAVE\0"},
//    {0x385b44fffec08638, 4,     true,   4,      455+100,              false,      "SLAVE\0"},
//};

// Table (non optimized (aligned)) for M-S1-S3-S4 (1+3)
// N = 3
// Sync = 1000
// Auto transition S1: Y
// Rate = 27.5 ms
//PROT_AddrMap_t addr_table[ADDR_TRANSLATION_TABLE_SIZE] =
//{
//    /* ID                Pos    Enable  Addr    Slot time         Is master   Name          */
//    {0x385B44FFFEC085D3, 0,     true,   255,    0,                true,       "MASTER\0"},
//    {0x385b44fffec0862b, 1,     true,   1,      0,                false,      "SLAVE\0"},
//    {0x385b44fffe5f5af2, 2,     false,  2,      630,              false,      "SLAVE\0"},
//    {0x385b44fffe5f5b23, 3,     true,   3,      210,              false,      "SLAVE\0"},
//    {0x385b44fffec08638, 4,     true,   4,      420,              false,      "SLAVE\0"},
//};

/**
* Return the device internal config table address.
*
* @param[in] index dans la table
* @return  config table address or NULL (not found).
*/
PROT_AddrMap_t* common_getConfigTable(uint8_t index)
{
   return (&addr_table[index] != NULL ? &addr_table[index] : NULL);
}

/**
 * Return the Master internal address.
 *
 * @return  Master device internal address or 0 (not found).
 */
uint8_t common_getMasterAddr(void)
{
    for (int i = 0; i < ADDR_TRANSLATION_TABLE_SIZE; i++)
    {
        if ((addr_table[i].ismaster == MASTER_TYPE) && addr_table[i].enable)
        {
            return addr_table[i].internalAddr;
        }
    }

    return 0;
}

/**
 * Return the device internal address according to the unique ID.
 *
 * @param[in] device unique ID.
 * @return  device internal address or 0 (not found).
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

	return 0;
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

/**
 * Return the device time slot according to the unique ID.
 *
 * @param[in] device unique ID.
 * @return  device internal slot time or UINT32_MAX if not found.
 */
uint32_t common_getSlotime(uint64_t uniqueId)
{
    for (int i = 0; i < ADDR_TRANSLATION_TABLE_SIZE; i++)
    {
        if (addr_table[i].uinqueId == uniqueId)
        {
            return addr_table[i].slotTime;
        }
    }

    return UINT32_MAX;
}

/**
 * Return the max time slot position.
 *
 * @param[in] -
 * @return  device internal slot time or 0 (not found).
 */
uint32_t common_getMaxSlotTime(void)
{
    uint32_t max = 0;

    for (int i = 0; i < ADDR_TRANSLATION_TABLE_SIZE; i++)
    {
        max = (addr_table[i].enable ? (addr_table[i].slotTime > max ? addr_table[i].slotTime : max) : max);
    }

    return max;
}


/**
 * Return the number of device .
 *
 * @param[in] type MASTER = true, SLAVE = false.
 * @param[in] all (false) or all (true)
 * @return  number of (enabled or not) devices.
 */
uint8_t common_getNbrDeviceOfType(bool type, bool all)
{
    uint8_t count = 0;

    for (int i = 0; i < ADDR_TRANSLATION_TABLE_SIZE; i++)
    {
        if ((addr_table[i].ismaster == type) && (all ? true : addr_table[i].enable))
        {
            count++;
        }
    }

    return count;
}
