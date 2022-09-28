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

// Table (non optimized (aligned)) for M-S1-S3-S4 (1+3) with delay 100 us for S1
// N = 3
// Sync = 1040
// Auto transition S1: Y (but not active as S1 slot > 0
// Rate = 19.57 ms (NEW)
#define SHIFT_TIME  (100)
#define FIRST_TIME_SLOT (10)
#define DELTA_TIME_SLOT (TIME_SLOT_SLAVE+10)

//PROT_AddrMap_t addr_table[ADDR_TRANSLATION_TABLE_SIZE] =
//{
//    /* ID                Pos    Enable  Addr    Slot time         Is master   Name          */
//    {0x385B44FFFEC085D3, 0,     true,   255,    0,                true,       "MASTER\0"},
//    {0x385b44fffec0862b, 1,     true,   1,      -95+SHIFT_TIME,   false,      "SLAVE\0"},
//    {0x385b44fffe5f5af2, 2,     false,  2,      610+SHIFT_TIME,   false,      "SLAVE\0"},
//    {0x385b44fffe5f5b23, 3,     true,   3,      210+SHIFT_TIME,   false,      "SLAVE\0"},
//    {0x385b44fffec08638, 4,     true,   4,      410+SHIFT_TIME,   false,      "SLAVE\0"},
//    {0x040D84FFFE88A3DF, 5,     true,   5,      610+SHIFT_TIME,   false,      "SLAVE\0"},
//    {0x040D84FFFE88A50A, 6,     true,   6,      810+SHIFT_TIME,   false,      "SLAVE\0"},
//    {0x040D84FFFE88A766, 7,     true,   7,     1010+SHIFT_TIME,   false,      "SLAVE\0"},
//    {0x040D84FFFE88AD13, 8,     true,   8,     1210+SHIFT_TIME,   false,      "SLAVE\0"},
//    {0x040D84FFFE88AB60, 9,     true,   9,     1410+SHIFT_TIME,   false,      "SLAVE\0"},
//};
PROT_AddrMap_t addr_table[ADDR_TRANSLATION_TABLE_SIZE] =
{
    /* ID                Pos    Enable  Addr    Slot time         Is master   Name          */
    {0x385B44FFFEC085D3, 0,     true,   255,    0,                                              true,       "MASTER\0"},
    {0x385b44fffec0862b, 1,     true,   1,      (0*DELTA_TIME_SLOT)+SHIFT_TIME-95,              false,      "SLAVE\0"},
    {0x385b44fffe5f5af2, 2,     false,  2,      (8*DELTA_TIME_SLOT)+SHIFT_TIME+FIRST_TIME_SLOT, false,      "SLAVE\0"},
    {0x385b44fffe5f5b23, 3,     true,   3,      (1*DELTA_TIME_SLOT)+SHIFT_TIME+FIRST_TIME_SLOT, false,      "SLAVE\0"},
    {0x385b44fffec08638, 4,     true,   4,      (2*DELTA_TIME_SLOT)+SHIFT_TIME+FIRST_TIME_SLOT, false,      "SLAVE\0"},
    {0x040D84FFFE88A3DF, 5,     true,   5,      (3*DELTA_TIME_SLOT)+SHIFT_TIME+FIRST_TIME_SLOT, false,      "SLAVE\0"},
    {0x040D84FFFE88A50A, 6,     true,   6,      (4*DELTA_TIME_SLOT)+SHIFT_TIME+FIRST_TIME_SLOT, false,      "SLAVE\0"},
    {0x040D84FFFE88A766, 7,     true,   7,      (5*DELTA_TIME_SLOT)+SHIFT_TIME+FIRST_TIME_SLOT, false,      "SLAVE\0"},
    {0x040D84FFFE88AD13, 8,     true,   8,      (6*DELTA_TIME_SLOT)+SHIFT_TIME+FIRST_TIME_SLOT, false,      "SLAVE\0"},
    {0x040D84FFFE88AB60, 9,     true,   9,      (7*DELTA_TIME_SLOT)+SHIFT_TIME+FIRST_TIME_SLOT, false,      "SLAVE\0"},
};

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
