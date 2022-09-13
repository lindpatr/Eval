/*
 * common_config.h
 *
 *  Created on: 6 sept. 2022
 *      Author: BEL-LinPat
 */

#ifndef COMMON_CONFIG_H_
#define COMMON_CONFIG_H_

// Include standard type headers to help define structures
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


#define SLAVE_IN_SYSTEM (4)                     // Netowrk is composed of X slaves
#define TIME_SLOT_DEF   (180U)                  // in us
#define TIME_SLOT_LAST  (290U)                  // in us
#define TIME_SLOT_MIN   (10)                    // in us
#define TIME_SLOT_MAX   (40000U)                // in us

// Master cycle of collecting data from Slaves
// Slot 0   : Master
// Slot 0   : Slave 1
// Slot 1   : Slave 2
// ...
// Slot N-1 : Slave N
#define SYNC_PERIOD  (((SLAVE_IN_SYSTEM+1)*TIME_SLOT_DEF))  // in us
#define SYNC_PERIOD_MIN (500)      // in us
#define SYNC_PERIOD_MAX (60000)      // in us


// In Slave, considering if no Sync from Master after 5 times SYNC_PERIOD that Master stop its process -> restart state machine from Slaves
#define SYNC_TIMEOUT_NB (5)
#define SYNC_TIMOUT_MIN (500)        // in us
#define SYNC_TIMOUT_MAX (1200000)    // in usT
#define SYNC_TIMEOUT    (SYNC_TIMEOUT_NB*SYNC_PERIOD)

#define SEC (1000000U)

#define MAX_SLAVE       (100)                   // Max. nbr of slaves in network
#define MAX_NODE        (MAX_SLAVE+1)           // Max nodes in network (master + slaves)


#define ADDR_TRANSLATION_TABLE_SIZE	(MAX_NODE)
#if (ADDR_TRANSLATION_TABLE_SIZE > INT8_MAX)
#error MAX_NODE shall be less than 127!
#endif

#define ADDR_INTERNAL_NAME_STRING_SIZE (10)

/**
 * @struct PROT_AddrMap_t
 *
 * @brief Contains address mapping for Master and Slave device according to their unique ID.
 */
typedef struct
{
    uint64_t uinqueId;                           // Unique device ID
    uint8_t posTab;                              // Position in tab (in order to handle Master ID and avoid tab of Master ID size)
    bool enable;                                 // Is the device part of the network?
    uint8_t internalAddr;                        // Internal address
    uint32_t slotTime;                           // Slot time
    bool ismaster;                               // true if master device
    char name[ADDR_INTERNAL_NAME_STRING_SIZE];   // Device name
} PROT_AddrMap_t;

/**
* Return the device internal config table address.
*
* @param[in] --
* @return  config table address or NULL (not found).
*/
PROT_AddrMap_t* common_getConfigTable(void);

/**
 * Return the Master internal address.
 *
 * @return  Master device internal address or 0 (not found).
 */
uint8_t common_getMasterAddr(void);

/**
 * Return the device internal address according to the unique ID.
 *
 * @param[in] device unique ID.
 * @return  device internal address or 255 (not found).
 */
uint8_t common_getAddr(uint64_t uniqueId);

/**
 * Return the device internal info according to the unique ID.
 *
 * @param[in] device unique ID.
 * @return  device internal info data or NULL (not found).
 */
PROT_AddrMap_t* common_getConfig(uint64_t uniqueId);

/**
 * Return the device time slot according to the unique ID.
 *
 * @param[in] device unique ID.
 * @return  device internal slot time.
 */
uint32_t common_getSlotime(uint64_t uniqueId);

/**
 * Return the max time slot position.
 *
 * @param[in] -
 * @return  device internal slot time or 0 (not found).
 */
uint32_t common_getMaxSlotTime(void);


/**
 * Return the number of device .
 *
 * @param[in] type MASTER = true, SLAVE = false.
 * @return  number of enabled devices.
 */
#define MASTER_TYPE (true)
#define SLAVE_TYPE (false)

uint8_t common_getNbrDeviceOfType(bool type);

#endif /* PROT_ADDRESS_H_ */
