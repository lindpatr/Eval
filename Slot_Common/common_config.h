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


//#define SLAVE_IN_SYSTEM (4)                   // Netowrk is composed of X slaves
//#define TIME_SLOT_DEF   (190U)                // in us --> only if time slot of a slave is defined as (Addr-1)*TIME_SLOT_DEF; current implementation is setting a specific time slot for each slave!
#define TIME_SLOT_ACQ   (210U)                  // in us
#define TIME_SLOT_MASTER_TX   (220U)            // in us
#define TIME_SLOT_RES   (115U)                  // in us
#define TIME_SLOT_SLAVE (190U)                  // in us
#define TIME_SLOT_CORR  (30U)                   // in us
#define TIME_SLOT_MIN   (10U)                   // in us
#define TIME_SLOT_MAX   (100000U)               // in us

// Master cycle of collecting data from Slaves
// Slot 0   : Master
// Slot 0   : Slave 1
// Slot 1   : Slave 2
// ...
// Slot N-1 : Slave N
//#define SYNC_PERIOD  (((SLAVE_IN_SYSTEM+1)*TIME_SLOT_DEF))  // in us --> only if time slot of a slave is defined as (Addr-1)*TIME_SLOT_DEF
#define SYNC_PERIOD     (1000/*20000*/)         // 20 ms
#define SYNC_PERIOD_MIN (400U)                  // in us
#define SYNC_PERIOD_MAX (1000000U)              // in us


// In Slave, considering if no Sync from Master after 5 times SYNC_PERIOD that Master stop its process -> restart state machine from Slaves; current implementation is setting a specific time slot for each slave!
#define SYNC_TIMEOUT_NB (5)
#define SYNC_TIMOUT_MIN (500U)                  // in us
#define SYNC_TIMOUT_MAX (5000000U)              // in us
#define SYNC_TIMEOUT_VAR    (3.0f/100.0f)
#define SYNC_TIMEOUT    ((uint32_t)((float)SYNC_TIMEOUT_NB*(1.0f+SYNC_TIMEOUT_VAR)*(float)SYNC_PERIOD))

#define SEC (1000000U)

#define MAX_SLAVE       (100)                   // Max. nbr of slaves in network
#define MAX_NODE        (MAX_SLAVE+1)           // Max nodes in network (master + slaves)


#define ADDR_TRANSLATION_TABLE_SIZE	(MAX_NODE)
#if (ADDR_TRANSLATION_TABLE_SIZE > INT8_MAX)
#error MAX_NODE shall be less than 127!
#endif

#define ADDR_INTERNAL_NAME_STRING_SIZE (10)


// Kind of possible messages
typedef enum
{
    kInvalidMsg = 0x00,
    kDataMsg    = 0xAA,
    kStatMsg    = 0x55,
    kDiagMsg    = 0xF0,
    kServiceMsg = 0xA5,
    kSetupMsg   = 0x5A
} TypeMsgEnum;

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
* @param[in] index dans la table
* @return  config table address or NULL (not found).
*/
PROT_AddrMap_t* common_getConfigTable(uint8_t index);

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
 * @param[in] all (false) or all (true)
 * @return  number of (enabled or not) devices.
 */
#define MASTER_TYPE (true)
#define SLAVE_TYPE (false)
#define ALL (true)
#define ENABLED (false)

uint8_t common_getNbrDeviceOfType(bool type, bool all);

#endif /* PROT_ADDRESS_H_ */
