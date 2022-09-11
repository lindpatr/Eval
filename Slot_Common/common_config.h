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


#define SLAVE_IN_SYSTEM (3)                     // Netowrk is composed of X slaves
#define TIME_SLOT       (240U)                  // in us

// Master cycle of collecting data from Slaves
// Slot 0   : Master
// Slot 0   : Slave 1
// Slot 1   : Slave 2
// ...
// Slot N-1 : Slave N
#define SYNC_PERIOD  ((SLAVE_IN_SYSTEM*TIME_SLOT)+(uint32_t)(0.25f*(float)TIME_SLOT)/*(uint32_t)(0.1f*SEC)*/)             // in us, TODO Nb slave + 0.5 car Slave 1 et Master communique dans slot 0 mais ce dernier dure plus long, à contrôler / analyser

// In Slave, considering if no Sync from Master after 5 times SYNC_PERIOD that Master stop its process -> restart state machine from Slaves
#define SYNC_TIMEOUT (5*SYNC_PERIOD)

#define SEC (1000000U)

#define MAX_SLAVE       (100)                   // Max. nbr of slaves in network
#define MAX_NODE        (MAX_SLAVE+1)           // Max nodes in network (master + slaves)
#define MASTER_ID       (255)                   // Master ID; Slaves ID [1..MAX_SLAVE]


#define ADDR_TRANSLATION_TABLE_SIZE	(MAX_NODE)
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
    uint8_t internalAddr;                        // Internal address
    uint32_t slotTime;                           // Slot time in uS
    bool ismaster;                               // true if master device
    char name[ADDR_INTERNAL_NAME_STRING_SIZE];   // Device name
} PROT_AddrMap_t;

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

#endif /* PROT_ADDRESS_H_ */
