#ifndef CAN_FWU_VERSION_H
#define CAN_FWU_VERSION_H

/** @file can_fwu_version.h
 *  @brief Single source of FWU versions and the ESC flash layout contract. */
#define CAN_FWU_STACK_VERSION "1.0.0" /**< version of the update stack */
#define CAN_FWU_FLASH_BASE 0x08000000U /**< start of ESC flash */
#define CAN_FWU_BOOT_ORIGIN CAN_FWU_FLASH_BASE /**< bootloader origin */
#define CAN_FWU_BOOT_LENGTH CAN_FWU_BOOT_SIZE /**< bootloader length, bytes */
#define CAN_FWU_APP_ORIGIN (CAN_FWU_FLASH_BASE + CAN_FWU_APP_OFFSET) /**< application origin */
#define CAN_FWU_APP_LENGTH CAN_FWU_APP_MAX /**< application length, bytes */

#endif
