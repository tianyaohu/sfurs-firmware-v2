#ifndef ESC_BOOT_ENTRY_H
#define ESC_BOOT_ENTRY_H
#include <stdint.h>

/** @file boot_entry.h
 *  @brief Request to reboot a running ESC application into its bootloader.
 *
 * Shared ABI, independent of the firmware-update ACK protocol.
 * Standard CAN FD, no BRS, exactly 8 bytes: "BOOT", 1, dev, ~dev, 0.
 * ID is exactly 0x650 | dev, dev in {1,2,4,8}; never a selection mask.
 * Both linkers reserve the last 16 bytes of SRAM, outside stack/data/bss.
 * Only offset applications may write this mailbox. Issue DSB before reset.
 */
#define ESC_BOOT_REQUEST_BASE 0x650U      /**< request CAN ID base, OR the device bit */
#define ESC_BOOT_MAILBOX_ADDR 0x20007FF0U /**< RAM mailbox address */
#define ESC_BOOT_MAGIC 0x42525131U        /**< mailbox magic, "BRQ1" */
/** @brief RAM mailbox surviving a software reset. */
struct esc_boot_mailbox {
  uint32_t magic;          /**< ::ESC_BOOT_MAGIC; written last */
  uint32_t inverse_magic;  /**< complement of magic */
  uint32_t device;         /**< wheel select bit that must reboot */
  uint32_t inverse_device; /**< complement of device */
};
_Static_assert(sizeof(struct esc_boot_mailbox) == 16U, "Mailbox ABI size");
/** @brief The mailbox in RAM. */
#define ESC_BOOT_MAILBOX ((volatile struct esc_boot_mailbox *)(uintptr_t)ESC_BOOT_MAILBOX_ADDR)

/** @brief Nonzero if @p dev is exactly one wheel bit (1, 2, 4 or 8). */
static inline int esc_boot_device_valid(uint32_t dev)
{ return dev == 1U || dev == 2U || dev == 4U || dev == 8U; }

/** @brief Fill the 8-byte request payload for @p dev. @return 0 if @p dev is invalid */
static inline int esc_boot_request_pack(uint8_t out[8], uint32_t dev)
{
  if (!esc_boot_device_valid(dev)) return 0;
  out[0] = 'B'; out[1] = 'O'; out[2] = 'O'; out[3] = 'T';
  out[4] = 1U; out[5] = (uint8_t)dev;
  out[6] = (uint8_t)(dev ^ 0xFFU); out[7] = 0U;
  return 1;
}

/** @brief Nonzero if @p id and payload @p p form a valid request for @p dev. */
static inline int esc_boot_request_valid(uint32_t id, const uint8_t *p,
                                         uint32_t dev)
{
  return esc_boot_device_valid(dev) && id == (ESC_BOOT_REQUEST_BASE | dev) &&
      p[0] == 'B' && p[1] == 'O' && p[2] == 'O' && p[3] == 'T' &&
      p[4] == 1U && p[5] == dev && p[6] == (uint8_t)(dev ^ 0xFFU) && p[7] == 0U;
}

/** @brief Store a boot request for @p dev; the caller barriers and resets. */
static inline void esc_boot_mailbox_write(volatile struct esc_boot_mailbox *m,
                                           uint32_t dev)
{
  m->magic = 0U;
  m->inverse_magic = ~ESC_BOOT_MAGIC;
  m->device = dev;
  m->inverse_device = ~dev;
  m->magic = ESC_BOOT_MAGIC; /* publish last; caller barriers before reset */
}

/** @brief Read and clear the mailbox.
 *
 * Caller also requires a software-reset cause, so power-on/stale RAM cannot
 * request boot. Always invalidates, including corrupt/wrong-device requests.
 * @return nonzero if a valid request for @p dev was pending */
static inline int esc_boot_mailbox_consume(volatile struct esc_boot_mailbox *m,
                                            uint32_t dev, int software_reset)
{
  int ok = software_reset && esc_boot_device_valid(dev) &&
      m->magic == ESC_BOOT_MAGIC && m->inverse_magic == ~ESC_BOOT_MAGIC &&
      m->device == dev && m->inverse_device == ~dev;
  m->magic = 0U;
  m->inverse_magic = 0U;
  m->device = 0U;
  m->inverse_device = 0U;
  return ok;
}
#endif
