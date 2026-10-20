#ifndef ESC_BOOT_METADATA_H
#define ESC_BOOT_METADATA_H
#include "can_fwu_validity.h"
#include <string.h>

/** @file boot_metadata.h
 *  @brief Commit of the validity record after a verified install. */

/** @brief Metadata flash callbacks the port supplies. */
struct esc_metadata_ops {
  int (*erase)(void *ctx); /**< erase the metadata page; 0 on success */
  int (*program)(unsigned word, uint64_t value, void *ctx); /**< program a doubleword; 0 on success */
  void *ctx;               /**< passed to every callback */
};

/**
 * @brief Write the validity record for an installed image.
 *
 * CRC and vectors are checked against live flash before any operation. A valid
 * but different committed identity is NOT permission to erase it. Invalid or
 * interrupted metadata is erased before any doubleword can be programmed.
 * @return 1 if the record is (now) committed, 0 if refused or a callback failed */
static inline int esc_metadata_commit(const struct can_fwu_validity *current,
    uint32_t size, uint32_t crc, uint32_t sp, uint32_t pc, uint32_t actual_crc,
    const struct esc_metadata_ops *ops)
{
  const struct can_fwu_validity desired = {FWU_VALID_MAGIC, ~FWU_VALID_MAGIC,
      size, crc, FWU_APP_BASE, CAN_FWU_PROTO_VERSION, {0U, 0U}};
  if (!can_fwu_vectors_valid(sp, pc, size) || actual_crc != crc) return 0;
  if (memcmp(current, &desired, sizeof(desired)) == 0) return 1;
  if (current->magic == FWU_VALID_MAGIC &&
      current->inverse_magic == ~FWU_VALID_MAGIC) return 0;
  if (ops->erase(ops->ctx) != 0) return 0;
  uint32_t words[8];
  memcpy(words, &desired, sizeof(words));
  for (unsigned i = 2U; i < 8U; i += 2U) {
    if (ops->program(i, words[i] | ((uint64_t)words[i + 1U] << 32),
                     ops->ctx) != 0) return 0;
  }
  return ops->program(0U, desired.magic |
      ((uint64_t)desired.inverse_magic << 32), ops->ctx) == 0;
}
#endif
