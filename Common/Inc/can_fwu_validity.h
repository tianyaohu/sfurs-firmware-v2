#ifndef CAN_FWU_VALIDITY_H
#define CAN_FWU_VALIDITY_H
#include "can_fwu_protocol.h"
#include "boot_entry.h"

#define FWU_APP_BASE 0x08004000U
#define FWU_METADATA_BASE 0x0801F800U
#define FWU_VALID_MAGIC 0x56414C49U

/* All fields are LE on STM32. Magic + complement is the LAST double-word
 * committed, after flash CRC and vectors validate. Erase metadata BEFORE
 * app erase; no magic means remain in boot even if vectors look plausible. */
struct can_fwu_validity {
  uint32_t magic, inverse_magic;
  uint32_t size, crc;
  uint32_t base, version;
  uint32_t reserved[2];
};

static inline int can_fwu_vectors_valid(uint32_t sp, uint32_t reset,
                                        uint32_t size)
{
  uint32_t pc = reset & ~1U;
  return size >= 8U && size <= CAN_FWU_APP_MAX &&
         sp > 0x20000000U && sp <= ESC_BOOT_MAILBOX_ADDR && (sp & 7U) == 0U &&
         (reset & 1U) != 0U && pc >= FWU_APP_BASE &&
         pc - FWU_APP_BASE < size;
}

static inline int can_fwu_validity_valid(const struct can_fwu_validity *m,
                                         uint32_t sp, uint32_t reset,
                                         uint32_t actual_crc)
{
  return m->magic == FWU_VALID_MAGIC &&
         m->inverse_magic == ~FWU_VALID_MAGIC &&
         m->base == FWU_APP_BASE && m->version == CAN_FWU_PROTO_VERSION &&
         can_fwu_vectors_valid(sp, reset, m->size) && actual_crc == m->crc;
}
#endif
