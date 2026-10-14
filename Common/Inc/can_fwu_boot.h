#ifndef CAN_FWU_BOOT_H
#define CAN_FWU_BOOT_H
#include "can_fwu_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* P3: portable ESC-side bootloader session. No HAL here; the G4 port
 * supplies erase/program/motor/now callbacks. Staged bytes are CRCed
 * on arrival; the G4 port adds flash-readback verify before jumping. */
enum can_fwu_boot_state {
  FWU_B_IDLE = 0,
  FWU_B_ENTERED,
  FWU_B_ERASED,
  FWU_B_RECEIVING,
  FWU_B_VERIFIED,
  FWU_B_JUMP_PENDING,
  FWU_B_FAULT
};

struct can_fwu_flash_ops {
  int (*erase)(uint32_t start_page, uint32_t num_pages, void *ctx);
  int (*program_dw)(uint32_t byte_offset, uint64_t dw, void *ctx);
  int (*motor_stopped)(void *ctx);
  void *ctx;
};

struct can_fwu_boot {
  enum can_fwu_boot_state state;
  uint32_t dev;
  uint32_t img_size;
  uint32_t img_crc; /* expected, given at ENTER time by host build */
  uint32_t received;
  uint32_t stream_crc;
  uint32_t expect_seq;
  uint32_t num_pages;
  uint64_t erased_pages; /* bit per page, max 56 pages for 112K/2K */
  uint8_t dw_buf[8];
  unsigned dw_fill;
  uint32_t last_tick;
  uint32_t fault;
  struct can_fwu_flash_ops ops;
};

void can_fwu_boot_init(struct can_fwu_boot *b, uint32_t dev,
                       const struct can_fwu_flash_ops *ops);
/* Returns 1 and fills ack id/payload when the frame addresses this target. */
int can_fwu_boot_frame(struct can_fwu_boot *b, uint32_t id,
                       const uint8_t payload[8], uint32_t now,
                       uint32_t *ack_id, uint8_t ack[8]);
/* Call each tick; 2 s without traffic aborts the session to a safe state. */
void can_fwu_boot_poll(struct can_fwu_boot *b, uint32_t now);

#ifdef __cplusplus
}
#endif
#endif
