#ifndef CAN_FWU_BOOT_H
#define CAN_FWU_BOOT_H
#include "can_fwu_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file can_fwu_boot.h
 * @brief Portable ESC-side bootloader session.
 *
 * No HAL here; the G4 port supplies erase/program/motor/now callbacks. Staged
 * bytes are CRCed on arrival; the G4 port adds flash-readback verify before
 * jumping.
 */

/** @brief Bootloader session state. */
enum can_fwu_boot_state {
  FWU_B_IDLE = 0,    /**< waiting for ENTER */
  FWU_B_ENTERED,     /**< ENTER accepted, flash not yet erased */
  FWU_B_ERASED,      /**< application pages erased */
  FWU_B_RECEIVING,   /**< DATA chunks arriving */
  FWU_B_VERIFIED,    /**< image CRC and size matched */
  FWU_B_JUMP_PENDING,/**< JUMP accepted, port jumps to the application */
  FWU_B_FAULT        /**< failed; a new ENTER with another session recovers */
};

/** @brief Flash and motor callbacks the target port supplies. */
struct can_fwu_flash_ops {
  int (*erase)(uint32_t start_page, uint32_t num_pages, void *ctx); /**< erase pages; 0 on success */
  int (*program_dw)(uint32_t byte_offset, uint64_t dw, void *ctx);  /**< program one doubleword; 0 on success */
  int (*motor_stopped)(void *ctx); /**< nonzero when the motor is safe to overwrite */
  void *ctx;                       /**< passed to every callback */
};

/** @brief Bootloader session; fields are internal. */
struct can_fwu_boot {
  enum can_fwu_boot_state state; /**< session state */
  uint32_t dev;         /**< this ESC's wheel select bit (1/2/4/8) */
  uint16_t session;     /**< session id from ENTER */
  uint8_t last_data[8]; /**< last accepted complete wire payload for exact retry */
  uint32_t img_size;    /**< image size announced by ENTER, bytes */
  uint32_t img_crc;     /**< expected CRC from successful VERIFY */
  uint32_t received;    /**< image bytes received */
  uint32_t stream_crc;  /**< running CRC of received bytes */
  uint32_t expect_seq;  /**< next DATA seq accepted */
  uint32_t num_pages;   /**< pages the image occupies */
  uint64_t erased_pages; /**< bit per page, max 56 pages for 112K/2K */
  uint8_t dw_buf[8];    /**< doubleword being assembled */
  unsigned dw_fill;     /**< bytes in dw_buf */
  uint32_t dw_base;     /**< flash offset of dw_buf */
  uint32_t last_tick;   /**< time of the last accepted progress, ms */
  uint32_t fault;       /**< ::can_fwu_status code when state is ::FWU_B_FAULT */
  struct can_fwu_flash_ops ops; /**< callbacks copy */
};

/** @brief Reset @p b to idle for wheel @p dev and bind the callbacks. */
void can_fwu_boot_init(struct can_fwu_boot *b, uint32_t dev,
                       const struct can_fwu_flash_ops *ops);
/**
 * @brief Handle one FWU frame.
 *
 * Same-session/same-size ENTER is idempotent. Other sessions are rejected
 * while active; after a fault a different nonzero session may recover. Only
 * the most recent successful DATA is replayable, with identical 8 bytes.
 * @param[in]  b       session
 * @param[in]  id      CAN ID
 * @param[in]  payload 8 payload bytes
 * @param[in]  now     time, ms
 * @param[out] ack_id  ID of the reply
 * @param[out] ack     8-byte correlated v2 ACK
 * @return 1 and a filled ACK when the frame addresses this target, else 0
 */
int can_fwu_boot_frame(struct can_fwu_boot *b, uint32_t id,
                       const uint8_t payload[8], uint32_t now,
                       uint32_t *ack_id, uint8_t ack[8]);
/**
 * @brief Idle timeout; call each tick.
 *
 * 2 s without accepted progress or an exact retry faults the session.
 * Rejected or malformed traffic never extends this timeout.
 */
void can_fwu_boot_poll(struct can_fwu_boot *b, uint32_t now);

#ifdef __cplusplus
}
#endif
#endif
