#ifndef CAN_FWU_HOST_H
#define CAN_FWU_HOST_H
#include "can_fwu_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file can_fwu_host.h
 * @brief Nucleo-side CAN firmware-update session.
 *
 * The transport is injected so host tests can fake it; firmware passes
 * main.c's FDCAN send/poll/abort. The session never starts on its own:
 * fwu_updater.c calls can_fwu_host_begin() only on an explicit request.
 */

/** @brief Session state. */
enum can_fwu_host_state {
  FWU_H_IDLE = 0,   /**< nothing started */
  FWU_H_ENTER_SENT, /**< waiting for the bootloader to acknowledge ENTER */
  FWU_H_ERASE_SENT, /**< waiting for the erase acknowledgement */
  FWU_H_SENDING,    /**< streaming DATA chunks */
  FWU_H_VERIFY_SENT,/**< waiting for the CRC verify acknowledgement */
  FWU_H_JUMP_SENT,  /**< waiting for the jump acknowledgement */
  FWU_H_DONE,       /**< image installed and started */
  FWU_H_FAULT       /**< failed; needs a new begin */
};

/** @brief Frame transport supplied by the caller. */
struct can_fwu_transport {
  int (*send)(uint32_t id, const uint8_t data[8]); /**< queue a frame; nonzero on success */
  int (*poll)(void);  /**< state of the last queued frame: 0 pending, 1 ok, 2 failed */
  void (*abort)(void); /**< cancel the pending frame */
};

/** @brief Update session; fields are internal, read state via ::can_fwu_host_done. */
struct can_fwu_host {
  enum can_fwu_host_state state; /**< session state */
  uint32_t dev;        /**< target wheel select bit (1/2/4/8) */
  uint16_t session;    /**< nonzero session id */
  const uint8_t *image; /**< image being sent */
  uint32_t img_size;   /**< image size, bytes */
  uint32_t img_crc;    /**< image CRC-32 */
  uint32_t seq_next;   /**< next DATA seq to send */
  uint32_t seq_acked;  /**< last acked seq */
  uint32_t num_pages;  /**< flash pages to erase */
  uint32_t tx_id;      /**< ID of the frame in flight */
  uint8_t tx_payload[8]; /**< payload of the frame in flight */
  int tx_pending;      /**< a frame is queued and not yet resolved */
  int tx_sent;         /**< the frame was accepted by the transport */
  int tx_is_data;      /**< the frame in flight is a DATA chunk */
  uint32_t tx_tick;    /**< time the frame in flight was sent, ms */
  uint32_t fault;      /**< ::can_fwu_status code when state is ::FWU_H_FAULT */
  struct can_fwu_transport t; /**< transport copy */
};

/** @brief Reset @p h to idle and bind the transport. */
void can_fwu_host_init(struct can_fwu_host *h,
                       const struct can_fwu_transport *t);
/**
 * @brief Start an update of one ESC.
 *
 * Requires @p bench_idle, a single valid device, and IDLE/DONE/FAULT state.
 * Each accepted begin allocates a fresh runtime-wide nonzero session16. Not
 * thread-safe; a session is never implicitly restarted while active.
 * @param h          session
 * @param dev        wheel select bit (1/2/4/8)
 * @param image      application image, must stay valid until done
 * @param img_size   image size, bytes
 * @param bench_idle nonzero when nothing else is driving the bus
 * @param now        Nucleo time, ms
 * @return 1 if the session started, 0 if refused
 */
int can_fwu_host_begin(struct can_fwu_host *h, uint32_t dev,
                       const uint8_t *image, uint32_t img_size, int bench_idle,
                       uint32_t now);
/**
 * @brief Feed one FWU ACK frame to the session.
 *
 * Only a sent outstanding request with matching device, command, session and
 * request detail can be completed (or faulted) by a v2 ACK. Others are ignored.
 */
void can_fwu_host_ack(struct can_fwu_host *h, uint32_t id,
                      const uint8_t payload[8], uint32_t now);
/** @brief Pump the transport: send the next frame when free, fault on a 100 ms missing ack. */
void can_fwu_host_poll(struct can_fwu_host *h, uint32_t now);
/** @brief Nonzero once the image is installed and the ESC started it. */
int can_fwu_host_done(const struct can_fwu_host *h);

#ifdef __cplusplus
}
#endif
#endif
