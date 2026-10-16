#ifndef CAN_FWU_HOST_H
#define CAN_FWU_HOST_H
#include "can_fwu_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Nucleo-side updater session. Transport is injected so host tests can fake
 * it; firmware passes main.c's FDCAN send/poll/abort. The session never
 * starts on its own: fwu_updater.c calls begin() only on an explicit request. */
enum can_fwu_host_state {
  FWU_H_IDLE = 0,
  FWU_H_ENTER_SENT,
  FWU_H_ERASE_SENT,
  FWU_H_SENDING,
  FWU_H_VERIFY_SENT,
  FWU_H_JUMP_SENT,
  FWU_H_DONE,
  FWU_H_FAULT
};

struct can_fwu_transport {
  int (*send)(uint32_t id, const uint8_t data[8]);
  int (*poll)(void); /* 0 pending, 1 ok, 2 failed */
  void (*abort)(void);
};

struct can_fwu_host {
  enum can_fwu_host_state state;
  uint32_t dev;
  uint16_t session;
  const uint8_t *image;
  uint32_t img_size;
  uint32_t img_crc;
  uint32_t seq_next;   /* next DATA seq to send */
  uint32_t seq_acked;  /* last acked seq */
  uint32_t num_pages;
  uint32_t tx_id;
  uint8_t tx_payload[8];
  int tx_pending;
  int tx_sent;
  int tx_is_data;
  uint32_t tx_tick;
  uint32_t fault;
  struct can_fwu_transport t;
};

void can_fwu_host_init(struct can_fwu_host *h,
                       const struct can_fwu_transport *t);
/* Requires bench_idle, a single valid device, and IDLE/DONE/FAULT state.
 * Each accepted begin allocates a fresh runtime-wide nonzero session16.
 * Not thread-safe; a session is never implicitly restarted while active. */
int can_fwu_host_begin(struct can_fwu_host *h, uint32_t dev,
                       const uint8_t *image, uint32_t img_size, int bench_idle,
                       uint32_t now);
/* Only a sent outstanding request with matching device, command, session and
 * request detail can be completed (or faulted) by a v2 ACK. Others are ignored. */
void can_fwu_host_ack(struct can_fwu_host *h, uint32_t id,
                      const uint8_t payload[8], uint32_t now);
/* Pump transport: sends next frame when free, faults on 100 ms no-ack. */
void can_fwu_host_poll(struct can_fwu_host *h, uint32_t now);
int can_fwu_host_done(const struct can_fwu_host *h);

#ifdef __cplusplus
}
#endif
#endif
