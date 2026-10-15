#include "can_fwu_host.h"

/* Runtime-wide allocator: reinitializing a host must not reuse a stale session.
 * Single-threaded updater; wraps after 65535 begins, skipping zero. */
static uint16_t next_session;

void can_fwu_host_init(struct can_fwu_host *h,
                       const struct can_fwu_transport *t)
{
  h->state = FWU_H_IDLE;
  h->dev = 0U;
  h->session = 0U;
  h->image = 0;
  h->img_size = 0U;
  h->img_crc = 0U;
  h->seq_next = 0U;
  h->seq_acked = 0U;
  h->num_pages = 0U;
  h->tx_id = 0U;
  h->tx_pending = 0;
  h->tx_sent = 0;
  h->tx_is_data = 0;
  h->tx_tick = 0U;
  h->fault = 0U;
  if (t != 0) {
    h->t = *t;
  } else {
    h->t.send = 0;
    h->t.poll = 0;
    h->t.abort = 0;
  }
}

static void queue(struct can_fwu_host *h, uint32_t id, const uint8_t p[8],
                  int is_data, uint32_t now)
{
  unsigned i;
  h->tx_id = id;
  for (i = 0; i < 8; i++) {
    h->tx_payload[i] = p[i];
  }
  h->tx_pending = 1;
  h->tx_sent = 0;
  h->tx_is_data = is_data;
  h->tx_tick = now;
}

int can_fwu_host_begin(struct can_fwu_host *h, uint32_t dev,
                       const uint8_t *image, uint32_t img_size, int bench_idle,
                       uint32_t now)
{
  uint8_t p[8];
  if ((h->state != FWU_H_IDLE && h->state != FWU_H_DONE &&
       h->state != FWU_H_FAULT) || !can_fwu_dev_valid(dev) ||
      !bench_idle || image == 0 || img_size < CAN_FWU_APP_MIN ||
      img_size > CAN_FWU_APP_MAX || h->t.send == 0) {
    return 0;
  }
  h->dev = dev;
  next_session++;
  if (next_session == 0U) next_session++;
  h->session = next_session;
  h->image = image;
  h->img_size = img_size;
  h->img_crc = can_fwu_crc32(image, img_size);
  h->seq_next = 0U;
  h->seq_acked = 0U;
  h->num_pages = can_fwu_pages_for_size(img_size);
  h->fault = 0U;
  h->state = FWU_H_ENTER_SENT;
  can_fwu_enter_session_pack(p, h->session, img_size);
  queue(h, can_fwu_id(CAN_FWU_ENTER_BASE, h->dev), p, 0, now);
  return 1;
}

static void to_fault(struct can_fwu_host *h, uint32_t code)
{
  if (h->tx_pending && h->t.abort != 0) {
    h->t.abort();
  }
  h->tx_pending = 0;
  h->tx_sent = 0;
  h->state = FWU_H_FAULT;
  h->fault = code;
}

void can_fwu_host_ack(struct can_fwu_host *h, uint32_t id,
                      const uint8_t payload[8], uint32_t now)
{
  uint32_t st, detail, seq, expected;
  uint8_t p[8];
  unsigned i;
  uint32_t off;
  if (h->state == FWU_H_IDLE || h->state == FWU_H_DONE ||
      h->state == FWU_H_FAULT) {
    return;
  }
  if (!h->tx_pending || !h->tx_sent ||
      id != can_fwu_id(CAN_FWU_ACK_BASE, h->dev) ||
      can_fwu_ack_command(payload) != (h->tx_id >> 4) ||
      can_fwu_ack_session(payload) != h->session) {
    return;
  }
  switch (h->state) {
  case FWU_H_ENTER_SENT: expected = CAN_FWU_BOOT_VERSION; break;
  case FWU_H_ERASE_SENT: expected = h->num_pages; break;
  case FWU_H_SENDING: expected = h->seq_next; break;
  case FWU_H_VERIFY_SENT: expected = h->img_crc; break;
  case FWU_H_JUMP_SENT: expected = 0U; break;
  default: return;
  }
  st = can_fwu_ack_status(payload);
  detail = can_fwu_ack_detail(payload);
  /* A previous DATA ACK, even an error, cannot complete/fault this request. */
  if (detail != expected) return;
  if (st != CAN_FWU_OK) {
    to_fault(h, st ? st : CAN_FWU_REJECT);
    return;
  }
  h->tx_pending = 0;
  h->tx_sent = 0;
  switch (h->state) {
  case FWU_H_ENTER_SENT:
    h->state = FWU_H_ERASE_SENT;
    can_fwu_erase_pack(p, 0U, h->num_pages);
    queue(h, can_fwu_id(CAN_FWU_ERASE_BASE, h->dev), p, 0, now);
    break;
  case FWU_H_ERASE_SENT:
    h->state = FWU_H_SENDING;
    /* fall through to first DATA */
    seq = h->seq_next;
    off = seq * CAN_FWU_CHUNK;
    for (i = 0; i < CAN_FWU_CHUNK; i++) {
      p[2 + i] = (off + i < h->img_size) ? h->image[off + i] : 0xFFU;
    }
    p[0] = (uint8_t)seq;
    p[1] = (uint8_t)(seq >> 8);
    queue(h, can_fwu_id(CAN_FWU_DATA_BASE, h->dev), p, 1, now);
    break;
  case FWU_H_SENDING:
    h->seq_acked = detail;
    h->seq_next++;
    if (h->seq_next < can_fwu_data_count(h->img_size)) {
      off = h->seq_next * CAN_FWU_CHUNK;
      for (i = 0; i < CAN_FWU_CHUNK; i++) {
        p[2 + i] = (off + i < h->img_size) ? h->image[off + i] : 0xFFU;
      }
      seq = h->seq_next;
      p[0] = (uint8_t)seq;
      p[1] = (uint8_t)(seq >> 8);
      queue(h, can_fwu_id(CAN_FWU_DATA_BASE, h->dev), p, 1, now);
    } else {
      h->state = FWU_H_VERIFY_SENT;
      can_fwu_verify_pack(p, h->img_crc, h->img_size);
      queue(h, can_fwu_id(CAN_FWU_VERIFY_BASE, h->dev), p, 0, now);
    }
    break;
  case FWU_H_VERIFY_SENT:
    h->state = FWU_H_JUMP_SENT;
    can_fwu_jump_pack(p);
    queue(h, can_fwu_id(CAN_FWU_JUMP_BASE, h->dev), p, 0, now);
    break;
  case FWU_H_JUMP_SENT:
    h->state = FWU_H_DONE;
    break;
  default:
    break;
  }
}

void can_fwu_host_poll(struct can_fwu_host *h, uint32_t now)
{
  int r;
  if (h->state == FWU_H_IDLE || h->state == FWU_H_DONE ||
      h->state == FWU_H_FAULT || !h->tx_pending) {
    return;
  }
  if (!h->tx_sent) {
    /* One-shot enqueue; the ACK timer starts at enqueue. */
    if (h->t.send == 0 || !h->t.send(h->tx_id, h->tx_payload)) {
      to_fault(h, CAN_FWU_FLASH_ERR);
      return;
    }
    h->tx_sent = 1;
    h->tx_tick = now;
    return;
  }
  if (h->t.poll != 0) {
    r = h->t.poll();
    if (r == 2) {
      to_fault(h, CAN_FWU_FLASH_ERR);
      return;
    }
    /* r == 1: frame left the FIFO; r == 0: still queued. Either way
     * keep waiting for the target ACK until the 100 ms budget expires. */
  }
  /* Page erase is slower than ordinary replies; do not use the DATA
   * budget for a multi-page flash erase. */
  uint32_t timeout = h->state == FWU_H_ERASE_SENT ? 2000U :
                     CAN_FWU_HOST_ACK_TIMEOUT_MS;
  if ((uint32_t)(now - h->tx_tick) >= timeout) {
    to_fault(h, CAN_FWU_REJECT);
  }
}

int can_fwu_host_done(const struct can_fwu_host *h)
{
  return h->state == FWU_H_DONE;
}
