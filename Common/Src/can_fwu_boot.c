#include "can_fwu_boot.h"
#include <string.h>

static int dev_match(uint32_t id, uint32_t base, uint32_t dev)
{
  return can_fwu_dev_valid(dev) && id == can_fwu_id(base, dev);
}

void can_fwu_boot_init(struct can_fwu_boot *b, uint32_t dev,
                       const struct can_fwu_flash_ops *ops)
{
  b->state = FWU_B_IDLE;
  b->dev = dev;
  b->session = 0U;
  memset(b->last_data, 0, sizeof(b->last_data));
  b->img_size = 0U;
  b->img_crc = 0U;
  b->received = 0U;
  b->stream_crc = 0U;
  b->expect_seq = 0U;
  b->num_pages = 0U;
  b->erased_pages = 0U;
  b->dw_fill = 0U;
  b->dw_base = 0U;
  b->last_tick = 0U;
  b->fault = 0U;
  if (ops != 0) {
    b->ops = *ops;
  } else {
    b->ops.erase = 0;
    b->ops.program_dw = 0;
    b->ops.motor_stopped = 0;
    b->ops.ctx = 0;
  }
}

static void to_fault(struct can_fwu_boot *b, uint32_t code)
{
  b->state = FWU_B_FAULT;
  b->fault = code;
}

static void reply(const struct can_fwu_boot *b, uint32_t id,
                  const uint8_t payload[8], uint8_t ack[8], uint32_t status)
{
  uint32_t base = id & ~0xFU;
  uint32_t detail = 0U;
  uint16_t session = b->session;
  switch (base) {
  case CAN_FWU_ENTER_BASE:
    session = can_fwu_enter_session(payload);
    detail = CAN_FWU_BOOT_VERSION;
    break;
  case CAN_FWU_ERASE_BASE: detail = can_fwu_get_u32(payload, 1); break;
  case CAN_FWU_DATA_BASE: detail = can_fwu_data_seq(payload); break;
  case CAN_FWU_VERIFY_BASE: detail = can_fwu_get_u32(payload, 0); break;
  default: break;
  }
  can_fwu_ack_reply_pack(ack, status, base, session, detail);
}

int can_fwu_boot_frame(struct can_fwu_boot *b, uint32_t id,
                       const uint8_t payload[8], uint32_t now,
                       uint32_t *ack_id, uint8_t ack[8])
{
  uint32_t page, n, i, seq, st = CAN_FWU_OK;
  int mine = dev_match(id, CAN_FWU_ENTER_BASE, b->dev) ||
             dev_match(id, CAN_FWU_ERASE_BASE, b->dev) ||
             dev_match(id, CAN_FWU_DATA_BASE, b->dev) ||
             dev_match(id, CAN_FWU_VERIFY_BASE, b->dev) ||
             dev_match(id, CAN_FWU_JUMP_BASE, b->dev);
  if (!mine) {
    return 0;
  }
  if (ack_id != 0) {
    *ack_id = can_fwu_id(CAN_FWU_ACK_BASE, b->dev);
  }
  if (dev_match(id, CAN_FWU_ENTER_BASE, b->dev)) {
    uint16_t session = can_fwu_enter_session(payload);
    if (can_fwu_enter_magic(payload) != CAN_FWU_MAGIC || session == 0U) {
      st = CAN_FWU_REJECT;
    } else {
      uint32_t size = can_fwu_get_u32(payload, 1);
      int stopped = (b->ops.motor_stopped != 0)
                        ? b->ops.motor_stopped(b->ops.ctx)
                        : 1;
      if (size < CAN_FWU_APP_MIN || size > CAN_FWU_APP_MAX) {
        st = CAN_FWU_BAD_SIZE;
      } else if (!stopped) {
        st = CAN_FWU_REJECT;
      } else if (b->state != FWU_B_IDLE && b->state != FWU_B_FAULT) {
        /* Never rewind flash/sequence state on a repeated ENTER or allow
         * a different session to take over a live update. A fault permits
         * recovery, but only with a fresh session (old retries stay stale). */
        if (session != b->session || size != b->img_size) st = CAN_FWU_REJECT;
      } else if (b->state == FWU_B_FAULT && session == b->session) {
        st = CAN_FWU_REJECT;
      } else {
        b->state = FWU_B_ENTERED;
        b->session = session;
        b->img_size = size;
        b->img_crc = 0U;
        b->received = 0U;
        b->stream_crc = 0U;
        b->expect_seq = 0U;
        b->num_pages = can_fwu_pages_for_size(size);
        b->erased_pages = 0U;
        b->dw_fill = 0U;
        b->dw_base = 0U;
        b->fault = 0U;
      }
    }
    if (st == CAN_FWU_OK) b->last_tick = now;
    reply(b, id, payload, ack, st);
    return 1;
  }

  if (b->state != FWU_B_ENTERED && b->state != FWU_B_ERASED &&
      b->state != FWU_B_RECEIVING && b->state != FWU_B_VERIFIED) {
    reply(b, id, payload, ack, CAN_FWU_REJECT);
    return 1;
  }

  if (dev_match(id, CAN_FWU_ERASE_BASE, b->dev)) {
    page = can_fwu_get_u32(payload, 0);
    n = can_fwu_get_u32(payload, 1);
    if ((b->state == FWU_B_ERASED || b->state == FWU_B_RECEIVING ||
         b->state == FWU_B_VERIFIED) && page == 0U && n == b->num_pages &&
        b->erased_pages != 0U) {
      /* Exact whole-image ERASE retry: acknowledge, never erase twice. */
    } else if (b->state != FWU_B_ENTERED) {
      st = CAN_FWU_REJECT;
    } else {
      /* This protocol erases the complete declared image in one request.
       * Partial/overflowing ranges must never permit subsequent writes. */
      if (page != 0U || n != b->num_pages || n == 0U ||
          b->ops.erase == 0) {
        st = CAN_FWU_REJECT;
      } else if (b->ops.erase(page, n, b->ops.ctx) != 0) {
        st = CAN_FWU_FLASH_ERR;
        to_fault(b, st);
      } else {
        for (i = 0; i < n; i++) {
          b->erased_pages |= (uint64_t)1U << (page + i);
        }
        b->state = FWU_B_ERASED;
        b->expect_seq = 0U;
      }
    }
    if (st == CAN_FWU_OK) b->last_tick = now;
    reply(b, id, payload, ack, st);
    return 1;
  }

  if (dev_match(id, CAN_FWU_DATA_BASE, b->dev)) {
    uint32_t remaining, take;
    seq = can_fwu_data_seq(payload);
    if (b->expect_seq != 0U && seq == b->expect_seq - 1U) {
      /* Compare padding too: only an identical wire retry is idempotent. */
      st = memcmp(payload, b->last_data, 8) == 0 ? CAN_FWU_OK : CAN_FWU_BAD_SEQ;
      if (st == CAN_FWU_OK) b->last_tick = now;
      reply(b, id, payload, ack, st);
      return 1;
    }
    if (b->state != FWU_B_ERASED && b->state != FWU_B_RECEIVING) {
      reply(b, id, payload, ack, CAN_FWU_REJECT);
      return 1;
    }
    if (seq != b->expect_seq) {
      reply(b, id, payload, ack, CAN_FWU_BAD_SEQ);
      return 1;
    }
    if (b->erased_pages == 0U || b->received >= b->img_size) {
      reply(b, id, payload, ack, CAN_FWU_REJECT);
      return 1;
    }
    remaining = b->img_size - b->received;
    take = remaining < CAN_FWU_CHUNK ? remaining : CAN_FWU_CHUNK;
    b->stream_crc = can_fwu_crc32_update(b->stream_crc, &payload[2], take);
    for (i = 0; i < take; i++) {
      if (b->dw_fill == 0U) {
        b->dw_base = b->received + i;
      }
      b->dw_buf[b->dw_fill++] = payload[2 + i];
      if (b->dw_fill == 8U) {
        uint64_t dw = 0U;
        unsigned k;
        for (k = 0; k < 8; k++) {
          dw |= (uint64_t)b->dw_buf[k] << (8U * k);
        }
        if (b->ops.program_dw == 0 ||
            b->ops.program_dw(b->dw_base, dw, b->ops.ctx) != 0) {
          to_fault(b, CAN_FWU_FLASH_ERR);
          reply(b, id, payload, ack, CAN_FWU_FLASH_ERR);
          return 1;
        }
        b->dw_fill = 0U;
      }
    }
    b->received += take;
    b->expect_seq++;
    b->state = FWU_B_RECEIVING;
    if (b->received >= b->img_size && b->dw_fill != 0U) {
      /* Pad final double-word with erased value and program it. */
      uint64_t dw = 0U;
      unsigned k;
      while (b->dw_fill < 8U) {
        b->dw_buf[b->dw_fill++] = 0xFFU;
      }
      for (k = 0; k < 8; k++) {
        dw |= (uint64_t)b->dw_buf[k] << (8U * k);
      }
      if (b->ops.program_dw == 0 ||
          b->ops.program_dw(b->dw_base, dw, b->ops.ctx) != 0) {
        to_fault(b, CAN_FWU_FLASH_ERR);
        reply(b, id, payload, ack, CAN_FWU_FLASH_ERR);
        return 1;
      }
      b->dw_fill = 0U;
    }
    memcpy(b->last_data, payload, 8);
    b->last_tick = now;
    reply(b, id, payload, ack, CAN_FWU_OK);
    return 1;
  }

  if (dev_match(id, CAN_FWU_VERIFY_BASE, b->dev)) {
    uint32_t crc = can_fwu_get_u32(payload, 0);
    uint32_t size = can_fwu_get_u32(payload, 1);
    if (b->received != b->img_size || size != b->img_size) {
      st = CAN_FWU_BAD_SIZE;
    } else if (crc != b->stream_crc) {
      st = CAN_FWU_CRC_MISMATCH;
    } else {
      b->img_crc = crc;
      b->state = FWU_B_VERIFIED;
    }
    if (st == CAN_FWU_OK) b->last_tick = now;
    reply(b, id, payload, ack, st);
    return 1;
  }

  /* JUMP */
  if (b->state != FWU_B_VERIFIED) {
    reply(b, id, payload, ack, CAN_FWU_REJECT);
    return 1;
  }
  if (can_fwu_get_u32(payload, 0) != CAN_FWU_MAGIC) {
    reply(b, id, payload, ack, CAN_FWU_REJECT);
    return 1;
  }
  b->state = FWU_B_JUMP_PENDING;
  b->last_tick = now;
  reply(b, id, payload, ack, CAN_FWU_OK);
  return 1;
}

void can_fwu_boot_poll(struct can_fwu_boot *b, uint32_t now)
{
  if (b->state == FWU_B_IDLE ||
      b->state == FWU_B_JUMP_PENDING || b->state == FWU_B_FAULT) {
    return;
  }
  if ((uint32_t)(now - b->last_tick) >= CAN_FWU_TARGET_IDLE_TIMEOUT_MS) {
    to_fault(b, CAN_FWU_REJECT);
  }
}
