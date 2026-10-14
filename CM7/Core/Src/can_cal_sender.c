#include "can_cal_sender.h"

volatile uint32_t can_cal_request = 0;
volatile uint32_t can_cal_result = CAN_CAL_NONE;
volatile uint32_t can_cal_used_mask = 0;
volatile uint32_t can_cal_last_request = 0, can_cal_request_count = 0;
volatile uint32_t can_cal_tx_count = 0, can_cal_tx_ack_count = 0;
volatile uint32_t can_cal_first_fault = 0, can_cal_first_fault_tick = 0;
static uint8_t pending, faulted;
static uint32_t pending_tick;

static void record_fault(uint32_t reason, uint32_t now)
{
  if (can_cal_first_fault == 0U) {
    can_cal_first_fault = reason;
    can_cal_first_fault_tick = now;
  }
  faulted = 1;
  can_cal_result = CAN_CAL_FAILED;
}

void can_cal_tick(uint32_t now, int bench_idle)
{
  uint32_t request = can_cal_request;
  can_cal_request = 0; /* Consume each trigger once, never retry a CAL. */
  if (request != 0U) {
    can_cal_last_request = request;
    ++can_cal_request_count;
  }
  if (request == CAN_CAL_STOP) {
    /* Explicit abort wins even when the bench is not idle. */
    if (pending) {
      can_cal_abort();
      pending = 0;
    }
    can_cal_result = CAN_CAL_REJECTED;
  } else {
    /* Settle any in-flight frame before accepting a new request. */
    if (pending) {
      int status = can_cal_poll();
      if (status != CAN_CAL_TX_PENDING ||
          (uint32_t)(now - pending_tick) >= 100U) {
        if (status == CAN_CAL_TX_PENDING) can_cal_abort();
        pending = 0;
        if (status == CAN_CAL_TX_OK) {
          ++can_cal_tx_ack_count;
          can_cal_result = CAN_CAL_ACKED;
        } else {
          record_fault(status == CAN_CAL_TX_PENDING ? 3U : 2U, now);
        }
      }
    }
    if (request != 0U) {
      if (!can_cal_dev_valid(request)) {
        can_cal_result = CAN_CAL_REJECTED;
      } else if (faulted) {
        can_cal_result = CAN_CAL_REJECTED;
      } else if (!bench_idle) {
        can_cal_result = CAN_CAL_REJECTED;
      } else if (pending) {
        can_cal_result = CAN_CAL_REJECTED;
      } else if ((can_cal_used_mask & request) != 0U) {
        /* One-shot per device: a possibly delivered CAL is never resent. */
        can_cal_result = CAN_CAL_REJECTED;
      } else {
        uint8_t data[4];
        can_cal_pack(data);
        /* Mark used before enqueue: even a failed write may have
         * reached the ESC. */
        can_cal_used_mask |= request;
        if (can_cal_send(can_cal_id(request), data)) {
          ++can_cal_tx_count;
          pending = 1;
          pending_tick = now;
          can_cal_result = CAN_CAL_REQUESTED;
        } else {
          record_fault(1U, now);
        }
      }
    }
  }
}
